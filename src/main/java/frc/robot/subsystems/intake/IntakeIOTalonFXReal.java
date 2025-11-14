package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.generated.TunerConstants;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class IntakeIOTalonFXReal implements IntakeIO {
    // Hardware configuration constants
    private static final int ROLLER_MOTOR_ID = 20;
    private static final double ROLLER_GEAR_RATIO = 3.0;
    
    // Hardware
    private final TalonFX rollerMotor;
    private final TalonFX arm = new TalonFX(21, TunerConstants.DrivetrainConstants.CANBusName);
    
    // Control requests
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    
    // Status signals
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> rollerAppliedVolts;
    private final StatusSignal<Current> rollerCurrent;
    private final StatusSignal<Temperature> rollerTemp;
    
    // Connection debouncer
    private final Debouncer rollerConnectedDebounce = new Debouncer(0.5);
    
    public IntakeIOTalonFX() {
        // Initialize roller motor
        rollerMotor = new TalonFX(ROLLER_MOTOR_ID, TunerConstants.DrivetrainConstants.CANBusName);
        
        // Configure roller motor
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        // Velocity PID configuration
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12;
        
        // Current limits
        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(config, 0.25));
        
        // Create status signals
        rollerVelocity = rollerMotor.getVelocity();
        rollerAppliedVolts = rollerMotor.getMotorVoltage();
        rollerPosition = rollerMotor.getRollerPosition();
        rollerCurrent = rollerMotor.getStatorCurrent();
        armVelocity = arm.getArmVelocity();
        armCurrent = arm.getStatorCurrent();
        armPosition = arm.getArmPosition();
        armVoltage = arm.getMotorVoltage();
        
        // Configure update frequencies
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            rollerVelocity, rollerAppliedVolts, rollerCurrent, rollerTemp
        );
        rollerMotor.optimizeBusUtilization();
    }
    
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        var status = BaseStatusSignal.refreshAll(
            rollerVelocity, rollerCurrent, rollerPosition, rollerAppliedVolts, armVelocity, armCurrent, armPosition, armVoltage
        );
        
        inputs.rollerConnected = rollerConnectedDebounce.calculate(status.isOK());
        inputs.rollerVelocityRPM = Units.radiansToRotations(
            rollerVelocity.getValueAsDouble()
        ) * 60.0 / ROLLER_GEAR_RATIO;
        inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
        inputs.rollerCurrentAmps = rollerCurrent.getValueAsDouble();
        inputs.rollerTempCelsius = rollerTemp.getValueAsDouble();
        
        inputs.deployed = deploySolenoid.get() == DoubleSolenoid.Value.kForward;
        inputs.hasGamePiece = !beamBreak.get();
    }
    
    @Override
    public void setRollerVelocity(double rpm) {
        double rotPerSec = rpm / 60.0 * ROLLER_GEAR_RATIO;
        rollerMotor.setControl(velocityRequest.withVelocity(rotPerSec));
    }
    
    @Override
    public void setRollerVoltage(double volts) {
        rollerMotor.setControl(voltageRequest.withOutput(volts));
    }
    
    @Override
    public void stopRoller() {
        rollerMotor.stopMotor();
    }
    
    @Override
    public void deploy() {
        deploySolenoid.set(DoubleSolenoid.Value.kForward);
    }
    
    @Override
    public void retract() {
        deploySolenoid.set(DoubleSolenoid.Value.kReverse);
    }
}