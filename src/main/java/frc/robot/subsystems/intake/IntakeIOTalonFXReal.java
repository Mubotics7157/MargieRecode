package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
    private static final int ARM_MOTOR_ID = 21;
    private static final double ROLLER_GEAR_RATIO = 3.0;
    private static final double ARM_GEAR_RATIO = 100.0;
    
    // Pneumatics and sensor IDs
    private static final int DEPLOY_SOLENOID_FORWARD = 0;
    private static final int DEPLOY_SOLENOID_REVERSE = 1;
    private static final int BEAM_BREAK_DIO = 0;
    
    // Hardware
    private final TalonFX rollerMotor;
    private final TalonFX armMotor;
    private final DoubleSolenoid deploySolenoid;
    private final DigitalInput beamBreak;
    
    // Control requests
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    
    // Status signals - Roller
    private final StatusSignal<Angle> rollerPosition;
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> rollerAppliedVolts;
    private final StatusSignal<Current> rollerCurrent;
    
    // Status signals - Arm
    private final StatusSignal<Angle> armPosition;
    private final StatusSignal<AngularVelocity> armVelocity;
    private final StatusSignal<Voltage> armAppliedVolts;
    private final StatusSignal<Current> armCurrent;
    
    public IntakeIOTalonFXReal() {
        // Initialize hardware
        rollerMotor = new TalonFX(ROLLER_MOTOR_ID, TunerConstants.DrivetrainConstants.CANBusName);
        armMotor = new TalonFX(ARM_MOTOR_ID, TunerConstants.DrivetrainConstants.CANBusName);
        deploySolenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH, 
            DEPLOY_SOLENOID_FORWARD, 
            DEPLOY_SOLENOID_REVERSE
        );
        beamBreak = new DigitalInput(BEAM_BREAK_DIO);
        
        // Configure roller motor
        var rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        // Current limits for roller
        rollerConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(rollerConfig, 0.25));
        
        // Configure arm motor
        var armConfig = new TalonFXConfiguration();
        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        // Current limits for arm
        armConfig.CurrentLimits.StatorCurrentLimit = 30.0;
        armConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // PID for arm position control (if needed)
        armConfig.Slot0.kP = 10.0;
        armConfig.Slot0.kI = 0.0;
        armConfig.Slot0.kD = 0.5;
        
        tryUntilOk(5, () -> armMotor.getConfigurator().apply(armConfig, 0.25));
        
        // Create status signals - Roller
        rollerPosition = rollerMotor.getPosition();
        rollerVelocity = rollerMotor.getVelocity();
        rollerAppliedVolts = rollerMotor.getMotorVoltage();
        rollerCurrent = rollerMotor.getStatorCurrent();
        
        // Create status signals - Arm
        armPosition = armMotor.getPosition();
        armVelocity = armMotor.getVelocity();
        armAppliedVolts = armMotor.getMotorVoltage();
        armCurrent = armMotor.getStatorCurrent();
        
        // Configure update frequencies
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            rollerPosition, rollerVelocity, rollerAppliedVolts, rollerCurrent,
            armPosition, armVelocity, armAppliedVolts, armCurrent
        );
        rollerMotor.optimizeBusUtilization();
        armMotor.optimizeBusUtilization();
    }
    
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        var rollerStatus = BaseStatusSignal.refreshAll(
            rollerPosition, rollerVelocity, rollerAppliedVolts, rollerCurrent
        );
        var armStatus = BaseStatusSignal.refreshAll(
            armPosition, armVelocity, armAppliedVolts, armCurrent
        );
        
        // Roller inputs
        inputs.rollerPosition = Units.rotationsToRadians(
            rollerPosition.getValueAsDouble()
        ) / ROLLER_GEAR_RATIO;
        inputs.rollerVelocity = Units.rotationsToRadians(
            rollerVelocity.getValueAsDouble()
        ) / ROLLER_GEAR_RATIO;
        inputs.rollerVoltage = rollerAppliedVolts.getValueAsDouble();
        inputs.rollerCurrent = rollerCurrent.getValueAsDouble();
        
        // Arm inputs
        inputs.armPosition = Units.rotationsToRadians(
            armPosition.getValueAsDouble()
        ) / ARM_GEAR_RATIO;
        inputs.armVelocity = Units.rotationsToRadians(
            armVelocity.getValueAsDouble()
        ) / ARM_GEAR_RATIO;
        inputs.armVoltage = armAppliedVolts.getValueAsDouble();
        inputs.armCurrent = armCurrent.getValueAsDouble();
        
        // Sensor inputs
        inputs.hasGamePiece = !beamBreak.get(); // Beam break is typically active low
    }
    
    @Override
    public void setRollerVoltage(double voltage) {
        rollerMotor.setControl(voltageRequest.withOutput(voltage));
    }
    
    @Override
    public void setArmVoltage(double voltage) {
        armMotor.setControl(voltageRequest.withOutput(voltage));
    }
    
    @Override
    public void resetArmPosition() {
        tryUntilOk(5, () -> armMotor.setPosition(0.0, 0.25));
    }
    
    @Override
    public void deployArm() {
        deploySolenoid.set(DoubleSolenoid.Value.kForward);
    }
    
    @Override
    public void stowArm() {
        deploySolenoid.set(DoubleSolenoid.Value.kReverse);
    }
    
    @Override
    public void stopRoller() {
        rollerMotor.stopMotor();
    }
}