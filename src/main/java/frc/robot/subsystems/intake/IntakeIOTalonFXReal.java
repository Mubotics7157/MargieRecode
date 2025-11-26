package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

public class IntakeIOTalonFXReal implements IntakeIO {
    // Hardware configuration constants
    private static final int ROLLER_MOTOR_ID = 20;
    private static final int ARM_MOTOR_ID = 19;
    private static final int INDEXER_MOTOR_ID = 17;
    private static final double ROLLER_GEAR_RATIO = 3.0;
    private static final double ARM_GEAR_RATIO = 1; // 248/9
    // Hardware
    private final TalonFX rollerMotor;
    private final TalonFX armMotor;
    private final TalonFX indexerMotor;

    // Control requests
    private final DutyCycleOut voltageRequest = new DutyCycleOut(0.0);
    private final MotionMagicExpoTorqueCurrentFOC positionRequest = new MotionMagicExpoTorqueCurrentFOC(0.0);

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

    // Status Signals - Indexer
    private final StatusSignal<AngularVelocity> indexerVelocity;
    private final StatusSignal<Voltage> indexerAppliedVolts;
    private final StatusSignal<Current> indexerCurrent;

    public IntakeIOTalonFXReal() {
        // Initialize hardware
        rollerMotor = new TalonFX(ROLLER_MOTOR_ID, "rio"); // CHANGE TO SWERVE LATER
        armMotor = new TalonFX(ARM_MOTOR_ID, "rio"); // CHANGE TO SWERVE LATER
        indexerMotor = new TalonFX(INDEXER_MOTOR_ID, "rio"); // CHANGE LATER TO SWERVE

        // Configure roller motor
        var rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Current limits for roller
        rollerConfig.CurrentLimits.StatorCurrentLimit = 60.0;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = 50.0;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(rollerConfig, 0.25));

        // Configure indexer motor
        var indexerConfig = new TalonFXConfiguration();
        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Current limits for roller
        indexerConfig.CurrentLimits.StatorCurrentLimit = 60.0;
        indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        indexerConfig.CurrentLimits.SupplyCurrentLimit = 50.0;
        indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        tryUntilOk(5, () -> indexerMotor.getConfigurator().apply(indexerConfig, 0.25));

        // Configure arm motor
        var armConfig = new TalonFXConfiguration();
        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Current limits for arm
        armConfig.CurrentLimits.StatorCurrentLimit = 30.0;
        armConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // PID for arm position control
        armConfig.Slot0.kP = 100.0;
        armConfig.Slot0.kI = 0.0;
        armConfig.Slot0.kD = 0.5;
        armConfig.Slot0.kS = 0.0;
        armConfig.Slot0.kV = 0.0;

        tryUntilOk(5, () -> armMotor.getConfigurator().apply(armConfig, 0.25));

        // Create status signals - Roller
        rollerPosition = rollerMotor.getPosition();
        rollerVelocity = rollerMotor.getVelocity();
        rollerAppliedVolts = rollerMotor.getMotorVoltage();
        rollerCurrent = rollerMotor.getStatorCurrent();

        // Create status signals - Indexer
        indexerVelocity = indexerMotor.getVelocity();
        indexerAppliedVolts = indexerMotor.getMotorVoltage();
        indexerCurrent = indexerMotor.getStatorCurrent();

        // Create status signals - Arm
        armPosition = armMotor.getPosition();
        armVelocity = armMotor.getVelocity();
        armAppliedVolts = armMotor.getMotorVoltage();
        armCurrent = armMotor.getStatorCurrent();

        // Configure update frequencies
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                rollerPosition,
                rollerVelocity,
                rollerAppliedVolts,
                rollerCurrent,
                armPosition,
                armVelocity,
                armAppliedVolts,
                armCurrent);
        rollerMotor.optimizeBusUtilization();
        armMotor.optimizeBusUtilization();
        indexerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(rollerPosition, rollerVelocity, rollerAppliedVolts, rollerCurrent);
        BaseStatusSignal.refreshAll(armPosition, armVelocity, armAppliedVolts, armCurrent);

        // Roller inputs
        inputs.rollerPosition = Units.rotationsToRadians(rollerPosition.getValueAsDouble()) / ROLLER_GEAR_RATIO;
        inputs.rollerVelocity = Units.rotationsToRadians(rollerVelocity.getValueAsDouble()) / ROLLER_GEAR_RATIO;
        inputs.rollerVoltage = rollerAppliedVolts.getValueAsDouble();
        inputs.rollerCurrent = rollerCurrent.getValueAsDouble();

        // Arm inputs
        inputs.armPosition = Units.rotationsToRadians(armPosition.getValueAsDouble()) / ARM_GEAR_RATIO;
        inputs.armVelocity = Units.rotationsToRadians(armVelocity.getValueAsDouble()) / ARM_GEAR_RATIO;
        inputs.armVoltage = armAppliedVolts.getValueAsDouble();
        inputs.armCurrent = armCurrent.getValueAsDouble();

        // Sensor inputs
        inputs.hasGamePiece = rollerMotor.getStatorCurrent().getValueAsDouble()
                > 100.0; // May need to change current limit and detection later
    }

    @Override
    public void setRollerVoltage(double voltage) {
        rollerMotor.setControl(voltageRequest.withOutput(voltage));
        indexerMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void setArmVoltage(double voltage) {
        armMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void setArmPosition(double positionRad) {
        double motorRotations = Units.radiansToRotations(positionRad) * ARM_GEAR_RATIO;
        armMotor.setControl(positionRequest.withPosition(motorRotations));
    }

    @Override
    public void resetArmPosition() {
        tryUntilOk(5, () -> armMotor.setPosition(0.0, 0.25));
    }

    @Override
    public void stopRoller() {
        rollerMotor.stopMotor();
        indexerMotor.stopMotor();
    }

    public void stopArmMotor(){
        armMotor.stopMotor();
    }
}
