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
    // Hardware
    private final TalonFX rollerMotor;
    private final TalonFX armMotor;
    private final TalonFX indexerMotor;

    // Control requests
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);
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
    private final StatusSignal<Angle> indexerPosition;
    private final StatusSignal<AngularVelocity> indexerVelocity;
    private final StatusSignal<Voltage> indexerAppliedVolts;
    private final StatusSignal<Current> indexerCurrent;

    public IntakeIOTalonFXReal() {
        // Initialize hardware
        rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, IntakeConstants.CAN_BUS);
        armMotor = new TalonFX(IntakeConstants.ARM_MOTOR_ID, IntakeConstants.CAN_BUS);
        indexerMotor = new TalonFX(IntakeConstants.INDEXER_MOTOR_ID, IntakeConstants.CAN_BUS);

        // Configure roller motor
        var rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Current limits for roller
        rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(rollerConfig, 0.25));

        // Configure indexer motor
        var indexerConfig = new TalonFXConfiguration();
        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Current limits for indexer
        indexerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.INDEXER_STATOR_CURRENT_LIMIT;
        indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        tryUntilOk(5, () -> indexerMotor.getConfigurator().apply(indexerConfig, 0.25));

        // Configure arm motor
        var armConfig = new TalonFXConfiguration();
        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Current limits for arm
        armConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.ARM_STATOR_CURRENT_LIMIT;
        armConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // PID for arm position control
        armConfig.Slot0.kP = IntakeConstants.ARM_KP;
        armConfig.Slot0.kI = IntakeConstants.ARM_KI;
        armConfig.Slot0.kD = IntakeConstants.ARM_KD;
        armConfig.Slot0.kS = IntakeConstants.ARM_KS;
        armConfig.Slot0.kV = IntakeConstants.ARM_KV;

        tryUntilOk(5, () -> armMotor.getConfigurator().apply(armConfig, 0.25));

        resetArmPosition();

        // Create status signals - Roller
        rollerPosition = rollerMotor.getPosition();
        rollerVelocity = rollerMotor.getVelocity();
        rollerAppliedVolts = rollerMotor.getMotorVoltage();
        rollerCurrent = rollerMotor.getStatorCurrent();

        // Create status signals - Indexer
        indexerPosition = indexerMotor.getPosition();
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
                IntakeConstants.STATUS_SIGNAL_UPDATE_FREQUENCY,
                rollerPosition,
                rollerVelocity,
                rollerAppliedVolts,
                rollerCurrent,
                armPosition,
                armVelocity,
                armAppliedVolts,
                armCurrent,
                indexerVelocity,
                indexerAppliedVolts,
                indexerCurrent,
                indexerPosition);
        rollerMotor.optimizeBusUtilization();
        armMotor.optimizeBusUtilization();
        indexerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(rollerPosition, rollerVelocity, rollerAppliedVolts, rollerCurrent);
        BaseStatusSignal.refreshAll(armPosition, armVelocity, armAppliedVolts, armCurrent);
        BaseStatusSignal.refreshAll(indexerVelocity, indexerAppliedVolts, indexerCurrent);

        // Roller inputs
        inputs.rollerPosition =
                Units.rotationsToRadians(rollerPosition.getValueAsDouble()) / IntakeConstants.ROLLER_GEAR_RATIO;
        inputs.rollerVelocity =
                Units.rotationsToRadians(rollerVelocity.getValueAsDouble()) / IntakeConstants.ROLLER_GEAR_RATIO;
        inputs.rollerVoltage = rollerAppliedVolts.getValueAsDouble();
        inputs.rollerCurrent = rollerCurrent.getValueAsDouble();

        // Arm inputs
        inputs.armPosition = Units.rotationsToRadians(armPosition.getValueAsDouble()) / IntakeConstants.ARM_GEAR_RATIO;
        inputs.armVelocity = Units.rotationsToRadians(armVelocity.getValueAsDouble()) / IntakeConstants.ARM_GEAR_RATIO;
        inputs.armVoltage = armAppliedVolts.getValueAsDouble();
        inputs.armCurrent = armCurrent.getValueAsDouble();

        // Indexer inputs
        inputs.indexerPosition =
                Units.rotationsToRadians(indexerPosition.getValueAsDouble()) / IntakeConstants.INDEXER_GEAR_RATIO;
        inputs.indexerVelocity =
                Units.rotationsToRadians(indexerVelocity.getValueAsDouble()) / IntakeConstants.INDEXER_GEAR_RATIO;
        inputs.indexerVoltage = indexerAppliedVolts.getValueAsDouble();
        inputs.indexerCurrent = indexerCurrent.getValueAsDouble();

        // Ball detection based on current spike in roller or indexer motors
        inputs.ballDetected = (inputs.rollerCurrent > IntakeConstants.BALL_DETECTION_CURRENT_THRESHOLD)
                || (inputs.indexerCurrent > IntakeConstants.BALL_DETECTION_CURRENT_THRESHOLD);
    }

    @Override
    public void setRollerDutyCycle(double value) {
        rollerMotor.setControl(dutyCycleRequest.withOutput(value));
    }

    @Override
    public void setArmVoltage(double voltage) {
        armMotor.setControl(dutyCycleRequest.withOutput(voltage));
    }

    @Override
    public void setArmPosition(double positionRad) {
        double motorRotations = Units.radiansToRotations(positionRad) * IntakeConstants.ARM_GEAR_RATIO;
        armMotor.setControl(positionRequest.withPosition(motorRotations));
    }

    @Override
    public void resetArmPosition() {
        tryUntilOk(5, () -> armMotor.setPosition(0.0, 0.25));
    }

    @Override
    public void stopRoller() {
        rollerMotor.stopMotor();
    }

    public void stopArmMotor() {
        armMotor.stopMotor();
    }

    @Override
    public void setIndexerDutyCycle(double value) {
        indexerMotor.setControl(dutyCycleRequest.withOutput(value));
    }

    @Override
    public void stopIndexer() {
        indexerMotor.stopMotor();
    }
}
