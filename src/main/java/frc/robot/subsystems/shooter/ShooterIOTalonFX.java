package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class ShooterIOTalonFX implements ShooterIO {
    // Motor CAN IDs
    private static final int TOP_LEFT_ROLLER_ID = 40;
    private static final int TOP_RIGHT_ROLLER_ID = 41;
    private static final int BOTTOM_LEFT_ROLLER_ID = 42;
    private static final int BOTTOM_RIGHT_ROLLER_ID = 43;
    private static final int HOOD_LEADER_ID = 44;
    private static final int HOOD_FOLLOWER_ID = 45;

    // Gear ratios and constants
    private static final double ROLLER_GEAR_RATIO = 1.0; // 1:1 direct drive
    private static final double HOOD_GEAR_RATIO = 50.0; // 50:1 reduction
    private static final double MIN_HOOD_ANGLE_DEGREES = 0.0;
    private static final double MAX_HOOD_ANGLE_DEGREES = 45.0;

    // Hardware
    private final TalonFX topLeftRoller;
    private final TalonFX topRightRoller;
    private final TalonFX bottomLeftRoller;
    private final TalonFX bottomRightRoller;
    private final TalonFX hoodLeader;
    private final TalonFX hoodFollower;

    // Control requests
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(true);
    private final MotionMagicVoltage hoodPositionRequest = new MotionMagicVoltage(0).withEnableFOC(true);
    private final NeutralOut neutralRequest = new NeutralOut();

    // Status signals for roller motors
    private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> topLeftVelocity;
    private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> topRightVelocity;
    private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> bottomLeftVelocity;
    private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> bottomRightVelocity;
    private final StatusSignal<edu.wpi.first.units.measure.Voltage> topLeftVoltage;
    private final StatusSignal<edu.wpi.first.units.measure.Current> topLeftCurrent;
    private final StatusSignal<edu.wpi.first.units.measure.Temperature> topLeftTemp;

    // Status signals for hood
    private final StatusSignal<edu.wpi.first.units.measure.Angle> hoodPosition;
    private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> hoodVelocity;
    private final StatusSignal<edu.wpi.first.units.measure.Voltage> hoodVoltage;
    private final StatusSignal<edu.wpi.first.units.measure.Current> hoodCurrent;

    private double targetHoodDegrees = 0.0;

    public ShooterIOTalonFX() {
        // Initialize roller motors
        topLeftRoller = new TalonFX(TOP_LEFT_ROLLER_ID, "rio");
        topRightRoller = new TalonFX(TOP_RIGHT_ROLLER_ID, "rio");
        bottomLeftRoller = new TalonFX(BOTTOM_LEFT_ROLLER_ID, "rio");
        bottomRightRoller = new TalonFX(BOTTOM_RIGHT_ROLLER_ID, "rio");

        // Initialize hood motors
        hoodLeader = new TalonFX(HOOD_LEADER_ID, "rio");
        hoodFollower = new TalonFX(HOOD_FOLLOWER_ID, "rio");

        // Configure roller motors
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Current limits for rollers
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = 40;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.StatorCurrentLimit = 80;

        // PID for velocity control
        rollerConfig.Slot0.kP = 0.2;
        rollerConfig.Slot0.kI = 0.0;
        rollerConfig.Slot0.kD = 0.0;
        rollerConfig.Slot0.kV = 0.11; // Feedforward

        // Apply config to all roller motors
        tryUntilOk(5, () -> topLeftRoller.getConfigurator().apply(rollerConfig, 0.25));

        // Right side rollers spin opposite direction
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        tryUntilOk(5, () -> topRightRoller.getConfigurator().apply(rollerConfig, 0.25));

        rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> bottomLeftRoller.getConfigurator().apply(rollerConfig, 0.25));

        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        tryUntilOk(5, () -> bottomRightRoller.getConfigurator().apply(rollerConfig, 0.25));

        // Configure hood motors
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Current limits for hood
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = 20;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.StatorCurrentLimit = 40;

        // Motion Magic configuration for smooth hood movement
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 50; // rotations per second
        hoodConfig.MotionMagic.MotionMagicAcceleration = 100; // rotations per second squared
        hoodConfig.MotionMagic.MotionMagicJerk = 1000; // rotations per second cubed

        // PID for position control
        hoodConfig.Slot0.kP = 50.0;
        hoodConfig.Slot0.kI = 0.0;
        hoodConfig.Slot0.kD = 1.0;
        hoodConfig.Slot0.kV = 0.0;
        hoodConfig.Slot0.kS = 0.2; // Static feedforward to overcome friction

        // Soft limits for hood
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                Units.degreesToRotations(MAX_HOOD_ANGLE_DEGREES) * HOOD_GEAR_RATIO;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                Units.degreesToRotations(MIN_HOOD_ANGLE_DEGREES) * HOOD_GEAR_RATIO;

        tryUntilOk(5, () -> hoodLeader.getConfigurator().apply(hoodConfig, 0.25));

        // Configure follower
        hoodFollower.setControl(new Follower(HOOD_LEADER_ID, true)); // Oppose leader

        // Reset hood position to zero
        resetHoodPosition();

        // Get status signals
        topLeftVelocity = topLeftRoller.getVelocity();
        topRightVelocity = topRightRoller.getVelocity();
        bottomLeftVelocity = bottomLeftRoller.getVelocity();
        bottomRightVelocity = bottomRightRoller.getVelocity();
        topLeftVoltage = topLeftRoller.getMotorVoltage();
        topLeftCurrent = topLeftRoller.getSupplyCurrent();
        topLeftTemp = topLeftRoller.getDeviceTemp();

        hoodPosition = hoodLeader.getPosition();
        hoodVelocity = hoodLeader.getVelocity();
        hoodVoltage = hoodLeader.getMotorVoltage();
        hoodCurrent = hoodLeader.getSupplyCurrent();

        // Configure update frequencies
        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                topLeftVelocity,
                topRightVelocity,
                bottomLeftVelocity,
                bottomRightVelocity,
                topLeftVoltage,
                topLeftCurrent,
                topLeftTemp,
                hoodPosition,
                hoodVelocity,
                hoodVoltage,
                hoodCurrent);

        topLeftRoller.optimizeBusUtilization();
        topRightRoller.optimizeBusUtilization();
        bottomLeftRoller.optimizeBusUtilization();
        bottomRightRoller.optimizeBusUtilization();
        hoodLeader.optimizeBusUtilization();
        hoodFollower.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                topLeftVelocity,
                topRightVelocity,
                bottomLeftVelocity,
                bottomRightVelocity,
                topLeftVoltage,
                topLeftCurrent,
                topLeftTemp,
                hoodPosition,
                hoodVelocity,
                hoodVoltage,
                hoodCurrent);

        // Individual roller velocities in RPM
        inputs.topLeftVelocityRPM = topLeftVelocity.getValue().in(RotationsPerSecond) * 60.0;
        inputs.topRightVelocityRPM = topRightVelocity.getValue().in(RotationsPerSecond) * 60.0;
        inputs.bottomLeftVelocityRPM = bottomLeftVelocity.getValue().in(RotationsPerSecond) * 60.0;
        inputs.bottomRightVelocityRPM = bottomRightVelocity.getValue().in(RotationsPerSecond) * 60.0;

        // Average velocity for main control
        inputs.velocityRPM = (inputs.topLeftVelocityRPM
                        + inputs.topRightVelocityRPM
                        + inputs.bottomLeftVelocityRPM
                        + inputs.bottomRightVelocityRPM)
                / 4.0;

        // Roller telemetry (from top left as representative)
        inputs.appliedVolts = topLeftVoltage.getValue().in(Volts);
        inputs.currentAmps = topLeftCurrent.getValue().in(Amps);
        inputs.temperatureCelsius = topLeftTemp.getValue().in(Celsius);

        // Hood telemetry
        inputs.hoodPositionDegrees =
                Units.rotationsToDegrees(hoodPosition.getValue().in(Rotations) / HOOD_GEAR_RATIO);
        inputs.hoodVelocityDegreesPerSec =
                Units.rotationsToDegrees(hoodVelocity.getValue().in(RotationsPerSecond) / HOOD_GEAR_RATIO);
        inputs.hoodAppliedVolts = hoodVoltage.getValue().in(Volts);
        inputs.hoodCurrentAmps = hoodCurrent.getValue().in(Amps);
        inputs.hoodAtSetpoint = Math.abs(inputs.hoodPositionDegrees - targetHoodDegrees) < 1.0;
    }

    @Override
    public void setVoltage(double volts) {
        topLeftRoller.setControl(voltageRequest.withOutput(volts));
        topRightRoller.setControl(voltageRequest.withOutput(volts));
        bottomLeftRoller.setControl(voltageRequest.withOutput(volts));
        bottomRightRoller.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void setVelocity(double velocityRPM, double ffVolts) {
        double velocityRPS = velocityRPM / 60.0;
        topLeftRoller.setControl(velocityRequest.withVelocity(velocityRPS).withFeedForward(ffVolts));
        topRightRoller.setControl(velocityRequest.withVelocity(velocityRPS).withFeedForward(ffVolts));
        bottomLeftRoller.setControl(velocityRequest.withVelocity(velocityRPS).withFeedForward(ffVolts));
        bottomRightRoller.setControl(velocityRequest.withVelocity(velocityRPS).withFeedForward(ffVolts));
    }

    @Override
    public void stop() {
        topLeftRoller.setControl(neutralRequest);
        topRightRoller.setControl(neutralRequest);
        bottomLeftRoller.setControl(neutralRequest);
        bottomRightRoller.setControl(neutralRequest);
    }

    @Override
    public void setBrakeMode(boolean enable) {
        NeutralModeValue mode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        topLeftRoller.setNeutralMode(mode);
        topRightRoller.setNeutralMode(mode);
        bottomLeftRoller.setNeutralMode(mode);
        bottomRightRoller.setNeutralMode(mode);
    }

    @Override
    public void setHoodPosition(double degrees) {
        targetHoodDegrees = Math.max(MIN_HOOD_ANGLE_DEGREES, Math.min(MAX_HOOD_ANGLE_DEGREES, degrees));
        double motorRotations = Units.degreesToRotations(targetHoodDegrees) * HOOD_GEAR_RATIO;
        hoodLeader.setControl(hoodPositionRequest.withPosition(motorRotations));
    }

    @Override
    public void stopHood() {
        hoodLeader.setControl(neutralRequest);
    }

    @Override
    public void resetHoodPosition() {
        tryUntilOk(5, () -> hoodLeader.setPosition(0.0, 0.25));
        targetHoodDegrees = 0.0;
    }
}
