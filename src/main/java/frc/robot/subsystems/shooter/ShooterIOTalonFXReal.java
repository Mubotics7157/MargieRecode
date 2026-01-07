package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterIOTalonFXReal implements ShooterIO {
    // Motor CAN IDs
    private static final int POOPER_ID = 25;
    private static final int FLYWHEEL_MIDDLE_ID = 22;
    private static final int FLYWHEEL_RIGHT_ID = 26;
    private static final int FLYWHEEL_2IN_ID = 23;
    private static final int HOOD_PIVOT_ID = 24;
    private static final int SHOOTER_MOTOR_ID = 27;

    // Gear ratios and constants
    private static final double ROLLER_GEAR_RATIO = 1.0; // 1:1 direct drive

    // Hood conversion: native units (0-12) correspond to exit shot angles (54-74 degrees)
    private static final double MIN_NATIVE_UNITS = 0.0;
    private static final double MAX_NATIVE_UNITS = 12.0;
    private static final double MIN_EXIT_ANGLE_DEGREES = 54.0;
    private static final double MAX_EXIT_ANGLE_DEGREES = 74.0;

    // Soft limits in native units (must match config below)
    private static final double SOFT_LIMIT_REVERSE_NATIVE = 1.0; // ~55.67 degrees
    private static final double SOFT_LIMIT_FORWARD_NATIVE = 12.0; // 74 degrees

    // Hardware
    private final TalonFX PooperMotor;
    private final TalonFX FlywheelMid;
    private final TalonFX FlywheelRight;
    private final TalonFX Flywheel2In;
    private final TalonFX ShooterMotor;
    private final TalonFX HoodPivot;

    // Control requests
    private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);
    private final NeutralOut neutralRequest = new NeutralOut();
    private final MotionMagicExpoTorqueCurrentFOC hoodPositionRequest = new MotionMagicExpoTorqueCurrentFOC(0);

    // Status signals for flywheel motors (main shooting)
    private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> flywheelMidVelocity;
    private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> flywheelRightVelocity;
    private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> flywheel2InVelocity;
    private final StatusSignal<edu.wpi.first.units.measure.Voltage> flywheelMidVoltage;
    private final StatusSignal<edu.wpi.first.units.measure.Current> flywheelMidCurrent;
    private final StatusSignal<edu.wpi.first.units.measure.Temperature> flywheelMidTemp;

    // Status signals for indexer motors
    private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> shooterMotorVelocity;
    private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> pooperVelocity;
    private final StatusSignal<edu.wpi.first.units.measure.Current> shooterMotorCurrent;

    // Status Signals for pooper motor
    private final StatusSignal<edu.wpi.first.units.measure.Current> pooperMotorCurrent;

    // Status signals for hood
    private final StatusSignal<edu.wpi.first.units.measure.Angle> hoodPosition;
    private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> hoodVelocity;
    private final StatusSignal<edu.wpi.first.units.measure.Voltage> hoodVoltage;
    private final StatusSignal<edu.wpi.first.units.measure.Current> hoodCurrent;

    private double targetHoodDegrees = MIN_EXIT_ANGLE_DEGREES;

    /** Converts exit shot angle (degrees) to native motor units. */
    private static double degreesToNative(double degrees) {
        return (degrees - MIN_EXIT_ANGLE_DEGREES)
                        / (MAX_EXIT_ANGLE_DEGREES - MIN_EXIT_ANGLE_DEGREES)
                        * (MAX_NATIVE_UNITS - MIN_NATIVE_UNITS)
                + MIN_NATIVE_UNITS;
    }

    /** Converts native motor units to exit shot angle (degrees). */
    private static double nativeToDegrees(double nativeUnits) {
        return (nativeUnits - MIN_NATIVE_UNITS)
                        / (MAX_NATIVE_UNITS - MIN_NATIVE_UNITS)
                        * (MAX_EXIT_ANGLE_DEGREES - MIN_EXIT_ANGLE_DEGREES)
                + MIN_EXIT_ANGLE_DEGREES;
    }

    /** Returns the minimum exit angle allowed by soft limits. */
    public static double getMinExitAngleDegrees() {
        return nativeToDegrees(SOFT_LIMIT_REVERSE_NATIVE);
    }

    /** Returns the maximum exit angle allowed by soft limits. */
    public static double getMaxExitAngleDegrees() {
        return nativeToDegrees(SOFT_LIMIT_FORWARD_NATIVE);
    }

    public ShooterIOTalonFXReal() {
        // Initialize roller motors
        PooperMotor = new TalonFX(POOPER_ID, "swerve");
        FlywheelMid = new TalonFX(FLYWHEEL_MIDDLE_ID, "swerve");
        FlywheelRight = new TalonFX(FLYWHEEL_RIGHT_ID, "swerve");
        Flywheel2In = new TalonFX(FLYWHEEL_2IN_ID, "swerve");
        ShooterMotor = new TalonFX(SHOOTER_MOTOR_ID, "swerve");
        // Initialize Hood Motor
        HoodPivot = new TalonFX(HOOD_PIVOT_ID, "swerve");

        // Configure roller motors
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Current limits for rollers
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.StatorCurrentLimit = 80;

        // PID for velocity control
        rollerConfig.Slot0.kP = 10;
        rollerConfig.Slot0.kI = 0.0;
        rollerConfig.Slot0.kD = 0.0;
        rollerConfig.Slot0.kV = 0.0;
        rollerConfig.Slot0.kS = 3;

        // Apply config to all roller motors
        tryUntilOk(5, () -> PooperMotor.getConfigurator().apply(rollerConfig, 0.25));

        rollerConfig.Slot0.kP = 5;
        rollerConfig.Slot0.kS = 0.7;
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        tryUntilOk(5, () -> FlywheelMid.getConfigurator().apply(rollerConfig, 0.25));
        tryUntilOk(5, () -> FlywheelRight.getConfigurator().apply(rollerConfig, 0.25));

        rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rollerConfig.Slot0.kP = 10;
        rollerConfig.Slot0.kS = 3;
        tryUntilOk(5, () -> Flywheel2In.getConfigurator().apply(rollerConfig, 0.25));
        tryUntilOk(5, () -> ShooterMotor.getConfigurator().apply(rollerConfig, 0.25));

        // Configure hood motors
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Current limits for hood
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.StatorCurrentLimit = 40;

        // Motion Magic configuration for smooth hood movement
        hoodConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
        hoodConfig.MotionMagic.MotionMagicExpo_kV = 0.12;

        // PID for position control
        hoodConfig.Slot0.kP = 20.0;
        hoodConfig.Slot0.kI = 0.0;
        hoodConfig.Slot0.kD = 2.0;
        hoodConfig.Slot0.kV = 0.0;
        hoodConfig.Slot0.kS = 0.0;
        hoodConfig.Slot0.kG = 3;
        hoodConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // Soft limits for hood
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 12;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1;

        tryUntilOk(5, () -> HoodPivot.getConfigurator().apply(hoodConfig, 0.25));

        // Reset hood position to zero
        resetHoodPosition();

        // Get status signals for flywheels
        flywheelMidVelocity = FlywheelMid.getVelocity();
        flywheelRightVelocity = FlywheelRight.getVelocity();
        flywheel2InVelocity = Flywheel2In.getVelocity();
        flywheelMidVoltage = FlywheelMid.getMotorVoltage();
        flywheelMidCurrent = FlywheelMid.getSupplyCurrent();
        flywheelMidTemp = FlywheelMid.getDeviceTemp();

        // Get status signals for indexer motors
        shooterMotorVelocity = ShooterMotor.getVelocity();
        pooperVelocity = PooperMotor.getVelocity();
        shooterMotorCurrent = ShooterMotor.getStatorCurrent();

        // Get status signals for hood
        hoodPosition = HoodPivot.getPosition();
        hoodVelocity = HoodPivot.getVelocity();
        hoodVoltage = HoodPivot.getMotorVoltage();
        hoodCurrent = HoodPivot.getSupplyCurrent();

        // Pooper status signals
        pooperMotorCurrent = PooperMotor.getStatorCurrent();

        // Configure update frequencies
        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                flywheelMidVelocity,
                flywheelRightVelocity,
                flywheel2InVelocity,
                flywheelMidVoltage,
                flywheelMidCurrent,
                flywheelMidTemp,
                shooterMotorVelocity,
                shooterMotorCurrent,
                pooperMotorCurrent,
                pooperVelocity,
                hoodPosition,
                hoodVelocity,
                hoodVoltage,
                hoodCurrent);

        PooperMotor.optimizeBusUtilization();
        FlywheelMid.optimizeBusUtilization();
        FlywheelRight.optimizeBusUtilization();
        Flywheel2In.optimizeBusUtilization();
        ShooterMotor.optimizeBusUtilization();
        HoodPivot.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                flywheelMidVelocity,
                flywheelRightVelocity,
                flywheel2InVelocity,
                flywheelMidVoltage,
                flywheelMidCurrent,
                flywheelMidTemp,
                shooterMotorVelocity,
                shooterMotorCurrent,
                pooperMotorCurrent,
                pooperVelocity,
                hoodPosition,
                hoodVelocity,
                hoodVoltage,
                hoodCurrent);

        // Hood flywheel velocities in RPM (FlywheelMid and FlywheelRight)
        inputs.flywheelMidVelocityRPM = flywheelMidVelocity.getValue().in(RotationsPerSecond) * 60.0;
        inputs.flywheelRightVelocityRPM = flywheelRightVelocity.getValue().in(RotationsPerSecond) * 60.0;
        inputs.flywheel2InVelocityRPM = flywheel2InVelocity.getValue().in(RotationsPerSecond) * 60.0;

        // Indexer motor velocities
        inputs.shooterMotorVelocityRPM = shooterMotorVelocity.getValue().in(RotationsPerSecond) * 60.0;
        inputs.pooperVelocityRPM = pooperVelocity.getValue().in(RotationsPerSecond) * 60.0;

        // Indexer current for ball detection
        inputs.indexerCurrentAmps = shooterMotorCurrent.getValue().in(Amps);

        // Pooper Current for ball detection
        inputs.pooperCurrentAmps = pooperMotorCurrent.getValue().in(Amps);

        // Average velocity of hood flywheels for main shooting control
        inputs.velocityRPM = (inputs.flywheelMidVelocityRPM + inputs.flywheelRightVelocityRPM) / 2.0;

        // Flywheel telemetry (from FlywheelMid as representative)
        inputs.temperatureCelsius = flywheelMidTemp.getValue().in(Celsius);

        // Hood telemetry - convert native units to exit angle degrees
        double nativePosition = hoodPosition.getValue().in(Rotations);
        inputs.hoodPositionDegrees = nativeToDegrees(nativePosition);
        inputs.hoodAtSetpoint = Math.abs(inputs.hoodPositionDegrees - targetHoodDegrees) < 1.0;
    }

    @Override
    public void setFlywheelVelocity(double velocity) {
        // Controls the hood flywheels (FlywheelMid and FlywheelRight)
        FlywheelMid.setControl(velocityRequest.withVelocity(velocity));
        FlywheelRight.setControl(velocityRequest.withVelocity(velocity));
    }

    @Override
    public void setFlywheel2InVelocity(double velocity) {
        // Controls the 2" flywheel that feeds into hood flywheels
        Flywheel2In.setControl(velocityRequest.withVelocity(velocity));
    }

    @Override
    public void setIndexerVelocity(double velocity) {
        // Controls the ShooterMotor (starwheels) for indexing
        ShooterMotor.setControl(velocityRequest.withVelocity(velocity));
    }

    @Override
    public void setPooperVelocity(double velocity) {
        // Controls the pooper motor
        // Positive = eject from robot (clockwise)
        // Negative = continue through ball path (counter-clockwise)
        PooperMotor.setControl(velocityRequest.withVelocity(velocity));
    }

    @Override
    public void stopFlywheels() {
        FlywheelMid.setControl(neutralRequest);
        FlywheelRight.setControl(neutralRequest);
        Flywheel2In.setControl(neutralRequest);
    }

    @Override
    public void stopIndexer() {
        ShooterMotor.setControl(neutralRequest);
    }

    @Override
    public void stopPooper() {
        PooperMotor.setControl(neutralRequest);
    }

    @Override
    public void stop() {
        stopFlywheels();
        stopIndexer();
        stopPooper();
    }

    @Override
    public void setBrakeMode(boolean enable) {
        NeutralModeValue mode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        FlywheelMid.setNeutralMode(mode);
        FlywheelRight.setNeutralMode(mode);
        Flywheel2In.setNeutralMode(mode);
        ShooterMotor.setNeutralMode(mode);
        PooperMotor.setNeutralMode(mode);
    }

    @Override
    public void setHoodPosition(double degrees) {
        // Clamp to valid exit angle range (soft limits will also enforce this)
        targetHoodDegrees = Math.max(getMinExitAngleDegrees(), Math.min(getMaxExitAngleDegrees(), degrees));
        double nativePosition = degreesToNative(targetHoodDegrees);
        HoodPivot.setControl(hoodPositionRequest.withPosition(nativePosition));
    }

    @Override
    public void stopHood() {
        HoodPivot.setControl(neutralRequest);
    }

    @Override
    public void resetHoodPosition() {
        tryUntilOk(5, () -> HoodPivot.setPosition(0.0, 0.25));
        targetHoodDegrees = MIN_EXIT_ANGLE_DEGREES;
    }
}
