package frc.robot.subsystems.shooter;

import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.ShotParameters;

/**
 * Constants and shot lookup table for the shooter subsystem. Contains motor IDs, gear ratios, PID gains, current
 * limits, and distance-based shot parameter interpolation following Team 254's convention.
 */
public final class ShooterConstants {

    private ShooterConstants() {}

    // ==================== Motor CAN IDs ====================
    public static final int POOPER_ID = 25;
    public static final int FLYWHEEL_MIDDLE_ID = 22;
    public static final int FLYWHEEL_RIGHT_ID = 26;
    public static final int FLYWHEEL_2IN_ID = 23;
    public static final int HOOD_PIVOT_ID = 24;
    public static final int SHOOTER_MOTOR_ID = 27;
    public static final String CAN_BUS = "swerve";

    // ==================== Gear Ratios ====================
    public static final double ROLLER_GEAR_RATIO = 1.0; // 1:1 direct drive

    // ==================== Hood Conversion Constants ====================
    // Native units (0-12) correspond to exit shot angles (54-74 degrees)
    public static final double MIN_NATIVE_UNITS = 0.0;
    public static final double MAX_NATIVE_UNITS = 12.0;
    public static final double MIN_EXIT_ANGLE_DEGREES = 54.0;
    public static final double MAX_EXIT_ANGLE_DEGREES = 74.0;

    // Soft limits in native units
    public static final double SOFT_LIMIT_REVERSE_NATIVE = 1.0; // ~55.67 degrees
    public static final double SOFT_LIMIT_FORWARD_NATIVE = 12.0; // 74 degrees

    // Hood exit angle limits (degrees) - derived from soft limits
    public static final double HOOD_MIN_ANGLE_DEGREES = 55.67; // ~1 native unit
    public static final double HOOD_MAX_ANGLE_DEGREES = 74.0; // 12 native units

    // ==================== Current Limits (Amps) ====================
    public static final double ROLLER_STATOR_CURRENT_LIMIT = 80.0;
    public static final double HOOD_STATOR_CURRENT_LIMIT = 40.0;

    // ==================== Roller PID Gains ====================
    public static final class RollerGains {
        // Pooper
        public static final double POOPER_KP = 10.0;
        public static final double POOPER_KI = 0.0;
        public static final double POOPER_KD = 0.0;
        public static final double POOPER_KV = 0.0;
        public static final double POOPER_KS = 3.0;

        // Flywheel Mid/Right
        public static final double FLYWHEEL_KP = 5.0;
        public static final double FLYWHEEL_KI = 0.0;
        public static final double FLYWHEEL_KD = 0.0;
        public static final double FLYWHEEL_KV = 0.7;
        public static final double FLYWHEEL_KS = 0.7;

        // Flywheel 2In and Shooter Motor
        public static final double FLYWHEEL_2IN_KP = 10.0;
        public static final double FLYWHEEL_2IN_KS = 3.0;
    }

    // ==================== Hood PID Gains ====================
    public static final class HoodGains {
        public static final double KP = 20.0;
        public static final double KI = 0.0;
        public static final double KD = 2.0;
        public static final double KV = 0.0;
        public static final double KS = 0.0;
        public static final double KG = 3.0;

        // Motion Magic configuration
        public static final double MOTION_MAGIC_EXPO_KA = 0.1;
        public static final double MOTION_MAGIC_EXPO_KV = 0.12;
    }

    // ==================== Status Signal Update Frequency ====================
    public static final double STATUS_SIGNAL_UPDATE_FREQUENCY = 100.0; // Hz

    // ==================== Ball Detection ====================
    public static final double BALL_DETECTION_CURRENT_THRESHOLD = 30.0; // Amps
    public static final double BALL_DETECTION_DEBOUNCE_TIME = 0.1; // Seconds

    // ==================== Flywheel Tolerances ====================
    public static final double FLYWHEEL_TOLERANCE_RPM = 100.0;
    public static final double HOOD_POSITION_TOLERANCE_DEGREES = 1.0;

    // ==================== Distance Thresholds (meters) ====================
    public static final double MIN_SHOOTING_DISTANCE = 1.0;
    public static final double MAX_SHOOTING_DISTANCE = 6.0;

    // ==================== 2" Flywheel Constant RPM ====================
    public static final double FLYWHEEL_2IN_RPM = 150.0;

    // ==================== Default Velocities ====================
    public static final double POOPER_FEED_VELOCITY = 50.0; // RPS (~3000 RPM)
    public static final double INDEXER_FEED_VELOCITY = 50.0; // RPS

    // ==================== Shot Parameter Map ====================
    private static final InterpolatingTreeMap<InterpolatingDouble, ShotParameters> SHOT_MAP = createShotMap();

    private static InterpolatingTreeMap<InterpolatingDouble, ShotParameters> createShotMap() {
        InterpolatingTreeMap<InterpolatingDouble, ShotParameters> map = new InterpolatingTreeMap<>();

        // Format: distance (m) -> ShotParameters(flywheelRPM, hoodAngleDegrees)
        // Hood angle is the exit shot angle in degrees (54-74 range)
        // These values should be tuned through testing

        map.put(new InterpolatingDouble(0.0), new ShotParameters(2500, 54));
        map.put(new InterpolatingDouble(10.0), new ShotParameters(3000, 74));

        return map;
    }

    /**
     * Gets interpolated shot parameters for the given distance.
     *
     * @param distanceMeters The distance to target in meters
     * @return The interpolated shot parameters
     */
    public static ShotParameters getShotParameters(double distanceMeters) {
        // Clamp distance to valid range
        double clampedDistance = Math.max(MIN_SHOOTING_DISTANCE, Math.min(MAX_SHOOTING_DISTANCE, distanceMeters));
        return SHOT_MAP.getInterpolated(new InterpolatingDouble(clampedDistance));
    }

    /**
     * Returns the default/fallback shot parameters for when distance is unknown.
     *
     * @return Default shot parameters (mid-range preset)
     */
    public static ShotParameters getDefaultShotParameters() {
        return new ShotParameters(3500, 63.17); // ~5.5 native units
    }

    // ==================== Mechanism Visualization ====================
    public static final class Mechanism {
        public static final double WIDTH = 4.0; // meters
        public static final double HEIGHT = 3.0; // meters

        // Flywheel velocity threshold for color change
        public static final double VELOCITY_THRESHOLD_RPM = 100.0;
    }
}
