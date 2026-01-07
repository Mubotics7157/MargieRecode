package frc.robot.subsystems.shooter;

import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.ShotParameters;

/**
 * Constants and shot lookup table for the shooter subsystem. Contains distance-based shot parameter interpolation
 * following Team 254's convention.
 */
public final class ShooterConstants {

    private ShooterConstants() {}

    // Motor CAN IDs
    public static final int POOPER_ID = 25;
    public static final int FLYWHEEL_MIDDLE_ID = 22;
    public static final int FLYWHEEL_RIGHT_ID = 26;
    public static final int FLYWHEEL_2IN_ID = 23;
    public static final int HOOD_PIVOT_ID = 24;
    public static final int SHOOTER_MOTOR_ID = 27;

    // Gear ratios
    public static final double ROLLER_GEAR_RATIO = 1.0;

    // Hood conversion: native units (0-12) correspond to exit shot angles (54-74 degrees)
    public static final double MIN_NATIVE_UNITS = 0.0;
    public static final double MAX_NATIVE_UNITS = 12.0;
    public static final double MIN_EXIT_ANGLE_DEGREES = 54.0;
    public static final double MAX_EXIT_ANGLE_DEGREES = 74.0;

    // Soft limits in native units
    public static final double SOFT_LIMIT_REVERSE_NATIVE = 1.0;
    public static final double SOFT_LIMIT_FORWARD_NATIVE = 12.0;

    // Ball detection
    public static final double BALL_DETECTION_CURRENT_THRESHOLD = 30.0; // Amps
    public static final double BALL_DETECTION_DEBOUNCE_TIME = 0.1; // Seconds

    // Current limits (Amps)
    public static final double ROLLER_CURRENT_LIMIT = 80.0;
    public static final double HOOD_CURRENT_LIMIT = 40.0;

    // Flywheel PID gains
    public static final double FLYWHEEL_KP = 70.0;
    public static final double FLYWHEEL_KI = 0.0;
    public static final double FLYWHEEL_KD = 0.0;
    public static final double FLYWHEEL_KV = 0.0;
    public static final double FLYWHEEL_KS = 0.0;

    // Hood PID gains
    public static final double HOOD_KP = 20.0;
    public static final double HOOD_KI = 0.0;
    public static final double HOOD_KD = 2.0;
    public static final double HOOD_KV = 0.0;
    public static final double HOOD_KS = 0.0;
    public static final double HOOD_KG = 3.0;

    // Motion Magic configuration
    public static final double HOOD_MOTION_MAGIC_KA = 0.1;
    public static final double HOOD_MOTION_MAGIC_KV = 0.12;

    // Simulation parameters
    public static final double SIM_FLYWHEEL_GEAR_RATIO = 1.0;
    public static final double SIM_HOOD_GEAR_RATIO = 50.0;
    public static final double SIM_FLYWHEEL_MOI = 0.004; // kg*m^2
    public static final double SIM_HOOD_MOI = 0.025; // kg*m^2

    /** Converts exit shot angle (degrees) to native motor units. */
    public static double degreesToNative(double degrees) {
        return (degrees - MIN_EXIT_ANGLE_DEGREES)
                        / (MAX_EXIT_ANGLE_DEGREES - MIN_EXIT_ANGLE_DEGREES)
                        * (MAX_NATIVE_UNITS - MIN_NATIVE_UNITS)
                + MIN_NATIVE_UNITS;
    }

    /** Converts native motor units to exit shot angle (degrees). */
    public static double nativeToDegrees(double nativeUnits) {
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

    // Distance thresholds (meters)
    public static final double MIN_SHOOTING_DISTANCE = 1.0;
    public static final double MAX_SHOOTING_DISTANCE = 6.0;

    // Flywheel tolerances
    public static final double FLYWHEEL_TOLERANCE_RPM = 100.0;

    // 2" flywheel constant RPM (does not vary with distance)
    public static final double FLYWHEEL_2IN_RPM = 150.0;

    // Hood exit angle limits (degrees)
    // Native units 0-12 correspond to exit angles 54-74 degrees
    public static final double HOOD_MIN_ANGLE_DEGREES = 55.67; // ~1 native unit
    public static final double HOOD_MAX_ANGLE_DEGREES = 74.0; // 12 native units

    /**
     * Shot parameter lookup table. Maps distance (meters) to shot parameters. Values are interpolated for distances
     * between defined points.
     */
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
}
