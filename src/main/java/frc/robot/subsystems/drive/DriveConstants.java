package frc.robot.subsystems.drive;

/**
 * Constants for the Drive subsystem. Contains robot physical parameters, PathPlanner configuration, and control gains.
 */
public final class DriveConstants {

    private DriveConstants() {}

    // Robot physical parameters
    public static final double ROBOT_MASS_KG = 39.0;
    public static final double ROBOT_MOI = 3.255;
    public static final double WHEEL_COF = 1.48;
    public static final double COG_HEIGHT_METERS = 0.2; // Center of gravity height for traction limits

    // PID constants for path following (LTE = Longitudinal Tracking Error, CTE = Cross Track Error)
    public static final double PATH_LTE_KP = 5.0;
    public static final double PATH_CTE_KP = 5.0;
    public static final double PATH_THETA_KP = 5.0;

    // Threaded controller configuration
    public static final double THREADED_CONTROLLER_PERIOD_SECONDS = 0.01; // 100Hz

    // Speed deadband thresholds for path following
    public static final double LINEAR_SPEED_DEADBAND = 0.01;
    public static final double ANGULAR_SPEED_DEADBAND = 0.01;

    // Default odometry update frequency (Hz)
    public static final double DEFAULT_ODOMETRY_UPDATE_FREQUENCY = 250.0;
}
