package frc.robot.subsystems.intake;

/**
 * Constants for the Intake subsystem. Contains motor IDs, gear ratios, current limits, PID gains, and position
 * constants.
 */
public final class IntakeConstants {

    private IntakeConstants() {}

    // ==================== Motor CAN IDs ====================
    public static final int ROLLER_MOTOR_ID = 20;
    public static final int ARM_MOTOR_ID = 19;
    public static final int INDEXER_MOTOR_ID = 17;
    public static final String CAN_BUS = "swerve";

    // ==================== Gear Ratios ====================
    public static final double ROLLER_GEAR_RATIO = 3.0;
    public static final double ARM_GEAR_RATIO = 1.0; // 248/9
    public static final double INDEXER_GEAR_RATIO = 3.0;

    // ==================== Arm Position Constants (radians) ====================
    public static final double ARM_STOWED_POSITION = 0.0;
    public static final double ARM_DEPLOYED_POSITION = -8.0; // -1.62

    // ==================== Current Limits (Amps) ====================
    public static final double ROLLER_STATOR_CURRENT_LIMIT = 60.0;
    public static final double INDEXER_STATOR_CURRENT_LIMIT = 60.0;
    public static final double ARM_STATOR_CURRENT_LIMIT = 30.0;

    // ==================== Ball Detection ====================
    public static final double BALL_DETECTION_CURRENT_THRESHOLD = 30.0; // Amps

    // ==================== Arm PID Gains ====================
    public static final double ARM_KP = 100.0;
    public static final double ARM_KI = 0.0;
    public static final double ARM_KD = 0.5;
    public static final double ARM_KS = 0.0;
    public static final double ARM_KV = 0.0;

    // ==================== Status Signal Update Frequency ====================
    public static final double STATUS_SIGNAL_UPDATE_FREQUENCY = 50.0; // Hz

    // ==================== Simulation Parameters ====================
    public static final class Sim {
        public static final double ROLLER_MOI = 0.001; // kg*m^2
        public static final double INDEXER_MOI = 0.001; // kg*m^2
        public static final double ARM_MOI = 0.1; // kg*m^2
        public static final double ARM_GEAR_RATIO = 100.0; // Sim uses different gear ratio

        // Arm position limits for simulation
        public static final double ARM_MIN_ANGLE = -10.0; // Below deployed position
        public static final double ARM_MAX_ANGLE = 1.0; // Above stowed position

        // Sim position control
        public static final double ARM_KP = 10.0;
        public static final double MAX_VOLTAGE = 12.0;

        // Ball detection simulation thresholds
        public static final double INTAKE_VOLTAGE_THRESHOLD = 6.0;
        public static final double INTAKE_VELOCITY_THRESHOLD = 10.0;
        public static final double OUTTAKE_VOLTAGE_THRESHOLD = -6.0;
    }

    // ==================== Mechanism Visualization ====================
    public static final class Mechanism {
        public static final double WIDTH = 3.0; // meters
        public static final double HEIGHT = 3.0; // meters
        public static final double ROOT_X = 1.5;
        public static final double ROOT_Y = 1.5;
        public static final double ARM_LENGTH = 0.5; // meters
        public static final double ROLLER_LENGTH = 0.15; // meters
        public static final double INDEXER_LENGTH = 0.2; // meters
        public static final double BALL_INDICATOR_LENGTH = 0.1; // meters
        public static final double BALL_INDICATOR_LENGTH_WITH_BALL = 0.15; // meters
        public static final double BALL_INDICATOR_LENGTH_NO_BALL = 0.05; // meters

        // Velocity threshold for color changes
        public static final double VELOCITY_THRESHOLD = 1.0; // rad/s

        // Position error tolerance for arm color
        public static final double ARM_POSITION_TOLERANCE = 0.5; // radians
    }
}
