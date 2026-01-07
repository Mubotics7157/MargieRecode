package frc.robot.subsystems.intake;

/** Constants for the intake subsystem. */
public final class IntakeConstants {

    private IntakeConstants() {}

    // Arm position constants (radians)
    public static final double ARM_STOWED_POSITION = 0.0;
    public static final double ARM_DEPLOYED_POSITION = -8.0;

    // Motor CAN IDs
    public static final int ROLLER_MOTOR_ID = 20;
    public static final int ARM_MOTOR_ID = 19;
    public static final int INDEXER_MOTOR_ID = 17;

    // Gear ratios
    public static final double ROLLER_GEAR_RATIO = 3.0;
    public static final double ARM_GEAR_RATIO = 1.0;
    public static final double INDEXER_GEAR_RATIO = 3.0;

    // Ball detection threshold (Amps)
    public static final double BALL_DETECTION_CURRENT_THRESHOLD = 30.0;

    // Current limits (Amps)
    public static final double ROLLER_CURRENT_LIMIT = 60.0;
    public static final double INDEXER_CURRENT_LIMIT = 60.0;
    public static final double ARM_CURRENT_LIMIT = 30.0;

    // Arm PID gains
    public static final double ARM_KP = 100.0;
    public static final double ARM_KI = 0.0;
    public static final double ARM_KD = 0.5;
    public static final double ARM_KS = 0.0;
    public static final double ARM_KV = 0.0;

    // Simulation parameters
    public static final double SIM_ROLLER_MOI = 0.001; // kg*m^2
    public static final double SIM_INDEXER_MOI = 0.001; // kg*m^2
    public static final double SIM_ARM_MOI = 0.1; // kg*m^2
    public static final double SIM_ARM_GEAR_RATIO = 100.0;
    public static final double SIM_ARM_MIN_ANGLE = -10.0; // radians
    public static final double SIM_ARM_MAX_ANGLE = 1.0; // radians
}
