package frc.robot.subsystems.superstructure;

/**
 * Constants for the Superstructure subsystem. Contains motor duty cycles, velocities, and tolerance values for
 * coordinated subsystem control.
 */
public final class SuperstructureConstants {

    private SuperstructureConstants() {}

    // ==================== Intake Motor Duty Cycles ====================
    public static final double INTAKE_ROLLER_DUTY_CYCLE = 0.5;
    public static final double INTAKE_INDEXER_DUTY_CYCLE = 0.5;
    public static final double OUTTAKE_ROLLER_DUTY_CYCLE = -0.5;
    public static final double OUTTAKE_INDEXER_DUTY_CYCLE = -0.5;
    public static final double FEED_TO_SHOOTER_DUTY_CYCLE = 0.8;

    // ==================== Shooter Indexer Velocity ====================
    public static final double SHOOTER_INDEXER_VELOCITY = 50.0; // RPS

    // ==================== Position Tolerances ====================
    public static final double ARM_POSITION_TOLERANCE = 0.1; // radians
}
