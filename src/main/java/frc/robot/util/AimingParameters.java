package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Record containing the calculated aiming parameters for shooting at a target. Following Team 254's convention for
 * structured aiming data.
 */
public record AimingParameters(
        Rotation2d targetHeading, double distanceToTarget, double effectiveDistance, boolean inRange) {

    /** Tolerance for heading alignment in radians. */
    public static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(2.0);

    /** Maximum effective shooting distance in meters. */
    public static final double MAX_SHOOTING_DISTANCE = 6.0;

    /** Minimum effective shooting distance in meters. */
    public static final double MIN_SHOOTING_DISTANCE = 1.0;

    /**
     * Creates aiming parameters with automatic range check.
     *
     * @param targetHeading The heading the robot should face to aim at target
     * @param distanceToTarget The straight-line distance to the target
     * @param effectiveDistance The effective distance accounting for height
     */
    public static AimingParameters create(Rotation2d targetHeading, double distanceToTarget, double effectiveDistance) {
        boolean inRange = distanceToTarget >= MIN_SHOOTING_DISTANCE && distanceToTarget <= MAX_SHOOTING_DISTANCE;
        return new AimingParameters(targetHeading, distanceToTarget, effectiveDistance, inRange);
    }

    /**
     * Checks if the given current heading is aligned with the target heading.
     *
     * @param currentHeading The robot's current heading
     * @return true if aligned within tolerance
     */
    public boolean isAligned(Rotation2d currentHeading) {
        double error = Math.abs(targetHeading.minus(currentHeading).getRadians());
        return error < HEADING_TOLERANCE_RADIANS;
    }
}
