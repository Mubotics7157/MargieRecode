package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.AimingParameters;
import frc.robot.util.ShotParameters;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Singleton class for managing robot state and calculating aiming parameters. Following Team 254's convention for
 * centralized state management.
 */
public class RobotState {
    private static RobotState instance;

    private Supplier<Pose2d> poseSupplier = () -> new Pose2d();
    private AimingParameters latestAimingParameters = AimingParameters.create(new Rotation2d(), 0, 0);

    /** Robot height for shot trajectory calculations (shooter exit height). */
    private static final double SHOOTER_HEIGHT_METERS = Units.inchesToMeters(24.0);

    private RobotState() {}

    public static synchronized RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    /** Sets the pose supplier for robot position. */
    public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    /** Returns the current robot pose. */
    public Pose2d getPose() {
        return poseSupplier.get();
    }

    /** Calculates and returns the aiming parameters for the hub target (center of field). */
    public AimingParameters calculateHubAimingParameters() {
        Pose2d robotPose = getPose();
        Translation3d hubPos = FieldConstants.getHub3d();
        Translation2d hubPos2d = hubPos.toTranslation2d();

        // Calculate vector from robot to hub
        Translation2d robotToHub = hubPos2d.minus(robotPose.getTranslation());

        // Calculate heading to face the hub
        Rotation2d targetHeading = robotToHub.getAngle();

        // Calculate horizontal distance
        double horizontalDistance = robotToHub.getNorm();

        // Calculate effective distance (accounting for height difference)
        double heightDifference = hubPos.getZ() - SHOOTER_HEIGHT_METERS;
        double effectiveDistance =
                Math.sqrt(horizontalDistance * horizontalDistance + heightDifference * heightDifference);

        latestAimingParameters = AimingParameters.create(targetHeading, horizontalDistance, effectiveDistance);

        // Log aiming data
        Logger.recordOutput("RobotState/TargetHeading", targetHeading.getDegrees());
        Logger.recordOutput("RobotState/DistanceToHub", horizontalDistance);
        Logger.recordOutput("RobotState/EffectiveDistance", effectiveDistance);
        Logger.recordOutput("RobotState/InRange", latestAimingParameters.inRange());
        Logger.recordOutput("RobotState/HubTarget", new Pose2d(hubPos2d, new Rotation2d()));

        return latestAimingParameters;
    }

    /** Returns the latest calculated aiming parameters without recalculating. */
    @AutoLogOutput(key = "RobotState/AimingValid")
    public AimingParameters getLatestAimingParameters() {
        return latestAimingParameters;
    }

    /** Returns the heading error in degrees for the current aiming target. */
    @AutoLogOutput(key = "RobotState/HeadingErrorDeg")
    public double getHeadingErrorDegrees() {
        return latestAimingParameters
                .targetHeading()
                .minus(getPose().getRotation())
                .getDegrees();
    }

    /** Returns true if the robot is currently aligned with the hub target. */
    @AutoLogOutput(key = "RobotState/IsAligned")
    public boolean isAlignedToHub() {
        return latestAimingParameters.isAligned(getPose().getRotation());
    }

    /** Returns true if the robot is within shooting range. */
    @AutoLogOutput(key = "RobotState/InShootingRange")
    public boolean isInShootingRange() {
        return latestAimingParameters.inRange();
    }

    /** Returns true if ready to shoot (aligned and in range). */
    @AutoLogOutput(key = "RobotState/ReadyToShoot")
    public boolean isReadyToShoot() {
        return isAlignedToHub() && isInShootingRange();
    }

    /**
     * Gets interpolated shot parameters based on current distance to target. Uses the shot lookup table from
     * ShooterConstants.
     *
     * @return Interpolated shot parameters for current distance
     */
    public ShotParameters getShotParameters() {
        double distance = latestAimingParameters.distanceToTarget();
        ShotParameters params = ShooterConstants.getShotParameters(distance);

        // Log shot parameters
        Logger.recordOutput("RobotState/ShotFlywheelRPM", params.getFlywheelRPM());
        Logger.recordOutput("RobotState/ShotHoodAngleDegrees", params.getHoodAngleDegrees());

        return params;
    }

    /**
     * Gets shot parameters for a specific distance. Useful for testing or manual override.
     *
     * @param distanceMeters The distance in meters
     * @return Interpolated shot parameters for the given distance
     */
    public ShotParameters getShotParametersForDistance(double distanceMeters) {
        return ShooterConstants.getShotParameters(distanceMeters);
    }
}
