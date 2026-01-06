package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.AimingParameters;
import frc.robot.util.ConcurrentTimeInterpolatableBuffer;
import frc.robot.util.ShotParameters;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Singleton class for managing robot state and calculating aiming parameters. Following Team 254's convention for
 * centralized state management. Enhanced with pose history buffer and concurrent chassis speeds tracking.
 */
public class RobotState {
    private static RobotState instance;

    // Pose history buffer (1.0 second history) - Based on Team 254
    private static final double POSE_BUFFER_HISTORY_SECONDS = 1.0;
    private final ConcurrentTimeInterpolatableBuffer<Pose2d> poseBuffer =
            ConcurrentTimeInterpolatableBuffer.createBuffer(POSE_BUFFER_HISTORY_SECONDS);

    // Concurrent chassis speeds tracking (5 variants) - Based on Team 254
    private final AtomicReference<ChassisSpeeds> measuredSpeedsRobotRelative =
            new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> measuredSpeedsFieldRelative =
            new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> desiredSpeedsRobotRelative =
            new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> desiredSpeedsFieldRelative =
            new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> fusedSpeeds = new AtomicReference<>(new ChassisSpeeds());

    // Latest pose and rotation for quick access
    private final AtomicReference<Pose2d> latestPose = new AtomicReference<>(new Pose2d());
    private final AtomicReference<Rotation2d> latestRotation = new AtomicReference<>(new Rotation2d());

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

    // ==================== POSE BUFFER METHODS (Based on Team 254) ====================

    /**
     * Record odometry data to the pose buffer. Should be called from Drive.periodic().
     *
     * @param timestamp The timestamp of the measurement
     * @param pose The robot pose at this timestamp
     * @param measuredSpeeds The measured chassis speeds (robot-relative)
     */
    public void recordOdometry(double timestamp, Pose2d pose, ChassisSpeeds measuredSpeeds) {
        poseBuffer.addSample(timestamp, pose);
        latestPose.set(pose);
        latestRotation.set(pose.getRotation());
        measuredSpeedsRobotRelative.set(measuredSpeeds);

        // Convert to field-relative speeds
        ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(measuredSpeeds, pose.getRotation());
        measuredSpeedsFieldRelative.set(fieldRelative);

        // Log pose buffer state
        Logger.recordOutput("RobotState/PoseBufferLatest", pose);
    }

    /**
     * Get the pose at a specific timestamp from the history buffer.
     *
     * @param timestamp The timestamp to query
     * @return The interpolated pose at that time, or empty if unavailable
     */
    public Optional<Pose2d> getPoseAtTime(double timestamp) {
        return poseBuffer.getSample(timestamp);
    }

    /**
     * Get the latest recorded pose from the buffer.
     *
     * @return The latest pose
     */
    public Pose2d getLatestPose() {
        return latestPose.get();
    }

    /**
     * Get the latest rotation from the buffer.
     *
     * @return The latest rotation
     */
    public Rotation2d getLatestRotation() {
        return latestRotation.get();
    }

    /**
     * Predict the robot pose at a future time based on current velocity.
     *
     * @param lookaheadSeconds How far into the future to predict
     * @return The predicted pose
     */
    public Pose2d getPredictedPose(double lookaheadSeconds) {
        Pose2d currentPose = latestPose.get();
        ChassisSpeeds speeds = measuredSpeedsRobotRelative.get();

        // Use twist to predict future pose
        Twist2d twist = new Twist2d(
                speeds.vxMetersPerSecond * lookaheadSeconds,
                speeds.vyMetersPerSecond * lookaheadSeconds,
                speeds.omegaRadiansPerSecond * lookaheadSeconds);

        return currentPose.exp(twist);
    }

    // ==================== CHASSIS SPEEDS METHODS (Based on Team 254) ====================

    /**
     * Set the desired chassis speeds (called when applying drive commands).
     *
     * @param robotRelative The desired robot-relative speeds
     * @param heading The current robot heading for field-relative conversion
     */
    public void setDesiredSpeeds(ChassisSpeeds robotRelative, Rotation2d heading) {
        desiredSpeedsRobotRelative.set(robotRelative);
        desiredSpeedsFieldRelative.set(ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, heading));
    }

    /**
     * Set the fused chassis speeds (odometry + gyro fusion).
     *
     * @param fused The fused field-relative speeds
     */
    public void setFusedSpeeds(ChassisSpeeds fused) {
        fusedSpeeds.set(fused);
    }

    /** @return Measured robot-relative chassis speeds */
    public ChassisSpeeds getMeasuredSpeedsRobotRelative() {
        return measuredSpeedsRobotRelative.get();
    }

    /** @return Measured field-relative chassis speeds */
    public ChassisSpeeds getMeasuredSpeedsFieldRelative() {
        return measuredSpeedsFieldRelative.get();
    }

    /** @return Desired robot-relative chassis speeds */
    public ChassisSpeeds getDesiredSpeedsRobotRelative() {
        return desiredSpeedsRobotRelative.get();
    }

    /** @return Desired field-relative chassis speeds */
    public ChassisSpeeds getDesiredSpeedsFieldRelative() {
        return desiredSpeedsFieldRelative.get();
    }

    /** @return Fused (odometry + gyro) field-relative chassis speeds */
    public ChassisSpeeds getFusedSpeeds() {
        return fusedSpeeds.get();
    }

    // ==================== ALLIANCE UTILITIES ====================

    /** @return True if the robot is on the red alliance */
    public boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

    /** Clear the pose buffer (e.g., when resetting odometry). */
    public void clearPoseBuffer() {
        poseBuffer.clear();
    }
}
