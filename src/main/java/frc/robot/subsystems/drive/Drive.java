// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.pathplanner.auto.AutoBuilder;
import frc.robot.lib.pathplanner.config.ModuleConfig;
import frc.robot.lib.pathplanner.config.PIDConstants;
import frc.robot.lib.pathplanner.config.RobotConfig;
import frc.robot.lib.pathplanner.controllers.PPHolonomicDriveController;
import frc.robot.lib.pathplanner.controllers.PathFollowingController;
import frc.robot.lib.pathplanner.pathfinding.LocalADStar;
import frc.robot.lib.pathplanner.pathfinding.Pathfinding;
import frc.robot.lib.pathplanner.trajectory.PathPlannerTrajectory;
import frc.robot.lib.pathplanner.trajectory.PathPlannerTrajectoryState;
import frc.robot.lib.pathplanner.util.PathPlannerLogging;
import frc.robot.subsystems.vision.Vision;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Drive subsystem using Team 254's architecture - CTRE SwerveDrivetrain handles all module control and optimization
 * internally via SwerveRequest.
 */
public class Drive extends SubsystemBase implements Vision.VisionConsumer {

    // Drive base radius calculated from module positions
    public static final double DRIVE_BASE_RADIUS = Math.max(
            Math.max(
                    Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                    Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                    Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                    Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    // IO and inputs
    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    // Alerts
    private final Alert gyroDisconnectedAlert =
            new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    // SysId routine
    private final SysIdRoutine sysId;

    // Swerve requests (CTRE handles optimization internally)
    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withDesaturateWheelSpeeds(true)
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective);

    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withDesaturateWheelSpeeds(true);

    private final SwerveRequest.ApplyRobotSpeeds applySpeedsRequest = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withDesaturateWheelSpeeds(true);

    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

    private final SwerveRequest.SysIdSwerveTranslation sysIdTranslationRequest =
            new SwerveRequest.SysIdSwerveTranslation();

    // Threaded path following controller (100Hz) - Based on Team 254
    private final PathFollowingController pathController;
    private final ThreadedPathController threadedController;

    /**
     * Threaded path following controller that runs at 100Hz. Based on Team 254's implementation for smoother autonomous
     * paths.
     */
    private class ThreadedPathController implements Consumer<PathPlannerTrajectory>, Runnable {

        private final PathFollowingController controller;
        private final Notifier notifier;
        private final Timer timer;

        private volatile PathPlannerTrajectory trajectory = null;
        private boolean hasSetPriority = false;

        public ThreadedPathController(PathFollowingController controller) {
            this.controller = controller;
            this.timer = new Timer();
            this.notifier = new Notifier(this);
            this.notifier.startPeriodic(DriveConstants.THREADED_CONTROLLER_PERIOD_SECONDS);
        }

        @Override
        public void accept(PathPlannerTrajectory traj) {
            trajectory = traj;
            timer.reset();
            timer.start();
            controller.reset(null, null);
        }

        @Override
        public void run() {
            if (!hasSetPriority) {
                hasSetPriority = Threads.setCurrentThreadPriority(true, 41);
            }

            PathPlannerTrajectory traj = trajectory;
            if (traj == null) return;

            double now = timer.get();
            PathPlannerTrajectoryState targetState = traj.sample(now);

            ChassisSpeeds speeds = controller.calculateRobotRelativeSpeeds(
                    RobotState.getInstance().getLatestPose(), targetState);

            if (DriverStation.isEnabled()) {
                // Apply deadband
                if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                        < DriveConstants.LINEAR_SPEED_DEADBAND) {
                    speeds.vxMetersPerSecond = 0.0;
                    speeds.vyMetersPerSecond = 0.0;
                }
                if (Math.abs(speeds.omegaRadiansPerSecond) < DriveConstants.ANGULAR_SPEED_DEADBAND) {
                    speeds.omegaRadiansPerSecond = 0.0;
                }

                io.setControl(applySpeedsRequest
                        .withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(targetState.feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(targetState.feedforwards.robotRelativeForcesYNewtons()));
            }
        }
    }

    public Drive(DriveIO io) {
        this.io = io;

        // Create module config for PathPlanner
        ModuleConfig moduleConfig = new ModuleConfig(
                TunerConstants.FrontLeft.WheelRadius,
                TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                DriveConstants.WHEEL_COF,
                DCMotor.getKrakenX60Foc(1).withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                TunerConstants.FrontLeft.SlipCurrent,
                1);

        // Create robot config with COG height for traction-limited trajectory generation
        RobotConfig robotConfig = new RobotConfig(
                DriveConstants.ROBOT_MASS_KG,
                DriveConstants.ROBOT_MOI,
                moduleConfig,
                DriveConstants.COG_HEIGHT_METERS,
                getModuleTranslations());

        // Create path following controller with 3 PID controllers (LTE, CTE, Theta)
        pathController = new PPHolonomicDriveController(
                new PIDConstants(DriveConstants.PATH_LTE_KP, 0.0, 0.0),
                new PIDConstants(DriveConstants.PATH_CTE_KP, 0.0, 0.0),
                new PIDConstants(DriveConstants.PATH_THETA_KP, 0.0, 0.0),
                DriveConstants.THREADED_CONTROLLER_PERIOD_SECONDS);

        // Create threaded controller wrapper
        threadedController = new ThreadedPathController(pathController);

        // Configure AutoBuilder for PathPlanner using threaded controller
        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getChassisSpeeds,
                threadedController,
                robotConfig,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);

        Pathfinding.setPathfinder(new LocalADStar());
        PathPlannerLogging.setLogActivePathCallback(
                activePath -> Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0])));
        PathPlannerLogging.setLogTargetPoseCallback(
                targetPose -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

        // Configure SysId
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, state -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        voltage -> io.setControl(sysIdTranslationRequest.withVolts(voltage)), null, this));
    }

    @Override
    public void periodic() {
        // Update inputs from IO
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);

        // Log module data
        io.logModules(null);

        // Record odometry to RobotState for pose history and speed tracking
        RobotState.getInstance().recordOdometry(Timer.getFPGATimestamp(), getPose(), getChassisSpeeds());

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            io.setControl(brakeRequest);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!inputs.gyroConnected && Constants.currentMode != Mode.SIM);
    }

    /** Runs the drive at the desired velocity using field-centric control. */
    public void runVelocityFieldCentric(double vxMetersPerSec, double vyMetersPerSec, double omegaRadPerSec) {
        io.setControl(fieldCentricRequest
                .withVelocityX(vxMetersPerSec)
                .withVelocityY(vyMetersPerSec)
                .withRotationalRate(omegaRadPerSec));
    }

    /** Runs the drive at the desired velocity using robot-centric control. */
    public void runVelocity(ChassisSpeeds speeds) {
        io.setControl(applySpeedsRequest.withSpeeds(speeds));
    }

    /** Runs the drive in a straight line with the specified drive output (for characterization). */
    public void runCharacterization(double output) {
        io.setControl(sysIdTranslationRequest.withVolts(Volts.of(output)));
    }

    /** Stops the drive. */
    public void stop() {
        io.setControl(brakeRequest);
    }

    /** Stops the drive and turns the modules to an X arrangement to resist movement. */
    public void stopWithX() {
        io.setControl(brakeRequest);
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /** Returns the current module states. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    public SwerveModuleState[] getModuleStates() {
        return inputs.moduleStates;
    }

    /** Returns the measured chassis speeds. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return inputs.speeds;
    }

    /** Returns the position of each module in radians (for characterization). */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = inputs.modulePositions[i].distanceMeters / TunerConstants.FrontLeft.WheelRadius;
        }
        return values;
    }

    /** Returns the average velocity of the modules in rotations/sec (for characterization). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += inputs.moduleStates[i].speedMetersPerSecond / TunerConstants.FrontLeft.WheelRadius / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return io.getPose();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return io.getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        io.resetPose(pose);
    }

    /** Adds a new timestamped vision measurement. */
    @Override
    public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        io.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }

    /** Returns a command that applies a SwerveRequest supplier. */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return io.applyRequest(requestSupplier, this);
    }

    /** Set control directly with a SwerveRequest. */
    public void setControl(SwerveRequest request) {
        io.setControl(request);
    }
}
