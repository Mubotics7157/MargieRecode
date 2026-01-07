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

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.lib.pathplanner.auto.AutoBuilder;
import frc.robot.lib.pathplanner.path.PathConstraints;
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
    private static final double DEADBAND = 0.1;
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    private DriveCommands() {}

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }

    /** Field relative drive command using two joysticks (controlling linear and angular velocities). */
    public static Command joystickDrive(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
        return Commands.run(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity =
                            getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                    // Apply rotation deadband
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Square rotation value for more precise control
                    omega = Math.copySign(omega * omega, omega);

                    // Calculate field-relative velocities
                    // CTRE's FieldCentric with OperatorPerspective handles alliance flipping
                    double vx = linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec();
                    double vy = linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec();
                    double omegaRadPerSec = omega * drive.getMaxAngularSpeedRadPerSec();

                    // Use CTRE's field-centric request directly - handles all optimization internally
                    drive.runVelocityFieldCentric(vx, vy, omegaRadPerSec);
                },
                drive);
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control. Possible use cases
     * include snapping to an angle, aiming at a vision target, or controlling absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> rotationSupplier) {

        // Create PID controller
        ProfiledPIDController angleController = new ProfiledPIDController(
                ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.run(
                        () -> {
                            // Get linear velocity
                            Translation2d linearVelocity =
                                    getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                            // Calculate angular speed from PID
                            double omegaRadPerSec = angleController.calculate(
                                    drive.getRotation().getRadians(),
                                    rotationSupplier.get().getRadians());

                            // Calculate field-relative velocities
                            // CTRE's FieldCentric with OperatorPerspective handles alliance flipping
                            double vx = linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec();
                            double vy = linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec();

                            // Use CTRE's field-centric request directly
                            drive.runVelocityFieldCentric(vx, vy, omegaRadPerSec);
                        },
                        drive)

                // Reset PID controller when command starts
                .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
    }

    /**
     * Field relative drive command that maintains heading when not actively turning. Based on Team 254's
     * DriveMaintainingHeadingCommand.
     *
     * <p>Features:
     *
     * <ul>
     *   <li>Dual mode control: Manual rotation vs heading maintenance
     *   <li>Mode switching: 0.25s timeout after last rotation input
     *   <li>Heading capture: Stores current heading when entering maintenance mode
     *   <li>Cardinal snap angles: Optional snap-to-angle for common directions (0, 90, 180, 270)
     * </ul>
     */
    public static Command joystickDriveMaintainHeading(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            boolean enableCardinalSnap) {

        // Constants
        final double STEER_DEADBAND = 0.1;
        final double MODE_SWITCH_TIMEOUT_SECONDS = 0.25;
        final double ROTATION_VELOCITY_THRESHOLD_DEG_PER_SEC = 10.0;
        final double HEADING_KP = 5.0;
        final double HEADING_KI = 0.0;
        final double HEADING_KD = 0.0;
        final double[] CARDINAL_ANGLES = {0.0, Math.PI / 2, Math.PI, -Math.PI / 2};
        final double SNAP_THRESHOLD_RAD = Math.toRadians(15.0);

        // State holder class for the command
        class HeadingState {
            Optional<Rotation2d> headingSetpoint = Optional.empty();
            double joystickLastTouched = -1;
        }
        final HeadingState state = new HeadingState();

        // Swerve requests - deadbands proportional to max speeds (per 254)
        final double maxSpeed = drive.getMaxLinearSpeedMetersPerSec();
        final double maxAngularRate = drive.getMaxAngularSpeedRadPerSec();

        final SwerveRequest.FieldCentric driveNoHeading = new SwerveRequest.FieldCentric()
                .withDeadband(maxSpeed * 0.05)
                .withRotationalDeadband(maxAngularRate * STEER_DEADBAND)
                .withDriveRequestType(
                        Constants.currentMode == Constants.Mode.SIM
                                ? SwerveModule.DriveRequestType.OpenLoopVoltage
                                : SwerveModule.DriveRequestType.Velocity)
                .withDesaturateWheelSpeeds(true);

        final SwerveRequest.FieldCentricFacingAngle driveWithHeading = new SwerveRequest.FieldCentricFacingAngle()
                .withDeadband(maxSpeed * 0.05)
                .withDriveRequestType(
                        Constants.currentMode == Constants.Mode.SIM
                                ? SwerveModule.DriveRequestType.OpenLoopVoltage
                                : SwerveModule.DriveRequestType.Velocity)
                .withDesaturateWheelSpeeds(true);

        // Configure heading controller PID
        driveWithHeading.HeadingController.setPID(HEADING_KP, HEADING_KI, HEADING_KD);

        final RobotState robotState = RobotState.getInstance();

        return Commands.run(
                        () -> {
                            // Get joystick inputs
                            Translation2d linearVelocity =
                                    getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                            double turn = omegaSupplier.getAsDouble();

                            // Scale to max speeds (using maxSpeed and maxAngularRate from outer scope)
                            double throttle = linearVelocity.getX() * maxSpeed;
                            double strafe = linearVelocity.getY() * maxSpeed;

                            // Handle alliance flipping
                            double throttleFieldFrame = robotState.isRedAlliance() ? -throttle : throttle;
                            double strafeFieldFrame = robotState.isRedAlliance() ? -strafe : strafe;

                            // Track when turn input was last provided
                            if (Math.abs(turn) > STEER_DEADBAND) {
                                state.joystickLastTouched = Timer.getFPGATimestamp();
                            }

                            // Get current rotation velocity for mode switching
                            double currentOmegaDegPerSec =
                                    Math.toDegrees(robotState.getMeasuredSpeedsRobotRelative().omegaRadiansPerSecond);

                            // Check if within timeout
                            boolean withinTimeout = state.joystickLastTouched >= 0
                                    && Timer.getFPGATimestamp() - state.joystickLastTouched
                                            < MODE_SWITCH_TIMEOUT_SECONDS;

                            // Determine if we should be in manual turn mode or heading maintenance mode
                            boolean manualTurnMode = Math.abs(turn) > STEER_DEADBAND
                                    || (withinTimeout
                                            && Math.abs(currentOmegaDegPerSec)
                                                    > ROTATION_VELOCITY_THRESHOLD_DEG_PER_SEC);

                            if (manualTurnMode) {
                                // Manual turn mode - direct rotational control (no squaring, per 254)
                                double turnRate = Math.copySign(turn * turn, turn) * maxAngularRate;

                                drive.setControl(driveNoHeading
                                        .withVelocityX(throttleFieldFrame)
                                        .withVelocityY(strafeFieldFrame)
                                        .withRotationalRate(turnRate));

                                state.headingSetpoint = Optional.empty();
                                Logger.recordOutput("DriveMaintainHeading/Mode", "Manual");
                            } else {
                                // Heading maintenance mode
                                if (state.headingSetpoint.isEmpty()) {
                                    // Capture current heading when entering maintenance mode
                                    Rotation2d currentHeading = drive.getRotation();

                                    // Optionally snap to nearest cardinal angle
                                    if (enableCardinalSnap) {
                                        double currentRad = currentHeading.getRadians();
                                        Rotation2d snapped = currentHeading;
                                        for (double cardinalRad : CARDINAL_ANGLES) {
                                            double diff = MathUtil.angleModulus(currentRad - cardinalRad);
                                            if (Math.abs(diff) < SNAP_THRESHOLD_RAD) {
                                                snapped = new Rotation2d(cardinalRad);
                                                Logger.recordOutput(
                                                        "DriveMaintainHeading/SnappedToCardinal",
                                                        Math.toDegrees(cardinalRad));
                                                break;
                                            }
                                        }
                                        state.headingSetpoint = Optional.of(snapped);
                                    } else {
                                        state.headingSetpoint = Optional.of(currentHeading);
                                    }
                                }

                                drive.setControl(driveWithHeading
                                        .withVelocityX(throttleFieldFrame)
                                        .withVelocityY(strafeFieldFrame)
                                        .withTargetDirection(state.headingSetpoint.get()));

                                Logger.recordOutput("DriveMaintainHeading/Mode", "Maintaining");
                                Logger.recordOutput(
                                        "DriveMaintainHeading/HeadingSetpoint",
                                        state.headingSetpoint.get().getDegrees());
                            }

                            // Log state
                            Logger.recordOutput("DriveMaintainHeading/ThrottleFieldFrame", throttleFieldFrame);
                            Logger.recordOutput("DriveMaintainHeading/StrafeFieldFrame", strafeFieldFrame);
                        },
                        drive)
                .beforeStarting(() -> {
                    state.headingSetpoint = Optional.empty();
                    state.joystickLastTouched = -1;
                });
    }

    /**
     * Field relative drive command that maintains heading when not actively turning. Cardinal snap angles enabled by
     * default.
     */
    public static Command joystickDriveMaintainHeading(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
        return joystickDriveMaintainHeading(drive, xSupplier, ySupplier, omegaSupplier, true);
    }

    // ==================== DYNAMIC PATHFINDING COMMANDS ====================

    // Pathfinding constraints - based on Team 254's approach
    private static final double PATHFIND_SPEED_SCALING = 0.8;
    private static final double PATHFIND_ACCEL_SCALING = 0.8;
    private static final double PATHFIND_ANGULAR_SCALING = 0.8;

    /**
     * Dynamically pathfind to a shooting position at the optimal distance from the hub. Based on Team 254's dynamic
     * pathfinding approach.
     *
     * <p>This command uses the AD* pathfinding algorithm to navigate around obstacles and find the optimal path to a
     * shooting position.
     *
     * @param drive The drive subsystem
     * @param shootingDistanceMeters Distance from the hub center to stop at for shooting
     * @return A command that pathfinds to the shooting position
     */
    public static Command pathfindToShootingPosition(Drive drive, double shootingDistanceMeters) {
        return Commands.defer(
                () -> {
                    // Get current robot pose
                    Pose2d currentPose = drive.getPose();
                    Translation2d hubPosition = FieldConstants.getHubPosition();

                    // Calculate the shooting position: on the line from hub to robot, at the desired distance
                    Translation2d robotToHub = hubPosition.minus(currentPose.getTranslation());
                    Rotation2d angleToHub = robotToHub.getAngle();

                    // Target position is shootingDistanceMeters away from hub, facing the hub
                    Translation2d targetTranslation =
                            hubPosition.minus(new Translation2d(shootingDistanceMeters, angleToHub));
                    Pose2d targetPose = new Pose2d(targetTranslation, angleToHub);

                    // Create path constraints
                    double maxSpeed = drive.getMaxLinearSpeedMetersPerSec() * PATHFIND_SPEED_SCALING;
                    double maxAccel = 3.0 * PATHFIND_ACCEL_SCALING; // m/s^2
                    double maxAngularSpeed = drive.getMaxAngularSpeedRadPerSec() * PATHFIND_ANGULAR_SCALING;
                    double maxAngularAccel = Math.PI * 2 * PATHFIND_ANGULAR_SCALING; // rad/s^2

                    // X and Y acceleration limits (same as max accel for symmetric constraints)
                    PathConstraints constraints = new PathConstraints(
                            maxSpeed, maxAccel, maxAngularSpeed, maxAngularAccel, maxAccel, maxAccel);

                    Logger.recordOutput("Pathfinding/TargetPose", targetPose);
                    Logger.recordOutput("Pathfinding/ShootingDistance", shootingDistanceMeters);

                    // Use AutoBuilder to create the pathfinding command
                    return AutoBuilder.pathfindToPoseFlipped(targetPose, constraints);
                },
                java.util.Set.of(drive));
    }

    /**
     * Dynamically pathfind to a specific field pose. Based on Team 254's dynamic pathfinding approach.
     *
     * @param drive The drive subsystem
     * @param targetPose The target pose to pathfind to (blue alliance origin)
     * @return A command that pathfinds to the target pose
     */
    public static Command pathfindToPose(Drive drive, Pose2d targetPose) {
        return Commands.defer(
                () -> {
                    double maxSpeed = drive.getMaxLinearSpeedMetersPerSec() * PATHFIND_SPEED_SCALING;
                    double maxAccel = 3.0 * PATHFIND_ACCEL_SCALING;
                    double maxAngularSpeed = drive.getMaxAngularSpeedRadPerSec() * PATHFIND_ANGULAR_SCALING;
                    double maxAngularAccel = Math.PI * 2 * PATHFIND_ANGULAR_SCALING;

                    PathConstraints constraints = new PathConstraints(
                            maxSpeed, maxAccel, maxAngularSpeed, maxAngularAccel, maxAccel, maxAccel);

                    Logger.recordOutput("Pathfinding/TargetPose", targetPose);

                    return AutoBuilder.pathfindToPoseFlipped(targetPose, constraints);
                },
                java.util.Set.of(drive));
    }

    /**
     * Dynamically pathfind to a specific field pose with custom constraints.
     *
     * @param drive The drive subsystem
     * @param targetPose The target pose to pathfind to (blue alliance origin)
     * @param maxVelocityMps Maximum velocity in meters per second
     * @param maxAccelMpsSq Maximum acceleration in meters per second squared
     * @return A command that pathfinds to the target pose
     */
    public static Command pathfindToPose(Drive drive, Pose2d targetPose, double maxVelocityMps, double maxAccelMpsSq) {
        return Commands.defer(
                () -> {
                    double maxAngularSpeed = drive.getMaxAngularSpeedRadPerSec() * PATHFIND_ANGULAR_SCALING;
                    double maxAngularAccel = Math.PI * 2 * PATHFIND_ANGULAR_SCALING;

                    PathConstraints constraints = new PathConstraints(
                            maxVelocityMps,
                            maxAccelMpsSq,
                            maxAngularSpeed,
                            maxAngularAccel,
                            maxAccelMpsSq,
                            maxAccelMpsSq);

                    Logger.recordOutput("Pathfinding/TargetPose", targetPose);

                    return AutoBuilder.pathfindToPoseFlipped(targetPose, constraints);
                },
                java.util.Set.of(drive));
    }

    // ==================== CHARACTERIZATION COMMANDS ====================

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(() -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),

                // Allow modules to orient
                Commands.run(
                                () -> {
                                    drive.runCharacterization(0.0);
                                },
                                drive)
                        .withTimeout(FF_START_DELAY),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(
                                () -> {
                                    double voltage = timer.get() * FF_RAMP_RATE;
                                    drive.runCharacterization(voltage);
                                    velocitySamples.add(drive.getFFCharacterizationVelocity());
                                    voltageSamples.add(voltage);
                                },
                                drive)

                        // When cancelled, calculate and print results
                        .finallyDo(() -> {
                            int n = velocitySamples.size();
                            double sumX = 0.0;
                            double sumY = 0.0;
                            double sumXY = 0.0;
                            double sumX2 = 0.0;
                            for (int i = 0; i < n; i++) {
                                sumX += velocitySamples.get(i);
                                sumY += voltageSamples.get(i);
                                sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                            }
                            double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                            double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                            NumberFormat formatter = new DecimalFormat("#0.00000");
                            System.out.println("********** Drive FF Characterization Results **********");
                            System.out.println("\tkS: " + formatter.format(kS));
                            System.out.println("\tkV: " + formatter.format(kV));
                        }));
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                        // Reset acceleration limiter
                        Commands.runOnce(() -> {
                            limiter.reset(0.0);
                        }),

                        // Turn in place, accelerating up to full speed
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                                },
                                drive)),

                // Measurement sequence
                Commands.sequence(
                        // Wait for modules to fully orient before starting measurement
                        Commands.waitSeconds(1.0),

                        // Record starting measurement
                        Commands.runOnce(() -> {
                            state.positions = drive.getWheelRadiusCharacterizationPositions();
                            state.lastAngle = drive.getRotation();
                            state.gyroDelta = 0.0;
                        }),

                        // Update gyro delta
                        Commands.run(() -> {
                                    var rotation = drive.getRotation();
                                    state.gyroDelta += Math.abs(
                                            rotation.minus(state.lastAngle).getRadians());
                                    state.lastAngle = rotation;
                                })

                                // When cancelled, calculate and print results
                                .finallyDo(() -> {
                                    double[] positions = drive.getWheelRadiusCharacterizationPositions();
                                    double wheelDelta = 0.0;
                                    for (int i = 0; i < 4; i++) {
                                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                                    }
                                    double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                                    NumberFormat formatter = new DecimalFormat("#0.000");
                                    System.out.println("********** Wheel Radius Characterization Results **********");
                                    System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                                    System.out.println(
                                            "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                                    System.out.println("\tWheel Radius: "
                                            + formatter.format(wheelRadius)
                                            + " meters, "
                                            + formatter.format(Units.metersToInches(wheelRadius))
                                            + " inches");
                                })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }
}
