package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Goal;
import frc.robot.util.AimingParameters;
import frc.robot.util.ShotParameters;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class SuperstructureCommands {

    // Heading controller constants (following Team 254 conventions)
    private static final double HEADING_KP = 3.5;
    private static final double HEADING_KI = 0.0;
    private static final double HEADING_KD = 0.0;
    private static final double HEADING_MAX_VELOCITY = 8.0; // rad/s
    private static final double HEADING_MAX_ACCELERATION = 20.0; // rad/s^2

    private SuperstructureCommands() {}

    // Intake command that intakes while held, and then returns to idle when released
    public static Command intake(Superstructure superstructure) {
        return Commands.startEnd(
                        () -> superstructure.setGoal(Goal.INTAKING),
                        () -> superstructure.setGoal(Goal.IDLE),
                        superstructure)
                .withName("Intake");
    }

    // Outtake command that intakes while held, and then returns to idle when released
    public static Command outtake(Superstructure superstructure) {
        return Commands.startEnd(
                        () -> superstructure.setGoal(Goal.OUTTAKING),
                        () -> superstructure.setGoal(Goal.IDLE),
                        superstructure)
                .withName("Outtake");
    }

    // Intake command for autos, waits until the intake has reached its goal
    public static Command intakeUntilReady(Superstructure superstructure) {
        return Commands.sequence(
                        Commands.runOnce(() -> superstructure.setGoal(Goal.INTAKING)),
                        Commands.waitUntil(superstructure::hasBall))
                .finallyDo(() -> superstructure.setGoal(Goal.IDLE))
                .withName("IntakeUntilReady");
    }

    // Set superstructure state to INTAKING without a return to idle
    public static Command setIntake(Superstructure superstructure) {
        return Commands.runOnce(() -> superstructure.setGoal(Goal.INTAKING)).withName("SetIntake");
    }

    // Set superstructure state to IDLE
    public static Command setIdle(Superstructure superstructure) {
        return Commands.runOnce(() -> superstructure.setGoal(Goal.IDLE)).withName("SetIdle");
    }

    // Shoot command - spins up shooter, waits for speed and hood, then feeds ball
    public static Command shoot(Superstructure superstructure, Shooter shooter) {
        return Commands.startEnd(
                        () -> {
                            shooter.configureTestShot();
                            shooter.enable();
                            superstructure.setGoal(Goal.SHOOTING);
                        },
                        () -> {
                            superstructure.setGoal(Goal.IDLE);
                        })
                .withName("Shoot");
    }

    // Stop shooter command
    public static Command stopShooter(Shooter shooter) {
        return Commands.runOnce(shooter::disable).withName("StopShooter");
    }

    public static Command poop(Superstructure superstructure) {
        return Commands.startEnd(
                        () -> superstructure.setGoal(Goal.POOPING),
                        () -> superstructure.setGoal(Goal.IDLE),
                        superstructure)
                .withName("Poop");
    }

    /**
     * Aim and shoot command - automatically rotates robot to face hub while shooting. Allows driver to control
     * translation while heading is controlled by the aiming system. Following Team 254's convention for integrated
     * aiming.
     *
     * @param drive The drive subsystem for heading control
     * @param superstructure The superstructure for shooter coordination
     * @param shooter The shooter subsystem
     * @param xSupplier Joystick X input for translation
     * @param ySupplier Joystick Y input for translation
     */
    public static Command aimAndShoot(
            Drive drive,
            Superstructure superstructure,
            Shooter shooter,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {

        // Create profiled PID controller for smooth heading control
        ProfiledPIDController headingController = new ProfiledPIDController(
                HEADING_KP,
                HEADING_KI,
                HEADING_KD,
                new TrapezoidProfile.Constraints(HEADING_MAX_VELOCITY, HEADING_MAX_ACCELERATION));
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.run(
                        () -> {
                            // Calculate aiming parameters
                            RobotState robotState = RobotState.getInstance();
                            AimingParameters aimingParams = robotState.calculateHubAimingParameters();

                            // Update shot parameters based on current distance (dynamic tracking)
                            ShotParameters shotParams = robotState.getShotParameters();
                            shooter.updateShotParameters(shotParams);

                            // Get joystick inputs with deadband
                            double x = applyDeadband(xSupplier.getAsDouble(), 0.1);
                            double y = applyDeadband(ySupplier.getAsDouble(), 0.1);

                            // Square inputs for finer control
                            double linearMagnitude = Math.hypot(x, y);
                            linearMagnitude = linearMagnitude * linearMagnitude;
                            Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
                            double scaledX = linearMagnitude * linearDirection.getCos();
                            double scaledY = linearMagnitude * linearDirection.getSin();

                            // Calculate heading output using profiled PID
                            double omega = headingController.calculate(
                                    drive.getRotation().getRadians(),
                                    aimingParams.targetHeading().getRadians());

                            // Create chassis speeds (field-relative)
                            ChassisSpeeds speeds = new ChassisSpeeds(
                                    scaledX * drive.getMaxLinearSpeedMetersPerSec(),
                                    scaledY * drive.getMaxLinearSpeedMetersPerSec(),
                                    omega);

                            // Convert to robot-relative and apply
                            boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation()));

                            // Log aiming state
                            Logger.recordOutput("Aiming/HeadingError", headingController.getPositionError());
                            Logger.recordOutput("Aiming/AtSetpoint", headingController.atSetpoint());
                            Logger.recordOutput("Aiming/OmegaOutput", omega);
                        },
                        drive)
                .beforeStarting(() -> {
                    // Reset heading controller on command start
                    headingController.reset(drive.getRotation().getRadians());

                    // Calculate initial aiming and shot parameters
                    RobotState robotState = RobotState.getInstance();
                    robotState.calculateHubAimingParameters();
                    ShotParameters initialParams = robotState.getShotParameters();

                    // Configure shooter with initial distance-based parameters
                    shooter.configureShot(initialParams);
                    shooter.enable();
                    superstructure.setGoal(Goal.SHOOTING);
                })
                .finallyDo(() -> {
                    // Return to idle when command ends
                    superstructure.setGoal(Goal.IDLE);
                })
                .withName("AimAndShoot");
    }

    /**
     * Aim at hub command - only rotates robot to face hub without shooting. Useful for pre-aiming before triggering the
     * shot.
     *
     * @param drive The drive subsystem for heading control
     * @param xSupplier Joystick X input for translation
     * @param ySupplier Joystick Y input for translation
     */
    public static Command aimAtHub(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        ProfiledPIDController headingController = new ProfiledPIDController(
                HEADING_KP,
                HEADING_KI,
                HEADING_KD,
                new TrapezoidProfile.Constraints(HEADING_MAX_VELOCITY, HEADING_MAX_ACCELERATION));
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.run(
                        () -> {
                            RobotState robotState = RobotState.getInstance();
                            AimingParameters aimingParams = robotState.calculateHubAimingParameters();

                            double x = applyDeadband(xSupplier.getAsDouble(), 0.1);
                            double y = applyDeadband(ySupplier.getAsDouble(), 0.1);

                            double linearMagnitude = Math.hypot(x, y);
                            linearMagnitude = linearMagnitude * linearMagnitude;
                            Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
                            double scaledX = linearMagnitude * linearDirection.getCos();
                            double scaledY = linearMagnitude * linearDirection.getSin();

                            double omega = headingController.calculate(
                                    drive.getRotation().getRadians(),
                                    aimingParams.targetHeading().getRadians());

                            ChassisSpeeds speeds = new ChassisSpeeds(
                                    scaledX * drive.getMaxLinearSpeedMetersPerSec(),
                                    scaledY * drive.getMaxLinearSpeedMetersPerSec(),
                                    omega);

                            boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation()));

                            Logger.recordOutput("Aiming/HeadingError", headingController.getPositionError());
                            Logger.recordOutput("Aiming/AtSetpoint", headingController.atSetpoint());
                        },
                        drive)
                .beforeStarting(
                        () -> headingController.reset(drive.getRotation().getRadians()))
                .withName("AimAtHub");
    }

    /** Applies a deadband to the input value. */
    private static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        return (value - Math.copySign(deadband, value)) / (1.0 - deadband);
    }
}
