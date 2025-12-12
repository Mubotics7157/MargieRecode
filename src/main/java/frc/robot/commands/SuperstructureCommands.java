package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Goal;

public class SuperstructureCommands {

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
                        Commands.waitUntil(superstructure::atGoal))
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
        return Commands.sequence(
                        // Configure and enable the shooter
                        Commands.runOnce(() -> {
                            shooter.configureSpeakerShot();
                            shooter.enable();
                        }),
                        // Wait for shooter and hood to reach setpoints
                        Commands.waitUntil(shooter::isReadyToShoot).withTimeout(2.0),
                        // Feed the ball
                        Commands.runOnce(() -> superstructure.setGoal(Goal.SHOOTING)),
                        // Keep feeding for 0.5 seconds
                        Commands.waitSeconds(0.5),
                        // Return to idle
                        Commands.runOnce(() -> {
                            superstructure.setGoal(Goal.IDLE);
                            shooter.disable();
                        }))
                .withName("Shoot");
    }

    // Auto intake command - intakes until ball is detected
    public static Command autoIntake(Superstructure superstructure) {
        return Commands.sequence(
                        Commands.runOnce(() -> superstructure.setGoal(Goal.INTAKING)),
                        Commands.waitUntil(superstructure::hasBall).withTimeout(3.0),
                        Commands.runOnce(() -> superstructure.setGoal(Goal.IDLE)))
                .withName("AutoIntake");
    }

    // Prepare to shoot command - spins up shooter but doesn't feed
    public static Command prepareShooter(Shooter shooter) {
        return Commands.runOnce(() -> {
                    shooter.configureSpeakerShot();
                    shooter.enable();
                })
                .withName("PrepareShooter");
    }

    // Stop shooter command
    public static Command stopShooter(Shooter shooter) {
        return Commands.runOnce(shooter::disable).withName("StopShooter");
    }

    // Amp shot command - lower velocity, higher angle
    public static Command ampShot(Superstructure superstructure, Shooter shooter) {
        return Commands.sequence(
                        Commands.runOnce(() -> {
                            shooter.configureAmpShot();
                            shooter.enable();
                        }),
                        Commands.waitUntil(shooter::isReadyToShoot).withTimeout(2.0),
                        Commands.runOnce(() -> superstructure.setGoal(Goal.SHOOTING)),
                        Commands.waitSeconds(0.5),
                        Commands.runOnce(() -> {
                            superstructure.setGoal(Goal.IDLE);
                            shooter.disable();
                        }))
                .withName("AmpShot");
    }

    // Long distance shot command
    public static Command longShot(Superstructure superstructure, Shooter shooter) {
        return Commands.sequence(
                        Commands.runOnce(() -> {
                            shooter.configureLongShot();
                            shooter.enable();
                        }),
                        Commands.waitUntil(shooter::isReadyToShoot).withTimeout(2.0),
                        Commands.runOnce(() -> superstructure.setGoal(Goal.SHOOTING)),
                        Commands.waitSeconds(0.5),
                        Commands.runOnce(() -> {
                            superstructure.setGoal(Goal.IDLE);
                            shooter.disable();
                        }))
                .withName("LongShot");
    }

    // Custom shot command with parameters
    public static Command customShot(
            Superstructure superstructure,
            Shooter shooter,
            double flywheelRPM,
            double flywheel2InRPM,
            double hoodDegrees) {
        return Commands.sequence(
                        Commands.runOnce(() -> {
                            shooter.configureShot(flywheelRPM, flywheel2InRPM, hoodDegrees);
                            shooter.enable();
                        }),
                        Commands.waitUntil(shooter::isReadyToShoot).withTimeout(3.0),
                        Commands.runOnce(() -> superstructure.setGoal(Goal.SHOOTING)),
                        Commands.waitSeconds(0.5),
                        Commands.runOnce(() -> {
                            superstructure.setGoal(Goal.IDLE);
                            shooter.disable();
                        }))
                .withName("CustomShot");
    }
}
