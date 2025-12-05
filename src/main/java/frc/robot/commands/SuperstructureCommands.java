package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
}
