package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Goal;

public class SuperstructureCommands {

    // Standard goal command - returns to STOW when released
    public static Command intake(Superstructure superstructure) {
        return Commands.startEnd(
                () -> superstructure.setGoal(Goal.INTAKING),
                () -> superstructure.setGoal(Goal.IDLE),
                superstructure)
            .withName("Intake");
    }

    public static Command outtake(Superstructure superstructure) {
        return Commands.startEnd(
                () -> superstructure.setGoal(Goal.OUTTAKING),
                () -> superstructure.setGoal(Goal.IDLE),
                superstructure)
            .withName("Outtake");
    }

    // Wait until goal is reached (for autonomous sequencing)
    public static Command intakeUntilReady(Superstructure superstructure) {
        return Commands.sequence(
            Commands.runOnce(() -> superstructure.setGoal(Goal.INTAKING)),
            Commands.waitUntil(Superstructure::atGoal)
        ).finallyDo(() -> superstructure.setGoal(Goal.IDLE))
         .withName("IntakeUntilReady");
    }
}
