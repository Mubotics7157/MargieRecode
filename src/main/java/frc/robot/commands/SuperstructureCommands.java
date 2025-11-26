package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.Superstructure;

public class SuperstructureCommands {

    // Command to return to idle state
    public static Command returnToIdle(Superstructure superstructure) {
        return Commands.runOnce(() -> superstructure.requestIdle(), superstructure)
                .withName("ReturnToIdle");
    }

    public static Command intake(Superstructure superstructure) {
        return Commands.startEnd(
                        () -> superstructure.requestIntake(), 
                        () -> superstructure.requestIdle(), 
                        superstructure)
                .withName("intake")
                .alongWith(Commands.run(() -> {}, superstructure.getIntake()));
    }

    // Command to manually outtake
    public static Command outtake(Superstructure superstructure) {
        return Commands.startEnd(
                        () -> superstructure.requestOuttake(), 
                        () -> superstructure.requestIdle(), 
                        superstructure)
                .withName("outtake");
    }

    // Command to wait until superstructure is ready
    public static Command waitForReady(Superstructure superstructure) {
        return Commands.waitUntil(() -> superstructure.isReady()).withName("WaitForReady");
    }

    // Command to emergency stop the superstructure
    public static Command emergencyStop(Superstructure superstructure) {
        return Commands.runOnce(() -> superstructure.emergencyStop(), superstructure)
                .withName("EmergencyStop");
    }
}
