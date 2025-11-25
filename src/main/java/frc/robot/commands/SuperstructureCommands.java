package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.Superstructurestate;
import frc.robot.subsystems.superstructure.Superstructure;

public class SuperstructureCommands {
    //Command to intake gamepiece with no time limits
    public static Command intake(Superstructure superstructure) {
        return Commands.sequence(
            Commands.runOnce(() -> superstructure.requestIntake(), superstructure),
            Commands.waitUntil(() -> superstructure.hasGamePiece()),
            Commands.runOnce(() -> superstructure.requestIdle(), superstructure)
        ).withName("Intake").alongWith(Commands.run(() -> {}, superstructure.getIntake()));
    }

    //Command to intake gamepiece with timeout, stops trying to intake after timeout seconds no matter if it has gamepiece or not
    public static Command intakeWithTimeout(Superstructure superstructure, double timeout) {
        return Commands.sequence(
            Commands.runOnce(() -> superstructure.requestIntake(), superstructure),
            Commands.either(
                Commands.waitUntil(() -> superstructure.hasGamePiece()),
                Commands.waitSeconds(timeout),
                () -> !superstructure.hasGamePiece()
            ),
            Commands.runOnce(() -> superstructure.requestIdle(), superstructure)
        ).withName("IntakeWithTimeout").alongWith(Commands.run(() -> {}, superstructure.getIntake()));
    }

    //Command to manually run intake
    public static Command manualIntake(Superstructure superstructure) {
        return Commands.startEnd(() -> superstructure.requestIntake(), () -> superstructure.requestIdle(), superstructure)
        .withName("ManualIntake").alongWith(Commands.run(() -> {}, superstructure.getIntake()));
    }

    //Command to outtake gamepiece
    public static Command outtake(Superstructure superstructure, double duration){
        return Commands.sequence(
            Commands.runOnce(() -> superstructure.requestOuttake(), superstructure),
            Commands.waitSeconds(duration),
            Commands.runOnce(() -> superstructure.requestIdle(), superstructure)
        ).withName("Eject");
    }

    //Command to manually outtake
    public static Command manualOuttake(Superstructure superstructure) {
        return Commands.startEnd(() -> superstructure.requestOuttake(), () -> superstructure.requestIdle(), superstructure).withName("ManualEject");
    }

    //Command to stow intake
    public static Command stow(Superstructure superstructure){
        return Commands.runOnce(()-> superstructure.requestIdle(), superstructure).withName("Stow");
    }

    //Command to wait until superstructure is ready
    public static Command waitForReady(Superstructure superstructure){
        return Commands.waitUntil(() -> superstructure.isReady()).withName("WaitForReady");
    }

    //Command to prep for shooting
    //REMMEBER TO FIX IMPLEMENTATION WHEN SHOOTER IS FULLY DONE
    public static Command prepareToShoot(Superstructure superstructure){
        return Commands.sequence(
            Commands.runOnce(() -> superstructure.requestShootPrepare(), superstructure),
            Commands.waitUntil(() -> superstructure.getCurrentState() == Superstructurestate.PREPARING_TO_SHOOT && superstructure.isReady())
        ).withName("PrepareToShoot");
    }

    //Command to shoot
    //REmember TO FIX IMPLEMENTATION WHEN SHOOTER IS FULLY DONE
    public static Command shoot(Superstructure superstructure){
        return Commands.sequence(
            Commands.runOnce(() -> superstructure.requestShoot(), superstructure),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> superstructure.requestIdle(), superstructure)
        ).withName("Shoot");
    }

    //Command to emergency stop the superstructure
    public static Command emergencyStop(Superstructure superstructure){
        return Commands.runOnce(() -> superstructure.emergencyStop(), superstructure).withName("EmergencyStop");
    }
}
