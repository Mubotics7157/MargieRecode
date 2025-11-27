package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.Idle;

public class Superstructure extends SubsystemBase {

    public enum Goal{
        IDLE,
        INTAKING,
        OUTTAKING;
    }

    private final Intake intake;
    private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

    private Goal currentGoal = Goal.IDLE;
    private Goal desiredGoal = Goal.IDLE;

    public Superstructure(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void periodic() {
        // Update inputs from subsystems
        Logger.processInputs("Superstructure", inputs);

        // Log current states
        Logger.recordOutput("Superstructure/CurrentState", currentGoal.toString());
        Logger.recordOutput("Superstructure/DesiredState", desiredGoal.toString());

        currentGoal = desiredGoal;

        switch (currentGoal) {
            case IDLE:
                intake.stowArm();
                intake.setRollerVoltage(0.0);
                break;
            case INTAKING:
                intake.deployArm();
                intake.setRollerVoltage(6);
                break;
            case OUTTAKING:
                intake.deployArm();
                intake.setRollerVoltage(-6);
                break;
        }
    }

    public Intake getIntake() {
        return this.intake;
    }

    public void setGoal (Goal goal){
        desiredGoal = goal;
    }
    
    public boolean atGoal(){
        if (currentGoal != desiredGoal){
            return false;
        }

        return switch (currentGoal){
            case IDLE -> intake.isArmAtPosition(Intake.ARM_STOWED_POSITION, 0.1);
            case INTAKING, OUTTAKING -> intake.isArmAtPosition(Intake.ARM_DEPLOYED_POSITION, 0.1);
        };
    }
}
