package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

    public enum Goal {
        IDLE,
        INTAKING,
        SHOOTING,
        OUTTAKING;
    }

    private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

    private final Intake intake;
    private final SuperstructureIO io;

    private Goal currentGoal = Goal.IDLE;
    private Goal desiredGoal = Goal.IDLE;

    public Superstructure(Intake intake, SuperstructureIO io) {
        this.intake = intake;
        this.io = io;
    }

    @Override
    public void periodic() {
        // Process Inputs
        io.updateInputs(inputs);
        Logger.processInputs("Superstructure", inputs);

        updateGoalInfo();

        // Log current states
        Logger.recordOutput("Superstructure/CurrentGoal", currentGoal.toString());
        Logger.recordOutput("Superstructure/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput("Superstructure/AtGoal", atGoal());

        currentGoal = desiredGoal;

        switch (currentGoal) {
            case IDLE:
                intake.stowArm();
                intake.setRollerDutyCycle(0.0);
                intake.setIndexerDutyCycle(0.0);
                break;
            case INTAKING:
                intake.deployArm();
                intake.setRollerDutyCycle(0.5);
                // If ball detected, hold it in the indexer
                if (intake.isBallDetected()) {
                    intake.holdBall();
                } else {
                    intake.setIndexerDutyCycle(0.5);
                }
                break;
            case SHOOTING:
                intake.stowArm();
                intake.setRollerDutyCycle(0.0);
                // Feed ball to shooter
                intake.feedToShooter();
                break;
            case OUTTAKING:
                intake.deployArm();
                intake.setRollerDutyCycle(-0.5);
                intake.setIndexerDutyCycle(-0.5);
                break;
        }
    }

    private void updateGoalInfo() {
        inputs.currentGoal = currentGoal;
        inputs.desiredGoal = desiredGoal;
        inputs.atGoal = atGoal();
    }

    public Intake getIntake() {
        return this.intake;
    }

    public Goal getCurrentGoal() {
        return currentGoal;
    }

    public Goal getDesiredGoal() {
        return desiredGoal;
    }

    public void setGoal(Goal goal) {
        desiredGoal = goal;
    }

    public boolean atGoal() {
        if (currentGoal != desiredGoal) {
            return false;
        }

        return switch (currentGoal) {
            case IDLE -> intake.isArmAtPosition(Intake.ARM_STOWED_POSITION, 0.1);
            case INTAKING, OUTTAKING -> intake.isArmAtPosition(Intake.ARM_DEPLOYED_POSITION, 0.1);
            case SHOOTING -> intake.isArmAtPosition(Intake.ARM_STOWED_POSITION, 0.1);
        };
    }

    public boolean hasBall() {
        return intake.isBallDetected();
    }
}
