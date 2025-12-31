package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
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
    private final Shooter shooter;
    private final SuperstructureIO io;

    private Goal currentGoal = Goal.IDLE;
    private Goal desiredGoal = Goal.IDLE;

    public Superstructure(Intake intake, Shooter shooter, SuperstructureIO io) {
        this.intake = intake;
        this.shooter = shooter;
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
                shooter.stopIndexer();
                shooter.stopPooper();
                shooter.disable();
                break;
            case INTAKING:
                intake.deployArm();
                intake.setRollerDutyCycle(0.5);
                intake.setIndexerDutyCycle(0.5);
                // Stop indexer and pooper when ball is detected to hold it in place
                if (shooter.isBallDetectedInShooter()) {
                    shooter.stopIndexer();
                    shooter.stopPooper();
                } else {
                    shooter.runIndexer(50.0); // Run starwheels to feed ball
                    shooter.feedBall(); // Run pooper to continue ball through path
                }
                break;
            case SHOOTING:
                intake.stowArm();
                intake.setRollerDutyCycle(0.0);
                // Wait for flywheels to reach target RPM before feeding ball
                if (shooter.flywheelAtSetpoint()) {
                    intake.feedToShooter();
                    shooter.runIndexer(50.0); // Run starwheels to feed ball
                    shooter.feedBall(); // Run pooper to continue ball through path
                } else {
                    intake.setIndexerDutyCycle(0.0);
                    shooter.stopIndexer();
                    shooter.stopPooper();
                }
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

    public Shooter getShooter() {
        return this.shooter;
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
