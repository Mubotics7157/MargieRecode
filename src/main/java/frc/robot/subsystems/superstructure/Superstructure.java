package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
    // Time constants in seconds
    private static final double StateTransitionTimeout = 2.0;

    // Position check tolerance in radians
    private static final double ArmPositionTolerance = 0.1;

    private final Intake intake;
    private final SuperstructureIO io;
    private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

    private Superstructurestate currentState = Superstructurestate.IDLE;
    private Superstructurestate desiredState = Superstructurestate.IDLE;
    private double stateStartTime = 0.0;

    public Superstructure(Intake intake, SuperstructureIO io) {
        this.intake = intake;
        this.io = io;
    }

    @Override
    public void periodic() {
        // Update inputs from subsystems
        io.updateInputs(inputs);
        Logger.processInputs("Superstructure", inputs);

        updateStateInfo();
        runStateMachine();

        // Log current states
        Logger.recordOutput("Superstructure/CurrentState", currentState.toString());
        Logger.recordOutput("Superstructure/DesiredState", desiredState.toString());
        Logger.recordOutput("Superstructure/StateTime", Timer.getFPGATimestamp() - stateStartTime);
    }

    public Intake getIntake() {
        return this.intake;
    }

    // Update current state info
    private void updateStateInfo() {
        inputs.currentState = currentState;
        inputs.desiredState = desiredState;
        inputs.stateStartTime = stateStartTime;

        // Intake status
        inputs.intakeDeployed = intake.isArmAtPosition(Intake.ARM_DEPLOYED_POSITION, ArmPositionTolerance);
        inputs.intakeArmPosition = intake.getArmPositionRad();
    }

    // Run state machine logic
    private void runStateMachine() {
        if (currentState == Superstructurestate.TRANSITIONING && isStateTimedOut()) {
            Logger.recordOutput("Superstructure/Warning", "State Timeout - Forcing Idle");
            transitionToIdle();
        }

        switch (currentState) {
            case IDLE:
                handleIdleState();
                break;
            case INTAKING:
                handleIntakingState();
                break;
            case STOWED:
                handleStowedState();
                break;
            case OUTTAKING:
                handleOuttakingState();
                break;
            case DEPLOYED:
                handleDeployedState();
                break;
            case TRANSITIONING:
                handleTransitioningState();
                break;
            case PREPARING_TO_SHOOT:
            case SHOOTING:
                transitionToIdle();
                break;
        }
    }

    private void handleIdleState() {
        switch (desiredState) {
            case INTAKING:
                intake.deployArm();
                changeState(Superstructurestate.TRANSITIONING);
                break;
            case OUTTAKING:
                intake.deployArm();
                changeState(Superstructurestate.TRANSITIONING);
                break;
            default:
                if (intake.isArmAtPosition(Intake.ARM_STOWED_POSITION, ArmPositionTolerance)) {
                    intake.stopArm(); // Stop arm if at stowed position
                }
                break;
        }
    }

    private void handleIntakingState() {
        intake.setRollerVoltage(0.8);
        intake.setArmPosition(Intake.ARM_DEPLOYED_POSITION);
        if(desiredState != Superstructurestate.INTAKING) {
            intake.stop();
            intake.stowArm();
            changeState(Superstructurestate.TRANSITIONING);
        }
    }

    private void handleTransitioningState() {
        switch (desiredState) {
            case INTAKING:
                intake.setRollerVoltage(0.5);
                changeState(Superstructurestate.INTAKING);
                break;

            case OUTTAKING:
                intake.setRollerVoltage(-0.5);
                changeState(Superstructurestate.OUTTAKING);
                break;

            case IDLE:
                intake.stop();
                changeState(Superstructurestate.IDLE);
                break;

            default:
                // Go to idle if unsure
                transitionToIdle();
                break;
        }
    }

    private void handleOuttakingState() {
        intake.setRollerVoltage(-0.5);

        if (desiredState != Superstructurestate.OUTTAKING) {
            intake.stop();
            intake.stowArm();
            changeState(Superstructurestate.TRANSITIONING);
        }
    }

    private void handleStowedState() {
        transitionToIdle();
    }

    private void handleDeployedState() {
        transitionToIdle();
    }

    private void changeState(Superstructurestate newState) {
        currentState = newState;
        stateStartTime = Timer.getFPGATimestamp();
        Logger.recordOutput("Superstructure/StateChange", "New State: " + newState.toString());
    }

    private void transitionToIdle() {
        desiredState = Superstructurestate.IDLE;
        intake.stowArm();
        intake.stop();
        changeState(Superstructurestate.TRANSITIONING);
    }

    private boolean isStateTimedOut() {
        double stateTime = Timer.getFPGATimestamp() - stateStartTime;
        return stateTime > StateTransitionTimeout;
    }

    public void requestIntake() {
        desiredState = Superstructurestate.INTAKING;
    }

    public void requestOuttake() {
        desiredState = Superstructurestate.OUTTAKING;
    }

    public void requestIdle() {
        desiredState = Superstructurestate.IDLE;
    }

    public void requestShoot() {
        if (currentState == Superstructurestate.PREPARING_TO_SHOOT) desiredState = Superstructurestate.SHOOTING;
    }

    public Superstructurestate getCurrentState() {
        return currentState;
    }

    public Superstructurestate getDesiredState() {
        return desiredState;
    }

    public boolean isReady() {
        return currentState == desiredState;
    }

    public void emergencyStop() {
        desiredState = Superstructurestate.IDLE;
        currentState = Superstructurestate.IDLE;
        intake.stop();
    }
}
