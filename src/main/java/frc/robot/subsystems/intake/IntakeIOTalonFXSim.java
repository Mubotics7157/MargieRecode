package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOTalonFXSim implements IntakeIO {
    // Motor simulations
    private final DCMotorSim rollerSim;
    private final DCMotorSim armSim;
    private final DCMotorSim indexerSim;

    // Applied voltages
    private double rollerAppliedVolts = 0.0;
    private double indexerAppliedVolts = 0.0;
    private double armAppliedVolts = 0.0;

    // Simulated game piece detection
    private boolean simulatedGamePiece = false;

    public IntakeIOTalonFXSim() {
        // Kraken X60 for roller (1 motor, geared)
        rollerSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60(1), IntakeConstants.SIM_ROLLER_MOI, IntakeConstants.ROLLER_GEAR_RATIO),
                DCMotor.getKrakenX60(1));

        // Kraken X60 for indexer (1 motor, geared)
        indexerSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60(1), IntakeConstants.SIM_INDEXER_MOI, IntakeConstants.INDEXER_GEAR_RATIO),
                DCMotor.getKrakenX60(1));

        // Kraken X60 for arm (1 motor, highly geared)
        armSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60(1), IntakeConstants.SIM_ARM_MOI, IntakeConstants.SIM_ARM_GEAR_RATIO),
                DCMotor.getKrakenX60(1));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // Update simulations
        rollerSim.update(0.02);
        armSim.update(0.02);
        indexerSim.update(0.02);

        // Clamp arm position to realistic bounds
        if (armSim.getAngularPositionRad() < IntakeConstants.SIM_ARM_MIN_ANGLE) {
            armSim.setState(IntakeConstants.SIM_ARM_MIN_ANGLE, 0.0);
        } else if (armSim.getAngularPositionRad() > IntakeConstants.SIM_ARM_MAX_ANGLE) {
            armSim.setState(IntakeConstants.SIM_ARM_MAX_ANGLE, 0.0);
        }

        // Update roller inputs
        inputs.rollerPosition = rollerSim.getAngularPositionRad();
        inputs.rollerVelocity = rollerSim.getAngularVelocityRadPerSec();
        inputs.rollerVoltage = rollerAppliedVolts;
        inputs.rollerCurrent = rollerSim.getCurrentDrawAmps();

        // Update arm inputs
        inputs.armPosition = armSim.getAngularPositionRad();
        inputs.armVelocity = armSim.getAngularVelocityRadPerSec();
        inputs.armVoltage = armAppliedVolts;
        inputs.armCurrent = armSim.getCurrentDrawAmps();

        // Update indexer inputs
        inputs.indexerPosition = indexerSim.getAngularPositionRad();
        inputs.indexerVelocity = indexerSim.getAngularVelocityRadPerSec();
        inputs.indexerVoltage = indexerAppliedVolts;
        inputs.indexerCurrent = indexerSim.getCurrentDrawAmps();

        // Simulate ball detection
        // Ball is detected when we're intaking and have been running for a bit
        if (!simulatedGamePiece
                && rollerAppliedVolts > 6.0
                && Math.abs(rollerSim.getAngularVelocityRadPerSec()) > 10.0) {
            simulatedGamePiece = true;
        }

        // Ball is ejected when we're outtaking
        if (simulatedGamePiece && rollerAppliedVolts < -6.0) {
            simulatedGamePiece = false;
        }

        // Ball detection based on simulated game piece state
        inputs.ballDetected = simulatedGamePiece;
    }

    @Override
    public void setRollerDutyCycle(double voltage) {
        rollerAppliedVolts = voltage;
        rollerSim.setInputVoltage(voltage);
    }

    @Override
    public void setArmVoltage(double voltage) {
        armAppliedVolts = voltage;
        armSim.setInputVoltage(voltage);
    }

    @Override
    public void setArmPosition(double positionRad) {
        // Simple P controller for position control in sim
        double kP = 10.0;
        double error = positionRad - armSim.getAngularPositionRad();
        double voltage = error * kP;

        // Clamp voltage
        voltage = Math.max(-12.0, Math.min(12.0, voltage));

        setArmVoltage(voltage);
    }

    @Override
    public void resetArmPosition() {
        armSim.setState(IntakeConstants.SIM_ARM_MIN_ANGLE, 0.0);
    }

    @Override
    public void stopRoller() {
        setRollerDutyCycle(0.0);
    }

    @Override
    public void stopArmMotor() {
        setArmVoltage(0.0);
    }

    @Override
    public void setIndexerDutyCycle(double value) {
        indexerAppliedVolts = value;
        indexerSim.setInputVoltage(value);
    }

    @Override
    public void stopIndexer() {
        setIndexerDutyCycle(0.0);
    }
}
