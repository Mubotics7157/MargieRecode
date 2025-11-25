package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOTalonFXSim implements IntakeIO {
    // Hardware configuration constants
    private static final double ROLLER_GEAR_RATIO = 3.0;
    private static final double ARM_GEAR_RATIO = 100.0;

    // Simulation parameters
    private static final double ROLLER_MOI = 0.001; // kg*m^2
    private static final double ARM_MOI = 0.1; // kg*m^2

    // Arm position constants (radians)
    private static final double ARM_MIN_ANGLE = 0.0;
    private static final double ARM_MAX_ANGLE = Math.PI / 2;

    // Motor simulations
    private final DCMotorSim rollerSim;
    private final DCMotorSim armSim;

    // Applied voltages
    private double rollerAppliedVolts = 0.0;
    private double armAppliedVolts = 0.0;

    // Simulated game piece detection
    private boolean simulatedGamePiece = false;

    public IntakeIOTalonFXSim() {
        // Kraken X60 for roller (1 motor, geared)
        // Using LinearSystemId to create the plant
        rollerSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), ROLLER_MOI, ROLLER_GEAR_RATIO),
                DCMotor.getKrakenX60(1));

        // Kraken X60 for arm (1 motor, highly geared)
        armSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), ARM_MOI, ARM_GEAR_RATIO),
                DCMotor.getKrakenX60(1));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // Update simulations
        rollerSim.update(0.02);
        armSim.update(0.02);

        // Clamp arm position to realistic bounds
        if (armSim.getAngularPositionRad() < ARM_MIN_ANGLE) {
            armSim.setState(ARM_MIN_ANGLE, 0.0);
        } else if (armSim.getAngularPositionRad() > ARM_MAX_ANGLE) {
            armSim.setState(ARM_MAX_ANGLE, 0.0);
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

        if (!simulatedGamePiece
                && rollerAppliedVolts > 6.0
                && Math.abs(rollerSim.getAngularVelocityRadPerSec()) > 10.0) {
            simulatedGamePiece = true;
        }

        if (simulatedGamePiece && rollerAppliedVolts < -6.0) {
            simulatedGamePiece = false;
        }

        inputs.hasGamePiece = simulatedGamePiece;
    }

    @Override
    public void setRollerVoltage(double voltage) {
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
        armSim.setState(ARM_MIN_ANGLE, 0.0);
    }

    @Override
    public void stopRoller() {
        setRollerVoltage(0.0);
    }

    // For testing - manually set game piece detection
    public void setSimulatedGamePiece(boolean hasGamePiece) {
        this.simulatedGamePiece = hasGamePiece;
    }
}
