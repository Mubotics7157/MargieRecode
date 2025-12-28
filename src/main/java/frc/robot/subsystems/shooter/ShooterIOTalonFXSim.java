package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOTalonFXSim implements ShooterIO {
    // Hardware configuration constants
    private static final double FLYWHEEL_GEAR_RATIO = 1.0; // Direct drive
    private static final double HOOD_GEAR_RATIO = 50.0; // 50:1 reduction

    // Simulation parameters
    private static final double FLYWHEEL_MOI = 0.004; // kg*m^2 (flywheel mass)
    private static final double HOOD_MOI = 0.025; // kg*m^2 (hood assembly mass)

    // Hood position constants (degrees)
    private static final double HOOD_MIN_ANGLE = 0.0;
    private static final double HOOD_MAX_ANGLE = 65.0; // Supports amp shot at 60 degrees

    // Motor simulations for flywheels
    private final DCMotorSim flywheelMidSim;
    private final DCMotorSim flywheelRightSim;
    private final DCMotorSim flywheel2InSim;

    // Motor simulations for indexer
    private final DCMotorSim shooterMotorSim; // Starwheels
    private final DCMotorSim pooperSim;

    // Motor simulation for hood pivot
    private final DCMotorSim hoodSim;

    // Applied voltages
    private double flywheelAppliedVolts = 0.0;
    private double flywheel2InAppliedVolts = 0.0;
    private double indexerAppliedVolts = 0.0;
    private double pooperAppliedVolts = 0.0;
    private double hoodAppliedVolts = 0.0;

    // Control state
    private double targetFlywheelRPM = 0.0;
    private double targetFlywheel2InRPM = 0.0;
    private double targetHoodDegrees = 0.0;
    private boolean flywheelClosedLoop = false;
    private boolean flywheel2InClosedLoop = false;
    private boolean hoodClosedLoop = false;

    // Simple PID gains for simulation
    private static final double VELOCITY_KP = 0.0002;
    private static final double HOOD_KP = 0.5;

    public ShooterIOTalonFXSim() {
        // Create flywheel simulations - Kraken X60 for each flywheel (direct drive)
        flywheelMidSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), FLYWHEEL_MOI, FLYWHEEL_GEAR_RATIO),
                DCMotor.getKrakenX60(1));

        flywheelRightSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), FLYWHEEL_MOI, FLYWHEEL_GEAR_RATIO),
                DCMotor.getKrakenX60(1));

        flywheel2InSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), FLYWHEEL_MOI, FLYWHEEL_GEAR_RATIO),
                DCMotor.getKrakenX60(1));

        // Create indexer simulations
        shooterMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1.0), DCMotor.getKrakenX60(1));

        pooperSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1.0), DCMotor.getKrakenX60(1));

        // Create hood simulation - 1 Kraken X60 motor with high gear reduction
        hoodSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), HOOD_MOI, HOOD_GEAR_RATIO),
                DCMotor.getKrakenX60(1));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Update closed-loop control if enabled
        if (flywheelClosedLoop) {
            updateFlywheelControl();
        }
        if (flywheel2InClosedLoop) {
            updateFlywheel2InControl();
        }
        if (hoodClosedLoop) {
            updateHoodControl();
        }

        // Update simulations
        flywheelMidSim.update(0.02);
        flywheelRightSim.update(0.02);
        flywheel2InSim.update(0.02);
        shooterMotorSim.update(0.02);
        pooperSim.update(0.02);
        hoodSim.update(0.02);

        // Clamp hood position to realistic bounds
        double hoodPositionRad = hoodSim.getAngularPositionRad() / HOOD_GEAR_RATIO;
        double hoodPositionDeg = Math.toDegrees(hoodPositionRad);

        if (hoodPositionDeg < HOOD_MIN_ANGLE) {
            hoodSim.setState(Math.toRadians(HOOD_MIN_ANGLE) * HOOD_GEAR_RATIO, 0.0);
        } else if (hoodPositionDeg > HOOD_MAX_ANGLE) {
            hoodSim.setState(Math.toRadians(HOOD_MAX_ANGLE) * HOOD_GEAR_RATIO, 0.0);
        }

        // Update flywheel velocities (convert rad/s to RPM)
        inputs.flywheelMidVelocityRPM = flywheelMidSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
        inputs.flywheelRightVelocityRPM = flywheelRightSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
        inputs.flywheel2InVelocityRPM = flywheel2InSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);

        // Indexer velocities
        inputs.shooterMotorVelocityRPM = shooterMotorSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
        inputs.pooperVelocityRPM = pooperSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);

        // Calculate average hood flywheel velocity
        inputs.velocityRPM = (inputs.flywheelMidVelocityRPM + inputs.flywheelRightVelocityRPM) / 2.0;

        // Update flywheel telemetry (use flywheelMid as representative)
        inputs.temperatureCelsius = 25.0; // Room temperature in sim

        // Update hood telemetry
        inputs.hoodPositionDegrees = Math.toDegrees(hoodSim.getAngularPositionRad() / HOOD_GEAR_RATIO);

        // Check if hood is at setpoint
        inputs.hoodAtSetpoint = Math.abs(inputs.hoodPositionDegrees - targetHoodDegrees) < 1.0;
    }

    @Override
    public void setFlywheelVelocity(double velocity) {
        flywheelClosedLoop = true;
        targetFlywheelRPM = velocity * 60.0; // Convert RPS to RPM for internal tracking
    }

    @Override
    public void setFlywheel2InVelocity(double velocity) {
        flywheel2InClosedLoop = true;
        targetFlywheel2InRPM = velocity * 60.0; // Convert RPS to RPM for internal tracking
    }

    @Override
    public void setIndexerVelocity(double velocity) {
        // Convert velocity (RPS) to voltage for simulation
        double volts = velocity * 60.0 * 0.0011; // Simple feedforward
        indexerAppliedVolts = volts;
        shooterMotorSim.setInputVoltage(volts);
    }

    @Override
    public void setPooperVelocity(double velocity) {
        // Convert velocity (RPS) to voltage for simulation
        double volts = velocity * 60.0 * 0.0011; // Simple feedforward
        pooperAppliedVolts = volts;
        pooperSim.setInputVoltage(volts);
    }

    @Override
    public void stopFlywheels() {
        flywheelClosedLoop = false;
        flywheel2InClosedLoop = false;
        flywheelAppliedVolts = 0.0;
        flywheel2InAppliedVolts = 0.0;
        flywheelMidSim.setInputVoltage(0.0);
        flywheelRightSim.setInputVoltage(0.0);
        flywheel2InSim.setInputVoltage(0.0);
    }

    @Override
    public void stopIndexer() {
        indexerAppliedVolts = 0.0;
        shooterMotorSim.setInputVoltage(0.0);
    }

    @Override
    public void stopPooper() {
        pooperAppliedVolts = 0.0;
        pooperSim.setInputVoltage(0.0);
    }

    @Override
    public void stop() {
        stopFlywheels();
        stopIndexer();
        stopPooper();
        stopHood();
    }

    @Override
    public void setBrakeMode(boolean enable) {
        // Brake mode doesn't affect simulation behavior
    }

    @Override
    public void setHoodPosition(double degrees) {
        hoodClosedLoop = true;
        targetHoodDegrees = Math.max(HOOD_MIN_ANGLE, Math.min(HOOD_MAX_ANGLE, degrees));
    }

    @Override
    public void stopHood() {
        hoodClosedLoop = false;
        hoodAppliedVolts = 0.0;
        hoodSim.setInputVoltage(0.0);
    }

    @Override
    public void resetHoodPosition() {
        hoodSim.setState(0.0, 0.0);
        targetHoodDegrees = 0.0;
    }

    // Helper method for flywheel velocity closed-loop control
    private void updateFlywheelControl() {
        double currentRPM =
                (flywheelMidSim.getAngularVelocityRadPerSec() + flywheelRightSim.getAngularVelocityRadPerSec())
                        * 60.0
                        / (4.0 * Math.PI);

        double error = targetFlywheelRPM - currentRPM;
        double ffVolts = targetFlywheelRPM * 0.0011;
        double voltage = ffVolts + (error * VELOCITY_KP);

        voltage = Math.max(-12.0, Math.min(12.0, voltage));

        flywheelAppliedVolts = voltage;
        flywheelMidSim.setInputVoltage(voltage);
        flywheelRightSim.setInputVoltage(voltage);
    }

    // Helper method for 2" flywheel velocity closed-loop control
    private void updateFlywheel2InControl() {
        double currentRPM = flywheel2InSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);

        double error = targetFlywheel2InRPM - currentRPM;
        double ffVolts = targetFlywheel2InRPM * 0.0011;
        double voltage = ffVolts + (error * VELOCITY_KP);

        voltage = Math.max(-12.0, Math.min(12.0, voltage));

        flywheel2InAppliedVolts = voltage;
        flywheel2InSim.setInputVoltage(voltage);
    }

    // Helper method for hood position closed-loop control
    private void updateHoodControl() {
        double currentDegrees = Math.toDegrees(hoodSim.getAngularPositionRad() / HOOD_GEAR_RATIO);

        double error = targetHoodDegrees - currentDegrees;
        double voltage = error * HOOD_KP;

        voltage += 0.5 * Math.cos(Math.toRadians(currentDegrees));

        voltage = Math.max(-12.0, Math.min(12.0, voltage));

        hoodAppliedVolts = voltage;
        hoodSim.setInputVoltage(voltage);
    }
}
