package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOTalonFXSim implements ShooterIO {
    // Hardware configuration constants
    private static final double ROLLER_GEAR_RATIO = 1.0; // Direct drive
    private static final double HOOD_GEAR_RATIO = 50.0; // 50:1 reduction

    // Simulation parameters
    private static final double ROLLER_MOI = 0.004; // kg*m^2 (flywheel mass)
    private static final double HOOD_MOI = 0.025; // kg*m^2 (hood assembly mass)

    // Hood position constants (degrees)
    private static final double HOOD_MIN_ANGLE = 0.0;
    private static final double HOOD_MAX_ANGLE = 45.0;

    // Motor simulations for rollers (4 separate motors)
    private final DCMotorSim topLeftRollerSim;
    private final DCMotorSim topRightRollerSim;
    private final DCMotorSim bottomLeftRollerSim;
    private final DCMotorSim bottomRightRollerSim;

    // Motor simulation for hood (2 motors working together)
    private final DCMotorSim hoodSim;

    // Applied voltages
    private double rollerAppliedVolts = 0.0;
    private double hoodAppliedVolts = 0.0;

    // Control state
    private double targetVelocityRPM = 0.0;
    private double targetHoodDegrees = 0.0;
    private boolean velocityClosedLoop = false;
    private boolean hoodClosedLoop = false;

    // Simple PID gains for simulation
    private static final double VELOCITY_KP = 0.0002;
    private static final double HOOD_KP = 0.5;

    public ShooterIOTalonFXSim() {
        // Create roller simulations - Kraken X60 for each roller (direct drive)
        topLeftRollerSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), ROLLER_MOI, ROLLER_GEAR_RATIO),
                DCMotor.getKrakenX60(1));

        topRightRollerSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), ROLLER_MOI, ROLLER_GEAR_RATIO),
                DCMotor.getKrakenX60(1));

        bottomLeftRollerSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), ROLLER_MOI, ROLLER_GEAR_RATIO),
                DCMotor.getKrakenX60(1));

        bottomRightRollerSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), ROLLER_MOI, ROLLER_GEAR_RATIO),
                DCMotor.getKrakenX60(1));

        // Create hood simulation - 2 Kraken X60 motors with high gear reduction
        hoodSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(2), HOOD_MOI, HOOD_GEAR_RATIO),
                DCMotor.getKrakenX60(2));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Update closed-loop control if enabled
        if (velocityClosedLoop) {
            updateVelocityControl();
        }
        if (hoodClosedLoop) {
            updateHoodControl();
        }

        // Update simulations
        topLeftRollerSim.update(0.02);
        topRightRollerSim.update(0.02);
        bottomLeftRollerSim.update(0.02);
        bottomRightRollerSim.update(0.02);
        hoodSim.update(0.02);

        // Clamp hood position to realistic bounds
        double hoodPositionRad = hoodSim.getAngularPositionRad() / HOOD_GEAR_RATIO;
        double hoodPositionDeg = Math.toDegrees(hoodPositionRad);

        if (hoodPositionDeg < HOOD_MIN_ANGLE) {
            hoodSim.setState(Math.toRadians(HOOD_MIN_ANGLE) * HOOD_GEAR_RATIO, 0.0);
        } else if (hoodPositionDeg > HOOD_MAX_ANGLE) {
            hoodSim.setState(Math.toRadians(HOOD_MAX_ANGLE) * HOOD_GEAR_RATIO, 0.0);
        }

        // Update individual roller velocities (convert rad/s to RPM)
        inputs.topLeftVelocityRPM = topLeftRollerSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
        inputs.topRightVelocityRPM = topRightRollerSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
        inputs.bottomLeftVelocityRPM = bottomLeftRollerSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
        inputs.bottomRightVelocityRPM = bottomRightRollerSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);

        // Calculate average velocity
        inputs.velocityRPM = (inputs.topLeftVelocityRPM
                        + inputs.topRightVelocityRPM
                        + inputs.bottomLeftVelocityRPM
                        + inputs.bottomRightVelocityRPM)
                / 4.0;

        // Update roller telemetry (use top left as representative)
        inputs.appliedVolts = rollerAppliedVolts;
        inputs.currentAmps = Math.abs(topLeftRollerSim.getCurrentDrawAmps())
                + Math.abs(topRightRollerSim.getCurrentDrawAmps())
                + Math.abs(bottomLeftRollerSim.getCurrentDrawAmps())
                + Math.abs(bottomRightRollerSim.getCurrentDrawAmps());
        inputs.temperatureCelsius = 25.0; // Room temperature in sim

        // Update hood telemetry
        inputs.hoodPositionDegrees = Math.toDegrees(hoodSim.getAngularPositionRad() / HOOD_GEAR_RATIO);
        inputs.hoodVelocityDegreesPerSec = Math.toDegrees(hoodSim.getAngularVelocityRadPerSec() / HOOD_GEAR_RATIO);
        inputs.hoodAppliedVolts = hoodAppliedVolts;
        inputs.hoodCurrentAmps = Math.abs(hoodSim.getCurrentDrawAmps());

        // Check if hood is at setpoint
        inputs.hoodAtSetpoint = Math.abs(inputs.hoodPositionDegrees - targetHoodDegrees) < 1.0;
    }

    @Override
    public void setVoltage(double volts) {
        velocityClosedLoop = false;
        rollerAppliedVolts = volts;
        applyRollerVoltage(volts);
    }

    @Override
    public void setVelocity(double velocityRPM, double ffVolts) {
        velocityClosedLoop = true;
        targetVelocityRPM = velocityRPM;
        // Feedforward voltage will be applied in updateVelocityControl
    }

    @Override
    public void stop() {
        setVoltage(0.0);
        stopHood();
        velocityClosedLoop = false;
        hoodClosedLoop = false;
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

    // Helper method to apply voltage to all roller motors
    private void applyRollerVoltage(double volts) {
        topLeftRollerSim.setInputVoltage(volts);
        topRightRollerSim.setInputVoltage(volts);
        bottomLeftRollerSim.setInputVoltage(volts);
        bottomRightRollerSim.setInputVoltage(volts);
    }

    // Helper method for velocity closed-loop control
    private void updateVelocityControl() {
        // Get current average velocity
        double currentRPM = (topLeftRollerSim.getAngularVelocityRadPerSec()
                        + topRightRollerSim.getAngularVelocityRadPerSec()
                        + bottomLeftRollerSim.getAngularVelocityRadPerSec()
                        + bottomRightRollerSim.getAngularVelocityRadPerSec())
                * 60.0
                / (8.0 * Math.PI);

        // Simple P controller with feedforward
        double error = targetVelocityRPM - currentRPM;
        double ffVolts = targetVelocityRPM * 0.0011; // Simple feedforward gain
        double voltage = ffVolts + (error * VELOCITY_KP);

        // Clamp voltage
        voltage = Math.max(-12.0, Math.min(12.0, voltage));

        rollerAppliedVolts = voltage;
        applyRollerVoltage(voltage);
    }

    // Helper method for hood position closed-loop control
    private void updateHoodControl() {
        // Get current position in degrees
        double currentDegrees = Math.toDegrees(hoodSim.getAngularPositionRad() / HOOD_GEAR_RATIO);

        // Simple P controller
        double error = targetHoodDegrees - currentDegrees;
        double voltage = error * HOOD_KP;

        // Add some gravity compensation (hood fights gravity)
        voltage += 0.5 * Math.cos(Math.toRadians(currentDegrees));

        // Clamp voltage
        voltage = Math.max(-12.0, Math.min(12.0, voltage));

        hoodAppliedVolts = voltage;
        hoodSim.setInputVoltage(voltage);
    }
}
