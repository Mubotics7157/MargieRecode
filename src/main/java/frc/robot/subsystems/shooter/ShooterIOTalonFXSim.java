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

    // Hood conversion: native units (0-12) correspond to exit shot angles (54-74 degrees)
    private static final double MIN_NATIVE_UNITS = 0.0;
    private static final double MAX_NATIVE_UNITS = 12.0;
    private static final double MIN_EXIT_ANGLE_DEGREES = 54.0;
    private static final double MAX_EXIT_ANGLE_DEGREES = 74.0;

    // Soft limits in native units
    private static final double SOFT_LIMIT_REVERSE_NATIVE = 1.0;
    private static final double SOFT_LIMIT_FORWARD_NATIVE = 12.0;

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
    private double targetHoodDegrees = MIN_EXIT_ANGLE_DEGREES; // Exit angle in degrees
    private boolean flywheelClosedLoop = false;
    private boolean flywheel2InClosedLoop = false;
    private boolean hoodClosedLoop = false;

    // Simple PID gains for simulation
    private static final double VELOCITY_KP = 0.0002;
    private static final double HOOD_KP = 0.5;

    /** Converts exit shot angle (degrees) to native motor units. */
    private static double degreesToNative(double degrees) {
        return (degrees - MIN_EXIT_ANGLE_DEGREES)
                        / (MAX_EXIT_ANGLE_DEGREES - MIN_EXIT_ANGLE_DEGREES)
                        * (MAX_NATIVE_UNITS - MIN_NATIVE_UNITS)
                + MIN_NATIVE_UNITS;
    }

    /** Converts native motor units to exit shot angle (degrees). */
    private static double nativeToDegrees(double nativeUnits) {
        return (nativeUnits - MIN_NATIVE_UNITS)
                        / (MAX_NATIVE_UNITS - MIN_NATIVE_UNITS)
                        * (MAX_EXIT_ANGLE_DEGREES - MIN_EXIT_ANGLE_DEGREES)
                + MIN_EXIT_ANGLE_DEGREES;
    }

    /** Returns the minimum exit angle allowed by soft limits. */
    private static double getMinExitAngleDegrees() {
        return nativeToDegrees(SOFT_LIMIT_REVERSE_NATIVE);
    }

    /** Returns the maximum exit angle allowed by soft limits. */
    private static double getMaxExitAngleDegrees() {
        return nativeToDegrees(SOFT_LIMIT_FORWARD_NATIVE);
    }

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

        // Clamp hood position to soft limit bounds (in native units)
        // Simulation uses radians internally: 2*PI radians = 1 native unit (rotation)
        double nativePosition = hoodSim.getAngularPositionRad() / (2.0 * Math.PI);

        if (nativePosition < SOFT_LIMIT_REVERSE_NATIVE) {
            hoodSim.setState(SOFT_LIMIT_REVERSE_NATIVE * 2.0 * Math.PI, 0.0);
            nativePosition = SOFT_LIMIT_REVERSE_NATIVE;
        } else if (nativePosition > SOFT_LIMIT_FORWARD_NATIVE) {
            hoodSim.setState(SOFT_LIMIT_FORWARD_NATIVE * 2.0 * Math.PI, 0.0);
            nativePosition = SOFT_LIMIT_FORWARD_NATIVE;
        }

        // Update flywheel velocities (convert rad/s to RPM)
        inputs.flywheelMidVelocityRPM = flywheelMidSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
        inputs.flywheelRightVelocityRPM = flywheelRightSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
        inputs.flywheel2InVelocityRPM = flywheel2InSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);

        // Indexer velocities
        inputs.shooterMotorVelocityRPM = shooterMotorSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
        inputs.pooperVelocityRPM = pooperSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);

        // Simulated pooper current (approximate based on motor current draw)
        inputs.pooperCurrentAmps = pooperSim.getCurrentDrawAmps();

        // Calculate average hood flywheel velocity
        inputs.velocityRPM = (inputs.flywheelMidVelocityRPM + inputs.flywheelRightVelocityRPM) / 2.0;

        // Update flywheel telemetry (use flywheelMid as representative)
        inputs.temperatureCelsius = 25.0; // Room temperature in sim

        // Update hood telemetry - convert native units to exit angle degrees
        inputs.hoodPositionDegrees = nativeToDegrees(nativePosition);

        // Check if hood is at setpoint (both in exit angle degrees)
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
        // Clamp to valid exit angle range (soft limits will also enforce this)
        targetHoodDegrees = Math.max(getMinExitAngleDegrees(), Math.min(getMaxExitAngleDegrees(), degrees));
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
        targetHoodDegrees = MIN_EXIT_ANGLE_DEGREES;
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
        // Get current position in native units, then convert to exit angle degrees
        double nativePosition = hoodSim.getAngularPositionRad() / (2.0 * Math.PI);
        double currentDegrees = nativeToDegrees(nativePosition);

        // Error in exit angle degrees
        double error = targetHoodDegrees - currentDegrees;
        double voltage = error * HOOD_KP;

        // Gravity compensation (approximate)
        voltage += 0.5 * Math.cos(Math.toRadians(currentDegrees - MIN_EXIT_ANGLE_DEGREES));

        voltage = Math.max(-12.0, Math.min(12.0, voltage));

        hoodAppliedVolts = voltage;
        hoodSim.setInputVoltage(voltage);
    }
}
