package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        // Hood flywheel motors (averaged values for main shooting)
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double temperatureCelsius = 0.0;

        // Individual flywheel velocities for debugging
        public double flywheelMidVelocityRPM = 0.0;
        public double flywheelRightVelocityRPM = 0.0;
        public double flywheel2InVelocityRPM = 0.0;

        // Indexer motor velocities
        public double shooterMotorVelocityRPM = 0.0; // Starwheels
        public double pooperVelocityRPM = 0.0;

        // Hood pivot
        public double hoodPositionDegrees = 0.0;
        public double hoodVelocityDegreesPerSec = 0.0;
        public double hoodAppliedVolts = 0.0;
        public double hoodCurrentAmps = 0.0;
        public boolean hoodAtSetpoint = false;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    // Hood flywheel control (FlywheelMid + FlywheelRight)
    public default void setFlywheelVoltage(double volts) {}

    public default void setFlywheelVelocity(double velocityRPM, double ffVolts) {}

    // 2" flywheel control (feeds into hood flywheels)
    public default void setFlywheel2InVoltage(double volts) {}

    public default void setFlywheel2InVelocity(double velocityRPM, double ffVolts) {}

    // Indexer control (ShooterMotor - starwheels)
    public default void setIndexerVoltage(double volts) {}

    // Pooper control (positive = eject, negative = continue through path)
    public default void setPooperVoltage(double volts) {}

    // Stop methods
    public default void stopFlywheels() {}

    public default void stopIndexer() {}

    public default void stopPooper() {}

    public default void stop() {}

    public default void setBrakeMode(boolean enable) {}

    // Hood pivot control
    public default void setHoodPosition(double degrees) {}

    public default void stopHood() {}

    public default void resetHoodPosition() {}
}
