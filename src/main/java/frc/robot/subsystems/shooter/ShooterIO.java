package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        // Roller motors (averaged values)
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double temperatureCelsius = 0.0;

        // Individual roller motor velocities for debugging
        public double topLeftVelocityRPM = 0.0;
        public double topRightVelocityRPM = 0.0;
        public double bottomLeftVelocityRPM = 0.0;
        public double bottomRightVelocityRPM = 0.0;

        // Hood motors
        public double hoodPositionDegrees = 0.0;
        public double hoodVelocityDegreesPerSec = 0.0;
        public double hoodAppliedVolts = 0.0;
        public double hoodCurrentAmps = 0.0;
        public boolean hoodAtSetpoint = false;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void setVelocity(double velocityRPM, double ffVolts) {}

    public default void stop() {}

    public default void setBrakeMode(boolean enable) {}

    public default void setHoodPosition(double degrees) {}

    public default void stopHood() {}

    public default void resetHoodPosition() {}
}
