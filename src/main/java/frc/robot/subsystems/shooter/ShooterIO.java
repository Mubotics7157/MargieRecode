package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double temperatureCelsius = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void setVelocity(double velocityRPM, double ffVolts) {}

    public default void stop() {}

    public default void setBrakeMode(boolean enable) {}
}