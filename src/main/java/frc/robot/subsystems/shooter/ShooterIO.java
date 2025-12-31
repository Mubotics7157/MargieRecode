package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        // Hood flywheel motors (averaged values for main shooting)
        public double velocityRPM = 0.0;
        public double temperatureCelsius = 0.0;

        // Pooper current for ball detection
        public double pooperCurrentAmps = 0.0;

        // Individual flywheel velocities for debugging
        public double flywheelMidVelocityRPM = 0.0;
        public double flywheelRightVelocityRPM = 0.0;
        public double flywheel2InVelocityRPM = 0.0;

        // Indexer motor velocities
        public double shooterMotorVelocityRPM = 0.0; // Starwheels
        public double pooperVelocityRPM = 0.0;

        // Indexer current for ball detection
        public double indexerCurrentAmps = 0.0;

        // Hood pivot
        public double hoodPositionDegrees = 0.0;
        public boolean hoodAtSetpoint = false;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setFlywheelVelocity(double velocity) {}

    public default void setFlywheel2InVelocity(double velocity) {}

    // Indexer control (ShooterMotor - starwheels)
    public default void setIndexerVelocity(double velocity) {}

    // Pooper control (positive = eject, negative = continue through path)
    public default void setPooperVelocity(double velocity) {}

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
