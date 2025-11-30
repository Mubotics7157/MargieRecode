package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        // Roller Motor
        public double rollerCurrent = 0.0;
        public double rollerVoltage = 0.0;
        public double rollerPosition = 0.0;
        public double rollerVelocity = 0.0;

        // Arm Motor
        public double armCurrent = 0.0;
        public double armVoltage = 0.0;
        public double armPosition = 0.0;
        public double armVelocity = 0.0;

        // Indexer Motor
        public double indexerCurrent = 0.0;
        public double indexerVoltage = 0.0;
        public double indexerPosition = 0.0;
        public double indexerVelocity = 0.0;
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setRollerVoltage(double voltage) {}

    default void setArmVoltage(double voltage) {}

    default void setArmPosition(double positionRad) {}

    default void resetArmPosition() {}

    default void stopRoller() {}

    default void stopArmMotor() {}
}
