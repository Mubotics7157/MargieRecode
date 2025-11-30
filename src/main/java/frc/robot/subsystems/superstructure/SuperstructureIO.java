package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLog;

public interface SuperstructureIO {
    @AutoLog
    class SuperstructureIOInputs {
        public Superstructure.Goal currentGoal = Superstructure.Goal.IDLE;
        public Superstructure.Goal desiredGoal = Superstructure.Goal.IDLE;
        public boolean atGoal = true;
    }

    default void updateInputs(SuperstructureIOInputs inputs) {}
}
