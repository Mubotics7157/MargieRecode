package frc.robot.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

public interface LedIO {
    @AutoLog
    class LedIOInputs {
        public int currentRed = 0;
        public int currentGreen = 0;
        public int currentBlue = 0;
        public String currentPattern = "Off";
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(LedIOInputs inputs) {}

    /** Sets all LEDs to a solid color. */
    default void setSolidColor(LedState state) {}

    /** Sets LEDs to a pattern of colors. */
    default void setPattern(LedState[] states) {}

    /** Gets the current LED state. */
    default LedState getCurrentState() {
        return LedState.kOff;
    }
}
