package frc.robot.subsystems.led;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

public class LedIOCANdle implements LedIO {
    private final CANdle candle;
    private final SolidColor solidColorRequest;
    private LedState currentState = LedState.kOff;
    private final int ledCount;

    /**
     * Creates a new LedIOCANdle.
     *
     * @param candleId CAN ID of the CANdle
     * @param canBus CAN bus name (empty string for default)
     * @param ledCount Total number of LEDs (including CANdle's 8 onboard LEDs)
     */
    public LedIOCANdle(int candleId, String canBus, int ledCount) {
        this.ledCount = ledCount;
        candle = new CANdle(candleId, canBus);

        // Configure the CANdle
        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.StripType = StripTypeValue.RGB;
        config.LED.BrightnessScalar = 1.0;
        candle.getConfigurator().apply(config);

        // Create control request for solid color
        solidColorRequest = new SolidColor(0, ledCount - 1);
    }

    @Override
    public void updateInputs(LedIOInputs inputs) {
        inputs.currentRed = currentState.red;
        inputs.currentGreen = currentState.green;
        inputs.currentBlue = currentState.blue;
        inputs.currentPattern = currentState.toString();
    }

    @Override
    public void setSolidColor(LedState state) {
        if (state == null) {
            state = LedState.kOff;
        }
        currentState = state;

        RGBWColor color = new RGBWColor(state.red, state.green, state.blue, 0);
        candle.setControl(solidColorRequest.withColor(color));
    }

    @Override
    public void setPattern(LedState[] states) {
        if (states == null || states.length == 0) {
            return;
        }

        // Phoenix 6 CANdle doesn't have direct pixel-by-pixel control like Phoenix 5
        // For patterns, we'll use multiple SolidColor requests for different segments
        // For now, use run-length encoding to set consecutive LEDs of same color

        LedState run = states[0];
        if (run == null) run = LedState.kOff;
        int runStart = 0;

        for (int i = 0; i < Math.min(states.length, ledCount); i++) {
            LedState current = states[i];
            if (current == null) current = LedState.kOff;

            if (!run.equals(current)) {
                // Set this segment
                RGBWColor color = new RGBWColor(run.red, run.green, run.blue, 0);
                SolidColor segmentRequest = new SolidColor(runStart, i - 1);
                candle.setControl(segmentRequest.withColor(color));
                runStart = i;
                run = current;
            }
        }

        // Write the final run
        int endIndex = Math.min(states.length, ledCount) - 1;
        RGBWColor color = new RGBWColor(run.red, run.green, run.blue, 0);
        SolidColor segmentRequest = new SolidColor(runStart, endIndex);
        candle.setControl(segmentRequest.withColor(color));

        // Update current state to first color in pattern
        currentState = states[0];
    }

    @Override
    public LedState getCurrentState() {
        return currentState;
    }
}
