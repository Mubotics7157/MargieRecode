package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.Superstructure;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Led extends SubsystemBase {
    private final LedIO io;
    private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

    // Optional reference to superstructure for automatic state-based colors
    private Superstructure superstructure = null;

    // Animation state
    private double breathPhase = 0.0;

    public Led(LedIO io) {
        this.io = io;
    }

    /**
     * Sets the superstructure reference for automatic state-based LED colors.
     *
     * @param superstructure The superstructure subsystem
     */
    public void setSuperstructure(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Led", inputs);

        // Log current state
        Logger.recordOutput("Led/CurrentColor", io.getCurrentState().toString());

        // If no command is running, use automatic state-based colors
        if (getCurrentCommand() == null) {
            updateAutomaticState();
        }

        Logger.recordOutput(
                "Led/CurrentCommand",
                getCurrentCommand() == null ? "Automatic" : getCurrentCommand().getName());
    }

    private void updateAutomaticState() {
        // Priority-based state determination
        if (DriverStation.isDisabled()) {
            // Show breathing rainbow in disabled mode
            updateBreathingRainbow();
            return;
        }

        // Check superstructure state
        if (superstructure != null) {
            switch (superstructure.getCurrentGoal()) {
                case INTAKING:
                    io.setSolidColor(LedState.kIntaking);
                    return;
                case SHOOTING:
                    io.setSolidColor(LedState.kShooting);
                    return;
                case OUTTAKING:
                    io.setSolidColor(LedState.kOrange);
                    return;
                case POOPING:
                    io.setSolidColor(LedState.kPooping);
                    return;
                case IDLE:
                default:
                    break;
            }

            // Show green if we have a ball
            if (superstructure.hasBall()) {
                io.setSolidColor(LedState.kHasPiece);
                return;
            }
        }

        // Default state - traveling rainbow when enabled
        if (DriverStation.isAutonomous() || DriverStation.isTeleop()) {
            updateTravelingRainbow();
        } else {
            io.setSolidColor(LedState.kOff);
        }
    }

    private void updateBreathingRainbow() {
        double now = Timer.getFPGATimestamp();

        // Calculate breathing brightness using sine wave (0.0 to 1.0)
        double breathValue = (Math.sin(now * 2 * Math.PI / LedConstants.BREATH_PERIOD) + 1.0) / 2.0;
        // Add minimum brightness so LEDs don't fully turn off
        double brightness = LedConstants.BREATH_MIN_BRIGHTNESS + (breathValue * LedConstants.BREATH_BRIGHTNESS_RANGE);

        // Calculate rainbow hue position (0.0 to 1.0)
        double huePosition = (now % LedConstants.RAINBOW_PERIOD) / LedConstants.RAINBOW_PERIOD;

        // Convert HSV to RGB (saturation = 1.0, value = brightness)
        LedState color = hsvToRgb(huePosition, 1.0, brightness);
        io.setSolidColor(color);
    }

    private void updateTravelingRainbow() {
        double now = Timer.getFPGATimestamp();
        int stripLength = LedConstants.STRIP_LED_COUNT;

        // Calculate position along strip using triangle wave for bounce effect
        double cyclePosition = (now % LedConstants.TRAVEL_PERIOD) / LedConstants.TRAVEL_PERIOD;
        // Triangle wave: 0->1->0 for bounce effect
        double normalizedPos = cyclePosition < 0.5 ? cyclePosition * 2.0 : 2.0 - (cyclePosition * 2.0);

        // Create pattern array for strip
        LedState[] pattern = new LedState[LedConstants.TOTAL_LED_COUNT];

        // Calculate the center position of the rainbow band
        double centerPos = normalizedPos * (stripLength - 1);

        for (int i = 0; i < LedConstants.TOTAL_LED_COUNT; i++) {
            // Get position relative to start of external strip (skip onboard LEDs)
            int stripPos = i - LedConstants.CANDLE_ONBOARD_LED_COUNT;

            if (stripPos < 0) {
                // Onboard LEDs - mirror first strip LED
                stripPos = 0;
            } else if (stripPos >= stripLength) {
                // Second strip - mirror position
                stripPos = stripPos % stripLength;
            }

            // Calculate distance from center of rainbow band
            double distance = Math.abs(stripPos - centerPos);

            if (distance < LedConstants.RAINBOW_BAND_WIDTH) {
                // Inside rainbow band - calculate hue based on position within band
                double hue = (stripPos - centerPos + LedConstants.RAINBOW_BAND_WIDTH)
                        / (LedConstants.RAINBOW_BAND_WIDTH * 2.0);
                hue = Math.max(0.0, Math.min(1.0, hue));
                pattern[i] = hsvToRgb(hue, 1.0, 1.0);
            } else {
                // Outside rainbow band - off
                pattern[i] = LedState.kOff;
            }
        }

        io.setPattern(pattern);
    }

    /** Converts HSV color to RGB LedState. H is 0-1, S is 0-1, V is 0-1. */
    private LedState hsvToRgb(double h, double s, double v) {
        double r, g, b;

        int i = (int) (h * 6);
        double f = h * 6 - i;
        double p = v * (1 - s);
        double q = v * (1 - f * s);
        double t = v * (1 - (1 - f) * s);

        switch (i % 6) {
            case 0 -> {
                r = v;
                g = t;
                b = p;
            }
            case 1 -> {
                r = q;
                g = v;
                b = p;
            }
            case 2 -> {
                r = p;
                g = v;
                b = t;
            }
            case 3 -> {
                r = p;
                g = q;
                b = v;
            }
            case 4 -> {
                r = t;
                g = p;
                b = v;
            }
            default -> {
                r = v;
                g = p;
                b = q;
            }
        }

        return new LedState((int) (r * 255), (int) (g * 255), (int) (b * 255));
    }

    /** Creates a command to set a solid color. */
    public Command commandSolidColor(LedState state) {
        return run(() -> io.setSolidColor(state)).ignoringDisable(true).withName("LED Solid: " + state.toString());
    }

    /** Creates a command to set a solid color from a supplier. */
    public Command commandSolidColor(Supplier<LedState> stateSupplier) {
        return run(() -> io.setSolidColor(stateSupplier.get()))
                .ignoringDisable(true)
                .withName("LED Solid Dynamic");
    }

    /** Creates a command to display a pattern. */
    public Command commandPattern(LedState[] pattern) {
        return run(() -> io.setPattern(pattern)).ignoringDisable(true).withName("LED Pattern");
    }

    /** Creates a blinking command alternating between two states. */
    public Command commandBlink(LedState stateOne, LedState stateTwo, double period) {
        return commandBlink(stateOne, stateTwo, period / 2.0, period / 2.0);
    }

    /** Creates a blinking command with separate durations for each state. */
    public Command commandBlink(LedState stateOne, LedState stateTwo, double durationOne, double durationTwo) {
        var state = new Object() {
            boolean showFirst = true;
            double timestamp = Timer.getFPGATimestamp();
        };

        return Commands.runOnce(() -> {
                    state.showFirst = true;
                    state.timestamp = Timer.getFPGATimestamp();
                })
                .andThen(run(() -> {
                    double now = Timer.getFPGATimestamp();
                    if (state.showFirst && now - state.timestamp >= durationOne) {
                        state.showFirst = false;
                        state.timestamp = now;
                    } else if (!state.showFirst && now - state.timestamp >= durationTwo) {
                        state.showFirst = true;
                        state.timestamp = now;
                    }
                    io.setSolidColor(state.showFirst ? stateOne : stateTwo);
                }))
                .ignoringDisable(true)
                .withName("LED Blink");
    }

    /** Gets the current LED state. */
    public LedState getCurrentState() {
        return io.getCurrentState();
    }
}
