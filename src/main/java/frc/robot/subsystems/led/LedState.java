package frc.robot.subsystems.led;

/** Represents an RGB color state for LEDs. */
public class LedState {
    // Basic colors
    public static final LedState kOff = new LedState(0, 0, 0);
    public static final LedState kRed = new LedState(255, 0, 0);
    public static final LedState kGreen = new LedState(0, 255, 0);
    public static final LedState kBlue = new LedState(0, 0, 255);
    public static final LedState kWhite = new LedState(255, 255, 255);
    public static final LedState kYellow = new LedState(255, 215, 0);
    public static final LedState kOrange = new LedState(255, 80, 0);
    public static final LedState kPurple = new LedState(255, 0, 255);
    public static final LedState kCyan = new LedState(0, 255, 255);
    public static final LedState kPink = new LedState(255, 0, 100);

    // Robot state colors
    public static final LedState kDisabled = kOrange;
    public static final LedState kEnabled = kGreen;
    public static final LedState kAutonomous = kBlue;
    public static final LedState kIntaking = kYellow;
    public static final LedState kHasPiece = kGreen;
    public static final LedState kShooting = kRed;
    public static final LedState kAiming = kPurple;
    public static final LedState kLowBattery = kRed;

    // Rainbow pattern for idle/celebration
    public static final LedState[] kRainbow = {kRed, kOrange, kYellow, kGreen, kCyan, kBlue, kPurple};

    public int red;
    public int green;
    public int blue;

    public LedState() {
        this.red = 0;
        this.green = 0;
        this.blue = 0;
    }

    public LedState(int red, int green, int blue) {
        this.red = red;
        this.green = green;
        this.blue = blue;
    }

    public void copyFrom(LedState other) {
        this.red = other.red;
        this.green = other.green;
        this.blue = other.blue;
    }

    @Override
    public boolean equals(Object other) {
        if (other == null) {
            return false;
        }
        if (other.getClass() != this.getClass()) {
            return false;
        }
        LedState s = (LedState) other;
        return this.red == s.red && this.green == s.green && this.blue == s.blue;
    }

    @Override
    public int hashCode() {
        return red * 65536 + green * 256 + blue;
    }

    @Override
    public String toString() {
        return String.format("#%02x%02x%02x", red, green, blue);
    }
}
