package frc.robot.subsystems.led;

/**
 * Constants for the LED subsystem. Contains CANdle configuration, LED counts, animation parameters, and voltage
 * thresholds.
 */
public final class LedConstants {

    private LedConstants() {}

    // ==================== CANdle Configuration ====================
    public static final int CANDLE_ID = 31;
    public static final String CAN_BUS = "swerve";

    // ==================== LED Counts ====================
    public static final int CANDLE_ONBOARD_LED_COUNT = 8;
    public static final int STRIP_LED_COUNT = 15; // LEDs per external strip
    public static final int NUM_STRIPS = 2;
    public static final int TOTAL_LED_COUNT = CANDLE_ONBOARD_LED_COUNT + (STRIP_LED_COUNT * NUM_STRIPS);

    // ==================== Animation Parameters ====================
    public static final double BREATH_PERIOD = 2.0; // seconds for one full breath cycle
    public static final double RAINBOW_PERIOD = 5.0; // seconds to cycle through all rainbow colors
    public static final double BREATH_MIN_BRIGHTNESS = 0.15; // minimum brightness during breathing
    public static final double BREATH_BRIGHTNESS_RANGE = 0.85; // brightness variation range

    // ==================== Traveling Rainbow Parameters ====================
    public static final double TRAVEL_PERIOD = 1.5; // seconds for one full up-down cycle
    public static final double RAINBOW_BAND_WIDTH = 5.0; // number of LEDs wide for the rainbow band
}
