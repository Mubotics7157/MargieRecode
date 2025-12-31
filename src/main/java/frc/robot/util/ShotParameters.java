package frc.robot.util;

/**
 * Represents the parameters for a shooter shot at a specific distance. Implements Interpolable to allow smooth
 * transitions between preset shot configurations. Following Team 254's convention for shot parameter management.
 *
 * <p>Note: The 2" flywheel runs at a constant RPM regardless of shot distance, so it is not included here. See
 * {@link frc.robot.subsystems.shooter.ShooterConstants#FLYWHEEL_2IN_RPM}.
 */
public class ShotParameters implements Interpolable<ShotParameters> {

    private final double flywheelRPM;
    private final double hoodAngleDegrees;

    /**
     * Creates a new ShotParameters instance.
     *
     * @param flywheelRPM The target flywheel RPM for the main flywheels
     * @param hoodAngleDegrees The hood exit shot angle in degrees (54-74 range)
     */
    public ShotParameters(double flywheelRPM, double hoodAngleDegrees) {
        this.flywheelRPM = flywheelRPM;
        this.hoodAngleDegrees = hoodAngleDegrees;
    }

    public double getFlywheelRPM() {
        return flywheelRPM;
    }

    /**
     * Gets the hood exit shot angle in degrees.
     *
     * @return The exit angle (54-74 degrees range)
     */
    public double getHoodAngleDegrees() {
        return hoodAngleDegrees;
    }

    /** @deprecated Use {@link #getHoodAngleDegrees()} instead. This method exists for backwards compatibility. */
    @Deprecated
    public double getHoodPosition() {
        return hoodAngleDegrees;
    }

    @Override
    public ShotParameters interpolate(ShotParameters other, double t) {
        return new ShotParameters(
                lerp(flywheelRPM, other.flywheelRPM, t), lerp(hoodAngleDegrees, other.hoodAngleDegrees, t));
    }

    /**
     * Linear interpolation helper.
     *
     * @param a Start value
     * @param b End value
     * @param t Interpolation factor [0, 1]
     * @return Interpolated value
     */
    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    @Override
    public String toString() {
        return String.format("ShotParameters[flywheel=%.0f RPM, hoodAngle=%.2f°]", flywheelRPM, hoodAngleDegrees);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        ShotParameters other = (ShotParameters) obj;
        return Double.compare(flywheelRPM, other.flywheelRPM) == 0
                && Double.compare(hoodAngleDegrees, other.hoodAngleDegrees) == 0;
    }

    @Override
    public int hashCode() {
        int result = Double.hashCode(flywheelRPM);
        result = 31 * result + Double.hashCode(hoodAngleDegrees);
        return result;
    }
}
