package frc.robot.util;

/**
 * A Double wrapper that implements Interpolable and InverseInterpolable for use as keys in InterpolatingTreeMap. Based
 * on Team 254's implementation.
 */
public class InterpolatingDouble
        implements Interpolable<InterpolatingDouble>,
                InverseInterpolable<InterpolatingDouble>,
                Comparable<InterpolatingDouble> {

    public final double value;

    public InterpolatingDouble(double value) {
        this.value = value;
    }

    @Override
    public InterpolatingDouble interpolate(InterpolatingDouble other, double t) {
        double interpolated = value + (other.value - value) * t;
        return new InterpolatingDouble(interpolated);
    }

    @Override
    public double inverseInterpolate(InterpolatingDouble upper, InterpolatingDouble query) {
        double upperValue = upper.value;
        double lowerValue = this.value;
        double queryValue = query.value;

        if (upperValue - lowerValue <= 0) {
            return 0.0;
        }
        return (queryValue - lowerValue) / (upperValue - lowerValue);
    }

    @Override
    public int compareTo(InterpolatingDouble other) {
        return Double.compare(value, other.value);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        InterpolatingDouble other = (InterpolatingDouble) obj;
        return Double.compare(value, other.value) == 0;
    }

    @Override
    public int hashCode() {
        return Double.hashCode(value);
    }

    @Override
    public String toString() {
        return String.valueOf(value);
    }
}
