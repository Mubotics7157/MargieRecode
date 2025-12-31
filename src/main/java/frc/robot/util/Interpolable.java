package frc.robot.util;

/**
 * Interface for values that can be interpolated. Used by InterpolatingTreeMap. Based on Team 254's implementation.
 *
 * @param <T> The type that can be interpolated
 */
public interface Interpolable<T> {
    /**
     * Interpolates between this value and another.
     *
     * @param other The other value to interpolate towards
     * @param t The interpolation parameter [0, 1]. 0 returns this, 1 returns other.
     * @return The interpolated value
     */
    T interpolate(T other, double t);
}
