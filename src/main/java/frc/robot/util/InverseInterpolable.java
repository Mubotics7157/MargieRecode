package frc.robot.util;

/**
 * Interface for values that can compute inverse interpolation (find t given bounds). Used by InterpolatingTreeMap for
 * key types. Based on Team 254's implementation.
 *
 * @param <T> The type that can be inverse-interpolated
 */
public interface InverseInterpolable<T> {
    /**
     * Given upper and lower bounds and a query value, find the interpolation parameter t such that interpolate(lower,
     * upper, t) would return the query value.
     *
     * @param upper The upper bound
     * @param query The query value
     * @return The interpolation parameter t in [0, 1]
     */
    double inverseInterpolate(T upper, T query);
}
