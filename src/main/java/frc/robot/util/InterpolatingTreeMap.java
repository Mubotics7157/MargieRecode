package frc.robot.util;

import java.util.TreeMap;

/**
 * A TreeMap that performs linear interpolation between stored values for keys that don't exist. Based on Team 254's
 * implementation.
 *
 * @param <K> Key type that must be Comparable and InverseInterpolable
 * @param <V> Value type that must be Interpolable
 */
public class InterpolatingTreeMap<K extends InverseInterpolable<K> & Comparable<K>, V extends Interpolable<V>>
        extends TreeMap<K, V> {

    private final int maxSize;

    /** Creates an InterpolatingTreeMap with unlimited size. */
    public InterpolatingTreeMap() {
        this.maxSize = 0;
    }

    /**
     * Creates an InterpolatingTreeMap with a maximum size. When the size is exceeded, the oldest (smallest key) entry
     * is removed.
     *
     * @param maxSize Maximum number of entries (0 for unlimited)
     */
    public InterpolatingTreeMap(int maxSize) {
        this.maxSize = maxSize;
    }

    @Override
    public V put(K key, V value) {
        if (maxSize > 0 && size() >= maxSize) {
            // Remove the smallest key when at capacity
            remove(firstKey());
        }
        return super.put(key, value);
    }

    /**
     * Gets an interpolated value for the given key. If the key exists, returns its value directly. Otherwise,
     * interpolates between the two nearest keys.
     *
     * @param key The key to look up
     * @return The interpolated value, or null if the map is empty
     */
    public V getInterpolated(K key) {
        // Exact match
        V exactValue = get(key);
        if (exactValue != null) {
            return exactValue;
        }

        // Get surrounding keys
        K floorKey = floorKey(key);
        K ceilingKey = ceilingKey(key);

        // Handle edge cases
        if (floorKey == null && ceilingKey == null) {
            return null;
        }
        if (floorKey == null) {
            return get(ceilingKey);
        }
        if (ceilingKey == null) {
            return get(floorKey);
        }

        // Interpolate between floor and ceiling
        V floorValue = get(floorKey);
        V ceilingValue = get(ceilingKey);

        // Calculate interpolation parameter
        double t = floorKey.inverseInterpolate(ceilingKey, key);

        return floorValue.interpolate(ceilingValue, t);
    }
}
