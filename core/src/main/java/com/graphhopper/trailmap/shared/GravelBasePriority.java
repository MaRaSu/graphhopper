/*
 * Trailmap - GravelBasePriority encoded value
 *
 * Pre-computed priority for gravel routing profile.
 * Represents how suitable an edge is for gravel cycling (0.0 to 1.0).
 *
 * Initial curve (user will tune):
 * - ZERO_MINUS (paved): 0.6 - acceptable but not preferred
 * - ZERO: 0.8 - good
 * - ZERO_PLUS: 0.95 - very good
 * - ONE: 1.0 - optimal
 * - TWO: 0.9 - still good
 * - THREE: 0.7 - getting rough
 * - FOUR: 0.3 - avoid
 * - UNKNOWN: 0.7 - conservative
 * - FERRY: 0.5 - special handling
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.routing.ev.DecimalEncodedValue;
import com.graphhopper.routing.ev.DecimalEncodedValueImpl;

public class GravelBasePriority {

    public static final String KEY = "gravel_base_priority";

    public static DecimalEncodedValue create() {
        // Range 0-1, factor 0.05 for precision (20 steps)
        return new DecimalEncodedValueImpl(KEY, 5, 0, 0.05, false, false, false);
    }

    /**
     * Get base priority for gravel profile.
     * Initial values - will be tuned based on testing.
     */
    public static double getPriority(GravelScale scale) {
        if (scale == null) {
            return 0.7;
        }
        switch (scale) {
            case ZERO_MINUS:
                return 0.95; // Paved - acceptable
            case ZERO:
                return 1.0; // Very smooth unpaved
            case ZERO_PLUS:
                return 1.0; // Good gravel
            case ONE:
                return 0.95; // Slightly riskier than ZERO_PLUS
            case TWO:
                return 0.8; // Rougher but usually rideable
            case THREE:
                return 0.5; // Getting really rough
            case FOUR:
                return 0.3; // Avoid
            case UNKNOWN:
                return 0.7; // Conservative
            case FERRY:
                return 0.5; // Special case
            default:
                return 0.7;
        }
    }
}
