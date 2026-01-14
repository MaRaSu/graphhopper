/*
 * Trailmap - MtbBasePriority encoded value
 *
 * Pre-computed priority for MTB routing profile.
 * Represents how suitable an edge is for MTB cycling (0.0 to 1.0).
 *
 * Initial curve (user will tune):
 * - Low scales (0-, 0, 0+) are perfectly fine for MTB
 * - Technical terrain (1-3) is often preferred
 * - Very technical (4+) requires skill limits
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.routing.ev.DecimalEncodedValue;
import com.graphhopper.routing.ev.DecimalEncodedValueImpl;

public class MtbBasePriority {

    public static final String KEY = "mtb_base_priority";

    public static DecimalEncodedValue create() {
        // Range 0-1, factor 0.05 for precision (20 steps)
        return new DecimalEncodedValueImpl(KEY, 5, 0, 0.05, false, false, false);
    }

    /**
     * Get base priority for MTB profile.
     * Initial values - will be tuned based on testing.
     */
    public static double getPriority(MtbScale scale) {
        if (scale == null) {
            return 0.8;
        }
        switch (scale) {
            case ZERO_MINUS: return 0.9;  // Easy, fine for MTB
            case ZERO: return 1.0;        // Easy trails, good
            case ZERO_PLUS: return 1.0;   // Light technical, good
            case ONE: return 1.0;         // Basic singletrack, good
            case TWO: return 0.95;        // Intermediate technical
            case THREE: return 0.85;      // Advanced - most riders ok
            case FOUR: return 0.6;        // Expert only
            case FIVE: return 0.4;        // Extreme
            case SIX: return 0.2;         // Very extreme
            case UNKNOWN: return 0.8;     // Conservative
            case FERRY: return 0.5;       // Special case
            default: return 0.8;
        }
    }
}
