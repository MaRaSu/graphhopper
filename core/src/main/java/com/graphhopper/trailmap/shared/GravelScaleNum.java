/*
 * Trailmap - GravelScaleNum encoded value
 *
 * Numeric representation of GravelScale for use in custom model formulas.
 * Values from DIFFICULTY_CONFIG.gravelBaseScores (route-profile-rules.ts).
 *
 * Scale: 0.0 to 10.0
 * - ZERO_MINUS: 0.5
 * - ZERO: 1.0
 * - ZERO_PLUS: 1.5
 * - ONE: 2.2
 * - TWO: 4.0
 * - THREE: 6.0
 * - FOUR: 10.0
 * - UNKNOWN: 4.0
 * - FERRY: 0
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.routing.ev.DecimalEncodedValue;
import com.graphhopper.routing.ev.DecimalEncodedValueImpl;

public class GravelScaleNum {

    public static final String KEY = "gravel_scale_num";

    public static DecimalEncodedValue create() {
        // Range 0-10, factor 0.5 for precision
        return new DecimalEncodedValueImpl(KEY, 5, 0, 0.5, false, false, false);
    }

    /**
     * Get numeric value for a GravelScale enum value.
     * Based on DIFFICULTY_CONFIG.gravelBaseScores.
     */
    public static double getNumericValue(GravelScale scale) {
        if (scale == null) {
            return 4.0; // UNKNOWN default
        }
        switch (scale) {
            case ZERO_MINUS: return 0.5;
            case ZERO: return 1.0;
            case ZERO_PLUS: return 1.5;
            case ONE: return 2.2;
            case TWO: return 4.0;
            case THREE: return 6.0;
            case FOUR: return 10.0;
            case UNKNOWN: return 4.0;
            case FERRY: return 0;
            default: return 4.0;
        }
    }
}
