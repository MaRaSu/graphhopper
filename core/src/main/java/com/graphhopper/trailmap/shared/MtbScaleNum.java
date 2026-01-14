/*
 * Trailmap - MtbScaleNum encoded value
 *
 * Numeric representation of MtbScale for use in custom model formulas.
 * Values from DIFFICULTY_CONFIG.mtbBaseScores (route-profile-rules.ts).
 *
 * Scale: 0.0 to 20.0
 * - ZERO_MINUS: 0.5
 * - ZERO: 1.0
 * - ZERO_PLUS: 1.5
 * - ONE: 2.0
 * - TWO: 3.5
 * - THREE: 7.0
 * - FOUR: 13.0
 * - FIVE: 15.0
 * - SIX: 20.0
 * - UNKNOWN: 5.0
 * - FERRY: 0
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.routing.ev.DecimalEncodedValue;
import com.graphhopper.routing.ev.DecimalEncodedValueImpl;

public class MtbScaleNum {

    public static final String KEY = "mtb_scale_num";

    public static DecimalEncodedValue create() {
        // Range 0-20, factor 0.5 for precision
        return new DecimalEncodedValueImpl(KEY, 6, 0, 0.5, false, false, false);
    }

    /**
     * Get numeric value for an MtbScale enum value.
     * Based on DIFFICULTY_CONFIG.mtbBaseScores.
     */
    public static double getNumericValue(MtbScale scale) {
        if (scale == null) {
            return 5.0; // UNKNOWN default
        }
        switch (scale) {
            case ZERO_MINUS: return 0.5;
            case ZERO: return 1.0;
            case ZERO_PLUS: return 1.5;
            case ONE: return 2.0;
            case TWO: return 3.5;
            case THREE: return 7.0;
            case FOUR: return 13.0;
            case FIVE: return 15.0;
            case SIX: return 20.0;
            case UNKNOWN: return 5.0;
            case FERRY: return 0;
            default: return 5.0;
        }
    }
}
