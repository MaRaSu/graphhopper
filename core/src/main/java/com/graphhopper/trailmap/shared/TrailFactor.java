/*
 * Trailmap - TrailFactor encoded value
 *
 * Pre-computed trail quality penalty factor (0.0 to 1.0) combining:
 * - width: narrow paths penalized below 0.6m threshold (Finnish OSM mapping convention)
 * - trail_visibility: bad/horrible/no visibility forces near-pushing speed
 * - obstacle=vegetation: overgrown trails penalized
 * - smoothness: surface condition penalty, moderated when mtb:scale is present
 *
 * Used as a direct multiplier on both speed and priority in custom model JSON.
 * Value of 1.0 means no penalty. Lower values = stronger penalty.
 *
 * Thresholds based on OSRM MTB profile with Finnish mapping adjustments.
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.routing.ev.DecimalEncodedValue;
import com.graphhopper.routing.ev.DecimalEncodedValueImpl;

public class TrailFactor {

    public static final String KEY = "trail_factor";

    public static DecimalEncodedValue create() {
        // Range 0-1.55, factor 0.05 (5 bits = 32 steps, we clamp to 1.0)
        return new DecimalEncodedValueImpl(KEY, 5, 0, 0.05, false, false, false);
    }
}
