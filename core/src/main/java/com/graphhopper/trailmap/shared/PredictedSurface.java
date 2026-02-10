/*
 * Trailmap - PredictedSurface encoded value
 *
 * Predicts the surface type from multiple OSM tags when surface tag is missing.
 * Profile-independent - used by all routing profiles for UI display and analysis.
 *
 * Full port of PredictedSurface enum from route-profile-types.ts
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.util.Helper;

/**
 * PredictedSurface encodes the predicted surface type for each edge.
 * Values match route-profile-types.ts PredictedSurface enum.
 */
public enum PredictedSurface {
    FERRY,          // Ferry routes
    ASPHALT,            // Includes all surfaces with similar benefits than asphalt
    ASPHALT_OR_UNPAVED, // Ambiguous: highway type equally likely paved or unpaved, no indicators
    COMPACTED,          // Meant for regular use, well-maintained
    FINE_GRAVEL,    // Outdoor paths
    MEDIUM_GRAVEL,  // "In between", ok for non-road bikes, but slower
    ROUGH_GRAVEL,   // Really big gravel, hard to cycle on
    GROUND,         // All ground types except the ones below
    SAND,           // Loose sand, difficult to cycle on
    MUD,            // Soft, wet earth, very difficult to cycle on
    GRASS,          // General grass surfaces (reserved for future use)
    UNKNOWN;        // Default when no rule matches

    public static final String KEY = "predicted_surface";

    public static EnumEncodedValue<PredictedSurface> create() {
        return new EnumEncodedValue<>(KEY, PredictedSurface.class);
    }

    @Override
    public String toString() {
        return Helper.toLowerCase(super.toString());
    }

    public static PredictedSurface find(String name) {
        if (Helper.isEmpty(name))
            return UNKNOWN;
        try {
            return PredictedSurface.valueOf(Helper.toUpperCase(name));
        } catch (IllegalArgumentException ex) {
            return UNKNOWN;
        }
    }
}
