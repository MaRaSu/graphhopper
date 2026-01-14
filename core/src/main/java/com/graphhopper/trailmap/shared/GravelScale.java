/*
 * Trailmap - GravelScale encoded value
 *
 * Unified difficulty scale for gravel/road cycling that combines multiple OSM tags
 * (surface, tracktype, smoothness, mtb:scale, etc.) into a single routing-friendly value.
 *
 * Scale values:
 * - ZERO_MINUS (0-): Paved surfaces - asphalt, concrete, paving stones
 * - ZERO (0): Well-maintained unpaved cycleways - fine gravel, excellent condition
 * - ZERO_PLUS (0+): Good gravel - compacted, fine gravel, good tracktypes
 * - ONE (1): Standard gravel - may have some roughness but rideable
 * - TWO (2): Rougher gravel - requires gravel bike, slower going
 * - THREE (3): Difficult terrain - uncertain conditions, risk of walking
 * - FOUR (4): Not rideable - paths, very rough tracks, mud, sand
 * - UNKNOWN: Missing data - treat conservatively
 * - FERRY: Ferry routes - special handling needed
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.util.Helper;

/**
 * GravelScale encodes the difficulty/suitability of a way for gravel cycling.
 * This is a synthetic value computed from multiple OSM tags during import.
 */
public enum GravelScale {
    // Order matters for comparison - easier to harder
    ZERO_MINUS,  // 0- : Paved surfaces
    ZERO,        // 0  : Well-maintained unpaved cycleways
    ZERO_PLUS,   // 0+ : Good gravel conditions
    ONE,         // 1  : Standard gravel
    TWO,         // 2  : Rougher gravel
    THREE,       // 3  : Difficult/uncertain
    FOUR,        // 4  : Not rideable (walking required)
    UNKNOWN,     // Missing data
    FERRY;       // Ferry routes

    public static final String KEY = "gravel_scale";

    public static EnumEncodedValue<GravelScale> create() {
        return new EnumEncodedValue<>(KEY, GravelScale.class);
    }

    /**
     * Returns the string representation used in custom models (0-, 0, 0+, 1, 2, 3, 4)
     */
    @Override
    public String toString() {
        switch (this) {
            case ZERO_MINUS: return "0-";
            case ZERO: return "0";
            case ZERO_PLUS: return "0+";
            case ONE: return "1";
            case TWO: return "2";
            case THREE: return "3";
            case FOUR: return "4";
            case UNKNOWN: return "unknown";
            case FERRY: return "ferry";
            default: return Helper.toLowerCase(super.toString());
        }
    }

    /**
     * Parse a string value to GravelScale.
     * Accepts both enum names (ZERO_MINUS) and display values (0-).
     */
    public static GravelScale find(String name) {
        if (Helper.isEmpty(name))
            return UNKNOWN;

        // Handle display format (0-, 0, 0+, 1, etc.)
        switch (name) {
            case "0-": return ZERO_MINUS;
            case "0": return ZERO;
            case "0+": return ZERO_PLUS;
            case "1": return ONE;
            case "2": return TWO;
            case "3": return THREE;
            case "4": return FOUR;
            case "unknown": return UNKNOWN;
            case "ferry": return FERRY;
        }

        // Try enum name format
        try {
            return GravelScale.valueOf(Helper.toUpperCase(name));
        } catch (IllegalArgumentException ex) {
            return UNKNOWN;
        }
    }

    /**
     * Check if this scale value is rideable (not walking/pushing required).
     */
    public boolean isRideable() {
        return this != FOUR && this != UNKNOWN;
    }

    /**
     * Check if this scale value represents paved surface.
     */
    public boolean isPaved() {
        return this == ZERO_MINUS;
    }

    /**
     * Get the ordinal value suitable for numeric comparisons in custom models.
     * Returns 0-8 where lower is better for gravel cycling.
     */
    public int getNumericValue() {
        return this.ordinal();
    }
}
