/*
 * Trailmap - MtbScale encoded value
 *
 * MTB technical difficulty scale based on mtb:scale OSM tag.
 * Includes inference for paths/tracks without explicit mtb:scale.
 *
 * Full port of MtbScale enum from route-profile-types.ts
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.util.Helper;

/**
 * MtbScale encodes the MTB technical difficulty for each edge.
 * Values match route-profile-types.ts MtbScale enum.
 *
 * Based on: https://wiki.openstreetmap.org/wiki/Key:mtb:scale
 */
public enum MtbScale {
    ZERO_MINUS,  // 0- : Easy, smooth surface, no obstacles
    ZERO,        // 0  : Easy, minor obstacles possible
    ZERO_PLUS,   // 0+ : Easy-intermediate transition
    ONE,         // 1  : Intermediate, small obstacles
    TWO,         // 2  : Difficult, larger obstacles
    THREE,       // 3  : Very difficult, large obstacles
    FOUR,        // 4  : Extremely difficult
    FIVE,        // 5  : Extreme, pushing likely
    SIX,         // 6  : Unrideable, walking required
    UNKNOWN,     // Unknown difficulty
    FERRY;       // Ferry routes

    public static final String KEY = "mtb_scale";

    public static EnumEncodedValue<MtbScale> create() {
        return new EnumEncodedValue<>(KEY, MtbScale.class);
    }

    /**
     * Returns the string representation used in custom models (0-, 0, 0+, 1, 2, 3, 4, 5, 6)
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
            case FIVE: return "5";
            case SIX: return "6";
            case UNKNOWN: return "unknown";
            case FERRY: return "ferry";
            default: return Helper.toLowerCase(super.toString());
        }
    }

    /**
     * Parse a string value to MtbScale.
     * Accepts both enum names (ZERO_MINUS) and display values (0-).
     */
    public static MtbScale find(String name) {
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
            case "5": return FIVE;
            case "6": return SIX;
            case "unknown": return UNKNOWN;
            case "ferry": return FERRY;
        }

        // Try enum name format
        try {
            return MtbScale.valueOf(Helper.toUpperCase(name));
        } catch (IllegalArgumentException ex) {
            return UNKNOWN;
        }
    }

    /**
     * Map OSM mtb:scale tag value to MtbScale enum.
     * Handles variants like 0-, 0, 0+, 1-, 1, 1+, etc.
     */
    public static MtbScale fromOsmTag(String osmValue) {
        if (osmValue == null || osmValue.isEmpty()) {
            return null;
        }

        // Special cases for 0 range
        if ("0-".equals(osmValue)) return ZERO_MINUS;
        if ("0".equals(osmValue)) return ZERO;
        if ("0+".equals(osmValue)) return ZERO_PLUS;

        // For 1 and higher, extract base number
        // 1-, 1, 1+ all map to ONE; 2-, 2, 2+ all map to TWO; etc.
        String baseNumber = osmValue.replaceAll("[+-]", "");
        try {
            int scale = Integer.parseInt(baseNumber);
            switch (scale) {
                case 1: return ONE;
                case 2: return TWO;
                case 3: return THREE;
                case 4: return FOUR;
                case 5: return FIVE;
                case 6: return SIX;
                default: return UNKNOWN;
            }
        } catch (NumberFormatException e) {
            return UNKNOWN;
        }
    }
}
