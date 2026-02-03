/*
 * Trailmap - MtbWinter encoded value
 *
 * Encodes winter MTB trail suitability from mtb:winter OSM tag.
 * Values: no (not suitable), 1 (best), 2/yes (good), 3 (rideable)
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.util.Helper;

/**
 * MtbWinter encodes winter MTB trail suitability from mtb:winter tag.
 */
public enum MtbWinter {
    // MISSING must be first (ordinal 0) - default when no tag present
    MISSING,    // No mtb:winter tag
    NO,         // "no": Not suitable for winter MTB
    ONE,        // "1": Best winter MTB conditions
    TWO,        // "2" or "yes": Good winter MTB conditions
    THREE;      // "3": Rideable but challenging

    public static final String KEY = "mtb_winter";

    public static EnumEncodedValue<MtbWinter> create() {
        return new EnumEncodedValue<>(KEY, MtbWinter.class);
    }

    @Override
    public String toString() {
        switch (this) {
            case MISSING: return "missing";
            case NO: return "no";
            case ONE: return "1";
            case TWO: return "2";
            case THREE: return "3";
            default: return Helper.toLowerCase(super.toString());
        }
    }

    public static MtbWinter find(String name) {
        if (Helper.isEmpty(name))
            return MISSING;
        switch (name) {
            case "no": case "0": return NO;
            case "1": return ONE;
            case "2": case "yes": return TWO;
            case "3": return THREE;
            case "missing": return MISSING;
        }
        try {
            return MtbWinter.valueOf(Helper.toUpperCase(name));
        } catch (IllegalArgumentException ex) {
            return MISSING;
        }
    }

    public static MtbWinter fromOsmTag(String osmValue) {
        if (osmValue == null || osmValue.isEmpty())
            return MISSING;
        switch (osmValue.trim().toLowerCase()) {
            case "no": return NO;
            case "1": return ONE;
            case "2": return TWO;
            case "yes": return TWO;
            case "3": return THREE;
            default: return MISSING;
        }
    }
}
