/*
 * Trailmap - PisteType encoded value
 *
 * Encodes the piste:type OSM tag for Nordic ski and winter sport routing.
 * Values from: https://wiki.openstreetmap.org/wiki/Key:piste:type
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.util.Helper;

/**
 * PisteType encodes the type of piste/ski trail from the OSM piste:type tag.
 */
public enum PisteType {
    // IMPORTANT: MISSING must be first (ordinal 0) - default when no tag present
    MISSING,     // No piste:type tag present (ordinal 0 = default)
    DOWNHILL,    // Alpine/downhill ski route
    NORDIC,      // Nordic/cross country ski trail
    SKITOUR,     // Backcountry ski touring
    SLED,        // Sledding piste
    HIKE,        // Winter hiking piste
    SLEIGH,      // Horse/husky drawn sleighs
    ICE_SKATE,   // Ice skating piste
    SNOW_PARK,   // Funpark with rails, quarter pipes
    PLAYGROUND,  // Ski playground for children
    SKI_JUMP,    // Ski jumping hill
    CONNECTION,  // Ways connecting ski lifts
    FATBIKE,     // Fatbike winter piste
    SNOWMOBILE,  // Snowmobile trails
    OTHER;       // Unrecognized piste:type value

    public static final String KEY = "piste_type";

    public static EnumEncodedValue<PisteType> create() {
        return new EnumEncodedValue<>(KEY, PisteType.class);
    }

    @Override
    public String toString() {
        return Helper.toLowerCase(super.toString());
    }

    public static PisteType find(String name) {
        if (Helper.isEmpty(name))
            return MISSING;
        try {
            return PisteType.valueOf(Helper.toUpperCase(name));
        } catch (IllegalArgumentException ex) {
            return MISSING;
        }
    }

    /**
     * Parse OSM piste:type tag value to enum.
     * Values from: https://wiki.openstreetmap.org/wiki/Key:piste:type
     */
    public static PisteType fromOsmTag(String osmValue) {
        if (osmValue == null || osmValue.isEmpty())
            return MISSING;

        switch (osmValue.toLowerCase()) {
            case "downhill": return DOWNHILL;
            case "nordic": return NORDIC;
            case "skitour": return SKITOUR;
            case "sled": return SLED;
            case "hike": return HIKE;
            case "sleigh": return SLEIGH;
            case "ice_skate": return ICE_SKATE;
            case "snow_park": return SNOW_PARK;
            case "playground": return PLAYGROUND;
            case "ski_jump": return SKI_JUMP;
            case "connection": return CONNECTION;
            case "fatbike": return FATBIKE;
            case "snowmobile": return SNOWMOBILE;
            default: return OTHER;
        }
    }
}
