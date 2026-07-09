/*
 * Trailmap - PredictedHighway encoded value
 *
 * Predicts the highway category from multiple OSM tags.
 * Profile-independent - used by all routing profiles for UI display and analysis.
 *
 * Full port of PredictedHighway enum from route-profile-types.ts
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.util.Helper;

/**
 * PredictedHighway encodes the predicted highway category for each edge.
 * Values match route-profile-types.ts PredictedHighway enum.
 */
public enum PredictedHighway {
    FERRY,          // Ferry routes

    // Roads which are part of the road network i.e. vehicles use these normally
    MOTORWAY,       // No cycling allowed
    MAJOR_ROAD,     // Cycling allowed, but not preferred (trunk, primary)
    MINOR_ROAD,     // Ideal for cycling (secondary, tertiary, residential, unclassified)

    // Smaller roads, not part of official road network, but passable with a car
    SERVICE_ROAD,   // Suitable for cycling, but may be rough - forest road or service access

    // Dedicated cycling & outdoors infrastructure
    CYCLEWAY,       // Dedicated cycling infrastructure
    FOOTWAY,        // highway=footway without bicycle permission - pedestrian only
    OUTDOOR_WAY,    // Wide outdoor paths and trails - suitable for cycling and well maintained
    OUTDOOR_PATH,   // Outdoor paths, smaller and less maintained, often suitable for cycling

    // Tracks i.e. not built roads or built roads where maintenance has ended long time ago
    GOOD_TRACK,     // Tractor or similar vehicles have created, ok for cycling
    ROUGH_TRACK,    // Tractor or similar vehicles have created, difficult or impossible to cycle

    // Paths i.e. not built, not maintained but formed through people and / or animals
    CITY_PATH,      // Paths in urban areas, often well-trodden but not officially maintained
    PATH,           // General "forest" paths

    UNKNOWN;        // Default when no rule matches

    public static final String KEY = "predicted_highway";

    public static EnumEncodedValue<PredictedHighway> create() {
        return new EnumEncodedValue<>(KEY, PredictedHighway.class);
    }

    @Override
    public String toString() {
        return Helper.toLowerCase(super.toString());
    }

    public static PredictedHighway find(String name) {
        if (Helper.isEmpty(name))
            return UNKNOWN;
        try {
            return PredictedHighway.valueOf(Helper.toUpperCase(name));
        } catch (IllegalArgumentException ex) {
            return UNKNOWN;
        }
    }

    // =====================================================================
    // Internal -> external projection
    // =====================================================================
    //
    // Some PredictedHighway values may be internal-only refinements: stored on the
    // edge for routing weight / analysis, but never exposed to API clients and not
    // distinguished by consumers that intentionally reason at the coarser level
    // (e.g. TbT instruction classification). There are none at present, so this is
    // the identity. When an internal-only value is introduced (e.g. a
    // service=driveway split of SERVICE_ROAD), project it to its coarse parent here.

    /**
     * Project this (possibly internal-only) value to the coarser value that crosses
     * the API wire and that classification consumers should reason about. Identity for
     * values that are already client-facing (currently all of them).
     *
     * <p>Called at every boundary that must not leak an internal refinement: response
     * serialization to clients, and road/trail classification that should treat a
     * refinement like its coarse parent. Routing weight (custom models) and analysis
     * read the raw enum directly and would still see the fine-grained value.
     */
    public PredictedHighway toExternal() {
        return this;
    }
}
