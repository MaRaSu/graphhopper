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

import java.util.EnumMap;
import java.util.Map;

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

    // Internal-only refinement of SERVICE_ROAD: highway=service + service=driveway.
    // Stored on the edge so routing profiles can weight driveways (often a poor
    // through-route) separately, while the routing and TbT consumers collapse it back
    // to SERVICE_ROAD via toExternal(). Intentionally NOT mirrored in
    // route-profile-types.ts. Note this is not a blanket guarantee — see toExternal()
    // for which channels project and which still expose the raw value.
    // The client-facing signal for a driveway is the issue_driveway flag.
    SERVICE_DRIVEWAY,

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
    // Some PredictedHighway values are internal-only refinements: stored on the
    // edge for routing weight / analysis, and not distinguished by consumers that
    // intentionally reason at the coarser level (e.g. TbT instruction classification).
    // This map is the single registry of those refinements -> the coarse value they
    // collapse to. Add an entry here when a new internal-only value is introduced;
    // values absent from the map are already client-facing and project to themselves.
    private static final Map<PredictedHighway, PredictedHighway> EXTERNAL_PROJECTION =
            new EnumMap<>(PredictedHighway.class);
    static {
        EXTERNAL_PROJECTION.put(SERVICE_DRIVEWAY, SERVICE_ROAD);
    }

    /**
     * Project this (possibly internal-only) value to the coarser value that crosses
     * the API wire and that classification consumers should reason about. Identity for
     * values that are already client-facing.
     *
     * <p>Use at every boundary that must not leak an internal refinement: response
     * serialization to clients, and road/trail classification that should treat the
     * refinement like its coarse parent. Routing weight (custom models) and analysis
     * read the raw enum directly and therefore still see the fine-grained value.
     *
     * <p><b>Where this actually applies.</b> Projected: path details
     * (TrailmapPathDetailsBuilderFactory) and TbT instructions (every read in
     * TrailmapInstructionsFromEdges goes through its phOf() accessor). NOT projected —
     * these stock GraphHopper endpoints serialize the raw enum and there is no
     * per-value filter for them: {@code /info} (advertises the full constant list),
     * {@code /mvt} (per-edge attributes on vector tiles) and {@code /spt?columns=...}.
     * Accepted: the server is private and all clients are controlled. Do not restate
     * this as a blanket "clients never see it" guarantee — it is not one.
     * See docs/gh_service_driveway.md.
     */
    public PredictedHighway toExternal() {
        return EXTERNAL_PROJECTION.getOrDefault(this, this);
    }
}
