/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Strategy interface for waypoint snapping.
 */
package com.graphhopper.trailmap.roundtrip.snapping;

import com.graphhopper.routing.ev.EncodedValueLookup;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.trailmap.roundtrip.config.RoundTripProfile;
import com.graphhopper.util.shapes.GHPoint;

/**
 * Strategy for snapping waypoints to the road network.
 * Different implementations can use different criteria for selecting the best snap point.
 */
public interface WaypointSnapStrategy {

    /**
     * Find the best snap point for a target location.
     *
     * @param targetPoint The projected waypoint location
     * @param profile Routing profile with snapping preferences
     * @param locationIndex GH location index for finding nearby edges
     * @param edgeFilter Base edge filter (e.g., subnetwork filter)
     * @param encodedValueLookup For reading encoded values from edges
     * @return Best snap, or null if none found within acceptable parameters
     */
    Snap findBestSnap(GHPoint targetPoint,
                      RoundTripProfile profile,
                      LocationIndex locationIndex,
                      EdgeFilter edgeFilter,
                      EncodedValueLookup encodedValueLookup);

    /**
     * Find the best snap point with snapping preference and seed-based variation.
     *
     * This extended method allows for:
     * - Preference-based scoring (gravel_scale, mtb_scale aware)
     * - Seed-based weighted random selection among top candidates
     *
     * @param targetPoint The projected waypoint location
     * @param profile Routing profile with snapping preferences
     * @param locationIndex GH location index for finding nearby edges
     * @param edgeFilter Base edge filter (e.g., subnetwork filter)
     * @param encodedValueLookup For reading encoded values from edges
     * @param preference Snapping preference pattern (determines scoring)
     * @param seed Random seed for deterministic variation
     * @param waypointIndex Index of this waypoint (for variation lookup)
     * @return Best snap, or null if none found within acceptable parameters
     */
    default Snap findBestSnap(GHPoint targetPoint,
                              RoundTripProfile profile,
                              LocationIndex locationIndex,
                              EdgeFilter edgeFilter,
                              EncodedValueLookup encodedValueLookup,
                              SnappingPreference preference,
                              long seed,
                              int waypointIndex) {
        // Default: ignore new params, call original
        return findBestSnap(targetPoint, profile, locationIndex, edgeFilter, encodedValueLookup);
    }

    /**
     * @return Name of this snapping strategy
     */
    String getName();

    /**
     * @return Whether this strategy considers profile preferences (vs just distance)
     */
    default boolean isProfileAware() {
        return false;
    }
}
