/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Strategy interface for scoring route quality.
 */
package com.graphhopper.trailmap.roundtrip.scoring;

import com.graphhopper.routing.Path;
import com.graphhopper.routing.ev.EncodedValueLookup;
import com.graphhopper.trailmap.roundtrip.config.RoundTripProfile;
import com.graphhopper.util.shapes.GHPoint;

import java.util.List;

/**
 * Strategy for scoring route quality against profile criteria.
 */
public interface RouteScorer {

    /**
     * Score a complete route (list of path legs).
     *
     * @param paths List of path legs making up the route
     * @param waypoints Original waypoints (for directness calculation)
     * @param profile Profile with scoring criteria
     * @param encodedValueLookup For reading edge encoded values
     * @param targetDistance Target distance in meters (for distance issue detection)
     * @return Detailed score breakdown
     */
    RouteScore score(List<Path> paths,
                     List<GHPoint> waypoints,
                     RoundTripProfile profile,
                     EncodedValueLookup encodedValueLookup,
                     double targetDistance);

    /**
     * @return Name of this scoring strategy
     */
    String getName();
}
