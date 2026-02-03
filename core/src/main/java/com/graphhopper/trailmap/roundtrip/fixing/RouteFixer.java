/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Strategy interface for fixing route quality issues.
 */
package com.graphhopper.trailmap.roundtrip.fixing;

import com.graphhopper.trailmap.roundtrip.config.RoundTripProfile;
import com.graphhopper.trailmap.roundtrip.scoring.IssueType;
import com.graphhopper.trailmap.roundtrip.scoring.RouteScore;
import com.graphhopper.util.shapes.GHPoint;

import java.util.List;
import java.util.Set;

/**
 * Strategy for fixing route quality issues by modifying waypoints.
 *
 * When a route doesn't meet quality criteria, fixers attempt to
 * improve it by adding, moving, or rotating waypoints.
 */
public interface RouteFixer {

    /**
     * Attempt to fix a route quality issue by modifying waypoints.
     *
     * @param currentWaypoints Current waypoint list
     * @param score Current route score with identified issues
     * @param problemLegIndex Index of the problematic leg (-1 for overall issues)
     * @param profile Routing profile
     * @return Fix result with modified waypoints, or failure
     */
    FixResult attemptFix(List<GHPoint> currentWaypoints,
                         RouteScore score,
                         int problemLegIndex,
                         RoundTripProfile profile);

    /**
     * @return Set of issue types this fixer can handle
     */
    Set<IssueType> handlesIssues();

    /**
     * @return Priority (lower = try first)
     */
    int priority();

    /**
     * @return Name of this fixer
     */
    String getName();

    /**
     * @return Description of what this fixer does
     */
    default String getDescription() {
        return getName();
    }
}
