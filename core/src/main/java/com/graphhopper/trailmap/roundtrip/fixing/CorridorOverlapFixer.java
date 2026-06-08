/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * CorridorOverlapFixer - diverts a leg that runs alongside another part of the route.
 */
package com.graphhopper.trailmap.roundtrip.fixing;

import com.graphhopper.trailmap.roundtrip.config.RoundTripProfile;
import com.graphhopper.trailmap.roundtrip.scoring.IssueType;
import com.graphhopper.trailmap.roundtrip.scoring.LegScore;
import com.graphhopper.trailmap.roundtrip.scoring.RouteScore;
import com.graphhopper.util.shapes.GHPoint;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * CorridorOverlapFixer breaks up "go out / come back on a parallel path" sections that the
 * edge-ID repetition fixers cannot see (different edges, ~10m apart).
 *
 * Strategy (lateral re-anchor): the scorer marks the overlapping (return) corridor with an
 * anchor point and a partner point on the other (outbound) corridor. This fixer inserts a
 * shaping via-point offset laterally AWAY from the partner, forcing that leg to bow out onto
 * a genuinely different road instead of the adjacent one.
 *
 * If no different road exists nearby the re-routed leg simply comes back to a similar shape;
 * the score then stays unacceptable and the loop falls through to other fixers / best-effort.
 */
public class CorridorOverlapFixer extends AbstractRouteFixer {

    private static final Logger logger = LoggerFactory.getLogger(CorridorOverlapFixer.class);

    /** Minimum corridor overlap on a leg to attempt a fix (meters). */
    private static final double MIN_OVERLAP_TO_FIX = 250.0;

    /** How far to push the shaping via-point off the shared corridor (meters). */
    private static final double LATERAL_OFFSET = 300.0;

    /** Skip if the new via-point would land within this distance of an existing waypoint (meters). */
    private static final double NEARBY_THRESHOLD = 150.0;

    /** Don't grow the waypoint list beyond this (keeps the fix loop bounded). */
    private static final int MAX_WAYPOINTS = 25;

    @Override
    public String getName() {
        return "corridor-overlap";
    }

    @Override
    public String getDescription() {
        return "Insert a shaping via-point to divert a leg off a nearby parallel corridor";
    }

    @Override
    public Set<IssueType> handlesIssues() {
        return Set.of(IssueType.CORRIDOR_OVERLAP);
    }

    @Override
    public int priority() {
        return 6; // After dead-end (5), before distance (8): it is a route-shape fix.
    }

    @Override
    public FixResult attemptFix(List<GHPoint> currentWaypoints,
                                 RouteScore score,
                                 int problemLegIndex,
                                 RoundTripProfile profile) {

        if (currentWaypoints == null || currentWaypoints.size() < 3) {
            return FixResult.failure("Need at least 3 waypoints to fix corridor overlap", getName());
        }
        if (score == null || score.getLegScores().isEmpty()) {
            return FixResult.failure("No scoring data available", getName());
        }
        if (currentWaypoints.size() >= MAX_WAYPOINTS) {
            return FixResult.failure("Waypoint limit reached, cannot insert shaping via-point", getName());
        }

        // Pick the leg with the worst corridor overlap that has an anchor + partner.
        LegScore worst = null;
        double worstOverlap = MIN_OVERLAP_TO_FIX;
        for (LegScore leg : score.getLegScores()) {
            if (leg.getCorridorOverlapDistance() > worstOverlap
                    && leg.getCorridorAnchor() != null && leg.getCorridorPartner() != null) {
                worstOverlap = leg.getCorridorOverlapDistance();
                worst = leg;
            }
        }

        if (worst == null) {
            return FixResult.failure("No corridor overlap with anchor/partner above threshold", getName());
        }

        int legIndex = worst.getLegIndex();
        GHPoint anchor = worst.getCorridorAnchor();
        GHPoint partner = worst.getCorridorPartner();

        // Insertion point shapes leg `legIndex` (waypoint[legIndex] -> waypoint[legIndex+1]).
        int insertIndex = legIndex + 1;
        if (insertIndex <= 0 || insertIndex > currentWaypoints.size() - 1) {
            return FixResult.failure("Cannot shape start/end leg", getName());
        }

        // Push the via-point away from the outbound corridor (partner -> anchor direction).
        double offsetBearing = bearing(partner, anchor);
        GHPoint via = project(anchor, offsetBearing, LATERAL_OFFSET);

        // Avoid clustering near an existing waypoint.
        for (GHPoint wp : currentWaypoints) {
            if (distance(wp, via) < NEARBY_THRESHOLD) {
                return FixResult.failure(
                    String.format("Shaping via-point too close (%.0fm) to an existing waypoint",
                        distance(wp, via)), getName());
            }
        }

        List<GHPoint> modified = new ArrayList<>(currentWaypoints);
        modified.add(insertIndex, via);

        String description = String.format(
            "Inserted shaping via-point at (%.5f,%.5f) to divert leg %d off %.0fm parallel corridor",
            via.getLat(), via.getLon(), legIndex, worstOverlap);
        logger.info("CorridorOverlapFixer: {}", description);

        return FixResult.success(modified, description, getName());
    }
}
