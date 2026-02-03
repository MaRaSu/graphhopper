/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * DeadEndFixer - moves waypoints to eliminate dead-end backtracking.
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
 * DeadEndFixer moves or removes waypoints to eliminate dead-end backtracking.
 *
 * Strategy:
 * 1. Find the leg with the worst dead-end (highest deadEndDistance)
 * 2. Get the forkPoint where the backtracking begins
 * 3. Check if another waypoint already exists near the forkPoint
 * 4. If nearby waypoint exists: REMOVE the dead-end waypoint (fork already covered)
 *    If no nearby waypoint: MOVE the waypoint to the forkPoint
 *
 * This causes the route to turn around at the fork rather than
 * going into the dead-end and coming back.
 *
 * The nearby-check prevents waypoint clustering when multiple dead-ends
 * on the same road get fixed - each fix would otherwise place waypoints
 * at the same fork, creating tiny stub legs that bypass detection.
 *
 * Use cases:
 * - Route goes out and returns on same road (dead-end road)
 * - Waypoint placed beyond road network end
 * - Constrained terrain (valley, peninsula) causing backtracking
 */
public class DeadEndFixer extends AbstractRouteFixer {

    private static final Logger logger = LoggerFactory.getLogger(DeadEndFixer.class);

    /** Minimum dead-end distance to trigger fix (meters) */
    private static final double MIN_DEAD_END_TO_FIX = 50.0;

    /** If another waypoint is within this distance of fork, remove instead of move */
    private static final double NEARBY_THRESHOLD = 150.0;

    /** Minimum waypoints required (start + at least 1 intermediate + end) */
    private static final int MIN_WAYPOINTS = 3;

    @Override
    public String getName() {
        return "dead-end";
    }

    @Override
    public String getDescription() {
        return "Move waypoint to fork point to eliminate dead-end backtracking";
    }

    @Override
    public Set<IssueType> handlesIssues() {
        return Set.of(
            IssueType.DEAD_END_BACKTRACK,
            IssueType.HIGH_REPETITION
        );
    }

    @Override
    public int priority() {
        return 5; // High priority - dead-ends are clearly identified and fixable
    }

    @Override
    public FixResult attemptFix(List<GHPoint> currentWaypoints,
                                 RouteScore score,
                                 int problemLegIndex,
                                 RoundTripProfile profile) {

        // Validate inputs
        if (currentWaypoints == null || currentWaypoints.size() < MIN_WAYPOINTS) {
            return FixResult.failure("Need at least 3 waypoints to fix dead-end", getName());
        }

        if (score == null || score.getLegScores().isEmpty()) {
            return FixResult.failure("No scoring data available", getName());
        }

        // Find the leg with the worst dead-end
        LegScore worstDeadEndLeg = null;
        double worstDeadEndDistance = MIN_DEAD_END_TO_FIX;

        for (LegScore leg : score.getLegScores()) {
            if (leg.getDeadEndDistance() > worstDeadEndDistance && leg.getForkPoint() != null) {
                worstDeadEndDistance = leg.getDeadEndDistance();
                worstDeadEndLeg = leg;
            }
        }

        if (worstDeadEndLeg == null) {
            return FixResult.failure("No dead-end with fork point found above threshold", getName());
        }

        int legIndex = worstDeadEndLeg.getLegIndex();
        GHPoint forkPoint = worstDeadEndLeg.getForkPoint();

        // Waypoint to fix is at legIndex + 1
        // (leg N goes from waypoint N to waypoint N+1, dead-end is at waypoint N+1)
        int waypointToFix = legIndex + 1;

        if (waypointToFix <= 0 || waypointToFix >= currentWaypoints.size() - 1) {
            return FixResult.failure("Cannot modify start or end waypoint", getName());
        }

        // Check if another waypoint already exists near the fork point
        int nearbyWaypointIndex = findWaypointNearFork(currentWaypoints, forkPoint, waypointToFix);

        List<GHPoint> modified = new ArrayList<>(currentWaypoints);
        String description;

        if (nearbyWaypointIndex >= 0) {
            // Another waypoint already covers this fork - remove the dead-end waypoint
            if (currentWaypoints.size() <= MIN_WAYPOINTS) {
                return FixResult.failure(
                    "Cannot remove waypoint - minimum waypoints reached", getName());
            }

            modified.remove(waypointToFix);
            description = String.format(
                "Removed waypoint %d (fork already covered by waypoint %d at %.0fm) to eliminate %.0fm dead-end",
                waypointToFix, nearbyWaypointIndex,
                distance(currentWaypoints.get(nearbyWaypointIndex), forkPoint),
                worstDeadEndDistance);

            logger.info("DeadEndFixer: {}", description);

        } else {
            // No nearby waypoint - move to fork point
            GHPoint oldWaypoint = currentWaypoints.get(waypointToFix);
            double moveDistance = distance(oldWaypoint, forkPoint);

            if (moveDistance < 20) {
                return FixResult.failure(
                    String.format("Fork point too close to current waypoint (%.0fm)", moveDistance),
                    getName());
            }

            modified.set(waypointToFix, forkPoint);
            description = String.format(
                "Moved waypoint %d to fork point (%.5f,%.5f) to eliminate %.0fm dead-end",
                waypointToFix, forkPoint.getLat(), forkPoint.getLon(), worstDeadEndDistance);

            logger.info("DeadEndFixer: {}", description);
        }

        return FixResult.success(modified, description, getName());
    }

    /**
     * Check if any waypoint (other than excludeIndex) is within NEARBY_THRESHOLD of forkPoint.
     *
     * @return Index of nearby waypoint, or -1 if none found
     */
    private int findWaypointNearFork(List<GHPoint> waypoints, GHPoint forkPoint, int excludeIndex) {
        for (int i = 0; i < waypoints.size(); i++) {
            if (i == excludeIndex) {
                continue;
            }
            if (distance(waypoints.get(i), forkPoint) < NEARBY_THRESHOLD) {
                return i;
            }
        }
        return -1;
    }
}
