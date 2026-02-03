/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * KinkFixer - adds a perpendicular waypoint to increase route variety.
 */
package com.graphhopper.trailmap.roundtrip.fixing;

import com.graphhopper.trailmap.roundtrip.config.RoundTripProfile;
import com.graphhopper.trailmap.roundtrip.scoring.IssueType;
import com.graphhopper.trailmap.roundtrip.scoring.RouteScore;
import com.graphhopper.util.shapes.GHPoint;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * KinkFixer adds a waypoint perpendicular to a problem leg.
 *
 * Strategy:
 * 1. Find the midpoint of the problem leg
 * 2. Calculate perpendicular bearing
 * 3. Project a "kink" point perpendicular to the leg
 * 4. Insert the kink point into the waypoint list
 *
 * This forces the route to bend, avoiding straight-line paths
 * and potentially finding different road types.
 *
 * Use cases:
 * - Route is too short or too direct
 * - A leg has poor surface/road quality
 * - Need to add variety to the route
 */
public class KinkFixer extends AbstractRouteFixer {

    private static final Logger logger = LoggerFactory.getLogger(KinkFixer.class);

    /** Kink distance as fraction of leg length */
    private final double kinkDistanceRatio;

    /** Whether to try both perpendicular directions */
    private final boolean tryBothDirections;

    /**
     * Create with default kink distance ratio of 0.3 (30% of leg length).
     */
    public KinkFixer() {
        this(0.3, false);
    }

    /**
     * Create with specified parameters.
     *
     * @param kinkDistanceRatio Kink distance as fraction of leg length (0.1 - 0.5)
     * @param tryBothDirections If true, try +90° first, then -90° if that fails
     */
    public KinkFixer(double kinkDistanceRatio, boolean tryBothDirections) {
        this.kinkDistanceRatio = Math.max(0.1, Math.min(0.5, kinkDistanceRatio));
        this.tryBothDirections = tryBothDirections;
    }

    @Override
    public String getName() {
        return "kink";
    }

    @Override
    public String getDescription() {
        return "Add perpendicular waypoint to increase route variety";
    }

    @Override
    public Set<IssueType> handlesIssues() {
        return Set.of(
            IssueType.TOO_SHORT,
            IssueType.TOO_DIRECT,
            IssueType.LOW_QUALITY_LEG,
            IssueType.HIGH_ASPHALT_RATIO,
            IssueType.MAIN_ROAD_TRAP
        );
    }

    @Override
    public int priority() {
        return 10; // Try first - least disruptive fix
    }

    @Override
    public FixResult attemptFix(List<GHPoint> currentWaypoints,
                                 RouteScore score,
                                 int problemLegIndex,
                                 RoundTripProfile profile) {

        // Validate inputs
        if (currentWaypoints == null || currentWaypoints.size() < 3) {
            return FixResult.failure("Need at least 3 waypoints to add kink", getName());
        }

        // Determine which leg to fix
        int legIdx = problemLegIndex;
        if (legIdx < 0) {
            // No specific leg - find the longest one
            legIdx = findLongestLeg(currentWaypoints);
        }

        if (legIdx < 0 || legIdx >= currentWaypoints.size() - 1) {
            return FixResult.failure("Invalid leg index: " + legIdx, getName());
        }

        GHPoint legStart = currentWaypoints.get(legIdx);
        GHPoint legEnd = currentWaypoints.get(legIdx + 1);

        // Calculate leg properties
        double legDistance = distance(legStart, legEnd);
        double legBearing = bearing(legStart, legEnd);

        if (legDistance < 100) {
            return FixResult.failure("Leg too short for kink: " + legDistance + "m", getName());
        }

        // Calculate kink point
        GHPoint midpoint = midpoint(legStart, legEnd);
        double kinkDistance = legDistance * kinkDistanceRatio;

        // First try: perpendicular to the right (+90°)
        double perpBearing = normalizeBearing(legBearing + 90);
        GHPoint kinkPoint = project(midpoint, perpBearing, kinkDistance);

        // Create modified waypoint list
        List<GHPoint> modified = new ArrayList<>(currentWaypoints.size() + 1);
        for (int i = 0; i <= legIdx; i++) {
            modified.add(currentWaypoints.get(i));
        }
        modified.add(kinkPoint);
        for (int i = legIdx + 1; i < currentWaypoints.size(); i++) {
            modified.add(currentWaypoints.get(i));
        }

        String description = String.format(
            "Added kink at leg %d: %.0fm perpendicular at %.0f° from midpoint",
            legIdx, kinkDistance, perpBearing);

        logger.debug("KinkFixer: {}", description);

        return FixResult.success(modified, description, getName());
    }

    /**
     * Find the longest leg in the waypoint list.
     */
    private int findLongestLeg(List<GHPoint> waypoints) {
        int longestIdx = 0;
        double maxDist = 0;

        for (int i = 0; i < waypoints.size() - 1; i++) {
            double dist = distance(waypoints.get(i), waypoints.get(i + 1));
            if (dist > maxDist) {
                maxDist = dist;
                longestIdx = i;
            }
        }

        return longestIdx;
    }
}
