/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * AnchorSwapFixer - moves a waypoint to escape bad road corridors.
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
 * AnchorSwapFixer moves a waypoint to a different sector.
 *
 * Strategy:
 * 1. Target the waypoint at the end of a problem leg
 * 2. Generate candidate positions in different angular sectors
 * 3. Select a candidate that's in a different sector from current
 * 4. Replace the waypoint with the new position
 *
 * This forces the router to find completely different roads
 * to reach the new waypoint location.
 *
 * Use cases:
 * - Route stuck on main roads ("main road trap")
 * - Too much asphalt when profile prefers gravel
 * - Breaking out of a corridor
 */
public class AnchorSwapFixer extends AbstractRouteFixer {

    private static final Logger logger = LoggerFactory.getLogger(AnchorSwapFixer.class);

    /** Number of candidate positions to generate */
    private final int candidateCount;

    /** Angular offset between candidates (degrees) */
    private final double sectorAngle;

    /** Whether to maintain distance from start */
    private final boolean maintainDistance;

    /**
     * Create with default parameters.
     */
    public AnchorSwapFixer() {
        this(3, 30.0, true);
    }

    /**
     * Create with specified parameters.
     *
     * @param candidateCount Number of candidates per direction (total = 2x)
     * @param sectorAngle Angular offset between candidates
     * @param maintainDistance If true, keep same distance from start
     */
    public AnchorSwapFixer(int candidateCount, double sectorAngle, boolean maintainDistance) {
        this.candidateCount = Math.max(1, Math.min(5, candidateCount));
        this.sectorAngle = Math.max(15, Math.min(60, sectorAngle));
        this.maintainDistance = maintainDistance;
    }

    @Override
    public String getName() {
        return "anchor-swap";
    }

    @Override
    public String getDescription() {
        return "Move waypoint to different sector to escape road corridor";
    }

    @Override
    public Set<IssueType> handlesIssues() {
        return Set.of(
            IssueType.HIGH_ASPHALT_RATIO,
            IssueType.MAIN_ROAD_TRAP,
            IssueType.LOW_UNPAVED_RATIO,
            IssueType.LOW_QUALITY_LEG
        );
    }

    @Override
    public int priority() {
        return 20; // Try after KinkFixer
    }

    @Override
    public FixResult attemptFix(List<GHPoint> currentWaypoints,
                                 RouteScore score,
                                 int problemLegIndex,
                                 RoundTripProfile profile) {

        // Validate inputs
        if (currentWaypoints == null || currentWaypoints.size() < 3) {
            return FixResult.failure("Need at least 3 waypoints to swap anchor", getName());
        }

        // Determine which waypoint to move
        // Target the endpoint of the problem leg (or middle waypoint if no specific leg)
        int targetIdx;
        if (problemLegIndex >= 0 && problemLegIndex < currentWaypoints.size() - 2) {
            targetIdx = problemLegIndex + 1;
        } else {
            // Find middle waypoint
            targetIdx = currentWaypoints.size() / 2;
        }

        // Don't move first or last waypoint (they're the same - start/end)
        if (targetIdx <= 0 || targetIdx >= currentWaypoints.size() - 1) {
            return FixResult.failure("Cannot swap start/end waypoint", getName());
        }

        GHPoint start = currentWaypoints.get(0);
        GHPoint current = currentWaypoints.get(targetIdx);

        // Calculate current position relative to start
        double currentBearing = bearing(start, current);
        double currentDist = distance(start, current);

        // Generate candidates in different sectors
        List<GHPoint> candidates = new ArrayList<>();
        for (int i = 1; i <= candidateCount; i++) {
            // Clockwise candidate
            double cwBearing = normalizeBearing(currentBearing + i * sectorAngle);
            candidates.add(project(start, cwBearing, currentDist));

            // Counter-clockwise candidate
            double ccwBearing = normalizeBearing(currentBearing - i * sectorAngle);
            candidates.add(project(start, ccwBearing, currentDist));
        }

        if (candidates.isEmpty()) {
            return FixResult.failure("No swap candidates generated", getName());
        }

        // Select the first candidate (in full implementation, would score and pick best)
        GHPoint newPosition = candidates.get(0);
        double newBearing = bearing(start, newPosition);

        // Create modified waypoint list
        List<GHPoint> modified = new ArrayList<>(currentWaypoints);
        modified.set(targetIdx, newPosition);

        String description = String.format(
            "Swapped waypoint %d: bearing %.0f° → %.0f° (offset %.0f°)",
            targetIdx, currentBearing, newBearing, sectorAngle);

        logger.debug("AnchorSwapFixer: {}", description);

        return FixResult.success(modified, description, getName());
    }
}
