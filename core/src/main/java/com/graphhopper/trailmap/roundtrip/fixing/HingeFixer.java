/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * HingeFixer - rotates the entire route around the start point.
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
 * HingeFixer rotates the entire route around the start point.
 *
 * Strategy:
 * 1. Keep start point fixed
 * 2. Find the "tip" (furthest point from start)
 * 3. Rotate the tip by a fixed angle around the start
 * 4. Proportionally adjust all intermediate waypoints
 *
 * This swings the entire loop to a new geographic sector,
 * completely bypassing blocked corridors.
 *
 * Use cases:
 * - Route failed due to blocked corridor (lake, mountain)
 * - Route has huge detour around obstacle
 * - Need to completely change route direction
 */
public class HingeFixer extends AbstractRouteFixer {

    private static final Logger logger = LoggerFactory.getLogger(HingeFixer.class);

    /** Rotation angle in degrees */
    private final double rotationAngle;

    /** Whether to try opposite direction if first rotation fails */
    private final boolean tryOpposite;

    /**
     * Create with default rotation angle of 45°.
     */
    public HingeFixer() {
        this(45.0, true);
    }

    /**
     * Create with specified parameters.
     *
     * @param rotationAngle Rotation angle in degrees (15-90)
     * @param tryOpposite If true, try -angle if +angle doesn't improve
     */
    public HingeFixer(double rotationAngle, boolean tryOpposite) {
        this.rotationAngle = Math.max(15, Math.min(90, rotationAngle));
        this.tryOpposite = tryOpposite;
    }

    @Override
    public String getName() {
        return "hinge";
    }

    @Override
    public String getDescription() {
        return "Rotate entire route around start to bypass obstacles";
    }

    @Override
    public Set<IssueType> handlesIssues() {
        return Set.of(
            IssueType.ROUTE_FAILED,
            IssueType.HUGE_DETOUR,
            IssueType.BLOCKED_CORRIDOR,
            IssueType.HIGH_REPETITION
        );
    }

    @Override
    public int priority() {
        return 30; // Last resort - most disruptive
    }

    @Override
    public FixResult attemptFix(List<GHPoint> currentWaypoints,
                                 RouteScore score,
                                 int problemLegIndex,
                                 RoundTripProfile profile) {

        // Validate inputs
        if (currentWaypoints == null || currentWaypoints.size() < 3) {
            return FixResult.failure("Need at least 3 waypoints to hinge", getName());
        }

        GHPoint start = currentWaypoints.get(0);

        // Find the tip (furthest point from start)
        int tipIndex = findFurthestFromStart(currentWaypoints);
        if (tipIndex <= 0 || tipIndex >= currentWaypoints.size() - 1) {
            return FixResult.failure("Could not find valid tip point", getName());
        }

        GHPoint tip = currentWaypoints.get(tipIndex);

        // Calculate current tip bearing and distance
        double currentBearing = bearing(start, tip);
        double tipDist = distance(start, tip);

        // Calculate new tip position (rotated)
        double newBearing = normalizeBearing(currentBearing + rotationAngle);
        GHPoint newTip = project(start, newBearing, tipDist);

        // Regenerate all waypoints proportionally
        List<GHPoint> modified = new ArrayList<>(currentWaypoints.size());
        modified.add(start);

        // Waypoints before tip: scale linearly from start to new tip
        for (int i = 1; i < tipIndex; i++) {
            GHPoint original = currentWaypoints.get(i);
            double originalBearing = bearing(start, original);
            double originalDist = distance(start, original);

            // Rotate bearing by same angle
            double rotatedBearing = normalizeBearing(originalBearing + rotationAngle);
            GHPoint rotated = project(start, rotatedBearing, originalDist);
            modified.add(rotated);
        }

        // Add new tip
        modified.add(newTip);

        // Waypoints after tip: scale from new tip back to start
        for (int i = tipIndex + 1; i < currentWaypoints.size() - 1; i++) {
            GHPoint original = currentWaypoints.get(i);
            double originalBearing = bearing(start, original);
            double originalDist = distance(start, original);

            // Rotate bearing
            double rotatedBearing = normalizeBearing(originalBearing + rotationAngle);
            GHPoint rotated = project(start, rotatedBearing, originalDist);
            modified.add(rotated);
        }

        // Close the loop back to start
        modified.add(start);

        String description = String.format(
            "Rotated route %.0f° around start (tip moved from %.0f° to %.0f°)",
            rotationAngle, currentBearing, newBearing);

        logger.debug("HingeFixer: {}", description);

        return FixResult.success(modified, description, getName());
    }
}
