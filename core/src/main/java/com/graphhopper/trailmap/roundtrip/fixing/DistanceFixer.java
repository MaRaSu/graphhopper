/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * DistanceFixer - adjusts waypoints to hit target distance.
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
 * DistanceFixer adjusts waypoints to achieve target distance.
 *
 * Strategy:
 * - Too long: scale waypoints toward center OR remove worst-scoring waypoint
 * - Too short: scale waypoints outward OR add perpendicular waypoint
 *
 * This fixer works with the distance information stored in RouteScore
 * (targetDistance, actualDistance) and applies geometric transformations
 * to the waypoint configuration.
 *
 * Priority: 8 (after DeadEndFixer at 5, before KinkFixer at 10)
 * - Dead-ends should be fixed first since they often affect distance
 * - Once dead-ends are fixed, distance adjustments can be made
 */
public class DistanceFixer extends AbstractRouteFixer {

    private static final Logger logger = LoggerFactory.getLogger(DistanceFixer.class);

    /** Threshold for "severely" off-target (triggers aggressive scaling) */
    private static final double SEVERE_THRESHOLD = 0.30;

    /** Minimum waypoints to maintain */
    private static final int MIN_WAYPOINTS = 3;

    /** Maximum scale factor per iteration */
    private static final double MAX_SCALE_OUT = 1.4;
    private static final double MAX_SCALE_IN = 0.7;

    @Override
    public String getName() {
        return "distance";
    }

    @Override
    public String getDescription() {
        return "Adjust waypoints to achieve target distance";
    }

    @Override
    public Set<IssueType> handlesIssues() {
        return Set.of(IssueType.TOO_LONG, IssueType.TOO_SHORT);
    }

    @Override
    public int priority() {
        return 8; // After dead-end (5), before kink (10)
    }

    @Override
    public FixResult attemptFix(List<GHPoint> currentWaypoints,
                                 RouteScore score,
                                 int problemLegIndex,
                                 RoundTripProfile profile) {

        if (currentWaypoints == null || currentWaypoints.size() < MIN_WAYPOINTS) {
            return FixResult.failure("Need at least 3 waypoints", getName());
        }

        if (score == null) {
            return FixResult.failure("No scoring data available", getName());
        }

        double targetDistance = score.getTargetDistance();
        double actualDistance = score.getActualDistance();

        if (targetDistance <= 0) {
            return FixResult.failure("No target distance set", getName());
        }

        double ratio = actualDistance / targetDistance;
        double deviation = Math.abs(ratio - 1.0);

        logger.info("DistanceFixer: actual={}m, target={}m, ratio={} (deviation: {}%)",
            Math.round(actualDistance), Math.round(targetDistance),
            String.format("%.2f", ratio), String.format("%.0f", deviation * 100));

        // Check if within tolerance (shouldn't be called if within tolerance, but safety check)
        if (score.isDistanceWithinTolerance()) {
            return FixResult.failure("Distance already within tolerance", getName());
        }

        // Decide strategy based on direction and severity
        if (ratio > 1.0) {
            // Too long - contract
            if (deviation > SEVERE_THRESHOLD) {
                return aggressiveContraction(currentWaypoints, score, ratio);
            } else {
                return moderateContraction(currentWaypoints, score, ratio);
            }
        } else {
            // Too short - expand
            if (deviation > SEVERE_THRESHOLD) {
                return aggressiveExpansion(currentWaypoints, score, ratio);
            } else {
                return moderateExpansion(currentWaypoints, score, ratio);
            }
        }
    }

    /**
     * Aggressive contraction for severely over-length routes.
     * Strategy: Scale ALL waypoints toward center + optionally remove worst waypoint.
     */
    private FixResult aggressiveContraction(List<GHPoint> waypoints, RouteScore score, double ratio) {
        GHPoint center = calculateCenter(waypoints);

        // Scale factor: if ratio is 1.67 (67% over), we want to scale to ~0.6 of current size
        // But cap at MAX_SCALE_IN to avoid too aggressive changes
        double scaleFactor = Math.max(MAX_SCALE_IN, 1.0 / ratio);

        logger.info("Aggressive contraction: scaling waypoints by {} toward center",
            String.format("%.2f", scaleFactor));

        List<GHPoint> newWaypoints = scaleWaypointsFromCenter(waypoints, center, scaleFactor);

        // If we have enough waypoints and score data, also remove the worst one
        if (newWaypoints.size() > MIN_WAYPOINTS + 1 && score.getLegScores() != null
            && !score.getLegScores().isEmpty()) {

            int worstWpIdx = findWorstWaypointIndex(newWaypoints, score);
            if (worstWpIdx > 0 && worstWpIdx < newWaypoints.size() - 1) {
                GHPoint removed = newWaypoints.remove(worstWpIdx);
                logger.info("Also removed worst-scoring waypoint {} at ({}, {})",
                    worstWpIdx, String.format("%.4f", removed.getLat()),
                    String.format("%.4f", removed.getLon()));
                return FixResult.success(newWaypoints,
                    String.format("Scaled by %.0f%% + removed waypoint %d", (1-scaleFactor)*100, worstWpIdx),
                    getName());
            }
        }

        return FixResult.success(newWaypoints,
            String.format("Scaled all waypoints by %.0f%% toward center", (1-scaleFactor)*100),
            getName());
    }

    /**
     * Moderate contraction for slightly over-length routes.
     * Strategy: Remove the worst-scoring intermediate waypoint OR scale slightly.
     */
    private FixResult moderateContraction(List<GHPoint> waypoints, RouteScore score, double ratio) {
        // Try removing worst waypoint first
        if (waypoints.size() > MIN_WAYPOINTS && score.getLegScores() != null
            && !score.getLegScores().isEmpty()) {

            int worstWpIdx = findWorstWaypointIndex(waypoints, score);
            if (worstWpIdx > 0 && worstWpIdx < waypoints.size() - 1) {
                List<GHPoint> newWaypoints = new ArrayList<>(waypoints);
                GHPoint removed = newWaypoints.remove(worstWpIdx);

                logger.info("Moderate contraction: removed waypoint {} at ({}, {})",
                    worstWpIdx, String.format("%.4f", removed.getLat()),
                    String.format("%.4f", removed.getLon()));

                return FixResult.success(newWaypoints,
                    "Removed worst-scoring waypoint " + worstWpIdx,
                    getName());
            }
        }

        // Fallback to scaling
        GHPoint center = calculateCenter(waypoints);
        double scaleFactor = 0.90; // 10% contraction

        List<GHPoint> newWaypoints = scaleWaypointsFromCenter(waypoints, center, scaleFactor);

        return FixResult.success(newWaypoints,
            "Scaled waypoints by 10% toward center",
            getName());
    }

    /**
     * Aggressive expansion for severely under-length routes.
     * Strategy: Scale ALL waypoints outward from center.
     */
    private FixResult aggressiveExpansion(List<GHPoint> waypoints, RouteScore score, double ratio) {
        GHPoint center = calculateCenter(waypoints);

        // Scale factor: if ratio is 0.5 (50% under), we want to scale to ~2x
        // But cap at MAX_SCALE_OUT
        double scaleFactor = Math.min(MAX_SCALE_OUT, 1.0 / ratio);

        logger.info("Aggressive expansion: scaling waypoints by {} from center",
            String.format("%.2f", scaleFactor));

        List<GHPoint> newWaypoints = scaleWaypointsFromCenter(waypoints, center, scaleFactor);

        return FixResult.success(newWaypoints,
            String.format("Scaled all waypoints by %.0f%% outward", (scaleFactor-1)*100),
            getName());
    }

    /**
     * Moderate expansion for slightly under-length routes.
     * Strategy: Add a waypoint perpendicular to the best-scoring leg.
     */
    private FixResult moderateExpansion(List<GHPoint> waypoints, RouteScore score, double ratio) {
        // Find best leg to expand
        int bestLegIdx = findBestLegIndex(waypoints, score);
        if (bestLegIdx < 0 || bestLegIdx >= waypoints.size() - 2) {
            // Fallback to scaling
            GHPoint center = calculateCenter(waypoints);
            List<GHPoint> newWaypoints = scaleWaypointsFromCenter(waypoints, center, 1.15);
            return FixResult.success(newWaypoints, "Scaled waypoints by 15% outward", getName());
        }

        GHPoint wpStart = waypoints.get(bestLegIdx);
        GHPoint wpEnd = waypoints.get(bestLegIdx + 1);

        // Calculate perpendicular point
        double midLat = (wpStart.getLat() + wpEnd.getLat()) / 2.0;
        double midLon = (wpStart.getLon() + wpEnd.getLon()) / 2.0;

        double bearing = bearing(wpStart, wpEnd);
        double perpBearing = bearing + 90;

        // Expansion distance based on how much we're under
        double deficitRatio = 1.0 - ratio;
        double targetDistance = score.getTargetDistance();
        double expansionDist = targetDistance * deficitRatio * 0.3; // 30% of deficit

        GHPoint newPoint = project(new GHPoint(midLat, midLon), perpBearing, expansionDist);

        List<GHPoint> newWaypoints = new ArrayList<>(waypoints);
        newWaypoints.add(bestLegIdx + 1, newPoint);

        logger.info("Moderate expansion: added waypoint at ({}, {}) perpendicular to leg {}",
            String.format("%.4f", newPoint.getLat()), String.format("%.4f", newPoint.getLon()), bestLegIdx);

        return FixResult.success(newWaypoints,
            "Added waypoint perpendicular to leg " + bestLegIdx,
            getName());
    }

    /**
     * Scale all intermediate waypoints from center.
     */
    private List<GHPoint> scaleWaypointsFromCenter(List<GHPoint> waypoints, GHPoint center, double scaleFactor) {
        List<GHPoint> newWaypoints = new ArrayList<>();
        GHPoint start = waypoints.get(0);
        newWaypoints.add(start); // Keep start fixed

        for (int i = 1; i < waypoints.size() - 1; i++) {
            GHPoint wp = waypoints.get(i);
            double newLat = center.getLat() + (wp.getLat() - center.getLat()) * scaleFactor;
            double newLon = center.getLon() + (wp.getLon() - center.getLon()) * scaleFactor;
            newWaypoints.add(new GHPoint(newLat, newLon));
        }

        newWaypoints.add(start); // Close loop (end = start)
        return newWaypoints;
    }

    /**
     * Calculate geometric center of waypoints.
     */
    private GHPoint calculateCenter(List<GHPoint> waypoints) {
        double sumLat = 0, sumLon = 0;
        int count = 0;

        // Exclude last point if it's same as first (closed loop)
        int endIdx = waypoints.size();
        if (waypoints.size() > 1 &&
            waypoints.get(0).getLat() == waypoints.get(waypoints.size()-1).getLat() &&
            waypoints.get(0).getLon() == waypoints.get(waypoints.size()-1).getLon()) {
            endIdx = waypoints.size() - 1;
        }

        for (int i = 0; i < endIdx; i++) {
            sumLat += waypoints.get(i).getLat();
            sumLon += waypoints.get(i).getLon();
            count++;
        }

        return new GHPoint(sumLat / count, sumLon / count);
    }

    /**
     * Find index of worst-scoring waypoint (based on adjacent leg scores).
     */
    private int findWorstWaypointIndex(List<GHPoint> waypoints, RouteScore score) {
        if (score.getLegScores() == null || score.getLegScores().isEmpty()) {
            return waypoints.size() / 2;
        }

        List<LegScore> legs = score.getLegScores();

        int worstIdx = -1;
        double worstScore = Double.MAX_VALUE;

        for (int wpIdx = 1; wpIdx < waypoints.size() - 1; wpIdx++) {
            double avgScore = 0;
            int legCount = 0;

            int legBefore = wpIdx - 1;
            int legAfter = wpIdx;

            if (legBefore >= 0 && legBefore < legs.size()) {
                avgScore += legs.get(legBefore).getScore();
                legCount++;
            }
            if (legAfter >= 0 && legAfter < legs.size()) {
                avgScore += legs.get(legAfter).getScore();
                legCount++;
            }

            if (legCount > 0) {
                avgScore /= legCount;
                if (avgScore < worstScore) {
                    worstScore = avgScore;
                    worstIdx = wpIdx;
                }
            }
        }

        return worstIdx > 0 ? worstIdx : waypoints.size() / 2;
    }

    /**
     * Find index of best-scoring leg.
     */
    private int findBestLegIndex(List<GHPoint> waypoints, RouteScore score) {
        if (score.getLegScores() == null || score.getLegScores().isEmpty()) {
            return waypoints.size() / 2 - 1;
        }

        List<LegScore> legs = score.getLegScores();
        int bestIdx = 0;
        double bestScore = Double.MIN_VALUE;

        for (int i = 0; i < legs.size() && i < waypoints.size() - 2; i++) {
            if (legs.get(i).getScore() > bestScore) {
                bestScore = legs.get(i).getScore();
                bestIdx = i;
            }
        }

        return bestIdx;
    }
}
