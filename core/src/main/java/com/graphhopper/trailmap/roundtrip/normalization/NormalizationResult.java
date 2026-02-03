/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Result container for waypoint normalization.
 */
package com.graphhopper.trailmap.roundtrip.normalization;

import com.graphhopper.util.shapes.GHPoint;

import java.util.Collections;
import java.util.List;

/**
 * Result of waypoint normalization process.
 *
 * <p>Contains the normalized waypoints that can reproduce the exploration route
 * when routed with the standard profile, along with match quality metrics.
 */
public class NormalizationResult {

    private final List<GHPoint> waypoints;
    private final double matchPercentage;
    private final int totalEdges;
    private final int matchedEdges;
    private final boolean success;
    private final String failureReason;

    private NormalizationResult(List<GHPoint> waypoints, double matchPercentage,
                                 int totalEdges, int matchedEdges,
                                 boolean success, String failureReason) {
        this.waypoints = waypoints != null ? Collections.unmodifiableList(waypoints) : Collections.emptyList();
        this.matchPercentage = matchPercentage;
        this.totalEdges = totalEdges;
        this.matchedEdges = matchedEdges;
        this.success = success;
        this.failureReason = failureReason;
    }

    /**
     * Create a successful normalization result.
     *
     * @param waypoints Normalized waypoints
     * @param matchPercentage Percentage of edges that matched
     * @param totalEdges Total edges in reference route
     * @param matchedEdges Number of edges that matched
     * @return Successful result
     */
    public static NormalizationResult success(List<GHPoint> waypoints, double matchPercentage,
                                               int totalEdges, int matchedEdges) {
        return new NormalizationResult(waypoints, matchPercentage, totalEdges, matchedEdges, true, null);
    }

    /**
     * Create a failed normalization result.
     *
     * @param reason Failure reason
     * @return Failed result
     */
    public static NormalizationResult failure(String reason) {
        return new NormalizationResult(Collections.emptyList(), 0, 0, 0, false, reason);
    }

    /**
     * Create a best-effort result when match percentage is below threshold.
     *
     * @param waypoints Best waypoints found
     * @param matchPercentage Achieved match percentage
     * @param totalEdges Total edges in reference route
     * @param matchedEdges Number of edges that matched
     * @return Best-effort result (success=false but waypoints available)
     */
    public static NormalizationResult bestEffort(List<GHPoint> waypoints, double matchPercentage,
                                                  int totalEdges, int matchedEdges) {
        String reason = String.format("Match percentage %.1f%% below threshold %.1f%%",
            matchPercentage, NormalizationConstants.MIN_MATCH_PERCENTAGE);
        return new NormalizationResult(waypoints, matchPercentage, totalEdges, matchedEdges, false, reason);
    }

    /**
     * Get the normalized waypoints.
     *
     * @return Unmodifiable list of waypoints
     */
    public List<GHPoint> getWaypoints() {
        return waypoints;
    }

    /**
     * Get the edge match percentage.
     *
     * @return Percentage of reference edges that matched (0-100)
     */
    public double getMatchPercentage() {
        return matchPercentage;
    }

    /**
     * Get the total number of edges in the reference route.
     *
     * @return Total edge count
     */
    public int getTotalEdges() {
        return totalEdges;
    }

    /**
     * Get the number of edges that matched.
     *
     * @return Matched edge count
     */
    public int getMatchedEdges() {
        return matchedEdges;
    }

    /**
     * Check if normalization was successful.
     *
     * @return true if match percentage met threshold
     */
    public boolean isSuccess() {
        return success;
    }

    /**
     * Check if waypoints are available (even if not fully successful).
     *
     * @return true if waypoints list is not empty
     */
    public boolean hasWaypoints() {
        return !waypoints.isEmpty();
    }

    /**
     * Get the failure reason if not successful.
     *
     * @return Failure reason or null if successful
     */
    public String getFailureReason() {
        return failureReason;
    }

    @Override
    public String toString() {
        if (success) {
            return String.format("NormalizationResult[success, %d waypoints, %.1f%% match (%d/%d edges)]",
                waypoints.size(), matchPercentage, matchedEdges, totalEdges);
        } else if (hasWaypoints()) {
            return String.format("NormalizationResult[best-effort, %d waypoints, %.1f%% match, reason=%s]",
                waypoints.size(), matchPercentage, failureReason);
        } else {
            return String.format("NormalizationResult[failed, reason=%s]", failureReason);
        }
    }
}
