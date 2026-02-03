/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Result container for exploration round-trip routing.
 */
package com.graphhopper.trailmap.roundtrip.exploration;

import com.graphhopper.ResponsePath;
import com.graphhopper.trailmap.roundtrip.normalization.NormalizationResult;
import com.graphhopper.trailmap.roundtrip.scoring.RouteScore;
import com.graphhopper.util.shapes.GHPoint;

import java.util.Collections;
import java.util.List;

/**
 * Result of exploration round-trip routing.
 *
 * <p>Contains both the final route (using standard profile waypoints) and
 * metadata about the exploration and normalization process.
 */
public class ExplorationRoundTripResult {

    private final ResponsePath path;
    private final List<GHPoint> normalizedWaypoints;
    private final List<GHPoint> explorationWaypoints;
    private final double normalizationMatchPercentage;
    private final RouteScore score;
    private final boolean success;
    private final String failureReason;
    private final boolean includeMetrics;

    private ExplorationRoundTripResult(ResponsePath path, List<GHPoint> normalizedWaypoints,
                                        List<GHPoint> explorationWaypoints,
                                        double normalizationMatchPercentage, RouteScore score,
                                        boolean success, String failureReason, boolean includeMetrics) {
        this.path = path;
        this.normalizedWaypoints = normalizedWaypoints != null
            ? Collections.unmodifiableList(normalizedWaypoints)
            : Collections.emptyList();
        this.explorationWaypoints = explorationWaypoints != null
            ? Collections.unmodifiableList(explorationWaypoints)
            : Collections.emptyList();
        this.normalizationMatchPercentage = normalizationMatchPercentage;
        this.score = score;
        this.success = success;
        this.failureReason = failureReason;
        this.includeMetrics = includeMetrics;
    }

    /**
     * Create a successful result.
     *
     * @param path Final route path
     * @param normalizedWaypoints Minimal waypoints from normalization
     * @param explorationWaypoints Geometric waypoints used for exploration route
     * @param matchPercentage Edge match percentage from normalization
     * @param score Quality score of the route
     * @param includeMetrics Whether to include metrics in response
     * @return Successful result
     */
    public static ExplorationRoundTripResult success(ResponsePath path, List<GHPoint> normalizedWaypoints,
                                                      List<GHPoint> explorationWaypoints,
                                                      double matchPercentage, RouteScore score,
                                                      boolean includeMetrics) {
        return new ExplorationRoundTripResult(path, normalizedWaypoints, explorationWaypoints,
            matchPercentage, score, true, null, includeMetrics);
    }

    /**
     * Create a failed result.
     *
     * @param reason Failure reason
     * @return Failed result
     */
    public static ExplorationRoundTripResult failure(String reason) {
        return new ExplorationRoundTripResult(null, null, null, 0, null, false, reason, false);
    }

    /**
     * Create result from normalization result.
     *
     * @param path Final route path
     * @param normResult Normalization result
     * @param explorationWaypoints Geometric waypoints used for exploration route (after fix loop)
     * @param score Quality score
     * @param includeMetrics Whether to include metrics
     * @return Result based on normalization outcome
     */
    public static ExplorationRoundTripResult fromNormalization(ResponsePath path,
                                                                NormalizationResult normResult,
                                                                List<GHPoint> explorationWaypoints,
                                                                RouteScore score,
                                                                boolean includeMetrics) {
        if (normResult.isSuccess() || normResult.hasWaypoints()) {
            return new ExplorationRoundTripResult(path, normResult.getWaypoints(),
                explorationWaypoints,
                normResult.getMatchPercentage(), score,
                normResult.isSuccess(), normResult.getFailureReason(), includeMetrics);
        } else {
            return failure(normResult.getFailureReason());
        }
    }

    /**
     * Create a non-finalized result (without normalization).
     *
     * <p>Used when round_trip.finalize=false. Returns the exploration route
     * directly without running normalization. The snapped exploration waypoints
     * are returned as both the waypoints and exploration_waypoints.
     *
     * @param path Exploration route path
     * @param snappedExplorationWaypoints Snapped positions of exploration waypoints
     * @param score Quality score (optional)
     * @param includeMetrics Whether to include metrics
     * @return Non-finalized result with exploration route
     */
    public static ExplorationRoundTripResult nonFinalized(ResponsePath path,
                                                           List<GHPoint> snappedExplorationWaypoints,
                                                           RouteScore score,
                                                           boolean includeMetrics) {
        // For non-finalized: snapped waypoints go in both normalized and exploration slots
        // match percentage is 0 (not applicable)
        return new ExplorationRoundTripResult(path, snappedExplorationWaypoints,
            snappedExplorationWaypoints, 0, score, true, null, includeMetrics);
    }

    /**
     * Get the final route path.
     *
     * @return Route path or null if failed
     */
    public ResponsePath getPath() {
        return path;
    }

    /**
     * Get the normalized waypoints.
     *
     * <p>These waypoints, when routed with the standard profile, should
     * reproduce the exploration route.
     *
     * @return Unmodifiable list of waypoints
     */
    public List<GHPoint> getNormalizedWaypoints() {
        return normalizedWaypoints;
    }

    /**
     * Get the exploration waypoints.
     *
     * <p>These are the geometric shape vertices (e.g., circle corners) used
     * to generate the exploration route. They represent the final waypoints
     * after the fix loop completed.
     *
     * @return Unmodifiable list of exploration waypoints
     */
    public List<GHPoint> getExplorationWaypoints() {
        return explorationWaypoints;
    }

    /**
     * Get the edge match percentage from normalization.
     *
     * @return Match percentage (0-100)
     */
    public double getNormalizationMatchPercentage() {
        return normalizationMatchPercentage;
    }

    /**
     * Get the quality score of the route.
     *
     * @return Route score or null
     */
    public RouteScore getScore() {
        return score;
    }

    /**
     * Check if routing was successful.
     *
     * @return true if successful
     */
    public boolean isSuccess() {
        return success;
    }

    /**
     * Get the failure reason if not successful.
     *
     * @return Failure reason or null if successful
     */
    public String getFailureReason() {
        return failureReason;
    }

    /**
     * Check if metrics should be included in response.
     *
     * @return true if metrics should be included
     */
    public boolean shouldIncludeMetrics() {
        return includeMetrics;
    }

    @Override
    public String toString() {
        if (success) {
            return String.format("ExplorationResult[success, %d waypoints, %.1f%% match, %.1f score]",
                normalizedWaypoints.size(), normalizationMatchPercentage,
                score != null ? score.getOverallScore() : 0);
        } else {
            return String.format("ExplorationResult[failed: %s]", failureReason);
        }
    }
}
