/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Result of enhanced round-trip route generation.
 */
package com.graphhopper.trailmap.roundtrip;

import com.graphhopper.ResponsePath;
import com.graphhopper.trailmap.roundtrip.scoring.RouteScore;

/**
 * Result of an enhanced round-trip route generation attempt.
 * Contains the route path (if successful) and quality score.
 */
public class EnhancedRoundTripResult {

    private final boolean success;
    private final ResponsePath path;
    private final RouteScore score;
    private final String failureReason;
    private final boolean includeMetrics;

    private EnhancedRoundTripResult(boolean success, ResponsePath path, RouteScore score,
                                     String failureReason, boolean includeMetrics) {
        this.success = success;
        this.path = path;
        this.score = score;
        this.failureReason = failureReason;
        this.includeMetrics = includeMetrics;
    }

    /**
     * Create a successful result.
     */
    public static EnhancedRoundTripResult success(ResponsePath path, RouteScore score) {
        return new EnhancedRoundTripResult(true, path, score, null, false);
    }

    /**
     * Create a successful result with metrics flag.
     */
    public static EnhancedRoundTripResult success(ResponsePath path, RouteScore score, boolean includeMetrics) {
        return new EnhancedRoundTripResult(true, path, score, null, includeMetrics);
    }

    /**
     * Create a failure result.
     */
    public static EnhancedRoundTripResult failure(String reason) {
        return new EnhancedRoundTripResult(false, null, null, reason, false);
    }

    /**
     * Create a failure result with partial score info.
     */
    public static EnhancedRoundTripResult failure(String reason, RouteScore partialScore) {
        return new EnhancedRoundTripResult(false, null, partialScore, reason, false);
    }

    // Getters

    public boolean isSuccess() {
        return success;
    }

    public ResponsePath getPath() {
        return path;
    }

    public RouteScore getScore() {
        return score;
    }

    public String getFailureReason() {
        return failureReason;
    }

    public boolean shouldIncludeMetrics() {
        return includeMetrics;
    }

    @Override
    public String toString() {
        if (success) {
            return String.format("EnhancedRoundTripResult{success=true, distance=%.0fm, score=%.1f}",
                path != null ? path.getDistance() : 0,
                score != null ? score.getOverallScore() : 0);
        } else {
            return String.format("EnhancedRoundTripResult{success=false, reason='%s'}", failureReason);
        }
    }
}
