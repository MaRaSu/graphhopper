package com.graphhopper.trailmap.convert;

import com.graphhopper.matching.EdgeMatch;
import com.graphhopper.util.shapes.GHPoint;

import java.util.List;

/**
 * One contiguous slice of the input track, classified as either matched (followable
 * by road network) or unmatched (gap to be represented as a coordinates segment).
 *
 * <p>Observation indices are into the original {@code track} array supplied in the
 * request; both bounds are inclusive.
 */
public sealed interface TrackRegion {

    int firstObservation();
    int lastObservation();

    /**
     * A contiguous run of observations matched to a contiguous run of edges.
     *
     * <p>The three parallel lists {@code matchedObsIndices}, {@code obsSnapPoints},
     * {@code obsEdgeIdxInSlice} have the same length and describe matched observations
     * in this region in order. Each entry is one matched observation:
     * <ul>
     *   <li>{@code matchedObsIndices[k]} — its index in the original observation array</li>
     *   <li>{@code obsSnapPoints[k]}     — its snapped point on the road network</li>
     *   <li>{@code obsEdgeIdxInSlice[k]} — its position in {@link #edgeMatches} (0 = first)</li>
     * </ul>
     */
    record Matched(
            int firstObservation,
            int lastObservation,
            List<Integer> matchedObsIndices,
            List<GHPoint> obsSnapPoints,
            int[] obsEdgeIdxInSlice,
            List<EdgeMatch> edgeMatches,
            /** Snapped start point on the road network (first matched obs in region). */
            GHPoint startSnap,
            /** Snapped end point on the road network (last matched obs in region). */
            GHPoint endSnap,
            /** Length of the matched path covered by this region (m). */
            double matchedLengthM
    ) implements TrackRegion {}

    /**
     * A contiguous run of observations that could not be matched well (snap distance
     * over threshold, or detour transition).
     */
    record Unmatched(
            int firstObservation,
            int lastObservation
    ) implements TrackRegion {}
}
