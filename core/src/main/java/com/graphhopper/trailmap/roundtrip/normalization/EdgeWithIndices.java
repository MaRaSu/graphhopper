/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Edge with polyline indices for precise range extraction.
 */
package com.graphhopper.trailmap.roundtrip.normalization;

/**
 * Represents a graph edge with its polyline index range.
 *
 * <p>This class tracks where an edge falls within a route's polyline,
 * enabling precise extraction of edge subsets for binary search comparison.
 *
 * <p>Ported from TypeScript's StitchedEdgeId type in gh-route-convert.ts.
 */
public class EdgeWithIndices {

    private final int edgeId;
    private final int startPolylineIndex;
    private final int endPolylineIndex;

    /**
     * Create an EdgeWithIndices.
     *
     * @param edgeId Edge ID in the graph
     * @param startPolylineIndex Index in polyline where this edge STARTS
     * @param endPolylineIndex Index in polyline where this edge ENDS
     */
    public EdgeWithIndices(int edgeId, int startPolylineIndex, int endPolylineIndex) {
        this.edgeId = edgeId;
        this.startPolylineIndex = startPolylineIndex;
        this.endPolylineIndex = endPolylineIndex;
    }

    /**
     * Get the edge ID.
     *
     * @return Edge ID
     */
    public int getEdgeId() {
        return edgeId;
    }

    /**
     * Get the polyline index where this edge starts.
     *
     * @return Start index in polyline
     */
    public int getStartPolylineIndex() {
        return startPolylineIndex;
    }

    /**
     * Get the polyline index where this edge ends.
     *
     * @return End index in polyline
     */
    public int getEndPolylineIndex() {
        return endPolylineIndex;
    }

    @Override
    public String toString() {
        return String.format("Edge[%d: %d-%d]", edgeId, startPolylineIndex, endPolylineIndex);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        EdgeWithIndices that = (EdgeWithIndices) o;
        return edgeId == that.edgeId &&
               startPolylineIndex == that.startPolylineIndex &&
               endPolylineIndex == that.endPolylineIndex;
    }

    @Override
    public int hashCode() {
        int result = edgeId;
        result = 31 * result + startPolylineIndex;
        result = 31 * result + endPolylineIndex;
        return result;
    }
}
