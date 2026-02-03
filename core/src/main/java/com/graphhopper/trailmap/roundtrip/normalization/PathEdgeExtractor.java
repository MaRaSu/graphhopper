/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Extracts original (base graph) edge IDs from Path objects.
 */
package com.graphhopper.trailmap.roundtrip.normalization;

import com.carrotsearch.hppc.IntArrayList;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.querygraph.VirtualEdgeIteratorState;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.FetchMode;
import com.graphhopper.util.GHUtility;
import com.graphhopper.util.PointList;

import java.util.ArrayList;
import java.util.List;

/**
 * Extracts original edge IDs from Path objects, correctly handling virtual edges.
 *
 * <p>When routing through a QueryGraph (which is used when there are via-points/snaps),
 * the edges returned by Path.getEdges() are QueryGraph edge IDs. These include "virtual"
 * edges that connect snap points to the real graph. Virtual edge IDs are per-QueryGraph
 * and cannot be compared across different QueryGraphs.
 *
 * <p>This class extracts the original base graph edge IDs by:
 * <ul>
 *   <li>For regular edges: using edge.getEdge()</li>
 *   <li>For virtual edges: unwrapping via VirtualEdgeIteratorState.getOriginalEdgeKey()</li>
 * </ul>
 *
 * <p>This mirrors the logic in {@link com.graphhopper.util.details.EdgeIdDetails}.
 */
public class PathEdgeExtractor {

    /**
     * Extract original (base graph) edge IDs from a Path.
     *
     * <p>This correctly handles virtual edges that appear when routing through QueryGraphs.
     * The returned edge IDs are base graph IDs that can be compared across different routes.
     *
     * @param path The path to extract edges from
     * @return List of original edge IDs, or empty list if path is null/empty
     */
    public static IntArrayList extractOriginalEdgeIds(Path path) {
        IntArrayList result = new IntArrayList();
        if (path == null || path.getEdges().isEmpty()) {
            return result;
        }

        path.forEveryEdge(new Path.EdgeVisitor() {
            @Override
            public void next(EdgeIteratorState edge, int index, int prevEdgeId) {
                result.add(getOriginalEdgeId(edge));
            }

            @Override
            public void finish() {
                // Nothing to do
            }
        });

        return result;
    }

    /**
     * Extract original edge IDs with their polyline index ranges.
     *
     * <p>This is useful for binary search algorithms that need to map
     * edge positions to path geometry positions.
     *
     * @param path The path to extract edges from
     * @return List of EdgeWithIndices containing edge ID and polyline position
     */
    public static List<EdgeWithIndices> extractEdgesWithIndices(Path path) {
        List<EdgeWithIndices> result = new ArrayList<>();
        if (path == null || path.getEdges().isEmpty()) {
            return result;
        }

        // Track polyline index as we iterate
        // Start at 1 because Path.calcPoints() adds the start node at index 0 before iterating edges
        int[] polylineIdx = {1};

        path.forEveryEdge(new Path.EdgeVisitor() {
            @Override
            public void next(EdgeIteratorState edge, int index, int prevEdgeId) {
                int edgeId = getOriginalEdgeId(edge);
                int startIdx = polylineIdx[0];

                // Get the number of points this edge contributes
                PointList edgeGeometry = edge.fetchWayGeometry(FetchMode.PILLAR_AND_ADJ);
                int endIdx = startIdx + edgeGeometry.size() - 1;

                result.add(new EdgeWithIndices(edgeId, startIdx, endIdx));
                polylineIdx[0] = startIdx + edgeGeometry.size();  // Advance to next position
            }

            @Override
            public void finish() {
                // Nothing to do
            }
        });

        return result;
    }

    /**
     * Get the original (base graph) edge ID from an EdgeIteratorState.
     *
     * <p>For virtual edges, this unwraps to get the underlying real edge.
     * This is the same logic used by GraphHopper's EdgeIdDetails.
     *
     * @param edge The edge to get the ID from
     * @return The original base graph edge ID
     */
    public static int getOriginalEdgeId(EdgeIteratorState edge) {
        if (edge instanceof VirtualEdgeIteratorState) {
            return GHUtility.getEdgeFromEdgeKey(((VirtualEdgeIteratorState) edge).getOriginalEdgeKey());
        } else {
            return edge.getEdge();
        }
    }
}
