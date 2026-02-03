/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Edge sequence comparison for waypoint normalization.
 */
package com.graphhopper.trailmap.roundtrip.normalization;

import com.carrotsearch.hppc.IntArrayList;
import com.graphhopper.routing.Path;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.PointList;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

/**
 * Compares edge sequences to determine if two routes follow the same path.
 *
 * <p>This class handles the complexity of edge comparison including:
 * <ul>
 *   <li>Virtual edge tolerance - short connector edges at via-points</li>
 *   <li>Consecutive duplicate removal</li>
 *   <li>Match percentage calculation</li>
 * </ul>
 *
 * <p>Ported from gh-route-convert.ts routeEdgesMatch() function.
 */
public class EdgeMatcher {

    private static final Logger logger = LoggerFactory.getLogger(EdgeMatcher.class);

    private final Graph graph;

    /**
     * Create an EdgeMatcher for the given graph.
     *
     * @param graph The graph to use for edge length calculations
     */
    public EdgeMatcher(Graph graph) {
        this.graph = graph;
    }

    /**
     * Compare two edge sequences for match, tolerating virtual edges.
     *
     * <p>Virtual edges are short connector edges (less than {@link NormalizationConstants#VIRTUAL_EDGE_MAX_LENGTH_M})
     * that GraphHopper inserts at via-points. When a waypoint is skipped, these edges don't appear,
     * causing false comparison failures.
     *
     * <p>Algorithm: Two-pointer traversal that skips virtual edges on mismatch.
     * <ul>
     *   <li>If edges match at current positions → advance both pointers</li>
     *   <li>If mismatch and ref edge is virtual → skip ref edge</li>
     *   <li>If mismatch and test edge is virtual → skip test edge</li>
     *   <li>If mismatch and both are significant → return false</li>
     * </ul>
     *
     * @param refEdges Reference edge ID sequence
     * @param testEdges Test edge ID sequence
     * @param refPath Reference path (for edge length calculation)
     * @param testPath Test path (for edge length calculation)
     * @return true if edge sequences match (with virtual edge tolerance)
     */
    public boolean edgesMatch(IntArrayList refEdges, IntArrayList testEdges,
                              Path refPath, Path testPath) {
        if (refEdges == null || testEdges == null || refEdges.isEmpty() || testEdges.isEmpty()) {
            return false;
        }

        // Deduplicate consecutive edges
        List<Integer> refDedup = deduplicateConsecutive(refEdges);
        List<Integer> testDedup = deduplicateConsecutive(testEdges);

        if (refDedup.isEmpty() || testDedup.isEmpty()) {
            return false;
        }

        int refIdx = 0;
        int testIdx = 0;

        while (refIdx < refDedup.size() && testIdx < testDedup.size()) {
            int refEdge = refDedup.get(refIdx);
            int testEdge = testDedup.get(testIdx);

            if (refEdge == testEdge) {
                // Match - advance both pointers
                refIdx++;
                testIdx++;
            } else if (isVirtualEdge(refEdge)) {
                // Ref edge is virtual - skip it
                refIdx++;
            } else if (isVirtualEdge(testEdge)) {
                // Test edge is virtual - skip it
                testIdx++;
            } else {
                // Both edges are significant and don't match - real mismatch
                logger.warn("DEBUG Edge mismatch at ref[{}]={} vs test[{}]={} (after {} matching edges)",
                    refIdx, refEdge, testIdx, testEdge, Math.min(refIdx, testIdx));
                return false;
            }
        }

        // Check remaining edges in ref are all virtual
        while (refIdx < refDedup.size()) {
            if (!isVirtualEdge(refDedup.get(refIdx))) {
                logger.warn("DEBUG Remaining significant edge in ref at [{}]={} (ref has {} total, test consumed {})",
                    refIdx, refDedup.get(refIdx), refDedup.size(), testIdx);
                return false;
            }
            refIdx++;
        }

        // Check remaining edges in test are all virtual
        while (testIdx < testDedup.size()) {
            if (!isVirtualEdge(testDedup.get(testIdx))) {
                logger.warn("DEBUG Remaining significant edge in test at [{}]={} (test has {} total, ref consumed {})",
                    testIdx, testDedup.get(testIdx), testDedup.size(), refIdx);
                return false;
            }
            testIdx++;
        }

        return true;
    }

    /**
     * Compare two edge sequences with polyline indices for match, tolerating virtual edges.
     *
     * <p>This version uses EdgeWithIndices which provides polyline positions for each edge,
     * enabling more accurate virtual edge detection using actual edge geometry.
     *
     * <p>Algorithm mirrors TypeScript's routeEdgesMatch():
     * <ul>
     *   <li>If edges match at current positions → advance both pointers</li>
     *   <li>If mismatch and ref edge is virtual → skip ref edge</li>
     *   <li>If mismatch and test edge is virtual → skip test edge</li>
     *   <li>If mismatch and both are significant → return false</li>
     * </ul>
     *
     * @param refEdges Reference edge list with polyline indices
     * @param testEdges Test edge list with polyline indices
     * @param refPolyline Reference route polyline (for edge length calculation)
     * @param testPolyline Test route polyline (for edge length calculation)
     * @return true if edge sequences match (with virtual edge tolerance)
     */
    public boolean edgesMatchWithIndices(List<EdgeWithIndices> refEdges, List<EdgeWithIndices> testEdges,
                                          PointList refPolyline, PointList testPolyline) {
        if (refEdges == null || testEdges == null || refEdges.isEmpty() || testEdges.isEmpty()) {
            return false;
        }

        // Deduplicate consecutive edges
        List<EdgeWithIndices> refDedup = deduplicateConsecutiveWithIndices(refEdges);
        List<EdgeWithIndices> testDedup = deduplicateConsecutiveWithIndices(testEdges);

        if (refDedup.isEmpty() || testDedup.isEmpty()) {
            return false;
        }

        int refIdx = 0;
        int testIdx = 0;

        while (refIdx < refDedup.size() && testIdx < testDedup.size()) {
            EdgeWithIndices refEdge = refDedup.get(refIdx);
            EdgeWithIndices testEdge = testDedup.get(testIdx);

            if (refEdge.getEdgeId() == testEdge.getEdgeId()) {
                // Match - advance both pointers
                refIdx++;
                testIdx++;
            } else if (isVirtualEdgeFromPolyline(refEdge, refPolyline)) {
                // Ref edge is virtual - skip it
                refIdx++;
            } else if (isVirtualEdgeFromPolyline(testEdge, testPolyline)) {
                // Test edge is virtual - skip it
                testIdx++;
            } else {
                // Both edges are significant and don't match - real mismatch
                logger.debug("Edge mismatch at ref[{}]={} vs test[{}]={}",
                    refIdx, refEdge.getEdgeId(), testIdx, testEdge.getEdgeId());
                return false;
            }
        }

        // Check remaining edges in ref are all virtual
        while (refIdx < refDedup.size()) {
            if (!isVirtualEdgeFromPolyline(refDedup.get(refIdx), refPolyline)) {
                logger.debug("Remaining significant edge in ref at [{}]={}",
                    refIdx, refDedup.get(refIdx).getEdgeId());
                return false;
            }
            refIdx++;
        }

        // Check remaining edges in test are all virtual
        while (testIdx < testDedup.size()) {
            if (!isVirtualEdgeFromPolyline(testDedup.get(testIdx), testPolyline)) {
                logger.debug("Remaining significant edge in test at [{}]={}",
                    testIdx, testDedup.get(testIdx).getEdgeId());
                return false;
            }
            testIdx++;
        }

        return true;
    }

    /**
     * Check if an edge is virtual based on its polyline extent.
     *
     * <p>Uses the polyline coordinates to calculate actual edge length,
     * which is more accurate than the graph edge distance for connector edges.
     *
     * @param edge Edge with polyline indices
     * @param polyline Route polyline
     * @return true if edge length is below virtual edge threshold
     */
    private boolean isVirtualEdgeFromPolyline(EdgeWithIndices edge, PointList polyline) {
        if (polyline == null || polyline.isEmpty()) {
            // Fallback to graph-based check
            return isVirtualEdge(edge.getEdgeId());
        }

        int startIdx = edge.getStartPolylineIndex();
        int endIdx = edge.getEndPolylineIndex();

        if (startIdx < 0 || endIdx >= polyline.size() || startIdx > endIdx) {
            return isVirtualEdge(edge.getEdgeId());
        }

        // Calculate edge length from polyline
        double length = 0;
        for (int i = startIdx; i < endIdx && i < polyline.size() - 1; i++) {
            length += distanceBetweenPoints(
                polyline.getLat(i), polyline.getLon(i),
                polyline.getLat(i + 1), polyline.getLon(i + 1));
        }

        return length < NormalizationConstants.VIRTUAL_EDGE_MAX_LENGTH_M;
    }

    /**
     * Calculate distance between two lat/lon points in meters.
     */
    private double distanceBetweenPoints(double lat1, double lon1, double lat2, double lon2) {
        double R = 6371000; // Earth radius in meters
        double dLat = Math.toRadians(lat2 - lat1);
        double dLon = Math.toRadians(lon2 - lon1);
        double a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
                   Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2)) *
                   Math.sin(dLon / 2) * Math.sin(dLon / 2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        return R * c;
    }

    /**
     * Remove consecutive duplicate edges from EdgeWithIndices list.
     */
    private List<EdgeWithIndices> deduplicateConsecutiveWithIndices(List<EdgeWithIndices> edges) {
        List<EdgeWithIndices> result = new ArrayList<>();
        int lastEdgeId = Integer.MIN_VALUE;

        for (EdgeWithIndices edge : edges) {
            if (edge.getEdgeId() != lastEdgeId) {
                result.add(edge);
                lastEdgeId = edge.getEdgeId();
            }
        }

        return result;
    }

    /**
     * Calculate match statistics between two edge sequences.
     *
     * @param refEdges Reference edge ID sequence
     * @param testEdges Test edge ID sequence
     * @return Match statistics
     */
    public MatchStats calculateMatchStats(IntArrayList refEdges, IntArrayList testEdges) {
        if (refEdges == null || testEdges == null || refEdges.isEmpty()) {
            return new MatchStats(0, 0, 0);
        }

        List<Integer> refDedup = deduplicateConsecutive(refEdges);
        List<Integer> testDedup = deduplicateConsecutive(testEdges);

        int matched = 0;
        int testIdx = 0;

        // Count how many ref edges appear in test (in order)
        for (int refEdge : refDedup) {
            // Skip virtual edges in counting
            if (isVirtualEdge(refEdge)) {
                continue;
            }

            // Find this edge in test sequence
            while (testIdx < testDedup.size()) {
                int testEdge = testDedup.get(testIdx);
                if (testEdge == refEdge) {
                    matched++;
                    testIdx++;
                    break;
                } else if (isVirtualEdge(testEdge)) {
                    testIdx++;
                } else {
                    // Different significant edge - might still find ref edge later
                    testIdx++;
                }
            }
        }

        // Count significant edges in ref
        int significantRefEdges = 0;
        for (int edge : refDedup) {
            if (!isVirtualEdge(edge)) {
                significantRefEdges++;
            }
        }

        double percentage = significantRefEdges > 0
            ? (matched * 100.0 / significantRefEdges)
            : 0;

        return new MatchStats(matched, significantRefEdges, percentage);
    }

    /**
     * Check if an edge is a virtual/connector edge.
     *
     * @param edgeId Edge ID to check
     * @return true if edge length is below virtual edge threshold
     */
    private boolean isVirtualEdge(int edgeId) {
        try {
            EdgeIteratorState edge = graph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            if (edge == null) {
                return false;
            }
            return edge.getDistance() < NormalizationConstants.VIRTUAL_EDGE_MAX_LENGTH_M;
        } catch (Exception e) {
            // If we can't determine, assume not virtual
            return false;
        }
    }

    /**
     * Remove consecutive duplicate edge IDs.
     *
     * @param edges Original edge list
     * @return List with consecutive duplicates removed
     */
    private List<Integer> deduplicateConsecutive(IntArrayList edges) {
        List<Integer> result = new ArrayList<>();
        int lastEdge = Integer.MIN_VALUE;

        for (int i = 0; i < edges.size(); i++) {
            int edge = edges.get(i);
            if (edge != lastEdge) {
                result.add(edge);
                lastEdge = edge;
            }
        }

        return result;
    }

    /**
     * Statistics about edge sequence matching.
     */
    public static class MatchStats {
        private final int matchedEdges;
        private final int totalEdges;
        private final double matchPercentage;

        public MatchStats(int matchedEdges, int totalEdges, double matchPercentage) {
            this.matchedEdges = matchedEdges;
            this.totalEdges = totalEdges;
            this.matchPercentage = matchPercentage;
        }

        public int getMatchedEdges() {
            return matchedEdges;
        }

        public int getTotalEdges() {
            return totalEdges;
        }

        public double getMatchPercentage() {
            return matchPercentage;
        }

        public boolean meetsThreshold() {
            return matchPercentage >= NormalizationConstants.MIN_MATCH_PERCENTAGE;
        }

        @Override
        public String toString() {
            return String.format("MatchStats[%d/%d edges, %.1f%%]",
                matchedEdges, totalEdges, matchPercentage);
        }
    }
}
