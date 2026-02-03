/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Waypoint normalization using binary search to INSERT via-points where needed.
 */
package com.graphhopper.trailmap.roundtrip.normalization;

import com.carrotsearch.hppc.IntArrayList;
import com.graphhopper.routing.EdgeRestrictions;
import com.graphhopper.routing.FlexiblePathCalculator;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.util.PointList;
import com.graphhopper.util.shapes.GHPoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

/**
 * Normalizes exploration routes into waypoints that reproduce the same path
 * when routed with the standard profile.
 *
 * <p>Algorithm:
 * <ol>
 *   <li>Start with original waypoints A and B</li>
 *   <li>Route A→B with standard profile, compare edges to exploration route</li>
 *   <li>If edges match: no via-points needed</li>
 *   <li>If edges differ: binary search to find farthest M where standard(A→M) matches exploration(A→M)</li>
 *   <li>Add M as waypoint, continue from M toward B</li>
 * </ol>
 *
 * <p>This class uses internal GraphHopper APIs (FlexiblePathCalculator, Path) and correctly
 * handles virtual edge unwrapping via {@link PathEdgeExtractor}.
 */
public class WaypointNormalizer {

    private static final Logger logger = LoggerFactory.getLogger(WaypointNormalizer.class);

    /**
     * Number of edges beyond the candidate point to place the anchor.
     * The anchor constrains the router to approach the candidate from the correct direction.
     */
    private static final int ANCHOR_OFFSET_EDGES = 10;

    private final BaseGraph graph;
    private final LocationIndex locationIndex;
    private final EdgeFilter edgeFilter;
    private final EdgeMatcher edgeMatcher;

    /**
     * Create a WaypointNormalizer.
     *
     * @param graph Base graph for edge lookups
     * @param locationIndex Location index for snapping points
     * @param edgeFilter Edge filter for snapping
     */
    public WaypointNormalizer(BaseGraph graph, LocationIndex locationIndex, EdgeFilter edgeFilter) {
        this.graph = graph;
        this.locationIndex = locationIndex;
        this.edgeFilter = edgeFilter;
        this.edgeMatcher = new EdgeMatcher(graph);
    }

    /**
     * Normalize exploration routes to minimal waypoints for standard profile routing.
     *
     * <p>Processes each leg independently and combines the results.
     *
     * @param originalWaypoints Original waypoints (start, end, or multiple for round-trip)
     * @param explorationPaths Exploration paths (one per leg between consecutive waypoints)
     * @param standardPathCalculatorFactory Factory to create path calculator for standard profile
     * @return Normalization result with waypoints that reproduce the exploration routes
     */
    public NormalizationResult normalize(List<GHPoint> originalWaypoints,
                                          List<Path> explorationPaths,
                                          Function<List<Snap>, FlexiblePathCalculator> standardPathCalculatorFactory) {
        if (originalWaypoints == null || originalWaypoints.size() < 2) {
            return NormalizationResult.failure("Need at least 2 waypoints");
        }

        if (explorationPaths == null || explorationPaths.isEmpty()) {
            return NormalizationResult.failure("No exploration paths provided");
        }

        if (explorationPaths.size() != originalWaypoints.size() - 1) {
            return NormalizationResult.failure("Path count doesn't match waypoint count: " +
                explorationPaths.size() + " paths for " + originalWaypoints.size() + " waypoints");
        }

        logger.info("Normalizing {} legs with {} total waypoints",
            explorationPaths.size(), originalWaypoints.size());

        List<GHPoint> allWaypoints = new ArrayList<>();
        int totalExplorationEdges = 0;
        int totalMatchedEdges = 0;

        // Process each leg
        for (int leg = 0; leg < explorationPaths.size(); leg++) {
            GHPoint startPoint = originalWaypoints.get(leg);
            GHPoint endPoint = originalWaypoints.get(leg + 1);
            Path explorationPath = explorationPaths.get(leg);

            // Extract original edge IDs from exploration path
            IntArrayList explorationEdges = PathEdgeExtractor.extractOriginalEdgeIds(explorationPath);
            List<EdgeWithIndices> explorationEdgesWithIndices = PathEdgeExtractor.extractEdgesWithIndices(explorationPath);
            PointList explorationPolyline = explorationPath.calcPoints();

            if (explorationEdges.isEmpty()) {
                logger.warn("Leg {}: No exploration edges", leg);
                if (allWaypoints.isEmpty()) {
                    allWaypoints.add(startPoint);
                }
                allWaypoints.add(endPoint);
                continue;
            }

            totalExplorationEdges += explorationEdges.size();

            logger.info("Leg {}: {} exploration edges, {} path points",
                leg, explorationEdges.size(), explorationPolyline.size());

            // Find via-points for this leg
            List<GHPoint> legWaypoints = findViaPointsForLeg(
                explorationEdges, explorationEdgesWithIndices, explorationPolyline,
                startPoint, endPoint, standardPathCalculatorFactory);

            // Add waypoints (skip start if not first leg, as it's already in list)
            if (allWaypoints.isEmpty()) {
                allWaypoints.addAll(legWaypoints);
            } else {
                // Skip first waypoint as it's the end of previous leg
                for (int i = 1; i < legWaypoints.size(); i++) {
                    allWaypoints.add(legWaypoints.get(i));
                }
            }
        }

        // Verify final result
        if (allWaypoints.size() < 2) {
            return NormalizationResult.failure("Failed to generate waypoints");
        }

        // Calculate final path and verify match
        Path finalPath = calculatePath(allWaypoints, standardPathCalculatorFactory);
        if (finalPath != null) {
            IntArrayList finalEdges = PathEdgeExtractor.extractOriginalEdgeIds(finalPath);

            // Combine all exploration edges for match calculation
            IntArrayList allExplorationEdges = new IntArrayList();
            for (Path expPath : explorationPaths) {
                IntArrayList legEdges = PathEdgeExtractor.extractOriginalEdgeIds(expPath);
                for (int i = 0; i < legEdges.size(); i++) {
                    allExplorationEdges.add(legEdges.get(i));
                }
            }

            EdgeMatcher.MatchStats stats = edgeMatcher.calculateMatchStats(allExplorationEdges, finalEdges);
            logger.info("Normalization complete: {} waypoints, {:.1f}% match ({}/{} edges)",
                allWaypoints.size(), stats.getMatchPercentage(),
                stats.getMatchedEdges(), stats.getTotalEdges());

            if (stats.meetsThreshold()) {
                return NormalizationResult.success(allWaypoints, stats.getMatchPercentage(),
                    stats.getTotalEdges(), stats.getMatchedEdges());
            } else {
                return NormalizationResult.bestEffort(allWaypoints, stats.getMatchPercentage(),
                    stats.getTotalEdges(), stats.getMatchedEdges());
            }
        }

        return NormalizationResult.bestEffort(allWaypoints, 0, totalExplorationEdges, 0);
    }

    /**
     * Find via-points for a single leg using binary search.
     *
     * <p>Algorithm: Linear progression with binary search
     * <ol>
     *   <li>Start at edge 0, try to reach end</li>
     *   <li>If direct route matches: done</li>
     *   <li>Binary search to find farthest M where standard(current→M) matches exploration(current→M)</li>
     *   <li>Add waypoint at M, continue from M</li>
     * </ol>
     */
    private List<GHPoint> findViaPointsForLeg(IntArrayList explorationEdges,
                                               List<EdgeWithIndices> explorationEdgesWithIndices,
                                               PointList explorationPolyline,
                                               GHPoint startPoint,
                                               GHPoint endPoint,
                                               Function<List<Snap>, FlexiblePathCalculator> pathCalculatorFactory) {
        List<GHPoint> result = new ArrayList<>();
        result.add(startPoint);

        int edgeCount = explorationEdges.size();
        int currentEdgeIdx = 0;
        int maxIterations = edgeCount + 1;
        int iterations = 0;
        GHPoint lastWaypoint = startPoint;  // Track last added waypoint for starting next segment

        logger.debug("Starting binary search: {} edges", edgeCount);

        while (currentEdgeIdx < edgeCount && iterations < maxIterations) {
            iterations++;

            // Use last waypoint as current position (not the edge junction)
            GHPoint currentPoint = lastWaypoint;

            // First check: can we reach the end directly?
            IntArrayList remainingEdges = getEdgeRange(explorationEdges, currentEdgeIdx, edgeCount);
            Path directToEnd = calculatePath(currentPoint, endPoint, pathCalculatorFactory);

            if (directToEnd != null) {
                IntArrayList directEdges = PathEdgeExtractor.extractOriginalEdgeIds(directToEnd);
                if (edgeMatcher.edgesMatch(remainingEdges, directEdges, null, null)) {
                    logger.debug("Iteration {}: Can reach end directly from edge {}", iterations, currentEdgeIdx);
                    break;
                }
            }

            // Binary search over EDGE indices to find farthest matching edge
            int low = currentEdgeIdx + 1;
            int high = edgeCount;
            int bestMatchEdgeIdx = -1;

            // DEBUG: Log first iteration details
            boolean firstBinarySearchIteration = true;

            while (low <= high) {
                int midEdgeIdx = low + (high - low) / 2;

                // Get path position for candidate point M
                int targetPathIdx = getPathIndexForEdge(explorationEdgesWithIndices, midEdgeIdx - 1, false);
                GHPoint candidatePoint = new GHPoint(
                    explorationPolyline.getLat(targetPathIdx),
                    explorationPolyline.getLon(targetPathIdx));

                // Calculate anchor point - constrains approach direction to candidate
                // Anchor is ANCHOR_OFFSET_EDGES beyond the candidate, or endPoint if near the end
                GHPoint anchorPoint;
                int anchorEdgeIdx = midEdgeIdx + ANCHOR_OFFSET_EDGES;
                if (anchorEdgeIdx >= edgeCount) {
                    // Near the end - use endpoint as anchor
                    anchorPoint = endPoint;
                } else {
                    int anchorPathIdx = getPathIndexForEdge(explorationEdgesWithIndices, anchorEdgeIdx - 1, false);
                    anchorPathIdx = Math.min(anchorPathIdx, explorationPolyline.size() - 1);
                    anchorPoint = new GHPoint(
                        explorationPolyline.getLat(anchorPathIdx),
                        explorationPolyline.getLon(anchorPathIdx));
                }

                // Route [current → candidate → anchor], extract only first leg edges
                IntArrayList candidateEdges = routeWithAnchor(currentPoint, candidatePoint, anchorPoint, pathCalculatorFactory);

                if (candidateEdges == null || candidateEdges.isEmpty()) {
                    // DEBUG: Log routing failure on first few iterations
                    if (iterations <= 3 && firstBinarySearchIteration) {
                        logger.warn("DEBUG iter={} BS: midEdgeIdx={}, routing returned null/empty", iterations, midEdgeIdx);
                    }
                    high = midEdgeIdx - 1;
                    continue;
                }

                // Get exploration edges for the SAME number of edges as the candidate
                // The standard route might reach point M in fewer edges than exploration took
                int edgesToCompare = candidateEdges.size();
                IntArrayList explorationSegment = getEdgeRange(explorationEdges, currentEdgeIdx,
                    Math.min(currentEdgeIdx + edgesToCompare, midEdgeIdx));

                // Compare - for a match, the standard edges should match the exploration prefix
                boolean matches = edgeMatcher.edgesMatch(explorationSegment, candidateEdges, null, null);

                // DEBUG: Log detailed comparison on first iteration only
                if (iterations == 1 && firstBinarySearchIteration) {
                    // Get the actual path for the standard route to see its geometry
                    Path debugPath = calculatePath(currentPoint, candidatePoint, pathCalculatorFactory);
                    PointList debugPoints = debugPath != null ? debugPath.calcPoints() : null;

                    logger.warn("DEBUG iter={} BS: midEdgeIdx={}", iterations, midEdgeIdx);
                    logger.warn("  candidatePoint (target): ({}, {})", candidatePoint.getLat(), candidatePoint.getLon());
                    logger.warn("  candidateEdges count={}, first5={}",
                        candidateEdges.size(), formatEdgeList(candidateEdges, 5));
                    logger.warn("  explorationSegment count={}, first5={}",
                        explorationSegment.size(), formatEdgeList(explorationSegment, 5));

                    if (debugPoints != null && debugPoints.size() > 0) {
                        logger.warn("  standard route: startPt=({}, {}), endPt=({}, {})",
                            debugPoints.getLat(0), debugPoints.getLon(0),
                            debugPoints.getLat(debugPoints.size()-1), debugPoints.getLon(debugPoints.size()-1));
                    }

                    // Show exploration polyline at key positions
                    int expEndIdx = getPathIndexForEdge(explorationEdgesWithIndices, midEdgeIdx - 1, false);
                    logger.warn("  exploration polyline at edge {}: ({}, {})",
                        midEdgeIdx - 1,
                        explorationPolyline.getLat(expEndIdx), explorationPolyline.getLon(expEndIdx));

                    // Show where edges 20 and 21 are (around the mismatch point)
                    if (explorationEdgesWithIndices.size() > 21) {
                        int edge20EndIdx = getPathIndexForEdge(explorationEdgesWithIndices, 20, false);
                        int edge21EndIdx = getPathIndexForEdge(explorationEdgesWithIndices, 21, false);
                        logger.warn("  exploration edge 20 ends at polyline idx {}: ({}, {})",
                            edge20EndIdx, explorationPolyline.getLat(edge20EndIdx), explorationPolyline.getLon(edge20EndIdx));
                        logger.warn("  exploration edge 21 ends at polyline idx {}: ({}, {})",
                            edge21EndIdx, explorationPolyline.getLat(edge21EndIdx), explorationPolyline.getLon(edge21EndIdx));
                    }

                    logger.warn("  matches={}", matches);
                    firstBinarySearchIteration = false;
                }

                if (matches) {
                    // DEBUG: Validate extractEdgesWithIndices against path.calcPoints()
                    if (iterations == 1 && midEdgeIdx == 22) {
                        logger.warn("DEBUG iter=1 MATCH at midEdgeIdx={}", midEdgeIdx);
                        logger.warn("  candidatePoint: ({}, {})", candidatePoint.getLat(), candidatePoint.getLon());

                        // VALIDATION: Compare edge indices to path polyline
                        logger.warn("  VALIDATION - REF path:");
                        logger.warn("    explorationPolyline.size() = {}", explorationPolyline.size());
                        logger.warn("    explorationEdgesWithIndices.size() = {}", explorationEdgesWithIndices.size());
                        if (!explorationEdgesWithIndices.isEmpty()) {
                            EdgeWithIndices lastEdge = explorationEdgesWithIndices.get(explorationEdgesWithIndices.size() - 1);
                            logger.warn("    last edge endIdx = {}, expected = {}",
                                lastEdge.getEndPolylineIndex(), explorationPolyline.size() - 1);
                            boolean valid = lastEdge.getEndPolylineIndex() == explorationPolyline.size() - 1;
                            logger.warn("    VALID: {}", valid);
                        }

                        // Print first 10 REF edges with validation
                        logger.warn("  REF path edges (first 10):");
                        for (int e = 0; e < Math.min(10, explorationEdgesWithIndices.size()); e++) {
                            EdgeWithIndices ei = explorationEdgesWithIndices.get(e);
                            int endIdx = ei.getEndPolylineIndex();
                            logger.warn("    edge[{}] id={}: polyIdx {}-{}, coord=({}, {})",
                                e, ei.getEdgeId(), ei.getStartPolylineIndex(), endIdx,
                                explorationPolyline.getLat(endIdx), explorationPolyline.getLon(endIdx));
                        }

                        // VALIDATION: Standard path
                        Path stdPath = calculatePath(currentPoint, candidatePoint, pathCalculatorFactory);
                        if (stdPath != null) {
                            PointList stdPoints = stdPath.calcPoints();
                            List<EdgeWithIndices> stdEdgesWithIdx = PathEdgeExtractor.extractEdgesWithIndices(stdPath);

                            logger.warn("  VALIDATION - STANDARD path:");
                            logger.warn("    stdPoints.size() = {}", stdPoints.size());
                            logger.warn("    stdEdgesWithIdx.size() = {}", stdEdgesWithIdx.size());
                            if (!stdEdgesWithIdx.isEmpty()) {
                                EdgeWithIndices lastEdge = stdEdgesWithIdx.get(stdEdgesWithIdx.size() - 1);
                                logger.warn("    last edge endIdx = {}, expected = {}",
                                    lastEdge.getEndPolylineIndex(), stdPoints.size() - 1);
                                boolean valid = lastEdge.getEndPolylineIndex() == stdPoints.size() - 1;
                                logger.warn("    VALID: {}", valid);
                            }

                            // Print first 10 STANDARD edges
                            logger.warn("  STANDARD path edges (first 10):");
                            for (int e = 0; e < Math.min(10, stdEdgesWithIdx.size()); e++) {
                                EdgeWithIndices ei = stdEdgesWithIdx.get(e);
                                int endIdx = ei.getEndPolylineIndex();
                                logger.warn("    edge[{}] id={}: polyIdx {}-{}, coord=({}, {})",
                                    e, ei.getEdgeId(), ei.getStartPolylineIndex(), endIdx,
                                    stdPoints.getLat(endIdx), stdPoints.getLon(endIdx));
                            }
                        }
                    }
                    bestMatchEdgeIdx = midEdgeIdx;
                    low = midEdgeIdx + 1;  // Try to go further
                } else {
                    high = midEdgeIdx - 1;  // Must be closer
                }
            }

            if (bestMatchEdgeIdx == -1) {
                // No match found - advance one edge
                if (iterations <= 5) {
                    logger.warn("DEBUG iter={}: No match found, advancing from edge {} to {}",
                        iterations, currentEdgeIdx, currentEdgeIdx + 1);
                }
                bestMatchEdgeIdx = Math.min(currentEdgeIdx + 1, edgeCount);
            } else {
                if (iterations <= 5) {
                    logger.warn("DEBUG iter={}: Binary search found match up to edge {}",
                        iterations, bestMatchEdgeIdx);
                }
            }

            // Add waypoint if not at end
            if (bestMatchEdgeIdx < edgeCount) {
                // Place waypoint at the farthest matching point found by binary search
                int waypointPathIdx = getPathIndexForEdge(explorationEdgesWithIndices, bestMatchEdgeIdx - 1, false);

                // Clamp to valid range
                waypointPathIdx = Math.min(waypointPathIdx, explorationPolyline.size() - 1);

                GHPoint waypoint = new GHPoint(
                    explorationPolyline.getLat(waypointPathIdx),
                    explorationPolyline.getLon(waypointPathIdx));
                result.add(waypoint);
                lastWaypoint = waypoint;  // Update for next iteration

                // DEBUG: Log waypoint snap info
                if (iterations <= 5) {
                    int explorationEdgeAtWaypoint = explorationEdges.get(Math.min(bestMatchEdgeIdx - 1, explorationEdges.size() - 1));
                    Snap waypointSnap = locationIndex.findClosest(waypoint.getLat(), waypoint.getLon(), edgeFilter);
                    int snappedEdge = waypointSnap != null && waypointSnap.isValid()
                        ? waypointSnap.getClosestEdge().getEdge() : -1;
                    logger.warn("DEBUG iter={}: Placed waypoint at edge {}, explorationEdge={}, snappedEdge={}, currentEdgeIdx becomes {}",
                        iterations, bestMatchEdgeIdx - 1, explorationEdgeAtWaypoint, snappedEdge, bestMatchEdgeIdx);
                }

                // Continue from the waypoint edge
                currentEdgeIdx = bestMatchEdgeIdx;
            } else {
                // Advance past the current edge
                currentEdgeIdx = bestMatchEdgeIdx + 1;
            }
        }

        // Always end with the end point
        result.add(endPoint);

        logger.debug("Found {} waypoints (including start and end) for {} edges", result.size(), edgeCount);
        return result;
    }

    /**
     * Get path index for an edge from edge indices list.
     */
    private int getPathIndexForEdge(List<EdgeWithIndices> edgesWithIndices, int edgeIdx, boolean start) {
        if (edgesWithIndices == null || edgeIdx < 0 || edgeIdx >= edgesWithIndices.size()) {
            return 0;
        }
        EdgeWithIndices edge = edgesWithIndices.get(edgeIdx);
        return start ? edge.getStartPolylineIndex() : edge.getEndPolylineIndex();
    }

    /**
     * Format edge list for logging (first N edges).
     */
    private String formatEdgeList(IntArrayList edges, int maxCount) {
        if (edges == null) return "null";
        StringBuilder sb = new StringBuilder("[");
        int count = Math.min(edges.size(), maxCount);
        for (int i = 0; i < count; i++) {
            if (i > 0) sb.append(",");
            sb.append(edges.get(i));
        }
        if (edges.size() > maxCount) {
            sb.append("...(").append(edges.size()).append(" total)");
        }
        sb.append("]");
        return sb.toString();
    }

    /**
     * Get a range of edges from the full edge list.
     */
    private IntArrayList getEdgeRange(IntArrayList allEdges, int startIdx, int endIdx) {
        IntArrayList result = new IntArrayList();
        for (int i = startIdx; i < endIdx && i < allEdges.size(); i++) {
            result.add(allEdges.get(i));
        }
        return result;
    }

    /**
     * Route from start to candidate with an anchor point, returning only the first leg's edges.
     *
     * <p>The anchor constrains the router to approach the candidate point from the correct
     * direction. Without the anchor, the router might find a path that reaches the candidate
     * from a different direction, which would cause subsequent segments to diverge.
     *
     * @param start Starting point
     * @param candidate Candidate waypoint (M) being tested
     * @param anchor Anchor point beyond M on the exploration path (may be null if at end)
     * @param pathCalculatorFactory Factory for creating path calculator
     * @return Edges for the first leg (start→candidate), or null if routing fails
     */
    private IntArrayList routeWithAnchor(GHPoint start, GHPoint candidate, GHPoint anchor,
                                          Function<List<Snap>, FlexiblePathCalculator> pathCalculatorFactory) {
        try {
            // Build waypoint list: [start, candidate] or [start, candidate, anchor]
            List<GHPoint> waypoints = new ArrayList<>();
            waypoints.add(start);
            waypoints.add(candidate);
            if (anchor != null) {
                waypoints.add(anchor);
            }

            // Snap all waypoints
            List<Snap> snaps = new ArrayList<>();
            for (int i = 0; i < waypoints.size(); i++) {
                GHPoint point = waypoints.get(i);
                Snap snap = locationIndex.findClosest(point.getLat(), point.getLon(), edgeFilter);
                if (snap == null || !snap.isValid()) {
                    return null;
                }
                snaps.add(snap);
                // DEBUG: Log snap details for first few calls
                if (logger.isDebugEnabled()) {
                    logger.debug("Snap[{}]: point=({},{}) -> edge={}, node={}",
                        i, point.getLat(), point.getLon(),
                        snap.getClosestEdge().getEdge(), snap.getClosestNode());
                }
            }

            // Create path calculator and route the first leg only
            FlexiblePathCalculator pathCalculator = pathCalculatorFactory.apply(snaps);
            int fromNode = snaps.get(0).getClosestNode();
            int toNode = snaps.get(1).getClosestNode();

            List<Path> paths = pathCalculator.calcPaths(fromNode, toNode, new EdgeRestrictions());
            if (paths.isEmpty() || !paths.get(0).isFound()) {
                return null;
            }

            // Extract edges from first leg only
            return PathEdgeExtractor.extractOriginalEdgeIds(paths.get(0));

        } catch (Exception e) {
            logger.debug("Error routing with anchor: {}", e.getMessage());
            return null;
        }
    }

    /**
     * Calculate path between two points using internal APIs.
     */
    private Path calculatePath(GHPoint start, GHPoint end,
                                Function<List<Snap>, FlexiblePathCalculator> pathCalculatorFactory) {
        List<GHPoint> waypoints = new ArrayList<>();
        waypoints.add(start);
        waypoints.add(end);
        return calculatePath(waypoints, pathCalculatorFactory);
    }

    /**
     * Calculate path through multiple waypoints using internal APIs.
     */
    private Path calculatePath(List<GHPoint> waypoints,
                                Function<List<Snap>, FlexiblePathCalculator> pathCalculatorFactory) {
        if (waypoints.size() < 2) {
            return null;
        }

        try {
            // Snap all waypoints
            List<Snap> snaps = new ArrayList<>();
            for (GHPoint point : waypoints) {
                Snap snap = locationIndex.findClosest(point.getLat(), point.getLon(), edgeFilter);
                if (snap == null || !snap.isValid()) {
                    logger.debug("Failed to snap point at {}", point);
                    return null;
                }
                snaps.add(snap);
            }

            // Create path calculator
            FlexiblePathCalculator pathCalculator = pathCalculatorFactory.apply(snaps);
            Path combinedPath = null;

            // Calculate path for each leg
            for (int i = 0; i < snaps.size() - 1; i++) {
                int fromNode = snaps.get(i).getClosestNode();
                int toNode = snaps.get(i + 1).getClosestNode();

                List<Path> paths = pathCalculator.calcPaths(fromNode, toNode, new EdgeRestrictions());

                if (paths.isEmpty() || !paths.get(0).isFound()) {
                    logger.debug("No path found between snaps {} and {}", i, i + 1);
                    return null;
                }

                Path legPath = paths.get(0);
                if (combinedPath == null) {
                    combinedPath = legPath;
                } else {
                    // Merge paths
                    for (int j = 0; j < legPath.getEdgeCount(); j++) {
                        combinedPath.addEdge(legPath.getEdges().get(j));
                    }
                    combinedPath.addDistance(legPath.getDistance());
                    combinedPath.addTime(legPath.getTime());
                }
            }

            return combinedPath;
        } catch (Exception e) {
            logger.debug("Error calculating path: {}", e.getMessage());
            return null;
        }
    }
}
