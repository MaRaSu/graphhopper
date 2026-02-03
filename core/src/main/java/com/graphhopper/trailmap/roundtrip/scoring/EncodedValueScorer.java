/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Route scorer that uses encoded values to evaluate quality.
 */
package com.graphhopper.trailmap.roundtrip.scoring;

import com.carrotsearch.hppc.IntHashSet;
import com.carrotsearch.hppc.IntIntHashMap;
import com.carrotsearch.hppc.IntIntMap;
import com.carrotsearch.hppc.IntSet;
import com.carrotsearch.hppc.cursors.IntCursor;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.ev.EncodedValueLookup;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.ev.RoadClass;
import com.graphhopper.routing.ev.StringEncodedValue;
import com.graphhopper.trailmap.roundtrip.config.RoundTripProfile;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.FetchMode;
import com.graphhopper.util.PointList;
import com.graphhopper.util.shapes.GHPoint;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Route scorer that analyzes encoded values on edges.
 *
 * Evaluates:
 * - Surface quality (% preferred surfaces, % avoided surfaces)
 * - Road type quality (% preferred highways, % avoided highways)
 * - Uniqueness (% edges not repeated across legs)
 * - Directness (actual distance vs straight-line distance)
 */
public class EncodedValueScorer implements RouteScorer {

    private static final Logger logger = LoggerFactory.getLogger(EncodedValueScorer.class);
    private static final DistanceCalcEarth DIST_CALC = DistanceCalcEarth.DIST_EARTH;

    // Dead-end detection parameters
    private static final double DEAD_END_WINDOW_DISTANCE = 1500.0; // Look within 1.5km of waypoint
    private static final double MIN_DEAD_END_THRESHOLD = 50.0; // Minimum 50m to count as dead-end

    // Surfaces considered unpaved
    private static final Set<String> UNPAVED_SURFACES = new HashSet<>(Arrays.asList(
            "unpaved", "compacted", "gravel", "fine_gravel", "ground", "dirt",
            "earth", "grass", "sand", "mud", "clay"));

    // Surfaces considered paved/asphalt
    private static final Set<String> PAVED_SURFACES = new HashSet<>(Arrays.asList(
            "asphalt", "concrete", "paved", "paving_stones", "sett"));

    @Override
    public String getName() {
        return "encoded-value";
    }

    @Override
    public RouteScore score(List<Path> paths,
            List<GHPoint> waypoints,
            RoundTripProfile profile,
            EncodedValueLookup encodedValueLookup,
            double targetDistance) {

        RouteScore score = new RouteScore();
        score.setTargetDistance(targetDistance);

        // Get encoded values (for future use)
        // StringEncodedValue surfaceEnc = getSurfaceEncodedValue(encodedValueLookup);
        // EnumEncodedValue<RoadClass> roadClassEnc =
        // getRoadClassEncodedValue(encodedValueLookup);

        // Track edges seen across all segments for repetition detection
        // Maps edge ID -> segment index where first seen
        IntIntMap edgeFirstSeenInSegment = new IntIntHashMap();

        double totalDistance = 0;
        double totalRepeatedDistance = 0;
        int totalEdgeCount = 0;
        int totalRepeatedEdgeCount = 0;

        StringBuilder debugLog = new StringBuilder();
        debugLog.append("\n=== ROUTE SCORE DEBUG ===\n");

        // Score each segment
        for (int segIdx = 0; segIdx < paths.size(); segIdx++) {
            Path path = paths.get(segIdx);
            double segmentDistance = path.getDistance();
            int segmentEdgeCount = 0;
            int segmentRepeatedEdgeCount = 0;
            double segmentRepeatedDistance = 0;

            // IMPORTANT: Analyze distant repetition BEFORE adding current segment's edges
            // to map
            // This detects edges repeated from non-adjacent previous segments
            double distantRepetitionDistance = analyzeDistantRepetition(segIdx, path, edgeFirstSeenInSegment);

            // Count edges and track repetition
            for (IntCursor c : path.getEdges()) {
                segmentEdgeCount++;
                int edgeId = c.value;

                if (edgeFirstSeenInSegment.containsKey(edgeId)) {
                    // This edge was used in a previous segment
                    segmentRepeatedEdgeCount++;
                    // Approximate edge distance (uniform distribution assumption)
                    segmentRepeatedDistance += segmentDistance / path.getEdges().size();
                } else {
                    // First time seeing this edge
                    edgeFirstSeenInSegment.put(edgeId, segIdx);
                }
            }

            // Calculate segment uniqueness ratio
            double segmentUniqueness = segmentEdgeCount > 0
                    ? 1.0 - ((double) segmentRepeatedEdgeCount / segmentEdgeCount)
                    : 1.0;

            double segmentRepetitionRatio = segmentDistance > 0
                    ? segmentRepeatedDistance / segmentDistance
                    : 0.0;

            // Calculate directness for this segment
            double straightLine = 0;
            if (segIdx < waypoints.size() - 1) {
                GHPoint from = waypoints.get(segIdx);
                GHPoint to = waypoints.get(segIdx + 1);
                straightLine = DIST_CALC.calcDist(from.getLat(), from.getLon(), to.getLat(), to.getLon());
            }
            double segmentDirectness = straightLine > 0 && segmentDistance > 0
                    ? straightLine / segmentDistance
                    : 0.5;

            // Create leg score
            LegScore legScore = new LegScore(segIdx);
            legScore.setDistance(segmentDistance);
            legScore.setUniquenessRatio(segmentUniqueness);
            legScore.setDirectnessRatio(segmentDirectness);
            // Placeholder values for surface/road quality until we implement edge analysis
            legScore.setSurfaceQuality(0.7);
            legScore.setRoadTypeQuality(0.7);

            // Set distant repetition distance (analyzed above, before adding edges)
            legScore.setDistantRepetitionDistance(distantRepetitionDistance);

            // Identify main issue based on metrics
            if (distantRepetitionDistance > 100) { // More than 100m distant repetition
                legScore.setMainIssue(IssueType.DISTANT_REPETITION);
            } else if (segmentUniqueness < 0.5) {
                legScore.setMainIssue(IssueType.HIGH_REPETITION);
            } else if (segmentDirectness > 0.9) {
                legScore.setMainIssue(IssueType.TOO_DIRECT);
            }

            // Calculate segment score
            RoundTripProfile.WeightsConfig weights = profile.getWeights();
            double segmentScore = legScore.getSurfaceQuality() * 100 * weights.getSurfaceQuality() +
                    legScore.getRoadTypeQuality() * 100 * weights.getRoadTypeQuality() +
                    segmentUniqueness * 100 * weights.getUniqueness() +
                    (segmentDirectness > 0.3 ? 1.0 : segmentDirectness / 0.3) * 100 * weights.getDirectness();
            legScore.setScore(segmentScore);

            if (segmentScore < profile.getScoring().getMinLegScore() && legScore.getMainIssue() == IssueType.NONE) {
                legScore.setMainIssue(IssueType.LOW_QUALITY_LEG);
            }

            score.getLegScores().add(legScore);

            // Accumulate totals
            totalDistance += segmentDistance;
            totalRepeatedDistance += segmentRepeatedDistance;
            totalEdgeCount += segmentEdgeCount;
            totalRepeatedEdgeCount += segmentRepeatedEdgeCount;

            // Debug log for this segment
            debugLog.append(String.format("Segment %d: %.1fkm, %d edges\n",
                    segIdx, segmentDistance / 1000.0, segmentEdgeCount));
            debugLog.append(String.format("  Uniqueness: %.0f%% (%d/%d edges repeated)\n",
                    segmentUniqueness * 100, segmentRepeatedEdgeCount, segmentEdgeCount));
            debugLog.append(String.format("  Directness: %.0f%% (straight=%.1fkm)\n",
                    segmentDirectness * 100, straightLine / 1000.0));
            debugLog.append(String.format("  Score: %.1f%s\n",
                    segmentScore,
                    legScore.getMainIssue() != IssueType.NONE ? " [" + legScore.getMainIssue() + "]" : " [OK]"));
        }

        // Second pass: Analyze dead-end backtracking between adjacent segments (Type 1)
        debugLog.append("\n--- DEAD-END ANALYSIS ---\n");
        double totalDeadEndDistance = 0;

        for (int i = 0; i < paths.size() - 1; i++) {
            Path outboundPath = paths.get(i);
            Path inboundPath = paths.get(i + 1);

            DeadEndAnalysis deadEnd = analyzeDeadEnd(outboundPath, inboundPath);
            if (deadEnd != null) {
                // Attribute dead-end to outbound segment (where the fork point is)
                LegScore outboundLeg = score.getLegScores().get(i);
                outboundLeg.setDeadEndDistance(deadEnd.deadEndDistance);
                outboundLeg.setForkEdgeIndex(deadEnd.forkEdgeIndex);
                outboundLeg.setForkPoint(deadEnd.forkPoint);

                // Update issue type if dead-end is more significant than current issue
                if (deadEnd.deadEndDistance >= MIN_DEAD_END_THRESHOLD &&
                        outboundLeg.getMainIssue() != IssueType.DISTANT_REPETITION) {
                    outboundLeg.setMainIssue(IssueType.DEAD_END_BACKTRACK);
                }

                totalDeadEndDistance += deadEnd.deadEndDistance;

                debugLog.append(String.format("Segments %d→%d: DEAD-END %.0fm backtrack",
                        i, i + 1, deadEnd.deadEndDistance));
                if (deadEnd.forkPoint != null) {
                    debugLog.append(String.format(" (fork: %.5f,%.5f)",
                            deadEnd.forkPoint.getLat(), deadEnd.forkPoint.getLon()));
                }
                debugLog.append("\n");
            } else {
                debugLog.append(String.format("Segments %d→%d: no dead-end\n", i, i + 1));
            }
        }

        score.withMetric("totalDeadEndDistance", totalDeadEndDistance);
        double totalDistantRepetitionDist = score.getLegScores().stream()
                .mapToDouble(LegScore::getDistantRepetitionDistance).sum();
        score.withMetric("totalDistantRepetitionDistance", totalDistantRepetitionDist);

        // Calculate route-level metrics
        double globalRepetitionRatio = totalDistance > 0 ? totalRepeatedDistance / totalDistance : 0;
        double uniqueEdgeRatio = totalEdgeCount > 0 ? 1.0 - ((double) totalRepeatedEdgeCount / totalEdgeCount) : 1.0;

        // Calculate overall directness
        double totalStraightLine = 0;
        if (waypoints.size() >= 2) {
            for (int i = 0; i < waypoints.size() - 1; i++) {
                GHPoint from = waypoints.get(i);
                GHPoint to = waypoints.get(i + 1);
                totalStraightLine += DIST_CALC.calcDist(from.getLat(), from.getLon(), to.getLat(), to.getLon());
            }
        }
        double overallDirectness = totalStraightLine > 0 && totalDistance > 0
                ? totalStraightLine / totalDistance
                : 0.5;

        // Store distance for unified quality model
        score.setActualDistance(totalDistance);

        // Store metrics
        score.withMetric("totalDistance", totalDistance);
        score.withMetric("totalEdges", (double) totalEdgeCount);
        score.withMetric("uniqueEdges", (double) edgeFirstSeenInSegment.size());
        score.withMetric("repeatedEdges", (double) totalRepeatedEdgeCount);
        score.withMetric("repetitionRatio", globalRepetitionRatio);
        score.withMetric("directnessRatio", overallDirectness);

        // Calculate weighted averages for surface/road quality
        double avgSurfaceQuality = score.getLegScores().stream()
                .mapToDouble(leg -> leg.getSurfaceQuality() * leg.getDistance())
                .sum() / totalDistance;
        double avgRoadTypeQuality = score.getLegScores().stream()
                .mapToDouble(leg -> leg.getRoadTypeQuality() * leg.getDistance())
                .sum() / totalDistance;

        score.withMetric("avgSurfaceQuality", avgSurfaceQuality);
        score.withMetric("avgRoadTypeQuality", avgRoadTypeQuality);

        // Calculate overall score
        RoundTripProfile.WeightsConfig weights = profile.getWeights();
        double uniquenessScore = 1.0 - globalRepetitionRatio;
        double directnessScore = calculateDirectnessScore(overallDirectness, profile.getScoring());

        double overallScore = avgSurfaceQuality * 100 * weights.getSurfaceQuality() +
                avgRoadTypeQuality * 100 * weights.getRoadTypeQuality() +
                uniquenessScore * 100 * weights.getUniqueness() +
                directnessScore * 100 * weights.getDirectness();

        score.setOverallScore(overallScore);
        score.withMetric("uniquenessScore", uniquenessScore);
        score.withMetric("directnessScore", directnessScore);

        // Check thresholds
        checkThresholds(score, profile);

        // Complete debug log
        debugLog.append("\n--- ROUTE TOTALS ---\n");
        debugLog.append(String.format("Total Distance: %.1fkm\n", totalDistance / 1000.0));
        debugLog.append(String.format("Total Edges: %d (unique: %d)\n", totalEdgeCount, edgeFirstSeenInSegment.size()));
        debugLog.append(
                String.format("Repetition: %.1f%% of distance on repeated edges\n", globalRepetitionRatio * 100));
        debugLog.append(String.format("  Dead-end backtrack: %.0fm\n", totalDeadEndDistance));
        debugLog.append(String.format("  Distant repetition: %.0fm\n", totalDistantRepetitionDist));
        debugLog.append(String.format("Directness: %.1f%%\n", overallDirectness * 100));
        debugLog.append(String.format("OVERALL SCORE: %.1f [%s]\n",
                overallScore, score.isAcceptable() ? "ACCEPTABLE" : "NOT ACCEPTABLE"));
        if (!score.getIssues().isEmpty()) {
            debugLog.append("Issues: ").append(score.getIssues()).append("\n");
        }

        // Log at appropriate levels
        logger.info("Route scored: {} [{}] segments={} repetition={}% (deadEnd={}m, distant={}m)",
                String.format("%.1f", overallScore),
                score.isAcceptable() ? "OK" : "FAIL",
                paths.size(),
                String.format("%.0f", globalRepetitionRatio * 100),
                String.format("%.0f", totalDeadEndDistance),
                String.format("%.0f", totalDistantRepetitionDist));
        logger.debug(debugLog.toString());

        return score;
    }

    /**
     * Calculate directness score (penalize if too direct or too indirect).
     */
    private double calculateDirectnessScore(double directnessRatio, RoundTripProfile.ScoringConfig scoring) {
        if (directnessRatio < scoring.getMinDirectnessRatio()) {
            return directnessRatio / scoring.getMinDirectnessRatio();
        } else if (directnessRatio > scoring.getMaxDirectnessRatio()) {
            return scoring.getMaxDirectnessRatio() / directnessRatio;
        }
        return 1.0;
    }

    /**
     * Check scoring thresholds and set acceptability.
     */
    private void checkThresholds(RouteScore score, RoundTripProfile profile) {
        RoundTripProfile.ScoringConfig scoring = profile.getScoring();
        boolean acceptable = true;

        // Check overall score
        if (score.getOverallScore() < scoring.getMinOverallScore()) {
            acceptable = false;
            score.withIssue(String.format("Overall score %.1f below threshold %.1f",
                    score.getOverallScore(), scoring.getMinOverallScore()));
        }

        // Check repetition
        Double repetitionRatio = score.getMetrics().get("repetitionRatio");
        if (repetitionRatio != null && repetitionRatio > scoring.getMaxRepetitionRatio()) {
            acceptable = false;
            score.withIssue(String.format("Repetition ratio %.0f%% exceeds maximum %.0f%%",
                    repetitionRatio * 100, scoring.getMaxRepetitionRatio() * 100));
        }

        // Check leg scores
        for (LegScore leg : score.getLegScores()) {
            if (leg.getScore() < scoring.getMinLegScore()) {
                acceptable = false;
                score.withIssue(String.format("Leg %d score %.1f below threshold %.1f",
                        leg.getLegIndex(), leg.getScore(), scoring.getMinLegScore()));
                break; // One bad leg is enough
            }
        }

        // Check for dead-end backtracking per segment
        for (LegScore leg : score.getLegScores()) {
            if (leg.getDeadEndDistance() >= MIN_DEAD_END_THRESHOLD && leg.getForkPoint() != null) {
                acceptable = false;
                score.withIssue(String.format("Segment %d has %.0fm dead-end backtrack",
                        leg.getLegIndex(), leg.getDeadEndDistance()));
                break; // One dead-end is enough to trigger fixing
            }
        }

        // Check distance tolerance (unified quality model)
        if (!score.isDistanceWithinTolerance() && score.getTargetDistance() > 0) {
            acceptable = false;
            IssueType distanceIssue = score.getDistanceIssue();
            double ratio = score.getDistanceRatio();
            score.withIssue(String.format("Distance %s: actual=%.0fm, target=%.0fm, ratio=%.2f",
                    distanceIssue == IssueType.TOO_LONG ? "too long" : "too short",
                    score.getActualDistance(), score.getTargetDistance(), ratio));
        }

        score.setAcceptable(acceptable);
    }

    private StringEncodedValue getSurfaceEncodedValue(EncodedValueLookup lookup) {
        try {
            if (lookup.hasEncodedValue("surface")) {
                return lookup.getStringEncodedValue("surface");
            }
        } catch (Exception e) {
            logger.debug("Surface encoded value not available");
        }
        return null;
    }

    @SuppressWarnings("unchecked")
    private EnumEncodedValue<RoadClass> getRoadClassEncodedValue(EncodedValueLookup lookup) {
        try {
            if (lookup.hasEncodedValue("road_class")) {
                return lookup.getEnumEncodedValue("road_class", RoadClass.class);
            }
        } catch (Exception e) {
            logger.debug("Road class encoded value not available");
        }
        return null;
    }

    // ========== Dead-End Detection ==========

    /**
     * Result of dead-end analysis between two adjacent segments.
     */
    private static class DeadEndAnalysis {
        final double deadEndDistance; // Total backtrack distance
        final int forkEdgeIndex; // Index in outbound segment where backtrack starts
        final GHPoint forkPoint; // Coordinate for fixer

        DeadEndAnalysis(double deadEndDistance, int forkEdgeIndex, GHPoint forkPoint) {
            this.deadEndDistance = deadEndDistance;
            this.forkEdgeIndex = forkEdgeIndex;
            this.forkPoint = forkPoint;
        }
    }

    /**
     * Simple edge info holder.
     */
    private static class EdgeInfo {
        final int edgeId;
        final double distance;
        final GHPoint midPoint; // Approximate midpoint for fork location

        EdgeInfo(int edgeId, double distance, GHPoint midPoint) {
            this.edgeId = edgeId;
            this.distance = distance;
            this.midPoint = midPoint;
        }
    }

    /**
     * Analyze dead-end backtracking between two adjacent segments.
     * Uses window-based approach: looks at edges within DEAD_END_WINDOW_DISTANCE
     * of the waypoint from both segments.
     *
     * @param outboundPath Previous segment (ending at waypoint)
     * @param inboundPath  Next segment (starting from waypoint)
     * @return DeadEndAnalysis with backtrack distance and fork info, or null if no
     *         dead-end
     */
    private DeadEndAnalysis analyzeDeadEnd(Path outboundPath, Path inboundPath) {
        // Get ordered edges with distances
        List<EdgeIteratorState> outboundEdges = outboundPath.calcEdges();
        List<EdgeIteratorState> inboundEdges = inboundPath.calcEdges();

        if (outboundEdges.isEmpty() || inboundEdges.isEmpty()) {
            return null;
        }

        // Collect edges from END of outbound within window
        List<EdgeInfo> outboundWindow = collectEdgesFromEnd(outboundEdges, DEAD_END_WINDOW_DISTANCE);

        // Collect edges from START of inbound within window
        List<EdgeInfo> inboundWindow = collectEdgesFromStart(inboundEdges, DEAD_END_WINDOW_DISTANCE);

        // Build set of edge IDs from outbound window
        Set<Integer> outboundEdgeIds = new HashSet<>();
        for (EdgeInfo ei : outboundWindow) {
            outboundEdgeIds.add(ei.edgeId);
        }

        // Count how much of inbound window overlaps with outbound
        double totalBacktrackDistance = 0;
        Set<Integer> repeatedEdgeIds = new HashSet<>();

        for (EdgeInfo ei : inboundWindow) {
            if (outboundEdgeIds.contains(ei.edgeId)) {
                totalBacktrackDistance += ei.distance;
                repeatedEdgeIds.add(ei.edgeId);
            }
        }

        // Check threshold
        if (totalBacktrackDistance < MIN_DEAD_END_THRESHOLD) {
            return null;
        }

        // Find fork point: walk from start-side toward waypoint to find first repeated
        // edge
        // outboundWindow is in reverse order: [0]=at waypoint, [n]=toward start
        // So we walk from high index (toward start) to low index (toward waypoint)
        int forkEdgeIndex = -1;
        GHPoint forkPoint = null;

        for (int i = outboundWindow.size() - 1; i >= 0; i--) {
            EdgeInfo ei = outboundWindow.get(i);
            if (repeatedEdgeIds.contains(ei.edgeId)) {
                // Found first repeated edge (where dead-end section begins)
                // Fork point should be on the edge BEFORE this in path order
                // In window order, "before in path" = higher index (further from waypoint)
                forkEdgeIndex = outboundEdges.size() - 1 - i;

                if (i < outboundWindow.size() - 1) {
                    // Use the previous-in-path edge's midpoint (i+1 in window order)
                    forkPoint = outboundWindow.get(i + 1).midPoint;
                } else {
                    // First repeated edge is at window boundary - fork is before window
                    int prevEdgeIdx = forkEdgeIndex - 1;
                    if (prevEdgeIdx >= 0) {
                        EdgeIteratorState prevEdge = outboundEdges.get(prevEdgeIdx);
                        forkPoint = getEdgeMidPoint(prevEdge);
                    } else {
                        // Backtrack starts from very beginning, use first repeated edge location
                        forkPoint = ei.midPoint;
                    }
                }
                break;
            }
        }

        logger.debug("Dead-end detected: {}m backtrack, fork at edge index {}",
                String.format("%.0f", totalBacktrackDistance), forkEdgeIndex);

        return new DeadEndAnalysis(totalBacktrackDistance, forkEdgeIndex, forkPoint);
    }

    /**
     * Collect edges from the END of a path within the specified distance.
     * Returns edges in reverse order (closest to end first).
     */
    private List<EdgeInfo> collectEdgesFromEnd(List<EdgeIteratorState> edges, double maxDistance) {
        List<EdgeInfo> result = new ArrayList<>();
        double accumulated = 0;

        for (int i = edges.size() - 1; i >= 0 && accumulated < maxDistance; i--) {
            EdgeIteratorState edge = edges.get(i);
            double dist = edge.getDistance();
            result.add(new EdgeInfo(edge.getEdge(), dist, getEdgeMidPoint(edge)));
            accumulated += dist;
        }

        return result;
    }

    /**
     * Collect edges from the START of a path within the specified distance.
     * Returns edges in order (closest to start first).
     */
    private List<EdgeInfo> collectEdgesFromStart(List<EdgeIteratorState> edges, double maxDistance) {
        List<EdgeInfo> result = new ArrayList<>();
        double accumulated = 0;

        for (int i = 0; i < edges.size() && accumulated < maxDistance; i++) {
            EdgeIteratorState edge = edges.get(i);
            double dist = edge.getDistance();
            result.add(new EdgeInfo(edge.getEdge(), dist, getEdgeMidPoint(edge)));
            accumulated += dist;
        }

        return result;
    }

    /**
     * Get approximate midpoint of an edge for fork location.
     */
    private GHPoint getEdgeMidPoint(EdgeIteratorState edge) {
        PointList points = edge.fetchWayGeometry(FetchMode.ALL);
        if (points.isEmpty()) {
            return null;
        }
        int midIdx = points.size() / 2;
        return new GHPoint(points.getLat(midIdx), points.getLon(midIdx));
    }

    /**
     * Analyze distant repetition: edges repeated from non-adjacent segments.
     *
     * <p>
     * This detects "out-and-back" patterns where the route uses edges from
     * earlier (non-adjacent) segments, indicating the route went somewhere,
     * possibly did a small loop, then returned on the same edges.
     *
     * <p>
     * Adjacent repetition (from immediately previous segment) is excluded
     * as it's handled separately by dead-end detection.
     *
     * @param currentSegmentIdx      Index of current segment
     * @param path                   Current segment path
     * @param edgeFirstSeenInSegment Map of edge ID -> segment index where first
     *                               seen
     * @return Total distance of distant repetition (edges from non-adjacent
     *         previous segments)
     */
    private double analyzeDistantRepetition(int currentSegmentIdx, Path path,
            IntIntMap edgeFirstSeenInSegment) {
        double distantDistance = 0;
        List<EdgeIteratorState> edges = path.calcEdges();

        for (EdgeIteratorState edge : edges) {
            int edgeId = edge.getEdge();
            if (edgeFirstSeenInSegment.containsKey(edgeId)) {
                int firstSeenSeg = edgeFirstSeenInSegment.get(edgeId);
                // Distant = repeated from non-adjacent segment (not the immediately previous
                // one)
                if (firstSeenSeg != currentSegmentIdx - 1) {
                    distantDistance += edge.getDistance();
                }
            }
        }

        return distantDistance;
    }
}
