/*
 * Trailmap - Route Conversion Service
 *
 * Converts exploration waypoints to normalized waypoints for client editing.
 */
package com.graphhopper.trailmap.roundtrip.exploration;

import com.carrotsearch.hppc.IntSet;
import com.graphhopper.ConvertResponse;
import com.graphhopper.coll.GHIntHashSet;
import com.graphhopper.routing.EdgeRestrictions;
import com.graphhopper.routing.FlexiblePathCalculator;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.weighting.AvoidEdgesWeighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.trailmap.roundtrip.normalization.NormalizationResult;
import com.graphhopper.trailmap.roundtrip.normalization.WaypointNormalizer;
import com.graphhopper.util.PointList;
import com.graphhopper.util.StopWatch;
import com.graphhopper.util.shapes.GHPoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

/**
 * Service for converting exploration waypoints to normalized waypoints.
 *
 * <p>This service recreates the exploration route from snapped waypoints,
 * then runs normalization to produce minimal waypoints for client editing.
 *
 * <p>The conversion process:
 * <ol>
 *   <li>Snap the input waypoints (should be trivial as they're already on network)</li>
 *   <li>Route with exploration profile + AvoidEdgesWeighting to recreate exploration paths</li>
 *   <li>Run WaypointNormalizer to find minimal waypoints for standard profile</li>
 *   <li>Calculate final route through normalized waypoints</li>
 * </ol>
 */
public class RouteConversionService {

    private static final Logger logger = LoggerFactory.getLogger(RouteConversionService.class);

    private final BaseGraph graph;
    private final LocationIndex locationIndex;
    private final EdgeFilter edgeFilter;

    public RouteConversionService(BaseGraph graph, LocationIndex locationIndex, EdgeFilter edgeFilter) {
        this.graph = graph;
        this.locationIndex = locationIndex;
        this.edgeFilter = edgeFilter;
    }

    /**
     * Convert exploration waypoints to normalized waypoints.
     *
     * @param explorationWaypoints Snapped exploration waypoints from non-finalized response
     * @param explorationPathCalculatorFactory Factory for exploration profile routing
     * @param standardPathCalculatorFactory Factory for standard profile routing
     * @return Conversion result with normalized waypoints
     */
    public ConvertResponse convert(List<GHPoint> explorationWaypoints,
                                    Function<List<Snap>, FlexiblePathCalculator> explorationPathCalculatorFactory,
                                    Function<List<Snap>, FlexiblePathCalculator> standardPathCalculatorFactory) {
        StopWatch sw = new StopWatch().start();

        if (explorationWaypoints == null || explorationWaypoints.size() < 2) {
            return ConvertResponse.failure("At least 2 waypoints are required");
        }

        logger.info("Converting {} exploration waypoints", explorationWaypoints.size());

        // Step 1: Snap waypoints
        // These are already snapped positions from the exploration response,
        // so findClosest should return the same edges
        List<Snap> snaps = snapWaypoints(explorationWaypoints);
        if (snaps == null) {
            return ConvertResponse.failure("Failed to snap exploration waypoints");
        }

        // Step 2: Recreate exploration paths with AvoidEdgesWeighting
        // This matches the logic in ExplorationRoundTripRouting.calculateExplorationRoute()
        ExplorationPathResult exploreResult = calculateExplorationPaths(snaps, explorationPathCalculatorFactory);
        if (exploreResult == null || exploreResult.paths.isEmpty()) {
            return ConvertResponse.failure("Failed to recreate exploration route");
        }

        logger.info("Recreated exploration route: {} legs, {} total edges",
            exploreResult.paths.size(), exploreResult.totalEdges);

        // Step 3: Normalize to minimal waypoints
        WaypointNormalizer normalizer = new WaypointNormalizer(graph, locationIndex, edgeFilter);
        NormalizationResult normResult = normalizer.normalize(
            explorationWaypoints, exploreResult.paths, standardPathCalculatorFactory);

        if (!normResult.hasWaypoints()) {
            return ConvertResponse.failure("Normalization failed: " + normResult.getFailureReason());
        }

        logger.info("Normalization complete: {} -> {} waypoints, {:.1f}% match",
            explorationWaypoints.size(), normResult.getWaypoints().size(), normResult.getMatchPercentage());

        // Step 4: Calculate final route through normalized waypoints
        List<Snap> finalSnaps = snapWaypoints(normResult.getWaypoints());
        if (finalSnaps == null) {
            return ConvertResponse.failure("Failed to snap normalized waypoints");
        }

        FinalRouteResult finalRoute = calculateFinalRoute(finalSnaps, standardPathCalculatorFactory);
        if (finalRoute == null) {
            return ConvertResponse.failure("Failed to calculate final route");
        }

        logger.info("Conversion completed in {}ms", sw.stop().getMillis());

        return ConvertResponse.success(
            normResult.getWaypoints(),
            finalRoute.points,
            finalRoute.distance,
            finalRoute.time,
            normResult.getMatchPercentage(),
            explorationWaypoints.size()
        );
    }

    /**
     * Snap waypoints to the road network.
     *
     * <p>Since these are already snapped positions from the exploration response,
     * this should find the same edges.
     */
    private List<Snap> snapWaypoints(List<GHPoint> waypoints) {
        List<Snap> snaps = new ArrayList<>();

        for (GHPoint point : waypoints) {
            Snap snap = locationIndex.findClosest(point.getLat(), point.getLon(), edgeFilter);
            if (snap == null || !snap.isValid()) {
                logger.warn("Failed to snap waypoint at {}", point);
                return null;
            }
            snaps.add(snap);
        }

        return snaps;
    }

    /**
     * Recreate exploration paths with AvoidEdgesWeighting.
     *
     * <p>This matches the logic in ExplorationRoundTripRouting.calculateExplorationRoute()
     * to ensure we get the same paths.
     */
    private ExplorationPathResult calculateExplorationPaths(
            List<Snap> snaps,
            Function<List<Snap>, FlexiblePathCalculator> pathCalculatorFactory) {

        if (snaps.size() < 2) {
            return null;
        }

        try {
            FlexiblePathCalculator pathCalculator = pathCalculatorFactory.apply(snaps);

            // Wrap with AvoidEdgesWeighting to prevent backtracking
            // This MUST match ExplorationRoundTripRouting.calculateExplorationRoute()
            IntSet previousEdges = new GHIntHashSet();
            AvoidEdgesWeighting avoidWeighting = new AvoidEdgesWeighting(pathCalculator.getWeighting())
                .setEdgePenaltyFactor(5.0);
            avoidWeighting.setAvoidedEdges(previousEdges);
            pathCalculator.setWeighting(avoidWeighting);

            List<Path> paths = new ArrayList<>();
            int totalEdges = 0;

            for (int i = 0; i < snaps.size() - 1; i++) {
                int fromNode = snaps.get(i).getClosestNode();
                int toNode = snaps.get(i + 1).getClosestNode();

                List<Path> legPaths = pathCalculator.calcPaths(fromNode, toNode, new EdgeRestrictions());

                if (legPaths.isEmpty() || !legPaths.get(0).isFound()) {
                    logger.warn("No path found between snaps {} and {}", i, i + 1);
                    return null;
                }

                Path legPath = legPaths.get(0);
                paths.add(legPath);
                totalEdges += legPath.getEdgeCount();

                // Add this leg's edges to avoidance set for subsequent legs
                for (int j = 0; j < legPath.getEdgeCount(); j++) {
                    previousEdges.add(legPath.getEdges().get(j));
                }
            }

            return new ExplorationPathResult(paths, totalEdges);

        } catch (Exception e) {
            logger.warn("Error calculating exploration paths: {}", e.getMessage(), e);
            return null;
        }
    }

    /**
     * Calculate final route through waypoints, returning merged points from leg paths.
     *
     * <p>We calculate each leg separately and merge their points,
     * because manually constructed Path objects don't have fromNode set
     * and cannot use calcPoints().
     */
    private FinalRouteResult calculateFinalRoute(List<Snap> snaps,
                                                  Function<List<Snap>, FlexiblePathCalculator> pathCalculatorFactory) {
        if (snaps.size() < 2) {
            return null;
        }

        try {
            FlexiblePathCalculator pathCalculator = pathCalculatorFactory.apply(snaps);
            PointList mergedPoints = null;
            double totalDistance = 0;
            long totalTime = 0;

            for (int i = 0; i < snaps.size() - 1; i++) {
                int fromNode = snaps.get(i).getClosestNode();
                int toNode = snaps.get(i + 1).getClosestNode();

                List<Path> paths = pathCalculator.calcPaths(fromNode, toNode, new EdgeRestrictions());

                if (paths.isEmpty() || !paths.get(0).isFound()) {
                    logger.warn("No path found between snaps {} and {}", i, i + 1);
                    return null;
                }

                Path legPath = paths.get(0);
                PointList legPoints = legPath.calcPoints();

                // Initialize mergedPoints on first leg, matching the elevation capability
                if (mergedPoints == null) {
                    mergedPoints = new PointList(legPoints.size() * snaps.size(), legPoints.is3D());
                }

                // Merge points, skipping first point of subsequent legs to avoid duplicates
                int startIdx = (i == 0) ? 0 : 1;
                for (int j = startIdx; j < legPoints.size(); j++) {
                    if (legPoints.is3D()) {
                        mergedPoints.add(legPoints.getLat(j), legPoints.getLon(j), legPoints.getEle(j));
                    } else {
                        mergedPoints.add(legPoints.getLat(j), legPoints.getLon(j));
                    }
                }

                totalDistance += legPath.getDistance();
                totalTime += legPath.getTime();
            }

            return new FinalRouteResult(mergedPoints, totalDistance, totalTime);

        } catch (Exception e) {
            logger.warn("Error calculating final route: {}", e.getMessage());
            return null;
        }
    }

    /**
     * Internal result holder for exploration path calculation.
     */
    private static class ExplorationPathResult {
        final List<Path> paths;
        final int totalEdges;

        ExplorationPathResult(List<Path> paths, int totalEdges) {
            this.paths = paths;
            this.totalEdges = totalEdges;
        }
    }

    /**
     * Internal result holder for final route calculation.
     */
    private static class FinalRouteResult {
        final PointList points;
        final double distance;
        final long time;

        FinalRouteResult(PointList points, double distance, long time) {
            this.points = points;
            this.distance = distance;
            this.time = time;
        }
    }
}
