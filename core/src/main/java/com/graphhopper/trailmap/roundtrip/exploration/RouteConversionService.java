/*
 * Trailmap - Route Conversion Service
 *
 * Converts exploration waypoints to normalized waypoints for client editing.
 */
package com.graphhopper.trailmap.roundtrip.exploration;

import com.carrotsearch.hppc.IntSet;
import com.graphhopper.ConvertResponse;
import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.ResponsePath;
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
import com.graphhopper.util.CustomModel;
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
 * <p>The conversion process:
 * <ol>
 *   <li><b>Stage A</b> — recreate the exploration paths from the snapped waypoints using the
 *       exploration profile + {@link AvoidEdgesWeighting} (the anti-backtracking penalty that is
 *       essential to route quality). This stays on the low-level routing path.</li>
 *   <li><b>Stage B</b> — {@link WaypointNormalizer} finds the minimal waypoints that reproduce
 *       those exploration paths under the standard profile, validated by directed edge_key
 *       comparison via the public routing API.</li>
 *   <li><b>Stage C</b> — route the standard profile through the normalized waypoints (public API)
 *       to produce the final geometry returned to the client.</li>
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
     * @param explorationWaypoints              Snapped exploration waypoints from a non-finalized response
     * @param explorationPathCalculatorFactory  Factory for exploration profile routing (Stage A)
     * @param profile                           Standard (rendering) profile name (Stages B/C)
     * @param customModel                       Optional custom model passed to the router (may be null)
     * @param router                            Public routing entry point (e.g. {@code Router::route})
     * @return Conversion result with normalized waypoints and final geometry
     */
    public ConvertResponse convert(List<GHPoint> explorationWaypoints,
                                   Function<List<Snap>, FlexiblePathCalculator> explorationPathCalculatorFactory,
                                   String profile,
                                   CustomModel customModel,
                                   Function<GHRequest, GHResponse> router) {
        StopWatch sw = new StopWatch().start();

        if (explorationWaypoints == null || explorationWaypoints.size() < 2) {
            return ConvertResponse.failure("At least 2 waypoints are required");
        }

        logger.info("Converting {} exploration waypoints", explorationWaypoints.size());

        // Stage A: snap waypoints (already snapped positions from the exploration response)
        List<Snap> snaps = snapWaypoints(explorationWaypoints);
        if (snaps == null) {
            return ConvertResponse.failure("Failed to snap exploration waypoints");
        }

        // Stage A: recreate exploration paths with AvoidEdgesWeighting (anti-backtracking).
        ExplorationPathResult exploreResult = calculateExplorationPaths(snaps, explorationPathCalculatorFactory);
        if (exploreResult == null || exploreResult.paths.isEmpty()) {
            return ConvertResponse.failure("Failed to recreate exploration route");
        }

        logger.info("Recreated exploration route: {} legs, {} total edges",
                exploreResult.paths.size(), exploreResult.totalEdges);

        // Stage B: normalize to minimal waypoints (edge_key validated via the public router).
        WaypointNormalizer normalizer = new WaypointNormalizer(graph);
        NormalizationResult normResult = normalizer.normalize(
                explorationWaypoints, exploreResult.paths, profile, customModel, router);

        if (!normResult.hasWaypoints()) {
            return ConvertResponse.failure("Normalization failed: " + normResult.getFailureReason());
        }

        logger.info("Normalization complete: {} -> {} waypoints, {}% match",
                explorationWaypoints.size(), normResult.getWaypoints().size(),
                String.format("%.1f", normResult.getMatchPercentage()));

        // Stage C: final route through the normalized waypoints via the public routing API.
        GHRequest finalReq = new GHRequest(new ArrayList<>(normResult.getWaypoints()));
        finalReq.setProfile(profile);
        if (customModel != null) finalReq.setCustomModel(customModel);
        finalReq.putHint("instructions", false);
        // Geometry is mandatory output for /convert; set explicitly so it can't fall back to a
        // server-wide routing.calc_points=false.
        finalReq.putHint("calc_points", true);

        GHResponse finalRsp;
        try {
            finalRsp = router.apply(finalReq);
        } catch (Exception e) {
            logger.warn("Error calculating final route: {}", e.getMessage());
            return ConvertResponse.failure("Failed to calculate final route");
        }
        if (finalRsp.hasErrors() || finalRsp.getBest() == null) {
            logger.warn("Final route had errors: {}", finalRsp.getErrors());
            return ConvertResponse.failure("Failed to calculate final route");
        }

        ResponsePath best = finalRsp.getBest();

        logger.info("Conversion completed in {}ms", sw.stop().getMillis());

        return ConvertResponse.success(
                normResult.getWaypoints(),
                best.getPoints(),
                best.getDistance(),
                best.getTime(),
                normResult.getMatchPercentage(),
                explorationWaypoints.size()
        );
    }

    /**
     * Snap waypoints to the road network. Since these are already snapped positions from the
     * exploration response, this should find the same edges.
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
     * Recreate exploration paths with {@link AvoidEdgesWeighting} to prevent backtracking.
     *
     * <p>This MUST match {@code ExplorationRoundTripRouting.calculateExplorationRoute()} so the
     * recreated paths are identical to the ones the exploration step produced.
     */
    private ExplorationPathResult calculateExplorationPaths(
            List<Snap> snaps,
            Function<List<Snap>, FlexiblePathCalculator> pathCalculatorFactory) {

        if (snaps.size() < 2) {
            return null;
        }

        try {
            FlexiblePathCalculator pathCalculator = pathCalculatorFactory.apply(snaps);

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
}
