/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Two-phase exploration round-trip routing orchestrator.
 */
package com.graphhopper.trailmap.roundtrip.exploration;

import com.carrotsearch.hppc.IntSet;
import com.graphhopper.GHRequest;
import com.graphhopper.ResponsePath;
import com.graphhopper.coll.GHIntHashSet;
import com.graphhopper.config.Profile;
import com.graphhopper.routing.EdgeRestrictions;
import com.graphhopper.routing.FlexiblePathCalculator;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.ev.EncodedValueLookup;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.weighting.AvoidEdgesWeighting;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.util.details.PathDetail;
import com.graphhopper.util.details.PathDetailsBuilderFactory;
import com.graphhopper.util.details.PathDetailsFromEdges;
import com.graphhopper.trailmap.roundtrip.config.RoundTripProfile;
import com.graphhopper.trailmap.roundtrip.fixing.DeadEndFixer;
import com.graphhopper.trailmap.roundtrip.fixing.FixResult;
import com.graphhopper.trailmap.roundtrip.geometry.*;
import com.graphhopper.trailmap.roundtrip.normalization.*;
import com.graphhopper.trailmap.roundtrip.scoring.*;
import com.graphhopper.trailmap.roundtrip.snapping.*;
import com.graphhopper.util.PointList;
import com.graphhopper.util.StopWatch;
import com.graphhopper.util.shapes.GHPoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.function.Function;

/**
 * Two-phase exploration round-trip routing.
 *
 * <p>This orchestrator implements the exploration approach:
 * <ol>
 *   <li><b>Phase 1 - Exploration:</b> Route with exploration profile that
 *       heavily favors quality over directness</li>
 *   <li><b>Phase 2 - Normalization:</b> Convert exploration route into minimal
 *       waypoints using standard profile</li>
 * </ol>
 *
 * <p>The result is a set of waypoints that, when routed by the client using
 * the standard profile, will reproduce the high-quality exploration route.
 *
 * <p>Activated via: {@code round_trip.mode=exploration}
 */
public class ExplorationRoundTripRouting {

    private static final Logger logger = LoggerFactory.getLogger(ExplorationRoundTripRouting.class);

    // Exploration profile suffix
    private static final String EXPLORE_PROFILE_SUFFIX = "_explore";

    // Default configuration
    private static final int DEFAULT_MAX_ATTEMPTS = 3;
    private static final int DEFAULT_MAX_FIX_ATTEMPTS = 40;

    // Dead-end fix configuration
    private static final double MIN_DEAD_END_TO_FIX = 50.0;

    // GH dependencies
    private final BaseGraph graph;
    private final LocationIndex locationIndex;
    private final EdgeFilter edgeFilter;
    private final EncodedValueLookup encodedValueLookup;
    private final Map<String, Profile> profilesByName;

    // Geometry strategies (reused from EnhancedRoundTripRouting)
    private final Map<String, TourGeometryStrategy> geometryStrategies;

    // Snapping strategy
    private final WaypointSnapStrategy snapper;

    // Route scorer (for quality reporting)
    private final RouteScorer scorer;

    // Dead-end fixer
    private final DeadEndFixer deadEndFixer;

    // Path details factory
    private final PathDetailsBuilderFactory pathDetailsBuilderFactory;

    /**
     * Create exploration round-trip routing.
     *
     * @param graph                      Base graph
     * @param locationIndex              Location index for snapping
     * @param edgeFilter                 Edge filter for snapping
     * @param encodedValueLookup         For reading encoded values
     * @param profilesByName             Map of profile name to Profile for profile lookup
     * @param pathDetailsBuilderFactory  Factory for creating path detail builders
     */
    public ExplorationRoundTripRouting(BaseGraph graph, LocationIndex locationIndex,
                                        EdgeFilter edgeFilter,
                                        EncodedValueLookup encodedValueLookup,
                                        Map<String, Profile> profilesByName,
                                        PathDetailsBuilderFactory pathDetailsBuilderFactory) {
        this.graph = graph;
        this.locationIndex = locationIndex;
        this.edgeFilter = edgeFilter;
        this.encodedValueLookup = encodedValueLookup;
        this.profilesByName = profilesByName;
        this.pathDetailsBuilderFactory = pathDetailsBuilderFactory;

        this.geometryStrategies = initGeometryStrategies();
        this.snapper = new ProfileAwareSnapper(graph);
        this.scorer = new EncodedValueScorer();
        this.deadEndFixer = new DeadEndFixer();
    }

    private Map<String, TourGeometryStrategy> initGeometryStrategies() {
        Map<String, TourGeometryStrategy> strategies = new LinkedHashMap<>();
        strategies.put("circle", new CircleTourGeometry());
        strategies.put("diamond", new DiamondTourGeometry(2.0));
        strategies.put("rectangle", new RectangleTourGeometry(5.0));
        strategies.put("petal", new PetalTourGeometry());
        strategies.put("figure8", new Figure8TourGeometry());
        strategies.put("teardrop", new TeardropTourGeometry());
        strategies.put("serpentine", new SerpentineTourGeometry());
        strategies.put("hourglass", new HourglassTourGeometry());
        return strategies;
    }

    /**
     * Generate an exploration round-trip route.
     *
     * @param request                           GH request with round-trip parameters
     * @param explorationPathCalculatorFactory  Factory for exploration profile routing
     * @param standardPathCalculatorFactory     Factory for standard profile routing (normalization)
     * @param exploreWeighting                  Weighting for exploration profile (used for path details)
     * @param standardWeighting                 Weighting for standard profile (used for path details)
     * @return Exploration result with normalized waypoints
     */
    public ExplorationRoundTripResult route(GHRequest request,
                                             Function<List<Snap>, FlexiblePathCalculator> explorationPathCalculatorFactory,
                                             Function<List<Snap>, FlexiblePathCalculator> standardPathCalculatorFactory,
                                             Weighting exploreWeighting,
                                             Weighting standardWeighting) {
        StopWatch sw = new StopWatch().start();

        // Parse parameters
        GHPoint start = request.getPoints().get(0);
        double distance = request.getHints().getDouble("round_trip.distance", 10000);
        double heading = request.getHeadings().isEmpty() ? Double.NaN : request.getHeadings().get(0);
        long seed = request.getHints().getLong("round_trip.seed", 0);
        String shapeName = request.getHints().getString("round_trip.shape", "circle");
        String standardProfileName = request.getProfile();
        int maxAttempts = request.getHints().getInt("round_trip.max_attempts", DEFAULT_MAX_ATTEMPTS);
        int maxFixes = request.getHints().getInt("round_trip.max_fixes", DEFAULT_MAX_FIX_ATTEMPTS);
        boolean returnMetrics = request.getHints().getBool("round_trip.return_metrics", false);
        boolean finalize = request.getHints().getBool("round_trip.finalize", true);
        String snappingPref = request.getHints().getString(SnappingPreference.API_PARAM, null);
        SnappingPreference snappingPreference = SnappingPreference.fromString(snappingPref);
        // Note: sample_interval hint is no longer used - normalization now works
        // per-leg with binary search insertion of via-points

        // Derive exploration profile name
        String exploreProfileName = deriveExplorationProfileName(standardProfileName);

        logger.info("Exploration round-trip: shape={}, distance={}m, profile={} -> {}, heading={}, finalize={}",
            shapeName, distance, standardProfileName, exploreProfileName, heading, finalize);

        // Get geometry strategy
        TourGeometryStrategy geometry = geometryStrategies.getOrDefault(shapeName,
            geometryStrategies.get("circle"));

        // Get profile for scoring
        RoundTripProfile roundTripProfile = getOrCreateProfile(standardProfileName);

        // Main attempt loop
        for (int attempt = 0; attempt < maxAttempts; attempt++) {
            long attemptSeed = seed + attempt;

            try {
                ExplorationRoundTripResult result = attemptExplorationRoute(
                    start, distance, heading, attemptSeed,
                    geometry, roundTripProfile, maxFixes, finalize,
                    explorationPathCalculatorFactory, standardPathCalculatorFactory,
                    returnMetrics, snappingPreference,
                    request.getPathDetails(), exploreWeighting, standardWeighting);

                if (result.isSuccess() || result.getNormalizedWaypoints().size() > 0) {
                    logger.info("Exploration round-trip completed in {}s on attempt {}, {} waypoints, {:.1f}% match",
                        sw.stop().getSeconds(), attempt + 1,
                        result.getNormalizedWaypoints().size(),
                        result.getNormalizationMatchPercentage());
                    return result;
                }

            } catch (Exception e) {
                logger.warn("Exploration attempt {} failed: {}", attempt + 1, e.getMessage(), e);
            }
        }

        return ExplorationRoundTripResult.failure(
            "Could not generate exploration route after " + maxAttempts + " attempts");
    }

    /**
     * Attempt a single exploration round-trip.
     *
     * <p>This method implements the full exploration flow:
     * <ol>
     *   <li>Generate geometric waypoints</li>
     *   <li>Route with exploration profile (using AvoidEdgesWeighting)</li>
     *   <li>Score and fix (DeadEndFixer) until acceptable</li>
     *   <li>If finalize=true: Normalize to minimal waypoints</li>
     *   <li>Build final response</li>
     * </ol>
     *
     * <p>When finalize=false, normalization is skipped and the exploration
     * route is returned directly with snapped exploration waypoints.
     */
    private ExplorationRoundTripResult attemptExplorationRoute(
            GHPoint start, double distance, double heading, long seed,
            TourGeometryStrategy geometry, RoundTripProfile profile,
            int maxFixes, boolean finalize,
            Function<List<Snap>, FlexiblePathCalculator> explorationPathCalculatorFactory,
            Function<List<Snap>, FlexiblePathCalculator> standardPathCalculatorFactory,
            boolean returnMetrics, SnappingPreference snappingPreference,
            List<String> requestedPathDetails, Weighting exploreWeighting, Weighting standardWeighting) {

        // ================================================================
        // Phase 1: Generate exploration route with quality control
        // ================================================================

        // Generate geometric waypoints
        List<GHPoint> currentWaypoints = geometry.generateWaypoints(start, distance, heading, seed);
        logger.debug("Generated {} geometric waypoints using {} geometry",
            currentWaypoints.size(), geometry.getName());

        // Snap waypoints
        List<Snap> explorationSnaps = snapWaypoints(currentWaypoints, profile, snappingPreference, seed);
        if (explorationSnaps == null) {
            return ExplorationRoundTripResult.failure("Failed to snap waypoints for exploration");
        }

        // Route with exploration profile using AvoidEdgesWeighting to prevent backtracking
        ExplorationRouteResult exploreResult = calculateExplorationRoute(
            explorationSnaps, explorationPathCalculatorFactory);
        if (exploreResult == null || exploreResult.combinedPath == null ||
                exploreResult.combinedPath.getEdgeCount() == 0) {
            return ExplorationRoundTripResult.failure("Failed to calculate exploration route");
        }

        logger.info("Exploration route: {} edges, {:.0f}m",
            exploreResult.combinedPath.getEdgeCount(), exploreResult.combinedPath.getDistance());

        // Score the exploration route
        RouteScore currentScore = scorer.score(exploreResult.paths, currentWaypoints,
            profile, encodedValueLookup, distance);

        // ================================================================
        // Fix Loop: Apply fixes (DeadEndFixer only for now) until acceptable
        // ================================================================

        for (int fixAttempt = 0; fixAttempt < maxFixes; fixAttempt++) {
            // Check if route is acceptable
            if (currentScore.isAcceptable()) {
                logger.info("Exploration route acceptable after {} fix attempts", fixAttempt);
                break;
            }

            // Try to fix dead-ends
            FixResult fix = tryFixDeadEnd(currentWaypoints, currentScore, profile);
            if (!fix.isSuccess()) {
                logger.debug("Fix loop ended at attempt {}: {}", fixAttempt + 1, fix.getDescription());
                break;
            }

            logger.debug("Applied fix {}: {}", fixAttempt + 1, fix.getDescription());
            currentWaypoints = fix.getModifiedWaypoints();

            // Re-snap modified waypoints
            explorationSnaps = snapWaypoints(currentWaypoints, profile, snappingPreference, seed);
            if (explorationSnaps == null) {
                logger.debug("Failed to snap after fix");
                continue;
            }

            // Re-route with exploration profile
            exploreResult = calculateExplorationRoute(explorationSnaps, explorationPathCalculatorFactory);
            if (exploreResult == null || exploreResult.combinedPath == null ||
                    exploreResult.combinedPath.getEdgeCount() == 0) {
                logger.debug("Failed to calculate route after fix");
                continue;
            }

            // Re-score
            currentScore = scorer.score(exploreResult.paths, currentWaypoints,
                profile, encodedValueLookup, distance);
            currentScore.withFix(fix.getDescription());

            logger.debug("After fix {}: score={}, issues={}",
                fixAttempt + 1, String.format("%.1f", currentScore.getOverallScore()),
                currentScore.getIssues());
        }

        // Extract snapped waypoint positions from explorationSnaps
        // These are the actual positions on the road network
        List<GHPoint> snappedExplorationWaypoints = new ArrayList<>();
        for (Snap snap : explorationSnaps) {
            snappedExplorationWaypoints.add(snap.getSnappedPoint());
        }

        // Capture final exploration waypoints after fix loop completes
        // These are the geometric waypoints that produced the exploration route
        List<GHPoint> finalExplorationWaypoints = new ArrayList<>(currentWaypoints);

        // ================================================================
        // Non-finalized path: Return exploration route without normalization
        // ================================================================

        if (!finalize) {
            logger.info("Non-finalized mode: returning exploration route with {} snapped waypoints",
                snappedExplorationWaypoints.size());

            // Build response path from exploration route (using leg paths, not combinedPath)
            ResponsePath responsePath = buildExplorationResponsePath(
                exploreResult.paths, exploreResult.combinedPath.getDistance(),
                exploreResult.combinedPath.getTime(), snappedExplorationWaypoints,
                requestedPathDetails, exploreWeighting);

            // Score for quality reporting (optional)
            RouteScore score = null;
            if (returnMetrics) {
                score = scorer.score(exploreResult.paths, currentWaypoints,
                    profile, encodedValueLookup, distance);
            }

            return ExplorationRoundTripResult.nonFinalized(responsePath,
                snappedExplorationWaypoints, score, returnMetrics);
        }

        // ================================================================
        // Phase 2: Normalize to minimal waypoints
        // ================================================================

        WaypointNormalizer normalizer = new WaypointNormalizer(
            graph, locationIndex, edgeFilter);

        // Pass original waypoints and per-leg exploration paths to normalizer
        // This enables the correct binary search approach: INSERT via-points where needed
        NormalizationResult normResult = normalizer.normalize(
            currentWaypoints, exploreResult.paths, standardPathCalculatorFactory);

        if (!normResult.hasWaypoints()) {
            return ExplorationRoundTripResult.failure(normResult.getFailureReason());
        }

        logger.info("Normalization: {} original waypoints -> {} final waypoints, {:.1f}% match",
            currentWaypoints.size(), normResult.getWaypoints().size(), normResult.getMatchPercentage());

        // ================================================================
        // Build final response
        // ================================================================

        // Route the normalized waypoints for the final path
        List<Snap> finalSnaps = snapWaypointsSimple(normResult.getWaypoints());
        if (finalSnaps == null) {
            return ExplorationRoundTripResult.failure("Failed to snap normalized waypoints");
        }

        Path finalPath = calculatePath(finalSnaps, standardPathCalculatorFactory);
        if (finalPath == null) {
            return ExplorationRoundTripResult.failure("Failed to calculate final route");
        }

        // Build response path with exploration waypoints
        ResponsePath responsePath = buildResponsePath(finalPath, finalSnaps, finalExplorationWaypoints,
            requestedPathDetails, standardWeighting);

        // Score for quality reporting
        RouteScore score = null;
        if (returnMetrics) {
            List<Path> paths = Collections.singletonList(finalPath);
            score = scorer.score(paths, normResult.getWaypoints(), profile, encodedValueLookup, distance);
        }

        return ExplorationRoundTripResult.fromNormalization(responsePath, normResult,
            finalExplorationWaypoints, score, returnMetrics);
    }

    /**
     * Try to fix dead-ends in the route using DeadEndFixer.
     *
     * @param waypoints Current waypoints
     * @param score Current route score
     * @param profile Round-trip profile
     * @return Fix result (success or failure)
     */
    private FixResult tryFixDeadEnd(List<GHPoint> waypoints, RouteScore score, RoundTripProfile profile) {
        if (score == null || score.getLegScores() == null) {
            return FixResult.failure("No scoring data available", "dead-end");
        }

        // Check for dead-ends that need fixing
        for (LegScore leg : score.getLegScores()) {
            if (leg.getDeadEndDistance() >= MIN_DEAD_END_TO_FIX && leg.getForkPoint() != null) {
                logger.debug("Found dead-end in leg {}: {}m with fork point",
                    leg.getLegIndex(), leg.getDeadEndDistance());
                return deadEndFixer.attemptFix(waypoints, score, leg.getLegIndex(), profile);
            }
        }

        return FixResult.failure("No dead-ends found", "dead-end");
    }

    /**
     * Calculate exploration route with AvoidEdgesWeighting to prevent backtracking.
     *
     * <p>Unlike standard routing, exploration mode uses AvoidEdgesWeighting because
     * the normalized waypoints (output) are routed with the STANDARD profile, not
     * the exploration profile. The exploration phase is internal only.
     *
     * @param snaps Snapped waypoints
     * @param pathCalculatorFactory Factory for creating path calculators
     * @return Exploration route result with paths and combined path
     */
    private ExplorationRouteResult calculateExplorationRoute(
            List<Snap> snaps,
            Function<List<Snap>, FlexiblePathCalculator> pathCalculatorFactory) {
        if (snaps.size() < 2) {
            return null;
        }

        try {
            FlexiblePathCalculator pathCalculator = pathCalculatorFactory.apply(snaps);

            // Wrap with AvoidEdgesWeighting to prevent backtracking
            IntSet previousEdges = new GHIntHashSet();
            AvoidEdgesWeighting avoidWeighting = new AvoidEdgesWeighting(pathCalculator.getWeighting())
                .setEdgePenaltyFactor(5.0);
            avoidWeighting.setAvoidedEdges(previousEdges);
            pathCalculator.setWeighting(avoidWeighting);

            List<Path> paths = new ArrayList<>();
            Path combinedPath = null;

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

                // Add this leg's edges to avoidance set for subsequent legs
                for (int j = 0; j < legPath.getEdgeCount(); j++) {
                    previousEdges.add(legPath.getEdges().get(j));
                }

                // Build combined path - always add edges to separate object
                // (avoid aliasing bug where combinedPath === paths[0])
                if (combinedPath == null) {
                    combinedPath = new Path(graph);
                }
                for (int j = 0; j < legPath.getEdgeCount(); j++) {
                    combinedPath.addEdge(legPath.getEdges().get(j));
                }
                combinedPath.addDistance(legPath.getDistance());
                combinedPath.addTime(legPath.getTime());
            }

            return new ExplorationRouteResult(paths, combinedPath);
        } catch (Exception e) {
            logger.warn("Error calculating exploration path: {}", e.getMessage());
            return null;
        }
    }

    /**
     * Internal result holder for exploration route calculation.
     */
    private static class ExplorationRouteResult {
        final List<Path> paths;
        final Path combinedPath;

        ExplorationRouteResult(List<Path> paths, Path combinedPath) {
            this.paths = paths;
            this.combinedPath = combinedPath;
        }
    }

    /**
     * Derive exploration profile name from standard profile name.
     *
     * <p>Maps: "gravel" -> "gravel_explore", "mtb" -> "mtb_explore", etc.
     */
    private String deriveExplorationProfileName(String standardProfileName) {
        String exploreName = standardProfileName + EXPLORE_PROFILE_SUFFIX;

        // Check if exploration profile exists
        if (profilesByName.containsKey(exploreName)) {
            return exploreName;
        }

        // Fallback: use standard profile
        logger.warn("Exploration profile '{}' not found, using standard profile '{}'",
            exploreName, standardProfileName);
        return standardProfileName;
    }

    /**
     * Get or create a RoundTripProfile for scoring.
     */
    private RoundTripProfile getOrCreateProfile(String profileName) {
        // Use default gravel profile for now
        // Future: load from configuration based on profileName
        if (profileName.contains("mtb")) {
            return RoundTripProfile.createMtbProfile();
        } else if (profileName.contains("road")) {
            return RoundTripProfile.createRoadBikeProfile();
        }
        return RoundTripProfile.createGravelProfile();
    }

    /**
     * Snap waypoints using profile-aware snapping.
     */
    private List<Snap> snapWaypoints(List<GHPoint> waypoints, RoundTripProfile profile,
            SnappingPreference snappingPreference, long seed) {
        List<Snap> snaps = new ArrayList<>();

        for (int i = 0; i < waypoints.size(); i++) {
            GHPoint point = waypoints.get(i);
            Snap snap = snapper.findBestSnap(point, profile, locationIndex, edgeFilter,
                encodedValueLookup, snappingPreference, seed, i);
            if (snap == null || !snap.isValid()) {
                // Try default snapping
                snap = locationIndex.findClosest(point.getLat(), point.getLon(), edgeFilter);
                if (snap == null || !snap.isValid()) {
                    logger.warn("Failed to snap waypoint at {}", point);
                    return null;
                }
            }
            snaps.add(snap);
        }

        return snaps;
    }

    /**
     * Simple snapping without profile awareness.
     */
    private List<Snap> snapWaypointsSimple(List<GHPoint> waypoints) {
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
     * Calculate path through waypoints.
     */
    private Path calculatePath(List<Snap> snaps,
                                Function<List<Snap>, FlexiblePathCalculator> pathCalculatorFactory) {
        if (snaps.size() < 2) {
            return null;
        }

        try {
            FlexiblePathCalculator pathCalculator = pathCalculatorFactory.apply(snaps);
            Path combinedPath = null;

            for (int i = 0; i < snaps.size() - 1; i++) {
                int fromNode = snaps.get(i).getClosestNode();
                int toNode = snaps.get(i + 1).getClosestNode();

                List<Path> paths = pathCalculator.calcPaths(fromNode, toNode, new EdgeRestrictions());

                if (paths.isEmpty() || !paths.get(0).isFound()) {
                    logger.warn("No path found between snaps {} and {}", i, i + 1);
                    return null;
                }

                Path legPath = paths.get(0);
                if (combinedPath == null) {
                    combinedPath = legPath;
                } else {
                    // Merge paths by adding edges
                    for (int j = 0; j < legPath.getEdgeCount(); j++) {
                        combinedPath.addEdge(legPath.getEdges().get(j));
                    }
                    combinedPath.addDistance(legPath.getDistance());
                    combinedPath.addTime(legPath.getTime());
                }
            }

            return combinedPath;
        } catch (Exception e) {
            logger.warn("Error calculating path: {}", e.getMessage());
            return null;
        }
    }

    /**
     * Build ResponsePath from Path, snaps, and exploration waypoints.
     *
     * @param path                  The calculated path
     * @param snaps                 Snapped normalized waypoints
     * @param explorationWaypoints  Final exploration waypoints (after fix loop)
     * @param requestedPathDetails  Path details to extract (e.g., surface, road_class)
     * @param weighting             Weighting for detail extraction
     * @return ResponsePath with all waypoint information and path details
     */
    private ResponsePath buildResponsePath(Path path, List<Snap> snaps,
                                            List<GHPoint> explorationWaypoints,
                                            List<String> requestedPathDetails,
                                            Weighting weighting) {
        ResponsePath responsePath = new ResponsePath();

        // Set basic path info
        responsePath.setDistance(path.getDistance());
        responsePath.setTime(path.getTime());

        // Set points
        PointList points = path.calcPoints();
        responsePath.setPoints(points);

        // Set waypoints (snapped normalized positions)
        PointList waypoints = new PointList(snaps.size(), false);
        for (Snap snap : snaps) {
            waypoints.add(snap.getSnappedPoint().getLat(), snap.getSnappedPoint().getLon());
        }
        responsePath.setWaypoints(waypoints);

        // Set exploration waypoints (geometric shape vertices after fix loop)
        if (explorationWaypoints != null && !explorationWaypoints.isEmpty()) {
            PointList exploreWps = new PointList(explorationWaypoints.size(), false);
            for (GHPoint p : explorationWaypoints) {
                exploreWps.add(p.getLat(), p.getLon());
            }
            responsePath.setExplorationWaypoints(exploreWps);
        }

        // Extract and add path details if requested
        if (requestedPathDetails != null && !requestedPathDetails.isEmpty()) {
            Map<String, List<PathDetail>> details = PathDetailsFromEdges.calcDetails(
                path, encodedValueLookup, weighting, requestedPathDetails,
                pathDetailsBuilderFactory, 0, graph);
            responsePath.addPathDetails(details);
        }

        return responsePath;
    }

    /**
     * Build ResponsePath for non-finalized exploration route.
     *
     * <p>Used when finalize=false. Returns the exploration route with snapped
     * waypoints (not normalized waypoints). The snapped_waypoints field contains
     * the snapped exploration waypoints that can be sent to /convert later.
     *
     * @param legPaths              The per-leg exploration paths (properly initialized Path objects)
     * @param totalDistance         Total route distance
     * @param totalTime             Total route time
     * @param snappedWaypoints      Snapped positions as GHPoints
     * @param requestedPathDetails  Path details to extract (e.g., surface, road_class)
     * @param weighting             Weighting for detail extraction
     * @return ResponsePath with exploration route and path details
     */
    private ResponsePath buildExplorationResponsePath(List<Path> legPaths, double totalDistance,
                                                       long totalTime, List<GHPoint> snappedWaypoints,
                                                       List<String> requestedPathDetails,
                                                       Weighting weighting) {
        ResponsePath responsePath = new ResponsePath();

        // Set basic path info
        responsePath.setDistance(totalDistance);
        responsePath.setTime(totalTime);

        // Build points by merging leg paths, tracking point indices for detail extraction
        PointList points = new PointList();
        int[] legStartIndices = new int[legPaths.size()];
        for (int i = 0; i < legPaths.size(); i++) {
            legStartIndices[i] = points.size();
            PointList legPoints = legPaths.get(i).calcPoints();
            // Skip first point of subsequent legs to avoid duplicates at junctions
            int startIdx = (i == 0) ? 0 : 1;
            for (int j = startIdx; j < legPoints.size(); j++) {
                points.add(legPoints.getLat(j), legPoints.getLon(j));
            }
        }
        responsePath.setPoints(points);

        // Set waypoints as the snapped exploration waypoints
        // These are the waypoints that can be sent to /convert
        PointList waypoints = new PointList(snappedWaypoints.size(), false);
        for (GHPoint p : snappedWaypoints) {
            waypoints.add(p.getLat(), p.getLon());
        }
        responsePath.setWaypoints(waypoints);

        // For non-finalized, exploration_waypoints is same as waypoints
        // (both are snapped positions)
        responsePath.setExplorationWaypoints(waypoints);

        // Extract and add path details from all legs if requested
        if (requestedPathDetails != null && !requestedPathDetails.isEmpty()) {
            int pointIndex = 0;
            for (int i = 0; i < legPaths.size(); i++) {
                Path legPath = legPaths.get(i);
                Map<String, List<PathDetail>> legDetails = PathDetailsFromEdges.calcDetails(
                    legPath, encodedValueLookup, weighting, requestedPathDetails,
                    pathDetailsBuilderFactory, pointIndex, graph);
                responsePath.addPathDetails(legDetails);
                // Update point index: add leg points minus 1 for shared junction (except first leg)
                pointIndex += legPath.calcPoints().size() - (i == 0 ? 0 : 1);
            }
        }

        return responsePath;
    }
}
