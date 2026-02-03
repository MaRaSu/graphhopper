/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Main orchestrator for enhanced round-trip route generation.
 */
package com.graphhopper.trailmap.roundtrip;

import com.graphhopper.GHRequest;
import com.graphhopper.ResponsePath;
import com.graphhopper.routing.EdgeRestrictions;
import com.graphhopper.routing.FlexiblePathCalculator;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.ev.EncodedValueLookup;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.trailmap.roundtrip.config.RoundTripProfile;
import com.graphhopper.trailmap.roundtrip.fixing.*;
import com.graphhopper.trailmap.roundtrip.geometry.*;
import com.graphhopper.trailmap.roundtrip.scoring.*;
import com.graphhopper.trailmap.roundtrip.snapping.*;
import com.graphhopper.util.PointList;
import com.graphhopper.util.shapes.GHPoint;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.function.Function;

/**
 * Enhanced round-trip routing with multiple geometry shapes, quality scoring,
 * and route fixing.
 *
 * This class orchestrates the enhanced round-trip generation process:
 * 1. Generate waypoints using selected geometry strategy
 * 2. Snap waypoints to road network (profile-aware)
 * 3. Calculate route between waypoints
 * 4. Score route quality against profile criteria
 * 5. Apply fixes if quality is insufficient
 * 6. Retry with different geometry seed if fixes fail
 */
public class EnhancedRoundTripRouting {

    private static final Logger logger = LoggerFactory.getLogger(EnhancedRoundTripRouting.class);

    // GH dependencies
    private final BaseGraph graph;
    private final LocationIndex locationIndex;
    private final Weighting baseWeighting;
    private final EdgeFilter edgeFilter;
    private final EncodedValueLookup encodedValueLookup;

    // Geometry strategies
    private final Map<String, TourGeometryStrategy> geometryStrategies;

    // Snapping strategy
    private final WaypointSnapStrategy snapper;

    // Scoring strategy
    private final RouteScorer scorer;

    // Route fixers (sorted by priority)
    private final List<RouteFixer> fixers;

    // Default configuration
    private static final int DEFAULT_MAX_GEOMETRY_ATTEMPTS = 3;
    private static final int DEFAULT_MAX_FIX_ATTEMPTS = 40;
    private static final int DEFAULT_MAX_SNAP_RETRIES = 3;

    /**
     * Create enhanced round-trip routing with default components.
     *
     * @param graph              Base graph
     * @param locationIndex      Location index for snapping
     * @param baseWeighting      Weighting for path calculation
     * @param edgeFilter         Edge filter for snapping
     * @param encodedValueLookup For reading encoded values (can be EncodingManager)
     */
    public EnhancedRoundTripRouting(BaseGraph graph, LocationIndex locationIndex,
            Weighting baseWeighting, EdgeFilter edgeFilter,
            EncodedValueLookup encodedValueLookup) {
        this.graph = graph;
        this.locationIndex = locationIndex;
        this.baseWeighting = baseWeighting;
        this.edgeFilter = edgeFilter;
        this.encodedValueLookup = encodedValueLookup;

        this.geometryStrategies = initGeometryStrategies();
        this.snapper = new ProfileAwareSnapper(graph);
        this.scorer = new EncodedValueScorer();
        this.fixers = initFixers();
    }

    /**
     * Create enhanced round-trip routing (backward compatible constructor).
     */
    public EnhancedRoundTripRouting(BaseGraph graph, LocationIndex locationIndex,
            Weighting baseWeighting, EdgeFilter edgeFilter) {
        this(graph, locationIndex, baseWeighting, edgeFilter, null);
    }

    private Map<String, TourGeometryStrategy> initGeometryStrategies() {
        Map<String, TourGeometryStrategy> strategies = new LinkedHashMap<>();
        // Perimeter-tracing geometries
        strategies.put("circle", new CircleTourGeometry());
        strategies.put("diamond", new DiamondTourGeometry(2.0));
        strategies.put("rectangle", new RectangleTourGeometry(5.0));
        strategies.put("petal", new PetalTourGeometry());
        strategies.put("figure8", new Figure8TourGeometry());
        strategies.put("teardrop", new TeardropTourGeometry());
        // Interior-traversing geometries
        strategies.put("serpentine", new SerpentineTourGeometry());
        strategies.put("hourglass", new HourglassTourGeometry());
        return strategies;
    }

    private List<RouteFixer> initFixers() {
        List<RouteFixer> list = new ArrayList<>();

        // Fixers in priority order (lower priority = tried first)
        list.add(new DeadEndFixer()); // Priority 5 - fix dead-ends first
        list.add(new DistanceFixer()); // Priority 8 - adjust distance

        // Disabled until validated:
        // list.add(new KinkFixer()); // Priority 10
        // list.add(new AnchorSwapFixer()); // Priority 20
        // list.add(new HingeFixer()); // Priority 30

        return list;
    }

    /**
     * Generate an enhanced round-trip route.
     *
     * @param request               GH request with round-trip parameters
     * @param pathCalculatorFactory Factory that creates path calculator from snaps
     *                              (with proper QueryGraph)
     * @return Enhanced result with route and quality score
     */
    public EnhancedRoundTripResult route(GHRequest request,
            Function<List<Snap>, FlexiblePathCalculator> pathCalculatorFactory) {
        // Parse parameters
        GHPoint start = request.getPoints().get(0);
        double distance = request.getHints().getDouble("round_trip.distance", 10000);
        double heading = request.getHeadings().isEmpty() ? Double.NaN : request.getHeadings().get(0);
        long seed = request.getHints().getLong("round_trip.seed", 0);
        String shapeName = request.getHints().getString("round_trip.shape", "circle");
        String profileName = request.getHints().getString("round_trip.quality_profile", request.getProfile());
        int maxAttempts = request.getHints().getInt("round_trip.max_attempts", DEFAULT_MAX_GEOMETRY_ATTEMPTS);
        int maxFixes = request.getHints().getInt("round_trip.max_fixes", DEFAULT_MAX_FIX_ATTEMPTS);
        boolean returnMetrics = request.getHints().getBool("round_trip.return_metrics", false);
        String snappingPref = request.getHints().getString(SnappingPreference.API_PARAM, null);
        SnappingPreference snappingPreference = SnappingPreference.fromString(snappingPref);

        // Get geometry strategy
        TourGeometryStrategy geometry = geometryStrategies.getOrDefault(shapeName,
                geometryStrategies.get("circle"));

        // Get or create profile
        RoundTripProfile profile = getOrCreateProfile(profileName);

        logger.info("Enhanced round-trip: shape={}, distance={}m, heading={}, profile={}, snapping={}",
                shapeName, distance, heading, profileName, snappingPreference.name());

        // Main attempt loop
        RouteScore lastScore = null;
        for (int attempt = 0; attempt < maxAttempts; attempt++) {
            long attemptSeed = seed + attempt;

            try {
                EnhancedRoundTripResult result = attemptRoute(
                        start, distance, heading, attemptSeed,
                        geometry, profile, maxFixes,
                        pathCalculatorFactory, returnMetrics,
                        snappingPreference);

                if (result.isSuccess()) {
                    RouteScore score = result.getScore();
                    if (score != null) {
                        score.setAttemptsUsed(attempt + 1);
                    }
                    logger.info("Enhanced round-trip succeeded on attempt {}, score={}",
                            attempt + 1, score != null ? String.format("%.1f", score.getOverallScore()) : "N/A");
                    return result;
                }

                // Keep track of last score for failure message
                if (result.getScore() != null) {
                    lastScore = result.getScore();
                }

            } catch (Exception e) {
                logger.warn("Enhanced round-trip attempt {} failed: {}", attempt + 1, e.getMessage());
            }
        }

        return EnhancedRoundTripResult.failure(
                "Could not generate acceptable route after " + maxAttempts + " attempts", lastScore);
    }

    /**
     * Attempt a single round-trip generation with optional fix iterations.
     */
    private EnhancedRoundTripResult attemptRoute(GHPoint start, double distance, double heading,
            long seed, TourGeometryStrategy geometry,
            RoundTripProfile profile, int maxFixes,
            Function<List<Snap>, FlexiblePathCalculator> pathCalculatorFactory,
            boolean returnMetrics,
            SnappingPreference snappingPreference) {
        // Generate waypoints
        List<GHPoint> waypoints = geometry.generateWaypoints(start, distance, heading, seed);
        logger.debug("Generated {} waypoints using {} geometry", waypoints.size(), geometry.getName());

        // Snap waypoints to road network
        List<Snap> snaps = snapWaypoints(waypoints, profile, snappingPreference, seed);
        if (snaps == null) {
            return EnhancedRoundTripResult.failure("Failed to snap waypoints to road network");
        }

        // Calculate route
        RouteCalculationResult calcResult = calculateRoute(snaps, pathCalculatorFactory);
        if (calcResult == null || calcResult.paths.isEmpty()) {
            return EnhancedRoundTripResult.failure("Failed to calculate route between waypoints");
        }

        // Score the route (includes distance check per unified quality model)
        RouteScore score = scorer.score(calcResult.paths, waypoints, profile, encodedValueLookup, distance);

        // Check if acceptable (includes dead-end check)
        if (score.isAcceptable()) {
            ResponsePath responsePath = buildResponsePath(calcResult);
            return EnhancedRoundTripResult.success(responsePath, score, returnMetrics);
        }

        logger.debug("Route not acceptable (score={}, issues={}), attempting fixes",
                String.format("%.1f", score.getOverallScore()), score.getIssues());

        // Fix iteration loop
        List<GHPoint> currentWaypoints = new ArrayList<>(waypoints);
        RouteScore currentScore = score;

        for (int fixAttempt = 0; fixAttempt < maxFixes; fixAttempt++) {
            // Try to fix
            FixResult fix = tryFix(currentWaypoints, currentScore, profile);
            if (!fix.isSuccess()) {
                logger.info("Fix loop ended at attempt {}/{}: no more fixes available ({})",
                        fixAttempt + 1, maxFixes, fix.getDescription());
                break;
            }

            // Check if this is the last allowed attempt
            if (fixAttempt == maxFixes - 1) {
                logger.warn("MAX FIX ATTEMPTS REACHED ({}) - stopping fix loop", maxFixes);
            }

            logger.debug("Applied fix {}: {}", fixAttempt + 1, fix.getDescription());
            currentWaypoints = fix.getModifiedWaypoints();

            // Re-snap modified waypoints
            snaps = snapWaypoints(currentWaypoints, profile, snappingPreference, seed);
            if (snaps == null) {
                logger.debug("Failed to snap after fix");
                continue;
            }

            // Re-calculate route
            calcResult = calculateRoute(snaps, pathCalculatorFactory);
            if (calcResult == null || calcResult.paths.isEmpty()) {
                logger.debug("Failed to calculate route after fix");
                continue;
            }

            // Re-score
            currentScore = scorer.score(calcResult.paths, currentWaypoints, profile, encodedValueLookup, distance);
            currentScore.withFix(fix.getDescription());

            // Check if now acceptable
            if (currentScore.isAcceptable()) {
                ResponsePath responsePath = buildResponsePath(calcResult);
                logger.info("Route acceptable after {} fix(es), score={}",
                        fixAttempt + 1, String.format("%.1f", currentScore.getOverallScore()));
                return EnhancedRoundTripResult.success(responsePath, currentScore, returnMetrics);
            }

            logger.debug("After fix {}: still not acceptable (issues={})",
                    fixAttempt + 1, currentScore.getIssues());
        }

        // Post-fix cleanup: merge any waypoints that ended up too close together
        List<GHPoint> cleanedWaypoints = mergeCloseWaypoints(currentWaypoints);
        if (cleanedWaypoints.size() < currentWaypoints.size()) {
            logger.info("Post-fix cleanup: merged {} close waypoints",
                    currentWaypoints.size() - cleanedWaypoints.size());

            // Re-route with cleaned waypoints
            snaps = snapWaypoints(cleanedWaypoints, profile, snappingPreference, seed);
            if (snaps != null) {
                calcResult = calculateRoute(snaps, pathCalculatorFactory);
                if (calcResult != null && !calcResult.paths.isEmpty()) {
                    int mergedCount = currentWaypoints.size() - cleanedWaypoints.size();
                    currentWaypoints = cleanedWaypoints;
                    currentScore = scorer.score(calcResult.paths, currentWaypoints, profile, encodedValueLookup,
                            distance);
                    currentScore.withFix("Merged " + mergedCount + " close waypoints");
                }
            }
        }

        // Log final distance ratio
        if (calcResult != null && !calcResult.paths.isEmpty()) {
            double actualDistance = calcResult.paths.stream().mapToDouble(Path::getDistance).sum();
            double finalRatio = actualDistance / distance;
            logger.info("Final distance: {}m (target: {}m, ratio: {})",
                    Math.round(actualDistance), Math.round(distance), String.format("%.2f", finalRatio));
        }

        // Return best attempt even if not acceptable
        if (calcResult != null && !calcResult.paths.isEmpty()) {
            ResponsePath responsePath = buildResponsePath(calcResult);
            return EnhancedRoundTripResult.success(responsePath, currentScore, returnMetrics);
        }

        return EnhancedRoundTripResult.failure("Could not generate acceptable route", currentScore);
    }

    /** Minimum dead-end distance to trigger fix (meters) */
    private static final double MIN_DEAD_END_TO_FIX = 50.0;

    /** Waypoints closer than this are merged in post-fix cleanup (meters) */
    private static final double WAYPOINT_MERGE_THRESHOLD = 150.0;

    /** Minimum waypoints to keep (start + 1 intermediate + end) */
    private static final int MIN_WAYPOINTS = 3;

    /**
     * Try to fix a route by checking DATA directly (not derived mainIssue field).
     * Priority: dead-ends first (most precise fix), then distance, then other
     * issues.
     *
     * Unified quality model: ALL issues (dead-ends, distance, surface, etc.)
     * are handled in this single fix loop.
     */
    private FixResult tryFix(List<GHPoint> waypoints, RouteScore score, RoundTripProfile profile) {
        if (score == null || score.getLegScores() == null) {
            return FixResult.failure("No scoring data available");
        }

        // Priority 1: Dead-ends - check raw data directly
        for (LegScore leg : score.getLegScores()) {
            if (leg.getDeadEndDistance() >= MIN_DEAD_END_TO_FIX && leg.getForkPoint() != null) {
                logger.debug("Found dead-end in leg {}: {}m with fork point",
                        leg.getLegIndex(), leg.getDeadEndDistance());

                // Find DeadEndFixer and apply it
                for (RouteFixer fixer : fixers) {
                    if (fixer.getName().equals("dead-end")) {
                        FixResult result = fixer.attemptFix(waypoints, score, leg.getLegIndex(), profile);
                        if (result.isSuccess()) {
                            return result;
                        }
                        logger.debug("DeadEndFixer failed: {}", result.getDescription());
                    }
                }
            }
        }

        // Priority 2: Distance issues
        IssueType distanceIssue = score.getDistanceIssue();
        if (distanceIssue != IssueType.NONE) {
            logger.debug("Found distance issue: {} (ratio: {})",
                    distanceIssue, String.format("%.2f", score.getDistanceRatio()));

            // Find DistanceFixer and apply it
            for (RouteFixer fixer : fixers) {
                if (fixer.getName().equals("distance")) {
                    FixResult result = fixer.attemptFix(waypoints, score, -1, profile);
                    if (result.isSuccess()) {
                        return result;
                    }
                    logger.debug("DistanceFixer failed: {}", result.getDescription());
                }
            }
        }

        // Priority 3: Other fixers (when re-enabled)
        // KinkFixer, AnchorSwapFixer, HingeFixer

        return FixResult.failure("No fixable issues found");
    }

    /**
     * Merge waypoints that are too close together.
     * Scans through waypoints and removes any that are within
     * WAYPOINT_MERGE_THRESHOLD
     * of the previous waypoint. Preserves start and end waypoints.
     *
     * @param waypoints Original waypoint list
     * @return New list with close waypoints removed (or same list if none removed)
     */
    private List<GHPoint> mergeCloseWaypoints(List<GHPoint> waypoints) {
        if (waypoints == null || waypoints.size() <= MIN_WAYPOINTS) {
            return waypoints;
        }

        List<GHPoint> result = new ArrayList<>();
        result.add(waypoints.get(0)); // Always keep start

        for (int i = 1; i < waypoints.size() - 1; i++) {
            GHPoint current = waypoints.get(i);
            GHPoint lastKept = result.get(result.size() - 1);

            double dist = distance(lastKept, current);
            if (dist >= WAYPOINT_MERGE_THRESHOLD) {
                result.add(current);
            } else {
                logger.debug("Merging waypoint {} ({}m from previous)", i, String.format("%.0f", dist));
            }

            // Safety: don't go below minimum
            if (result.size() == 1 && i == waypoints.size() - 2) {
                // Only start kept, and this is the last intermediate - must keep it
                if (!result.contains(current)) {
                    result.add(current);
                }
            }
        }

        result.add(waypoints.get(waypoints.size() - 1)); // Always keep end

        // Ensure minimum waypoints
        if (result.size() < MIN_WAYPOINTS) {
            return waypoints; // Abort merge, keep original
        }

        return result;
    }

    /**
     * Calculate distance between two points in meters.
     */
    private double distance(GHPoint p1, GHPoint p2) {
        // Haversine formula
        double lat1 = Math.toRadians(p1.getLat());
        double lat2 = Math.toRadians(p2.getLat());
        double dLat = lat2 - lat1;
        double dLon = Math.toRadians(p2.getLon() - p1.getLon());

        double a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
                Math.cos(lat1) * Math.cos(lat2) *
                        Math.sin(dLon / 2) * Math.sin(dLon / 2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

        return 6371000 * c; // Earth radius in meters
    }

    /**
     * Snap waypoints to the road network using profile-aware snapping.
     *
     * @param waypoints          List of waypoints to snap
     * @param profile            Round-trip profile with snapping preferences
     * @param snappingPreference Preference pattern for edge selection
     * @param seed               Random seed for deterministic variation
     * @return List of snaps, or null if snapping failed
     */
    private List<Snap> snapWaypoints(List<GHPoint> waypoints, RoundTripProfile profile,
            SnappingPreference snappingPreference, long seed) {
        List<Snap> snaps = new ArrayList<>(waypoints.size());

        // First snap is the start (also used as end) - index 0
        Snap startSnap = snapper.findBestSnap(
                waypoints.get(0), profile, locationIndex, edgeFilter, encodedValueLookup,
                snappingPreference, seed, 0);
        if (startSnap == null || !startSnap.isValid()) {
            // Fallback to simple snap
            startSnap = locationIndex.findClosest(
                    waypoints.get(0).getLat(), waypoints.get(0).getLon(), edgeFilter);
            if (!startSnap.isValid()) {
                logger.warn("Could not snap start point");
                return null;
            }
        }
        snaps.add(startSnap);

        // Snap intermediate waypoints
        for (int i = 1; i < waypoints.size() - 1; i++) {
            GHPoint wp = waypoints.get(i);
            Snap snap = snapper.findBestSnap(wp, profile, locationIndex, edgeFilter, encodedValueLookup,
                    snappingPreference, seed, i);
            if (snap == null || !snap.isValid()) {
                // Fallback to simple snap
                snap = locationIndex.findClosest(wp.getLat(), wp.getLon(), edgeFilter);
                if (!snap.isValid()) {
                    logger.warn("Could not snap waypoint {}", i);
                    return null;
                }
            }
            snaps.add(snap);
        }

        // Last snap is back to start
        snaps.add(startSnap);

        return snaps;
    }

    /**
     * Calculate paths between all snapped waypoints.
     *
     * Note: We intentionally do NOT use AvoidEdgesWeighting here.
     * Each leg is routed independently so that waypoints can be
     * re-routed by the client to produce the same path.
     * Repetition is handled by scoring and waypoint adjustment, not route-time
     * avoidance.
     */
    private RouteCalculationResult calculateRoute(List<Snap> snaps,
            Function<List<Snap>, FlexiblePathCalculator> pathCalculatorFactory) {
        // Create path calculator with proper QueryGraph (factory handles this)
        FlexiblePathCalculator pathCalculator = pathCalculatorFactory.apply(snaps);

        List<Path> paths = new ArrayList<>();
        PointList wayPoints = new PointList(snaps.size(), graph.getNodeAccess().is3D());

        for (int i = 1; i < snaps.size(); i++) {
            Snap fromSnap = snaps.get(i - 1);
            Snap toSnap = snaps.get(i);

            // Get node IDs from snaps (these may be virtual nodes after QueryGraph
            // creation)
            int fromNode = fromSnap.getClosestNode();
            int toNode = toSnap.getClosestNode();

            if (fromNode < 0 || toNode < 0) {
                logger.warn("Invalid node IDs: from={}, to={}", fromNode, toNode);
                return null;
            }

            // Calculate path for this leg (independent routing, no edge avoidance)
            List<Path> legPaths = pathCalculator.calcPaths(fromNode, toNode, new EdgeRestrictions());
            if (legPaths.isEmpty()) {
                logger.warn("No path found for leg {} ({} -> {})", i, fromNode, toNode);
                return null;
            }

            Path path = legPaths.get(0);

            // Check if path was found
            if (!path.isFound()) {
                logger.warn("Path not found for leg {} ({} -> {})", i, fromNode, toNode);
                return null;
            }

            // Add waypoint coordinates from snaps (not from NodeAccess which doesn't know
            // virtual nodes)
            if (i == 1) {
                GHPoint fromPt = fromSnap.getSnappedPoint();
                wayPoints.add(fromPt.getLat(), fromPt.getLon(), Double.NaN);
            }
            GHPoint toPt = toSnap.getSnappedPoint();
            wayPoints.add(toPt.getLat(), toPt.getLon(), Double.NaN);

            paths.add(path);
        }

        if (paths.isEmpty()) {
            logger.warn("No paths calculated");
            return null;
        }

        return new RouteCalculationResult(paths, wayPoints);
    }

    /**
     * Build a ResponsePath from calculated paths.
     */
    private ResponsePath buildResponsePath(RouteCalculationResult result) {
        ResponsePath response = new ResponsePath();

        // Initialize points list - response.getPoints() returns immutable EMPTY by
        // default
        PointList allPoints = new PointList(100, graph.getNodeAccess().is3D());
        boolean firstPath = true;

        // Merge all leg paths
        for (Path path : result.paths) {
            response.setDistance(response.getDistance() + path.getDistance());
            response.setTime(response.getTime() + path.getTime());

            // Add points from path
            PointList points = path.calcPoints();
            if (firstPath) {
                // Add all points from first path
                for (int i = 0; i < points.size(); i++) {
                    allPoints.add(points.getLat(i), points.getLon(i), points.getEle(i));
                }
                firstPath = false;
            } else {
                // Skip first point to avoid duplicate at connection
                for (int i = 1; i < points.size(); i++) {
                    allPoints.add(points.getLat(i), points.getLon(i), points.getEle(i));
                }
            }
        }

        response.setPoints(allPoints);
        response.setWaypoints(result.wayPoints);

        return response;
    }

    /**
     * Get or create a round-trip profile for the given name.
     */
    private RoundTripProfile getOrCreateProfile(String profileName) {
        // In future: load from JSON config
        // For now: return default profiles
        if (profileName == null) {
            return RoundTripProfile.createGravelProfile();
        }
        String lower = profileName.toLowerCase();
        if (lower.contains("gravel")) {
            return RoundTripProfile.createGravelProfile();
        } else if (lower.contains("road")) {
            return RoundTripProfile.createRoadBikeProfile();
        } else if (lower.contains("mtb")) {
            return RoundTripProfile.createMtbProfile();
        }
        // Default to gravel profile
        return RoundTripProfile.createGravelProfile();
    }

    /**
     * Get available geometry shape names.
     */
    public Set<String> getAvailableShapes() {
        return Collections.unmodifiableSet(geometryStrategies.keySet());
    }

    /**
     * Get available fixer names.
     */
    public List<String> getAvailableFixers() {
        List<String> names = new ArrayList<>();
        for (RouteFixer fixer : fixers) {
            names.add(fixer.getName());
        }
        return names;
    }

    /**
     * Internal result holder for route calculation.
     */
    private static class RouteCalculationResult {
        final List<Path> paths;
        final PointList wayPoints;

        RouteCalculationResult(List<Path> paths, PointList wayPoints) {
            this.paths = paths;
            this.wayPoints = wayPoints;
        }
    }
}
