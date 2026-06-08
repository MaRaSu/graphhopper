package com.graphhopper.trailmap.roundtrip.normalization;

import com.carrotsearch.hppc.IntArrayList;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.graphhopper.ConvertRequest;
import com.graphhopper.ConvertResponse;
import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.ResponsePath;
import com.graphhopper.jackson.Jackson;
import com.graphhopper.config.Profile;
import com.graphhopper.routing.AlgorithmOptions;
import com.graphhopper.routing.DefaultWeightingFactory;
import com.graphhopper.routing.FlexiblePathCalculator;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.RoutingAlgorithmFactorySimple;
import com.graphhopper.routing.querygraph.QueryGraph;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.util.Parameters;
import com.graphhopper.util.PMap;
import com.graphhopper.util.PointList;
import com.graphhopper.util.details.PathDetail;
import com.graphhopper.util.shapes.GHPoint;
import org.junit.jupiter.api.*;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Integration test for WaypointNormalizer.
 *
 * Uses the existing graph-cache built by the server.
 * Run from graphhopper directory with:
 *   ./run-normalizer-test.sh
 */
public class WaypointNormalizerIntegrationTest {

    // Config file path (relative to graphhopper/core where mvn runs)
    private static final String CONFIG_FILE = "../trailmap-config.yml";

    private static GraphHopper hopper;
    private static boolean isInitialized = false;

    @BeforeAll
    static void setupGraphHopper() {
        System.out.println("=== WaypointNormalizer Integration Test ===");

        File configFile = new File(CONFIG_FILE);
        System.out.println("Config file: " + configFile.getAbsolutePath());
        System.out.println("Config exists: " + configFile.exists());

        if (!configFile.exists()) {
            System.err.println("ERROR: Config file not found at " + configFile.getAbsolutePath());
            isInitialized = false;
            return;
        }

        try {
            // Load config from YAML - need to extract the "graphhopper" section
            // since trailmap-config.yml uses dropwizard format with "graphhopper:" wrapper
            ObjectMapper mapper = Jackson.initObjectMapper(new ObjectMapper(new YAMLFactory()));
            var rootNode = mapper.readTree(configFile);
            var ghNode = rootNode.get("graphhopper");
            if (ghNode == null) {
                throw new IllegalStateException("Config file missing 'graphhopper:' section");
            }
            GraphHopperConfig config = mapper.treeToValue(ghNode, GraphHopperConfig.class);

            // Fix the graph location path - config is relative to graphhopper dir,
            // but test runs from graphhopper/core
            String graphLocation = config.getString("graph.location", "../data/graph-cache");
            String fixedGraphLocation = "../" + graphLocation;  // Add ../ to account for core subdir
            config.putObject("graph.location", fixedGraphLocation);

            System.out.println("Graph location (fixed): " + fixedGraphLocation);

            hopper = new GraphHopper();
            hopper.init(config);
            hopper.load();  // Just load existing graph, don't import

            isInitialized = true;
            System.out.println("GraphHopper loaded successfully");
            System.out.println("Base graph nodes: " + hopper.getBaseGraph().getNodes());
            System.out.println("Base graph edges: " + hopper.getBaseGraph().getEdges());
            System.out.println("Available profiles: " + hopper.getProfiles().stream().map(p -> p.getName()).toList());
        } catch (Exception e) {
            System.err.println("Failed to load GraphHopper: " + e.getMessage());
            e.printStackTrace();
            isInitialized = false;
        }
    }

    @AfterAll
    static void cleanup() {
        if (hopper != null) {
            hopper.close();
        }
        // Don't delete graph-cache - it's shared with the server!
    }

    @Test
    void testFinlandTwoPointRoute() {
        Assumptions.assumeTrue(isInitialized, "GraphHopper not initialized");

        // Finland test coordinates from user's app
        // API uses [lon, lat], GHPoint uses (lat, lon)
        GHPoint start = new GHPoint(61.500251515052696, 23.675591313614376);
        GHPoint end = new GHPoint(61.54892181915068, 23.329713703700946);

        System.out.println("\n=== Test: Finland Two Point Route ===");
        System.out.println("Start: " + start);
        System.out.println("End: " + end);

        // Route with both profiles
        RouteResult gravelRoute = routeWithProfile("gravel", Arrays.asList(start, end));
        RouteResult gravelExploreRoute = routeWithProfile("gravel_explore", Arrays.asList(start, end));

        assertNotNull(gravelRoute, "Gravel route should not be null");
        assertNotNull(gravelExploreRoute, "Gravel explore route should not be null");

        System.out.println("\nGravel (standard) route:");
        System.out.println("  Distance: " + gravelRoute.distance + "m");
        System.out.println("  Edges: " + gravelRoute.edges.size());
        System.out.println("  Edge IDs: " + edgesToString(gravelRoute.edges, 20));

        System.out.println("\nGravel explore route:");
        System.out.println("  Distance: " + gravelExploreRoute.distance + "m");
        System.out.println("  Edges: " + gravelExploreRoute.edges.size());
        System.out.println("  Edge IDs: " + edgesToString(gravelExploreRoute.edges, 20));

        // Compare edges
        EdgeMatcher matcher = new EdgeMatcher(hopper.getBaseGraph());
        boolean edgesMatch = matcher.edgesMatch(
            gravelExploreRoute.edges, gravelRoute.edges, null, null);

        System.out.println("\nEdges match: " + edgesMatch);

        if (!edgesMatch) {
            EdgeMatcher.MatchStats stats = matcher.calculateMatchStats(
                gravelExploreRoute.edges, gravelRoute.edges);
            System.out.println("Match stats: " + stats);
        }
    }

    @Test
    @Disabled("Enable and provide coordinates")
    void testNormalizationWithCoordinates() {
        Assumptions.assumeTrue(isInitialized, "GraphHopper not initialized");

        // TODO: Replace with actual test coordinates from user
        List<GHPoint> originalWaypoints = Arrays.asList(
            new GHPoint(0, 0),  // Replace with real coordinates
            new GHPoint(0, 0)   // Replace with real coordinates
        );

        testNormalization(originalWaypoints, "bike_explore", "bike");
    }

    /**
     * Main test method - tests normalization with given waypoints and profiles.
     */
    void testNormalization(List<GHPoint> originalWaypoints,
                           String explorationProfile,
                           String standardProfile) {
        System.out.println("\n=== Normalization Test ===");
        System.out.println("Original waypoints: " + originalWaypoints.size());
        for (int i = 0; i < originalWaypoints.size(); i++) {
            System.out.println("  [" + i + "] " + originalWaypoints.get(i));
        }
        System.out.println("Exploration profile: " + explorationProfile);
        System.out.println("Standard profile: " + standardProfile);

        // Step 1: Route with exploration profile (per leg)
        System.out.println("\n--- Step 1: Exploration Routing ---");

        for (int i = 0; i < originalWaypoints.size() - 1; i++) {
            GHPoint legStart = originalWaypoints.get(i);
            GHPoint legEnd = originalWaypoints.get(i + 1);

            RouteResult legRoute = routeWithProfile(explorationProfile,
                Arrays.asList(legStart, legEnd));

            if (legRoute != null) {
                System.out.println("Leg " + i + " -> " + (i + 1) + ":");
                System.out.println("  Distance: " + legRoute.distance + "m");
                System.out.println("  Edges: " + legRoute.edges.size());
                System.out.println("  Edge IDs: " + edgesToString(legRoute.edges, 10));
                // Note: We'd need to convert RouteResult to Path for the normalizer
                // For now, just log the edges
            } else {
                System.out.println("Leg " + i + " -> " + (i + 1) + ": FAILED");
            }
        }

        // Step 2: Route with standard profile (per leg)
        System.out.println("\n--- Step 2: Standard Routing (for comparison) ---");
        for (int i = 0; i < originalWaypoints.size() - 1; i++) {
            GHPoint legStart = originalWaypoints.get(i);
            GHPoint legEnd = originalWaypoints.get(i + 1);

            RouteResult legRoute = routeWithProfile(standardProfile,
                Arrays.asList(legStart, legEnd));

            if (legRoute != null) {
                System.out.println("Leg " + i + " -> " + (i + 1) + ":");
                System.out.println("  Distance: " + legRoute.distance + "m");
                System.out.println("  Edges: " + legRoute.edges.size());
                System.out.println("  Edge IDs: " + edgesToString(legRoute.edges, 10));
            } else {
                System.out.println("Leg " + i + " -> " + (i + 1) + ": FAILED");
            }
        }

        // Step 3: Compare edges for each leg
        System.out.println("\n--- Step 3: Edge Comparison ---");
        EdgeMatcher matcher = new EdgeMatcher(hopper.getBaseGraph());

        for (int i = 0; i < originalWaypoints.size() - 1; i++) {
            GHPoint legStart = originalWaypoints.get(i);
            GHPoint legEnd = originalWaypoints.get(i + 1);

            RouteResult exploreRoute = routeWithProfile(explorationProfile,
                Arrays.asList(legStart, legEnd));
            RouteResult standardRoute = routeWithProfile(standardProfile,
                Arrays.asList(legStart, legEnd));

            if (exploreRoute != null && standardRoute != null) {
                boolean match = matcher.edgesMatch(
                    exploreRoute.edges, standardRoute.edges, null, null);
                EdgeMatcher.MatchStats stats = matcher.calculateMatchStats(
                    exploreRoute.edges, standardRoute.edges);

                System.out.println("Leg " + i + " -> " + (i + 1) + ":");
                System.out.println("  Edges match: " + match);
                System.out.println("  Stats: " + stats);

                if (!match) {
                    // Detailed comparison
                    System.out.println("  Explore edges: " + edgesToString(exploreRoute.edges, 20));
                    System.out.println("  Standard edges: " + edgesToString(standardRoute.edges, 20));
                }
            }
        }
    }

    /**
     * Route between waypoints using specified profile.
     */
    private RouteResult routeWithProfile(String profile, List<GHPoint> waypoints) {
        try {
            GHRequest request = new GHRequest(waypoints)
                .setProfile(profile)
                .putHint(Parameters.Routing.INSTRUCTIONS, false)
                .setPathDetails(java.util.Collections.singletonList("edge_id"));

            GHResponse response = hopper.route(request);

            if (response.hasErrors()) {
                System.err.println("Routing error for " + profile + ": " + response.getErrors());
                return null;
            }

            ResponsePath path = response.getBest();

            // Extract edge IDs from path details
            IntArrayList edges = new IntArrayList();
            List<PathDetail> edgeIdDetails = path.getPathDetails().get("edge_id");
            if (edgeIdDetails != null) {
                for (PathDetail detail : edgeIdDetails) {
                    edges.add(((Number) detail.getValue()).intValue());
                }
            }

            return new RouteResult(path.getDistance(), path.getTime(), edges);
        } catch (Exception e) {
            System.err.println("Error routing with " + profile + ": " + e.getMessage());
            return null;
        }
    }

    private String edgesToString(IntArrayList edges, int maxCount) {
        StringBuilder sb = new StringBuilder();
        int count = Math.min(edges.size(), maxCount);
        for (int i = 0; i < count; i++) {
            if (i > 0) sb.append(", ");
            sb.append(edges.get(i));
        }
        if (edges.size() > maxCount) {
            sb.append(", ... (").append(edges.size() - maxCount).append(" more)");
        }
        return sb.toString();
    }

    /**
     * Result holder for route calculation.
     */
    private static class RouteResult {
        final double distance;
        final long time;
        final IntArrayList edges;

        RouteResult(double distance, long time, IntArrayList edges) {
            this.distance = distance;
            this.time = time;
            this.edges = edges;
        }
    }

    // ========================================================================
    // Test cases - to be filled in with real coordinates from user
    // ========================================================================

    @Test
    void testNormalizerWithFinlandRoute() {
        Assumptions.assumeTrue(isInitialized, "GraphHopper not initialized");

        // Finland test coordinates
        GHPoint start = new GHPoint(61.500251515052696, 23.675591313614376);
        GHPoint end = new GHPoint(61.54892181915068, 23.329713703700946);
        List<GHPoint> waypoints = Arrays.asList(start, end);

        System.out.println("\n=== Test: Normalizer with Finland Route ===");
        System.out.println("Start: " + start);
        System.out.println("End: " + end);

        // Get exploration route using internal APIs
        LocationIndex locationIndex = hopper.getLocationIndex();
        EdgeFilter edgeFilter = EdgeFilter.ALL_EDGES;

        // Snap waypoints
        List<Snap> snaps = new ArrayList<>();
        for (GHPoint point : waypoints) {
            Snap snap = locationIndex.findClosest(point.getLat(), point.getLon(), edgeFilter);
            assertTrue(snap.isValid(), "Should snap point: " + point);
            snaps.add(snap);
        }

        // Create path calculator factory for exploration profile
        Function<List<Snap>, FlexiblePathCalculator> explorePathCalculatorFactory = createPathCalculatorFactory("gravel_explore");
        Function<List<Snap>, FlexiblePathCalculator> standardPathCalculatorFactory = createPathCalculatorFactory("gravel");

        // Calculate exploration path
        List<Path> explorationPaths = new ArrayList<>();
        Path combinedExplorePath = null;

        QueryGraph queryGraph = QueryGraph.create(hopper.getBaseGraph(), snaps);
        FlexiblePathCalculator explorePathCalculator = explorePathCalculatorFactory.apply(snaps);

        for (int i = 0; i < snaps.size() - 1; i++) {
            int fromNode = snaps.get(i).getClosestNode();
            int toNode = snaps.get(i + 1).getClosestNode();

            List<Path> paths = explorePathCalculator.calcPaths(fromNode, toNode, new com.graphhopper.routing.EdgeRestrictions());
            assertFalse(paths.isEmpty(), "Should find exploration path for leg " + i);
            assertTrue(paths.get(0).isFound(), "Exploration path should be found for leg " + i);

            Path legPath = paths.get(0);
            explorationPaths.add(legPath);

            if (combinedExplorePath == null) {
                combinedExplorePath = legPath;
            } else {
                for (int j = 0; j < legPath.getEdgeCount(); j++) {
                    combinedExplorePath.addEdge(legPath.getEdges().get(j));
                }
            }
        }

        // Extract edge IDs using PathEdgeExtractor
        IntArrayList explorationEdges = PathEdgeExtractor.extractOriginalEdgeIds(combinedExplorePath);

        System.out.println("\nExploration route:");
        System.out.println("  Edges: " + explorationEdges.size());
        System.out.println("  First 20 edges: " + edgesToString(explorationEdges, 20));

        // Create normalizer (edge_key + public-route pattern)
        WaypointNormalizer normalizer = new WaypointNormalizer(hopper.getBaseGraph());

        // Run normalization: standard profile validated via the public routing API
        System.out.println("\n--- Running Normalization ---");
        NormalizationResult result = normalizer.normalize(
            waypoints, explorationPaths, "gravel", null, hopper::route);

        System.out.println("\nNormalization Result:");
        System.out.println("  Success: " + result.isSuccess());
        System.out.println("  Waypoints: " + result.getWaypoints().size());
        System.out.println("  Match percentage: " + result.getMatchPercentage() + "%");

        if (result.hasWaypoints()) {
            System.out.println("  Waypoint list:");
            for (int i = 0; i < result.getWaypoints().size(); i++) {
                GHPoint wp = result.getWaypoints().get(i);
                System.out.println("    [" + i + "] " + wp.getLat() + ", " + wp.getLon());
            }
        }

        // Assertions
        assertTrue(result.hasWaypoints(), "Should have waypoints");
        assertTrue(result.getWaypoints().size() >= 2, "Should have at least start and end");
        assertTrue(result.getWaypoints().size() <= 20,
            "Should have reasonable number of waypoints (<=20), got: " + result.getWaypoints().size());

        System.out.println("\n=== Test PASSED ===");
    }

    /**
     * Creates a path calculator factory for the given profile.
     * This mirrors how ExplorationRoundTripRouting creates path calculators.
     */
    private Function<List<Snap>, FlexiblePathCalculator> createPathCalculatorFactory(String profileName) {
        Profile profile = hopper.getProfile(profileName);
        assertNotNull(profile, "Profile should exist: " + profileName);

        return snaps -> {
            QueryGraph queryGraph = QueryGraph.create(hopper.getBaseGraph(), snaps);
            Weighting weighting = new DefaultWeightingFactory(
                hopper.getBaseGraph(), hopper.getEncodingManager()
            ).createWeighting(profile, new PMap(), false);

            AlgorithmOptions algoOpts = new AlgorithmOptions()
                .setAlgorithm(Parameters.Algorithms.ASTAR_BI);

            return new FlexiblePathCalculator(
                queryGraph,
                new RoutingAlgorithmFactorySimple(),
                weighting,
                algoOpts);
        };
    }

    /**
     * DIAGNOSTIC TEST 1: Same profile for reference and target.
     * Routes A→B with same profile twice. Edge IDs must match 100%.
     * This validates data structures and comparison logic work correctly.
     */
    @Test
    void testDiagnostic_SameProfileShouldMatch() {
        Assumptions.assumeTrue(isInitialized, "GraphHopper not initialized");

        GHPoint start = new GHPoint(61.500251515052696, 23.675591313614376);
        GHPoint end = new GHPoint(61.54892181915068, 23.329713703700946);
        List<GHPoint> waypoints = Arrays.asList(start, end);

        System.out.println("\n=== DIAGNOSTIC TEST 1: Same Profile Should Match ===");
        System.out.println("Using 'gravel' profile for both reference and target");

        // Route 1: Reference
        RouteResult route1 = routeWithProfile("gravel", waypoints);
        assertNotNull(route1, "Route 1 should succeed");

        // Route 2: Target (same profile, same waypoints)
        RouteResult route2 = routeWithProfile("gravel", waypoints);
        assertNotNull(route2, "Route 2 should succeed");

        System.out.println("\nRoute 1 (reference):");
        System.out.println("  Distance: " + route1.distance + "m");
        System.out.println("  Edges: " + route1.edges.size());
        System.out.println("  First 10 edges: " + edgesToString(route1.edges, 10));

        System.out.println("\nRoute 2 (target):");
        System.out.println("  Distance: " + route2.distance + "m");
        System.out.println("  Edges: " + route2.edges.size());
        System.out.println("  First 10 edges: " + edgesToString(route2.edges, 10));

        // Compare edge counts
        assertEquals(route1.edges.size(), route2.edges.size(),
            "Edge counts should be identical");

        // Compare all edge IDs
        boolean allMatch = true;
        for (int i = 0; i < route1.edges.size(); i++) {
            if (route1.edges.get(i) != route2.edges.get(i)) {
                System.out.println("  MISMATCH at index " + i + ": " +
                    route1.edges.get(i) + " vs " + route2.edges.get(i));
                allMatch = false;
            }
        }

        System.out.println("\nAll edges match: " + allMatch);
        assertTrue(allMatch, "All edge IDs should be identical for same profile");

        // Also test EdgeMatcher
        EdgeMatcher matcher = new EdgeMatcher(hopper.getBaseGraph());
        boolean matcherResult = matcher.edgesMatch(route1.edges, route2.edges, null, null);
        System.out.println("EdgeMatcher.edgesMatch(): " + matcherResult);
        assertTrue(matcherResult, "EdgeMatcher should report match for identical edges");

        System.out.println("\n=== DIAGNOSTIC TEST 1: PASSED ===");
    }

    /**
     * DIAGNOSTIC TEST 2: Different profiles for reference and target.
     * Shows how exploration vs standard profiles differ.
     */
    @Test
    void testDiagnostic_DifferentProfilesComparison() {
        Assumptions.assumeTrue(isInitialized, "GraphHopper not initialized");

        GHPoint start = new GHPoint(61.500251515052696, 23.675591313614376);
        GHPoint end = new GHPoint(61.54892181915068, 23.329713703700946);
        List<GHPoint> waypoints = Arrays.asList(start, end);

        System.out.println("\n=== DIAGNOSTIC TEST 2: Different Profiles Comparison ===");

        // Reference: exploration profile
        RouteResult exploreRoute = routeWithProfile("gravel_explore", waypoints);
        assertNotNull(exploreRoute, "Explore route should succeed");

        // Target: standard profile
        RouteResult standardRoute = routeWithProfile("gravel", waypoints);
        assertNotNull(standardRoute, "Standard route should succeed");

        System.out.println("\nExploration route (gravel_explore):");
        System.out.println("  Distance: " + exploreRoute.distance + "m");
        System.out.println("  Edges: " + exploreRoute.edges.size());
        System.out.println("  First 15 edges: " + edgesToString(exploreRoute.edges, 15));

        System.out.println("\nStandard route (gravel):");
        System.out.println("  Distance: " + standardRoute.distance + "m");
        System.out.println("  Edges: " + standardRoute.edges.size());
        System.out.println("  First 15 edges: " + edgesToString(standardRoute.edges, 15));

        // Find first divergence point
        int firstDivergence = -1;
        int minLen = Math.min(exploreRoute.edges.size(), standardRoute.edges.size());
        for (int i = 0; i < minLen; i++) {
            if (exploreRoute.edges.get(i) != standardRoute.edges.get(i)) {
                firstDivergence = i;
                break;
            }
        }

        if (firstDivergence >= 0) {
            System.out.println("\nFirst divergence at edge index: " + firstDivergence);
            System.out.println("  Explore edge: " + exploreRoute.edges.get(firstDivergence));
            System.out.println("  Standard edge: " + standardRoute.edges.get(firstDivergence));
            if (firstDivergence > 0) {
                System.out.println("  Previous edge (both): " + exploreRoute.edges.get(firstDivergence - 1));
            }
        } else {
            System.out.println("\nNo divergence found - routes are identical!");
        }

        // Count matching edges
        int matchCount = 0;
        for (int i = 0; i < minLen; i++) {
            if (exploreRoute.edges.get(i) == standardRoute.edges.get(i)) {
                matchCount++;
            }
        }
        System.out.println("\nMatching edges (in sequence): " + matchCount + "/" + minLen);

        System.out.println("\n=== DIAGNOSTIC TEST 2: COMPLETE ===");
    }

    /**
     * DIAGNOSTIC TEST 3: Route A→M where M is a point from exploration route.
     * Pick a waypoint M close to start on exploration route, route A→M with standard,
     * check if we can get matching edges for initial segment.
     */
    @Test
    void testDiagnostic_ShortSegmentComparison() {
        Assumptions.assumeTrue(isInitialized, "GraphHopper not initialized");

        GHPoint start = new GHPoint(61.500251515052696, 23.675591313614376);
        GHPoint end = new GHPoint(61.54892181915068, 23.329713703700946);
        List<GHPoint> fullWaypoints = Arrays.asList(start, end);

        System.out.println("\n=== DIAGNOSTIC TEST 3: Short Segment Comparison ===");

        // Get exploration route
        RouteResult exploreRoute = routeWithProfile("gravel_explore", fullWaypoints);
        assertNotNull(exploreRoute, "Explore route should succeed");

        System.out.println("\nFull exploration route: " + exploreRoute.edges.size() + " edges");

        // Get the route path (geometry) to pick intermediate waypoint
        GHRequest request = new GHRequest(fullWaypoints)
            .setProfile("gravel_explore")
            .putHint(Parameters.Routing.INSTRUCTIONS, false);
        GHResponse response = hopper.route(request);
        assertFalse(response.hasErrors(), "Should get exploration route");

        PointList points = response.getBest().getPoints();
        System.out.println("Route path has " + points.size() + " points");

        // Test with M at various positions along the route
        int[] testIndices = {10, 50, 100, 200};  // positions along route path

        for (int pointIdx : testIndices) {
            if (pointIdx >= points.size()) continue;

            GHPoint waypointM = new GHPoint(points.getLat(pointIdx), points.getLon(pointIdx));
            List<GHPoint> segmentWaypoints = Arrays.asList(start, waypointM);

            System.out.println("\n--- Testing M at point index " + pointIdx + " ---");
            System.out.println("M coordinates: " + waypointM.getLat() + ", " + waypointM.getLon());

            // Route A→M with exploration profile (reference for this segment)
            RouteResult exploreSegment = routeWithProfile("gravel_explore", segmentWaypoints);

            // Route A→M with standard profile (candidate)
            RouteResult standardSegment = routeWithProfile("gravel", segmentWaypoints);

            if (exploreSegment == null || standardSegment == null) {
                System.out.println("  Failed to route segment");
                continue;
            }

            System.out.println("  Explore segment: " + exploreSegment.edges.size() + " edges");
            System.out.println("    First 10: " + edgesToString(exploreSegment.edges, 10));
            System.out.println("  Standard segment: " + standardSegment.edges.size() + " edges");
            System.out.println("    First 10: " + edgesToString(standardSegment.edges, 10));

            // Check if they match
            EdgeMatcher matcher = new EdgeMatcher(hopper.getBaseGraph());
            boolean matches = matcher.edgesMatch(exploreSegment.edges, standardSegment.edges, null, null);
            System.out.println("  EdgeMatcher result: " + matches);

            // Find first difference
            int firstDiff = -1;
            int minLen = Math.min(exploreSegment.edges.size(), standardSegment.edges.size());
            for (int i = 0; i < minLen; i++) {
                if (exploreSegment.edges.get(i) != standardSegment.edges.get(i)) {
                    firstDiff = i;
                    break;
                }
            }
            if (firstDiff >= 0) {
                System.out.println("  First difference at index " + firstDiff + ": " +
                    exploreSegment.edges.get(firstDiff) + " vs " + standardSegment.edges.get(firstDiff));
            } else if (exploreSegment.edges.size() != standardSegment.edges.size()) {
                System.out.println("  Same edges but different lengths");
            } else {
                System.out.println("  IDENTICAL edges!");
            }
        }

        System.out.println("\n=== DIAGNOSTIC TEST 3: COMPLETE ===");
    }

    /**
     * Regression for the AutoRoute /convert HTTP 400 after the gravel profile gained turn costs.
     * Uses the exact failing payload (lon,lat in the API → GHPoint(lat,lon)). Before the edge_key
     * refactor this threw "Weightings supporting turn costs cannot be used with node-based
     * traversal mode"; now /convert routes Stages B/C through the public API which sets edge-based
     * traversal for turn-cost profiles.
     */
    @Test
    void testConvertWithGravelTurnCostProfile() {
        Assumptions.assumeTrue(isInitialized, "GraphHopper not initialized");

        List<GHPoint> waypoints = Arrays.asList(
            new GHPoint(60.221024, 24.643975),
            new GHPoint(60.211171, 24.76993),
            new GHPoint(60.294155, 24.683643),
            new GHPoint(60.285106, 24.599902),
            new GHPoint(60.263037, 24.631957),
            new GHPoint(60.221024, 24.643975));

        ConvertRequest request = new ConvertRequest("gravel", waypoints);
        ConvertResponse response = hopper.convert(request);

        System.out.println("\n=== Test: /convert with gravel (turn-cost) profile ===");
        System.out.println("Response: " + response);

        assertFalse(response.hasErrors(),
            "convert must not error for a turn-cost profile: " + response.getErrors());
        assertTrue(response.getNormalizedWaypoints().size() >= 2,
            "should produce at least start+end normalized waypoints");
        assertTrue(response.getPoints().size() > 0, "should produce route geometry");
    }

    @Test
    @Disabled("Waiting for test coordinates")
    void testCase1_ShortRoute() {
        // TODO: Add test coordinates from user
    }

    @Test
    @Disabled("Waiting for test coordinates")
    void testCase2_MediumRoute() {
        // TODO: Add test coordinates from user
    }
}
