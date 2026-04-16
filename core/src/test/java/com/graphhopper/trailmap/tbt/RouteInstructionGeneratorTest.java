package com.graphhopper.trailmap.tbt;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.PropertyNamingStrategies;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.ResponsePath;
import com.graphhopper.config.Profile;
import com.graphhopper.jackson.GraphHopperModule;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.json.Statement;
import com.graphhopper.util.CustomModel;
import com.graphhopper.routing.ev.BooleanEncodedValue;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.ev.RoadClass;
import com.graphhopper.routing.ev.VehicleAccess;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.trailmap.shared.PredictedHighway;
import com.graphhopper.trailmap.TrailmapGraphHopper;
import com.graphhopper.trailmap.shared.TrailmapImportRegistry;
import com.graphhopper.routing.InstructionsHelper;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.util.*;
import com.graphhopper.util.details.PathDetail;
import com.graphhopper.util.shapes.GHPoint;
import org.junit.jupiter.api.*;

import java.io.File;
import java.io.FileInputStream;
import java.util.*;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Integration test for RouteInstructionGenerator using the Finland graph data.
 * <p>
 * Requires the pre-built graph cache at ../../data/graph-cache (built from finland_3.osm.pbf).
 * Skipped automatically if the graph cache is not available.
 */
public class RouteInstructionGeneratorTest {

    // Paths relative to core/ (maven working directory)
    private static final String GRAPH_LOCATION = "../../data/graph-cache";
    private static final String OSM_FILE = "../../data/finland_3.osm.pbf";
    private static final String CONFIG_FILE = "../trailmap-config.yml";

    // Test route: Tampere area
    private static final double START_LAT = 61.500752;
    private static final double START_LNG = 23.684245;
    private static final double END_LAT = 61.503679;
    private static final double END_LNG = 23.698272;

    private static GraphHopper hopper;

    @BeforeAll
    static void setup() throws Exception {
        File graphDir = new File(GRAPH_LOCATION);
        Assumptions.assumeTrue(graphDir.exists() && graphDir.isDirectory(),
                "Graph cache not found at " + graphDir.getAbsolutePath() + " — skipping integration tests");

        // Parse the Dropwizard YAML config and extract the 'graphhopper' subtree
        ObjectMapper yamlMapper = new ObjectMapper(new YAMLFactory());
        yamlMapper.setPropertyNamingStrategy(PropertyNamingStrategies.SNAKE_CASE);
        yamlMapper.registerModule(new GraphHopperModule());

        JsonNode root = yamlMapper.readTree(new FileInputStream(new File(CONFIG_FILE)));
        JsonNode ghNode = root.get("graphhopper");
        assertNotNull(ghNode, "trailmap-config.yml must have a 'graphhopper' top-level key");

        GraphHopperConfig config = yamlMapper.treeToValue(ghNode, GraphHopperConfig.class);

        // Override paths for the test environment
        config.putObject("graph.location", GRAPH_LOCATION);
        config.putObject("datareader.file", OSM_FILE);

        hopper = new TrailmapGraphHopper();
        hopper.setImportRegistry(new TrailmapImportRegistry());
        hopper.setAllowWrites(false);
        hopper.init(config);
        hopper.importOrLoad();
    }

    @AfterAll
    static void teardown() {
        if (hopper != null) {
            hopper.close();
        }
    }

    /**
     * Sanity check: standard GH routing works for our test coordinates.
     */
    @Test
    void testStandardRoutingWorks() {
        GHRequest req = new GHRequest(START_LAT, START_LNG, END_LAT, END_LNG)
                .setProfile("gravel_mtb");
        GHResponse rsp = hopper.route(req);
        assertFalse(rsp.hasErrors(), "Routing failed: " + rsp.getErrors());

        ResponsePath path = rsp.getBest();
        assertTrue(path.getDistance() > 0, "Route should have positive distance");

        InstructionList instructions = path.getInstructions();
        assertNotNull(instructions);
        assertTrue(instructions.size() > 0, "Should have at least one instruction");

        System.out.println("Standard routing: " + instructions.size() + " instructions, "
                + Math.round(path.getDistance()) + "m");
        for (Instruction instr : instructions) {
            System.out.println("  sign=" + instr.getSign() + " dist=" + Math.round(instr.getDistance())
                    + "m name=\"" + instr.getName() + "\"");
        }
    }

    /**
     * Verify that edge_id path details can be extracted from a route.
     */
    @Test
    void testEdgeIdExtraction() {
        GHRequest req = new GHRequest(START_LAT, START_LNG, END_LAT, END_LNG)
                .setProfile("gravel_mtb");
        req.setPathDetails(List.of("edge_id"));
        req.putHint("instructions", false);
        req.putHint("calc_points", true);

        GHResponse rsp = hopper.route(req);
        assertFalse(rsp.hasErrors(), "Routing failed: " + rsp.getErrors());

        List<PathDetail> edgeDetails = rsp.getBest().getPathDetails().get("edge_id");
        assertNotNull(edgeDetails, "edge_id details should be present");
        assertFalse(edgeDetails.isEmpty(), "Should have at least one edge");

        System.out.println("Edge IDs (" + edgeDetails.size() + " edges): ");
        for (PathDetail d : edgeDetails) {
            System.out.print(d.getValue() + " ");
        }
        System.out.println();
    }

    /**
     * Core test: RouteInstructionGenerator produces instructions for a simple single-segment route.
     */
    @Test
    void testSingleSegmentInstructionGeneration() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = buildSingleSegmentRequest(
                "gravel_mtb", "gravel", "fi");

        RouteInstructionGenerator.Result result = generator.generate(request);

        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");
        assertNotNull(result.polyline);
        assertTrue(result.polyline.size() > 0, "Should produce a polyline");

        // The last instruction should be FINISH
        Instruction lastInstr = result.instructions.get(result.instructions.size() - 1);
        assertEquals(Instruction.FINISH, lastInstr.getSign(), "Last instruction should be FINISH");

        System.out.println("\n=== RouteInstructionGenerator output ===");
        System.out.println("Instructions: " + result.instructions.size());
        System.out.println("Polyline points: " + result.polyline.size());
        int pointsIndex = 0;
        for (Instruction instr : result.instructions) {
            int end = pointsIndex + instr.getLength();
            System.out.printf("  sign=%d interval=[%d,%d] dist=%.1fm name=\"%s\" text=\"%s\"%n",
                    instr.getSign(), pointsIndex, end,
                    instr.getDistance(), instr.getName(),
                    instr.getTurnDescription(result.instructions.getTr()));
            pointsIndex = end;
        }
    }

    /**
     * Compare our instruction output with standard GH instructions for the same route.
     */
    @Test
    void testCompareWithStandardInstructions() {
        // Get standard GH instructions
        GHRequest stdReq = new GHRequest(START_LAT, START_LNG, END_LAT, END_LNG)
                .setProfile("gravel_mtb");
        GHResponse stdRsp = hopper.route(stdReq);
        assertFalse(stdRsp.hasErrors());
        InstructionList stdInstructions = stdRsp.getBest().getInstructions();

        // Get our instructions (use gravel_mtb for both routing and instruction for fair comparison)
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = buildSingleSegmentRequest(
                "gravel_mtb", "gravel_mtb", "fi");

        RouteInstructionGenerator.Result result = generator.generate(request);

        System.out.println("\n=== Comparison ===");
        System.out.println("Standard GH: " + stdInstructions.size() + " instructions, "
                + Math.round(stdRsp.getBest().getDistance()) + "m");
        System.out.println("Our generator: " + result.instructions.size() + " instructions");

        System.out.println("\nStandard GH instructions:");
        for (Instruction instr : stdInstructions) {
            System.out.printf("  sign=%d dist=%.1fm name=\"%s\"%n",
                    instr.getSign(), instr.getDistance(), instr.getName());
        }
        System.out.println("\nOur instructions:");
        double totalInstrDist = 0;
        for (Instruction instr : result.instructions) {
            System.out.printf("  sign=%d dist=%.1fm name=\"%s\"%n",
                    instr.getSign(), instr.getDistance(), instr.getName());
            totalInstrDist += instr.getDistance();
        }

        // After remapInstructionGeometry recalculates distances from the route polyline,
        // the sum of instruction distances should match the polyline distance (computed
        // from consecutive points in the full route polyline) within a small tolerance.
        double polylineDist = 0;
        PointList poly = result.polyline;
        for (int i = 0; i < poly.size() - 1; i++) {
            polylineDist += DistanceCalcEarth.DIST_EARTH.calcDist(
                    poly.getLat(i), poly.getLon(i), poly.getLat(i + 1), poly.getLon(i + 1));
        }
        System.out.printf("\nTotal instruction distance: %.1fm, Polyline distance: %.1fm, diff: %.1fm%n",
                totalInstrDist, polylineDist, Math.abs(totalInstrDist - polylineDist));
        assertEquals(polylineDist, totalInstrDist, 1.0,
                "Total instruction distance should match route polyline distance within 1m");
    }

    /**
     * Two-segment route: three waypoints, two followRoads segments with the same profile.
     * The second segment has an initial_heading.
     */
    @Test
    void testTwoSegmentInstructionGeneration() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = buildTwoSegmentRequest();

        RouteInstructionGenerator.Result result = generator.generate(request);

        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");
        assertNotNull(result.polyline);
        assertTrue(result.polyline.size() > 0, "Should produce a polyline");

        // The last instruction should be FINISH
        Instruction lastInstr = result.instructions.get(result.instructions.size() - 1);
        assertEquals(Instruction.FINISH, lastInstr.getSign(), "Last instruction should be FINISH");

        // There should be exactly one FINISH instruction (no intermediate FINISHes)
        long finishCount = 0;
        for (Instruction instr : result.instructions) {
            if (instr.getSign() == Instruction.FINISH) finishCount++;
        }
        assertEquals(1, finishCount, "Should have exactly one FINISH instruction");

        System.out.println("\n=== Two-segment RouteInstructionGenerator output ===");
        System.out.println("Instructions: " + result.instructions.size());
        System.out.println("Polyline points: " + result.polyline.size());
        int pointsIndex = 0;
        for (Instruction instr : result.instructions) {
            int end = pointsIndex + instr.getLength();
            System.out.printf("  sign=%d interval=[%d,%d] dist=%.1fm name=\"%s\" text=\"%s\"%n",
                    instr.getSign(), pointsIndex, end,
                    instr.getDistance(), instr.getName(),
                    instr.getTurnDescription(result.instructions.getTr()));
            pointsIndex = end;
        }
    }

    /**
     * Diagnostic test: compare per-segment polylines from standard GH routing with
     * the polyline from RouteInstructionGenerator for a multi-segment route.
     * <p>
     * Routes: (61.500143, 23.681118) -> (61.504726, 23.70125) -> (61.501035, 23.711962)
     * seg1: wp1->wp2 profile=gravel_mtb
     * seg2: wp2->wp3 profile=gravel_mtb initial_heading=113.88
     * instruction_profile=gravel, locale=fi
     */
    @Test
    void testMultiSegmentPolylineMatchesPerSegmentRouting() {
        // Waypoints
        double wp1Lat = 61.500143, wp1Lng = 23.681118;
        double wp2Lat = 61.504726, wp2Lng = 23.70125;
        double wp3Lat = 61.501035, wp3Lng = 23.711962;

        // --- Route each segment individually via standard GH routing ---
        // Use same settings as routeSection: edge_id path details, instructions=false
        // Segment 1: wp1 -> wp2
        GHRequest req1 = new GHRequest(wp1Lat, wp1Lng, wp2Lat, wp2Lng).setProfile("gravel_mtb");
        req1.putHint("calc_points", true);
        req1.putHint("instructions", false);
        req1.setPathDetails(List.of("edge_id"));
        GHResponse rsp1 = hopper.route(req1);
        assertFalse(rsp1.hasErrors(), "Seg1 routing failed: " + rsp1.getErrors());
        PointList seg1Polyline = rsp1.getBest().getPoints();

        // Compute heading at end of seg1 for seg2's initial_heading
        double seg1EndHeading = Double.NaN;
        if (seg1Polyline.size() >= 2) {
            seg1EndHeading = AngleCalc.ANGLE_CALC.calcAzimuth(
                    seg1Polyline.getLat(seg1Polyline.size() - 2), seg1Polyline.getLon(seg1Polyline.size() - 2),
                    seg1Polyline.getLat(seg1Polyline.size() - 1), seg1Polyline.getLon(seg1Polyline.size() - 1));
        }

        // Use snapped end point of seg1 as start of seg2 (what the real routing does)
        double seg2StartLat = seg1Polyline.getLat(seg1Polyline.size() - 1);
        double seg2StartLng = seg1Polyline.getLon(seg1Polyline.size() - 1);

        // Segment 2: snapped_end_of_seg1 -> wp3, with initial_heading=113.88
        GHRequest req2 = new GHRequest(seg2StartLat, seg2StartLng, wp3Lat, wp3Lng).setProfile("gravel_mtb");
        req2.putHint("calc_points", true);
        req2.putHint("instructions", false);
        req2.setPathDetails(List.of("edge_id"));
        req2.setHeadings(List.of(113.88, Double.NaN));
        GHResponse rsp2 = hopper.route(req2);
        assertFalse(rsp2.hasErrors(), "Seg2 routing failed: " + rsp2.getErrors());
        PointList seg2Polyline = rsp2.getBest().getPoints();

        // Build "expected" polyline by concatenating seg1 + seg2 (skip duplicate at boundary)
        PointList expectedPolyline = new PointList(seg1Polyline.size() + seg2Polyline.size(), seg1Polyline.is3D());
        for (int i = 0; i < seg1Polyline.size(); i++) {
            expectedPolyline.add(seg1Polyline.getLat(i), seg1Polyline.getLon(i),
                    seg1Polyline.is3D() ? seg1Polyline.getEle(i) : Double.NaN);
        }
        for (int i = 1; i < seg2Polyline.size(); i++) { // skip first point (duplicate)
            expectedPolyline.add(seg2Polyline.getLat(i), seg2Polyline.getLon(i),
                    seg2Polyline.is3D() ? seg2Polyline.getEle(i) : Double.NaN);
        }

        // --- Run the same request through RouteInstructionGenerator ---
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest instrRequest = buildThreeWaypointRequest(
                wp1Lat, wp1Lng, wp2Lat, wp2Lng, wp3Lat, wp3Lng, 113.88);

        RouteInstructionGenerator.Result result = generator.generate(instrRequest);

        PointList actualPolyline = result.polyline;

        // --- Print both polylines for comparison ---
        System.out.println("\n=== Multi-segment polyline comparison ===");
        System.out.println("Expected (per-segment routing): " + expectedPolyline.size() + " points");
        System.out.println("Actual (instruction generator): " + actualPolyline.size() + " points");

        System.out.println("\nExpected polyline (first 10 + last 5 points):");
        printPolyline(expectedPolyline, 10, 5);

        System.out.println("\nActual polyline (first 10 + last 5 points):");
        printPolyline(actualPolyline, 10, 5);

        // --- Compare start and end points ---
        System.out.println("\nStart point comparison:");
        System.out.printf("  Expected: (%.7f, %.7f)%n", expectedPolyline.getLat(0), expectedPolyline.getLon(0));
        System.out.printf("  Actual:   (%.7f, %.7f)%n", actualPolyline.getLat(0), actualPolyline.getLon(0));

        System.out.println("End point comparison:");
        System.out.printf("  Expected: (%.7f, %.7f)%n",
                expectedPolyline.getLat(expectedPolyline.size() - 1),
                expectedPolyline.getLon(expectedPolyline.size() - 1));
        System.out.printf("  Actual:   (%.7f, %.7f)%n",
                actualPolyline.getLat(actualPolyline.size() - 1),
                actualPolyline.getLon(actualPolyline.size() - 1));

        // Find seg1/seg2 boundary in expected polyline
        System.out.println("\nSegment boundary (end of seg1 / start of seg2):");
        System.out.printf("  Seg1 end: (%.7f, %.7f)%n",
                seg1Polyline.getLat(seg1Polyline.size() - 1),
                seg1Polyline.getLon(seg1Polyline.size() - 1));
        System.out.printf("  Seg2 start: (%.7f, %.7f)%n",
                seg2Polyline.getLat(0), seg2Polyline.getLon(0));

        // --- Assertions ---
        // Start point should match within ~1m (snapping tolerance)
        double startDist = DistanceCalcEarth.DIST_EARTH.calcDist(
                expectedPolyline.getLat(0), expectedPolyline.getLon(0),
                actualPolyline.getLat(0), actualPolyline.getLon(0));
        System.out.printf("Start point distance: %.2f m%n", startDist);
        assertTrue(startDist < 5.0, "Start points should be within 5m, but distance is " + startDist + "m");

        // End point should match within ~1m
        double endDist = DistanceCalcEarth.DIST_EARTH.calcDist(
                expectedPolyline.getLat(expectedPolyline.size() - 1),
                expectedPolyline.getLon(expectedPolyline.size() - 1),
                actualPolyline.getLat(actualPolyline.size() - 1),
                actualPolyline.getLon(actualPolyline.size() - 1));
        System.out.printf("End point distance: %.2f m%n", endDist);
        assertTrue(endDist < 5.0, "End points should be within 5m, but distance is " + endDist + "m");

        // Point count should be similar (within a small tolerance for boundary handling)
        int pointDiff = Math.abs(expectedPolyline.size() - actualPolyline.size());
        System.out.printf("Point count difference: %d%n", pointDiff);
        assertTrue(pointDiff <= 2, "Point counts should be within 2, but differ by " + pointDiff);

        // Instructions should still be valid
        assertTrue(result.instructions.size() > 0, "Should have instructions");
        Instruction lastInstr = result.instructions.get(result.instructions.size() - 1);
        assertEquals(Instruction.FINISH, lastInstr.getSign(), "Last instruction should be FINISH");

        // Print instructions
        System.out.println("\nInstructions:");
        int pointsIndex = 0;
        for (Instruction instr : result.instructions) {
            int end = pointsIndex + instr.getLength();
            System.out.printf("  sign=%d interval=[%d,%d] dist=%.1fm name=\"%s\"%n",
                    instr.getSign(), pointsIndex, end,
                    instr.getDistance(), instr.getName());
            pointsIndex = end;
        }

        // The instruction intervals should cover the full polyline.
        // FINISH has getLength()==0, so pointsIndex ends at polyline.size()-1
        // (the index of the last point).
        System.out.printf("Total polyline points: %d, last instruction interval end: %d%n",
                actualPolyline.size(), pointsIndex);
        assertEquals(actualPolyline.size() - 1, pointsIndex,
                "Instruction intervals should cover the full polyline (last interval end = polyline size - 1)");
    }

    private void printPolyline(PointList polyline, int headCount, int tailCount) {
        int size = polyline.size();
        int printHead = Math.min(headCount, size);
        for (int i = 0; i < printHead; i++) {
            System.out.printf("  [%d] (%.7f, %.7f)%n", i, polyline.getLat(i), polyline.getLon(i));
        }
        if (size > headCount + tailCount) {
            System.out.println("  ... (" + (size - headCount - tailCount) + " more points) ...");
        }
        int printTailStart = Math.max(printHead, size - tailCount);
        for (int i = printTailStart; i < size; i++) {
            System.out.printf("  [%d] (%.7f, %.7f)%n", i, polyline.getLat(i), polyline.getLon(i));
        }
    }

    /**
     * Three-segment route: four waypoints, three followRoads segments with the same profile.
     * The third segment has an initial_heading.
     * This reproduces a bug where edge chain extraction fails with:
     * "Edge X is not connected to node Y" due to duplicate consecutive edge IDs
     * at via-point leg boundaries in multi-leg GH routes.
     */
    @Test
    void testThreeSegmentInstructionGeneration() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = buildThreeSegmentRequest();

        RouteInstructionGenerator.Result result = generator.generate(request);

        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");
        assertNotNull(result.polyline);
        assertTrue(result.polyline.size() > 0, "Should produce a polyline");

        // The last instruction should be FINISH
        Instruction lastInstr = result.instructions.get(result.instructions.size() - 1);
        assertEquals(Instruction.FINISH, lastInstr.getSign(), "Last instruction should be FINISH");

        // There should be exactly one FINISH instruction
        long finishCount = 0;
        for (Instruction instr : result.instructions) {
            if (instr.getSign() == Instruction.FINISH) finishCount++;
        }
        assertEquals(1, finishCount, "Should have exactly one FINISH instruction");

        System.out.println("\n=== Three-segment RouteInstructionGenerator output ===");
        System.out.println("Instructions: " + result.instructions.size());
        System.out.println("Polyline points: " + result.polyline.size());
        int pointsIndex = 0;
        for (Instruction instr : result.instructions) {
            int end = pointsIndex + instr.getLength();
            System.out.printf("  sign=%d interval=[%d,%d] dist=%.1fm name=\"%s\" text=\"%s\"%n",
                    instr.getSign(), pointsIndex, end,
                    instr.getDistance(), instr.getName(),
                    instr.getTurnDescription(result.instructions.getTr()));
            pointsIndex = end;
        }
    }

    /**
     * Validates that heading_penalty works correctly with custom models.
     * Uses coordinates that exposed a routing discrepancy.
     * Routes two segments via the instruction generator and also individually
     * via standard GH routing, then compares distances.
     */
    @Test
    void testHeadingPenaltyRouting() {
        // Waypoints
        double wp1Lat = 61.577723, wp1Lng = 23.277481;
        double wp2Lat = 61.602294, wp2Lng = 23.24386;
        double wp3Lat = 61.637615, wp3Lng = 23.315964;

        // Build the custom model with priority rules
        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_surface == ASPHALT_OR_UNPAVED || predicted_surface == ASPHALT",
                Statement.Op.MULTIPLY, "0.8"));
        cm.addToPriority(Statement.If("predicted_highway == CYCLEWAY",
                Statement.Op.MULTIPLY, "1.3"));

        // --- Build instruction request with two segments ---
        TrailmapInstructionRequest instrRequest = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint w1 = new TrailmapInstructionRequest.Waypoint();
        w1.setId("hp_wp1");
        TrailmapInstructionRequest.Coordinates c1 = new TrailmapInstructionRequest.Coordinates();
        c1.setLat(wp1Lat);
        c1.setLng(wp1Lng);
        w1.setCoordinates(c1);

        TrailmapInstructionRequest.Waypoint w2 = new TrailmapInstructionRequest.Waypoint();
        w2.setId("hp_wp2");
        TrailmapInstructionRequest.Coordinates c2 = new TrailmapInstructionRequest.Coordinates();
        c2.setLat(wp2Lat);
        c2.setLng(wp2Lng);
        w2.setCoordinates(c2);

        TrailmapInstructionRequest.Waypoint w3 = new TrailmapInstructionRequest.Waypoint();
        w3.setId("hp_wp3");
        TrailmapInstructionRequest.Coordinates c3 = new TrailmapInstructionRequest.Coordinates();
        c3.setLat(wp3Lat);
        c3.setLng(wp3Lng);
        w3.setCoordinates(c3);

        instrRequest.setWaypoints(List.of(w1, w2, w3));
        instrRequest.setSnapPreventions(List.of("ferry"));

        // Segment 1: wp1 -> wp2, profile=gravel, custom_model, no heading
        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("hp_wp1");
        seg1.setEnd("hp_wp2");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("gravel");
        seg1.setCustomModel(cm);

        // Segment 2: wp2 -> wp3, profile=gravel, custom_model, initial_heading + heading_penalty
        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("hp_wp2");
        seg2.setEnd("hp_wp3");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2.setProfile("gravel");
        seg2.setCustomModel(cm);
        seg2.setInitialHeading(312.1334920719508);
        seg2.setHeadingPenalty(60.0);

        instrRequest.setSegments(List.of(seg1, seg2));
        instrRequest.setInstructionProfile("gravel");
        instrRequest.setLocale("fi");

        // --- Run through RouteInstructionGenerator ---
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        RouteInstructionGenerator.Result result = generator.generate(instrRequest);
        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        // Compute total instruction distance
        double totalInstrDist = 0;
        for (Instruction instr : result.instructions) {
            totalInstrDist += instr.getDistance();
        }

        // --- Route each segment individually via standard GH routing ---
        // Segment 1: wp1 -> wp2
        GHRequest req1 = new GHRequest(wp1Lat, wp1Lng, wp2Lat, wp2Lng).setProfile("gravel");
        req1.setCustomModel(cm);
        req1.setSnapPreventions(List.of("ferry"));
        req1.putHint("heading_penalty", 60);
        GHResponse rsp1 = hopper.route(req1);
        assertFalse(rsp1.hasErrors(), "Seg1 standard routing failed: " + rsp1.getErrors());
        double seg1Dist = rsp1.getBest().getDistance();

        // Segment 2: wp2 -> wp3 with heading
        GHRequest req2 = new GHRequest(wp2Lat, wp2Lng, wp3Lat, wp3Lng).setProfile("gravel");
        req2.setCustomModel(cm);
        req2.setSnapPreventions(List.of("ferry"));
        req2.putHint("heading_penalty", 60);
        req2.setHeadings(List.of(312.13, Double.NaN));
        GHResponse rsp2 = hopper.route(req2);
        assertFalse(rsp2.hasErrors(), "Seg2 standard routing failed: " + rsp2.getErrors());
        double seg2Dist = rsp2.getBest().getDistance();

        double totalStdDist = seg1Dist + seg2Dist;

        // --- Print comparison ---
        System.out.println("\n=== Heading Penalty Routing Comparison ===");
        System.out.println("Standard GH seg1 distance: " + Math.round(seg1Dist) + "m");
        System.out.println("Standard GH seg2 distance: " + Math.round(seg2Dist) + "m");
        System.out.println("Standard GH total distance: " + Math.round(totalStdDist) + "m");
        System.out.println("Instruction generator total distance: " + Math.round(totalInstrDist) + "m");
        System.out.println("Difference: " + Math.round(Math.abs(totalStdDist - totalInstrDist)) + "m");

        System.out.println("\nInstructions (" + result.instructions.size() + "):");
        int pointsIndex = 0;
        for (Instruction instr : result.instructions) {
            int end = pointsIndex + instr.getLength();
            System.out.printf("  sign=%d interval=[%d,%d] dist=%.1fm name=\"%s\"%n",
                    instr.getSign(), pointsIndex, end,
                    instr.getDistance(), instr.getName());
            pointsIndex = end;
        }

        // Generous tolerance - just verify they are in the same ballpark (within 50m)
        assertEquals(totalStdDist, totalInstrDist, 50.0,
                "Total distances should be within 50m. Standard: " + Math.round(totalStdDist)
                        + "m, Instructions: " + Math.round(totalInstrDist) + "m");

        // Basic structural assertions
        Instruction lastInstr = result.instructions.get(result.instructions.size() - 1);
        assertEquals(Instruction.FINISH, lastInstr.getSign(), "Last instruction should be FINISH");

        long finishCount = 0;
        for (Instruction instr : result.instructions) {
            if (instr.getSign() == Instruction.FINISH) finishCount++;
        }
        assertEquals(1, finishCount, "Should have exactly one FINISH instruction");
    }

    /**
     * Diagnostic test for U-turn at segment boundary.
     * Routes two segments individually via standard GH (with instructions) to see what GH
     * generates for each, then runs through RouteInstructionGenerator and compares.
     * Goal: identify why a U-turn instruction at the segment 2 boundary is missing.
     */
    @Test
    void testUturnAtBoundaryDiagnostic() {
        // Waypoints
        double wp1Lat = 61.577236, wp1Lng = 23.278753;
        double wp2Lat = 61.602494, wp2Lng = 23.243388;
        double wp3Lat = 61.636935, wp3Lng = 23.257746;

        // Build the custom model
        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_surface == ASPHALT_OR_UNPAVED || predicted_surface == ASPHALT",
                Statement.Op.MULTIPLY, "0.8"));
        cm.addToPriority(Statement.If("predicted_highway == CYCLEWAY",
                Statement.Op.MULTIPLY, "1.3"));

        // ========== STEP 1: Route seg1 with standard GH (with instructions) ==========
        System.out.println("\n========== U-TURN DIAGNOSTIC ==========");
        System.out.println("\n--- Segment 1: Standard GH routing with instructions ---");
        GHRequest req1 = new GHRequest(wp1Lat, wp1Lng, wp2Lat, wp2Lng).setProfile("gravel");
        req1.setCustomModel(cm);
        req1.setSnapPreventions(List.of("ferry"));
        GHResponse rsp1 = hopper.route(req1);
        assertFalse(rsp1.hasErrors(), "Seg1 routing failed: " + rsp1.getErrors());

        InstructionList seg1Instructions = rsp1.getBest().getInstructions();
        PointList seg1Polyline = rsp1.getBest().getPoints();
        System.out.println("Seg1 distance: " + Math.round(rsp1.getBest().getDistance()) + "m");
        System.out.println("Seg1 instructions (" + seg1Instructions.size() + "):");
        for (int i = 0; i < seg1Instructions.size(); i++) {
            Instruction instr = seg1Instructions.get(i);
            System.out.printf("  [%d] sign=%d (%s) dist=%.1fm name=\"%s\" text=\"%s\"%n",
                    i, instr.getSign(), signName(instr.getSign()),
                    instr.getDistance(), instr.getName(),
                    instr.getTurnDescription(seg1Instructions.getTr()));
        }

        // Compute heading at end of seg1
        double seg1EndHeading = Double.NaN;
        if (seg1Polyline.size() >= 2) {
            seg1EndHeading = AngleCalc.ANGLE_CALC.calcAzimuth(
                    seg1Polyline.getLat(seg1Polyline.size() - 2), seg1Polyline.getLon(seg1Polyline.size() - 2),
                    seg1Polyline.getLat(seg1Polyline.size() - 1), seg1Polyline.getLon(seg1Polyline.size() - 1));
        }
        System.out.printf("Seg1 end heading (computed): %.5f%n", seg1EndHeading);
        System.out.printf("Seg1 end point: (%.7f, %.7f)%n",
                seg1Polyline.getLat(seg1Polyline.size() - 1),
                seg1Polyline.getLon(seg1Polyline.size() - 1));

        // ========== STEP 2: Route seg2 with standard GH (with instructions) ==========
        // Use snapped end of seg1 as start, and the specified heading
        double seg2StartLat = seg1Polyline.getLat(seg1Polyline.size() - 1);
        double seg2StartLng = seg1Polyline.getLon(seg1Polyline.size() - 1);
        double seg2Heading = 311.76719918378234;

        System.out.println("\n--- Segment 2: Standard GH routing with instructions ---");
        System.out.printf("Seg2 start: (%.7f, %.7f), heading=%.5f, heading_penalty=60.0%n",
                seg2StartLat, seg2StartLng, seg2Heading);
        GHRequest req2 = new GHRequest(seg2StartLat, seg2StartLng, wp3Lat, wp3Lng).setProfile("gravel");
        req2.setCustomModel(cm);
        req2.setSnapPreventions(List.of("ferry"));
        req2.setHeadings(List.of(seg2Heading, Double.NaN));
        req2.putHint("heading_penalty", 60.0);
        GHResponse rsp2 = hopper.route(req2);
        assertFalse(rsp2.hasErrors(), "Seg2 routing failed: " + rsp2.getErrors());

        InstructionList seg2Instructions = rsp2.getBest().getInstructions();
        PointList seg2Polyline = rsp2.getBest().getPoints();
        System.out.println("Seg2 distance: " + Math.round(rsp2.getBest().getDistance()) + "m");
        System.out.println("Seg2 instructions (" + seg2Instructions.size() + "):");
        for (int i = 0; i < seg2Instructions.size(); i++) {
            Instruction instr = seg2Instructions.get(i);
            System.out.printf("  [%d] sign=%d (%s) dist=%.1fm name=\"%s\" text=\"%s\"%n",
                    i, instr.getSign(), signName(instr.getSign()),
                    instr.getDistance(), instr.getName(),
                    instr.getTurnDescription(seg2Instructions.getTr()));
        }

        // ========== STEP 3: Also route seg2 WITHOUT heading (to see the "natural" route) ==========
        System.out.println("\n--- Segment 2 WITHOUT heading (natural route) ---");
        GHRequest req2NoHeading = new GHRequest(seg2StartLat, seg2StartLng, wp3Lat, wp3Lng).setProfile("gravel");
        req2NoHeading.setCustomModel(cm);
        req2NoHeading.setSnapPreventions(List.of("ferry"));
        GHResponse rsp2NoHeading = hopper.route(req2NoHeading);
        if (!rsp2NoHeading.hasErrors()) {
            InstructionList seg2NoHeadingInstr = rsp2NoHeading.getBest().getInstructions();
            System.out.println("Seg2 (no heading) distance: " + Math.round(rsp2NoHeading.getBest().getDistance()) + "m");
            System.out.println("Seg2 (no heading) instructions (" + seg2NoHeadingInstr.size() + "):");
            for (int i = 0; i < seg2NoHeadingInstr.size(); i++) {
                Instruction instr = seg2NoHeadingInstr.get(i);
                System.out.printf("  [%d] sign=%d (%s) dist=%.1fm name=\"%s\" text=\"%s\"%n",
                        i, instr.getSign(), signName(instr.getSign()),
                        instr.getDistance(), instr.getName(),
                        instr.getTurnDescription(seg2NoHeadingInstr.getTr()));
            }
        }

        // ========== STEP 3b: Get seg1 edge IDs ==========
        System.out.println("\n--- Segment 1: Edge-based routing (edge_id details) ---");
        GHRequest req1Edges = new GHRequest(wp1Lat, wp1Lng, wp2Lat, wp2Lng).setProfile("gravel");
        req1Edges.setCustomModel(cm);
        req1Edges.setSnapPreventions(List.of("ferry"));
        req1Edges.setPathDetails(List.of("edge_id"));
        req1Edges.putHint("instructions", false);
        req1Edges.putHint("calc_points", true);
        GHResponse rsp1Edges = hopper.route(req1Edges);
        assertFalse(rsp1Edges.hasErrors());
        List<PathDetail> seg1EdgeDetails = rsp1Edges.getBest().getPathDetails().get("edge_id");
        System.out.println("Seg1 edge IDs (" + seg1EdgeDetails.size() + "):");
        for (PathDetail d : seg1EdgeDetails) {
            System.out.print("  " + d.getValue());
        }
        System.out.println();
        System.out.println("Seg1 LAST 5 edge IDs:");
        for (int i = Math.max(0, seg1EdgeDetails.size() - 5); i < seg1EdgeDetails.size(); i++) {
            System.out.print("  " + seg1EdgeDetails.get(i).getValue());
        }
        System.out.println();

        // ========== STEP 4: Route seg2 with edge_id details (what RouteInstructionGenerator does) ==========
        System.out.println("\n--- Segment 2: Edge-based routing (instructions=false, edge_id details) ---");
        GHRequest req2Edges = new GHRequest(seg2StartLat, seg2StartLng, wp3Lat, wp3Lng).setProfile("gravel");
        req2Edges.setCustomModel(cm);
        req2Edges.setSnapPreventions(List.of("ferry"));
        req2Edges.setHeadings(List.of(seg2Heading, Double.NaN));
        req2Edges.putHint("heading_penalty", 60.0);
        req2Edges.setPathDetails(List.of("edge_id"));
        req2Edges.putHint("instructions", false);
        req2Edges.putHint("calc_points", true);
        GHResponse rsp2Edges = hopper.route(req2Edges);
        assertFalse(rsp2Edges.hasErrors(), "Seg2 edge routing failed: " + rsp2Edges.getErrors());

        List<PathDetail> seg2EdgeDetails = rsp2Edges.getBest().getPathDetails().get("edge_id");
        System.out.println("Seg2 edge IDs (" + seg2EdgeDetails.size() + "):");
        for (PathDetail d : seg2EdgeDetails) {
            System.out.print("  " + d.getValue());
        }
        System.out.println();

        // ========== STEP 4b: Compare edge IDs at boundary ==========
        System.out.println("\n--- Edge ID overlap analysis ---");
        // Get last few edges of seg1 and first few edges of seg2
        List<Integer> seg1Edges = new ArrayList<>();
        for (PathDetail d : seg1EdgeDetails) seg1Edges.add((Integer) d.getValue());
        List<Integer> seg2Edges = new ArrayList<>();
        for (PathDetail d : seg2EdgeDetails) seg2Edges.add((Integer) d.getValue());

        System.out.println("Seg1 last edge: " + seg1Edges.get(seg1Edges.size() - 1));
        System.out.println("Seg2 first edge: " + seg2Edges.get(0));
        System.out.println("Same edge at boundary? " + (seg1Edges.get(seg1Edges.size() - 1).equals(seg2Edges.get(0))));

        // Check for reversed overlap: seg2's first edges may be seg1's last edges in reverse
        System.out.println("\nSeg1 last 5 edges (forward order): ");
        for (int i = Math.max(0, seg1Edges.size() - 5); i < seg1Edges.size(); i++) {
            System.out.print(seg1Edges.get(i) + " ");
        }
        System.out.println("\nSeg2 first 5 edges: ");
        for (int i = 0; i < Math.min(5, seg2Edges.size()); i++) {
            System.out.print(seg2Edges.get(i) + " ");
        }
        System.out.println();

        // Check: are any of seg2's first edges the same as seg1's last edges (reversed traversal = U-turn)?
        Set<Integer> seg1LastEdges = new HashSet<>();
        for (int i = Math.max(0, seg1Edges.size() - 10); i < seg1Edges.size(); i++) {
            seg1LastEdges.add(seg1Edges.get(i));
        }
        System.out.println("\nSeg2 edges that overlap with seg1's last 10 edges:");
        for (int i = 0; i < seg2Edges.size(); i++) {
            if (seg1LastEdges.contains(seg2Edges.get(i))) {
                System.out.println("  seg2[" + i + "] = edge " + seg2Edges.get(i) + " (also in seg1's last 10)");
            } else {
                System.out.println("  seg2[" + i + "] = edge " + seg2Edges.get(i) + " (FIRST NON-OVERLAPPING)");
                break;
            }
        }

        // ========== STEP 5: Run through RouteInstructionGenerator ==========
        System.out.println("\n--- RouteInstructionGenerator output ---");
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest instrRequest = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint w1 = new TrailmapInstructionRequest.Waypoint();
        w1.setId("uturn_wp1");
        TrailmapInstructionRequest.Coordinates c1 = new TrailmapInstructionRequest.Coordinates();
        c1.setLat(wp1Lat); c1.setLng(wp1Lng);
        w1.setCoordinates(c1);

        TrailmapInstructionRequest.Waypoint w2 = new TrailmapInstructionRequest.Waypoint();
        w2.setId("uturn_wp2");
        TrailmapInstructionRequest.Coordinates c2 = new TrailmapInstructionRequest.Coordinates();
        c2.setLat(wp2Lat); c2.setLng(wp2Lng);
        w2.setCoordinates(c2);

        TrailmapInstructionRequest.Waypoint w3 = new TrailmapInstructionRequest.Waypoint();
        w3.setId("uturn_wp3");
        TrailmapInstructionRequest.Coordinates c3 = new TrailmapInstructionRequest.Coordinates();
        c3.setLat(wp3Lat); c3.setLng(wp3Lng);
        w3.setCoordinates(c3);

        instrRequest.setWaypoints(List.of(w1, w2, w3));
        instrRequest.setSnapPreventions(List.of("ferry"));

        TrailmapInstructionRequest.Segment seg1Req = new TrailmapInstructionRequest.Segment();
        seg1Req.setStart("uturn_wp1");
        seg1Req.setEnd("uturn_wp2");
        seg1Req.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1Req.setProfile("gravel");
        seg1Req.setCustomModel(cm);

        TrailmapInstructionRequest.Segment seg2Req = new TrailmapInstructionRequest.Segment();
        seg2Req.setStart("uturn_wp2");
        seg2Req.setEnd("uturn_wp3");
        seg2Req.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2Req.setProfile("gravel");
        seg2Req.setCustomModel(cm);
        seg2Req.setInitialHeading(seg2Heading);
        seg2Req.setHeadingPenalty(60.0);

        instrRequest.setSegments(List.of(seg1Req, seg2Req));
        instrRequest.setInstructionProfile("gravel");
        instrRequest.setLocale("fi");

        RouteInstructionGenerator.Result result = generator.generate(instrRequest);
        assertNotNull(result);

        System.out.println("Total instructions (" + result.instructions.size() + "):");
        boolean foundUturn = false;
        int pointsIndex = 0;
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            int end = pointsIndex + instr.getLength();
            System.out.printf("  [%d] sign=%d (%s) interval=[%d,%d] dist=%.1fm name=\"%s\" text=\"%s\"%n",
                    i, instr.getSign(), signName(instr.getSign()),
                    pointsIndex, end,
                    instr.getDistance(), instr.getName(),
                    instr.getTurnDescription(result.instructions.getTr()));
            if (instr.getSign() == Instruction.U_TURN_LEFT
                    || instr.getSign() == Instruction.U_TURN_RIGHT
                    || instr.getSign() == Instruction.U_TURN_UNKNOWN) {
                foundUturn = true;
            }
            pointsIndex = end;
        }

        // ========== STEP 6: Analysis ==========
        System.out.println("\n========== ANALYSIS ==========");
        System.out.println("Seg2 first instruction sign: " + seg2Instructions.get(0).getSign()
                + " (" + signName(seg2Instructions.get(0).getSign()) + ")");
        System.out.println("U-turn found in generator output: " + foundUturn);

        assertTrue(foundUturn, "Expected U-turn instruction in generator output");
    }

    /** Human-readable name for instruction sign constants */
    private static String signName(int sign) {
        switch (sign) {
            case Instruction.U_TURN_UNKNOWN: return "U_TURN_UNKNOWN";
            case Instruction.U_TURN_LEFT: return "U_TURN_LEFT";
            case Instruction.KEEP_LEFT: return "KEEP_LEFT";
            case Instruction.LEAVE_ROUNDABOUT: return "LEAVE_ROUNDABOUT";
            case Instruction.TURN_SHARP_LEFT: return "TURN_SHARP_LEFT";
            case Instruction.TURN_LEFT: return "TURN_LEFT";
            case Instruction.TURN_SLIGHT_LEFT: return "TURN_SLIGHT_LEFT";
            case Instruction.CONTINUE_ON_STREET: return "CONTINUE_ON_STREET";
            case Instruction.TURN_SLIGHT_RIGHT: return "TURN_SLIGHT_RIGHT";
            case Instruction.TURN_RIGHT: return "TURN_RIGHT";
            case Instruction.TURN_SHARP_RIGHT: return "TURN_SHARP_RIGHT";
            case Instruction.FINISH: return "FINISH";
            case Instruction.REACHED_VIA: return "REACHED_VIA";
            case Instruction.USE_ROUNDABOUT: return "USE_ROUNDABOUT";
            case Instruction.KEEP_RIGHT: return "KEEP_RIGHT";
            case Instruction.U_TURN_RIGHT: return "U_TURN_RIGHT";
            default: return "UNKNOWN(" + sign + ")";
        }
    }

    /**
     * Diagnostic test for instruction quality on a 3-segment gravel route in Tampere.
     *
     * Known issues reported:
     * - "Liity vasen tie Kalimankuja" should be "turn left then turn right to Kalimankuja"
     * - "Vasen sitten oikea 38m" should be "vasen, sitten liity oikea pyörätie"
     * - U-turn + sharp left from wrong-fork snapping with ~1-2m distances
     * - "Liity oikea pyörätie" should be "loiva oikea pyörätie" (slight right, not join)
     * - "Liity vasen pyörätiet" should be "vasen pyörätie" (proper turn, not join)
     */
    @Test
    void testThreeSegmentGravelRouteInstructionQuality() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = buildGravelRouteQualityRequest();

        RouteInstructionGenerator.Result result = generator.generate(request);

        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== 3-SEGMENT GRAVEL ROUTE — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        // Apply post-processing (same as TrailmapInstructionResource does)
        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions);

        // The last instruction should be FINISH
        Instruction lastInstr = result.instructions.get(result.instructions.size() - 1);
        assertEquals(Instruction.FINISH, lastInstr.getSign(), "Last instruction should be FINISH");

        // Exactly one FINISH
        long finishCount = 0;
        for (Instruction instr : result.instructions) {
            if (instr.getSign() == Instruction.FINISH) finishCount++;
        }
        assertEquals(1, finishCount, "Should have exactly one FINISH instruction");

        System.out.println("\n========== 3-SEGMENT GRAVEL ROUTE — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        System.out.println("Polyline points: " + result.polyline.size());
        printInstructionsDetailed(result);

        // Also route each segment individually with standard GH instructions for comparison
        System.out.println("\n--- Per-segment standard GH instructions for comparison ---");

        // Segment 1: wp1 -> wp2
        System.out.println("\nSegment 1 (wp1->wp2, gravel, no heading):");
        GHRequest req1 = new GHRequest(61.504844, 23.650666, 61.509259, 23.665011)
                .setProfile("gravel");
        req1.setSnapPreventions(List.of("ferry"));
        GHResponse rsp1 = hopper.route(req1);
        assertFalse(rsp1.hasErrors(), "Seg1 routing failed: " + rsp1.getErrors());
        printStdInstructions(rsp1.getBest());

        // Compute seg1 end heading for seg2
        PointList seg1Poly = rsp1.getBest().getPoints();
        double seg1EndHeading = Double.NaN;
        if (seg1Poly.size() >= 2) {
            seg1EndHeading = AngleCalc.ANGLE_CALC.calcAzimuth(
                    seg1Poly.getLat(seg1Poly.size() - 2), seg1Poly.getLon(seg1Poly.size() - 2),
                    seg1Poly.getLat(seg1Poly.size() - 1), seg1Poly.getLon(seg1Poly.size() - 1));
        }
        double seg1SnappedEndLat = seg1Poly.getLat(seg1Poly.size() - 1);
        double seg1SnappedEndLng = seg1Poly.getLon(seg1Poly.size() - 1);
        System.out.printf("  Seg1 snapped end: (%.7f, %.7f), end heading: %.5f%n",
                seg1SnappedEndLat, seg1SnappedEndLng, seg1EndHeading);

        // Segment 2: wp2 -> wp3, initial_heading=45.68
        System.out.println("\nSegment 2 (wp2->wp3, gravel, heading=45.69, penalty=60):");
        GHRequest req2 = new GHRequest(seg1SnappedEndLat, seg1SnappedEndLng, 61.513681, 23.658535)
                .setProfile("gravel");
        req2.setSnapPreventions(List.of("ferry"));
        req2.setHeadings(List.of(45.68583480118048, Double.NaN));
        req2.putHint("heading_penalty", 60.0);
        GHResponse rsp2 = hopper.route(req2);
        assertFalse(rsp2.hasErrors(), "Seg2 routing failed: " + rsp2.getErrors());
        printStdInstructions(rsp2.getBest());

        // Compute seg2 end heading for seg3
        PointList seg2Poly = rsp2.getBest().getPoints();
        double seg2EndHeading = Double.NaN;
        if (seg2Poly.size() >= 2) {
            seg2EndHeading = AngleCalc.ANGLE_CALC.calcAzimuth(
                    seg2Poly.getLat(seg2Poly.size() - 2), seg2Poly.getLon(seg2Poly.size() - 2),
                    seg2Poly.getLat(seg2Poly.size() - 1), seg2Poly.getLon(seg2Poly.size() - 1));
        }
        double seg2SnappedEndLat = seg2Poly.getLat(seg2Poly.size() - 1);
        double seg2SnappedEndLng = seg2Poly.getLon(seg2Poly.size() - 1);
        System.out.printf("  Seg2 snapped end: (%.7f, %.7f), end heading: %.5f%n",
                seg2SnappedEndLat, seg2SnappedEndLng, seg2EndHeading);

        // Segment 3: wp3 -> wp4, initial_heading=90.78
        System.out.println("\nSegment 3 (wp3->wp4, gravel, heading=90.78, penalty=60):");
        GHRequest req3 = new GHRequest(seg2SnappedEndLat, seg2SnappedEndLng, 61.523506, 23.635385)
                .setProfile("gravel");
        req3.setSnapPreventions(List.of("ferry"));
        req3.setHeadings(List.of(90.77994607064392, Double.NaN));
        req3.putHint("heading_penalty", 60.0);
        GHResponse rsp3 = hopper.route(req3);
        assertFalse(rsp3.hasErrors(), "Seg3 routing failed: " + rsp3.getErrors());
        printStdInstructions(rsp3.getBest());

        // Edge ID analysis at boundaries
        System.out.println("\n--- Edge ID analysis ---");
        GHRequest req1e = new GHRequest(61.504844, 23.650666, 61.509259, 23.665011)
                .setProfile("gravel");
        req1e.setSnapPreventions(List.of("ferry"));
        req1e.setPathDetails(List.of("edge_id"));
        req1e.putHint("instructions", false);
        req1e.putHint("calc_points", true);
        GHResponse rsp1e = hopper.route(req1e);
        List<PathDetail> seg1Edges = rsp1e.getBest().getPathDetails().get("edge_id");

        GHRequest req2e = new GHRequest(seg1SnappedEndLat, seg1SnappedEndLng, 61.513681, 23.658535)
                .setProfile("gravel");
        req2e.setSnapPreventions(List.of("ferry"));
        req2e.setHeadings(List.of(45.68583480118048, Double.NaN));
        req2e.putHint("heading_penalty", 60.0);
        req2e.setPathDetails(List.of("edge_id"));
        req2e.putHint("instructions", false);
        req2e.putHint("calc_points", true);
        GHResponse rsp2e = hopper.route(req2e);
        List<PathDetail> seg2Edges = rsp2e.getBest().getPathDetails().get("edge_id");

        GHRequest req3e = new GHRequest(seg2SnappedEndLat, seg2SnappedEndLng, 61.523506, 23.635385)
                .setProfile("gravel");
        req3e.setSnapPreventions(List.of("ferry"));
        req3e.setHeadings(List.of(90.77994607064392, Double.NaN));
        req3e.putHint("heading_penalty", 60.0);
        req3e.setPathDetails(List.of("edge_id"));
        req3e.putHint("instructions", false);
        req3e.putHint("calc_points", true);
        GHResponse rsp3e = hopper.route(req3e);
        List<PathDetail> seg3Edges = rsp3e.getBest().getPathDetails().get("edge_id");

        System.out.println("Seg1 edges (" + seg1Edges.size() + "): last 5 = ");
        for (int i = Math.max(0, seg1Edges.size() - 5); i < seg1Edges.size(); i++) {
            System.out.print(seg1Edges.get(i).getValue() + " ");
        }
        System.out.println();

        System.out.println("Seg2 edges (" + seg2Edges.size() + "): first 5 = ");
        for (int i = 0; i < Math.min(5, seg2Edges.size()); i++) {
            System.out.print(seg2Edges.get(i).getValue() + " ");
        }
        System.out.print(" ... last 5 = ");
        for (int i = Math.max(0, seg2Edges.size() - 5); i < seg2Edges.size(); i++) {
            System.out.print(seg2Edges.get(i).getValue() + " ");
        }
        System.out.println();

        System.out.println("Seg3 edges (" + seg3Edges.size() + "): first 5 = ");
        for (int i = 0; i < Math.min(5, seg3Edges.size()); i++) {
            System.out.print(seg3Edges.get(i).getValue() + " ");
        }
        System.out.println();

        System.out.println("\nBoundary 1: seg1 last=" + seg1Edges.get(seg1Edges.size()-1).getValue()
                + " seg2 first=" + seg2Edges.get(0).getValue());
        System.out.println("Boundary 2: seg2 last=" + seg2Edges.get(seg2Edges.size()-1).getValue()
                + " seg3 first=" + seg3Edges.get(0).getValue());

        // Polyline around the seg2/seg3 boundary (U-turn area)
        System.out.println("\n--- Polyline around seg2/seg3 boundary (indices 75-90) ---");
        PointList poly = result.polyline;
        for (int i = Math.max(0, 75); i < Math.min(poly.size(), 90); i++) {
            double dist = 0;
            if (i > 0) {
                dist = DistanceCalcEarth.DIST_EARTH.calcDist(
                        poly.getLat(i-1), poly.getLon(i-1), poly.getLat(i), poly.getLon(i));
            }
            System.out.printf("  [%d] (%.7f, %.7f) dist_from_prev=%.1fm%n",
                    i, poly.getLat(i), poly.getLon(i), dist);
        }
    }

    private void printStdInstructions(ResponsePath path) {
        InstructionList instructions = path.getInstructions();
        System.out.println("  Distance: " + Math.round(path.getDistance()) + "m, "
                + instructions.size() + " instructions:");
        for (int i = 0; i < instructions.size(); i++) {
            Instruction instr = instructions.get(i);
            System.out.printf("    [%d] sign=%d (%s) dist=%.1fm name=\"%s\" text=\"%s\"%n",
                    i, instr.getSign(), signName(instr.getSign()),
                    instr.getDistance(), instr.getName(),
                    instr.getTurnDescription(instructions.getTr()));
        }
    }

    private TrailmapInstructionRequest buildGravelRouteQualityRequest() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("dSzJCa8y5VQO_R7cGbyQ6", 61.504844, 23.650666);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("V8U0AM9AVfH16vcpB8hlS", 61.509259, 23.665011);
        TrailmapInstructionRequest.Waypoint wp3 = makeWaypoint("OlRvwidW2kuyAIMl2ce4z", 61.513681, 23.658535);
        TrailmapInstructionRequest.Waypoint wp4 = makeWaypoint("ufjCSuF4BFkVmXR49Kw8I", 61.523506, 23.635385);

        request.setWaypoints(List.of(wp1, wp2, wp3, wp4));

        // Segment 1: wp1 -> wp2, gravel, no heading
        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("dSzJCa8y5VQO_R7cGbyQ6");
        seg1.setEnd("V8U0AM9AVfH16vcpB8hlS");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("gravel");

        // Segment 2: wp2 -> wp3, gravel, heading=45.69, penalty=60
        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("V8U0AM9AVfH16vcpB8hlS");
        seg2.setEnd("OlRvwidW2kuyAIMl2ce4z");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2.setProfile("gravel");
        seg2.setInitialHeading(45.68583480118048);
        seg2.setHeadingPenalty(60.0);

        // Segment 3: wp3 -> wp4, gravel, heading=90.78, penalty=60
        TrailmapInstructionRequest.Segment seg3 = new TrailmapInstructionRequest.Segment();
        seg3.setStart("OlRvwidW2kuyAIMl2ce4z");
        seg3.setEnd("ufjCSuF4BFkVmXR49Kw8I");
        seg3.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg3.setProfile("gravel");
        seg3.setInitialHeading(90.77994607064392);
        seg3.setHeadingPenalty(60.0);

        request.setSegments(List.of(seg1, seg2, seg3));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        return request;
    }

    // ---- Helpers ----

    private TrailmapInstructionRequest buildThreeSegmentRequest() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        // Waypoint 1
        TrailmapInstructionRequest.Waypoint wp1 = new TrailmapInstructionRequest.Waypoint();
        wp1.setId("b3LiaXPGejDScWxE7FWic");
        TrailmapInstructionRequest.Coordinates c1 = new TrailmapInstructionRequest.Coordinates();
        c1.setLat(61.504628);
        c1.setLng(23.701588);
        wp1.setCoordinates(c1);

        // Waypoint 2
        TrailmapInstructionRequest.Waypoint wp2 = new TrailmapInstructionRequest.Waypoint();
        wp2.setId("CxCY8V0jMaIF_2AvJqLxI");
        TrailmapInstructionRequest.Coordinates c2 = new TrailmapInstructionRequest.Coordinates();
        c2.setLat(61.503761);
        c2.setLng(23.706487);
        wp2.setCoordinates(c2);

        // Waypoint 3
        TrailmapInstructionRequest.Waypoint wp3 = new TrailmapInstructionRequest.Waypoint();
        wp3.setId("h32CFUDDvjobsGisEzoP2");
        TrailmapInstructionRequest.Coordinates c3 = new TrailmapInstructionRequest.Coordinates();
        c3.setLat(61.503036);
        c3.setLng(23.708274);
        wp3.setCoordinates(c3);

        // Waypoint 4
        TrailmapInstructionRequest.Waypoint wp4 = new TrailmapInstructionRequest.Waypoint();
        wp4.setId("sliCvSryH_PwMIitVrPYA");
        TrailmapInstructionRequest.Coordinates c4 = new TrailmapInstructionRequest.Coordinates();
        c4.setLat(61.498915);
        c4.setLng(23.718261);
        wp4.setCoordinates(c4);

        request.setWaypoints(List.of(wp1, wp2, wp3, wp4));

        // Segment 1: wp1 -> wp2
        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("b3LiaXPGejDScWxE7FWic");
        seg1.setEnd("CxCY8V0jMaIF_2AvJqLxI");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("gravel_mtb");

        // Segment 2: wp2 -> wp3
        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("CxCY8V0jMaIF_2AvJqLxI");
        seg2.setEnd("h32CFUDDvjobsGisEzoP2");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2.setProfile("gravel_mtb");

        // Segment 3: wp3 -> wp4 with initial_heading
        TrailmapInstructionRequest.Segment seg3 = new TrailmapInstructionRequest.Segment();
        seg3.setStart("h32CFUDDvjobsGisEzoP2");
        seg3.setEnd("sliCvSryH_PwMIitVrPYA");
        seg3.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg3.setProfile("gravel_mtb");
        seg3.setInitialHeading(155.8775128628056);

        request.setSegments(List.of(seg1, seg2, seg3));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");

        return request;
    }

    private TrailmapInstructionRequest buildTwoSegmentRequest() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        // Waypoint 1
        TrailmapInstructionRequest.Waypoint wp1 = new TrailmapInstructionRequest.Waypoint();
        wp1.setId("txfSSsFwjfJDCi7PqBlTg");
        TrailmapInstructionRequest.Coordinates c1 = new TrailmapInstructionRequest.Coordinates();
        c1.setLat(61.500784);
        c1.setLng(23.685416);
        wp1.setCoordinates(c1);

        // Waypoint 2
        TrailmapInstructionRequest.Waypoint wp2 = new TrailmapInstructionRequest.Waypoint();
        wp2.setId("d50NJNDUi1fnn9d_Vt-gO");
        TrailmapInstructionRequest.Coordinates c2 = new TrailmapInstructionRequest.Coordinates();
        c2.setLat(61.504067);
        c2.setLng(23.695978);
        wp2.setCoordinates(c2);

        // Waypoint 3
        TrailmapInstructionRequest.Waypoint wp3 = new TrailmapInstructionRequest.Waypoint();
        wp3.setId("9T8vaMjodfj53wEdzb16L");
        TrailmapInstructionRequest.Coordinates c3 = new TrailmapInstructionRequest.Coordinates();
        c3.setLat(61.505441);
        c3.setLng(23.698538);
        wp3.setCoordinates(c3);

        request.setWaypoints(List.of(wp1, wp2, wp3));

        // Segment 1
        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("txfSSsFwjfJDCi7PqBlTg");
        seg1.setEnd("d50NJNDUi1fnn9d_Vt-gO");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("gravel_mtb");

        // Segment 2
        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("d50NJNDUi1fnn9d_Vt-gO");
        seg2.setEnd("9T8vaMjodfj53wEdzb16L");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2.setProfile("gravel_mtb");
        seg2.setInitialHeading(88.28477990934573);

        request.setSegments(List.of(seg1, seg2));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");

        return request;
    }

    private TrailmapInstructionRequest buildThreeWaypointRequest(
            double lat1, double lng1, double lat2, double lng2,
            double lat3, double lng3, double seg2Heading) {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = new TrailmapInstructionRequest.Waypoint();
        wp1.setId("wp1");
        TrailmapInstructionRequest.Coordinates c1 = new TrailmapInstructionRequest.Coordinates();
        c1.setLat(lat1);
        c1.setLng(lng1);
        wp1.setCoordinates(c1);

        TrailmapInstructionRequest.Waypoint wp2 = new TrailmapInstructionRequest.Waypoint();
        wp2.setId("wp2");
        TrailmapInstructionRequest.Coordinates c2 = new TrailmapInstructionRequest.Coordinates();
        c2.setLat(lat2);
        c2.setLng(lng2);
        wp2.setCoordinates(c2);

        TrailmapInstructionRequest.Waypoint wp3 = new TrailmapInstructionRequest.Waypoint();
        wp3.setId("wp3");
        TrailmapInstructionRequest.Coordinates c3 = new TrailmapInstructionRequest.Coordinates();
        c3.setLat(lat3);
        c3.setLng(lng3);
        wp3.setCoordinates(c3);

        request.setWaypoints(List.of(wp1, wp2, wp3));

        // Segment 1: wp1 -> wp2
        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("wp1");
        seg1.setEnd("wp2");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("gravel_mtb");

        // Segment 2: wp2 -> wp3 with initial_heading
        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("wp2");
        seg2.setEnd("wp3");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2.setProfile("gravel_mtb");
        seg2.setInitialHeading(seg2Heading);

        request.setSegments(List.of(seg1, seg2));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");

        return request;
    }

    /**
     * Diagnostic test for the boundary edge deduplication issue.
     *
     * When two segments share a boundary edge (seg1 ends on it, seg2 starts on it),
     * stitchEdgeChains() removes the shared edge from seg2. This causes seg2's
     * synthetic path to lose the context needed for the first real turn instruction.
     *
     * Specifically: if the shared edge is a CYCLEWAY and the next edge is a PATH,
     * the CYCLEWAY→PATH turn instruction is never generated because:
     * - Seg1 doesn't generate it (seg1's instructions end before/at the shared edge)
     * - Seg2 lost the CYCLEWAY edge, so InstructionsFromEdges starts at PATH
     *   with CONTINUE_ON_STREET (no prev edge context), which appendInstructions strips.
     *
     * Test coordinates from production bug report:
     * w1=(61.500578, 23.666027) → w2=(61.500578, 23.662072) → w3=(61.501156, 23.662261)
     *
     * Single-segment w2→w3 produces correct "turn right onto PATH" instruction.
     * Two-segment w1→w2→w3 loses that instruction, showing only "turn right onto Hedelmäkatu".
     */
    @Test
    void testBoundaryEdgeDeduplicationLosesInstruction() {
        double w1Lat = 61.500578, w1Lng = 23.666027;
        double w2Lat = 61.500578, w2Lng = 23.662072;
        double w3Lat = 61.501156, w3Lng = 23.662261;

        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        // ========== STEP 1: Single-segment w2→w3 (the correct baseline) ==========
        System.out.println("\n========== BOUNDARY EDGE DEDUP DIAGNOSTIC ==========");
        System.out.println("\n--- STEP 1: Single-segment w2→w3 (correct baseline) ---");

        TrailmapInstructionRequest singleReq = new TrailmapInstructionRequest();
        TrailmapInstructionRequest.Waypoint sw1 = makeWaypoint("sw1", w2Lat, w2Lng);
        TrailmapInstructionRequest.Waypoint sw2 = makeWaypoint("sw2", w3Lat, w3Lng);
        singleReq.setWaypoints(List.of(sw1, sw2));
        TrailmapInstructionRequest.Segment sSeg = new TrailmapInstructionRequest.Segment();
        sSeg.setStart("sw1");
        sSeg.setEnd("sw2");
        sSeg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        sSeg.setProfile("gravel");
        singleReq.setSegments(List.of(sSeg));
        singleReq.setInstructionProfile("gravel");
        singleReq.setLocale("fi");
        singleReq.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result singleResult = generator.generate(singleReq);
        System.out.println("Single-segment instructions (" + singleResult.instructions.size() + "):");
        printInstructionsDetailed(singleResult);

        // ========== STEP 2: Two-segment w1→w2→w3 (the broken case) ==========
        System.out.println("\n--- STEP 2: Two-segment w1→w2→w3 (the broken case) ---");

        TrailmapInstructionRequest twoSegReq = new TrailmapInstructionRequest();
        TrailmapInstructionRequest.Waypoint tw1 = makeWaypoint("tw1", w1Lat, w1Lng);
        TrailmapInstructionRequest.Waypoint tw2 = makeWaypoint("tw2", w2Lat, w2Lng);
        TrailmapInstructionRequest.Waypoint tw3 = makeWaypoint("tw3", w3Lat, w3Lng);
        twoSegReq.setWaypoints(List.of(tw1, tw2, tw3));
        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("tw1");
        seg1.setEnd("tw2");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("gravel");
        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("tw2");
        seg2.setEnd("tw3");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2.setProfile("gravel");
        twoSegReq.setSegments(List.of(seg1, seg2));
        twoSegReq.setInstructionProfile("gravel");
        twoSegReq.setLocale("fi");
        twoSegReq.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result twoSegResult = generator.generate(twoSegReq);
        System.out.println("Two-segment instructions (" + twoSegResult.instructions.size() + "):");
        printInstructionsDetailed(twoSegResult);

        // ========== STEP 3: Edge ID analysis at the boundary ==========
        System.out.println("\n--- STEP 3: Edge ID analysis ---");

        // Route seg1 (w1→w2) and get edge IDs
        GHRequest req1 = new GHRequest(w1Lat, w1Lng, w2Lat, w2Lng).setProfile("gravel");
        req1.setPathDetails(List.of("edge_id"));
        req1.putHint("instructions", false);
        req1.putHint("calc_points", true);
        req1.setSnapPreventions(List.of("ferry"));
        GHResponse rsp1 = hopper.route(req1);
        assertFalse(rsp1.hasErrors(), "Seg1 routing failed: " + rsp1.getErrors());

        List<PathDetail> seg1EdgeDetails = rsp1.getBest().getPathDetails().get("edge_id");
        PointList seg1Poly = rsp1.getBest().getPoints();

        System.out.println("Seg1 edge IDs (" + seg1EdgeDetails.size() + "): ");
        for (PathDetail d : seg1EdgeDetails) System.out.print(d.getValue() + " ");
        System.out.println();

        // Compute heading at end of seg1 (what heading chaining does)
        double chainedHeading = Double.NaN;
        if (seg1Poly.size() >= 2) {
            chainedHeading = AngleCalc.ANGLE_CALC.calcAzimuth(
                    seg1Poly.getLat(seg1Poly.size() - 2), seg1Poly.getLon(seg1Poly.size() - 2),
                    seg1Poly.getLat(seg1Poly.size() - 1), seg1Poly.getLon(seg1Poly.size() - 1));
        }
        double snappedEndLat = seg1Poly.getLat(seg1Poly.size() - 1);
        double snappedEndLng = seg1Poly.getLon(seg1Poly.size() - 1);
        System.out.printf("Seg1 snapped end: (%.7f, %.7f), chained heading: %.2f%n",
                snappedEndLat, snappedEndLng, chainedHeading);

        // Route seg2 with chained heading + snapped start (what RouteInstructionGenerator does)
        GHRequest req2Chained = new GHRequest(snappedEndLat, snappedEndLng, w3Lat, w3Lng).setProfile("gravel");
        req2Chained.setPathDetails(List.of("edge_id"));
        req2Chained.putHint("instructions", false);
        req2Chained.putHint("calc_points", true);
        req2Chained.setSnapPreventions(List.of("ferry"));
        if (!Double.isNaN(chainedHeading)) {
            req2Chained.setHeadings(List.of(chainedHeading, Double.NaN));
        }
        GHResponse rsp2Chained = hopper.route(req2Chained);
        assertFalse(rsp2Chained.hasErrors(), "Seg2 (chained) routing failed: " + rsp2Chained.getErrors());

        List<PathDetail> seg2ChainedEdgeDetails = rsp2Chained.getBest().getPathDetails().get("edge_id");
        System.out.println("Seg2 (with chained heading) edge IDs (" + seg2ChainedEdgeDetails.size() + "): ");
        for (PathDetail d : seg2ChainedEdgeDetails) System.out.print(d.getValue() + " ");
        System.out.println();

        // Route seg2 WITHOUT heading (the standalone case)
        GHRequest req2Standalone = new GHRequest(w2Lat, w2Lng, w3Lat, w3Lng).setProfile("gravel");
        req2Standalone.setPathDetails(List.of("edge_id"));
        req2Standalone.putHint("instructions", false);
        req2Standalone.putHint("calc_points", true);
        req2Standalone.setSnapPreventions(List.of("ferry"));
        GHResponse rsp2Standalone = hopper.route(req2Standalone);
        assertFalse(rsp2Standalone.hasErrors(), "Seg2 (standalone) routing failed: " + rsp2Standalone.getErrors());

        List<PathDetail> seg2StandaloneEdgeDetails = rsp2Standalone.getBest().getPathDetails().get("edge_id");
        System.out.println("Seg2 (standalone, no heading) edge IDs (" + seg2StandaloneEdgeDetails.size() + "): ");
        for (PathDetail d : seg2StandaloneEdgeDetails) System.out.print(d.getValue() + " ");
        System.out.println();

        // ========== STEP 4: Boundary analysis ==========
        System.out.println("\n--- STEP 4: Boundary analysis ---");
        int lastSeg1Edge = (Integer) seg1EdgeDetails.get(seg1EdgeDetails.size() - 1).getValue();
        int firstSeg2Edge = (Integer) seg2ChainedEdgeDetails.get(0).getValue();
        System.out.println("Seg1 last edge: " + lastSeg1Edge);
        System.out.println("Seg2 first edge (chained): " + firstSeg2Edge);
        System.out.println("Same edge at boundary? " + (lastSeg1Edge == firstSeg2Edge));

        if (lastSeg1Edge == firstSeg2Edge) {
            System.out.println("\n*** CONFIRMED: Shared boundary edge " + lastSeg1Edge + " ***");
            System.out.println("stitchEdgeChains() will dedup this edge from seg2,");
            System.out.println("causing seg2's synthetic path to lose the first edge context.");

            if (seg2ChainedEdgeDetails.size() >= 2) {
                int secondSeg2Edge = (Integer) seg2ChainedEdgeDetails.get(1).getValue();
                System.out.println("After dedup, seg2's synthetic path will start at edge: " + secondSeg2Edge);
                System.out.println("The turn from edge " + lastSeg1Edge + " to edge " + secondSeg2Edge
                        + " is lost (never computed by InstructionsFromEdges for either segment).");
            }
        }

        // ========== STEP 5: Check what instructions seg2's synthetic path produces ==========
        // Build synthetic path for seg2 WITH the shared edge (as if no dedup)
        System.out.println("\n--- STEP 5: Synthetic path instruction comparison ---");

        // Extract seg2 edge IDs (chained)
        List<Integer> seg2EdgesChained = new ArrayList<>();
        for (PathDetail d : seg2ChainedEdgeDetails) {
            int eid = (Integer) d.getValue();
            if (seg2EdgesChained.isEmpty() || seg2EdgesChained.get(seg2EdgesChained.size() - 1) != eid) {
                seg2EdgesChained.add(eid);
            }
        }

        // Extract seg2 edge IDs (standalone)
        List<Integer> seg2EdgesStandalone = new ArrayList<>();
        for (PathDetail d : seg2StandaloneEdgeDetails) {
            int eid = (Integer) d.getValue();
            if (seg2EdgesStandalone.isEmpty() || seg2EdgesStandalone.get(seg2EdgesStandalone.size() - 1) != eid) {
                seg2EdgesStandalone.add(eid);
            }
        }

        System.out.println("Seg2 deduped edge IDs (chained): " + seg2EdgesChained);
        System.out.println("Seg2 deduped edge IDs (standalone): " + seg2EdgesStandalone);

        // Compare: are they the same edges?
        boolean sameEdges = seg2EdgesChained.equals(seg2EdgesStandalone);
        System.out.println("Same edges (chained vs standalone)? " + sameEdges);
        if (!sameEdges) {
            System.out.println("*** Edge chains differ — heading chaining affects routing! ***");
        }

        // ========== ANALYSIS ==========
        System.out.println("\n========== ANALYSIS ==========");
        System.out.println("The issue: stitchEdgeChains() removes the shared boundary edge from seg2.");
        System.out.println("This removes the context InstructionsFromEdges needs to generate the");
        System.out.println("CYCLEWAY→PATH turn instruction. The first instruction of seg2's synthetic");
        System.out.println("path becomes CONTINUE_ON_STREET on PATH, which appendInstructions strips.");
        System.out.println("Result: the CYCLEWAY→PATH turn is lost entirely.");
    }

    /** Helper: create a waypoint */
    private TrailmapInstructionRequest.Waypoint makeWaypoint(String id, double lat, double lng) {
        TrailmapInstructionRequest.Waypoint wp = new TrailmapInstructionRequest.Waypoint();
        wp.setId(id);
        TrailmapInstructionRequest.Coordinates c = new TrailmapInstructionRequest.Coordinates();
        c.setLat(lat);
        c.setLng(lng);
        wp.setCoordinates(c);
        return wp;
    }

    /** Helper: print instructions with all extraInfo details */
    private void printInstructionsDetailed(RouteInstructionGenerator.Result result) {
        int pointsIndex = 0;
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            int end = pointsIndex + instr.getLength();
            System.out.printf("  [%d] sign=%d (%s) interval=[%d,%d] dist=%.1fm name=\"%s\" text=\"%s\"%n",
                    i, instr.getSign(), signName(instr.getSign()),
                    pointsIndex, end,
                    instr.getDistance(), instr.getName(),
                    instr.getTurnDescription(result.instructions.getTr()));
            // Print key extraInfo
            Map<String, Object> extra = instr.getExtraInfoJSON();
            if (extra.containsKey("predicted_highway"))
                System.out.println("       predicted_highway=" + extra.get("predicted_highway"));
            if (extra.containsKey("prev_predicted_highway"))
                System.out.println("       prev_predicted_highway=" + extra.get("prev_predicted_highway"));
            if (extra.containsKey("road_class"))
                System.out.println("       road_class=" + extra.get("road_class"));
            if (extra.containsKey("prev_road_class"))
                System.out.println("       prev_road_class=" + extra.get("prev_road_class"));
            if (extra.containsKey("turn_angle_deg"))
                System.out.println("       turn_angle_deg=" + extra.get("turn_angle_deg"));
            if (extra.containsKey("junction_alternatives"))
                System.out.println("       junction_alternatives=" + extra.get("junction_alternatives"));
            if (extra.containsKey("junction_alt_predicted_highways"))
                System.out.println("       junction_alt_predicted_highways=" + extra.get("junction_alt_predicted_highways"));
            if (extra.containsKey("junction_has_higher_road"))
                System.out.println("       junction_has_higher_road=" + extra.get("junction_has_higher_road"));
            if (extra.containsKey("source_road_continues"))
                System.out.println("       source_road_continues=" + extra.get("source_road_continues"));
            if (extra.containsKey("join_direction"))
                System.out.println("       join_direction=" + extra.get("join_direction"));
            if (extra.containsKey("join_target_type"))
                System.out.println("       join_target_type=" + extra.get("join_target_type"));
            if (extra.containsKey("then_turn"))
                System.out.println("       then_turn=" + extra.get("then_turn"));
            if (extra.containsKey("tbt_available"))
                System.out.println("       tbt_available=" + extra.get("tbt_available"));
            if (extra.containsKey("segment_type"))
                System.out.println("       segment_type=" + extra.get("segment_type"));
            if (extra.containsKey("confirm_reason"))
                System.out.println("       confirm_reason=" + extra.get("confirm_reason"));
            if (extra.containsKey("tbt_resumed"))
                System.out.println("       tbt_resumed=" + extra.get("tbt_resumed"));
            if (extra.containsKey("prev_segment_type"))
                System.out.println("       prev_segment_type=" + extra.get("prev_segment_type"));
            if (extra.containsKey("next_segment_type"))
                System.out.println("       next_segment_type=" + extra.get("next_segment_type"));
            if (extra.containsKey("tbt_priority"))
                System.out.println("       tbt_priority=" + extra.get("tbt_priority"));
            pointsIndex = end;
        }
    }

    private TrailmapInstructionRequest buildSingleSegmentRequest(String routingProfile,
                                                                  String instructionProfile,
                                                                  String locale) {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = new TrailmapInstructionRequest.Waypoint();
        wp1.setId("start");
        TrailmapInstructionRequest.Coordinates c1 = new TrailmapInstructionRequest.Coordinates();
        c1.setLat(START_LAT);
        c1.setLng(START_LNG);
        wp1.setCoordinates(c1);

        TrailmapInstructionRequest.Waypoint wp2 = new TrailmapInstructionRequest.Waypoint();
        wp2.setId("end");
        TrailmapInstructionRequest.Coordinates c2 = new TrailmapInstructionRequest.Coordinates();
        c2.setLat(END_LAT);
        c2.setLng(END_LNG);
        wp2.setCoordinates(c2);

        request.setWaypoints(List.of(wp1, wp2));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("start");
        seg.setEnd("end");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile(routingProfile);

        request.setSegments(List.of(seg));
        request.setInstructionProfile(instructionProfile);
        request.setLocale(locale);

        return request;
    }

    /**
     * Diagnostic test for missing turns on a trailmap_foot 2-segment route.
     *
     * API payload:
     *   3 waypoints, 2 segments (both trailmap_foot), instruction_profile=gravel, locale=fi
     *   Segment 2 has initial_heading=78.02 and heading_penalty=60
     *
     * Known issue: response returns only 2 left turns. Two turns are missing:
     *   - A right turn ~80-90m after the 2nd returned left turn
     *   - A left turn ~40m before the route ends
     */
    @Test
    void testFootRouteMissingTurns() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        // Build request matching the API payload exactly
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint w1 = makeWaypoint("GTKC9r7HhdrUYfqcX9GLX", 61.499257, 23.677376);
        TrailmapInstructionRequest.Waypoint w2 = makeWaypoint("1v9JT-Clrm8w7N51x4ZFP", 61.50065, 23.678926);
        TrailmapInstructionRequest.Waypoint w3 = makeWaypoint("wxWAxyqejR7gHybC6JMag", 61.501256, 23.685815);
        request.setWaypoints(List.of(w1, w2, w3));

        // Segment 1: wp1 -> wp2, trailmap_foot, no heading
        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("GTKC9r7HhdrUYfqcX9GLX");
        seg1.setEnd("1v9JT-Clrm8w7N51x4ZFP");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("trailmap_foot");

        // Segment 2: wp2 -> wp3, trailmap_foot, initial_heading + heading_penalty
        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("1v9JT-Clrm8w7N51x4ZFP");
        seg2.setEnd("wxWAxyqejR7gHybC6JMag");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2.setProfile("trailmap_foot");
        seg2.setInitialHeading(78.02216610253362);
        seg2.setHeadingPenalty(60.0);

        request.setSegments(List.of(seg1, seg2));
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        // --- Test with BOTH instruction profiles to compare ---
        // First: "gravel" (the buggy client value) — gravel weighting can't see footways
        request.setInstructionProfile("gravel");
        RouteInstructionGenerator.Result resultGravel = generator.generate(request);

        System.out.println("\n========== instruction_profile=gravel (ORIGINAL BUG) ==========");
        System.out.println("Instructions: " + resultGravel.instructions.size());
        printInstructionsDetailed(resultGravel);

        // Second: "trailmap_foot" — foot weighting should see all footways
        request.setInstructionProfile("trailmap_foot");
        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== FOOT ROUTE MISSING TURNS — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        System.out.println("Polyline points: " + result.polyline.size());
        printInstructionsDetailed(result);

        // Apply post-processing (same as TrailmapInstructionResource does)
        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions);

        System.out.println("\n========== FOOT ROUTE MISSING TURNS — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        // --- Also route each segment individually with standard GH for comparison ---
        System.out.println("\n--- Per-segment standard GH instructions for comparison ---");

        // Segment 1: wp1 -> wp2
        System.out.println("\nSegment 1 (trailmap_foot, no heading):");
        GHRequest req1 = new GHRequest(61.499257, 23.677376, 61.50065, 23.678926)
                .setProfile("trailmap_foot");
        req1.setSnapPreventions(List.of("ferry"));
        GHResponse rsp1 = hopper.route(req1);
        assertFalse(rsp1.hasErrors(), "Seg1 routing failed: " + rsp1.getErrors());
        printStdInstructions(rsp1.getBest());

        // Compute seg1 end heading
        PointList seg1Poly = rsp1.getBest().getPoints();
        double seg1EndHeading = Double.NaN;
        if (seg1Poly.size() >= 2) {
            seg1EndHeading = AngleCalc.ANGLE_CALC.calcAzimuth(
                    seg1Poly.getLat(seg1Poly.size() - 2), seg1Poly.getLon(seg1Poly.size() - 2),
                    seg1Poly.getLat(seg1Poly.size() - 1), seg1Poly.getLon(seg1Poly.size() - 1));
        }
        System.out.printf("  Seg1 end heading: %.5f (API used: 78.02217)%n", seg1EndHeading);

        // Segment 2: wp2 -> wp3 with heading
        System.out.println("\nSegment 2 (trailmap_foot, heading=78.02, penalty=60):");
        double seg2StartLat = seg1Poly.getLat(seg1Poly.size() - 1);
        double seg2StartLng = seg1Poly.getLon(seg1Poly.size() - 1);
        GHRequest req2 = new GHRequest(seg2StartLat, seg2StartLng, 61.501256, 23.685815)
                .setProfile("trailmap_foot");
        req2.setSnapPreventions(List.of("ferry"));
        req2.setHeadings(List.of(78.02216610253362, Double.NaN));
        req2.putHint("heading_penalty", 60.0);
        GHResponse rsp2 = hopper.route(req2);
        assertFalse(rsp2.hasErrors(), "Seg2 routing failed: " + rsp2.getErrors());
        printStdInstructions(rsp2.getBest());

        // Count non-FINISH, non-CONTINUE_ON_STREET turn instructions
        int turnCount = 0;
        for (Instruction instr : result.instructions) {
            int sign = instr.getSign();
            if (sign != Instruction.FINISH && sign != Instruction.CONTINUE_ON_STREET
                    && sign != Instruction.IGNORE) {
                turnCount++;
            }
        }
        System.out.println("\nTurn instructions (excluding FINISH/CONTINUE): " + turnCount);
        System.out.println("Expected: at least 4 turns (2 lefts + 1 right + 1 left near end)");
    }

    /**
     * Diagnostic test for foot route instruction quality issues.
     *
     * API payload:
     *   3 waypoints, 2 segments (both trailmap_foot), instruction_profile=trailmap_foot, locale=fi
     *   Segment 2 has initial_heading=322.45 and heading_penalty=60
     *
     * Known issues:
     *   1. 3rd instruction: "join right" — but this is actually a T-junction (cycleway ends)
     *      followed by a left turn ~20m later. Not a "join" pattern.
     *   2. 2nd-last instruction: compound "right then left 5m" — this IS a classic
     *      "join cycleway on right" (road continues, very short distance).
     */
    @Test
    void testFootRouteJoinInstructionQuality() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        // Build request matching the API payload exactly
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint w1 = makeWaypoint("XJzM57ukSay7zGCK5JrvZ", 61.506394, 23.697305);
        TrailmapInstructionRequest.Waypoint w2 = makeWaypoint("L2qKQ-5ffv3xUiB3aRFGh", 61.50716, 23.698392);
        TrailmapInstructionRequest.Waypoint w3 = makeWaypoint("Wk_mLUvB6iVY2uA0GlrqC", 61.506078, 23.69144);
        request.setWaypoints(List.of(w1, w2, w3));

        // Segment 1: wp1 -> wp2, trailmap_foot, no heading
        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("XJzM57ukSay7zGCK5JrvZ");
        seg1.setEnd("L2qKQ-5ffv3xUiB3aRFGh");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("trailmap_foot");

        // Segment 2: wp2 -> wp3, trailmap_foot, initial_heading + heading_penalty
        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("L2qKQ-5ffv3xUiB3aRFGh");
        seg2.setEnd("Wk_mLUvB6iVY2uA0GlrqC");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2.setProfile("trailmap_foot");
        seg2.setInitialHeading(322.45489185598296);
        seg2.setHeadingPenalty(60.0);

        request.setSegments(List.of(seg1, seg2));
        request.setInstructionProfile("trailmap_foot");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        // --- BEFORE post-processing ---
        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== FOOT JOIN QUALITY — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        System.out.println("Polyline points: " + result.polyline.size());
        printInstructionsDetailed(result);

        // --- AFTER post-processing ---
        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions);

        System.out.println("\n========== FOOT JOIN QUALITY — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        // --- Per-segment standard GH instructions for comparison ---
        System.out.println("\n--- Per-segment standard GH instructions for comparison ---");

        // Segment 1: wp1 -> wp2
        System.out.println("\nSegment 1 (trailmap_foot, no heading):");
        GHRequest req1 = new GHRequest(61.506394, 23.697305, 61.50716, 23.698392)
                .setProfile("trailmap_foot");
        req1.setSnapPreventions(List.of("ferry"));
        GHResponse rsp1 = hopper.route(req1);
        assertFalse(rsp1.hasErrors(), "Seg1 routing failed: " + rsp1.getErrors());
        printStdInstructions(rsp1.getBest());

        // Compute seg1 end heading
        PointList seg1Poly = rsp1.getBest().getPoints();
        double seg1EndHeading = Double.NaN;
        if (seg1Poly.size() >= 2) {
            seg1EndHeading = AngleCalc.ANGLE_CALC.calcAzimuth(
                    seg1Poly.getLat(seg1Poly.size() - 2), seg1Poly.getLon(seg1Poly.size() - 2),
                    seg1Poly.getLat(seg1Poly.size() - 1), seg1Poly.getLon(seg1Poly.size() - 1));
        }
        double seg1SnappedEndLat = seg1Poly.getLat(seg1Poly.size() - 1);
        double seg1SnappedEndLng = seg1Poly.getLon(seg1Poly.size() - 1);
        System.out.printf("  Seg1 snapped end: (%.7f, %.7f), end heading: %.5f (API used: 322.45489)%n",
                seg1SnappedEndLat, seg1SnappedEndLng, seg1EndHeading);

        // Segment 2: wp2 -> wp3 with heading
        System.out.println("\nSegment 2 (trailmap_foot, heading=322.45, penalty=60):");
        GHRequest req2 = new GHRequest(seg1SnappedEndLat, seg1SnappedEndLng, 61.506078, 23.69144)
                .setProfile("trailmap_foot");
        req2.setSnapPreventions(List.of("ferry"));
        req2.setHeadings(List.of(322.45489185598296, Double.NaN));
        req2.putHint("heading_penalty", 60.0);
        GHResponse rsp2 = hopper.route(req2);
        assertFalse(rsp2.hasErrors(), "Seg2 routing failed: " + rsp2.getErrors());
        printStdInstructions(rsp2.getBest());

        // --- Edge ID analysis at boundary ---
        System.out.println("\n--- Edge ID analysis ---");

        GHRequest req1e = new GHRequest(61.506394, 23.697305, 61.50716, 23.698392)
                .setProfile("trailmap_foot");
        req1e.setSnapPreventions(List.of("ferry"));
        req1e.setPathDetails(List.of("edge_id"));
        req1e.putHint("instructions", false);
        req1e.putHint("calc_points", true);
        GHResponse rsp1e = hopper.route(req1e);
        List<PathDetail> seg1Edges = rsp1e.getBest().getPathDetails().get("edge_id");

        GHRequest req2e = new GHRequest(seg1SnappedEndLat, seg1SnappedEndLng, 61.506078, 23.69144)
                .setProfile("trailmap_foot");
        req2e.setSnapPreventions(List.of("ferry"));
        req2e.setHeadings(List.of(322.45489185598296, Double.NaN));
        req2e.putHint("heading_penalty", 60.0);
        req2e.setPathDetails(List.of("edge_id"));
        req2e.putHint("instructions", false);
        req2e.putHint("calc_points", true);
        GHResponse rsp2e = hopper.route(req2e);
        List<PathDetail> seg2Edges = rsp2e.getBest().getPathDetails().get("edge_id");

        System.out.println("Seg1 edge IDs (" + seg1Edges.size() + "): ");
        for (PathDetail d : seg1Edges) System.out.print(d.getValue() + " ");
        System.out.println();

        System.out.println("Seg2 edge IDs (" + seg2Edges.size() + "): ");
        for (PathDetail d : seg2Edges) System.out.print(d.getValue() + " ");
        System.out.println();

        System.out.println("\nBoundary: seg1 last=" + seg1Edges.get(seg1Edges.size()-1).getValue()
                + " seg2 first=" + seg2Edges.get(0).getValue());

        // --- Polyline with inter-point distances ---
        System.out.println("\n--- Full polyline with distances ---");
        PointList poly = result.polyline;
        double cumDist = 0;
        for (int i = 0; i < poly.size(); i++) {
            double dist = 0;
            if (i > 0) {
                dist = DistanceCalcEarth.DIST_EARTH.calcDist(
                        poly.getLat(i-1), poly.getLon(i-1), poly.getLat(i), poly.getLon(i));
            }
            cumDist += dist;
            System.out.printf("  [%d] (%.7f, %.7f) step=%.1fm cumul=%.1fm%n",
                    i, poly.getLat(i), poly.getLon(i), dist, cumDist);
        }
    }

    /**
     * Diagnostic test for waypoint snap stub artifact.
     *
     * Scenario: 3 waypoints going roughly south→north. The middle waypoint (wp2) snaps
     * to a crossing east-west road at a junction, offset ~5-15m from the through-road.
     * This creates a short detour: turn onto crossing road → U-turn → turn back.
     * The route polyline barely deviates visually, but instruction generation produces
     * confusing left-uturn-left (or similar) instructions.
     *
     * API payload (trailmap_foot, Tampere area):
     * wp1: (61.503523, 23.691088) → wp2: (61.504324, 23.691833) → wp3: (61.505278, 23.693146)
     * Segment 2 has initial_heading=299.645 and heading_penalty=60
     *
     * Goal: understand the instruction sequence generated at the wp2 boundary,
     * validate that a U-turn is present, and characterize the stub pattern
     * for designing a Stage 2 suppression pass.
     */
    @Test
    void testWaypointSnapStubAtJunction() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        // Build request matching the API payload exactly
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("yzNi3rVcB52VdBO9T0KFY", 61.503523, 23.691088);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("Qc1FvwZKTiihKgjZTISK5", 61.504324, 23.691833);
        TrailmapInstructionRequest.Waypoint wp3 = makeWaypoint("sPyZx5FCs_8q3oh289nTn", 61.505278, 23.693146);
        request.setWaypoints(List.of(wp1, wp2, wp3));

        // Segment 1: wp1 -> wp2, trailmap_foot, no heading
        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("yzNi3rVcB52VdBO9T0KFY");
        seg1.setEnd("Qc1FvwZKTiihKgjZTISK5");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("trailmap_foot");

        // Segment 2: wp2 -> wp3, trailmap_foot, initial_heading + heading_penalty
        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("Qc1FvwZKTiihKgjZTISK5");
        seg2.setEnd("sPyZx5FCs_8q3oh289nTn");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2.setProfile("trailmap_foot");
        seg2.setInitialHeading(299.64515003038747);
        seg2.setHeadingPenalty(60.0);

        request.setSegments(List.of(seg1, seg2));
        request.setInstructionProfile("trailmap_foot");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        // ========== STEP 1: Full pipeline — BEFORE post-processing ==========
        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== WAYPOINT SNAP STUB — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        System.out.println("Polyline points: " + result.polyline.size());
        printInstructionsDetailed(result);

        // ========== STEP 2: Full pipeline — AFTER post-processing ==========
        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions);

        System.out.println("\n========== WAYPOINT SNAP STUB — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        // ========== STEP 3: Scan for U-turn instructions to confirm stub pattern ==========
        System.out.println("\n========== STUB PATTERN ANALYSIS ==========");
        boolean foundUturn = false;
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            int sign = instr.getSign();
            if (sign == Instruction.U_TURN_UNKNOWN || sign == Instruction.U_TURN_LEFT || sign == Instruction.U_TURN_RIGHT) {
                foundUturn = true;
                System.out.printf("U-turn found at instruction [%d]: sign=%d (%s) dist=%.1fm%n",
                        i, sign, signName(sign), instr.getDistance());

                // Print the surrounding context (instruction before and after)
                if (i > 0) {
                    Instruction prev = result.instructions.get(i - 1);
                    System.out.printf("  BEFORE [%d]: sign=%d (%s) dist=%.1fm name=\"%s\"%n",
                            i - 1, prev.getSign(), signName(prev.getSign()),
                            prev.getDistance(), prev.getName());
                }
                if (i + 1 < result.instructions.size()) {
                    Instruction next = result.instructions.get(i + 1);
                    System.out.printf("  AFTER  [%d]: sign=%d (%s) dist=%.1fm name=\"%s\"%n",
                            i + 1, next.getSign(), signName(next.getSign()),
                            next.getDistance(), next.getName());
                }

                // Compute total stub distance (instruction before U-turn + U-turn itself)
                if (i > 0) {
                    double stubDist = result.instructions.get(i - 1).getDistance() + instr.getDistance();
                    System.out.printf("  Stub round-trip distance (prev + uturn): %.1fm%n", stubDist);
                }
            }
        }
        if (!foundUturn) {
            System.out.println("WARNING: No U-turn instruction found — stub may have been absorbed or pattern differs");
        }

        // ========== STEP 4: Per-segment standard GH routing for comparison ==========
        System.out.println("\n--- Per-segment standard GH instructions for comparison ---");

        // Segment 1: wp1 -> wp2
        System.out.println("\nSegment 1 (trailmap_foot, no heading):");
        GHRequest req1 = new GHRequest(61.503523, 23.691088, 61.504324, 23.691833)
                .setProfile("trailmap_foot");
        req1.setSnapPreventions(List.of("ferry"));
        GHResponse rsp1 = hopper.route(req1);
        assertFalse(rsp1.hasErrors(), "Seg1 routing failed: " + rsp1.getErrors());
        printStdInstructions(rsp1.getBest());

        // Compute seg1 end heading and snapped end point
        PointList seg1Poly = rsp1.getBest().getPoints();
        double seg1EndHeading = Double.NaN;
        if (seg1Poly.size() >= 2) {
            seg1EndHeading = AngleCalc.ANGLE_CALC.calcAzimuth(
                    seg1Poly.getLat(seg1Poly.size() - 2), seg1Poly.getLon(seg1Poly.size() - 2),
                    seg1Poly.getLat(seg1Poly.size() - 1), seg1Poly.getLon(seg1Poly.size() - 1));
        }
        double seg1SnappedEndLat = seg1Poly.getLat(seg1Poly.size() - 1);
        double seg1SnappedEndLng = seg1Poly.getLon(seg1Poly.size() - 1);
        System.out.printf("  Seg1 snapped end: (%.7f, %.7f), end heading: %.5f (API payload: 299.64515)%n",
                seg1SnappedEndLat, seg1SnappedEndLng, seg1EndHeading);

        // Segment 2: wp2 -> wp3 with heading
        System.out.println("\nSegment 2 (trailmap_foot, heading=299.645, penalty=60):");
        GHRequest req2 = new GHRequest(seg1SnappedEndLat, seg1SnappedEndLng, 61.505278, 23.693146)
                .setProfile("trailmap_foot");
        req2.setSnapPreventions(List.of("ferry"));
        req2.setHeadings(List.of(299.64515003038747, Double.NaN));
        req2.putHint("heading_penalty", 60.0);
        GHResponse rsp2 = hopper.route(req2);
        assertFalse(rsp2.hasErrors(), "Seg2 routing failed: " + rsp2.getErrors());
        printStdInstructions(rsp2.getBest());

        // Segment 2 WITHOUT heading for comparison
        System.out.println("\nSegment 2 WITHOUT heading (natural route):");
        GHRequest req2NoHeading = new GHRequest(seg1SnappedEndLat, seg1SnappedEndLng, 61.505278, 23.693146)
                .setProfile("trailmap_foot");
        req2NoHeading.setSnapPreventions(List.of("ferry"));
        GHResponse rsp2NoHeading = hopper.route(req2NoHeading);
        if (!rsp2NoHeading.hasErrors()) {
            printStdInstructions(rsp2NoHeading.getBest());
        }

        // ========== STEP 5: Edge ID analysis at boundary ==========
        System.out.println("\n--- Edge ID analysis ---");

        // Seg1 edge IDs
        GHRequest req1Edges = new GHRequest(61.503523, 23.691088, 61.504324, 23.691833)
                .setProfile("trailmap_foot");
        req1Edges.setSnapPreventions(List.of("ferry"));
        req1Edges.setPathDetails(List.of("edge_id"));
        req1Edges.putHint("instructions", false);
        req1Edges.putHint("calc_points", true);
        GHResponse rsp1Edges = hopper.route(req1Edges);
        assertFalse(rsp1Edges.hasErrors());
        List<PathDetail> seg1EdgeDetails = rsp1Edges.getBest().getPathDetails().get("edge_id");
        System.out.println("Seg1 edge IDs (" + seg1EdgeDetails.size() + "):");
        for (PathDetail d : seg1EdgeDetails) {
            System.out.print("  " + d.getValue());
        }
        System.out.println();

        // Seg2 edge IDs (with heading, from snapped end of seg1)
        GHRequest req2Edges = new GHRequest(seg1SnappedEndLat, seg1SnappedEndLng, 61.505278, 23.693146)
                .setProfile("trailmap_foot");
        req2Edges.setSnapPreventions(List.of("ferry"));
        req2Edges.setPathDetails(List.of("edge_id"));
        req2Edges.putHint("instructions", false);
        req2Edges.putHint("calc_points", true);
        req2Edges.setHeadings(List.of(299.64515003038747, Double.NaN));
        req2Edges.putHint("heading_penalty", 60.0);
        GHResponse rsp2Edges = hopper.route(req2Edges);
        assertFalse(rsp2Edges.hasErrors());
        List<PathDetail> seg2EdgeDetails = rsp2Edges.getBest().getPathDetails().get("edge_id");
        System.out.println("Seg2 edge IDs (" + seg2EdgeDetails.size() + "):");
        for (PathDetail d : seg2EdgeDetails) {
            System.out.print("  " + d.getValue());
        }
        System.out.println();

        // Check for shared/overlapping edges at boundary
        if (!seg1EdgeDetails.isEmpty() && !seg2EdgeDetails.isEmpty()) {
            int lastSeg1Edge = (int) seg1EdgeDetails.get(seg1EdgeDetails.size() - 1).getValue();
            int firstSeg2Edge = (int) seg2EdgeDetails.get(0).getValue();
            System.out.printf("Boundary: seg1 last edge=%d, seg2 first edge=%d, same=%b%n",
                    lastSeg1Edge, firstSeg2Edge, lastSeg1Edge == firstSeg2Edge);
        }

        // ========== STEP 6: Full polyline with inter-point distances ==========
        System.out.println("\n--- Full polyline with distances (around wp2 area) ---");
        PointList poly = result.polyline;
        double cumDist = 0;
        for (int i = 0; i < poly.size(); i++) {
            double dist = 0;
            if (i > 0) {
                dist = DistanceCalcEarth.DIST_EARTH.calcDist(
                        poly.getLat(i-1), poly.getLon(i-1), poly.getLat(i), poly.getLon(i));
            }
            cumDist += dist;
            System.out.printf("  [%d] (%.7f, %.7f) step=%.1fm cumul=%.1fm%n",
                    i, poly.getLat(i), poly.getLon(i), dist, cumDist);
        }
    }

    /**
     * Diagnostic test: fork near end of route where a track diverges from a path.
     *
     * Route: (61.518776, 23.611821) -> (61.519755, 23.610541) with MTB profile.
     * Goal: check whether an instruction is generated at the fork between track and path.
     */
    @Test
    void testForkNearEnd_track_vs_path() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        // Build request matching the API payload exactly
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("3yo44X_wd_4wdTWbBCQxc", 61.518972, 23.611517);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("CSDKBBuUaFYYYpGBAToOy", 61.51958, 23.610079);
        request.setWaypoints(List.of(wp1, wp2));

        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("3yo44X_wd_4wdTWbBCQxc");
        seg1.setEnd("CSDKBBuUaFYYYpGBAToOy");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("mtb");

        request.setSegments(List.of(seg1));
        request.setInstructionProfile("mtb");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        // Generate instructions (before post-processing)
        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== FORK NEAR END (track vs path) — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        System.out.println("Polyline points: " + result.polyline.size());
        printInstructionsDetailed(result);

        // Print polyline with distances
        System.out.println("\n--- Polyline coordinates ---");
        PointList poly = result.polyline;
        double cumDist = 0;
        for (int i = 0; i < poly.size(); i++) {
            double dist = 0;
            if (i > 0) {
                dist = DistanceCalcEarth.DIST_EARTH.calcDist(
                        poly.getLat(i-1), poly.getLon(i-1), poly.getLat(i), poly.getLon(i));
            }
            cumDist += dist;
            System.out.printf("  [%d] (%.7f, %.7f) step=%.1fm cumul=%.1fm%n",
                    i, poly.getLat(i), poly.getLon(i), dist, cumDist);
        }

        // Get edge IDs for the route
        System.out.println("\n--- Edge details ---");
        GHRequest edgeReq = new GHRequest(61.518972, 23.611517, 61.51958, 23.610079);
        edgeReq.setProfile("mtb");
        edgeReq.setSnapPreventions(List.of("ferry"));
        edgeReq.setPathDetails(List.of("edge_id", "road_class", "predicted_highway"));
        edgeReq.putHint("instructions", false);
        edgeReq.putHint("calc_points", true);
        GHResponse edgeRsp = hopper.route(edgeReq);
        assertFalse(edgeRsp.hasErrors());
        for (String key : List.of("edge_id", "road_class", "predicted_highway")) {
            List<PathDetail> details = edgeRsp.getBest().getPathDetails().get(key);
            System.out.println(key + " (" + details.size() + " segments):");
            for (PathDetail d : details) {
                System.out.printf("  [%d-%d] %s%n", d.getFirst(), d.getLast(), d.getValue());
            }
        }

        // Inspect junction alternatives along the route — with angle data
        System.out.println("\n--- Junction inspection (with angles) ---");
        BaseGraph bg = hopper.getBaseGraph();
        EnumEncodedValue<RoadClass> rcEnc = encodingManager.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<PredictedHighway> phEnc = encodingManager.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
        BooleanEncodedValue accessEnc = encodingManager.getBooleanEncodedValue(VehicleAccess.key("bike"));
        List<PathDetail> edgeIds = edgeRsp.getBest().getPathDetails().get("edge_id");
        Set<Integer> routeEdgeIds = new HashSet<>();
        for (PathDetail d : edgeIds) routeEdgeIds.add((Integer) d.getValue());

        EdgeExplorer explorer = bg.createEdgeExplorer();
        int prevEdgeId = -1;
        for (PathDetail d : edgeIds) {
            int edgeId = (Integer) d.getValue();
            EdgeIteratorState routeEdge = bg.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            int adjNode = routeEdge.getAdjNode();

            // Compute incoming bearing from the previous route edge into this node
            double incomingBearing = Double.NaN;
            if (prevEdgeId >= 0) {
                // Use last two polyline points of previous edge segment for incoming orientation
                EdgeIteratorState prevRouteEdge = bg.getEdgeIteratorState(prevEdgeId, Integer.MIN_VALUE);
                PointList prevPl = prevRouteEdge.fetchWayGeometry(com.graphhopper.util.FetchMode.ALL);
                if (prevPl.size() >= 2) {
                    double lat1 = prevPl.getLat(prevPl.size() - 2);
                    double lon1 = prevPl.getLon(prevPl.size() - 2);
                    double lat2 = prevPl.getLat(prevPl.size() - 1);
                    double lon2 = prevPl.getLon(prevPl.size() - 1);
                    incomingBearing = com.graphhopper.util.AngleCalc.ANGLE_CALC.calcAzimuth(lat1, lon1, lat2, lon2);
                }
            }

            EdgeIterator iter = explorer.setBaseNode(adjNode);
            int altCount = 0;
            StringBuilder altInfo = new StringBuilder();
            while (iter.next()) {
                if (iter.getEdge() == edgeId) continue;
                boolean isRouteEdge = routeEdgeIds.contains(iter.getEdge());
                RoadClass altRC = iter.get(rcEnc);
                PredictedHighway altPH = iter.get(phEnc);
                String name = iter.getName();

                // Compute bearing of this outgoing edge
                String angleStr = "";
                if (!Double.isNaN(incomingBearing)) {
                    PointList altPl = iter.fetchWayGeometry(com.graphhopper.util.FetchMode.ALL);
                    if (altPl.size() >= 2) {
                        double aLat = altPl.getLat(0);
                        double aLon = altPl.getLon(0);
                        double bLat = altPl.getLat(1);
                        double bLon = altPl.getLon(1);
                        double outBearing = com.graphhopper.util.AngleCalc.ANGLE_CALC.calcAzimuth(aLat, aLon, bLat, bLon);
                        double delta = outBearing - incomingBearing;
                        if (delta > 180) delta -= 360;
                        if (delta < -180) delta += 360;
                        angleStr = String.format(" angle=%.1f°", delta);
                    }
                }

                altInfo.append(String.format("    edge=%d rc=%s ph=%s name=\"%s\" route=%b%s%n",
                        iter.getEdge(), altRC, altPH, name, isRouteEdge, angleStr));
                altCount++;
            }
            if (altCount > 1) {
                double nodeLat = bg.getNodeAccess().getLat(adjNode);
                double nodeLon = bg.getNodeAccess().getLon(adjNode);
                System.out.printf("Junction at node %d (%.7f, %.7f) after edge %d (inBearing=%.1f):%n",
                        adjNode, nodeLat, nodeLon, edgeId,
                        Double.isNaN(incomingBearing) ? 0.0 : incomingBearing);
                System.out.print(altInfo);
            }
            prevEdgeId = edgeId;
        }

        // Apply post-processing
        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions);

        System.out.println("\n========== FORK NEAR END (track vs path) — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        // The last instruction should be FINISH
        Instruction lastInstr = result.instructions.get(result.instructions.size() - 1);
        assertEquals(Instruction.FINISH, lastInstr.getSign(), "Last instruction should be FINISH");
    }

    // ========================================================================
    // Direct segment tests
    // ========================================================================

    /**
     * Route with a direct segment in the middle: followRoads → direct → followRoads.
     * Reproduces the 3D PointList crash and verifies the synthetic instruction.
     *
     * Uses the exact coordinates from the reported failing API request.
     */
    @Test
    void testDirectSegmentMiddle() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        // Build request: followRoads → direct → followRoads
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.499292, 23.677684),
                makeWaypoint("wp2", 61.499559, 23.676201),
                makeWaypoint("wp3", 61.499610, 23.670164),
                makeWaypoint("wp4", 61.500511, 23.677141)
        ));

        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("wp1");
        seg1.setEnd("wp2");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("gravel_mtb");

        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("wp2");
        seg2.setEnd("wp3");
        seg2.setType(TrailmapInstructionRequest.TYPE_DIRECT);

        TrailmapInstructionRequest.Segment seg3 = new TrailmapInstructionRequest.Segment();
        seg3.setStart("wp3");
        seg3.setEnd("wp4");
        seg3.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg3.setProfile("gravel_mtb");

        request.setSegments(List.of(seg1, seg2, seg3));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        // This used to throw: "Cannot add point without elevation data in 3D mode"
        RouteInstructionGenerator.Result result = generator.generate(request);

        assertNotNull(result);
        assertTrue(result.polyline.size() > 0, "Should produce a polyline");
        assertTrue(result.polyline.is3D(), "Polyline should be 3D");
        assertTrue(result.instructions.size() >= 3, "Should have at least 3 instructions (route + direct + FINISH)");

        // Find the direct segment instruction
        Instruction directInstr = null;
        int directIndex = -1;
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            if (Boolean.FALSE.equals(instr.getExtraInfoJSON().get("tbt_available"))) {
                directInstr = instr;
                directIndex = i;
                break;
            }
        }
        assertNotNull(directInstr, "Should have a direct segment instruction");
        assertEquals(Instruction.CONTINUE_ON_STREET, directInstr.getSign());
        assertEquals("direct", directInstr.getExtraInfoJSON().get("segment_type"));
        assertEquals(false, directInstr.getExtraInfoJSON().get("tbt_available"));
        assertEquals("entering_direct_segment", directInstr.getExtraInfoJSON().get("confirm_reason"));
        assertTrue(directInstr.getDistance() > 0, "Direct segment should have positive distance");

        // The instruction before the direct should have next_segment_type
        if (directIndex > 0) {
            Instruction beforeDirect = result.instructions.get(directIndex - 1);
            assertEquals("direct", beforeDirect.getExtraInfoJSON().get("next_segment_type"),
                    "Instruction before direct should have next_segment_type");
        }

        // The instruction after the direct should have tbt_resumed
        if (directIndex + 1 < result.instructions.size()) {
            Instruction afterDirect = result.instructions.get(directIndex + 1);
            if (afterDirect.getSign() != Instruction.FINISH) {
                assertEquals(true, afterDirect.getExtraInfoJSON().get("tbt_resumed"),
                        "First instruction after direct should have tbt_resumed");
                assertEquals("direct", afterDirect.getExtraInfoJSON().get("prev_segment_type"),
                        "First instruction after direct should have prev_segment_type");
            }
        }

        // Last instruction should be FINISH
        Instruction lastInstr = result.instructions.get(result.instructions.size() - 1);
        assertEquals(Instruction.FINISH, lastInstr.getSign());

        // Total instruction distances should match polyline distance
        double totalInstrDist = 0;
        for (Instruction instr : result.instructions) {
            totalInstrDist += instr.getDistance();
        }
        double polylineDist = 0;
        PointList poly = result.polyline;
        for (int i = 0; i < poly.size() - 1; i++) {
            polylineDist += DistanceCalcEarth.DIST_EARTH.calcDist(
                    poly.getLat(i), poly.getLon(i), poly.getLat(i + 1), poly.getLon(i + 1));
        }
        assertEquals(polylineDist, totalInstrDist, 2.0,
                "Total instruction distance should match polyline distance within 2m");

        // Apply post-processing and verify direct instruction survives
        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions);

        boolean foundDirectAfterPostProcess = false;
        for (Instruction instr : result.instructions) {
            if (Boolean.FALSE.equals(instr.getExtraInfoJSON().get("tbt_available"))) {
                foundDirectAfterPostProcess = true;
                assertEquals("voice", instr.getExtraInfoJSON().get("tbt_priority"),
                        "Direct segment instruction should have voice priority after post-processing");
                break;
            }
        }
        assertTrue(foundDirectAfterPostProcess,
                "Direct segment instruction should survive post-processing");

        System.out.println("\n=== Direct segment middle test ===");
        printInstructionsDetailed(result);
    }

    /**
     * Route that starts with a direct segment: direct → followRoads.
     * Verifies no crash when there's no preceding instruction to annotate.
     */
    @Test
    void testDirectSegmentAtStart() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.499292, 23.677684),
                makeWaypoint("wp2", 61.499559, 23.676201),
                makeWaypoint("wp3", 61.500511, 23.677141)
        ));

        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("wp1");
        seg1.setEnd("wp2");
        seg1.setType(TrailmapInstructionRequest.TYPE_DIRECT);

        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("wp2");
        seg2.setEnd("wp3");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2.setProfile("gravel_mtb");

        request.setSegments(List.of(seg1, seg2));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");

        RouteInstructionGenerator.Result result = generator.generate(request);

        assertNotNull(result);
        assertTrue(result.instructions.size() >= 2, "Should have at least direct + FINISH");

        // First instruction should be the direct segment
        Instruction first = result.instructions.get(0);
        assertEquals(false, first.getExtraInfoJSON().get("tbt_available"),
                "First instruction should be the direct segment");

        // Second instruction (first routed) should have tbt_resumed
        if (result.instructions.size() > 1) {
            Instruction second = result.instructions.get(1);
            if (second.getSign() != Instruction.FINISH) {
                assertEquals(true, second.getExtraInfoJSON().get("tbt_resumed"),
                        "First routed instruction should have tbt_resumed");
            }
        }

        System.out.println("\n=== Direct segment at start test ===");
        printInstructionsDetailed(result);
    }

    /**
     * Route that ends with a direct segment: followRoads → direct.
     * Verifies FINISH is still present and the direct instruction has no tbt_resumed after it.
     */
    @Test
    void testDirectSegmentAtEnd() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.499292, 23.677684),
                makeWaypoint("wp2", 61.499559, 23.676201),
                makeWaypoint("wp3", 61.499610, 23.670164)
        ));

        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("wp1");
        seg1.setEnd("wp2");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("gravel_mtb");

        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("wp2");
        seg2.setEnd("wp3");
        seg2.setType(TrailmapInstructionRequest.TYPE_DIRECT);

        request.setSegments(List.of(seg1, seg2));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");

        RouteInstructionGenerator.Result result = generator.generate(request);

        assertNotNull(result);

        // Should have a direct instruction
        boolean foundDirect = false;
        for (Instruction instr : result.instructions) {
            if (Boolean.FALSE.equals(instr.getExtraInfoJSON().get("tbt_available"))) {
                foundDirect = true;
                break;
            }
        }
        assertTrue(foundDirect, "Should have a direct segment instruction");

        // Last instruction should be FINISH (from the routed segment)
        Instruction lastInstr = result.instructions.get(result.instructions.size() - 1);
        assertEquals(Instruction.FINISH, lastInstr.getSign(), "Last instruction should be FINISH");

        System.out.println("\n=== Direct segment at end test ===");
        printInstructionsDetailed(result);
    }

    /**
     * Diagnostic: short gravel route where cycleway meets road.
     * Issue: produces KEEP_RIGHT instead of TURN_SLIGHT_RIGHT when PredictedHighway changes.
     *
     * API payload:
     *   2 waypoints, 1 segment (gravel), custom_model with cycleway priority boost,
     *   instruction_profile=gravel, locale=fi, snap_preventions=["ferry"]
     */
    @Test
    void testCyclewayToRoadKeepRight() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("sahd_DFqI6u4yi_VHFc5e", 61.467213, 23.653461);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("ckgzlFu9_12oHhCOKDSIS", 61.467741, 23.653907);
        request.setWaypoints(List.of(wp1, wp2));

        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_highway == CYCLEWAY",
                Statement.Op.MULTIPLY, "1.3"));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("sahd_DFqI6u4yi_VHFc5e");
        seg.setEnd("ckgzlFu9_12oHhCOKDSIS");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");
        seg.setCustomModel(cm);

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertNotNull(result.instructions);

        System.out.println("\n========== CYCLEWAY→ROAD KEEP_RIGHT ISSUE — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions);

        System.out.println("\n========== CYCLEWAY→ROAD KEEP_RIGHT ISSUE — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);
    }

    /**
     * Diagnostic: road to cycleway, road continues straight.
     * Issue: "keep right" is misleading when turning off a road onto a cycleway.
     */
    @Test
    void testRoadToCyclewayKeepRight() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("kvG9Jq3Yxd0kpVBk1tC4G", 61.473361, 23.690552);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("0TGGhWRAkHvHgOE7zP01Y", 61.473788, 23.692248);
        request.setWaypoints(List.of(wp1, wp2));

        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_highway == CYCLEWAY",
                Statement.Op.MULTIPLY, "1.3"));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("kvG9Jq3Yxd0kpVBk1tC4G");
        seg.setEnd("0TGGhWRAkHvHgOE7zP01Y");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");
        seg.setCustomModel(cm);

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);

        System.out.println("\n========== ROAD→CYCLEWAY KEEP_RIGHT — BEFORE POST-PROCESSING ==========");
        printInstructionsDetailed(result);

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions);

        System.out.println("\n========== ROAD→CYCLEWAY KEEP_RIGHT — AFTER POST-PROCESSING ==========");
        printInstructionsDetailed(result);
    }

    /**
     * Diagnostic: asphalt cycleway to unpaved cycleway, clear left turn.
     * Issue: "keep left" when inbound cycleway continues straight — should be "left".
     */
    @Test
    void testAsphaltToUnpavedCyclewayKeepLeft() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("T26qdATOrCeF4gi7h5T5d", 61.486074, 23.760044);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("Ws9lDw3hY1HsRK952LcJD", 61.486355, 23.760642);
        request.setWaypoints(List.of(wp1, wp2));

        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_highway == CYCLEWAY",
                Statement.Op.MULTIPLY, "1.3"));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("T26qdATOrCeF4gi7h5T5d");
        seg.setEnd("Ws9lDw3hY1HsRK952LcJD");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");
        seg.setCustomModel(cm);

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);

        System.out.println("\n========== ASPHALT→UNPAVED CYCLEWAY KEEP_LEFT — BEFORE POST-PROCESSING ==========");
        printInstructionsDetailed(result);

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions);

        System.out.println("\n========== ASPHALT→UNPAVED CYCLEWAY KEEP_LEFT — AFTER POST-PROCESSING ==========");
        printInstructionsDetailed(result);
    }

    /**
     * Diagnostic: complex turns — tight right, then super-tight left, then right to path.
     * UI shows "join left path" which doesn't match the reality of 3 distinct turns.
     */
    @Test
    void testComplexTurnsFootRoute() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("VDwykGPfUqD8-wiLUlM7E", 61.499065, 23.631531);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("IQA5kBUzOjN_guDR-XpKy", 61.499816, 23.631703);
        request.setWaypoints(List.of(wp1, wp2));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("VDwykGPfUqD8-wiLUlM7E");
        seg.setEnd("IQA5kBUzOjN_guDR-XpKy");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("trailmap_foot");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("trailmap_foot");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);

        System.out.println("\n========== COMPLEX TURNS FOOT ROUTE — BEFORE POST-PROCESSING ==========");
        printInstructionsDetailed(result);

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions);

        System.out.println("\n========== COMPLEX TURNS FOOT ROUTE — AFTER POST-PROCESSING ==========");
        printInstructionsDetailed(result);
    }

    /**
     * Diagnostic: cycleway to road, expected M1 join but not triggering.
     */
    @Test
    void testMissingJoinCyclewayToRoad() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("I3h4j1CpONoKolmGbcB9C", 61.503449, 23.684035);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("hNp7HohwC2AAR7d1E1ySH", 61.502604, 23.687161);
        request.setWaypoints(List.of(wp1, wp2));

        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_highway == CYCLEWAY",
                Statement.Op.MULTIPLY, "1.3"));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("I3h4j1CpONoKolmGbcB9C");
        seg.setEnd("hNp7HohwC2AAR7d1E1ySH");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");
        seg.setCustomModel(cm);
        seg.setInitialHeading(139.8063235836866);
        seg.setHeadingPenalty(60.0);

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);

        System.out.println("\n========== MISSING JOIN CW→ROAD — BEFORE POST-PROCESSING ==========");
        printInstructionsDetailed(result);

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions);

        System.out.println("\n========== MISSING JOIN CW→ROAD — AFTER POST-PROCESSING ==========");
        printInstructionsDetailed(result);
    }

    /**
     * Diagnostic: straight on asphalt cycleway, unpaved fork to the left.
     * Issue: "keep right" when going perfectly straight — should be suppressed or "continue straight".
     */
    @Test
    void testStraightCyclewayUnpavedForkKeepRight() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("I2ZUL26YRY9YO6JJiKHCU", 61.491062, 23.753742);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("s9BAuM3jcnO1Upb4TBsPt", 61.491025, 23.752542);
        request.setWaypoints(List.of(wp1, wp2));

        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_highway == CYCLEWAY",
                Statement.Op.MULTIPLY, "1.3"));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("I2ZUL26YRY9YO6JJiKHCU");
        seg.setEnd("s9BAuM3jcnO1Upb4TBsPt");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");
        seg.setCustomModel(cm);

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);

        System.out.println("\n========== STRAIGHT CYCLEWAY + UNPAVED FORK — BEFORE POST-PROCESSING ==========");
        printInstructionsDetailed(result);

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions);

        System.out.println("\n========== STRAIGHT CYCLEWAY + UNPAVED FORK — AFTER POST-PROCESSING ==========");
        printInstructionsDetailed(result);
    }

    // ========================================================================
    // Debug: polyline index investigation after consecutive direct segments
    // ========================================================================

    /**
     * Debug test for investigating wrong polyline indexes (intervals) on instructions
     * that follow a run of consecutive direct segments.
     *
     * Route structure: 2 followRoads → 8 direct → 3 followRoads
     * Reproduces exact API payload from reported issue.
     */
    @Test
    void testPolylineIndexAfterConsecutiveDirectSegments() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        // Build request matching API payload exactly
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        request.setWaypoints(List.of(
                makeWaypoint("FLBMDDyn94DFQxx1YdtxL", 61.514433, 23.581875),
                makeWaypoint("595lGL-su9RA-NnUsXzlh", 61.517204, 23.585335),
                makeWaypoint("Z9qPdEVqRwVgkr-EAu2BU", 61.517508, 23.586106),
                makeWaypoint("90-SjUk5a1L00Z3ijcgsP", 61.51891328984175, 23.586272450858047),
                makeWaypoint("O-iQMxY9nlcSQvqvGeFhm", 61.51977095040931, 23.58437534948368),
                makeWaypoint("wMAanVIHNX43pAk4KDUqQ", 61.52051110420439, 23.584473900204102),
                makeWaypoint("U8QjSy9Hw8iW_GLlUhvbG", 61.52095753765434, 23.58755361022807),
                makeWaypoint("fusYIHMQAaBbWi0S4tBf5", 61.5206990769648, 23.590830421692885),
                makeWaypoint("SziEAXeQuYr-gfL2kQLyz", 61.519982424720666, 23.591594189779016),
                makeWaypoint("VueBHX_-q2qTAw1UKw5IC", 61.51843157969307, 23.592702885387695),
                makeWaypoint("KAnSM2t5DM2s148tplIA7", 61.51704515272817, 23.59427969692007),
                makeWaypoint("694nh1iKysbtBW3aEGInE", 61.516024, 23.596282),
                makeWaypoint("KEQzGqdPawO8QguRYuxZt", 61.515548, 23.594369),
                makeWaypoint("afUAebI7WK0uszTLd-Ths", 61.516169, 23.592415)
        ));

        // Custom model used by all followRoads segments
        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_highway == CYCLEWAY",
                Statement.Op.MULTIPLY, "1.3"));

        // Segment 1: followRoads (no heading)
        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("FLBMDDyn94DFQxx1YdtxL");
        seg1.setEnd("595lGL-su9RA-NnUsXzlh");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("gravel");
        seg1.setCustomModel(cm);

        // Segment 2: followRoads (with heading)
        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("595lGL-su9RA-NnUsXzlh");
        seg2.setEnd("Z9qPdEVqRwVgkr-EAu2BU");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2.setProfile("gravel");
        seg2.setCustomModel(cm);
        seg2.setInitialHeading(90.62059343708455);
        seg2.setHeadingPenalty(60.0);

        // Segments 3-10: all direct
        TrailmapInstructionRequest.Segment seg3 = new TrailmapInstructionRequest.Segment();
        seg3.setStart("Z9qPdEVqRwVgkr-EAu2BU");
        seg3.setEnd("90-SjUk5a1L00Z3ijcgsP");
        seg3.setType(TrailmapInstructionRequest.TYPE_DIRECT);

        TrailmapInstructionRequest.Segment seg4 = new TrailmapInstructionRequest.Segment();
        seg4.setStart("90-SjUk5a1L00Z3ijcgsP");
        seg4.setEnd("O-iQMxY9nlcSQvqvGeFhm");
        seg4.setType(TrailmapInstructionRequest.TYPE_DIRECT);

        TrailmapInstructionRequest.Segment seg5 = new TrailmapInstructionRequest.Segment();
        seg5.setStart("O-iQMxY9nlcSQvqvGeFhm");
        seg5.setEnd("wMAanVIHNX43pAk4KDUqQ");
        seg5.setType(TrailmapInstructionRequest.TYPE_DIRECT);

        TrailmapInstructionRequest.Segment seg6 = new TrailmapInstructionRequest.Segment();
        seg6.setStart("wMAanVIHNX43pAk4KDUqQ");
        seg6.setEnd("U8QjSy9Hw8iW_GLlUhvbG");
        seg6.setType(TrailmapInstructionRequest.TYPE_DIRECT);

        TrailmapInstructionRequest.Segment seg7 = new TrailmapInstructionRequest.Segment();
        seg7.setStart("U8QjSy9Hw8iW_GLlUhvbG");
        seg7.setEnd("fusYIHMQAaBbWi0S4tBf5");
        seg7.setType(TrailmapInstructionRequest.TYPE_DIRECT);

        TrailmapInstructionRequest.Segment seg8 = new TrailmapInstructionRequest.Segment();
        seg8.setStart("fusYIHMQAaBbWi0S4tBf5");
        seg8.setEnd("SziEAXeQuYr-gfL2kQLyz");
        seg8.setType(TrailmapInstructionRequest.TYPE_DIRECT);

        TrailmapInstructionRequest.Segment seg9 = new TrailmapInstructionRequest.Segment();
        seg9.setStart("SziEAXeQuYr-gfL2kQLyz");
        seg9.setEnd("VueBHX_-q2qTAw1UKw5IC");
        seg9.setType(TrailmapInstructionRequest.TYPE_DIRECT);

        TrailmapInstructionRequest.Segment seg10 = new TrailmapInstructionRequest.Segment();
        seg10.setStart("VueBHX_-q2qTAw1UKw5IC");
        seg10.setEnd("KAnSM2t5DM2s148tplIA7");
        seg10.setType(TrailmapInstructionRequest.TYPE_DIRECT);

        // Segment 11: followRoads (no heading)
        TrailmapInstructionRequest.Segment seg11 = new TrailmapInstructionRequest.Segment();
        seg11.setStart("KAnSM2t5DM2s148tplIA7");
        seg11.setEnd("694nh1iKysbtBW3aEGInE");
        seg11.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg11.setProfile("gravel");
        seg11.setCustomModel(cm);

        // Segment 12: followRoads (with heading)
        TrailmapInstructionRequest.Segment seg12 = new TrailmapInstructionRequest.Segment();
        seg12.setStart("694nh1iKysbtBW3aEGInE");
        seg12.setEnd("KEQzGqdPawO8QguRYuxZt");
        seg12.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg12.setProfile("gravel");
        seg12.setCustomModel(cm);
        seg12.setInitialHeading(160.75524855722847);
        seg12.setHeadingPenalty(60.0);

        // Segment 13: followRoads (with heading)
        TrailmapInstructionRequest.Segment seg13 = new TrailmapInstructionRequest.Segment();
        seg13.setStart("KEQzGqdPawO8QguRYuxZt");
        seg13.setEnd("afUAebI7WK0uszTLd-Ths");
        seg13.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg13.setProfile("gravel");
        seg13.setCustomModel(cm);
        seg13.setInitialHeading(242.16485867817147);
        seg13.setHeadingPenalty(60.0);

        request.setSegments(List.of(seg1, seg2, seg3, seg4, seg5, seg6, seg7, seg8, seg9, seg10,
                seg11, seg12, seg13));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        // Generate instructions
        RouteInstructionGenerator.Result result = generator.generate(request);

        PointList poly = result.polyline;
        System.out.println("\n========== CONSECUTIVE DIRECT SEGMENTS — POLYLINE INDEX DEBUG ==========");
        System.out.println("Full polyline: " + poly.size() + " points");

        // Dump full polyline with indices
        System.out.println("\n--- Full Polyline Points ---");
        for (int i = 0; i < poly.size(); i++) {
            System.out.printf("  poly[%d] = (%.8f, %.8f)%n", i, poly.getLat(i), poly.getLon(i));
        }

        // --- Replay remapInstructionGeometry's coordinate matching logic ---
        // remapInstructionGeometry() already ran inside generate(). We can't see the
        // pre-remap instruction PointLists. But we CAN replay the matching algorithm
        // using the POST-remap first points (which are from the polyline slices that
        // remapInstructionGeometry assigned). This tells us the polyStart values.
        //
        // More usefully: we can check each instruction's remapped PointList against
        // the polyline to see which slice it was assigned.
        System.out.println("\n--- Replaying remapInstructionGeometry matching (post-remap) ---");
        List<Integer> replayedStarts = new ArrayList<>();
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            if (instr.getSign() == Instruction.FINISH) {
                replayedStarts.add(poly.size() - 1);
                System.out.printf("  instr[%d] FINISH -> polyStart=%d%n", i, poly.size() - 1);
                continue;
            }
            if (i == 0) {
                replayedStarts.add(0);
                System.out.printf("  instr[%d] first -> polyStart=0%n", i);
                continue;
            }
            PointList instrPts = instr.getPoints();
            if (instrPts.size() == 0) {
                int prev = replayedStarts.get(replayedStarts.size() - 1);
                replayedStarts.add(prev);
                System.out.printf("  instr[%d] empty -> polyStart=%d (same as prev)%n", i, prev);
                continue;
            }
            double targetLat = instrPts.getLat(0);
            double targetLon = instrPts.getLon(0);
            int searchFrom = replayedStarts.get(replayedStarts.size() - 1);
            int bestIdx = searchFrom;
            double bestDist = Double.MAX_VALUE;
            for (int pi = searchFrom; pi < poly.size(); pi++) {
                double dist = Math.abs(poly.getLat(pi) - targetLat)
                        + Math.abs(poly.getLon(pi) - targetLon);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestIdx = pi;
                }
                if (bestDist < 1e-6) break;
            }
            replayedStarts.add(bestIdx);
            boolean isDirect = Boolean.FALSE.equals(instr.getExtraInfoJSON().get("tbt_available"));
            System.out.printf("  instr[%d] target=(%.8f,%.8f) searchFrom=%d -> bestIdx=%d bestDist=%.2e %s%s%n",
                    i, targetLat, targetLon, searchFrom, bestIdx, bestDist,
                    bestDist < 1e-6 ? " EXACT" : " APPROX",
                    isDirect ? " [DIRECT]" : "");
        }

        // --- Detailed instruction dump with polyline slice analysis ---
        System.out.println("\n--- Instructions BEFORE post-processing (with polyline slice analysis) ---");
        int pointsIndex = 0;
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            int instrLen = instr.getLength();
            int end = pointsIndex + instrLen;

            PointList pts = instr.getPoints();
            String firstPt = pts.size() > 0
                    ? String.format("(%.8f, %.8f)", pts.getLat(0), pts.getLon(0)) : "(empty)";
            String lastPt = pts.size() > 1
                    ? String.format("(%.8f, %.8f)", pts.getLat(pts.size() - 1), pts.getLon(pts.size() - 1)) : firstPt;

            boolean isDirect = Boolean.FALSE.equals(instr.getExtraInfoJSON().get("tbt_available"));
            System.out.printf("  [%d] sign=%d (%s) interval=[%d,%d] nPts=%d dist=%.1fm name=\"%s\"%s%n",
                    i, instr.getSign(), signName(instr.getSign()),
                    pointsIndex, end, instrLen, instr.getDistance(), instr.getName(),
                    isDirect ? " [DIRECT]" : "");
            System.out.printf("       firstPt=%s lastPt=%s%n", firstPt, lastPt);

            // For direct instructions, show what the expected polyline span should be
            if (isDirect && pts.size() > 0) {
                // The original gap geometry is just 2 points (start+end waypoints).
                // After remapping, it might have absorbed extra polyline points.
                System.out.printf("       direct_nPts=%d (expected ~2 for a direct segment)%n", instrLen);
            }

            // Print key extraInfo
            Map<String, Object> extra = instr.getExtraInfoJSON();
            for (String key : List.of("tbt_available", "segment_type", "tbt_resumed",
                    "prev_segment_type", "next_segment_type", "predicted_highway",
                    "road_class", "turn_angle_deg")) {
                if (extra.containsKey(key))
                    System.out.println("       " + key + "=" + extra.get(key));
            }

            pointsIndex = end;
        }

        // Coverage and distance checks
        System.out.printf("\n--- Coverage check: last interval end=%d, polyline size=%d%n",
                pointsIndex, poly.size());
        if (pointsIndex != poly.size()) {
            System.out.println("  *** INTERVAL MISMATCH: instructions don't cover entire polyline!");
            System.out.printf("  *** Excess: %d points beyond polyline end%n", pointsIndex - poly.size());
        }

        double totalInstrDist = 0;
        for (Instruction instr : result.instructions) totalInstrDist += instr.getDistance();
        double polylineDist = 0;
        for (int pi = 0; pi < poly.size() - 1; pi++) {
            polylineDist += DistanceCalcEarth.DIST_EARTH.calcDist(
                    poly.getLat(pi), poly.getLon(pi), poly.getLat(pi + 1), poly.getLon(pi + 1));
        }
        System.out.printf("--- Distance check: instructions=%.1fm, polyline=%.1fm, diff=%.1fm%n",
                totalInstrDist, polylineDist, Math.abs(totalInstrDist - polylineDist));

        // --- Route segment 11 (KAnSM→694nh) standalone to get pre-remap coordinates ---
        System.out.println("\n--- Standalone route: segment 11 (KAnSM -> 694nh) ---");
        {
            TrailmapInstructionRequest standaloneReq = new TrailmapInstructionRequest();
            standaloneReq.setWaypoints(List.of(
                    makeWaypoint("wp_s", 61.51704515272817, 23.59427969692007),
                    makeWaypoint("wp_e", 61.516024, 23.596282)
            ));
            TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
            seg.setStart("wp_s");
            seg.setEnd("wp_e");
            seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
            seg.setProfile("gravel");
            seg.setCustomModel(cm);
            standaloneReq.setSegments(List.of(seg));
            standaloneReq.setInstructionProfile("gravel");
            standaloneReq.setLocale("fi");
            standaloneReq.setSnapPreventions(List.of("ferry"));

            RouteInstructionGenerator.Result standaloneResult = generator.generate(standaloneReq);
            System.out.println("Standalone polyline: " + standaloneResult.polyline.size() + " points");
            for (int i = 0; i < standaloneResult.polyline.size(); i++) {
                System.out.printf("  s_poly[%d] = (%.8f, %.8f)%n",
                        i, standaloneResult.polyline.getLat(i), standaloneResult.polyline.getLon(i));
            }
            System.out.println("Standalone instructions:");
            int spi = 0;
            for (int i = 0; i < standaloneResult.instructions.size(); i++) {
                Instruction si = standaloneResult.instructions.get(i);
                int se = spi + si.getLength();
                System.out.printf("  [%d] sign=%d (%s) interval=[%d,%d] nPts=%d dist=%.1fm name=\"%s\"%n",
                        i, si.getSign(), signName(si.getSign()), spi, se, si.getLength(),
                        si.getDistance(), si.getName());
                PointList sp = si.getPoints();
                if (sp.size() > 0) {
                    System.out.printf("       firstPt=(%.8f, %.8f)%n", sp.getLat(0), sp.getLon(0));
                    // Try to find this coordinate in the full route polyline
                    double tLat = sp.getLat(0), tLon = sp.getLon(0);
                    int bestPi = -1;
                    double bestD = Double.MAX_VALUE;
                    for (int pi = 0; pi < poly.size(); pi++) {
                        double d = Math.abs(poly.getLat(pi) - tLat) + Math.abs(poly.getLon(pi) - tLon);
                        if (d < bestD) { bestD = d; bestPi = pi; }
                        if (bestD < 1e-6) break;
                    }
                    System.out.printf("       -> nearest in full poly: poly[%d] dist=%.2e%s%n",
                            bestPi, bestD, bestD < 1e-6 ? " EXACT" : " NO_EXACT_MATCH");
                }
                spi = se;
            }
        }

        // --- Route segments 12+13 standalone ---
        System.out.println("\n--- Standalone route: segment 12 (694nh -> KEQz, heading=160.8) ---");
        {
            TrailmapInstructionRequest req12 = new TrailmapInstructionRequest();
            req12.setWaypoints(List.of(
                    makeWaypoint("wp_s", 61.516024, 23.596282),
                    makeWaypoint("wp_e", 61.515548, 23.594369)
            ));
            TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
            seg.setStart("wp_s");
            seg.setEnd("wp_e");
            seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
            seg.setProfile("gravel");
            seg.setCustomModel(cm);
            seg.setInitialHeading(160.75524855722847);
            seg.setHeadingPenalty(60.0);
            req12.setSegments(List.of(seg));
            req12.setInstructionProfile("gravel");
            req12.setLocale("fi");
            req12.setSnapPreventions(List.of("ferry"));

            RouteInstructionGenerator.Result r12 = generator.generate(req12);
            System.out.println("Seg12 polyline: " + r12.polyline.size() + " points");
            for (int i = 0; i < r12.polyline.size(); i++) {
                System.out.printf("  s12_poly[%d] = (%.8f, %.8f)%n",
                        i, r12.polyline.getLat(i), r12.polyline.getLon(i));
            }
            System.out.println("Seg12 instructions:");
            int spi = 0;
            for (int i = 0; i < r12.instructions.size(); i++) {
                Instruction si = r12.instructions.get(i);
                int se = spi + si.getLength();
                System.out.printf("  [%d] sign=%d (%s) interval=[%d,%d] nPts=%d dist=%.1fm name=\"%s\"%n",
                        i, si.getSign(), signName(si.getSign()), spi, se, si.getLength(),
                        si.getDistance(), si.getName());
                PointList sp = si.getPoints();
                if (sp.size() > 0) {
                    System.out.printf("       firstPt=(%.8f, %.8f)%n", sp.getLat(0), sp.getLon(0));
                    double tLat = sp.getLat(0), tLon = sp.getLon(0);
                    int bestPi = -1;
                    double bestD = Double.MAX_VALUE;
                    for (int pi = 0; pi < poly.size(); pi++) {
                        double d = Math.abs(poly.getLat(pi) - tLat) + Math.abs(poly.getLon(pi) - tLon);
                        if (d < bestD) { bestD = d; bestPi = pi; }
                        if (bestD < 1e-6) break;
                    }
                    System.out.printf("       -> nearest in full poly: poly[%d] dist=%.2e%s%n",
                            bestPi, bestD, bestD < 1e-6 ? " EXACT" : " NO_EXACT_MATCH");
                }
                spi = se;
            }
        }

        System.out.println("\n--- Standalone route: segment 13 (KEQz -> afUA, heading=242.2) ---");
        {
            TrailmapInstructionRequest req13 = new TrailmapInstructionRequest();
            req13.setWaypoints(List.of(
                    makeWaypoint("wp_s", 61.515548, 23.594369),
                    makeWaypoint("wp_e", 61.516169, 23.592415)
            ));
            TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
            seg.setStart("wp_s");
            seg.setEnd("wp_e");
            seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
            seg.setProfile("gravel");
            seg.setCustomModel(cm);
            seg.setInitialHeading(242.16485867817147);
            seg.setHeadingPenalty(60.0);
            req13.setSegments(List.of(seg));
            req13.setInstructionProfile("gravel");
            req13.setLocale("fi");
            req13.setSnapPreventions(List.of("ferry"));

            RouteInstructionGenerator.Result r13 = generator.generate(req13);
            System.out.println("Seg13 polyline: " + r13.polyline.size() + " points");
            for (int i = 0; i < r13.polyline.size(); i++) {
                System.out.printf("  s13_poly[%d] = (%.8f, %.8f)%n",
                        i, r13.polyline.getLat(i), r13.polyline.getLon(i));
            }
            System.out.println("Seg13 instructions:");
            int spi = 0;
            for (int i = 0; i < r13.instructions.size(); i++) {
                Instruction si = r13.instructions.get(i);
                int se = spi + si.getLength();
                System.out.printf("  [%d] sign=%d (%s) interval=[%d,%d] nPts=%d dist=%.1fm name=\"%s\"%n",
                        i, si.getSign(), signName(si.getSign()), spi, se, si.getLength(),
                        si.getDistance(), si.getName());
                PointList sp = si.getPoints();
                if (sp.size() > 0) {
                    System.out.printf("       firstPt=(%.8f, %.8f)%n", sp.getLat(0), sp.getLon(0));
                    double tLat = sp.getLat(0), tLon = sp.getLon(0);
                    int bestPi = -1;
                    double bestD = Double.MAX_VALUE;
                    for (int pi = 0; pi < poly.size(); pi++) {
                        double d = Math.abs(poly.getLat(pi) - tLat) + Math.abs(poly.getLon(pi) - tLon);
                        if (d < bestD) { bestD = d; bestPi = pi; }
                        if (bestD < 1e-6) break;
                    }
                    System.out.printf("       -> nearest in full poly: poly[%d] dist=%.2e%s%n",
                            bestPi, bestD, bestD < 1e-6 ? " EXACT" : " NO_EXACT_MATCH");
                }
                spi = se;
            }
        }

        // --- Manual synthetic path construction to see PRE-remap instruction geometry ---
        System.out.println("\n--- Manual synthetic path: seg11 (KAnSM -> 694nh) ---");
        {
            // Route segment 11 via GH
            GHRequest ghReq = new GHRequest(61.51704515272817, 23.59427969692007, 61.516024, 23.596282);
            ghReq.setProfile("gravel");
            ghReq.setCustomModel(cm);
            ghReq.setPathDetails(List.of("edge_id"));
            ghReq.putHint("instructions", false);
            ghReq.putHint("calc_points", true);
            ghReq.setSnapPreventions(List.of("ferry"));
            GHResponse ghRsp = hopper.route(ghReq);
            assertFalse(ghRsp.hasErrors(), "Seg11 routing failed: " + ghRsp.getErrors());
            ResponsePath rp = ghRsp.getBest();

            // Extract edge IDs
            List<PathDetail> edgeDetails = rp.getPathDetails().get("edge_id");
            List<Integer> edgeIds = new ArrayList<>();
            for (PathDetail d : edgeDetails) {
                int eid = ((Number) d.getValue()).intValue();
                if (edgeIds.isEmpty() || edgeIds.get(edgeIds.size() - 1) != eid) {
                    edgeIds.add(eid);
                }
            }
            System.out.println("  Edge IDs: " + edgeIds);

            // Build synthetic path (replicate buildSyntheticPath logic)
            // Resolve fromNode from edge connectivity
            EdgeIteratorState firstEdge = baseGraph.getEdgeIteratorState(edgeIds.get(0), Integer.MIN_VALUE);
            int nodeA = firstEdge.getBaseNode();
            int nodeB = firstEdge.getAdjNode();
            int fromNode;
            if (edgeIds.size() >= 2) {
                EdgeIteratorState secondEdge = baseGraph.getEdgeIteratorState(edgeIds.get(1), Integer.MIN_VALUE);
                boolean aConn = secondEdge.getBaseNode() == nodeA || secondEdge.getAdjNode() == nodeA;
                boolean bConn = secondEdge.getBaseNode() == nodeB || secondEdge.getAdjNode() == nodeB;
                fromNode = (aConn && !bConn) ? nodeB : nodeA;
            } else {
                // Single edge: proximity
                double distA = DistanceCalcEarth.DIST_EARTH.calcDist(61.51704515272817, 23.59427969692007,
                        baseGraph.getNodeAccess().getLat(nodeA), baseGraph.getNodeAccess().getLon(nodeA));
                double distB = DistanceCalcEarth.DIST_EARTH.calcDist(61.51704515272817, 23.59427969692007,
                        baseGraph.getNodeAccess().getLat(nodeB), baseGraph.getNodeAccess().getLon(nodeB));
                fromNode = distA <= distB ? nodeA : nodeB;
            }
            System.out.printf("  fromNode=%d at (%.8f, %.8f)%n", fromNode,
                    baseGraph.getNodeAccess().getLat(fromNode), baseGraph.getNodeAccess().getLon(fromNode));

            // Build path
            com.graphhopper.routing.Path synthPath = new com.graphhopper.routing.Path(baseGraph);
            for (int eid : edgeIds) synthPath.addEdge(eid);
            synthPath.setFromNode(fromNode);
            synthPath.setFound(true);
            int currNode = fromNode;
            for (int eid : edgeIds) {
                EdgeIteratorState e = baseGraph.getEdgeIteratorState(eid, Integer.MIN_VALUE);
                currNode = (e.getBaseNode() == currNode) ? e.getAdjNode() : e.getBaseNode();
            }
            synthPath.setEndNode(currNode);
            System.out.printf("  endNode=%d at (%.8f, %.8f)%n", currNode,
                    baseGraph.getNodeAccess().getLat(currNode), baseGraph.getNodeAccess().getLon(currNode));

            // Generate instructions from synthetic path
            Profile instrProfile = hopper.getProfile("gravel");
            Weighting weighting = ((TrailmapGraphHopper) hopper).createWeighting(instrProfile, new com.graphhopper.util.PMap());
            Translation tr = hopper.getTranslationMap().getWithFallBack(Locale.forLanguageTag("fi"));

            InstructionList rawInstructions = TrailmapInstructionsFromEdges.calcInstructions(
                    synthPath, baseGraph, weighting, hopper.getEncodingManager(), tr);

            System.out.println("  Raw instructions (PRE-remap):");
            for (int i = 0; i < rawInstructions.size(); i++) {
                Instruction ri = rawInstructions.get(i);
                PointList rp2 = ri.getPoints();
                String fp = rp2.size() > 0 ? String.format("(%.8f, %.8f)", rp2.getLat(0), rp2.getLon(0)) : "(empty)";
                System.out.printf("    [%d] sign=%d (%s) nPts=%d dist=%.1fm firstPt=%s name=\"%s\"%n",
                        i, ri.getSign(), signName(ri.getSign()), rp2.size(), ri.getDistance(), fp, ri.getName());

                // Match against full polyline
                if (rp2.size() > 0) {
                    double tLat = rp2.getLat(0), tLon = rp2.getLon(0);
                    int bestPi = -1;
                    double bestD = Double.MAX_VALUE;
                    for (int pi = 0; pi < poly.size(); pi++) {
                        double d = Math.abs(poly.getLat(pi) - tLat) + Math.abs(poly.getLon(pi) - tLon);
                        if (d < bestD) { bestD = d; bestPi = pi; }
                        if (bestD < 1e-6) break;
                    }
                    System.out.printf("         -> full poly match: poly[%d] dist=%.2e%s%n",
                            bestPi, bestD, bestD < 1e-6 ? " EXACT" : " NO_EXACT");
                    // Also search from index 22 (what remapInstructionGeometry would do)
                    int bestPi22 = 22;
                    double bestD22 = Double.MAX_VALUE;
                    for (int pi = 22; pi < poly.size(); pi++) {
                        double d = Math.abs(poly.getLat(pi) - tLat) + Math.abs(poly.getLon(pi) - tLon);
                        if (d < bestD22) { bestD22 = d; bestPi22 = pi; }
                        if (bestD22 < 1e-6) break;
                    }
                    System.out.printf("         -> from idx 22: poly[%d] dist=%.2e%s%n",
                            bestPi22, bestD22, bestD22 < 1e-6 ? " EXACT" : " NO_EXACT");
                }
            }
        }

        // Apply post-processing
        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions);

        System.out.println("\n--- Instructions AFTER post-processing ---");
        printInstructionsDetailed(result);
    }

    // ========================================================================
    // Debug: false M1 join-side-path on right-left-right sequence
    // ========================================================================

    /**
     * Debug test: a short foot route with 3 ~90° turns (right, left, right)
     * incorrectly collapsed into a single M1 join-side-path merge.
     */
    @Test
    void testFalseM1OnRightLeftRight() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("C7gQmZ9YjyHtB0C90qQ6S", 61.459012, 23.828659),
                makeWaypoint("IyrZ1KcB4Q_MbrpvO86d5", 61.458676, 23.829354)
        ));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("C7gQmZ9YjyHtB0C90qQ6S");
        seg.setEnd("IyrZ1KcB4Q_MbrpvO86d5");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("trailmap_foot");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("trailmap_foot");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);

        System.out.println("\n========== FALSE M1 ON RIGHT-LEFT-RIGHT — BEFORE POST-PROCESSING ==========");
        System.out.println("Polyline: " + result.polyline.size() + " points");
        printInstructionsDetailed(result);

        // Detailed M1 analysis: check each consecutive pair
        System.out.println("\n--- M1 pair analysis ---");
        for (int i = 0; i < result.instructions.size() - 1; i++) {
            Instruction first = result.instructions.get(i);
            Instruction second = result.instructions.get(i + 1);
            if (first.getSign() == Instruction.FINISH || second.getSign() == Instruction.FINISH) continue;

            int s1 = first.getSign();
            int s2 = second.getSign();
            boolean oppDir = (s1 < 0 && s2 > 0) || (s1 > 0 && s2 < 0);
            double dist = first.getDistance();

            Map<String, Object> e1 = first.getExtraInfoJSON();
            Map<String, Object> e2 = second.getExtraInfoJSON();
            String prevPH = (String) e1.get("prev_predicted_highway");
            String firstPH = (String) e1.get("predicted_highway");
            String secondPH = (String) e2.get("predicted_highway");
            Boolean srcCont = (Boolean) e1.get("source_road_continues");

            Object a1Obj = e1.get("turn_angle_deg");
            Object a2Obj = e2.get("turn_angle_deg");
            double a1 = a1Obj instanceof Number ? ((Number) a1Obj).doubleValue() : 0;
            double a2 = a2Obj instanceof Number ? ((Number) a2Obj).doubleValue() : 0;
            double net = Math.abs(a1 + a2);
            boolean geomOk = Math.abs(a1) >= 60 && Math.abs(a1) <= 120
                    && Math.abs(a2) >= 60 && Math.abs(a2) <= 120 && net <= 45;

            boolean phChange = prevPH != null && secondPH != null && !prevPH.equals(secondPH);

            System.out.printf("  pair [%d]+[%d]: s1=%d s2=%d dist=%.1fm oppDir=%s%n",
                    i, i + 1, s1, s2, dist, oppDir);
            System.out.printf("    prevPH=%s firstPH=%s secondPH=%s phChange=%s srcCont=%s%n",
                    prevPH, firstPH, secondPH, phChange, srcCont);
            System.out.printf("    a1=%.1f a2=%.1f net=%.1f geomOk=%s%n", a1, a2, net, geomOk);
            System.out.printf("    M1 candidate: dist<=15=%s oppDir=%s phChange=%s (geomOk||srcCont)=%s%n",
                    dist <= 15, oppDir, phChange, geomOk || Boolean.TRUE.equals(srcCont));
        }

        // Apply post-processing
        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions);

        System.out.println("\n========== FALSE M1 ON RIGHT-LEFT-RIGHT — AFTER POST-PROCESSING ==========");
        printInstructionsDetailed(result);
    }

    // ========================================================================
    // Debug: missing instruction at cycleway→path fork
    // ========================================================================

    /**
     * Debug test: riding on cycleway, fork where cycleway continues slight-right
     * and path goes slight-left. Route takes the path but no turn instruction
     * is generated at the fork.
     */
    @Test
    void testMissingInstructionAtCyclewayPathFork() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("eVPPy2b9VTcaqkIJcplky", 61.442844, 23.83458),
                makeWaypoint("ODF9jaKgYZr39X79PB4Mx", 61.442834, 23.832708)
        ));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("eVPPy2b9VTcaqkIJcplky");
        seg.setEnd("ODF9jaKgYZr39X79PB4Mx");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("trailmap_foot");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("trailmap_foot");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);

        System.out.println("\n========== MISSING INSTRUCTION AT CYCLEWAY→PATH FORK — BEFORE POST-PROCESSING ==========");
        System.out.println("Polyline: " + result.polyline.size() + " points");
        for (int i = 0; i < result.polyline.size(); i++) {
            System.out.printf("  poly[%d] = (%.8f, %.8f)%n",
                    i, result.polyline.getLat(i), result.polyline.getLon(i));
        }
        printInstructionsDetailed(result);

        // Now route with standard GH instructions for comparison
        System.out.println("\n--- Standard GH routing for comparison ---");
        GHRequest ghReq = new GHRequest(61.442844, 23.83458, 61.442834, 23.832708);
        ghReq.setProfile("trailmap_foot");
        ghReq.setSnapPreventions(List.of("ferry"));
        GHResponse ghRsp = hopper.route(ghReq);
        if (!ghRsp.hasErrors()) {
            ResponsePath rp = ghRsp.getBest();
            System.out.println("GH instructions:");
            for (Instruction instr : rp.getInstructions()) {
                System.out.printf("  sign=%d (%s) dist=%.1fm name=\"%s\" text=\"%s\"%n",
                        instr.getSign(), signName(instr.getSign()),
                        instr.getDistance(), instr.getName(),
                        instr.getTurnDescription(rp.getInstructions().getTr()));
            }
        }

        // Route the same via the synthetic path to see raw edge-level decisions
        System.out.println("\n--- Synthetic path raw instructions ---");
        {
            GHRequest routeReq = new GHRequest(61.442844, 23.83458, 61.442834, 23.832708);
            routeReq.setProfile("trailmap_foot");
            routeReq.setPathDetails(List.of("edge_id"));
            routeReq.putHint("instructions", false);
            routeReq.putHint("calc_points", true);
            routeReq.setSnapPreventions(List.of("ferry"));
            GHResponse routeRsp = hopper.route(routeReq);
            if (!routeRsp.hasErrors()) {
                ResponsePath rp = routeRsp.getBest();
                List<PathDetail> edgeDetails = rp.getPathDetails().get("edge_id");
                List<Integer> edgeIds = new ArrayList<>();
                for (PathDetail d : edgeDetails) {
                    int eid = ((Number) d.getValue()).intValue();
                    if (edgeIds.isEmpty() || edgeIds.get(edgeIds.size() - 1) != eid) {
                        edgeIds.add(eid);
                    }
                }
                System.out.println("Edge IDs: " + edgeIds);
                System.out.println("Edge count: " + edgeIds.size());

                // Print edge details: road class, predicted highway, nodes
                EnumEncodedValue<RoadClass> rcEnc = encodingManager.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
                EnumEncodedValue<PredictedHighway> phEnc = encodingManager.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
                for (int eid : edgeIds) {
                    EdgeIteratorState edge = baseGraph.getEdgeIteratorState(eid, Integer.MIN_VALUE);
                    RoadClass rc = edge.get(rcEnc);
                    PredictedHighway ph = edge.get(phEnc);
                    System.out.printf("  edge %d: base=%d adj=%d rc=%s ph=%s name=\"%s\"%n",
                            eid, edge.getBaseNode(), edge.getAdjNode(), rc, ph, edge.getName());
                }

                // Build synthetic path and get raw instructions
                EdgeIteratorState firstEdge = baseGraph.getEdgeIteratorState(edgeIds.get(0), Integer.MIN_VALUE);
                int nodeA = firstEdge.getBaseNode();
                int nodeB = firstEdge.getAdjNode();
                int fromNode;
                if (edgeIds.size() >= 2) {
                    EdgeIteratorState secondEdge = baseGraph.getEdgeIteratorState(edgeIds.get(1), Integer.MIN_VALUE);
                    boolean aConn = secondEdge.getBaseNode() == nodeA || secondEdge.getAdjNode() == nodeA;
                    boolean bConn = secondEdge.getBaseNode() == nodeB || secondEdge.getAdjNode() == nodeB;
                    fromNode = (aConn && !bConn) ? nodeB : nodeA;
                } else {
                    double distA = DistanceCalcEarth.DIST_EARTH.calcDist(61.442844, 23.83458,
                            baseGraph.getNodeAccess().getLat(nodeA), baseGraph.getNodeAccess().getLon(nodeA));
                    double distB = DistanceCalcEarth.DIST_EARTH.calcDist(61.442844, 23.83458,
                            baseGraph.getNodeAccess().getLat(nodeB), baseGraph.getNodeAccess().getLon(nodeB));
                    fromNode = distA <= distB ? nodeA : nodeB;
                }

                com.graphhopper.routing.Path synthPath = new com.graphhopper.routing.Path(baseGraph);
                for (int eid : edgeIds) synthPath.addEdge(eid);
                synthPath.setFromNode(fromNode);
                synthPath.setFound(true);
                int currNode = fromNode;
                for (int eid : edgeIds) {
                    EdgeIteratorState e = baseGraph.getEdgeIteratorState(eid, Integer.MIN_VALUE);
                    currNode = (e.getBaseNode() == currNode) ? e.getAdjNode() : e.getBaseNode();
                }
                synthPath.setEndNode(currNode);

                Profile instrProfile = hopper.getProfile("trailmap_foot");
                Weighting weighting = ((TrailmapGraphHopper) hopper).createWeighting(instrProfile, new com.graphhopper.util.PMap());
                Translation tr = hopper.getTranslationMap().getWithFallBack(Locale.forLanguageTag("fi"));

                InstructionList rawInstructions = TrailmapInstructionsFromEdges.calcInstructions(
                        synthPath, baseGraph, weighting, encodingManager, tr);

                // Compute junction angles at each edge transition
                System.out.println("\nJunction angle analysis:");
                int walkNode = fromNode;
                for (int ei = 0; ei < edgeIds.size() - 1; ei++) {
                    int prevEid = edgeIds.get(ei);
                    int nextEid = edgeIds.get(ei + 1);
                    EdgeIteratorState prevE = baseGraph.getEdgeIteratorState(prevEid, Integer.MIN_VALUE);
                    EdgeIteratorState nextE = baseGraph.getEdgeIteratorState(nextEid, Integer.MIN_VALUE);
                    int junctionNode = (prevE.getBaseNode() == walkNode) ? prevE.getAdjNode() : prevE.getBaseNode();

                    // Compute approach orientation from previous edge's last two points
                    PointList prevPts = prevE.fetchWayGeometry(com.graphhopper.util.FetchMode.ALL);
                    // Walk direction: find the last two points in traversal direction
                    int prevFromNode = walkNode;
                    boolean prevForward = prevE.getBaseNode() == prevFromNode;
                    double doublePrevLat, doublePrevLon, juncLat, juncLon;
                    if (prevForward) {
                        doublePrevLat = prevPts.getLat(prevPts.size() - 2);
                        doublePrevLon = prevPts.getLon(prevPts.size() - 2);
                        juncLat = prevPts.getLat(prevPts.size() - 1);
                        juncLon = prevPts.getLon(prevPts.size() - 1);
                    } else {
                        doublePrevLat = prevPts.getLat(1);
                        doublePrevLon = prevPts.getLon(1);
                        juncLat = prevPts.getLat(0);
                        juncLon = prevPts.getLon(0);
                    }
                    double prevOrientation = AngleCalc.ANGLE_CALC.calcOrientation(doublePrevLat, doublePrevLon, juncLat, juncLon);

                    // Route edge angle
                    GHPoint routePt = InstructionsHelper.getPointForOrientationCalculation(nextE, baseGraph.getNodeAccess());
                    double routeDelta = InstructionsHelper.calculateOrientationDelta(juncLat, juncLon, routePt.getLat(), routePt.getLon(), prevOrientation);
                    int routeSign = InstructionsHelper.calculateSign(juncLat, juncLon, routePt.getLat(), routePt.getLon(), prevOrientation);

                    PredictedHighway prevPH2 = prevE.get(phEnc);
                    PredictedHighway nextPH2 = nextE.get(phEnc);

                    System.out.printf("  edge %d→%d at node %d: routeDelta=%.4f (%.1f°) sign=%d prevPH=%s nextPH=%s%n",
                            prevEid, nextEid, junctionNode, routeDelta, Math.toDegrees(routeDelta), routeSign, prevPH2, nextPH2);

                    // Check all alternatives at this junction
                    EdgeExplorer explorer = baseGraph.createEdgeExplorer();
                    EdgeIterator iter = explorer.setBaseNode(junctionNode);
                    while (iter.next()) {
                        if (iter.getEdge() == prevEid || iter.getEdge() == nextEid) continue;
                        GHPoint altPt = InstructionsHelper.getPointForOrientationCalculation(iter, baseGraph.getNodeAccess());
                        double altDelta = InstructionsHelper.calculateOrientationDelta(juncLat, juncLon, altPt.getLat(), altPt.getLon(), prevOrientation);
                        PredictedHighway altPH = iter.get(phEnc);
                        System.out.printf("    alt edge %d: delta=%.4f (%.1f°) ph=%s name=\"%s\"%n",
                                iter.getEdge(), altDelta, Math.toDegrees(altDelta), altPH, iter.getName());
                    }

                    walkNode = junctionNode;
                }

                System.out.println("\nRaw Trailmap instructions (before any post-processing):");
                for (int i = 0; i < rawInstructions.size(); i++) {
                    Instruction ri = rawInstructions.get(i);
                    System.out.printf("  [%d] sign=%d (%s) nPts=%d dist=%.1fm name=\"%s\"%n",
                            i, ri.getSign(), signName(ri.getSign()), ri.getPoints().size(),
                            ri.getDistance(), ri.getName());
                    Map<String, Object> extra = ri.getExtraInfoJSON();
                    for (String key : List.of("predicted_highway", "prev_predicted_highway",
                            "road_class", "prev_road_class", "turn_angle_deg",
                            "junction_alternatives", "junction_alt_predicted_highways",
                            "source_road_continues", "trail_fork", "has_name")) {
                        if (extra.containsKey(key))
                            System.out.println("       " + key + "=" + extra.get(key));
                    }
                }
            }
        }

        // Apply post-processing
        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions);

        System.out.println("\n========== MISSING INSTRUCTION AT CYCLEWAY→PATH FORK — AFTER POST-PROCESSING ==========");
        printInstructionsDetailed(result);
    }

    /**
     * Diagnostic: visual guidance candidate — unpaved cycleway turns left,
     * unpaved footway continues straight ahead.
     * Investigating whether current rules suppress the turn instruction
     * at this junction (fork with similar surface, route requires active turn).
     *
     * Extended route: start further south to approach the junction, end further
     * north-west to continue on the cycleway past the fork.
     */
    @Test
    void testVisualGuidanceCyclewayFootwayFork() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        // Original API payload coordinates — one junction within this 71m segment
        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("EPSEb7ADPf6YRDf-YeJeE", 61.482473, 23.749178);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("2S2sUFn4IyfNqsjlXAuAi", 61.482827, 23.748475);
        request.setWaypoints(List.of(wp1, wp2));

        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_highway == CYCLEWAY",
                Statement.Op.MULTIPLY, "1.3"));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("EPSEb7ADPf6YRDf-YeJeE");
        seg.setEnd("2S2sUFn4IyfNqsjlXAuAi");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");
        seg.setCustomModel(cm);

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);

        System.out.println("\n========== VISUAL GUIDANCE: CYCLEWAY↰ + FOOTWAY↑ FORK — BEFORE POST-PROCESSING ==========");
        printInstructionsDetailed(result);

        // Print route geometry points to understand the path
        System.out.println("\n--- Route geometry points ---");
        PointList points = result.instructions.get(0).getPoints();
        // Collect all points from all instructions
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            PointList pts = instr.getPoints();
            for (int j = 0; j < pts.size(); j++) {
                System.out.printf("  instr[%d] pt[%d]: %.6f, %.6f%n", i, j, pts.getLat(j), pts.getLon(j));
            }
        }

        // Standard GH routing with edge-level details to find junctions
        System.out.println("\n--- Standard GH routing with edge details ---");
        GHRequest ghReq = new GHRequest(61.482473, 23.749178, 61.482827, 23.748475).setProfile("gravel");
        CustomModel ghCm = new CustomModel();
        ghCm.addToPriority(Statement.If("predicted_highway == CYCLEWAY",
                Statement.Op.MULTIPLY, "1.3"));
        ghReq.setCustomModel(ghCm);
        ghReq.setSnapPreventions(List.of("ferry"));
        ghReq.setPathDetails(List.of("edge_id", "road_class", "predicted_highway"));
        GHResponse ghRsp = hopper.route(ghReq);
        if (!ghRsp.hasErrors()) {
            ResponsePath path = ghRsp.getBest();
            System.out.println("Distance: " + Math.round(path.getDistance()) + "m");
            System.out.println("Edge IDs: " + path.getPathDetails().get("edge_id"));
            System.out.println("Road classes: " + path.getPathDetails().get("road_class"));
            System.out.println("Predicted highways: " + path.getPathDetails().get("predicted_highway"));

            // GH standard instructions for comparison
            InstructionList ghInstr = path.getInstructions();
            System.out.println("GH instructions (" + ghInstr.size() + "):");
            for (int i = 0; i < ghInstr.size(); i++) {
                Instruction instr = ghInstr.get(i);
                System.out.printf("  [%d] sign=%d dist=%.1fm name=\"%s\"%n",
                        i, instr.getSign(), instr.getDistance(), instr.getName());
            }
        } else {
            System.out.println("GH routing error: " + ghRsp.getErrors());
        }

        // Inspect the junction topology at each graph node along the route
        // with angles to understand which suppression rule fires
        System.out.println("\n--- Junction analysis along route (with angles) ---");
        BaseGraph bg = hopper.getBaseGraph();
        EncodingManager em = hopper.getEncodingManager();
        EnumEncodedValue<PredictedHighway> phEnc = em.getEnumEncodedValue("predicted_highway", PredictedHighway.class);
        EnumEncodedValue<RoadClass> rcEnc = em.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EdgeExplorer explorer = bg.createEdgeExplorer();

        if (!ghRsp.hasErrors()) {
            List<PathDetail> edgeDetails = ghRsp.getBest().getPathDetails().get("edge_id");
            PointList routePoints = ghRsp.getBest().getPoints();

            // Walk through consecutive route edges to compute turn angles at junctions
            for (int d = 0; d < edgeDetails.size() - 1; d++) {
                int edgeId = (int) edgeDetails.get(d).getValue();
                int nextEdgeId = (int) edgeDetails.get(d + 1).getValue();

                // Get the shared junction node between consecutive edges
                EdgeIteratorState eState = bg.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
                EdgeIteratorState nState = bg.getEdgeIteratorState(nextEdgeId, Integer.MIN_VALUE);

                // Find the shared node
                int junctionNode = -1;
                if (eState.getAdjNode() == nState.getBaseNode() || eState.getAdjNode() == nState.getAdjNode())
                    junctionNode = eState.getAdjNode();
                else if (eState.getBaseNode() == nState.getBaseNode() || eState.getBaseNode() == nState.getAdjNode())
                    junctionNode = eState.getBaseNode();

                if (junctionNode < 0) continue;

                // Compute approach orientation from previous geometry points
                int fromPt = edgeDetails.get(d).getLast() - 1;
                int toPt = edgeDetails.get(d).getLast();
                int nextPt = edgeDetails.get(d + 1).getFirst();
                int nextPt2 = Math.min(edgeDetails.get(d + 1).getFirst() + 1, routePoints.size() - 1);

                if (fromPt < 0) fromPt = 0;

                double prevLat2 = routePoints.getLat(fromPt);
                double prevLon2 = routePoints.getLon(fromPt);
                double jLat = routePoints.getLat(toPt);
                double jLon = routePoints.getLon(toPt);
                double nextLat2 = routePoints.getLat(nextPt2);
                double nextLon2 = routePoints.getLon(nextPt2);

                double orientation = AngleCalc.ANGLE_CALC.calcOrientation(prevLat2, prevLon2, jLat, jLon);
                int sign2 = InstructionsHelper.calculateSign(jLat, jLon, nextLat2, nextLon2, orientation);
                double delta2 = InstructionsHelper.calculateOrientationDelta(jLat, jLon, nextLat2, nextLon2, orientation);

                double nodeLat = bg.getNodeAccess().getLat(junctionNode);
                double nodeLon = bg.getNodeAccess().getLon(junctionNode);

                // Count edges at this node
                EdgeIterator iter = explorer.setBaseNode(junctionNode);
                List<String> altInfo = new ArrayList<>();
                while (iter.next()) {
                    PredictedHighway ph = iter.get(phEnc);
                    RoadClass rc = iter.get(rcEnc);
                    altInfo.add(String.format("edge=%d ph=%s rc=%s name=\"%s\"",
                            iter.getEdge(), ph, rc, iter.getName()));
                }

                if (altInfo.size() > 2) {
                    System.out.printf("  Junction node=%d (%.6f, %.6f) route sign=%d delta=%.3f rad (%.1f°) edges=%d:%n",
                            junctionNode, nodeLat, nodeLon, sign2, delta2, Math.toDegrees(delta2), altInfo.size());

                    // Show angle for EVERY edge at this junction relative to approach direction
                    EdgeIterator iter2 = explorer.setBaseNode(junctionNode);
                    while (iter2.next()) {
                        GHPoint altPt = InstructionsHelper.getPointForOrientationCalculation(iter2, bg.getNodeAccess());
                        double altDelta = InstructionsHelper.calculateOrientationDelta(
                                jLat, jLon, altPt.getLat(), altPt.getLon(), orientation);
                        int altSign = InstructionsHelper.calculateSign(jLat, jLon, altPt.getLat(), altPt.getLon(), orientation);
                        PredictedHighway altPH = iter2.get(phEnc);
                        RoadClass altRC = iter2.get(rcEnc);
                        boolean isIncoming = iter2.getEdge() == edgeId;
                        boolean isRoute = iter2.getEdge() == nextEdgeId;
                        String marker = isIncoming ? " ← INCOMING" : isRoute ? " ← ROUTE" : "";
                        System.out.printf("    edge=%d ph=%s rc=%s name=\"%s\" delta=%.1f° sign=%d%s%n",
                                iter2.getEdge(), altPH, altRC, iter2.getName(),
                                Math.toDegrees(altDelta), altSign, marker);
                    }
                }
            }
        }

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions);

        System.out.println("\n========== VISUAL GUIDANCE: CYCLEWAY↰ + FOOTWAY↑ FORK — AFTER POST-PROCESSING ==========");
        printInstructionsDetailed(result);
    }

    // ========================================================================
    // Route alternative selection debugging
    // ========================================================================

    /**
     * Diagnostic: /route returns main + alternative with weights 308 vs 309 (very close).
     * /instructions appears to generate instructions for the alternative route, not the main one.
     * Waypoints are well away from the deviation point, so snapping is not the cause.
     *
     * API payload:
     *   2 waypoints, 1 segment (gravel), instruction_profile=gravel, locale=fi, snap_preventions=["ferry"]
     *
     * Investigation: compare GH route results (with and without alternatives) against
     * the TbT generator's route to see if they diverge and why.
     */
    /**
     * Diagnostic: /route (with alternative_route algorithm) returns best + alternative with
     * very close weights (~308 vs ~309). /instructions appears to pick the alternative.
     *
     * Key insight: /route uses alternative_route algorithm, but /instructions (via routeSection)
     * uses the default algorithm (Dijkstra/A*). These can produce different "best" paths when
     * weights are very close.
     *
     * Uses exact coordinates from both API calls to reproduce.
     */
    @Test
    void testRouteAlternativeSelection_closeWeights() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        // Coordinates from /route call (note: /route uses [lng, lat] GeoJSON order)
        double routeStartLat = 61.526983, routeStartLng = 23.643721;
        double routeEndLat = 61.52380178892702, routeEndLng = 23.627764795806314;

        // Coordinates from /instructions call
        double instrStartLat = 61.526983, instrStartLng = 23.643721;
        double instrEndLat = 61.523813, instrEndLng = 23.627779;

        System.out.println("\n========== ROUTE ALTERNATIVE SELECTION — CLOSE WEIGHTS ==========");
        System.out.printf("Endpoint delta: lat=%.10f lng=%.10f%n",
                Math.abs(routeEndLat - instrEndLat), Math.abs(routeEndLng - instrEndLng));

        // --- Step 1: /route with alternative_route algorithm (what the client sees) ---
        GHRequest altReq = new GHRequest(routeStartLat, routeStartLng, routeEndLat, routeEndLng);
        altReq.setProfile("gravel");
        altReq.setSnapPreventions(List.of("ferry"));
        altReq.setPathDetails(List.of("edge_id"));
        altReq.putHint("instructions", false);
        altReq.putHint("calc_points", true);
        altReq.setAlgorithm("alternative_route");
        altReq.putHint("alternative_route.max_paths", 3);
        GHResponse altRsp = hopper.route(altReq);
        assertFalse(altRsp.hasErrors(), "Alternative route should succeed");

        System.out.println("\n--- /route (alternative_route algorithm) with ROUTE coordinates ---");
        List<List<Integer>> altEdgeLists = new ArrayList<>();
        for (int i = 0; i < altRsp.getAll().size(); i++) {
            ResponsePath p = altRsp.getAll().get(i);
            List<PathDetail> eds = p.getPathDetails().get("edge_id");
            List<Integer> eids = new ArrayList<>();
            for (PathDetail d : eds) eids.add((Integer) d.getValue());
            altEdgeLists.add(eids);
            System.out.printf("  Alt %d: weight=%.3f distance=%.1fm edges=%d%n",
                    i, p.getRouteWeight(), p.getDistance(), eids.size());
        }

        // --- Step 2: Default algorithm with ROUTE coordinates ---
        GHRequest defaultRouteCoordReq = new GHRequest(routeStartLat, routeStartLng, routeEndLat, routeEndLng);
        defaultRouteCoordReq.setProfile("gravel");
        defaultRouteCoordReq.setSnapPreventions(List.of("ferry"));
        defaultRouteCoordReq.setPathDetails(List.of("edge_id"));
        defaultRouteCoordReq.putHint("instructions", false);
        defaultRouteCoordReq.putHint("calc_points", true);
        GHResponse defaultRouteCoordRsp = hopper.route(defaultRouteCoordReq);
        assertFalse(defaultRouteCoordRsp.hasErrors());

        ResponsePath defaultRouteCoordPath = defaultRouteCoordRsp.getBest();
        List<Integer> defaultRouteCoordEdges = new ArrayList<>();
        for (PathDetail d : defaultRouteCoordPath.getPathDetails().get("edge_id"))
            defaultRouteCoordEdges.add((Integer) d.getValue());
        System.out.printf("\n--- Default algorithm, ROUTE coordinates: weight=%.3f distance=%.1fm edges=%d%n",
                defaultRouteCoordPath.getRouteWeight(), defaultRouteCoordPath.getDistance(), defaultRouteCoordEdges.size());

        // --- Step 3: Default algorithm with INSTRUCTION coordinates ---
        GHRequest defaultInstrCoordReq = new GHRequest(instrStartLat, instrStartLng, instrEndLat, instrEndLng);
        defaultInstrCoordReq.setProfile("gravel");
        defaultInstrCoordReq.setSnapPreventions(List.of("ferry"));
        defaultInstrCoordReq.setPathDetails(List.of("edge_id"));
        defaultInstrCoordReq.putHint("instructions", false);
        defaultInstrCoordReq.putHint("calc_points", true);
        GHResponse defaultInstrCoordRsp = hopper.route(defaultInstrCoordReq);
        assertFalse(defaultInstrCoordRsp.hasErrors());

        ResponsePath defaultInstrCoordPath = defaultInstrCoordRsp.getBest();
        List<Integer> defaultInstrCoordEdges = new ArrayList<>();
        for (PathDetail d : defaultInstrCoordPath.getPathDetails().get("edge_id"))
            defaultInstrCoordEdges.add((Integer) d.getValue());
        System.out.printf("--- Default algorithm, INSTR coordinates:  weight=%.3f distance=%.1fm edges=%d%n",
                defaultInstrCoordPath.getRouteWeight(), defaultInstrCoordPath.getDistance(), defaultInstrCoordEdges.size());

        // --- Step 4: Compare all three edge sequences ---
        System.out.println("\n--- Edge sequence comparison ---");

        // alt0 vs default-route-coords
        int div1 = findDivergence(altEdgeLists.get(0), defaultRouteCoordEdges);
        if (div1 >= 0)
            System.out.printf("alt0 vs default(route-coords) DIVERGE at idx %d%n", div1);
        else
            System.out.println("alt0 vs default(route-coords): IDENTICAL");

        // alt0 vs default-instr-coords
        int div2 = findDivergence(altEdgeLists.get(0), defaultInstrCoordEdges);
        if (div2 >= 0)
            System.out.printf("alt0 vs default(instr-coords) DIVERGE at idx %d%n", div2);
        else
            System.out.println("alt0 vs default(instr-coords): IDENTICAL");

        // default-route-coords vs default-instr-coords
        int div3 = findDivergence(defaultRouteCoordEdges, defaultInstrCoordEdges);
        if (div3 >= 0)
            System.out.printf("default(route-coords) vs default(instr-coords) DIVERGE at idx %d%n", div3);
        else
            System.out.println("default(route-coords) vs default(instr-coords): IDENTICAL");

        // If alternatives exist, check alt1
        if (altEdgeLists.size() >= 2) {
            int div4 = findDivergence(altEdgeLists.get(1), defaultInstrCoordEdges);
            if (div4 >= 0)
                System.out.printf("alt1 vs default(instr-coords) DIVERGE at idx %d%n", div4);
            else
                System.out.println("alt1 vs default(instr-coords): IDENTICAL — TbT picks the ALTERNATIVE!");

            int div5 = findDivergence(altEdgeLists.get(1), defaultRouteCoordEdges);
            if (div5 >= 0)
                System.out.printf("alt1 vs default(route-coords) DIVERGE at idx %d%n", div5);
            else
                System.out.println("alt1 vs default(route-coords): IDENTICAL");
        }

        // --- Step 5: TbT generator output ---
        TrailmapInstructionRequest instrRequest = new TrailmapInstructionRequest();
        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("wp1", instrStartLat, instrStartLng);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("wp2", instrEndLat, instrEndLng);
        instrRequest.setWaypoints(List.of(wp1, wp2));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("wp1");
        seg.setEnd("wp2");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        instrRequest.setSegments(List.of(seg));
        instrRequest.setInstructionProfile("gravel");
        instrRequest.setLocale("fi");
        instrRequest.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(instrRequest);
        assertNotNull(result);

        System.out.println("\n--- TbT generator instructions (before post-processing) ---");
        printInstructionsDetailed(result);

        // Match TbT polyline against alternatives
        if (altRsp.getAll().size() >= 2) {
            PointList tbtPoly = result.polyline;
            double matchScore0 = polylineMatchScore(tbtPoly, altRsp.getAll().get(0).getPoints());
            double matchScore1 = polylineMatchScore(tbtPoly, altRsp.getAll().get(1).getPoints());
            System.out.printf("\nTbT polyline match vs alt0 (best): %.1fm avg deviation%n", matchScore0);
            System.out.printf("TbT polyline match vs alt1 (2nd):  %.1fm avg deviation%n", matchScore1);
            if (matchScore1 < matchScore0)
                System.out.println(">>> TbT matches ALTERNATIVE better than BEST!");
        }

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions);
        System.out.println("\n--- After post-processing ---");
        printInstructionsDetailed(result);
    }

    /**
     * Diagnostic test for 500 error on a 25-segment gravel route (Hämeenlinna area).
     * Two segments have via_points. Reproducing API payload from 2026-04-13.
     */
    @Test
    void testServer500_gravelRoute25Segments() throws Exception {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();
        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        // All waypoints
        double[][] wps = {
            {60.738336, 24.773349}, // 0
            {60.738375, 24.773214}, // 1
            {60.737164, 24.788244}, // 2
            {60.736912, 24.791316}, // 3
            {60.735785, 24.793158}, // 4
            {60.733048, 24.796468}, // 5
            {60.732525, 24.797316}, // 6
            {60.735424, 24.802489}, // 7
            {60.738594, 24.80937},  // 8
            {60.737856, 24.813473}, // 9
            {60.735268, 24.819204}, // 10
            {60.732328, 24.821916}, // 11
            {60.731199, 24.825709}, // 12
            {60.728539, 24.827266}, // 13
            {60.726875, 24.826537}, // 14
            {60.725742, 24.827716}, // 15
            {60.721354, 24.824954}, // 16
            {60.720514, 24.829812}, // 17
            {60.719396, 24.832938}, // 18
            {60.712525, 24.855294}, // 19
            {60.704676, 24.865769}, // 20
            {60.70121, 24.864856},  // 21
            {60.696557, 24.865852}, // 22
            {60.693148, 24.868628}, // 23
            {60.69165, 24.868976},  // 24
            {60.683171, 24.88355},  // 25
            {60.681586, 24.897519}, // 26
        };
        String[] wpIds = {
            "4IrRwLQi6JXqeSXlqfjn4", "SG02hBbsbqgzGQsKrSWfq", "iuq77x8viaxgTVOowV1NB",
            "q4TiasHKzC8R-PRE1DedB", "iOwve0CUI9t8kJ-FEoL3e", "UXd6-t7Y2RMnZrJNlx33J",
            "V1Iq1jihpdTovR2JKtYgN", "tnvJGpst8wlVn6N_ofutu", "9Zbgy-75u0JZaQ4MFi-sK",
            "Xl6YqwIQXT75AWN3pNaWU", "Tahak0pAc6aMueCbCUHLr", "5Hl8j8xaMpAxGzI9fix_4",
            "2QSaKNZT1VbeP8AzCASyh", "zqwt-IlOUNZiCIy0CeF3L", "3UcVxdDIhIq1QTXs4HtEd",
            "Z1wOVEEOrR57Vg2IPYCgM", "p_SE0RU4Kop0OFJPFBhsL", "0GQPszFg2f4qnwSvESia1",
            "7O2m9lia_ZH_lanvsxrS3", "wugfaAYEyXuh4fyn7rv1i", "N3cBZ8iziFXwVeWINwMb7",
            "xIgE7dn8qm6PYcTEvKLTu", "Z2HRof5m8Z3B4hFGv2f1C", "gMAEc_IVgfAIAIQhNG9hV",
            "eZdpZ7xdP-FkTAAXk1NYb", "ACuo9ZKP4nxBg6hVX98wE", "jLlGAAbkEXUoslFyttrqk",
        };

        List<TrailmapInstructionRequest.Waypoint> waypoints = new ArrayList<>();
        for (int i = 0; i < wps.length; i++) {
            waypoints.add(makeWaypoint(wpIds[i], wps[i][0], wps[i][1]));
        }
        request.setWaypoints(waypoints);

        // Segment definitions: [startIdx, endIdx, heading, headingPenalty]
        // via_points handled separately for segments 12 and 18
        double[][] segDefs = {
            {0,  1,  Double.NaN, Double.NaN},
            {1,  2,  300.3990388831496, 60},
            {2,  3,  93.63646047988937, 60},
            {3,  4,  93.38250530151743, 60},
            {4,  5,  93.23140951444259, 60},
            {5,  6,  69.74421065816438, 60},
            {6,  7,  68.30371198905266, 60},
            {7,  8,  64.30177725127044, 60},
            {8,  9,  1.1737351976685204, 60},
            {9,  10, 145.29079169364888, 60},
            {10, 11, 208.11651439809592, 60},
            {11, 12, 1.1669782851600985, 60},
            {12, 13, 138.59940160086774, 60}, // seg index 12 — has via_points
            {13, 14, 150.42849678799797, 60},
            {14, 15, 141.1854217212342, 60},
            {15, 16, 196.48709016905784, 60},
            {16, 17, 81.69045262057341, 60},
            {17, 18, 115.78797956158331, 60},
            {18, 19, 119.38970760998791, 60}, // seg index 18 — has via_points
            {19, 20, 149.9120913792127, 60},
            {20, 21, 117.56355104430503, 60},
            {21, 22, 198.2794609943801, 60},
            {22, 23, 152.06650782868155, 60},
            {23, 24, 191.00879619455, 60},
            {24, 25, 138.35204548642696, 60},
            {25, 26, 116.55900569008554, 60},
        };

        List<TrailmapInstructionRequest.Segment> segments = new ArrayList<>();
        for (int i = 0; i < segDefs.length; i++) {
            int si = (int) segDefs[i][0];
            int ei = (int) segDefs[i][1];
            TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
            seg.setStart(wpIds[si]);
            seg.setEnd(wpIds[ei]);
            seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
            seg.setProfile("gravel");
            if (!Double.isNaN(segDefs[i][2])) {
                seg.setInitialHeading(segDefs[i][2]);
            }
            if (!Double.isNaN(segDefs[i][3])) {
                seg.setHeadingPenalty(segDefs[i][3]);
            }

            // via_points for segment 12 (wp12 -> wp13)
            if (i == 12) {
                List<TrailmapInstructionRequest.Coordinates> vias = new ArrayList<>();
                double[][] viaCoords = {
                    {60.731199, 24.825709},
                    {60.731173, 24.825755},
                    {60.729332, 24.826175},
                };
                for (double[] vc : viaCoords) {
                    TrailmapInstructionRequest.Coordinates c = new TrailmapInstructionRequest.Coordinates();
                    c.setLat(vc[0]);
                    c.setLng(vc[1]);
                    vias.add(c);
                }
                seg.setViaPoints(vias);
            }

            // via_points for segment 18 (wp18 -> wp19)
            if (i == 18) {
                List<TrailmapInstructionRequest.Coordinates> vias = new ArrayList<>();
                double[][] viaCoords = {
                    {60.721694, 24.845961},
                    {60.721511, 24.846598},
                    {60.712666, 24.855127},
                };
                for (double[] vc : viaCoords) {
                    TrailmapInstructionRequest.Coordinates c = new TrailmapInstructionRequest.Coordinates();
                    c.setLat(vc[0]);
                    c.setLng(vc[1]);
                    vias.add(c);
                }
                seg.setViaPoints(vias);
            }

            segments.add(seg);
        }

        request.setSegments(segments);
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        System.out.println("=== Segment 12 deep diagnostic ===");
        // Segment 12 has via_points. Note: first via_point == start waypoint!
        TrailmapInstructionRequest.Segment seg12 = segments.get(12);
        System.out.println("  Start wp: " + seg12.getStart() + " = " + wps[12][0] + ", " + wps[12][1]);
        System.out.println("  End wp:   " + seg12.getEnd() + " = " + wps[13][0] + ", " + wps[13][1]);
        System.out.println("  Via points:");
        for (TrailmapInstructionRequest.Coordinates via : seg12.getViaPoints()) {
            System.out.println("    " + via.getLat() + ", " + via.getLng());
        }
        System.out.println("  Heading: " + seg12.getInitialHeading() + ", penalty: " + seg12.getHeadingPenalty());

        // Route segment 12 directly via GH to see raw edge IDs
        GHRequest ghReq = new GHRequest();
        // section.points = [start, via1, via2, via3, end]
        ghReq.addPoint(new GHPoint(wps[12][0], wps[12][1]));  // start
        ghReq.addPoint(new GHPoint(60.731199, 24.825709));     // via1 = same as start!
        ghReq.addPoint(new GHPoint(60.731173, 24.825755));     // via2
        ghReq.addPoint(new GHPoint(60.729332, 24.826175));     // via3
        ghReq.addPoint(new GHPoint(wps[13][0], wps[13][1]));   // end
        ghReq.setProfile("gravel");
        ghReq.setPathDetails(List.of("edge_id"));
        ghReq.putHint("instructions", false);
        ghReq.putHint("calc_points", true);
        ghReq.setSnapPreventions(List.of("ferry"));
        ghReq.putHint("heading_penalty", 60.0);
        ghReq.setHeadings(List.of(138.59940160086774, Double.NaN, Double.NaN, Double.NaN, Double.NaN));

        GHResponse ghResp = hopper.route(ghReq);
        if (ghResp.hasErrors()) {
            System.out.println("  GH routing failed: " + ghResp.getErrors());
        } else {
            ResponsePath rp = ghResp.getBest();
            List<PathDetail> edgeDetails = rp.getPathDetails().get("edge_id");
            System.out.println("\n  Raw edge_id path details (" + edgeDetails.size() + " entries):");
            for (int i = 0; i < edgeDetails.size(); i++) {
                PathDetail d = edgeDetails.get(i);
                int edgeId = (Integer) d.getValue();
                EdgeIteratorState edge = baseGraph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
                System.out.printf("    [%d] edge=%d  interval=[%d,%d)  nodes=%d-%d%n",
                    i, edgeId, d.getFirst(), d.getLast(), edge.getBaseNode(), edge.getAdjNode());
            }

            // Deduplicate and show connectivity
            System.out.println("\n  After deduplication + connectivity check:");
            List<Integer> deduped = new ArrayList<>();
            int prev = -1;
            for (PathDetail d : edgeDetails) {
                int eid = (Integer) d.getValue();
                if (eid != prev) {
                    deduped.add(eid);
                } else {
                    System.out.println("    (skipped duplicate edge " + eid + ")");
                }
                prev = eid;
            }
            System.out.println("  Deduped edges: " + deduped.size());

            // Walk the chain and find the break
            if (deduped.size() >= 2) {
                // Determine fromNode
                EdgeIteratorState first = baseGraph.getEdgeIteratorState(deduped.get(0), Integer.MIN_VALUE);
                EdgeIteratorState second = baseGraph.getEdgeIteratorState(deduped.get(1), Integer.MIN_VALUE);
                int nodeA = first.getBaseNode(), nodeB = first.getAdjNode();
                boolean aConn = second.getBaseNode() == nodeA || second.getAdjNode() == nodeA;
                boolean bConn = second.getBaseNode() == nodeB || second.getAdjNode() == nodeB;
                int fromNode;
                if (aConn && !bConn) fromNode = nodeB;
                else if (bConn && !aConn) fromNode = nodeA;
                else {
                    fromNode = nodeA; // ambiguous, just pick one
                    System.out.println("  (ambiguous connectivity: aConn=" + aConn + " bConn=" + bConn + ", picking nodeA=" + nodeA + ")");
                }
                System.out.println("  fromNode: " + fromNode);

                int curNode = fromNode;
                for (int i = 0; i < deduped.size(); i++) {
                    int eid = deduped.get(i);
                    EdgeIteratorState e = baseGraph.getEdgeIteratorState(eid, Integer.MIN_VALUE);
                    int eBase = e.getBaseNode(), eAdj = e.getAdjNode();
                    if (eBase == curNode) {
                        System.out.printf("    edge[%d]=%d: %d->%d OK%n", i, eid, eBase, eAdj);
                        curNode = eAdj;
                    } else if (eAdj == curNode) {
                        System.out.printf("    edge[%d]=%d: %d<-%d OK%n", i, eid, eBase, eAdj);
                        curNode = eBase;
                    } else {
                        System.out.printf("    edge[%d]=%d: BREAK! curNode=%d but edge has %d-%d%n",
                            i, eid, curNode, eBase, eAdj);
                        break;
                    }
                }
            }

            // Also try without the duplicate via point (start == via1)
            System.out.println("\n  === Retry without first via_point (== start) ===");
            GHRequest ghReq2 = new GHRequest();
            ghReq2.addPoint(new GHPoint(wps[12][0], wps[12][1]));  // start
            // Skip via1 since it equals start
            ghReq2.addPoint(new GHPoint(60.731173, 24.825755));     // via2
            ghReq2.addPoint(new GHPoint(60.729332, 24.826175));     // via3
            ghReq2.addPoint(new GHPoint(wps[13][0], wps[13][1]));   // end
            ghReq2.setProfile("gravel");
            ghReq2.setPathDetails(List.of("edge_id"));
            ghReq2.putHint("instructions", false);
            ghReq2.putHint("calc_points", true);
            ghReq2.setSnapPreventions(List.of("ferry"));
            ghReq2.putHint("heading_penalty", 60.0);
            ghReq2.setHeadings(List.of(138.59940160086774, Double.NaN, Double.NaN, Double.NaN));

            GHResponse ghResp2 = hopper.route(ghReq2);
            if (ghResp2.hasErrors()) {
                System.out.println("    GH routing failed: " + ghResp2.getErrors());
            } else {
                ResponsePath rp2 = ghResp2.getBest();
                List<PathDetail> ed2 = rp2.getPathDetails().get("edge_id");
                System.out.println("    Edge details (" + ed2.size() + " entries):");
                List<Integer> deduped2 = new ArrayList<>();
                prev = -1;
                for (PathDetail d : ed2) {
                    int eid = (Integer) d.getValue();
                    EdgeIteratorState e = baseGraph.getEdgeIteratorState(eid, Integer.MIN_VALUE);
                    System.out.printf("      edge=%d  interval=[%d,%d)  nodes=%d-%d%s%n",
                        eid, d.getFirst(), d.getLast(), e.getBaseNode(), e.getAdjNode(),
                        eid == prev ? " (DUP)" : "");
                    if (eid != prev) deduped2.add(eid);
                    prev = eid;
                }
                System.out.println("    Deduped: " + deduped2.size() + " edges — checking connectivity...");
                // Quick walk
                if (deduped2.size() >= 2) {
                    EdgeIteratorState f1 = baseGraph.getEdgeIteratorState(deduped2.get(0), Integer.MIN_VALUE);
                    EdgeIteratorState f2 = baseGraph.getEdgeIteratorState(deduped2.get(1), Integer.MIN_VALUE);
                    int nA = f1.getBaseNode(), nB = f1.getAdjNode();
                    boolean ac = f2.getBaseNode() == nA || f2.getAdjNode() == nA;
                    boolean bc = f2.getBaseNode() == nB || f2.getAdjNode() == nB;
                    int fn = (ac && !bc) ? nB : (bc && !ac) ? nA : nA;
                    int cn = fn;
                    boolean allOk = true;
                    for (int i = 0; i < deduped2.size(); i++) {
                        EdgeIteratorState e = baseGraph.getEdgeIteratorState(deduped2.get(i), Integer.MIN_VALUE);
                        if (e.getBaseNode() == cn) cn = e.getAdjNode();
                        else if (e.getAdjNode() == cn) cn = e.getBaseNode();
                        else { System.out.printf("    BREAK at edge[%d]=%d%n", i, deduped2.get(i)); allOk = false; break; }
                    }
                    if (allOk) System.out.println("    All edges connected OK!");
                }
            }
        }

        // Now try the full request through RouteInstructionGenerator
        System.out.println("\n=== Full 26-segment request ===");
        try {
            RouteInstructionGenerator.Result result = generator.generate(request);
            System.out.println("SUCCESS: " + result.instructions.size() + " instructions");
            printInstructionsDetailed(result);

            InstructionPostProcessor postProcessor = new InstructionPostProcessor();
            postProcessor.process(result.instructions);
            System.out.println("\n--- After post-processing ---");
            printInstructionsDetailed(result);
        } catch (Exception e) {
            System.out.printf("FAILED — %s: %s%n", e.getClass().getSimpleName(), e.getMessage());
            e.printStackTrace(System.out);
            fail("Full request should not throw: " + e.getMessage());
        }
    }

    private int findDivergence(List<Integer> a, List<Integer> b) {
        int minLen = Math.min(a.size(), b.size());
        for (int i = 0; i < minLen; i++) {
            if (!a.get(i).equals(b.get(i))) return i;
        }
        if (a.size() != b.size()) return minLen;
        return -1; // identical
    }

    /**
     * Compute average point-to-nearest-point distance between two polylines.
     * Lower = better match.
     */
    private double polylineMatchScore(PointList a, PointList b) {
        if (a.isEmpty() || b.isEmpty()) return Double.MAX_VALUE;
        double totalDist = 0;
        // Sample every 5th point from a, find nearest in b
        int samples = 0;
        for (int i = 0; i < a.size(); i += 5) {
            double minDist = Double.MAX_VALUE;
            for (int j = 0; j < b.size(); j++) {
                double d = DistanceCalcEarth.DIST_EARTH.calcDist(
                        a.getLat(i), a.getLon(i), b.getLat(j), b.getLon(j));
                if (d < minDist) minDist = d;
            }
            totalDist += minDist;
            samples++;
        }
        return totalDist / samples;
    }

    /**
     * Diagnostic: compound "liity oikea" instruction followed by omitted turn left.
     *
     * API payload:
     *   2 waypoints, 1 segment (gravel), instruction_profile=gravel, locale=fi
     *   snap_preventions=["ferry"]
     *
     * Issue: 2nd turn instruction becomes compound "liity oikea" type (M1 join-side-path),
     * but the following turn left is omitted.
     */
    @Test
    void testCompoundJoinRightFollowedByOmittedTurnLeft() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        // Build request from API payload
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("QmFAzrng56zLDvRjUhwe4", 61.471, 23.741675);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("ewnzNjoiZK4yj7tFAz0oa", 61.46983, 23.742539);

        request.setWaypoints(List.of(wp1, wp2));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("QmFAzrng56zLDvRjUhwe4");
        seg.setEnd("ewnzNjoiZK4yj7tFAz0oa");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        // Generate instructions (before post-processing)
        RouteInstructionGenerator.Result result = generator.generate(request);

        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n=== BEFORE post-processing ===");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        // Apply post-processing
        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions);

        System.out.println("\n=== AFTER post-processing ===");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        // Diagnostic: check what happens around the "join right" instruction
        System.out.println("\n=== Diagnostic: looking for join_direction and then_turn ===");
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            Map<String, Object> extra = instr.getExtraInfoJSON();
            if (extra.containsKey("join_direction") || extra.containsKey("then_turn")) {
                System.out.printf("  [%d] sign=%d (%s) dist=%.1fm name=\"%s\"%n",
                        i, instr.getSign(), signName(instr.getSign()),
                        instr.getDistance(), instr.getName());
                System.out.println("       join_direction=" + extra.get("join_direction"));
                System.out.println("       join_target_type=" + extra.get("join_target_type"));
                System.out.println("       then_turn=" + extra.get("then_turn"));

                // Check what the next instruction is
                if (i + 1 < result.instructions.size()) {
                    Instruction next = result.instructions.get(i + 1);
                    Map<String, Object> nextExtra = next.getExtraInfoJSON();
                    System.out.printf("  [%d] NEXT: sign=%d (%s) dist=%.1fm name=\"%s\"%n",
                            i + 1, next.getSign(), signName(next.getSign()),
                            next.getDistance(), next.getName());
                    System.out.println("       predicted_highway=" + nextExtra.get("predicted_highway"));
                    System.out.println("       turn_angle_deg=" + nextExtra.get("turn_angle_deg"));
                }
            }
        }
    }

    /**
     * Diagnostic: cycleway Y-junctions with informal names not getting instructions.
     *
     * API payload:
     *   2 waypoints, 1 segment (gravel), instruction_profile=gravel, locale=fi,
     *   snap_preventions=["ferry"]
     *
     * Route follows cycleways in Tampere area with several Y-junctions between
     * same-class unpaved paths. The cycleways have informal names that cause S5
     * to suppress instructions (hasName=true + same name + same RoadClass → IGNORE).
     * But on unpaved cycleways these names are informal and the alternatives look
     * identical — rider needs fork guidance.
     */
    @Test
    void testCyclewayYJunctionInformalNames() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("Ng7_PIMugzmhxMvc3KMr-", 61.492078, 23.742357);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("itcPH1CdyS6RPDS1_PZRT", 61.490486, 23.751273);
        request.setWaypoints(List.of(wp1, wp2));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("Ng7_PIMugzmhxMvc3KMr-");
        seg.setEnd("itcPH1CdyS6RPDS1_PZRT");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertNotNull(result.instructions);

        System.out.println("\n========== CYCLEWAY Y-JUNCTION INFORMAL NAMES — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        // Also print surface and trail_fork info for each instruction
        System.out.println("\n--- Extended info ---");
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            Map<String, Object> extra = instr.getExtraInfoJSON();
            System.out.printf("  [%d] sign=%d (%s) name=\"%s\"%n",
                    i, instr.getSign(), signName(instr.getSign()), instr.getName());
            if (extra.containsKey("surface"))
                System.out.println("       surface=" + extra.get("surface"));
            if (extra.containsKey("prev_surface"))
                System.out.println("       prev_surface=" + extra.get("prev_surface"));
            if (extra.containsKey("predicted_surface"))
                System.out.println("       predicted_surface=" + extra.get("predicted_surface"));
            if (extra.containsKey("trail_fork"))
                System.out.println("       trail_fork=" + extra.get("trail_fork"));
            if (extra.containsKey("has_name"))
                System.out.println("       has_name=" + extra.get("has_name"));
            if (extra.containsKey("junction_alt_road_classes"))
                System.out.println("       junction_alt_road_classes=" + extra.get("junction_alt_road_classes"));
            if (extra.containsKey("junction_alt_predicted_highways"))
                System.out.println("       junction_alt_predicted_highways=" + extra.get("junction_alt_predicted_highways"));
        }

        // Examine the route via standard GH to see the edges and junctions
        System.out.println("\n--- Standard GH route for comparison ---");
        GHRequest ghReq = new GHRequest(61.492078, 23.742357, 61.490486, 23.751273)
                .setProfile("gravel_mtb");
        ghReq.setPathDetails(List.of("edge_id", "road_class", "street_name"));
        GHResponse ghRsp = hopper.route(ghReq);
        if (!ghRsp.hasErrors()) {
            ResponsePath path = ghRsp.getBest();
            System.out.println("GH distance: " + Math.round(path.getDistance()) + "m");
            System.out.println("GH instructions: " + path.getInstructions().size());
            for (Instruction instr : path.getInstructions()) {
                System.out.printf("  GH: sign=%d (%s) dist=%.1fm name=\"%s\"%n",
                        instr.getSign(), signName(instr.getSign()),
                        instr.getDistance(), instr.getName());
            }
            // Print edge details
            List<PathDetail> edgeDetails = path.getPathDetails().get("edge_id");
            List<PathDetail> rcDetails = path.getPathDetails().get("road_class");
            List<PathDetail> nameDetails = path.getPathDetails().get("street_name");
            System.out.println("\nEdge chain (" + edgeDetails.size() + " edges):");
            for (int i = 0; i < edgeDetails.size(); i++) {
                String rc = i < rcDetails.size() ? String.valueOf(rcDetails.get(i).getValue()) : "?";
                String nm = i < nameDetails.size() ? String.valueOf(nameDetails.get(i).getValue()) : "?";
                System.out.printf("  edge=%s rc=%s name=\"%s\"%n",
                        edgeDetails.get(i).getValue(), rc, nm);
            }
        }

        // Also examine junction topology along the route using edge explorer
        System.out.println("\n--- Junction analysis along route edges ---");
        EnumEncodedValue<RoadClass> rcEnc = encodingManager.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<PredictedHighway> phEnc = encodingManager.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
        BooleanEncodedValue accessEnc = encodingManager.getBooleanEncodedValue(VehicleAccess.key("bike"));
        EdgeExplorer explorer = baseGraph.createEdgeExplorer();

        GHRequest ghReq2 = new GHRequest(61.492078, 23.742357, 61.490486, 23.751273)
                .setProfile("gravel_mtb");
        ghReq2.setPathDetails(List.of("edge_id"));
        ghReq2.putHint("instructions", false);
        ghReq2.putHint("calc_points", true);
        GHResponse ghRsp2 = hopper.route(ghReq2);
        if (!ghRsp2.hasErrors()) {
            List<PathDetail> edgeIds = ghRsp2.getBest().getPathDetails().get("edge_id");
            Set<Integer> routeEdgeSet = new HashSet<>();
            for (PathDetail d : edgeIds) {
                routeEdgeSet.add((Integer) d.getValue());
            }

            // For each edge, examine the base node's junction
            for (int ei = 0; ei < edgeIds.size(); ei++) {
                int edgeId = (Integer) edgeIds.get(ei).getValue();
                EdgeIteratorState edgeState = baseGraph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
                int baseNode = edgeState.getBaseNode();
                int adjNode = edgeState.getAdjNode();

                // Count alternatives at adjNode (the end of this edge / start of next)
                EdgeIterator iter = explorer.setBaseNode(adjNode);
                int altCount = 0;
                StringBuilder altInfo = new StringBuilder();
                while (iter.next()) {
                    if (iter.getEdge() == edgeId) continue; // skip the edge we came from
                    RoadClass altRC = iter.get(rcEnc);
                    PredictedHighway altPH = iter.get(phEnc);
                    String altName = iter.getName();
                    boolean altAccess = iter.get(accessEnc);
                    altInfo.append(String.format("\n         alt: edge=%d rc=%s ph=%s name=\"%s\" access=%s",
                            iter.getEdge(), altRC, altPH, altName, altAccess));
                    altCount++;
                }
                if (altCount > 1) { // Y-junction or more
                    String edgeName = edgeState.getName();
                    RoadClass edgeRC = edgeState.get(rcEnc);
                    PredictedHighway edgePH = edgeState.get(phEnc);
                    System.out.printf("  Junction at node %d (after edge %d, rc=%s, ph=%s, name=\"%s\") — %d alternatives:%s%n",
                            adjNode, edgeId, edgeRC, edgePH, edgeName, altCount, altInfo);
                }
            }
        }

        // Deep-dive: node 1998008 — the "Eteläpuistonpolku" Y-fork
        // Edge 2462063 ("Pyynikin rantapolku") → node 1998008 → edge 2462767 ("Eteläpuistonpolku")
        // getTurn for edge 2462767 evaluates this junction. What suppresses it?
        System.out.println("\n--- Deep-dive: node 1998008 (Eteläpuistonpolku Y-fork) ---");
        int prevEdgeId = 2462063;
        int currEdgeId = 2462767;
        EdgeIteratorState prevEdgeState = baseGraph.getEdgeIteratorState(prevEdgeId, Integer.MIN_VALUE);
        EdgeIteratorState currEdgeState = baseGraph.getEdgeIteratorState(currEdgeId, Integer.MIN_VALUE);

        // The junction node — figure out which node is shared
        int junctionNode = -1;
        if (prevEdgeState.getAdjNode() == currEdgeState.getBaseNode()) junctionNode = prevEdgeState.getAdjNode();
        else if (prevEdgeState.getAdjNode() == currEdgeState.getAdjNode()) junctionNode = prevEdgeState.getAdjNode();
        else if (prevEdgeState.getBaseNode() == currEdgeState.getBaseNode()) junctionNode = prevEdgeState.getBaseNode();
        else if (prevEdgeState.getBaseNode() == currEdgeState.getAdjNode()) junctionNode = prevEdgeState.getBaseNode();
        System.out.println("  Junction node: " + junctionNode);
        System.out.println("  prevEdge " + prevEdgeId + ": base=" + prevEdgeState.getBaseNode() + " adj=" + prevEdgeState.getAdjNode()
                + " name=\"" + prevEdgeState.getName() + "\" rc=" + prevEdgeState.get(rcEnc) + " ph=" + prevEdgeState.get(phEnc));
        System.out.println("  currEdge " + currEdgeId + ": base=" + currEdgeState.getBaseNode() + " adj=" + currEdgeState.getAdjNode()
                + " name=\"" + currEdgeState.getName() + "\" rc=" + currEdgeState.get(rcEnc) + " ph=" + currEdgeState.get(phEnc));

        // mergedOrSplitWay precondition: same name AND same RC between prev and curr
        boolean sameName = InstructionsHelper.isSameName(currEdgeState.getName(), prevEdgeState.getName());
        boolean sameRC = currEdgeState.get(rcEnc) == prevEdgeState.get(rcEnc);
        System.out.println("  mergedOrSplitWay precondition: sameName=" + sameName + " sameRC=" + sameRC);

        // S4 preconditions: !sameName, samePH, delta near zero, otherContinue null
        boolean s4NameDiff = !sameName;
        boolean s4SamePH = currEdgeState.get(phEnc) == prevEdgeState.get(phEnc);
        System.out.println("  S4 preconditions: nameDiff=" + s4NameDiff + " samePH=" + s4SamePH);

        // List all edges at junction node
        if (junctionNode >= 0) {
            EdgeIterator jIter = explorer.setBaseNode(junctionNode);
            System.out.println("  All edges at junction node " + junctionNode + ":");
            while (jIter.next()) {
                System.out.printf("    edge=%d adj=%d rc=%s ph=%s name=\"%s\" access=%s%n",
                        jIter.getEdge(), jIter.getAdjNode(),
                        jIter.get(rcEnc), jIter.get(phEnc),
                        jIter.getName(), jIter.get(accessEnc));
            }
        }

        // Check: does S4's otherContinue==null hold?
        // otherContinue comes from getOtherContinue() which looks for edges going roughly same direction
        // We can't call it directly, but we can check if there's a "continuing" alternative
        System.out.println("  (Note: S4 requires otherContinue==null. If another edge at this junction");
        System.out.println("   goes roughly straight, otherContinue != null and S4 is bypassed.)");

        // Check which instruction intervals map to which edges
        // Route has 11 edges, instructions have intervals [0,5], [5,6], [6,11], [11,17], [17,17]
        // The "Eteläpuistonpolku" section starts at edge 2462767 (edge index 8 in the chain)
        // Instruction [3] covers interval [11,17] — that's 6 points, matching edges 2462063→end
        // No instruction starts at the transition to "Eteläpuistonpolku"
        System.out.println("\n--- Instruction-to-edge mapping ---");
        System.out.println("  Instruction [3] interval=[11,17] covers the 'Eteläpuistonpolku' section");
        System.out.println("  But its name is still 'Pyynikin rantapolku' — it was created BEFORE");
        System.out.println("  the name change. The name-change junction produced no instruction.");

        // Verify: the "leaving current street" check needs !outgoingEdgesAreSlower
        // Let's check weighting for the alternative edge at node 1998008
        System.out.println("\n--- Weighting check at node 1998008 ---");
        Weighting w = hopper.createWeighting(hopper.getProfile("gravel_mtb"), new PMap());
        EdgeIteratorState altEdge = baseGraph.getEdgeIteratorState(2462060, Integer.MIN_VALUE);
        EdgeIteratorState routeEdge = baseGraph.getEdgeIteratorState(2462767, Integer.MIN_VALUE);
        double altWeightFwd = weighting(w, altEdge, false);
        double altWeightBwd = weighting(w, altEdge, true);
        double routeWeightFwd = weighting(w, routeEdge, false);
        double routeWeightBwd = weighting(w, routeEdge, true);
        System.out.printf("  alt edge 2462060: fwd=%.2f bwd=%.2f%n", altWeightFwd, altWeightBwd);
        System.out.printf("  route edge 2462767: fwd=%.2f bwd=%.2f%n", routeWeightFwd, routeWeightBwd);

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions);

        System.out.println("\n========== CYCLEWAY Y-JUNCTION INFORMAL NAMES — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);
    }

    /** Helper: get edge weight safely */
    private double weighting(Weighting w, EdgeIteratorState edge, boolean reverse) {
        try {
            return w.calcEdgeWeight(edge, reverse);
        } catch (Exception e) {
            return Double.NaN;
        }
    }
}
