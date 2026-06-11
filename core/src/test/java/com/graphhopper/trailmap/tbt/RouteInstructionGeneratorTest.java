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
import com.graphhopper.routing.ev.Surface;
import com.graphhopper.routing.ev.VehicleAccess;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.trailmap.shared.PredictedHighway;
import com.graphhopper.trailmap.shared.PredictedSurface;
import com.graphhopper.trailmap.shared.GravelScaleNum;
import com.graphhopper.trailmap.TrailmapGraphHopper;
import com.graphhopper.trailmap.shared.TrailmapImportRegistry;
import com.graphhopper.routing.InstructionsHelper;
import com.graphhopper.routing.Path;
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
        postProcessor.process(result.instructions, request.getInstructionProfile());

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
            if (extra.containsKey("trail_fork"))
                System.out.println("       trail_fork=" + extra.get("trail_fork"));
            if (extra.containsKey("visual_guidance"))
                System.out.println("       visual_guidance=" + extra.get("visual_guidance"));
            if (extra.containsKey("reframed_from"))
                System.out.println("       reframed_from=" + extra.get("reframed_from") + " ("
                        + signName(((Number) extra.get("reframed_from")).intValue()) + ")");
            if (extra.containsKey("reframer_shape"))
                System.out.println("       reframer_shape=" + extra.get("reframer_shape"));
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
        postProcessor.process(result.instructions, request.getInstructionProfile());

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
        postProcessor.process(result.instructions, request.getInstructionProfile());

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
        postProcessor.process(result.instructions, request.getInstructionProfile());

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
        postProcessor.process(result.instructions, request.getInstructionProfile());

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
        postProcessor.process(result.instructions, request.getInstructionProfile());

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
        postProcessor.process(result.instructions, request.getInstructionProfile());

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
        postProcessor.process(result.instructions, request.getInstructionProfile());

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
        postProcessor.process(result.instructions, request.getInstructionProfile());

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
        postProcessor.process(result.instructions, request.getInstructionProfile());

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
        postProcessor.process(result.instructions, request.getInstructionProfile());

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
        postProcessor.process(result.instructions, request.getInstructionProfile());

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
        postProcessor.process(result.instructions, request.getInstructionProfile());

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
        postProcessor.process(result.instructions, request.getInstructionProfile());

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
        postProcessor.process(result.instructions, request.getInstructionProfile());

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
        postProcessor.process(result.instructions, request.getInstructionProfile());

        System.out.println("\n========== VISUAL GUIDANCE: CYCLEWAY↰ + FOOTWAY↑ FORK — AFTER POST-PROCESSING ==========");
        printInstructionsDetailed(result);
    }

    /**
     * Diagnostic: short gravel route around 61.50377,23.658394 → 61.504136,23.657891
     * (Tampere). Reported via API: route makes a clear right turn but the response has
     * no turn instruction. User hypothesis: the alternative continuing straight is a
     * footway on the same (asphalt) surface as the route's cycleway, so a suppression
     * rule fires on PH-difference even though both options look identical on the
     * ground (only an OSM-tag distinction, not a visible one — only a regulatory
     * traffic sign distinguishes them, and many users miss it).
     *
     * Goal of this test: print the junction(s) along the route with PH, RoadClass,
     * surface, and per-edge angle so we can identify which suppression rule fires
     * (S1 / S2 / S3 / S5 / E2 / fork-handler) and decide whether the rule should
     * gain a "same-major-surface ⇒ do not suppress" gate.
     *
     * No assertions beyond non-null/non-empty result — this is the debug stage of the
     * documented two-stage workflow (debug ⇒ validate). Validation tests come later.
     */
    @Test
    void testCyclewayRightTurnFootwayStraight_suppressedTurn() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        // Original API payload: 2 waypoints, 1 segment (gravel), instruction_profile=gravel,
        // locale=fi, snap_preventions=["ferry"]. No custom_model provided in the payload.
        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("aZ7vebZ1VLcMBvaH-b9As", 61.50377, 23.658394);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("vDy0QlvS1zJk4NjrE43zu", 61.504136, 23.657891);
        request.setWaypoints(List.of(wp1, wp2));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("aZ7vebZ1VLcMBvaH-b9As");
        seg.setEnd("vDy0QlvS1zJk4NjrE43zu");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== CYCLEWAY↱ + FOOTWAY↑ FORK (asphalt) — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        System.out.println("Polyline points: " + result.polyline.size());
        printInstructionsDetailed(result);

        // Print ALL extraInfo on each instruction (predicted_surface, prev_surface, surface,
        // trail_fork, visual_guidance) — printInstructionsDetailed omits some surface fields.
        System.out.println("\n--- Full extraInfo per instruction ---");
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            Map<String, Object> extra = instr.getExtraInfoJSON();
            System.out.printf("  [%d] sign=%d (%s) dist=%.1fm name=\"%s\"%n",
                    i, instr.getSign(), signName(instr.getSign()),
                    instr.getDistance(), instr.getName());
            for (Map.Entry<String, Object> e : extra.entrySet()) {
                System.out.println("       " + e.getKey() + "=" + e.getValue());
            }
        }

        // Standard GH routing with edge-level details to find junctions
        System.out.println("\n--- Standard GH routing with edge details ---");
        GHRequest ghReq = new GHRequest(61.50377, 23.658394, 61.504136, 23.657891).setProfile("gravel");
        ghReq.setSnapPreventions(List.of("ferry"));
        ghReq.setPathDetails(List.of("edge_id", "road_class", "predicted_highway"));
        GHResponse ghRsp = hopper.route(ghReq);
        assertFalse(ghRsp.hasErrors(), "Standard GH routing failed: " + ghRsp.getErrors());

        ResponsePath path = ghRsp.getBest();
        System.out.println("Distance: " + Math.round(path.getDistance()) + "m");
        System.out.println("Edge IDs: " + path.getPathDetails().get("edge_id"));
        System.out.println("Road classes: " + path.getPathDetails().get("road_class"));
        System.out.println("Predicted highways: " + path.getPathDetails().get("predicted_highway"));

        // GH standard instructions for comparison (uses original GH suppression logic — useful
        // baseline to see what the un-Trailmap-ified instruction stream looks like).
        InstructionList ghInstr = path.getInstructions();
        System.out.println("Standard GH instructions (" + ghInstr.size() + "):");
        for (int i = 0; i < ghInstr.size(); i++) {
            Instruction instr = ghInstr.get(i);
            System.out.printf("  [%d] sign=%d (%s) dist=%.1fm name=\"%s\"%n",
                    i, instr.getSign(), signName(instr.getSign()),
                    instr.getDistance(), instr.getName());
        }

        // Per-junction analysis: PH, RoadClass, surface, predicted_surface, name, angle for
        // each edge incident to the junction. This is what we need to identify which rule
        // suppressed the turn.
        System.out.println("\n--- Junction analysis along route (with angles, RC/PH/surface) ---");
        BaseGraph bg = hopper.getBaseGraph();
        EncodingManager em = hopper.getEncodingManager();
        EnumEncodedValue<PredictedHighway> phEnc =
                em.getEnumEncodedValue("predicted_highway", PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> psEnc =
                em.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class);
        EnumEncodedValue<RoadClass> rcEnc =
                em.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EdgeExplorer explorer = bg.createEdgeExplorer();

        List<PathDetail> edgeDetails = path.getPathDetails().get("edge_id");
        PointList routePoints = path.getPoints();

        for (int d = 0; d < edgeDetails.size() - 1; d++) {
            int edgeId = (int) edgeDetails.get(d).getValue();
            int nextEdgeId = (int) edgeDetails.get(d + 1).getValue();

            EdgeIteratorState eState = bg.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            EdgeIteratorState nState = bg.getEdgeIteratorState(nextEdgeId, Integer.MIN_VALUE);

            int junctionNode = -1;
            if (eState.getAdjNode() == nState.getBaseNode() || eState.getAdjNode() == nState.getAdjNode())
                junctionNode = eState.getAdjNode();
            else if (eState.getBaseNode() == nState.getBaseNode() || eState.getBaseNode() == nState.getAdjNode())
                junctionNode = eState.getBaseNode();
            if (junctionNode < 0) continue;

            int fromPt = edgeDetails.get(d).getLast() - 1;
            int toPt = edgeDetails.get(d).getLast();
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

            EdgeIterator iter = explorer.setBaseNode(junctionNode);
            int edgeCount = 0;
            while (iter.next()) edgeCount++;

            // Print every node along the route — including degree-2 pass-through nodes —
            // so we can see whether the right turn is missing because the graph has no
            // junction at the apparent fork, or because a junction exists but suppression
            // is firing.
            System.out.printf("%nNode=%d (%.6f, %.6f) route sign=%d delta=%.3f rad (%.1f°) edgeCount=%d:%n",
                    junctionNode, nodeLat, nodeLon, sign2, delta2, Math.toDegrees(delta2), edgeCount);

            EdgeIterator iter2 = explorer.setBaseNode(junctionNode);
            while (iter2.next()) {
                GHPoint altPt = InstructionsHelper.getPointForOrientationCalculation(iter2, bg.getNodeAccess());
                double altDelta = InstructionsHelper.calculateOrientationDelta(
                        jLat, jLon, altPt.getLat(), altPt.getLon(), orientation);
                int altSign = InstructionsHelper.calculateSign(jLat, jLon, altPt.getLat(), altPt.getLon(), orientation);
                PredictedHighway altPH = iter2.get(phEnc);
                PredictedSurface altPS = psEnc != null ? iter2.get(psEnc) : null;
                RoadClass altRC = iter2.get(rcEnc);
                boolean isIncoming = iter2.getEdge() == edgeId;
                boolean isRoute = iter2.getEdge() == nextEdgeId;
                String marker = isIncoming ? " ← INCOMING" : isRoute ? " ← ROUTE" : "";
                System.out.printf("    edge=%d ph=%s rc=%s pred_surf=%s name=\"%s\" delta=%.1f° sign=%d%s%n",
                        iter2.getEdge(), altPH, altRC, altPS, iter2.getName(),
                        Math.toDegrees(altDelta), altSign, marker);
            }
        }

        // Apply post-processing — show how (if at all) Stage 2 changes the output.
        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        System.out.println("\n========== CYCLEWAY↱ + FOOTWAY↑ FORK (asphalt) — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);
    }

    /**
     * Diagnostic: M1 ("join left/right") false-positive on a "cross the T" pattern.
     *
     * API payload: 61.528925,23.697886 → 61.528892,23.697208 (gravel, fi).
     *
     * <h3>Issue</h3>
     * Rider on Pikkusaarenkuja (residential) is asked to "join left cycleway".
     * Reality the rider sees on the map: Pikkusaarenkuja dead-ends in a T into
     * Lentävänniemenkatu ~5m past the route's left turn (out of getTurn's local
     * view, since the route doesn't take that path), and the "link" used by M1
     * is itself a normal cycleway running perpendicular through Pikkusaarenkuja,
     * not a short connector. So the rider's mental model is closer to
     * "cross the road, continue (slight offset) onto the cycleway" than to a
     * parallel side-path join.
     *
     * <h3>Discriminator considered</h3>
     * At the second turn (right onto the target cycleway), the local junction
     * already exposes the signal: {@code source_road_continues=true} because
     * the link cycleway has an alternative going straight (Δ ≈ +0.1°, same PH).
     * In a textbook M1 (e.g. {@code MINOR_ROAD → 5m SERVICE_ROAD → CYCLEWAY})
     * the connector dead-ends at the join junction → {@code false}. So the
     * gate "M1 must not fire when {@code second.source_road_continues == true}"
     * would catch this case using existing extraInfo, no new fields, no Stage 1
     * change.
     *
     * <h3>Decision (2026-05-06)</h3>
     * Not implementing the discriminator. M1 works well across many cases and
     * adding more gates risks regressing them. Plan instead is to soften the
     * rider-facing presentation of "join" instructions on the client side
     * (TTS phrasing, pre-alert form) so that even when M1 fires on a
     * cross-the-T-style geometry it reads naturally to the rider. Keeping this
     * diagnostic as a regression watch — if a future M1-related change lands,
     * re-run this case to see whether the output drifts.
     */
    @Test
    void testCrossTheT_falseM1Join_cyclewayAcrossTJunction() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("_tjaZSkskN_luk1kVA5Oq", 61.528925, 23.697886);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("o4DeEAkoshMHT1C1BYpfp", 61.528892, 23.697208);
        request.setWaypoints(List.of(wp1, wp2));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("_tjaZSkskN_luk1kVA5Oq");
        seg.setEnd("o4DeEAkoshMHT1C1BYpfp");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== CROSS-THE-T M1 FALSE JOIN — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        System.out.println("Polyline points: " + result.polyline.size());
        printInstructionsDetailed(result);

        // Print the FULL extraInfo on every instruction — we want every key, not just
        // the curated subset printInstructionsDetailed knows about.
        System.out.println("\n--- Full extraInfo per instruction (pre-postprocess) ---");
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            System.out.printf("  [%d] sign=%d (%s) dist=%.1fm name=\"%s\"%n",
                    i, instr.getSign(), signName(instr.getSign()),
                    instr.getDistance(), instr.getName());
            for (Map.Entry<String, Object> e : instr.getExtraInfoJSON().entrySet()) {
                System.out.println("       " + e.getKey() + "=" + e.getValue());
            }
        }

        // Standard GH routing — gives us edge IDs for junction topology walks.
        System.out.println("\n--- Standard GH routing with edge details ---");
        GHRequest ghReq = new GHRequest(61.528925, 23.697886, 61.528892, 23.697208).setProfile("gravel");
        ghReq.setSnapPreventions(List.of("ferry"));
        ghReq.setPathDetails(List.of("edge_id", "road_class", "predicted_highway"));
        GHResponse ghRsp = hopper.route(ghReq);
        assertFalse(ghRsp.hasErrors(), "Standard GH routing failed: " + ghRsp.getErrors());

        ResponsePath path = ghRsp.getBest();
        System.out.println("Distance: " + Math.round(path.getDistance()) + "m");
        System.out.println("Edge IDs: " + path.getPathDetails().get("edge_id"));
        System.out.println("Road classes: " + path.getPathDetails().get("road_class"));
        System.out.println("Predicted highways: " + path.getPathDetails().get("predicted_highway"));

        InstructionList ghInstr = path.getInstructions();
        System.out.println("Standard GH instructions (" + ghInstr.size() + "):");
        for (int i = 0; i < ghInstr.size(); i++) {
            Instruction instr = ghInstr.get(i);
            System.out.printf("  [%d] sign=%d (%s) dist=%.1fm name=\"%s\"%n",
                    i, instr.getSign(), signName(instr.getSign()),
                    instr.getDistance(), instr.getName());
        }

        // Per-junction topology along the route — for every node on the route,
        // print every incident edge with PH/RC/surface/name/angle and a marker for
        // the route's incoming + outgoing edges. We're looking for the dead-end T
        // pattern (perpendicular road continuing both ways at the source-road end).
        System.out.println("\n--- Junction analysis along route (with angles, RC/PH/surface) ---");
        BaseGraph bg = hopper.getBaseGraph();
        EncodingManager em = hopper.getEncodingManager();
        EnumEncodedValue<PredictedHighway> phEnc =
                em.getEnumEncodedValue("predicted_highway", PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> psEnc =
                em.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class);
        EnumEncodedValue<RoadClass> rcEnc =
                em.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EdgeExplorer explorer = bg.createEdgeExplorer();

        List<PathDetail> edgeDetails = path.getPathDetails().get("edge_id");
        PointList routePoints = path.getPoints();

        for (int d = 0; d < edgeDetails.size() - 1; d++) {
            int edgeId = (int) edgeDetails.get(d).getValue();
            int nextEdgeId = (int) edgeDetails.get(d + 1).getValue();

            EdgeIteratorState eState = bg.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            EdgeIteratorState nState = bg.getEdgeIteratorState(nextEdgeId, Integer.MIN_VALUE);

            int junctionNode = -1;
            if (eState.getAdjNode() == nState.getBaseNode() || eState.getAdjNode() == nState.getAdjNode())
                junctionNode = eState.getAdjNode();
            else if (eState.getBaseNode() == nState.getBaseNode() || eState.getBaseNode() == nState.getAdjNode())
                junctionNode = eState.getBaseNode();
            if (junctionNode < 0) continue;

            int fromPt = edgeDetails.get(d).getLast() - 1;
            int toPt = edgeDetails.get(d).getLast();
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

            EdgeIterator iter = explorer.setBaseNode(junctionNode);
            int edgeCount = 0;
            while (iter.next()) edgeCount++;

            System.out.printf("%nNode=%d (%.6f, %.6f) route sign=%d delta=%.3f rad (%.1f°) edgeCount=%d:%n",
                    junctionNode, nodeLat, nodeLon, sign2, delta2, Math.toDegrees(delta2), edgeCount);

            EdgeIterator iter2 = explorer.setBaseNode(junctionNode);
            while (iter2.next()) {
                GHPoint altPt = InstructionsHelper.getPointForOrientationCalculation(iter2, bg.getNodeAccess());
                double altDelta = InstructionsHelper.calculateOrientationDelta(
                        jLat, jLon, altPt.getLat(), altPt.getLon(), orientation);
                int altSign = InstructionsHelper.calculateSign(jLat, jLon, altPt.getLat(), altPt.getLon(), orientation);
                PredictedHighway altPH = iter2.get(phEnc);
                PredictedSurface altPS = psEnc != null ? iter2.get(psEnc) : null;
                RoadClass altRC = iter2.get(rcEnc);
                boolean isIncoming = iter2.getEdge() == edgeId;
                boolean isRoute = iter2.getEdge() == nextEdgeId;
                String marker = isIncoming ? " ← INCOMING" : isRoute ? " ← ROUTE" : "";
                System.out.printf("    edge=%d ph=%s rc=%s pred_surf=%s name=\"%s\" delta=%.1f° sign=%d%s%n",
                        iter2.getEdge(), altPH, altRC, altPS, iter2.getName(),
                        Math.toDegrees(altDelta), altSign, marker);
            }
        }

        // Now post-process and show how M1 (or whoever) shapes the final instruction list.
        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        System.out.println("\n========== CROSS-THE-T M1 FALSE JOIN — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        System.out.println("\n--- Full extraInfo per instruction (post-postprocess) ---");
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            System.out.printf("  [%d] sign=%d (%s) dist=%.1fm name=\"%s\"%n",
                    i, instr.getSign(), signName(instr.getSign()),
                    instr.getDistance(), instr.getName());
            for (Map.Entry<String, Object> e : instr.getExtraInfoJSON().entrySet()) {
                System.out.println("       " + e.getKey() + "=" + e.getValue());
            }
        }
    }

    /**
     * Diagnostic for M3 ("stay left/right") consecutive-fork merge thresholds.
     * <p>
     * API payload (foot): 61.427946,23.890105 → 61.426671,23.890025 — user reported
     * two "stay on left" KEEP_LEFT instructions ~140m apart being merged. Goal of this
     * test: print KEEP_LEFT/KEEP_RIGHT distances for the same waypoints under both
     * trailmap_foot and gravel profiles, so we can pick threshold-bracketing cases for
     * the validation test pair.
     */
    @Test
    void testM3StayLeftThresholds_footAndGravel() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        for (String profile : new String[]{"trailmap_foot", "gravel"}) {
            TrailmapInstructionRequest request = new TrailmapInstructionRequest();
            request.setWaypoints(List.of(
                    makeWaypoint("MdwMF-B90KYbDQq6Ul_Gf", 61.427946, 23.890105),
                    makeWaypoint("zmENlCkWzEeXzzMR1JHmB", 61.426671, 23.890025)));

            TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
            seg.setStart("MdwMF-B90KYbDQq6Ul_Gf");
            seg.setEnd("zmENlCkWzEeXzzMR1JHmB");
            seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
            seg.setProfile(profile);

            request.setSegments(List.of(seg));
            request.setInstructionProfile(profile);
            request.setLocale("fi");
            request.setSnapPreventions(List.of("ferry"));

            RouteInstructionGenerator.Result result = generator.generate(request);
            assertNotNull(result);

            System.out.printf("%n========== M3 INVESTIGATION — profile=%s — BEFORE POST-PROCESSING ==========%n", profile);
            System.out.println("Instructions: " + result.instructions.size());
            printInstructionsDetailed(result);

            // Print KEEP-only summary
            System.out.println("\n--- KEEP_LEFT / KEEP_RIGHT only (pre-postprocess) ---");
            for (int i = 0; i < result.instructions.size(); i++) {
                Instruction instr = result.instructions.get(i);
                if (instr.getSign() == Instruction.KEEP_LEFT || instr.getSign() == Instruction.KEEP_RIGHT) {
                    System.out.printf("  [%d] %s dist=%.1fm pred_highway=%s name=\"%s\"%n",
                            i, signName(instr.getSign()), instr.getDistance(),
                            instr.getExtraInfoJSON().get("predicted_highway"),
                            instr.getName());
                }
            }

            // Apply post-processing using NEW profile-aware overload
            InstructionPostProcessor postProcessor = new InstructionPostProcessor();
            postProcessor.process(result.instructions, request.getInstructionProfile());

            System.out.printf("%n========== M3 INVESTIGATION — profile=%s — AFTER POST-PROCESSING ==========%n", profile);
            System.out.println("Instructions: " + result.instructions.size());
            printInstructionsDetailed(result);
        }
    }

    /**
     * Diagnostic: cycleway T-junction with footway-left + cycleway-right — variant of
     * the original E5 case, but at a T rather than a straight-with-fork.
     *
     * API payload (gravel): 61.508318,23.687523 → 61.508691,23.687772.
     *
     * Reported behaviour: no turn instruction at the T-junction. The route is on a
     * cycleway and the bike-routable continuation is the right-hand cycleway. To the
     * left, a footway departs that visually looks identical to the cycleway (only a
     * traffic sign 20m out distinguishes them).
     *
     * Goal of this test: print junction topology around the route's turn point so we
     * can see whether the turn is suppressed by S1 (same PH + all alts different PH),
     * by the forced-path branch (footway invisible to bike weighting → no alts), or
     * by some other rule. Compare standard GH instructions against TbT to bracket the
     * difference. No production-code changes; analysis only.
     */
    @Test
    void testCyclewayTJunctionFootwayLeft_suppressedTurn() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("rQb5GcsGGrvcNj4hhU9RP", 61.508318, 23.687523),
                makeWaypoint("qvCCj4IkrwQxEp0dPr0os", 61.508691, 23.687772)));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("rQb5GcsGGrvcNj4hhU9RP");
        seg.setEnd("qvCCj4IkrwQxEp0dPr0os");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== CYCLEWAY T-JUNCTION + FOOTWAY-LEFT — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        System.out.println("Polyline points: " + result.polyline.size());
        printInstructionsDetailed(result);

        // Full extraInfo dump
        System.out.println("\n--- Full extraInfo per instruction (pre-postprocess) ---");
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            System.out.printf("  [%d] sign=%d (%s) dist=%.1fm name=\"%s\"%n",
                    i, instr.getSign(), signName(instr.getSign()),
                    instr.getDistance(), instr.getName());
            for (Map.Entry<String, Object> e : instr.getExtraInfoJSON().entrySet()) {
                System.out.println("       " + e.getKey() + "=" + e.getValue());
            }
        }

        // Standard GH for comparison
        System.out.println("\n--- Standard GH routing with edge details ---");
        GHRequest ghReq = new GHRequest(61.508318, 23.687523, 61.508691, 23.687772).setProfile("gravel");
        ghReq.setSnapPreventions(List.of("ferry"));
        ghReq.setPathDetails(List.of("edge_id", "road_class", "predicted_highway"));
        GHResponse ghRsp = hopper.route(ghReq);
        assertFalse(ghRsp.hasErrors(), "Standard GH routing failed: " + ghRsp.getErrors());

        ResponsePath path = ghRsp.getBest();
        System.out.println("Distance: " + Math.round(path.getDistance()) + "m");
        System.out.println("Edge IDs: " + path.getPathDetails().get("edge_id"));
        System.out.println("Road classes: " + path.getPathDetails().get("road_class"));
        System.out.println("Predicted highways: " + path.getPathDetails().get("predicted_highway"));

        InstructionList ghInstr = path.getInstructions();
        System.out.println("Standard GH instructions (" + ghInstr.size() + "):");
        for (int i = 0; i < ghInstr.size(); i++) {
            Instruction instr = ghInstr.get(i);
            System.out.printf("  [%d] sign=%d (%s) dist=%.1fm name=\"%s\"%n",
                    i, instr.getSign(), signName(instr.getSign()),
                    instr.getDistance(), instr.getName());
        }

        // Per-junction topology along the route (every node, not just true junctions —
        // we want to see all edges including pass-throughs and access-blocked alts).
        System.out.println("\n--- Junction analysis along route (with angles, RC/PH/surface) ---");
        BaseGraph bg = hopper.getBaseGraph();
        EncodingManager em = hopper.getEncodingManager();
        EnumEncodedValue<PredictedHighway> phEnc =
                em.getEnumEncodedValue("predicted_highway", PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> psEnc =
                em.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class);
        EnumEncodedValue<RoadClass> rcEnc =
                em.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EdgeExplorer explorer = bg.createEdgeExplorer();

        List<PathDetail> edgeDetails = path.getPathDetails().get("edge_id");
        PointList routePoints = path.getPoints();

        for (int d = 0; d < edgeDetails.size() - 1; d++) {
            int edgeId = (int) edgeDetails.get(d).getValue();
            int nextEdgeId = (int) edgeDetails.get(d + 1).getValue();

            EdgeIteratorState eState = bg.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            EdgeIteratorState nState = bg.getEdgeIteratorState(nextEdgeId, Integer.MIN_VALUE);

            int junctionNode = -1;
            if (eState.getAdjNode() == nState.getBaseNode() || eState.getAdjNode() == nState.getAdjNode())
                junctionNode = eState.getAdjNode();
            else if (eState.getBaseNode() == nState.getBaseNode() || eState.getBaseNode() == nState.getAdjNode())
                junctionNode = eState.getBaseNode();
            if (junctionNode < 0) continue;

            int fromPt = edgeDetails.get(d).getLast() - 1;
            int toPt = edgeDetails.get(d).getLast();
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

            EdgeIterator iter = explorer.setBaseNode(junctionNode);
            int edgeCount = 0;
            while (iter.next()) edgeCount++;

            System.out.printf("%nNode=%d (%.6f, %.6f) route sign=%d delta=%.3f rad (%.1f°) edgeCount=%d:%n",
                    junctionNode, nodeLat, nodeLon, sign2, delta2, Math.toDegrees(delta2), edgeCount);

            EdgeIterator iter2 = explorer.setBaseNode(junctionNode);
            while (iter2.next()) {
                GHPoint altPt = InstructionsHelper.getPointForOrientationCalculation(iter2, bg.getNodeAccess());
                double altDelta = InstructionsHelper.calculateOrientationDelta(
                        jLat, jLon, altPt.getLat(), altPt.getLon(), orientation);
                int altSign = InstructionsHelper.calculateSign(jLat, jLon, altPt.getLat(), altPt.getLon(), orientation);
                PredictedHighway altPH = iter2.get(phEnc);
                PredictedSurface altPS = psEnc != null ? iter2.get(psEnc) : null;
                RoadClass altRC = iter2.get(rcEnc);
                boolean isIncoming = iter2.getEdge() == edgeId;
                boolean isRoute = iter2.getEdge() == nextEdgeId;
                String marker = isIncoming ? " ← INCOMING" : isRoute ? " ← ROUTE" : "";
                System.out.printf("    edge=%d ph=%s rc=%s pred_surf=%s name=\"%s\" delta=%.1f° sign=%d%s%n",
                        iter2.getEdge(), altPH, altRC, altPS, iter2.getName(),
                        Math.toDegrees(altDelta), altSign, marker);
            }
        }

        // Apply post-processing
        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        System.out.println("\n========== CYCLEWAY T-JUNCTION + FOOTWAY-LEFT — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);
    }

    /**
     * Diagnostic: open case, no preconception. API payload (gravel):
     * 61.524176,23.626523 → 61.524667,23.62469. Print full instruction stream
     * before and after post-processing, junction topology along the route, and
     * standard GH instructions for comparison so the issue can be spotted from
     * the data alone.
     */
    @Test
    void testOpenCase_61_524176_23_626523_to_61_524667_23_62469() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("3Tdlvr6-Gh7Ydrtdmunv7", 61.524176, 23.626523),
                makeWaypoint("cGy_eDRCy7N1vrpP_mEDZ", 61.524667, 23.62469)));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("3Tdlvr6-Gh7Ydrtdmunv7");
        seg.setEnd("cGy_eDRCy7N1vrpP_mEDZ");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== OPEN CASE 61_524176 → 61_524667 — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        System.out.println("Polyline points: " + result.polyline.size());
        printInstructionsDetailed(result);

        System.out.println("\n--- Full extraInfo per instruction (pre-postprocess) ---");
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            System.out.printf("  [%d] sign=%d (%s) dist=%.1fm name=\"%s\"%n",
                    i, instr.getSign(), signName(instr.getSign()),
                    instr.getDistance(), instr.getName());
            for (Map.Entry<String, Object> e : instr.getExtraInfoJSON().entrySet()) {
                System.out.println("       " + e.getKey() + "=" + e.getValue());
            }
        }

        // Standard GH for comparison
        System.out.println("\n--- Standard GH routing with edge details ---");
        GHRequest ghReq = new GHRequest(61.524176, 23.626523, 61.524667, 23.62469).setProfile("gravel");
        ghReq.setSnapPreventions(List.of("ferry"));
        ghReq.setPathDetails(List.of("edge_id", "road_class", "predicted_highway"));
        GHResponse ghRsp = hopper.route(ghReq);
        assertFalse(ghRsp.hasErrors(), "Standard GH routing failed: " + ghRsp.getErrors());

        ResponsePath path = ghRsp.getBest();
        System.out.println("Distance: " + Math.round(path.getDistance()) + "m");
        System.out.println("Edge IDs: " + path.getPathDetails().get("edge_id"));
        System.out.println("Road classes: " + path.getPathDetails().get("road_class"));
        System.out.println("Predicted highways: " + path.getPathDetails().get("predicted_highway"));

        InstructionList ghInstr = path.getInstructions();
        System.out.println("Standard GH instructions (" + ghInstr.size() + "):");
        for (int i = 0; i < ghInstr.size(); i++) {
            Instruction instr = ghInstr.get(i);
            System.out.printf("  [%d] sign=%d (%s) dist=%.1fm name=\"%s\"%n",
                    i, instr.getSign(), signName(instr.getSign()),
                    instr.getDistance(), instr.getName());
        }

        // Per-junction topology along the route
        System.out.println("\n--- Junction analysis along route (with angles, RC/PH/surface) ---");
        BaseGraph bg = hopper.getBaseGraph();
        EncodingManager em = hopper.getEncodingManager();
        EnumEncodedValue<PredictedHighway> phEnc =
                em.getEnumEncodedValue("predicted_highway", PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> psEnc =
                em.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class);
        EnumEncodedValue<RoadClass> rcEnc =
                em.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EdgeExplorer explorer = bg.createEdgeExplorer();

        List<PathDetail> edgeDetails = path.getPathDetails().get("edge_id");
        PointList routePoints = path.getPoints();

        for (int d = 0; d < edgeDetails.size() - 1; d++) {
            int edgeId = (int) edgeDetails.get(d).getValue();
            int nextEdgeId = (int) edgeDetails.get(d + 1).getValue();

            EdgeIteratorState eState = bg.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            EdgeIteratorState nState = bg.getEdgeIteratorState(nextEdgeId, Integer.MIN_VALUE);

            int junctionNode = -1;
            if (eState.getAdjNode() == nState.getBaseNode() || eState.getAdjNode() == nState.getAdjNode())
                junctionNode = eState.getAdjNode();
            else if (eState.getBaseNode() == nState.getBaseNode() || eState.getBaseNode() == nState.getAdjNode())
                junctionNode = eState.getBaseNode();
            if (junctionNode < 0) continue;

            int fromPt = edgeDetails.get(d).getLast() - 1;
            int toPt = edgeDetails.get(d).getLast();
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

            EdgeIterator iter = explorer.setBaseNode(junctionNode);
            int edgeCount = 0;
            while (iter.next()) edgeCount++;

            System.out.printf("%nNode=%d (%.6f, %.6f) route sign=%d delta=%.3f rad (%.1f°) edgeCount=%d:%n",
                    junctionNode, nodeLat, nodeLon, sign2, delta2, Math.toDegrees(delta2), edgeCount);

            EdgeIterator iter2 = explorer.setBaseNode(junctionNode);
            while (iter2.next()) {
                GHPoint altPt = InstructionsHelper.getPointForOrientationCalculation(iter2, bg.getNodeAccess());
                double altDelta = InstructionsHelper.calculateOrientationDelta(
                        jLat, jLon, altPt.getLat(), altPt.getLon(), orientation);
                int altSign = InstructionsHelper.calculateSign(jLat, jLon, altPt.getLat(), altPt.getLon(), orientation);
                PredictedHighway altPH = iter2.get(phEnc);
                PredictedSurface altPS = psEnc != null ? iter2.get(psEnc) : null;
                RoadClass altRC = iter2.get(rcEnc);
                boolean isIncoming = iter2.getEdge() == edgeId;
                boolean isRoute = iter2.getEdge() == nextEdgeId;
                String marker = isIncoming ? " ← INCOMING" : isRoute ? " ← ROUTE" : "";
                System.out.printf("    edge=%d ph=%s rc=%s pred_surf=%s name=\"%s\" delta=%.1f° sign=%d%s%n",
                        iter2.getEdge(), altPH, altRC, altPS, iter2.getName(),
                        Math.toDegrees(altDelta), altSign, marker);
            }
        }

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        System.out.println("\n========== OPEN CASE 61_524176 → 61_524667 — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        System.out.println("\n--- Full extraInfo per instruction (post-postprocess) ---");
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            System.out.printf("  [%d] sign=%d (%s) dist=%.1fm name=\"%s\"%n",
                    i, instr.getSign(), signName(instr.getSign()),
                    instr.getDistance(), instr.getName());
            for (Map.Entry<String, Object> e : instr.getExtraInfoJSON().entrySet()) {
                System.out.println("       " + e.getKey() + "=" + e.getValue());
            }
        }
    }

    /**
     * Diagnostic: false "join left" at incoming-road-ends pattern. API payload
     * (gravel): 61.459038,23.451706 → 61.458822,23.452307.
     * <p>
     * Reported behaviour (per user): instruction stream contains a "join left"
     * but the rider's experience is: incoming road ENDS at a bigger road, then
     * left, then right onto a new path. A standard sidepath join doesn't apply
     * because the incoming source road dead-ends — the rider must turn at the
     * T-junction; the subsequent right is a separate navigation event onto a
     * different way.
     * <p>
     * Goal of this test: print full instruction stream before/after post-processing,
     * junction topology along the route, standard GH instructions for comparison
     * so we can identify whether the M1 cascade guard catches it (it shouldn't —
     * source_road_continues should already be false at a T-junction with road end),
     * or whether the M1 geometry-only fallback is firing on a legitimate T-end.
     */
    @Test
    void testOpenCase_61_459038_23_451706_to_61_458822_23_452307() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("WoqoZzlXA4lG2Bab89Lda", 61.459038, 23.451706),
                makeWaypoint("pf-fnELpqBDCZPl1al26U", 61.458822, 23.452307)));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("WoqoZzlXA4lG2Bab89Lda");
        seg.setEnd("pf-fnELpqBDCZPl1al26U");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== OPEN CASE 61_459038 → 61_458822 — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        System.out.println("Polyline points: " + result.polyline.size());
        printInstructionsDetailed(result);

        System.out.println("\n--- Full extraInfo per instruction (pre-postprocess) ---");
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            System.out.printf("  [%d] sign=%d (%s) dist=%.1fm name=\"%s\"%n",
                    i, instr.getSign(), signName(instr.getSign()),
                    instr.getDistance(), instr.getName());
            for (Map.Entry<String, Object> e : instr.getExtraInfoJSON().entrySet()) {
                System.out.println("       " + e.getKey() + "=" + e.getValue());
            }
        }

        System.out.println("\n--- Standard GH routing with edge details ---");
        GHRequest ghReq = new GHRequest(61.459038, 23.451706, 61.458822, 23.452307).setProfile("gravel");
        ghReq.setSnapPreventions(List.of("ferry"));
        ghReq.setPathDetails(List.of("edge_id", "road_class", "predicted_highway"));
        GHResponse ghRsp = hopper.route(ghReq);
        assertFalse(ghRsp.hasErrors(), "Standard GH routing failed: " + ghRsp.getErrors());

        ResponsePath path = ghRsp.getBest();
        System.out.println("Distance: " + Math.round(path.getDistance()) + "m");
        System.out.println("Edge IDs: " + path.getPathDetails().get("edge_id"));
        System.out.println("Road classes: " + path.getPathDetails().get("road_class"));
        System.out.println("Predicted highways: " + path.getPathDetails().get("predicted_highway"));

        InstructionList ghInstr = path.getInstructions();
        System.out.println("Standard GH instructions (" + ghInstr.size() + "):");
        for (int i = 0; i < ghInstr.size(); i++) {
            Instruction instr = ghInstr.get(i);
            System.out.printf("  [%d] sign=%d (%s) dist=%.1fm name=\"%s\"%n",
                    i, instr.getSign(), signName(instr.getSign()),
                    instr.getDistance(), instr.getName());
        }

        System.out.println("\n--- Junction analysis along route (with angles, RC/PH/surface) ---");
        BaseGraph bg = hopper.getBaseGraph();
        EncodingManager em = hopper.getEncodingManager();
        EnumEncodedValue<PredictedHighway> phEnc =
                em.getEnumEncodedValue("predicted_highway", PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> psEnc =
                em.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class);
        EnumEncodedValue<RoadClass> rcEnc =
                em.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EdgeExplorer explorer = bg.createEdgeExplorer();

        List<PathDetail> edgeDetails = path.getPathDetails().get("edge_id");
        PointList routePoints = path.getPoints();

        for (int d = 0; d < edgeDetails.size() - 1; d++) {
            int edgeId = (int) edgeDetails.get(d).getValue();
            int nextEdgeId = (int) edgeDetails.get(d + 1).getValue();

            EdgeIteratorState eState = bg.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            EdgeIteratorState nState = bg.getEdgeIteratorState(nextEdgeId, Integer.MIN_VALUE);

            int junctionNode = -1;
            if (eState.getAdjNode() == nState.getBaseNode() || eState.getAdjNode() == nState.getAdjNode())
                junctionNode = eState.getAdjNode();
            else if (eState.getBaseNode() == nState.getBaseNode() || eState.getBaseNode() == nState.getAdjNode())
                junctionNode = eState.getBaseNode();
            if (junctionNode < 0) continue;

            int fromPt = edgeDetails.get(d).getLast() - 1;
            int toPt = edgeDetails.get(d).getLast();
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

            EdgeIterator iter = explorer.setBaseNode(junctionNode);
            int edgeCount = 0;
            while (iter.next()) edgeCount++;

            System.out.printf("%nNode=%d (%.6f, %.6f) route sign=%d delta=%.3f rad (%.1f°) edgeCount=%d:%n",
                    junctionNode, nodeLat, nodeLon, sign2, delta2, Math.toDegrees(delta2), edgeCount);

            EdgeIterator iter2 = explorer.setBaseNode(junctionNode);
            while (iter2.next()) {
                GHPoint altPt = InstructionsHelper.getPointForOrientationCalculation(iter2, bg.getNodeAccess());
                double altDelta = InstructionsHelper.calculateOrientationDelta(
                        jLat, jLon, altPt.getLat(), altPt.getLon(), orientation);
                int altSign = InstructionsHelper.calculateSign(jLat, jLon, altPt.getLat(), altPt.getLon(), orientation);
                PredictedHighway altPH = iter2.get(phEnc);
                PredictedSurface altPS = psEnc != null ? iter2.get(psEnc) : null;
                RoadClass altRC = iter2.get(rcEnc);
                boolean isIncoming = iter2.getEdge() == edgeId;
                boolean isRoute = iter2.getEdge() == nextEdgeId;
                String marker = isIncoming ? " ← INCOMING" : isRoute ? " ← ROUTE" : "";
                System.out.printf("    edge=%d ph=%s rc=%s pred_surf=%s name=\"%s\" delta=%.1f° sign=%d%s%n",
                        iter2.getEdge(), altPH, altRC, altPS, iter2.getName(),
                        Math.toDegrees(altDelta), altSign, marker);
            }
        }

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        System.out.println("\n========== OPEN CASE 61_459038 → 61_458822 — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        System.out.println("\n--- Full extraInfo per instruction (post-postprocess) ---");
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            System.out.printf("  [%d] sign=%d (%s) dist=%.1fm name=\"%s\"%n",
                    i, instr.getSign(), signName(instr.getSign()),
                    instr.getDistance(), instr.getName());
            for (Map.Entry<String, Object> e : instr.getExtraInfoJSON().entrySet()) {
                System.out.println("       " + e.getKey() + "=" + e.getValue());
            }
        }
    }

    /**
     * Verification: for each M1-relevant route we have, find every instruction
     * post-processing where M1 fired (carries {@code join_direction}) and compute
     * the proposed discriminator: at that instruction's first turn junction, what
     * is the smallest {@code |altDelta|} (excluding incoming and route edges)?
     * <p>
     * Discriminator: if min |altDelta| ≤ 30°, some way continues nearly straight
     * past the first junction → M1 is semantically valid. Otherwise the source
     * way truly ends → M1 should be rejected.
     * <p>
     * No production code change. Output supports the design discussion only.
     */
    @Test
    void verifyM1ContinuationDiscriminator_acrossKnownCases() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        CustomModel cwBoost = new CustomModel();
        cwBoost.addToPriority(Statement.If("predicted_highway == CYCLEWAY",
                Statement.Op.MULTIPLY, "1.3"));

        // Each entry: descriptive name, expected outcome, profile, customModel|null,
        // initialHeading|NaN, lat1, lng1, lat2, lng2.
        Object[][] cases = {
            {"E5 base case (61.50377→61.504136)", "no M1 expected (E5 emits, no opposite-pair)",
                    "gravel", null, Double.NaN,
                    61.50377, 23.658394, 61.504136, 23.657891},
            {"Cross-the-T (Pikkusaarenkuja)", "M1 fires today; we left it alone (source continues)",
                    "gravel", null, Double.NaN,
                    61.528925, 23.697886, 61.528892, 23.697208},
            {"M1 cascade (sidepath + extra turns)", "M1 fires once correctly (cascade fix)",
                    "gravel", null, Double.NaN,
                    61.524176, 23.626523, 61.524667, 23.62469},
            {"Real T-end (Taivalkunnantie) — NEW", "M1 fires today (BUG); should NOT fire",
                    "gravel", null, Double.NaN,
                    61.459038, 23.451706, 61.458822, 23.452307},
            {"Test 12 testMissingJoinCyclewayToRoad", "M1 should fire (legitimate join)",
                    "gravel", cwBoost, 139.8063235836866,
                    61.503449, 23.684035, 61.502604, 23.687161},
            {"Test 11 testComplexTurnsFootRoute (foot)", "M1 should NOT fire (real turns)",
                    "trailmap_foot", null, Double.NaN,
                    61.499065, 23.631531, 61.499816, 23.631703},
            {"Test 15 testFalseM1OnRightLeftRight (foot)", "M1 should NOT fire on the false-M1 pair",
                    "trailmap_foot", null, Double.NaN,
                    61.459012, 23.828659, 61.458676, 23.829354},
        };

        EncodingManager em = hopper.getEncodingManager();
        EnumEncodedValue<PredictedHighway> phEnc =
                em.getEnumEncodedValue("predicted_highway", PredictedHighway.class);
        EnumEncodedValue<RoadClass> rcEnc =
                em.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EdgeExplorer explorer = baseGraph.createEdgeExplorer();

        for (Object[] tc : cases) {
            String name = (String) tc[0];
            String expected = (String) tc[1];
            String profile = (String) tc[2];
            CustomModel cm = (CustomModel) tc[3];
            double initialHeading = (Double) tc[4];
            double lat1 = (Double) tc[5], lng1 = (Double) tc[6];
            double lat2 = (Double) tc[7], lng2 = (Double) tc[8];

            System.out.println("\n========================================================================");
            System.out.println("CASE: " + name);
            System.out.println("EXPECTED: " + expected);
            System.out.println("========================================================================");

            // Build TbT request
            TrailmapInstructionRequest request = new TrailmapInstructionRequest();
            request.setWaypoints(List.of(
                    makeWaypoint("a", lat1, lng1),
                    makeWaypoint("b", lat2, lng2)));
            TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
            seg.setStart("a");
            seg.setEnd("b");
            seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
            seg.setProfile(profile);
            if (cm != null) seg.setCustomModel(cm);
            if (!Double.isNaN(initialHeading)) {
                seg.setInitialHeading(initialHeading);
                seg.setHeadingPenalty(60.0);
            }
            request.setSegments(List.of(seg));
            request.setInstructionProfile(profile);
            request.setLocale("fi");
            request.setSnapPreventions(List.of("ferry"));

            RouteInstructionGenerator.Result result = generator.generate(request);

            // Standard GH path with edge_id details (for matching turn points to graph nodes)
            GHRequest ghReq = new GHRequest(lat1, lng1, lat2, lng2).setProfile(profile);
            if (cm != null) ghReq.setCustomModel(cm);
            ghReq.setSnapPreventions(List.of("ferry"));
            ghReq.setPathDetails(List.of("edge_id"));
            GHResponse ghRsp = hopper.route(ghReq);
            if (ghRsp.hasErrors()) {
                System.out.println("  Routing failed: " + ghRsp.getErrors());
                continue;
            }
            ResponsePath path = ghRsp.getBest();
            List<PathDetail> edgeDetails = path.getPathDetails().get("edge_id");
            PointList routePoints = path.getPoints();

            // Run post-processor; M1-fired instructions get join_direction
            InstructionPostProcessor postProcessor = new InstructionPostProcessor();
            postProcessor.process(result.instructions, request.getInstructionProfile());

            // For each M1-fired instruction, find its first turn point and compute discriminator
            int m1Count = 0;
            for (Instruction instr : result.instructions) {
                if (!instr.getExtraInfoJSON().containsKey("join_direction")) continue;
                m1Count++;

                double turnLat = instr.getPoints().getLat(0);
                double turnLon = instr.getPoints().getLon(0);
                Object angleObj = instr.getExtraInfoJSON().get("turn_angle_deg");
                double turnAngle = (angleObj instanceof Number) ? ((Number) angleObj).doubleValue() : 0.0;

                // Find the polyline index in routePoints matching the turn point
                int matchIdx = -1;
                for (int p = 0; p < routePoints.size(); p++) {
                    if (Math.abs(routePoints.getLat(p) - turnLat) < 1e-7
                            && Math.abs(routePoints.getLon(p) - turnLon) < 1e-7) {
                        matchIdx = p;
                        break;
                    }
                }

                // Find the edge transition where this point sits (boundary between consecutive edges)
                int edgeIdx = -1;
                for (int d = 0; d < edgeDetails.size() - 1; d++) {
                    if (edgeDetails.get(d).getLast() == matchIdx) {
                        edgeIdx = d;
                        break;
                    }
                }
                if (edgeIdx < 0) {
                    System.out.printf("  M1 #%d: turn at %.6f,%.6f turn_angle_deg=%.1f — could not match polyline index%n",
                            m1Count, turnLat, turnLon, turnAngle);
                    continue;
                }

                // Find the junction node at this edge boundary
                int incomingEdgeId = (int) edgeDetails.get(edgeIdx).getValue();
                int routeEdgeId = (int) edgeDetails.get(edgeIdx + 1).getValue();
                EdgeIteratorState eState = baseGraph.getEdgeIteratorState(incomingEdgeId, Integer.MIN_VALUE);
                EdgeIteratorState nState = baseGraph.getEdgeIteratorState(routeEdgeId, Integer.MIN_VALUE);
                int junctionNode = -1;
                if (eState.getAdjNode() == nState.getBaseNode() || eState.getAdjNode() == nState.getAdjNode())
                    junctionNode = eState.getAdjNode();
                else if (eState.getBaseNode() == nState.getBaseNode() || eState.getBaseNode() == nState.getAdjNode())
                    junctionNode = eState.getBaseNode();
                if (junctionNode < 0) continue;

                // Approach orientation from the previous polyline point
                int fromPt = Math.max(0, matchIdx - 1);
                double prevLat = routePoints.getLat(fromPt);
                double prevLon = routePoints.getLon(fromPt);
                double jLat = routePoints.getLat(matchIdx);
                double jLon = routePoints.getLon(matchIdx);
                double orientation = AngleCalc.ANGLE_CALC.calcOrientation(prevLat, prevLon, jLat, jLon);

                // Walk all alts at the junction, compute min |altDelta|
                double minAltDelta = Double.POSITIVE_INFINITY;
                String straightestAltDesc = "";
                EdgeIterator iter = explorer.setBaseNode(junctionNode);
                while (iter.next()) {
                    if (iter.getEdge() == incomingEdgeId) continue;
                    if (iter.getEdge() == routeEdgeId) continue;
                    GHPoint altPt = InstructionsHelper.getPointForOrientationCalculation(iter, baseGraph.getNodeAccess());
                    double altDelta = InstructionsHelper.calculateOrientationDelta(
                            jLat, jLon, altPt.getLat(), altPt.getLon(), orientation);
                    double absDelta = Math.abs(Math.toDegrees(altDelta));
                    if (absDelta < minAltDelta) {
                        minAltDelta = absDelta;
                        PredictedHighway altPH = iter.get(phEnc);
                        RoadClass altRC = iter.get(rcEnc);
                        straightestAltDesc = String.format("edge=%d ph=%s rc=%s name=\"%s\" delta=%.1f°",
                                iter.getEdge(), altPH, altRC, iter.getName(),
                                Math.toDegrees(altDelta));
                    }
                }

                String verdict;
                if (Double.isInfinite(minAltDelta)) {
                    verdict = "no alts at junction (edge case)";
                } else if (minAltDelta <= 30.0) {
                    verdict = "WOULD FIRE under proposed gate (continuation present)";
                } else {
                    verdict = "WOULD NOT FIRE under proposed gate (real T-end)";
                }

                System.out.printf("  M1 #%d at junction node=%d (%.6f, %.6f)%n",
                        m1Count, junctionNode, jLat, jLon);
                System.out.printf("    turn_angle_deg=%.1f join_direction=%s join_target_type=%s prev_PH=%s currPH=%s%n",
                        turnAngle, instr.getExtraInfoJSON().get("join_direction"),
                        instr.getExtraInfoJSON().get("join_target_type"),
                        instr.getExtraInfoJSON().get("prev_predicted_highway"),
                        instr.getExtraInfoJSON().get("predicted_highway"));
                System.out.printf("    min |altDelta| = %.1f°  (straightest alt: %s)%n",
                        minAltDelta, straightestAltDesc);
                System.out.println("    => " + verdict);
            }
            if (m1Count == 0) {
                System.out.println("  (no M1-fired instructions in this route)");
            }
        }
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
        postProcessor.process(result.instructions, instrRequest.getInstructionProfile());
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
            postProcessor.process(result.instructions, request.getInstructionProfile());
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
        postProcessor.process(result.instructions, request.getInstructionProfile());

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
        postProcessor.process(result.instructions, request.getInstructionProfile());

        System.out.println("\n========== CYCLEWAY Y-JUNCTION INFORMAL NAMES — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);
    }

    /**
     * Diagnostic: short cycleway route in Tampere where only one turn instruction is
     * outputted, but the user reports there is another cycleway junction shortly before
     * the turn where an alt cycleway leaves at roughly 60°. Either an instruction or
     * a visual-guidance side-channel candidate is expected.
     *
     * API payload (verbatim):
     *   waypoints:
     *     - id=ANdR37cK4kPTn1N4_-XO4 (61.511014, 23.611499)
     *     - id=UQiiR4E-s5mlQtAQBi4O8 (61.511865, 23.612196)
     *   segments: 1 followRoads, profile=gravel
     *   instruction_profile=gravel, locale=fi, snap_preventions=[ferry]
     */
    @Test
    void testCyclewayMissingPreTurnJunction_61_511_23_611() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        // ----- Build request exactly matching API payload -----
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("ANdR37cK4kPTn1N4_-XO4", 61.511014, 23.611499);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("UQiiR4E-s5mlQtAQBi4O8", 61.511865, 23.612196);
        request.setWaypoints(List.of(wp1, wp2));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("ANdR37cK4kPTn1N4_-XO4");
        seg.setEnd("UQiiR4E-s5mlQtAQBi4O8");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        // ----- Stage 1: generate -----
        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== MISSING PRE-TURN CYCLEWAY JUNCTION — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        // ----- Polyline dump (so the geometry is locatable) -----
        System.out.println("\n--- Polyline (lat, lng) ---");
        for (int i = 0; i < result.polyline.size(); i++) {
            System.out.printf("  [%d] %.7f, %.7f%n", i, result.polyline.getLat(i), result.polyline.getLon(i));
        }

        // ----- Edge chain via standard GH (same gravel profile) -----
        EnumEncodedValue<RoadClass> rcEnc = encodingManager.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<PredictedHighway> phEnc = encodingManager.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> psEnc = encodingManager.hasEncodedValue(PredictedSurface.KEY)
                ? encodingManager.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class) : null;
        BooleanEncodedValue bikeAccessEnc = encodingManager.getBooleanEncodedValue(VehicleAccess.key("bike"));

        GHRequest ghReq = new GHRequest(61.511014, 23.611499, 61.511865, 23.612196).setProfile("gravel");
        ghReq.setPathDetails(List.of("edge_id"));
        ghReq.putHint("instructions", false);
        ghReq.putHint("calc_points", true);
        GHResponse ghRsp = hopper.route(ghReq);
        assertFalse(ghRsp.hasErrors(), "GH route failed: " + ghRsp.getErrors());

        List<PathDetail> edgeIdDetails = ghRsp.getBest().getPathDetails().get("edge_id");
        assertNotNull(edgeIdDetails);
        System.out.println("\n--- Edge chain (" + edgeIdDetails.size() + " edges) ---");

        // Determine starting node (mirror of determineStartNode helper but inline + null-safe)
        EdgeExplorer explorer = baseGraph.createEdgeExplorer();
        int prevNode;
        {
            EdgeIteratorState e0 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(0).getValue(), Integer.MIN_VALUE);
            if (edgeIdDetails.size() == 1) {
                prevNode = e0.getBaseNode();
            } else {
                EdgeIteratorState e1 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(1).getValue(), Integer.MIN_VALUE);
                int b1 = e0.getBaseNode(), a1 = e0.getAdjNode();
                int b2 = e1.getBaseNode(), a2 = e1.getAdjNode();
                int shared;
                if (b1 == b2 || b1 == a2) shared = b1;
                else shared = a1;
                prevNode = (e0.getBaseNode() == shared) ? e0.getAdjNode() : e0.getBaseNode();
            }
        }

        // Walk edges in the actual route direction and analyze each junction
        for (int ei = 0; ei < edgeIdDetails.size(); ei++) {
            int edgeId = (Integer) edgeIdDetails.get(ei).getValue();
            EdgeIteratorState raw = baseGraph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            int nextNode = (raw.getBaseNode() == prevNode) ? raw.getAdjNode() : raw.getBaseNode();
            // Orient base=prevNode, adj=nextNode (the API returns the edge oriented so getAdjNode() == 2nd arg)
            EdgeIteratorState routeEdge = baseGraph.getEdgeIteratorState(edgeId, nextNode);
            int adjNode = routeEdge.getAdjNode();

            RoadClass rc = routeEdge.get(rcEnc);
            PredictedHighway ph = routeEdge.get(phEnc);
            PredictedSurface ps = psEnc != null ? routeEdge.get(psEnc) : null;
            String name = routeEdge.getName();

            // Geometry endpoints (with tower nodes)
            PointList geo = routeEdge.fetchWayGeometry(FetchMode.ALL);
            double startLat = geo.getLat(0), startLon = geo.getLon(0);
            double endLat = geo.getLat(geo.size() - 1), endLon = geo.getLon(geo.size() - 1);

            System.out.printf("%n[edge %d] id=%d base=%d→adj=%d  rc=%s  ph=%s  ps=%s  name=\"%s\"  len=%.1fm%n",
                    ei, edgeId, prevNode, adjNode, rc, ph, ps, name, routeEdge.getDistance());
            System.out.printf("        start=(%.7f, %.7f)  end=(%.7f, %.7f)%n",
                    startLat, startLon, endLat, endLon);

            // Compute incoming bearing into adjNode (last segment of this edge's geometry)
            int gN = geo.size();
            double inFromLat = geo.getLat(gN - 2), inFromLon = geo.getLon(gN - 2);
            double inToLat = geo.getLat(gN - 1), inToLon = geo.getLon(gN - 1);
            double incomingBearing = AngleCalc.ANGLE_CALC.calcOrientation(inFromLat, inFromLon, inToLat, inToLon);

            // Enumerate alternatives at adjNode, excluding the edge we came in on
            int routeNextEdgeId = (ei + 1 < edgeIdDetails.size()) ? (Integer) edgeIdDetails.get(ei + 1).getValue() : -1;
            EdgeIterator iter = explorer.setBaseNode(adjNode);
            int altCount = 0;
            List<String> altLines = new ArrayList<>();
            while (iter.next()) {
                if (iter.getEdge() == edgeId) continue; // came in on this edge
                altCount++;
                int altEdgeId = iter.getEdge();
                int altAdj = iter.getAdjNode();
                RoadClass altRC = iter.get(rcEnc);
                PredictedHighway altPH = iter.get(phEnc);
                PredictedSurface altPS = psEnc != null ? iter.get(psEnc) : null;
                String altName = iter.getName();
                boolean altAccess = iter.get(bikeAccessEnc);

                // Alt's outgoing bearing (start of its way geometry, leaving adjNode)
                PointList altGeo = iter.fetchWayGeometry(FetchMode.ALL);
                double altFromLat = altGeo.getLat(0), altFromLon = altGeo.getLon(0);
                double altToLat = altGeo.getLat(1), altToLon = altGeo.getLon(1);
                double altBearing = AngleCalc.ANGLE_CALC.calcOrientation(altFromLat, altFromLon, altToLat, altToLon);

                // Delta from incoming direction (positive = right turn)
                double aligned = AngleCalc.ANGLE_CALC.alignOrientation(incomingBearing, altBearing);
                double deltaRad = aligned - incomingBearing;
                double deltaDeg = Math.toDegrees(deltaRad);

                String tag = (altEdgeId == routeNextEdgeId) ? "ROUTE" : "alt  ";
                altLines.add(String.format(
                        "        %s edge=%d adj=%d  rc=%s ph=%s ps=%s access=%s  Δ=%+.1f°  name=\"%s\"",
                        tag, altEdgeId, altAdj, altRC, altPH, altPS, altAccess, deltaDeg, altName));
            }
            if (altCount > 0) {
                System.out.printf("    junction at node %d — %d outgoing edge(s) (excluding incoming):%n", adjNode, altCount);
                for (String s : altLines) System.out.println(s);
            } else {
                System.out.printf("    junction at node %d — no alternatives (dead-end / 2-degree node)%n", adjNode);
            }

            prevNode = adjNode;
        }

        // ----- Stage 2: post-process and re-print -----
        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        System.out.println("\n========== MISSING PRE-TURN CYCLEWAY JUNCTION — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);
    }

    /**
     * Diagnostic: cycleway-vs-service-road and service-road-vs-residential junctions
     * where no instructions are emitted but the cyclist sees ambiguous fork choices.
     *
     * API payload (verbatim):
     *   waypoints:
     *     - id=t6gGSoLjQsB4WmBZKNNCk (61.530601, 23.702663)
     *     - id=otfJ6UVVeGcQTjcpSn6xp (61.528947, 23.697955)
     *   segments: 1 followRoads, profile=gravel
     *   instruction_profile=gravel, locale=fi, snap_preventions=[ferry]
     *
     * Reported issues:
     *   1) Cycleway (unpaved, missing surface tag) → service road look-alike: no instruction.
     *   2) Unpaved service road → fork with asphalt residential road + asphalt-but-surfaceless
     *      service road: rider expected to "pysy vasen" with no instruction.
     */
    @Test
    void testCyclewayServiceRoadIndistinguishable_61_530_23_702() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("t6gGSoLjQsB4WmBZKNNCk", 61.530601, 23.702663);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("otfJ6UVVeGcQTjcpSn6xp", 61.528947, 23.697955);
        request.setWaypoints(List.of(wp1, wp2));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("t6gGSoLjQsB4WmBZKNNCk");
        seg.setEnd("otfJ6UVVeGcQTjcpSn6xp");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== CYCLEWAY/SERVICE-ROAD AMBIGUITY — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        System.out.println("\n--- Polyline (lat, lng) ---");
        for (int i = 0; i < result.polyline.size(); i++) {
            System.out.printf("  [%d] %.7f, %.7f%n", i, result.polyline.getLat(i), result.polyline.getLon(i));
        }

        EnumEncodedValue<RoadClass> rcEnc = encodingManager.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<PredictedHighway> phEnc = encodingManager.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> psEnc = encodingManager.hasEncodedValue(PredictedSurface.KEY)
                ? encodingManager.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class) : null;
        EnumEncodedValue<Surface> surfEnc = encodingManager.hasEncodedValue(Surface.KEY)
                ? encodingManager.getEnumEncodedValue(Surface.KEY, Surface.class) : null;
        BooleanEncodedValue bikeAccessEnc = encodingManager.getBooleanEncodedValue(VehicleAccess.key("bike"));

        GHRequest ghReq = new GHRequest(61.530601, 23.702663, 61.528947, 23.697955).setProfile("gravel");
        ghReq.setPathDetails(List.of("edge_id"));
        ghReq.putHint("instructions", false);
        ghReq.putHint("calc_points", true);
        GHResponse ghRsp = hopper.route(ghReq);
        assertFalse(ghRsp.hasErrors(), "GH route failed: " + ghRsp.getErrors());

        List<PathDetail> edgeIdDetails = ghRsp.getBest().getPathDetails().get("edge_id");
        assertNotNull(edgeIdDetails);
        System.out.println("\n--- Edge chain (" + edgeIdDetails.size() + " edges) ---");

        EdgeExplorer explorer = baseGraph.createEdgeExplorer();
        int prevNode;
        {
            EdgeIteratorState e0 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(0).getValue(), Integer.MIN_VALUE);
            if (edgeIdDetails.size() == 1) {
                prevNode = e0.getBaseNode();
            } else {
                EdgeIteratorState e1 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(1).getValue(), Integer.MIN_VALUE);
                int b1 = e0.getBaseNode(), a1 = e0.getAdjNode();
                int b2 = e1.getBaseNode(), a2 = e1.getAdjNode();
                int shared;
                if (b1 == b2 || b1 == a2) shared = b1;
                else shared = a1;
                prevNode = (e0.getBaseNode() == shared) ? e0.getAdjNode() : e0.getBaseNode();
            }
        }

        for (int ei = 0; ei < edgeIdDetails.size(); ei++) {
            int edgeId = (Integer) edgeIdDetails.get(ei).getValue();
            EdgeIteratorState raw = baseGraph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            int nextNode = (raw.getBaseNode() == prevNode) ? raw.getAdjNode() : raw.getBaseNode();
            EdgeIteratorState routeEdge = baseGraph.getEdgeIteratorState(edgeId, nextNode);
            int adjNode = routeEdge.getAdjNode();

            RoadClass rc = routeEdge.get(rcEnc);
            PredictedHighway ph = routeEdge.get(phEnc);
            PredictedSurface ps = psEnc != null ? routeEdge.get(psEnc) : null;
            Surface surf = surfEnc != null ? routeEdge.get(surfEnc) : null;
            String name = routeEdge.getName();

            PointList geo = routeEdge.fetchWayGeometry(FetchMode.ALL);
            double startLat = geo.getLat(0), startLon = geo.getLon(0);
            double endLat = geo.getLat(geo.size() - 1), endLon = geo.getLon(geo.size() - 1);

            System.out.printf("%n[edge %d] id=%d base=%d→adj=%d  rc=%s  ph=%s  ps=%s  surf=%s  name=\"%s\"  len=%.1fm%n",
                    ei, edgeId, prevNode, adjNode, rc, ph, ps, surf, name, routeEdge.getDistance());
            System.out.printf("        start=(%.7f, %.7f)  end=(%.7f, %.7f)%n",
                    startLat, startLon, endLat, endLon);

            int gN = geo.size();
            double inFromLat = geo.getLat(gN - 2), inFromLon = geo.getLon(gN - 2);
            double inToLat = geo.getLat(gN - 1), inToLon = geo.getLon(gN - 1);
            double incomingBearing = AngleCalc.ANGLE_CALC.calcOrientation(inFromLat, inFromLon, inToLat, inToLon);

            int routeNextEdgeId = (ei + 1 < edgeIdDetails.size()) ? (Integer) edgeIdDetails.get(ei + 1).getValue() : -1;
            EdgeIterator iter = explorer.setBaseNode(adjNode);
            int altCount = 0;
            List<String> altLines = new ArrayList<>();
            while (iter.next()) {
                if (iter.getEdge() == edgeId) continue;
                altCount++;
                int altEdgeId = iter.getEdge();
                int altAdj = iter.getAdjNode();
                RoadClass altRC = iter.get(rcEnc);
                PredictedHighway altPH = iter.get(phEnc);
                PredictedSurface altPS = psEnc != null ? iter.get(psEnc) : null;
                Surface altSurf = surfEnc != null ? iter.get(surfEnc) : null;
                String altName = iter.getName();
                boolean altAccess = iter.get(bikeAccessEnc);

                PointList altGeo = iter.fetchWayGeometry(FetchMode.ALL);
                double altFromLat = altGeo.getLat(0), altFromLon = altGeo.getLon(0);
                double altToLat = altGeo.getLat(1), altToLon = altGeo.getLon(1);
                double altBearing = AngleCalc.ANGLE_CALC.calcOrientation(altFromLat, altFromLon, altToLat, altToLon);

                double aligned = AngleCalc.ANGLE_CALC.alignOrientation(incomingBearing, altBearing);
                double deltaRad = aligned - incomingBearing;
                double deltaDeg = Math.toDegrees(deltaRad);

                String tag = (altEdgeId == routeNextEdgeId) ? "ROUTE" : "alt  ";
                altLines.add(String.format(
                        "        %s edge=%d adj=%d  rc=%s ph=%s ps=%s surf=%s access=%s  Δ=%+.1f°  name=\"%s\"",
                        tag, altEdgeId, altAdj, altRC, altPH, altPS, altSurf, altAccess, deltaDeg, altName));
            }
            if (altCount > 0) {
                System.out.printf("    junction at node %d — %d outgoing edge(s) (excluding incoming):%n", adjNode, altCount);
                for (String s : altLines) System.out.println(s);
            } else {
                System.out.printf("    junction at node %d — no alternatives (dead-end / 2-degree node)%n", adjNode);
            }

            prevNode = adjNode;
        }

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        System.out.println("\n========== CYCLEWAY/SERVICE-ROAD AMBIGUITY — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);
    }

    /**
     * Diagnostic for the road → cycleway transition where the road ends at a
     * vehicle turnaround / cul-de-sac and a cycleway picks up. The user
     * reports that no instruction is emitted, but on the ground the rider may
     * be uncertain because the road-end visually says "stop" rather than
     * "continue onto cycleway".
     *
     * API payload (forward):
     *   waypoints:
     *     - id=hvH1f2dBl3s_0xB2EHrXx (61.500518, 23.655585)   ← presumed road
     *     - id=l-hO7UTD9iVIbMFSbNObO (61.500109, 23.655586)   ← presumed cycleway
     *   segments: 1 followRoads, profile=gravel
     *   instruction_profile=gravel, locale=fi, snap_preventions=[ferry]
     *
     * Expected (current behavior): the road→cycleway transition is
     * suppressed by the forced-path branch in {@code getTurn()}
     * ({@code nrOfPossibleTurns <= 1 → IGNORE}). This test prints the
     * generator output AND walks the routed edge sequence with a per-junction
     * dump so we can confirm which junction has only one allowed continuation
     * and inspect the data we'd have to work with for a future rule change
     * that emits a hint at road→non-road transitions.
     *
     * Also runs the reverse direction (cycleway → road) as the secondary
     * case the user mentioned. No assertions beyond "didn't crash".
     */
    @Test
    void testRoadEndCyclewayTransition_61_500_23_655() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        // ---- FORWARD: road → cycleway (the API payload) ----
        TrailmapInstructionRequest fwdReq = new TrailmapInstructionRequest();
        TrailmapInstructionRequest.Waypoint fwdWp1 = makeWaypoint("hvH1f2dBl3s_0xB2EHrXx", 61.500518, 23.655585);
        TrailmapInstructionRequest.Waypoint fwdWp2 = makeWaypoint("l-hO7UTD9iVIbMFSbNObO", 61.500109, 23.655586);
        fwdReq.setWaypoints(List.of(fwdWp1, fwdWp2));

        TrailmapInstructionRequest.Segment fwdSeg = new TrailmapInstructionRequest.Segment();
        fwdSeg.setStart("hvH1f2dBl3s_0xB2EHrXx");
        fwdSeg.setEnd("l-hO7UTD9iVIbMFSbNObO");
        fwdSeg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        fwdSeg.setProfile("gravel");

        fwdReq.setSegments(List.of(fwdSeg));
        fwdReq.setInstructionProfile("gravel");
        fwdReq.setLocale("fi");
        fwdReq.setSnapPreventions(List.of("ferry"));

        analyzeRoadNonRoadTransition("FORWARD road→cycleway", generator, fwdReq,
                "gravel",
                61.500518, 23.655585, 61.500109, 23.655586);

        // ---- REVERSE: cycleway → road (secondary case) ----
        TrailmapInstructionRequest revReq = new TrailmapInstructionRequest();
        TrailmapInstructionRequest.Waypoint revWp1 = makeWaypoint("rev_start", 61.500109, 23.655586);
        TrailmapInstructionRequest.Waypoint revWp2 = makeWaypoint("rev_end",   61.500518, 23.655585);
        revReq.setWaypoints(List.of(revWp1, revWp2));

        TrailmapInstructionRequest.Segment revSeg = new TrailmapInstructionRequest.Segment();
        revSeg.setStart("rev_start");
        revSeg.setEnd("rev_end");
        revSeg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        revSeg.setProfile("gravel");

        revReq.setSegments(List.of(revSeg));
        revReq.setInstructionProfile("gravel");
        revReq.setLocale("fi");
        revReq.setSnapPreventions(List.of("ferry"));

        analyzeRoadNonRoadTransition("REVERSE cycleway→road", generator, revReq,
                "gravel",
                61.500109, 23.655586, 61.500518, 23.655585);
    }

    /**
     * Diagnostic for a "zigzag straight-through" case: two consecutive turns whose
     * connecting segment is very short (<5 m) and whose net direction change is
     * near-zero. M2 currently merges them into "loiva oikea, sitten heti vasen",
     * but on the ground the rider experiences a near-straight cycleway with a
     * minor S-bend. Reality should read closer to "suoraan" (or no instruction).
     *
     * API payload (verbatim):
     *   waypoints:
     *     - id=rhjZmjPbTweIoZcvbUs-t (61.511523, 23.687208)
     *     - id=uOftI2IKTnT33Hpld0m_D (61.511789, 23.687448)
     *   segments: 1 followRoads, profile=gravel
     *   instruction_profile=gravel, locale=fi, snap_preventions=[ferry]
     */
    @Test
    void testShortZigzagAsStraight_61_511_23_687() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest req = new TrailmapInstructionRequest();
        req.setWaypoints(List.of(
                makeWaypoint("rhjZmjPbTweIoZcvbUs-t", 61.511523, 23.687208),
                makeWaypoint("uOftI2IKTnT33Hpld0m_D", 61.511789, 23.687448)));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("rhjZmjPbTweIoZcvbUs-t");
        seg.setEnd("uOftI2IKTnT33Hpld0m_D");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        req.setSegments(List.of(seg));
        req.setInstructionProfile("gravel");
        req.setLocale("fi");
        req.setSnapPreventions(List.of("ferry"));

        analyzeRoadNonRoadTransition("ZIGZAG short connector", generator, req,
                "gravel",
                61.511523, 23.687208, 61.511789, 23.687448);
    }

    /**
     * Diagnostic for a "wrong-pair M2 merge" case: three turns on a cycleway
     * where the rider perceives the close pair as turns 2+3, but M2 greedily
     * merges turns 1+2 (because it advances past 2 after merging) and leaves
     * turn 3 standalone. Question: do the actual connector distances support
     * the rider's grouping, and is M2's greedy ordering producing a suboptimal
     * pairing?
     *
     * API payload (verbatim):
     *   waypoints:
     *     - id=MWWB_hZK5KMowJtj-8ukR (61.508160, 23.687510)
     *     - id=Ks6yCK3T3mqTw6AN83mwh (61.508697, 23.687776)
     *   segments: 1 followRoads, profile=gravel
     *   instruction_profile=gravel, locale=fi, snap_preventions=[ferry]
     */
    @Test
    void testThreeTurnGreedyMergeOrdering_61_508_23_687() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest req = new TrailmapInstructionRequest();
        req.setWaypoints(List.of(
                makeWaypoint("MWWB_hZK5KMowJtj-8ukR", 61.508160, 23.687510),
                makeWaypoint("Ks6yCK3T3mqTw6AN83mwh", 61.508697, 23.687776)));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("MWWB_hZK5KMowJtj-8ukR");
        seg.setEnd("Ks6yCK3T3mqTw6AN83mwh");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        req.setSegments(List.of(seg));
        req.setInstructionProfile("gravel");
        req.setLocale("fi");
        req.setSnapPreventions(List.of("ferry"));

        analyzeRoadNonRoadTransition("THREE-TURN greedy pairing", generator, req,
                "gravel",
                61.508160, 23.687510, 61.508697, 23.687776);
    }

    /**
     * For one routed direction: print Stage-1 instruction output, then walk
     * the GH edge sequence and dump per-junction context (incident edges,
     * computed allowed/visible turn counts under the bike weighting, and
     * whether the forced-path suppression branch in
     * {@link TrailmapInstructionsFromEdges#getTurn} would fire). Used to
     * understand why no instruction is emitted at road→non-road transitions
     * (and the reverse).
     */
    private void analyzeRoadNonRoadTransition(String label,
                                              RouteInstructionGenerator generator,
                                              TrailmapInstructionRequest request,
                                              String routingProfile,
                                              double startLat, double startLng,
                                              double endLat, double endLng) {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager em = hopper.getEncodingManager();

        System.out.println("\n========== " + label + " — STAGE 1 OUTPUT (pre post-processing) ==========");
        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result, "Generator returned null for " + label);
        assertNotNull(result.instructions);
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        // Standard GH route to extract the edge sequence (generator does not expose edge IDs).
        GHRequest ghReq = new GHRequest(startLat, startLng, endLat, endLng).setProfile(routingProfile);
        ghReq.setPathDetails(List.of("edge_id"));
        ghReq.putHint("instructions", false);
        ghReq.putHint("calc_points", true);
        ghReq.setSnapPreventions(List.of("ferry"));
        GHResponse ghRsp = hopper.route(ghReq);
        assertFalse(ghRsp.hasErrors(), label + " — GH route failed: " + ghRsp.getErrors());

        List<PathDetail> edgeIdDetails = ghRsp.getBest().getPathDetails().get("edge_id");
        assertNotNull(edgeIdDetails, label + " — no edge_id details");
        assertFalse(edgeIdDetails.isEmpty(), label + " — empty edge chain");

        // Resolve encoded values used for the dump.
        EnumEncodedValue<RoadClass> rcEnc = em.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<PredictedHighway> phEnc = em.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> psEnc = em.hasEncodedValue(PredictedSurface.KEY)
                ? em.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class) : null;
        EnumEncodedValue<Surface> surfEnc = em.hasEncodedValue(Surface.KEY)
                ? em.getEnumEncodedValue(Surface.KEY, Surface.class) : null;
        BooleanEncodedValue bikeAccessEnc = em.getBooleanEncodedValue(VehicleAccess.key("bike"));

        // Use the actual gravel weighting to mirror what InstructionsOutgoingEdges sees:
        // an alternative is "allowed" iff calcEdgeWeight(edge, false) is finite when the
        // edge iterator is rooted at the via node (so reverse=false ≡ outgoing direction).
        Profile profile = hopper.getProfile(routingProfile);
        assertNotNull(profile, label + " — profile not found: " + routingProfile);
        Weighting weighting = createWeightingWithCm(profile, null);

        EdgeExplorer explorer = baseGraph.createEdgeExplorer();

        System.out.println("\n--- " + label + " — Edge chain (" + edgeIdDetails.size()
                + " edges) with junction context ---");

        // Determine the actual route start node by reading how edge[0] connects to edge[1].
        int prevNode;
        {
            EdgeIteratorState e0 = baseGraph.getEdgeIteratorState(
                    (Integer) edgeIdDetails.get(0).getValue(), Integer.MIN_VALUE);
            if (edgeIdDetails.size() == 1) {
                prevNode = e0.getBaseNode();
            } else {
                EdgeIteratorState e1 = baseGraph.getEdgeIteratorState(
                        (Integer) edgeIdDetails.get(1).getValue(), Integer.MIN_VALUE);
                int b1 = e0.getBaseNode(), a1 = e0.getAdjNode();
                int b2 = e1.getBaseNode(), a2 = e1.getAdjNode();
                int shared = (b1 == b2 || b1 == a2) ? b1 : a1;
                prevNode = (e0.getBaseNode() == shared) ? e0.getAdjNode() : e0.getBaseNode();
            }
        }

        for (int ei = 0; ei < edgeIdDetails.size(); ei++) {
            int edgeId = (Integer) edgeIdDetails.get(ei).getValue();
            EdgeIteratorState raw = baseGraph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            int nextNode = (raw.getBaseNode() == prevNode) ? raw.getAdjNode() : raw.getBaseNode();
            EdgeIteratorState routeEdge = baseGraph.getEdgeIteratorState(edgeId, nextNode);
            int adjNode = routeEdge.getAdjNode();

            RoadClass rc = routeEdge.get(rcEnc);
            PredictedHighway ph = routeEdge.get(phEnc);
            PredictedSurface ps = psEnc != null ? routeEdge.get(psEnc) : null;
            Surface surf = surfEnc != null ? routeEdge.get(surfEnc) : null;
            String name = routeEdge.getName();

            PointList geo = routeEdge.fetchWayGeometry(FetchMode.ALL);
            double startGLat = geo.getLat(0), startGLon = geo.getLon(0);
            double endGLat = geo.getLat(geo.size() - 1), endGLon = geo.getLon(geo.size() - 1);

            System.out.printf("%n[edge %d] id=%d base=%d→adj=%d  rc=%s  ph=%s  ps=%s  surf=%s  name=\"%s\"  len=%.1fm%n",
                    ei, edgeId, prevNode, adjNode, rc, ph, ps, surf, name, routeEdge.getDistance());
            System.out.printf("        start=(%.7f, %.7f)  end=(%.7f, %.7f)%n",
                    startGLat, startGLon, endGLat, endGLon);

            // Incoming bearing into the adj (via) node — last segment of the route edge geometry.
            int gN = geo.size();
            double inFromLat = geo.getLat(gN - 2), inFromLon = geo.getLon(gN - 2);
            double inToLat   = geo.getLat(gN - 1), inToLon   = geo.getLon(gN - 1);
            double incomingBearing = AngleCalc.ANGLE_CALC.calcOrientation(inFromLat, inFromLon, inToLat, inToLon);

            int routeNextEdgeId = (ei + 1 < edgeIdDetails.size())
                    ? (Integer) edgeIdDetails.get(ei + 1).getValue() : -1;

            // Walk every graph-incident edge at the via node. Mirrors what the Stage 1 fork
            // does via allExplorer in InstructionsOutgoingEdges — including the prev edge,
            // which we tag separately so it is never counted as an alternative.
            EdgeIterator iter = explorer.setBaseNode(adjNode);
            int allowedAlts = 0;       // matches InstructionsOutgoingEdges.allowedAlternativeTurns.size()
            int visibleAlts = 0;       // matches InstructionsOutgoingEdges.visibleAlternativeTurns.size()
            List<String> altLines = new ArrayList<>();
            String routeNextSummary = null;
            while (iter.next()) {
                int altEdgeId = iter.getEdge();
                int altAdj = iter.getAdjNode();
                boolean isPrev = (altEdgeId == edgeId);             // the route edge we just traversed
                boolean isRouteNext = (altEdgeId == routeNextEdgeId);

                RoadClass altRC = iter.get(rcEnc);
                PredictedHighway altPH = iter.get(phEnc);
                PredictedSurface altPS = psEnc != null ? iter.get(psEnc) : null;
                Surface altSurf = surfEnc != null ? iter.get(surfEnc) : null;
                String altName = iter.getName();
                boolean altBikeAccessTag = iter.get(bikeAccessEnc);

                // Outgoing bearing from the via node along this incident edge.
                PointList altGeo = iter.fetchWayGeometry(FetchMode.ALL);
                double altFromLat = altGeo.getLat(0), altFromLon = altGeo.getLon(0);
                double altToLat   = altGeo.getLat(1), altToLon   = altGeo.getLon(1);
                double altBearing = AngleCalc.ANGLE_CALC.calcOrientation(altFromLat, altFromLon, altToLat, altToLon);
                double aligned = AngleCalc.ANGLE_CALC.alignOrientation(incomingBearing, altBearing);
                double deltaDeg = Math.toDegrees(aligned - incomingBearing);

                // Match InstructionsOutgoingEdges accessibility check exactly:
                //   allowed:  Double.isFinite(weighting.calcEdgeWeight(edge, false))
                //   visible:  allowed OR Double.isFinite(weighting.calcEdgeWeight(edge, true))
                double fwdW = weighting(weighting, iter, false);
                double bwdW = weighting(weighting, iter, true);
                boolean fwdFinite = Double.isFinite(fwdW);
                boolean bwdFinite = Double.isFinite(bwdW);

                // Tag and accounting. Stage 1 excludes (a) the prev edge and (b) the route's
                // next edge from getAllowedAlternativeTurns / getVisibleAlternativeTurns.
                String tag;
                if (isPrev) tag = "PREV ";
                else if (isRouteNext) tag = "ROUTE";
                else tag = "alt  ";

                String line = String.format(
                        "        %s edge=%d adj=%d  rc=%s ph=%s ps=%s surf=%s  bike_tag=%s fwdW=%s bwdW=%s  Δ=%+6.1f°  name=\"%s\"",
                        tag, altEdgeId, altAdj, altRC, altPH, altPS, altSurf,
                        altBikeAccessTag,
                        fwdFinite ? "ok" : "INF",
                        bwdFinite ? "ok" : "INF",
                        deltaDeg, altName);

                if (isRouteNext) {
                    routeNextSummary = line;
                } else if (!isPrev) {
                    altLines.add(line);
                    if (fwdFinite) allowedAlts++;
                    if (fwdFinite || bwdFinite) visibleAlts++;
                }
            }

            int allowedTurns = 1 + allowedAlts;   // matches getAllowedTurns() (1 = the route's next edge)
            int visibleTurns = 1 + visibleAlts;   // matches getVisibleTurns()
            boolean forcedPathBranch = allowedTurns <= 1;

            System.out.printf("    junction at node %d — incoming bearing=%.1f°  alt(non-route)=%d  allowed_turns=%d  visible_turns=%d%n",
                    adjNode, Math.toDegrees(incomingBearing), altLines.size(), allowedTurns, visibleTurns);
            if (routeNextSummary != null) System.out.println(routeNextSummary);
            for (String s : altLines) System.out.println(s);
            System.out.printf("    => Stage 1 forced-path branch (nrOfPossibleTurns<=1) %s%n",
                    forcedPathBranch ? "FIRES (suppresses unless |sign|>1 && visible_turns>1, or E5)" : "does NOT fire");

            prevNode = adjNode;
        }

        // Apply post-processing too, so we see the final state the client would receive.
        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());
        System.out.println("\n========== " + label + " — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);
    }

    // =========================================================================
    // Investigation: spurious "loiva oikea" / TURN_SLIGHT_RIGHT instructions on
    // gravel routes where there is either no real fork or no confusable
    // alternative. Four reported API payloads, plus the angle-threshold
    // question (case D). Each test runs the same diagnostic walk so the four
    // junctions can be compared side-by-side.
    // =========================================================================

    /**
     * CASE A — single-segment gravel route around (61.5164, 23.6896) → (61.5170, 23.6904).
     * Reported: TURN_SLIGHT_RIGHT emitted but there is no real confusable alt.
     */
    @Test
    void testLoivaOikeaCaseA_61_516_23_689() {
        runSingleSegmentDiagnostic(
                "CASE A: TURN_SLIGHT_RIGHT without confusable alt",
                "YYC2p9mV_vF_7vF9wHiOJ", 61.516442, 23.68955,
                "HexkPdmGBeNOnBKWx4Ztx", 61.516993, 23.690446);
    }

    /**
     * CASE B — single-segment gravel route around (61.5193, 23.6979) → (61.5195, 23.6998).
     * Reported: similar spurious TURN_SLIGHT_RIGHT.
     */
    @Test
    void testLoivaOikeaCaseB_61_519_23_697() {
        runSingleSegmentDiagnostic(
                "CASE B: TURN_SLIGHT_RIGHT similar unnecessary",
                "SIW9bPSooVQD68SLKIkKa", 61.519257, 23.697865,
                "HG6cnTmWgzlm-kuCYIUGq", 61.519525, 23.699756);
    }

    /**
     * CASE C — single-segment gravel route around (61.5254, 23.7105) → (61.5252, 23.7115).
     * Reported: alt angle &lt; 90° so an instruction is defensible, but the sign came out
     * as TURN_RIGHT where TURN_SLIGHT_RIGHT seems closer to the geometry.
     */
    @Test
    void testRightVsSlightRightCaseC_61_525_23_710() {
        runSingleSegmentDiagnostic(
                "CASE C: TURN_RIGHT but feels like TURN_SLIGHT_RIGHT",
                "axiwYO0K70ULF5HIeuWbL", 61.525408, 23.71047,
                "KuRbVjrEfj3OUkGTJWC6B", 61.525187, 23.711459);
    }

    /**
     * CASE D — single-segment gravel route around (61.5307, 23.7188) → (61.5306, 23.7178).
     * Reported: TURN_SLIGHT_RIGHT could also be straight; calibration question on the
     * 11.5° / 0.2 rad CONTINUE_ON_STREET threshold.
     */
    @Test
    void testLoivaOikeaOrStraightCaseD_61_530_23_718() {
        runSingleSegmentDiagnostic(
                "CASE D: TURN_SLIGHT_RIGHT or straight (threshold case)",
                "21qqwiEJroNeuReTyK1Cd", 61.530728, 23.718766,
                "9uwP9IsbieTIiIg7alJjQ", 61.530604, 23.717849);
    }

    /**
     * Diagnostic for spurious M2 then_turn ("sitten 20m vasen") on a Ponsantie
     * approach — first instruction "turn right onto Ponsantie" is correct, but
     * the merged compound claims a follow-up left turn that does not exist on
     * the route.
     *
     * API payload (verbatim):
     *   waypoints:
     *     - id=sQ955_67-dAs-E9nVyHbp (61.452637, 24.116549)
     *     - id=fiYZXAN6O6pQVuQCkJ85A (61.451964, 24.116690)
     *   segments: 1 followRoads, profile=gravel
     *   instruction_profile=gravel, locale=fi, snap_preventions=[ferry]
     */
    @Test
    void testSpuriousThenLeftPonsantie_61_452_24_116() {
        runSingleSegmentDiagnostic(
                "SPURIOUS then-left on Ponsantie approach",
                "sQ955_67-dAs-E9nVyHbp", 61.452637, 24.116549,
                "fiYZXAN6O6pQVuQCkJ85A", 61.451964, 24.116690);
    }

    /**
     * Diagnostic: missing 2nd "right" instruction after a left turn at a
     * t-junction. Reported user mental model:
     *   1. Coming from unpaved cycleway to a t-junction → take left  (correct, emitted)
     *   2. ~15m later, asphalt cycleway forks: route turns ~90° right onto an
     *      unpaved cycleway, while an asphalt footway continues straight
     *      (visually a confusable alternative). The right turn is NOT emitted.
     *
     * API payload (verbatim):
     *   waypoints:
     *     - id=h5GZQDYcrqZAwP5p5Tr_D (61.470086, 23.865304)
     *     - id=QPU2Toex4WWPf7Qoccj7s (61.469662, 23.865102)
     *   segments: 1 followRoads, profile=gravel,
     *     custom_model={priority:[{if: predicted_highway == MAJOR_ROAD, multiply_by: 1.45}]}
     *   instruction_profile=gravel, locale=fi, snap_preventions=[ferry]
     */
    @Test
    void testMissingRightAtCyclewayFootwayFork_61_470_23_865() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager em = hopper.getEncodingManager();
        TranslationMap tm = hopper.getTranslationMap();
        RouteInstructionGenerator generator = new RouteInstructionGenerator(hopper, baseGraph, em, tm);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("h5GZQDYcrqZAwP5p5Tr_D", 61.470086, 23.865304),
                makeWaypoint("QPU2Toex4WWPf7Qoccj7s", 61.469662, 23.865102)));

        // Mirror the API payload's custom_model exactly (priority boost for MAJOR_ROAD).
        // For a short cycleway hop this should not change the chosen route, but we keep
        // it for fidelity with the reported payload.
        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_highway == MAJOR_ROAD",
                Statement.Op.MULTIPLY, "1.45"));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("h5GZQDYcrqZAwP5p5Tr_D");
        seg.setEnd("QPU2Toex4WWPf7Qoccj7s");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");
        seg.setCustomModel(cm);

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        analyzeRoadNonRoadTransition(
                "MISSING right at cycleway↑footway fork (asphalt cycleway → unpaved cycleway, footway continues straight)",
                generator, request, "gravel",
                61.470086, 23.865304, 61.469662, 23.865102);
    }

    /**
     * Diagnostic: 90° LEFT turn on cycleway with a same-surface (asphalt)
     * footway continuing near-straight at a single junction. Reported issue:
     * no instruction is emitted at all — not even {@code tbt_priority: "visual"}.
     *
     * Sibling case to {@link #testMissingRightAtCyclewayFootwayFork_61_470_23_865}
     * but reduced to a single junction so the rule chain can be studied in
     * isolation, and to confirm whether the same E5 surface-gate widening
     * would cover this scenario too.
     *
     * API payload (verbatim):
     *   waypoints:
     *     - id=DxKC_cfqphflJnHtQsH9L (61.482546, 23.749163)
     *     - id=lZ-H0lwAsW_NH7RB9xK_x (61.482827, 23.748476)
     *   segments: 1 followRoads, profile=gravel,
     *     custom_model={priority:[{if: predicted_highway == MAJOR_ROAD, multiply_by: 1.45}]}
     *   instruction_profile=gravel, locale=fi, snap_preventions=[ferry]
     */
    @Test
    void testMissingLeftAtCyclewayFootwayFork_61_482_23_749() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager em = hopper.getEncodingManager();
        TranslationMap tm = hopper.getTranslationMap();
        RouteInstructionGenerator generator = new RouteInstructionGenerator(hopper, baseGraph, em, tm);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("DxKC_cfqphflJnHtQsH9L", 61.482546, 23.749163),
                makeWaypoint("lZ-H0lwAsW_NH7RB9xK_x", 61.482827, 23.748476)));

        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_highway == MAJOR_ROAD",
                Statement.Op.MULTIPLY, "1.45"));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("DxKC_cfqphflJnHtQsH9L");
        seg.setEnd("lZ-H0lwAsW_NH7RB9xK_x");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");
        seg.setCustomModel(cm);

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        analyzeRoadNonRoadTransition(
                "MISSING left at cycleway↑footway fork (single junction, same-surface footway straight)",
                generator, request, "gravel",
                61.482546, 23.749163, 61.482827, 23.748476);
    }

    /**
     * Diagnostic: short 2-junction route where the 1st junction emits a fork
     * instruction correctly, but the 2nd junction — a cycleway/footway fork
     * with the route staying on the cycleway — produces no instruction at all.
     * User report: this 2nd junction should meet the latest "is confusable"
     * rules (E2 / E5) and produce some emission (real or visual).
     *
     * API payload (verbatim, no custom_model):
     *   waypoints:
     *     - id=481ICti0rqdScytZw-K34 (61.467159, 23.643481)
     *     - id=7rBc9VQQlTfoI_c7jTCGa (61.467333, 23.643604)
     *   segments: 1 followRoads, profile=gravel
     *   instruction_profile=gravel, locale=fi, snap_preventions=[ferry]
     */
    @Test
    void testMissingSecondJunctionCyclewayFootwayFork_61_467_23_643() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager em = hopper.getEncodingManager();
        TranslationMap tm = hopper.getTranslationMap();
        RouteInstructionGenerator generator = new RouteInstructionGenerator(hopper, baseGraph, em, tm);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("481ICti0rqdScytZw-K34", 61.467159, 23.643481),
                makeWaypoint("7rBc9VQQlTfoI_c7jTCGa", 61.467333, 23.643604)));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("481ICti0rqdScytZw-K34");
        seg.setEnd("7rBc9VQQlTfoI_c7jTCGa");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        analyzeRoadNonRoadTransition(
                "MISSING 2nd-junction emission at cycleway/footway fork (route on cycleway)",
                generator, request, "gravel",
                61.467159, 23.643481, 61.467333, 23.643604);
    }

    /**
     * Diagnostic: longer real-world route reported to generate too many instructions
     * after the case-1 (E5 surface gate) and case-2 (S3 non-road escape) fixes landed.
     * Goal: produce a per-junction dump so the user can mark which instructions are
     * extraneous, before diagnosing causes.
     *
     * API payload (verbatim, no custom_model):
     *   waypoints:
     *     - id=70kiMv-9Goi-btiF14m7K (61.477057, 24.027534)
     *     - id=AmTbWDt5hJMx5I7LGl1vy (61.471512, 24.050015)
     *   segments: 1 followRoads, profile=gravel
     *   instruction_profile=gravel, locale=fi, snap_preventions=[ferry]
     */
    @Test
    void testOverEmissionLongerRoute_61_477_24_027() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager em = hopper.getEncodingManager();
        TranslationMap tm = hopper.getTranslationMap();
        RouteInstructionGenerator generator = new RouteInstructionGenerator(hopper, baseGraph, em, tm);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("70kiMv-9Goi-btiF14m7K", 61.477057, 24.027534),
                makeWaypoint("AmTbWDt5hJMx5I7LGl1vy", 61.471512, 24.050015)));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("70kiMv-9Goi-btiF14m7K");
        seg.setEnd("AmTbWDt5hJMx5I7LGl1vy");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        analyzeRoadNonRoadTransition(
                "OVER-EMISSION longer route 61.477,24.027 → 61.471,24.050",
                generator, request, "gravel",
                61.477057, 24.027534, 61.471512, 24.050015);
    }

    /**
     * Diagnostic: short cycleway route with consecutive FOOTWAY branches, where a
     * "suoraan" / CONTINUE_ON_STREET is reported as emitted at a junction it
     * shouldn't fire on, after the case-2/3 fixes landed.
     *
     * API payload (verbatim):
     *   waypoints:
     *     - id=3-W07qUsDFJuu4vn3cRJK (61.493269, 23.758057)
     *     - id=Nr43g0zk-t2UArA-l-Fk4 (61.493533, 23.757662)
     *   segments: 1 followRoads, profile=gravel,
     *     custom_model={priority:[{if: predicted_highway == MAJOR_ROAD, multiply_by: 1.45}]}
     *   instruction_profile=gravel, locale=fi, snap_preventions=[ferry]
     */
    @Test
    void testSuoraanCyclewayFootwayConsecutive_61_493_23_758() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager em = hopper.getEncodingManager();
        TranslationMap tm = hopper.getTranslationMap();
        RouteInstructionGenerator generator = new RouteInstructionGenerator(hopper, baseGraph, em, tm);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("3-W07qUsDFJuu4vn3cRJK", 61.493269, 23.758057),
                makeWaypoint("Nr43g0zk-t2UArA-l-Fk4", 61.493533, 23.757662)));

        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_highway == MAJOR_ROAD",
                Statement.Op.MULTIPLY, "1.45"));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("3-W07qUsDFJuu4vn3cRJK");
        seg.setEnd("Nr43g0zk-t2UArA-l-Fk4");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");
        seg.setCustomModel(cm);

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        analyzeRoadNonRoadTransition(
                "SUORAAN extra at consecutive cycleway↔footway junctions (61.493,23.758)",
                generator, request, "gravel",
                61.493269, 23.758057, 61.493533, 23.757662);
    }

    /**
     * Build a single-segment followRoads gravel request and hand it to
     * {@link #analyzeRoadNonRoadTransition} so all four "loiva oikea"
     * cases use identical diagnostic output.
     */
    private void runSingleSegmentDiagnostic(String label,
                                            String wp1Id, double wp1Lat, double wp1Lng,
                                            String wp2Id, double wp2Lat, double wp2Lng) {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager em = hopper.getEncodingManager();
        TranslationMap tm = hopper.getTranslationMap();
        RouteInstructionGenerator generator = new RouteInstructionGenerator(hopper, baseGraph, em, tm);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint(wp1Id, wp1Lat, wp1Lng),
                makeWaypoint(wp2Id, wp2Lat, wp2Lng)));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart(wp1Id);
        seg.setEnd(wp2Id);
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        analyzeRoadNonRoadTransition(label, generator, request, "gravel",
                wp1Lat, wp1Lng, wp2Lat, wp2Lng);
    }

    /** Helper: get edge weight safely */
    private double weighting(Weighting w, EdgeIteratorState edge, boolean reverse) {
        try {
            return w.calcEdgeWeight(edge, reverse);
        } catch (Exception e) {
            return Double.NaN;
        }
    }

    // =========================================================================
    // Turn-cost impact analysis
    //
    // Each test method here represents one routing case. For each case we run
    // four routes — forward and reverse, each with profile=roadbike (turn costs
    // ON) and profile=roadbike_no_tc (turn costs OFF, identical priority/speed).
    // For the turn-costs-on routes we walk the resulting edge sequence and
    // call weighting.calcTurnWeight(prev, viaNode, curr) at each junction to
    // get the actual penalty applied during routing.
    //
    // Output is verbose (for diagnostic synthesis), no assertions beyond
    // "didn't crash". Re-running these methods after JSON tuning changes shows
    // both the magnitude shift on the same route AND any route-shape change
    // that the new tuning caused.
    //
    // Run a single case:
    //   mvn test -pl core -Dtest=RouteInstructionGeneratorTest#analyzeTurnCost_majorRoadDetour
    //   (from the graphhopper/ directory)
    // =========================================================================

    /**
     * Case: between two points on a major road. Forward routing currently
     * detours off the major (bordering on undesirable). Reverse routing stays
     * on the major (good — turn-cost crossing-traffic penalty for the
     * left-rejoin is doing the work).
     */
    @Test
    void analyzeTurnCost_majorRoadDetour() {
        runCase("majorRoadDetour", "roadbike", null,
                61.318733722910906, 24.2846477010672,
                61.297954,          24.307599);
    }

    /**
     * Case: Joensuu-area waypoints. With the current production rules the route
     * takes a major detour both ways. User dislikes the left-turn-side detour;
     * the right-turn-side is borderline.
     */
    @Test
    void analyzeTurnCost_majorDetourBothWays() {
        runCase("majorDetourBothWays", "roadbike", null,
                62.727963, 29.954837,
                62.742685, 29.992);
    }

    /**
     * Case: short hop across a big traffic-lights junction (Tampere area).
     * The dev roadbike (TC on) is reported to stay on the cycleway through
     * the junction. The pre-TC production roadbike is reported to make a
     * "stupid leap" onto the road in the middle of the junction. This test
     * routes both with TC on (current dev roadbike) and TC off
     * (roadbike_no_tc) so we can see the divergence and read out which
     * penalty groups are doing the work.
     */
    @Test
    void analyzeTurnCost_trafficLightsJunction() {
        runCase("trafficLightsJunction", "roadbike", null,
                61.518276,           23.645734,
                61.52000843033031,   23.64186532815708);
    }

    /**
     * Case: short hop where the gravel TC route reportedly takes a mostly-paved
     * cycleway, while the no-TC route takes mostly-unpaved cycleways. The
     * point of this test is to surface which turn penalties on the unpaved
     * route are flipping the choice toward the paved alternative.
     *
     * Requires gravel_no_tc to be present in trailmap-config.yml (mirror of
     * roadbike_no_tc — same priority/speed/elevation, no turn_costs).
     */
    @Test
    void analyzeTurnCost_gravelPavedDetour() {
        runCase("gravelPavedDetour", "gravel", "gravel_no_tc", null,
                61.53375,                  23.704257,
                61.528785086928025,        23.69653456253289);
    }

    /**
     * Case: gravel detours off a trunk road even when the user pushes a
     * +1.45× priority multiplier on MAJOR_ROAD via per-request custom model.
     * To diagnose, we route the same waypoints under several setups and
     * compare edge weights:
     *   1) gravel TC with the user's custom model (the actual reported case)
     *   2) gravel TC baseline (no custom model)
     *   3) gravel_no_tc with the user's custom model (isolates turn-cost role)
     *   4) roadbike (reference for the direct trunk-road path)
     */
    @Test
    void analyzeTurnCost_trunkRoadDetour() {
        final double aLat = 60.168683,           aLng = 23.959564;
        final double bLat = 60.166419286195634,  bLng = 23.95178806807928;

        String userRules = """
                {
                  "priority": [
                    { "if": "predicted_highway == MAJOR_ROAD", "multiply_by": "1.45" }
                  ],
                  "speed": []
                }
                """;
        CustomModel userCm = customModelFromJson(userRules);

        System.out.println("\n========== CASE: trunkRoadDetour ==========");

        System.out.println("\n##### (1) gravel TC + user custom_model (the reported case) #####");
        analyzeRoute("Forward A→B", aLat, aLng, bLat, bLng, "gravel", userCm);

        System.out.println("\n##### (2) gravel TC, no custom_model (baseline) #####");
        analyzeRoute("Forward A→B", aLat, aLng, bLat, bLng, "gravel", null);

        System.out.println("\n##### (3) gravel_no_tc + user custom_model (isolates TC role) #####");
        analyzeRoute("Forward A→B", aLat, aLng, bLat, bLng, "gravel_no_tc", userCm);

        System.out.println("\n##### (4) roadbike (reference — direct trunk-road route) #####");
        analyzeRoute("Forward A→B", aLat, aLng, bLat, bLng, "roadbike", null);
    }

    /**
     * Same waypoints as analyzeTurnCost_majorDetourBothWays, but routed via a
     * forced via-point on Lieksantie (the major road) — splitting into two
     * shorter legs whose individual lengths make the Jakokoskentie detour
     * unattractive. Used to MEASURE the direct route's weight rather than
     * infer it. Total weight = leg1 + leg2.
     */
    @Test
    void analyzeTurnCost_majorDetourBothWays_directViaWaypoint() {
        final double aLat = 62.727963, aLng = 29.954837;
        final double viaLat = 62.73433452009371, viaLng = 29.973117193452623;
        final double bLat = 62.742685, bLng = 29.992;

        System.out.println("\n========== CASE: majorDetourBothWays — direct route via via-point ==========");

        System.out.println("\n## FORWARD direct: A → via → B ##");
        RouteData fwd1 = analyzeRoute("leg1 A→via", aLat, aLng, viaLat, viaLng, "roadbike", null);
        RouteData fwd2 = analyzeRoute("leg2 via→B", viaLat, viaLng, bLat, bLng, "roadbike", null);
        RouteData fwd1NoTC = analyzeRoute("leg1 A→via", aLat, aLng, viaLat, viaLng, "roadbike_no_tc", null);
        RouteData fwd2NoTC = analyzeRoute("leg2 via→B", viaLat, viaLng, bLat, bLng, "roadbike_no_tc", null);
        if (fwd1 != null && fwd2 != null) {
            System.out.printf("%n  FORWARD direct total weight (TC):    %.1f + %.1f = %.1f%n",
                    fwd1.totalWeight(), fwd2.totalWeight(), fwd1.totalWeight() + fwd2.totalWeight());
        }
        if (fwd1NoTC != null && fwd2NoTC != null) {
            System.out.printf("  FORWARD direct total weight (no TC): %.1f + %.1f = %.1f%n",
                    fwd1NoTC.totalWeight(), fwd2NoTC.totalWeight(), fwd1NoTC.totalWeight() + fwd2NoTC.totalWeight());
        }

        System.out.println("\n## REVERSE direct: B → via → A ##");
        RouteData rev1 = analyzeRoute("leg1 B→via", bLat, bLng, viaLat, viaLng, "roadbike", null);
        RouteData rev2 = analyzeRoute("leg2 via→A", viaLat, viaLng, aLat, aLng, "roadbike", null);
        RouteData rev1NoTC = analyzeRoute("leg1 B→via", bLat, bLng, viaLat, viaLng, "roadbike_no_tc", null);
        RouteData rev2NoTC = analyzeRoute("leg2 via→A", viaLat, viaLng, aLat, aLng, "roadbike_no_tc", null);
        if (rev1 != null && rev2 != null) {
            System.out.printf("%n  REVERSE direct total weight (TC):    %.1f + %.1f = %.1f%n",
                    rev1.totalWeight(), rev2.totalWeight(), rev1.totalWeight() + rev2.totalWeight());
        }
        if (rev1NoTC != null && rev2NoTC != null) {
            System.out.printf("  REVERSE direct total weight (no TC): %.1f + %.1f = %.1f%n",
                    rev1NoTC.totalWeight(), rev2NoTC.totalWeight(), rev1NoTC.totalWeight() + rev2NoTC.totalWeight());
        }
    }

    /**
     * Template for fast iteration: copy this method, change the JSON literal,
     * re-run. No graph re-import needed because rules are passed via
     * GHRequest.setCustomModel() to the roadbike_test_tc profile (which has
     * an empty turn_penalty in its custom model file plus
     * allow_turn_penalty_in_request: true). The merge in CustomModel.merge
     * appends the request's rules to the profile's empty list, so the rules
     * below apply alone.
     *
     * Enabled to verify the per-request iteration mechanism — produces output
     * identical to analyzeTurnCost_majorRoadDetour (same rules, different
     * delivery path).
     */
    @Test
    void analyzeTurnCost_majorRoadDetour_experimental() {
        String candidateRules = """
                {
                  "turn_penalty": [
                    {
                      "if": "road_name_hash == 0 || road_name_hash != prev_road_name_hash",
                      "do": [
                        { "if": "change_angle >= 110 || change_angle <= -110", "add": "3" },
                        { "else_if": "(change_angle >= 80 && change_angle < 110) || (change_angle <= -80 && change_angle > -110)", "add": "1" },

                        { "if": "change_angle <= -10 && prev_predicted_highway == MAJOR_ROAD", "add": "10" },
                        { "else_if": "change_angle <= -10 && prev_predicted_highway == MINOR_ROAD", "add": "4" },

                        { "if": "change_angle >= 10 && prev_predicted_highway == MAJOR_ROAD", "add": "3" },

                        { "if": "predicted_highway == MAJOR_ROAD && prev_predicted_highway != MAJOR_ROAD",
                          "do": [
                            { "if": "change_angle <= -10", "add": "15" },
                            { "else": "", "add": "10" }
                          ]
                        },

                        { "if": "predicted_highway == MINOR_ROAD && prev_predicted_highway != MAJOR_ROAD && prev_predicted_highway != MINOR_ROAD",
                          "do": [
                            { "if": "change_angle <= -10", "add": "6" },
                            { "else": "", "add": "4" }
                          ]
                        }
                      ]
                    }
                  ]
                }
                """;
        runCase("majorRoadDetour (experimental)", "roadbike_test_tc", customModelFromJson(candidateRules),
                61.318733722910906, 24.2846477010672,
                61.297954,          24.307599);
    }

    /**
     * Diagnostic for a single-junction case where the route is emitted as
     * {@code TURN_SLIGHT_RIGHT} but the rider's perception is "straight" because
     * an alternative at the same junction also goes off to the right (well below
     * 90°). The instruction is technically angle-correct but contextually
     * misleading: relative to the available alternatives the route is the
     * "straightest" choice, yet it is announced as a slight right turn.
     *
     * Question this test should help answer: which rule path in {@code getTurn()}
     * fires, what are the absolute angles of the route and the visible
     * alternative(s), and is there any existing relative-angle comparison that
     * could downgrade the sign in this scenario.
     *
     * API payload (verbatim):
     *   waypoints:
     *     - id=s9CvqNqTucgpr7Dfzf9me (61.471504, 24.047197)
     *     - id=hT0J8HDxZHbjauDRoqrDV (61.471573, 24.048454)
     *   segments: 1 followRoads, profile=gravel
     *   custom_model:
     *     priority: [{ if: "predicted_highway == MAJOR_ROAD", multiply_by: "1.45" }]
     *   instruction_profile=gravel, locale=fi, snap_preventions=[ferry]
     */
    @Test
    void testSlightRightWhenRouteIsStraightestOption_61_471_24_047() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("s9CvqNqTucgpr7Dfzf9me", 61.471504, 24.047197);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("hT0J8HDxZHbjauDRoqrDV", 61.471573, 24.048454);
        request.setWaypoints(List.of(wp1, wp2));

        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_highway == MAJOR_ROAD",
                Statement.Op.MULTIPLY, "1.45"));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("s9CvqNqTucgpr7Dfzf9me");
        seg.setEnd("hT0J8HDxZHbjauDRoqrDV");
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
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== SLIGHT RIGHT WHEN ROUTE IS STRAIGHTEST — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        System.out.println("\n--- Polyline (lat, lng) ---");
        for (int i = 0; i < result.polyline.size(); i++) {
            System.out.printf("  [%d] %.7f, %.7f%n", i, result.polyline.getLat(i), result.polyline.getLon(i));
        }

        // ---- Walk the routed edge sequence and dump per-junction alternatives ----
        EnumEncodedValue<RoadClass> rcEnc = encodingManager.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<PredictedHighway> phEnc = encodingManager.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> psEnc = encodingManager.hasEncodedValue(PredictedSurface.KEY)
                ? encodingManager.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class) : null;
        EnumEncodedValue<Surface> surfEnc = encodingManager.hasEncodedValue(Surface.KEY)
                ? encodingManager.getEnumEncodedValue(Surface.KEY, Surface.class) : null;
        BooleanEncodedValue bikeAccessEnc = encodingManager.getBooleanEncodedValue(VehicleAccess.key("bike"));

        // Re-route via standard GH (with the same custom model) to get edge_id details
        GHRequest ghReq = new GHRequest(61.471504, 24.047197, 61.471573, 24.048454).setProfile("gravel");
        ghReq.setCustomModel(cm);
        ghReq.setPathDetails(List.of("edge_id"));
        ghReq.putHint("instructions", false);
        ghReq.putHint("calc_points", true);
        ghReq.setSnapPreventions(List.of("ferry"));
        GHResponse ghRsp = hopper.route(ghReq);
        assertFalse(ghRsp.hasErrors(), "GH route failed: " + ghRsp.getErrors());

        List<PathDetail> edgeIdDetails = ghRsp.getBest().getPathDetails().get("edge_id");
        assertNotNull(edgeIdDetails);
        System.out.println("\n--- Edge chain (" + edgeIdDetails.size() + " edges) ---");

        EdgeExplorer explorer = baseGraph.createEdgeExplorer();
        int prevNode;
        {
            EdgeIteratorState e0 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(0).getValue(), Integer.MIN_VALUE);
            if (edgeIdDetails.size() == 1) {
                prevNode = e0.getBaseNode();
            } else {
                EdgeIteratorState e1 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(1).getValue(), Integer.MIN_VALUE);
                int b1 = e0.getBaseNode(), a1 = e0.getAdjNode();
                int b2 = e1.getBaseNode(), a2 = e1.getAdjNode();
                int shared;
                if (b1 == b2 || b1 == a2) shared = b1;
                else shared = a1;
                prevNode = (e0.getBaseNode() == shared) ? e0.getAdjNode() : e0.getBaseNode();
            }
        }

        for (int ei = 0; ei < edgeIdDetails.size(); ei++) {
            int edgeId = (Integer) edgeIdDetails.get(ei).getValue();
            EdgeIteratorState raw = baseGraph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            int nextNode = (raw.getBaseNode() == prevNode) ? raw.getAdjNode() : raw.getBaseNode();
            EdgeIteratorState routeEdge = baseGraph.getEdgeIteratorState(edgeId, nextNode);
            int adjNode = routeEdge.getAdjNode();

            RoadClass rc = routeEdge.get(rcEnc);
            PredictedHighway ph = routeEdge.get(phEnc);
            PredictedSurface ps = psEnc != null ? routeEdge.get(psEnc) : null;
            Surface surf = surfEnc != null ? routeEdge.get(surfEnc) : null;
            String name = routeEdge.getName();

            PointList geo = routeEdge.fetchWayGeometry(FetchMode.ALL);
            double startLat = geo.getLat(0), startLon = geo.getLon(0);
            double endLat = geo.getLat(geo.size() - 1), endLon = geo.getLon(geo.size() - 1);

            System.out.printf("%n[edge %d] id=%d base=%d→adj=%d  rc=%s  ph=%s  ps=%s  surf=%s  name=\"%s\"  len=%.1fm%n",
                    ei, edgeId, prevNode, adjNode, rc, ph, ps, surf, name, routeEdge.getDistance());
            System.out.printf("        start=(%.7f, %.7f)  end=(%.7f, %.7f)%n",
                    startLat, startLon, endLat, endLon);

            // Incoming bearing = bearing of the last segment of THIS edge (i.e., the
            // direction the rider is facing as they reach adjNode). This is what the
            // junction analysis at adjNode is measured against.
            int gN = geo.size();
            double inFromLat = geo.getLat(gN - 2), inFromLon = geo.getLon(gN - 2);
            double inToLat = geo.getLat(gN - 1), inToLon = geo.getLon(gN - 1);
            double incomingBearing = AngleCalc.ANGLE_CALC.calcOrientation(inFromLat, inFromLon, inToLat, inToLon);

            int routeNextEdgeId = (ei + 1 < edgeIdDetails.size()) ? (Integer) edgeIdDetails.get(ei + 1).getValue() : -1;
            EdgeIterator iter = explorer.setBaseNode(adjNode);
            int altCount = 0;
            List<String> altLines = new ArrayList<>();
            while (iter.next()) {
                if (iter.getEdge() == edgeId) continue;
                altCount++;
                int altEdgeId = iter.getEdge();
                int altAdj = iter.getAdjNode();
                RoadClass altRC = iter.get(rcEnc);
                PredictedHighway altPH = iter.get(phEnc);
                PredictedSurface altPS = psEnc != null ? iter.get(psEnc) : null;
                Surface altSurf = surfEnc != null ? iter.get(surfEnc) : null;
                String altName = iter.getName();
                boolean altAccess = iter.get(bikeAccessEnc);

                PointList altGeo = iter.fetchWayGeometry(FetchMode.ALL);
                double altFromLat = altGeo.getLat(0), altFromLon = altGeo.getLon(0);
                double altToLat = altGeo.getLat(1), altToLon = altGeo.getLon(1);
                double altBearing = AngleCalc.ANGLE_CALC.calcOrientation(altFromLat, altFromLon, altToLat, altToLon);

                double aligned = AngleCalc.ANGLE_CALC.alignOrientation(incomingBearing, altBearing);
                double deltaRad = aligned - incomingBearing;
                double deltaDeg = Math.toDegrees(deltaRad);
                int altSign = InstructionsHelper.calculateSign(inToLat, inToLon, altToLat, altToLon, incomingBearing);

                String tag = (altEdgeId == routeNextEdgeId) ? "ROUTE" : "alt  ";
                altLines.add(String.format(
                        "        %s edge=%d adj=%d  rc=%s ph=%s ps=%s surf=%s access=%s  Δ=%+.1f° sign=%s  name=\"%s\"",
                        tag, altEdgeId, altAdj, altRC, altPH, altPS, altSurf, altAccess,
                        deltaDeg, signName(altSign), altName));
            }
            if (altCount > 0) {
                System.out.printf("    junction at node %d — %d outgoing edge(s) (excluding incoming):%n", adjNode, altCount);
                for (String s : altLines) System.out.println(s);
            } else {
                System.out.printf("    junction at node %d — no alternatives (dead-end / 2-degree node)%n", adjNode);
            }

            prevNode = adjNode;
        }

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        System.out.println("\n========== SLIGHT RIGHT WHEN ROUTE IS STRAIGHTEST — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);
    }

    /**
     * Diagnostic: user reports that on a gravel route with multiple Y-forks onto
     * WIDE paths (OSM width > 1m) NO TbT instruction is emitted, and asks whether
     * at least {@code tbt_priority: "visual"} should fire so the client wakes the
     * display.
     *
     * API payload (verbatim):
     *   waypoints:
     *     - id=ExSLM3qca83PGLxgSLgku (61.562009, 23.510509)
     *     - id=bqZZnCgYpR5LMozmyDiKt (61.564174, 23.501469)
     *   segments: 1 followRoads, profile=gravel
     *   instruction_profile=gravel, locale=fi, snap_preventions=[ferry]
     *
     * This walks the routed edge chain and dumps each junction's alternatives
     * (PH, surface, gravel_scale, bike access, angle, sign) so we can see which
     * suppression rule fires at each Y-fork and whether width is observable at all.
     */
    @Test
    void testWideYForkNoInstruction_61_562_23_510() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("ExSLM3qca83PGLxgSLgku", 61.562009, 23.510509);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("bqZZnCgYpR5LMozmyDiKt", 61.564174, 23.501469);
        request.setWaypoints(List.of(wp1, wp2));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("ExSLM3qca83PGLxgSLgku");
        seg.setEnd("bqZZnCgYpR5LMozmyDiKt");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        // Is a plain OSM width / max_width EV even present in the graph?
        System.out.println("\n========== WIDE Y-FORK DIAGNOSTIC ==========");
        System.out.println("has EV 'max_width'  = " + encodingManager.hasEncodedValue("max_width"));
        System.out.println("has EV 'width'      = " + encodingManager.hasEncodedValue("width"));
        System.out.println("has EV 'smoothness' = " + encodingManager.hasEncodedValue("smoothness"));
        System.out.println("has EV 'issue_narrow' = " + encodingManager.hasEncodedValue("issue_narrow"));

        System.out.println("\n--- Instructions BEFORE post-processing ---");
        printInstructionsDetailed(result);

        // ---- Walk routed edge sequence, dump per-junction alternatives ----
        EnumEncodedValue<RoadClass> rcEnc = encodingManager.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<PredictedHighway> phEnc = encodingManager.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> psEnc = encodingManager.hasEncodedValue(PredictedSurface.KEY)
                ? encodingManager.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class) : null;
        EnumEncodedValue<Surface> surfEnc = encodingManager.hasEncodedValue(Surface.KEY)
                ? encodingManager.getEnumEncodedValue(Surface.KEY, Surface.class) : null;
        com.graphhopper.routing.ev.DecimalEncodedValue gsEnc = encodingManager.hasEncodedValue(GravelScaleNum.KEY)
                ? encodingManager.getDecimalEncodedValue(GravelScaleNum.KEY) : null;
        BooleanEncodedValue bikeAccessEnc = encodingManager.getBooleanEncodedValue(VehicleAccess.key("bike"));

        GHRequest ghReq = new GHRequest(61.562009, 23.510509, 61.564174, 23.501469).setProfile("gravel");
        ghReq.setPathDetails(List.of("edge_id"));
        ghReq.putHint("instructions", false);
        ghReq.putHint("calc_points", true);
        ghReq.setSnapPreventions(List.of("ferry"));
        GHResponse ghRsp = hopper.route(ghReq);
        assertFalse(ghRsp.hasErrors(), "GH route failed: " + ghRsp.getErrors());

        List<PathDetail> edgeIdDetails = ghRsp.getBest().getPathDetails().get("edge_id");
        assertNotNull(edgeIdDetails);
        System.out.println("\n--- Edge chain (" + edgeIdDetails.size() + " edges) ---");

        EdgeExplorer explorer = baseGraph.createEdgeExplorer();
        int prevNode;
        {
            EdgeIteratorState e0 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(0).getValue(), Integer.MIN_VALUE);
            if (edgeIdDetails.size() == 1) {
                prevNode = e0.getBaseNode();
            } else {
                EdgeIteratorState e1 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(1).getValue(), Integer.MIN_VALUE);
                int b1 = e0.getBaseNode(), a1 = e0.getAdjNode();
                int b2 = e1.getBaseNode(), a2 = e1.getAdjNode();
                int shared;
                if (b1 == b2 || b1 == a2) shared = b1;
                else shared = a1;
                prevNode = (e0.getBaseNode() == shared) ? e0.getAdjNode() : e0.getBaseNode();
            }
        }

        for (int ei = 0; ei < edgeIdDetails.size(); ei++) {
            int edgeId = (Integer) edgeIdDetails.get(ei).getValue();
            EdgeIteratorState raw = baseGraph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            int nextNode = (raw.getBaseNode() == prevNode) ? raw.getAdjNode() : raw.getBaseNode();
            EdgeIteratorState routeEdge = baseGraph.getEdgeIteratorState(edgeId, nextNode);
            int adjNode = routeEdge.getAdjNode();

            RoadClass rc = routeEdge.get(rcEnc);
            PredictedHighway ph = routeEdge.get(phEnc);
            PredictedSurface ps = psEnc != null ? routeEdge.get(psEnc) : null;
            Surface surf = surfEnc != null ? routeEdge.get(surfEnc) : null;
            Double gs = gsEnc != null ? routeEdge.get(gsEnc) : null;
            String name = routeEdge.getName();

            PointList geo = routeEdge.fetchWayGeometry(FetchMode.ALL);
            int gN = geo.size();
            double inFromLat = geo.getLat(gN - 2), inFromLon = geo.getLon(gN - 2);
            double inToLat = geo.getLat(gN - 1), inToLon = geo.getLon(gN - 1);
            double incomingBearing = AngleCalc.ANGLE_CALC.calcOrientation(inFromLat, inFromLon, inToLat, inToLon);

            System.out.printf("%n[edge %d] id=%d base=%d→adj=%d  rc=%s ph=%s ps=%s surf=%s gs=%s  name=\"%s\"  len=%.1fm%n",
                    ei, edgeId, prevNode, adjNode, rc, ph, ps, surf, gs, name, routeEdge.getDistance());

            int routeNextEdgeId = (ei + 1 < edgeIdDetails.size()) ? (Integer) edgeIdDetails.get(ei + 1).getValue() : -1;
            EdgeIterator iter = explorer.setBaseNode(adjNode);
            int altCount = 0;
            List<String> altLines = new ArrayList<>();
            while (iter.next()) {
                if (iter.getEdge() == edgeId) continue;
                altCount++;
                int altEdgeId = iter.getEdge();
                RoadClass altRC = iter.get(rcEnc);
                PredictedHighway altPH = iter.get(phEnc);
                PredictedSurface altPS = psEnc != null ? iter.get(psEnc) : null;
                Surface altSurf = surfEnc != null ? iter.get(surfEnc) : null;
                Double altGS = gsEnc != null ? iter.get(gsEnc) : null;
                String altName = iter.getName();
                boolean altAccess = iter.get(bikeAccessEnc);

                PointList altGeo = iter.fetchWayGeometry(FetchMode.ALL);
                double altToLat = altGeo.getLat(1), altToLon = altGeo.getLon(1);
                double altBearing = AngleCalc.ANGLE_CALC.calcOrientation(
                        altGeo.getLat(0), altGeo.getLon(0), altToLat, altToLon);
                double deltaDeg = Math.toDegrees(
                        AngleCalc.ANGLE_CALC.alignOrientation(incomingBearing, altBearing) - incomingBearing);
                int altSign = InstructionsHelper.calculateSign(inToLat, inToLon, altToLat, altToLon, incomingBearing);

                String tag = (altEdgeId == routeNextEdgeId) ? "ROUTE" : "alt  ";
                altLines.add(String.format(
                        "        %s edge=%d  rc=%s ph=%s ps=%s surf=%s gs=%s access=%s  Δ=%+.1f° sign=%s  name=\"%s\"",
                        tag, altEdgeId, altRC, altPH, altPS, altSurf, altGS, altAccess,
                        deltaDeg, signName(altSign), altName));
            }
            if (altCount > 0) {
                System.out.printf("    junction at node %d — %d outgoing edge(s):%n", adjNode, altCount);
                for (String s : altLines) System.out.println(s);
            } else {
                System.out.printf("    junction at node %d — no alternatives%n", adjNode);
            }
            prevNode = adjNode;
        }

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());
        System.out.println("\n--- Instructions AFTER post-processing ---");
        printInstructionsDetailed(result);
    }

    /**
     * Diagnostic: user reports a U-turn generated entirely by waypoint misplacement
     * on a 3-waypoint / 2-segment gravel_mtb route, and that U-turn suppression is
     * not firing.
     *
     * API payload (verbatim):
     *   waypoints:
     *     - id=xjpeHskaIV6G-mUgON3kO (61.558387, 23.508853)
     *     - id=_TOpe9hqP4baKWqMknld1 (61.557721, 23.512598)  <-- middle wp
     *     - id=6vOqMY6KvM6XsH47e9jfb (61.556869, 23.511434)
     *   segments:
     *     - wp1->wp2 followRoads gravel_mtb initial_heading=100.347 heading_penalty=60
     *     - wp2->wp3 followRoads gravel_mtb initial_heading=33.220  heading_penalty=60
     *   instruction_profile=gravel_mtb, locale=fi, snap_preventions=[ferry]
     *
     * Dumps instructions BEFORE and AFTER post-processing plus both per-segment edge
     * chains so we can see where the U-turn comes from and which suppression gate fails.
     */
    @Test
    void testUturnFromWptMisplacement_61_558_23_508() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        TrailmapInstructionRequest.Waypoint w1 = makeWaypoint("xjpeHskaIV6G-mUgON3kO", 61.558387, 23.508853);
        TrailmapInstructionRequest.Waypoint w2 = makeWaypoint("_TOpe9hqP4baKWqMknld1", 61.557721, 23.512598);
        TrailmapInstructionRequest.Waypoint w3 = makeWaypoint("6vOqMY6KvM6XsH47e9jfb", 61.556869, 23.511434);
        request.setWaypoints(List.of(w1, w2, w3));

        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("xjpeHskaIV6G-mUgON3kO");
        seg1.setEnd("_TOpe9hqP4baKWqMknld1");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("gravel_mtb");
        seg1.setInitialHeading(100.34706486247228);
        seg1.setHeadingPenalty(60.0);

        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("_TOpe9hqP4baKWqMknld1");
        seg2.setEnd("6vOqMY6KvM6XsH47e9jfb");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2.setProfile("gravel_mtb");
        seg2.setInitialHeading(33.21980871124032);
        seg2.setHeadingPenalty(60.0);

        request.setSegments(List.of(seg1, seg2));
        request.setInstructionProfile("gravel_mtb");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== U-TURN FROM WPT MISPLACEMENT — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        // Per-segment edge chains via standard GH, mirroring the request's headings/penalty.
        dumpSegmentEdgeChain(baseGraph, encodingManager, "SEG1 wp1->wp2",
                61.558387, 23.508853, 61.557721, 23.512598, 100.34706486247228);
        dumpSegmentEdgeChain(baseGraph, encodingManager, "SEG2 wp2->wp3",
                61.557721, 23.512598, 61.556869, 23.511434, 33.21980871124032);

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());
        System.out.println("\n========== U-TURN FROM WPT MISPLACEMENT — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        // Flag any remaining U-turn for quick visibility
        for (int i = 0; i < result.instructions.size(); i++) {
            int s = result.instructions.get(i).getSign();
            if (s == Instruction.U_TURN_UNKNOWN || s == Instruction.U_TURN_LEFT || s == Instruction.U_TURN_RIGHT) {
                System.out.printf(">>> U-turn survived at index %d: sign=%d (%s) dist=%.1fm name=\"%s\"%n",
                        i, s, signName(s), result.instructions.get(i).getDistance(),
                        result.instructions.get(i).getName());
            }
        }
    }

    /**
     * Measurement diagnostic: replicate the generator's exact seg1->seg2 chaining
     * (seg2 start overridden to seg1's snapped end, heading = seg1 exit azimuth) and
     * compute the boundary U-turn overshoot length several ways, to validate how a
     * boundary-U-turn fix should measure the *traversed* overlap.
     */
    @Test
    void measureBoundaryUturnOvershoot_61_558_23_508() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        NodeAccess na = baseGraph.getNodeAccess();
        DistanceCalcEarth dc = DistanceCalcEarth.DIST_EARTH;

        // --- seg1: wp1 -> wp2, heading = seg1 initial_heading ---
        GHRequest r1 = new GHRequest(61.558387, 23.508853, 61.557721, 23.512598).setProfile("gravel_mtb");
        r1.setHeadings(List.of(100.34706486247228, Double.NaN));
        r1.putHint("heading_penalty", 60);
        r1.setPathDetails(List.of("edge_id"));
        r1.putHint("instructions", false);
        r1.putHint("calc_points", true);
        r1.setSnapPreventions(List.of("ferry"));
        ResponsePath p1 = hopper.route(r1).getBest();
        List<Integer> e1 = new ArrayList<>();
        for (PathDetail d : p1.getPathDetails().get("edge_id")) e1.add((Integer) d.getValue());
        PointList poly1 = p1.getPoints();

        // generator chains: seg2 start = seg1 last point, heading = azimuth of seg1 last leg
        double startLat = poly1.getLat(poly1.size() - 1), startLon = poly1.getLon(poly1.size() - 1);
        double chainHeading = AngleCalc.ANGLE_CALC.calcAzimuth(
                poly1.getLat(poly1.size() - 2), poly1.getLon(poly1.size() - 2), startLat, startLon);

        // --- seg2: (chained start) -> wp3, heading = chainHeading ---
        GHRequest r2 = new GHRequest(startLat, startLon, 61.556869, 23.511434).setProfile("gravel_mtb");
        r2.setHeadings(List.of(chainHeading, Double.NaN));
        r2.putHint("heading_penalty", 60);
        r2.setPathDetails(List.of("edge_id"));
        r2.putHint("instructions", false);
        r2.putHint("calc_points", true);
        r2.setSnapPreventions(List.of("ferry"));
        ResponsePath p2 = hopper.route(r2).getBest();
        List<Integer> e2 = new ArrayList<>();
        for (PathDetail d : p2.getPathDetails().get("edge_id")) e2.add((Integer) d.getValue());
        PointList poly2 = p2.getPoints();

        System.out.println("\n========== BOUNDARY U-TURN OVERSHOOT MEASUREMENT ==========");
        System.out.println("seg1 edges=" + e1 + " polyPts=" + poly1.size() + " dist=" + Math.round(p1.getDistance()));
        System.out.println("seg2 edges=" + e2 + " polyPts=" + poly2.size() + " dist=" + Math.round(p2.getDistance()));
        System.out.printf("chained seg2 start=(%.7f,%.7f) heading=%.1f%n", startLat, startLon, chainHeading);

        int lastPrev = e1.get(e1.size() - 1);
        int firstCurr = e2.get(0);
        System.out.println("lastPrevEdge=" + lastPrev + " firstCurrentEdge=" + firstCurr
                + " sameEdge=" + (lastPrev == firstCurr));
        if (lastPrev != firstCurr) { System.out.println("Not a same-edge boundary — abort"); return; }

        EdgeIteratorState shared = baseGraph.getEdgeIteratorState(lastPrev, Integer.MIN_VALUE);
        int sBase = shared.getBaseNode(), sAdj = shared.getAdjNode();

        // boundary node = node of shared edge that both seg1-penult and seg2-second connect to
        int secondToLastPrev = e1.get(e1.size() - 2);
        int secondCurr = e2.get(1);
        EdgeIteratorState pen = baseGraph.getEdgeIteratorState(secondToLastPrev, Integer.MIN_VALUE);
        EdgeIteratorState cur2 = baseGraph.getEdgeIteratorState(secondCurr, Integer.MIN_VALUE);
        Set<Integer> penN = Set.of(pen.getBaseNode(), pen.getAdjNode());
        Set<Integer> cur2N = Set.of(cur2.getBaseNode(), cur2.getAdjNode());
        int prevEntry = penN.contains(sBase) ? sBase : (penN.contains(sAdj) ? sAdj : -1);
        int currExit = cur2N.contains(sBase) ? sBase : (cur2N.contains(sAdj) ? sAdj : -1);
        System.out.printf("shared edge %d: base=%d adj=%d fullLen=%.1fm%n", lastPrev, sBase, sAdj, shared.getDistance());
        System.out.printf("prevEntryNode=%d currExitNode=%d  isUturn=%b%n",
                prevEntry, currExit, prevEntry != -1 && prevEntry == currExit);

        int junction = prevEntry; // the node both segments pivot around
        double jLat = na.getLat(junction), jLon = na.getLon(junction);
        System.out.printf("junction node=%d coords=(%.7f,%.7f)%n", junction, jLat, jLon);
        System.out.printf("snap point (seg1 last == seg2 first): seg1Last=(%.7f,%.7f) seg2First=(%.7f,%.7f) coincide=%.2fm%n",
                poly1.getLat(poly1.size() - 1), poly1.getLon(poly1.size() - 1),
                poly2.getLat(0), poly2.getLon(0),
                dc.calcDist(poly1.getLat(poly1.size() - 1), poly1.getLon(poly1.size() - 1),
                        poly2.getLat(0), poly2.getLon(0)));

        // Measure 1: straight-line distance junction node -> snap point
        double dGeom = dc.calcDist(jLat, jLon, startLat, startLon);
        System.out.printf("OVERSHOOT measure 1 (junction-node -> snap, straight line): %.2fm%n", dGeom);

        // Measure 2: length of seg1 polyline tail from the LAST time it passes the junction node
        int jIdxInPoly1 = -1;
        for (int i = poly1.size() - 1; i >= 0; i--) {
            if (dc.calcDist(poly1.getLat(i), poly1.getLon(i), jLat, jLon) < 0.5) { jIdxInPoly1 = i; break; }
        }
        double dTail1 = 0;
        if (jIdxInPoly1 >= 0) {
            for (int i = jIdxInPoly1; i < poly1.size() - 1; i++)
                dTail1 += dc.calcDist(poly1.getLat(i), poly1.getLon(i), poly1.getLat(i + 1), poly1.getLon(i + 1));
        }
        System.out.printf("OVERSHOOT measure 2 (seg1 polyline tail past junction, idx=%d): %.2fm%n", jIdxInPoly1, dTail1);

        // Measure 3: length of seg2 polyline head until it first reaches the junction node (the return leg)
        int jIdxInPoly2 = -1;
        for (int i = 0; i < poly2.size(); i++) {
            if (dc.calcDist(poly2.getLat(i), poly2.getLon(i), jLat, jLon) < 0.5) { jIdxInPoly2 = i; break; }
        }
        double dHead2 = 0;
        if (jIdxInPoly2 >= 0) {
            for (int i = 0; i < jIdxInPoly2; i++)
                dHead2 += dc.calcDist(poly2.getLat(i), poly2.getLon(i), poly2.getLat(i + 1), poly2.getLon(i + 1));
        }
        System.out.printf("OVERSHOOT measure 3 (seg2 return-leg head to junction, idx=%d): %.2fm%n", jIdxInPoly2, dHead2);
    }

    /**
     * Option-C validation: prove that regenerating instructions over the MERGED edge
     * chain (seg1 edges + seg2 edges, with both copies of the short overshoot piece
     * removed) yields a clean continuing turn and NO U-turn — i.e. the generator-level
     * seam fix produces a consistent result "for free" from normal turn generation.
     * Also rebuilds the spur-free boundary polyline and checks distance consistency.
     */
    @Test
    void validateMergedChainRegeneration_61_558_23_508() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager em = hopper.getEncodingManager();
        DistanceCalcEarth dc = DistanceCalcEarth.DIST_EARTH;
        Translation tr = hopper.getTranslationMap().getWithFallBack(Locale.forLanguageTag("fi"));
        Weighting weighting = hopper.createWeighting(hopper.getProfile("gravel_mtb"), new PMap());

        // seg1 wp1->wp2
        GHRequest r1 = new GHRequest(61.558387, 23.508853, 61.557721, 23.512598).setProfile("gravel_mtb");
        r1.setHeadings(List.of(100.34706486247228, Double.NaN));
        r1.putHint("heading_penalty", 60);
        r1.setPathDetails(List.of("edge_id"));
        r1.putHint("instructions", false);
        r1.setSnapPreventions(List.of("ferry"));
        ResponsePath p1 = hopper.route(r1).getBest();
        List<Integer> e1 = new ArrayList<>();
        for (PathDetail d : p1.getPathDetails().get("edge_id")) e1.add((Integer) d.getValue());
        PointList poly1 = p1.getPoints();

        double startLat = poly1.getLat(poly1.size() - 1), startLon = poly1.getLon(poly1.size() - 1);
        double chainHeading = AngleCalc.ANGLE_CALC.calcAzimuth(
                poly1.getLat(poly1.size() - 2), poly1.getLon(poly1.size() - 2), startLat, startLon);

        // seg2 (chained start) -> wp3
        GHRequest r2 = new GHRequest(startLat, startLon, 61.556869, 23.511434).setProfile("gravel_mtb");
        r2.setHeadings(List.of(chainHeading, Double.NaN));
        r2.putHint("heading_penalty", 60);
        r2.setPathDetails(List.of("edge_id"));
        r2.putHint("instructions", false);
        r2.setSnapPreventions(List.of("ferry"));
        ResponsePath p2 = hopper.route(r2).getBest();
        List<Integer> e2 = new ArrayList<>();
        for (PathDetail d : p2.getPathDetails().get("edge_id")) e2.add((Integer) d.getValue());
        PointList poly2 = p2.getPoints();

        // Build merged chain: drop seg1's trailing overshoot piece and seg2's leading copy.
        assertEquals(e1.get(e1.size() - 1), e2.get(0), "boundary should be a shared edge");
        List<Integer> merged = new ArrayList<>(e1.subList(0, e1.size() - 1));
        merged.addAll(e2.subList(1, e2.size()));
        System.out.println("\n========== MERGED-CHAIN REGENERATION (Option C core) ==========");
        System.out.println("seg1=" + e1 + "  seg2=" + e2);
        System.out.println("merged (overshoot removed)=" + merged);

        // Resolve from-node deterministically (node of merged[0] not shared with merged[1]).
        EdgeIteratorState m0 = baseGraph.getEdgeIteratorState(merged.get(0), Integer.MIN_VALUE);
        EdgeIteratorState m1 = baseGraph.getEdgeIteratorState(merged.get(1), Integer.MIN_VALUE);
        int a = m0.getBaseNode(), b = m0.getAdjNode();
        boolean aShared = (m1.getBaseNode() == a || m1.getAdjNode() == a);
        int fromNode = aShared ? b : a;

        Path path = new Path(baseGraph);
        for (int id : merged) path.addEdge(id);
        path.setFromNode(fromNode);
        path.setFound(true);
        int cur = fromNode;
        for (int id : merged) {
            EdgeIteratorState e = baseGraph.getEdgeIteratorState(id, Integer.MIN_VALUE);
            cur = (e.getBaseNode() == cur) ? e.getAdjNode() : e.getBaseNode();
        }
        path.setEndNode(cur);

        InstructionList instr = TrailmapInstructionsFromEdges.calcInstructions(path, baseGraph, weighting, em, tr);
        System.out.println("\nRegenerated instructions over merged chain:");
        int uturns = 0;
        for (int i = 0; i < instr.size(); i++) {
            Instruction in = instr.get(i);
            if (in.getSign() == Instruction.U_TURN_UNKNOWN || in.getSign() == Instruction.U_TURN_LEFT
                    || in.getSign() == Instruction.U_TURN_RIGHT) uturns++;
            System.out.printf("  [%d] sign=%d (%s) dist=%.1fm name=\"%s\"%n",
                    i, in.getSign(), signName(in.getSign()), in.getDistance(), in.getName());
        }
        System.out.println("U-turn count in regenerated chain = " + uturns);

        // Build spur-free boundary polyline: seg1 up to junction + seg2 from junction.
        NodeAccess na = baseGraph.getNodeAccess();
        // junction = shared node both legs pivot on
        EdgeIteratorState shared = baseGraph.getEdgeIteratorState(e1.get(e1.size() - 1), Integer.MIN_VALUE);
        EdgeIteratorState pen = baseGraph.getEdgeIteratorState(e1.get(e1.size() - 2), Integer.MIN_VALUE);
        Set<Integer> penN = Set.of(pen.getBaseNode(), pen.getAdjNode());
        int junction = penN.contains(shared.getBaseNode()) ? shared.getBaseNode() : shared.getAdjNode();
        double jLat = na.getLat(junction), jLon = na.getLon(junction);
        int j1 = -1; for (int i = poly1.size() - 1; i >= 0; i--) if (dc.calcDist(poly1.getLat(i), poly1.getLon(i), jLat, jLon) < 0.5) { j1 = i; break; }
        int j2 = -1; for (int i = 0; i < poly2.size(); i++) if (dc.calcDist(poly2.getLat(i), poly2.getLon(i), jLat, jLon) < 0.5) { j2 = i; break; }
        double mergedPolyLen = 0;
        for (int i = 0; i < j1; i++) mergedPolyLen += dc.calcDist(poly1.getLat(i), poly1.getLon(i), poly1.getLat(i + 1), poly1.getLon(i + 1));
        for (int i = j2; i < poly2.size() - 1; i++) mergedPolyLen += dc.calcDist(poly2.getLat(i), poly2.getLon(i), poly2.getLat(i + 1), poly2.getLon(i + 1));
        double rawConcatLen = p1.getDistance() + p2.getDistance();
        System.out.printf("%nspur-free boundary polyline: junctionIdx seg1=%d seg2=%d%n", j1, j2);
        System.out.printf("merged polyline length=%.1fm  raw concat length=%.1fm  removed spur=%.2fm%n",
                mergedPolyLen, rawConcatLen, rawConcatLen - mergedPolyLen);

        assertEquals(0, uturns, "merged-chain regeneration must produce no U-turn");
    }

    /**
     * Measure the seg1/seg2 boundary overshoot for the crossSegmentUturn route (w1->w2->w3),
     * to confirm whether its boundary U-turn is a short snap artifact (continuation, flanking
     * edges differ → erased) or a genuine reversal (flanking edges same → preserved).
     */
    @Test
    void measureCrossSegmentUturnOvershoot() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        NodeAccess na = baseGraph.getNodeAccess();
        DistanceCalcEarth dc = DistanceCalcEarth.DIST_EARTH;

        GHRequest r1 = new GHRequest(61.505403, 23.679752, 61.505903, 23.666556).setProfile("gravel");
        r1.setHeadings(List.of(282.83505622838345, Double.NaN));
        r1.putHint("heading_penalty", 60);
        r1.setPathDetails(List.of("edge_id"));
        r1.putHint("instructions", false);
        r1.setSnapPreventions(List.of("ferry"));
        ResponsePath p1 = hopper.route(r1).getBest();
        List<Integer> e1 = new ArrayList<>();
        for (PathDetail d : p1.getPathDetails().get("edge_id")) e1.add((Integer) d.getValue());
        PointList poly1 = p1.getPoints();

        double startLat = poly1.getLat(poly1.size() - 1), startLon = poly1.getLon(poly1.size() - 1);
        double chainHeading = AngleCalc.ANGLE_CALC.calcAzimuth(
                poly1.getLat(poly1.size() - 2), poly1.getLon(poly1.size() - 2), startLat, startLon);

        GHRequest r2 = new GHRequest(startLat, startLon, 61.520546, 23.640742).setProfile("gravel");
        r2.setHeadings(List.of(chainHeading, Double.NaN));
        r2.putHint("heading_penalty", 60);
        r2.setPathDetails(List.of("edge_id"));
        r2.putHint("instructions", false);
        r2.setSnapPreventions(List.of("ferry"));
        ResponsePath p2 = hopper.route(r2).getBest();
        List<Integer> e2 = new ArrayList<>();
        for (PathDetail d : p2.getPathDetails().get("edge_id")) e2.add((Integer) d.getValue());

        System.out.println("\n========== CROSS-SEGMENT U-TURN OVERSHOOT (w2) ==========");
        System.out.println("seg1=" + e1 + "  seg2=" + e2);
        int lastPrev = e1.get(e1.size() - 1), firstCurr = e2.get(0);
        System.out.println("lastPrevEdge=" + lastPrev + " firstCurrentEdge=" + firstCurr
                + " sameEdge=" + (lastPrev == firstCurr));
        if (lastPrev != firstCurr || e1.size() < 2 || e2.size() < 2) { System.out.println("not a same-edge >=2 boundary"); return; }

        EdgeIteratorState shared = baseGraph.getEdgeIteratorState(lastPrev, Integer.MIN_VALUE);
        int sBase = shared.getBaseNode(), sAdj = shared.getAdjNode();
        EdgeIteratorState pen = baseGraph.getEdgeIteratorState(e1.get(e1.size() - 2), Integer.MIN_VALUE);
        Set<Integer> penN = Set.of(pen.getBaseNode(), pen.getAdjNode());
        int junction = penN.contains(sBase) ? sBase : (penN.contains(sAdj) ? sAdj : -1);
        double jLat = na.getLat(junction), jLon = na.getLon(junction);
        double overshoot = dc.calcDist(jLat, jLon, startLat, startLon);
        int flankPrev = e1.get(e1.size() - 2), flankCurr = e2.get(1);
        System.out.printf("junction=%d  OVERSHOOT=%.2fm  flankingEdges: prev=%d curr=%d same=%b%n",
                junction, overshoot, flankPrev, flankCurr, flankPrev == flankCurr);
        System.out.println(overshoot <= 15.0 && flankPrev != flankCurr
                ? ">>> SHORT artifact + continuation (flanks differ) → correctly ERASED"
                : ">>> would be preserved");
    }

    /** Dump the GH-routed edge chain for one segment with the given heading. */
    private void dumpSegmentEdgeChain(BaseGraph baseGraph, EncodingManager encodingManager,
                                      String label, double fromLat, double fromLon,
                                      double toLat, double toLon, double heading) {
        EnumEncodedValue<RoadClass> rcEnc = encodingManager.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<PredictedHighway> phEnc = encodingManager.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);

        GHRequest ghReq = new GHRequest(fromLat, fromLon, toLat, toLon).setProfile("gravel_mtb");
        ghReq.setHeadings(List.of(heading, Double.NaN));
        ghReq.putHint("heading_penalty", 60);
        ghReq.setPathDetails(List.of("edge_id"));
        ghReq.putHint("instructions", false);
        ghReq.putHint("calc_points", true);
        ghReq.setSnapPreventions(List.of("ferry"));
        GHResponse ghRsp = hopper.route(ghReq);
        System.out.printf("%n--- %s edge chain ---%n", label);
        if (ghRsp.hasErrors()) {
            System.out.println("   route error: " + ghRsp.getErrors());
            return;
        }
        List<PathDetail> edgeIdDetails = ghRsp.getBest().getPathDetails().get("edge_id");
        System.out.printf("   distance=%.1fm  edges=%d%n", ghRsp.getBest().getDistance(), edgeIdDetails.size());
        for (int i = 0; i < edgeIdDetails.size(); i++) {
            int edgeId = (Integer) edgeIdDetails.get(i).getValue();
            EdgeIteratorState e = baseGraph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            System.out.printf("   [%d] edge=%d base=%d adj=%d rc=%s ph=%s len=%.1fm name=\"%s\"%n",
                    i, edgeId, e.getBaseNode(), e.getAdjNode(), e.get(rcEnc), e.get(phEnc),
                    e.getDistance(), e.getName());
        }
    }

    /**
     * Diagnostic for a real-route case reported during field testing of the
     * reframer. On the current production GH server (no reframer, no recent
     * Trailmap changes) the latter junction emits a single KEEP_LEFT instruction.
     * Test what the new dev server produces and dump per-junction alts so we can
     * judge whether the reframer (or some other recent change) altered behaviour
     * here, and whether the new output matches rider perception.
     *
     * API payload (verbatim):
     *   waypoints:
     *     - id=2mmTPWg18Yp700kDUgqN2 (61.511377, 23.611531)
     *     - id=eMNdFrhNNsLQA3hXhkRZ6 (61.511904, 23.612293)
     *   segments: 1 followRoads, profile=gravel
     *   instruction_profile=gravel, locale=fi, snap_preventions=[ferry]
     */
    @Test
    void testFieldReport_keepLeftAt_61_511_23_611() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("2mmTPWg18Yp700kDUgqN2", 61.511377, 23.611531);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("eMNdFrhNNsLQA3hXhkRZ6", 61.511904, 23.612293);
        request.setWaypoints(List.of(wp1, wp2));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("2mmTPWg18Yp700kDUgqN2");
        seg.setEnd("eMNdFrhNNsLQA3hXhkRZ6");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== FIELD REPORT 61_511_23_611 — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        System.out.println("\n--- Polyline (lat, lng) ---");
        for (int i = 0; i < result.polyline.size(); i++) {
            System.out.printf("  [%d] %.7f, %.7f%n", i, result.polyline.getLat(i), result.polyline.getLon(i));
        }

        EnumEncodedValue<RoadClass> rcEnc = encodingManager.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<PredictedHighway> phEnc = encodingManager.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> psEnc = encodingManager.hasEncodedValue(PredictedSurface.KEY)
                ? encodingManager.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class) : null;
        EnumEncodedValue<Surface> surfEnc = encodingManager.hasEncodedValue(Surface.KEY)
                ? encodingManager.getEnumEncodedValue(Surface.KEY, Surface.class) : null;
        BooleanEncodedValue bikeAccessEnc = encodingManager.getBooleanEncodedValue(VehicleAccess.key("bike"));

        GHRequest ghReq = new GHRequest(61.511377, 23.611531, 61.511904, 23.612293).setProfile("gravel");
        ghReq.setPathDetails(List.of("edge_id"));
        ghReq.putHint("instructions", false);
        ghReq.putHint("calc_points", true);
        ghReq.setSnapPreventions(List.of("ferry"));
        GHResponse ghRsp = hopper.route(ghReq);
        assertFalse(ghRsp.hasErrors(), "GH route failed: " + ghRsp.getErrors());

        List<PathDetail> edgeIdDetails = ghRsp.getBest().getPathDetails().get("edge_id");
        assertNotNull(edgeIdDetails);
        System.out.println("\n--- Edge chain (" + edgeIdDetails.size() + " edges) ---");

        EdgeExplorer explorer = baseGraph.createEdgeExplorer();
        int prevNode;
        {
            EdgeIteratorState e0 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(0).getValue(), Integer.MIN_VALUE);
            if (edgeIdDetails.size() == 1) {
                prevNode = e0.getBaseNode();
            } else {
                EdgeIteratorState e1 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(1).getValue(), Integer.MIN_VALUE);
                int b1 = e0.getBaseNode(), a1 = e0.getAdjNode();
                int b2 = e1.getBaseNode(), a2 = e1.getAdjNode();
                int shared;
                if (b1 == b2 || b1 == a2) shared = b1;
                else shared = a1;
                prevNode = (e0.getBaseNode() == shared) ? e0.getAdjNode() : e0.getBaseNode();
            }
        }

        for (int ei = 0; ei < edgeIdDetails.size(); ei++) {
            int edgeId = (Integer) edgeIdDetails.get(ei).getValue();
            EdgeIteratorState raw = baseGraph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            int nextNode = (raw.getBaseNode() == prevNode) ? raw.getAdjNode() : raw.getBaseNode();
            EdgeIteratorState routeEdge = baseGraph.getEdgeIteratorState(edgeId, nextNode);
            int adjNode = routeEdge.getAdjNode();

            RoadClass rc = routeEdge.get(rcEnc);
            PredictedHighway ph = routeEdge.get(phEnc);
            PredictedSurface ps = psEnc != null ? routeEdge.get(psEnc) : null;
            Surface surf = surfEnc != null ? routeEdge.get(surfEnc) : null;
            String name = routeEdge.getName();

            PointList geo = routeEdge.fetchWayGeometry(FetchMode.ALL);
            double startLat = geo.getLat(0), startLon = geo.getLon(0);
            double endLat = geo.getLat(geo.size() - 1), endLon = geo.getLon(geo.size() - 1);

            System.out.printf("%n[edge %d] id=%d base=%d→adj=%d  rc=%s  ph=%s  ps=%s  surf=%s  name=\"%s\"  len=%.1fm%n",
                    ei, edgeId, prevNode, adjNode, rc, ph, ps, surf, name, routeEdge.getDistance());
            System.out.printf("        start=(%.7f, %.7f)  end=(%.7f, %.7f)%n",
                    startLat, startLon, endLat, endLon);

            int gN = geo.size();
            double inFromLat = geo.getLat(gN - 2), inFromLon = geo.getLon(gN - 2);
            double inToLat = geo.getLat(gN - 1), inToLon = geo.getLon(gN - 1);
            double incomingBearing = AngleCalc.ANGLE_CALC.calcOrientation(inFromLat, inFromLon, inToLat, inToLon);

            int routeNextEdgeId = (ei + 1 < edgeIdDetails.size()) ? (Integer) edgeIdDetails.get(ei + 1).getValue() : -1;
            EdgeIterator iter = explorer.setBaseNode(adjNode);
            int altCount = 0;
            List<String> altLines = new ArrayList<>();
            while (iter.next()) {
                if (iter.getEdge() == edgeId) continue;
                altCount++;
                int altEdgeId = iter.getEdge();
                int altAdj = iter.getAdjNode();
                RoadClass altRC = iter.get(rcEnc);
                PredictedHighway altPH = iter.get(phEnc);
                PredictedSurface altPS = psEnc != null ? iter.get(psEnc) : null;
                Surface altSurf = surfEnc != null ? iter.get(surfEnc) : null;
                String altName = iter.getName();
                boolean altAccess = iter.get(bikeAccessEnc);

                PointList altGeo = iter.fetchWayGeometry(FetchMode.ALL);
                double altFromLat = altGeo.getLat(0), altFromLon = altGeo.getLon(0);
                double altToLat = altGeo.getLat(1), altToLon = altGeo.getLon(1);
                double altBearing = AngleCalc.ANGLE_CALC.calcOrientation(altFromLat, altFromLon, altToLat, altToLon);

                double aligned = AngleCalc.ANGLE_CALC.alignOrientation(incomingBearing, altBearing);
                double deltaRad = aligned - incomingBearing;
                double deltaDeg = Math.toDegrees(deltaRad);
                int altSign = InstructionsHelper.calculateSign(inToLat, inToLon, altToLat, altToLon, incomingBearing);

                String tag = (altEdgeId == routeNextEdgeId) ? "ROUTE" : "alt  ";
                altLines.add(String.format(
                        "        %s edge=%d adj=%d  rc=%s ph=%s ps=%s surf=%s access=%s  Δ=%+.1f° sign=%s  name=\"%s\"",
                        tag, altEdgeId, altAdj, altRC, altPH, altPS, altSurf, altAccess,
                        deltaDeg, signName(altSign), altName));
            }
            if (altCount > 0) {
                System.out.printf("    junction at node %d — %d outgoing edge(s) (excluding incoming):%n", adjNode, altCount);
                for (String s : altLines) System.out.println(s);
            } else {
                System.out.printf("    junction at node %d — no alternatives (dead-end / 2-degree node)%n", adjNode);
            }

            prevNode = adjNode;
        }

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        System.out.println("\n========== FIELD REPORT 61_511_23_611 — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);
    }

    /**
     * Diagnostic for a second field-report case: a short route where the junction
     * used to emit KEEP_LEFT (on a pre-reframer dev or on prod) but now emits
     * CONTINUE_ON_STREET after the reframer changes. User reports the geometry
     * really feels like a Y-fork.
     *
     * API payload (verbatim):
     *   waypoints:
     *     - id=JKZOKXfGJU3LXHJdrg0Xt (61.513608, 23.611567)
     *     - id=yn8tIjbADukZkmd2dbQ39 (61.513902, 23.6121)
     *   segments: 1 followRoads, profile=gravel
     *   custom_model:
     *     priority: [{ if: "predicted_highway == MAJOR_ROAD", multiply_by: "1.45" }]
     *   instruction_profile=gravel, locale=fi, snap_preventions=[ferry]
     */
    @Test
    void testFieldReport_yForkAt_61_513_23_611() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("JKZOKXfGJU3LXHJdrg0Xt", 61.513608, 23.611567);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("yn8tIjbADukZkmd2dbQ39", 61.513902, 23.6121);
        request.setWaypoints(List.of(wp1, wp2));

        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_highway == MAJOR_ROAD",
                Statement.Op.MULTIPLY, "1.45"));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("JKZOKXfGJU3LXHJdrg0Xt");
        seg.setEnd("yn8tIjbADukZkmd2dbQ39");
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
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== FIELD REPORT 61_513_23_611 — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        System.out.println("\n--- Polyline (lat, lng) ---");
        for (int i = 0; i < result.polyline.size(); i++) {
            System.out.printf("  [%d] %.7f, %.7f%n", i, result.polyline.getLat(i), result.polyline.getLon(i));
        }

        EnumEncodedValue<RoadClass> rcEnc = encodingManager.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<PredictedHighway> phEnc = encodingManager.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> psEnc = encodingManager.hasEncodedValue(PredictedSurface.KEY)
                ? encodingManager.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class) : null;
        EnumEncodedValue<Surface> surfEnc = encodingManager.hasEncodedValue(Surface.KEY)
                ? encodingManager.getEnumEncodedValue(Surface.KEY, Surface.class) : null;
        BooleanEncodedValue bikeAccessEnc = encodingManager.getBooleanEncodedValue(VehicleAccess.key("bike"));

        GHRequest ghReq = new GHRequest(61.513608, 23.611567, 61.513902, 23.6121).setProfile("gravel");
        ghReq.setCustomModel(cm);
        ghReq.setPathDetails(List.of("edge_id"));
        ghReq.putHint("instructions", false);
        ghReq.putHint("calc_points", true);
        ghReq.setSnapPreventions(List.of("ferry"));
        GHResponse ghRsp = hopper.route(ghReq);
        assertFalse(ghRsp.hasErrors(), "GH route failed: " + ghRsp.getErrors());

        List<PathDetail> edgeIdDetails = ghRsp.getBest().getPathDetails().get("edge_id");
        assertNotNull(edgeIdDetails);
        System.out.println("\n--- Edge chain (" + edgeIdDetails.size() + " edges) ---");

        EdgeExplorer explorer = baseGraph.createEdgeExplorer();
        int prevNode;
        {
            EdgeIteratorState e0 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(0).getValue(), Integer.MIN_VALUE);
            if (edgeIdDetails.size() == 1) {
                prevNode = e0.getBaseNode();
            } else {
                EdgeIteratorState e1 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(1).getValue(), Integer.MIN_VALUE);
                int b1 = e0.getBaseNode(), a1 = e0.getAdjNode();
                int b2 = e1.getBaseNode(), a2 = e1.getAdjNode();
                int shared;
                if (b1 == b2 || b1 == a2) shared = b1;
                else shared = a1;
                prevNode = (e0.getBaseNode() == shared) ? e0.getAdjNode() : e0.getBaseNode();
            }
        }

        for (int ei = 0; ei < edgeIdDetails.size(); ei++) {
            int edgeId = (Integer) edgeIdDetails.get(ei).getValue();
            EdgeIteratorState raw = baseGraph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            int nextNode = (raw.getBaseNode() == prevNode) ? raw.getAdjNode() : raw.getBaseNode();
            EdgeIteratorState routeEdge = baseGraph.getEdgeIteratorState(edgeId, nextNode);
            int adjNode = routeEdge.getAdjNode();

            RoadClass rc = routeEdge.get(rcEnc);
            PredictedHighway ph = routeEdge.get(phEnc);
            PredictedSurface ps = psEnc != null ? routeEdge.get(psEnc) : null;
            Surface surf = surfEnc != null ? routeEdge.get(surfEnc) : null;
            String name = routeEdge.getName();

            PointList geo = routeEdge.fetchWayGeometry(FetchMode.ALL);
            double startLat = geo.getLat(0), startLon = geo.getLon(0);
            double endLat = geo.getLat(geo.size() - 1), endLon = geo.getLon(geo.size() - 1);

            System.out.printf("%n[edge %d] id=%d base=%d→adj=%d  rc=%s  ph=%s  ps=%s  surf=%s  name=\"%s\"  len=%.1fm%n",
                    ei, edgeId, prevNode, adjNode, rc, ph, ps, surf, name, routeEdge.getDistance());
            System.out.printf("        start=(%.7f, %.7f)  end=(%.7f, %.7f)%n",
                    startLat, startLon, endLat, endLon);

            int gN = geo.size();
            double inFromLat = geo.getLat(gN - 2), inFromLon = geo.getLon(gN - 2);
            double inToLat = geo.getLat(gN - 1), inToLon = geo.getLon(gN - 1);
            double incomingBearing = AngleCalc.ANGLE_CALC.calcOrientation(inFromLat, inFromLon, inToLat, inToLon);

            int routeNextEdgeId = (ei + 1 < edgeIdDetails.size()) ? (Integer) edgeIdDetails.get(ei + 1).getValue() : -1;
            EdgeIterator iter = explorer.setBaseNode(adjNode);
            int altCount = 0;
            List<String> altLines = new ArrayList<>();
            while (iter.next()) {
                if (iter.getEdge() == edgeId) continue;
                altCount++;
                int altEdgeId = iter.getEdge();
                int altAdj = iter.getAdjNode();
                RoadClass altRC = iter.get(rcEnc);
                PredictedHighway altPH = iter.get(phEnc);
                PredictedSurface altPS = psEnc != null ? iter.get(psEnc) : null;
                Surface altSurf = surfEnc != null ? iter.get(surfEnc) : null;
                String altName = iter.getName();
                boolean altAccess = iter.get(bikeAccessEnc);

                PointList altGeo = iter.fetchWayGeometry(FetchMode.ALL);
                double altFromLat = altGeo.getLat(0), altFromLon = altGeo.getLon(0);
                double altToLat = altGeo.getLat(1), altToLon = altGeo.getLon(1);
                double altBearing = AngleCalc.ANGLE_CALC.calcOrientation(altFromLat, altFromLon, altToLat, altToLon);

                double aligned = AngleCalc.ANGLE_CALC.alignOrientation(incomingBearing, altBearing);
                double deltaRad = aligned - incomingBearing;
                double deltaDeg = Math.toDegrees(deltaRad);
                int altSign = InstructionsHelper.calculateSign(inToLat, inToLon, altToLat, altToLon, incomingBearing);

                String tag = (altEdgeId == routeNextEdgeId) ? "ROUTE" : "alt  ";
                altLines.add(String.format(
                        "        %s edge=%d adj=%d  rc=%s ph=%s ps=%s surf=%s access=%s  Δ=%+.1f° sign=%s  name=\"%s\"",
                        tag, altEdgeId, altAdj, altRC, altPH, altPS, altSurf, altAccess,
                        deltaDeg, signName(altSign), altName));
            }
            if (altCount > 0) {
                System.out.printf("    junction at node %d — %d outgoing edge(s) (excluding incoming):%n", adjNode, altCount);
                for (String s : altLines) System.out.println(s);
            } else {
                System.out.printf("    junction at node %d — no alternatives (dead-end / 2-degree node)%n", adjNode);
            }

            prevNode = adjNode;
        }

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        System.out.println("\n========== FIELD REPORT 61_513_23_611 — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);
    }

    /**
     * Third field-report case: a short route where the junction used to emit
     * KEEP_LEFT (or KEEP_RIGHT depending on which fork was picked) but now emits
     * CONTINUE_ON_STREET (or similar). User reports both alternatives at this
     * junction go roughly forward — neither can confidently be labelled "straight"
     * given OSM geometric imprecision.
     *
     * API payload (verbatim):
     *   waypoints:
     *     - id=Jxg-xkQWCN6NV8sg28yMk (61.51312, 23.61352)
     *     - id=_6XnYQ62BoAc_nXmHE8xk (61.513487, 23.613448)
     *   segments: 1 followRoads, profile=gravel
     *   instruction_profile=gravel, locale=fi, snap_preventions=[ferry]
     */
    @Test
    void testFieldReport_equalForkAt_61_513_23_613() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("Jxg-xkQWCN6NV8sg28yMk", 61.51312, 23.61352);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("_6XnYQ62BoAc_nXmHE8xk", 61.513487, 23.613448);
        request.setWaypoints(List.of(wp1, wp2));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("Jxg-xkQWCN6NV8sg28yMk");
        seg.setEnd("_6XnYQ62BoAc_nXmHE8xk");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== FIELD REPORT 61_513_23_613 — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        System.out.println("\n--- Polyline (lat, lng) ---");
        for (int i = 0; i < result.polyline.size(); i++) {
            System.out.printf("  [%d] %.7f, %.7f%n", i, result.polyline.getLat(i), result.polyline.getLon(i));
        }

        EnumEncodedValue<RoadClass> rcEnc = encodingManager.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<PredictedHighway> phEnc = encodingManager.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> psEnc = encodingManager.hasEncodedValue(PredictedSurface.KEY)
                ? encodingManager.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class) : null;
        EnumEncodedValue<Surface> surfEnc = encodingManager.hasEncodedValue(Surface.KEY)
                ? encodingManager.getEnumEncodedValue(Surface.KEY, Surface.class) : null;
        BooleanEncodedValue bikeAccessEnc = encodingManager.getBooleanEncodedValue(VehicleAccess.key("bike"));

        GHRequest ghReq = new GHRequest(61.51312, 23.61352, 61.513487, 23.613448).setProfile("gravel");
        ghReq.setPathDetails(List.of("edge_id"));
        ghReq.putHint("instructions", false);
        ghReq.putHint("calc_points", true);
        ghReq.setSnapPreventions(List.of("ferry"));
        GHResponse ghRsp = hopper.route(ghReq);
        assertFalse(ghRsp.hasErrors(), "GH route failed: " + ghRsp.getErrors());

        List<PathDetail> edgeIdDetails = ghRsp.getBest().getPathDetails().get("edge_id");
        assertNotNull(edgeIdDetails);
        System.out.println("\n--- Edge chain (" + edgeIdDetails.size() + " edges) ---");

        EdgeExplorer explorer = baseGraph.createEdgeExplorer();
        int prevNode;
        {
            EdgeIteratorState e0 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(0).getValue(), Integer.MIN_VALUE);
            if (edgeIdDetails.size() == 1) {
                prevNode = e0.getBaseNode();
            } else {
                EdgeIteratorState e1 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(1).getValue(), Integer.MIN_VALUE);
                int b1 = e0.getBaseNode(), a1 = e0.getAdjNode();
                int b2 = e1.getBaseNode(), a2 = e1.getAdjNode();
                int shared;
                if (b1 == b2 || b1 == a2) shared = b1;
                else shared = a1;
                prevNode = (e0.getBaseNode() == shared) ? e0.getAdjNode() : e0.getBaseNode();
            }
        }

        for (int ei = 0; ei < edgeIdDetails.size(); ei++) {
            int edgeId = (Integer) edgeIdDetails.get(ei).getValue();
            EdgeIteratorState raw = baseGraph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            int nextNode = (raw.getBaseNode() == prevNode) ? raw.getAdjNode() : raw.getBaseNode();
            EdgeIteratorState routeEdge = baseGraph.getEdgeIteratorState(edgeId, nextNode);
            int adjNode = routeEdge.getAdjNode();

            RoadClass rc = routeEdge.get(rcEnc);
            PredictedHighway ph = routeEdge.get(phEnc);
            PredictedSurface ps = psEnc != null ? routeEdge.get(psEnc) : null;
            Surface surf = surfEnc != null ? routeEdge.get(surfEnc) : null;
            String name = routeEdge.getName();

            PointList geo = routeEdge.fetchWayGeometry(FetchMode.ALL);
            double startLat = geo.getLat(0), startLon = geo.getLon(0);
            double endLat = geo.getLat(geo.size() - 1), endLon = geo.getLon(geo.size() - 1);

            System.out.printf("%n[edge %d] id=%d base=%d→adj=%d  rc=%s  ph=%s  ps=%s  surf=%s  name=\"%s\"  len=%.1fm%n",
                    ei, edgeId, prevNode, adjNode, rc, ph, ps, surf, name, routeEdge.getDistance());
            System.out.printf("        start=(%.7f, %.7f)  end=(%.7f, %.7f)%n",
                    startLat, startLon, endLat, endLon);

            int gN = geo.size();
            double inFromLat = geo.getLat(gN - 2), inFromLon = geo.getLon(gN - 2);
            double inToLat = geo.getLat(gN - 1), inToLon = geo.getLon(gN - 1);
            double incomingBearing = AngleCalc.ANGLE_CALC.calcOrientation(inFromLat, inFromLon, inToLat, inToLon);

            int routeNextEdgeId = (ei + 1 < edgeIdDetails.size()) ? (Integer) edgeIdDetails.get(ei + 1).getValue() : -1;
            EdgeIterator iter = explorer.setBaseNode(adjNode);
            int altCount = 0;
            List<String> altLines = new ArrayList<>();
            while (iter.next()) {
                if (iter.getEdge() == edgeId) continue;
                altCount++;
                int altEdgeId = iter.getEdge();
                int altAdj = iter.getAdjNode();
                RoadClass altRC = iter.get(rcEnc);
                PredictedHighway altPH = iter.get(phEnc);
                PredictedSurface altPS = psEnc != null ? iter.get(psEnc) : null;
                Surface altSurf = surfEnc != null ? iter.get(surfEnc) : null;
                String altName = iter.getName();
                boolean altAccess = iter.get(bikeAccessEnc);

                PointList altGeo = iter.fetchWayGeometry(FetchMode.ALL);
                double altFromLat = altGeo.getLat(0), altFromLon = altGeo.getLon(0);
                double altToLat = altGeo.getLat(1), altToLon = altGeo.getLon(1);
                double altBearing = AngleCalc.ANGLE_CALC.calcOrientation(altFromLat, altFromLon, altToLat, altToLon);

                double aligned = AngleCalc.ANGLE_CALC.alignOrientation(incomingBearing, altBearing);
                double deltaRad = aligned - incomingBearing;
                double deltaDeg = Math.toDegrees(deltaRad);
                int altSign = InstructionsHelper.calculateSign(inToLat, inToLon, altToLat, altToLon, incomingBearing);

                String tag = (altEdgeId == routeNextEdgeId) ? "ROUTE" : "alt  ";
                altLines.add(String.format(
                        "        %s edge=%d adj=%d  rc=%s ph=%s ps=%s surf=%s access=%s  Δ=%+.1f° sign=%s  name=\"%s\"",
                        tag, altEdgeId, altAdj, altRC, altPH, altPS, altSurf, altAccess,
                        deltaDeg, signName(altSign), altName));
            }
            if (altCount > 0) {
                System.out.printf("    junction at node %d — %d outgoing edge(s) (excluding incoming):%n", adjNode, altCount);
                for (String s : altLines) System.out.println(s);
            } else {
                System.out.printf("    junction at node %d — no alternatives (dead-end / 2-degree node)%n", adjNode);
            }

            prevNode = adjNode;
        }

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        System.out.println("\n========== FIELD REPORT 61_513_23_613 — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);
    }

    /**
     * Fourth field-report case: another fork where pre-reframer both branches got
     * KEEP_LEFT/KEEP_RIGHT (depending on which fork was chosen) but the reframer
     * now changes the right fork to CONTINUE_ON_STREET.
     *
     * API payload (verbatim):
     *   waypoints:
     *     - id=H9eXnbDSg1RNOxItlwAqo (61.512168, 23.613316)
     *     - id=dDX-gNU65aDMoOHZbKMNd (61.512245, 23.614143)
     *   segments: 1 followRoads, profile=gravel
     *   instruction_profile=gravel, locale=fi, snap_preventions=[ferry]
     */
    @Test
    void testFieldReport_rightForkAt_61_512_23_613() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("H9eXnbDSg1RNOxItlwAqo", 61.512168, 23.613316);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("dDX-gNU65aDMoOHZbKMNd", 61.512245, 23.614143);
        request.setWaypoints(List.of(wp1, wp2));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("H9eXnbDSg1RNOxItlwAqo");
        seg.setEnd("dDX-gNU65aDMoOHZbKMNd");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== FIELD REPORT 61_512_23_613 — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        System.out.println("\n--- Polyline (lat, lng) ---");
        for (int i = 0; i < result.polyline.size(); i++) {
            System.out.printf("  [%d] %.7f, %.7f%n", i, result.polyline.getLat(i), result.polyline.getLon(i));
        }

        EnumEncodedValue<RoadClass> rcEnc = encodingManager.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<PredictedHighway> phEnc = encodingManager.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> psEnc = encodingManager.hasEncodedValue(PredictedSurface.KEY)
                ? encodingManager.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class) : null;
        EnumEncodedValue<Surface> surfEnc = encodingManager.hasEncodedValue(Surface.KEY)
                ? encodingManager.getEnumEncodedValue(Surface.KEY, Surface.class) : null;
        BooleanEncodedValue bikeAccessEnc = encodingManager.getBooleanEncodedValue(VehicleAccess.key("bike"));

        GHRequest ghReq = new GHRequest(61.512168, 23.613316, 61.512245, 23.614143).setProfile("gravel");
        ghReq.setPathDetails(List.of("edge_id"));
        ghReq.putHint("instructions", false);
        ghReq.putHint("calc_points", true);
        ghReq.setSnapPreventions(List.of("ferry"));
        GHResponse ghRsp = hopper.route(ghReq);
        assertFalse(ghRsp.hasErrors(), "GH route failed: " + ghRsp.getErrors());

        List<PathDetail> edgeIdDetails = ghRsp.getBest().getPathDetails().get("edge_id");
        assertNotNull(edgeIdDetails);
        System.out.println("\n--- Edge chain (" + edgeIdDetails.size() + " edges) ---");

        EdgeExplorer explorer = baseGraph.createEdgeExplorer();
        int prevNode;
        {
            EdgeIteratorState e0 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(0).getValue(), Integer.MIN_VALUE);
            if (edgeIdDetails.size() == 1) {
                prevNode = e0.getBaseNode();
            } else {
                EdgeIteratorState e1 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(1).getValue(), Integer.MIN_VALUE);
                int b1 = e0.getBaseNode(), a1 = e0.getAdjNode();
                int b2 = e1.getBaseNode(), a2 = e1.getAdjNode();
                int shared;
                if (b1 == b2 || b1 == a2) shared = b1;
                else shared = a1;
                prevNode = (e0.getBaseNode() == shared) ? e0.getAdjNode() : e0.getBaseNode();
            }
        }

        for (int ei = 0; ei < edgeIdDetails.size(); ei++) {
            int edgeId = (Integer) edgeIdDetails.get(ei).getValue();
            EdgeIteratorState raw = baseGraph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            int nextNode = (raw.getBaseNode() == prevNode) ? raw.getAdjNode() : raw.getBaseNode();
            EdgeIteratorState routeEdge = baseGraph.getEdgeIteratorState(edgeId, nextNode);
            int adjNode = routeEdge.getAdjNode();

            RoadClass rc = routeEdge.get(rcEnc);
            PredictedHighway ph = routeEdge.get(phEnc);
            PredictedSurface ps = psEnc != null ? routeEdge.get(psEnc) : null;
            Surface surf = surfEnc != null ? routeEdge.get(surfEnc) : null;
            String name = routeEdge.getName();

            PointList geo = routeEdge.fetchWayGeometry(FetchMode.ALL);
            double startLat = geo.getLat(0), startLon = geo.getLon(0);
            double endLat = geo.getLat(geo.size() - 1), endLon = geo.getLon(geo.size() - 1);

            System.out.printf("%n[edge %d] id=%d base=%d→adj=%d  rc=%s  ph=%s  ps=%s  surf=%s  name=\"%s\"  len=%.1fm%n",
                    ei, edgeId, prevNode, adjNode, rc, ph, ps, surf, name, routeEdge.getDistance());
            System.out.printf("        start=(%.7f, %.7f)  end=(%.7f, %.7f)%n",
                    startLat, startLon, endLat, endLon);

            int gN = geo.size();
            double inFromLat = geo.getLat(gN - 2), inFromLon = geo.getLon(gN - 2);
            double inToLat = geo.getLat(gN - 1), inToLon = geo.getLon(gN - 1);
            double incomingBearing = AngleCalc.ANGLE_CALC.calcOrientation(inFromLat, inFromLon, inToLat, inToLon);

            int routeNextEdgeId = (ei + 1 < edgeIdDetails.size()) ? (Integer) edgeIdDetails.get(ei + 1).getValue() : -1;
            EdgeIterator iter = explorer.setBaseNode(adjNode);
            int altCount = 0;
            List<String> altLines = new ArrayList<>();
            while (iter.next()) {
                if (iter.getEdge() == edgeId) continue;
                altCount++;
                int altEdgeId = iter.getEdge();
                int altAdj = iter.getAdjNode();
                RoadClass altRC = iter.get(rcEnc);
                PredictedHighway altPH = iter.get(phEnc);
                PredictedSurface altPS = psEnc != null ? iter.get(psEnc) : null;
                Surface altSurf = surfEnc != null ? iter.get(surfEnc) : null;
                String altName = iter.getName();
                boolean altAccess = iter.get(bikeAccessEnc);

                PointList altGeo = iter.fetchWayGeometry(FetchMode.ALL);
                double altFromLat = altGeo.getLat(0), altFromLon = altGeo.getLon(0);
                double altToLat = altGeo.getLat(1), altToLon = altGeo.getLon(1);
                double altBearing = AngleCalc.ANGLE_CALC.calcOrientation(altFromLat, altFromLon, altToLat, altToLon);

                double aligned = AngleCalc.ANGLE_CALC.alignOrientation(incomingBearing, altBearing);
                double deltaRad = aligned - incomingBearing;
                double deltaDeg = Math.toDegrees(deltaRad);
                int altSign = InstructionsHelper.calculateSign(inToLat, inToLon, altToLat, altToLon, incomingBearing);

                String tag = (altEdgeId == routeNextEdgeId) ? "ROUTE" : "alt  ";
                altLines.add(String.format(
                        "        %s edge=%d adj=%d  rc=%s ph=%s ps=%s surf=%s access=%s  Δ=%+.1f° sign=%s  name=\"%s\"",
                        tag, altEdgeId, altAdj, altRC, altPH, altPS, altSurf, altAccess,
                        deltaDeg, signName(altSign), altName));
            }
            if (altCount > 0) {
                System.out.printf("    junction at node %d — %d outgoing edge(s) (excluding incoming):%n", adjNode, altCount);
                for (String s : altLines) System.out.println(s);
            } else {
                System.out.printf("    junction at node %d — no alternatives (dead-end / 2-degree node)%n", adjNode);
            }

            prevNode = adjNode;
        }

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        System.out.println("\n========== FIELD REPORT 61_512_23_613 — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);
    }

    /**
     * Fifth field-report case: route 61.467452/23.726012 → 61.467305/23.726692
     * (gravel, fi). User reports: "I am travelling on an asphalt cycleway, the
     * turn is AWAY from the cycleway, to a small path. Needs to be 'slight left',
     * not 'straight' as both PredictedHighway type and surface change so
     * significantly." The current output is a reframed CONTINUE_ON_STREET
     * ("suoraan"). Both reframer demote-to-CONTINUE paths (Shape 1 side-turn,
     * Shape 4 sandwich near-straight) gate on bothNonRoadAtJunction but NOT on
     * the route's type-change — at a CYCLEWAY → PATH/FOOTWAY transition both
     * sides are non-road yet the rule chain's slight-turn cue is intentional
     * and should survive the reframer.
     *
     * API payload (verbatim):
     *   waypoints:
     *     - id=5jpiaf_Jt9gVAe3vXNI3P (61.467452, 23.726012)
     *     - id=t2N1FaCKiZOIQU07YzCpl (61.467305, 23.726692)
     *   segments: 1 followRoads, profile=gravel
     *   custom_model:
     *     priority: [{ if: "predicted_highway == MAJOR_ROAD", multiply_by: "1.45" }]
     *   instruction_profile=gravel, locale=fi, snap_preventions=[ferry]
     */
    @Test
    void testFieldReport_cyclewayToPath_61_467_23_726() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("5jpiaf_Jt9gVAe3vXNI3P", 61.467452, 23.726012);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("t2N1FaCKiZOIQU07YzCpl", 61.467305, 23.726692);
        request.setWaypoints(List.of(wp1, wp2));

        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_highway == MAJOR_ROAD",
                Statement.Op.MULTIPLY, "1.45"));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("5jpiaf_Jt9gVAe3vXNI3P");
        seg.setEnd("t2N1FaCKiZOIQU07YzCpl");
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
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== FIELD REPORT 61_467_23_726 — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        System.out.println("\n--- Polyline (lat, lng) ---");
        for (int i = 0; i < result.polyline.size(); i++) {
            System.out.printf("  [%d] %.7f, %.7f%n", i, result.polyline.getLat(i), result.polyline.getLon(i));
        }

        EnumEncodedValue<RoadClass> rcEnc = encodingManager.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<PredictedHighway> phEnc = encodingManager.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> psEnc = encodingManager.hasEncodedValue(PredictedSurface.KEY)
                ? encodingManager.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class) : null;
        EnumEncodedValue<Surface> surfEnc = encodingManager.hasEncodedValue(Surface.KEY)
                ? encodingManager.getEnumEncodedValue(Surface.KEY, Surface.class) : null;
        BooleanEncodedValue bikeAccessEnc = encodingManager.getBooleanEncodedValue(VehicleAccess.key("bike"));

        GHRequest ghReq = new GHRequest(61.467452, 23.726012, 61.467305, 23.726692).setProfile("gravel");
        ghReq.setCustomModel(cm);
        ghReq.setPathDetails(List.of("edge_id"));
        ghReq.putHint("instructions", false);
        ghReq.putHint("calc_points", true);
        ghReq.setSnapPreventions(List.of("ferry"));
        GHResponse ghRsp = hopper.route(ghReq);
        assertFalse(ghRsp.hasErrors(), "GH route failed: " + ghRsp.getErrors());

        List<PathDetail> edgeIdDetails = ghRsp.getBest().getPathDetails().get("edge_id");
        assertNotNull(edgeIdDetails);
        System.out.println("\n--- Edge chain (" + edgeIdDetails.size() + " edges) ---");

        EdgeExplorer explorer = baseGraph.createEdgeExplorer();
        int prevNode;
        {
            EdgeIteratorState e0 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(0).getValue(), Integer.MIN_VALUE);
            if (edgeIdDetails.size() == 1) {
                prevNode = e0.getBaseNode();
            } else {
                EdgeIteratorState e1 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(1).getValue(), Integer.MIN_VALUE);
                int b1 = e0.getBaseNode(), a1 = e0.getAdjNode();
                int b2 = e1.getBaseNode(), a2 = e1.getAdjNode();
                int shared;
                if (b1 == b2 || b1 == a2) shared = b1;
                else shared = a1;
                prevNode = (e0.getBaseNode() == shared) ? e0.getAdjNode() : e0.getBaseNode();
            }
        }

        for (int ei = 0; ei < edgeIdDetails.size(); ei++) {
            int edgeId = (Integer) edgeIdDetails.get(ei).getValue();
            EdgeIteratorState raw = baseGraph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            int nextNode = (raw.getBaseNode() == prevNode) ? raw.getAdjNode() : raw.getBaseNode();
            EdgeIteratorState routeEdge = baseGraph.getEdgeIteratorState(edgeId, nextNode);
            int adjNode = routeEdge.getAdjNode();

            RoadClass rc = routeEdge.get(rcEnc);
            PredictedHighway ph = routeEdge.get(phEnc);
            PredictedSurface ps = psEnc != null ? routeEdge.get(psEnc) : null;
            Surface surf = surfEnc != null ? routeEdge.get(surfEnc) : null;
            String name = routeEdge.getName();

            PointList geo = routeEdge.fetchWayGeometry(FetchMode.ALL);
            double startLat = geo.getLat(0), startLon = geo.getLon(0);
            double endLat = geo.getLat(geo.size() - 1), endLon = geo.getLon(geo.size() - 1);

            System.out.printf("%n[edge %d] id=%d base=%d→adj=%d  rc=%s  ph=%s  ps=%s  surf=%s  name=\"%s\"  len=%.1fm%n",
                    ei, edgeId, prevNode, adjNode, rc, ph, ps, surf, name, routeEdge.getDistance());
            System.out.printf("        start=(%.7f, %.7f)  end=(%.7f, %.7f)%n",
                    startLat, startLon, endLat, endLon);

            int gN = geo.size();
            double inFromLat = geo.getLat(gN - 2), inFromLon = geo.getLon(gN - 2);
            double inToLat = geo.getLat(gN - 1), inToLon = geo.getLon(gN - 1);
            double incomingBearing = AngleCalc.ANGLE_CALC.calcOrientation(inFromLat, inFromLon, inToLat, inToLon);

            int routeNextEdgeId = (ei + 1 < edgeIdDetails.size()) ? (Integer) edgeIdDetails.get(ei + 1).getValue() : -1;
            EdgeIterator iter = explorer.setBaseNode(adjNode);
            int altCount = 0;
            List<String> altLines = new ArrayList<>();
            while (iter.next()) {
                if (iter.getEdge() == edgeId) continue;
                altCount++;
                int altEdgeId = iter.getEdge();
                int altAdj = iter.getAdjNode();
                RoadClass altRC = iter.get(rcEnc);
                PredictedHighway altPH = iter.get(phEnc);
                PredictedSurface altPS = psEnc != null ? iter.get(psEnc) : null;
                Surface altSurf = surfEnc != null ? iter.get(surfEnc) : null;
                String altName = iter.getName();
                boolean altAccess = iter.get(bikeAccessEnc);

                PointList altGeo = iter.fetchWayGeometry(FetchMode.ALL);
                double altFromLat = altGeo.getLat(0), altFromLon = altGeo.getLon(0);
                double altToLat = altGeo.getLat(1), altToLon = altGeo.getLon(1);
                double altBearing = AngleCalc.ANGLE_CALC.calcOrientation(altFromLat, altFromLon, altToLat, altToLon);

                double aligned = AngleCalc.ANGLE_CALC.alignOrientation(incomingBearing, altBearing);
                double deltaRad = aligned - incomingBearing;
                double deltaDeg = Math.toDegrees(deltaRad);
                int altSign = InstructionsHelper.calculateSign(inToLat, inToLon, altToLat, altToLon, incomingBearing);

                String tag = (altEdgeId == routeNextEdgeId) ? "ROUTE" : "alt  ";
                altLines.add(String.format(
                        "        %s edge=%d adj=%d  rc=%s ph=%s ps=%s surf=%s access=%s  Δ=%+.1f° sign=%s  name=\"%s\"",
                        tag, altEdgeId, altAdj, altRC, altPH, altPS, altSurf, altAccess,
                        deltaDeg, signName(altSign), altName));
            }
            if (altCount > 0) {
                System.out.printf("    junction at node %d — %d outgoing edge(s) (excluding incoming):%n", adjNode, altCount);
                for (String s : altLines) System.out.println(s);
            } else {
                System.out.printf("    junction at node %d — no alternatives (dead-end / 2-degree node)%n", adjNode);
            }

            prevNode = adjNode;
        }

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        System.out.println("\n========== FIELD REPORT 61_467_23_726 — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);
    }

    /**
     * Sixth field-report case: route 61.467288/23.73144 → 61.467325/23.728313
     * (gravel, fi). User reports the LAST junction emits "right" (TURN_RIGHT)
     * but in practice the route geometry sits very close to "slight right", and
     * there is a competing "tight right" (TURN_SHARP_RIGHT or near it) that the
     * rider could plausibly confuse with the route turn given OSM angle
     * imprecision in this kind of junction. Diagnostic only — collect the alt
     * angles, surfaces, PHs at the last junction so we can see exactly what the
     * rule chain and the reframer were given.
     *
     * API payload (verbatim):
     *   waypoints:
     *     - id=OU0X7mJ85l1KHvRSU8yYF (61.467288, 23.73144)
     *     - id=Ajm9vVlmf4oG78WFC2Yhj (61.467325, 23.728313)
     *   segments: 1 followRoads, profile=gravel
     *   custom_model:
     *     priority: [{ if: "predicted_highway == MAJOR_ROAD", multiply_by: "1.45" }]
     *   instruction_profile=gravel, locale=fi, snap_preventions=[ferry]
     */
    @Test
    void testFieldReport_rightVsSharpRight_61_467_23_731() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("OU0X7mJ85l1KHvRSU8yYF", 61.467288, 23.73144);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("Ajm9vVlmf4oG78WFC2Yhj", 61.467325, 23.728313);
        request.setWaypoints(List.of(wp1, wp2));

        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_highway == MAJOR_ROAD",
                Statement.Op.MULTIPLY, "1.45"));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("OU0X7mJ85l1KHvRSU8yYF");
        seg.setEnd("Ajm9vVlmf4oG78WFC2Yhj");
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
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== FIELD REPORT 61_467_23_731 — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        System.out.println("\n--- Polyline (lat, lng) ---");
        for (int i = 0; i < result.polyline.size(); i++) {
            System.out.printf("  [%d] %.7f, %.7f%n", i, result.polyline.getLat(i), result.polyline.getLon(i));
        }

        EnumEncodedValue<RoadClass> rcEnc = encodingManager.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<PredictedHighway> phEnc = encodingManager.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> psEnc = encodingManager.hasEncodedValue(PredictedSurface.KEY)
                ? encodingManager.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class) : null;
        EnumEncodedValue<Surface> surfEnc = encodingManager.hasEncodedValue(Surface.KEY)
                ? encodingManager.getEnumEncodedValue(Surface.KEY, Surface.class) : null;
        BooleanEncodedValue bikeAccessEnc = encodingManager.getBooleanEncodedValue(VehicleAccess.key("bike"));

        GHRequest ghReq = new GHRequest(61.467288, 23.73144, 61.467325, 23.728313).setProfile("gravel");
        ghReq.setCustomModel(cm);
        ghReq.setPathDetails(List.of("edge_id"));
        ghReq.putHint("instructions", false);
        ghReq.putHint("calc_points", true);
        ghReq.setSnapPreventions(List.of("ferry"));
        GHResponse ghRsp = hopper.route(ghReq);
        assertFalse(ghRsp.hasErrors(), "GH route failed: " + ghRsp.getErrors());

        List<PathDetail> edgeIdDetails = ghRsp.getBest().getPathDetails().get("edge_id");
        assertNotNull(edgeIdDetails);
        System.out.println("\n--- Edge chain (" + edgeIdDetails.size() + " edges) ---");

        EdgeExplorer explorer = baseGraph.createEdgeExplorer();
        int prevNode;
        {
            EdgeIteratorState e0 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(0).getValue(), Integer.MIN_VALUE);
            if (edgeIdDetails.size() == 1) {
                prevNode = e0.getBaseNode();
            } else {
                EdgeIteratorState e1 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(1).getValue(), Integer.MIN_VALUE);
                int b1 = e0.getBaseNode(), a1 = e0.getAdjNode();
                int b2 = e1.getBaseNode(), a2 = e1.getAdjNode();
                int shared;
                if (b1 == b2 || b1 == a2) shared = b1;
                else shared = a1;
                prevNode = (e0.getBaseNode() == shared) ? e0.getAdjNode() : e0.getBaseNode();
            }
        }

        for (int ei = 0; ei < edgeIdDetails.size(); ei++) {
            int edgeId = (Integer) edgeIdDetails.get(ei).getValue();
            EdgeIteratorState raw = baseGraph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            int nextNode = (raw.getBaseNode() == prevNode) ? raw.getAdjNode() : raw.getBaseNode();
            EdgeIteratorState routeEdge = baseGraph.getEdgeIteratorState(edgeId, nextNode);
            int adjNode = routeEdge.getAdjNode();

            RoadClass rc = routeEdge.get(rcEnc);
            PredictedHighway ph = routeEdge.get(phEnc);
            PredictedSurface ps = psEnc != null ? routeEdge.get(psEnc) : null;
            Surface surf = surfEnc != null ? routeEdge.get(surfEnc) : null;
            String name = routeEdge.getName();

            PointList geo = routeEdge.fetchWayGeometry(FetchMode.ALL);
            double startLat = geo.getLat(0), startLon = geo.getLon(0);
            double endLat = geo.getLat(geo.size() - 1), endLon = geo.getLon(geo.size() - 1);

            System.out.printf("%n[edge %d] id=%d base=%d→adj=%d  rc=%s  ph=%s  ps=%s  surf=%s  name=\"%s\"  len=%.1fm%n",
                    ei, edgeId, prevNode, adjNode, rc, ph, ps, surf, name, routeEdge.getDistance());
            System.out.printf("        start=(%.7f, %.7f)  end=(%.7f, %.7f)%n",
                    startLat, startLon, endLat, endLon);

            int gN = geo.size();
            double inFromLat = geo.getLat(gN - 2), inFromLon = geo.getLon(gN - 2);
            double inToLat = geo.getLat(gN - 1), inToLon = geo.getLon(gN - 1);
            double incomingBearing = AngleCalc.ANGLE_CALC.calcOrientation(inFromLat, inFromLon, inToLat, inToLon);

            int routeNextEdgeId = (ei + 1 < edgeIdDetails.size()) ? (Integer) edgeIdDetails.get(ei + 1).getValue() : -1;
            EdgeIterator iter = explorer.setBaseNode(adjNode);
            int altCount = 0;
            List<String> altLines = new ArrayList<>();
            while (iter.next()) {
                if (iter.getEdge() == edgeId) continue;
                altCount++;
                int altEdgeId = iter.getEdge();
                int altAdj = iter.getAdjNode();
                RoadClass altRC = iter.get(rcEnc);
                PredictedHighway altPH = iter.get(phEnc);
                PredictedSurface altPS = psEnc != null ? iter.get(psEnc) : null;
                Surface altSurf = surfEnc != null ? iter.get(surfEnc) : null;
                String altName = iter.getName();
                boolean altAccess = iter.get(bikeAccessEnc);

                PointList altGeo = iter.fetchWayGeometry(FetchMode.ALL);
                double altFromLat = altGeo.getLat(0), altFromLon = altGeo.getLon(0);
                double altToLat = altGeo.getLat(1), altToLon = altGeo.getLon(1);
                double altBearing = AngleCalc.ANGLE_CALC.calcOrientation(altFromLat, altFromLon, altToLat, altToLon);

                double aligned = AngleCalc.ANGLE_CALC.alignOrientation(incomingBearing, altBearing);
                double deltaRad = aligned - incomingBearing;
                double deltaDeg = Math.toDegrees(deltaRad);
                int altSign = InstructionsHelper.calculateSign(inToLat, inToLon, altToLat, altToLon, incomingBearing);

                String tag = (altEdgeId == routeNextEdgeId) ? "ROUTE" : "alt  ";
                altLines.add(String.format(
                        "        %s edge=%d adj=%d  rc=%s ph=%s ps=%s surf=%s access=%s  Δ=%+.1f° sign=%s  name=\"%s\"",
                        tag, altEdgeId, altAdj, altRC, altPH, altPS, altSurf, altAccess,
                        deltaDeg, signName(altSign), altName));
            }
            if (altCount > 0) {
                System.out.printf("    junction at node %d — %d outgoing edge(s) (excluding incoming):%n", adjNode, altCount);
                for (String s : altLines) System.out.println(s);
            } else {
                System.out.printf("    junction at node %d — no alternatives (dead-end / 2-degree node)%n", adjNode);
            }

            prevNode = adjNode;
        }

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        System.out.println("\n========== FIELD REPORT 61_467_23_731 — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);
    }

    /** Holder for one routed result so we can compare across profiles. */
    private static final class RouteData {
        final String profile;
        final List<Integer> edgeIds;
        final double distanceKm;
        final long timeS;
        final double edgeWeightSum;
        final double turnPenaltySum;
        RouteData(String profile, List<Integer> edgeIds,
                  double distanceKm, long timeS,
                  double edgeWeightSum, double turnPenaltySum) {
            this.profile = profile;
            this.edgeIds = edgeIds;
            this.distanceKm = distanceKm;
            this.timeS = timeS;
            this.edgeWeightSum = edgeWeightSum;
            this.turnPenaltySum = turnPenaltySum;
        }
        double totalWeight() { return edgeWeightSum + turnPenaltySum; }
    }

    private static final ObjectMapper TURN_COST_JSON_MAPPER = createTurnCostJsonMapper();
    private static ObjectMapper createTurnCostJsonMapper() {
        ObjectMapper m = new ObjectMapper();
        m.registerModule(new GraphHopperModule());
        return m;
    }

    /** Build a CustomModel from a JSON literal. Useful for fast iteration on turn_penalty rules. */
    private static CustomModel customModelFromJson(String json) {
        try {
            return TURN_COST_JSON_MAPPER.readValue(json, CustomModel.class);
        } catch (Exception e) {
            throw new RuntimeException("Failed to parse custom model JSON: " + e.getMessage(), e);
        }
    }

    /**
     * Run one routing case: forward and reverse, each with the given turn-cost
     * profile (and optional per-request CustomModel) plus the no-TC profile.
     * If queryCm is non-null, the tcProfile must allow per-request turn_penalty
     * (i.e. roadbike_test_tc). Defaults the comparison profile to roadbike_no_tc.
     */
    private void runCase(String name, String tcProfile, CustomModel queryCm,
                         double aLat, double aLng, double bLat, double bLng) {
        runCase(name, tcProfile, "roadbike_no_tc", queryCm, aLat, aLng, bLat, bLng);
    }

    /**
     * Same as runCase above, but with explicit no-TC comparison profile name
     * (use e.g. "gravel_no_tc" when investigating the gravel profile).
     */
    private void runCase(String name, String tcProfile, String noTcProfile, CustomModel queryCm,
                         double aLat, double aLng, double bLat, double bLng) {
        System.out.println("\n========== CASE: " + name + " ==========");

        RouteData fwdTC   = analyzeRoute("Forward A→B", aLat, aLng, bLat, bLng, tcProfile, queryCm);
        RouteData fwdNoTC = analyzeRoute("Forward A→B", aLat, aLng, bLat, bLng, noTcProfile, null);
        printCounterfactualIfRoutesDiffer("Forward A→B", fwdTC, fwdNoTC, tcProfile, noTcProfile, queryCm);

        RouteData revTC   = analyzeRoute("Reverse B→A", bLat, bLng, aLat, aLng, tcProfile, queryCm);
        RouteData revNoTC = analyzeRoute("Reverse B→A", bLat, bLng, aLat, aLng, noTcProfile, null);
        printCounterfactualIfRoutesDiffer("Reverse B→A", revTC, revNoTC, tcProfile, noTcProfile, queryCm);
    }

    /**
     * Routes (start, end, profile, [optional CustomModel]), walks the edge
     * sequence to compute Σ calcEdgeWeight (with correct traversal direction)
     * and Σ calcTurnWeight, prints per-junction penalty lines, and returns
     * the route data for comparison.
     */
    private RouteData analyzeRoute(String label, double startLat, double startLng,
                                   double endLat, double endLng, String profileName,
                                   CustomModel queryCm) {
        System.out.println("\n--- " + label + " | profile=" + profileName
                + (queryCm != null ? " (per-request rules)" : "") + " ---");

        GHRequest req = new GHRequest(startLat, startLng, endLat, endLng)
                .setProfile(profileName);
        req.setPathDetails(List.of("edge_id"));
        req.putHint("instructions", false);
        req.setSnapPreventions(List.of("ferry"));
        if (queryCm != null) req.setCustomModel(queryCm);

        GHResponse rsp = hopper.route(req);
        if (rsp.hasErrors()) {
            System.out.println("  ROUTING FAILED: " + rsp.getErrors());
            return null;
        }

        ResponsePath path = rsp.getBest();
        double distKm = path.getDistance() / 1000.0;
        long timeS = path.getTime() / 1000;

        List<PathDetail> edgeDetails = path.getPathDetails().get("edge_id");
        if (edgeDetails == null || edgeDetails.isEmpty()) {
            System.out.println("  (no edge_id detail — cannot walk)");
            return null;
        }
        List<Integer> edgeIds = new ArrayList<>();
        for (PathDetail d : edgeDetails) {
            int eid = ((Number) d.getValue()).intValue();
            if (edgeIds.isEmpty() || edgeIds.get(edgeIds.size() - 1) != eid) {
                edgeIds.add(eid);
            }
        }

        Profile profile = hopper.getProfile(profileName);
        if (profile == null) {
            System.out.println("  (profile not found)");
            return null;
        }

        Weighting weighting = createWeightingWithCm(profile, queryCm);
        BaseGraph baseGraph = hopper.getBaseGraph();

        // Walk edges to get Σ calcEdgeWeight with correct reverse flag.
        int startNode = determineStartNode(edgeIds, baseGraph);
        double edgeWeightSum = 0;
        int currentNode = startNode;
        for (int eid : edgeIds) {
            EdgeIteratorState edge = baseGraph.getEdgeIteratorState(eid, Integer.MIN_VALUE);
            boolean reverse;
            if (edge.getBaseNode() == currentNode) {
                reverse = false;
                currentNode = edge.getAdjNode();
            } else {
                reverse = true;
                currentNode = edge.getBaseNode();
            }
            edgeWeightSum += weighting.calcEdgeWeight(edge, reverse);
        }

        // Walk junctions for Σ calcTurnWeight, change_angle, and per-junction info.
        EncodingManager em = hopper.getEncodingManager();
        EnumEncodedValue<PredictedHighway> phEnc =
                em.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> psEnc =
                em.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class);
        com.graphhopper.routing.ev.DecimalEncodedValue gsnEnc =
                em.getDecimalEncodedValue(GravelScaleNum.KEY);
        com.graphhopper.routing.ev.IntEncodedValue rnhEnc =
                em.getIntEncodedValue("road_name_hash");
        com.graphhopper.routing.ev.DecimalEncodedValue orientationEnc =
                em.getDecimalEncodedValue("orientation");

        double turnPenaltySum = 0;
        List<String> junctionLines = new ArrayList<>();

        for (int i = 0; i < edgeIds.size() - 1; i++) {
            int prevEid = edgeIds.get(i);
            int currEid = edgeIds.get(i + 1);
            EdgeIteratorState prevE = baseGraph.getEdgeIteratorState(prevEid, Integer.MIN_VALUE);
            EdgeIteratorState currE = baseGraph.getEdgeIteratorState(currEid, Integer.MIN_VALUE);
            int viaNode = findSharedNode(prevE, currE);
            if (viaNode < 0) continue;

            double penalty = weighting.calcTurnWeight(prevEid, viaNode, currEid);
            turnPenaltySum += penalty;

            // Compute change_angle the same way CustomModelParser does.
            boolean inRev = !baseGraph.isAdjNode(prevEid, viaNode);
            boolean outRev = !baseGraph.isAdjNode(currEid, viaNode);
            double angle = com.graphhopper.routing.weighting.custom.CustomWeightingHelper
                    .calcChangeAngle(baseGraph.getEdgeAccess(), orientationEnc,
                            prevEid, inRev, currEid, outRev);

            double juncLat = baseGraph.getNodeAccess().getLat(viaNode);
            double juncLng = baseGraph.getNodeAccess().getLon(viaNode);
            PredictedHighway prevPH = prevE.get(phEnc);
            PredictedHighway currPH = currE.get(phEnc);
            PredictedSurface prevPS = prevE.get(psEnc);
            PredictedSurface currPS = currE.get(psEnc);
            double prevGSN = prevE.get(gsnEnc);
            double currGSN = currE.get(gsnEnc);
            String prevName = nullToBlank(prevE.getName());
            String currName = nullToBlank(currE.getName());
            int prevHash = rnhEnc.getInt(false, prevEid, baseGraph.getEdgeAccess());
            int currHash = rnhEnc.getInt(false, currEid, baseGraph.getEdgeAccess());

            junctionLines.add(String.format(
                    "    [%02d] %.5f,%.5f | angle=%+6.1f° | %s→%s | %s→%s | gsn %.1f→%.1f | \"%s\"→\"%s\" | hash %d→%d | +%.1fs",
                    i + 1, juncLat, juncLng, angle, prevPH, currPH, prevPS, currPS,
                    prevGSN, currGSN, prevName, currName, prevHash, currHash, penalty));
        }

        System.out.printf("  distance=%.3fkm  time=%ds  edges=%d%n", distKm, timeS, edgeIds.size());
        System.out.printf("  edge_weight_sum=%.1f  turn_penalty_sum=%.1f  total_weight=%.1f%n",
                edgeWeightSum, turnPenaltySum, edgeWeightSum + turnPenaltySum);
        for (String ln : junctionLines) System.out.println(ln);

        return new RouteData(profileName, edgeIds, distKm, timeS, edgeWeightSum, turnPenaltySum);
    }

    /**
     * If the two profiles chose different routes, evaluate each route under
     * the OTHER profile's weighting. This shows whether turn costs were the
     * decisive factor — and by how much. Also walks the no-TC route's
     * junctions under TC weighting so we see exactly what penalties WOULD
     * have applied.
     */
    private void printCounterfactualIfRoutesDiffer(String label, RouteData tcRoute, RouteData noTcRoute,
                                                   String tcProfileName, String noTcProfileName,
                                                   CustomModel queryCm) {
        if (tcRoute == null || noTcRoute == null) return;
        if (tcRoute.edgeIds.equals(noTcRoute.edgeIds)) {
            System.out.println("\n[" + label + "] both profiles chose the same route — no counterfactual");
            return;
        }
        System.out.println("\n[" + label + "] routes diverged — counterfactual evaluation:");
        BaseGraph baseGraph = hopper.getBaseGraph();
        Weighting wTC = createWeightingWithCm(hopper.getProfile(tcProfileName), queryCm);
        Weighting wNoTC = createWeightingWithCm(hopper.getProfile(noTcProfileName), null);

        double tcRouteUnderNoTC = computePathWeight(tcRoute.edgeIds, wNoTC, baseGraph);
        double noTcRouteUnderTC = computePathWeight(noTcRoute.edgeIds, wTC, baseGraph);

        System.out.printf("  TC-chosen route under TC weighting:    %.1f (chosen by %s)%n", tcRoute.totalWeight(), tcProfileName);
        System.out.printf("  TC-chosen route under no-TC weighting: %.1f%n", tcRouteUnderNoTC);
        System.out.printf("  no-TC-chosen route under TC weighting: %.1f%n", noTcRouteUnderTC);
        System.out.printf("  no-TC-chosen route under no-TC weighting: %.1f (chosen by %s)%n", noTcRoute.totalWeight(), noTcProfileName);
        double flipMargin = noTcRouteUnderTC - tcRoute.totalWeight();
        System.out.printf("  flip margin (no-TC route under TC) - (TC route under TC) = %.1f%n", flipMargin);
        System.out.println("    (positive ⇒ TC route correctly preferred under turn-cost weighting by this margin)");

        // Show what TC weighting WOULD charge at each junction of the no-TC-chosen route.
        System.out.println("  no-TC-chosen route junctions under TC weighting:");
        printJunctionsUnderWeighting(noTcRoute.edgeIds, wTC, baseGraph);
    }

    /** Walks an edge sequence and prints each junction's angle and penalty under the given weighting. */
    private void printJunctionsUnderWeighting(List<Integer> edgeIds, Weighting weighting, BaseGraph baseGraph) {
        EncodingManager em = hopper.getEncodingManager();
        EnumEncodedValue<PredictedHighway> phEnc =
                em.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> psEnc =
                em.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class);
        com.graphhopper.routing.ev.DecimalEncodedValue gsnEnc =
                em.getDecimalEncodedValue(GravelScaleNum.KEY);
        com.graphhopper.routing.ev.IntEncodedValue rnhEnc =
                em.getIntEncodedValue("road_name_hash");
        com.graphhopper.routing.ev.DecimalEncodedValue orientationEnc =
                em.getDecimalEncodedValue("orientation");

        for (int i = 0; i < edgeIds.size() - 1; i++) {
            int prevEid = edgeIds.get(i);
            int currEid = edgeIds.get(i + 1);
            EdgeIteratorState prevE = baseGraph.getEdgeIteratorState(prevEid, Integer.MIN_VALUE);
            EdgeIteratorState currE = baseGraph.getEdgeIteratorState(currEid, Integer.MIN_VALUE);
            int viaNode = findSharedNode(prevE, currE);
            if (viaNode < 0) continue;

            double penalty = weighting.calcTurnWeight(prevEid, viaNode, currEid);
            boolean inRev = !baseGraph.isAdjNode(prevEid, viaNode);
            boolean outRev = !baseGraph.isAdjNode(currEid, viaNode);
            double angle = com.graphhopper.routing.weighting.custom.CustomWeightingHelper
                    .calcChangeAngle(baseGraph.getEdgeAccess(), orientationEnc,
                            prevEid, inRev, currEid, outRev);

            double juncLat = baseGraph.getNodeAccess().getLat(viaNode);
            double juncLng = baseGraph.getNodeAccess().getLon(viaNode);
            PredictedHighway prevPH = prevE.get(phEnc);
            PredictedHighway currPH = currE.get(phEnc);
            PredictedSurface prevPS = prevE.get(psEnc);
            PredictedSurface currPS = currE.get(psEnc);
            double prevGSN = prevE.get(gsnEnc);
            double currGSN = currE.get(gsnEnc);
            String prevName = nullToBlank(prevE.getName());
            String currName = nullToBlank(currE.getName());
            int prevHash = rnhEnc.getInt(false, prevEid, baseGraph.getEdgeAccess());
            int currHash = rnhEnc.getInt(false, currEid, baseGraph.getEdgeAccess());

            System.out.printf("    [%02d] %.5f,%.5f | angle=%+6.1f° | %s→%s | %s→%s | gsn %.1f→%.1f | \"%s\"→\"%s\" | hash %d→%d | +%.1fs%n",
                    i + 1, juncLat, juncLng, angle, prevPH, currPH, prevPS, currPS,
                    prevGSN, currGSN, prevName, currName, prevHash, currHash, penalty);
        }
    }

    /** Compute Σ calcEdgeWeight + Σ calcTurnWeight for an edge sequence under a given Weighting. */
    private double computePathWeight(List<Integer> edgeIds, Weighting weighting, BaseGraph baseGraph) {
        if (edgeIds.isEmpty()) return 0;
        int startNode = determineStartNode(edgeIds, baseGraph);
        double sum = 0;
        int currentNode = startNode;
        for (int eid : edgeIds) {
            EdgeIteratorState edge = baseGraph.getEdgeIteratorState(eid, Integer.MIN_VALUE);
            boolean reverse;
            if (edge.getBaseNode() == currentNode) {
                reverse = false;
                currentNode = edge.getAdjNode();
            } else {
                reverse = true;
                currentNode = edge.getBaseNode();
            }
            sum += weighting.calcEdgeWeight(edge, reverse);
        }
        for (int i = 0; i < edgeIds.size() - 1; i++) {
            int prevEid = edgeIds.get(i);
            int currEid = edgeIds.get(i + 1);
            EdgeIteratorState prevE = baseGraph.getEdgeIteratorState(prevEid, Integer.MIN_VALUE);
            EdgeIteratorState currE = baseGraph.getEdgeIteratorState(currEid, Integer.MIN_VALUE);
            int viaNode = findSharedNode(prevE, currE);
            if (viaNode < 0) continue;
            sum += weighting.calcTurnWeight(prevEid, viaNode, currEid);
        }
        return sum;
    }

    /** Determine the starting node of the route by checking how edge[0] connects to edge[1]. */
    private int determineStartNode(List<Integer> edgeIds, BaseGraph baseGraph) {
        EdgeIteratorState e0 = baseGraph.getEdgeIteratorState(edgeIds.get(0), Integer.MIN_VALUE);
        if (edgeIds.size() == 1) return e0.getBaseNode();
        EdgeIteratorState e1 = baseGraph.getEdgeIteratorState(edgeIds.get(1), Integer.MIN_VALUE);
        int shared = findSharedNode(e0, e1);
        return (e0.getBaseNode() == shared) ? e0.getAdjNode() : e0.getBaseNode();
    }

    /** Find the shared graph node between two consecutive edges, or -1 if none. */
    private int findSharedNode(EdgeIteratorState e1, EdgeIteratorState e2) {
        int b1 = e1.getBaseNode(), a1 = e1.getAdjNode();
        int b2 = e2.getBaseNode(), a2 = e2.getAdjNode();
        if (b1 == b2 || b1 == a2) return b1;
        if (a1 == b2 || a1 == a2) return a1;
        return -1;
    }

    private static String nullToBlank(String s) {
        return s == null ? "" : s;
    }

    /** Create a Weighting for the given profile, optionally merged with a query-time CustomModel. */
    private Weighting createWeightingWithCm(Profile profile, CustomModel queryCm) {
        PMap hints = new PMap();
        if (queryCm != null) hints.putObject(CustomModel.KEY, queryCm);
        return ((TrailmapGraphHopper) hopper).createWeighting(profile, hints);
    }

    // =============================================================================
    // Diagnostic: U-turn at WP2 breaks instruction alignment for the rest of route
    // =============================================================================
    //
    // Production bug report (2026-05-10):
    //   A 30km route had correct instructions for the first ~2km, then NO instructions
    //   for the middle ~28km, then a flurry of misaligned instructions near the end.
    //   The user identified one waypoint that causes a U-turn at the segment boundary
    //   as the culprit. Moving the waypoint ~12m east eliminates the U-turn and the
    //   instructions become correct end-to-end.
    //
    // Test compares the two API payloads from the bug report:
    //   - "Broken" : WP2 = (61.505903, 23.666556), seg1 has initial_heading=282.83
    //   - "Fixed"  : WP2 = (61.505917, 23.666779), seg1 has NO initial_heading
    //
    // The two payloads differ both in WP2 coords (~12m) AND seg1's initial_heading.
    // The U-turn at the seg1/seg2 boundary in the "broken" payload is the suspected
    // root cause; the "fixed" payload routes WP2 to a position where no U-turn occurs.
    //
    // No assertions — purely diagnostic. Read the printed instruction intervals and
    // polyline correlations to see where the U-turn instruction lands in the polyline.
    @Test
    void diagUturnWaypointBreaksRemap() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        // ---- Common waypoints (only WP2 differs between cases) ----
        double w1Lat = 61.505403, w1Lng = 23.679752;
        double w3Lat = 61.520546, w3Lng = 23.640742;
        double w4Lat = 61.531336, w4Lng = 23.636473;

        // ---- BROKEN case ----
        double brokenW2Lat = 61.505903, brokenW2Lng = 23.666556;
        TrailmapInstructionRequest brokenReq = buildUturnDiagRequest(
                w1Lat, w1Lng, brokenW2Lat, brokenW2Lng, w3Lat, w3Lng, w4Lat, w4Lng,
                /*seg1Heading=*/ 282.83505622838345,
                /*seg2Heading=*/ 263.8884413748203,
                /*seg3Heading=*/ 315.47069676557135);

        // ---- FIXED case ----
        double fixedW2Lat = 61.505917, fixedW2Lng = 23.666779;
        TrailmapInstructionRequest fixedReq = buildUturnDiagRequest(
                w1Lat, w1Lng, fixedW2Lat, fixedW2Lng, w3Lat, w3Lng, w4Lat, w4Lng,
                /*seg1Heading=*/ null,
                /*seg2Heading=*/ 264.62148601809116,
                /*seg3Heading=*/ 315.47069676557135);

        System.out.println("\n=================================================================");
        System.out.println("== U-TURN WAYPOINT DIAGNOSTIC ==");
        System.out.println("=================================================================");

        System.out.println("\n----- BROKEN CASE: WP2=(" + brokenW2Lat + "," + brokenW2Lng + ") -----");
        analyzeUturnRequest(generator, brokenReq, "broken");

        System.out.println("\n----- FIXED CASE: WP2=(" + fixedW2Lat + "," + fixedW2Lng + ") -----");
        analyzeUturnRequest(generator, fixedReq, "fixed");

        System.out.println("\n=================================================================");
        System.out.println("== END U-TURN DIAGNOSTIC ==");
        System.out.println("=================================================================");
    }

    /** Build the 4-waypoint, 3-segment request used by diagUturnWaypointBreaksRemap. */
    private TrailmapInstructionRequest buildUturnDiagRequest(
            double w1Lat, double w1Lng, double w2Lat, double w2Lng,
            double w3Lat, double w3Lng, double w4Lat, double w4Lng,
            Double seg1Heading, Double seg2Heading, Double seg3Heading) {
        TrailmapInstructionRequest req = new TrailmapInstructionRequest();
        req.setWaypoints(List.of(
                makeWaypoint("w1", w1Lat, w1Lng),
                makeWaypoint("w2", w2Lat, w2Lng),
                makeWaypoint("w3", w3Lat, w3Lng),
                makeWaypoint("w4", w4Lat, w4Lng)));

        TrailmapInstructionRequest.Segment s1 = new TrailmapInstructionRequest.Segment();
        s1.setStart("w1"); s1.setEnd("w2");
        s1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        s1.setProfile("gravel");
        if (seg1Heading != null) { s1.setInitialHeading(seg1Heading); s1.setHeadingPenalty(60.0); }

        TrailmapInstructionRequest.Segment s2 = new TrailmapInstructionRequest.Segment();
        s2.setStart("w2"); s2.setEnd("w3");
        s2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        s2.setProfile("gravel");
        if (seg2Heading != null) { s2.setInitialHeading(seg2Heading); s2.setHeadingPenalty(60.0); }

        TrailmapInstructionRequest.Segment s3 = new TrailmapInstructionRequest.Segment();
        s3.setStart("w3"); s3.setEnd("w4");
        s3.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        s3.setProfile("gravel");
        if (seg3Heading != null) { s3.setInitialHeading(seg3Heading); s3.setHeadingPenalty(60.0); }

        req.setSegments(List.of(s1, s2, s3));
        req.setInstructionProfile("gravel");
        req.setLocale("fi");
        req.setSnapPreventions(List.of("ferry"));
        return req;
    }

    /**
     * Print a comprehensive diagnostic dump for a single request: per-segment edge IDs
     * (so we can detect U-turn boundaries), full polyline size, generated instruction
     * list with intervals, and where each non-FINISH instruction's first point sits
     * relative to the polyline.
     */
    private void analyzeUturnRequest(RouteInstructionGenerator generator,
                                     TrailmapInstructionRequest req,
                                     String label) {
        BaseGraph baseGraph = hopper.getBaseGraph();
        // Step 1: Per-segment routing to extract raw edge IDs and snap points.
        // Mirrors what RouteInstructionGenerator.generate() does internally so we can
        // see the boundary state without instrumenting production code.
        List<List<Integer>> perSegEdgeIds = new ArrayList<>();
        List<PointList> perSegPolyline = new ArrayList<>();
        Map<String, TrailmapInstructionRequest.Coordinates> wpMap = new HashMap<>();
        for (TrailmapInstructionRequest.Waypoint wp : req.getWaypoints()) {
            wpMap.put(wp.getId(), wp.getCoordinates());
        }

        Double chainedHeading = null;
        double chainedLat = Double.NaN, chainedLon = Double.NaN;
        for (int si = 0; si < req.getSegments().size(); si++) {
            TrailmapInstructionRequest.Segment seg = req.getSegments().get(si);
            TrailmapInstructionRequest.Coordinates startC = wpMap.get(seg.getStart());
            TrailmapInstructionRequest.Coordinates endC = wpMap.get(seg.getEnd());
            double sLat = chainedHeading != null ? chainedLat : startC.getLat();
            double sLng = chainedHeading != null ? chainedLon : startC.getLng();
            GHRequest gr = new GHRequest(sLat, sLng, endC.getLat(), endC.getLng())
                    .setProfile(seg.getProfile());
            gr.setPathDetails(List.of("edge_id"));
            gr.putHint("instructions", false);
            gr.putHint("calc_points", true);
            gr.setSnapPreventions(req.getSnapPreventions());
            Double heading = chainedHeading != null ? chainedHeading : seg.getInitialHeading();
            if (heading != null && !heading.isNaN()) {
                gr.setHeadings(List.of(heading, Double.NaN));
            }
            if (seg.getHeadingPenalty() != null) {
                gr.putHint("heading_penalty", seg.getHeadingPenalty());
            }
            GHResponse rsp = hopper.route(gr);
            if (rsp.hasErrors()) {
                System.out.println("  seg" + (si + 1) + " routing failed: " + rsp.getErrors());
                perSegEdgeIds.add(new ArrayList<>());
                perSegPolyline.add(new PointList());
                continue;
            }
            List<Integer> edgeIds = new ArrayList<>();
            for (PathDetail d : rsp.getBest().getPathDetails().get("edge_id")) {
                edgeIds.add((Integer) d.getValue());
            }
            perSegEdgeIds.add(edgeIds);
            PointList poly = rsp.getBest().getPoints();
            perSegPolyline.add(poly);

            // Chain heading + snapped end for next segment (same as generator does).
            if (poly.size() >= 2) {
                chainedHeading = AngleCalc.ANGLE_CALC.calcAzimuth(
                        poly.getLat(poly.size() - 2), poly.getLon(poly.size() - 2),
                        poly.getLat(poly.size() - 1), poly.getLon(poly.size() - 1));
                chainedLat = poly.getLat(poly.size() - 1);
                chainedLon = poly.getLon(poly.size() - 1);
            }
        }

        // Step 2: Show edge IDs around each boundary and detect U-turn boundaries.
        System.out.println("\nPer-segment edge counts:");
        for (int si = 0; si < perSegEdgeIds.size(); si++) {
            List<Integer> ids = perSegEdgeIds.get(si);
            System.out.println("  seg" + (si + 1) + ": " + ids.size() + " edges, polyline pts=" + perSegPolyline.get(si).size());
        }

        System.out.println("\nBoundary analysis:");
        for (int si = 0; si + 1 < perSegEdgeIds.size(); si++) {
            List<Integer> prev = perSegEdgeIds.get(si);
            List<Integer> curr = perSegEdgeIds.get(si + 1);
            if (prev.isEmpty() || curr.isEmpty()) continue;
            int lastPrev = prev.get(prev.size() - 1);
            int firstCurr = curr.get(0);
            System.out.printf("  boundary seg%d→seg%d: lastPrev=%d firstCurr=%d%n",
                    si + 1, si + 2, lastPrev, firstCurr);
            if (lastPrev == firstCurr) {
                System.out.println("    >> Same edge at boundary");
                if (prev.size() >= 2 && curr.size() >= 2) {
                    int secondToLastPrev = prev.get(prev.size() - 2);
                    int secondCurr = curr.get(1);
                    EdgeIteratorState shared = baseGraph.getEdgeIteratorState(lastPrev, Integer.MIN_VALUE);
                    int nodeA = shared.getBaseNode(), nodeB = shared.getAdjNode();
                    EdgeIteratorState prevPenult = baseGraph.getEdgeIteratorState(secondToLastPrev, Integer.MIN_VALUE);
                    EdgeIteratorState currSecond = baseGraph.getEdgeIteratorState(secondCurr, Integer.MIN_VALUE);
                    Set<Integer> prevPenNodes = Set.of(prevPenult.getBaseNode(), prevPenult.getAdjNode());
                    Set<Integer> currSecNodes = Set.of(currSecond.getBaseNode(), currSecond.getAdjNode());
                    int prevEntry = prevPenNodes.contains(nodeA) ? nodeA
                            : (prevPenNodes.contains(nodeB) ? nodeB : -1);
                    int currExit = currSecNodes.contains(nodeA) ? nodeA
                            : (currSecNodes.contains(nodeB) ? nodeB : -1);
                    boolean isUturn = prevEntry != -1 && prevEntry == currExit;
                    System.out.printf("    shared edge %d (nodeA=%d nodeB=%d), prevEntry=%d currExit=%d → U-TURN=%s%n",
                            lastPrev, nodeA, nodeB, prevEntry, currExit, isUturn);
                    if (isUturn) {
                        NodeAccess na = baseGraph.getNodeAccess();
                        // Snap point ≈ last point of prev polyline (also = first point of curr polyline).
                        PointList prevP = perSegPolyline.get(si);
                        double snapLat = prevP.getLat(prevP.size() - 1);
                        double snapLon = prevP.getLon(prevP.size() - 1);
                        double dToA = DistanceCalcEarth.DIST_EARTH.calcDist(
                                snapLat, snapLon, na.getLat(nodeA), na.getLon(nodeA));
                        double dToB = DistanceCalcEarth.DIST_EARTH.calcDist(
                                snapLat, snapLon, na.getLat(nodeB), na.getLon(nodeB));
                        System.out.printf("    snap point=(%.7f, %.7f) — dist to nodeA=%.2fm, to nodeB=%.2fm%n",
                                snapLat, snapLon, dToA, dToB);
                        // Which node is the synthetic-path fromNode? Per resolveFromNode:
                        // the endpoint that does NOT connect to secondCurr (since edgeIds[0..1] differ).
                        boolean aConnects = currSecond.getBaseNode() == nodeA || currSecond.getAdjNode() == nodeA;
                        boolean bConnects = currSecond.getBaseNode() == nodeB || currSecond.getAdjNode() == nodeB;
                        int fromNode;
                        if (aConnects && !bConnects) fromNode = nodeB;
                        else if (bConnects && !aConnects) fromNode = nodeA;
                        else fromNode = -1;
                        System.out.printf("    seg%d synthetic-path fromNode=%d (the FAR end of shared edge)%n",
                                si + 2, fromNode);
                        if (fromNode != -1) {
                            System.out.printf("    fromNode coords=(%.7f, %.7f)%n",
                                    na.getLat(fromNode), na.getLon(fromNode));
                            double fnDistToSnap = DistanceCalcEarth.DIST_EARTH.calcDist(
                                    snapLat, snapLon, na.getLat(fromNode), na.getLon(fromNode));
                            System.out.printf("    fromNode is %.2fm from snap point%n", fnDistToSnap);
                        }
                    }
                }
            } else {
                EdgeIteratorState pe = baseGraph.getEdgeIteratorState(lastPrev, Integer.MIN_VALUE);
                EdgeIteratorState ce = baseGraph.getEdgeIteratorState(firstCurr, Integer.MIN_VALUE);
                Set<Integer> pn = Set.of(pe.getBaseNode(), pe.getAdjNode());
                boolean shared = pn.contains(ce.getBaseNode()) || pn.contains(ce.getAdjNode());
                System.out.println("    " + (shared ? "share a node (natural concatenation)"
                                                  : "DISCONNECTED — would need micro-routing"));
            }
        }

        // Step 3: Generate instructions and print full list with intervals & key extraInfo.
        RouteInstructionGenerator.Result result = generator.generate(req);
        System.out.println("\nGenerated " + result.instructions.size() + " instructions, polyline size="
                + result.polyline.size());
        printInstructionsDetailed(result);

        // Step 4: For each non-FINISH instruction, report its first-point coords and
        // how that coordinate matches against the full polyline. This is what the
        // remapInstructionGeometry coordinate-match loop sees.
        System.out.println("\nInstruction first-point / polyline correlation:");
        int polyIdx = 0;
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction in = result.instructions.get(i);
            if (in.getSign() == Instruction.FINISH) continue;
            PointList pts = in.getPoints();
            if (pts.size() == 0) continue;
            double tLat = pts.getLat(0), tLon = pts.getLon(0);
            // After remap, the instruction's polyStart is the index of its first point in the full polyline.
            // Find min |Δlat|+|Δlon| match across the full polyline (without monotonicity constraint).
            int bestIdx = 0; double bestDist = Double.MAX_VALUE;
            for (int pi = 0; pi < result.polyline.size(); pi++) {
                double d = Math.abs(result.polyline.getLat(pi) - tLat)
                        + Math.abs(result.polyline.getLon(pi) - tLon);
                if (d < bestDist) { bestDist = d; bestIdx = pi; }
            }
            // Convert match distance from lat+lon degrees to approx meters at this latitude.
            double approxMeters = bestDist * 111000.0 / 2.0;
            System.out.printf("  [%d] sign=%d (%s) first-pt=(%.7f,%.7f) globalBestPolyIdx=%d (~%.2fm off)  intervalStart(after remap)=%d%n",
                    i, in.getSign(), signName(in.getSign()), tLat, tLon, bestIdx, approxMeters, polyIdx);
            polyIdx += in.getLength();
        }
    }

    // =============================================================================
    // Diagnostic: short out-and-back U-turn that DOES NOT break follow-up instructions
    // =============================================================================
    //
    // Second snippet from the bug investigation (2026-05-11): a 4-waypoint route with
    // a short out-and-back between WP2 and WP3 (only ~3m apart along latitude). The
    // U-turn is correctly emitted and follow-up instructions remain correctly aligned —
    // unlike the failing case in diagUturnWaypointBreaksRemap.
    //
    // Goal: identify what is structurally different about this U-turn boundary that
    // lets remapInstructionGeometry succeed where it fails in the broken case.
    @Test
    void diagShortOutAndBackUturnSucceeds() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest req = buildUturnDiagRequest(
                /*w1*/ 61.510674, 23.597011,
                /*w2*/ 61.513734, 23.596761,
                /*w3*/ 61.516034, 23.596418,
                /*w4*/ 61.515663, 23.589621,
                /*seg1Heading=*/ null,
                /*seg2Heading=*/ 18.46348753966555,
                /*seg3Heading=*/ 73.86924874832908);

        System.out.println("\n=================================================================");
        System.out.println("== WORKING U-TURN DIAGNOSTIC (short out-and-back) ==");
        System.out.println("=================================================================");
        analyzeUturnRequest(generator, req, "short-out-and-back");
        System.out.println("\n=================================================================");
        System.out.println("== END WORKING U-TURN DIAGNOSTIC ==");
        System.out.println("=================================================================");
    }

    /**
     * Field report: short single-segment gravel route where the rider observes
     * a 3-way fork (three outbound alternatives) and the route takes the MIDDLE
     * branch — but the produced instruction is "stay left" (KEEP_LEFT). Misleading
     * because "stay left" implies picking the left of two close-together options,
     * not the middle of three.
     *
     * API payload (verbatim):
     *   waypoints:
     *     - id=erg4HmbkGFAjSAiQJTzKc (61.496551, 23.631929)
     *     - id=A76LzfC4Gi2D83v-hurYq (61.49645,   23.628265)
     *   segments: 1 followRoads, profile=gravel
     *   instruction_profile=gravel, locale=fi, snap_preventions=[ferry]
     *
     * Goal of this diagnostic: dump the edge chain and, at each junction, list
     * every outgoing alternative with its angle delta, PH, surface, access, and
     * sign so we can see (a) whether the graph really shows 3 outbound forward
     * branches at the relevant junction, (b) where the route lands among them,
     * and (c) which rule (fork handler / leaving-current-street fallback /
     * reframer Shape 3 upgrade) emitted KEEP_LEFT.
     */
    @Test
    void testFieldReport_stayLeftMidOf3WayFork_61_496_23_631() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("erg4HmbkGFAjSAiQJTzKc", 61.496551, 23.631929);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("A76LzfC4Gi2D83v-hurYq", 61.49645, 23.628265);
        request.setWaypoints(List.of(wp1, wp2));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("erg4HmbkGFAjSAiQJTzKc");
        seg.setEnd("A76LzfC4Gi2D83v-hurYq");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== FIELD REPORT 61_496_23_631 — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        System.out.println("\n--- Polyline (lat, lng) ---");
        for (int i = 0; i < result.polyline.size(); i++) {
            System.out.printf("  [%d] %.7f, %.7f%n", i, result.polyline.getLat(i), result.polyline.getLon(i));
        }

        EnumEncodedValue<RoadClass> rcEnc = encodingManager.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<PredictedHighway> phEnc = encodingManager.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> psEnc = encodingManager.hasEncodedValue(PredictedSurface.KEY)
                ? encodingManager.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class) : null;
        EnumEncodedValue<Surface> surfEnc = encodingManager.hasEncodedValue(Surface.KEY)
                ? encodingManager.getEnumEncodedValue(Surface.KEY, Surface.class) : null;
        BooleanEncodedValue bikeAccessEnc = encodingManager.getBooleanEncodedValue(VehicleAccess.key("bike"));

        GHRequest ghReq = new GHRequest(61.496551, 23.631929, 61.49645, 23.628265).setProfile("gravel");
        ghReq.setPathDetails(List.of("edge_id"));
        ghReq.putHint("instructions", false);
        ghReq.putHint("calc_points", true);
        ghReq.setSnapPreventions(List.of("ferry"));
        GHResponse ghRsp = hopper.route(ghReq);
        assertFalse(ghRsp.hasErrors(), "GH route failed: " + ghRsp.getErrors());

        List<PathDetail> edgeIdDetails = ghRsp.getBest().getPathDetails().get("edge_id");
        assertNotNull(edgeIdDetails);
        System.out.println("\n--- Edge chain (" + edgeIdDetails.size() + " edges) ---");

        EdgeExplorer explorer = baseGraph.createEdgeExplorer();
        int prevNode;
        {
            EdgeIteratorState e0 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(0).getValue(), Integer.MIN_VALUE);
            if (edgeIdDetails.size() == 1) {
                prevNode = e0.getBaseNode();
            } else {
                EdgeIteratorState e1 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(1).getValue(), Integer.MIN_VALUE);
                int b1 = e0.getBaseNode(), a1 = e0.getAdjNode();
                int b2 = e1.getBaseNode(), a2 = e1.getAdjNode();
                int shared;
                if (b1 == b2 || b1 == a2) shared = b1;
                else shared = a1;
                prevNode = (e0.getBaseNode() == shared) ? e0.getAdjNode() : e0.getBaseNode();
            }
        }

        for (int ei = 0; ei < edgeIdDetails.size(); ei++) {
            int edgeId = (Integer) edgeIdDetails.get(ei).getValue();
            EdgeIteratorState raw = baseGraph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            int nextNode = (raw.getBaseNode() == prevNode) ? raw.getAdjNode() : raw.getBaseNode();
            EdgeIteratorState routeEdge = baseGraph.getEdgeIteratorState(edgeId, nextNode);
            int adjNode = routeEdge.getAdjNode();

            RoadClass rc = routeEdge.get(rcEnc);
            PredictedHighway ph = routeEdge.get(phEnc);
            PredictedSurface ps = psEnc != null ? routeEdge.get(psEnc) : null;
            Surface surf = surfEnc != null ? routeEdge.get(surfEnc) : null;
            String name = routeEdge.getName();

            PointList geo = routeEdge.fetchWayGeometry(FetchMode.ALL);
            double startLat = geo.getLat(0), startLon = geo.getLon(0);
            double endLat = geo.getLat(geo.size() - 1), endLon = geo.getLon(geo.size() - 1);

            System.out.printf("%n[edge %d] id=%d base=%d→adj=%d  rc=%s  ph=%s  ps=%s  surf=%s  name=\"%s\"  len=%.1fm%n",
                    ei, edgeId, prevNode, adjNode, rc, ph, ps, surf, name, routeEdge.getDistance());
            System.out.printf("        start=(%.7f, %.7f)  end=(%.7f, %.7f)%n",
                    startLat, startLon, endLat, endLon);

            int gN = geo.size();
            double inFromLat = geo.getLat(gN - 2), inFromLon = geo.getLon(gN - 2);
            double inToLat = geo.getLat(gN - 1), inToLon = geo.getLon(gN - 1);
            double incomingBearing = AngleCalc.ANGLE_CALC.calcOrientation(inFromLat, inFromLon, inToLat, inToLon);

            int routeNextEdgeId = (ei + 1 < edgeIdDetails.size()) ? (Integer) edgeIdDetails.get(ei + 1).getValue() : -1;
            EdgeIterator iter = explorer.setBaseNode(adjNode);
            int altCount = 0;
            List<String> altLines = new ArrayList<>();
            while (iter.next()) {
                if (iter.getEdge() == edgeId) continue;
                altCount++;
                int altEdgeId = iter.getEdge();
                int altAdj = iter.getAdjNode();
                RoadClass altRC = iter.get(rcEnc);
                PredictedHighway altPH = iter.get(phEnc);
                PredictedSurface altPS = psEnc != null ? iter.get(psEnc) : null;
                Surface altSurf = surfEnc != null ? iter.get(surfEnc) : null;
                String altName = iter.getName();
                boolean altAccess = iter.get(bikeAccessEnc);

                PointList altGeo = iter.fetchWayGeometry(FetchMode.ALL);
                double altFromLat = altGeo.getLat(0), altFromLon = altGeo.getLon(0);
                double altToLat = altGeo.getLat(1), altToLon = altGeo.getLon(1);
                double altBearing = AngleCalc.ANGLE_CALC.calcOrientation(altFromLat, altFromLon, altToLat, altToLon);

                double aligned = AngleCalc.ANGLE_CALC.alignOrientation(incomingBearing, altBearing);
                double deltaRad = aligned - incomingBearing;
                double deltaDeg = Math.toDegrees(deltaRad);
                int altSign = InstructionsHelper.calculateSign(inToLat, inToLon, altToLat, altToLon, incomingBearing);

                String tag = (altEdgeId == routeNextEdgeId) ? "ROUTE" : "alt  ";
                altLines.add(String.format(
                        "        %s edge=%d adj=%d  rc=%s ph=%s ps=%s surf=%s access=%s  Δ=%+.1f° sign=%s  name=\"%s\"",
                        tag, altEdgeId, altAdj, altRC, altPH, altPS, altSurf, altAccess,
                        deltaDeg, signName(altSign), altName));
            }
            if (altCount > 0) {
                System.out.printf("    junction at node %d — %d outgoing edge(s) (excluding incoming):%n", adjNode, altCount);
                for (String s : altLines) System.out.println(s);
            } else {
                System.out.printf("    junction at node %d — no alternatives (dead-end / 2-degree node)%n", adjNode);
            }

            prevNode = adjNode;
        }

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        System.out.println("\n========== FIELD REPORT 61_496_23_631 — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);
    }

    // ========================================================================
    // Shape 6 (soft real turn) — TURN_LEFT/RIGHT → TURN_SLIGHT_LEFT/RIGHT
    // ========================================================================

    /**
     * Diagnostic for Shape 6 — user-provided cases.
     *
     * Case A: route 61.469603/23.634817 → 61.469733/23.634276 (gravel)
     *   User: "It has a junction were surface changes." Shape 6's surface guard
     *   was intentionally dropped, so the demote should still fire if other
     *   gates pass (route 40°–55° unanchored or 40°–70° anchored, non-road,
     *   no PH change, forbidden cone empty).
     *
     * Case B: route 61.464976/23.531487 → 61.465163/23.530462 (gravel)
     *   User: "Is a perfect case example."
     */
    @Test
    void testShape6_softRealTurn_userCases() {
        runShape6Case("Case A (61.469603,23.634817 → 61.469733,23.634276)",
                "wpA1", 61.469603, 23.634817,
                "wpA2", 61.469733, 23.634276);
        runShape6Case("Case B (61.464976,23.531487 → 61.465163,23.530462)",
                "wpB1", 61.464976, 23.531487,
                "wpB2", 61.465163, 23.530462);
    }

    private void runShape6Case(String label,
                                String id1, double lat1, double lng1,
                                String id2, double lat2, double lng2) {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(makeWaypoint(id1, lat1, lng1), makeWaypoint(id2, lat2, lng2)));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart(id1);
        seg.setEnd(id2);
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");
        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== SHAPE 6 — " + label + " — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        EnumEncodedValue<RoadClass> rcEnc = encodingManager.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<PredictedHighway> phEnc = encodingManager.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> psEnc = encodingManager.hasEncodedValue(PredictedSurface.KEY)
                ? encodingManager.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class) : null;
        BooleanEncodedValue bikeAccessEnc = encodingManager.getBooleanEncodedValue(VehicleAccess.key("bike"));

        GHRequest ghReq = new GHRequest(lat1, lng1, lat2, lng2).setProfile("gravel");
        ghReq.setPathDetails(List.of("edge_id"));
        ghReq.putHint("instructions", false);
        ghReq.putHint("calc_points", true);
        ghReq.setSnapPreventions(List.of("ferry"));
        GHResponse ghRsp = hopper.route(ghReq);
        assertFalse(ghRsp.hasErrors(), "GH route failed: " + ghRsp.getErrors());

        List<PathDetail> edgeIdDetails = ghRsp.getBest().getPathDetails().get("edge_id");
        assertNotNull(edgeIdDetails);
        System.out.println("\n--- Edge chain with junction alts (" + edgeIdDetails.size() + " edges) ---");

        EdgeExplorer explorer = baseGraph.createEdgeExplorer();
        int prevNode;
        {
            EdgeIteratorState e0 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(0).getValue(), Integer.MIN_VALUE);
            if (edgeIdDetails.size() == 1) {
                prevNode = e0.getBaseNode();
            } else {
                EdgeIteratorState e1 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(1).getValue(), Integer.MIN_VALUE);
                int b1 = e0.getBaseNode(), a1 = e0.getAdjNode();
                int b2 = e1.getBaseNode(), a2 = e1.getAdjNode();
                int shared;
                if (b1 == b2 || b1 == a2) shared = b1;
                else shared = a1;
                prevNode = (e0.getBaseNode() == shared) ? e0.getAdjNode() : e0.getBaseNode();
            }
        }

        for (int ei = 0; ei < edgeIdDetails.size(); ei++) {
            int edgeId = (Integer) edgeIdDetails.get(ei).getValue();
            EdgeIteratorState raw = baseGraph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            int nextNode = (raw.getBaseNode() == prevNode) ? raw.getAdjNode() : raw.getBaseNode();
            EdgeIteratorState routeEdge = baseGraph.getEdgeIteratorState(edgeId, nextNode);
            int adjNode = routeEdge.getAdjNode();

            RoadClass rc = routeEdge.get(rcEnc);
            PredictedHighway ph = routeEdge.get(phEnc);
            PredictedSurface ps = psEnc != null ? routeEdge.get(psEnc) : null;
            String name = routeEdge.getName();

            PointList edgeGeo = routeEdge.fetchWayGeometry(FetchMode.ALL);
            double inFromLat = edgeGeo.getLat(edgeGeo.size() - 2);
            double inFromLon = edgeGeo.getLon(edgeGeo.size() - 2);
            double inToLat = edgeGeo.getLat(edgeGeo.size() - 1);
            double inToLon = edgeGeo.getLon(edgeGeo.size() - 1);
            double incomingBearing = AngleCalc.ANGLE_CALC.calcOrientation(inFromLat, inFromLon, inToLat, inToLon);
            System.out.printf("  [%d] edge=%d adj=%d rc=%s ph=%s ps=%s name=\"%s\"%n",
                    ei, edgeId, adjNode, rc, ph, ps, name);

            int routeNextEdgeId = (ei + 1 < edgeIdDetails.size()) ? (Integer) edgeIdDetails.get(ei + 1).getValue() : -1;
            EdgeIterator iter = explorer.setBaseNode(adjNode);
            int altCount = 0;
            List<String> altLines = new ArrayList<>();
            while (iter.next()) {
                if (iter.getEdge() == edgeId) continue;
                altCount++;
                int altEdgeId = iter.getEdge();
                RoadClass altRC = iter.get(rcEnc);
                PredictedHighway altPH = iter.get(phEnc);
                PredictedSurface altPS = psEnc != null ? iter.get(psEnc) : null;
                String altName = iter.getName();
                boolean altAccess = iter.get(bikeAccessEnc);

                PointList altGeo = iter.fetchWayGeometry(FetchMode.ALL);
                double altFromLat = altGeo.getLat(0), altFromLon = altGeo.getLon(0);
                double altToLat = altGeo.getLat(1), altToLon = altGeo.getLon(1);
                double altBearing = AngleCalc.ANGLE_CALC.calcOrientation(altFromLat, altFromLon, altToLat, altToLon);
                double aligned = AngleCalc.ANGLE_CALC.alignOrientation(incomingBearing, altBearing);
                double deltaDeg = Math.toDegrees(aligned - incomingBearing);
                int altSign = InstructionsHelper.calculateSign(inToLat, inToLon, altToLat, altToLon, incomingBearing);

                String tag = (altEdgeId == routeNextEdgeId) ? "ROUTE" : "alt  ";
                altLines.add(String.format(
                        "        %s edge=%d  rc=%s ph=%s ps=%s access=%s  Δ=%+.1f° sign=%s  name=\"%s\"",
                        tag, altEdgeId, altRC, altPH, altPS, altAccess,
                        deltaDeg, signName(altSign), altName));
            }
            if (altCount > 0) {
                for (String s : altLines) System.out.println(s);
            }

            prevNode = adjNode;
        }
    }

    // ========================================================================
    // Missing left turn: Kalliojärventie → Houkkalammintie
    // ========================================================================

    /**
     * Diagnostic for a reported missing left-turn instruction.
     *
     * API payload:
     *   waypoints: (61.39616, 23.708264) → (61.394999, 23.709944)
     *   single segment, type=followRoads, profile=gravel
     *   instruction_profile=gravel, locale=fi, snap_preventions=["ferry"]
     *
     * Reported issue: the route turns LEFT from Kalliojärventie onto
     * Houkkalammintie, yet no instruction is emitted for that turn.
     *
     * Goal: trace which rule (S1/S2/S3/S4/S5/S6, fork handler,
     * leaving-current-street fallback, F1/F2, or reframer) is responsible
     * for the missing turn, by dumping:
     *   - Stage 1 instructions with full extraInfo
     *   - Edge chain with junction alts (PH, surface, angle, name, access)
     *   - Post-processed instructions
     */
    @Test
    void testFieldReport_missingLeftKalliojarventieToHoukkalammintie_61_396_23_708() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        double startLat = 61.39616, startLng = 23.708264;
        double endLat = 61.394999, endLng = 23.709944;

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("M2YfKzGrMyoDYuh26MJce", startLat, startLng);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("92YYJR6rTV2tpPOK09XYV", endLat, endLng);
        request.setWaypoints(List.of(wp1, wp2));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("M2YfKzGrMyoDYuh26MJce");
        seg.setEnd("92YYJR6rTV2tpPOK09XYV");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== FIELD REPORT 61_396_23_708 Kalliojarventie→Houkkalammintie — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        System.out.println("\n--- Polyline (lat, lng) ---");
        for (int i = 0; i < result.polyline.size(); i++) {
            System.out.printf("  [%d] %.7f, %.7f%n", i, result.polyline.getLat(i), result.polyline.getLon(i));
        }

        EnumEncodedValue<RoadClass> rcEnc = encodingManager.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<PredictedHighway> phEnc = encodingManager.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> psEnc = encodingManager.hasEncodedValue(PredictedSurface.KEY)
                ? encodingManager.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class) : null;
        EnumEncodedValue<Surface> surfEnc = encodingManager.hasEncodedValue(Surface.KEY)
                ? encodingManager.getEnumEncodedValue(Surface.KEY, Surface.class) : null;
        BooleanEncodedValue bikeAccessEnc = encodingManager.getBooleanEncodedValue(VehicleAccess.key("bike"));

        GHRequest ghReq = new GHRequest(startLat, startLng, endLat, endLng).setProfile("gravel");
        ghReq.setPathDetails(List.of("edge_id"));
        ghReq.putHint("instructions", false);
        ghReq.putHint("calc_points", true);
        ghReq.setSnapPreventions(List.of("ferry"));
        GHResponse ghRsp = hopper.route(ghReq);
        assertFalse(ghRsp.hasErrors(), "GH route failed: " + ghRsp.getErrors());

        List<PathDetail> edgeIdDetails = ghRsp.getBest().getPathDetails().get("edge_id");
        assertNotNull(edgeIdDetails);
        System.out.println("\n--- Edge chain (" + edgeIdDetails.size() + " edges) ---");

        EdgeExplorer explorer = baseGraph.createEdgeExplorer();
        int prevNode;
        {
            EdgeIteratorState e0 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(0).getValue(), Integer.MIN_VALUE);
            if (edgeIdDetails.size() == 1) {
                prevNode = e0.getBaseNode();
            } else {
                EdgeIteratorState e1 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(1).getValue(), Integer.MIN_VALUE);
                int b1 = e0.getBaseNode(), a1 = e0.getAdjNode();
                int b2 = e1.getBaseNode(), a2 = e1.getAdjNode();
                int shared;
                if (b1 == b2 || b1 == a2) shared = b1;
                else shared = a1;
                prevNode = (e0.getBaseNode() == shared) ? e0.getAdjNode() : e0.getBaseNode();
            }
        }

        for (int ei = 0; ei < edgeIdDetails.size(); ei++) {
            int edgeId = (Integer) edgeIdDetails.get(ei).getValue();
            EdgeIteratorState raw = baseGraph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            int nextNode = (raw.getBaseNode() == prevNode) ? raw.getAdjNode() : raw.getBaseNode();
            EdgeIteratorState routeEdge = baseGraph.getEdgeIteratorState(edgeId, nextNode);
            int adjNode = routeEdge.getAdjNode();

            RoadClass rc = routeEdge.get(rcEnc);
            PredictedHighway ph = routeEdge.get(phEnc);
            PredictedSurface ps = psEnc != null ? routeEdge.get(psEnc) : null;
            Surface surf = surfEnc != null ? routeEdge.get(surfEnc) : null;
            String name = routeEdge.getName();

            PointList geo = routeEdge.fetchWayGeometry(FetchMode.ALL);
            double startGeoLat = geo.getLat(0), startGeoLon = geo.getLon(0);
            double endGeoLat = geo.getLat(geo.size() - 1), endGeoLon = geo.getLon(geo.size() - 1);

            System.out.printf("%n[edge %d] id=%d base=%d→adj=%d  rc=%s  ph=%s  ps=%s  surf=%s  name=\"%s\"  len=%.1fm%n",
                    ei, edgeId, prevNode, adjNode, rc, ph, ps, surf, name, routeEdge.getDistance());
            System.out.printf("        start=(%.7f, %.7f)  end=(%.7f, %.7f)%n",
                    startGeoLat, startGeoLon, endGeoLat, endGeoLon);

            int gN = geo.size();
            double inFromLat = geo.getLat(gN - 2), inFromLon = geo.getLon(gN - 2);
            double inToLat = geo.getLat(gN - 1), inToLon = geo.getLon(gN - 1);
            double incomingBearing = AngleCalc.ANGLE_CALC.calcOrientation(inFromLat, inFromLon, inToLat, inToLon);

            int routeNextEdgeId = (ei + 1 < edgeIdDetails.size()) ? (Integer) edgeIdDetails.get(ei + 1).getValue() : -1;
            EdgeIterator iter = explorer.setBaseNode(adjNode);
            int altCount = 0;
            List<String> altLines = new ArrayList<>();
            while (iter.next()) {
                if (iter.getEdge() == edgeId) continue;
                altCount++;
                int altEdgeId = iter.getEdge();
                int altAdj = iter.getAdjNode();
                RoadClass altRC = iter.get(rcEnc);
                PredictedHighway altPH = iter.get(phEnc);
                PredictedSurface altPS = psEnc != null ? iter.get(psEnc) : null;
                Surface altSurf = surfEnc != null ? iter.get(surfEnc) : null;
                String altName = iter.getName();
                boolean altAccess = iter.get(bikeAccessEnc);

                PointList altGeo = iter.fetchWayGeometry(FetchMode.ALL);
                double altFromLat = altGeo.getLat(0), altFromLon = altGeo.getLon(0);
                double altToLat = altGeo.getLat(1), altToLon = altGeo.getLon(1);
                double altBearing = AngleCalc.ANGLE_CALC.calcOrientation(altFromLat, altFromLon, altToLat, altToLon);

                double aligned = AngleCalc.ANGLE_CALC.alignOrientation(incomingBearing, altBearing);
                double deltaRad = aligned - incomingBearing;
                double deltaDeg = Math.toDegrees(deltaRad);
                int altSign = InstructionsHelper.calculateSign(inToLat, inToLon, altToLat, altToLon, incomingBearing);

                String tag = (altEdgeId == routeNextEdgeId) ? "ROUTE" : "alt  ";
                altLines.add(String.format(
                        "        %s edge=%d adj=%d  rc=%s ph=%s ps=%s surf=%s access=%s  Δ=%+.1f° sign=%s  name=\"%s\"",
                        tag, altEdgeId, altAdj, altRC, altPH, altPS, altSurf, altAccess,
                        deltaDeg, signName(altSign), altName));
            }
            if (altCount > 0) {
                System.out.printf("    junction at node %d — %d outgoing edge(s) (excluding incoming):%n", adjNode, altCount);
                for (String s : altLines) System.out.println(s);
            } else {
                System.out.printf("    junction at node %d — no alternatives (dead-end / 2-degree node)%n", adjNode);
            }

            prevNode = adjNode;
        }

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        System.out.println("\n========== FIELD REPORT 61_396_23_708 Kalliojarventie→Houkkalammintie — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);
    }

    /**
     * Diagnostic: short gravel route between two API-supplied waypoints near
     * Ylöjärvi/Tampere where a clear Y-fork along the way does not generate
     * any instruction.
     *
     * API payload (gravel, fi, ferry preventions):
     *   start: (61.552818, 23.524485)  id=r8Xh-ur-vUyRgXxMn0DSc
     *   end:   (61.552495, 23.520808)  id=jvvCsXiAVNs7PeNV9ypO_
     *
     * Reported: there is a clearly visible Y-fork on this segment but no
     * instruction is emitted. Goal: identify which rule (S1/S2/S3/S4/S5/S6,
     * fork handler, leaving-current-street fallback, F1/F2, or reframer)
     * silences it, by dumping:
     *   - Stage 1 instructions with full extraInfo
     *   - Edge chain with per-junction alternatives (PH, surface, angle, name, access)
     *   - Post-processed instructions
     */
    @Test
    void testFieldReport_missingYForkGravel_61_5528_23_5245() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        double startLat = 61.552818, startLng = 23.524485;
        double endLat = 61.552495, endLng = 23.520808;

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("r8Xh-ur-vUyRgXxMn0DSc", startLat, startLng);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("jvvCsXiAVNs7PeNV9ypO_", endLat, endLng);
        request.setWaypoints(List.of(wp1, wp2));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("r8Xh-ur-vUyRgXxMn0DSc");
        seg.setEnd("jvvCsXiAVNs7PeNV9ypO_");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== FIELD REPORT 61_5528_23_5245 missing Y-fork (gravel) — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        System.out.println("\n--- Polyline (lat, lng) ---");
        for (int i = 0; i < result.polyline.size(); i++) {
            System.out.printf("  [%d] %.7f, %.7f%n", i, result.polyline.getLat(i), result.polyline.getLon(i));
        }

        EnumEncodedValue<RoadClass> rcEnc = encodingManager.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<PredictedHighway> phEnc = encodingManager.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> psEnc = encodingManager.hasEncodedValue(PredictedSurface.KEY)
                ? encodingManager.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class) : null;
        EnumEncodedValue<Surface> surfEnc = encodingManager.hasEncodedValue(Surface.KEY)
                ? encodingManager.getEnumEncodedValue(Surface.KEY, Surface.class) : null;
        BooleanEncodedValue bikeAccessEnc = encodingManager.getBooleanEncodedValue(VehicleAccess.key("bike"));

        GHRequest ghReq = new GHRequest(startLat, startLng, endLat, endLng).setProfile("gravel");
        ghReq.setPathDetails(List.of("edge_id"));
        ghReq.putHint("instructions", false);
        ghReq.putHint("calc_points", true);
        ghReq.setSnapPreventions(List.of("ferry"));
        GHResponse ghRsp = hopper.route(ghReq);
        assertFalse(ghRsp.hasErrors(), "GH route failed: " + ghRsp.getErrors());

        List<PathDetail> edgeIdDetails = ghRsp.getBest().getPathDetails().get("edge_id");
        assertNotNull(edgeIdDetails);
        System.out.println("\n--- Edge chain (" + edgeIdDetails.size() + " edges) ---");

        EdgeExplorer explorer = baseGraph.createEdgeExplorer();
        int prevNode;
        {
            EdgeIteratorState e0 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(0).getValue(), Integer.MIN_VALUE);
            if (edgeIdDetails.size() == 1) {
                prevNode = e0.getBaseNode();
            } else {
                EdgeIteratorState e1 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(1).getValue(), Integer.MIN_VALUE);
                int b1 = e0.getBaseNode(), a1 = e0.getAdjNode();
                int b2 = e1.getBaseNode(), a2 = e1.getAdjNode();
                int shared;
                if (b1 == b2 || b1 == a2) shared = b1;
                else shared = a1;
                prevNode = (e0.getBaseNode() == shared) ? e0.getAdjNode() : e0.getBaseNode();
            }
        }

        for (int ei = 0; ei < edgeIdDetails.size(); ei++) {
            int edgeId = (Integer) edgeIdDetails.get(ei).getValue();
            EdgeIteratorState raw = baseGraph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            int nextNode = (raw.getBaseNode() == prevNode) ? raw.getAdjNode() : raw.getBaseNode();
            EdgeIteratorState routeEdge = baseGraph.getEdgeIteratorState(edgeId, nextNode);
            int adjNode = routeEdge.getAdjNode();

            RoadClass rc = routeEdge.get(rcEnc);
            PredictedHighway ph = routeEdge.get(phEnc);
            PredictedSurface ps = psEnc != null ? routeEdge.get(psEnc) : null;
            Surface surf = surfEnc != null ? routeEdge.get(surfEnc) : null;
            String name = routeEdge.getName();

            PointList geo = routeEdge.fetchWayGeometry(FetchMode.ALL);
            double startGeoLat = geo.getLat(0), startGeoLon = geo.getLon(0);
            double endGeoLat = geo.getLat(geo.size() - 1), endGeoLon = geo.getLon(geo.size() - 1);

            System.out.printf("%n[edge %d] id=%d base=%d→adj=%d  rc=%s  ph=%s  ps=%s  surf=%s  name=\"%s\"  len=%.1fm%n",
                    ei, edgeId, prevNode, adjNode, rc, ph, ps, surf, name, routeEdge.getDistance());
            System.out.printf("        start=(%.7f, %.7f)  end=(%.7f, %.7f)%n",
                    startGeoLat, startGeoLon, endGeoLat, endGeoLon);

            int gN = geo.size();
            double inFromLat = geo.getLat(gN - 2), inFromLon = geo.getLon(gN - 2);
            double inToLat = geo.getLat(gN - 1), inToLon = geo.getLon(gN - 1);
            double incomingBearing = AngleCalc.ANGLE_CALC.calcOrientation(inFromLat, inFromLon, inToLat, inToLon);

            int routeNextEdgeId = (ei + 1 < edgeIdDetails.size()) ? (Integer) edgeIdDetails.get(ei + 1).getValue() : -1;
            EdgeIterator iter = explorer.setBaseNode(adjNode);
            int altCount = 0;
            List<String> altLines = new ArrayList<>();
            while (iter.next()) {
                if (iter.getEdge() == edgeId) continue;
                altCount++;
                int altEdgeId = iter.getEdge();
                int altAdj = iter.getAdjNode();
                RoadClass altRC = iter.get(rcEnc);
                PredictedHighway altPH = iter.get(phEnc);
                PredictedSurface altPS = psEnc != null ? iter.get(psEnc) : null;
                Surface altSurf = surfEnc != null ? iter.get(surfEnc) : null;
                String altName = iter.getName();
                boolean altAccess = iter.get(bikeAccessEnc);

                PointList altGeo = iter.fetchWayGeometry(FetchMode.ALL);
                double altFromLat = altGeo.getLat(0), altFromLon = altGeo.getLon(0);
                double altToLat = altGeo.getLat(1), altToLon = altGeo.getLon(1);
                double altBearing = AngleCalc.ANGLE_CALC.calcOrientation(altFromLat, altFromLon, altToLat, altToLon);

                double aligned = AngleCalc.ANGLE_CALC.alignOrientation(incomingBearing, altBearing);
                double deltaRad = aligned - incomingBearing;
                double deltaDeg = Math.toDegrees(deltaRad);
                int altSign = InstructionsHelper.calculateSign(inToLat, inToLon, altToLat, altToLon, incomingBearing);

                String tag = (altEdgeId == routeNextEdgeId) ? "ROUTE" : "alt  ";
                altLines.add(String.format(
                        "        %s edge=%d adj=%d  rc=%s ph=%s ps=%s surf=%s access=%s  Δ=%+.1f° sign=%s  name=\"%s\"",
                        tag, altEdgeId, altAdj, altRC, altPH, altPS, altSurf, altAccess,
                        deltaDeg, signName(altSign), altName));
            }
            if (altCount > 0) {
                System.out.printf("    junction at node %d — %d outgoing edge(s) (excluding incoming):%n", adjNode, altCount);
                for (String s : altLines) System.out.println(s);
            } else {
                System.out.printf("    junction at node %d — no alternatives (dead-end / 2-degree node)%n", adjNode);
            }

            prevNode = adjNode;
        }

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        System.out.println("\n========== FIELD REPORT 61_5528_23_5245 missing Y-fork (gravel) — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);
    }

    /**
     * Field report: instruction is "loiva oikea" (TURN_SLIGHT_RIGHT) but the route
     * is near-straight at the junction while another alt is ALSO slight-right. The
     * rider can't tell which "slight right" is meant; guidance should just say
     * "straight" (CONTINUE_ON_STREET) since the route is the near-straight option.
     *
     * API payload (verbatim):
     *   waypoints:
     *     - id=Bgf0UtqMMxD9rIij2mFmP (61.52782, 23.626389)
     *     - id=EtdDLFhR6IHLz6UPQT8hx (61.527685, 23.627943)
     *   segments: 1 followRoads, profile=gravel
     *   instruction_profile=gravel, locale=fi, snap_preventions=[ferry]
     *
     * Dumps each routed junction's alternatives (PH, surface, angle, sign) so we
     * can see the route's angle and the competing slight-right alt, and which
     * reframer shape fires (or fails to fire).
     */
    @Test
    void testLoivaOikeaShouldBeStraight_61_527_23_626() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        TrailmapInstructionRequest.Waypoint wp1 = makeWaypoint("Bgf0UtqMMxD9rIij2mFmP", 61.52782, 23.626389);
        TrailmapInstructionRequest.Waypoint wp2 = makeWaypoint("EtdDLFhR6IHLz6UPQT8hx", 61.527685, 23.627943);
        request.setWaypoints(List.of(wp1, wp2));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("Bgf0UtqMMxD9rIij2mFmP");
        seg.setEnd("EtdDLFhR6IHLz6UPQT8hx");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertNotNull(result.instructions);
        assertTrue(result.instructions.size() > 0, "Should produce instructions");

        System.out.println("\n========== LOIVA OIKEA SHOULD BE STRAIGHT — BEFORE POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);

        System.out.println("\n--- Polyline (lat, lng) ---");
        for (int i = 0; i < result.polyline.size(); i++) {
            System.out.printf("  [%d] %.7f, %.7f%n", i, result.polyline.getLat(i), result.polyline.getLon(i));
        }

        // ---- Walk the routed edge sequence and dump per-junction alternatives ----
        EnumEncodedValue<RoadClass> rcEnc = encodingManager.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<PredictedHighway> phEnc = encodingManager.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> psEnc = encodingManager.hasEncodedValue(PredictedSurface.KEY)
                ? encodingManager.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class) : null;
        EnumEncodedValue<Surface> surfEnc = encodingManager.hasEncodedValue(Surface.KEY)
                ? encodingManager.getEnumEncodedValue(Surface.KEY, Surface.class) : null;
        BooleanEncodedValue bikeAccessEnc = encodingManager.getBooleanEncodedValue(VehicleAccess.key("bike"));

        GHRequest ghReq = new GHRequest(61.52782, 23.626389, 61.527685, 23.627943).setProfile("gravel");
        ghReq.setPathDetails(List.of("edge_id"));
        ghReq.putHint("instructions", false);
        ghReq.putHint("calc_points", true);
        ghReq.setSnapPreventions(List.of("ferry"));
        GHResponse ghRsp = hopper.route(ghReq);
        assertFalse(ghRsp.hasErrors(), "GH route failed: " + ghRsp.getErrors());

        List<PathDetail> edgeIdDetails = ghRsp.getBest().getPathDetails().get("edge_id");
        assertNotNull(edgeIdDetails);
        System.out.println("\n--- Edge chain (" + edgeIdDetails.size() + " edges) ---");

        EdgeExplorer explorer = baseGraph.createEdgeExplorer();
        int prevNode;
        {
            EdgeIteratorState e0 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(0).getValue(), Integer.MIN_VALUE);
            if (edgeIdDetails.size() == 1) {
                prevNode = e0.getBaseNode();
            } else {
                EdgeIteratorState e1 = baseGraph.getEdgeIteratorState((Integer) edgeIdDetails.get(1).getValue(), Integer.MIN_VALUE);
                int b1 = e0.getBaseNode(), a1 = e0.getAdjNode();
                int b2 = e1.getBaseNode(), a2 = e1.getAdjNode();
                int shared;
                if (b1 == b2 || b1 == a2) shared = b1;
                else shared = a1;
                prevNode = (e0.getBaseNode() == shared) ? e0.getAdjNode() : e0.getBaseNode();
            }
        }

        for (int ei = 0; ei < edgeIdDetails.size(); ei++) {
            int edgeId = (Integer) edgeIdDetails.get(ei).getValue();
            EdgeIteratorState raw = baseGraph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            int nextNode = (raw.getBaseNode() == prevNode) ? raw.getAdjNode() : raw.getBaseNode();
            EdgeIteratorState routeEdge = baseGraph.getEdgeIteratorState(edgeId, nextNode);
            int adjNode = routeEdge.getAdjNode();

            RoadClass rc = routeEdge.get(rcEnc);
            PredictedHighway ph = routeEdge.get(phEnc);
            PredictedSurface ps = psEnc != null ? routeEdge.get(psEnc) : null;
            Surface surf = surfEnc != null ? routeEdge.get(surfEnc) : null;
            String name = routeEdge.getName();

            PointList geo = routeEdge.fetchWayGeometry(FetchMode.ALL);
            double startLat = geo.getLat(0), startLon = geo.getLon(0);
            double endLat = geo.getLat(geo.size() - 1), endLon = geo.getLon(geo.size() - 1);

            System.out.printf("%n[edge %d] id=%d base=%d→adj=%d  rc=%s  ph=%s  ps=%s  surf=%s  name=\"%s\"  len=%.1fm%n",
                    ei, edgeId, prevNode, adjNode, rc, ph, ps, surf, name, routeEdge.getDistance());
            System.out.printf("        start=(%.7f, %.7f)  end=(%.7f, %.7f)%n",
                    startLat, startLon, endLat, endLon);

            int gN = geo.size();
            double inFromLat = geo.getLat(gN - 2), inFromLon = geo.getLon(gN - 2);
            double inToLat = geo.getLat(gN - 1), inToLon = geo.getLon(gN - 1);
            double incomingBearing = AngleCalc.ANGLE_CALC.calcOrientation(inFromLat, inFromLon, inToLat, inToLon);

            int routeNextEdgeId = (ei + 1 < edgeIdDetails.size()) ? (Integer) edgeIdDetails.get(ei + 1).getValue() : -1;
            EdgeIterator iter = explorer.setBaseNode(adjNode);
            int altCount = 0;
            List<String> altLines = new ArrayList<>();
            while (iter.next()) {
                if (iter.getEdge() == edgeId) continue;
                altCount++;
                int altEdgeId = iter.getEdge();
                int altAdj = iter.getAdjNode();
                RoadClass altRC = iter.get(rcEnc);
                PredictedHighway altPH = iter.get(phEnc);
                PredictedSurface altPS = psEnc != null ? iter.get(psEnc) : null;
                Surface altSurf = surfEnc != null ? iter.get(surfEnc) : null;
                String altName = iter.getName();
                boolean altAccess = iter.get(bikeAccessEnc);

                PointList altGeo = iter.fetchWayGeometry(FetchMode.ALL);
                double altFromLat = altGeo.getLat(0), altFromLon = altGeo.getLon(0);
                double altToLat = altGeo.getLat(1), altToLon = altGeo.getLon(1);
                double altBearing = AngleCalc.ANGLE_CALC.calcOrientation(altFromLat, altFromLon, altToLat, altToLon);

                double aligned = AngleCalc.ANGLE_CALC.alignOrientation(incomingBearing, altBearing);
                double deltaRad = aligned - incomingBearing;
                double deltaDeg = Math.toDegrees(deltaRad);
                int altSign = InstructionsHelper.calculateSign(inToLat, inToLon, altToLat, altToLon, incomingBearing);

                String tag = (altEdgeId == routeNextEdgeId) ? "ROUTE" : "alt  ";
                altLines.add(String.format(
                        "        %s edge=%d adj=%d  rc=%s ph=%s ps=%s surf=%s access=%s  Δ=%+.1f° sign=%s  name=\"%s\"",
                        tag, altEdgeId, altAdj, altRC, altPH, altPS, altSurf, altAccess,
                        deltaDeg, signName(altSign), altName));
            }
            if (altCount > 0) {
                System.out.printf("    junction at node %d — %d outgoing edge(s) (excluding incoming):%n", adjNode, altCount);
                for (String s : altLines) System.out.println(s);
            } else {
                System.out.printf("    junction at node %d — no alternatives (dead-end / 2-degree node)%n", adjNode);
            }

            prevNode = adjNode;
        }

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        System.out.println("\n========== LOIVA OIKEA SHOULD BE STRAIGHT — AFTER POST-PROCESSING ==========");
        System.out.println("Instructions: " + result.instructions.size());
        printInstructionsDetailed(result);
    }

    // ------------------------------------------------------------------
    // DIAGNOSTIC: ~3x detour from server-side heading chaining
    // 4 waypoints / 3 segments, gravel, snap_preventions=[ferry], locale fi.
    // Client renders ~300m route with 2 short stubs at the middle waypoints.
    // Server /instructions returns ~900m: leaving wp2 it continues in the
    // INBOUND heading (chained) instead of turning back as a stub.
    // ------------------------------------------------------------------
    private static double polylineDist(PointList p) {
        double d = 0;
        for (int i = 0; i < p.size() - 1; i++) {
            d += DistanceCalcEarth.DIST_EARTH.calcDist(
                    p.getLat(i), p.getLon(i), p.getLat(i + 1), p.getLon(i + 1));
        }
        return d;
    }

    /** Route one segment via plain GH exactly like a single GHRequest. */
    private ResponsePath routeSegPlain(double aLat, double aLng, double bLat, double bLng,
                                       String profile, Double heading, Double headingPenalty,
                                       List<String> snapPreventions) {
        GHRequest req = new GHRequest();
        req.addPoint(new GHPoint(aLat, aLng));
        req.addPoint(new GHPoint(bLat, bLng));
        req.setProfile(profile);
        req.setPathDetails(List.of("edge_id"));
        req.putHint("instructions", false);
        req.putHint("calc_points", true);
        if (snapPreventions != null && !snapPreventions.isEmpty()) {
            req.setSnapPreventions(snapPreventions);
        }
        if (headingPenalty != null) {
            req.putHint("heading_penalty", headingPenalty);
        }
        if (heading != null && !heading.isNaN()) {
            req.setHeadings(List.of(heading, Double.NaN));
        }
        GHResponse rsp = hopper.route(req);
        if (rsp.hasErrors()) {
            throw new IllegalStateException("route failed: " + rsp.getErrors());
        }
        return rsp.getBest();
    }

    @Test
    void diagDetourFromHeadingChaining_61_550_23_554() {
        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();

        // Exact payload coordinates
        double w1Lat = 61.550574, w1Lng = 23.554379; // 6srnVdr4KDPqjmmxOcIj0
        double w2Lat = 61.550035, w2Lng = 23.552747; // JItIt-N00nnKKyU95Lfb6
        double w3Lat = 61.549208, w3Lng = 23.55411;  // K6BGbecDYbDyzPUi86yjN
        double w4Lat = 61.549411, w4Lng = 23.554461; // AoTFlcVcmn-INWe3W6UpG
        double seg3Heading = 146.6696150702877;
        double seg3Penalty = 60;
        String profile = "gravel";
        List<String> snapPrev = List.of("ferry");

        // === 1) Full generator result (the server /instructions path) ===
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("6srnVdr4KDPqjmmxOcIj0", w1Lat, w1Lng),
                makeWaypoint("JItIt-N00nnKKyU95Lfb6", w2Lat, w2Lng),
                makeWaypoint("K6BGbecDYbDyzPUi86yjN", w3Lat, w3Lng),
                makeWaypoint("AoTFlcVcmn-INWe3W6UpG", w4Lat, w4Lng)));

        TrailmapInstructionRequest.Segment s1 = new TrailmapInstructionRequest.Segment();
        s1.setStart("6srnVdr4KDPqjmmxOcIj0"); s1.setEnd("JItIt-N00nnKKyU95Lfb6");
        s1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS); s1.setProfile(profile);

        TrailmapInstructionRequest.Segment s2 = new TrailmapInstructionRequest.Segment();
        s2.setStart("JItIt-N00nnKKyU95Lfb6"); s2.setEnd("K6BGbecDYbDyzPUi86yjN");
        s2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS); s2.setProfile(profile);
        // NOTE: seg2 has NO initial_heading and NO heading_penalty in the payload.

        TrailmapInstructionRequest.Segment s3 = new TrailmapInstructionRequest.Segment();
        s3.setStart("K6BGbecDYbDyzPUi86yjN"); s3.setEnd("AoTFlcVcmn-INWe3W6UpG");
        s3.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS); s3.setProfile(profile);
        s3.setInitialHeading(seg3Heading); s3.setHeadingPenalty(seg3Penalty);

        request.setSegments(List.of(s1, s2, s3));
        request.setInstructionProfile(profile);
        request.setLocale("fi");
        request.setSnapPreventions(snapPrev);

        RouteInstructionGenerator generator = new RouteInstructionGenerator(
                hopper, baseGraph, encodingManager, translationMap);
        RouteInstructionGenerator.Result result = generator.generate(request);

        System.out.println("\n========== [GENERATOR] full /instructions result ==========");
        double genTotal = polylineDist(result.polyline);
        System.out.printf("Generator polyline: %d points, total length = %.1f m%n",
                result.polyline.size(), genTotal);
        System.out.println("Instructions (" + result.instructions.size() + "):");
        printInstructionsDetailed(result);

        // === 2) CLIENT-style per-segment routing (each segment standalone) ===
        // seg1/seg2: own initial_heading = none -> no heading.  seg3: 146.67 / penalty 60.
        System.out.println("\n========== [CLIENT] per-segment, own headings only, no chaining ==========");
        ResponsePath c1 = routeSegPlain(w1Lat, w1Lng, w2Lat, w2Lng, profile, null, null, snapPrev);
        ResponsePath c2 = routeSegPlain(w2Lat, w2Lng, w3Lat, w3Lng, profile, null, null, snapPrev);
        ResponsePath c3 = routeSegPlain(w3Lat, w3Lng, w4Lat, w4Lng, profile, seg3Heading, seg3Penalty, snapPrev);
        double cd1 = c1.getDistance(), cd2 = c2.getDistance(), cd3 = c3.getDistance();
        System.out.printf("  client seg1 dist = %.1f m (%d pts)%n", cd1, c1.getPoints().size());
        System.out.printf("  client seg2 dist = %.1f m (%d pts)   <-- expected ~stub%n", cd2, c2.getPoints().size());
        System.out.printf("  client seg3 dist = %.1f m (%d pts)%n", cd3, c3.getPoints().size());
        System.out.printf("  CLIENT TOTAL = %.1f m%n", cd1 + cd2 + cd3);

        // === 3) GENERATOR-style chained routing, reproduced standalone ===
        // Chain heading + start override from previous segment's polyline tail.
        System.out.println("\n========== [SERVER-CHAINED] reproduce generator chaining per-segment ==========");
        // seg1: no chaining yet (first segment), own heading = none
        ResponsePath g1 = routeSegPlain(w1Lat, w1Lng, w2Lat, w2Lng, profile, null, null, snapPrev);
        PointList g1p = g1.getPoints();
        double g1EndLat = g1p.getLat(g1p.size() - 1), g1EndLon = g1p.getLon(g1p.size() - 1);
        double inboundAz = AngleCalc.ANGLE_CALC.calcAzimuth(
                g1p.getLat(g1p.size() - 2), g1p.getLon(g1p.size() - 2), g1EndLat, g1EndLon);
        System.out.printf("  seg1 routed dist = %.1f m, snapped end = (%.6f, %.6f)%n",
                g1.getDistance(), g1EndLat, g1EndLon);
        System.out.printf("  --> INBOUND azimuth into wp2 (last 2 polyline pts) = %.2f deg%n", inboundAz);
        System.out.printf("  --> this azimuth is CHAINED as seg2 start heading, and seg2 start coord%n"
                + "      is OVERRIDDEN to seg1 snapped end (%.6f, %.6f)%n", g1EndLat, g1EndLon);

        // seg2: chained heading = inboundAz, start override = seg1 snapped end, NO explicit penalty
        ResponsePath g2 = routeSegPlain(g1EndLat, g1EndLon, w3Lat, w3Lng, profile, inboundAz, null, snapPrev);
        PointList g2p = g2.getPoints();
        double g2EndLat = g2p.getLat(g2p.size() - 1), g2EndLon = g2p.getLon(g2p.size() - 1);
        System.out.printf("  seg2 CHAINED dist = %.1f m (%d pts)   <-- expected BLOW-UP%n",
                g2.getDistance(), g2p.size());
        double chainAz2 = AngleCalc.ANGLE_CALC.calcAzimuth(
                g2p.getLat(g2p.size() - 2), g2p.getLon(g2p.size() - 2), g2EndLat, g2EndLon);

        // seg3: server applies own initial_heading (146.67) since seg3 has one... but note the
        // generator code is `heading = nextHeading != null ? nextHeading : section.initialHeading`,
        // so a chained heading OVERRIDES seg3's own 146.67. Reproduce both to be explicit.
        ResponsePath g3chained = routeSegPlain(g2EndLat, g2EndLon, w4Lat, w4Lng, profile, chainAz2, null, snapPrev);
        ResponsePath g3own = routeSegPlain(g2EndLat, g2EndLon, w4Lat, w4Lng, profile, seg3Heading, seg3Penalty, snapPrev);
        System.out.printf("  seg3 (chained az %.2f, no pen) dist = %.1f m%n", chainAz2, g3chained.getDistance());
        System.out.printf("  seg3 (own heading %.2f, pen 60) dist = %.1f m%n", seg3Heading, g3own.getDistance());
        System.out.printf("  SERVER-CHAINED TOTAL (using chained seg3) = %.1f m%n",
                g1.getDistance() + g2.getDistance() + g3chained.getDistance());

        // === 4) Control: route seg2 the client way (free start, own coords) for direct A/B ===
        System.out.println("\n========== [A/B] seg2 client-free vs server-chained ==========");
        System.out.printf("  seg2 CLIENT (free start, own coord wp2)      = %.1f m%n", cd2);
        System.out.printf("  seg2 SERVER (chained inbound az %.2f, override start) = %.1f m%n",
                inboundAz, g2.getDistance());
        System.out.printf("  blow-up factor seg2 = %.2fx%n", cd2 > 0 ? g2.getDistance() / cd2 : Double.NaN);

        // Also: does the no-penalty default matter? Route chained-heading seg2 WITH an explicit
        // large penalty vs the implicit GH default to see if penalty presence changes anything.
        ResponsePath g2bigPen = routeSegPlain(g1EndLat, g1EndLon, w3Lat, w3Lng, profile, inboundAz, 300.0, snapPrev);
        ResponsePath g2zeroPen = routeSegPlain(g1EndLat, g1EndLon, w3Lat, w3Lng, profile, inboundAz, 0.0, snapPrev);
        System.out.printf("  seg2 chained az, penalty=300 = %.1f m%n", g2bigPen.getDistance());
        System.out.printf("  seg2 chained az, penalty=0   = %.1f m%n", g2zeroPen.getDistance());

        System.out.println("\n========== SUMMARY ==========");
        System.out.printf("  CLIENT total  ~= %.1f m%n", cd1 + cd2 + cd3);
        System.out.printf("  GENERATOR total = %.1f m  (factor %.2fx)%n",
                genTotal, (cd1 + cd2 + cd3) > 0 ? genTotal / (cd1 + cd2 + cd3) : Double.NaN);
    }
}
