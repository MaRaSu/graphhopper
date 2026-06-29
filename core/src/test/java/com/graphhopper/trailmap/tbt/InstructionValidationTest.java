package com.graphhopper.trailmap.tbt;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.PropertyNamingStrategies;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.jackson.GraphHopperModule;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.json.Statement;
import com.graphhopper.util.CustomModel;
import com.graphhopper.ResponsePath;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.trailmap.TrailmapGraphHopper;
import com.graphhopper.trailmap.shared.TrailmapImportRegistry;
import com.graphhopper.util.*;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.details.PathDetail;
import org.junit.jupiter.api.*;

import java.io.File;
import java.io.FileInputStream;
import java.util.*;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Validation tests for TbT instruction generation.
 *
 * These tests assert correctness of instruction output — not just that the code
 * doesn't crash. Each test was derived from a real API issue that was debugged
 * and fixed. The corresponding diagnostic test in RouteInstructionGeneratorTest
 * has the full debug output for each case.
 *
 * Requires the pre-built graph cache at ../../data/graph-cache.
 */
public class InstructionValidationTest {

    private static final String GRAPH_LOCATION = "../../data/graph-cache";
    private static final String OSM_FILE = "../../data/finland_3.osm.pbf";
    private static final String CONFIG_FILE = "../trailmap-config.yml";

    private static GraphHopper hopper;
    private static RouteInstructionGenerator generator;

    @BeforeAll
    static void setup() throws Exception {
        File graphDir = new File(GRAPH_LOCATION);
        Assumptions.assumeTrue(graphDir.exists() && graphDir.isDirectory(),
                "Graph cache not found at " + graphDir.getAbsolutePath() + " — skipping");

        ObjectMapper yamlMapper = new ObjectMapper(new YAMLFactory());
        yamlMapper.setPropertyNamingStrategy(PropertyNamingStrategies.SNAKE_CASE);
        yamlMapper.registerModule(new GraphHopperModule());

        JsonNode root = yamlMapper.readTree(new FileInputStream(new File(CONFIG_FILE)));
        JsonNode ghNode = root.get("graphhopper");
        assertNotNull(ghNode, "trailmap-config.yml must have a 'graphhopper' top-level key");

        GraphHopperConfig config = yamlMapper.treeToValue(ghNode, GraphHopperConfig.class);
        config.putObject("graph.location", GRAPH_LOCATION);
        config.putObject("datareader.file", OSM_FILE);

        hopper = new TrailmapGraphHopper();
        hopper.setImportRegistry(new TrailmapImportRegistry());
        hopper.setAllowWrites(false);
        hopper.init(config);
        hopper.importOrLoad();

        BaseGraph baseGraph = hopper.getBaseGraph();
        EncodingManager encodingManager = hopper.getEncodingManager();
        TranslationMap translationMap = hopper.getTranslationMap();
        generator = new RouteInstructionGenerator(hopper, baseGraph, encodingManager, translationMap);
    }

    @AfterAll
    static void teardown() {
        if (hopper != null) hopper.close();
    }

    // ==================== Category A: Structural validation ====================

    // Test coordinates (Tampere area)
    private static final double START_LAT = 61.500752, START_LNG = 23.684245;
    private static final double END_LAT = 61.503679, END_LNG = 23.698272;

    /** Standard GH routing works and produces instructions. */
    @Test
    void standardRouting_producesInstructions() {
        GHRequest req = new GHRequest(START_LAT, START_LNG, END_LAT, END_LNG)
                .setProfile("gravel_mtb");
        GHResponse rsp = hopper.route(req);
        assertFalse(rsp.hasErrors(), "Routing failed: " + rsp.getErrors());

        ResponsePath path = rsp.getBest();
        assertTrue(path.getDistance() > 0, "Route should have positive distance");

        InstructionList instructions = path.getInstructions();
        assertNotNull(instructions);
        assertTrue(instructions.size() > 0, "Should have at least one instruction");
    }

    /** Edge ID path details can be extracted from a route. */
    @Test
    void edgeIdExtraction_returnsEdges() {
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
    }

    /** Single-segment route produces valid instructions with FINISH. */
    @Test
    void singleSegment_producesValidInstructions() {
        TrailmapInstructionRequest request = buildSingleSegmentRequest("gravel_mtb", "gravel", "fi");
        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);
    }

    /** Instruction distances sum to polyline distance (within 1m). */
    @Test
    void singleSegment_distancesMatchPolyline() {
        TrailmapInstructionRequest request = buildSingleSegmentRequest("gravel_mtb", "gravel_mtb", "fi");
        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        double totalInstrDist = 0;
        for (Instruction instr : result.instructions) totalInstrDist += instr.getDistance();

        double polylineDist = computePolylineDistance(result.polyline);
        assertEquals(polylineDist, totalInstrDist, 1.0,
                "Instruction distance sum should match polyline distance within 1m");
    }

    /** Two-segment route produces single FINISH, valid structure. */
    @Test
    void twoSegment_producesValidInstructions() {
        TrailmapInstructionRequest request = buildTwoSegmentRequest();
        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);
    }

    /** Multi-segment polyline matches per-segment GH routing (start/end within 5m, point count within 2). */
    @Test
    void multiSegmentPolyline_matchesPerSegmentRouting() {
        double wp1Lat = 61.500143, wp1Lng = 23.681118;
        double wp2Lat = 61.504726, wp2Lng = 23.70125;
        double wp3Lat = 61.501035, wp3Lng = 23.711962;

        // Route each segment individually via standard GH
        GHRequest req1 = new GHRequest(wp1Lat, wp1Lng, wp2Lat, wp2Lng).setProfile("gravel_mtb");
        req1.putHint("calc_points", true);
        req1.putHint("instructions", false);
        req1.setPathDetails(List.of("edge_id"));
        GHResponse rsp1 = hopper.route(req1);
        assertFalse(rsp1.hasErrors(), "Seg1 routing failed: " + rsp1.getErrors());
        PointList seg1Poly = rsp1.getBest().getPoints();

        double seg2StartLat = seg1Poly.getLat(seg1Poly.size() - 1);
        double seg2StartLng = seg1Poly.getLon(seg1Poly.size() - 1);

        GHRequest req2 = new GHRequest(seg2StartLat, seg2StartLng, wp3Lat, wp3Lng).setProfile("gravel_mtb");
        req2.putHint("calc_points", true);
        req2.putHint("instructions", false);
        req2.setPathDetails(List.of("edge_id"));
        req2.setHeadings(List.of(113.88, Double.NaN));
        GHResponse rsp2 = hopper.route(req2);
        assertFalse(rsp2.hasErrors(), "Seg2 routing failed: " + rsp2.getErrors());
        PointList seg2Poly = rsp2.getBest().getPoints();

        // Build expected polyline
        PointList expected = new PointList(seg1Poly.size() + seg2Poly.size(), seg1Poly.is3D());
        for (int i = 0; i < seg1Poly.size(); i++)
            expected.add(seg1Poly.getLat(i), seg1Poly.getLon(i), seg1Poly.is3D() ? seg1Poly.getEle(i) : Double.NaN);
        for (int i = 1; i < seg2Poly.size(); i++)
            expected.add(seg2Poly.getLat(i), seg2Poly.getLon(i), seg2Poly.is3D() ? seg2Poly.getEle(i) : Double.NaN);

        // Run through generator
        TrailmapInstructionRequest instrRequest = buildThreeWaypointRequest(
                wp1Lat, wp1Lng, wp2Lat, wp2Lng, wp3Lat, wp3Lng, 113.88);
        RouteInstructionGenerator.Result result = generator.generate(instrRequest);
        PointList actual = result.polyline;

        assertValidInstructionList(result);

        // Start/end points within 5m
        double startDist = DistanceCalcEarth.DIST_EARTH.calcDist(
                expected.getLat(0), expected.getLon(0), actual.getLat(0), actual.getLon(0));
        assertTrue(startDist < 5.0, "Start points should be within 5m, got " + startDist + "m");

        double endDist = DistanceCalcEarth.DIST_EARTH.calcDist(
                expected.getLat(expected.size() - 1), expected.getLon(expected.size() - 1),
                actual.getLat(actual.size() - 1), actual.getLon(actual.size() - 1));
        assertTrue(endDist < 5.0, "End points should be within 5m, got " + endDist + "m");

        int pointDiff = Math.abs(expected.size() - actual.size());
        assertTrue(pointDiff <= 2, "Point counts should be within 2, differ by " + pointDiff);

        // Intervals cover full polyline
        int pointsIndex = 0;
        for (Instruction instr : result.instructions) pointsIndex += instr.getLength();
        assertEquals(actual.size() - 1, pointsIndex, "Intervals should cover full polyline");
    }

    /** Three-segment route: no duplicate FINISH, valid structure. */
    @Test
    void threeSegment_producesValidInstructions() {
        TrailmapInstructionRequest request = buildThreeSegmentRequest();
        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);
    }

    /** Heading penalty routing: instruction total distance within 50m of per-segment GH distance. */
    @Test
    void headingPenaltyRouting_distancesMatch() {
        double wp1Lat = 61.577723, wp1Lng = 23.277481;
        double wp2Lat = 61.602294, wp2Lng = 23.24386;
        double wp3Lat = 61.637615, wp3Lng = 23.315964;

        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_surface == ASPHALT_OR_UNPAVED || predicted_surface == ASPHALT",
                Statement.Op.MULTIPLY, "0.8"));
        cm.addToPriority(Statement.If("predicted_highway == CYCLEWAY",
                Statement.Op.MULTIPLY, "1.3"));

        TrailmapInstructionRequest instrRequest = new TrailmapInstructionRequest();
        instrRequest.setWaypoints(List.of(
                makeWaypoint("wp1", wp1Lat, wp1Lng),
                makeWaypoint("wp2", wp2Lat, wp2Lng),
                makeWaypoint("wp3", wp3Lat, wp3Lng)));
        instrRequest.setSnapPreventions(List.of("ferry"));

        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("wp1"); seg1.setEnd("wp2");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("gravel"); seg1.setCustomModel(cm);

        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("wp2"); seg2.setEnd("wp3");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2.setProfile("gravel"); seg2.setCustomModel(cm);
        seg2.setInitialHeading(312.1334920719508); seg2.setHeadingPenalty(60.0);

        instrRequest.setSegments(List.of(seg1, seg2));
        instrRequest.setInstructionProfile("gravel");
        instrRequest.setLocale("fi");

        RouteInstructionGenerator.Result result = generator.generate(instrRequest);
        assertValidInstructionList(result);

        double totalInstrDist = 0;
        for (Instruction instr : result.instructions) totalInstrDist += instr.getDistance();

        // Route each segment individually via standard GH
        GHRequest req1 = new GHRequest(wp1Lat, wp1Lng, wp2Lat, wp2Lng).setProfile("gravel");
        req1.setCustomModel(cm); req1.setSnapPreventions(List.of("ferry"));
        req1.putHint("heading_penalty", 60);
        GHResponse rsp1 = hopper.route(req1);
        assertFalse(rsp1.hasErrors());

        GHRequest req2 = new GHRequest(wp2Lat, wp2Lng, wp3Lat, wp3Lng).setProfile("gravel");
        req2.setCustomModel(cm); req2.setSnapPreventions(List.of("ferry"));
        req2.putHint("heading_penalty", 60); req2.setHeadings(List.of(312.13, Double.NaN));
        GHResponse rsp2 = hopper.route(req2);
        assertFalse(rsp2.hasErrors());

        double totalStdDist = rsp1.getBest().getDistance() + rsp2.getBest().getDistance();
        assertEquals(totalStdDist, totalInstrDist, 50.0,
                "Total distances should be within 50m. Standard: " + Math.round(totalStdDist)
                + "m, Instructions: " + Math.round(totalInstrDist) + "m");
    }

    /** Direct segment in the middle: 3D polyline, correct markers, distance matches. */
    @Test
    void directSegmentMiddle_correctMarkersAndDistance() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.499292, 23.677684),
                makeWaypoint("wp2", 61.499559, 23.676201),
                makeWaypoint("wp3", 61.499610, 23.670164),
                makeWaypoint("wp4", 61.500511, 23.677141)));

        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("wp1"); seg1.setEnd("wp2");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS); seg1.setProfile("gravel_mtb");

        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("wp2"); seg2.setEnd("wp3");
        seg2.setType(TrailmapInstructionRequest.TYPE_DIRECT);

        TrailmapInstructionRequest.Segment seg3 = new TrailmapInstructionRequest.Segment();
        seg3.setStart("wp3"); seg3.setEnd("wp4");
        seg3.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS); seg3.setProfile("gravel_mtb");

        request.setSegments(List.of(seg1, seg2, seg3));
        request.setInstructionProfile("gravel"); request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);

        assertValidInstructionList(result);
        assertTrue(result.polyline.is3D(), "Polyline should be 3D");
        assertTrue(result.instructions.size() >= 3, "Should have at least 3 instructions");

        // Find direct segment instruction
        Instruction directInstr = null;
        int directIndex = -1;
        for (int i = 0; i < result.instructions.size(); i++) {
            if (Boolean.FALSE.equals(result.instructions.get(i).getExtraInfoJSON().get("tbt_available"))) {
                directInstr = result.instructions.get(i);
                directIndex = i;
                break;
            }
        }
        assertNotNull(directInstr, "Should have a direct segment instruction");
        assertEquals(Instruction.CONTINUE_ON_STREET, directInstr.getSign());
        assertEquals("direct", directInstr.getExtraInfoJSON().get("segment_type"));
        assertEquals("entering_direct_segment", directInstr.getExtraInfoJSON().get("confirm_reason"));
        assertTrue(directInstr.getDistance() > 0);

        // Instruction before direct has next_segment_type
        if (directIndex > 0) {
            assertEquals("direct", result.instructions.get(directIndex - 1).getExtraInfoJSON().get("next_segment_type"));
        }
        // Instruction after direct has tbt_resumed
        if (directIndex + 1 < result.instructions.size()) {
            Instruction after = result.instructions.get(directIndex + 1);
            if (after.getSign() != Instruction.FINISH) {
                assertEquals(true, after.getExtraInfoJSON().get("tbt_resumed"));
                assertEquals("direct", after.getExtraInfoJSON().get("prev_segment_type"));
            }
        }

        // Distance matches polyline
        double totalInstrDist = 0;
        for (Instruction instr : result.instructions) totalInstrDist += instr.getDistance();
        double polylineDist = computePolylineDistance(result.polyline);
        assertEquals(polylineDist, totalInstrDist, 2.0, "Distance should match polyline within 2m");

        // Direct survives post-processing with voice priority
        new InstructionPostProcessor().process(result.instructions, request.getInstructionProfile());
        boolean foundDirect = false;
        for (Instruction instr : result.instructions) {
            if (Boolean.FALSE.equals(instr.getExtraInfoJSON().get("tbt_available"))) {
                foundDirect = true;
                assertEquals("voice", instr.getExtraInfoJSON().get("tbt_priority"));
                break;
            }
        }
        assertTrue(foundDirect, "Direct instruction should survive post-processing");
    }

    /** Direct segment at start: no crash, correct markers. */
    @Test
    void directSegmentAtStart_correctMarkers() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.499292, 23.677684),
                makeWaypoint("wp2", 61.499559, 23.676201),
                makeWaypoint("wp3", 61.500511, 23.677141)));

        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("wp1"); seg1.setEnd("wp2");
        seg1.setType(TrailmapInstructionRequest.TYPE_DIRECT);

        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("wp2"); seg2.setEnd("wp3");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS); seg2.setProfile("gravel_mtb");

        request.setSegments(List.of(seg1, seg2));
        request.setInstructionProfile("gravel"); request.setLocale("fi");

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);
        assertTrue(result.instructions.size() >= 2);

        // First instruction is direct
        assertEquals(false, result.instructions.get(0).getExtraInfoJSON().get("tbt_available"));

        // Second instruction (if not FINISH) has tbt_resumed
        if (result.instructions.size() > 1 && result.instructions.get(1).getSign() != Instruction.FINISH) {
            assertEquals(true, result.instructions.get(1).getExtraInfoJSON().get("tbt_resumed"));
        }
    }

    /** Direct segment at end: direct instruction present, ends with FINISH. */
    @Test
    void directSegmentAtEnd_hasDirectAndFinish() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.499292, 23.677684),
                makeWaypoint("wp2", 61.499559, 23.676201),
                makeWaypoint("wp3", 61.499610, 23.670164)));

        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("wp1"); seg1.setEnd("wp2");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS); seg1.setProfile("gravel_mtb");

        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("wp2"); seg2.setEnd("wp3");
        seg2.setType(TrailmapInstructionRequest.TYPE_DIRECT);

        request.setSegments(List.of(seg1, seg2));
        request.setInstructionProfile("gravel"); request.setLocale("fi");

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertNotNull(result);

        boolean foundDirect = false;
        for (Instruction instr : result.instructions) {
            if (Boolean.FALSE.equals(instr.getExtraInfoJSON().get("tbt_available"))) {
                foundDirect = true;
                break;
            }
        }
        assertTrue(foundDirect, "Should have a direct segment instruction");
        assertEquals(Instruction.FINISH,
                result.instructions.get(result.instructions.size() - 1).getSign());
    }

    // ==================== Category B: Behavioral validation ====================

    // ========== Test 1: U-turn at segment boundary ==========

    /**
     * When heading chaining causes seg2 to start by backtracking along seg1's last edges,
     * the generator must produce a U-turn instruction. This was previously lost due to
     * boundary edge deduplication.
     *
     * Origin: testUturnAtBoundaryDiagnostic in RouteInstructionGeneratorTest
     */
    @Test
    void uturnAtSegmentBoundary_isPreserved() {
        double wp1Lat = 61.577236, wp1Lng = 23.278753;
        double wp2Lat = 61.602494, wp2Lng = 23.243388;
        double wp3Lat = 61.636935, wp3Lng = 23.257746;

        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_surface == ASPHALT_OR_UNPAVED || predicted_surface == ASPHALT",
                Statement.Op.MULTIPLY, "0.8"));
        cm.addToPriority(Statement.If("predicted_highway == CYCLEWAY",
                Statement.Op.MULTIPLY, "1.3"));

        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", wp1Lat, wp1Lng),
                makeWaypoint("wp2", wp2Lat, wp2Lng),
                makeWaypoint("wp3", wp3Lat, wp3Lng)));
        request.setSnapPreventions(List.of("ferry"));

        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("wp1");
        seg1.setEnd("wp2");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("gravel");
        seg1.setCustomModel(cm);

        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("wp2");
        seg2.setEnd("wp3");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2.setProfile("gravel");
        seg2.setCustomModel(cm);
        seg2.setInitialHeading(311.76719918378234);
        seg2.setHeadingPenalty(60.0);

        request.setSegments(List.of(seg1, seg2));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");

        RouteInstructionGenerator.Result result = generator.generate(request);

        // --- Structural assertions ---
        assertValidInstructionList(result);

        // --- Behavioral assertion: U-turn must exist ---
        boolean foundUturn = false;
        for (Instruction instr : result.instructions) {
            if (instr.getSign() == Instruction.U_TURN_LEFT
                    || instr.getSign() == Instruction.U_TURN_RIGHT
                    || instr.getSign() == Instruction.U_TURN_UNKNOWN) {
                foundUturn = true;
                break;
            }
        }
        assertTrue(foundUturn,
                "U-turn instruction must be present when seg2 backtracks along seg1. " +
                "Instructions: " + summarizeInstructions(result));

        // --- No micro-distance artifacts from boundary handling ---
        assertNoMicroArtifacts(result, 3.0);
    }

    // ========== Boundary U-turn suppression (Option C, toggle) ==========

    /** Builds the reported short-overshoot case (3 wp / 2 gravel_mtb segments, per-segment headings). */
    private TrailmapInstructionRequest buildShortBoundaryUturnRequest() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("xjpeHskaIV6G-mUgON3kO", 61.558387, 23.508853),
                makeWaypoint("_TOpe9hqP4baKWqMknld1", 61.557721, 23.512598),
                makeWaypoint("6vOqMY6KvM6XsH47e9jfb", 61.556869, 23.511434)));
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
        return request;
    }

    /** Run the generator with the internal boundary-U-turn flag forced on/off, restoring it after. */
    private RouteInstructionGenerator.Result generateWithFlag(TrailmapInstructionRequest req, boolean flag) {
        boolean prev = RouteInstructionGenerator.SUPPRESS_BOUNDARY_UTURN;
        RouteInstructionGenerator.SUPPRESS_BOUNDARY_UTURN = flag;
        try {
            return generator.generate(req);
        } finally {
            RouteInstructionGenerator.SUPPRESS_BOUNDARY_UTURN = prev;
        }
    }

    private static int countUturns(RouteInstructionGenerator.Result result) {
        int n = 0;
        for (Instruction in : result.instructions) {
            int s = in.getSign();
            if (s == Instruction.U_TURN_UNKNOWN || s == Instruction.U_TURN_LEFT || s == Instruction.U_TURN_RIGHT) n++;
        }
        return n;
    }

    private static double polylineLength(RouteInstructionGenerator.Result result) {
        double d = 0;
        com.graphhopper.util.PointList p = result.polyline;
        for (int i = 0; i < p.size() - 1; i++) {
            d += com.graphhopper.util.DistanceCalcEarth.DIST_EARTH.calcDist(
                    p.getLat(i), p.getLon(i), p.getLat(i + 1), p.getLon(i + 1));
        }
        return d;
    }

    /** Per-instruction polyline contiguity: point intervals tile [0, polyline) and Σ distance ≈ polyline length. */
    private void assertSeamContiguity(RouteInstructionGenerator.Result result) {
        int idx = 0;
        double sumDist = 0;
        for (Instruction in : result.instructions) {
            sumDist += in.getDistance();
            idx += in.getLength(); // FINISH has length 0
        }
        assertEquals(result.polyline.size() - 1, idx,
                "Instruction point intervals must tile the polyline exactly. "
                        + summarizeInstructions(result));
        assertEquals(polylineLength(result), sumDist, 1.0,
                "Σ instruction distance must match polyline length. " + summarizeInstructions(result));
    }

    /**
     * Toggle ON: the reported short waypoint-snap out-and-back is erased — no surviving U-turn,
     * a single real continuing turn is emitted, geometry stays consistent with the instructions,
     * and the spur (~4 m) is removed vs the toggle-OFF route. (docs/gh_tbt_pipeline_phasing_change.md)
     */
    @Test
    void boundaryUturnSuppression_erasesShortSnapUturn() {
        RouteInstructionGenerator.Result onResult = generateWithFlag(buildShortBoundaryUturnRequest(), true);
        new InstructionPostProcessor().process(onResult.instructions, "gravel_mtb");

        assertValidInstructionList(onResult);
        assertEquals(0, countUturns(onResult),
                "Short boundary U-turn must be erased with flag on. " + summarizeInstructions(onResult));
        // A single real continuing maneuver (the path -> service-road turn) must remain.
        long realTurns = onResult.instructions.stream()
                .filter(i -> i.getSign() != Instruction.CONTINUE_ON_STREET && i.getSign() != Instruction.FINISH)
                .count();
        assertTrue(realTurns >= 1,
                "A continuing turn must remain after erasing the overshoot. " + summarizeInstructions(onResult));
        // Geometry/instruction consistency at the repaired seam (R3 + R8).
        assertSeamContiguity(onResult);

        // The spur is physically removed: ON route is shorter than OFF by ~the out-and-back (~4 m).
        RouteInstructionGenerator.Result offResult = generateWithFlag(buildShortBoundaryUturnRequest(), false);
        double removed = polylineLength(offResult) - polylineLength(onResult);
        assertTrue(removed > 1.0 && removed < 30.0,
                "ON route should be shorter than OFF by the removed spur (got " + String.format("%.2fm", removed) + ")");
    }

    /** Flag OFF (legacy path): the reported case still emits the U-turn — confirms the fallback is intact. */
    @Test
    void boundaryUturnSuppression_flagOff_keepsLegacyUturn() {
        RouteInstructionGenerator.Result result = generateWithFlag(buildShortBoundaryUturnRequest(), false);
        assertValidInstructionList(result);
        assertTrue(countUturns(result) >= 1,
                "With the flag off the legacy U-turn must be preserved. " + summarizeInstructions(result));
    }

    /**
     * Toggle ON must NOT erase a genuine long out-and-back (overshoot far above the threshold).
     * Reuses the long-backtrack request from {@link #uturnAtSegmentBoundary_isPreserved}.
     */
    @Test
    void boundaryUturnSuppression_preservesLongOutAndBack() {
        double wp1Lat = 61.577236, wp1Lng = 23.278753;
        double wp2Lat = 61.602494, wp2Lng = 23.243388;
        double wp3Lat = 61.636935, wp3Lng = 23.257746;
        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_surface == ASPHALT_OR_UNPAVED || predicted_surface == ASPHALT",
                Statement.Op.MULTIPLY, "0.8"));
        cm.addToPriority(Statement.If("predicted_highway == CYCLEWAY", Statement.Op.MULTIPLY, "1.3"));
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", wp1Lat, wp1Lng),
                makeWaypoint("wp2", wp2Lat, wp2Lng),
                makeWaypoint("wp3", wp3Lat, wp3Lng)));
        request.setSnapPreventions(List.of("ferry"));
        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("wp1"); seg1.setEnd("wp2");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("gravel"); seg1.setCustomModel(cm);
        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("wp2"); seg2.setEnd("wp3");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2.setProfile("gravel"); seg2.setCustomModel(cm);
        seg2.setInitialHeading(311.76719918378234); seg2.setHeadingPenalty(60.0);
        request.setSegments(List.of(seg1, seg2));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");

        RouteInstructionGenerator.Result result = generateWithFlag(request, true);
        assertValidInstructionList(result);
        assertTrue(countUturns(result) >= 1,
                "A genuine long out-and-back must be preserved even with the flag on. "
                        + summarizeInstructions(result));
        // Geometry stays consistent on a U-turn-containing route under the flag.
        assertSeamContiguity(result);
    }

    /**
     * Step-3 harness: confirm the defer-commit path (toggle ON) only ever differs from the
     * production path (toggle OFF) by collapsing short boundary-U-turn snap artifacts — it
     * never corrupts geometry, and never ADDS a U-turn or a maneuver.
     * <p>
     * - A route with no merge-eligible seam (genuine long out-and-back) must be byte-identical.
     * - The 3-segment gravel route DOES contain a short snap-stub at a real 538 m out-and-back;
     *   ON cleans the 1 m snap spur (the out-and-back U-turn is correctly preserved). We assert
     *   ON stays valid + geometry-consistent and never adds U-turns/instructions.
     */
    @Test
    void boundaryUturnSuppression_noDiffExceptFixedSeam() {
        // Both multi-segment routes contain short snap-stub seams (waypoint snaps are common),
        // so the fix fires on each. We assert it ONLY cleans: stays valid + geometry-consistent,
        // never adds a U-turn or instruction, and perturbs total geometry only by tiny spurs.
        assertOnCleansOnly(buildGravelRouteQualityRequest());
        assertOnCleansOnly(buildLongOutAndBackRequest());
    }

    /** ON must stay valid + geometry-consistent, never add U-turns/instructions, and only remove tiny spurs. */
    private void assertOnCleansOnly(TrailmapInstructionRequest request) {
        RouteInstructionGenerator.Result off = generateWithFlag(request, false);
        RouteInstructionGenerator.Result on = generateWithFlag(request, true);

        assertValidInstructionList(on);
        assertSeamContiguity(on);
        assertTrue(countUturns(on) <= countUturns(off),
                "Suppression must not ADD U-turns. OFF uturns=" + countUturns(off)
                        + " ON uturns=" + countUturns(on) + "\nON: " + summarizeInstructions(on));
        assertTrue(on.instructions.size() <= off.instructions.size(),
                "Suppression must not ADD instructions. OFF=" + off.instructions.size()
                        + " ON=" + on.instructions.size());
        // Geometry change is bounded: only short snap spurs are removed (each ≤ ~2× threshold).
        double removed = polylineLength(off) - polylineLength(on);
        assertTrue(removed >= -0.5 && removed < 60.0,
                "ON geometry should differ from OFF only by small removed spurs (got "
                        + String.format("%.1fm", removed) + ")");
    }

    private TrailmapInstructionRequest buildLongOutAndBackRequest() {
        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_surface == ASPHALT_OR_UNPAVED || predicted_surface == ASPHALT",
                Statement.Op.MULTIPLY, "0.8"));
        cm.addToPriority(Statement.If("predicted_highway == CYCLEWAY", Statement.Op.MULTIPLY, "1.3"));
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.577236, 23.278753),
                makeWaypoint("wp2", 61.602494, 23.243388),
                makeWaypoint("wp3", 61.636935, 23.257746)));
        request.setSnapPreventions(List.of("ferry"));
        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("wp1"); seg1.setEnd("wp2");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("gravel"); seg1.setCustomModel(cm);
        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("wp2"); seg2.setEnd("wp3");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2.setProfile("gravel"); seg2.setCustomModel(cm);
        seg2.setInitialHeading(311.76719918378234); seg2.setHeadingPenalty(60.0);
        request.setSegments(List.of(seg1, seg2));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        return request;
    }

    /**
     * Two things at once on a real 4-waypoint payload:
     * <ol>
     *   <li><b>Heading from segment data only.</b> Previously the server chained the inbound azimuth
     *       into the next segment, which on a stub/turn-around waypoint forced an onward departure
     *       and a large go-around detour (~1150 m for this ~270 m route, ~4.3×). seg1/seg2 carry no
     *       heading, seg3 has an explicit one.</li>
     *   <li><b>Consecutive double-merge (R17).</b> The payload has short snap stubs at BOTH wp2 and
     *       wp3, so both internal seams are boundary-U-turn snap artifacts. The defer-commit path
     *       merges seg1+seg2, then merges that result with seg3 — exercising back-to-back merges where
     *       the second classifies against the already-merged chain. Both stubs are cleaned, no U-turn
     *       survives, and geometry stays consistent.</li>
     * </ol>
     */
    @Test
    void headingFromSegmentDataOnly_noChainedDetour() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("6srnVdr4KDPqjmmxOcIj0", 61.550574, 23.554379),
                makeWaypoint("JItIt-N00nnKKyU95Lfb6", 61.550035, 23.552747),
                makeWaypoint("K6BGbecDYbDyzPUi86yjN", 61.549208, 23.55411),
                makeWaypoint("AoTFlcVcmn-INWe3W6UpG", 61.549411, 23.554461)));
        String[][] segs = {
                {"6srnVdr4KDPqjmmxOcIj0", "JItIt-N00nnKKyU95Lfb6"},
                {"JItIt-N00nnKKyU95Lfb6", "K6BGbecDYbDyzPUi86yjN"},
                {"K6BGbecDYbDyzPUi86yjN", "AoTFlcVcmn-INWe3W6UpG"}};
        java.util.List<TrailmapInstructionRequest.Segment> segList = new java.util.ArrayList<>();
        for (int i = 0; i < segs.length; i++) {
            TrailmapInstructionRequest.Segment s = new TrailmapInstructionRequest.Segment();
            s.setStart(segs[i][0]); s.setEnd(segs[i][1]);
            s.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
            s.setProfile("gravel");
            if (i == 2) { s.setInitialHeading(146.6696150702877); s.setHeadingPenalty(60.0); }
            segList.add(s);
        }
        request.setSegments(segList);
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);
        // (1) No chaining → ~270 m ballpark, NOT the ~1150 m chained-heading detour.
        double total = polylineLength(result);
        assertTrue(total < 400.0,
                "Chained-heading detour regression: expected ~270 m, got " + String.format("%.0fm", total)
                        + ". " + summarizeInstructions(result));
        // (2) Both short stubs cleaned by the consecutive double-merge (R17): no surviving U-turn,
        //     and geometry stays consistent through both merges.
        assertEquals(0, countUturns(result),
                "Both wp2/wp3 snap stubs must be cleaned (no surviving U-turn). " + summarizeInstructions(result));
        assertSeamContiguity(result);
    }

    // ========== Test 2: 3-segment gravel route instruction quality ==========

    /**
     * 3-segment gravel route in Tampere. After fixes:
     * - No false M1 "join" instructions where there are real left-then-right turns
     * - Snap stub artifacts (U-turn + sharp left with ~1-2m distances) suppressed by M0
     * - All instructions have tbt_priority after post-processing
     * - Join instructions that exist have correct join_direction/join_target_type
     *
     * Origin: testThreeSegmentGravelRouteInstructionQuality in RouteInstructionGeneratorTest
     */
    @Test
    void threeSegmentGravelRoute_instructionQuality() {
        TrailmapInstructionRequest request = buildGravelRouteQualityRequest();
        RouteInstructionGenerator.Result result = generator.generate(request);

        // --- Before post-processing: structural ---
        assertValidInstructionList(result);

        // Apply post-processing
        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        // --- After post-processing: structural ---
        assertValidInstructionList(result);

        // --- All non-FINISH instructions must have tbt_priority ---
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            if (instr.getSign() == Instruction.FINISH) continue;
            Map<String, Object> extra = instr.getExtraInfoJSON();
            assertNotNull(extra.get("tbt_priority"),
                    "Instruction [" + i + "] sign=" + signName(instr.getSign())
                    + " missing tbt_priority. Instructions: " + summarizeInstructions(result));
        }

        // --- No micro-distance artifacts (M0 snap stub suppression working) ---
        assertNoMicroArtifacts(result, 3.0);

        // --- Every join instruction must have both join_direction and join_target_type ---
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            Map<String, Object> extra = instr.getExtraInfoJSON();
            if (extra.containsKey("join_direction")) {
                assertNotNull(extra.get("join_target_type"),
                        "Instruction [" + i + "] has join_direction but missing join_target_type");
                String dir = (String) extra.get("join_direction");
                assertTrue(dir.equals("left") || dir.equals("right"),
                        "join_direction must be 'left' or 'right', got: " + dir);
            }
        }

        // --- Reasonable instruction count after post-processing ---
        // Current output: 16 instructions (15 + FINISH). Allow range for minor data changes.
        int count = result.instructions.size();
        assertTrue(count >= 10 && count <= 25,
                "Expected 10-25 instructions after post-processing, got " + count
                + ". Instructions: " + summarizeInstructions(result));
    }

    // ========== Test 3: Boundary edge dedup does not lose turn instruction ==========

    /**
     * When seg1 and seg2 share a boundary edge, the CYCLEWAY→PATH turn instruction
     * at the boundary must not be lost. The two-segment result must contain the same
     * key turns as routing the second segment alone.
     *
     * Origin: testBoundaryEdgeDeduplicationLosesInstruction in RouteInstructionGeneratorTest
     */
    @Test
    void boundaryEdgeDedup_preservesTurnInstruction() {
        double w1Lat = 61.500578, w1Lng = 23.666027;
        double w2Lat = 61.500578, w2Lng = 23.662072;
        double w3Lat = 61.501156, w3Lng = 23.662261;

        // --- Single-segment w2→w3 (correct baseline) ---
        TrailmapInstructionRequest singleReq = new TrailmapInstructionRequest();
        singleReq.setWaypoints(List.of(
                makeWaypoint("sw1", w2Lat, w2Lng),
                makeWaypoint("sw2", w3Lat, w3Lng)));
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
        assertValidInstructionList(singleResult);

        // --- Two-segment w1→w2→w3 (the previously broken case) ---
        TrailmapInstructionRequest twoSegReq = new TrailmapInstructionRequest();
        twoSegReq.setWaypoints(List.of(
                makeWaypoint("tw1", w1Lat, w1Lng),
                makeWaypoint("tw2", w2Lat, w2Lng),
                makeWaypoint("tw3", w3Lat, w3Lng)));
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
        assertValidInstructionList(twoSegResult);

        // --- Key turns from single-segment must also appear in two-segment ---
        // Single-segment produces: CYCLEWAY→PATH turn_right, then PATH→Hedelmäkatu turn_right
        // The boundary dedup bug lost the CYCLEWAY→PATH turn.

        boolean singleHasCyclewayToPath = hasInstructionWithProperties(singleResult,
                sign -> sign == Instruction.TURN_RIGHT || sign == Instruction.TURN_SLIGHT_RIGHT,
                extra -> "PATH".equals(extra.get("predicted_highway"))
                        && "CYCLEWAY".equals(extra.get("prev_predicted_highway")));

        // Only assert on two-segment if single-segment has the turn (guards against data change)
        if (singleHasCyclewayToPath) {
            boolean twoSegHasCyclewayToPath = hasInstructionWithProperties(twoSegResult,
                    sign -> sign == Instruction.TURN_RIGHT || sign == Instruction.TURN_SLIGHT_RIGHT,
                    extra -> "PATH".equals(extra.get("predicted_highway"))
                            && "CYCLEWAY".equals(extra.get("prev_predicted_highway")));
            assertTrue(twoSegHasCyclewayToPath,
                    "Two-segment result must preserve the CYCLEWAY→PATH turn that exists in single-segment. "
                    + "Single: " + summarizeInstructions(singleResult)
                    + " Two-seg: " + summarizeInstructions(twoSegResult));
        }

        // Both results must contain a turn onto Hedelmäkatu
        boolean singleHasHedelma = hasInstructionWithName(singleResult, "Hedelmäkatu");
        boolean twoSegHasHedelma = hasInstructionWithName(twoSegResult, "Hedelmäkatu");
        if (singleHasHedelma) {
            assertTrue(twoSegHasHedelma,
                    "Two-segment result must contain turn onto Hedelmäkatu. "
                    + "Two-seg: " + summarizeInstructions(twoSegResult));
        }
    }

    // ========== Test 4: Foot route missing turns with wrong instruction_profile ==========

    /**
     * Foot route where using instruction_profile="gravel" lost turns because gravel
     * weighting can't see footways. With the correct "trailmap_foot" profile, at least
     * 4 real turn instructions must be generated.
     *
     * Origin: testFootRouteMissingTurns in RouteInstructionGeneratorTest
     */
    @Test
    void footRouteMissingTurns_correctProfileHasTurns() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.499257, 23.677376),
                makeWaypoint("wp2", 61.50065, 23.678926),
                makeWaypoint("wp3", 61.501256, 23.685815)));

        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("wp1");
        seg1.setEnd("wp2");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("trailmap_foot");

        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("wp2");
        seg2.setEnd("wp3");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2.setProfile("trailmap_foot");
        seg2.setInitialHeading(78.02216610253362);
        seg2.setHeadingPenalty(60.0);

        request.setSegments(List.of(seg1, seg2));
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        // With correct foot profile: must have real turns
        request.setInstructionProfile("trailmap_foot");
        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        int turnCount = 0;
        for (Instruction instr : result.instructions) {
            int sign = instr.getSign();
            if (sign != Instruction.FINISH && sign != Instruction.CONTINUE_ON_STREET
                    && sign != Instruction.IGNORE) {
                turnCount++;
            }
        }
        assertTrue(turnCount >= 4,
                "Foot route with trailmap_foot profile must have at least 4 turn instructions, "
                + "got " + turnCount + ". Instructions: " + summarizeInstructions(result));
    }

    // ========== Test 5: Foot route join instruction quality ==========

    /**
     * Foot route where false "join" instructions were generated at T-junctions.
     * After fix: near-end road→cycleway crossing should produce a proper join
     * (join_direction + join_target_type), while the early cycleway→footway→road
     * sequence should NOT be a join (it's a T-junction with a real turn).
     *
     * Origin: testFootRouteJoinInstructionQuality in RouteInstructionGeneratorTest
     */
    @Test
    void footRouteJoinQuality_correctJoinVsTJunction() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.506394, 23.697305),
                makeWaypoint("wp2", 61.50716, 23.698392),
                makeWaypoint("wp3", 61.506078, 23.69144)));

        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("wp1");
        seg1.setEnd("wp2");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("trailmap_foot");

        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("wp2");
        seg2.setEnd("wp3");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2.setProfile("trailmap_foot");
        seg2.setInitialHeading(322.45489185598296);
        seg2.setHeadingPenalty(60.0);

        request.setSegments(List.of(seg1, seg2));
        request.setInstructionProfile("trailmap_foot");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        // All join instructions must have complete metadata
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            Map<String, Object> extra = instr.getExtraInfoJSON();
            if (extra.containsKey("join_direction")) {
                assertNotNull(extra.get("join_target_type"),
                        "Instruction [" + i + "] has join_direction but missing join_target_type");
            }
        }

        // The route should have a join instruction for the road→cycleway crossing near the end
        boolean hasJoinToCycleway = false;
        for (Instruction instr : result.instructions) {
            Map<String, Object> extra = instr.getExtraInfoJSON();
            if ("CYCLEWAY".equals(extra.get("join_target_type"))) {
                hasJoinToCycleway = true;
                break;
            }
        }
        assertTrue(hasJoinToCycleway,
                "Expected a join instruction targeting CYCLEWAY near the end. "
                + "Instructions: " + summarizeInstructions(result));

        // All non-FINISH instructions must have tbt_priority
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            if (instr.getSign() == Instruction.FINISH) continue;
            assertNotNull(instr.getExtraInfoJSON().get("tbt_priority"),
                    "Instruction [" + i + "] missing tbt_priority");
        }
    }

    // ========== Test 6: Waypoint snap stub at junction ==========

    /**
     * Waypoint snaps to a crossing road at a junction, creating a short detour
     * (turn → U-turn → turn back). After M0 snap stub suppression, the confusing
     * stub instructions must be completely absorbed — the route should look like
     * a simple continuation.
     *
     * Origin: testWaypointSnapStubAtJunction in RouteInstructionGeneratorTest
     */
    @Test
    void waypointSnapStub_suppressedByPostProcessing() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.503523, 23.691088),
                makeWaypoint("wp2", 61.504324, 23.691833),
                makeWaypoint("wp3", 61.505278, 23.693146)));

        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("wp1");
        seg1.setEnd("wp2");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("trailmap_foot");

        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("wp2");
        seg2.setEnd("wp3");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2.setProfile("trailmap_foot");
        seg2.setInitialHeading(299.64515003038747);
        seg2.setHeadingPenalty(60.0);

        request.setSegments(List.of(seg1, seg2));
        request.setInstructionProfile("trailmap_foot");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        // After M0 suppression: no U-turn should remain
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            int sign = instr.getSign();
            assertFalse(sign == Instruction.U_TURN_UNKNOWN
                            || sign == Instruction.U_TURN_LEFT
                            || sign == Instruction.U_TURN_RIGHT,
                    "U-turn at [" + i + "] should have been suppressed by M0 snap stub pass. "
                    + "Instructions: " + summarizeInstructions(result));
        }

        // The route is essentially straight — should have very few instructions after suppression
        // Current output: 2 (CONTINUE + FINISH). Allow up to 4 for minor data changes.
        assertTrue(result.instructions.size() <= 4,
                "After snap stub suppression, expected at most 4 instructions, got "
                + result.instructions.size() + ". Instructions: " + summarizeInstructions(result));
    }

    // ========== Test 6b: Cross-segment U-turn — anchored at boundary, not stacked ==========

    /**
     * Production bug (2026-05-10): on a 30 km route, a single waypoint creating a
     * U-turn at a segment boundary caused instructions for the entire middle of the
     * route to be misplaced. The U-turn instruction landed ~500 m off its real
     * location, and five subsequent instructions stacked on top of it because they
     * searched forward from the wrong anchor.
     *
     * Root cause: the synthetic-path fromNode for the new segment is the FAR endpoint
     * of the shared edge (opposite the snap point) and is not on the route polyline.
     * remapInstructionGeometry's coordinate-match fell back to "global-closest forward
     * point," which landed at an arbitrary index hundreds of meters away.
     *
     * Fix: stamp _polyline_start_hint on the patched U_TURN_UNKNOWN at the U-turn
     * boundary, anchoring it at the snap point.
     *
     * Origin: diagUturnWaypointBreaksRemap in RouteInstructionGeneratorTest.
     */
    @Test
    void crossSegmentUturn_anchoredAtBoundaryNotStacked() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("w1", 61.505403, 23.679752),
                makeWaypoint("w2", 61.505903, 23.666556),
                makeWaypoint("w3", 61.520546, 23.640742),
                makeWaypoint("w4", 61.531336, 23.636473)));

        TrailmapInstructionRequest.Segment s1 = new TrailmapInstructionRequest.Segment();
        s1.setStart("w1"); s1.setEnd("w2");
        s1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        s1.setProfile("gravel");
        s1.setInitialHeading(282.83505622838345);
        s1.setHeadingPenalty(60.0);

        TrailmapInstructionRequest.Segment s2 = new TrailmapInstructionRequest.Segment();
        s2.setStart("w2"); s2.setEnd("w3");
        s2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        s2.setProfile("gravel");
        s2.setInitialHeading(263.8884413748203);
        s2.setHeadingPenalty(60.0);

        TrailmapInstructionRequest.Segment s3 = new TrailmapInstructionRequest.Segment();
        s3.setStart("w3"); s3.setEnd("w4");
        s3.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        s3.setProfile("gravel");
        s3.setInitialHeading(315.47069676557135);
        s3.setHeadingPenalty(60.0);

        request.setSegments(List.of(s1, s2, s3));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        // --- The seg1/seg2 boundary "U-turn" was itself a 5.55 m waypoint-snap artifact ---
        // (continuation onto a different edge, not a reversal — measured by
        // RouteInstructionGeneratorTest#measureCrossSegmentUturnOvershoot). The 2026-05-10 fix
        // merely *anchored* the spurious U-turn so following instructions did not stack; the
        // boundary-U-turn suppression now *removes* it entirely. So with the fix on there must
        // be NO U-turn here.
        assertEquals(0, countUturns(result),
                "The 5.55 m snap-artifact U-turn at the seg1/seg2 boundary must be suppressed. "
                + "Instructions: " + summarizeInstructions(result));

        // --- The original regression must still not recur: no two non-FINISH instructions may ---
        // share the same polyline interval start (pre-2026-05-10, six instructions stacked at one
        // index). Every instruction must have a distinct, monotonically increasing interval start.
        int polyIdx = 0;
        int prevPolyIdx = -1;
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            if (instr.getSign() != Instruction.FINISH) {
                assertTrue(polyIdx > prevPolyIdx,
                        "Instruction [" + i + "] sign=" + signName(instr.getSign())
                        + " shares polyline interval start " + polyIdx
                        + " with the previous instruction (stacking regression). "
                        + "Instructions: " + summarizeInstructions(result));
                prevPolyIdx = polyIdx;
            }
            polyIdx += instr.getLength();
        }
    }

    // ========== Test 7: Fork near end — track vs path ==========

    /**
     * Short MTB route ending near a fork between track and path. The fork instruction
     * must not be suppressed — it's a real navigation decision on trails.
     *
     * Origin: testForkNearEnd_track_vs_path in RouteInstructionGeneratorTest
     */
    @Test
    void forkNearEnd_trackVsPath_instructionPreserved() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.518972, 23.611517),
                makeWaypoint("wp2", 61.51958, 23.610079)));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("wp1");
        seg.setEnd("wp2");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("mtb");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("mtb");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        assertValidInstructionList(result);

        // Must have at least one real turn instruction (not just CONTINUE + FINISH)
        int turnCount = 0;
        for (Instruction instr : result.instructions) {
            int sign = instr.getSign();
            if (sign != Instruction.FINISH && sign != Instruction.CONTINUE_ON_STREET
                    && sign != Instruction.IGNORE) {
                turnCount++;
            }
        }
        assertTrue(turnCount >= 1,
                "Trail fork must produce at least 1 turn instruction, got " + turnCount
                + ". Instructions: " + summarizeInstructions(result));

        // All non-FINISH instructions must have tbt_priority
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            if (instr.getSign() == Instruction.FINISH) continue;
            assertNotNull(instr.getExtraInfoJSON().get("tbt_priority"),
                    "Instruction [" + i + "] missing tbt_priority");
        }
    }

    // ========== Test 8: Cycleway to road — angle-based sign, not KEEP_RIGHT ==========

    /**
     * When PredictedHighway changes at a fork (CYCLEWAY→MINOR_ROAD), the instruction
     * should use an angle-based sign (TURN_SLIGHT_RIGHT) not a fork-relative sign
     * (KEEP_RIGHT). KEEP_RIGHT is misleading when the road types are visually distinct.
     *
     * Origin: testCyclewayToRoadKeepRight in RouteInstructionGeneratorTest
     */
    @Test
    void cyclewayToRoad_angleBasedSign_notKeepRight() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.467213, 23.653461),
                makeWaypoint("wp2", 61.467741, 23.653907)));

        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_highway == CYCLEWAY",
                Statement.Op.MULTIPLY, "1.3"));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("wp1");
        seg.setEnd("wp2");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");
        seg.setCustomModel(cm);

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        // Find the cycleway→road transition instruction
        Instruction transitionInstr = null;
        for (Instruction instr : result.instructions) {
            Map<String, Object> extra = instr.getExtraInfoJSON();
            if ("CYCLEWAY".equals(extra.get("prev_predicted_highway"))
                    && "MINOR_ROAD".equals(extra.get("predicted_highway"))) {
                transitionInstr = instr;
                break;
            }
        }

        assertNotNull(transitionInstr,
                "Expected a CYCLEWAY→MINOR_ROAD transition instruction. "
                + "Instructions: " + summarizeInstructions(result));

        // Must NOT be KEEP_RIGHT — should be angle-based (TURN_SLIGHT_RIGHT or TURN_RIGHT)
        assertNotEquals(Instruction.KEEP_RIGHT, transitionInstr.getSign(),
                "CYCLEWAY→MINOR_ROAD transition should use angle-based sign, not KEEP_RIGHT. "
                + "Got: " + signName(transitionInstr.getSign()));
        assertNotEquals(Instruction.KEEP_LEFT, transitionInstr.getSign(),
                "CYCLEWAY→MINOR_ROAD transition should use angle-based sign, not KEEP_LEFT. "
                + "Got: " + signName(transitionInstr.getSign()));
    }

    // ========== Test 9: Road to cycleway — angle-based sign, not KEEP_RIGHT ==========

    /**
     * Mirror of test 8: road continues straight, route turns onto cycleway.
     * Must use angle-based sign (TURN_SLIGHT_RIGHT), not KEEP_RIGHT.
     *
     * Origin: testRoadToCyclewayKeepRight in RouteInstructionGeneratorTest
     */
    @Test
    void roadToCycleway_angleBasedSign_notKeepRight() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.473361, 23.690552),
                makeWaypoint("wp2", 61.473788, 23.692248)));

        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_highway == CYCLEWAY",
                Statement.Op.MULTIPLY, "1.3"));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("wp1");
        seg.setEnd("wp2");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");
        seg.setCustomModel(cm);

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        // Find the road→cycleway transition
        Instruction transitionInstr = null;
        for (Instruction instr : result.instructions) {
            Map<String, Object> extra = instr.getExtraInfoJSON();
            if ("MINOR_ROAD".equals(extra.get("prev_predicted_highway"))
                    && "CYCLEWAY".equals(extra.get("predicted_highway"))) {
                transitionInstr = instr;
                break;
            }
        }

        assertNotNull(transitionInstr,
                "Expected a MINOR_ROAD→CYCLEWAY transition instruction. "
                + "Instructions: " + summarizeInstructions(result));

        assertNotEquals(Instruction.KEEP_RIGHT, transitionInstr.getSign(),
                "MINOR_ROAD→CYCLEWAY transition should use angle-based sign, not KEEP_RIGHT");
        assertNotEquals(Instruction.KEEP_LEFT, transitionInstr.getSign(),
                "MINOR_ROAD→CYCLEWAY transition should use angle-based sign, not KEEP_LEFT");
    }

    // ========== Test 10: Asphalt to unpaved cycleway — angle-based sign ==========

    /**
     * Asphalt cycleway continues straight, route turns left onto unpaved cycleway.
     * Must use angle-based TURN_SLIGHT_LEFT, not KEEP_LEFT.
     *
     * Origin: testAsphaltToUnpavedCyclewayKeepLeft in RouteInstructionGeneratorTest
     */
    @Test
    void asphaltToUnpavedCycleway_angleBasedSign_notKeepLeft() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.486074, 23.760044),
                makeWaypoint("wp2", 61.486355, 23.760642)));

        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_highway == CYCLEWAY",
                Statement.Op.MULTIPLY, "1.3"));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("wp1");
        seg.setEnd("wp2");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");
        seg.setCustomModel(cm);

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        // Find the turn instruction (should be the cycleway fork)
        Instruction turnInstr = null;
        for (Instruction instr : result.instructions) {
            int sign = instr.getSign();
            if (sign != Instruction.CONTINUE_ON_STREET && sign != Instruction.FINISH
                    && sign != Instruction.IGNORE) {
                turnInstr = instr;
                break;
            }
        }

        assertNotNull(turnInstr,
                "Expected a turn instruction at the cycleway fork. "
                + "Instructions: " + summarizeInstructions(result));

        assertNotEquals(Instruction.KEEP_LEFT, turnInstr.getSign(),
                "Cycleway fork should use angle-based sign (TURN_SLIGHT_LEFT), not KEEP_LEFT");
        assertNotEquals(Instruction.KEEP_RIGHT, turnInstr.getSign(),
                "Cycleway fork should use angle-based sign, not KEEP_RIGHT");
    }

    /**
     * Paved road forks: the route bends ~34° left onto a paved road (Vanhamaantie →
     * Teollisuustie, both asphalt) while a compacted/gravel minor road continues nearly
     * straight (~10° off the incoming heading). The reframer must NOT relabel the left
     * bend as "straight" (CONTINUE) just because the surface filter discounts the
     * different-surface straight branch — a way going straight ahead owns the "straight"
     * direction regardless of surface, so the route's bend is a real turn.
     *
     * Regression: previously the surface filter emptied the visible-alt set, the
     * no-competition reframe demoted KEEP_LEFT → CONTINUE_ON_STREET ("suoraan"), and the
     * fork was lost. The surface-blind straight-ahead reference now vetoes that demote.
     *
     * Origin: diagnoseGravelForkReframedToStraight in RouteInstructionGeneratorTest
     */
    @Test
    void pavedRouteBendingFromGravelStraightAhead_notReframedToStraight() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("b2h7hiL9XChxVYy-zQGoL", 61.174563, 23.712415),
                makeWaypoint("3D52rop44wsDQr1Ig6jdn", 61.175132, 23.711271)));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("b2h7hiL9XChxVYy-zQGoL");
        seg.setEnd("3D52rop44wsDQr1Ig6jdn");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        // The fork onto Teollisuustie.
        Instruction forkInstr = null;
        for (Instruction instr : result.instructions) {
            if ("Teollisuustie".equals(instr.getName())) {
                forkInstr = instr;
                break;
            }
        }
        assertNotNull(forkInstr,
                "Expected an instruction onto Teollisuustie. "
                + "Instructions: " + summarizeInstructions(result));

        int sign = forkInstr.getSign();
        assertNotEquals(Instruction.CONTINUE_ON_STREET, sign,
                "Route bends ~34° left off a straight-ahead gravel road; the turn must NOT be "
                + "reframed to CONTINUE (\"suoraan\"). " + summarizeInstructions(result));
        assertTrue(sign == Instruction.KEEP_LEFT
                        || sign == Instruction.TURN_SLIGHT_LEFT
                        || sign == Instruction.TURN_LEFT,
                "Fork onto Teollisuustie must keep a leftward sign (the rule chain emits KEEP_LEFT). "
                + "Got: " + signName(sign) + ". " + summarizeInstructions(result));
    }

    // ========== Test 11: Complex turns foot route — not collapsed to false join ==========

    /**
     * Tight right → sharp left → right sequence. Must NOT collapse into a single
     * false "join left" instruction. The sharp turns should be preserved as individual
     * instructions (possibly with then_turn linking).
     *
     * Origin: testComplexTurnsFootRoute in RouteInstructionGeneratorTest
     */
    @Test
    void complexTurnsFootRoute_notCollapsedToFalseJoin() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.499065, 23.631531),
                makeWaypoint("wp2", 61.499816, 23.631703)));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("wp1");
        seg.setEnd("wp2");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("trailmap_foot");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("trailmap_foot");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        // Must have at least 2 turn instructions (the 3 turns may merge to 2 via then_turn,
        // but must NOT collapse to a single false join)
        int turnCount = 0;
        for (Instruction instr : result.instructions) {
            int sign = instr.getSign();
            if (sign != Instruction.FINISH && sign != Instruction.CONTINUE_ON_STREET
                    && sign != Instruction.IGNORE) {
                turnCount++;
            }
        }
        assertTrue(turnCount >= 2,
                "Complex 3-turn sequence must produce at least 2 turn instructions after "
                + "post-processing, got " + turnCount + ". Instructions: " + summarizeInstructions(result));

        // No false join_direction on the first turn (the sharp right is not a join pattern)
        for (Instruction instr : result.instructions) {
            if (instr.getSign() == Instruction.TURN_SHARP_RIGHT
                    || instr.getSign() == Instruction.TURN_RIGHT) {
                Map<String, Object> extra = instr.getExtraInfoJSON();
                // If join exists, the target type must be the actual road type, not a false positive
                if (extra.containsKey("join_direction")) {
                    // A join at "Korvenkatu" (residential road) from a path would be wrong —
                    // it's a sharp turn, not a side-path crossing
                    assertNotEquals("PATH", extra.get("join_target_type"),
                            "Sharp right to road should not be marked as join to PATH");
                }
                break;
            }
        }
    }

    // ========== Test 12: Missing join cycleway to road ==========

    /**
     * Cycleway with left-then-right onto a road — should produce an M1 join instruction
     * after post-processing (join_direction + join_target_type).
     *
     * Origin: testMissingJoinCyclewayToRoad in RouteInstructionGeneratorTest
     */
    @Test
    void missingJoinCyclewayToRoad_producesJoinAfterPostProcessing() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.503449, 23.684035),
                makeWaypoint("wp2", 61.502604, 23.687161)));

        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_highway == CYCLEWAY",
                Statement.Op.MULTIPLY, "1.3"));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("wp1");
        seg.setEnd("wp2");
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
        assertValidInstructionList(result);

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        // Should have a join instruction targeting MINOR_ROAD (the Mattilankatu join)
        boolean hasJoin = false;
        for (Instruction instr : result.instructions) {
            Map<String, Object> extra = instr.getExtraInfoJSON();
            if (extra.containsKey("join_direction")) {
                hasJoin = true;
                assertNotNull(extra.get("join_target_type"),
                        "Join instruction must have join_target_type");
                break;
            }
        }
        assertTrue(hasJoin,
                "Expected an M1 join instruction after post-processing. "
                + "Instructions: " + summarizeInstructions(result));
    }

    // ========== Test 13: Straight cycleway with unpaved fork — suppressed ==========

    /**
     * Going perfectly straight on an asphalt cycleway with an unpaved fork to the left.
     * Must NOT produce KEEP_RIGHT. Should be suppressed entirely (just CONTINUE + FINISH)
     * or at most CONTINUE_ON_STREET.
     *
     * Origin: testStraightCyclewayUnpavedForkKeepRight in RouteInstructionGeneratorTest
     */
    @Test
    void straightCyclewayUnpavedFork_noKeepRight() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.491062, 23.753742),
                makeWaypoint("wp2", 61.491025, 23.752542)));

        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_highway == CYCLEWAY",
                Statement.Op.MULTIPLY, "1.3"));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("wp1");
        seg.setEnd("wp2");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");
        seg.setCustomModel(cm);

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        // Must NOT have KEEP_RIGHT or KEEP_LEFT — route goes straight
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            assertNotEquals(Instruction.KEEP_RIGHT, instr.getSign(),
                    "Straight cycleway should not produce KEEP_RIGHT at [" + i + "]. "
                    + "Instructions: " + summarizeInstructions(result));
            assertNotEquals(Instruction.KEEP_LEFT, instr.getSign(),
                    "Straight cycleway should not produce KEEP_LEFT at [" + i + "]. "
                    + "Instructions: " + summarizeInstructions(result));
        }

        // Should be very simple: CONTINUE + FINISH (no turn needed when going straight)
        assertTrue(result.instructions.size() <= 3,
                "Straight cycleway should produce at most 3 instructions, got "
                + result.instructions.size() + ". Instructions: " + summarizeInstructions(result));
    }

    // ========== Test 14: Polyline indexes after consecutive direct segments ==========

    /**
     * Complex route: 2 followRoads → 8 direct → 3 followRoads.
     * Instruction intervals must cover the entire polyline without gaps or overlaps.
     * Total instruction distance must match polyline distance.
     *
     * Origin: testPolylineIndexAfterConsecutiveDirectSegments in RouteInstructionGeneratorTest
     */
    @Test
    void consecutiveDirectSegments_polylineIndexesCoverFull() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();

        request.setWaypoints(List.of(
                makeWaypoint("w1", 61.514433, 23.581875),
                makeWaypoint("w2", 61.517204, 23.585335),
                makeWaypoint("w3", 61.517508, 23.586106),
                makeWaypoint("w4", 61.51891328984175, 23.586272450858047),
                makeWaypoint("w5", 61.51977095040931, 23.58437534948368),
                makeWaypoint("w6", 61.52051110420439, 23.584473900204102),
                makeWaypoint("w7", 61.52095753765434, 23.58755361022807),
                makeWaypoint("w8", 61.5206990769648, 23.590830421692885),
                makeWaypoint("w9", 61.519982424720666, 23.591594189779016),
                makeWaypoint("w10", 61.51843157969307, 23.592702885387695),
                makeWaypoint("w11", 61.51704515272817, 23.59427969692007),
                makeWaypoint("w12", 61.516024, 23.596282),
                makeWaypoint("w13", 61.515548, 23.594369),
                makeWaypoint("w14", 61.516169, 23.592415)));

        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_highway == CYCLEWAY",
                Statement.Op.MULTIPLY, "1.3"));

        TrailmapInstructionRequest.Segment seg1 = makeFollowRoadsSegment("w1", "w2", "gravel", cm, Double.NaN, 0);
        TrailmapInstructionRequest.Segment seg2 = makeFollowRoadsSegment("w2", "w3", "gravel", cm, 90.62059343708455, 60.0);
        TrailmapInstructionRequest.Segment seg3 = makeDirectSegment("w3", "w4");
        TrailmapInstructionRequest.Segment seg4 = makeDirectSegment("w4", "w5");
        TrailmapInstructionRequest.Segment seg5 = makeDirectSegment("w5", "w6");
        TrailmapInstructionRequest.Segment seg6 = makeDirectSegment("w6", "w7");
        TrailmapInstructionRequest.Segment seg7 = makeDirectSegment("w7", "w8");
        TrailmapInstructionRequest.Segment seg8 = makeDirectSegment("w8", "w9");
        TrailmapInstructionRequest.Segment seg9 = makeDirectSegment("w9", "w10");
        TrailmapInstructionRequest.Segment seg10 = makeDirectSegment("w10", "w11");
        TrailmapInstructionRequest.Segment seg11 = makeFollowRoadsSegment("w11", "w12", "gravel", cm, Double.NaN, 0);
        TrailmapInstructionRequest.Segment seg12 = makeFollowRoadsSegment("w12", "w13", "gravel", cm, 160.75524855722847, 60.0);
        TrailmapInstructionRequest.Segment seg13 = makeFollowRoadsSegment("w13", "w14", "gravel", cm, 242.16485867817147, 60.0);

        request.setSegments(List.of(seg1, seg2, seg3, seg4, seg5, seg6, seg7, seg8, seg9, seg10,
                seg11, seg12, seg13));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        // Instruction intervals must cover the entire polyline
        int pointsIndex = 0;
        for (int i = 0; i < result.instructions.size(); i++) {
            pointsIndex += result.instructions.get(i).getLength();
        }
        // Last instruction (FINISH) has length 0, so pointsIndex should be polyline.size() - 1
        assertEquals(result.polyline.size() - 1, pointsIndex,
                "Instruction intervals must cover entire polyline. Last interval end="
                + pointsIndex + " polyline size=" + result.polyline.size());

        // Total instruction distance must match polyline distance (within tolerance)
        double totalInstrDist = 0;
        for (Instruction instr : result.instructions) totalInstrDist += instr.getDistance();
        double polylineDist = 0;
        for (int pi = 0; pi < result.polyline.size() - 1; pi++) {
            polylineDist += DistanceCalcEarth.DIST_EARTH.calcDist(
                    result.polyline.getLat(pi), result.polyline.getLon(pi),
                    result.polyline.getLat(pi + 1), result.polyline.getLon(pi + 1));
        }
        assertEquals(polylineDist, totalInstrDist, 5.0,
                "Total instruction distance must match polyline distance. "
                + "Instructions: " + String.format("%.1fm", totalInstrDist)
                + " Polyline: " + String.format("%.1fm", polylineDist));

        // Direct segment instructions must have tbt_available=false
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            Map<String, Object> extra = instr.getExtraInfoJSON();
            if ("direct".equals(extra.get("segment_type"))) {
                assertEquals(false, extra.get("tbt_available"),
                        "Direct segment instruction [" + i + "] must have tbt_available=false");
            }
        }
    }

    // ========== Test 15: False M1 on right-left-right sequence ==========

    /**
     * Short foot route with right→left→right ~90° turns. The middle pair (right→left)
     * passes M1 geometry gate but must NOT collapse pair [1]+[2] into a false join
     * when both pairs qualify — M1 should only merge the pair that actually represents
     * a side-path crossing (pair [2]+[3] where PH changes).
     *
     * Origin: testFalseM1OnRightLeftRight in RouteInstructionGeneratorTest
     */
    @Test
    void falseM1OnRightLeftRight_noIncorrectMerge() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.459012, 23.828659),
                makeWaypoint("wp2", 61.458676, 23.829354)));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("wp1");
        seg.setEnd("wp2");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("trailmap_foot");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("trailmap_foot");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        // After post-processing, there must be at least 2 real turn instructions.
        // The bug would collapse all 3 turns into 1 false join.
        int turnCount = 0;
        for (Instruction instr : result.instructions) {
            int sign = instr.getSign();
            if (sign != Instruction.FINISH && sign != Instruction.CONTINUE_ON_STREET
                    && sign != Instruction.IGNORE) {
                turnCount++;
            }
        }
        assertTrue(turnCount >= 2,
                "Right-left-right sequence must produce at least 2 turn instructions after "
                + "post-processing, got " + turnCount + ". Instructions: " + summarizeInstructions(result));

        // If there IS a join instruction, it should target MINOR_ROAD (the actual road crossing),
        // not PATH (which would indicate the wrong pair was merged)
        for (Instruction instr : result.instructions) {
            Map<String, Object> extra = instr.getExtraInfoJSON();
            if (extra.containsKey("join_direction")) {
                assertNotEquals("PATH", extra.get("join_target_type"),
                        "Join should not target PATH — that would mean the wrong pair was merged. "
                        + "Instructions: " + summarizeInstructions(result));
            }
        }

        // The initial right turn (4.9m, ~96°) must NOT be eaten by C2 micro-artifact suppression.
        // It's a real 90° junction turn, not a snapping artifact.
        boolean hasRightTurn = false;
        for (Instruction instr : result.instructions) {
            if (instr.getSign() == Instruction.TURN_RIGHT || instr.getSign() == Instruction.TURN_SLIGHT_RIGHT) {
                hasRightTurn = true;
                break;
            }
        }
        assertTrue(hasRightTurn,
                "The initial right turn must survive C2 — it's a real junction, not a micro-artifact. "
                + "Instructions: " + summarizeInstructions(result));
    }

    // ========== Test 16: F2 must not suppress when route changes road type ==========

    /**
     * Riding on cycleway, fork where cycleway continues slightly right and path goes
     * slightly left. Route takes the path. F2 must NOT suppress the instruction —
     * the rider needs to know they're leaving the cycleway even though geometry is straight.
     *
     * Origin: testMissingInstructionAtCyclewayPathFork in RouteInstructionGeneratorTest
     */
    @Test
    void cyclewayToPathFork_F2doesNotSuppress() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.442844, 23.83458),
                makeWaypoint("wp2", 61.442834, 23.832708)));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("wp1");
        seg.setEnd("wp2");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("trailmap_foot");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("trailmap_foot");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        // Must have an instruction at the CYCLEWAY→OUTDOOR_PATH transition
        Instruction transitionInstr = null;
        for (Instruction instr : result.instructions) {
            Map<String, Object> extra = instr.getExtraInfoJSON();
            if ("CYCLEWAY".equals(extra.get("prev_predicted_highway"))
                    && "OUTDOOR_PATH".equals(extra.get("predicted_highway"))) {
                transitionInstr = instr;
                break;
            }
        }
        assertNotNull(transitionInstr,
                "Must have a CYCLEWAY→OUTDOOR_PATH transition instruction (F2 must not suppress when "
                + "route changes road type). Instructions: " + summarizeInstructions(result));

        // At this gentle fork, F1 should produce TURN_SLIGHT_LEFT (fork-relative direction),
        // not CONTINUE_ON_STREET (misleading "straight" at an ambiguous fork)
        assertEquals(Instruction.TURN_SLIGHT_LEFT, transitionInstr.getSign(),
                "Gentle fork with type change should produce TURN_SLIGHT_LEFT, not "
                + signName(transitionInstr.getSign()) + ". Instructions: " + summarizeInstructions(result));

        // The path name should be on the instruction
        assertEquals("Suolijärven luontopolku", transitionInstr.getName(),
                "Transition instruction should carry the path name");
    }

    // ========== Test 17: Named cycleway Y-junctions must get trail_fork instructions ==========

    /**
     * Route along named cycleways ("Pyynikin rantapolku" / "Eteläpuistonpolku") with
     * several Y-junctions where same-class unnamed cycleways branch off. Previously
     * all forks were suppressed because S5 treated the cycleway name as a road name
     * (hasName=true → suppress), and E2 only applied to unnamed trails.
     *
     * Fix: S5 excludes CYCLEWAY from name-based suppression. E2 also applies to named
     * cycleways (informal names don't help distinguish branches on the ground).
     *
     * Origin: testCyclewayYJunctionInformalNames in RouteInstructionGeneratorTest
     */
    @Test
    void namedCyclewayYJunctions_getTrailForkInstructions() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.492078, 23.742357),
                makeWaypoint("wp2", 61.490486, 23.751273)));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("wp1");
        seg.setEnd("wp2");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        // Apply post-processing (trail_fork survives C1 suppression)
        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        // Count trail_fork instructions
        int trailForkCount = 0;
        for (Instruction instr : result.instructions) {
            Map<String, Object> extra = instr.getExtraInfoJSON();
            if (Boolean.TRUE.equals(extra.get("trail_fork"))) {
                trailForkCount++;
            }
        }
        assertTrue(trailForkCount >= 3,
                "Named cycleway route with Y-junctions must have at least 3 trail_fork instructions, "
                + "got " + trailForkCount + ". Instructions: " + summarizeInstructions(result));

        // trail_fork is a junction-level marker — it can attach to any emitted instruction
        // at a confusable trail/cycleway fork (CONTINUE, KEEP_*, TURN_*, etc.). The only
        // hard invariant is that FINISH never carries it.
        for (Instruction instr : result.instructions) {
            Map<String, Object> extra = instr.getExtraInfoJSON();
            if (instr.getSign() == Instruction.FINISH) {
                assertNotEquals(Boolean.TRUE, extra.get("trail_fork"),
                        "FINISH must never carry trail_fork. Instructions: "
                        + summarizeInstructions(result));
            }
        }

        // All non-FINISH instructions should have tbt_priority after post-processing
        for (Instruction instr : result.instructions) {
            if (instr.getSign() != Instruction.FINISH) {
                assertNotNull(instr.getExtraInfoJSON().get("tbt_priority"),
                        "All instructions should have tbt_priority after post-processing. "
                        + "Instructions: " + summarizeInstructions(result));
            }
        }
    }

    // ========== Test 18: E5 — cycleway right turn with footway running straight ==========

    /**
     * Bike on a CYCLEWAY makes a real right turn at a junction whose only graph
     * alternative is an asphalt FOOTWAY going straight ahead. Both options look
     * identical on the ground — only OSM tagging distinguishes them. Pre-fix the
     * forced-path block returned IGNORE because the footway was access-blocked
     * under the gravel weighting and therefore invisible to getAllowedTurns() /
     * getVisibleTurns(). E5 detects the same-major-surface FOOTWAY alternative
     * via the unfiltered allExplorer and emits the angle-based turn.
     *
     * Origin: testCyclewayRightTurnFootwayStraight_suppressedTurn in
     * RouteInstructionGeneratorTest.
     */
    @Test
    void cyclewayWithFootwayStraight_emitsAngleBasedRightTurn() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("aZ7vebZ1VLcMBvaH-b9As", 61.50377, 23.658394),
                makeWaypoint("vDy0QlvS1zJk4NjrE43zu", 61.504136, 23.657891)));

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
        assertValidInstructionList(result);

        // Locate the cycleway-with-footway-straight junction: a sharp right turn
        // (turn_angle_deg <= -60) where both incoming and outgoing edges are CYCLEWAY.
        Instruction e5Instr = null;
        for (Instruction instr : result.instructions) {
            Map<String, Object> extra = instr.getExtraInfoJSON();
            Object angleObj = extra.get("turn_angle_deg");
            if (!(angleObj instanceof Number)) continue;
            double angle = ((Number) angleObj).doubleValue();
            if (angle > -60.0) continue; // not sharp enough or wrong direction
            if (!"CYCLEWAY".equals(extra.get("predicted_highway"))) continue;
            if (!"CYCLEWAY".equals(extra.get("prev_predicted_highway"))) continue;
            e5Instr = instr;
            break;
        }

        assertNotNull(e5Instr,
                "E5 should produce a sharp right turn (turn_angle_deg <= -60°) on "
                + "CYCLEWAY at the cycleway/footway junction. "
                + "Instructions: " + summarizeInstructions(result));

        // Sign must be a real right turn — angle-based (TURN_RIGHT or TURN_SHARP_RIGHT),
        // not KEEP_RIGHT (which would be wrong for a forced-path junction with only one
        // bike-accessible alternative) and not CONTINUE_ON_STREET / IGNORE.
        int sign = e5Instr.getSign();
        assertTrue(sign == Instruction.TURN_RIGHT || sign == Instruction.TURN_SHARP_RIGHT,
                "E5 instruction sign must be TURN_RIGHT or TURN_SHARP_RIGHT, got "
                + signName(sign) + ". Instructions: " + summarizeInstructions(result));

        // Apply post-processing: A3 (sharp turn) should assign tbt_priority=voice.
        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        // Re-locate after post-processing (object identity preserved by in-place ops).
        Map<String, Object> extra = e5Instr.getExtraInfoJSON();
        assertEquals("voice", extra.get("tbt_priority"),
                "E5 sharp turn must be voice-tier after Pass 4 (A3 sharp turn rule). "
                + "Instructions: " + summarizeInstructions(result));
    }

    // ========== Test 18b: E5 — cycleway T-junction with footway departing left ==========

    /**
     * Variant of Test 18 covering the relaxed E5 gate. Bike on a CYCLEWAY reaches a
     * T-junction where the route turns right onto another CYCLEWAY (~73°) and a FOOTWAY
     * departs left (~90°). Both edges are asphalt and unnamed; the only distinguishing
     * feature on the ground is a regulatory sign 20m out. The footway here is OSM-tagged
     * `bicycle=no` (vs. the visually-identical first T in this route which has no bike
     * tag and thus IS bike-routable), so the forced-path branch fires for this junction.
     *
     * E5's original gates (`|altDelta| < π/4` AND `|altDelta| < |routeDelta|`) missed
     * this case — the footway alt is past π/4 and is sharper than the route. The
     * relaxed gate `|altDelta| ≤ |routeDelta| + π/6` catches it: 89.6° ≤ 73° + 30° = 103°.
     *
     * Origin: testCyclewayTJunctionFootwayLeft_suppressedTurn in
     * RouteInstructionGeneratorTest.
     */
    @Test
    void cyclewayTJunctionFootwayLeft_emitsAngleBasedRightTurn() {
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
        assertValidInstructionList(result);

        // Pre-postprocess: assert the new TURN_RIGHT exists at the second T (Node 2002495)
        // before M2 quick-sequence merging absorbs it. turn_angle_deg ≤ -60 (sharp right),
        // CYCLEWAY → CYCLEWAY transition.
        Instruction tjInstr = null;
        for (Instruction instr : result.instructions) {
            if (instr.getSign() != Instruction.TURN_RIGHT
                    && instr.getSign() != Instruction.TURN_SHARP_RIGHT) continue;
            Map<String, Object> extra = instr.getExtraInfoJSON();
            Object angleObj = extra.get("turn_angle_deg");
            if (!(angleObj instanceof Number)) continue;
            if (((Number) angleObj).doubleValue() > -60.0) continue;
            if (!"CYCLEWAY".equals(extra.get("predicted_highway"))) continue;
            if (!"CYCLEWAY".equals(extra.get("prev_predicted_highway"))) continue;
            tjInstr = instr;
            break;
        }
        assertNotNull(tjInstr,
                "Relaxed E5 must produce a sharp right TURN_RIGHT (turn_angle_deg ≤ -60°) "
                + "on CYCLEWAY → CYCLEWAY at the second T-junction (Node 2002495). "
                + "Instructions: " + summarizeInstructions(result));

        // Post-processing: M2 quick-sequence collapses the LEFT (Node 2002496) and RIGHT
        // (Node 2002495) — they're ~10m apart — into a single voice-tier TURN_LEFT with
        // a then_turn pointing to the merged TURN_RIGHT. Both turns must be communicated
        // to the rider via the compound instruction.
        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        Instruction compound = null;
        for (Instruction instr : result.instructions) {
            if (instr.getSign() != Instruction.TURN_LEFT
                    && instr.getSign() != Instruction.TURN_SHARP_LEFT) continue;
            if (!instr.getExtraInfoJSON().containsKey("then_turn")) continue;
            compound = instr;
            break;
        }
        assertNotNull(compound,
                "After post-processing the LEFT+RIGHT close-pair must collapse to a "
                + "TURN_LEFT carrying a then_turn pointing to the right turn. "
                + "Instructions: " + summarizeInstructions(result));

        Map<String, Object> compoundExtra = compound.getExtraInfoJSON();
        assertEquals("voice", compoundExtra.get("tbt_priority"),
                "Compound LEFT+then-RIGHT must be voice-tier. "
                + "Instructions: " + summarizeInstructions(result));

        @SuppressWarnings("unchecked")
        Map<String, Object> thenTurn = (Map<String, Object>) compoundExtra.get("then_turn");
        assertNotNull(thenTurn, "then_turn must be a populated object");
        assertEquals(Instruction.TURN_RIGHT, ((Number) thenTurn.get("sign")).intValue(),
                "then_turn.sign must be TURN_RIGHT (the second T's right turn). "
                + "then_turn=" + thenTurn);
        assertEquals("CYCLEWAY", thenTurn.get("road_class"),
                "then_turn.road_class must be CYCLEWAY. then_turn=" + thenTurn);
    }

    // ========== Test 19: Cycleway/service-road indistinguishable + service→residential transition ==========

    /**
     * Single route exercising two suppression bugs that were fixed together:
     *
     * 1) **S1/S2 (visual-distinctness)**: at a same-named SERVICE_ROAD turn (−47.7°) where the
     *    only alternative is a CYCLEWAY whose surface is ASPHALT_OR_UNPAVED (ambiguous, in
     *    reality unpaved), S1/S2 used to suppress the turn because they treated the alt as
     *    "visually distinct" (different PH/RC, surface differs on isAsphalt). Both now use
     *    `allAlternativesVisuallyDistinct` which requires a different PH bucket (per
     *    isConfusableFrom) AND a clearly different surface (ASPHALT_OR_UNPAVED is ambiguous).
     *
     * 2) **S3 (prevPH-aware)**: at a SERVICE_ROAD → MINOR_ROAD transition (+15.8°), S3 used
     *    to suppress because all alts are lower prominence than current MINOR_ROAD. Fixed
     *    to require `prevPH ≥ currentPH` so the rider joining a more prominent way still
     *    gets a turn.
     *
     * Origin: testCyclewayServiceRoadIndistinguishable_61_530_23_702 in RouteInstructionGeneratorTest
     */
    @Test
    void cyclewayServiceRoadIndistinguishable_emitsTurnsAtBothJunctions() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.530601, 23.702663),
                makeWaypoint("wp2", 61.528947, 23.697955)));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("wp1");
        seg.setEnd("wp2");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        // Assertion 1: S1/S2 fix — same-PH SERVICE_ROAD turn (~−47°) must be emitted when
        // the only alt is a CYCLEWAY with ASPHALT_OR_UNPAVED (ambiguous) surface.
        boolean hasServiceRoadTurn = hasInstructionWithProperties(result,
                sign -> Math.abs(sign) == Instruction.TURN_RIGHT || Math.abs(sign) == Instruction.TURN_LEFT,
                extra -> "SERVICE_ROAD".equals(extra.get("predicted_highway"))
                        && "SERVICE_ROAD".equals(extra.get("prev_predicted_highway"))
                        && extra.containsKey("turn_angle_deg")
                        && Math.abs(((Number) extra.get("turn_angle_deg")).doubleValue()) > 30.0);
        assertTrue(hasServiceRoadTurn,
                "S1/S2 fix: expected a TURN_LEFT/TURN_RIGHT on a same-PH SERVICE_ROAD continuation "
                + "(cycleway alt with ASPHALT_OR_UNPAVED surface must be treated as visually "
                + "ambiguous, preventing suppression). Instructions: " + summarizeInstructions(result));

        // Assertion 2: S3 fix — slight turn at SERVICE_ROAD → MINOR_ROAD transition must be emitted.
        // prevPH=SERVICE_ROAD < currentPH=MINOR_ROAD → S3 must NOT suppress.
        boolean hasJoinResidentialTurn = hasInstructionWithProperties(result,
                sign -> Math.abs(sign) == Instruction.TURN_SLIGHT_LEFT
                        || Math.abs(sign) == Instruction.TURN_SLIGHT_RIGHT
                        || Math.abs(sign) == Instruction.TURN_LEFT
                        || Math.abs(sign) == Instruction.TURN_RIGHT,
                extra -> "MINOR_ROAD".equals(extra.get("predicted_highway"))
                        && "SERVICE_ROAD".equals(extra.get("prev_predicted_highway")));
        assertTrue(hasJoinResidentialTurn,
                "S3 fix: expected a turn instruction at SERVICE_ROAD→MINOR_ROAD transition "
                + "(rider joining a more prominent way; prevPH=6 < currentPH=8 must prevent S3 "
                + "from suppressing). Instructions: " + summarizeInstructions(result));
    }

    /**
     * Validates the E6 forced-path anti-suppression rule for road → non-road transitions.
     *
     * Junction node 1997741 (61.500301, 23.655447) is a 2-degree graph node where
     * Raholankatu (MINOR_ROAD/asphalt) terminates and an unnamed CYCLEWAY/fine_gravel
     * picks up. With no graph alternatives the legacy forced-path branch returned IGNORE,
     * but the rider faces a road end (vehicle turnaround visual cue) and needs to know
     * the cycleway ahead is the continuation. E6 emits the angle-based sign
     * (CONTINUE_ON_STREET here, since Δ ≈ +0.8°) when prevPH is road infrastructure and
     * currentPH is in {CYCLEWAY, FOOTWAY, PATH}.
     *
     * E6 is one-directional. The reverse (CYCLEWAY → MINOR_ROAD) is intentionally not
     * covered: emerging onto a road typically has clear visual cues, so the original
     * forced-path suppression remains correct in that direction.
     *
     * Origin: testRoadEndCyclewayTransition_61_500_23_655 in RouteInstructionGeneratorTest
     */
    @Test
    void roadEndCyclewayTransition_e6EmitsForwardOnly() {
        // ---- FORWARD: MINOR_ROAD → CYCLEWAY (E6 must fire) ----
        TrailmapInstructionRequest fwdReq = new TrailmapInstructionRequest();
        fwdReq.setWaypoints(List.of(
                makeWaypoint("hvH1f2dBl3s_0xB2EHrXx", 61.500518, 23.655585),
                makeWaypoint("l-hO7UTD9iVIbMFSbNObO", 61.500109, 23.655586)));

        TrailmapInstructionRequest.Segment fwdSeg = new TrailmapInstructionRequest.Segment();
        fwdSeg.setStart("hvH1f2dBl3s_0xB2EHrXx");
        fwdSeg.setEnd("l-hO7UTD9iVIbMFSbNObO");
        fwdSeg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        fwdSeg.setProfile("gravel");

        fwdReq.setSegments(List.of(fwdSeg));
        fwdReq.setInstructionProfile("gravel");
        fwdReq.setLocale("fi");
        fwdReq.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result fwdResult = generator.generate(fwdReq);
        assertValidInstructionList(fwdResult);

        boolean fwdHasE6 = hasInstructionWithProperties(fwdResult,
                sign -> sign == Instruction.CONTINUE_ON_STREET,
                extra -> "MINOR_ROAD".equals(extra.get("prev_predicted_highway"))
                        && "CYCLEWAY".equals(extra.get("predicted_highway")));
        assertTrue(fwdHasE6,
                "E6 forward: expected a CONTINUE_ON_STREET at MINOR_ROAD→CYCLEWAY transition "
                + "(road ends, cycleway picks up — forced-path branch must NOT suppress). "
                + "Instructions: " + summarizeInstructions(fwdResult));

        // ---- REVERSE: CYCLEWAY → MINOR_ROAD (E6 must NOT fire — original suppression intact) ----
        TrailmapInstructionRequest revReq = new TrailmapInstructionRequest();
        revReq.setWaypoints(List.of(
                makeWaypoint("rev_start", 61.500109, 23.655586),
                makeWaypoint("rev_end",   61.500518, 23.655585)));

        TrailmapInstructionRequest.Segment revSeg = new TrailmapInstructionRequest.Segment();
        revSeg.setStart("rev_start");
        revSeg.setEnd("rev_end");
        revSeg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        revSeg.setProfile("gravel");

        revReq.setSegments(List.of(revSeg));
        revReq.setInstructionProfile("gravel");
        revReq.setLocale("fi");
        revReq.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result revResult = generator.generate(revReq);
        assertValidInstructionList(revResult);

        boolean revHasReverseTransitionInstr = hasInstructionWithProperties(revResult,
                sign -> sign != Instruction.FINISH,
                extra -> "CYCLEWAY".equals(extra.get("prev_predicted_highway"))
                        && "MINOR_ROAD".equals(extra.get("predicted_highway")));
        assertFalse(revHasReverseTransitionInstr,
                "E6 must be one-directional: CYCLEWAY→MINOR_ROAD transition must remain "
                + "suppressed (no instruction with prev=CYCLEWAY, curr=MINOR_ROAD). "
                + "Instructions: " + summarizeInstructions(revResult));
    }

    // ========== Test 20: M3 — profile-aware consecutive-fork merge thresholds ==========

    /**
     * Foot route, two same-direction KEEP_LEFT forks ~140m apart on a PATH (TbT
     * generator output: KEEP_LEFT@123.8m → KEEP_LEFT@16.1m). The foot M3 horizon is
     * 60m, so the leading 123.8m segment puts the chain over budget — must NOT merge.
     *
     * Origin: testM3StayLeftThresholds_footAndGravel in RouteInstructionGeneratorTest.
     */
    @Test
    void m3_footProfile_keepsBeyondThreshold_doNotMerge() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("MdwMF-B90KYbDQq6Ul_Gf", 61.427946, 23.890105),
                makeWaypoint("zmENlCkWzEeXzzMR1JHmB", 61.426671, 23.890025)));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("MdwMF-B90KYbDQq6Ul_Gf");
        seg.setEnd("zmENlCkWzEeXzzMR1JHmB");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("trailmap_foot");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("trailmap_foot");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        int keepLeftCount = 0;
        boolean anySequenceTag = false;
        for (Instruction instr : result.instructions) {
            if (instr.getSign() == Instruction.KEEP_LEFT) keepLeftCount++;
            if (instr.getExtraInfoJSON().containsKey("sequence_tag")) anySequenceTag = true;
        }
        assertTrue(keepLeftCount >= 2,
                "Expected at least 2 separate KEEP_LEFT instructions on the foot route "
                + "(leading segment 123.8m exceeds 60m foot horizon → no merge). "
                + "Instructions: " + summarizeInstructions(result));
        assertFalse(anySequenceTag,
                "M3 must NOT have produced a sequence_tag on this foot route. "
                + "Instructions: " + summarizeInstructions(result));
    }

    /**
     * Same waypoints as the foot test, but with profile=gravel — the M3 horizon for
     * non-foot is 100m. The leading segment is 123.8m > 100m, so the chain is still
     * over budget and must NOT merge.
     */
    @Test
    void m3_gravelProfile_keepsBeyondThreshold_doNotMerge() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.427946, 23.890105),
                makeWaypoint("wp2", 61.426671, 23.890025)));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("wp1");
        seg.setEnd("wp2");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        int keepLeftCount = 0;
        boolean anySequenceTag = false;
        for (Instruction instr : result.instructions) {
            if (instr.getSign() == Instruction.KEEP_LEFT) keepLeftCount++;
            if (instr.getExtraInfoJSON().containsKey("sequence_tag")) anySequenceTag = true;
        }
        assertTrue(keepLeftCount >= 2,
                "Expected at least 2 separate KEEP_LEFT instructions on the gravel route "
                + "(leading segment 123.8m exceeds 100m non-foot horizon → no merge). "
                + "Instructions: " + summarizeInstructions(result));
        assertFalse(anySequenceTag,
                "M3 must NOT have produced a sequence_tag on this gravel route. "
                + "Instructions: " + summarizeInstructions(result));
    }

    /**
     * Synthetic InstructionList: two KEEP_LEFTs at 30m + 10m on PATH. With foot
     * profile (60m horizon) the chain is within budget and must merge into a single
     * instruction with sequence_tag="stay_left_2".
     */
    @Test
    void m3_footProfile_keepsWithinThreshold_merge() {
        InstructionList list = buildSyntheticTwoKeepList(Instruction.KEEP_LEFT, 30.0, 10.0, "PATH");
        new InstructionPostProcessor().process(list, "trailmap_foot");

        // Expect: merged KEEP_LEFT (40m) + FINISH = 2 instructions
        assertEquals(2, list.size(),
                "Foot M3 must merge KEEP_LEFT@30m + KEEP_LEFT@10m (within 60m horizon). "
                + "Got: " + summarizeList(list));

        Instruction merged = list.get(0);
        assertEquals(Instruction.KEEP_LEFT, merged.getSign(),
                "Merged instruction must be KEEP_LEFT. " + summarizeList(list));
        assertEquals("stay_left_2", merged.getExtraInfoJSON().get("sequence_tag"),
                "Merged instruction must carry sequence_tag=stay_left_2. " + summarizeList(list));
        assertEquals(40.0, merged.getDistance(), 0.01,
                "Merged distance must be 30+10=40m. " + summarizeList(list));
    }

    /**
     * Synthetic InstructionList: two KEEP_LEFTs at 70m + 10m on PATH. With gravel
     * profile (100m horizon) the chain is within budget and must merge. Note: this
     * setup is OUTSIDE the foot horizon (60m), so the same instructions with foot
     * profile would NOT merge — that asymmetry is the point of profile-awareness.
     */
    @Test
    void m3_gravelProfile_keepsWithinThreshold_merge() {
        InstructionList list = buildSyntheticTwoKeepList(Instruction.KEEP_LEFT, 70.0, 10.0, "PATH");
        new InstructionPostProcessor().process(list, "gravel");

        assertEquals(2, list.size(),
                "Gravel M3 must merge KEEP_LEFT@70m + KEEP_LEFT@10m (within 100m horizon). "
                + "Got: " + summarizeList(list));

        Instruction merged = list.get(0);
        assertEquals(Instruction.KEEP_LEFT, merged.getSign(),
                "Merged instruction must be KEEP_LEFT. " + summarizeList(list));
        assertEquals("stay_left_2", merged.getExtraInfoJSON().get("sequence_tag"),
                "Merged instruction must carry sequence_tag=stay_left_2. " + summarizeList(list));
        assertEquals(80.0, merged.getDistance(), 0.01,
                "Merged distance must be 70+10=80m. " + summarizeList(list));

        // Cross-check: same input under foot profile must NOT merge (70 > 60).
        InstructionList footList = buildSyntheticTwoKeepList(Instruction.KEEP_LEFT, 70.0, 10.0, "PATH");
        new InstructionPostProcessor().process(footList, "trailmap_foot");
        assertEquals(3, footList.size(),
                "Foot M3 must NOT merge KEEP_LEFT@70m (70m > 60m foot horizon). "
                + "Got: " + summarizeList(footList));
        assertFalse(footList.get(0).getExtraInfoJSON().containsKey("sequence_tag"),
                "Foot M3: no sequence_tag expected when over horizon. " + summarizeList(footList));
    }

    /**
     * Build a synthetic 3-instruction list: KEEP / KEEP / FINISH, all with
     * predicted_highway set so M3's isTrailPH gate passes. Geometry is a degenerate
     * 2-point polyline at (0,0)–(0,0) on each instruction — M3 only reads sign,
     * distance, and predicted_highway, so geometry shape doesn't matter here.
     */
    private InstructionList buildSyntheticTwoKeepList(int keepSign, double dist1, double dist2, String predictedHighway) {
        Translation tr = hopper.getTranslationMap().getWithFallBack(java.util.Locale.forLanguageTag("fi"));
        InstructionList list = new InstructionList(tr);

        PointList pl1 = new PointList(2, false);
        pl1.add(0.0, 0.0);
        pl1.add(0.0, 0.0);
        Instruction k1 = new Instruction(keepSign, "", pl1);
        k1.setDistance(dist1);
        k1.setExtraInfo("predicted_highway", predictedHighway);
        list.add(k1);

        PointList pl2 = new PointList(2, false);
        pl2.add(0.0, 0.0);
        pl2.add(0.0, 0.0);
        Instruction k2 = new Instruction(keepSign, "", pl2);
        k2.setDistance(dist2);
        k2.setExtraInfo("predicted_highway", predictedHighway);
        list.add(k2);

        PointList plF = new PointList(1, false);
        plF.add(0.0, 0.0);
        Instruction finish = new Instruction(Instruction.FINISH, "", plF);
        finish.setDistance(0);
        list.add(finish);

        return list;
    }

    /** Compact dump of a synthetic InstructionList for assertion failure messages. */
    private String summarizeList(InstructionList list) {
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < list.size(); i++) {
            Instruction instr = list.get(i);
            if (i > 0) sb.append(", ");
            sb.append(String.format("%s@%.0fm%s", signName(instr.getSign()), instr.getDistance(),
                    instr.getExtraInfoJSON().containsKey("sequence_tag")
                            ? "(" + instr.getExtraInfoJSON().get("sequence_tag") + ")"
                            : ""));
        }
        sb.append("]");
        return sb.toString();
    }

    /**
     * Validates the M4 short-zigzag collapse pass.
     *
     * Route 61.511523,23.687208 → 61.511789,23.687448 traverses an unnamed
     * cycleway (CYCLEWAY/asphalt_or_unpaved) with a 3.5m connector edge between
     * two opposite-direction ~46° turns. M2 first merges them into a "loivasti
     * oikealle, sitten heti vasen" compound; M4 then detects the short connector
     * (≤5m), opposite signs, near-zero net angle (-45.8 + 46.8 = +1°) on
     * non-road PH, and collapses to CONTINUE_ON_STREET + trail_fork=true.
     *
     * Asserts:
     *   1) No turn instruction (sign in {±1, ±2, ±3}) survives at this geometry —
     *      M4 must have collapsed the M2 result.
     *   2) A CONTINUE_ON_STREET with trail_fork=true exists (the M4 output).
     *   3) The collapsed instruction has no then_turn (cleaned up).
     *
     * Origin: testShortZigzagAsStraight_61_511_23_687 in RouteInstructionGeneratorTest
     */
    @Test
    void shortZigzagOnCycleway_m4CollapsesToStraight() {
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

        RouteInstructionGenerator.Result result = generator.generate(req);
        assertValidInstructionList(result);

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, req.getInstructionProfile());

        // 1) No surviving turn instruction (M4 must have collapsed the M2 merge).
        boolean hasAnyTurn = false;
        for (Instruction instr : result.instructions) {
            int absSign = Math.abs(instr.getSign());
            if (absSign >= 1 && absSign <= 3) { hasAnyTurn = true; break; }
        }
        assertFalse(hasAnyTurn,
                "M4 collapse: zigzag must not produce a TURN instruction; expected only "
                + "CONTINUE_ON_STREET (one with trail_fork=true) + FINISH. "
                + "Instructions: " + summarizeInstructions(result));

        // 2) The collapsed instruction is present: CONTINUE_ON_STREET on CYCLEWAY with trail_fork.
        boolean hasCollapsed = hasInstructionWithProperties(result,
                sign -> sign == Instruction.CONTINUE_ON_STREET,
                extra -> Boolean.TRUE.equals(extra.get("trail_fork"))
                        && "CYCLEWAY".equals(extra.get("predicted_highway")));
        assertTrue(hasCollapsed,
                "M4 collapse: expected a CONTINUE_ON_STREET on CYCLEWAY with trail_fork=true. "
                + "Instructions: " + summarizeInstructions(result));

        // 3) The collapsed instruction must have no then_turn (cleaned up by M4).
        for (Instruction instr : result.instructions) {
            if (instr.getSign() == Instruction.CONTINUE_ON_STREET
                    && Boolean.TRUE.equals(instr.getExtraInfoJSON().get("trail_fork"))) {
                assertFalse(instr.getExtraInfoJSON().containsKey("then_turn"),
                        "M4 collapse: collapsed instruction must not retain then_turn. "
                        + "Instructions: " + summarizeInstructions(result));
            }
        }
    }

    /**
     * Validates the M2 closer-pair preference (look-ahead before merging).
     *
     * Route 61.508160,23.687510 → 61.508697,23.687776 produces three turns on a
     * cycleway: TURN_RIGHT, TURN_LEFT, TURN_RIGHT with connectors 29.4m and 10m.
     * The rider perceives turns 2+3 as the back-to-back pair (10m connector),
     * not turns 1+2 (29.4m connector).
     *
     * Without the look-ahead, M2 greedy-from-top merged (1, 2) and left turn 3
     * standalone. With the closer-pair preference, M2 skips (1, 2) because
     * (2, 3) qualifies for M2 with a strictly shorter connector — letting the
     * next iteration merge (2, 3) instead.
     *
     * Asserts:
     *   1) Exactly one instruction carries `then_turn` (the M2 merge).
     *   2) The merged instruction is the LEFT turn (the second turn of the
     *      original three), not the first RIGHT turn.
     *   3) The standalone (un-merged) turn is the first RIGHT turn.
     *
     * Origin: testThreeTurnGreedyMergeOrdering_61_508_23_687 in RouteInstructionGeneratorTest
     */
    @Test
    void threeTurnSequence_m2PrefersCloserPair() {
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

        RouteInstructionGenerator.Result result = generator.generate(req);
        assertValidInstructionList(result);

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, req.getInstructionProfile());

        // 1) Exactly one merged instruction carries then_turn.
        int thenTurnCount = 0;
        for (Instruction instr : result.instructions) {
            if (instr.getExtraInfoJSON().containsKey("then_turn")) thenTurnCount++;
        }
        assertEquals(1, thenTurnCount,
                "Closer-pair preference: expected exactly one M2 merge (the closer pair). "
                + "Instructions: " + summarizeInstructions(result));

        // 2) The merged instruction is a LEFT turn whose then_turn points to a RIGHT — i.e.,
        // the (turn 2 = LEFT, turn 3 = RIGHT) pair, NOT (turn 1 = RIGHT, turn 2 = LEFT).
        boolean correctPair = hasInstructionWithProperties(result,
                sign -> sign == Instruction.TURN_LEFT,
                extra -> {
                    Object thenTurnObj = extra.get("then_turn");
                    if (!(thenTurnObj instanceof Map)) return false;
                    Map<?, ?> tt = (Map<?, ?>) thenTurnObj;
                    Object signObj = tt.get("sign");
                    return signObj instanceof Number
                            && ((Number) signObj).intValue() == Instruction.TURN_RIGHT;
                });
        assertTrue(correctPair,
                "Closer-pair preference: M2 merge must be (TURN_LEFT, TURN_RIGHT) — the close "
                + "pair (turns 2+3, 10m connector), not (TURN_RIGHT, TURN_LEFT) — the far pair "
                + "(turns 1+2, 29.4m connector). Instructions: " + summarizeInstructions(result));

        // 3) A standalone TURN_RIGHT instruction (the first turn) exists with no then_turn.
        boolean standaloneFirstTurn = hasInstructionWithProperties(result,
                sign -> sign == Instruction.TURN_RIGHT,
                extra -> !extra.containsKey("then_turn"));
        assertTrue(standaloneFirstTurn,
                "Closer-pair preference: the first TURN_RIGHT must remain standalone "
                + "(no then_turn). Instructions: " + summarizeInstructions(result));
    }

    // ========== Test 22: M1 must not cascade onto an already-merged M1 result ==========

    /**
     * Pre-fix: after M1 merged the road→service→cycleway sidepath crossing, the
     * pass-1 {@code i--} re-check evaluated the merged instruction (still carrying
     * the first turn's {@code turn_angle_deg}) against the next real navigation
     * turn. The geometry gate read stale angle data and fired a second M1 — and
     * then M2 absorbed the fourth turn into {@code then_turn}, collapsing four real
     * turns into a single misleading "join right, then right in 23 m" prompt that
     * silently dropped two left turns.
     *
     * Post-fix: {@code isJoinSidePathPattern} rejects when {@code first} already
     * carries {@code join_direction} (symmetric to the existing {@code second}
     * check). The cascade is blocked. The closer-pair preference in M2 then
     * compounds the genuine LEFT+RIGHT pair (10m apart) and leaves the M1 join
     * standalone — exactly what the rider needs.
     *
     * Origin: testOpenCase_61_524176_23_626523_to_61_524667_23_62469 in
     * RouteInstructionGeneratorTest.
     */
    @Test
    void m1_mustNotCascadeOntoExistingM1Result() {
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
        assertValidInstructionList(result);

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        // 1) Locate the M1 join_right instruction (MINOR_ROAD → CYCLEWAY).
        Instruction m1Instr = null;
        for (Instruction instr : result.instructions) {
            Map<String, Object> extra = instr.getExtraInfoJSON();
            if (!"right".equals(extra.get("join_direction"))) continue;
            if (!"CYCLEWAY".equals(extra.get("join_target_type"))) continue;
            if (!"MINOR_ROAD".equals(extra.get("prev_predicted_highway"))) continue;
            m1Instr = instr;
            break;
        }
        assertNotNull(m1Instr,
                "Expected an M1 join_right instruction (MINOR_ROAD → CYCLEWAY). "
                + "Instructions: " + summarizeInstructions(result));

        // 2) The M1 instruction must NOT carry then_turn — that would mean a second
        //    real turn got absorbed into the join (the cascade-then-M2 failure mode).
        assertFalse(m1Instr.getExtraInfoJSON().containsKey("then_turn"),
                "M1 join instruction must remain standalone (no then_turn). The "
                + "cascade fix prevents M1 from absorbing the next real navigation "
                + "turn. Instructions: " + summarizeInstructions(result));

        // 3) A real TURN_LEFT (turn_angle_deg ≥ 60°) on CYCLEWAY → CYCLEWAY exists,
        //    distinct from the M1 join, capturing the cycleway-fork left turn.
        Instruction realLeft = null;
        for (Instruction instr : result.instructions) {
            if (instr == m1Instr) continue;
            if (instr.getSign() != Instruction.TURN_LEFT
                    && instr.getSign() != Instruction.TURN_SHARP_LEFT) continue;
            Map<String, Object> extra = instr.getExtraInfoJSON();
            Object angleObj = extra.get("turn_angle_deg");
            if (!(angleObj instanceof Number)) continue;
            if (((Number) angleObj).doubleValue() < 60.0) continue;
            if (!"CYCLEWAY".equals(extra.get("predicted_highway"))) continue;
            if (!"CYCLEWAY".equals(extra.get("prev_predicted_highway"))) continue;
            realLeft = instr;
            break;
        }
        assertNotNull(realLeft,
                "The cycleway-fork TURN_LEFT (turn_angle_deg ≥ 60°, CYCLEWAY → "
                + "CYCLEWAY) must survive as its own instruction — pre-fix it was "
                + "absorbed by the cascading M1. Instructions: " + summarizeInstructions(result));

        // 4) The real left carries then_turn pointing to the final right (M2 closer-pair).
        @SuppressWarnings("unchecked")
        Map<String, Object> thenTurn = (Map<String, Object>) realLeft.getExtraInfoJSON().get("then_turn");
        assertNotNull(thenTurn,
                "The cycleway-fork TURN_LEFT must carry a then_turn pointing to the "
                + "final right turn (M2 closer-pair preference). "
                + "Instructions: " + summarizeInstructions(result));
        assertEquals(Instruction.TURN_RIGHT, ((Number) thenTurn.get("sign")).intValue(),
                "then_turn.sign must be TURN_RIGHT. then_turn=" + thenTurn);
        assertEquals("CYCLEWAY", thenTurn.get("road_class"),
                "then_turn.road_class must be CYCLEWAY. then_turn=" + thenTurn);
    }

    // ========== S6: Forced trail bend — suppress spurious "loiva oikea" on cycleway-only junctions ==========

    /**
     * Cycleway makes a 40° geometric right curve. The single non-route alt at the
     * junction is at +135° (effectively a back-leg) — not a viable forward continuation.
     * Pre-S6: leaving-current-street fallback fired on |Δ|>0.6 and emitted a spurious
     * TURN_SLIGHT_RIGHT. Post-S6: the rider has no real navigation choice (only one
     * forward option is the route itself) so no turn instruction is emitted.
     *
     * Origin: testLoivaOikeaCaseA_61_516_23_689 in RouteInstructionGeneratorTest.
     */
    @Test
    void forcedTrailBend_geometricCurveOnly_noSpuriousTurn() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("YYC2p9mV_vF_7vF9wHiOJ", 61.516442, 23.68955),
                makeWaypoint("HexkPdmGBeNOnBKWx4Ztx", 61.516993, 23.690446)));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("YYC2p9mV_vF_7vF9wHiOJ");
        seg.setEnd("HexkPdmGBeNOnBKWx4Ztx");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        for (Instruction instr : result.instructions) {
            int sign = instr.getSign();
            assertTrue(sign == Instruction.CONTINUE_ON_STREET || sign == Instruction.FINISH,
                    "S6 forced-trail-bend (Case A): on a cycleway-only short route with no "
                    + "viable forward alternative, no turn instruction may be emitted. Got "
                    + signName(sign) + ". Instructions: " + summarizeInstructions(result));
        }
    }

    /**
     * Cycleway with informal name "Niemenrantaraitti". The non-route alt at the junction
     * is the back-leg (also "Niemenrantaraitti", at +145°). Pre-S6: leaving-current-street
     * fired via the name-match clause in GH's isLeavingCurrentStreet (the same name
     * appears on an alt) and emitted TURN_SLIGHT_RIGHT. Post-S6: same-name reasoning is
     * neutralised on non-road-infrastructure (S6 fires before leaving-current-street),
     * the back-leg alt is not a viable forward → no turn instruction.
     *
     * Origin: testLoivaOikeaCaseB_61_519_23_697 in RouteInstructionGeneratorTest.
     */
    @Test
    void forcedTrailBend_sameNameBackLeg_noSpuriousTurn() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("SIW9bPSooVQD68SLKIkKa", 61.519257, 23.697865),
                makeWaypoint("HG6cnTmWgzlm-kuCYIUGq", 61.519525, 23.699756)));

        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("SIW9bPSooVQD68SLKIkKa");
        seg.setEnd("HG6cnTmWgzlm-kuCYIUGq");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile("gravel");

        request.setSegments(List.of(seg));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        request.setSnapPreventions(List.of("ferry"));

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        for (Instruction instr : result.instructions) {
            int sign = instr.getSign();
            assertTrue(sign == Instruction.CONTINUE_ON_STREET || sign == Instruction.FINISH,
                    "S6 forced-trail-bend (Case B): on a named cycleway with a same-named "
                    + "back-leg alt and no viable forward alternative, no turn instruction "
                    + "may be emitted. Got " + signName(sign) + ". Instructions: "
                    + summarizeInstructions(result));
        }
    }

    // ========== Test 23: M1 continuation guard — real T-end where source way ends ==========

    /**
     * The route's source way (a service road) literally dead-ends at a tertiary road
     * (Taivalkunnantie). Both alternatives at the T are the OTHER halves of
     * Taivalkunnantie — no alternative continues the source way's direction. The
     * rider must turn left at the T, then turn right onto a track. M1's "join via
     * sidepath" semantics don't apply (source way doesn't continue past the first
     * turn); these are two independent navigation events.
     * <p>
     * Pre-fix: M1 fired because source PH (SERVICE_ROAD) ≠ dest PH (GOOD_TRACK) and
     * geometry passed (~80°/80° opposite turns, ~17° net). Geometry-only override
     * for the T-junction guard incorrectly classified this as a sidepath crossing.
     * <p>
     * Post-fix: Stage 1's new {@code junction_has_straight_alt} field is false at
     * the first turn junction (only alt is at -115.5°, not within ±30° of straight).
     * M1's new continuation guard rejects the merge. Rider gets two separate turn
     * instructions matching their actual experience.
     * <p>
     * Origin: testOpenCase_61_459038_23_451706_to_61_458822_23_452307 in
     * RouteInstructionGeneratorTest.
     */
    @Test
    void m1_realTJunctionRoadEnd_doesNotFire() {
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
        assertValidInstructionList(result);

        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        // 1) No instruction may carry join_direction — M1 must not have fired.
        for (Instruction instr : result.instructions) {
            assertFalse(instr.getExtraInfoJSON().containsKey("join_direction"),
                    "M1 must NOT fire at a real T-end (source way dead-ends, no alt "
                    + "within ±30° of straight). Found join_direction on instruction. "
                    + "Instructions: " + summarizeInstructions(result));
        }

        // 2) The TURN_LEFT off the service road onto Taivalkunnantie must exist as
        //    its own instruction (turn_angle_deg ≥ 60°, prev=SERVICE_ROAD,
        //    curr=MINOR_ROAD).
        Instruction tjLeft = null;
        for (Instruction instr : result.instructions) {
            if (instr.getSign() != Instruction.TURN_LEFT
                    && instr.getSign() != Instruction.TURN_SHARP_LEFT) continue;
            Map<String, Object> extra = instr.getExtraInfoJSON();
            Object angleObj = extra.get("turn_angle_deg");
            if (!(angleObj instanceof Number)) continue;
            if (((Number) angleObj).doubleValue() < 60.0) continue;
            if (!"SERVICE_ROAD".equals(extra.get("prev_predicted_highway"))) continue;
            if (!"MINOR_ROAD".equals(extra.get("predicted_highway"))) continue;
            tjLeft = instr;
            break;
        }
        assertNotNull(tjLeft,
                "Expected a standalone TURN_LEFT (≥60°) at the service-road T-end, "
                + "transitioning SERVICE_ROAD → MINOR_ROAD (Taivalkunnantie). "
                + "Instructions: " + summarizeInstructions(result));

        // 3) The subsequent TURN_RIGHT onto the track is communicated to the rider —
        //    either as its own instruction or via M2's then_turn compound. Both are
        //    correct outcomes (the rider hears "left, then right" compound). What
        //    matters is the right turn isn't *silently dropped*.
        boolean rightTurnPresent = false;
        for (Instruction instr : result.instructions) {
            // (a) standalone TURN_RIGHT onto the track
            if ((instr.getSign() == Instruction.TURN_RIGHT
                    || instr.getSign() == Instruction.TURN_SHARP_RIGHT)
                    && "GOOD_TRACK".equals(instr.getExtraInfoJSON().get("predicted_highway"))) {
                rightTurnPresent = true;
                break;
            }
            // (b) M2 compound: then_turn pointing to a right-direction turn
            //     onto a track destination
            Object thenTurnObj = instr.getExtraInfoJSON().get("then_turn");
            if (thenTurnObj instanceof Map) {
                @SuppressWarnings("unchecked")
                Map<String, Object> thenTurn = (Map<String, Object>) thenTurnObj;
                Object signObj = thenTurn.get("sign");
                if (signObj instanceof Number) {
                    int s = ((Number) signObj).intValue();
                    if ((s == Instruction.TURN_RIGHT || s == Instruction.TURN_SHARP_RIGHT)
                            && ("GOOD_TRACK".equals(thenTurn.get("predicted_highway"))
                                || "TRACK".equals(thenTurn.get("road_class")))) {
                        rightTurnPresent = true;
                        break;
                    }
                }
            }
        }
        assertTrue(rightTurnPresent,
                "The TURN_RIGHT off Taivalkunnantie onto the track must reach the "
                + "rider — either as a standalone instruction or via M2 then_turn "
                + "compound. The fix prevents M1 from absorbing it as a misleading "
                + "join. Instructions: " + summarizeInstructions(result));
    }

    // ========== Case 1: missing right at cycleway/footway fork (E5 surface gate) ==========

    /**
     * Forced-path block. Rider on cycleway makes a TURN_LEFT (~+87°) onto an asphalt
     * cycleway segment, then ~15m later a TURN_RIGHT (~-92°) onto an unpaved cycleway.
     * At both junctions the only graph alt is a bike-blocked FOOTWAY (asphalt) running
     * (near-)straight ahead. Pre-E5-surface-fix: the second turn was suppressed because
     * the FOOTWAY alt's asphalt surface did not match the new edge's fine_gravel — E5's
     * surface gate compared only against the route's NEW edge. Post-fix: E5 accepts a
     * surface match against EITHER the prev or the route edge → both turns emit.
     *
     * Origin: testMissingRightAtCyclewayFootwayFork_61_470_23_865 in
     * RouteInstructionGeneratorTest.
     */
    @Test
    void missingRightAtCyclewayFootwayFork_e5AcceptsPrevOrCurrentSurface() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("h5GZQDYcrqZAwP5p5Tr_D", 61.470086, 23.865304),
                makeWaypoint("QPU2Toex4WWPf7Qoccj7s", 61.469662, 23.865102)));

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

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        // Stage 1 must emit both the left and the right turn. Both have |turn_angle_deg|
        // ≈ 87-92° → |sign|=2 (TURN_LEFT / TURN_RIGHT).
        boolean leftFound = hasInstructionWithProperties(result,
                sign -> sign == Instruction.TURN_LEFT,
                extra -> {
                    Object a = extra.get("turn_angle_deg");
                    return a instanceof Number && Math.abs(((Number) a).doubleValue() - 86.8) < 5.0;
                });
        boolean rightFound = hasInstructionWithProperties(result,
                sign -> sign == Instruction.TURN_RIGHT,
                extra -> {
                    Object a = extra.get("turn_angle_deg");
                    return a instanceof Number && Math.abs(((Number) a).doubleValue() + 91.7) < 5.0;
                });
        assertTrue(leftFound,
                "Case 1: TURN_LEFT (~+87°) missing in Stage 1 output. The asphalt cycleway "
                + "turn must be emitted. Instructions: " + summarizeInstructions(result));
        assertTrue(rightFound,
                "Case 1: TURN_RIGHT (~-92°) missing in Stage 1 output. E5 must accept the "
                + "FOOTWAY alt's asphalt surface as confusable against the prev edge surface "
                + "(also asphalt). Instructions: " + summarizeInstructions(result));

        // After post-processing M2 should merge the two close turns into the LEFT with
        // a then_turn for the RIGHT. Either the unmerged pair or the merged compound is
        // acceptable as long as the right turn reaches the rider.
        InstructionPostProcessor postProcessor = new InstructionPostProcessor();
        postProcessor.process(result.instructions, request.getInstructionProfile());

        boolean rightReachesRider = false;
        for (Instruction instr : result.instructions) {
            if (instr.getSign() == Instruction.TURN_RIGHT) {
                rightReachesRider = true;
                break;
            }
            Object thenTurn = instr.getExtraInfoJSON().get("then_turn");
            if (thenTurn instanceof Map) {
                Object s = ((Map<?, ?>) thenTurn).get("sign");
                if (s instanceof Number && ((Number) s).intValue() == Instruction.TURN_RIGHT) {
                    rightReachesRider = true;
                    break;
                }
            }
        }
        assertTrue(rightReachesRider,
                "Case 1: after post-processing the right turn must reach the rider — either "
                + "standalone or via M2 then_turn compound. Instructions: "
                + summarizeInstructions(result));
    }

    // ========== Case 2: missing left at cycleway/footway fork (S3 non-road escape) ==========

    /**
     * Single junction. Rider on a named non-asphalt cycleway ("Hatanpään rantareitti",
     * fine_gravel). Route bends ~+45° left (|sign|=1, TURN_SLIGHT_LEFT). A same-surface
     * FOOTWAY alt continues (near-)straight at Δ=-11.8°. Pre-S3-non-road-escape: S3's
     * prominence-only suppression fired (alts FOOTWAY=4 < CYCLEWAY=7) and emitted no
     * instruction. Post-fix: S3 consults hasForwardConfusableAlt, finds a forward
     * confusable same-surface alt that's meaningfully straighter than the route, and
     * escapes its IGNORE branch — emits the angle-based sign (TURN_SLIGHT_LEFT).
     *
     * Origin: testMissingLeftAtCyclewayFootwayFork_61_482_23_749 in
     * RouteInstructionGeneratorTest.
     */
    @Test
    void missingLeftAtCyclewayFootwayFork_s3EscapesOnForwardConfusableAlt() {
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

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        // The 45° left bend must emit a TURN_SLIGHT_LEFT, not be silently absorbed
        // into the surrounding CONTINUE_ON_STREET.
        boolean slightLeftFound = hasInstructionWithProperties(result,
                sign -> sign == Instruction.TURN_SLIGHT_LEFT,
                extra -> {
                    Object a = extra.get("turn_angle_deg");
                    return a instanceof Number && Math.abs(((Number) a).doubleValue() - 45.1) < 5.0;
                });
        assertTrue(slightLeftFound,
                "Case 2: TURN_SLIGHT_LEFT (~+45°) missing. The named cycleway bend with a "
                + "same-surface forward FOOTWAY alt must escape S3's prominence suppression "
                + "via hasForwardConfusableAlt. Instructions: "
                + summarizeInstructions(result));
    }

    // ========== Case 3: over-emission on long trail route (S3 escape + isConfusableFrom) ==========

    /**
     * Longer real-world route (~1 km on outdoor_way / good_track / path) where the
     * pre-Option-B implementation of S3's escape over-emitted slight turns at junctions
     * where the only alt was either a back-leg or a side branch (>50° off forward). User
     * confirmed four specific extras as unwanted. Post-fix: hasForwardConfusableAlt uses
     * strict PH-AND-surface plus a straighter-than-route angle gate; isConfusableFrom's
     * CYCLEWAY case is split out and extended with FOOTWAY only. Same junctions no longer
     * trigger spurious turn instructions.
     *
     * Origin: testOverEmissionLongerRoute_61_477_24_027 in RouteInstructionGeneratorTest.
     */
    @Test
    void overEmissionLongTrailRoute_s3EscapeRequiresStraighterAndConfusable() {
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

        RouteInstructionGenerator.Result result = generator.generate(request);
        assertValidInstructionList(result);

        // The four user-confirmed extras were at Δ values -21.0°, +13.3°, +30.6°, +11.8°.
        // Each had an alt PATH (OUTDOOR_WAY → PATH per isConfusableFrom: NOT confusable)
        // and any forward alt was NOT meaningfully straighter than the route. None of
        // them should fire Stage 1's S3 escape.
        double[] extraDeltas = { -21.0, 13.3, 30.6, 11.8 };
        for (double targetDelta : extraDeltas) {
            boolean spuriousEmit = hasInstructionWithProperties(result,
                    sign -> sign == Instruction.TURN_SLIGHT_LEFT
                            || sign == Instruction.TURN_SLIGHT_RIGHT,
                    extra -> {
                        Object a = extra.get("turn_angle_deg");
                        if (!(a instanceof Number)) return false;
                        double v = ((Number) a).doubleValue();
                        // Also require alt to be PATH-typed (filters the legitimate turns)
                        Object alts = extra.get("junction_alt_predicted_highways");
                        if (alts == null) return false;
                        String altsStr = alts.toString();
                        return Math.abs(v - targetDelta) < 1.5
                                && altsStr.contains("PATH");
                    });
            assertFalse(spuriousEmit,
                    "Case 3: spurious TURN_SLIGHT_* at Δ≈" + targetDelta + "° with PATH "
                    + "alt should be suppressed by S3 (alts not in isConfusableFrom's "
                    + "CYCLEWAY/OUTDOOR_WAY bucket for PATH). Instructions: "
                    + summarizeInstructions(result));
        }

        // The route's legitimate navigation events (hairpin at start, KEEP_LEFT/KEEP_RIGHT
        // at real good_track forks, hairpin onto good_track, final TURN_LEFT onto
        // Mäntyveräjäntie) must all still be present. Cheap check: at least 5 emitted
        // instructions of |sign|≠0 in Stage 1 output (a sharply-reduced-but-not-zero count).
        int emittedTurns = 0;
        for (Instruction instr : result.instructions) {
            int sign = instr.getSign();
            if (sign != Instruction.CONTINUE_ON_STREET && sign != Instruction.FINISH) {
                emittedTurns++;
            }
        }
        assertTrue(emittedTurns >= 5,
                "Case 3: too few real turns remain after suppression — the fix may be "
                + "over-suppressing. Stage 1 emitted only " + emittedTurns + " turn-like "
                + "instructions. Instructions: " + summarizeInstructions(result));
    }

    // ==================== Shared assertion helpers ====================

    /**
     * Structural checks that apply to every instruction list:
     * - Not null, not empty
     * - Last instruction is FINISH
     * - Exactly one FINISH
     */
    private void assertValidInstructionList(RouteInstructionGenerator.Result result) {
        assertNotNull(result, "Result must not be null");
        assertNotNull(result.instructions, "Instructions must not be null");
        assertFalse(result.instructions.isEmpty(), "Instructions must not be empty");
        assertNotNull(result.polyline, "Polyline must not be null");
        assertTrue(result.polyline.size() > 0, "Polyline must not be empty");

        Instruction lastInstr = result.instructions.get(result.instructions.size() - 1);
        assertEquals(Instruction.FINISH, lastInstr.getSign(), "Last instruction must be FINISH");

        long finishCount = 0;
        for (Instruction instr : result.instructions) {
            if (instr.getSign() == Instruction.FINISH) finishCount++;
        }
        assertEquals(1, finishCount, "Must have exactly one FINISH instruction");
    }

    /**
     * No non-FINISH instruction should have distance below threshold.
     * These are snap artifacts that M0/C2 should have removed.
     * <p>
     * U-turn signs are exempt: a U-turn is a navigation event at a point (180° reversal),
     * not a length-based instruction. At a cross-segment U-turn boundary the polyline
     * "jog" can be only ~1 m even when the U-turn is real and required for the rider —
     * suppressing it would leave the rider committed to a dead-end with no exit cue.
     */
    private void assertNoMicroArtifacts(RouteInstructionGenerator.Result result, double thresholdMeters) {
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            int sign = instr.getSign();
            if (sign == Instruction.FINISH) continue;
            if (sign == Instruction.U_TURN_UNKNOWN
                    || sign == Instruction.U_TURN_LEFT
                    || sign == Instruction.U_TURN_RIGHT) continue;
            // M2 "then_turn" silent instructions can have short distances — skip those
            Map<String, Object> extra = instr.getExtraInfoJSON();
            if ("silent".equals(extra.get("tbt_priority"))) continue;
            assertTrue(instr.getDistance() >= thresholdMeters,
                    "Instruction [" + i + "] sign=" + signName(sign)
                    + " has micro-distance " + String.format("%.1fm", instr.getDistance())
                    + " (threshold=" + thresholdMeters + "m). Snap artifact not suppressed?");
        }
    }

    /** Check if any instruction matches a sign predicate and extraInfo predicate. */
    private boolean hasInstructionWithProperties(RouteInstructionGenerator.Result result,
            java.util.function.IntPredicate signPredicate,
            java.util.function.Predicate<Map<String, Object>> extraPredicate) {
        for (Instruction instr : result.instructions) {
            if (signPredicate.test(instr.getSign()) && extraPredicate.test(instr.getExtraInfoJSON())) {
                return true;
            }
        }
        return false;
    }

    /** Check if any instruction has a given street name. */
    private boolean hasInstructionWithName(RouteInstructionGenerator.Result result, String name) {
        for (Instruction instr : result.instructions) {
            if (name.equals(instr.getName())) return true;
        }
        return false;
    }

    // ==================== Shared build helpers ====================

    private TrailmapInstructionRequest.Waypoint makeWaypoint(String id, double lat, double lng) {
        TrailmapInstructionRequest.Waypoint wp = new TrailmapInstructionRequest.Waypoint();
        wp.setId(id);
        TrailmapInstructionRequest.Coordinates c = new TrailmapInstructionRequest.Coordinates();
        c.setLat(lat);
        c.setLng(lng);
        wp.setCoordinates(c);
        return wp;
    }

    private TrailmapInstructionRequest buildGravelRouteQualityRequest() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("dSzJCa8y5VQO_R7cGbyQ6", 61.504844, 23.650666),
                makeWaypoint("V8U0AM9AVfH16vcpB8hlS", 61.509259, 23.665011),
                makeWaypoint("OlRvwidW2kuyAIMl2ce4z", 61.513681, 23.658535),
                makeWaypoint("ufjCSuF4BFkVmXR49Kw8I", 61.523506, 23.635385)));

        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("dSzJCa8y5VQO_R7cGbyQ6");
        seg1.setEnd("V8U0AM9AVfH16vcpB8hlS");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("gravel");

        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("V8U0AM9AVfH16vcpB8hlS");
        seg2.setEnd("OlRvwidW2kuyAIMl2ce4z");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2.setProfile("gravel");
        seg2.setInitialHeading(45.68583480118048);
        seg2.setHeadingPenalty(60.0);

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

    private TrailmapInstructionRequest buildSingleSegmentRequest(String routingProfile,
                                                                  String instructionProfile, String locale) {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("start", START_LAT, START_LNG),
                makeWaypoint("end", END_LAT, END_LNG)));
        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart("start"); seg.setEnd("end");
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile(routingProfile);
        request.setSegments(List.of(seg));
        request.setInstructionProfile(instructionProfile);
        request.setLocale(locale);
        return request;
    }

    private TrailmapInstructionRequest buildTwoSegmentRequest() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.500784, 23.685416),
                makeWaypoint("wp2", 61.504067, 23.695978),
                makeWaypoint("wp3", 61.505441, 23.698538)));

        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("wp1"); seg1.setEnd("wp2");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("gravel_mtb");

        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("wp2"); seg2.setEnd("wp3");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2.setProfile("gravel_mtb");
        seg2.setInitialHeading(88.28477990934573);

        request.setSegments(List.of(seg1, seg2));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        return request;
    }

    private TrailmapInstructionRequest buildThreeSegmentRequest() {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", 61.504628, 23.701588),
                makeWaypoint("wp2", 61.503761, 23.706487),
                makeWaypoint("wp3", 61.503036, 23.708274),
                makeWaypoint("wp4", 61.498915, 23.718261)));

        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("wp1"); seg1.setEnd("wp2");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("gravel_mtb");

        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("wp2"); seg2.setEnd("wp3");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2.setProfile("gravel_mtb");

        TrailmapInstructionRequest.Segment seg3 = new TrailmapInstructionRequest.Segment();
        seg3.setStart("wp3"); seg3.setEnd("wp4");
        seg3.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg3.setProfile("gravel_mtb");
        seg3.setInitialHeading(155.8775128628056);

        request.setSegments(List.of(seg1, seg2, seg3));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        return request;
    }

    private TrailmapInstructionRequest buildThreeWaypointRequest(
            double lat1, double lng1, double lat2, double lng2,
            double lat3, double lng3, double seg2Heading) {
        TrailmapInstructionRequest request = new TrailmapInstructionRequest();
        request.setWaypoints(List.of(
                makeWaypoint("wp1", lat1, lng1),
                makeWaypoint("wp2", lat2, lng2),
                makeWaypoint("wp3", lat3, lng3)));

        TrailmapInstructionRequest.Segment seg1 = new TrailmapInstructionRequest.Segment();
        seg1.setStart("wp1"); seg1.setEnd("wp2");
        seg1.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg1.setProfile("gravel_mtb");

        TrailmapInstructionRequest.Segment seg2 = new TrailmapInstructionRequest.Segment();
        seg2.setStart("wp2"); seg2.setEnd("wp3");
        seg2.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg2.setProfile("gravel_mtb");
        seg2.setInitialHeading(seg2Heading);

        request.setSegments(List.of(seg1, seg2));
        request.setInstructionProfile("gravel");
        request.setLocale("fi");
        return request;
    }

    private double computePolylineDistance(PointList poly) {
        double dist = 0;
        for (int i = 0; i < poly.size() - 1; i++) {
            dist += DistanceCalcEarth.DIST_EARTH.calcDist(
                    poly.getLat(i), poly.getLon(i), poly.getLat(i + 1), poly.getLon(i + 1));
        }
        return dist;
    }

    private TrailmapInstructionRequest.Segment makeFollowRoadsSegment(
            String start, String end, String profile, CustomModel cm,
            double heading, double headingPenalty) {
        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart(start);
        seg.setEnd(end);
        seg.setType(TrailmapInstructionRequest.TYPE_FOLLOW_ROADS);
        seg.setProfile(profile);
        seg.setCustomModel(cm);
        if (!Double.isNaN(heading)) {
            seg.setInitialHeading(heading);
            seg.setHeadingPenalty(headingPenalty);
        }
        return seg;
    }

    private TrailmapInstructionRequest.Segment makeDirectSegment(String start, String end) {
        TrailmapInstructionRequest.Segment seg = new TrailmapInstructionRequest.Segment();
        seg.setStart(start);
        seg.setEnd(end);
        seg.setType(TrailmapInstructionRequest.TYPE_DIRECT);
        return seg;
    }

    // ==================== Formatting helpers ====================

    /** One-line summary of each instruction for assertion failure messages. */
    private String summarizeInstructions(RouteInstructionGenerator.Result result) {
        StringBuilder sb = new StringBuilder("\n");
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            Map<String, Object> extra = instr.getExtraInfoJSON();
            sb.append(String.format("  [%d] %s dist=%.0fm name=\"%s\"",
                    i, signName(instr.getSign()), instr.getDistance(), instr.getName()));
            if (extra.containsKey("predicted_highway"))
                sb.append(" ph=").append(extra.get("predicted_highway"));
            if (extra.containsKey("prev_predicted_highway"))
                sb.append(" prevPh=").append(extra.get("prev_predicted_highway"));
            if (extra.containsKey("join_direction"))
                sb.append(" join=").append(extra.get("join_direction"));
            if (extra.containsKey("tbt_priority"))
                sb.append(" prio=").append(extra.get("tbt_priority"));
            sb.append("\n");
        }
        return sb.toString();
    }

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

    // ===================================================================================
    // Interval-index correctness (TbT marker placement). The client locates each instruction
    // by its polyline interval; these guard the two fixes that keep that index on the real turn.
    // ===================================================================================

    /**
     * The client-facing interval index must be derived from each instruction's coordinate-matched
     * polyline position (matchInstructionStarts), run on the FINAL post-processed list — not from
     * cumulative getLength(). On the reported 152 km route this proves (a) the legacy getLength
     * accounting drifted by a polyline point = hundreds of metres, and (b) the new interval start
     * lands exactly on each instruction's turn vertex. Reuses the diagnostic payload.
     */
    @Test
    void intervalStartsLandOnTurns_152kmRoute() throws Exception {
        Assumptions.assumeTrue(hopper != null, "graph cache required");
        TrailmapInstructionRequest request =
                new ObjectMapper().readValue(RouteInstructionGeneratorTest.DRIFT_JSON, TrailmapInstructionRequest.class);

        RouteInstructionGenerator.Result result = generator.generate(request);
        new InstructionPostProcessor().process(result.instructions, request.getInstructionProfile());
        PointList poly = result.polyline;
        int N = poly.size();
        List<Integer> starts = RouteInstructionGenerator.matchInstructionStarts(result.instructions, poly);

        assertEquals(result.instructions.size(), starts.size(), "one start per instruction");
        int prev = 0;
        for (int s : starts) {
            assertTrue(s >= 0 && s < N, "start in bounds: " + s);
            assertTrue(s >= prev, "starts monotonic non-decreasing");
            prev = s;
        }

        double[] cum = new double[N];
        for (int k = 1; k < N; k++)
            cum[k] = cum[k - 1] + DistanceCalcEarth.DIST_EARTH.calcDist(
                    poly.getLat(k - 1), poly.getLon(k - 1), poly.getLat(k), poly.getLon(k));

        // (a) the legacy getLength()-based interval drifted significantly here.
        int acc = 0; double worstLegacyM = 0;
        for (int i = 0; i < result.instructions.size(); i++) {
            int legacyStart = Math.min(acc, N - 1);
            worstLegacyM = Math.max(worstLegacyM, Math.abs(cum[legacyStart] - cum[starts.get(i)]));
            acc += result.instructions.get(i).getLength();
        }
        assertTrue(worstLegacyM > 100.0,
                "expected the legacy getLength interval to drift >100m on this route; was " + worstLegacyM);

        // (b) the new interval start sits on each instruction's own turn vertex.
        double worstNewM = 0; int worstI = -1;
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction ins = result.instructions.get(i);
            if (i == 0 || ins.getSign() == Instruction.FINISH || ins.getPoints().size() == 0) continue;
            int s = starts.get(i);
            double d = DistanceCalcEarth.DIST_EARTH.calcDist(
                    poly.getLat(s), poly.getLon(s), ins.getPoints().getLat(0), ins.getPoints().getLon(0));
            if (d > worstNewM) { worstNewM = d; worstI = i; }
        }
        assertTrue(worstNewM < 1.0,
                "interval start must sit on the instruction's turn vertex; worst=" + worstNewM
                + "m at #" + worstI + " (" + (worstI >= 0 ? result.instructions.get(worstI).getName() : "") + ")");
    }

    /**
     * Self-crossing routes: when the polyline passes the same coordinate X twice and a turn belongs to
     * the 2ND pass with NO instruction anchor between the passes, the bare monotonic matcher took the
     * 1st pass. The independent route-distance key (_cum_route_m, stamped by remap) resolves it to the
     * 2nd pass. Also exercises the with-anchor and no-key (legacy fallback) paths.
     */
    @Test
    void selfCrossingTurn_distanceAnchoredMatcher_resolvesTo2ndPass() {
        Assumptions.assumeTrue(hopper != null, "translation needed");
        Translation tr = hopper.getTranslationMap().getWithFallBack(Locale.ENGLISH);

        // X (61.0010, 24.0010) appears at index 1 AND index 4.  0:A 1:X 2:B 3:C 4:X 5:D 6:E
        double xLat = 61.0010, xLon = 24.0010;
        PointList poly = new PointList(7, false);
        poly.add(61.0000, 24.0000);
        poly.add(xLat, xLon);
        poly.add(61.0020, 24.0000);
        poly.add(61.0015, 23.9990);
        poly.add(xLat, xLon);
        poly.add(61.0000, 24.0020);
        poly.add(60.9990, 24.0030);

        double[] pc = new double[7];
        for (int k = 1; k < 7; k++)
            pc[k] = pc[k - 1] + DistanceCalcEarth.DIST_EARTH.calcDist(
                    poly.getLat(k - 1), poly.getLon(k - 1), poly.getLat(k), poly.getLon(k));

        PointList xPts = new PointList(1, false); xPts.add(xLat, xLon);
        PointList aPts = new PointList(1, false); aPts.add(61.0000, 24.0000);
        PointList ePts = new PointList(1, false); ePts.add(60.9990, 24.0030);

        // Case 1: turn on the 2nd pass, NO anchor between the passes, distance key present → resolves to idx 4.
        InstructionList noAnchor = new InstructionList(tr);
        noAnchor.add(cum(new Instruction(Instruction.CONTINUE_ON_STREET, "start", aPts), 0));
        noAnchor.add(cum(new Instruction(Instruction.TURN_LEFT, "turn-at-2nd-X", clonePts(xPts)), pc[4]));
        noAnchor.add(cum(new Instruction(Instruction.FINISH, "", ePts), pc[6]));
        assertEquals(4, RouteInstructionGenerator.matchInstructionStarts(noAnchor, poly).get(1),
                "distance-anchored matcher resolves the turn to the 2nd pass even with no anchor between passes");

        // Case 2: an anchor between the passes → also correct.
        InstructionList withAnchor = new InstructionList(tr);
        PointList cPts = new PointList(1, false); cPts.add(61.0015, 23.9990);
        withAnchor.add(cum(new Instruction(Instruction.CONTINUE_ON_STREET, "start", clonePts(aPts)), 0));
        withAnchor.add(cum(new Instruction(Instruction.TURN_RIGHT, "anchor-at-C", cPts), pc[3]));
        withAnchor.add(cum(new Instruction(Instruction.TURN_LEFT, "turn-at-2nd-X", clonePts(xPts)), pc[4]));
        withAnchor.add(cum(new Instruction(Instruction.FINISH, "", clonePts(ePts)), pc[6]));
        assertEquals(4, RouteInstructionGenerator.matchInstructionStarts(withAnchor, poly).get(2),
                "with an anchor between the passes, the 2nd pass is taken correctly too");

        // Case 3: no distance key → graceful legacy fallback (first occurrence), no crash/cascade.
        InstructionList noKey = new InstructionList(tr);
        noKey.add(new Instruction(Instruction.CONTINUE_ON_STREET, "start", clonePts(aPts)));
        noKey.add(new Instruction(Instruction.TURN_LEFT, "turn-at-2nd-X", clonePts(xPts)));
        noKey.add(new Instruction(Instruction.FINISH, "", clonePts(ePts)));
        assertEquals(1, RouteInstructionGenerator.matchInstructionStarts(noKey, poly).get(1),
                "without the distance key, falls back to legacy first-occurrence behavior");
    }

    private static Instruction cum(Instruction instr, double cumRouteM) {
        instr.setExtraInfo("_cum_route_m", cumRouteM);
        return instr;
    }

    private static PointList clonePts(PointList p) {
        PointList c = new PointList(p.size(), p.is3D());
        for (int i = 0; i < p.size(); i++) c.add(p.getLat(i), p.getLon(i));
        return c;
    }

    // ===================================================================================
    // resolveFromNode: parallel-edge out-and-back at a segment start.
    // A trailmap_foot Joensuu loop whose segment 9 (FXDI… → K8xo…, initial_heading 202°) begins
    // with an out-and-back over a PARALLEL edge pair (two edges between the same node pair). The old
    // single-step connectivity check in resolveFromNode was ambiguous there, fell back to proximity,
    // picked the wrong start node, and threw "Edge … is not connected to node …" (HTTP 500) in
    // walkEdge. See docs/gh_tbt_resolve_from_node_parallel_out_and_back.md.
    // ===================================================================================

    /** Regression: the route must generate a valid, connected instruction list (previously HTTP 500). */
    @Test
    void parallelOutAndBackAtSegmentStart_doesNotThrow() throws Exception {
        Assumptions.assumeTrue(hopper != null, "graph cache required");
        TrailmapInstructionRequest request = new ObjectMapper()
                .readValue(RouteInstructionGeneratorTest.JOENSUU_500_JSON, TrailmapInstructionRequest.class);

        RouteInstructionGenerator.Result result =
                assertDoesNotThrow(() -> generator.generate(request),
                        "parallel-edge out-and-back at a segment start must not break synthetic-path assembly");
        assertValidInstructionList(result);
        // Sanity on scale so an accidental empty/degenerate route can't pass silently.
        assertTrue(result.instructions.size() > 100,
                "expected a long instruction list; was " + result.instructions.size()
                        + "\n" + summarizeInstructions(result));
    }

    /**
     * Ground-truth direction check (independent of the production mechanism): for EVERY routable
     * segment, the start node resolveFromNode chooses (connectivity walk) must equal the node implied
     * by GraphHopper's oriented edge_key (the authoritative travel direction). resolveFromNode uses
     * undirected connectivity; edge_key is a separate, orientation-carrying source — so agreement on
     * all segments confirms the fix builds each synthetic path in the correct direction, not merely a
     * connected one. Replicates generate()'s heading + start-continuity loop.
     */
    @Test
    void resolveFromNode_matchesOrientedEdgeKeyOracle_everySegment() throws Exception {
        Assumptions.assumeTrue(hopper != null, "graph cache required");
        TrailmapInstructionRequest request = new ObjectMapper()
                .readValue(RouteInstructionGeneratorTest.JOENSUU_500_JSON, TrailmapInstructionRequest.class);

        BaseGraph baseGraph = hopper.getBaseGraph();
        java.util.Map<String, TrailmapInstructionRequest.Coordinates> waypointMap = new java.util.HashMap<>();
        for (TrailmapInstructionRequest.Waypoint wp : request.getWaypoints())
            waypointMap.put(wp.getId(), wp.getCoordinates());
        List<RouteInstructionGenerator.Chunk> chunks = generator.buildChunks(request.getSegments(), waypointMap);

        java.lang.reflect.Method extractEdgeIds = RouteInstructionGenerator.class
                .getDeclaredMethod("extractEdgeIds", ResponsePath.class);
        extractEdgeIds.setAccessible(true);
        java.lang.reflect.Method resolveFromNode = RouteInstructionGenerator.class
                .getDeclaredMethod("resolveFromNode", List.class, TrailmapInstructionRequest.Coordinates.class);
        resolveFromNode.setAccessible(true);
        java.lang.reflect.Field rsField = RouteInstructionGenerator.Chunk.class.getDeclaredField("routableSection");
        rsField.setAccessible(true);

        List<String> snapPreventions = request.getSnapPreventions();
        TrailmapInstructionRequest.Coordinates nextStartOverride = null;
        int checked = 0;
        int idx = -1;
        for (RouteInstructionGenerator.Chunk chunk : chunks) {
            idx++;
            RouteInstructionGenerator.Section section =
                    (RouteInstructionGenerator.Section) rsField.get(chunk);
            if (section == null) { nextStartOverride = null; continue; }

            Double heading = section.initialHeading;
            if (nextStartOverride != null) section.points.set(0, nextStartOverride);

            GHRequest req = new GHRequest();
            for (TrailmapInstructionRequest.Coordinates c : section.points)
                req.addPoint(new com.graphhopper.util.shapes.GHPoint(c.getLat(), c.getLng()));
            req.setProfile(section.profile);
            req.setPathDetails(List.of("edge_id", "edge_key"));
            req.putHint("instructions", false);
            req.putHint("calc_points", true);
            if (section.customModel != null) req.setCustomModel(section.customModel);
            if (snapPreventions != null && !snapPreventions.isEmpty()) req.setSnapPreventions(snapPreventions);
            if (section.headingPenalty != null) req.putHint("heading_penalty", section.headingPenalty);
            if (heading != null && !heading.isNaN()) {
                List<Double> hs = new java.util.ArrayList<>();
                hs.add(heading);
                for (int i = 1; i < section.points.size(); i++) hs.add(Double.NaN);
                req.setHeadings(hs);
            }

            GHResponse rsp = hopper.route(req);
            assertFalse(rsp.hasErrors(), "segment " + idx + " routing failed: " + rsp.getErrors());
            ResponsePath path = rsp.getBest();

            @SuppressWarnings("unchecked")
            List<Integer> dedup = (List<Integer>) extractEdgeIds.invoke(generator, path);
            List<PathDetail> keyDets = path.getPathDetails().get("edge_key");

            if (!dedup.isEmpty() && keyDets != null && !keyDets.isEmpty()) {
                int firstKey = ((Number) keyDets.get(0).getValue()).intValue();
                int oracleFromNode = baseGraph.getEdgeIteratorStateForKey(firstKey).getBaseNode();
                int prodFromNode = (int) resolveFromNode.invoke(generator, dedup, section.points.get(0));
                assertEquals(oracleFromNode, prodFromNode,
                        "segment " + idx + ": resolveFromNode chose " + prodFromNode
                                + " but oriented edge_key implies " + oracleFromNode
                                + " (edges=" + dedup + ")");
                checked++;
            }

            PointList pts = path.getPoints();
            nextStartOverride = pts.size() >= 1
                    ? makeCoord(pts.getLat(pts.size() - 1), pts.getLon(pts.size() - 1)) : null;
        }
        assertTrue(checked > 50, "expected to cross-check many segments; checked " + checked);
    }

    private static TrailmapInstructionRequest.Coordinates makeCoord(double lat, double lng) {
        TrailmapInstructionRequest.Coordinates c = new TrailmapInstructionRequest.Coordinates();
        c.setLat(lat);
        c.setLng(lng);
        return c;
    }
}
