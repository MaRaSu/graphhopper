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
        new InstructionPostProcessor().process(result.instructions);
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
        postProcessor.process(result.instructions);

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
        postProcessor.process(result.instructions);

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
        postProcessor.process(result.instructions);

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
        postProcessor.process(result.instructions);

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
        postProcessor.process(result.instructions);

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
        postProcessor.process(result.instructions);

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
        postProcessor.process(result.instructions);

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
        postProcessor.process(result.instructions);

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
        postProcessor.process(result.instructions);

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

        // All trail_fork instructions should be CONTINUE_ON_STREET (not turns)
        for (Instruction instr : result.instructions) {
            Map<String, Object> extra = instr.getExtraInfoJSON();
            if (Boolean.TRUE.equals(extra.get("trail_fork"))) {
                assertEquals(Instruction.CONTINUE_ON_STREET, instr.getSign(),
                        "trail_fork instruction should be CONTINUE_ON_STREET, got "
                        + signName(instr.getSign()) + ". Instructions: " + summarizeInstructions(result));
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
     */
    private void assertNoMicroArtifacts(RouteInstructionGenerator.Result result, double thresholdMeters) {
        for (int i = 0; i < result.instructions.size(); i++) {
            Instruction instr = result.instructions.get(i);
            if (instr.getSign() == Instruction.FINISH) continue;
            // M2 "then_turn" silent instructions can have short distances — skip those
            Map<String, Object> extra = instr.getExtraInfoJSON();
            if ("silent".equals(extra.get("tbt_priority"))) continue;
            assertTrue(instr.getDistance() >= thresholdMeters,
                    "Instruction [" + i + "] sign=" + signName(instr.getSign())
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
}
