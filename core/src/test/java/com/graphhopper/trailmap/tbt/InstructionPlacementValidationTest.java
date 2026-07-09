package com.graphhopper.trailmap.tbt;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.PropertyNamingStrategies;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.graphhopper.GraphHopper;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.jackson.GraphHopperModule;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.trailmap.TrailmapGraphHopper;
import com.graphhopper.trailmap.shared.TrailmapImportRegistry;
import com.graphhopper.util.TranslationMap;
import org.junit.jupiter.api.*;

import java.io.File;
import java.io.FileInputStream;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Placement validation: every TbT instruction must be anchored at the correct route distance
 * on the response polyline, verified against the independent oriented-geometry oracle
 * ({@link InstructionPlacementOracle}). This is the standing regression for the wrong-occurrence /
 * phantom-point / seam-anchor placement bug class on out-and-back and self-crossing routes.
 * See docs/gh_tbt_instruction_placement_validation_design.md.
 *
 * The pipeline mirrors the HTTP endpoint exactly: generate() → InstructionPostProcessor.process()
 * → matchInstructionStarts() — the validated intervals are the ones the client would receive.
 *
 * Corpus: drop any known-good API payload JSON into
 * src/test/resources/com/graphhopper/trailmap/tbt/placement/good/ — it is picked up automatically.
 */
public class InstructionPlacementValidationTest {

    private static final String GRAPH_LOCATION = "../../data/graph-cache";
    private static final String OSM_FILE = "../../data/finland_3.osm.pbf";
    private static final String CONFIG_FILE = "../trailmap-config.yml";
    private static final String CORPUS_DIR = "src/test/resources/com/graphhopper/trailmap/tbt/placement/good";

    private static GraphHopper hopper;
    private static RouteInstructionGenerator generator;
    private static InstructionPostProcessor postProcessor;

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
        TranslationMap translationMap = hopper.getTranslationMap();
        generator = new RouteInstructionGenerator(hopper, baseGraph, hopper.getEncodingManager(), translationMap);
        postProcessor = new InstructionPostProcessor();
    }

    @AfterAll
    static void teardown() {
        if (hopper != null) hopper.close();
    }

    static TrailmapInstructionRequest parsePayload(String json) throws Exception {
        // Mirror the endpoint's Dropwizard mapper: snake_case (CustomModel.distanceInfluence
        // ← distance_influence) + GraphHopperModule (custom-model statement deserialization).
        ObjectMapper mapper = new ObjectMapper();
        mapper.setPropertyNamingStrategy(PropertyNamingStrategies.SNAKE_CASE);
        mapper.registerModule(new GraphHopperModule());
        return mapper.readValue(json, TrailmapInstructionRequest.class);
    }

    static TrailmapInstructionRequest loadPayloadResource(String name) throws Exception {
        try (var in = InstructionPlacementValidationTest.class.getResourceAsStream("placement/" + name)) {
            assertNotNull(in, "payload resource not found: placement/" + name);
            return parsePayload(new String(in.readAllBytes()));
        }
    }

    static InstructionPlacementOracle.Report run(TrailmapInstructionRequest request) {
        return InstructionPlacementOracle.runAndValidate(
                hopper.getBaseGraph(), generator, postProcessor, request);
    }

    private static void assertPlacementMatchesOracle(InstructionPlacementOracle.Report report, String label) {
        assertTrue(report.oracleBuildOk(),
                label + ": oracle build failed (disconnected assembly — the HTTP-500 class)\n" + report.table());
        // CRITICAL CHECK (design §4): distance-anchored occurrence match on every instruction.
        assertTrue(report.check1Failures().isEmpty(),
                label + ": " + report.check1Failures().size()
                        + " instruction(s) misplaced (|s_report − s_oracle| > "
                        + InstructionPlacementOracle.EPS_DIST_M + " m)\n" + report.table());
        // Supporting structural checks (monotonicity, tiling).
        assertTrue(report.structuralFailures.isEmpty(),
                label + ": structural placement failures\n" + report.table());
        // Sanity: the oracle actually checked a meaningful share of the instructions.
        assertTrue(report.checkedCount >= Math.max(1, report.rows.size() / 2),
                label + ": oracle checked only " + report.checkedCount + "/" + report.rows.size()
                        + " instructions — harness coverage regression\n" + report.table());
    }

    // ==================== known-good baseline ====================

    /**
     * Joensuu 500 loop (200+ instructions, parallel-edge out-and-back at a segment start):
     * the baseline from the resolveFromNode fix. Placement checking (new coverage, 2026-07-08)
     * found the same corrupted-_cum_route_m wrong-occurrence here (+191 m = 2 x 95.7 m spur)
     * plus seam anchor slips (+11..43 m); all fixed 2026-07-09. Oracle build must succeed and
     * every instruction must sit at its oracle distance.
     */
    @Test
    void joensuu500_allInstructionPlacementsMatchOracle() throws Exception {
        Assumptions.assumeTrue(hopper != null, "graph cache required");
        TrailmapInstructionRequest request = parsePayload(RouteInstructionGeneratorTest.JOENSUU_500_JSON);
        InstructionPlacementOracle.Report report = run(request);
        assertTrue(report.rows.size() > 100, "expected a long instruction list\n" + report.table());
        assertPlacementMatchesOracle(report, "joensuu500");
    }

    // ==================== live fail case ====================

    /**
     * Saariselkä 30-waypoint mtb loop (contains a direct gap and a via_points segment):
     * reported 2026-07-08 — the KEEP_RIGHT at 20.4 km was pinned to the wrong pass of a
     * ~2.2 km out-and-back (interval +4462 m = 2 x spur), cascading the following left/u-turn
     * ~2-4 km off their junctions. Root cause: _cum_route_m inflated by the strip-merge
     * double-counting stitched shared boundary edges (14.2 km over 26 boundaries on this
     * route). Fixed 2026-07-09; this is the standing regression.
     */
    @Test
    void saariselka_misplacedLeftAndUturn_placementMatchesOracle() throws Exception {
        Assumptions.assumeTrue(hopper != null, "graph cache required");
        TrailmapInstructionRequest request = loadPayloadResource("saariselka_misplaced_left_uturn.json");
        InstructionPlacementOracle.Report report = run(request);
        assertPlacementMatchesOracle(report, "saariselka");
    }

    /**
     * Saariselkä 2-segment 1.1 km route (reported 2026-07-09): a MATCH_SANITY_M false positive.
     * The route start snaps ~862 m into a very long edge, so instruction 0's pre-remap synthetic
     * distance (and hence the _cum_route_m delta of the FIRST leg) is inflated by the pre-snap
     * span. The sanity band then rejects instr 1's unique, CORRECT coordinate match as a
     * suspected wrong occurrence and recovery-places it ~873 m off; instr 2 cascades (anchor not
     * advanced). Regression introduced with the 2026-07-09 matcher sanity band; the leg-after-a-
     * seam expectation must be exempted or the key corrected for pre-snap offset.
     */
    @Test
    void saariselka2seg_presnapFirstLeg_sanityBandMustNotRejectCorrectMatch() throws Exception {
        Assumptions.assumeTrue(hopper != null, "graph cache required");
        TrailmapInstructionRequest request = loadPayloadResource("saariselka_2seg_presnap_sanity_band.json");
        InstructionPlacementOracle.Report report = run(request);
        assertPlacementMatchesOracle(report, "saariselka2seg");
    }

    // ==================== drop-in corpus ====================

    /** Every payload in placement/good/ must place all instructions at oracle distance. */
    @Test
    void corpus_allGoodPayloadsPlaceInstructionsAtOracleDistance() throws Exception {
        Assumptions.assumeTrue(hopper != null, "graph cache required");
        File dir = new File(CORPUS_DIR);
        File[] files = dir.isDirectory()
                ? dir.listFiles((d, n) -> n.endsWith(".json")) : null;
        Assumptions.assumeTrue(files != null && files.length > 0,
                "no corpus payloads in " + CORPUS_DIR + " — skipping");

        List<String> failures = new ArrayList<>();
        for (File f : files) {
            TrailmapInstructionRequest request = parsePayload(Files.readString(f.toPath()));
            InstructionPlacementOracle.Report report = run(request);
            if (!report.allPass()) {
                failures.add("=== " + f.getName() + " ===\n" + report.table());
            }
        }
        assertTrue(failures.isEmpty(), String.join("\n", failures));
    }
}
