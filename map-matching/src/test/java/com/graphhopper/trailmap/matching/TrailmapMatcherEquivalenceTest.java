package com.graphhopper.trailmap.matching;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.PropertyNamingStrategies;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.graphhopper.GraphHopper;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.jackson.GraphHopperModule;
import com.graphhopper.matching.EdgeMatch;
import com.graphhopper.matching.MapMatching;
import com.graphhopper.matching.MatchResult;
import com.graphhopper.matching.Observation;
import com.graphhopper.matching.Tracepoint;
import com.graphhopper.trailmap.TrailmapGraphHopper;
import com.graphhopper.trailmap.shared.TrailmapImportRegistry;
import com.graphhopper.util.PMap;
import com.graphhopper.util.shapes.GHPoint;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import java.io.File;
import java.io.FileInputStream;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Phase 0 verification for the Trailmap matcher fork.
 *
 * <p>Proves that {@link TrailmapMapMatching} with a default-constructed {@link MatcherConfig}
 * is <b>byte-for-byte equivalent</b> to stock {@link MapMatching}: same edge sequence, same
 * per-observation tracepoints (snap distance, edge id, filtered/matched flags,
 * distanceFromPrevious). If this holds, the fork is a faithful baseline and any later
 * deviation is attributable to a config lever, not a copy error.
 *
 * <p>Also a convenience harness: the {@code print...} test methods (disabled by default)
 * let you eyeball Phase 1 / Phase 6 effects against the canonical baseline.
 *
 * <p>Skipped unless the graph cache exists at {@code ../../data/graph-cache}.
 *
 * <pre>
 *   mvn -f graphhopper/pom.xml -pl map-matching test \
 *       -Dtest=TrailmapMatcherEquivalenceTest -Dsurefire.useFile=false
 * </pre>
 */
public class TrailmapMatcherEquivalenceTest {

    private static final String GRAPH_LOCATION = "../../data/graph-cache";
    private static final String OSM_FILE = "../../data/finland_4.osm.pbf";
    private static final String CONFIG_FILE = "../trailmap-config.yml";
    private static final String GPX_DIR = "../../data/gpx-for-testing/";

    // Fixtures spanning the known behavioural cases (see implementation_v2 §5).
    private static final String[] FIXTURES = {
            "8wpt.gpx", "junarata.gpx", "sorel.gpx", "suora-reitti-hankala-2.gpx",
            "l_16km.gpx", "l_16km_start.gpx", "out-n-back.gpx", "mtbhelppo_split.gpx",
            "sbc_23.gpx"
    };

    private static final double[] SIGMAS = {10.0, 20.0};

    private static final String PROFILE = "gravel";

    private static GraphHopper hopper;

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
    }

    @AfterAll
    static void teardown() {
        if (hopper != null) hopper.close();
    }

    @Test
    void forkIsByteIdenticalToStockAtDefaultConfig() throws Exception {
        int compared = 0;
        for (String fixture : FIXTURES) {
            File gpx = new File(GPX_DIR + fixture);
            if (!gpx.exists()) continue;
            List<Observation> obs = parseGpx(gpx);

            for (double sigma : SIGMAS) {
                MatchResult stock = stockMatch(obs, sigma);
                MatchResult fork = forkMatch(obs, sigma, new MatcherConfig());

                assertEdgeMatchesEqual(stock, fork, fixture + " @sigma=" + sigma);
                assertTracepointsEqual(stock, fork, fixture + " @sigma=" + sigma);
                compared++;
            }
        }
        assertTrue(compared > 0, "No fixtures were compared — check GPX_DIR");
        System.out.println("Phase 0 equivalence verified across " + compared + " (fixture, sigma) cases.");
    }

    /**
     * Diagnostic: prints whether each phase actually changes the matched output on the
     * canonical M-case (sbc_23 @ gravel, sigma=10). Not an assertion — evidence for the
     * "did anything change?" question.
     */
    @Test
    void printPhaseEffectsOnSbc23() throws Exception {
        File gpx = new File(GPX_DIR + "sbc_23.gpx");
        Assumptions.assumeTrue(gpx.exists(), "sbc_23.gpx not found");
        List<Observation> obs = parseGpx(gpx);
        double sigma = 10.0;

        PMap hints = new PMap();
        hints.putObject("profile", PROFILE);

        // Canonical fork (Phase 0).
        MatcherConfig canon = new MatcherConfig();
        canon.measurementErrorSigma = sigma;
        TrailmapMapMatching mc = TrailmapMapMatching.fromGraphHopper(hopper, hints, canon);
        MatchResult rc = mc.match(new ArrayList<>(obs));
        System.out.println("=== sbc_23 gravel sigma=10 ===");
        System.out.println("[Phase0 canonical] " + summarize(rc));

        // Phase 1 (M1): radius = 3 x sigma.
        MatcherConfig p1 = new MatcherConfig();
        p1.measurementErrorSigma = sigma;
        p1.candidateRadiusSigmaMult = 3.0;
        TrailmapMapMatching m1 = TrailmapMapMatching.fromGraphHopper(hopper, hints, p1);
        MatchResult r1 = m1.match(new ArrayList<>(obs));
        System.out.println("[Phase1 radius=3sigma] " + summarize(r1)
                + "  edgesDifferFromCanonical=" + edgeSeqDiffers(rc, r1));

        // The real failure case: sigma=20 (hand-drawn "default" preset). Canonical picks the
        // shortcut path; does M1 (wider radius) alone change it?
        double sigma20 = 20.0;
        MatcherConfig canon20 = new MatcherConfig();
        canon20.measurementErrorSigma = sigma20;
        MatchResult rc20 = TrailmapMapMatching.fromGraphHopper(hopper, hints, canon20).match(new ArrayList<>(obs));
        System.out.println("[Phase0 canonical sigma=20] " + summarize(rc20));

        MatcherConfig p1s20 = new MatcherConfig();
        p1s20.measurementErrorSigma = sigma20;
        p1s20.candidateRadiusSigmaMult = 3.0;
        MatchResult r1s20 = TrailmapMapMatching.fromGraphHopper(hopper, hints, p1s20).match(new ArrayList<>(obs));
        System.out.println("[Phase1 radius=3sigma sigma=20] " + summarize(r1s20)
                + "  edgesDifferFromCanonical20=" + edgeSeqDiffers(rc20, r1s20));

        // Phase 6 (P3): auto sigma.
        MatcherConfig p3 = new MatcherConfig();
        p3.measurementErrorSigma = sigma;
        p3.autoSigma = true;
        TrailmapMapMatching m3 = TrailmapMapMatching.fromGraphHopper(hopper, hints, p3);
        MatchResult r3 = m3.match(new ArrayList<>(obs));
        System.out.println("[Phase6 autoSigma] " + summarize(r3)
                + "  estimatedSigma=" + m3.getStatistics().get("autoSigmaEstimatedM")
                + "  seed=" + m3.getStatistics().get("autoSigmaSeedM")
                + "  edgesDifferFromCanonical=" + edgeSeqDiffers(rc, r3));
    }

    private static String summarize(MatchResult r) {
        StringBuilder sb = new StringBuilder();
        sb.append("edges=").append(r.getEdgeMatches().size());
        sb.append(" matchLen=").append(String.format("%.1f", r.getMatchLength()));
        sb.append(" snaps=[");
        List<Tracepoint> tps = r.getTracepoints();
        for (int i = 0; i < tps.size(); i++) {
            Double d = tps.get(i).getDistance();
            sb.append(i).append(':').append(d == null ? "-" : String.format("%.1f", d));
            if (i < tps.size() - 1) sb.append(' ');
        }
        sb.append(']');
        return sb.toString();
    }

    private static boolean edgeSeqDiffers(MatchResult a, MatchResult b) {
        List<EdgeMatch> ea = a.getEdgeMatches(), eb = b.getEdgeMatches();
        if (ea.size() != eb.size()) return true;
        for (int i = 0; i < ea.size(); i++) {
            if (ea.get(i).getEdgeState().getEdge() != eb.get(i).getEdgeState().getEdge()) return true;
        }
        return false;
    }

    // -- matcher builders --

    private MatchResult stockMatch(List<Observation> obs, double sigma) {
        PMap hints = new PMap();
        hints.putObject("profile", PROFILE);
        MapMatching m = MapMatching.fromGraphHopper(hopper, hints);
        m.setMeasurementErrorSigma(sigma);
        return m.match(new ArrayList<>(obs));
    }

    private MatchResult forkMatch(List<Observation> obs, double sigma, MatcherConfig cfg) {
        cfg.measurementErrorSigma = sigma;
        PMap hints = new PMap();
        hints.putObject("profile", PROFILE);
        TrailmapMapMatching m = TrailmapMapMatching.fromGraphHopper(hopper, hints, cfg);
        return m.match(new ArrayList<>(obs));
    }

    // -- comparisons --

    private void assertEdgeMatchesEqual(MatchResult a, MatchResult b, String ctx) {
        List<EdgeMatch> ea = a.getEdgeMatches();
        List<EdgeMatch> eb = b.getEdgeMatches();
        assertEquals(ea.size(), eb.size(), ctx + ": edge-match count differs");
        for (int i = 0; i < ea.size(); i++) {
            assertEquals(ea.get(i).getEdgeState().getEdge(), eb.get(i).getEdgeState().getEdge(),
                    ctx + ": edge id differs at " + i);
            assertEquals(ea.get(i).getEdgeState().getBaseNode(), eb.get(i).getEdgeState().getBaseNode(),
                    ctx + ": baseNode differs at " + i);
            assertEquals(ea.get(i).getEdgeState().getAdjNode(), eb.get(i).getEdgeState().getAdjNode(),
                    ctx + ": adjNode differs at " + i);
        }
    }

    private void assertTracepointsEqual(MatchResult a, MatchResult b, String ctx) {
        List<Tracepoint> ta = a.getTracepoints();
        List<Tracepoint> tb = b.getTracepoints();
        assertNotNull(ta, ctx + ": stock tracepoints null");
        assertNotNull(tb, ctx + ": fork tracepoints null");
        assertEquals(ta.size(), tb.size(), ctx + ": tracepoint count differs");
        for (int i = 0; i < ta.size(); i++) {
            Tracepoint x = ta.get(i), y = tb.get(i);
            assertEquals(x.isMatched(), y.isMatched(), ctx + ": matched differs at " + i);
            assertEquals(x.isFiltered(), y.isFiltered(), ctx + ": filtered differs at " + i);
            assertEquals(x.getEdgeId(), y.getEdgeId(), ctx + ": edgeId differs at " + i);
            assertDoubleEq(x.getDistance(), y.getDistance(), ctx + ": snap distance differs at " + i);
            assertDoubleEq(x.getDistanceFromPrevious(), y.getDistanceFromPrevious(),
                    ctx + ": distanceFromPrevious differs at " + i);
        }
    }

    private static void assertDoubleEq(Double x, Double y, String msg) {
        if (x == null || y == null) {
            assertEquals(x, y, msg);
        } else {
            assertEquals(x, y, 1e-9, msg);
        }
    }

    static List<Observation> parseGpx(File gpxFile) throws Exception {
        DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
        dbf.setNamespaceAware(false);
        DocumentBuilder db = dbf.newDocumentBuilder();
        Document doc = db.parse(gpxFile);
        NodeList trkpts = doc.getElementsByTagName("trkpt");
        List<Observation> out = new ArrayList<>(trkpts.getLength());
        for (int i = 0; i < trkpts.getLength(); i++) {
            Node n = trkpts.item(i);
            Node latAttr = n.getAttributes().getNamedItem("lat");
            Node lonAttr = n.getAttributes().getNamedItem("lon");
            if (latAttr == null || lonAttr == null) continue;
            double lat = Double.parseDouble(latAttr.getNodeValue());
            double lon = Double.parseDouble(lonAttr.getNodeValue());
            out.add(new Observation(new GHPoint(lat, lon)));
        }
        if (out.isEmpty()) throw new IllegalArgumentException("No <trkpt> in " + gpxFile);
        return out;
    }
}
