package com.graphhopper.trailmap.analysis.gravel;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.HashMap;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

/**
 * Phase 2 (Step 3) end-to-end on the Tampere extract: same pipeline as {@link
 * GravelSegmentToolTampereTest} but with <b>weighted connectors ENABLED</b>. Writes to a separate
 * output dir (data/gravel-out-connectors) so the Phase-1 artifacts in data/gravel-out are left
 * intact for side-by-side comparison. Uses the default connector weights/budget (conservative).
 */
class GravelConnectorTampereTest {

    private static final String DATA = "/Users/suomimar/Dropbox/dev/map-server/routing_v2/data";
    private static final String PBF = DATA + "/tampere.osm.pbf";

    @Test
    void extractWithConnectors() throws Exception {
        File pbf = new File(PBF);
        assumeTrue(pbf.exists(), "tampere.osm.pbf not present — skipping");

        GravelAnalysisConfig cfg = new GravelAnalysisConfig();
        cfg.gravelSizeThresholdM = 2000;       // match the Phase-1 map config
        cfg.enableConnectors = true;           // weighted connectors (default weights + budget)
        cfg.validate();

        File graphDir = new File(DATA, "tampere-analysis-gh");
        File outDir = new File(DATA, "gravel-out-connectors");

        GravelSegmentTool.Stats st = GravelSegmentTool.run(cfg, PBF, graphDir.getAbsolutePath(), outDir);

        System.out.println("=== Tampere gravel extraction (WEIGHTED CONNECTORS, budget "
                + cfg.connectorCostBudget + ") ===");
        System.out.println("connector weights: " + cfg.connectorWeights
                + " mtb overrides: " + cfg.connectorMtbWeightOverrides);
        System.out.println("roles: TARGET=" + st.targetEdges + " ANCHOR=" + st.anchorEdges
                + " IGNORED=" + st.ignoredEdges + " CONNECTOR-candidate=" + st.connectorEdges);
        System.out.println("connectors: admitted " + st.connectorChainsAdmitted + " corridors ("
                + st.connectorEdgesAdmitted + " edges), dropped " + st.connectorEdgesRemoved + " edges");
        System.out.println("prunedNotNetwork=" + st.prunedNotNetwork);
        System.out.println("logicalRoads=" + st.logicalRoads + " qualifyingWays=" + st.qualifyingWays);

        Map<Long, String> conn = readConnectivity(st.attrsFile);
        long through = conn.values().stream().filter("through"::equals).count();
        long island = conn.values().stream().filter("island"::equals).count();
        long connector = conn.values().stream().filter("connector"::equals).count();
        System.out.println("connectivity: through=" + through + " island=" + island
                + " connector=" + connector);
        System.out.println("41726977 (near-miss scale-2 track) -> " + conn.get(41726977L));
        System.out.println("743395639 (Salmuksentie) -> " + conn.get(743395639L));
        System.out.println("output: " + outDir.getAbsolutePath());

        assertTrue(st.connectorEdges > 0, "Tampere has near-Target connector candidates");
        assertTrue(st.connectorEdgesAdmitted > 0, "some weighted connector corridors are admitted");
        assertTrue(connector > 0, "admitted connectors are emitted with connectivity=connector");
        assertTrue(st.qualifyingWays > 0, "expected a non-empty qualifying way set");
    }

    private static Map<Long, String> readConnectivity(File attrsFile) throws Exception {
        JsonNode ways = new ObjectMapper().readTree(attrsFile).get("ways");
        Map<Long, String> out = new HashMap<>();
        for (java.util.Iterator<String> it = ways.fieldNames(); it.hasNext(); ) {
            String k = it.next();
            JsonNode c = ways.get(k).get("connectivity");
            if (c != null) out.put(Long.parseLong(k), c.asText());
        }
        return out;
    }
}
