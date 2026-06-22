package com.graphhopper.trailmap.analysis.gravel;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

/**
 * Real-extract run on the Tampere PBF (a Finnish area with genuine gravel networks,
 * forestry dead-ends, and asphalt roads). Skips automatically where the extract is absent
 * (e.g. CI). Outputs are written to data/gravel-out for inspection and way-ID verification.
 */
class GravelSegmentToolTampereTest {

    private static final String DATA = "/Users/suomimar/Dropbox/dev/map-server/routing_v2/data";
    private static final String PBF = DATA + "/tampere.osm.pbf";

    @Test
    void extractTampereGravelNetwork() throws Exception {
        File pbf = new File(PBF);
        assumeTrue(pbf.exists(), "tampere.osm.pbf not present — skipping real-extract run");

        GravelAnalysisConfig cfg = new GravelAnalysisConfig();
        // DEBUG config: Step 1 (case A) + Step 2 (standalone τ = 2 km); Step 3 (connectors) OFF.
        cfg.gravelSizeThresholdM = 2000;
        cfg.enableStandaloneRescue = true;
        cfg.enableConnectors = false;
        cfg.validate();

        File graphDir = new File(DATA, "tampere-analysis-gh");
        File outDir = new File(DATA, "gravel-out");

        GravelSegmentTool.Stats st = GravelSegmentTool.run(cfg, PBF, graphDir.getAbsolutePath(), outDir);

        System.out.println("=== Tampere gravel extraction ===");
        System.out.println("nodes=" + st.nodes + " workingEdges=" + st.workingEdges);
        System.out.println("roles: TARGET=" + st.targetEdges + " ANCHOR=" + st.anchorEdges
                + " IGNORED=" + st.ignoredEdges);
        System.out.println("prunedNotNetwork=" + st.prunedNotNetwork
                + " (dead-ends / isolated stubs)");
        System.out.println("logicalRoads=" + st.logicalRoads + " qualifyingWays=" + st.qualifyingWays);
        System.out.println("output: " + st.waysFile.getAbsolutePath());

        assertTrue(st.targetEdges > 0, "Tampere should contain qualifying gravel edges");
        assertTrue(st.qualifyingWays > 0, "expected a non-empty qualifying way set");
        assertTrue(st.waysFile.exists() && st.roadsFile.exists());

        // --- Step 1 oracles (goal §6 case A): gravel must be 2-edge-connected to the road grid. ---
        Set<Long> qualifying = readWayIds(st.waysFile);
        // Every reported dead-end drops: it reaches the grid only one way (often via a road stub,
        // which is not part of the grid).
        assertFalse(qualifying.contains(41428429L), "Hirviniemenranta dead-end -> dropped");
        assertFalse(qualifying.contains(79735420L), "Kivivuorentie dead-end -> dropped");
        assertFalse(qualifying.contains(986643062L), "Haulaniementie dead-end -> dropped");
        assertFalse(qualifying.contains(211468847L), "Kirjoniementie dead-end -> dropped");
        assertFalse(qualifying.contains(41035427L), "Junkkarintie dead-end -> dropped");
        // Dead-end whose only "loop" is a ~10 m turnaround: cyclic core (~278 m) < τ -> dropped.
        assertFalse(qualifying.contains(145212615L), "dead-end with tiny turnaround -> dropped");
        // Dead-end ending at an isolated ~63 m service loop (parking/turning yard). That stray road
        // loop is 2-edge-connected in isolation but its component core < gridMinComponentCoreLenM, so
        // it is NOT grid and cannot anchor the dead-end as a false second junction (Option B).
        assertFalse(qualifying.contains(329963439L), "dead-end via isolated tiny road loop -> dropped");
        // access=private gravel service network: bike_access is false both ways (BikeAccessParser),
        // so the access pre-gate makes it IGNORED — excluded from output AND connectivity.
        assertFalse(qualifying.contains(670296455L), "access=private way -> not bike-traversable -> dropped");
        // With the surface gate relaxed to "non-asphalt", the Salmuksentie loop's GROUND-surface
        // tracks are now TARGET, so it's a native gravel loop (> τ) kept by Step 2 — no connectors.
        assertTrue(qualifying.contains(743395639L), "Salmuksentie kept as native gravel (Step 2)");

        // --- connectivity prop (through-route vs standalone island) ---
        Map<Long, String> conn = readConnectivity(st.attrsFile);
        long through = conn.values().stream().filter("through"::equals).count();
        long island = conn.values().stream().filter("island"::equals).count();
        System.out.println("connectivity: through=" + through + " island=" + island
                + " mixed=" + st.mixedConnectivityWays);
        assertTrue(through > 0 && island > 0, "both connectivity classes present");
        // Salmuksentie is a standalone gravel loop rescued by Step 2 (case B) -> island.
        assertEquals("island", conn.get(743395639L), "Salmuksentie standalone loop -> island");
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

    private static Set<Long> readWayIds(File waysFile) throws Exception {
        JsonNode root = new ObjectMapper().readTree(waysFile);
        Set<Long> ids = new HashSet<>();
        for (JsonNode n : root.get("way_ids")) ids.add(n.asLong());
        return ids;
    }
}
