package com.graphhopper.trailmap.analysis.gravel;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.HashSet;
import java.util.Set;

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
        cfg.gravelSizeThresholdM = 4000;  // a dead-end cluster/island needs >= 4 km qualifying gravel to survive
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

        // --- Whole-appendage pruning regression oracles (Phase C/D, 4 km threshold) ---
        Set<Long> qualifying = readWayIds(st.waysFile);
        // A thin dead-end spur (388 m, degree-1) off a network -> pruned whole.
        assertFalse(qualifying.contains(41428429L), "Hirviniemenranta is a thin dead-end -> pruned");
        // A gravel road 2-edge-connected to the real-road backbone -> kept (its terminal dead-end
        // prunes, but the road itself stays — 'mostly kept').
        assertTrue(qualifying.contains(27060078L), "Latohuhdantie reaches the backbone -> kept");
        // The Varsamäentie/Salmuksentie cluster is one network kept WHOLE — the access stick is no
        // longer dropped while its loop is kept (the half-way bug). All three stay together.
        assertTrue(qualifying.contains(1254246199L), "Varsamäentie (access stick) kept with its cluster");
        assertTrue(qualifying.contains(743395639L), "Salmuksentie kept with its cluster");
        assertTrue(qualifying.contains(232474445L), "Salmuksentie loop kept with its cluster");
    }

    private static Set<Long> readWayIds(File waysFile) throws Exception {
        JsonNode root = new ObjectMapper().readTree(waysFile);
        Set<Long> ids = new HashSet<>();
        for (JsonNode n : root.get("way_ids")) ids.add(n.asLong());
        return ids;
    }
}
