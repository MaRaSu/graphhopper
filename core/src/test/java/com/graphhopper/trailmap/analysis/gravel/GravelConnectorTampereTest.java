package com.graphhopper.trailmap.analysis.gravel;

import org.junit.jupiter.api.Test;

import java.io.File;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

/**
 * Phase 2 end-to-end on the Tampere extract: same pipeline as {@link GravelSegmentToolTampereTest}
 * but with bounded connectors ENABLED. Writes to a separate output dir (data/gravel-out-connectors)
 * so the Phase-1 artifacts in data/gravel-out are left intact for side-by-side comparison.
 */
class GravelConnectorTampereTest {

    private static final String DATA = "/Users/suomimar/Dropbox/dev/map-server/routing_v2/data";
    private static final String PBF = DATA + "/tampere.osm.pbf";

    @Test
    void extractWithConnectors() throws Exception {
        File pbf = new File(PBF);
        assumeTrue(pbf.exists(), "tampere.osm.pbf not present — skipping");

        GravelAnalysisConfig cfg = new GravelAnalysisConfig();
        cfg.gravelSizeThresholdM = 4000;
        cfg.enableConnectors = true;
        cfg.connectorMaxChainLenM = 200;       // a short crossing (tunable)
        cfg.connectorMinAttachmentPoints = 2;  // must bridge two things
        cfg.validate();

        File graphDir = new File(DATA, "tampere-analysis-gh");
        File outDir = new File(DATA, "gravel-out-connectors");

        GravelSegmentTool.Stats st = GravelSegmentTool.run(cfg, PBF, graphDir.getAbsolutePath(), outDir);

        System.out.println("=== Tampere gravel extraction (CONNECTORS ON, <=200m) ===");
        System.out.println("roles: TARGET=" + st.targetEdges + " ANCHOR=" + st.anchorEdges
                + " IGNORED=" + st.ignoredEdges + " CONNECTOR-candidate=" + st.connectorEdges);
        System.out.println("connectors: admitted " + st.connectorChainsAdmitted + " chains ("
                + st.connectorEdgesAdmitted + " edges), dropped " + st.connectorEdgesRemoved + " edges");
        System.out.println("prunedNotNetwork=" + st.prunedNotNetwork);
        System.out.println("logicalRoads=" + st.logicalRoads + " qualifyingWays=" + st.qualifyingWays);
        System.out.println("output: " + outDir.getAbsolutePath());

        assertTrue(st.connectorEdges > 0, "Tampere has path/gravel-4 connector candidates");
        assertTrue(st.qualifyingWays > 0, "expected a non-empty qualifying way set");
    }
}
