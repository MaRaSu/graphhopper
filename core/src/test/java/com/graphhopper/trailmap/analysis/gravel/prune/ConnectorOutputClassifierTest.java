package com.graphhopper.trailmap.analysis.gravel.prune;

import com.graphhopper.trailmap.analysis.gravel.graph.AnalysisGraph;
import com.graphhopper.trailmap.analysis.gravel.role.EdgeRole;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Output classification of admitted connectors: a connector is "needed" iff it is a bridge in the
 * kept network (removing it would disconnect kept gravel); a connector on a cycle (a loop beside
 * already-connected gravel) is "redundant". This labels output only — it never mutates connectivity.
 */
class ConnectorOutputClassifierTest {

    private static void target(AnalysisGraph.Builder b, int id, int from, int to) {
        b.addEdge(id, from, to, EdgeRole.TARGET, 100, id, null, null, 0, false);
    }

    private static void connector(AnalysisGraph.Builder b, int id, int from, int to) {
        b.addEdge(id, from, to, EdgeRole.CONNECTOR, 100, id, null, null, 0, false);
    }

    @Test
    void bridgeConnectorIsNeeded() {
        // Two separate gravel components joined ONLY by the connector -> it is a bridge -> needed.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        target(b, 0, 10, 11);
        target(b, 1, 20, 21);
        connector(b, 2, 11, 20);
        AnalysisGraph g = b.build();
        boolean[] needed = ConnectorOutputClassifier.classify(g);
        assertTrue(needed[2], "sole link between two gravel components is needed");
    }

    @Test
    void loopConnectorIsRedundant() {
        // Gravel triangle 10-11-12 (already connected); a connector 10-12 is parallel to the gravel
        // path -> it lies on a cycle -> not a bridge -> redundant.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        target(b, 0, 10, 11);
        target(b, 1, 11, 12);
        target(b, 2, 12, 10);
        connector(b, 3, 10, 12);
        AnalysisGraph g = b.build();
        boolean[] needed = ConnectorOutputClassifier.classify(g);
        assertFalse(needed[3], "loop beside already-connected gravel is redundant");
    }

    @Test
    void multiHopChainAllNeeded() {
        // A two-hop connector chain that is the sole route from an island to the rest: both bridges.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        target(b, 0, 10, 11);          // island A
        target(b, 1, 30, 31);          // island B (the "grid" side)
        connector(b, 2, 11, 20);       // A -> intermediate node 20
        connector(b, 3, 20, 30);       // intermediate -> B  (node 20 has degree 2)
        AnalysisGraph g = b.build();
        boolean[] needed = ConnectorOutputClassifier.classify(g);
        assertTrue(needed[2] && needed[3], "every link of a sole connector chain is needed");
    }
}
