package com.graphhopper.trailmap.analysis.gravel.prune;

import com.graphhopper.trailmap.analysis.gravel.graph.AnalysisGraph;
import com.graphhopper.trailmap.analysis.gravel.role.EdgeRole;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

/**
 * Biconnected (block-cut) decomposition on small hand-built graphs (design §15). Bridges
 * are singleton blocks; a 2-connected subgraph (loop/blob) is one block.
 */
class BiconnectedDecompositionTest {

    /** Add a TARGET edge (edge id == its index, names irrelevant to structure). */
    private static int t(AnalysisGraph.Builder b, int id, int from, int to) {
        b.addEdge(id, from, to, EdgeRole.TARGET, 1.0, id, null, null, 0);
        return id;
    }

    @Test
    void plainTailEachEdgeIsItsOwnBridgeBlock() {
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        t(b, 0, 0, 1);
        t(b, 1, 1, 2);
        t(b, 2, 2, 3);
        AnalysisGraph g = b.build();
        BiconnectedDecomposition d = BiconnectedDecomposition.decompose(g);
        assertEquals(3, d.blockCount());
        assertNotEquals(d.blockOf(0), d.blockOf(1));
        assertNotEquals(d.blockOf(1), d.blockOf(2));
    }

    @Test
    void lollipopLoopIsOneBlockStickEdgesAreBridges() {
        // stick 0-1-2, triangle loop 2-3-4-2
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        t(b, 0, 0, 1);
        t(b, 1, 1, 2);
        t(b, 2, 2, 3);
        t(b, 3, 3, 4);
        t(b, 4, 4, 2);
        AnalysisGraph g = b.build();
        BiconnectedDecomposition d = BiconnectedDecomposition.decompose(g);
        // loop edges share one block
        assertEquals(d.blockOf(2), d.blockOf(3));
        assertEquals(d.blockOf(3), d.blockOf(4));
        // stick edges are separate bridge blocks
        assertNotEquals(d.blockOf(0), d.blockOf(1));
        assertNotEquals(d.blockOf(1), d.blockOf(2));
        assertEquals(3, d.blockCount());   // {e0} {e1} {loop}
    }

    @Test
    void sticklessLoopAtJunctionIsOneBlock() {
        // anchor backbone edge (5-0); triangle loop directly at junction 0: 0-1-2-0
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        b.addEdge(0, 5, 0, EdgeRole.ANCHOR, 1.0, 0, null, null, 0);
        t(b, 1, 0, 1);
        t(b, 2, 1, 2);
        t(b, 3, 2, 0);
        AnalysisGraph g = b.build();
        BiconnectedDecomposition d = BiconnectedDecomposition.decompose(g);
        assertEquals(d.blockOf(1), d.blockOf(2));
        assertEquals(d.blockOf(2), d.blockOf(3));
        assertNotEquals(d.blockOf(0), d.blockOf(1));   // anchor bridge is its own block
        assertEquals(2, d.blockCount());
    }

    @Test
    void twoConnectedBlobIsOneBlock() {
        // square 0-1-2-3-0
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        t(b, 0, 0, 1);
        t(b, 1, 1, 2);
        t(b, 2, 2, 3);
        t(b, 3, 3, 0);
        AnalysisGraph g = b.build();
        BiconnectedDecomposition d = BiconnectedDecomposition.decompose(g);
        assertEquals(1, d.blockCount());
        assertEquals(d.blockOf(0), d.blockOf(3));
    }

    @Test
    void throughRoadBetweenTwoAnchorsAllBridges() {
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        b.addEdge(0, 0, 1, EdgeRole.ANCHOR, 1.0, 0, null, null, 0);
        t(b, 1, 1, 2);
        b.addEdge(2, 2, 3, EdgeRole.ANCHOR, 1.0, 2, null, null, 0);
        AnalysisGraph g = b.build();
        BiconnectedDecomposition d = BiconnectedDecomposition.decompose(g);
        assertEquals(3, d.blockCount());
    }
}
