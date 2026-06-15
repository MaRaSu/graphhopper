package com.graphhopper.trailmap.analysis.gravel.prune;

import com.graphhopper.trailmap.analysis.gravel.graph.AnalysisGraph;
import com.graphhopper.trailmap.analysis.gravel.role.EdgeRole;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Phase 2 connector pre-pass: a bounded connector chain that bridges >= 2 attachment points is
 * admitted (promoted to ANCHOR); a chain too long, or one that dead-ends, is removed. Ids equal
 * insertion order so role()/isRemoved() (which take the insertion index) line up.
 */
class ConnectorResolverTest {

    private static final double MAX_LEN = 100.0;
    private static final int MIN_ATTACH = 2;

    private static void target(AnalysisGraph.Builder b, int id, int from, int to, double len) {
        b.addEdge(id, from, to, EdgeRole.TARGET, len, id, null, null, 0, false);
    }

    private static void connector(AnalysisGraph.Builder b, int id, int from, int to, double len) {
        b.addEdge(id, from, to, EdgeRole.CONNECTOR, len, id, null, null, 0, false);
    }

    private static ConnectorResolver.Result resolve(AnalysisGraph g) {
        return new ConnectorResolver(g, MAX_LEN, MIN_ATTACH).resolve();
    }

    @Test
    void shortBridgingChainAdmitted() {
        // Two separate gravel stubs; a 50 m connector between their endpoints bridges them.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        target(b, 0, 0, 1, 100);      // attachment node 1
        target(b, 1, 2, 3, 100);      // attachment node 2
        connector(b, 2, 1, 2, 50);    // links nodes 1 and 2; len 50 <= 100, 2 attachments
        AnalysisGraph g = b.build();
        ConnectorResolver.Result r = resolve(g);
        assertEquals(EdgeRole.ANCHOR, g.role(2), "short bridging connector promoted to ANCHOR");
        assertFalse(g.isRemoved(2));
        assertEquals(1, r.admittedChains);
        assertEquals(1, r.admittedEdges);
    }

    @Test
    void longChainRemoved() {
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        target(b, 0, 0, 1, 100);
        target(b, 1, 2, 3, 100);
        connector(b, 2, 1, 2, 200);   // 200 m > 100 -> not a short crossing
        AnalysisGraph g = b.build();
        ConnectorResolver.Result r = resolve(g);
        assertTrue(g.isRemoved(2), "over-long connector dropped");
        assertEquals(1, r.removedEdges);
    }

    @Test
    void deadEndingChainRemoved() {
        // A connector that reaches only one attachment point bridges nothing -> dropped.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        target(b, 0, 0, 1, 100);      // attachment node 1
        connector(b, 1, 1, 5, 50);    // node 5 is not an attachment -> only 1 attachment
        AnalysisGraph g = b.build();
        resolve(g);
        assertTrue(g.isRemoved(1), "dead-ending connector dropped");
    }

    @Test
    void multiEdgeChainAdmittedByTotalLength() {
        // A connector CHAIN (two edges) is bounded by its TOTAL length, not per edge.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        target(b, 0, 0, 1, 100);      // attachment node 1
        target(b, 1, 2, 3, 100);      // attachment node 2
        connector(b, 2, 1, 4, 40);
        connector(b, 3, 4, 2, 40);    // chain 1-4-2, total 80 <= 100, bridges nodes 1 and 2
        AnalysisGraph g = b.build();
        resolve(g);
        assertEquals(EdgeRole.ANCHOR, g.role(2), "chain edge admitted");
        assertEquals(EdgeRole.ANCHOR, g.role(3), "chain edge admitted");
    }

    @Test
    void multiEdgeChainRemovedWhenTotalTooLong() {
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        target(b, 0, 0, 1, 100);
        target(b, 1, 2, 3, 100);
        connector(b, 2, 1, 4, 60);
        connector(b, 3, 4, 2, 60);    // chain total 120 > 100 -> dropped whole
        AnalysisGraph g = b.build();
        resolve(g);
        assertTrue(g.isRemoved(2) && g.isRemoved(3), "over-long chain dropped whole");
    }

    @Test
    void noConnectorsIsNoOp() {
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        target(b, 0, 0, 1, 100);
        AnalysisGraph g = b.build();
        ConnectorResolver.Result r = resolve(g);
        assertEquals(0, r.admittedEdges);
        assertEquals(0, r.removedEdges);
        assertFalse(g.isRemoved(0));
    }
}
