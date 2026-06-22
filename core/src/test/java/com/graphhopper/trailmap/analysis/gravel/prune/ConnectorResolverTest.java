package com.graphhopper.trailmap.analysis.gravel.prune;

import com.graphhopper.trailmap.analysis.gravel.graph.AnalysisGraph;
import com.graphhopper.trailmap.analysis.gravel.role.EdgeRole;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Phase 2 connector pre-pass (weighted per-corridor). A connector edge costs {@code length × weight};
 * a corridor bridging two distinct attachment points is admitted iff its cheapest weighted cost ≤ the
 * budget. Ids equal insertion order so role()/isRemoved() (which take the insertion index) line up.
 */
class ConnectorResolverTest {

    private static final double BUDGET = 100.0;

    private static void target(AnalysisGraph.Builder b, int id, int from, int to, double len) {
        b.addEdge(id, from, to, EdgeRole.TARGET, len, id, null, null, 0, false);
    }

    private static void connector(AnalysisGraph.Builder b, int id, int from, int to, double len) {
        b.addEdge(id, from, to, EdgeRole.CONNECTOR, len, id, null, null, 0, false);
    }

    private static void anchor(AnalysisGraph.Builder b, int id, int from, int to, double len) {
        b.addEdge(id, from, to, EdgeRole.ANCHOR, len, id, null, null, 0, true);
    }

    /** Resolve at BUDGET with unit weight on every connector (so cost == raw length). */
    private static ConnectorResolver.Result resolveUnit(AnalysisGraph g) {
        for (int e = 0; e < g.edgeCount(); e++)
            if (g.role(e) == EdgeRole.CONNECTOR) g.setConnectorWeight(e, 1.0);
        return new ConnectorResolver(g, BUDGET).resolve();
    }

    @Test
    void shortBridgingCorridorAdmitted() {
        // Two separate gravel stubs; a 50 m connector between their endpoints bridges them.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        target(b, 0, 0, 1, 100);      // attachment node 1
        target(b, 1, 2, 3, 100);      // attachment node 2
        connector(b, 2, 1, 2, 50);    // links nodes 1 and 2; cost 50 <= 100
        AnalysisGraph g = b.build();
        ConnectorResolver.Result r = resolveUnit(g);
        assertEquals(EdgeRole.CONNECTOR, g.role(2), "admitted connector keeps CONNECTOR role");
        assertFalse(g.isRemoved(2), "admitted connector not removed");
        assertEquals(1, r.corridors);
        assertEquals(1, r.admittedEdges);
    }

    @Test
    void overBudgetCorridorRemoved() {
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        target(b, 0, 0, 1, 100);
        target(b, 1, 2, 3, 100);
        connector(b, 2, 1, 2, 200);   // cost 200 > 100
        AnalysisGraph g = b.build();
        ConnectorResolver.Result r = resolveUnit(g);
        assertTrue(g.isRemoved(2), "over-budget connector dropped");
        assertEquals(1, r.removedEdges);
    }

    @Test
    void deadEndingConnectorRemoved() {
        // A connector that reaches only one attachment region bridges nothing -> dropped.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        target(b, 0, 0, 1, 100);      // attachment node 1
        connector(b, 1, 1, 5, 50);    // node 5 is not an attachment -> only one region
        AnalysisGraph g = b.build();
        resolveUnit(g);
        assertTrue(g.isRemoved(1), "dead-ending connector dropped");
    }

    @Test
    void multiEdgeCorridorAdmittedByTotalCost() {
        // A two-edge corridor is bounded by its TOTAL cost, not per edge.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        target(b, 0, 0, 1, 100);      // attachment node 1
        target(b, 1, 2, 3, 100);      // attachment node 2
        connector(b, 2, 1, 4, 40);
        connector(b, 3, 4, 2, 40);    // corridor 1-4-2, total 80 <= 100
        AnalysisGraph g = b.build();
        resolveUnit(g);
        assertFalse(g.isRemoved(2), "admitted corridor edge kept");
        assertFalse(g.isRemoved(3), "admitted corridor edge kept");
    }

    @Test
    void multiEdgeCorridorRemovedWhenTotalTooLong() {
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        target(b, 0, 0, 1, 100);
        target(b, 1, 2, 3, 100);
        connector(b, 2, 1, 4, 60);
        connector(b, 3, 4, 2, 60);    // corridor total 120 > 100 -> dropped whole
        AnalysisGraph g = b.build();
        resolveUnit(g);
        assertTrue(g.isRemoved(2) && g.isRemoved(3), "over-budget corridor dropped whole");
    }

    @Test
    void heavyWeightShortensReach() {
        // Same 80 m connector between two attachments: admitted at weight 1 (cost 80), dropped at
        // weight 2 (cost 160 > budget) — the quality weight, not raw length, decides.
        for (double[] tc : new double[][]{{1.0, 0}, {2.0, 1}}) {
            AnalysisGraph.Builder b = new AnalysisGraph.Builder();
            target(b, 0, 0, 1, 100);
            target(b, 1, 2, 3, 100);
            connector(b, 2, 1, 2, 80);
            AnalysisGraph g = b.build();
            g.setConnectorWeight(2, tc[0]);
            new ConnectorResolver(g, BUDGET).resolve();
            assertEquals(tc[1] == 1, g.isRemoved(2), "weight " + tc[0]);
        }
    }

    @Test
    void mixedQualityCorridorCostsByWeight() {
        // Corridor 1-4-2 mixing a light part (50 m × 1 = 50) and a dear part (10 m × weight).
        // weight 4 -> 50+40 = 90 <= 100 admitted; weight 8 -> 50+80 = 130 > 100 dropped.
        for (double[] tc : new double[][]{{4.0, 0}, {8.0, 1}}) {
            AnalysisGraph.Builder b = new AnalysisGraph.Builder();
            target(b, 0, 0, 1, 100);
            target(b, 1, 2, 3, 100);
            connector(b, 2, 1, 4, 50);
            connector(b, 3, 4, 2, 10);
            AnalysisGraph g = b.build();
            g.setConnectorWeight(2, 1.0);
            g.setConnectorWeight(3, tc[0]);
            new ConnectorResolver(g, BUDGET).resolve();
            boolean expectDropped = tc[1] == 1;
            assertEquals(expectDropped, g.isRemoved(2), "light part @ dear weight " + tc[0]);
            assertEquals(expectDropped, g.isRemoved(3), "dear part @ weight " + tc[0]);
        }
    }

    @Test
    void noConnectorsIsNoOp() {
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        target(b, 0, 0, 1, 100);
        AnalysisGraph g = b.build();
        ConnectorResolver.Result r = resolveUnit(g);
        assertEquals(0, r.admittedEdges);
        assertEquals(0, r.removedEdges);
        assertFalse(g.isRemoved(0));
    }

    // --- removeConnectorsNotServingGravel: connectors must serve RETAINED gravel (finding 2a) ---

    @Test
    void roadToRoadConnectorDroppedFromOutput() {
        // Two ANCHOR roads bridged by a connector, NO gravel anywhere. The resolver admits the
        // connector (it bridges two attachment regions), but it serves no gravel → must be removed.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        anchor(b, 0, 0, 1, 100);
        anchor(b, 1, 2, 3, 100);
        connector(b, 2, 1, 2, 50);
        AnalysisGraph g = b.build();
        resolveUnit(g);
        assertFalse(g.isRemoved(2), "resolver admits the road-to-road connector");
        int removed = ConnectorResolver.removeConnectorsNotServingGravel(g);
        assertEquals(1, removed);
        assertTrue(g.isRemoved(2), "connector serving no gravel is dropped from output");
    }

    @Test
    void connectorServingRetainedGravelKept() {
        // Connector links retained TARGET gravel to an ANCHOR road → serves gravel → kept.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        target(b, 0, 0, 1, 100);      // retained gravel at node 1
        anchor(b, 1, 2, 3, 100);      // road at node 2
        connector(b, 2, 1, 2, 50);
        AnalysisGraph g = b.build();
        resolveUnit(g);
        int removed = ConnectorResolver.removeConnectorsNotServingGravel(g);
        assertEquals(0, removed);
        assertFalse(g.isRemoved(2), "connector serving retained gravel is kept");
    }

    @Test
    void connectorOrphanedByPrunedGravelDropped() {
        // Connector admitted while gravel was present, but that gravel is pruned by the filter
        // (simulated by removing the TARGET edge) → connector now serves no retained gravel → dropped.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        target(b, 0, 0, 1, 100);      // gravel that will be pruned
        anchor(b, 1, 2, 3, 100);
        connector(b, 2, 1, 2, 50);
        AnalysisGraph g = b.build();
        resolveUnit(g);
        g.remove(0);                  // filter prunes the gravel
        int removed = ConnectorResolver.removeConnectorsNotServingGravel(g);
        assertEquals(1, removed);
        assertTrue(g.isRemoved(2), "connector orphaned by pruned gravel is dropped");
    }
}
