package com.graphhopper.trailmap.analysis.gravel.group;

import com.graphhopper.trailmap.analysis.gravel.graph.AnalysisGraph;
import com.graphhopper.trailmap.analysis.gravel.role.EdgeRole;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Phase E grouping (design §9 / §15): contiguous same-name runs, ref lever, unnamed handling.
 */
class LogicalRoadGrouperTest {

    private static void target(AnalysisGraph.Builder b, int id, int from, int to, String name, String ref) {
        b.addEdge(id, from, to, EdgeRole.TARGET, 100.0, id, name, ref, 0);
    }

    private static LogicalRoadGrouper.LogicalRoad named(LogicalRoadGrouper.Result r, String name) {
        return r.roads.stream().filter(x -> name.equals(x.name)).findFirst().orElseThrow();
    }

    @Test
    void sameNameAcrossJunctionMergesIntoOneRoad() {
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        target(b, 100, 0, 1, "Metsätie", null);
        target(b, 101, 1, 2, "Metsätie", null);   // shares node 1
        AnalysisGraph g = b.build();
        LogicalRoadGrouper.Result r = new LogicalRoadGrouper(g, false, true).group();
        assertEquals(1, r.roads.size());
        assertEquals(2, named(r, "Metsätie").wayIds.size());
        assertEquals(2, r.allWayIds.size());
    }

    @Test
    void sameNameDisconnectedYieldsTwoGroups() {
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        target(b, 100, 0, 1, "Foo", null);
        target(b, 101, 2, 3, "Foo", null);   // no shared node
        AnalysisGraph g = b.build();
        LogicalRoadGrouper.Result r = new LogicalRoadGrouper(g, false, true).group();
        assertEquals(2, r.roads.size());
    }

    @Test
    void refLeverSplitsWhenRequired() {
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        target(b, 100, 0, 1, "Foo", "A");
        target(b, 101, 1, 2, "Foo", "B");   // shares node, same name, different ref

        AnalysisGraph g1 = b.build();
        // requireSameRef = false -> merge
        assertEquals(1, new LogicalRoadGrouper(g1, false, true).group().roads.size());

        AnalysisGraph g2 = b.build();
        // requireSameRef = true -> split
        assertEquals(2, new LogicalRoadGrouper(g2, true, true).group().roads.size());
    }

    @Test
    void caseAndWhitespaceInsensitiveNameMatch() {
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        target(b, 100, 0, 1, "Metsätie", null);
        target(b, 101, 1, 2, "  metsätie ", null);
        AnalysisGraph g = b.build();
        LogicalRoadGrouper.Result r = new LogicalRoadGrouper(g, false, true).group();
        assertEquals(1, r.roads.size());
    }

    @Test
    void unnamedHandlingRespectsEmitLever() {
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        target(b, 100, 0, 1, "", null);
        target(b, 101, 2, 3, null, null);
        AnalysisGraph g = b.build();

        LogicalRoadGrouper.Result emit = new LogicalRoadGrouper(g, false, true).group();
        assertEquals(1, emit.roads.size());
        assertEquals(LogicalRoadGrouper.UNNAMED, emit.roads.get(0).name);
        assertEquals(2, emit.roads.get(0).wayIds.size());
        assertEquals(2, emit.allWayIds.size(), "unnamed ways still appear in the flat qualifying list");

        AnalysisGraph g2 = b.build();
        LogicalRoadGrouper.Result drop = new LogicalRoadGrouper(g2, false, false).group();
        assertTrue(drop.roads.isEmpty());
        assertEquals(2, drop.allWayIds.size());
    }
}
