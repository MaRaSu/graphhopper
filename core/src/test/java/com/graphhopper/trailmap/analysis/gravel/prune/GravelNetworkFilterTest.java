package com.graphhopper.trailmap.analysis.gravel.prune;

import com.graphhopper.trailmap.analysis.gravel.graph.AnalysisGraph;
import com.graphhopper.trailmap.analysis.gravel.role.EdgeRole;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Whole-appendage dead-end pruning (spec §C/§D). A gravel cluster reaching the real-road backbone
 * two edge-independent ways is a through-network (kept); one reaching it at a single point (or not
 * at all) is a dead-end cluster / island, kept or dropped <b>as one unit</b> by its total
 * qualifying length. Threshold 100 m.
 */
class GravelNetworkFilterTest {

    private static final double THRESHOLD = 100.0;

    private static void target(AnalysisGraph.Builder b, int id, int from, int to, double len) {
        b.addEdge(id, from, to, EdgeRole.TARGET, len, id, null, null, 0, false);
    }

    /** A real-road ANCHOR edge: part of the backbone, so it anchors a non-dead-end. */
    private static void backbone(AnalysisGraph.Builder b, int id, int from, int to, double len) {
        b.addEdge(id, from, to, EdgeRole.ANCHOR, len, id, null, null, 0, true);
    }

    /** A rough/ground ANCHOR track: carries connectivity but is NOT backbone. */
    private static void track(AnalysisGraph.Builder b, int id, int from, int to, double len) {
        b.addEdge(id, from, to, EdgeRole.ANCHOR, len, id, null, null, 0, false);
    }

    private static int filter(AnalysisGraph g) {
        return new GravelNetworkFilter(g, THRESHOLD).filter();
    }

    @Test
    void thinDeadEndPrunedSubstantialDeadEndKept() {
        // A dead-end spur off the backbone: pruned if thin, kept whole if substantial (the size
        // threshold separates a thin dead-end from a substantial area attached at one point).
        AnalysisGraph.Builder thin = new AnalysisGraph.Builder();
        backbone(thin, 0, 10, 0, 1);
        target(thin, 1, 0, 1, 50);     // 50 m < 100 -> thin dead-end
        AnalysisGraph gThin = thin.build();
        filter(gThin);
        assertTrue(gThin.isRemoved(1), "thin dead-end pruned");

        AnalysisGraph.Builder big = new AnalysisGraph.Builder();
        backbone(big, 0, 10, 0, 1);
        target(big, 1, 0, 1, 800);     // 800 m >= 100 -> substantial, kept
        AnalysisGraph gBig = big.build();
        filter(gBig);
        assertFalse(gBig.isRemoved(1), "substantial dead-end attached at one point kept");
    }

    @Test
    void islandKeptWhenBigEnough() {
        AnalysisGraph.Builder big = new AnalysisGraph.Builder();
        target(big, 0, 0, 1, 40);
        target(big, 1, 1, 2, 40);
        target(big, 2, 2, 0, 40);      // isolated triangle, 120 m
        AnalysisGraph gBig = big.build();
        filter(gBig);
        assertFalse(gBig.isRemoved(0), "120 m island kept");

        AnalysisGraph.Builder small = new AnalysisGraph.Builder();
        target(small, 0, 0, 1, 20);
        target(small, 1, 1, 2, 20);
        target(small, 2, 2, 0, 20);    // 60 m island
        AnalysisGraph gSmall = small.build();
        filter(gSmall);
        assertTrue(gSmall.isRemoved(0), "60 m island dropped");
    }

    @Test
    void throughRouteViaBackboneKept() {
        // Gravel out onto a real road and back forms a cycle through the backbone -> 2-edge-connected
        // to the backbone -> kept regardless of size.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        backbone(b, 0, 10, 11, 50);    // a real road between nodes 10 and 11
        target(b, 1, 10, 20, 80);
        target(b, 2, 20, 11, 80);      // gravel 10->20->11; cycle closes through the backbone
        AnalysisGraph g = b.build();
        filter(g);
        assertFalse(g.isRemoved(1), "through-route gravel kept");
        assertFalse(g.isRemoved(2), "through-route gravel kept");
    }

    @Test
    void wholeAppendageKeptAsUnit() {
        // Access stick + terminal loop, reachable only via the stick. Kept WHOLE (stick included)
        // when the appendage meets the threshold.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        backbone(b, 0, 10, 0, 1);
        target(b, 1, 0, 1, 40);        // access stick (a bridge to the backbone)
        target(b, 2, 1, 2, 40);
        target(b, 3, 2, 3, 40);
        target(b, 4, 3, 1, 40);        // terminal loop 1-2-3; appendage = 40 + 120 = 160 >= 100
        AnalysisGraph g = b.build();
        filter(g);
        assertFalse(g.isRemoved(1), "stick kept");
        assertFalse(g.isRemoved(2), "loop kept");
        assertFalse(g.isRemoved(3), "loop kept");
        assertFalse(g.isRemoved(4), "loop kept");
    }

    @Test
    void wholeAppendageDroppedAsUnit_noHalfWay() {
        // Same shape, but below threshold: the WHOLE appendage drops. Crucially the loop is NOT kept
        // while its only access stick is dropped — that half-way split is the bug this rewrite fixes.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        backbone(b, 0, 10, 0, 1);
        target(b, 1, 0, 1, 10);        // stick
        target(b, 2, 1, 2, 10);
        target(b, 3, 2, 3, 10);
        target(b, 4, 3, 1, 10);        // loop; appendage = 40 < 100
        AnalysisGraph g = b.build();
        filter(g);
        assertTrue(g.isRemoved(1) && g.isRemoved(2) && g.isRemoved(3) && g.isRemoved(4),
                "whole appendage (stick + loop) dropped as one unit");
    }

    @Test
    void thinSpurOffCoreLoopRemoved() {
        // Hirviniemenranta shape: a thin spur off a loop that is itself 2-edge-connected to the
        // backbone. The loop is core (kept); the spur is its own thin pendant (pruned).
        // NOTE: isRemoved() takes the insertion INDEX, so ids must equal insertion order here.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        backbone(b, 0, 10, 0, 1);      // idx 0: node 0 anchored
        backbone(b, 1, 11, 2, 1);      // idx 1: node 2 anchored -> triangle touches backbone twice
        target(b, 2, 0, 1, 50);        // idx 2
        target(b, 3, 1, 2, 50);        // idx 3
        target(b, 4, 2, 0, 50);        // idx 4: triangle 0-1-2 -> 2EC to backbone -> core
        target(b, 5, 1, 9, 20);        // idx 5: thin spur off node 1
        AnalysisGraph g = b.build();
        filter(g);
        assertFalse(g.isRemoved(2), "core loop kept");
        assertFalse(g.isRemoved(3), "core loop kept");
        assertFalse(g.isRemoved(4), "core loop kept");
        assertTrue(g.isRemoved(5), "thin spur off the core removed");
    }

    @Test
    void roughTrackDoesNotAnchorDeadEnd() {
        // A rough/ground ANCHOR track is connectivity, not backbone. A thin gravel spur whose only
        // "exit" is such a track is still a dead-end -> pruned (the spur reaches no real road).
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        track(b, 0, 10, 0, 1);         // rough track touching node 0 (NOT backbone)
        target(b, 1, 0, 1, 50);        // 50 m gravel spur; no real-road backbone anywhere
        AnalysisGraph g = b.build();
        filter(g);
        assertTrue(g.isRemoved(1), "spur reachable only via a rough track is a dead-end -> pruned");
    }

    @Test
    void linearChainOffBackboneDroppedWhenThin() {
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        backbone(b, 0, 10, 0, 1);
        target(b, 1, 0, 1, 30);
        target(b, 2, 1, 2, 30);        // chain off the backbone, 60 m < 100
        AnalysisGraph g = b.build();
        int removed = filter(g);
        assertTrue(removed == 2 && g.isRemoved(1) && g.isRemoved(2), "thin chain dropped whole");
    }
}
