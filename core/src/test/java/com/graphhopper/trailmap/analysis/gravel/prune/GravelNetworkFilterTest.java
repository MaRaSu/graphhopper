package com.graphhopper.trailmap.analysis.gravel.prune;

import com.graphhopper.trailmap.analysis.gravel.graph.AnalysisGraph;
import com.graphhopper.trailmap.analysis.gravel.role.EdgeRole;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Step 1 (goal §6 case A): keep GRAVEL that is 2-edge-connected to the road grid; drop everything
 * standalone. The grid is the 2-edge-connected core of the roads — road stubs are not grid.
 *
 * NOTE: isRemoved() takes the insertion INDEX, so each edge's id equals its insertion order here.
 */
class GravelNetworkFilterTest {

    /** A real road (backbone ANCHOR). */
    private static void road(AnalysisGraph.Builder b, int id, int from, int to) {
        road(b, id, from, to, 1);
    }

    private static void road(AnalysisGraph.Builder b, int id, int from, int to, double len) {
        b.addEdge(id, from, to, EdgeRole.ANCHOR, len, id, null, null, 0, true);
    }

    private static void gravel(AnalysisGraph.Builder b, int id, int from, int to) {
        gravel(b, id, from, to, 1);
    }

    private static void gravel(AnalysisGraph.Builder b, int id, int from, int to, double len) {
        b.addEdge(id, from, to, EdgeRole.TARGET, len, id, null, null, 0, false);
    }

    /** Step 1 only (τ = ∞: no standalone rescue). */
    private static void filter(AnalysisGraph g) {
        new GravelNetworkFilter(g, Double.POSITIVE_INFINITY).filter();
    }

    private static void filter(AnalysisGraph g, double tau) {
        new GravelNetworkFilter(g, tau).filter();
    }

    private static void filter(AnalysisGraph g, double tau, double gridFloor) {
        new GravelNetworkFilter(g, tau, gridFloor).filter();
    }

    /** A road triangle on {10,11,12} — a genuine grid (a cycle). */
    private static void roadGrid(AnalysisGraph.Builder b) {
        road(b, 0, 10, 11);
        road(b, 1, 11, 12);
        road(b, 2, 12, 10);
    }

    @Test
    void throughGravelKept() {
        // Gravel path between two grid vertices (10 → 20 → 11): a through-route → 2EC to grid.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        roadGrid(b);
        gravel(b, 3, 10, 20);
        gravel(b, 4, 20, 11);
        AnalysisGraph g = b.build();
        filter(g);
        assertFalse(g.isRemoved(3), "through gravel kept");
        assertFalse(g.isRemoved(4), "through gravel kept");
    }

    @Test
    void deadEndSpurDropped() {
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        roadGrid(b);
        gravel(b, 3, 10, 20);   // 20 is a dead tip
        AnalysisGraph g = b.build();
        filter(g);
        assertTrue(g.isRemoved(3), "dead-end spur dropped (not 2EC to grid)");
    }

    @Test
    void lollipopAtSingleGridVertexDropped() {
        // A loop reachable from the grid only via a single stick → standalone → dropped in Step 1.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        roadGrid(b);
        gravel(b, 3, 10, 20);   // stick
        gravel(b, 4, 20, 21);
        gravel(b, 5, 21, 22);
        gravel(b, 6, 22, 20);   // loop 20-21-22
        AnalysisGraph g = b.build();
        filter(g);
        assertTrue(g.isRemoved(3) && g.isRemoved(4) && g.isRemoved(5) && g.isRemoved(6),
                "single-attachment lollipop dropped");
    }

    @Test
    void gravelLoopWithTwoGridAttachmentsKept() {
        // Gravel loop 10-20-11-21-10 touches the grid at 10 and 11 → through → 2EC to grid.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        roadGrid(b);
        gravel(b, 3, 10, 20);
        gravel(b, 4, 20, 11);
        gravel(b, 5, 10, 21);
        gravel(b, 6, 21, 11);
        AnalysisGraph g = b.build();
        filter(g);
        assertFalse(g.isRemoved(3));
        assertFalse(g.isRemoved(4));
        assertFalse(g.isRemoved(5));
        assertFalse(g.isRemoved(6));
    }

    @Test
    void roadStubDoesNotAnchor() {
        // A road stub (10→30, dead) is not part of the grid, so a gravel run that loops back only
        // through it is NOT 2-edge-connected to the grid → dropped.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        roadGrid(b);
        road(b, 3, 10, 30);     // road stub, 30 otherwise dead
        gravel(b, 4, 10, 40);
        gravel(b, 5, 40, 30);   // gravel 10-40-30; closes a cycle only via the stub
        AnalysisGraph g = b.build();
        filter(g);
        assertTrue(g.isRemoved(4) && g.isRemoved(5), "gravel anchored only by a road stub dropped");
    }

    @Test
    void loopThroughSingleGridVertexNotCaseA() {
        // A gravel loop sharing exactly ONE vertex with the grid (10): reachable from a single
        // junction, so NOT a through-route → not case A (it is case B, handled by Step 2).
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        roadGrid(b);
        gravel(b, 3, 10, 20);
        gravel(b, 4, 20, 21);
        gravel(b, 5, 21, 10);   // loop 10-20-21 touches the grid only at vertex 10
        AnalysisGraph g = b.build();
        filter(g);              // τ = ∞ → case A only
        assertTrue(g.isRemoved(3) && g.isRemoved(4) && g.isRemoved(5),
                "single-junction loop is not case A");
    }

    @Test
    void spurOffThroughRouteDropped() {
        // Through-route 10-20-11 kept; a spur off its interior (20-30) is a dead-end → dropped.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        roadGrid(b);
        gravel(b, 3, 10, 20);
        gravel(b, 4, 20, 11);   // through-route 10-20-11
        gravel(b, 5, 20, 30);   // spur off node 20
        AnalysisGraph g = b.build();
        filter(g);
        assertFalse(g.isRemoved(3), "through-route kept");
        assertFalse(g.isRemoved(4), "through-route kept");
        assertTrue(g.isRemoved(5), "spur dropped");
    }

    @Test
    void gridIncludesBetweenCycleBridgeChain() {
        // Grid = two road triangles joined by a 2-edge road bridge chain through an intermediate
        // vertex 16. 16 must count as grid (it is between two cycles), so a gravel through-route
        // 16-20-10 (both endpoints grid) is kept.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        road(b, 0, 10, 11);
        road(b, 1, 11, 12);
        road(b, 2, 12, 10);     // triangle A
        road(b, 3, 13, 14);
        road(b, 4, 14, 15);
        road(b, 5, 15, 13);     // triangle B
        road(b, 6, 12, 16);
        road(b, 7, 16, 13);     // bridge chain A=12 — 16 — 13=B  (16 not on any road cycle)
        gravel(b, 8, 16, 20);
        gravel(b, 9, 20, 10);   // through-route from intermediate grid vertex 16 to grid vertex 10
        AnalysisGraph g = b.build();
        filter(g);
        assertFalse(g.isRemoved(8), "gravel anchored at a between-cycle bridge vertex kept");
        assertFalse(g.isRemoved(9), "gravel through-route to the grid kept");
    }

    /**
     * Real grid (triangle, cyclic core 300) at {10,11,12}; a SEPARATE stray road loop (triangle,
     * core 30) at {50,51,52} that connects to the rest only through gravel (so it is its own road
     * component); gravel runs from a real grid junction (10) to the stray loop (50).
     */
    private static AnalysisGraph.Builder strayRoadLoopGraph() {
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        road(b, 0, 10, 11, 100); road(b, 1, 11, 12, 100); road(b, 2, 12, 10, 100); // real grid, core 300
        road(b, 3, 50, 51, 10);  road(b, 4, 51, 52, 10);  road(b, 5, 52, 50, 10);  // stray loop, core 30
        gravel(b, 6, 10, 20);    // real grid junction 10 → 20
        gravel(b, 7, 20, 50);    // 20 → stray-loop vertex 50
        return b;
    }

    @Test
    void strayTinyRoadLoopDoesNotAnchorGravel() {
        // Goal §5 / Option B. With NO grid floor, the stray loop is 2-edge-connected in isolation, so
        // it counts as grid and the gravel looks like a through-route between two junctions (10 and
        // 50) → wrongly kept (this is the bug: way 329963439 via the ~63 m service loop).
        AnalysisGraph g0 = strayRoadLoopGraph().build();
        filter(g0, Double.POSITIVE_INFINITY, 0);
        assertFalse(g0.isRemoved(6) || g0.isRemoved(7),
                "with no grid floor the stray loop anchors the gravel (the bug)");

        // With the floor between the stray loop's core (30) and the real grid's (300), the stray loop
        // is NOT grid, so the gravel reaches the real grid at a single junction → dead-end → dropped.
        AnalysisGraph g1 = strayRoadLoopGraph().build();
        filter(g1, Double.POSITIVE_INFINITY, 100);
        assertTrue(g1.isRemoved(6) && g1.isRemoved(7),
                "stray tiny road loop must not anchor a dead-end gravel road");
    }

    @Test
    void isolatedGravelLoopDropped() {
        // No roads at all → no grid → all gravel is standalone → dropped in Step 1.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        gravel(b, 0, 20, 21);
        gravel(b, 1, 21, 22);
        gravel(b, 2, 22, 20);
        AnalysisGraph g = b.build();
        filter(g);
        assertTrue(g.isRemoved(0) && g.isRemoved(1) && g.isRemoved(2), "isolated gravel loop dropped");
    }

    // --- Step 2 (case B): standalone rescue by τ on the looped core ---

    @Test
    void isolatedLoopKeptWhenCoreMeetsTau() {
        // Isolated gravel loop, core = 300. Kept at τ=200, dropped at τ=400.
        for (double[] tc : new double[][]{{200, 0}, {400, 1}}) {
            AnalysisGraph.Builder b = new AnalysisGraph.Builder();
            gravel(b, 0, 20, 21, 100);
            gravel(b, 1, 21, 22, 100);
            gravel(b, 2, 22, 20, 100);
            AnalysisGraph g = b.build();
            filter(g, tc[0]);
            boolean expectDropped = tc[1] == 1;
            assertTrue(g.isRemoved(0) == expectDropped, "loop @ tau=" + tc[0]);
        }
    }

    @Test
    void lollipopKeptWholeWhenLoopMeetsTau() {
        // Loop (core 300) off the grid via a stick. At τ=200 the loop AND its access stick are kept
        // (no half-way); at τ=400 the whole thing drops.
        AnalysisGraph.Builder keep = new AnalysisGraph.Builder();
        roadGrid(keep);
        gravel(keep, 3, 10, 20, 50);    // access stick (a bridge to the grid)
        gravel(keep, 4, 20, 21, 100);
        gravel(keep, 5, 21, 22, 100);
        gravel(keep, 6, 22, 20, 100);   // loop, core 300
        AnalysisGraph gk = keep.build();
        filter(gk, 200);
        assertFalse(gk.isRemoved(3), "access stick kept with the loop");
        assertFalse(gk.isRemoved(4));
        assertFalse(gk.isRemoved(5));
        assertFalse(gk.isRemoved(6));

        AnalysisGraph.Builder drop = new AnalysisGraph.Builder();
        roadGrid(drop);
        gravel(drop, 3, 10, 20, 50);
        gravel(drop, 4, 20, 21, 100);
        gravel(drop, 5, 21, 22, 100);
        gravel(drop, 6, 22, 20, 100);
        AnalysisGraph gd = drop.build();
        filter(gd, 400);
        assertTrue(gd.isRemoved(3) && gd.isRemoved(4) && gd.isRemoved(5) && gd.isRemoved(6),
                "whole lollipop dropped below tau");
    }

    @Test
    void straightStandaloneDroppedRegardlessOfLength() {
        // A long straight standalone run has no looped core → dropped even at a tiny τ.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        roadGrid(b);
        gravel(b, 3, 10, 20, 1000);
        gravel(b, 4, 20, 21, 1000);
        gravel(b, 5, 21, 22, 1000);   // 3 km straight, no loop
        AnalysisGraph g = b.build();
        filter(g, 1);
        assertTrue(g.isRemoved(3) && g.isRemoved(4) && g.isRemoved(5), "long straight dropped");
    }

    @Test
    void tailInsideKeptClusterPruned() {
        // A qualifying isolated loop (core 300) with a tail spur: loop kept, tail pruned uniformly.
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        gravel(b, 0, 20, 21, 100);
        gravel(b, 1, 21, 22, 100);
        gravel(b, 2, 22, 20, 100);    // loop, core 300
        gravel(b, 3, 20, 30, 100);    // tail spur off the loop (30 dead)
        AnalysisGraph g = b.build();
        filter(g, 200);
        assertFalse(g.isRemoved(0), "loop kept");
        assertFalse(g.isRemoved(1), "loop kept");
        assertFalse(g.isRemoved(2), "loop kept");
        assertTrue(g.isRemoved(3), "tail inside kept cluster pruned");
    }
}
