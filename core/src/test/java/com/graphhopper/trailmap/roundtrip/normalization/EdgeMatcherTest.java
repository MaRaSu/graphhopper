package com.graphhopper.trailmap.roundtrip.normalization;

import com.carrotsearch.hppc.IntArrayList;
import com.graphhopper.routing.ev.DecimalEncodedValue;
import com.graphhopper.routing.ev.DecimalEncodedValueImpl;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.storage.BaseGraph;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for EdgeMatcher - the core edge comparison logic.
 */
public class EdgeMatcherTest {

    private DecimalEncodedValue speedEnc;
    private EncodingManager em;
    private BaseGraph graph;
    private EdgeMatcher edgeMatcher;

    @BeforeEach
    void setUp() {
        speedEnc = new DecimalEncodedValueImpl("speed", 5, 5, true);
        em = EncodingManager.start().add(speedEnc).build();
        graph = new BaseGraph.Builder(em).create();

        // Create a simple graph with edges of known lengths
        // Node layout:
        //   0 --- 1 --- 2
        //   |     |     |
        //   3 --- 4 --- 5
        //
        // Each edge is 100m (significant, not virtual)

        graph.getNodeAccess().setNode(0, 0.001, 0.000);
        graph.getNodeAccess().setNode(1, 0.001, 0.001);
        graph.getNodeAccess().setNode(2, 0.001, 0.002);
        graph.getNodeAccess().setNode(3, 0.000, 0.000);
        graph.getNodeAccess().setNode(4, 0.000, 0.001);
        graph.getNodeAccess().setNode(5, 0.000, 0.002);

        // Create edges (edge IDs will be 0, 1, 2, 3, 4, 5, 6)
        graph.edge(0, 1).setDistance(100).set(speedEnc, 10, 10);  // edge 0
        graph.edge(1, 2).setDistance(100).set(speedEnc, 10, 10);  // edge 1
        graph.edge(0, 3).setDistance(100).set(speedEnc, 10, 10);  // edge 2
        graph.edge(1, 4).setDistance(100).set(speedEnc, 10, 10);  // edge 3
        graph.edge(2, 5).setDistance(100).set(speedEnc, 10, 10);  // edge 4
        graph.edge(3, 4).setDistance(100).set(speedEnc, 10, 10);  // edge 5
        graph.edge(4, 5).setDistance(100).set(speedEnc, 10, 10);  // edge 6

        // Add a node 6 close to 0 and create a short "virtual" edge (< 5m)
        graph.getNodeAccess().setNode(6, 0.001001, 0.000001);  // Very close to node 0
        graph.edge(0, 6).setDistance(3).set(speedEnc, 10, 10);    // edge 7 - virtual (< 5m)

        edgeMatcher = new EdgeMatcher(graph);
    }

    @Test
    void testIdenticalEdgesMatch() {
        IntArrayList ref = IntArrayList.from(0, 1, 4);
        IntArrayList test = IntArrayList.from(0, 1, 4);

        assertTrue(edgeMatcher.edgesMatch(ref, test, null, null),
            "Identical edge sequences should match");
    }

    @Test
    void testDifferentEdgesDoNotMatch() {
        IntArrayList ref = IntArrayList.from(0, 1);
        IntArrayList test = IntArrayList.from(2, 5);

        assertFalse(edgeMatcher.edgesMatch(ref, test, null, null),
            "Different edge sequences should not match");
    }

    @Test
    void testEmptySequencesDoNotMatch() {
        IntArrayList ref = new IntArrayList();
        IntArrayList test = new IntArrayList();

        assertFalse(edgeMatcher.edgesMatch(ref, test, null, null),
            "Empty sequences should not match");
    }

    @Test
    void testVirtualEdgeInRefIsSkipped() {
        // ref has virtual edge 7 at start, test doesn't
        IntArrayList ref = IntArrayList.from(7, 0, 1);
        IntArrayList test = IntArrayList.from(0, 1);

        assertTrue(edgeMatcher.edgesMatch(ref, test, null, null),
            "Virtual edge in ref should be skipped");
    }

    @Test
    void testVirtualEdgeInTestIsSkipped() {
        // test has virtual edge 7 at start, ref doesn't
        IntArrayList ref = IntArrayList.from(0, 1);
        IntArrayList test = IntArrayList.from(7, 0, 1);

        assertTrue(edgeMatcher.edgesMatch(ref, test, null, null),
            "Virtual edge in test should be skipped");
    }

    @Test
    void testConsecutiveDuplicatesAreRemoved() {
        IntArrayList ref = IntArrayList.from(0, 0, 1, 1, 4);
        IntArrayList test = IntArrayList.from(0, 1, 4);

        assertTrue(edgeMatcher.edgesMatch(ref, test, null, null),
            "Consecutive duplicates should be removed before comparison");
    }

    @Test
    void testMatchStats() {
        IntArrayList ref = IntArrayList.from(0, 1, 4, 6);
        IntArrayList test = IntArrayList.from(0, 1, 6);  // Missing edge 4

        EdgeMatcher.MatchStats stats = edgeMatcher.calculateMatchStats(ref, test);

        System.out.println("Match stats: " + stats);
        assertTrue(stats.getMatchPercentage() > 0, "Should have some matches");
        assertTrue(stats.getMatchPercentage() < 100, "Should not be 100% match");
    }

    @Test
    void testSingleEdgeMatch() {
        IntArrayList ref = IntArrayList.from(0);
        IntArrayList test = IntArrayList.from(0);

        assertTrue(edgeMatcher.edgesMatch(ref, test, null, null),
            "Single identical edge should match");
    }

    @Test
    void testSingleEdgeMismatch() {
        IntArrayList ref = IntArrayList.from(0);
        IntArrayList test = IntArrayList.from(1);

        assertFalse(edgeMatcher.edgesMatch(ref, test, null, null),
            "Single different edge should not match");
    }

    @Test
    void testPartialOverlapDoesNotMatch() {
        // ref = [0, 1, 4], test = [0, 1, 6] - diverge at end
        IntArrayList ref = IntArrayList.from(0, 1, 4);
        IntArrayList test = IntArrayList.from(0, 1, 6);

        assertFalse(edgeMatcher.edgesMatch(ref, test, null, null),
            "Partially overlapping sequences should not match");
    }

    @Test
    void testRefHasExtraSignificantEdgesAtEnd() {
        IntArrayList ref = IntArrayList.from(0, 1, 4);
        IntArrayList test = IntArrayList.from(0, 1);

        assertFalse(edgeMatcher.edgesMatch(ref, test, null, null),
            "Ref with extra significant edges at end should not match");
    }

    @Test
    void testTestHasExtraSignificantEdgesAtEnd() {
        IntArrayList ref = IntArrayList.from(0, 1);
        IntArrayList test = IntArrayList.from(0, 1, 4);

        assertFalse(edgeMatcher.edgesMatch(ref, test, null, null),
            "Test with extra significant edges at end should not match");
    }
}
