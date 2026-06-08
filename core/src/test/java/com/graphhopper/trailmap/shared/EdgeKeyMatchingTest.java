package com.graphhopper.trailmap.shared;

import com.graphhopper.util.PointList;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Pure unit tests for the graph-independent helpers of {@link EdgeKeyMatching}. These cover the
 * "bullet-proof" edge_key comparison logic extracted from the /convert_track optimizer (the
 * twin-edge tolerance needs a graph and is exercised by the integration suites instead).
 */
class EdgeKeyMatchingTest {

    @Test
    void dedupConsecutive_collapsesRuns() {
        assertArrayEquals(new int[]{1, 2, 3, 1},
                EdgeKeyMatching.dedupConsecutive(new int[]{1, 1, 2, 2, 2, 3, 1}));
        assertArrayEquals(new int[]{}, EdgeKeyMatching.dedupConsecutive(new int[]{}));
        assertArrayEquals(new int[]{7}, EdgeKeyMatching.dedupConsecutive(new int[]{7, 7, 7}));
    }

    @Test
    void basicRule_exactMatch() {
        assertTrue(EdgeKeyMatching.basicRule(new int[]{10, 20, 30}, new int[]{10, 20, 30}));
        assertTrue(EdgeKeyMatching.basicRule(new int[]{}, new int[]{}));
    }

    @Test
    void basicRule_oneEdgeBoundaryTolerance() {
        int[] e = {10, 20, 30};
        assertTrue(EdgeKeyMatching.basicRule(e, new int[]{20, 30}), "leading edge stripped (x=1)");
        assertTrue(EdgeKeyMatching.basicRule(e, new int[]{10, 20}), "trailing edge stripped (y=1)");
        assertTrue(EdgeKeyMatching.basicRule(e, new int[]{20}), "both ends stripped (x=1,y=1)");
    }

    @Test
    void basicRule_rejectsDifferentPath() {
        assertFalse(EdgeKeyMatching.basicRule(new int[]{10, 20, 30}, new int[]{10, 99, 30}));
        assertFalse(EdgeKeyMatching.basicRule(new int[]{10, 20, 30}, new int[]{40, 50}));
    }

    @Test
    void basicRule_directionMatters() {
        // edge 5 forward = key 10, reverse = key 11; a wrong-direction traversal must not match.
        assertFalse(EdgeKeyMatching.basicRule(new int[]{10, 12}, new int[]{11, 12}));
    }

    @Test
    void exitHeadingOf_dueNorthIsZero() {
        PointList pts = new PointList(2, false);
        pts.add(60.0, 24.0);
        pts.add(60.001, 24.0); // moving north
        double h = EdgeKeyMatching.exitHeadingOf(pts);
        assertEquals(0.0, h, 1.0);
    }

    @Test
    void exitHeadingOf_tooShortIsNaN() {
        PointList pts = new PointList(1, false);
        pts.add(60.0, 24.0);
        assertTrue(Double.isNaN(EdgeKeyMatching.exitHeadingOf(pts)));
    }
}
