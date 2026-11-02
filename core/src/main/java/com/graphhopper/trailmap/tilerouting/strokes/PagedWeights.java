package com.graphhopper.trailmap.tilerouting.strokes;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReferenceArray;

/**
 * Directed edge weights shared by one request's {@link PathDijkstra}s, in pages of {@link #PAGE} keys created when a
 * key in them is first needed (NaN = not computed yet). Replaces a static float array over all edge keys: the graph is
 * sorted along a Hilbert curve, so a request touches few pages. Reads and writes of a slot race exactly as they did
 * on that array (any racer writes the same value).
 */
final class PagedWeights {

    static final int SHIFT = 12, PAGE = 1 << SHIFT, MASK = PAGE - 1;

    static int pagesFor(int keys) {
        return (keys >>> SHIFT) + 1;
    }

    private final AtomicReferenceArray<float[]> dir;
    private final AtomicInteger pages = new AtomicInteger();

    PagedWeights(int keys) {
        dir = new AtomicReferenceArray<>(pagesFor(keys));
    }

    float[] page(int key) {
        int i = key >>> SHIFT;
        float[] p = dir.get(i);
        if (p != null) return p;
        float[] q = new float[PAGE];
        Arrays.fill(q, Float.NaN);
        if (dir.compareAndSet(i, null, q)) {
            pages.incrementAndGet();
            return q;
        }
        return dir.get(i);
    }

    /** Weight slots held (pages × page size). */
    long keysHeld() {
        return (long) pages.get() * PAGE;
    }
}
