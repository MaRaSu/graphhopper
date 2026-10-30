/*
 * Trailmap - Personal routing
 */
package com.graphhopper.trailmap.personal;

import com.graphhopper.storage.BaseGraph;
import com.graphhopper.util.DistanceCalc;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.FetchMode;
import com.graphhopper.util.PointList;

import java.util.concurrent.atomic.AtomicLong;

/**
 * How familiar each edge is to one user, worked out lazily: the first time a search looks at an
 * edge, its geometry is sampled every few metres against the user's cells, and the result is kept
 * for the life of this object (one per user and data set, shared by that user's requests until the
 * store drops it). Nothing is computed up front, so a user's first request does not wait for a
 * table build; it pays only for the edges its own search touches.
 *
 * <p>Per edge: {@code share}, the part of its length that lies in the user's cells (0..1), and
 * {@code rides}, the average ride count of those cells. Virtual edges (the snapped start and end
 * pieces of a request) are computed every time and never cached, since their ids are per request.
 *
 * <p>Both values of an edge live in one {@code char} (low byte: 1 + share in percent, 0 = not
 * computed; high byte: rides), written in a single store without locks. A reader therefore never
 * sees one value without the other. Concurrent requests for the same user may compute the same
 * edge twice; both write the same value, so the race is harmless.
 */
public final class PersonalFamiliarity {

    /** Sampling step along an edge, metres. A cell at zoom 20 is 13-19 m in Finland. */
    static final double STEP_M = 5.0;

    private static final DistanceCalc DIST = DistanceCalcEarth.DIST_EARTH;

    private final PersonalCells cells;
    private final BaseGraph graph;
    /** Per edge: low byte 0 = not computed, else 1 + share in percent (1..101); high byte rides (0..255). */
    private final char[] packed;
    private final AtomicLong computed = new AtomicLong();

    public PersonalFamiliarity(PersonalCells cells, BaseGraph graph) {
        this.cells = cells;
        this.graph = graph;
        this.packed = new char[graph.getEdges()];
    }

    public PersonalCells getCells() {
        return cells;
    }

    /** Edges computed so far (for logging and the previewer). */
    public long getComputedEdges() {
        return computed.get();
    }

    /** Share of this edge's length in the user's cells, 0..1. */
    public double share(EdgeIteratorState edge) {
        int id = edge.getEdge();
        if (id >= 0 && id < packed.length)
            return shareOf(ensure(id));
        return measure(edge.fetchWayGeometry(FetchMode.ALL))[0];
    }

    /** Average ride count over the visited part of this edge, 0 if never ridden. */
    public int rides(EdgeIteratorState edge) {
        int id = edge.getEdge();
        if (id >= 0 && id < packed.length)
            return ensure(id) >>> 8;
        return (int) Math.round(measure(edge.fetchWayGeometry(FetchMode.ALL))[1]);
    }

    /** Share for a base-graph edge id (turn costs see only real edges). */
    public double share(int edgeId) {
        if (edgeId < 0 || edgeId >= packed.length) return 0;
        return shareOf(ensure(edgeId));
    }

    private static double shareOf(char v) {
        return ((v & 0xFF) - 1) / 100.0;
    }

    private char ensure(int id) {
        char v = packed[id];
        if (v != 0) return v;
        EdgeIteratorState e = graph.getEdgeIteratorState(id, Integer.MIN_VALUE);
        double[] m = measure(e.fetchWayGeometry(FetchMode.ALL));
        long rides = Math.min(255, Math.round(m[1]));
        long pct = 1 + Math.round(m[0] * 100);
        v = (char) ((rides << 8) | pct);
        packed[id] = v;
        computed.incrementAndGet();
        return v;
    }

    /** {share 0..1, average rides over visited samples}. */
    double[] measure(PointList pl) {
        int n = pl.size();
        if (n == 0) return new double[]{0, 0};
        if (n == 1) {
            int r = cells.ridesAt(pl.getLat(0), pl.getLon(0));
            return new double[]{r > 0 ? 1 : 0, r};
        }
        double total = 0, hit = 0, ridesSum = 0;
        for (int i = 0; i < n - 1; i++) {
            double lat1 = pl.getLat(i), lon1 = pl.getLon(i), lat2 = pl.getLat(i + 1), lon2 = pl.getLon(i + 1);
            double len = DIST.calcDist(lat1, lon1, lat2, lon2);
            int samples = Math.max(1, (int) Math.ceil(len / STEP_M));
            double part = len / samples;
            for (int s = 0; s < samples; s++) {
                double t = (s + 0.5) / samples;
                int r = cells.ridesAt(lat1 + (lat2 - lat1) * t, lon1 + (lon2 - lon1) * t);
                total += part;
                if (r > 0) {
                    hit += part;
                    ridesSum += r * part;
                }
            }
        }
        if (total <= 0) {
            int r = cells.ridesAt(pl.getLat(0), pl.getLon(0));
            return new double[]{r > 0 ? 1 : 0, r};
        }
        return new double[]{hit / total, hit > 0 ? ridesSum / hit : 0};
    }
}
