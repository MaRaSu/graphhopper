package com.graphhopper.trailmap.tilerouting.looprepair;

import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.trailmap.tilerouting.foundation.Portal;
import com.graphhopper.trailmap.tilerouting.foundation.SparseKeys;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;

import java.util.*;

/**
 * Plain-via cost table for candidate 2: road distance (along the profile's min-weight path, with turn costs) between
 * every pair of via locations. A plain via (no heading, no pass_through) can be left and reached in either direction,
 * so each location is both directed states of its snapped edge, as sources and as targets. Copy of the foundation's
 * {@code EdgeDijkstra} without the pass_through re-entry penalty, plus a distance label carried along the weight label.
 * Used only to propose moves; every move is accepted or rejected on the real routed geometry.
 */
final class PlainCostTable {

    /** One via location: both directed states of the edge it sits on. */
    static final class Loc {
        int id;
        double lat, lon;
        String tileKey; // null = home or a shape point
        List<Portal> states = new ArrayList<>();
    }

    private final BaseGraph graph;
    private final Weighting weighting;
    private final EdgeExplorer explorer;
    /** Per-search state in slots of {@link #keys} (cleared per search): memory follows the search, not the graph. */
    private final SparseKeys keys = new SparseKeys();
    private double[] label = new double[keys.capacity()], dlabel = new double[keys.capacity()];
    private double[] hk = new double[1 << 16];
    private int[] hv = new int[1 << 16], hn = new int[1 << 16];
    private int size;
    long settled, searches;
    /** Peak keys of the extra worker tables of {@link #buildParallel} (memory check). */
    long keysHeldAll;
    boolean anyBudgetHit;

    PlainCostTable(BaseGraph graph, Weighting weighting) {
        this.graph = graph;
        this.weighting = weighting;
        this.explorer = graph.createEdgeExplorer();
    }

    /** Most edge keys held at once by one search of this instance. */
    int peakKeys() {
        return keys.peak();
    }

    /** dist[i][j] in metres, +inf if not reached within the budget. */
    /** Road search only to locations within this straight-line radius; farther ones get {@link #DETOUR} × straight line. */
    double radiusM = 6000; // prototype tr.c2.tableRadiusM
    static final double DETOUR = 1.4;
    long estimated;

    double[][] build(List<Loc> locs, int maxSettled, int capacity) {
        int n = locs.size();
        double[][] d = new double[capacity][capacity];
        for (int i = 0; i < n; i++) System.arraycopy(row(locs.get(i), locs, maxSettled), 0, d[i], 0, n);
        return d;
    }

    /** {@link #build} with one searcher per thread (each has its own label arrays); stats are summed into this one. */
    double[][] buildParallel(List<Loc> locs, int maxSettled, int capacity, int threads) throws Exception {
        if (threads <= 1) return build(locs, maxSettled, capacity);
        int n = locs.size();
        double[][] d = new double[capacity][capacity];
        List<PlainCostTable> workers = new ArrayList<>();
        workers.add(this);
        for (int k = 1; k < threads; k++) {
            PlainCostTable w = new PlainCostTable(graph, weighting);
            w.radiusM = radiusM;
            workers.add(w);
        }
        java.util.concurrent.ExecutorService ex = java.util.concurrent.Executors.newFixedThreadPool(threads);
        try {
            java.util.concurrent.atomic.AtomicInteger next = new java.util.concurrent.atomic.AtomicInteger();
            List<java.util.concurrent.Future<?>> fs = new ArrayList<>();
            for (PlainCostTable w : workers)
                fs.add(ex.submit(() -> {
                    for (int i; (i = next.getAndIncrement()) < n; )
                        System.arraycopy(w.row(locs.get(i), locs, maxSettled), 0, d[i], 0, n);
                }));
            for (java.util.concurrent.Future<?> fu : fs) fu.get();
        } finally {
            ex.shutdown();
        }
        for (PlainCostTable w : workers)
            if (w != this) {
                keysHeldAll += w.peakKeys();
                settled += w.settled;
                searches += w.searches;
                estimated += w.estimated;
                anyBudgetHit |= w.anyBudgetHit;
            }
        return d;
    }

    /** Distances from one location to every location of the list (road within the radius, estimate beyond). */
    double[] row(Loc src, List<Loc> locs, int maxSettled) {
        List<Portal> targets = new ArrayList<>();
        List<Integer> owner = new ArrayList<>();
        double[] r = new double[locs.size()];
        Arrays.fill(r, Double.POSITIVE_INFINITY);
        for (Loc l : locs) {
            double sl = com.graphhopper.trailmap.tilerouting.foundation.Foundation.dist(src.lat, src.lon, l.lat, l.lon);
            if (sl > radiusM) {
                r[l.id] = DETOUR * sl;
                estimated++;
                continue;
            }
            for (Portal p : l.states) {
                targets.add(p);
                owner.add(l.id);
            }
        }
        double[] c = search(src.states, targets, owner, maxSettled);
        for (int t = 0; t < targets.size(); t++) {
            int o = owner.get(t);
            double prev = r[o];
            r[o] = Math.min(Double.isInfinite(prev) ? Double.POSITIVE_INFINITY : prev, c[t]);
        }
        if (src.id < r.length) r[src.id] = 0;
        return r;
    }

    private double[] search(List<Portal> sources, List<Portal> targets, List<Integer> owner, int maxSettled) {
        keys.clear();
        size = 0;
        searches++;
        double[] res = new double[targets.size()];
        double[] resW = new double[targets.size()];
        Arrays.fill(res, Double.POSITIVE_INFINITY);
        Arrays.fill(resW, Double.POSITIVE_INFINITY);
        Map<Integer, List<Integer>> byKey = new HashMap<>();
        for (int i = 0; i < targets.size(); i++) byKey.computeIfAbsent(targets.get(i).key, k -> new ArrayList<>()).add(i);
        for (Portal p : sources) {
            List<Integer> same = byKey.get(p.key);
            if (same != null)
                for (int t : same) {
                    Portal q = targets.get(t);
                    if (q.frac >= p.frac) {
                        double w = (q.frac - p.frac) * p.edgeWeight;
                        if (w < resW[t]) {
                            resW[t] = w;
                            res[t] = (q.frac - p.frac) * p.edgeLenM;
                        }
                    }
                }
            relaxFrom(p.edge, p.toNode, (1 - p.frac) * p.edgeWeight, (1 - p.frac) * p.edgeLenM);
        }
        int remaining = byKey.size();
        Set<Integer> done = new HashSet<>(), ownerDone = new HashSet<>();
        int ownersLeft = new HashSet<>(owner).size();
        if (ownersLeft == 0) return res;
        long s = 0;
        while (size > 0) {
            double c = hk[0];
            int key = hv[0], node = hn[0];
            pop();
            int ks = keys.slot(key);
            if (ks < 0 || c > label[ks]) continue;
            double dk = dlabel[ks];
            s++;
            List<Integer> hit = byKey.get(key);
            if (hit != null && done.add(key)) {
                for (int t : hit) {
                    Portal q = targets.get(t);
                    double w = c - (1 - q.frac) * q.edgeWeight;
                    if (w < resW[t]) {
                        resW[t] = w;
                        res[t] = dk - (1 - q.frac) * q.edgeLenM;
                    }
                }
                if (--remaining == 0) break;
                // a location is done once one of its states is settled: its other state may be unreachable (one-way,
                // turn restriction) and must not keep the search running to the budget
                for (int t : hit) if (ownerDone.add(owner.get(t))) ownersLeft--;
                if (ownersLeft == 0) break;
            }
            if (s >= maxSettled) {
                anyBudgetHit = true;
                break;
            }
            relaxFrom(key >> 1, node, c, dk);
        }
        settled += s;
        return res;
    }

    private void relaxFrom(int inEdge, int node, double c, double dc) {
        EdgeIterator it = explorer.setBaseNode(node);
        while (it.next()) {
            double w = weighting.calcEdgeWeight(it, false);
            if (!Double.isFinite(w)) continue;
            double t = weighting.calcTurnWeight(inEdge, node, it.getEdge());
            if (!Double.isFinite(t)) continue;
            double nc = c + t + w;
            int k = it.getEdgeKey();
            int s = keys.slot(k);
            if (s >= 0 && label[s] <= nc) continue;
            if (s < 0) {
                s = keys.insert(k);
                if (s >= label.length) {
                    label = Arrays.copyOf(label, keys.capacity());
                    dlabel = Arrays.copyOf(dlabel, keys.capacity());
                }
            }
            label[s] = nc;
            dlabel[s] = dc + it.getDistance();
            push(nc, k, it.getAdjNode());
        }
    }

    private void push(double c, int key, int node) {
        if (size == hk.length) {
            hk = Arrays.copyOf(hk, 2 * size);
            hv = Arrays.copyOf(hv, 2 * size);
            hn = Arrays.copyOf(hn, 2 * size);
        }
        int i = size++;
        while (i > 0) {
            int p = (i - 1) >> 1;
            if (hk[p] <= c) break;
            hk[i] = hk[p];
            hv[i] = hv[p];
            hn[i] = hn[p];
            i = p;
        }
        hk[i] = c;
        hv[i] = key;
        hn[i] = node;
    }

    private void pop() {
        size--;
        if (size == 0) return;
        double c = hk[size];
        int key = hv[size], node = hn[size];
        int i = 0;
        while (true) {
            int l = 2 * i + 1;
            if (l >= size) break;
            int m = l + 1 < size && hk[l + 1] < hk[l] ? l + 1 : l;
            if (hk[m] >= c) break;
            hk[i] = hk[m];
            hv[i] = hv[m];
            hn[i] = hn[m];
            i = m;
        }
        hk[i] = c;
        hv[i] = key;
        hn[i] = node;
    }
}
