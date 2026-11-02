package com.graphhopper.trailmap.tilerouting.strokes;

import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.trailmap.tilerouting.foundation.SparseKeys;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.util.EdgeIteratorState;

import java.util.*;

/**
 * Edge-based Dijkstra between points on edges with PLAIN via semantics (what GraphHopper does for a via point with
 * no heading and no pass_through): a source may be left in either direction, a target may be reached from either
 * direction, and nothing is charged for turning around at the point. Profile weighting and turn costs as GraphHopper.
 * <p>
 * Per target it records the cost, the direction the path leaves the source and arrives at the target (0 = edge storage
 * direction, 1 = against it), and the requested tiles the path crosses (via {@link EdgeTiles}). A copy of the
 * foundation's {@code EdgeDijkstra} idea plus parents; one instance per thread.
 */
final class PathDijkstra {

    /** A point on an edge: storage fraction f (0 = base node, 1 = adj node, by geometry length). */
    static final class Pt {
        int id;
        int edge;
        double f;
        double lat, lon;
        /** Can the route leave from here and come back home / reach here from home (directed floods). */
        String label = "";

        @Override
        public String toString() {
            return String.format(Locale.ROOT, "pt%d e%d f=%.2f %.5f,%.5f %s", id, edge, f, lat, lon, label);
        }
    }

    /** Targets prepared once, reused by every search over the same point set. */
    static final class Targets {
        final List<Pt> pts;
        final Map<Integer, int[]> byEdge = new HashMap<>();
        /** Number of directed keys per target that must settle before the target is final. */
        final int[] keysNeeded;
        /** Largest directed weight of the target's edge (for the early-final test). */
        final double[] edgeW;

        Targets(List<Pt> pts, Weighting w, BaseGraph g, SparseKeys reachableKeys) {
            this.pts = pts;
            keysNeeded = new int[pts.size()];
            edgeW = new double[pts.size()];
            Map<Integer, List<Integer>> m = new HashMap<>();
            for (int i = 0; i < pts.size(); i++) {
                Pt p = pts.get(i);
                m.computeIfAbsent(p.edge, k -> new ArrayList<>()).add(i);
                for (int d = 0; d < 2; d++) {
                    int key = 2 * p.edge + d;
                    double wk = w.calcEdgeWeight(g.getEdgeIteratorStateForKey(key), false);
                    if (Double.isFinite(wk) && (reachableKeys == null || reachableKeys.contains(key))) {
                        keysNeeded[i]++;
                        edgeW[i] = Math.max(edgeW[i], wk);
                    }
                }
            }
            for (Map.Entry<Integer, List<Integer>> e : m.entrySet())
                byEdge.put(e.getKey(), e.getValue().stream().mapToInt(Integer::intValue).toArray());
        }
    }

    static final class Row {
        double[] cost;
        byte[] arr, dep;
        int[][] tiles;
        int settled;
        boolean budgetHit;
    }

    private final BaseGraph graph;
    private final Weighting weighting;
    private final EdgeExplorer explorer;
    private EdgeTiles edgeTiles;
    /**
     * Search state in pages of {@link PagedWeights#PAGE} edge keys, allocated when a search first labels a key in
     * them and kept for this searcher's later searches (a run stamp marks what the current search labelled). The
     * graph is sorted along a Hilbert curve (GraphHopper's graph.sort), so a search's keys fill few pages: memory
     * follows the region searched, not the graph. The directory is one reference per page.
     */
    private static final class Page {
        final double[] label = new double[PagedWeights.PAGE];
        final int[] stamp = new int[PagedWeights.PAGE], done = new int[PagedWeights.PAGE], parent = new int[PagedWeights.PAGE];
        final byte[] origin = new byte[PagedWeights.PAGE];
    }

    private final Page[] pages;
    private int pagesHeld;
    private int run;
    private double[] hk = new double[1 << 16];
    private int[] hv = new int[1 << 16], hn = new int[1 << 16];
    private int size;
    private int[] tileStamp;
    private int tileRun;

    PathDijkstra(BaseGraph graph, Weighting weighting, EdgeTiles edgeTiles, PagedWeights weightCache) {
        this.graph = graph;
        this.weighting = weighting;
        this.explorer = graph.createEdgeExplorer();
        this.edgeTiles = edgeTiles;
        this.wCache = weightCache;
        this.pages = new Page[PagedWeights.pagesFor(2 * graph.getEdges())];
        tileStamp = new int[Math.max(1, edgeTiles.tileCount())];
    }

    private Page pageFor(int key) {
        Page p = pages[key >>> PagedWeights.SHIFT];
        if (p == null) {
            p = new Page();
            pages[key >>> PagedWeights.SHIFT] = p;
            pagesHeld++;
        }
        return p;
    }

    /**
     * Directed edge weights, computed once per key and shared by the request's searchers (one per candidate, so per
     * request; was a static array over all edge keys). As before, the search that computes a weight uses it at double
     * precision and every later use reads the stored float.
     */
    private final PagedWeights wCache;

    /** Edge-key slots this searcher holds (its pages × page size). */
    long keysHeld() {
        return (long) pagesHeld * PagedWeights.PAGE;
    }

    /** Reuse the (large) label arrays for the next job's tiles. */
    void setEdgeTiles(EdgeTiles et) {
        this.edgeTiles = et;
        this.tileStamp = new int[Math.max(1, et.tileCount())];
        this.tileRun = 0;
    }

    EdgeTiles edgeTiles() {
        return edgeTiles;
    }

    double keyWeight(int key) {
        return weighting.calcEdgeWeight(graph.getEdgeIteratorStateForKey(key), false);
    }

    Row search(Pt src, Targets tg, int maxSettled, boolean withTiles) {
        return search(src, tg, maxSettled, withTiles, Double.POSITIVE_INFINITY);
    }

    /** costBound: stop once the frontier passes this weight; targets not reached by then are left +inf (unknown). */
    Row search(Pt src, Targets tg, int maxSettled, boolean withTiles, double costBound) {
        run++;
        size = 0;
        List<Pt> targets = tg.pts;
        int n = targets.size();
        Row r = new Row();
        r.cost = new double[n];
        r.arr = new byte[n];
        r.dep = new byte[n];
        r.tiles = withTiles ? new int[n][] : null;
        Arrays.fill(r.cost, Double.POSITIVE_INFINITY);
        int[] arrKey = new int[n];
        Arrays.fill(arrKey, -1);
        boolean[] sameEdge = new boolean[n];
        int[] left = tg.keysNeeded.clone();
        int remaining = 0;
        for (int v : left) if (v > 0) remaining++;

        int[] same = tg.byEdge.get(src.edge);
        for (int d = 0; d < 2; d++) {
            int key = 2 * src.edge + d;
            EdgeIteratorState st = graph.getEdgeIteratorStateForKey(key);
            double w = weighting.calcEdgeWeight(st, false);
            if (!Double.isFinite(w)) continue;
            if (same != null)
                for (int t : same) {
                    Pt q = targets.get(t);
                    double c = d == 0 ? (q.f >= src.f ? (q.f - src.f) * w : Double.NaN) : (q.f <= src.f ? (src.f - q.f) * w : Double.NaN);
                    if (!Double.isNaN(c) && c < r.cost[t]) {
                        r.cost[t] = c;
                        r.arr[t] = (byte) d;
                        r.dep[t] = (byte) d;
                        sameEdge[t] = true;
                    }
                }
            double rem = d == 0 ? 1 - src.f : src.f;
            relax(src.edge, st.getAdjNode(), rem * w, -(2 + d), (byte) d);
        }

        while (size > 0) {
            double c = hk[0];
            int key = hv[0], node = hn[0];
            pop();
            Page pk = pages[key >>> PagedWeights.SHIFT];
            int ik = key & PagedWeights.MASK;
            if (pk.stamp[ik] != run || c > pk.label[ik] || pk.done[ik] == run) continue;
            if (c > costBound) {
                r.budgetHit = true;
                break;
            }
            pk.done[ik] = run;
            byte org = pk.origin[ik];
            r.settled++;
            int[] hit = tg.byEdge.get(key >> 1);
            if (hit != null) {
                int d = key & 1;
                double w = keyWeight(key);
                for (int t : hit) {
                    Pt q = targets.get(t);
                    double rem = d == 0 ? 1 - q.f : q.f;
                    double ct = c - rem * w;
                    if (ct < r.cost[t]) {
                        r.cost[t] = ct;
                        r.arr[t] = (byte) d;
                        r.dep[t] = org;
                        arrKey[t] = key;
                        sameEdge[t] = false;
                    }
                    if (left[t] > 0 && --left[t] == 0) remaining--;
                }
                if (remaining == 0) break;
            }
            // a target is final once the frontier is past its best cost + its edge weight (no later arrival is cheaper)
            if ((r.settled & 127) == 0) {
                for (int t = 0; t < n; t++)
                    if (left[t] > 0 && c >= r.cost[t] + tg.edgeW[t]) {
                        left[t] = 0;
                        remaining--;
                    }
                if (remaining == 0) break;
            }
            if (r.settled >= maxSettled) {
                r.budgetHit = true;
                break;
            }
            relax(key >> 1, node, c, key, org);
        }
        if (Double.isFinite(costBound))
            for (int t = 0; t < n; t++)
                if (left[t] > 0 && r.cost[t] + tg.edgeW[t] > costBound) r.cost[t] = Double.POSITIVE_INFINITY; // not final
        if (withTiles)
            for (int t = 0; t < n; t++) {
                if (!Double.isFinite(r.cost[t])) continue;
                r.tiles[t] = pathTiles(src, targets.get(t), sameEdge[t], arrKey[t], r.dep[t]);
            }
        return r;
    }

    /** Requested tiles whose inset the path src → q crosses (partial first and last edges by fraction). */
    private int[] pathTiles(Pt src, Pt q, boolean sameEdge, int arrKey, int dep) {
        tileRun++;
        IntList out = new IntList();
        if (sameEdge) {
            addPartial(src.edge, Math.min(src.f, q.f), Math.max(src.f, q.f), out);
            return out.toArray();
        }
        int d = arrKey & 1;
        addPartial(q.edge, d == 0 ? 0 : q.f, d == 0 ? q.f : 1, out);
        int p = parentOf(arrKey);
        int guard = 0;
        while (p >= 0 && guard++ < 1_000_000) {
            addPartial(p >> 1, 0, 1, out);
            p = parentOf(p);
        }
        addPartial(src.edge, dep == 0 ? src.f : 0, dep == 0 ? 1 : src.f, out);
        return out.toArray();
    }

    private int parentOf(int key) {
        return pages[key >>> PagedWeights.SHIFT].parent[key & PagedWeights.MASK];
    }

    private void addPartial(int edge, double fa, double fb, IntList out) {
        EdgeTiles.Iv[] ivs = edgeTiles.get(edge);
        if (ivs == null) return;
        for (EdgeTiles.Iv iv : ivs) {
            double a = Math.max(iv.f0, fa), b = Math.min(iv.f1, fb);
            if ((b - a) * iv.edgeLen < EdgeTiles.MIN_OVERLAP_M) continue;
            if (tileStamp[iv.tile] == tileRun) continue;
            tileStamp[iv.tile] = tileRun;
            out.add(iv.tile);
        }
    }

    private void relax(int inEdge, int node, double c, int par, byte org) {
        EdgeIterator it = explorer.setBaseNode(node);
        while (it.next()) {
            int k = it.getEdgeKey();
            float[] wp = wCache.page(k);
            int wi = k & PagedWeights.MASK;
            float cw = wp[wi];
            double w;
            if (Float.isNaN(cw)) {
                w = weighting.calcEdgeWeight(it, false);
                wp[wi] = (float) w;
            } else w = cw;
            if (!Double.isFinite(w)) continue;
            double t = weighting.calcTurnWeight(inEdge, node, it.getEdge());
            if (!Double.isFinite(t)) continue;
            double nc = c + t + w;
            Page pg = pages[k >>> PagedWeights.SHIFT];
            int i = k & PagedWeights.MASK;
            if (pg != null && pg.stamp[i] == run && pg.label[i] <= nc) continue;
            if (pg != null && pg.done[i] == run) continue;
            if (pg == null) pg = pageFor(k);
            pg.stamp[i] = run;
            pg.label[i] = nc;
            pg.parent[i] = par;
            pg.origin[i] = org;
            push(nc, k, it.getAdjNode());
        }
    }

    static final class IntList {
        int[] a = new int[8];
        int n;

        void add(int v) {
            if (n == a.length) a = Arrays.copyOf(a, 2 * n);
            a[n++] = v;
        }

        int[] toArray() {
            return Arrays.copyOf(a, n);
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
