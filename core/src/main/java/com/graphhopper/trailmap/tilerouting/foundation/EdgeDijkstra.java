package com.graphhopper.trailmap.tilerouting.foundation;

import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;

import java.util.*;

/**
 * Bounded one-to-many, edge-based search on the base graph with the profile's weighting and turn costs
 * (GraphHopper's DijkstraOneToMany is node-based, so it cannot price turns). A state is a directed edge key; its
 * label is the cost of arriving at the end of that edge. Sources and targets are {@link Portal}s (points part-way
 * along an edge): leaving a source costs the rest of its edge, arriving at a target costs the part up to its point.
 * <p>
 * Optionally goal-directed (A*): the heuristic is the straight-line distance to the bounding box of the targets times
 * the weighting's minimum weight per metre, which is consistent, so a settled label is final and costs stay exact.
 * The search stops when every target is settled or the settle budget runs out.
 * <p>
 * Labels live in {@link SparseKeys} slots, cleared per search, so memory follows what a search touches, not the graph
 * size; one instance serves many searches. The path to a target ({@link #pathKeys}) is valid until the next search.
 * Not thread safe.
 */
public final class EdgeDijkstra {

    public static final class Result {
        /** Cost per target, +inf if not reached. */
        public double[] cost;
        /** Index of the source the best path to each target starts from (-1 if not reached). */
        public int[] origin;
        /** Edge key at which each target was settled; -1 when reached along its own source edge (or not reached). */
        public int[] viaKey;
        public int settled;
        /** The settle budget ran out before every target was reached: unreached targets are unresolved, not unreachable. */
        public boolean budgetHit;
    }

    private final Weighting weighting;
    private final NodeAccess na;
    private final EdgeExplorer explorer;
    private final SparseKeys keys = new SparseKeys();
    private double[] label;
    private boolean[] closed;
    private int[] parent, originOf;

    private double[] hk = new double[1 << 16];
    private int[] hv = new int[1 << 16], hn = new int[1 << 16];
    private int size;
    private final Set<Integer> sourceEdges = new HashSet<>();
    /**
     * Extra cost for riding back over a source edge. 0 for plain via points (the contract since c1-i2); 300 mirrors
     * GraphHopper's default heading_penalty on the unfavoured incoming edge of a pass_through via.
     */
    public double reentryPenalty = 0;

    // A* target box (degrees) and weight per metre; aStar=false gives plain Dijkstra
    private boolean aStar;
    private double bMinLat, bMaxLat, bMinLon, bMaxLon, mPerLon, minWpm;

    public EdgeDijkstra(BaseGraph graph, Weighting weighting) {
        this.weighting = weighting;
        this.na = graph.getNodeAccess();
        this.explorer = graph.createEdgeExplorer();
        sizeSlots();
    }

    private void sizeSlots() {
        int n = keys.capacity();
        label = label == null ? new double[n] : Arrays.copyOf(label, n);
        closed = closed == null ? new boolean[n] : Arrays.copyOf(closed, n);
        parent = parent == null ? new int[n] : Arrays.copyOf(parent, n);
        originOf = originOf == null ? new int[n] : Arrays.copyOf(originOf, n);
    }

    /** Most edge keys held at once by one search of this instance. */
    public int peakKeys() {
        return keys.peak();
    }

    /** Plain Dijkstra: costs from any of {@code sources} (each with an optional extra start cost) to every target. */
    public Result search(List<Portal> sources, double[] startCost, List<Portal> targets, int maxSettled) {
        return search(sources, startCost, targets, maxSettled, false);
    }

    public Result search(List<Portal> sources, double[] startCost, List<Portal> targets, int maxSettled, boolean goalDirected) {
        return search(sources, startCost, targets, null, maxSettled, goalDirected);
    }

    /**
     * As above; targets with the same {@code group} id (e.g. the two directions of one via point) count as done when
     * the first of them settles. The others keep the cost found so far (+inf if not reached), which is never below
     * the settled one's when they end at the same box, so a min over the group stays exact.
     */
    public Result search(List<Portal> sources, double[] startCost, List<Portal> targets, int[] group, int maxSettled,
                         boolean goalDirected) {
        keys.clear();
        size = 0;
        sourceEdges.clear();
        setupHeuristic(goalDirected, targets);
        Result res = new Result();
        res.cost = new double[targets.size()];
        res.origin = new int[targets.size()];
        res.viaKey = new int[targets.size()];
        Arrays.fill(res.cost, Double.POSITIVE_INFINITY);
        Arrays.fill(res.origin, -1);
        Arrays.fill(res.viaKey, -1);
        Map<Integer, List<Integer>> byKey = new HashMap<>();
        for (int i = 0; i < targets.size(); i++)
            byKey.computeIfAbsent(targets.get(i).key, k -> new ArrayList<>()).add(i);

        for (int s = 0; s < sources.size(); s++) {
            Portal p = sources.get(s);
            double c0 = startCost == null ? 0 : startCost[s];
            if (!Double.isFinite(c0)) continue;
            // a target further along the same directed edge is reached without leaving it
            List<Integer> same = byKey.get(p.key);
            if (same != null)
                for (int t : same) {
                    Portal q = targets.get(t);
                    double c = c0 + (q.frac - p.frac) * p.edgeWeight;
                    if (q.frac >= p.frac && c < res.cost[t]) {
                        res.cost[t] = c;
                        res.origin[t] = s;
                        res.viaKey[t] = -1;
                    }
                }
            // leave the source edge at its end node; its own key gets a label only if the search comes round again
            if (!"home".equals(p.tileKey)) sourceEdges.add(p.edge); // the start has no incoming edge
            relaxFrom(p.edge, p.toNode, c0 + (1 - p.frac) * p.edgeWeight, -1 - s, s);
        }

        // groups: done when every member is settled, or when the pop level f has passed best + maxRest (an unsettled
        // member ends at a node with h = 0, so its label is >= f and its cost >= f - its rest of edge >= best)
        int[] gid = new int[targets.size()];
        Map<Integer, Integer> gIndex = new HashMap<>();
        for (int t = 0; t < targets.size(); t++)
            gid[t] = gIndex.computeIfAbsent(group == null ? t : group[t], x -> gIndex.size());
        int nG = gIndex.size();
        double[] gBest = new double[nG], gRest = new double[nG];
        int[] gLeft = new int[nG];
        boolean[] gDone = new boolean[nG];
        Arrays.fill(gBest, Double.POSITIVE_INFINITY);
        for (int t = 0; t < targets.size(); t++) {
            Portal q = targets.get(t);
            gRest[gid[t]] = Math.max(gRest[gid[t]], (1 - q.frac) * q.edgeWeight);
            gLeft[gid[t]]++;
            gBest[gid[t]] = Math.min(gBest[gid[t]], res.cost[t]);
        }
        int remaining = nG;
        double nextCheck = Double.POSITIVE_INFINITY;
        for (int g = 0; g < nG; g++) nextCheck = Math.min(nextCheck, gBest[g] + gRest[g]);
        while (size > 0 && remaining > 0) {
            double f = hk[0];
            int key = hv[0], node = hn[0];
            pop();
            int ks = keys.slot(key);
            if (ks < 0 || closed[ks]) continue;
            closed[ks] = true;
            double c = label[ks];
            int org = originOf[ks];
            res.settled++;
            List<Integer> hit = byKey.get(key);
            if (hit != null) {
                for (int t : hit) {
                    Portal q = targets.get(t);
                    double ct = c - (1 - q.frac) * q.edgeWeight;
                    if (ct < res.cost[t]) {
                        res.cost[t] = ct;
                        res.origin[t] = org;
                        res.viaKey[t] = key;
                    }
                    int g = gid[t];
                    gBest[g] = Math.min(gBest[g], res.cost[t]);
                    nextCheck = Math.min(nextCheck, gBest[g] + gRest[g]);
                    if (--gLeft[g] == 0 && !gDone[g]) {
                        gDone[g] = true;
                        remaining--;
                    }
                }
            }
            if (f >= nextCheck) {
                nextCheck = Double.POSITIVE_INFINITY;
                for (int g = 0; g < nG; g++) {
                    if (gDone[g]) continue;
                    if (f >= gBest[g] + gRest[g]) {
                        gDone[g] = true;
                        remaining--;
                    } else nextCheck = Math.min(nextCheck, gBest[g] + gRest[g]);
                }
            }
            if (remaining == 0) break;
            if (res.settled >= maxSettled) {
                res.budgetHit = true;
                break;
            }
            relaxFrom(key >> 1, node, c, key, org);
        }
        return res;
    }

    /**
     * Edge keys of the best path to target {@code t} of the last search, source edge excluded, target edge included
     * (empty when the target lies further along the source edge).
     */
    public List<Integer> pathKeys(Result res, int t) {
        LinkedList<Integer> out = new LinkedList<>();
        int k = res.viaKey[t];
        while (k >= 0) {
            out.addFirst(k);
            k = parent[this.keys.slot(k)];
        }
        return out;
    }

    private void relaxFrom(int inEdge, int node, double c, int fromKey, int origin) {
        EdgeIterator it = explorer.setBaseNode(node);
        while (it.next()) {
            double w = weighting.calcEdgeWeight(it, false);
            if (!Double.isFinite(w)) continue;
            double t = weighting.calcTurnWeight(inEdge, node, it.getEdge());
            if (!Double.isFinite(t)) continue;
            double nc = c + t + w;
            if (reentryPenalty > 0 && sourceEdges.contains(it.getEdge())) nc += reentryPenalty;
            int k = it.getEdgeKey();
            int s = keys.slot(k);
            if (s >= 0 && (closed[s] || label[s] <= nc)) continue;
            if (s < 0) {
                s = keys.insert(k);
                if (s >= label.length) sizeSlots();
                closed[s] = false;
            }
            label[s] = nc;
            parent[s] = fromKey;
            originOf[s] = origin;
            int adj = it.getAdjNode();
            push(nc + h(adj), k, adj);
        }
    }

    private void setupHeuristic(boolean on, List<Portal> targets) {
        aStar = on && !targets.isEmpty();
        if (!aStar) return;
        bMinLat = bMinLon = Double.MAX_VALUE;
        bMaxLat = bMaxLon = -Double.MAX_VALUE;
        for (Portal q : targets) {
            // the whole target edge's end nodes, so the bound also holds where the label is taken (its end node)
            for (int nd : new int[]{q.fromNode, q.toNode}) {
                bMinLat = Math.min(bMinLat, na.getLat(nd));
                bMaxLat = Math.max(bMaxLat, na.getLat(nd));
                bMinLon = Math.min(bMinLon, na.getLon(nd));
                bMaxLon = Math.max(bMaxLon, na.getLon(nd));
            }
            bMinLat = Math.min(bMinLat, q.lat);
            bMaxLat = Math.max(bMaxLat, q.lat);
            bMinLon = Math.min(bMinLon, q.lon);
            bMaxLon = Math.max(bMaxLon, q.lon);
        }
        // a little under the true metres per degree so the straight-line bound never overestimates
        mPerLon = 0.99 * TileMath.M_PER_DEG_LON_EQ * Math.cos(Math.toRadians(bMaxLat + 0.5));
        minWpm = Double.isNaN(minWeightPerM) ? weighting.calcMinWeightPerDistance() : minWeightPerM;
    }

    /**
     * Lower bound of weight per metre for the A* heuristic; NaN = the weighting's own bound. A tighter bound measured
     * over the graph's edges keeps the search exact (turn costs are never negative) and settles far fewer states.
     */
    public double minWeightPerM = Double.NaN;

    private double h(int node) {
        if (!aStar) return 0;
        double lat = na.getLat(node), lon = na.getLon(node);
        double dy = lat < bMinLat ? bMinLat - lat : lat > bMaxLat ? lat - bMaxLat : 0;
        double dx = lon < bMinLon ? bMinLon - lon : lon > bMaxLon ? lon - bMaxLon : 0;
        double my = dy * 0.99 * TileMath.M_PER_DEG_LAT, mx = dx * mPerLon;
        return Math.sqrt(mx * mx + my * my) * minWpm;
    }

    // ------------------------------------------------------------------ binary heap, lazy deletion

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
