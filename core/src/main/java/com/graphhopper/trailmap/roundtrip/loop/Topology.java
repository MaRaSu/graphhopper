package com.graphhopper.trailmap.roundtrip.loop;

import com.graphhopper.GraphHopper;
import com.graphhopper.routing.ev.BooleanEncodedValue;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.ev.Subnetwork;
import com.graphhopper.routing.ev.RoadEnvironment;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.trailmap.shared.GravelScale;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.FetchMode;
import com.graphhopper.util.PointList;

import java.util.*;

/**
 * AutoRoute round 2, candidate c2 (copied from round-1 C): offline topology around one start (prototype, test package only).
 * <ol>
 *   <li>Rideable core: edges within a radius of the start that the exploration weight can ride, without
 *       gravel_scale 4 and ferries (a's builder, copied).</li>
 *   <li>Corridor contraction: an edge whose interior runs within {@code corridorM} of another live edge
 *       (a cycleway beside a road) is dropped from the topology, shortest first, so a road and its
 *       parallel way count as one corridor.</li>
 *   <li>Bridges (Tarjan) and 2-edge-connected blocks on the contracted graph. Corner locations are the
 *       nodes of the largest block: pendant trees, peninsula tips behind a bridge and areas reachable only
 *       over scale 4 are excluded.</li>
 *   <li>Network distance from the start (metres along the exploration-weight shortest-path tree).</li>
 *   <li>Coarse "no-way" mask: grid cells without an allowed node (lakes, roadless forest).</li>
 * </ol>
 */
public class Topology {

    public static final class Params {
        // production: the round-4 area16 settings, fixed
        double corridorM = 20;
        double corridorShare = 0.85;
        double corridorEndSkipM = 25;
        double corridorMinEdgeM = 60;
        double cellM = 1000;
        boolean include4 = true;                    // c2.include4

        @Override
        public String toString() {
            return "corridor=" + corridorM + "m/" + corridorShare + " cell=" + cellM + "m include4=" + include4;
        }
    }

    final Params params = new Params();
    double lat0, lon0, mPerLat, mPerLon;
    int n, m, startLocal;
    int[] eu, ev, globalEdge;
    double[] wF, wB, dist;
    int[] gStart;
    double[] gx, gy;
    int[] adjStart, adjEdge;
    double[] nx, ny;               // node positions (projected)
    boolean[] corridorDup;         // dropped by corridor contraction
    boolean[] hard4;               // gravel_scale 4 (only kept when params.include4)
    boolean[] bridge;
    int[] block;                   // 2-edge-connected block id per node (-1 = none)
    boolean[] allowedNode;
    int[] coreDeg;                 // degree in the contracted main block (junction = 3+)
    BitSet allowedGlobalEdge = new BitSet();
    double[] netDist;              // metres from the start along the weight-shortest-path tree
    Set<Long> occupiedCells = new HashSet<>();
    String summary;

    double x(double lon) { return (lon - lon0) * mPerLon; }
    double y(double lat) { return (lat - lat0) * mPerLat; }
    double lon(double x) { return lon0 + x / mPerLon; }
    double lat(double y) { return lat0 + y / mPerLat; }

    long cell(double x, double y) {
        long cx = (long) Math.floor(x / params.cellM), cy = (long) Math.floor(y / params.cellM);
        return (cx << 32) ^ (cy & 0xffffffffL);
    }

    boolean occupied(double lat, double lon) {
        return occupiedCells.contains(cell(x(lon), y(lat)));
    }

    /** exploreProfile: the profile whose small islands (subnetwork) the start must not snap to, as /route does. */
    static Topology build(GraphHopper hopper, Weighting weighting, String exploreProfile, double lat, double lon, double radiusM) {
        BooleanEncodedValue subnetwork = hopper.getEncodingManager().getBooleanEncodedValue(Subnetwork.key(exploreProfile));
        Topology t = new Topology();
        EnumEncodedValue<GravelScale> gs = hopper.getEncodingManager().getEnumEncodedValue(GravelScale.KEY, GravelScale.class);
        EnumEncodedValue<RoadEnvironment> re = hopper.getEncodingManager().getEnumEncodedValue(RoadEnvironment.KEY, RoadEnvironment.class);
        java.util.function.Predicate<EdgeIteratorState> rideable = e -> {
            GravelScale g = e.get(gs);
            if (g == GravelScale.FOUR || g == GravelScale.FERRY) return false;
            if (e.get(re) == RoadEnvironment.FERRY) return false;
            return Double.isFinite(weighting.calcEdgeWeight(e, false)) || Double.isFinite(weighting.calcEdgeWeight(e, true));
        };
        // c2 i3: keep gravel_scale 4 edges in the graph (flagged hard4) so the waypoint cover sees the shortcuts
        // /convert's exploration re-route can take; the loop search and the topology still exclude them
        java.util.function.Predicate<EdgeIteratorState> traversable = !t.params.include4 ? rideable : e -> {
            if (e.get(gs) == GravelScale.FERRY || e.get(re) == RoadEnvironment.FERRY) return false;
            return Double.isFinite(weighting.calcEdgeWeight(e, false)) || Double.isFinite(weighting.calcEdgeWeight(e, true));
        };
        List<Boolean> hard4 = new ArrayList<>();
        BaseGraph g = hopper.getBaseGraph();
        NodeAccess na = g.getNodeAccess();
        Snap snap = hopper.getLocationIndex().findClosest(lat, lon, e -> rideable.test(e) && !e.get(subnetwork));
        if (!snap.isValid()) return null;
        EdgeIteratorState se = snap.getClosestEdge();
        int sNode = d2(lat, lon, na.getLat(se.getBaseNode()), na.getLon(se.getBaseNode()))
                <= d2(lat, lon, na.getLat(se.getAdjNode()), na.getLon(se.getAdjNode())) ? se.getBaseNode() : se.getAdjNode();
        t.lat0 = lat;
        t.lon0 = lon;
        t.mPerLat = 110_540;
        t.mPerLon = 111_320 * Math.cos(Math.toRadians(lat));

        Map<Integer, Integer> local = new HashMap<>();
        BitSet seenEdge = new BitSet(g.getEdges());
        IntList eu = new IntList(), ev = new IntList(), gStart = new IntList(), ge = new IntList();
        DoubleList wF = new DoubleList(), wB = new DoubleList(), dd = new DoubleList(), gx = new DoubleList(), gy = new DoubleList();
        DoubleList nx = new DoubleList(), ny = new DoubleList();
        ArrayDeque<Integer> queue = new ArrayDeque<>();
        local.put(sNode, 0);
        nx.add(t.x(na.getLon(sNode)));
        ny.add(t.y(na.getLat(sNode)));
        queue.add(sNode);
        EdgeExplorer explorer = g.createEdgeExplorer();
        while (!queue.isEmpty()) {
            int u = queue.poll();
            EdgeIterator it = explorer.setBaseNode(u);
            while (it.next()) {
                int edge = it.getEdge();
                if (seenEdge.get(edge)) continue;
                int v = it.getAdjNode();
                if (v == u) { seenEdge.set(edge); continue; }
                if (Math.sqrt(d2(lat, lon, na.getLat(v), na.getLon(v))) > radiusM) continue;
                seenEdge.set(edge);
                if (!traversable.test(it)) continue;
                hard4.add(it.get(gs) == GravelScale.FOUR);
                Integer lv = local.get(v);
                if (lv == null) {
                    lv = local.size();
                    local.put(v, lv);
                    nx.add(t.x(na.getLon(v)));
                    ny.add(t.y(na.getLat(v)));
                    queue.add(v);
                }
                eu.add(local.get(u));
                ev.add(lv);
                ge.add(edge);
                wF.add(weighting.calcEdgeWeight(it, false));
                wB.add(weighting.calcEdgeWeight(it, true));
                dd.add(it.getDistance());
                gStart.add(gx.size());
                PointList pl = it.fetchWayGeometry(FetchMode.ALL);
                for (int i = 0; i < pl.size(); i++) {
                    gx.add(t.x(pl.getLon(i)));
                    gy.add(t.y(pl.getLat(i)));
                }
            }
        }
        t.n = local.size();
        t.m = eu.size();
        t.startLocal = 0;
        t.eu = eu.toArray();
        t.ev = ev.toArray();
        t.globalEdge = ge.toArray();
        t.wF = wF.toArray();
        t.wB = wB.toArray();
        t.dist = dd.toArray();
        t.hard4 = new boolean[t.m];
        for (int e = 0; e < t.m; e++) t.hard4[e] = hard4.get(e);
        gStart.add(gx.size());
        t.gStart = gStart.toArray();
        t.gx = gx.toArray();
        t.gy = gy.toArray();
        t.nx = nx.toArray();
        t.ny = ny.toArray();
        int[] deg = new int[t.n + 1];
        for (int e = 0; e < t.m; e++) {
            deg[t.eu[e]]++;
            deg[t.ev[e]]++;
        }
        t.adjStart = new int[t.n + 1];
        for (int i = 0; i < t.n; i++) t.adjStart[i + 1] = t.adjStart[i] + deg[i];
        int[] fill = Arrays.copyOf(t.adjStart, t.n);
        t.adjEdge = new int[2 * t.m];
        for (int e = 0; e < t.m; e++) {
            t.adjEdge[fill[t.eu[e]]++] = e;
            t.adjEdge[fill[t.ev[e]]++] = ~e;
        }
        t.contractCorridors();
        t.findBridgesAndBlocks();
        t.networkDistance();
        return t;
    }

    /** Drop edges that run beside another live edge for most of their interior, shortest first. */
    void contractCorridors() {
        corridorDup = new boolean[m];
        // scale-4 edges never count as topology (they stay out of blocks, bridges and the corner mask)
        for (int e = 0; e < m; e++) corridorDup[e] = hard4[e];
        double cs = params.corridorM;
        Map<Long, IntList> grid = new HashMap<>();
        // segment index s = geometry point index of the segment start; its edge is segEdge[s]
        int[] segEdge = new int[gx.length];
        for (int e = 0; e < m; e++)
            for (int i = gStart[e]; i < gStart[e + 1] - 1; i++) {
                segEdge[i] = e;
                double minX = Math.min(gx[i], gx[i + 1]), maxX = Math.max(gx[i], gx[i + 1]);
                double minY = Math.min(gy[i], gy[i + 1]), maxY = Math.max(gy[i], gy[i + 1]);
                for (long cx = (long) Math.floor(minX / cs); cx <= (long) Math.floor(maxX / cs); cx++)
                    for (long cy = (long) Math.floor(minY / cs); cy <= (long) Math.floor(maxY / cs); cy++)
                        grid.computeIfAbsent((cx << 32) ^ (cy & 0xffffffffL), k -> new IntList()).add(i);
            }
        Integer[] order = new Integer[m];
        for (int e = 0; e < m; e++) order[e] = e;
        Arrays.sort(order, Comparator.comparingDouble(e -> dist[e]));
        int dropped = 0;
        double droppedM = 0;
        for (int e : order) {
            if (dist[e] < params.corridorMinEdgeM) continue;
            // sample the interior every 15 m
            int samples = 0, covered = 0;
            double along = 0;
            for (int i = gStart[e]; i < gStart[e + 1] - 1; i++) {
                double sx = gx[i], sy = gy[i], dx = gx[i + 1] - sx, dy = gy[i + 1] - sy;
                double len = Math.sqrt(dx * dx + dy * dy);
                for (double s = (15 - along % 15) % 15; s < len; s += 15) {
                    double pos = along + s;
                    if (pos < params.corridorEndSkipM || pos > dist[e] - params.corridorEndSkipM) continue;
                    double px = sx + dx * s / len, py = sy + dy * s / len;
                    samples++;
                    if (nearOtherLiveEdge(grid, segEdge, e, px, py, cs)) covered++;
                }
                along += len;
            }
            if (samples >= 2 && covered >= params.corridorShare * samples) {
                corridorDup[e] = true;
                dropped++;
                droppedM += dist[e];
            }
        }
        summary = String.format(Locale.ROOT, "nodes=%d edges=%d corridorDropped=%d (%.0f km)", n, m, dropped, droppedM / 1000);
    }

    private boolean nearOtherLiveEdge(Map<Long, IntList> grid, int[] segEdge, int self, double px, double py, double cs) {
        long cx0 = (long) Math.floor(px / cs), cy0 = (long) Math.floor(py / cs);
        for (long cx = cx0 - 1; cx <= cx0 + 1; cx++)
            for (long cy = cy0 - 1; cy <= cy0 + 1; cy++) {
                IntList l = grid.get((cx << 32) ^ (cy & 0xffffffffL));
                if (l == null) continue;
                for (int k = 0; k < l.n; k++) {
                    int i = l.a[k];
                    int e = segEdge[i];
                    if (e == self || corridorDup[e]) continue;
                    if (segDist(gx[i] - px, gy[i] - py, gx[i + 1] - px, gy[i + 1] - py) <= cs) return true;
                }
            }
        return false;
    }

    /** Iterative Tarjan bridge finding on the contracted graph, then 2-edge-connected blocks. */
    void findBridgesAndBlocks() {
        bridge = new boolean[m];
        int[] disc = new int[n], low = new int[n], parentEdge = new int[n], iter = new int[n];
        Arrays.fill(disc, -1);
        int time = 0;
        int[] stack = new int[n];
        for (int r = 0; r < n; r++) {
            if (disc[r] >= 0) continue;
            int sp = 0;
            stack[sp++] = r;
            disc[r] = low[r] = time++;
            parentEdge[r] = -1;
            iter[r] = adjStart[r];
            while (sp > 0) {
                int u = stack[sp - 1];
                if (iter[u] < adjStart[u + 1]) {
                    int a = adjEdge[iter[u]++];
                    int e = a >= 0 ? a : ~a;
                    if (corridorDup[e] || e == parentEdge[u]) continue;
                    int v = a >= 0 ? ev[e] : eu[e];
                    if (disc[v] < 0) {
                        disc[v] = low[v] = time++;
                        parentEdge[v] = e;
                        iter[v] = adjStart[v];
                        stack[sp++] = v;
                    } else {
                        low[u] = Math.min(low[u], disc[v]);
                    }
                } else {
                    sp--;
                    if (sp > 0) {
                        int p = stack[sp - 1];
                        low[p] = Math.min(low[p], low[u]);
                        if (low[u] > disc[p]) bridge[parentEdge[u]] = true;
                    }
                }
            }
        }
        block = new int[n];
        Arrays.fill(block, -1);
        List<Double> blockLen = new ArrayList<>();
        ArrayDeque<Integer> q = new ArrayDeque<>();
        for (int r = 0; r < n; r++) {
            if (block[r] >= 0) continue;
            int id = blockLen.size();
            double len = 0;
            block[r] = id;
            q.add(r);
            while (!q.isEmpty()) {
                int u = q.poll();
                for (int k = adjStart[u]; k < adjStart[u + 1]; k++) {
                    int a = adjEdge[k];
                    int e = a >= 0 ? a : ~a;
                    if (corridorDup[e] || bridge[e]) continue;
                    if (a >= 0) len += dist[e]; // count each edge once (from its u side)
                    int v = a >= 0 ? ev[e] : eu[e];
                    if (block[v] < 0) {
                        block[v] = id;
                        q.add(v);
                    }
                }
            }
            blockLen.add(len);
        }
        int main = 0;
        for (int i = 1; i < blockLen.size(); i++) if (blockLen.get(i) > blockLen.get(main)) main = i;
        allowedNode = new boolean[n];
        int allowed = 0;
        for (int v = 0; v < n; v++)
            if (block[v] == main) {
                allowedNode[v] = true;
                allowed++;
                occupiedCells.add(cell(nx[v], ny[v]));
            }
        int bridges = 0;
        coreDeg = new int[n];
        for (int e = 0; e < m; e++) {
            if (bridge[e]) bridges++;
            if (!bridge[e] && !corridorDup[e] && allowedNode[eu[e]] && allowedNode[ev[e]]) {
                coreDeg[eu[e]]++;
                coreDeg[ev[e]]++;
            }
            if (!bridge[e] && allowedNode[eu[e]] && allowedNode[ev[e]]) allowedGlobalEdge.set(globalEdge[e]);
        }
        summary += String.format(Locale.ROOT, " bridges=%d blocks=%d mainBlock=%.0fkm allowedNodes=%d (%.0f%%) cells=%d",
                bridges, blockLen.size(), blockLen.get(main) / 1000, allowed, 100.0 * allowed / n, occupiedCells.size());
    }

    void networkDistance() {
        double[] best = new double[n];
        netDist = new double[n];
        Arrays.fill(best, Double.POSITIVE_INFINITY);
        Arrays.fill(netDist, Double.POSITIVE_INFINITY);
        best[startLocal] = 0;
        netDist[startLocal] = 0;
        PriorityQueue<double[]> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> a[0]));
        pq.add(new double[]{0, startLocal});
        while (!pq.isEmpty()) {
            double[] top = pq.poll();
            int u = (int) top[1];
            if (top[0] > best[u]) continue;
            for (int k = adjStart[u]; k < adjStart[u + 1]; k++) {
                int a = adjEdge[k];
                int e = a >= 0 ? a : ~a;
                double w = a >= 0 ? wF[e] : wB[e];
                if (!Double.isFinite(w) || hard4[e]) continue;
                int v = a >= 0 ? ev[e] : eu[e];
                double nd = top[0] + w;
                if (nd < best[v]) {
                    best[v] = nd;
                    netDist[v] = netDist[u] + dist[e];
                    pq.add(new double[]{nd, v});
                }
            }
        }
    }

    /** Median network / straight-line ratio of allowed nodes whose straight distance is in [lo, hi]. */
    double detourRatio(double lo, double hi) {
        List<Double> r = new ArrayList<>();
        for (int v = 0; v < n; v += 7) {
            if (!allowedNode[v] || !Double.isFinite(netDist[v])) continue;
            double s = Math.hypot(nx[v], ny[v]);
            if (s >= lo && s <= hi) r.add(netDist[v] / s);
        }
        if (r.isEmpty()) return 1.3;
        Collections.sort(r);
        return r.get(r.size() / 2);
    }

    /** The allowed node nearest to (px, py) whose network distance is within target*(1 +- band), or -1. */
    int nearestInBand(double px, double py, double target, double band, double maxOffM, int minDeg) {
        int best = -1;
        double bestD = maxOffM * maxOffM;
        for (int v = 0; v < n; v++) {
            if (!allowedNode[v] || coreDeg[v] < minDeg) continue;
            double nd = netDist[v];
            if (!(nd >= target * (1 - band) && nd <= target * (1 + band))) continue;
            double dx = nx[v] - px, dy = ny[v] - py, d = dx * dx + dy * dy;
            if (d < bestD) {
                bestD = d;
                best = v;
            }
        }
        return best;
    }

    static double segDist(double ax, double ay, double bx, double by) {
        double dx = bx - ax, dy = by - ay;
        double len2 = dx * dx + dy * dy;
        double t = len2 == 0 ? 0 : Math.max(0, Math.min(1, -(ax * dx + ay * dy) / len2));
        double px = ax + t * dx, py = ay + t * dy;
        return Math.sqrt(px * px + py * py);
    }

    static double d2(double lat1, double lon1, double lat2, double lon2) {
        double dy = (lat2 - lat1) * 110_540, dx = (lon2 - lon1) * 111_320 * Math.cos(Math.toRadians(lat1));
        return dx * dx + dy * dy;
    }

    static final class IntList {
        int[] a = new int[16];
        int n;
        void add(int v) { if (n == a.length) a = Arrays.copyOf(a, n * 2); a[n++] = v; }
        int size() { return n; }
        int[] toArray() { return Arrays.copyOf(a, n); }
    }

    static final class DoubleList {
        double[] a = new double[1024];
        int n;
        void add(double v) { if (n == a.length) a = Arrays.copyOf(a, n * 2); a[n++] = v; }
        int size() { return n; }
        double[] toArray() { return Arrays.copyOf(a, n); }
    }
}
