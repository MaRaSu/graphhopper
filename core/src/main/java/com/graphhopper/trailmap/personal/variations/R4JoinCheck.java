/*
 * Trailmap - Personal routing, "bit of new" variations
 */
package com.graphhopper.trailmap.personal.variations;

import com.carrotsearch.hppc.IntArrayList;
import com.carrotsearch.hppc.IntHashSet;
import com.carrotsearch.hppc.IntObjectHashMap;
import com.carrotsearch.hppc.LongHashSet;
import com.carrotsearch.hppc.LongObjectHashMap;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.ev.RoadClass;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.trailmap.personal.variations.R4Guard.PtIndex;
import com.graphhopper.util.*;

import java.util.*;

/**
 * Round-4 graph checks used by the round-4 generator, copied from the prototype's {@code R4Checks}: check 1
 * (awkward join, with the continuation it found, for the generator's one repair) and check 2's probe (the
 * missed-option detours, the generator's fallback source). Reporting-only parts of the prototype (traces, nearby
 * street names, summaries) are left out. One instance per request (it keeps per-leg state).
 *
 * <p><b>Check 1, awkward join.</b> A graph deviation = a maximal run of the alternative's edges off main's edges,
 * between two main nodes (first return), longer than 60 m and farther than 20 m from main somewhere. In its last
 * {@code scanM} metres, at every junction where the alternative turns by at least {@code sharpDeg} off the way it is
 * on: the continuation = the other outgoing edge with the smallest turn among those on the same way (same name or
 * road class, or running alongside it), at most {@code contTurnDeg}. Following that way must reach a main node,
 * downstream of the deviation's exit, within {@code contFactor} × the alternative's own distance + {@code contSlackM}.
 * The exit end is checked the same way on the reversed path.
 *
 * <p><b>Check 2's probe.</b> Per departure (a main node every {@code depStepM}) one edge-based Dijkstra with the
 * history-free profile weight that leaves main and records the cheapest first-return path per rejoin node that has
 * been at least {@code farM} from main, within the span, cost and extra-length bounds.
 */
public final class R4JoinCheck {

    // check 1 levers (round-4 i3)
    final double scanM = 600;
    final double sharpDeg = 60;
    final double contTurnDeg = 45;
    final double contFactor = 1.5;
    final double contSlackM = 200;
    final double minAltM = 100;
    final double junctionOffM = 20;
    final double alongM = 15;      // within this many metres of the current way ...
    final double alongFrac = 0.6;  // ... along at least this share ...
    final double alongLen = 100;   // ... of the next this many metres
    // check 2 (probe) levers (round-4 i3)
    final double depStepM = 350;
    final double spanMin = 0.10, spanMax = 0.40;
    final double farM = 150;
    final double costRatio = 1.2;
    final double extraMax = 0.15;
    final double maxLenRatio = 2.0; // the guard's "no big detour": length <= 2 x replaced span
    final int probeSettled = 10000;
    final double pruneSlack = 300;
    // snapping
    final double snapTol = 8;

    final BaseGraph g;
    final NodeAccess na;
    final LocationIndex locationIndex;
    final Weighting profileW;
    final EnumEncodedValue<RoadClass> rcEnc;

    public R4JoinCheck(BaseGraph g, EncodingManager em, LocationIndex locationIndex, Weighting profileW) {
        this.g = g;
        this.na = g.getNodeAccess();
        this.locationIndex = locationIndex;
        this.profileW = profileW;
        this.rcEnc = em.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
    }

    // ================================================================== leg context (main route)

    double kx, ky;

    /** A path on the graph: nodes[0..n], edges[0..n-1] (edge t from nodes[t] to nodes[t+1]), cum[t] = metres at nodes[t]. */
    public static final class GPath {
        public int[] nodes, edges;
        public double[] cum;
        public double polyLen, coverage; // coverage = share of the polyline the snapped path reaches

        public double length() {
            return cum[cum.length - 1];
        }
    }

    public final class Leg {
        public final GPath main;
        public final double L;
        public final IntHashSet mainEdges = new IntHashSet();
        public final IntObjectHashMap<double[]> mainAlong = new IntObjectHashMap<>(); // node -> {along, index}
        public final double[] pwCum; // profile weight along main path, per node
        final double[][] mainP;
        final LongObjectHashMap<IntArrayList> coarse = new LongObjectHashMap<>();
        final IntObjectHashMap<double[]> distCache = new IntObjectHashMap<>();
        final double[] mainPAlong;
        public final double corridor;

        /** with the main route's graph path already known (the generator's own main route); null = snap the polyline. */
        public Leg(List<double[]> mainCoords, GPath given) {
            kx = 111320 * Math.cos(mainCoords.get(0)[1] * Math.PI / 180);
            ky = 110540;
            main = given != null ? given : snap(mainCoords);
            L = main.length();
            corridor = Math.max(500, 0.15 * L);
            for (int e : main.edges) mainEdges.add(e);
            for (int t = 0; t < main.nodes.length; t++)
                if (!mainAlong.containsKey(main.nodes[t])) mainAlong.put(main.nodes[t], new double[]{main.cum[t], t});
            pwCum = new double[main.nodes.length];
            for (int t = 0; t < main.edges.length; t++) {
                EdgeIteratorState s = g.getEdgeIteratorState(main.edges[t], main.nodes[t + 1]);
                double w = profileW.calcEdgeWeight(s, false);
                pwCum[t + 1] = pwCum[t] + (Double.isFinite(w) ? w : 0);
            }
            mainP = resample(mainCoords, 5);
            mainPAlong = new double[mainP.length];
            for (int i = 1; i < mainP.length; i++)
                mainPAlong[i] = mainPAlong[i - 1] + Math.hypot(mainP[i][0] - mainP[i - 1][0], mainP[i][1] - mainP[i - 1][1]);
            for (int i = 0; i < mainP.length; i++) {
                long k = PtIndex.key((long) Math.floor(mainP[i][0] / 100), (long) Math.floor(mainP[i][1] / 100));
                IntArrayList l = coarse.get(k);
                if (l == null) coarse.put(k, l = new IntArrayList());
                l.add(i);
            }
        }

        /** {distance to main polyline, along of the nearest main sample} for a projected point (ring search, 100 m cells). */
        double[] distXY(double x, double y) {
            long gx = (long) Math.floor(x / 100), gy = (long) Math.floor(y / 100);
            double best = Double.POSITIVE_INFINITY;
            int at = -1;
            for (int r = 0; r < 60; r++) {
                for (long dx = -r; dx <= r; dx++)
                    for (long dy = -r; dy <= r; dy++) {
                        if (Math.max(Math.abs(dx), Math.abs(dy)) != r) continue;
                        IntArrayList l = coarse.get(PtIndex.key(gx + dx, gy + dy));
                        if (l == null) continue;
                        for (int t = 0; t < l.size(); t++) {
                            double[] m = mainP[l.get(t)];
                            double d = Math.hypot(m[0] - x, m[1] - y);
                            if (d < best) {
                                best = d;
                                at = l.get(t);
                            }
                        }
                    }
                if (best <= r * 100) break;
            }
            return new double[]{best, at < 0 ? -1e9 : mainPAlong[at], at};
        }

        /** {max distance, offset x, offset y, side} of an edge's geometry from main (offset = farthest point minus its nearest main sample). */
        double[] edgeFar(EdgeIteratorState s) {
            PointList pl = s.fetchWayGeometry(FetchMode.ALL);
            double[] r = {0, 0, 0, -1};
            for (int i = 0; i < pl.size(); i++) {
                double x = pl.getLon(i) * kx, y = pl.getLat(i) * ky;
                double[] q = distXY(x, y);
                if (q[0] > r[0] && q[2] >= 0) {
                    int at = (int) q[2];
                    r[0] = q[0];
                    r[1] = x - mainP[at][0];
                    r[2] = y - mainP[at][1];
                    double[] a = mainP[Math.max(0, at - 2)], b = mainP[Math.min(mainP.length - 1, at + 2)];
                    double dx = b[0] - a[0], dy = b[1] - a[1];
                    r[3] = dx * r[2] - dy * r[1] > 0 ? 0 : 1; // 0 = left of main's direction, 1 = right
                }
            }
            return r;
        }

        double[] nodeDist(int node) {
            double[] c = distCache.get(node);
            if (c == null) distCache.put(node, c = distXY(na.getLon(node) * kx, na.getLat(node) * ky));
            return c;
        }
    }

    double[][] resample(List<double[]> coords, double step) {
        List<double[]> out = new ArrayList<>();
        for (int i = 0; i < coords.size() - 1; i++) {
            double ax = coords.get(i)[0] * kx, ay = coords.get(i)[1] * ky, bx = coords.get(i + 1)[0] * kx, by = coords.get(i + 1)[1] * ky;
            int n = (int) Math.max(1, Math.ceil(Math.hypot(bx - ax, by - ay) / step));
            for (int s = 0; s < n; s++) out.add(new double[]{ax + (bx - ax) * s / n, ay + (by - ay) * s / n});
        }
        if (!coords.isEmpty()) {
            double[] c = coords.get(coords.size() - 1);
            out.add(new double[]{c[0] * kx, c[1] * ky});
        }
        return out.toArray(new double[0][]);
    }

    // ================================================================== snapping a polyline onto the graph

    static final class SLabel {
        final int node, edge, along;
        final double cost;
        final SLabel parent;

        SLabel(int node, int edge, int along, double cost, SLabel parent) {
            this.node = node;
            this.edge = edge;
            this.along = along;
            this.cost = cost;
            this.parent = parent;
        }
    }

    /** nearest polyline sample of (x,y) within index window [lo, hi]: {dist, index}. */
    static double[] nearIn(double[][] P, double x, double y, int lo, int hi) {
        lo = Math.max(0, lo);
        hi = Math.min(P.length - 1, hi);
        double best = Double.POSITIVE_INFINITY;
        int at = -1;
        for (int i = lo; i <= hi; i++) {
            double d = Math.hypot(P[i][0] - x, P[i][1] - y);
            if (d < best) {
                best = d;
                at = i;
            }
        }
        return new double[]{best, at};
    }

    static long skey(int node, int along) {
        return ((long) node << 12) | (along / 20);
    }

    /** Only used when the main route has a point that is not a tower node between two of its edges (rare). */
    public GPath snap(List<double[]> coords) {
        double[][] P = resample(coords, 5);
        int n = P.length;
        GPath out = new GPath();
        double polyLen = 0;
        for (int i = 1; i < n; i++) polyLen += Math.hypot(P[i][0] - P[i - 1][0], P[i][1] - P[i - 1][1]);
        out.polyLen = polyLen;
        PriorityQueue<SLabel> pq = new PriorityQueue<>(Comparator.comparingDouble(l -> l.cost));
        double[] c0 = coords.get(0);
        Snap sn = locationIndex.findClosest(c0[1], c0[0], EdgeFilter.ALL_EDGES);
        if (!sn.isValid()) throw new IllegalStateException("no start snap");
        EdgeIteratorState se = sn.getClosestEdge();
        for (int u : new int[]{se.getBaseNode(), se.getAdjNode()}) {
            double x = na.getLon(u) * kx, y = na.getLat(u) * ky;
            int win = (int) Math.ceil(se.getDistance() / 5) + 4;
            double[] q = nearIn(P, x, y, 0, win);
            pq.add(new SLabel(u, -1, (int) q[1], q[0], null));
        }
        // state = (node, phase along the polyline in 100 m buckets): a polyline may pass the same node twice
        LongHashSet settled = new LongHashSet();
        EdgeExplorer ex = g.createEdgeExplorer();
        SLabel bestEnd = null;
        int guardCount = 0;
        while (!pq.isEmpty()) {
            SLabel cur = pq.poll();
            if (!settled.add(skey(cur.node, cur.along))) continue;
            if (++guardCount > 200000) break;
            if (bestEnd == null || cur.along > bestEnd.along) bestEnd = cur;
            if (cur.along >= n - 1) continue; // terminal
            EdgeIterator it = ex.setBaseNode(cur.node);
            while (it.next()) {
                int w = it.getAdjNode();
                if (w == cur.node) continue;
                PointList pl = it.fetchWayGeometry(FetchMode.ALL);
                double len = it.getDistance();
                int hi = cur.along + (int) Math.ceil(len / 5 * 1.6) + 12;
                int lo = cur.along - 6;
                // sample the edge geometry every <= 5 m, each sample must be within snapTol of the polyline, moving forward
                double sumD = 0;
                int cnt = 0, prevIdx = cur.along;
                boolean ok = true;
                for (int p = 0; p < pl.size() - 1 && ok; p++) {
                    double ax = pl.getLon(p) * kx, ay = pl.getLat(p) * ky, bx = pl.getLon(p + 1) * kx, by = pl.getLat(p + 1) * ky;
                    int m = (int) Math.max(1, Math.ceil(Math.hypot(bx - ax, by - ay) / 5));
                    for (int s = (p == 0 ? 1 : 0); s <= m; s++) {
                        double x = ax + (bx - ax) * s / m, y = ay + (by - ay) * s / m;
                        double[] q = nearIn(P, x, y, Math.max(lo, prevIdx - 6), hi);
                        if (q[0] > snapTol) {
                            if (prevIdx >= n - 3) { // the polyline ends inside this edge (the leg's end snap): terminal
                                prevIdx = n - 1;
                                p = pl.size();
                                break;
                            }
                            ok = false;
                            break;
                        }
                        prevIdx = Math.max(prevIdx, (int) q[1]);
                        sumD += q[0];
                        cnt++;
                    }
                }
                if (!ok) continue;
                if (prevIdx <= cur.along && len > 5) continue; // no progress along the polyline
                if (settled.contains(skey(w, prevIdx))) continue;
                double cost = cur.cost + len + 2 * sumD / Math.max(1, cnt);
                pq.add(new SLabel(w, it.getEdge(), prevIdx, cost, cur));
            }
        }
        List<SLabel> chain = new ArrayList<>();
        for (SLabel l = bestEnd; l != null; l = l.parent) chain.add(l);
        Collections.reverse(chain);
        int k = chain.size();
        out.nodes = new int[k];
        out.edges = new int[Math.max(0, k - 1)];
        out.cum = new double[k];
        for (int t = 0; t < k; t++) {
            out.nodes[t] = chain.get(t).node;
            if (t > 0) {
                out.edges[t - 1] = chain.get(t).edge;
                out.cum[t] = out.cum[t - 1] + g.getEdgeIteratorState(chain.get(t).edge, Integer.MIN_VALUE).getDistance();
            }
        }
        out.coverage = bestEnd == null ? 0 : bestEnd.along / (double) Math.max(1, n - 1);
        return out;
    }

    // ================================================================== geometry helpers

    /** edge state oriented from -> to (adj = to). */
    EdgeIteratorState oriented(int edge, int to) {
        return g.getEdgeIteratorState(edge, to);
    }

    /** bearing (radians, planar) of the last ~15 m of an edge arriving at its adj node. */
    double bearingIn(EdgeIteratorState s) {
        PointList pl = s.fetchWayGeometry(FetchMode.ALL);
        int last = pl.size() - 1;
        double ex = pl.getLon(last) * kx, ey = pl.getLat(last) * ky;
        double x = ex, y = ey;
        for (int i = last - 1; i >= 0; i--) {
            x = pl.getLon(i) * kx;
            y = pl.getLat(i) * ky;
            if (Math.hypot(ex - x, ey - y) >= 15) break;
        }
        return Math.atan2(ex - x, ey - y);
    }

    /** bearing of the first ~15 m of an edge leaving its base node. */
    double bearingOut(EdgeIteratorState s) {
        PointList pl = s.fetchWayGeometry(FetchMode.ALL);
        double sx = pl.getLon(0) * kx, sy = pl.getLat(0) * ky;
        double x = sx, y = sy;
        for (int i = 1; i < pl.size(); i++) {
            x = pl.getLon(i) * kx;
            y = pl.getLat(i) * ky;
            if (Math.hypot(x - sx, y - sy) >= 15) break;
        }
        return Math.atan2(x - sx, y - sy);
    }

    static double turnDeg(double bIn, double bOut) {
        double d = Math.toDegrees(bOut - bIn);
        while (d > 180) d -= 360;
        while (d < -180) d += 360;
        return Math.abs(d);
    }

    String name(EdgeIteratorState s) {
        String n = s.getName();
        return n == null ? "" : n;
    }

    RoadClass rc(EdgeIteratorState s) {
        return s.get(rcEnc);
    }

    boolean sameWay(EdgeIteratorState way, EdgeIteratorState c) {
        String a = name(way), b = name(c);
        if (!a.isEmpty() && a.equals(b)) return true;
        return rc(way) == rc(c);
    }

    // ================================================================== graph deviations of an alternative

    public static final class GDev {
        public int s, e;       // node indices in the alt path: s = exit main node, e = first-return main node
        public int side = -1;  // side of its farthest point: 0 left of main's direction, 1 right
        public double len, maxD, exitAlong, rejoinAlong;
    }

    public List<GDev> deviations(Leg leg, GPath alt) {
        List<GDev> out = new ArrayList<>();
        int m = alt.edges.length;
        int t = 0;
        while (t < m) {
            if (leg.mainEdges.contains(alt.edges[t]) || !leg.mainAlong.containsKey(alt.nodes[t])) {
                t++;
                continue;
            }
            int s = t, e = -1;
            for (int u = t + 1; u <= m; u++)
                if (leg.mainAlong.containsKey(alt.nodes[u])) {
                    e = u;
                    break;
                }
            if (e < 0) break;
            GDev d = new GDev();
            d.s = s;
            d.e = e;
            d.len = alt.cum[e] - alt.cum[s];
            for (int u = s; u < e; u++) {
                double[] fr = leg.edgeFar(oriented(alt.edges[u], alt.nodes[u + 1]));
                if (fr[0] > d.maxD) {
                    d.maxD = fr[0];
                    d.side = (int) fr[3];
                }
            }
            d.exitAlong = leg.mainAlong.get(alt.nodes[s])[0];
            d.rejoinAlong = leg.mainAlong.get(alt.nodes[e])[0];
            if (d.len >= 60 && d.maxD > 20) out.add(d);
            t = e;
        }
        return out;
    }

    // ================================================================== check 1: awkward join

    public static final class Flag {
        public String end;
        public int junction, mainIdx; // junction index in the walking-order path; main path index the continuation reaches
        public int[] contKeys;        // continuation edge keys in walking order (for repair)
    }

    public List<Flag> awkwardJoins(Leg leg, GPath alt) {
        List<Flag> flags = new ArrayList<>();
        for (GDev d : deviations(leg, alt)) {
            // rejoin end: forward on the alt path
            scanEnd(leg, alt.nodes, alt.edges, alt.cum, d.s, d.e, d, false, flags);
            // exit end: the reversed path, its deviation runs from index (n-1-e) to (n-1-s)
            int n = alt.nodes.length;
            int[] rn = new int[n], re = new int[n - 1];
            double[] rc = new double[n];
            double tot = alt.cum[n - 1];
            for (int t = 0; t < n; t++) {
                rn[t] = alt.nodes[n - 1 - t];
                rc[t] = tot - alt.cum[n - 1 - t];
            }
            for (int t = 0; t < n - 1; t++) re[t] = alt.edges[n - 2 - t];
            scanEnd(leg, rn, re, rc, n - 1 - d.e, n - 1 - d.s, d, true, flags);
        }
        return flags;
    }

    /** junctions in the last scanM of nodes[s..e] (e = main node), walking direction = array order. */
    void scanEnd(Leg leg, int[] nodes, int[] edges, double[] cum, int s, int e, GDev d, boolean reversed, List<Flag> flags) {
        EdgeExplorer ex = g.createEdgeExplorer();
        for (int t = s + 1; t < e; t++) {
            double remain = cum[e] - cum[t];
            if (remain > scanM || remain < minAltM) continue; // closer than minAltM it is the join itself
            int v = nodes[t];
            EdgeIteratorState ein = oriented(edges[t - 1], v);
            EdgeIteratorState eout = oriented(edges[t], nodes[t + 1]);
            if (ein.getBaseNode() == ein.getAdjNode() || eout.getBaseNode() == eout.getAdjNode()) continue; // loop edges
            double bIn = bearingIn(ein);
            double turn = turnDeg(bIn, bearingOut(eout));
            if (turn < sharpDeg) continue;
            // turning "off the way it is on" = onto an edge that is not the same way (same name or class) by the same
            // definition as the continuation; a sharp turn that stays on the way (a street corner, a cycleway grid) is not it
            if (sameWay(ein, eout)) continue;
            if (leg.nodeDist(v)[0] <= junctionOffM) continue; // still within the guard's 20 m of main: not on the deviation proper
            double limit = contFactor * remain + contSlackM;
            PtIndex W = wayIndex(v, ein, limit + alongLen);
            // the alternative's next alongLen metres running alongside the current way: it stays on the way
            List<double[]> ahead = new ArrayList<>();
            for (int u = t; u < edges.length && ahead.size() * 5 < alongLen; u++)
                addSamples(oriented(edges[u], nodes[u + 1]), ahead, alongLen - ahead.size() * 5);
            if (alongside(ahead, W)) continue;
            // continuation: the other outgoing edge on the way (or alongside it) with the smallest turn
            EdgeIterator it = ex.setBaseNode(v);
            int degree = 0;
            EdgeIteratorState cont = null;
            double contTurn = 1e9;
            while (it.next()) {
                degree++;
                int ed = it.getEdge();
                if (ed == edges[t - 1] || ed == edges[t] || it.getAdjNode() == v) continue;
                if (!rideable(it, reversed)) continue;
                if (!onWay(ein, it, W)) continue;
                double ct = turnDeg(bIn, bearingOut(it));
                if (ct < contTurn) {
                    contTurn = ct;
                    cont = it.detach(false);
                }
            }
            if (degree < 3 || cont == null || contTurn > contTurnDeg) continue;
            double[] hit = followWay(leg, ein, cont, v, limit, reversed, d, W);
            if (hit == null) continue;
            Flag f = new Flag();
            f.junction = t;
            f.mainIdx = (int) hit[2];
            f.contKeys = lastFollowKeys;
            f.end = reversed ? "exit" : "rejoin";
            flags.add(f);
        }
    }

    /** rideable in the rider's direction: walking order, or against it when the path is reversed. */
    boolean rideable(EdgeIteratorState s, boolean reversed) {
        return Double.isFinite(profileW.calcEdgeWeight(s, reversed));
    }

    static final class WLabel {
        final int edge, adj;
        final double dist, bearing;
        final EdgeIteratorState state;
        final WLabel parent;

        WLabel(EdgeIteratorState state, double dist, double bearing, WLabel parent) {
            this.parent = parent;
            this.state = state;
            this.edge = state.getEdge();
            this.adj = state.getAdjNode();
            this.dist = dist;
            this.bearing = bearing;
        }
    }

    /**
     * Follow the way (same name or class as {@code way}) from {@code first} until a main node on the deviation's
     * replaced side (forward: along >= exit; reversed: along <= rejoin). Returns {distance, main along, main index} or null.
     */
    double[] followWay(Leg leg, EdgeIteratorState way, EdgeIteratorState first, int v, double limit, boolean reversed, GDev d, PtIndex W) {
        lastFollowKeys = null;
        PriorityQueue<WLabel> pq = new PriorityQueue<>(Comparator.comparingDouble(l -> l.dist));
        pq.add(new WLabel(first, first.getDistance(), bearingIn(first), null));
        IntHashSet settled = new IntHashSet();
        EdgeExplorer ex = g.createEdgeExplorer();
        while (!pq.isEmpty()) {
            WLabel cur = pq.poll();
            if (cur.dist > limit) return null;
            if (!settled.add(cur.state.getEdgeKey())) continue;
            int a = cur.adj;
            if (a == v) continue;
            double[] ma = leg.mainAlong.get(a);
            if (ma != null) {
                boolean ok = reversed ? ma[0] <= d.rejoinAlong : ma[0] >= d.exitAlong;
                if (ok) {
                    IntArrayList ks = new IntArrayList();
                    for (WLabel l = cur; l != null; l = l.parent) ks.add(l.state.getEdgeKey());
                    int[] arr = new int[ks.size()];
                    for (int x = 0; x < arr.length; x++) arr[x] = ks.get(arr.length - 1 - x);
                    lastFollowKeys = arr;
                    return new double[]{cur.dist, ma[0], ma[1]};
                }
                continue; // touching main elsewhere ends this branch
            }
            EdgeIterator it = ex.setBaseNode(a);
            while (it.next()) {
                if (it.getEdge() == cur.edge || it.getAdjNode() == a) continue;
                if (!rideable(it, reversed) || !onWay(way, it, W)) continue;
                if (turnDeg(cur.bearing, bearingOut(it)) >= sharpDeg) continue;
                double nd = cur.dist + it.getDistance();
                if (nd > limit) continue;
                EdgeIteratorState st = it.detach(false);
                pq.add(new WLabel(st, nd, bearingIn(st), cur));
            }
        }
        return null;
    }

    /** edge keys (walking order) of the last successful {@link #followWay}. */
    int[] lastFollowKeys;

    // ------------------------------------------------------------------ "alongside": a way a few metres beside the current way is the same place

    /** samples (projected, every 5 m) of the geometry of the current way: edges of the same way reachable from v within radius. */
    PtIndex wayIndex(int v, EdgeIteratorState way, double radius) {
        List<double[]> pts = new ArrayList<>();
        addSamples(way, pts, Double.MAX_VALUE);
        PriorityQueue<double[]> pq = new PriorityQueue<>(Comparator.comparingDouble(x -> x[1]));
        pq.add(new double[]{v, 0});
        IntHashSet seen = new IntHashSet(), edges = new IntHashSet();
        edges.add(way.getEdge());
        EdgeExplorer ex = g.createEdgeExplorer();
        while (!pq.isEmpty()) {
            double[] cur = pq.poll();
            int u = (int) cur[0];
            if (!seen.add(u)) continue;
            EdgeIterator it = ex.setBaseNode(u);
            while (it.next()) {
                if (!sameWay(way, it) || !edges.add(it.getEdge())) continue;
                addSamples(it, pts, Double.MAX_VALUE);
                double nd = cur[1] + it.getDistance();
                if (nd <= radius) pq.add(new double[]{it.getAdjNode(), nd});
            }
        }
        return new PtIndex(pts.toArray(new double[0][]), alongM);
    }

    void addSamples(EdgeIteratorState s, List<double[]> out, double maxM) {
        PointList pl = s.fetchWayGeometry(FetchMode.ALL);
        double run = 0;
        for (int p = 0; p < pl.size() - 1; p++) {
            double ax = pl.getLon(p) * kx, ay = pl.getLat(p) * ky, bx = pl.getLon(p + 1) * kx, by = pl.getLat(p + 1) * ky;
            double sl = Math.hypot(bx - ax, by - ay);
            int m = (int) Math.max(1, Math.ceil(sl / 5));
            for (int t = 0; t < m; t++) {
                if (run + sl * t / m > maxM) return;
                out.add(new double[]{ax + (bx - ax) * t / m, ay + (by - ay) * t / m});
            }
            run += sl;
        }
        if (run <= maxM) out.add(new double[]{pl.getLon(pl.size() - 1) * kx, pl.getLat(pl.size() - 1) * ky});
    }

    boolean alongside(List<double[]> samples, PtIndex W) {
        if (samples.isEmpty()) return false;
        int c = 0;
        for (double[] p : samples) if (W.near(p)[0] <= alongM) c++;
        return c >= alongFrac * samples.size();
    }

    /** an edge (oriented from the current node) is on the way: same name or class, or running alongside it. */
    boolean onWay(EdgeIteratorState way, EdgeIteratorState c, PtIndex W) {
        if (sameWay(way, c)) return true;
        List<double[]> s = new ArrayList<>();
        addSamples(c, s, alongLen);
        return alongside(s, W);
    }

    // ================================================================== check 2's probe: missed easy options

    public static final class Detour {
        public int i, j;               // main path node indices
        public double len, span, extra, ratio, maxD, a0, a1;
        public int[] edges;
        public int side = -1;
    }

    static final class PLabel {
        final int key, edge, adj, far;
        final double w, ew, dist;
        final PLabel parent;

        PLabel(int key, int edge, int adj, int far, double w, double ew, double dist, PLabel parent) {
            this.key = key;
            this.edge = edge;
            this.adj = adj;
            this.far = far;
            this.w = w;
            this.ew = ew;
            this.dist = dist;
            this.parent = parent;
        }
    }

    public List<Detour> probe(Leg leg) {
        List<Detour> out = new ArrayList<>();
        GPath M = leg.main;
        double L = leg.L;
        double next = 0;
        EdgeExplorer ex = g.createEdgeExplorer();
        for (int k = 0; k < M.nodes.length - 1; k++) {
            if (M.cum[k] < next) continue;
            if (M.cum[k] > (1 - spanMin) * L) break;
            next = M.cum[k] + depStepM;
            out.addAll(probeFrom(leg, k, ex));
        }
        return out;
    }

    List<Detour> probeFrom(Leg leg, int k, EdgeExplorer ex) {
        GPath M = leg.main;
        double L = leg.L;
        int dep = M.nodes[k];
        double a0 = M.cum[k];
        double maxLen = spanMax * L + extraMax * L;
        PriorityQueue<PLabel> pq = new PriorityQueue<>(Comparator.comparingDouble(l -> l.w));
        Map<Integer, PLabel> arrival = new HashMap<>();
        IntHashSet settled = new IntHashSet();
        int inEdge = k > 0 ? M.edges[k - 1] : -1;
        EdgeIterator it = ex.setBaseNode(dep);
        PLabel root = new PLabel(-1, inEdge, dep, 0, 0, 0, 0, null);
        while (it.next()) {
            if (leg.mainEdges.contains(it.getEdge()) || it.getAdjNode() == dep) continue;
            relaxP(leg, root, it, k, a0, maxLen, pq, arrival, settled);
        }
        int count = 0;
        while (!pq.isEmpty()) {
            PLabel cur = pq.poll();
            if (!settled.add(cur.key * 2 + cur.far)) continue;
            if (++count > probeSettled) break;
            EdgeIterator e2 = ex.setBaseNode(cur.adj);
            while (e2.next()) {
                if (e2.getEdge() == cur.edge || leg.mainEdges.contains(e2.getEdge())) continue;
                relaxP(leg, cur, e2, k, a0, maxLen, pq, arrival, settled);
            }
        }
        List<Detour> res = new ArrayList<>();
        for (Map.Entry<Integer, PLabel> en : arrival.entrySet()) {
            PLabel last = en.getValue();
            int j = en.getKey();
            Detour dt = new Detour();
            dt.i = k;
            dt.j = j;
            dt.a0 = a0;
            dt.a1 = M.cum[j];
            dt.span = dt.a1 - dt.a0;
            dt.len = last.dist;
            dt.extra = dt.len - dt.span;
            double repW = leg.pwCum[j] - leg.pwCum[k];
            dt.ratio = (last.ew / Math.max(1, dt.len)) / Math.max(1e-9, repW / Math.max(1, dt.span));
            if (dt.extra > extraMax * L || dt.ratio > costRatio) continue;
            IntArrayList es = new IntArrayList();
            for (PLabel l = last; l != null && l.key >= 0; l = l.parent) es.add(l.key);
            int[] ek = new int[es.size()];
            for (int t = 0; t < ek.length; t++) ek[t] = es.get(ek.length - 1 - t);
            IntHashSet seen = new IntHashSet();
            boolean rep = false;
            double maxD = 0;
            for (int key : ek) {
                EdgeIteratorState s = g.getEdgeIteratorStateForKey(key);
                if (!seen.add(s.getEdge())) rep = true;
                double[] fr = leg.edgeFar(s);
                if (fr[0] > maxD) {
                    maxD = fr[0];
                    dt.side = (int) fr[3];
                }
            }
            if (rep || maxD < farM || maxD > leg.corridor || dt.len > maxLenRatio * dt.span) continue;
            dt.maxD = maxD;
            dt.edges = new int[ek.length];
            for (int t = 0; t < ek.length; t++) dt.edges[t] = ek[t] >> 1;
            res.add(dt);
        }
        return res;
    }

    void relaxP(Leg leg, PLabel cur, EdgeIteratorState it, int k, double a0, double maxLen, PriorityQueue<PLabel> pq,
                Map<Integer, PLabel> arrival, IntHashSet settled) {
        double L = leg.L;
        double d = cur.dist + it.getDistance();
        if (d > maxLen) return;
        double w = profileW.calcEdgeWeight(it, false);
        if (!Double.isFinite(w)) return;
        double tw = cur.edge < 0 ? 0 : profileW.calcTurnWeight(cur.edge, cur.adj, it.getEdge());
        if (!Double.isFinite(tw)) return;
        int key = it.getEdgeKey();
        int a = it.getAdjNode();
        double[] ma = leg.mainAlong.get(a);
        if (ma != null) { // first return
            int j = (int) ma[1];
            double span = ma[0] - a0;
            int far = cur.far; // the built detour's geometry decides (a long last edge may itself go far)
            if (j <= k || span < spanMin * L || span > spanMax * L) return;
            PLabel nl = new PLabel(key, it.getEdge(), a, far, cur.w + w + tw, cur.ew + w, d, cur);
            // prefer far labels; among the same kind the cheapest
            PLabel old = arrival.get(j);
            if (old == null || (nl.far > old.far) || (nl.far == old.far && nl.w < old.w)) arrival.put(j, nl);
            return;
        }
        double[] nd = leg.nodeDist(a);
        if (nd[0] > leg.corridor) return;
        if (d + nd[0] > maxLen) return;
        double x = nd[1];
        if (x < a0 - pruneSlack || x > a0 + spanMax * L + pruneSlack) return;
        if (d - (x - a0) > extraMax * L + pruneSlack) return;
        int far = cur.far == 1 || nd[0] >= farM ? 1 : 0;
        if (settled.contains(key * 2 + far)) return;
        pq.add(new PLabel(key, it.getEdge(), a, far, cur.w + w + tw, cur.ew + w, d, cur));
    }
}
