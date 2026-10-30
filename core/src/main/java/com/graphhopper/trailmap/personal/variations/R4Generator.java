/*
 * Trailmap - Personal routing, "bit of new" variations
 */
package com.graphhopper.trailmap.personal.variations;

import com.carrotsearch.hppc.IntArrayList;
import com.carrotsearch.hppc.IntHashSet;
import com.carrotsearch.hppc.IntObjectHashMap;
import com.carrotsearch.hppc.LongDoubleHashMap;
import com.carrotsearch.hppc.LongObjectHashMap;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.trailmap.personal.PersonalFamiliarity;
import com.graphhopper.trailmap.personal.variations.R4Guard.PtIndex;
import com.graphhopper.util.*;

import java.util.*;

/**
 * Round-4 generator ("round4" alternatives), a faithful copy of the round-4 i3 prototype
 * ({@code R4GenTest}, algo/personal_routing/rounds/4), test scaffolding removed, i3's settings as defaults.
 *
 * <p>One inventory of detours from two families, both with round-3 bounds and the guard port as filters:
 * <ul>
 * <li><b>B</b>: first-return ears from 4 departure windows, one multi-seed search per window, one label tree per side.</li>
 * <li><b>A</b>: round-3 A's banded search without the faces: one-to-many Dijkstra per start and side that must stay
 * at least {@code band} from main except in an exit zone near the start and a short entry run before the rejoin;
 * the best few rejoins per search by a cheap proxy.</li>
 * </ul>
 * Every candidate is checked for an awkward join ({@link R4JoinCheck} check 1); a flagged one is repaired once along
 * the continuation the check found, else dropped. Selection: the joint pair assembly, with a bonus when the two
 * alternatives are led by different families; one retry search when there is no second alternative; then the
 * missed-option probe's detours as a fallback source (family P).
 *
 * <p>Detour flavour ({@code none | avoid | explore}): the searches use {@code earW} (the profile, or the profile with
 * an avoid/explore personal weighting), and the candidate value adds the flavour term.
 *
 * <p>One instance per request.
 */
public final class R4Generator {

    // ---- family B (round-3 B i2 settings)
    final int nWin = 4;
    final double winSpan = 0.72;
    final double minProg = 0.11;
    final double maxProg = 0.34;
    final double maxEarLen = 0.36;
    final double earExtra = 0.12;
    final double rho = 1.35;
    final int maxSettled = 20000;
    final int guardPerSide = 15;
    final double gapMin = 250;
    final double pruneSlack = 250;
    final double dupFrac = 0.6;
    final int perWinSide = 4;
    final int maxEars = 40;
    final int retry = 1;
    // ---- family A (round-3 A settings, banded search; round-4 i3 start spacing and settle cap)
    final boolean useA = true;
    final double aStep = 0.12;           // start spacing along main, fraction of L
    final double aStartMax = 0.68;
    final double aMinSpan = 0.12, aMaxSpan = 0.32;
    final double band = 150, zone = 300, alongMargin = 250;
    final double corrFrac = 0.8;
    final double aMaxDist = 0.42;
    final double aRho = 1.3;
    final double aExtra = 0.18;
    final int aSettled = 6000;
    final int aPerSearch = 4;
    final double aProxyGap = 0.03;
    // ---- merge
    final int joinCheck = 2;   // 0 off, 1 drop flagged, 2 repair once then drop
    final double famBonus = 0.3;
    final int probeMode = 2;   // missed-option probe detours as a third source (family P): 0 off, 1 always, 2 only when < 2 alternatives
    final String flavour;

    final BaseGraph g;
    final NodeAccess na;
    final Weighting profileW, earW;
    final PersonalFamiliarity life, last12;
    final R4JoinCheck ck;

    MainPath b;
    double L, corridor;
    double[] pwCum;
    R4Guard guard;
    R4JoinCheck.Leg cleg;
    static final double CELL = 100;
    double kx, ky;
    LongObjectHashMap<IntArrayList> pGrid;
    double[] pX, pY, pAlong;
    LongDoubleHashMap cellLb, cellAlong;
    double winLo, winHi;

    /**
     * @param earW   weighting of the detour searches: the profile weighting, or for avoid/explore the profile wrapped in a
     *               personal weighting (mode avoid with the lifetime history, explore with the last-12-month history,
     *               strength 1, no switch penalty)
     * @param life   lifetime history (habit), null for flavour none
     * @param last12 last-12-month history (novelty), null for flavour none
     */
    public R4Generator(BaseGraph g, EncodingManager em, LocationIndex locationIndex, Weighting profileW, Weighting earW,
                       PersonalFamiliarity life, PersonalFamiliarity last12, String flavour) {
        this.g = g;
        this.na = g.getNodeAccess();
        this.profileW = profileW;
        this.earW = earW;
        this.life = life;
        this.last12 = last12;
        this.flavour = flavour;
        this.ck = new R4JoinCheck(g, em, locationIndex, profileW);
    }

    static final class Label {
        final int key, edge, adj, dep, side;
        final double w, dist, run;
        final Label parent;

        Label(int key, int edge, int adj, double w, double dist, Label parent, int dep, int side, double run) {
            this.key = key;
            this.edge = edge;
            this.adj = adj;
            this.w = w;
            this.dist = dist;
            this.parent = parent;
            this.dep = dep;
            this.side = side;
            this.run = run;
        }
    }

    /** One detour: main node i to main node j over edge keys. */
    public static final class Ear {
        int i, j, side, win;
        public String fam;
        int[] keys;
        List<double[]> geom;
        public double len, span, extra, quality, maxD, value, novel, habitEar, habitMain;
        public double a0, a1;
        double[][] devPts;
        double[] bbox;
        int id;
        public boolean repaired;
    }

    static final class Stats {
        int searches, settled, arrivals, cheap, guardChecked, good, joinFlagged, joinRepaired, joinDropped;
        int[] goodPerWin;
        Map<String, Integer> famGood = new TreeMap<>();
        String retryKind = "-";
        double msB, msA, msJoin, msPick;
    }

    // ================================================================ preparation

    void prepare() {
        L = b.distance;
        corridor = Math.max(500, 0.15 * L);
        int n = b.size();
        pwCum = new double[n];
        for (int x = 1; x < n; x++) {
            double w = 0;
            if (b.node[x] >= 0) {
                EdgeIteratorState e = g.getEdgeIteratorState(b.outEdge[x - 1], b.node[x]);
                if (e != null) w = profileW.calcEdgeWeight(e, false);
            }
            pwCum[x] = pwCum[x - 1] + (Double.isFinite(w) ? w : 0);
        }
        guard = new R4Guard(b.coords);
        kx = 111320 * Math.cos(b.coords.get(0)[1] * Math.PI / 180);
        ky = 110540;
        List<double[]> pts = new ArrayList<>();
        for (int i = 0; i < b.coords.size() - 1; i++) {
            double[] a = b.coords.get(i), c = b.coords.get(i + 1);
            double d = Math.hypot((c[0] - a[0]) * kx, (c[1] - a[1]) * ky);
            int m = (int) Math.max(1, Math.ceil(d / 25));
            for (int s = 0; s < m; s++) pts.add(new double[]{a[0] + (c[0] - a[0]) * s / m, a[1] + (c[1] - a[1]) * s / m});
        }
        pts.add(b.coords.get(b.coords.size() - 1));
        pX = new double[pts.size()];
        pY = new double[pts.size()];
        pAlong = new double[pts.size()];
        pGrid = new LongObjectHashMap<>();
        cellLb = new LongDoubleHashMap();
        cellAlong = new LongDoubleHashMap();
        for (int i = 0; i < pts.size(); i++) {
            pX[i] = pts.get(i)[0] * kx;
            pY[i] = pts.get(i)[1] * ky;
            if (i > 0) pAlong[i] = pAlong[i - 1] + Math.hypot(pX[i] - pX[i - 1], pY[i] - pY[i - 1]);
            long key = PtIndex.key((long) Math.floor(pX[i] / CELL), (long) Math.floor(pY[i] / CELL));
            IntArrayList l = pGrid.get(key);
            if (l == null) pGrid.put(key, l = new IntArrayList());
            l.add(i);
        }
        nodeInfoCache.clear();
        // check-1 context on the generator's own main path
        boolean ok = true;
        for (int k = 0; k < n; k++) if (b.node[k] < 0) ok = false;
        R4JoinCheck.GPath gp = null;
        if (ok && n >= 2) {
            gp = new R4JoinCheck.GPath();
            gp.nodes = b.node.clone();
            gp.edges = new int[n - 1];
            gp.cum = new double[n];
            for (int k = 0; k < n - 1; k++) gp.edges[k] = b.outEdge[k];
            for (int k = 0; k < n; k++) gp.cum[k] = b.along[k];
            gp.polyLen = L;
            gp.coverage = 1;
        }
        cleg = ck.new Leg(b.coords, gp);
    }

    double lastAlong;

    double distLb(int node) {
        double x = na.getLon(node) * kx, y = na.getLat(node) * ky;
        long gx = (long) Math.floor(x / CELL), gy = (long) Math.floor(y / CELL);
        long key = PtIndex.key(gx, gy);
        if (cellLb.containsKey(key)) {
            lastAlong = cellAlong.get(key);
            return cellLb.get(key);
        }
        double cx = (gx + 0.5) * CELL, cy = (gy + 0.5) * CELL, best = Double.POSITIVE_INFINITY;
        int bestI = -1;
        int maxRing = (int) Math.ceil(corridor / CELL) + 2;
        for (int r = 0; r <= maxRing; r++) {
            if (best < (r - 1) * CELL) break;
            for (long dx = -r; dx <= r; dx++)
                for (long dy = -r; dy <= r; dy++) {
                    if (Math.max(Math.abs(dx), Math.abs(dy)) != r) continue;
                    IntArrayList l = pGrid.get(PtIndex.key(gx + dx, gy + dy));
                    if (l == null) continue;
                    for (int t = 0; t < l.size(); t++) {
                        int i = l.get(t);
                        double dd = Math.hypot(pX[i] - cx, pY[i] - cy);
                        if (dd < best) {
                            best = dd;
                            bestI = i;
                        }
                    }
                }
        }
        double lb = Math.max(0, best - CELL * Math.sqrt(0.5) - 13);
        cellLb.put(key, lb);
        lastAlong = bestI < 0 ? -1e9 : pAlong[bestI];
        cellAlong.put(key, lastAlong);
        return lb;
    }

    /** {distance to main (25 m samples), along, side 0 left / 1 right} of a node, exact ring search. */
    final IntObjectHashMap<double[]> nodeInfoCache = new IntObjectHashMap<>();

    double[] nodeInfo(int node) {
        double[] c = nodeInfoCache.get(node);
        if (c != null) return c;
        double x = na.getLon(node) * kx, y = na.getLat(node) * ky;
        long gx = (long) Math.floor(x / CELL), gy = (long) Math.floor(y / CELL);
        double best = Double.POSITIVE_INFINITY;
        int bi = -1;
        int maxRing = (int) Math.ceil(corridor / CELL) + 3;
        for (int r = 0; r <= maxRing; r++) {
            for (long dx = -r; dx <= r; dx++)
                for (long dy = -r; dy <= r; dy++) {
                    if (Math.max(Math.abs(dx), Math.abs(dy)) != r) continue;
                    IntArrayList l = pGrid.get(PtIndex.key(gx + dx, gy + dy));
                    if (l == null) continue;
                    for (int t = 0; t < l.size(); t++) {
                        int i = l.get(t);
                        double dd = Math.hypot(pX[i] - x, pY[i] - y);
                        if (dd < best) {
                            best = dd;
                            bi = i;
                        }
                    }
                }
            if (best <= r * CELL) break;
        }
        int side = 0;
        double al = -1e9;
        if (bi >= 0) {
            int i0 = Math.max(0, bi - 1), i1 = Math.min(pX.length - 1, bi + 1);
            double mx = pX[i1] - pX[i0], my = pY[i1] - pY[i0];
            side = mx * (y - pY[bi]) - my * (x - pX[bi]) > 0 ? 0 : 1;
            al = pAlong[bi];
        }
        c = new double[]{best, al, side};
        nodeInfoCache.put(node, c);
        return c;
    }

    int sideOf(int k, EdgeIteratorState e) {
        int pi = b.ptIdx[k];
        int pj = Math.min(b.coords.size() - 1, pi + 1);
        double[] a = b.coords.get(pi), c = b.coords.get(pj);
        double mx = (c[0] - a[0]) * kx, my = (c[1] - a[1]) * ky;
        PointList pl = e.fetchWayGeometry(FetchMode.ALL);
        double ex = (pl.getLon(1) - pl.getLon(0)) * kx, ey = (pl.getLat(1) - pl.getLat(0)) * ky;
        return mx * ey - my * ex > 0 ? 0 : 1;
    }

    // ================================================================ family B: first-return ears

    int rejoinIndex(int dep, int v) {
        IntArrayList l = b.occ.get(v);
        if (l == null) return -1;
        for (int t = 0; t < l.size(); t++) {
            int j = l.get(t);
            double p = b.along[j] - b.along[dep];
            if (j > dep && p >= minProg * L && p <= maxProg * L) return j;
        }
        return -1;
    }

    List<Ear> searchB(double lo, double hi, int win, IntHashSet forbidden, Stats st) {
        st.searches++;
        winLo = lo;
        winHi = hi;
        EdgeExplorer ex = g.createEdgeExplorer();
        PriorityQueue<Label> pq = new PriorityQueue<>(Comparator.comparingDouble(l -> l.w));
        IntHashSet settled = new IntHashSet();
        Map<Integer, Label> arrival = new HashMap<>();
        double maxDist = maxEarLen * L;
        int k0 = -1;
        for (int k = 0; k < b.size(); k++) {
            if (b.along[k] < lo || b.along[k] > hi || b.node[k] < 0) continue;
            if (k0 < 0) k0 = k;
            double seed = pwCum[k] - pwCum[k0];
            Label root = new Label(-1, b.inEdge[k], b.node[k], seed, 0, null, k, -1, 0);
            EdgeIterator it = ex.setBaseNode(b.node[k]);
            while (it.next()) {
                int e = it.getEdge();
                if (b.edgeSet.contains(e) || (forbidden != null && forbidden.contains(e))) continue;
                relaxB(root, it, sideOf(k, it), k, arrival, pq, settled, maxDist);
            }
        }
        int count = 0;
        while (!pq.isEmpty()) {
            Label cur = pq.poll();
            if (!settled.add(cur.key * 2 + cur.side)) continue;
            if (++count > maxSettled) break;
            EdgeIterator it = ex.setBaseNode(cur.adj);
            while (it.next()) {
                int e = it.getEdge();
                if (e == cur.edge || b.edgeSet.contains(e) || (forbidden != null && forbidden.contains(e))) continue;
                relaxB(cur, it, cur.side, cur.dep, arrival, pq, settled, maxDist);
            }
        }
        st.settled += count;
        st.arrivals += arrival.size();
        List<Ear> cheap = new ArrayList<>();
        for (Map.Entry<Integer, Label> en : arrival.entrySet()) {
            Ear ear = build(keysOf(en.getValue()), en.getValue().dep, en.getKey() >> 1, en.getValue().side, win, "B", st);
            if (ear != null) cheap.add(ear);
        }
        return filterGuard(cheap, st);
    }

    void relaxB(Label cur, EdgeIteratorState it, int side, int dep, Map<Integer, Label> arrival, PriorityQueue<Label> pq,
                IntHashSet settled, double maxDist) {
        int e = it.getEdge();
        double d = cur.dist + it.getDistance();
        if (d > maxDist) return;
        double w = earW.calcEdgeWeight(it, false);
        if (!Double.isFinite(w)) return;
        double tw = profileW.calcTurnWeight(cur.edge, cur.adj, e);
        if (!Double.isFinite(tw)) return;
        int key = it.getEdgeKey();
        if (settled.contains(key * 2 + side)) return;
        int a = it.getAdjNode();
        double nw = cur.w + w + tw;
        if (b.occ.containsKey(a)) {
            int j = rejoinIndex(dep, a);
            if (j < 0) return;
            double rw = profileW.calcTurnWeight(e, a, b.outEdge[j]);
            if (!Double.isFinite(rw) || e == b.outEdge[j]) return;
            int ak = j * 2 + side;
            Label old = arrival.get(ak);
            double tot = nw + rw + (pwCum[pwCum.length - 1] - pwCum[j]);
            if (old == null || tot < old.w) arrival.put(ak, new Label(key, e, a, tot, d, cur, dep, side, 0));
            return;
        }
        double lb = distLb(a);
        if (lb > corridor) return;
        if (d + lb > maxDist) return;
        double x = lastAlong;
        if (x < winLo - pruneSlack || x > winHi + maxProg * L + pruneSlack) return;
        if (d - (x - b.along[dep]) > earExtra * L + pruneSlack) return;
        pq.add(new Label(key, e, a, nw, d, cur, dep, side, 0));
    }

    // ================================================================ family A: banded other-side search

    List<Ear> searchA(int si, int side, Stats st) {
        st.searches++;
        int s = b.node[si];
        double as = b.along[si], minSpan = aMinSpan * L, maxSpan = aMaxSpan * L;
        double a0 = as - alongMargin, a1 = as + maxSpan + alongMargin, t0 = as + minSpan - zone, t1 = as + maxSpan + zone;
        double reach = corrFrac * corridor, maxDist = aMaxDist * L;
        double sx = na.getLon(s) * kx, sy = na.getLat(s) * ky;
        EdgeExplorer ex = g.createEdgeExplorer();
        PriorityQueue<Label> pq = new PriorityQueue<>(Comparator.comparingDouble(l -> l.w));
        IntHashSet settled = new IntHashSet();
        IntObjectHashMap<Label> arr = new IntObjectHashMap<>();
        Label root = new Label(-1, b.inEdge[si], s, 0, 0, null, si, side, 0);
        Label cur = root;
        int count = 0;
        while (cur != null) {
            EdgeIterator it = ex.setBaseNode(cur.adj);
            while (it.next()) {
                int e = it.getEdge();
                if (e == cur.edge || b.edgeSet.contains(e)) continue;
                int key = it.getEdgeKey();
                if (settled.contains(key)) continue;
                double d = cur.dist + it.getDistance();
                if (d > maxDist) continue;
                double w = earW.calcEdgeWeight(it, false);
                if (!Double.isFinite(w)) continue;
                double tw = profileW.calcTurnWeight(cur.edge, cur.adj, e);
                if (!Double.isFinite(tw)) continue;
                int a = it.getAdjNode();
                double gw = cur.w + w + tw;
                if (b.occ.containsKey(a)) { // first return
                    IntArrayList ks = b.occ.get(a);
                    for (int x = 0; x < ks.size(); x++) {
                        int k = ks.get(x);
                        double span = b.along[k] - as;
                        if (k <= si || span < minSpan || span > maxSpan) continue;
                        double tw2 = profileW.calcTurnWeight(e, a, b.outEdge[k]);
                        if (!Double.isFinite(tw2) || e == b.outEdge[k]) continue;
                        Label old = arr.get(k);
                        if (old == null || gw + tw2 < old.w) arr.put(k, new Label(key, e, a, gw + tw2, d, cur, si, side, 0));
                    }
                    continue;
                }
                double[] inf = nodeInfo(a);
                if (inf[0] > reach) continue;
                double run = 0;
                boolean far = inf[0] >= band && (int) inf[2] == side && inf[1] >= a0 && inf[1] <= a1;
                if (!far) {
                    double dz = Math.hypot(na.getLon(a) * kx - sx, na.getLat(a) * ky - sy);
                    if (dz > zone) { // not in the exit zone: an entry run before a rejoin
                        if (inf[1] < t0 || inf[1] > t1 || !(inf[0] < 25 || (int) inf[2] == side)) continue;
                        run = cur.run + it.getDistance();
                        if (run > zone) continue;
                    }
                }
                double back = Math.hypot(inf[0], Math.max(0, as + minSpan - inf[1]));
                if (d + back > maxDist) continue;
                pq.add(new Label(key, e, a, gw, d, cur, si, side, run));
            }
            cur = null;
            while (!pq.isEmpty()) {
                Label l = pq.poll();
                if (settled.add(l.key)) {
                    cur = l;
                    break;
                }
            }
            if (++count > aSettled) break;
        }
        st.settled += count;
        st.arrivals += arr.size();
        // the best few rejoins by a cheap proxy, spans apart
        List<Object[]> prox = new ArrayList<>();
        for (var en : arr) {
            int j = en.key;
            Label l = en.value;
            double span = b.along[j] - as;
            double repW = pwCum[j] - pwCum[si];
            double ratio = (l.w / Math.max(1, l.dist)) / Math.max(1e-9, repW / Math.max(1, span));
            double p = -Math.max(0, ratio - 1) - 0.4 * Math.max(0, l.dist - span) / Math.max(1, span) - 0.6 * Math.abs(span / L - 0.2);
            prox.add(new Object[]{p, j, l});
        }
        prox.sort((x, y) -> Double.compare((double) y[0], (double) x[0]));
        List<Ear> cheap = new ArrayList<>();
        List<Double> spans = new ArrayList<>();
        for (Object[] o : prox) {
            if (cheap.size() >= aPerSearch) break;
            int j = (int) o[1];
            double span = b.along[j] - as;
            boolean close = false;
            for (double sp : spans) if (Math.abs(sp - span) < aProxyGap * L) close = true;
            if (close) continue;
            Ear ear = build(keysOf((Label) o[2]), si, j, side, -1, "A", st);
            if (ear == null) continue;
            spans.add(span);
            cheap.add(ear);
        }
        return filterGuard(cheap, st);
    }

    // ================================================================ shared candidate checks

    static int[] keysOf(Label last) {
        IntArrayList keys = new IntArrayList();
        for (Label l = last; l != null && l.key >= 0; l = l.parent) keys.add(l.key);
        int[] ek = new int[keys.size()];
        for (int t = 0; t < ek.length; t++) ek[t] = keys.get(ek.length - 1 - t);
        return ek;
    }

    /** cheap edge checks with the family's round-3 bounds (B: progress, extra 0.12 L, rho 1.35; A: span, extra 0.18 L, rho 1.3). */
    Ear build(int[] ek, int i, int j, int side, int win, String fam, Stats st) {
        IntHashSet seen = new IntHashSet();
        double len = 0, pw = 0, hEar = 0, nEar = 0;
        for (int key : ek) {
            EdgeIteratorState e = g.getEdgeIteratorStateForKey(key);
            if (!seen.add(e.getEdge())) return null;
            len += e.getDistance();
            pw += profileW.calcEdgeWeight(e, false);
            if (life != null) hEar += e.getDistance() * habit(e);
            if (last12 != null) nEar += e.getDistance() * (last12.share(e) < 0.3 ? 1 : 0);
        }
        if (j <= i) return null;
        Ear ear = new Ear();
        ear.i = i;
        ear.j = j;
        ear.side = side;
        ear.win = win;
        ear.fam = fam;
        ear.keys = ek;
        ear.len = len;
        ear.span = b.along[j] - b.along[i];
        ear.extra = len - ear.span;
        ear.a0 = b.along[i];
        ear.a1 = b.along[j];
        boolean isA = fam.equals("A");
        if (len < 0.1 * L * 0.95 || len > 0.4 * L + (isA ? 2 * zone : 0)) return null;
        if (ear.extra > (isA ? aExtra : earExtra) * L || len > (isA ? 2.0 : 1.9) * ear.span + (isA ? 2 * zone : 0)) return null;
        double repW = pwCum[j] - pwCum[i];
        ear.quality = (pw / Math.max(1, len)) / Math.max(1e-9, repW / Math.max(1, ear.span));
        if (!(ear.quality <= (isA ? aRho : fam.equals("P") ? 1.2 : rho))) return null;
        ear.habitEar = hEar / Math.max(1, len);
        ear.novel = nEar / Math.max(1, len);
        if (life != null) {
            double hm = 0;
            for (int x = i; x < j; x++) {
                EdgeIteratorState e = g.getEdgeIteratorState(b.outEdge[x], Integer.MIN_VALUE);
                hm += e.getDistance() * habit(e);
            }
            ear.habitMain = hm / Math.max(1, ear.span);
        }
        ear.value = earValue(ear, 300);
        return ear;
    }

    double habit(EdgeIteratorState e) {
        double s = life.share(e);
        if (s <= 0) return 0;
        return s * Math.min(1, life.rides(e) / 10.0);
    }

    double earValue(Ear e, double maxD) {
        double v = 1 + 2 * Math.min(e.span, 0.3 * L) / L + 0.5 * Math.min(maxD, 400) / 400
                - 3 * Math.max(0, e.extra) / L - Math.max(0, e.quality - 1);
        if (flavour.equals("avoid")) v += 1.0 * (e.habitMain - e.habitEar);
        if (flavour.equals("explore")) v += 1.0 * e.novel;
        return v;
    }

    List<double[]> earGeom(Ear ear) {
        List<double[]> geom = new ArrayList<>();
        geom.add(b.coords.get(b.ptIdx[ear.i]));
        for (int key : ear.keys) {
            PointList pl = g.getEdgeIteratorStateForKey(key).fetchWayGeometry(FetchMode.PILLAR_AND_ADJ);
            for (int t = 0; t < pl.size(); t++) geom.add(MainPath.coord(pl, t));
        }
        geom.set(geom.size() - 1, b.coords.get(b.ptIdx[ear.j]));
        return geom;
    }

    /** best-value first, near-copies skipped, at most guardPerSide guard checks per side, then the join check. */
    List<Ear> filterGuard(List<Ear> cheap, Stats st) {
        st.cheap += cheap.size();
        cheap.sort((x, y) -> Double.compare(y.value, x.value));
        List<Ear> out = new ArrayList<>();
        List<IntHashSet> acc = new ArrayList<>();
        int[] checked = new int[2];
        for (Ear ear : cheap) {
            if (checked[ear.side] >= guardPerSide) continue;
            if (isDup(ear, out, acc)) continue;
            checked[ear.side]++;
            st.guardChecked++;
            Ear ok = guardEar(ear, st) ? joinCheck(ear, st) : null;
            if (ok != null) {
                out.add(ok);
                IntHashSet es2 = new IntHashSet();
                for (int key : ok.keys) es2.add(key >> 1);
                acc.add(es2);
            }
        }
        return out;
    }

    boolean isDup(Ear ear, List<Ear> out, List<IntHashSet> acc) {
        if (dupFrac >= 1) return false;
        for (int t = 0; t < out.size(); t++) {
            if (out.get(t).side != ear.side) continue;
            double shared = 0;
            for (int key : ear.keys) if (acc.get(t).contains(key >> 1)) shared += g.getEdgeIteratorStateForKey(key).getDistance();
            if (shared > dupFrac * Math.min(ear.len, out.get(t).len)) return true;
        }
        return false;
    }

    boolean guardEar(Ear ear, Stats st) {
        ear.geom = earGeom(ear);
        List<double[]> piece = new ArrayList<>();
        int pi = b.ptIdx[ear.i], pj = b.ptIdx[ear.j];
        int s0 = pi;
        while (s0 > 0 && b.cumPt[pi] - b.cumPt[s0] < 40) s0--;
        for (int k = s0; k < pi; k++) piece.add(b.coords.get(k));
        piece.addAll(ear.geom);
        int e0 = pj;
        while (e0 < b.coords.size() - 1 && b.cumPt[e0] - b.cumPt[pj] < 40) e0++;
        for (int k = pj + 1; k <= e0; k++) piece.add(b.coords.get(k));
        double[][] Pp = guard.resample(piece);
        List<R4Guard.Dev> devs = guard.deviations(Pp);
        if (devs.size() != 1) return false;
        R4Guard.Dev d = devs.get(0);
        if (!d.good) return false;
        ear.maxD = d.maxD;
        ear.len = d.len;
        ear.span = d.span;
        List<double[]> pts = new ArrayList<>();
        for (int i = d.s; i <= d.e; i++) pts.add(Pp[i]);
        ear.devPts = pts.toArray(new double[0][]);
        ear.bbox = new double[]{Double.MAX_VALUE, Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
        for (double[] p : ear.devPts) {
            ear.bbox[0] = Math.min(ear.bbox[0], p[0]);
            ear.bbox[1] = Math.min(ear.bbox[1], p[1]);
            ear.bbox[2] = Math.max(ear.bbox[2], p[0]);
            ear.bbox[3] = Math.max(ear.bbox[3], p[1]);
        }
        ear.value = earValue(ear, d.maxD);
        st.good++;
        return true;
    }

    /** the ear as a graph path from main node i to main node j (for check 1). */
    R4JoinCheck.GPath earPath(Ear ear) {
        R4JoinCheck.GPath p = new R4JoinCheck.GPath();
        int n = ear.keys.length;
        p.nodes = new int[n + 1];
        p.edges = new int[n];
        p.cum = new double[n + 1];
        p.nodes[0] = b.node[ear.i];
        for (int t = 0; t < n; t++) {
            EdgeIteratorState e = g.getEdgeIteratorStateForKey(ear.keys[t]);
            p.edges[t] = e.getEdge();
            p.nodes[t + 1] = e.getAdjNode();
            p.cum[t + 1] = p.cum[t] + e.getDistance();
        }
        p.polyLen = p.cum[n];
        p.coverage = 1;
        return p;
    }

    /** check 1 on the candidate: keep it, repair it once along the continuation the check found, or drop it. */
    Ear joinCheck(Ear ear, Stats st) {
        if (joinCheck == 0 || b.node[ear.i] < 0 || b.node[ear.j] < 0) return ear;
        long tj = System.nanoTime();
        try {
            return joinCheck0(ear, st);
        } finally {
            st.msJoin += (System.nanoTime() - tj) / 1e6;
        }
    }

    Ear joinCheck0(Ear ear, Stats st) {
        List<R4JoinCheck.Flag> fl = ck.awkwardJoins(cleg, earPath(ear));
        if (fl.isEmpty()) return ear;
        st.joinFlagged++;
        if (joinCheck >= 2 && !ear.repaired) {
            for (R4JoinCheck.Flag f : fl) {
                Ear r = repair(ear, f, st);
                if (r == null) continue;
                st.joinRepaired++;
                return r;
            }
        }
        st.joinDropped++;
        return null;
    }

    Ear repair(Ear ear, R4JoinCheck.Flag f, Stats st) {
        if (f.contKeys == null) return null;
        int n = ear.keys.length;
        int[] nk;
        int ni, nj;
        if (f.end.equals("rejoin")) { // keep the ear up to the junction, then the continuation to main
            int t = f.junction; // node index in the ear path (forward)
            nk = new int[t + f.contKeys.length];
            System.arraycopy(ear.keys, 0, nk, 0, t);
            System.arraycopy(f.contKeys, 0, nk, t, f.contKeys.length);
            ni = ear.i;
            nj = f.mainIdx;
        } else { // exit: the continuation (walked backwards) from main to the junction, then the ear onwards
            int t = n - f.junction; // node index in the forward ear path
            nk = new int[f.contKeys.length + (n - t)];
            for (int x = 0; x < f.contKeys.length; x++) nk[x] = f.contKeys[f.contKeys.length - 1 - x] ^ 1;
            System.arraycopy(ear.keys, t, nk, f.contKeys.length, n - t);
            ni = f.mainIdx;
            nj = ear.j;
        }
        if (nj <= ni) return null;
        Ear r = build(nk, ni, nj, ear.side, ear.win, ear.fam, st);
        if (r == null) return null;
        r.repaired = true;
        if (!guardEar(r, st)) return null;
        List<R4JoinCheck.Flag> again = ck.awkwardJoins(cleg, earPath(r));
        return again.isEmpty() ? r : null;
    }

    // ================================================================ family P: probe detours (fallback)

    List<Ear> probeEars(Stats st) {
        if (cleg == null) return List.of();
        List<Ear> cheap = new ArrayList<>();
        for (R4JoinCheck.Detour d : ck.probe(cleg)) {
            IntArrayList keys = new IntArrayList();
            // rebuild edge keys in walking order from the departure node
            int cur = b.node[d.i];
            boolean ok = true;
            for (int e : d.edges) {
                EdgeIteratorState s = g.getEdgeIteratorState(e, Integer.MIN_VALUE);
                int adj = s.getBaseNode() == cur ? s.getAdjNode() : s.getAdjNode() == cur ? s.getBaseNode() : -1;
                if (adj < 0) {
                    ok = false;
                    break;
                }
                EdgeIteratorState o = g.getEdgeIteratorState(e, adj);
                keys.add(o.getEdgeKey());
                cur = adj;
            }
            if (!ok) continue;
            Ear ear = build(keys.toArray(), d.i, d.j, d.side, -1, "P", st);
            if (ear != null) cheap.add(ear);
        }
        return filterGuard(cheap, st);
    }

    // ================================================================ assembly + selection

    /** One round-4 alternative: the main route with one to three detours spliced in. */
    public static final class Alt {
        public List<Ear> ears;
        double score;
        public List<double[]> coords;
        R4Guard.AltScore gs;
        public String lead;
    }

    List<double[]> splice(List<Ear> ears) {
        List<Ear> s = new ArrayList<>(ears);
        s.sort(Comparator.comparingInt(x -> x.i));
        List<double[]> out = new ArrayList<>();
        int cur = 0;
        for (Ear e : s) {
            int pi = b.ptIdx[e.i], pj = b.ptIdx[e.j];
            for (int k = cur; k <= pi; k++) out.add(b.coords.get(k));
            for (int k = 1; k < e.geom.size() - 1; k++) out.add(e.geom.get(k));
            cur = pj;
        }
        for (int k = cur; k < b.coords.size(); k++) out.add(b.coords.get(k));
        return out;
    }

    double[][] ov;

    List<Alt> assemble(List<Ear> inv) {
        List<Ear> ears = new ArrayList<>(inv);
        ears.sort((x, y) -> Double.compare(y.value, x.value));
        if (perWinSide > 0) { // the best few per (region, side, family); A ears use their start's region
            Map<String, Integer> cnt = new HashMap<>();
            List<Ear> kept = new ArrayList<>();
            for (Ear e : ears) {
                int reg = e.win >= 0 ? e.win : (int) Math.min(nWin, Math.floor(e.a0 / (winSpan * L / nWin)));
                String k = e.fam + reg + ":" + e.side;
                int c = cnt.getOrDefault(k, 0);
                if (c >= perWinSide) continue;
                cnt.put(k, c + 1);
                kept.add(e);
            }
            ears = kept;
        }
        if (ears.size() > maxEars) ears = new ArrayList<>(ears.subList(0, maxEars));
        ears.sort(Comparator.comparingDouble(e -> e.a0));
        int n = ears.size();
        for (int x = 0; x < n; x++) ears.get(x).id = x;
        ov = new double[n][n];
        PtIndex[] idx = new PtIndex[n];
        for (int f = 0; f < n; f++)
            for (int e = 0; e < n; e++) {
                if (e == f) {
                    ov[f][e] = 1;
                    continue;
                }
                Ear F = ears.get(f), E = ears.get(e);
                if (F.bbox[0] > E.bbox[2] + 30 || E.bbox[0] > F.bbox[2] + 30 || F.bbox[1] > E.bbox[3] + 30 || E.bbox[1] > F.bbox[3] + 30)
                    continue;
                if (idx[e] == null) idx[e] = new PtIndex(E.devPts, 30);
                int c = 0, m = 0;
                for (int t = 0; t < F.devPts.length; t += 3, m++) if (idx[e].near(F.devPts[t])[0] <= 30) c++;
                ov[f][e] = c / (double) Math.max(1, m);
            }
        int need = L >= 6000 ? 2 : 1;
        List<Alt> alts = new ArrayList<>();
        for (int x = 0; x < n; x++) {
            if (need <= 1) add(alts, List.of(ears.get(x)));
            for (int y = x + 1; y < n; y++) {
                if (ears.get(y).a0 - ears.get(x).a1 < gapMin) continue;
                add(alts, List.of(ears.get(x), ears.get(y)));
                for (int z = y + 1; z < n; z++) {
                    if (ears.get(z).a0 - ears.get(y).a1 < gapMin) continue;
                    add(alts, List.of(ears.get(x), ears.get(y), ears.get(z)));
                }
            }
        }
        alts.sort((p, q) -> Double.compare(q.score, p.score));
        return alts;
    }

    void add(List<Alt> alts, List<Ear> ears) {
        double span = 0, extra = 0, v = 0, longest = -1;
        String lead = null;
        for (Ear e : ears) {
            span += e.span;
            extra += Math.max(0, e.len - e.span);
            v += e.value;
            if (e.len > longest) {
                longest = e.len;
                lead = e.fam;
            }
        }
        if (span < 0.17 * L || span > 0.58 * L || extra > 0.18 * L) return;
        Alt a = new Alt();
        a.ears = ears;
        a.score = v;
        a.lead = lead;
        alts.add(a);
    }

    double overlapBound(Alt p, Alt q) {
        double hit = 0, tot = 0;
        for (Ear f : q.ears) {
            double s = 0;
            for (Ear e : p.ears) s += ov[f.id][e.id];
            hit += Math.min(1, s) * f.devPts.length;
            tot += f.devPts.length;
        }
        return tot == 0 ? 0 : hit / tot;
    }

    boolean real(Alt a) {
        if (a.gs == null) {
            a.coords = splice(a.ears);
            a.gs = guard.score(a.coords);
        }
        return a.gs.real;
    }

    double bonus(Alt p, Alt q) {
        return p.lead != null && q.lead != null && !p.lead.equals(q.lead) ? famBonus : 0;
    }

    Alt[] pick(List<Alt> alts, Stats st) {
        List<Alt> top = new ArrayList<>(alts.subList(0, Math.min(alts.size(), 300)));
        Set<Alt> failed = Collections.newSetFromMap(new IdentityHashMap<>());
        Alt[] res = {null, null};
        for (int round = 0; round < 12; round++) {
            Alt b1 = null, b2 = null;
            double best = -1e9;
            for (Alt p : top) {
                if (failed.contains(p)) continue;
                if (p.score + (top.isEmpty() ? 0 : top.get(0).score) + famBonus <= best) break;
                for (Alt q : top) {
                    if (q == p || failed.contains(q)) continue;
                    if (p.score + q.score + famBonus <= best) break;
                    if (overlapBound(p, q) > 0.3) continue;
                    double sc = p.score + q.score + bonus(p, q);
                    if (sc <= best) continue;
                    best = sc;
                    b1 = p;
                    b2 = q;
                }
            }
            if (b1 == null) break;
            boolean ok1 = real(b1), ok2 = real(b2);
            if (ok1 && ok2 && !R4Guard.distinct(b1.gs.devPts, b2.gs.devPts)) {
                failed.add(b2);
                continue;
            }
            if (ok1 && ok2) {
                res[0] = b1;
                res[1] = b2;
                break;
            }
            if (!ok1) failed.add(b1);
            if (!ok2) failed.add(b2);
        }
        if (res[0] == null)
            for (int t = 0; t < Math.min(top.size(), 20); t++) {
                Alt a = top.get(t);
                if (failed.contains(a)) continue;
                if (real(a)) {
                    res[0] = a;
                    break;
                }
                failed.add(a);
            }
        return res;
    }

    // ================================================================ per request

    /** Up to two alternatives (null entries when none). */
    public Alt[] run(MainPath main) {
        this.b = main;
        Stats st = new Stats();
        prepare();
        st.goodPerWin = new int[nWin + 1];
        double w = winSpan * L / nWin;
        List<Ear> inv = new ArrayList<>();
        long tb = System.nanoTime();
        for (int k = 0; k < nWin; k++) {
            List<Ear> got = searchB(k * w, (k + 1) * w, k, null, st);
            st.goodPerWin[k] += got.size();
            inv.addAll(got);
        }
        st.msB += (System.nanoTime() - tb) / 1e6;
        long ta = System.nanoTime();
        if (useA) {
            double next = 0;
            for (int k = 0; k < b.size(); k++) {
                if (b.node[k] < 0 || b.along[k] < next) continue;
                if (b.along[k] > aStartMax * L) break;
                next = b.along[k] + aStep * L;
                for (int side = 0; side < 2; side++) inv.addAll(searchA(k, side, st));
            }
        }
        st.msA += (System.nanoTime() - ta) / 1e6;
        if (probeMode == 1) inv.addAll(probeEars(st));
        for (Ear e : inv) st.famGood.merge(e.fam, 1, Integer::sum);
        long tp = System.nanoTime();
        Alt[] pair = pick(assemble(inv), st);
        st.msPick += (System.nanoTime() - tp) / 1e6;
        if (retry > 0 && pair[1] == null) { // round-3 B's single retry
            IntHashSet forb = new IntHashSet();
            double lo, hi;
            int win;
            if (pair[0] != null) {
                Ear tgt = null;
                int fewest = Integer.MAX_VALUE;
                for (Ear e : pair[0].ears) {
                    int c = 0;
                    for (Ear f : inv) if (f != e && f.win == e.win && R4Guard.overlapFrac(e.devPts, f.devPts) <= 0.3) c++;
                    if (c < fewest) {
                        fewest = c;
                        tgt = e;
                    }
                }
                double cl = 0;
                for (int key : tgt.keys) {
                    EdgeIteratorState e = g.getEdgeIteratorStateForKey(key);
                    double mid = cl + e.getDistance() / 2;
                    if (mid > 0.15 * tgt.len && mid < 0.85 * tgt.len) forb.add(e.getEdge());
                    cl += e.getDistance();
                }
                win = tgt.win >= 0 ? tgt.win : (int) Math.min(nWin - 1, Math.floor(tgt.a0 / w));
                lo = win * w;
                hi = (win + 1) * w;
                st.retryKind = "forbid-core:w" + win;
            } else {
                win = 0;
                for (int k = 1; k < nWin; k++) if (st.goodPerWin[k] < st.goodPerWin[win]) win = k;
                lo = (win + 0.5) * w;
                hi = (win + 1.5) * w;
                st.retryKind = "shift:w" + win;
            }
            List<Ear> got = searchB(lo, hi, nWin, forb, st);
            st.goodPerWin[nWin] += got.size();
            inv.addAll(got);
            pair = pick(assemble(inv), st);
        }
        if (probeMode == 2 && pair[1] == null) { // fallback: the missed-option probe's detours, through the same filters
            List<Ear> got = probeEars(st);
            for (Ear e : got) st.famGood.merge(e.fam, 1, Integer::sum);
            if (!got.isEmpty()) {
                inv.addAll(got);
                pair = pick(assemble(inv), st);
                st.retryKind += "+probe" + got.size();
            }
        }
        return pair;
    }

    /** main length the round-4 percentages refer to (the route's distance). */
    public double mainLength() {
        return L;
    }
}
