/*
 * Trailmap - Personal routing, "bit of new" variations
 */
package com.graphhopper.trailmap.personal.variations;

import com.carrotsearch.hppc.IntArrayList;
import com.carrotsearch.hppc.IntDoubleHashMap;
import com.carrotsearch.hppc.IntHashSet;
import com.carrotsearch.hppc.IntObjectHashMap;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.trailmap.personal.PersonalFamiliarity;
import com.graphhopper.util.*;

import java.util.*;

/**
 * Round-3 candidate A generator ("a3" alternatives, "face-band repair"), a faithful copy of the round-3 A i5
 * prototype ({@code R3AFaceBandTest}, algo/personal_routing/rounds/3/a), test scaffolding and the unused
 * iteration-1 code removed, i5's settings as defaults.
 *
 * <p>Per request: the planar faces ("blocks") left and right of the main route, walked over edges the profile can
 * ride (rotation system by edge bearing, dead-end spurs cancelled); neighbouring same-side blocks merged into places
 * whose main stretch is about one deviation long. The faces only choose where a search starts and on which side: per
 * place start and side one banded one-to-many Dijkstra that may not stay within {@code band} of main except a short
 * exit zone and an entry run before the rejoin (first return to main ends a path); the best few rejoins per search by
 * a cheap proxy. Candidates: cost/km ≤ {@code rho} × the replaced stretch and exactly one good deviation under the
 * guard port. Selection: interval DP (≤ K deviations, spacing, extra and span budgets) with a guard check of the
 * spliced alternative, alt 1 tried with K and K-1, alt 2 from what stays clear of alt 1 (other side preferred).
 *
 * <p>Detour flavour ({@code none | avoid | explore}): the search multiplies the profile's edge cost by 1 + habit
 * (avoid, lifetime history) or 1 + the last-12-month share (explore), and the candidate value adds the flavour term;
 * quality is still judged on the plain profile cost.
 *
 * <p>One instance per request.
 */
public final class A3Generator {

    // ---- parameters (round-3 A i5)
    final String flavour;                // none | avoid | explore
    final double minSpanF = 0.12;        // place main stretch, fraction of L
    final double maxSpanF = 0.32;
    final double startSpacingF = 0.04;   // min along between place starts (fraction of L)
    final int perStart = 2;              // places per start block
    final int perStartArr = 4;           // rejoins kept per one-to-many search
    final int pairK = 1;                 // also try alt 1 with K-1 slices and keep the better pair
    final double band = 150;             // forbidden band near main (m)
    final double zone = 300;             // exit/entry zone radius around the place ends (m)
    final double alongMargin = 250;      // region may extend this far before/after the stretch
    final double faceMaxStepsD = 3000;
    final double faceMaxLenF = 1.2;      // face perimeter cap, fraction of L (+2 km)
    final int maxSettled = 10000;
    final double rho = 1.3;              // cost/km of deviation <= rho × replaced
    final int K = 3;
    final double spacing = 200;          // min metres along main between deviations
    final double extraBudgetF = 0.18;    // total extra length, fraction of L
    final double spanBudgetF = 0.58;     // total replaced span, fraction of L
    final double alt2Overlap = 0.25;     // candidate excluded from alt 2 above this share near alt 1
    final double otherSideBonus = 0.2;
    final double maxDistF = 0.42;        // search radius in path metres, fraction of L
    final boolean mainTurns = true;      // replaced-stretch cost includes main's turn costs
    final double preDup = 0.85;          // skip building a path sharing this much with one already built
    final double corrFrac = 0.8;         // search reach as a fraction of the guard's corridor limit
    final boolean tightPrune = true;     // return prune also counts the along distance still needed to reach minSpan
    final double proxyGap = 0.03;        // min span difference between rejoins of one search (fraction of L)
    final double flavourBeta = 1.0;      // flavour multiplier in the search
    final double flavourValue = 0.6;     // flavour weight in the candidate value

    final BaseGraph g;
    final Weighting profileW;
    final PersonalFamiliarity life, last12;

    /**
     * @param life   lifetime history (habit; needed for avoid), may be null otherwise
     * @param last12 last-12-month history (needed for explore), may be null otherwise
     */
    public A3Generator(BaseGraph g, Weighting profileW, PersonalFamiliarity life, PersonalFamiliarity last12, String flavour) {
        this.g = g;
        this.profileW = profileW;
        this.life = life;
        this.last12 = last12;
        this.flavour = flavour;
    }

    // ------------------------------------------------------------------ main route

    /** Main route as tower-node sequence: node[k] between main edges e_k and e_{k+1}. */
    static final class Base {
        List<double[]> coords;
        int[] node, ptIdx, inEdge, outEdge;
        double[] along;      // projected along metres at node[k] (guard samples)
        double[] pwCum;      // profile weight along main at node[k]
        IntHashSet edgeSet;
        IntObjectHashMap<IntArrayList> occ;
        IntObjectHashMap<Integer> fwdKeyPos = new IntObjectHashMap<>(); // forward main half-edge key -> k (node[k-1] -> node[k])
        A3Guard guard;

        int size() {
            return node.length;
        }
    }

    /** Per-request preparation on top of the main route. */
    Base prepare(MainPath m) {
        Base b = new Base();
        b.coords = m.coords;
        b.node = m.node;
        b.ptIdx = m.ptIdx;
        b.inEdge = m.inEdge;
        b.outEdge = m.outEdge;
        b.edgeSet = m.edgeSet;
        b.occ = m.occ;
        int n = b.size();
        b.fwdKeyPos = new IntObjectHashMap<>();
        b.guard = new A3Guard(b.coords);
        b.along = new double[n];
        for (int k = 0; k < n; k++) b.along[k] = b.guard.cum[b.guard.sampleOfPoint[b.ptIdx[k]]];
        b.pwCum = new double[n];
        for (int k = 1; k < n; k++) {
            double w = 0;
            if (b.node[k] >= 0) {
                EdgeIteratorState e = g.getEdgeIteratorState(b.inEdge[k], b.node[k]);
                if (e != null) w = profileW.calcEdgeWeight(e, false);
                // the replaced stretch pays its turn costs too (the deviation's cost includes them)
                if (mainTurns && k + 1 < n) {
                    double tw = profileW.calcTurnWeight(b.inEdge[k], b.node[k], b.outEdge[k]);
                    if (Double.isFinite(tw)) w += tw;
                }
            }
            b.pwCum[k] = b.pwCum[k - 1] + (Double.isFinite(w) ? w : 0);
        }
        for (int k = 1; k < n; k++) {
            if (b.node[k] < 0 || b.node[k - 1] < 0) continue;
            EdgeIteratorState e = g.getEdgeIteratorState(b.inEdge[k], b.node[k]);
            if (e != null && e.getBaseNode() == b.node[k - 1]) b.fwdKeyPos.put(e.getEdgeKey(), k);
        }
        return b;
    }

    // ------------------------------------------------------------------ per-request state

    final class Leg {
        final Base b;
        final double L, corrCap;
        final EdgeExplorer ex = g.createEdgeExplorer();
        final IntObjectHashMap<int[]> rot = new IntObjectHashMap<>();
        final IntObjectHashMap<double[]> geomInfo = new IntObjectHashMap<>(); // edge id -> [d, along, side] × points (storage order)
        int searches, settled;
        final IntDoubleHashMap wCache = new IntDoubleHashMap();

        Leg(Base b) {
            this.b = b;
            L = b.guard.L;
            corrCap = corrFrac * Math.max(500, 0.15 * L);
        }

        boolean rideable(EdgeIteratorState e, boolean rev) {
            return Double.isFinite(profileW.calcEdgeWeight(e, rev));
        }

        /** Outgoing rideable half-edge keys at v, sorted counter-clockwise by first-segment bearing. */
        int[] rot(int v) {
            int[] r = rot.get(v);
            if (r != null) return r;
            List<double[]> l = new ArrayList<>();
            EdgeIterator it = ex.setBaseNode(v);
            while (it.next()) {
                if (it.getAdjNode() == v) continue;
                if (!rideable(it, false) && !rideable(it, true)) continue;
                PointList pl = it.fetchWayGeometry(FetchMode.ALL);
                double[] p0 = b.guard.proj(pl.getLon(0), pl.getLat(0));
                double[] p1 = p0;
                for (int t = 1; t < pl.size(); t++) {
                    p1 = b.guard.proj(pl.getLon(t), pl.getLat(t));
                    if (Math.hypot(p1[0] - p0[0], p1[1] - p0[1]) >= 3) break;
                }
                l.add(new double[]{Math.atan2(p1[1] - p0[1], p1[0] - p0[0]), it.getEdgeKey()});
            }
            l.sort(Comparator.comparingDouble(x -> x[0]));
            r = new int[l.size()];
            for (int t = 0; t < r.length; t++) r[t] = (int) l.get(t)[1];
            rot.put(v, r);
            return r;
        }

        int adjOf(int key) {
            return g.getEdgeIteratorStateForKey(key).getAdjNode();
        }

        int baseOf(int key) {
            return g.getEdgeIteratorStateForKey(key).getBaseNode();
        }

        boolean mainNode(int v) {
            return b.occ.containsKey(v);
        }

        /** Face walk from a main forward half-edge; left = always the leftmost exit (face on the left). */
        IntArrayList walk(int startKey, boolean left) {
            IntArrayList cyc = new IntArrayList();
            int h = startKey, steps = 0;
            double len = 0, maxLen = faceMaxLenF * L + 2000;
            do {
                cyc.add(h);
                EdgeIteratorState s = g.getEdgeIteratorStateForKey(h);
                int v = s.getAdjNode();
                len += s.getDistance();
                int[] r = rot(v);
                int back = GHUtility.reverseEdgeKey(h), idx = -1;
                for (int t = 0; t < r.length; t++)
                    if (r[t] == back) {
                        idx = t;
                        break;
                    }
                if (idx < 0) return null;
                int deg = r.length;
                h = left ? r[(idx - 1 + deg) % deg] : r[(idx + 1) % deg];
                if (++steps > faceMaxStepsD || len > maxLen) return null;
            } while (h != startKey);
            // cancel dead-end spurs: a half-edge followed by its reverse
            IntArrayList st = new IntArrayList();
            for (int t = 0; t < cyc.size(); t++) {
                int k = cyc.get(t);
                if (st.size() > 0 && st.get(st.size() - 1) == GHUtility.reverseEdgeKey(k)) st.remove(st.size() - 1);
                else st.add(k);
            }
            return st;
        }

        double[] info(int edge) {
            double[] inf = geomInfo.get(edge);
            if (inf != null) return inf;
            PointList pl = g.getEdgeIteratorState(edge, Integer.MIN_VALUE).fetchWayGeometry(FetchMode.ALL);
            inf = new double[pl.size() * 3];
            for (int t = 0; t < pl.size(); t++) {
                double[] p = b.guard.proj(pl.getLon(t), pl.getLat(t));
                double[] q = b.guard.lateral(p[0], p[1], corrCap + 100);
                inf[3 * t] = q[0];
                inf[3 * t + 1] = q[1];
                inf[3 * t + 2] = q[2];
            }
            geomInfo.put(edge, inf);
            return inf;
        }
    }

    /** One raw block: main stretch node[i]..node[j] on one side, with its far-side chain (half-edge keys i -> j). */
    static final class Block {
        int side, i, j;
        int[] chain;
        boolean rideable;
        double len;
    }

    /** One deviation candidate: main node i to main node j over edge keys. */
    public static final class Cand {
        int side, i, j;
        int[] keys;
        List<double[]> geom;
        double[][] offPts; // projected resampled points farther than 20 m from main
        public double len, span, extra, cost, repCost, costRatio, maxD, meanD, value, habitDev, habitRep, novel;
        int sharp;
        public double a0, a1;
        IntHashSet edges = new IntHashSet();
    }

    // ------------------------------------------------------------------ blocks

    List<Block> blocks(Leg lg, int side) {
        Base b = lg.b;
        boolean left = side > 0;
        IntHashSet done = new IntHashSet();
        List<Block> out = new ArrayList<>();
        for (int k = 1; k < b.size(); k++) {
            if (b.node[k] < 0 || b.node[k - 1] < 0) continue;
            EdgeIteratorState e = g.getEdgeIteratorState(b.inEdge[k], b.node[k]);
            if (e == null || e.getBaseNode() != b.node[k - 1]) continue;
            int key = e.getEdgeKey();
            if (done.contains(key)) continue;
            IntArrayList cyc = lg.walk(key, left);
            if (cyc == null) {
                done.add(key);
                continue;
            }
            for (int t = 0; t < cyc.size(); t++) if (b.fwdKeyPos.containsKey(cyc.get(t))) done.add(cyc.get(t));
            // split into non-main pieces between main nodes
            IntArrayList piece = new IntArrayList();
            int pieceStart = -1;
            for (int t = 0; t < cyc.size(); t++) {
                int h = cyc.get(t);
                int eid = GHUtility.getEdgeFromEdgeKey(h);
                if (b.edgeSet.contains(eid)) {
                    piece.clear();
                    pieceStart = -1;
                    continue;
                }
                if (piece.isEmpty()) pieceStart = lg.baseOf(h);
                piece.add(h);
                int a = lg.adjOf(h);
                if (lg.mainNode(a)) {
                    if (pieceStart >= 0 && lg.mainNode(pieceStart)) {
                        Block bl = toBlock(lg, side, pieceStart, a, piece);
                        if (bl != null) out.add(bl);
                    }
                    piece.clear();
                    pieceStart = -1;
                }
            }
        }
        // dedupe identical stretches (keep the shorter chain)
        out.sort(Comparator.<Block>comparingInt(x -> x.i).thenComparingInt(x -> x.j).thenComparingDouble(x -> x.len));
        List<Block> d = new ArrayList<>();
        for (Block x : out)
            if (d.isEmpty() || d.get(d.size() - 1).i != x.i || d.get(d.size() - 1).j != x.j) d.add(x);
        return d;
    }

    Block toBlock(Leg lg, int side, int startNode, int endNode, IntArrayList piece) {
        if (startNode == endNode) return null;
        IntArrayList ps = lg.b.occ.get(startNode), qs = lg.b.occ.get(endNode);
        int bp = -1, bq = -1;
        for (int x = 0; x < ps.size(); x++)
            for (int y = 0; y < qs.size(); y++) {
                int p = ps.get(x), q = qs.get(y);
                if (q < p && (bp < 0 || p - q < bp - bq)) {
                    bp = p;
                    bq = q;
                }
            }
        if (bp < 0) return null;
        Block bl = new Block();
        bl.side = side;
        bl.i = bq;
        bl.j = bp;
        bl.chain = new int[piece.size()];
        bl.rideable = true;
        for (int t = 0; t < piece.size(); t++) {
            int k = GHUtility.reverseEdgeKey(piece.get(piece.size() - 1 - t));
            bl.chain[t] = k;
            EdgeIteratorState s = g.getEdgeIteratorStateForKey(k);
            bl.len += s.getDistance();
            if (!Double.isFinite(profileW.calcEdgeWeight(s, false))) bl.rideable = false;
        }
        return bl;
    }

    /** Places: contiguous same-side block runs whose main stretch is about one deviation long. */
    List<Block> places(Leg lg, List<Block> bl) {
        Base b = lg.b;
        double minSpan = minSpanF * lg.L, maxSpan = maxSpanF * lg.L, lastStart = Double.NEGATIVE_INFINITY;
        List<Block> out = new ArrayList<>();
        for (int a = 0; a < bl.size(); a++) {
            Block s0 = bl.get(a);
            if (b.along[s0.i] - lastStart < startSpacingF * lg.L) continue;
            IntArrayList chain = new IntArrayList();
            boolean ride = true;
            int cur = a, emitted = 0;
            double lastEmit = 0;
            Block c = s0;
            while (true) {
                for (int k : c.chain) {
                    if (chain.size() > 0 && chain.get(chain.size() - 1) == GHUtility.reverseEdgeKey(k)) chain.remove(chain.size() - 1);
                    else chain.add(k);
                }
                ride &= c.rideable;
                double span = b.along[c.j] - b.along[s0.i];
                if (span > maxSpan) break;
                if (span >= minSpan && (emitted == 0 || span >= lastEmit * 1.3)) {
                    Block p = new Block();
                    p.side = s0.side;
                    p.i = s0.i;
                    p.j = c.j;
                    p.rideable = ride;
                    p.chain = chain.toArray();
                    out.add(p);
                    emitted++;
                    lastEmit = span;
                    if (emitted >= perStart) break;
                }
                // next adjacent block: starts where this one ends (smallest j)
                int nx = -1;
                for (int t = cur + 1; t < bl.size(); t++) {
                    Block o = bl.get(t);
                    if (o.i > c.j) break;
                    if (o.i == c.j && (nx < 0 || o.j < bl.get(nx).j)) nx = t;
                }
                if (nx < 0) break;
                cur = nx;
                c = bl.get(nx);
            }
            if (emitted > 0) lastStart = b.along[s0.i];
        }
        return out;
    }

    // ------------------------------------------------------------------ candidates

    double flavourMult(EdgeIteratorState e) {
        switch (flavour) {
            case "avoid":
                return 1 + flavourBeta * habit(e);
            case "explore":
                return 1 + flavourBeta * last12.share(e);
            default:
                return 1;
        }
    }

    double habit(EdgeIteratorState e) {
        if (life == null) return 0;
        double s = life.share(e);
        return s <= 0 ? 0 : s * Math.min(1, life.rides(e) / 10.0);
    }

    static final class SLabel {
        final int key, edge, node;
        final double gw, dist, bandRun;
        final SLabel parent;

        SLabel(int key, int edge, int node, double gw, double dist, double bandRun, SLabel parent) {
            this.key = key;
            this.edge = edge;
            this.node = node;
            this.gw = gw;
            this.dist = dist;
            this.bandRun = bandRun;
            this.parent = parent;
        }
    }

    /**
     * Edge class for the one-to-many banded search from main index si on one side:
     * 0 forbidden, 1 far side (all points ≥ band off main, right side, inside the along window and reach),
     * 2 exit zone (the near-main points lie within {@code zone} of the start), 3 entry candidate (near-main points
     * on the right side or within 25 m, inside the target window; allowed for at most {@code zone} metres in a row).
     */
    int edgeClass(Leg lg, int edge, int side, double a0, double a1, double t0, double t1, double reach, double[] ps) {
        double[] inf = lg.info(edge);
        PointList pl = null;
        boolean notFar = false, allStart = true, entryOk = true;
        for (int t = 0; t < inf.length / 3; t++) {
            double d = inf[3 * t], al = inf[3 * t + 1];
            int sd = (int) inf[3 * t + 2];
            if (d > reach) return 0;
            if (d >= band && sd == side && al >= a0 && al <= a1) continue;
            notFar = true;
            if (!(d < 25 || sd == side)) return 0;
            if (pl == null) pl = g.getEdgeIteratorState(edge, Integer.MIN_VALUE).fetchWayGeometry(FetchMode.ALL);
            double[] q = lg.b.guard.proj(pl.getLon(t), pl.getLat(t));
            if (Math.hypot(q[0] - ps[0], q[1] - ps[1]) > zone) allStart = false;
            if (al < t0 || al > t1) entryOk = false;
        }
        if (!notFar) return 1;
        if (allStart) return 2;
        return entryOk ? 3 : 0;
    }

    /** One banded Dijkstra from main index si on one side; best path to every main index k reached (first return). */
    IntObjectHashMap<int[]> searchFrom(Leg lg, int side, int si, double reach, IntObjectHashMap<double[]> meta) {
        Base b = lg.b;
        int s = b.node[si];
        double minSpan = minSpanF * lg.L, maxSpan = maxSpanF * lg.L, as = b.along[si];
        double a0 = as - alongMargin, a1 = as + maxSpan + alongMargin, t0 = as + minSpan - zone, t1 = as + maxSpan + zone;
        double[] ps = b.guard.proj(b.coords.get(b.ptIdx[si])[0], b.coords.get(b.ptIdx[si])[1]);
        double maxDist = maxDistF * lg.L;
        lg.searches++;
        PriorityQueue<SLabel> pq = new PriorityQueue<>(Comparator.comparingDouble(l -> l.gw));
        IntHashSet settled = new IntHashSet();
        IntObjectHashMap<Integer> cls = new IntObjectHashMap<>();
        IntObjectHashMap<SLabel> arr = new IntObjectHashMap<>();
        SLabel cur = new SLabel(-1, b.inEdge[si], s, 0, 0, 0, null);
        int count = 0;
        while (cur != null) {
            EdgeIterator it = lg.ex.setBaseNode(cur.node);
            while (it.next()) {
                int e = it.getEdge();
                if (e == cur.edge || b.edgeSet.contains(e)) continue;
                int key = it.getEdgeKey();
                if (settled.contains(key)) continue;
                double d = cur.dist + it.getDistance();
                if (d > maxDist) continue;
                Integer c = cls.get(e);
                if (c == null) cls.put(e, c = edgeClass(lg, e, side, a0, a1, t0, t1, reach, ps));
                if (c == 0) continue;
                double run = c == 3 ? cur.bandRun + it.getDistance() : 0;
                if (run > zone) continue;
                double ew;
                if (lg.wCache.containsKey(key)) ew = lg.wCache.get(key);
                else {
                    ew = profileW.calcEdgeWeight(it, false);
                    if (Double.isFinite(ew)) ew *= flavourMult(it);
                    lg.wCache.put(key, ew);
                }
                if (Double.isInfinite(ew)) continue;
                double tw = profileW.calcTurnWeight(cur.edge, cur.node, e);
                if (Double.isInfinite(tw)) continue;
                double gw = cur.gw + ew + tw;
                int a = it.getAdjNode();
                if (lg.mainNode(a)) { // first return: record, never relax out of main
                    IntArrayList ks = b.occ.get(a);
                    for (int x = 0; x < ks.size(); x++) {
                        int k = ks.get(x);
                        double span = b.along[k] - as;
                        if (k <= si || span < minSpan || span > maxSpan) continue;
                        double tw2 = profileW.calcTurnWeight(e, a, b.outEdge[k]);
                        if (Double.isInfinite(tw2) || e == b.outEdge[k]) continue;
                        SLabel old = arr.get(k);
                        if (old == null || gw + tw2 < old.gw) arr.put(k, new SLabel(key, e, a, gw + tw2, d, run, cur));
                    }
                    continue;
                }
                // return prune: the way back to main is at least the adj node's lateral offset
                double[] inf = lg.info(e);
                double latAdj = (key & 1) == 0 ? inf[inf.length - 3] : inf[0];
                double alAdj = (key & 1) == 0 ? inf[inf.length - 2] : inf[1];
                double back = tightPrune ? Math.hypot(latAdj, Math.max(0, as + minSpan - alAdj)) : latAdj;
                if (d + back > maxDist) continue;
                pq.add(new SLabel(key, e, a, gw, d, run, cur));
            }
            cur = null;
            while (!pq.isEmpty()) {
                SLabel l = pq.poll();
                if (settled.add(l.key)) {
                    cur = l;
                    break;
                }
            }
            if (++count > maxSettled) break;
        }
        lg.settled += count;
        IntObjectHashMap<int[]> out = new IntObjectHashMap<>();
        for (var en : arr) {
            IntArrayList keys = new IntArrayList();
            for (SLabel l = en.value; l != null && l.key >= 0; l = l.parent) keys.add(l.key);
            int[] ks = new int[keys.size()];
            for (int x = 0; x < ks.length; x++) ks[x] = keys.get(ks.length - 1 - x);
            out.put(en.key, ks);
            if (meta != null) meta.put(en.key, new double[]{en.value.gw, en.value.dist});
        }
        return out;
    }

    /** Profile cost (edges + turns, no flavour) of a deviation node[i] -> node[j]. */
    double profileCost(Base b, int i, int j, int[] keys) {
        double c = 0;
        int prevEdge = b.inEdge[i], prevNode = b.node[i];
        for (int k : keys) {
            EdgeIteratorState e = g.getEdgeIteratorStateForKey(k);
            c += profileW.calcEdgeWeight(e, false) + profileW.calcTurnWeight(prevEdge, prevNode, e.getEdge());
            prevEdge = e.getEdge();
            prevNode = e.getAdjNode();
        }
        return c + profileW.calcTurnWeight(prevEdge, prevNode, b.outEdge[j]);
    }

    /** Build + score a candidate from a key chain node[i] -> node[j]; null when it fails. */
    Cand build(Leg lg, int side, int i, int j, int[] keys) {
        Base b = lg.b;
        Cand c = new Cand();
        c.side = side;
        c.i = i;
        c.j = j;
        c.keys = keys;
        IntHashSet seen = new IntHashSet();
        double cost = 0, len = 0, hab = 0, nov = 0;
        int prevEdge = b.inEdge[i];
        int prevNode = b.node[i];
        List<double[]> geom = new ArrayList<>();
        geom.add(b.coords.get(b.ptIdx[i]));
        for (int k : keys) {
            EdgeIteratorState e = g.getEdgeIteratorStateForKey(k);
            if (!seen.add(e.getEdge())) return null;
            double w = profileW.calcEdgeWeight(e, false);
            double tw = profileW.calcTurnWeight(prevEdge, prevNode, e.getEdge());
            if (Double.isInfinite(w) || Double.isInfinite(tw)) return null;
            cost += w + tw;
            len += e.getDistance();
            hab += habit(e) * e.getDistance();
            if (last12 != null) nov += (last12.share(e) < 0.3 ? 1 : 0) * e.getDistance();
            prevEdge = e.getEdge();
            prevNode = e.getAdjNode();
            PointList pl = e.fetchWayGeometry(FetchMode.PILLAR_AND_ADJ);
            for (int t = 0; t < pl.size(); t++) geom.add(MainPath.coord(pl, t));
            c.edges.add(e.getEdge());
        }
        double tw2 = profileW.calcTurnWeight(prevEdge, prevNode, b.outEdge[j]);
        if (Double.isInfinite(tw2)) return null;
        cost += tw2;
        geom.set(geom.size() - 1, b.coords.get(b.ptIdx[j]));
        c.geom = geom;
        c.cost = cost;
        c.a0 = b.along[i];
        c.a1 = b.along[j];
        c.span = c.a1 - c.a0;
        c.repCost = b.pwCum[j] - b.pwCum[i];
        c.costRatio = (cost / Math.max(1, len)) / Math.max(1e-9, c.repCost / Math.max(1, c.span));
        if (c.costRatio > rho) return null;
        // cheap pre-checks on edge lengths (the guard's deviation is the path minus at most two zones)
        if (len < 0.1 * lg.L || len - 2 * zone > 0.4 * lg.L || len - 2 * zone > 2 * c.span) return null;
        double[][] P = b.guard.resample(geom, null);
        c.len = A3Guard.polyLen(P, 0, P.length - 1);
        c.extra = c.len - c.span;
        c.habitDev = hab / Math.max(1, len);
        c.novel = nov / Math.max(1, len);
        // habit of the replaced main stretch
        double hr = 0, lr = 0;
        for (int k = i + 1; k <= j; k++) {
            EdgeIteratorState e = g.getEdgeIteratorState(b.inEdge[k], Integer.MIN_VALUE);
            hr += habit(e) * e.getDistance();
            lr += e.getDistance();
        }
        c.habitRep = hr / Math.max(1, lr);
        // guard check of the piece alone: exactly one good deviation
        List<A3Guard.Dev> devs = b.guard.deviations(P);
        if (devs.size() != 1) return null;
        A3Guard.Dev d = devs.get(0);
        b.guard.judge(d, P, true);
        if (!d.good) return null;
        c.maxD = d.maxD;
        List<double[]> off = new ArrayList<>();
        double sumD = 0;
        for (int t = d.s; t <= d.e; t++) {
            double fd = Math.min(lg.corrCap + 100, b.guard.fastDist(P[t], lg.corrCap + 100));
            sumD += fd;
            if (fd > 20) off.add(P[t]);
        }
        c.meanD = sumD / Math.max(1, d.e - d.s + 1);
        c.offPts = off.toArray(new double[0][]);
        if (c.costRatio > rho) return null;
        // smoothness: deflections > 60 deg at tower nodes inside the deviation
        int sharp = 0;
        for (int x = 0; x + 1 < keys.length; x++) {
            double h1 = bearing(g.getEdgeIteratorStateForKey(keys[x]), true), h2 = bearing(g.getEdgeIteratorStateForKey(keys[x + 1]), false);
            double dd = Math.abs(h2 - h1);
            if (dd > Math.PI) dd = 2 * Math.PI - dd;
            if (dd > Math.PI / 3) sharp++;
        }
        c.sharp = sharp;
        double v = 1 + 0.4 * Math.min(1, c.meanD / 300) - 1.0 * Math.max(0, c.costRatio - 1) - 0.4 * Math.max(0, c.extra) / Math.max(1, c.span)
                - 0.03 * sharp - 0.6 * Math.abs(c.span / lg.L - 0.2);
        if (flavour.equals("avoid")) v += flavourValue * (c.habitRep - c.habitDev);
        if (flavour.equals("explore")) v += flavourValue * (c.novel - 0.5);
        c.value = v;
        return c;
    }

    /** Heading of an edge at its end (atEnd) or its start, in the travel direction. */
    double bearing(EdgeIteratorState e, boolean atEnd) {
        PointList pl = e.fetchWayGeometry(FetchMode.ALL);
        int n = pl.size();
        int a = atEnd ? n - 2 : 0, c = atEnd ? n - 1 : 1;
        double dx = (pl.getLon(c) - pl.getLon(a)) * Math.cos(Math.toRadians(pl.getLat(a))), dy = pl.getLat(c) - pl.getLat(a);
        return Math.atan2(dy, dx);
    }

    // ------------------------------------------------------------------ selection

    /** Weighted interval scheduling with two budgets (extra length, replaced span); copied from round 2's DP. */
    static List<Cand> selectDP(List<Cand> cands, int k, double spacingM, double extraBudget, double spanBudget) {
        List<Cand> c = new ArrayList<>(cands);
        c.sort(Comparator.comparingDouble(x -> x.a1));
        double binE = 50, binS = 100;
        int n = c.size(), B = (int) Math.floor(extraBudget / binE), S = (int) Math.floor(spanBudget / binS);
        if (n == 0 || k <= 0) return List.of();
        double[] ends = new double[n];
        for (int x = 0; x < n; x++) ends[x] = c.get(x).a1;
        int[] prev = new int[n], bins = new int[n], sbins = new int[n];
        for (int x = 0; x < n; x++) {
            double lim = c.get(x).a0 - spacingM;
            int lo = 0, hi = x;
            while (lo < hi) {
                int mid = (lo + hi) >>> 1;
                if (ends[mid] <= lim) lo = mid + 1;
                else hi = mid;
            }
            prev[x] = lo;
            bins[x] = (int) Math.ceil(Math.max(0, c.get(x).extra) / binE);
            sbins[x] = (int) Math.ceil(Math.max(0, c.get(x).span) / binS);
        }
        double[][][][] f = new double[n + 1][k + 1][B + 1][S + 1];
        for (int p = 1; p <= n; p++) {
            Cand s = c.get(p - 1);
            for (int q = 0; q <= k; q++)
                for (int r = 0; r <= B; r++)
                    for (int u = 0; u <= S; u++) {
                        double best = f[p - 1][q][r][u];
                        if (q > 0 && bins[p - 1] <= r && sbins[p - 1] <= u && s.value > 0) {
                            double v = s.value + f[prev[p - 1]][q - 1][r - bins[p - 1]][u - sbins[p - 1]];
                            if (v > best) best = v;
                        }
                        f[p][q][r][u] = best;
                    }
        }
        List<Cand> out = new ArrayList<>();
        int p = n, q = k, r = B, u = S;
        while (p > 0 && q > 0) {
            Cand s = c.get(p - 1);
            if (f[p][q][r][u] != f[p - 1][q][r][u]) {
                out.add(s);
                r -= bins[p - 1];
                u -= sbins[p - 1];
                q--;
                p = prev[p - 1];
            } else p--;
        }
        Collections.reverse(out);
        return out;
    }

    static List<double[]> splice(Base b, List<Cand> chosen) {
        List<Cand> s = new ArrayList<>(chosen);
        s.sort(Comparator.comparingInt(x -> x.i));
        List<double[]> out = new ArrayList<>();
        int cur = 0;
        for (Cand sl : s) {
            int pi = b.ptIdx[sl.i], pj = b.ptIdx[sl.j];
            for (int k = cur; k <= pi; k++) out.add(b.coords.get(k));
            for (int k = 1; k < sl.geom.size() - 1; k++) out.add(sl.geom.get(k));
            cur = pj;
        }
        for (int k = cur; k < b.coords.size(); k++) out.add(b.coords.get(k));
        return out;
    }

    /** One A3 alternative: the main route with its chosen deviations spliced in. */
    public static final class Pick {
        public List<Cand> chosen;
        public List<double[]> coords;
        A3Guard.Alt judged;
    }

    /** DP + guard check; on failure drop the slices behind bad deviations and retry (bounded). */
    Pick pick(Leg lg, List<Cand> pool0, int k) {
        List<Cand> pool = new ArrayList<>(pool0);
        Pick bestEffort = null;
        for (int tries = 0; tries < 8; tries++) {
            List<Cand> ch = selectDP(pool, k, spacing, extraBudgetF * lg.L, spanBudgetF * lg.L);
            if (ch.isEmpty()) break;
            Pick pk = new Pick();
            pk.chosen = ch;
            pk.coords = splice(lg.b, ch);
            pk.judged = lg.b.guard.judgeAlt(pk.coords);
            if (pk.judged.real) return pk;
            if (pk.judged.bad == 0 && (bestEffort == null || pk.judged.good > bestEffort.judged.good)) bestEffort = pk;
            // remove slices overlapping bad deviations; if none bad, remove the weakest slice
            boolean removed = false;
            for (A3Guard.Dev d : pk.judged.devs) {
                if (d.good) continue;
                double da0 = d.exitI * A3Guard.STEP, da1 = d.rejoinI * A3Guard.STEP;
                for (Cand c : ch)
                    if (c.a1 >= Math.min(da0, da1) - 50 && c.a0 <= Math.max(da0, da1) + 50) removed |= pool.remove(c);
            }
            if (!removed) {
                if (pk.judged.good < (lg.L >= 6000 ? 2 : 1)) break; // not enough material: more DP won't add slices
                Cand w = Collections.min(ch, Comparator.comparingDouble(x -> x.value));
                pool.remove(w);
            }
        }
        return bestEffort;
    }

    // ------------------------------------------------------------------ per request

    /** The A3 result: up to two alternatives and the main length their percentages refer to. */
    public static final class Result {
        public final List<Pick> alts = new ArrayList<>();
        public double L;
    }

    public Result run(MainPath main) {
        Base b = prepare(main);
        Leg lg = new Leg(b);
        Result res = new Result();
        res.L = lg.L;
        List<Cand> cands = new ArrayList<>();
        for (int side : new int[]{1, -1}) {
            List<Block> bl = blocks(lg, side);
            List<Block> pl = places(lg, bl);
            // one banded one-to-many search per place start (the search reach is the corridor cap)
            Set<Integer> starts = new TreeSet<>();
            for (Block p : pl) starts.add(p.i);
            List<IntHashSet> builtSets = new ArrayList<>();
            for (int si : starts) {
                double reach = lg.corrCap;
                IntObjectHashMap<double[]> meta = new IntObjectHashMap<>();
                IntObjectHashMap<int[]> arr = searchFrom(lg, side, si, reach, meta);
                int[] ks = arr.keys().toArray();
                Arrays.sort(ks);
                IntHashSet pickK = new IntHashSet();
                // rank every rejoin by a cheap proxy (label cost and length vs the replaced stretch), keep the best
                // perStartArr with spans at least proxyGap·L apart
                List<double[]> rk = new ArrayList<>();
                for (int k : ks) {
                    double[] mt = meta.get(k);
                    double span = b.along[k] - b.along[si], len = mt[1];
                    double rep = b.pwCum[k] - b.pwCum[si];
                    // quality is judged on the profile's cost; the label cost carries the flavour multiplier
                    double pc = flavour.equals("none") ? mt[0] : profileCost(b, si, k, arr.get(k));
                    double cr = (pc / Math.max(1, len)) / Math.max(1e-9, rep / Math.max(1, span));
                    if (cr > rho || len < 0.1 * lg.L || len - 2 * zone > 0.4 * lg.L || len - 2 * zone > 2 * span) continue;
                    double pv = -1.0 * Math.max(0, cr - 1) - 0.4 * Math.max(0, len - span) / Math.max(1, span) - 0.6 * Math.abs(span / lg.L - 0.2);
                    rk.add(new double[]{pv, k, span});
                }
                rk.sort((x, y) -> Double.compare(y[0], x[0]));
                List<Double> taken = new ArrayList<>();
                for (double[] x : rk) {
                    if (pickK.size() >= perStartArr) break;
                    boolean near = false;
                    for (double t : taken) if (Math.abs(t - x[2]) < proxyGap * lg.L) near = true;
                    if (near) continue;
                    taken.add(x[2]);
                    pickK.add((int) x[1]);
                }
                int[] pks = pickK.toArray();
                Arrays.sort(pks); // deterministic order (hppc sets iterate in a per-instance order)
                for (int kv : pks) {
                    int[] pk = arr.get(kv);
                    IntHashSet es = new IntHashSet();
                    for (int k : pk) es.add(GHUtility.getEdgeFromEdgeKey(k));
                    boolean dup = false;
                    for (IntHashSet o : builtSets) {
                        IntHashSet inter = new IntHashSet(o);
                        inter.retainAll(es);
                        if (inter.size() >= preDup * Math.min(o.size(), es.size())) {
                            dup = true;
                            break;
                        }
                    }
                    if (dup) continue;
                    builtSets.add(es);
                    Cand rep = build(lg, side, si, kv, pk);
                    if (rep == null) continue;
                    cands.add(rep);
                }
            }
        }
        // dedupe near-identical candidates (edge Jaccard >= 0.7), keep the higher value
        cands.sort((x, y) -> Double.compare(y.value, x.value));
        List<Cand> uniq = new ArrayList<>();
        for (Cand c : cands) {
            boolean dup = false;
            for (Cand u : uniq) {
                if (u.side != c.side) continue;
                IntHashSet inter = new IntHashSet(u.edges);
                inter.retainAll(c.edges);
                if (inter.size() >= 0.7 * Math.min(u.edges.size(), c.edges.size())) {
                    dup = true;
                    break;
                }
            }
            if (!dup) uniq.add(c);
        }
        Pick[] bestPair = null;
        double bestPts = -1;
        for (int k1 = K; k1 >= Math.max(1, pairK > 0 ? K - 1 : K); k1--) {
            Pick[] pr = pair(lg, uniq, k1);
            if (pr[0] == null) continue;
            double pts = (pr[0].judged.real ? 1 : 0) + (pr[1] != null && pr[1].judged.real ? 1 : 0)
                    + 0.1 * (pr[1] != null ? 1 : 0) + 0.01 * (pr[0].judged.good + (pr[1] == null ? 0 : pr[1].judged.good));
            if (pts > bestPts) {
                bestPts = pts;
                bestPair = pr;
            }
        }
        if (bestPair != null) {
            res.alts.add(bestPair[0]);
            if (bestPair[1] != null) res.alts.add(bestPair[1]);
        }
        return res;
    }

    /** Alt 1 with at most k1 slices, then alt 2 from what stays clear of alt 1 (other side preferred). */
    Pick[] pair(Leg lg, List<Cand> uniq, int k1) {
        Pick a1 = pick(lg, uniq, k1);
        if (a1 == null) return new Pick[]{null, null};
        List<double[]> d1 = A3Guard.devPoints(a1.judged);
        List<Cand> pool2 = new ArrayList<>();
        for (Cand c : uniq) {
            if (a1.chosen.contains(c)) continue;
            double ov = A3Guard.nearShare(d1, Arrays.asList(c.offPts));
            if (ov > alt2Overlap) continue;
            Cand c2 = shallow(c);
            for (Cand x : a1.chosen)
                if (x.side != c.side && x.a1 > c.a0 && x.a0 < c.a1) {
                    c2.value += otherSideBonus;
                    break;
                }
            pool2.add(c2);
        }
        Pick a2 = pick(lg, pool2, K);
        for (int tries = 0; a2 != null && tries < 4 && !A3Guard.distinct(a1.judged, a2.judged); tries++) {
            Cand worst = null;
            double wv = -1;
            for (Cand c : a2.chosen) {
                double ov = A3Guard.nearShare(d1, Arrays.asList(c.offPts));
                if (ov > wv) {
                    wv = ov;
                    worst = c;
                }
            }
            pool2.remove(worst);
            a2 = pick(lg, pool2, K);
        }
        if (a2 != null && !A3Guard.distinct(a1.judged, a2.judged)) a2 = null;
        return new Pick[]{a1, a2};
    }

    static Cand shallow(Cand c) {
        Cand x = new Cand();
        x.side = c.side;
        x.i = c.i;
        x.j = c.j;
        x.keys = c.keys;
        x.geom = c.geom;
        x.offPts = c.offPts;
        x.len = c.len;
        x.span = c.span;
        x.extra = c.extra;
        x.cost = c.cost;
        x.repCost = c.repCost;
        x.costRatio = c.costRatio;
        x.maxD = c.maxD;
        x.meanD = c.meanD;
        x.value = c.value;
        x.habitDev = c.habitDev;
        x.habitRep = c.habitRep;
        x.novel = c.novel;
        x.sharp = c.sharp;
        x.a0 = c.a0;
        x.a1 = c.a1;
        x.edges = c.edges;
        return x;
    }
}
