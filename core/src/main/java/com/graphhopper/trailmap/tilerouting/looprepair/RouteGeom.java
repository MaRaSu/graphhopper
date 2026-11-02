package com.graphhopper.trailmap.tilerouting.looprepair;

import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.ResponsePath;
import com.graphhopper.trailmap.tilerouting.foundation.Foundation;
import com.graphhopper.util.Instruction;
import com.graphhopper.util.PointList;
import com.graphhopper.util.shapes.GHPoint;

import java.util.*;

/**
 * The real route of a candidate-2 state: legs between consecutive plain via points, each routed in-process by
 * GraphHopper exactly as the verifier's single /route call does (no headings, no pass_through: every leg is
 * independent), cached by (from, to) location. The objective is read off the assembled geometry.
 * <p>
 * Round 2 (variety lever): the objective is the gravel profile's own route weight (what /route minimises, summed
 * over the legs) plus the out-and-back, u-turn and miss terms in the same weight units. Each term is metres × the
 * route's reference weight per metre {@code wpm} (the seed route's weight / length, fixed per job so a hard
 * candidate can't inflate its own credit): out-and-back costs λ · wpm · obM, so removing X m of out-and-back earns
 * at most λ × the weight of riding X m on the route's own roads, and never makes a hard trail look cheap.
 * λ = variety off | mild | strong (tr.c2.variety; values tr.c2.lambda.off|mild|strong = 0 / 0.3 / 0.7).
 * tr.c2.objective=metres restores the round-1 geometric objective (len + obLambda·obM + ...).
 */
final class RouteGeom {

    static final class Leg {
        List<double[]> coords = new ArrayList<>();
        double distM = Double.POSITIVE_INFINITY, weight = Double.POSITIVE_INFINITY;
        int uTurnInstr;
        String error;
    }

    /** Objective levers (the prototype's defaults: tr.c2.objective=weight, obLambda 0.5, uTurnM 300, missM 1e7). */
    static final class Weights {
        final boolean metres = false;
        final String variety;
        double lambda;
        double obLambda = 0.5;
        double uTurnM = 300;
        double missM = 10_000_000;
        /** Reference weight per metre; set from the seed by {@link RouteGeom#calibrate}. */
        double wpm = Double.NaN;

        Weights(String variety) {
            this.variety = variety;
            // the prototype's tr.c2.lambda.<variety> defaults
            if ("off".equals(variety)) lambda = 0;
            else if ("mild".equals(variety)) lambda = 0.3;
            else if ("strong".equals(variety)) lambda = 0.7;
            else throw new IllegalArgumentException("variety must be off|mild|strong, got " + variety);
        }

        @Override
        public String toString() {
            return metres ? String.format(Locale.ROOT, "objective=metres obLambda=%.2f uTurnM=%.0f missM=%.0f", obLambda, uTurnM, missM)
                    : String.format(Locale.ROOT, "objective=weight variety=%s lambda=%.2f wpm=%.4f uTurnM=%.0f missM=%.0f", variety, lambda, wpm, uTurnM, missM);
        }
    }

    static final class Eval {
        double objective = Double.POSITIVE_INFINITY, lengthM, obM, weight, walkW;
        int uTurns, viaReversals, missed;
        BitSet fetched = new BitSet();
        List<double[]> coords;

        @Override
        public String toString() {
            return String.format(Locale.ROOT, "obj=%.0f w=%.0f walkW=%.0f len=%.2fkm ob=%.2fkm u=%d+%d missed=%d", objective, weight, walkW, lengthM / 1000,
                    obM / 1000, uTurns, viaReversals, missed);
        }
    }

    private final Foundation f;
    private final List<PlainCostTable.Loc> locs;
    private final List<Foundation.TileInfo> tiles;
    private final Weights w;
    private final Map<Long, Leg> cache = new HashMap<>();
    long legsRouted, routeNanos, evals;

    RouteGeom(Foundation f, List<PlainCostTable.Loc> locs, List<Foundation.TileInfo> tiles, Weights w) {
        this.f = f;
        this.locs = locs;
        this.tiles = tiles;
        this.w = w;
    }

    Leg leg(int a, int b) {
        long key = (long) a * 1_000_003L + b;
        Leg l = cache.get(key);
        if (l != null) return l;
        long t0 = System.nanoTime();
        l = new Leg();
        PlainCostTable.Loc la = locs.get(a), lb = locs.get(b);
        GHRequest req = new GHRequest(Arrays.asList(new GHPoint(la.lat, la.lon), new GHPoint(lb.lat, lb.lon))).setProfile(f.profile);
        // the job's safety level (round 2): the same per-request custom model the verifier sends
        if (f.customModel != null) req.setCustomModel(f.customModel);
        GHResponse res = f.hopper.route(req);
        if (res.hasErrors()) l.error = res.getErrors().toString();
        else {
            ResponsePath p = res.getBest();
            PointList pl = p.getPoints();
            for (int i = 0; i < pl.size(); i++) l.coords.add(new double[]{pl.getLat(i), pl.getLon(i)});
            l.distM = p.getDistance();
            l.weight = p.getRouteWeight();
            for (Instruction in : p.getInstructions()) {
                int s = in.getSign();
                if (s == Instruction.U_TURN_UNKNOWN || s == Instruction.U_TURN_LEFT || s == Instruction.U_TURN_RIGHT) l.uTurnInstr++;
            }
        }
        routeNanos += System.nanoTime() - t0;
        legsRouted++;
        cache.put(key, l);
        return l;
    }

    Eval eval(List<Integer> stops) {
        evals++;
        Eval e = new Eval();
        List<double[]> coords = new ArrayList<>();
        List<Integer> joints = new ArrayList<>();
        double len = 0, wt = 0;
        int u = 0;
        for (int i = 0; i + 1 < stops.size(); i++) {
            Leg l = leg(stops.get(i), stops.get(i + 1));
            if (l.error != null) return e;
            len += l.distM;
            wt += l.weight;
            u += l.uTurnInstr;
            if (i > 0) joints.add(coords.size() - 1);
            for (int k = coords.isEmpty() ? 0 : 1; k < l.coords.size(); k++) coords.add(l.coords.get(k));
            if (l.coords.size() == 1 && coords.isEmpty()) coords.add(l.coords.get(0));
        }
        e.coords = coords;
        e.lengthM = len;
        e.weight = wt;
        e.uTurns = u;
        for (int j : joints) if (reversesAt(coords, j, 30, 150)) e.viaReversals++;
        // walk tiles (round 3): no rideable way enters one, so the routed line never fetches it; a stop on one of its
        // walk locations (road point A, walkM > 0) does, by the walk A → T → A the export adds (priced like c1/c3)
        double walkW = 0;
        for (int s : stops) {
            PlainCostTable.Loc l = locs.get(s);
            if (l.tileKey != null && !l.states.isEmpty() && l.states.get(0).walkM > 0)
                walkW += 2 * l.states.get(0).walkM * f.params.walkWeightPerM;
        }
        e.walkW = walkW;
        for (int t = 0; t < tiles.size(); t++) {
            Foundation.TileInfo ti = tiles.get(t);
            boolean got = ti.walk
                    ? stops.stream().anyMatch(s -> ti.tile.key.equals(locs.get(s).tileKey) && !locs.get(s).states.isEmpty() && locs.get(s).states.get(0).walkM > 0)
                    : Foundation.fetched(ti.tile, coords);
            if (got) e.fetched.set(t);
        }
        e.missed = tiles.size() - e.fetched.cardinality();
        e.obM = coords.size() < 2 ? 0 : Overlap.outAndBackM(coords);
        double wpm = Double.isFinite(w.wpm) ? w.wpm : (len > 0 ? wt / len : 1);
        if (w.metres) e.objective = len + walkW / wpm + w.obLambda * e.obM + w.uTurnM * (e.uTurns + e.viaReversals) + w.missM * e.missed;
        else e.objective = wt + walkW + wpm * (w.lambda * e.obM + w.uTurnM * (e.uTurns + e.viaReversals) + w.missM * e.missed);
        return e;
    }

    /** Fix the reference weight per metre from the seed route (once per job), then re-score it. */
    Eval calibrate(List<Integer> seedStops, Eval seedEval) {
        if (w.metres || Double.isFinite(w.wpm) || !(seedEval.lengthM > 0) || !Double.isFinite(seedEval.weight)) return seedEval;
        w.wpm = seedEval.weight / seedEval.lengthM;
        return eval(seedStops);
    }

    /** The verifier's via reversal: bearing over 30 m before vs 30 m after the joint turns ≥ 150°. */
    static boolean reversesAt(List<double[]> c, int i, double armM, double minDeg) {
        double[] a = along(c, i, -1, armM), b = along(c, i, +1, armM);
        if (a == null || b == null) return false;
        double[] p = c.get(i);
        double h1 = Foundation.bearing(a[0], a[1], p[0], p[1]), h2 = Foundation.bearing(p[0], p[1], b[0], b[1]);
        double d = Math.abs(h1 - h2) % 360;
        if (d > 180) d = 360 - d;
        return d >= minDeg;
    }

    private static double[] along(List<double[]> c, int i, int dir, double m) {
        double acc = 0;
        int k = i;
        while (true) {
            int nk = k + dir;
            if (nk < 0 || nk >= c.size()) return acc > 0 ? c.get(k) : null;
            double[] p = c.get(k), q = c.get(nk);
            double d = Foundation.dist(p[0], p[1], q[0], q[1]);
            if (acc + d >= m) {
                double t = d > 0 ? (m - acc) / d : 0;
                return new double[]{p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])};
            }
            acc += d;
            k = nk;
        }
    }
}
