/*
 * Trailmap - Personal routing, "bit of new" variations
 */
package com.graphhopper.trailmap.personal.variations;

import com.graphhopper.ResponsePath;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.trailmap.personal.PersonalFamiliarity;
import com.graphhopper.trailmap.personal.PersonalParams;
import com.graphhopper.trailmap.personal.PersonalWeighting;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.FetchMode;
import com.graphhopper.util.PMap;
import com.graphhopper.util.details.PathDetail;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

/**
 * "Bit of new": variations of a main route that leave it for a while and come back, from two generators run
 * side by side, unchanged from their prototypes: {@link R4Generator} (round 4, up to two alternatives) and
 * {@link A3Generator} (round-3 A, up to two alternatives). An A3 alternative is dropped only when it is geometrically
 * the same as an earlier path: at least {@link #DUP_SHARE} of its off-main length lies within {@link #DUP_M} of one
 * earlier path (main, a round-4 alternative or an earlier A3 one).
 *
 * <p>The main route must carry the {@code edge_id} path detail ({@link MainPath#EDGE_ID}).
 */
public final class BitOfNew {

    /** A3 duplicate test: share of its off-main length near one earlier path, and the distance that counts as near. */
    public static final double DUP_SHARE = 0.9;
    public static final double DUP_M = 30;

    public enum History {
        NONE, AVOID, EXPLORE;

        public static History of(String s) {
            try {
                return valueOf((s == null || s.isBlank() ? "none" : s.trim()).toUpperCase(Locale.ROOT));
            } catch (IllegalArgumentException e) {
                throw new IllegalArgumentException("bit_of_new_history must be one of none, avoid, explore");
            }
        }

        public String key() {
            return name().toLowerCase(Locale.ROOT);
        }
    }

    /** One deviation of a variation from the main route. */
    public static final class Deviation {
        public double fromPct, toPct, lengthM, extraM, maxDistM;
    }

    /** One variation: the whole route A to B ({lon, lat(, ele)}), its source ("round4" or "a3") and its deviations. */
    public static final class Variation {
        public String source;
        public List<double[]> coords;
        public List<Deviation> deviations = new ArrayList<>();
        public double lengthM;
        /**
         * The variation's edges in travel order as an {@code edge_id} path detail over {@link #coords}: the main route's
         * entries outside the deviations, the deviations' edges inside them. null when the indices don't add up to
         * the coordinates (then nothing is claimed about the variation's edges).
         */
        public List<PathDetail> edgeIds;
    }

    /** A deviation as both generators splice it: main node i to main node j over edge keys. */
    private record Detour(int i, int j, int[] keys) {
    }

    public static final class Result {
        public History history;
        /** round-4 alternatives, in the generator's order */
        public final List<Variation> round4 = new ArrayList<>();
        /** A3 alternatives as the generator gave them (before the duplicate test) */
        public final List<Variation> a3 = new ArrayList<>();
        /** the offered alternatives: round 4's, then A3's that are not duplicates */
        public final List<Variation> offered = new ArrayList<>();
        public int a3Duplicates;
        public double msRound4, msA3, msTotal;
    }

    private BitOfNew() {
    }

    /**
     * @param profileW the profile's weighting, without personal adjustments (quality is judged on it)
     * @param lifetime the user's lifetime history (needed for avoid), may be null otherwise
     * @param last12   the user's last-12-month history (needed for explore), may be null otherwise
     */
    public static Result compute(BaseGraph g, EncodingManager em, LocationIndex locationIndex, Weighting profileW,
                                 ResponsePath main, History history, PersonalFamiliarity lifetime, PersonalFamiliarity last12) {
        if (history == History.AVOID && lifetime == null)
            throw new IllegalArgumentException("bit_of_new_history=avoid needs the lifetime history");
        if (history == History.EXPLORE && last12 == null)
            throw new IllegalArgumentException("bit_of_new_history=explore needs the last-12-month history");
        long t0 = System.nanoTime();
        Result res = new Result();
        res.history = history;
        MainPath mp = MainPath.from(g, main);
        String flavour = history.key();

        long t1 = System.nanoTime();
        Weighting earW = profileW;
        if (history == History.AVOID) earW = new PersonalWeighting(profileW, lifetime, flavourParams("avoid"));
        else if (history == History.EXPLORE) earW = new PersonalWeighting(profileW, last12, flavourParams("explore"));
        PersonalFamiliarity r4life = history == History.NONE ? null : lifetime;
        PersonalFamiliarity r4last12 = history == History.NONE ? null : last12;
        R4Generator r4 = new R4Generator(g, em, locationIndex, profileW, earW, r4life, r4last12, flavour);
        R4Generator.Alt[] pair = r4.run(mp);
        double L4 = r4.mainLength();
        for (R4Generator.Alt a : pair) {
            if (a == null) continue;
            Variation v = new Variation();
            v.source = "round4";
            v.coords = a.coords;
            for (R4Generator.Ear e : a.ears) v.deviations.add(dev(e.a0, e.a1, e.len, e.span, e.maxD, L4));
            List<Detour> ds = new ArrayList<>();
            for (R4Generator.Ear e : a.ears) ds.add(new Detour(e.i, e.j, e.keys));
            v.edgeIds = spliceEdgeIds(g, main, mp, ds, v.coords.size());
            res.round4.add(v);
        }
        long t2 = System.nanoTime();

        A3Generator a3 = new A3Generator(g, profileW, lifetime, last12, flavour);
        A3Generator.Result ar = a3.run(mp);
        for (A3Generator.Pick p : ar.alts) {
            Variation v = new Variation();
            v.source = "a3";
            v.coords = p.coords;
            List<A3Generator.Cand> ch = new ArrayList<>(p.chosen);
            ch.sort((x, y) -> Double.compare(x.a0, y.a0));
            for (A3Generator.Cand c : ch) v.deviations.add(dev(c.a0, c.a1, c.len, c.span, c.maxD, ar.L));
            List<Detour> ds = new ArrayList<>();
            for (A3Generator.Cand c : ch) ds.add(new Detour(c.i, c.j, c.keys));
            v.edgeIds = spliceEdgeIds(g, main, mp, ds, v.coords.size());
            res.a3.add(v);
        }
        long t3 = System.nanoTime();

        // offered: round 4's, then A3's unless geometrically the same as an earlier path
        R4Guard gd = new R4Guard(mp.coords);
        List<double[][]> earlier = new ArrayList<>();
        earlier.add(gd.mainP);
        for (Variation v : res.round4) {
            res.offered.add(v);
            earlier.add(gd.resample(v.coords));
        }
        for (Variation v : res.a3) {
            double[][] P = gd.resample(v.coords);
            if (sameAsEarlier(gd, P, earlier)) {
                res.a3Duplicates++;
                continue;
            }
            res.offered.add(v);
            earlier.add(P);
        }
        for (Variation v : res.offered) v.lengthM = lengthM(v.coords);
        long t4 = System.nanoTime();
        res.msRound4 = (t2 - t1) / 1e6;
        res.msA3 = (t3 - t2) / 1e6;
        res.msTotal = (t4 - t0) / 1e6;
        return res;
    }

    /** The detour searches' personal weighting for avoid / explore (strength 1, no switch penalty, as the prototypes). */
    static PersonalParams flavourParams(String mode) {
        PMap h = new PMap();
        h.putObject(PersonalParams.MODE, mode);
        h.putObject(PersonalParams.USER, "bit_of_new");
        h.putObject(PersonalParams.CELLS, "history");
        h.putObject(PersonalParams.STRENGTH, 1.0);
        h.putObject(PersonalParams.SWITCH_PENALTY, 0.0);
        h.putObject(PersonalParams.RIDES_SATURATION, 10.0);
        return PersonalParams.from(h);
    }

    /**
     * The edge_id path detail of a variation spliced as R4Generator.splice / A3Generator.splice splice its points:
     * main points up to node i, then each deviation edge's pillar and adjacent points (the last edge's adjacent point
     * is main node j's point), then the main points from node j on. Main entry k ends at node k, so entries 0..i stay,
     * the deviation's edges follow, and entries j+1.. continue shifted. null when the result does not end at the last
     * of {@code nPoints}.
     */
    static List<PathDetail> spliceEdgeIds(BaseGraph g, ResponsePath main, MainPath mp, List<Detour> detours, int nPoints) {
        List<PathDetail> det = main.getPathDetails().get(MainPath.EDGE_ID);
        List<Detour> s = new ArrayList<>(detours);
        s.sort(Comparator.comparingInt(Detour::i));
        List<PathDetail> out = new ArrayList<>();
        int k = 0, shift = 0;
        for (Detour d : s) {
            if (d.i < k - 1 || d.j < d.i || d.j + 1 > det.size()) return null;
            for (; k <= d.i; k++) out.add(shifted(det.get(k), shift));
            int at = mp.ptIdx[d.i] + shift;
            for (int key : d.keys) {
                EdgeIteratorState e = g.getEdgeIteratorStateForKey(key);
                int n = e.fetchWayGeometry(FetchMode.PILLAR_AND_ADJ).size();
                PathDetail pd = new PathDetail(e.getEdge());
                pd.setFirst(at);
                pd.setLast(at + n);
                out.add(pd);
                at += n;
            }
            shift = at - mp.ptIdx[d.j];
            k = d.j + 1;
        }
        for (; k < det.size(); k++) out.add(shifted(det.get(k), shift));
        if (out.isEmpty() || out.get(out.size() - 1).getLast() != nPoints - 1) return null;
        return out;
    }

    private static PathDetail shifted(PathDetail d, int shift) {
        PathDetail pd = new PathDetail(d.getValue());
        pd.setFirst(d.getFirst() + shift);
        pd.setLast(d.getLast() + shift);
        return pd;
    }

    static Deviation dev(double a0, double a1, double len, double span, double maxD, double L) {
        Deviation d = new Deviation();
        d.fromPct = 100 * a0 / L;
        d.toPct = 100 * a1 / L;
        d.lengthM = len;
        d.extraM = len - span;
        d.maxDistM = maxD;
        return d;
    }

    /** at least DUP_SHARE of P's off-main samples lie within DUP_M of one earlier path. */
    static boolean sameAsEarlier(R4Guard gd, double[][] P, List<double[][]> earlier) {
        List<double[]> off = new ArrayList<>();
        for (double[] p : P) if (gd.offMain(p)) off.add(p);
        if (off.isEmpty()) return true; // nothing off main: the main route itself
        for (double[][] E : earlier) {
            R4Guard.PtIndex idx = new R4Guard.PtIndex(E, DUP_M);
            int near = 0;
            for (double[] p : off) if (idx.near(p)[0] <= DUP_M) near++;
            if (near >= DUP_SHARE * off.size()) return true;
        }
        return false;
    }

    public static double lengthM(List<double[]> c) {
        double L = 0;
        for (int i = 0; i < c.size() - 1; i++)
            L += DistanceCalcEarth.DIST_EARTH.calcDist(c.get(i)[1], c.get(i)[0], c.get(i + 1)[1], c.get(i + 1)[0]);
        return L;
    }
}
