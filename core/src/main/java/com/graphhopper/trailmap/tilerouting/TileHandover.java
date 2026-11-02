package com.graphhopper.trailmap.tilerouting;

import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.ResponsePath;
import com.graphhopper.trailmap.tilerouting.foundation.Foundation;
import com.graphhopper.trailmap.tilerouting.foundation.Via;
import com.graphhopper.util.PointList;
import com.graphhopper.util.details.PathDetail;
import com.graphhopper.util.shapes.GHPoint;

import java.util.*;

/**
 * Hands a tour to the app the way the app rebuilds it (W21, algo/REROUTE_REQUIREMENT.md): the app stores waypoints and
 * one profile per segment and re-routes every segment alone with plain /route, without the safety level's custom
 * model. Where that plain route leaves the tour's leg (a strict / no_trails leg that would take the banned trails
 * back), visible "pin" waypoints are added on the leg until plain /route follows it edge for edge.
 * <p>
 * Pins sit mid-edge, never on a node: at a node GraphHopper routes the two legs separately and charges no turn
 * there, so a pin on a node doesn't hold the route (lead_routing, alternative_route re-creation, 2026-10-31). The
 * search is the /convert normaliser's (WaypointNormalizer): from the last pin, the farthest edge midpoint that plain
 * routing still reaches along the leg, by doubling then bisecting. A segment that can't be pinned is reported with
 * its metres, never turned into something else (owner, 2026-10-31: "I first want to see what cannot be pinned!").
 */
public final class TileHandover {

    /** A segment plain routing doesn't reproduce even with pins: the plan's segment index and its metres off the leg. */
    public static final class Flag {
        public int segment;
        public double offM;
        public String reason;
    }

    public static final class Stats {
        public int segments, pinnedSegments, pins, probes;
        /** Metres of leg each pin guides (pin to the next waypoint), for the owner's visible vs hidden question. */
        public List<Double> guidedM = new ArrayList<>();
        public List<Flag> flags = new ArrayList<>();
    }

    /** One routed leg: its edge intervals and points. */
    static final class Leg {
        List<int[]> edges = new ArrayList<>(); // [fromPoint, toPoint, edgeId]
        PointList points;
        double distanceM;
        String error;

        int[] ids() {
            int[] out = new int[edges.size()];
            int n = 0;
            for (int[] e : edges) if (n == 0 || out[n - 1] != e[2]) out[n++] = e[2];
            return Arrays.copyOf(out, n);
        }
    }

    private final Foundation f;
    public final Stats stats = new Stats();

    public TileHandover(Foundation f) {
        this.f = f;
    }

    /** The plan with pins added to every followRoads segment plain routing doesn't reproduce. */
    public Foundation.Plan pin(Foundation.Plan plan) {
        Foundation.Plan out = new Foundation.Plan();
        out.walks.addAll(plan.walks);
        out.vias.add(plan.vias.get(0));
        for (int i = 0; i + 1 < plan.vias.size(); i++) {
            Via a = plan.vias.get(i), b = plan.vias.get(i + 1);
            String type = plan.segments.get(i);
            if (Foundation.FOLLOW_ROADS.equals(type)) {
                stats.segments++;
                // the segment's index in the response (after earlier pins): a flagged segment gets no pins, so it stays this
                for (Via p : pins(out.segments.size(), a, b)) {
                    out.vias.add(p);
                    out.segments.add(Foundation.FOLLOW_ROADS);
                }
            }
            out.segments.add(type);
            out.vias.add(b);
        }
        return out;
    }

    private List<Via> pins(int seg, Via a, Via b) {
        Leg want = route(a.lat, a.lon, b.lat, b.lon, true);
        if (want.error != null) return flag(seg, Double.NaN, "the tour's own leg doesn't route: " + want.error);
        int[] wantIds = want.ids();
        Leg plain = route(a.lat, a.lon, b.lat, b.lon, false);
        if (plain.error == null && Arrays.equals(plain.ids(), wantIds)) return List.of();

        // candidates: the midpoint of each of the leg's edge intervals (1..n-1), then the leg end
        int n = want.edges.size();
        double[][] cand = new double[n + 1][];
        for (int k = 1; k < n; k++) cand[k] = midpoint(want, want.edges.get(k));
        cand[n] = new double[]{b.lat, b.lon};

        List<Via> pins = new ArrayList<>();
        int cursor = 0;
        double[] from = {a.lat, a.lon};
        while (cursor < n) {
            int ok = -1, tried = cursor, step = 1;
            while (true) {
                int j = Math.min(cursor + step, n);
                if (j == tried) break;
                tried = j;
                if (!reproduces(from, cand[j], want, cursor, j)) break;
                ok = j;
                if (j == n) break;
                step *= 2;
            }
            if (ok < 0)
                return flag(seg, rebuild(want, a, b, List.of()).offM,
                        "plain /route doesn't follow the leg even between neighbouring edge midpoints (edge interval " + cursor + " of " + n + ")");
            if (tried > ok && ok < n) {
                int lo = ok + 1, hi = tried - 1;
                while (lo <= hi) {
                    int mid = (lo + hi) >>> 1;
                    if (reproduces(from, cand[mid], want, cursor, mid)) {
                        ok = mid;
                        lo = mid + 1;
                    } else hi = mid - 1;
                }
            }
            if (ok == n) break;
            double[] p = cand[ok];
            pins.add(new Via(Math.round(p[0] * 1e6) / 1e6, Math.round(p[1] * 1e6) / 1e6, Double.NaN, "pin", null));
            from = new double[]{pins.get(pins.size() - 1).lat, pins.get(pins.size() - 1).lon};
            cursor = ok;
        }
        // the pinned segment as the app rebuilds it, piece by piece
        Rebuilt rb = rebuild(want, a, b, pins);
        if (!rb.exact) return flag(seg, rb.offM, "pinned pieces still differ from the leg");
        stats.pinnedSegments++;
        stats.pins += pins.size();
        // each pin guides the piece from it to the next waypoint
        for (int k = 1; k < rb.pieceM.size(); k++) stats.guidedM.add((double) Math.round(rb.pieceM.get(k)));
        return pins;
    }

    static final class Rebuilt {
        boolean exact;
        double offM;
        List<Double> pieceM = new ArrayList<>();
    }

    /**
     * The segment as the app rebuilds it (plain, piece by piece through the pins): exact when its collapsed edge ids
     * equal the leg's; offM = its metres on edges the leg doesn't use (recreate.mjs's onlyReplayM).
     */
    private Rebuilt rebuild(Leg want, Via a, Via b, List<Via> pins) {
        Rebuilt r = new Rebuilt();
        Set<Integer> wantSet = new HashSet<>();
        for (int[] e : want.edges) wantSet.add(e[2]);
        List<double[]> pts = new ArrayList<>();
        pts.add(new double[]{a.lat, a.lon});
        for (Via p : pins) pts.add(new double[]{p.lat, p.lon});
        pts.add(new double[]{b.lat, b.lon});
        List<int[]> all = new ArrayList<>();
        for (int k = 0; k + 1 < pts.size(); k++) {
            Leg l = route(pts.get(k)[0], pts.get(k)[1], pts.get(k + 1)[0], pts.get(k + 1)[1], false);
            if (l.error != null) {
                r.offM = Double.POSITIVE_INFINITY;
                return r;
            }
            r.pieceM.add(l.distanceM);
            for (int[] e : l.edges) {
                all.add(e);
                if (!wantSet.contains(e[2])) r.offM += length(l.points, e[0], e[1]);
            }
        }
        r.exact = Arrays.equals(collapse(all), want.ids());
        return r;
    }

    private List<Via> flag(int seg, double offM, String reason) {
        Flag fl = new Flag();
        fl.segment = seg;
        fl.offM = offM;
        fl.reason = reason;
        stats.flags.add(fl);
        return List.of();
    }

    /** Does plain routing from p to q follow the leg's edge intervals cursor..j? */
    private boolean reproduces(double[] p, double[] q, Leg want, int cursor, int j) {
        stats.probes++;
        Leg got = route(p[0], p[1], q[0], q[1], false);
        if (got.error != null) return false;
        int[] exp = collapse(want.edges.subList(cursor, Math.min(j + 1, want.edges.size())));
        return Arrays.equals(got.ids(), exp);
    }

    private static int[] collapse(List<int[]> edges) {
        int[] out = new int[edges.size()];
        int n = 0;
        for (int[] e : edges) if (n == 0 || out[n - 1] != e[2]) out[n++] = e[2];
        return Arrays.copyOf(out, n);
    }

    private static double length(PointList pl, int from, int to) {
        double m = 0;
        for (int i = from; i < to; i++) m += Foundation.dist(pl.getLat(i), pl.getLon(i), pl.getLat(i + 1), pl.getLon(i + 1));
        return m;
    }

    /** The point halfway along an edge interval of the leg (by length): on the edge, off its nodes. */
    private static double[] midpoint(Leg leg, int[] e) {
        PointList pl = leg.points;
        double half = length(pl, e[0], e[1]) / 2, s = 0;
        for (int i = e[0]; i < e[1]; i++) {
            double d = Foundation.dist(pl.getLat(i), pl.getLon(i), pl.getLat(i + 1), pl.getLon(i + 1));
            if (s + d >= half && d > 0) {
                double t = (half - s) / d;
                return new double[]{pl.getLat(i) + t * (pl.getLat(i + 1) - pl.getLat(i)), pl.getLon(i) + t * (pl.getLon(i + 1) - pl.getLon(i))};
            }
            s += d;
        }
        return new double[]{pl.getLat(e[0]), pl.getLon(e[0])};
    }

    /**
     * One segment as the app routes it (shared gh-routing-service.ts: points, profile, snap_preventions ferry, no
     * heading), with the safety model when {@code tour} (the tour's own leg) and without it otherwise.
     */
    Leg route(double aLat, double aLon, double bLat, double bLon, boolean tour) {
        GHRequest req = new GHRequest(new GHPoint(aLat, aLon), new GHPoint(bLat, bLon)).setProfile(f.profile);
        req.setSnapPreventions(List.of("ferry"));
        req.setPathDetails(List.of("edge_id"));
        req.putHint("instructions", false);
        if (tour && f.customModel != null) req.setCustomModel(f.customModel);
        GHResponse rsp = f.hopper.route(req);
        Leg l = new Leg();
        if (rsp.hasErrors()) {
            l.error = rsp.getErrors().toString();
            return l;
        }
        ResponsePath p = rsp.getBest();
        l.points = p.getPoints();
        l.distanceM = p.getDistance();
        for (PathDetail d : p.getPathDetails().getOrDefault("edge_id", List.of()))
            if (d.getLast() > d.getFirst()) l.edges.add(new int[]{d.getFirst(), d.getLast(), ((Number) d.getValue()).intValue()});
        return l;
    }
}
