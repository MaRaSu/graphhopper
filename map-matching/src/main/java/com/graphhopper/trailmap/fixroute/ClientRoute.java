package com.graphhopper.trailmap.fixroute;

import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.ResponsePath;
import com.graphhopper.trailmap.shared.EdgeKeyMatching;
import com.graphhopper.util.CustomModel;
import com.graphhopper.util.PointList;
import com.graphhopper.util.details.PathDetail;
import com.graphhopper.util.shapes.GHPoint;

import java.util.ArrayList;
import java.util.List;

/**
 * Routes one (sub-)leg <b>exactly as the client app sends it to {@code /route}</b> — the parity
 * rule (design doc §2.1). Every route {@code /fix_route} uses to accept anything goes through here.
 *
 * <p>Client request, verified in the client code (2026-09-24): points {@code [start, …via, end]} in
 * one call; the leg's profile; its custom model verbatim; {@code snap_preventions} as sent (always
 * {@code ["ferry"]} today); when the leg has a heading, {@code headings = [heading, NaN, …]} plus
 * {@code heading_penalty}; nothing else that affects the path (no pass_through, curbside, algorithm).
 * The client's {@code elevation}, {@code details}, locale and point encoding do not change the path.
 */
public final class ClientRoute {

    private ClientRoute() {
    }

    /** Routing settings of one leg, as the client holds them. */
    /**
     * {@code barriers}: the W26 flag. {@code crossable}: the barriers ("lat,lon" at 1e-6) a probe may cross, those the
     * leg's own route (the app's /route + flag answer) crosses; empty = none.
     */
    public record Settings(String profile, CustomModel customModel, List<String> snapPreventions, boolean barriers,
                           java.util.Set<String> crossable) {
        public Settings(String profile, CustomModel customModel, List<String> snapPreventions) {
            this(profile, customModel, snapPreventions, false, java.util.Set.of());
        }

        public Settings(String profile, CustomModel customModel, List<String> snapPreventions, boolean barriers) {
            this(profile, customModel, snapPreventions, barriers, java.util.Set.of());
        }

        public Settings withCrossable(java.util.Set<String> crossable) {
            return new Settings(profile, customModel, snapPreventions, barriers, crossable);
        }
    }

    /** One routed leg: geometry, directed edge keys, length, exit bearing. Null fields on failure. */
    /** {@code keyLengthsM[i]}: length [m] travelled on dedup'd {@code edgeKeys[i]}. */
    public record Leg(boolean ok, String error, List<double[]> points, int[] edgeKeys,
                      double distanceM, double exitHeading, double firstEdgeM, double lastEdgeM, double[] keyLengthsM) {
        public Leg(boolean ok, String error, List<double[]> points, int[] edgeKeys, double distanceM, double exitHeading) {
            this(ok, error, points, edgeKeys, distanceM, exitHeading, Double.NaN, Double.NaN, null);
        }
    }

    public static GHRequest request(Settings s, GHPoint start, List<GHPoint> via, GHPoint end,
                                    Double heading, Double headingPenalty) {
        GHRequest req = new GHRequest();
        req.addPoint(start);
        if (via != null) for (GHPoint v : via) req.addPoint(v);
        req.addPoint(end);
        req.setProfile(s.profile());
        if (s.customModel() != null) req.setCustomModel(s.customModel());
        // Always set explicitly: the web layer would otherwise substitute the server's configured
        // default when the client sends none. An empty list means "none" here as it does there.
        req.setSnapPreventions(s.snapPreventions() == null ? List.of() : s.snapPreventions());
        if (heading != null && !heading.isNaN()) {
            List<Double> headings = new ArrayList<>();
            headings.add(heading);
            for (int i = 1; i < req.getPoints().size(); i++) headings.add(Double.NaN);
            req.setHeadings(headings);
            if (headingPenalty != null) req.putHint("heading_penalty", headingPenalty);
        }
        req.setPathDetails(List.of("edge_key"));
        req.putHint("instructions", false);
        req.putHint("calc_points", true);
        // W26: as the app sends it with the flag (barrier-aware snapping, crossing search)
        if (s.barriers()) req.putHint(com.graphhopper.trailmap.barrier.BarrierRouter.HINT, com.graphhopper.trailmap.barrier.BarrierRouter.VERSION);
        return req;
    }

    /** Length [m] the route travels on its first (last) edge_key — consecutive same-key details merged. */
    private static double runLength(List<PathDetail> ek, List<double[]> pts, boolean first) {
        int n = ek.size();
        int i = first ? 0 : n - 1;
        Object key = ek.get(i).getValue();
        int from = ek.get(i).getFirst(), to = ek.get(i).getLast();
        if (first) {
            while (i + 1 < n && ek.get(i + 1).getValue().equals(key)) to = ek.get(++i).getLast();
        } else {
            while (i - 1 >= 0 && ek.get(i - 1).getValue().equals(key)) from = ek.get(--i).getFirst();
        }
        double d = 0;
        for (int k = from + 1; k <= to && k < pts.size(); k++) {
            double[] a = pts.get(k - 1), b = pts.get(k);
            d += com.graphhopper.util.DistanceCalcEarth.DIST_EARTH.calcDist(a[0], a[1], b[0], b[1]);
        }
        return d;
    }

    /** the barriers ("lat,lon") the response's main path crosses (hints.barriers.paths[0].crossings); empty if none */
    @SuppressWarnings("unchecked")
    private static java.util.Set<String> crossings(GHResponse rsp) {
        java.util.Set<String> out = new java.util.TreeSet<>();
        Object b = rsp.getHints().getObject(com.graphhopper.trailmap.barrier.BarrierRouter.HINT, null);
        if (!(b instanceof java.util.Map)) return out;
        Object paths = ((java.util.Map<String, Object>) b).get("paths");
        if (!(paths instanceof List) || ((List<?>) paths).isEmpty()) return out;
        Object c = ((java.util.Map<String, Object>) ((List<?>) paths).get(0)).get("crossings");
        if (!(c instanceof List)) return out;
        for (Object x : (List<?>) c)
            for (Object bar : (List<?>) ((java.util.Map<String, Object>) x).get("barriers")) {
                java.util.Map<String, Object> m = (java.util.Map<String, Object>) bar;
                out.add(m.get("lat") + "," + m.get("lon"));
            }
        return out;
    }

    /** W26: the barriers the app's /route + flag answer crosses on this leg (empty if it needs none, or fails) */
    public static java.util.Set<String> crossedBarriers(GraphHopper hopper, Settings s, GHPoint start, List<GHPoint> via, GHPoint end,
                                                        Double heading, Double headingPenalty) {
        try {
            GHResponse rsp = hopper.route(request(s, start, via, end, heading, headingPenalty));
            return rsp.hasErrors() ? java.util.Set.of() : crossings(rsp);
        } catch (Exception e) {
            return java.util.Set.of();
        }
    }

    public static Leg route(GraphHopper hopper, Settings s, GHPoint start, List<GHPoint> via, GHPoint end,
                            Double heading, Double headingPenalty) {
        return route(hopper, s, start, via, end, heading, headingPenalty, false);
    }

    /**
     * W26 (flag): the leg itself exactly as the app routes it with the flag, a crossing included (the app's
     * /route + flag answer: its "cross here / don't cross" dialog). Only for the leg's own route, never a repair probe.
     */
    public static Leg routeAsApp(GraphHopper hopper, Settings s, GHPoint start, List<GHPoint> via, GHPoint end,
                                 Double heading, Double headingPenalty) {
        return route(hopper, s, start, via, end, heading, headingPenalty, true);
    }

    private static Leg route(GraphHopper hopper, Settings s, GHPoint start, List<GHPoint> via, GHPoint end,
                             Double heading, Double headingPenalty, boolean acceptCrossing) {
        GHResponse rsp;
        try {
            rsp = hopper.route(request(s, start, via, end, heading, headingPenalty));
        } catch (Exception e) {
            return new Leg(false, e.getClass().getSimpleName() + ": " + e.getMessage(), null, null, 0, Double.NaN);
        }
        if (rsp.hasErrors()) {
            return new Leg(false, rsp.getErrors().toString(), null, null, 0, Double.NaN);
        }
        // W26: a route that needs a barrier crossing is no followRoads leg (the app would have to split it and ask):
        // not accepted, so the stretch gets the usual unroutable handling (unroutable_policy)
        if (s.barriers() && !acceptCrossing) {
            java.util.Set<String> c = crossings(rsp);
            // a probe may cross only the ask barriers the leg's own route crosses (the app's dialog covers those)
            if (!c.isEmpty() && !s.crossable().containsAll(c))
                return new Leg(false, "needs a barrier crossing", null, null, 0, Double.NaN);
        }
        ResponsePath path = rsp.getBest();
        PointList pl = path.getPoints();
        List<double[]> pts = new ArrayList<>(pl.size());
        for (int i = 0; i < pl.size(); i++) pts.add(new double[]{pl.getLat(i), pl.getLon(i)});
        List<PathDetail> ek = path.getPathDetails().get("edge_key");
        int[] keys = ek == null ? new int[0] : EdgeKeyMatching.edgeKeysFromDetails(ek);
        double first = Double.NaN, last = Double.NaN;
        double[] lens = new double[keys.length];
        if (ek != null && !ek.isEmpty()) {
            first = runLength(ek, pts, true);
            last = runLength(ek, pts, false);
            int k = -1;
            Object prev = null;
            for (PathDetail d : ek) {
                if (!d.getValue().equals(prev)) {
                    k++;
                    prev = d.getValue();
                }
                for (int p = d.getFirst() + 1; p <= d.getLast() && p < pts.size() && k < lens.length; p++) {
                    double[] a = pts.get(p - 1), b = pts.get(p);
                    lens[k] += com.graphhopper.util.DistanceCalcEarth.DIST_EARTH.calcDist(a[0], a[1], b[0], b[1]);
                }
            }
        }
        return new Leg(true, null, pts, keys, path.getDistance(), EdgeKeyMatching.exitHeadingOf(pl), first, last, lens);
    }
}
