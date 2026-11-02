/*
 * Trailmap - Personal routing
 */
package com.graphhopper.trailmap.personal;

import com.carrotsearch.hppc.IntArrayList;
import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.ResponsePath;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.FetchMode;
import com.graphhopper.util.PMap;
import com.graphhopper.util.PointList;
import com.graphhopper.util.details.PathDetail;
import com.graphhopper.util.shapes.GHPoint;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

/**
 * Via points that make plain {@code /route} ride a generated path again. The app stores a route as waypoints, a
 * profile and per-segment via points, and rebuilds every segment with plain {@code POST /route} [A, ...vias, B]; a
 * path from /personal_route must come back from exactly that call.
 *
 * <p>The pin step routes the plain request the app will send (same profile, custom model, snap preventions and
 * first-point heading; nothing personal) leg by leg: from A (later from the last via), the farthest edge of the path
 * that a plain leg still reaches along the path gets the next via, at its middle; until a leg reaches B or
 * {@code maxVias} vias are used. The app's whole request [A, ...vias, B] is then routed once more to say whether the
 * path is re-created. Vias are rounded to the response's polyline precision before they are tried, since the app
 * sends what it decoded, and the plain routes run from the path's own first to last point (as decoded): the app moves
 * the segment's waypoints there and rebuilds from them.
 *
 * <p>Never worse than the vias the path came with (alternative_route's): they are replayed first and kept when they
 * reproduce the path, when the time budget is gone, when nothing could be pinned, and when the pinned vias do not
 * re-create it either and do not come strictly closer (fewer metres missing plus extra).
 *
 * <p>Edges are compared as {@code edge_id} sequences with repeats collapsed (a via splits its edge in two halves that
 * carry the same id).
 */
public final class RecreateVias {

    public static final String EDGE_ID = "edge_id";

    /** The outcome for one path. */
    public static final class Result {
        /** the vias, in travel order (empty: A to B alone reproduces the path) */
        public PointList vias;
        /** true when plain /route [A, ...vias, B] rides exactly the path's edges */
        public boolean identical;
        /**
         * metres (along the path's own points) of the path's edge ids the rebuilt route does not ride at all, and of the
         * rebuilt route's edge ids the path does not; whether it is re-created is decided on the edge sequence
         */
        public double missingM, extraM;
        /** plain routes computed */
        public int routes;
        public double ms;
        /** why pinning stopped early, or null */
        public String stopped;
        /** true when {@link #vias} are the vias the path came with (rounded), kept */
        public boolean keptGiven;

        public Map<String, Object> toMap() {
            Map<String, Object> m = new java.util.LinkedHashMap<>();
            m.put("identical", identical);
            m.put("vias", vias == null ? 0 : vias.size());
            m.put("missing_m", Math.round(missingM));
            m.put("extra_m", Math.round(extraM));
            m.put("routes", routes);
            m.put("ms", Math.round(ms));
            if (stopped != null) m.put("stopped", stopped);
            return m;
        }
    }

    private final BaseGraph graph;
    private final LocationIndex locationIndex;
    /** plain /route as the server answers it */
    private final Function<GHRequest, GHResponse> router;
    private final GHRequest template;
    private final int maxVias;
    private final double multiplier;
    private final long deadlineNanos;
    /** the last plain route computed (after {@link #route}: the app's whole request) */
    private ResponsePath lastPath;
    /** where the app's rebuild starts and ends: the shown path's first and last point, as decoded */
    private GHPoint a, b;

    /**
     * @param template   the plain request the app sends for this segment: two points (see {@link #plainRequest})
     * @param multiplier the response's polyline multiplier (vias are rounded to it), or 0 for unrounded vias
     * @param deadline   System.nanoTime() after which no further plain route is started
     */
    public RecreateVias(GraphHopper hopper, GHRequest template, int maxVias, double multiplier, long deadline) {
        this(hopper.getBaseGraph(), hopper.getLocationIndex(), hopper::route, template, maxVias, multiplier, deadline);
    }

    public RecreateVias(BaseGraph graph, LocationIndex locationIndex, Function<GHRequest, GHResponse> router,
                        GHRequest template, int maxVias, double multiplier, long deadline) {
        if (template.getPoints().size() != 2)
            throw new IllegalArgumentException("re-create via points need exactly two points (A to B)");
        this.graph = graph;
        this.locationIndex = locationIndex;
        this.router = router;
        this.template = template;
        this.maxVias = maxVias;
        this.multiplier = multiplier;
        this.deadlineNanos = deadline;
    }

    /**
     * The request the app sends when it rebuilds this segment: the personal request without anything personal or
     * generating (personal_*, bit_of_new_*, algorithm and its alternative_route.* settings), only the edge_id path
     * detail, no instructions. Null when the request has per-point settings the app never sends with vias
     * (curbsides, point hints): then no vias are made.
     */
    public static GHRequest plainRequest(GHRequest request) {
        if (request.getPoints().size() != 2 || !request.getCurbsides().isEmpty() || !request.getPointHints().isEmpty())
            return null;
        GHRequest r = new GHRequest(new ArrayList<>(request.getPoints()));
        r.setProfile(request.getProfile());
        r.setLocale(request.getLocale());
        r.setCustomModel(request.getCustomModel());
        r.setSnapPreventions(new ArrayList<>(request.getSnapPreventions()));
        r.setHeadings(new ArrayList<>(request.getHeadings()));
        PMap hints = new PMap();
        for (Map.Entry<String, Object> e : request.getHints().toMap().entrySet()) {
            String k = e.getKey();
            if (k.startsWith("personal_") || k.startsWith("bit_of_new") || k.startsWith("alternative_route")
                    || k.equals("algorithm"))
                continue;
            hints.putObject(k, e.getValue());
        }
        r.getHints().putAll(hints);
        r.getHints().putObject("instructions", false);
        r.getHints().putObject("calc_points", true);
        r.setPathDetails(List.of(EDGE_ID));
        return r;
    }

    /** The path's edges in travel order, repeats collapsed; null without the edge_id path detail. */
    public static IntArrayList edgeIds(List<PathDetail> details) {
        if (details == null) return null;
        IntArrayList out = new IntArrayList(details.size());
        for (PathDetail d : details) {
            int e = ((Number) d.getValue()).intValue();
            if (out.isEmpty() || out.get(out.size() - 1) != e) out.add(e);
        }
        return out;
    }

    public static IntArrayList edgeIds(ResponsePath p) {
        return edgeIds(p.getPathDetails().get(EDGE_ID));
    }

    /** A via: where along the target (edge index) and the point. */
    private record Via(int k, double lat, double lon, double ele) {
    }

    /**
     * @param shown the generated path, with the {@code edge_id} path detail; the vias it came with (alternative_route)
     *              are tried first and kept when they already reproduce it
     */
    public Result pin(ResponsePath shown) {
        long t0 = System.nanoTime();
        IntArrayList target = edgeIds(shown);
        // the app moves the segment's waypoints onto the route's first and last point (as decoded) and rebuilds
        // from there, so that is where the plain routes start and end
        PointList pts = shown.getPoints();
        a = new GHPoint(round(pts.getLat(0)), round(pts.getLon(0)));
        b = new GHPoint(round(pts.getLat(pts.size() - 1)), round(pts.getLon(pts.size() - 1)));
        PointList start = shown.getAltRouteViaPoints();
        Result res = new Result();
        IntArrayList rebuilt;
        List<Via> vias = new ArrayList<>();
        // as the app will send them: rounded to the response precision
        List<Via> given = new ArrayList<>();
        if (start != null)
            for (int i = 0; i < start.size(); i++)
                given.add(new Via(i, round(start.getLat(i)), round(start.getLon(i)), start.is3D() ? start.getEle(i) : 0));
        IntArrayList givenRebuilt = null;
        ResponsePath givenPath = null;
        if (!given.isEmpty()) {
            givenRebuilt = route(given, res);
            givenPath = lastPath;
            if (givenRebuilt != null && givenRebuilt.equals(target)) {
                res.keptGiven = true;
                return finish(res, given, shown, target, givenRebuilt, t0);
            }
            if (System.nanoTime() > deadlineNanos) {
                // out of time: the path's own vias, as they are
                res.stopped = "time budget";
                res.keptGiven = true;
                return finish(res, given, shown, target, givenRebuilt, t0);
            }
        }

        // Greedy farthest reach: plain /route computes each leg between consecutive points on its own, so the vias
        // are right exactly when every leg rides its stretch of the target. From the current point: the farthest
        // target edge whose leg still rides the target up to it (galloping, then binary search: a stretch of a
        // plain route is itself the plain route, so reach is monotone up to ties) gets the next via, until the leg
        // to B rides the rest. That uses the fewest vias a leg-by-leg check allows, and keeps the legs tried short.
        int n = target.size();
        // where a via may go: not A's or B's edge, not a very short edge (its middle is next to a junction, so the
        // leg after it may start on the neighbour), not an edge whose middle snaps onto another edge
        IntArrayList cand = new IntArrayList();
        for (int k = 1; k < n - 1; k++) if (viaFits(target.get(k))) cand.add(k);
        GHPoint from = a;
        int fromK = 0, next = 0; // next: the first candidate after fromK
        IntArrayList direct = leg(from, b, true, res);
        boolean reachedB = rides(direct, target, 0, n - 1);
        if (given.isEmpty()) {
            // a path without vias of its own is rebuilt as A to B alone: that is what the new vias must beat
            givenRebuilt = direct;
            givenPath = lastPath;
        }
        while (!reachedB) {
            if (vias.size() >= maxVias) {
                res.stopped = "max vias";
                break;
            }
            while (next < cand.size() && cand.get(next) <= fromK) next++;
            int lo = next - 1, hi = -1; // candidates: lo reached (or none yet), hi not reached (-1: unknown yet)
            for (int step = 1; hi < 0; step *= 2) {
                if (System.nanoTime() > deadlineNanos) break;
                int probe = next + step - 1;
                if (probe >= cand.size()) {
                    if (rides(leg(from, b, fromK == 0, res), target, fromK, n - 1)) reachedB = true;
                    hi = cand.size();
                } else if (ridesTo(from, fromK, cand.get(probe), target, res)) lo = probe;
                else hi = probe;
            }
            if (reachedB) break;
            while (hi - lo > 1 && System.nanoTime() <= deadlineNanos) {
                int mid = (lo + hi) >>> 1;
                if (ridesTo(from, fromK, cand.get(mid), target, res)) lo = mid;
                else hi = mid;
            }
            if (System.nanoTime() > deadlineNanos) {
                res.stopped = "time budget";
                break;
            }
            if (lo < next) {
                // nothing ahead is reachable along the path (e.g. it rides one of two ways drawn on the same line
                // and plain routing takes the other): pin the next place a via can go and carry on, so only this
                // stretch differs
                if (next >= cand.size()) {
                    res.stopped = "no leg from edge " + fromK + " of " + n + " rides the path to B";
                    break;
                }
                if (res.stopped == null) res.stopped = "no leg from edge " + fromK + " of " + n + " rides the path to a via";
                lo = next;
            }
            Via v = viaOn(target, cand.get(lo));
            vias.add(v);
            from = new GHPoint(v.lat, v.lon);
            fromK = cand.get(lo);
        }
        // nothing pinned (e.g. the time budget ran out first): the path's own vias at least keep plain /route on its
        // corridor, where none would give the main route back
        if (vias.isEmpty() && !given.isEmpty()) {
            lastPath = givenPath;
            res.keptGiven = true;
            return finish(res, given, shown, target, givenRebuilt, t0);
        }
        // the app's own request, all points at once: what the response promises
        rebuilt = route(vias, res);
        if (rebuilt == null && res.stopped == null) res.stopped = "plain route failed";
        Result mine = finish(res, vias, shown, target, rebuilt, t0);
        if (mine.identical || givenRebuilt == null) return mine;
        // never worse than the vias the path came with (none: A to B alone): keep them unless the new ones come
        // strictly closer
        Result theirs = new Result();
        theirs.routes = mine.routes;
        theirs.stopped = mine.stopped;
        theirs.keptGiven = true;
        lastPath = givenPath;
        theirs = finish(theirs, given, shown, target, givenRebuilt, t0);
        return mine.missingM + mine.extraM < theirs.missingM + theirs.extraM ? mine : theirs;
    }

    /** Edges shorter than this get no via (their middle is next to a junction). */
    static final double MIN_VIA_EDGE_M = 5;

    /** A via at this edge's middle lands on it: long enough, and the nearest edge to its middle is the edge itself. */
    boolean viaFits(int edge) {
        EdgeIteratorState e = graph.getEdgeIteratorState(edge, Integer.MIN_VALUE);
        if (e.getDistance() < MIN_VIA_EDGE_M) return false;
        IntArrayList one = new IntArrayList();
        one.add(edge);
        Via v = viaOn(one, 0);
        // the app's request snaps the same rounded point the same way; the legs then show whether it lands right
        Snap snap = locationIndex.findClosest(v.lat, v.lon, EdgeFilter.ALL_EDGES);
        return snap.isValid() && snap.getClosestEdge().getEdge() == edge;
    }

    /** True when the leg from the current point to a via on target edge k rides the target up to it. */
    private boolean ridesTo(GHPoint from, int fromK, int k, IntArrayList target, Result res) {
        Via v = viaOn(target, k);
        return rides(leg(from, new GHPoint(v.lat, v.lon), fromK == 0, res), target, fromK, k);
    }

    /** True when the leg's edges are exactly target[from..to]. */
    static boolean rides(IntArrayList leg, IntArrayList target, int from, int to) {
        if (leg == null || leg.size() != to - from + 1) return false;
        for (int i = 0; i < leg.size(); i++) if (leg.get(i) != target.get(from + i)) return false;
        return true;
    }

    /** One leg as plain /route computes it inside the app's request (the heading applies to the first leg only). */
    IntArrayList leg(GHPoint from, GHPoint to, boolean first, Result res) {
        return edges(List.of(from, to), first ? null : List.of(), res);
    }

    private Result finish(Result res, List<Via> vias, ResponsePath shown, IntArrayList target, IntArrayList rebuilt, long t0) {
        PointList pl = new PointList(vias.size(), graph.getNodeAccess().is3D());
        for (Via v : vias) {
            if (pl.is3D()) pl.add(v.lat, v.lon, v.ele);
            else pl.add(v.lat, v.lon);
        }
        res.vias = pl;
        res.identical = rebuilt != null && rebuilt.equals(target);
        Map<Integer, Double> want = metresByEdge(shown), got = rebuilt == null ? Map.of() : metresByEdge(lastPath);
        for (Map.Entry<Integer, Double> e : want.entrySet()) if (!got.containsKey(e.getKey())) res.missingM += e.getValue();
        for (Map.Entry<Integer, Double> e : got.entrySet()) if (!want.containsKey(e.getKey())) res.extraM += e.getValue();
        if (res.identical) res.stopped = null;
        res.ms = (System.nanoTime() - t0) / 1e6;
        return res;
    }

    /**
     * Metres of each edge id along a path: the path's own points (as the response gives them, rounded to its
     * precision) summed over every interval of that edge in the edge_id detail, haversine on lat/lon.
     */
    Map<Integer, Double> metresByEdge(ResponsePath p) {
        Map<Integer, Double> out = new HashMap<>();
        List<PathDetail> det = p.getPathDetails().get(EDGE_ID);
        PointList pts = p.getPoints();
        if (det == null) return out;
        for (PathDetail d : det) {
            double m = 0;
            for (int i = d.getFirst(); i < d.getLast(); i++)
                m += DistanceCalcEarth.DIST_EARTH.calcDist(round(pts.getLat(i)), round(pts.getLon(i)),
                        round(pts.getLat(i + 1)), round(pts.getLon(i + 1)));
            out.merge(((Number) d.getValue()).intValue(), m, Double::sum);
        }
        return out;
    }

    /** The point halfway along target edge k's geometry, rounded to the response precision. */
    Via viaOn(IntArrayList target, int k) {
        EdgeIteratorState e = graph.getEdgeIteratorState(target.get(k), Integer.MIN_VALUE);
        PointList g = e.fetchWayGeometry(FetchMode.ALL);
        double total = 0;
        for (int i = 1; i < g.size(); i++) total += seg(g, i);
        double half = total / 2, acc = 0;
        double lat = g.getLat(0), lon = g.getLon(0), ele = g.is3D() ? g.getEle(0) : 0;
        for (int i = 1; i < g.size(); i++) {
            double s = seg(g, i);
            if (acc + s >= half && s > 0) {
                double f = (half - acc) / s;
                lat = g.getLat(i - 1) + f * (g.getLat(i) - g.getLat(i - 1));
                lon = g.getLon(i - 1) + f * (g.getLon(i) - g.getLon(i - 1));
                if (g.is3D()) ele = g.getEle(i - 1) + f * (g.getEle(i) - g.getEle(i - 1));
                break;
            }
            acc += s;
        }
        return new Via(k, round(lat), round(lon), ele);
    }

    private static double seg(PointList g, int i) {
        return DistanceCalcEarth.DIST_EARTH.calcDist(g.getLat(i - 1), g.getLon(i - 1), g.getLat(i), g.getLon(i));
    }

    /** As the app gets it: ResponsePathSerializer encodes Math.round(x * multiplier), the app divides back. */
    private double round(double x) {
        return multiplier > 0 ? Math.round(x * multiplier) / multiplier : x;
    }

    /** Plain /route [A, ...vias, B] as the app sends it; its edges collapsed, or null when it fails. */
    IntArrayList route(List<Via> vias, Result res) {
        List<GHPoint> pts = new ArrayList<>();
        pts.add(a);
        for (Via v : vias) pts.add(new GHPoint(v.lat, v.lon));
        pts.add(b);
        return edges(pts, null, res);
    }

    /**
     * Plain /route over these points with the template's settings. headings null: the app's [heading, NaN, ...] (the
     * first point's heading, no constraint elsewhere); an empty list: none (a leg that starts at a via).
     */
    private IntArrayList edges(List<GHPoint> pts, List<Double> headings, Result res) {
        GHRequest r = new GHRequest(pts);
        r.setProfile(template.getProfile());
        r.setLocale(template.getLocale());
        r.setCustomModel(template.getCustomModel());
        r.setSnapPreventions(template.getSnapPreventions());
        r.setPathDetails(template.getPathDetails());
        r.getHints().putAll(template.getHints());
        List<Double> h = template.getHeadings();
        if (headings == null && !h.isEmpty()) {
            List<Double> hs = new ArrayList<>();
            hs.add(h.get(0));
            for (int i = 1; i < pts.size() - 1; i++) hs.add(Double.NaN);
            hs.add(h.size() > 1 ? h.get(h.size() - 1) : Double.NaN);
            r.setHeadings(hs);
        }
        res.routes++;
        GHResponse rsp = router.apply(r);
        if (rsp.hasErrors()) {
            res.stopped = "plain route failed: " + rsp.getErrors().get(0).getMessage();
            return null;
        }
        lastPath = rsp.getBest();
        return edgeIds(lastPath);
    }
}
