package com.graphhopper.trailmap.fixroute;

import com.graphhopper.GraphHopper;
import com.graphhopper.trailmap.fixroute.FixRouteRequest.LatLng;
import com.graphhopper.util.shapes.GHPoint;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.FileWriter;
import java.io.Writer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * IMPACT STUDY (test-only, no engine change): the deviation rule scaled by leg length — the owner's
 * bands of 2026-09-29 (short legs pick exact trails; from ~10 km the planner wants a corridor, not
 * specific ways). Per aligned leg: would today's route still be a deviation under the scaled rule,
 * and for legs from {@link #CORRIDOR_FROM_M} a prototype of the corridor fix (one pin per deviation,
 * no track matching). Compared against the current engine's result. Writes {@code scale.jsonl}.
 */
public class LegScaleImpactTest {

    static final String OUT_DIR = "../../data/fix-route-out/scale";
    /** From this leg length the corridor prototype replaces the current engine's result. */
    static final double CORRIDOR_FROM_M = 10_000;
    /** Corridor prototype: at most one pin per this many metres of leg, and at most 5. */
    static final double PIN_PER_M = 50_000;
    static final int PIN_MAX = 5;

    private static GraphHopper hopper;
    private static ExecutorService pool;

    @BeforeAll
    static void setup() throws Exception {
        Assumptions.assumeTrue(FixRouteFixtures.available(), "fixtures not found");
        Assumptions.assumeTrue(FixRouteTestGraph.available(), "graph cache not found");
        hopper = FixRouteTestGraph.open();
        pool = Executors.newFixedThreadPool(Math.max(2, Runtime.getRuntime().availableProcessors() - 2));
    }

    @AfterAll
    static void teardown() {
        if (pool != null) pool.shutdown();
        if (hopper != null) hopper.close();
    }

    /** Piecewise-linear through the owner's anchors; beyond the last anchor proportional to length. */
    static double interp(double len, double[][] anchors, double shareBeyond) {
        if (len <= anchors[0][0]) return anchors[0][1];
        for (int i = 1; i < anchors.length; i++) {
            if (len <= anchors[i][0]) {
                double t = (len - anchors[i - 1][0]) / (anchors[i][0] - anchors[i - 1][0]);
                return anchors[i - 1][1] + t * (anchors[i][1] - anchors[i - 1][1]);
            }
        }
        return Math.max(anchors[anchors.length - 1][1], shareBeyond * len);
    }

    static final double[][] PEAK = {{2000, 30}, {5000, 50}, {10000, 100}, {20000, 500}};
    static final double[][] MINLEN = {{2000, 25}, {5000, 100}, {10000, 250}, {20000, 1000}};

    static Deviation.Rule scaled(Deviation.Rule base, double len) {
        return new Deviation.Rule(interp(len, PEAK, 0.025), interp(len, MINLEN, 0.05),
                Math.max(base.detourMinM(), 0.02 * len), base.detourMinRatio(), base.zoneM(), base.armM());
    }

    static final Deviation.Rule BASE = new Deviation.Rule(30, 25, 100, 0.3, 30, 150);

    record Proto(int pins, boolean within, List<double[]> points, long ms, int probes) {
    }

    /** Corridor prototype: pin the saved route at the middle of the worst deviating saved stretch. */
    static Proto corridor(ClientRoute.Settings st, GHPoint a, GHPoint b, Double heading, Double penalty,
                          List<double[]> saved, Deviation.Rule rule, double len) {
        long t0 = System.currentTimeMillis();
        int cap = (int) Math.min(PIN_MAX, Math.max(1, Math.ceil(len / PIN_PER_M)));
        List<GHPoint> pins = new ArrayList<>();
        List<Double> pinArc = new ArrayList<>();
        int probes = 0;
        List<double[]> cur = routeVia(st, a, pins, b, heading, penalty);
        probes++;
        Deviation.Ends ends = Deviation.Ends.of(rule, true, true);
        while (cur != null && !Deviation.within(cur, saved, rule, ends) && pins.size() < cap) {
            Deviation.Stretch worst = null;
            for (Deviation.Stretch s : Deviation.stretches(cur, saved, rule, ends)) {
                if (s.onRoute() || !s.isDeviation(rule)) continue;
                if (worst == null || s.lengthM() * s.peakM() > worst.lengthM() * worst.peakM()) worst = s;
            }
            if (worst == null) break; // only route-side deviations: no saved point to pin
            double arc = (worst.fromM() + worst.toM()) / 2;
            double[] p = pointAt(saved, arc);
            int k = 0;
            while (k < pinArc.size() && pinArc.get(k) < arc) k++;
            List<GHPoint> np = new ArrayList<>(pins);
            np.add(k, new GHPoint(p[0], p[1]));
            List<double[]> nr = routeVia(st, a, np, b, heading, penalty);
            probes++;
            if (nr == null) break;
            double before = offSum(cur, saved, rule, ends), after = offSum(nr, saved, rule, ends);
            if (after >= before) break;
            pins = np;
            pinArc.add(k, arc);
            cur = nr;
        }
        boolean ok = cur != null && Deviation.within(cur, saved, rule, ends);
        return new Proto(pins.size(), ok, cur, System.currentTimeMillis() - t0, probes);
    }

    static double offSum(List<double[]> route, List<double[]> saved, Deviation.Rule rule, Deviation.Ends ends) {
        double s = 0;
        for (Deviation.Stretch d : Deviation.stretches(route, saved, rule, ends)) if (d.isDeviation(rule)) s += d.lengthM();
        return s;
    }

    /** Sub-legs routed one by one (as the client does for added waypoints), the heading on the first only. */
    static List<double[]> routeVia(ClientRoute.Settings st, GHPoint a, List<GHPoint> pins, GHPoint b, Double heading, Double penalty) {
        List<GHPoint> pts = new ArrayList<>();
        pts.add(a);
        pts.addAll(pins);
        pts.add(b);
        List<double[]> out = new ArrayList<>();
        for (int i = 0; i + 1 < pts.size(); i++) {
            ClientRoute.Leg l = ClientRoute.route(hopper, st, pts.get(i), List.of(), pts.get(i + 1),
                    i == 0 ? heading : null, i == 0 ? penalty : null);
            if (!l.ok()) return null;
            out.addAll(l.points());
        }
        return out;
    }

    static double[] pointAt(List<double[]> line, double arcM) {
        double acc = 0;
        for (int i = 1; i < line.size(); i++) {
            double[] p = line.get(i - 1), q = line.get(i);
            double d = com.graphhopper.util.DistanceCalcEarth.DIST_EARTH.calcDist(p[0], p[1], q[0], q[1]);
            if (acc + d >= arcM && d > 0) {
                double t = (arcM - acc) / d;
                return new double[]{p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])};
            }
            acc += d;
        }
        return line.get(line.size() - 1);
    }

    // ------------------------------------------------------------------------------------------
    // Long-leg SAMPLING prototype (owner direction 2026-09-29): legs >= SAMPLE_FROM_M get via points
    // sampled along the saved route — no deviation check, no iteration. Samples only on roads /
    // cycleways (footways for walk profiles), never tracks / paths / service roads; bigger roads
    // preferred; mid-edge, not at a junction or a dead end, where the saved route clearly rides.
    // ------------------------------------------------------------------------------------------

    static final double SAMPLE_FROM_M = 20_000;
    /** Spacing S(L) = SAMPLE_BASE_M × sqrt(L / 50 km): 20 km → 6.3, 50 → 10, 100 → 14, 300 → 24.5 km. */
    static final double SAMPLE_BASE_M = Double.parseDouble(System.getProperty("sample.base", "10000"));
    static final double SAMPLE_WINDOW = 0.2;
    /** Variant B ({@code -Dsample.dense=true}): sample only inside corridor deviations, every DENSE_M. */
    static final boolean DENSE_IN_DEVIATIONS = Boolean.getBoolean("sample.dense");
    static final double DENSE_M = Double.parseDouble(System.getProperty("sample.denseM", "2000"));

    /**
     * Owner's long-leg corridor (2026-09-29): "last time via this city, now another city 20 km away".
     * Width 1 km at 20 km, linear to 10 % of the leg at 100 km, 10 % beyond. A deviation reaches
     * half a width off, over at least a width; via points one width apart ({@code -Dsample.wide=true}).
     */
    static final boolean WIDE = Boolean.getBoolean("sample.wide");

    static double corridorWidth(double len) {
        if (len >= 100_000) return 0.10 * len;
        return 1_000 + (len - 20_000) / 80_000 * 9_000;
    }

    static Deviation.Rule wide(double len) {
        double w = corridorWidth(len);
        return new Deviation.Rule(w / 2, w, w, 0.3, 30, 150);
    }
    static final double SAMPLE_STEP_M = 50, ALIGN_TOL_M = 10, ALIGN_MIN_M = 150, ALIGN_CAP_M = 500, EDGE_END_M = 30;

    static double spacing(double len) {
        return SAMPLE_BASE_M * Math.sqrt(len / 50_000);
    }

    static int rank(com.graphhopper.routing.ev.RoadClass c, boolean walk) {
        switch (c) {
            case MOTORWAY: case TRUNK: return 7;
            case PRIMARY: return 6;
            case SECONDARY: return 5;
            case TERTIARY: return 4;
            case UNCLASSIFIED: case ROAD: return 3;
            case RESIDENTIAL: case CYCLEWAY: return 2;
            case LIVING_STREET: return 1;
            case FOOTWAY: case PEDESTRIAN: return walk ? 1 : -1;
            default: return -1; // track, path, service, steps, bridleway, ...
        }
    }

    com.graphhopper.routing.weighting.Weighting weighting(ClientRoute.Settings s) {
        com.graphhopper.util.PMap hints = new com.graphhopper.util.PMap();
        if (s.customModel() != null) hints.putObject(com.graphhopper.util.CustomModel.KEY, s.customModel());
        return hopper.createWeighting(hopper.getProfile(s.profile()), hints);
    }

    com.graphhopper.routing.util.EdgeFilter usable(ClientRoute.Settings s) {
        com.graphhopper.routing.weighting.Weighting w = weighting(s);
        return new com.graphhopper.routing.util.DefaultSnapFilter(w,
                hopper.getEncodingManager().getBooleanEncodedValue(com.graphhopper.routing.ev.Subnetwork.key(s.profile())));
    }

    record Sample(GHPoint p, double arc, int rank, double align) {
    }

    /**
     * Owner fallback (2026-09-29): a window with no road / cycleway spot tries service roads, then
     * tracks ({@code -Dsample.fallback=true}). Tier 1 = roads, cycleways (footways for walk).
     */
    static final boolean FALLBACK = Boolean.getBoolean("sample.fallback");
    int pickTier = 1;
    final int[] tierUsed = new int[4];

    static boolean tierAllows(com.graphhopper.routing.ev.RoadClass c, boolean walk, int tier) {
        if (rank(c, walk) > 0) return true;
        if (tier >= 2 && c == com.graphhopper.routing.ev.RoadClass.SERVICE) return true;
        return tier >= 3 && c == com.graphhopper.routing.ev.RoadClass.TRACK;
    }

    Sample pickTiered(List<double[]> saved, double[] cum, double lo, double hi, com.graphhopper.routing.util.EdgeFilter f,
                      com.graphhopper.routing.ev.EnumEncodedValue<com.graphhopper.routing.ev.RoadClass> rcEv, boolean walk,
                      com.graphhopper.routing.weighting.Weighting w) {
        for (int t = 1; t <= (FALLBACK ? 3 : 1); t++) {
            pickTier = t;
            Sample s = pick(saved, cum, lo, hi, f, rcEv, walk, w);
            if (s != null) {
                tierUsed[t]++;
                pickTier = 1;
                return s;
            }
        }
        pickTier = 1;
        return null;
    }

    /** The best sample spot in [lo, hi] along the saved line, or null. */
    Sample pick(List<double[]> saved, double[] cum, double lo, double hi, com.graphhopper.routing.util.EdgeFilter f,
                com.graphhopper.routing.ev.EnumEncodedValue<com.graphhopper.routing.ev.RoadClass> rcEv, boolean walk,
                com.graphhopper.routing.weighting.Weighting w) {
        com.graphhopper.util.EdgeExplorer ex = hopper.getBaseGraph().createEdgeExplorer();
        com.graphhopper.storage.NodeAccess na = hopper.getBaseGraph().getNodeAccess();
        com.graphhopper.util.DistanceCalc dc = com.graphhopper.util.DistanceCalcEarth.DIST_EARTH;
        Sample best = null;
        double bestScore = 0;
        for (double arc = lo; arc <= hi; arc += SAMPLE_STEP_M) {
            double[] p = at(saved, cum, arc);
            com.graphhopper.routing.util.EdgeFilter allowed = e -> f.accept(e) && tierAllows(e.get(rcEv), walk, pickTier);
            com.graphhopper.storage.index.Snap sn = hopper.getLocationIndex().findClosest(p[0], p[1], allowed);
            if (!sn.isValid() || sn.getQueryDistance() > ALIGN_TOL_M) continue;
            if (sn.getSnappedPosition() != com.graphhopper.storage.index.Snap.Position.EDGE) continue;
            com.graphhopper.util.EdgeIteratorState e = sn.getClosestEdge();
            int base = e.getBaseNode(), adj = e.getAdjNode();
            GHPoint sp = sn.getSnappedPoint();
            if (dc.calcDist(sp.lat, sp.lon, na.getLat(base), na.getLon(base)) < EDGE_END_M
                    || dc.calcDist(sp.lat, sp.lon, na.getLat(adj), na.getLon(adj)) < EDGE_END_M) continue;
            if (degree(ex, base) < 2 || degree(ex, adj) < 2) continue; // a dead end
            com.graphhopper.util.PointList g = e.fetchWayGeometry(com.graphhopper.util.FetchMode.ALL);
            double align = 0;
            for (int dir = -1; dir <= 1; dir += 2) {
                for (double d = 10; d <= ALIGN_CAP_M; d += 10) {
                    double a2 = arc + dir * d;
                    if (a2 < 0 || a2 > cum[cum.length - 1]) break;
                    double[] q = at(saved, cum, a2);
                    if (distToLine(q, g) > ALIGN_TOL_M) break;
                    align += 10;
                }
            }
            if (align < ALIGN_MIN_M) continue;
            // Rideable both ways: no one-way carriageway (a via on the wrong one forces a U-turn).
            if (w != null && (Double.isInfinite(w.calcEdgeWeight(e, false)) || Double.isInfinite(w.calcEdgeWeight(e, true)))) continue;
            int rk = Math.max(0, rank(e.get(rcEv), walk));
            // Stay near the target: sliding to the best-aligned spot tends to land where today's route runs anyway.
            double mid = (lo + hi) / 2, half = Math.max(1, (hi - lo) / 2);
            double score = align * (1 + 0.25 * rk) * (1 - 0.7 * Math.abs(arc - mid) / half);
            if (score > bestScore) {
                bestScore = score;
                best = new Sample(sp, arc, rk, align);
            }
        }
        return best;
    }

    static int degree(com.graphhopper.util.EdgeExplorer ex, int node) {
        int n = 0;
        com.graphhopper.util.EdgeIterator it = ex.setBaseNode(node);
        while (it.next()) n++;
        return n;
    }

    static double distToLine(double[] p, com.graphhopper.util.PointList g) {
        double best = Double.POSITIVE_INFINITY, mLon = 111_320 * Math.cos(Math.toRadians(p[0]));
        for (int i = 1; i < g.size(); i++) {
            double ax = (g.getLon(i - 1) - p[1]) * mLon, ay = (g.getLat(i - 1) - p[0]) * 110_540;
            double bx = (g.getLon(i) - p[1]) * mLon, by = (g.getLat(i) - p[0]) * 110_540;
            double dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
            double t = l2 == 0 ? 0 : Math.max(0, Math.min(1, -(ax * dx + ay * dy) / l2));
            double x = ax + t * dx, y = ay + t * dy;
            best = Math.min(best, Math.sqrt(x * x + y * y));
        }
        return best;
    }

    static double[] cumulative(List<double[]> line) {
        double[] c = new double[line.size()];
        for (int i = 1; i < line.size(); i++)
            c[i] = c[i - 1] + com.graphhopper.util.DistanceCalcEarth.DIST_EARTH.calcDist(line.get(i - 1)[0], line.get(i - 1)[1], line.get(i)[0], line.get(i)[1]);
        return c;
    }

    static double[] at(List<double[]> line, double[] cum, double arc) {
        int i = java.util.Arrays.binarySearch(cum, arc);
        if (i >= 0) return line.get(i);
        i = -i - 1;
        if (i <= 0) return line.get(0);
        if (i >= cum.length) return line.get(line.size() - 1);
        double t = (arc - cum[i - 1]) / Math.max(1e-9, cum[i] - cum[i - 1]);
        double[] a = line.get(i - 1), b = line.get(i);
        return new double[]{a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])};
    }

    /** Metres the route rides back at via point {@code v} (arrives and leaves on the same line). */
    static double uturnAt(List<double[]> route, GHPoint v) {
        double[] rc = cumulative(route);
        int k = 0;
        double bd = Double.POSITIVE_INFINITY;
        for (int i = 0; i < route.size(); i++) {
            double d = com.graphhopper.util.DistanceCalcEarth.DIST_EARTH.calcDist(route.get(i)[0], route.get(i)[1], v.lat, v.lon);
            if (d < bd) {
                bd = d;
                k = i;
            }
        }
        double back = 0;
        for (double d = 5; d <= 300; d += 5) {
            double ta = rc[k] + d, tb = rc[k] - d;
            if (ta > rc[rc.length - 1] || tb < 0) break;
            double[] pa = at(route, rc, ta), pb = at(route, rc, tb);
            if (com.graphhopper.util.DistanceCalcEarth.DIST_EARTH.calcDist(pa[0], pa[1], pb[0], pb[1]) > 8) break;
            back = d;
        }
        return back;
    }

    /** {@code -Dsample.export=route:leg,...}: before/after preview files (make_before_after.py format). */
    static final List<String> EXPORT = List.of(System.getProperty("sample.export", "").split(","));

    static void exportSide(File dir, long id, int leg, List<double[]> saved, List<double[]> today, List<double[]> fin,
                           List<double[]> wps, List<double[]> vias, List<List<double[]>> context, double[] ends,
                           int uturns, double savedLeft, double walkOff, double savedLeftToday, double walkOffToday) throws java.io.IOException {
        dir.mkdirs();
        StringBuilder g = new StringBuilder("{\"type\":\"FeatureCollection\",\"features\":[");
        g.append(line(saved, "saved")).append(',').append(line(today, "before")).append(',').append(line(fin, "match_final"));
        for (List<double[]> c : context) g.append(',').append(line(c, "context"));
        for (double[] w : wps) g.append(',').append(point(w, "match_new_waypoint"));
        for (double[] v : vias) g.append(',').append(point(v, "via_point"));
        g.append(',').append(point(new double[]{ends[0], ends[1]}, "fixed_waypoint")).append(',').append(point(new double[]{ends[2], ends[3]}, "fixed_waypoint"));
        g.append(String.format(Locale.ROOT, "],\"properties\":{\"route\":%d,\"leg\":%d,\"checks\":{\"uturns\":%d,\"uturn_m\":0,\"wrongway\":\"-\","
                        + "\"saved_left_m\":%.0f,\"walk_off_m\":%.0f,\"saved_left_today_m\":%.0f,\"walk_off_today_m\":%.0f}}}",
                id, leg, uturns, savedLeft, walkOff, savedLeftToday, walkOffToday));
        try (Writer w = new FileWriter(new File(dir, "route-" + id + "_leg-" + leg + ".geojson"))) {
            w.write(g.toString());
        }
    }

    static String line(List<double[]> pts, String name) {
        StringBuilder sb = new StringBuilder("{\"type\":\"Feature\",\"properties\":{\"name\":\"" + name + "\"},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
        int stepK = Math.max(1, pts.size() / 20000);
        for (int i = 0; i < pts.size(); i += stepK) {
            if (i > 0) sb.append(',');
            sb.append(String.format(Locale.ROOT, "[%.5f,%.5f]", pts.get(i)[1], pts.get(i)[0]));
        }
        return sb.append("]}}").toString();
    }

    static String point(double[] p, String name) {
        return String.format(Locale.ROOT, "{\"type\":\"Feature\",\"properties\":{\"name\":\"%s\"},\"geometry\":{\"type\":\"Point\",\"coordinates\":[%.6f,%.6f]}}", name, p[1], p[0]);
    }

    @Test
    void longLegSampling() throws Exception {
        String only = System.getProperty("fix.routes");
        List<Long> ids = new ArrayList<>();
        if (only != null && !only.isBlank()) for (String s : only.split(",")) ids.add(Long.parseLong(s.trim()));
        else ids = FixRouteFixtures.allIds();
        new File(OUT_DIR).mkdirs();
        com.graphhopper.routing.ev.EnumEncodedValue<com.graphhopper.routing.ev.RoadClass> rcEv =
                hopper.getEncodingManager().getEnumEncodedValue(com.graphhopper.routing.ev.RoadClass.KEY, com.graphhopper.routing.ev.RoadClass.class);
        double[] tot = new double[12];
        StringBuilder rows = new StringBuilder();
        try (Writer out = new FileWriter(new File(OUT_DIR, "sampling.jsonl"))) {
            for (long id : ids) {
                FixRouteFixtures.Route r = FixRouteFixtures.load(id);
                FixRouteRequest q = FixRouteCorpusTest.request(r);
                boolean anyLong = false;
                for (FixRouteFixtures.Segment s : r.segments) if (s.savedLengthM != null && s.savedLengthM >= SAMPLE_FROM_M) anyLong = true;
                if (!anyLong) continue;
                FixRouteResponse r1 = FixRouteEngine.fix(hopper, pool, q);
                ReferenceTrack ref = new ReferenceTrack(q.reference);
                Map<String, FixRouteRequest.Waypoint> byId = new HashMap<>();
                for (FixRouteRequest.Waypoint w : q.waypoints) byId.put(w.id, w);
                for (FixRouteResponse.Leg l : r1.legs) {
                    if (l.metrics.refFromM == null || l.metrics.refToM == null) continue;
                    FixRouteRequest.Segment s = q.segments.get(l.index);
                    double len = l.metrics.refToM - l.metrics.refFromM;
                    if (!s.isFollowRoads() || len < SAMPLE_FROM_M) continue;
                    if (s.viaPoints != null && !s.viaPoints.isEmpty()) continue; // a chosen alternative: left alone
                    List<double[]> saved = ref.slice(l.metrics.refFromM, l.metrics.refToM);
                    double[] cum = cumulative(saved);
                    FixRouteRequest.Waypoint wa = q.waypoints.get(l.index), wb = q.waypoints.get(l.index + 1);
                    GHPoint a = new GHPoint(wa.coordinates.lat, wa.coordinates.lng), b = new GHPoint(wb.coordinates.lat, wb.coordinates.lng);
                    ClientRoute.Settings st = new ClientRoute.Settings(s.profile, s.customModel, q.snapPreventions);
                    boolean walk = s.profile != null && (s.profile.contains("foot") || s.profile.contains("hike") || s.profile.contains("walk"));
                    long t0 = System.currentTimeMillis();
                    ClientRoute.Leg today = ClientRoute.route(hopper, st, a, List.of(), b, s.initialHeading, s.headingPenalty);
                    long msToday = System.currentTimeMillis() - t0;
                    if (!today.ok()) continue;
                    // Current engine's result as the client routes it.
                    List<double[]> now = new ArrayList<>();
                    List<double[]> nowVias = new ArrayList<>();
                    GHPoint from = a;
                    for (FixRouteResponse.Segment rs : l.segments) {
                        GHPoint to = rs.end.waypointId != null ? new GHPoint(byId.get(rs.end.waypointId).coordinates.lat, byId.get(rs.end.waypointId).coordinates.lng)
                                : new GHPoint(rs.end.newPoint.lat, rs.end.newPoint.lng);
                        if (FixRouteRequest.TYPE_FOLLOW_ROADS.equals(rs.type)) {
                            List<GHPoint> rv = new ArrayList<>();
                            if (rs.viaPoints != null) for (LatLng x : rs.viaPoints) {
                                rv.add(new GHPoint(x.lat, x.lng));
                                nowVias.add(new double[]{x.lat, x.lng});
                            }
                            ClientRoute.Leg cl = ClientRoute.route(hopper, st, from, rv, to, rs.initialHeading,
                                    rs.initialHeading != null ? RouteFixer.CLIENT_HEADING_PENALTY : null);
                            if (cl.ok()) now.addAll(cl.points());
                        }
                        from = to;
                    }
                    if (now.size() < 2) now = today.points();
                    // Sampling: targets every S, the best spot within ±20 % of S.
                    long t1 = System.currentTimeMillis();
                    double S = spacing(len);
                    com.graphhopper.routing.util.EdgeFilter f = usable(st);
                    com.graphhopper.routing.weighting.Weighting wt = weighting(st);
                    List<Sample> samples = new ArrayList<>();
                    int missing = 0;
                    if (DENSE_IN_DEVIATIONS) {
                        // Variant B: only where today's route leaves the corridor (scaled rule, saved side),
                        // one sample every DENSE_M inside each such stretch.
                        Deviation.Rule rr = WIDE ? wide(len) : scaled(BASE, len);
                        for (Deviation.Stretch d : Deviation.stretches(today.points(), saved, rr, Deviation.Ends.of(rr, true, true))) {
                            if (d.onRoute() || !d.isDeviation(rr)) continue;
                            double span = d.toM() - d.fromM();
                            int k = Math.max(1, Math.min(Integer.getInteger("sample.maxPerDev", 1000),
                                    (int) Math.round(span / (WIDE ? corridorWidth(len) : DENSE_M))));
                            double step = span / k;
                            for (int j = 0; j < k; j++) {
                                double t = d.fromM() + (j + 0.5) * step;
                                Sample sm = pickTiered(saved, cum, Math.max(0, t - SAMPLE_WINDOW * step), Math.min(len, t + SAMPLE_WINDOW * step), f, rcEv, walk, wt);
                                if (sm == null) missing++;
                                else samples.add(sm);
                            }
                        }
                    }
                    for (double t = S; !DENSE_IN_DEVIATIONS && t < len - S / 2; t += S) {
                        Sample sm = pick(saved, cum, Math.max(0, t - SAMPLE_WINDOW * S), Math.min(len, t + SAMPLE_WINDOW * S), f, rcEv, walk, wt);
                        if (sm == null) missing++;
                        else samples.add(sm);
                    }
                    long msPick = System.currentTimeMillis() - t1;
                    if (Boolean.getBoolean("sample.debug")) {
                        com.graphhopper.util.PointList tp = new com.graphhopper.util.PointList();
                        for (double[] x : today.points()) tp.add(x[0], x[1]);
                        for (double t = S; t < len - S / 2; t += S) {
                            double[] tgt = at(saved, cum, t);
                            System.out.printf(Locale.ROOT, "SDBG %d:%d target %.1f km: today %.0f m off there%n", id, l.index, t / 1e3, distToLine(tgt, tp));
                        }
                        for (Sample sm : samples)
                            System.out.printf(Locale.ROOT, "SDBG %d:%d sample at %.1f km rank %d align %.0f m, today %.0f m away%n", id, l.index,
                                    sm.arc() / 1e3, sm.rank(), sm.align(), distToLine(new double[]{sm.p().lat, sm.p().lon}, tp));
                        // Where the saved route is far from today's route: what road classes lie under it?
                        Map<String, Integer> cls = new java.util.TreeMap<>();
                        for (double t = 0; t < len; t += 100) {
                            double[] x = at(saved, cum, t);
                            if (distToLine(x, tp) < 100) continue;
                            com.graphhopper.storage.index.Snap sn = hopper.getLocationIndex().findClosest(x[0], x[1], f);
                            String c = !sn.isValid() || sn.getQueryDistance() > ALIGN_TOL_M ? "none" : sn.getClosestEdge().get(rcEv).toString();
                            cls.merge(c, 1, Integer::sum);
                        }
                        System.out.println("SDBG " + id + ":" + l.index + " road classes under the deviating saved route (per 100 m): " + cls);
                    }
                    List<GHPoint> vias = new ArrayList<>();
                    for (Sample sm : samples) vias.add(sm.p());
                    long t2 = System.currentTimeMillis();
                    ClientRoute.Leg sr = ClientRoute.route(hopper, st, a, vias, b, s.initialHeading, s.headingPenalty);
                    int calls = 1, dropped = 0;
                    if (sr.ok()) {
                        List<GHPoint> keep = new ArrayList<>();
                        for (GHPoint v : vias) if (uturnAt(sr.points(), v) < 30) keep.add(v);
                        if (keep.size() < vias.size()) {
                            dropped = vias.size() - keep.size();
                            vias = keep;
                            sr = ClientRoute.route(hopper, st, a, vias, b, s.initialHeading, s.headingPenalty);
                            calls++;
                        }
                    }
                    long msSampled = System.currentTimeMillis() - t2;
                    List<double[]> samp = sr.ok() ? sr.points() : today.points();
                    int uturnsLeft = 0;
                    if (sr.ok()) for (GHPoint v : vias) if (uturnAt(samp, v) >= 30) uturnsLeft++;
                    Deviation.Rule rule = WIDE ? wide(len) : scaled(BASE, len);
                    Deviation.Ends ends = Deviation.Ends.of(rule, true, true);
                    double dToday = offSum(today.points(), saved, rule, ends), dNow = offSum(now, saved, rule, ends), dSamp = offSum(samp, saved, rule, ends);
                    double wToday = RequestReplayTest.coWalkOff(today.points(), saved, 30), wNow = RequestReplayTest.coWalkOff(now, saved, 30), wSamp = RequestReplayTest.coWalkOff(samp, saved, 30);
                    double extra = (sr.ok() ? sr.distanceM() : today.distanceM()) - today.distanceM();
                    tot[0] += len; tot[1] += dToday; tot[2] += dNow; tot[3] += dSamp; tot[4] += wToday; tot[5] += wNow; tot[6] += wSamp;
                    tot[7] += msToday; tot[8] += msPick + msSampled; tot[9] += l.metrics.ms; tot[10] += vias.size(); tot[11] += l.metrics.addedWaypoints;
                    rows.append(String.format(Locale.ROOT, "  %d:%d %5.1f km S=%4.1f km vias %2d (missing %d, dropped %d, U-turns left %d) | corridor-off today %5.1f now %5.1f sampled %5.1f km | 30m-off today %6.1f now %5.1f sampled %6.1f km | %+5.1f km longer | ms today %5d now %5d sampled %5d (pick %d, %d calls) | now +%d wp%n",
                            id, l.index, len / 1e3, S / 1e3, vias.size(), missing, dropped, uturnsLeft, dToday / 1e3, dNow / 1e3, dSamp / 1e3,
                            wToday / 1e3, wNow / 1e3, wSamp / 1e3, extra / 1e3, msToday, l.metrics.ms, msPick + msSampled, msPick, calls, l.metrics.addedWaypoints));
                    String key = id + ":" + l.index;
                    if (EXPORT.contains(key)) {
                        List<List<double[]>> context = new ArrayList<>();
                        for (int nb : new int[]{l.index - 1, l.index + 1}) {
                            if (nb < 0 || nb >= q.segments.size() || !q.segments.get(nb).isFollowRoads()) continue;
                            FixRouteRequest.Segment ns = q.segments.get(nb);
                            FixRouteRequest.Waypoint na0 = q.waypoints.get(nb), nb0 = q.waypoints.get(nb + 1);
                            ClientRoute.Leg cl = ClientRoute.route(hopper, new ClientRoute.Settings(ns.profile, ns.customModel, q.snapPreventions),
                                    new GHPoint(na0.coordinates.lat, na0.coordinates.lng), List.of(), new GHPoint(nb0.coordinates.lat, nb0.coordinates.lng),
                                    ns.initialHeading, ns.headingPenalty);
                            if (cl.ok()) context.add(cl.points());
                        }
                        List<double[]> addedWps = new ArrayList<>();
                        for (FixRouteResponse.Segment rs : l.segments) if (rs.end.newPoint != null) addedWps.add(new double[]{rs.end.newPoint.lat, rs.end.newPoint.lng});
                        List<double[]> viaPts = new ArrayList<>();
                        for (GHPoint v : vias) viaPts.add(new double[]{v.lat, v.lon});
                        double[] ends2 = {a.lat, a.lon, b.lat, b.lon};
                        exportSide(new File(OUT_DIR, "preview-old"), id, l.index, saved, today.points(), now, addedWps, nowVias, context, ends2,
                                0, RequestReplayTest.farLength(saved, now, 30), wNow, RequestReplayTest.farLength(saved, today.points(), 30), wToday);
                        exportSide(new File(OUT_DIR, "preview-new"), id, l.index, saved, today.points(), samp, List.of(), viaPts, context, ends2,
                                uturnsLeft, RequestReplayTest.farLength(saved, samp, 30), wSamp, RequestReplayTest.farLength(saved, today.points(), 30), wToday);
                    }
                    StringBuilder vj = new StringBuilder("[");
                    for (int k = 0; k < vias.size(); k++) vj.append(k == 0 ? "" : ",").append(String.format(Locale.ROOT, "[%.6f,%.6f]", vias.get(k).lat, vias.get(k).lon));
                    out.write(String.format(Locale.ROOT, "{\"route\":%d,\"leg\":%d,\"len_m\":%.0f,\"vias\":%s,\"dev_today_m\":%.0f,\"dev_now_m\":%.0f,\"dev_sampled_m\":%.0f}%n",
                            id, l.index, len, vj.append(']'), dToday, dNow, dSamp));
                }
            }
        }
        System.out.println("SAMPLING legs >= " + SAMPLE_FROM_M / 1e3 + " km:\n" + rows);
        System.out.printf(Locale.ROOT, "SAMPLING via points by road tier: road/cycleway %d, service %d, track %d%n", tierUsed[1], tierUsed[2], tierUsed[3]);
        System.out.printf(Locale.ROOT, "SAMPLING total %.0f km of legs | corridor-off today %.1f / now %.1f / sampled %.1f km | 30m-off today %.1f / now %.1f / sampled %.1f km | ms today %.0f, now %.0f, sampled %.0f | via points %.0f vs now waypoints %.0f%n",
                tot[0] / 1e3, tot[1] / 1e3, tot[2] / 1e3, tot[3] / 1e3, tot[4] / 1e3, tot[5] / 1e3, tot[6] / 1e3, tot[7], tot[9], tot[8], tot[10], tot[11]);
    }

    @Test
    void impact() throws Exception {
        String only = System.getProperty("fix.routes");
        List<Long> ids = new ArrayList<>();
        if (only != null && !only.isBlank()) for (String s : only.split(",")) ids.add(Long.parseLong(s.trim()));
        else ids = FixRouteFixtures.allIds();
        new File(OUT_DIR).mkdirs();
        Map<String, Object[]> verdictLegs = new HashMap<>();
        for (Object[] v : FixRouteCorpusTest.OWNER_VERDICTS) verdictLegs.put(v[0] + ":" + v[1], v);
        String[] band = {"0-2", "2-5", "5-10", "10-20", "20-50", "50+"};
        double[] bandLo = {0, 2e3, 5e3, 10e3, 20e3, 50e3};
        int nb = band.length;
        int[] legs = new int[nb], curFixed = new int[nb], curWp = new int[nb], left = new int[nb], still = new int[nb], newWp = new int[nb];
        double[] offToday = new double[nb], offCur = new double[nb], offNew = new double[nb];
        // Corridor-scale measure: deviating length under the SCALED rule (what the new intent calls wrong).
        double[] devToday = new double[nb], devCur = new double[nb], devNew = new double[nb];
        long[] msCur = new long[nb], msNew = new long[nb];
        StringBuilder flips = new StringBuilder(), longLegs = new StringBuilder();
        try (Writer out = new FileWriter(new File(OUT_DIR, "scale.jsonl"))) {
            for (long id : ids) {
                FixRouteFixtures.Route r = FixRouteFixtures.load(id);
                FixRouteRequest q = FixRouteCorpusTest.request(r);
                FixRouteResponse r1 = FixRouteEngine.fix(hopper, pool, q);
                // The current engine's result as the client routes it: each returned segment on its own.
                List<Integer> parents = new ArrayList<>();
                List<List<double[]>> geoms = new ArrayList<>();
                Map<String, FixRouteRequest.Waypoint> byId = new HashMap<>();
                for (FixRouteRequest.Waypoint w : q.waypoints) byId.put(w.id, w);
                for (FixRouteResponse.Leg rl : r1.legs) {
                    FixRouteRequest.Segment parent = q.segments.get(rl.index);
                    FixRouteRequest.Waypoint w0 = q.waypoints.get(rl.index);
                    GHPoint from = new GHPoint(w0.coordinates.lat, w0.coordinates.lng);
                    for (FixRouteResponse.Segment rs : rl.segments) {
                        GHPoint to = rs.end.waypointId != null
                                ? new GHPoint(byId.get(rs.end.waypointId).coordinates.lat, byId.get(rs.end.waypointId).coordinates.lng)
                                : new GHPoint(rs.end.newPoint.lat, rs.end.newPoint.lng);
                        List<double[]> g = null;
                        if (FixRouteRequest.TYPE_FOLLOW_ROADS.equals(rs.type)) {
                            List<GHPoint> v = new ArrayList<>();
                            if (rs.viaPoints != null) for (LatLng x : rs.viaPoints) v.add(new GHPoint(x.lat, x.lng));
                            ClientRoute.Leg cl = ClientRoute.route(hopper, new ClientRoute.Settings(parent.profile, parent.customModel, q.snapPreventions),
                                    from, v, to, rs.initialHeading, rs.initialHeading != null ? RouteFixer.CLIENT_HEADING_PENALTY : null);
                            if (cl.ok()) g = cl.points();
                        }
                        parents.add(rl.index);
                        geoms.add(g);
                        from = to;
                    }
                }
                ReferenceTrack ref = new ReferenceTrack(q.reference);
                for (FixRouteResponse.Leg l : r1.legs) {
                    if (l.metrics.refFromM == null || l.metrics.refToM == null) continue;
                    FixRouteRequest.Segment s = q.segments.get(l.index);
                    double len = l.metrics.refToM - l.metrics.refFromM;
                    if (!s.isFollowRoads() || len < 1) continue;
                    List<double[]> saved = ref.slice(l.metrics.refFromM, l.metrics.refToM);
                    FixRouteRequest.Waypoint wa = q.waypoints.get(l.index), wb = q.waypoints.get(l.index + 1);
                    GHPoint a = new GHPoint(wa.coordinates.lat, wa.coordinates.lng), b = new GHPoint(wb.coordinates.lat, wb.coordinates.lng);
                    List<GHPoint> via = new ArrayList<>();
                    for (LatLng v : s.viaPoints) via.add(new GHPoint(v.lat, v.lng));
                    ClientRoute.Settings st = new ClientRoute.Settings(s.profile, s.customModel, q.snapPreventions);
                    ClientRoute.Leg today = ClientRoute.route(hopper, st, a, via, b, s.initialHeading, s.headingPenalty);
                    if (!today.ok()) continue;
                    List<double[]> after = new ArrayList<>();
                    for (int i = 0; i < parents.size(); i++) if (parents.get(i) == l.index && geoms.get(i) != null) after.addAll(geoms.get(i));
                    if (Boolean.getBoolean("scale.debug") && l.metrics.addedWaypoints > 0)
                        System.out.println("DBG " + id + ":" + l.index + " parents=" + parents + " afterPts=" + after.size() + " todayPts=" + today.points().size());
                    if (after.size() < 2) after = today.points();
                    int bi = 0;
                    while (bi + 1 < nb && len >= bandLo[bi + 1]) bi++;
                    Deviation.Rule rule = scaled(BASE, len);
                    boolean withinScaled = Deviation.within(today.points(), saved, rule, Deviation.Ends.of(rule, true, true));
                    boolean curActs = !l.status.equals(FixRouteResponse.OK) && !l.status.equals(FixRouteResponse.NO_ROUTE);
                    double wToday = RequestReplayTest.coWalkOff(today.points(), saved, FixRouteCorpusTest.QUALITY_TOL_M);
                    double wCur = RequestReplayTest.coWalkOff(after, saved, FixRouteCorpusTest.QUALITY_TOL_M);
                    // The new result: left as today when within the scaled rule; from CORRIDOR_FROM_M the
                    // corridor prototype; below it the current engine's result stands (an upper bound on
                    // its waypoints: the scaled rule would accept more pieces).
                    String newState;
                    int nWp;
                    double wNew;
                    long mNew;
                    List<double[]> newRoute;
                    if (withinScaled) {
                        newState = "left";
                        nWp = 0;
                        wNew = wToday;
                        mNew = 0;
                        newRoute = today.points();
                    } else if (len >= CORRIDOR_FROM_M) {
                        Proto p = corridor(st, a, b, s.initialHeading, s.headingPenalty, saved, rule, len);
                        newState = p.within() ? "corridor_ok" : "corridor_partial";
                        nWp = p.pins();
                        newRoute = p.points() == null ? today.points() : p.points();
                        wNew = RequestReplayTest.coWalkOff(newRoute, saved, FixRouteCorpusTest.QUALITY_TOL_M);
                        mNew = p.ms();
                    } else {
                        newState = "as_now";
                        nWp = l.metrics.addedWaypoints;
                        wNew = wCur;
                        mNew = l.metrics.ms;
                        newRoute = after;
                    }
                    legs[bi]++;
                    if (curActs) curFixed[bi]++;
                    curWp[bi] += l.metrics.addedWaypoints;
                    if (withinScaled && curActs) left[bi]++;
                    if (!withinScaled) still[bi]++;
                    newWp[bi] += nWp;
                    Deviation.Ends se = Deviation.Ends.of(rule, true, true);
                    devToday[bi] += offSum(today.points(), saved, rule, se);
                    devCur[bi] += offSum(after, saved, rule, se);
                    devNew[bi] += offSum(newRoute, saved, rule, se);
                    offToday[bi] += wToday;
                    offCur[bi] += wCur;
                    offNew[bi] += wNew;
                    msCur[bi] += l.metrics.ms;
                    msNew[bi] += mNew;
                    String key = id + ":" + l.index;
                    Object[] v = verdictLegs.get(key);
                    boolean changed = curActs && (withinScaled || nWp != l.metrics.addedWaypoints);
                    if (v != null && changed)
                        flips.append(String.format(Locale.ROOT, "  %s (%.1f km) owner %s/%s wp; now %s +%d; scaled: %s +%d%n",
                                key, len / 1e3, v[2], v[3], l.status, l.metrics.addedWaypoints, newState, nWp));
                    if (len >= CORRIDOR_FROM_M && (curActs || !withinScaled))
                        longLegs.append(String.format(Locale.ROOT, "  %s %.1f km: now %s +%d (%d ms, off %.2f km) -> %s +%d (%d ms, off %.2f km); today off %.2f km%n",
                                key, len / 1e3, l.status, l.metrics.addedWaypoints, l.metrics.ms, wCur / 1e3, newState, nWp, mNew, wNew / 1e3, wToday / 1e3));
                    out.write(String.format(Locale.ROOT, "{\"route\":%d,\"leg\":%d,\"len_m\":%.0f,\"status\":\"%s\",\"added\":%d,\"peak_rule_m\":%.0f,\"minlen_rule_m\":%.0f,\"new\":\"%s\",\"new_added\":%d,\"walk_today_m\":%.0f,\"walk_now_m\":%.0f,\"walk_new_m\":%.0f}%n",
                            id, l.index, len, l.status, l.metrics.addedWaypoints, rule.peakM(), rule.minLengthM(), newState, nWp, wToday, wCur, wNew));
                }
            }
        }
        System.out.println("SCALE band     legs  now-acting  now-wp | left-as-today  still-deviating  new-wp | off30-today  off30-now  off30-new | dev-today  dev-now  dev-new (km) | ms-now  ms-new");
        for (int i = 0; i < nb; i++)
            System.out.printf(Locale.ROOT, "SCALE %-7s %5d  %10d  %6d | %13d  %15d  %6d | %11.1f  %9.1f  %9.1f | %9.1f  %7.1f  %7.1f | %6d  %6d%n",
                    band[i], legs[i], curFixed[i], curWp[i], left[i], still[i], newWp[i], offToday[i] / 1e3, offCur[i] / 1e3, offNew[i] / 1e3,
                    devToday[i] / 1e3, devCur[i] / 1e3, devNew[i] / 1e3, msCur[i], msNew[i]);
        System.out.println("SCALE owner verdicts that change:\n" + flips);
        System.out.println("SCALE legs >= 10 km that act now or still deviate:\n" + longLegs);
    }
}
