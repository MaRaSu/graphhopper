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
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.TreeMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * THE critical validation for {@code /fix_route} (design doc §7), over the real-route corpus:
 * <ol>
 *   <li>fix the route;</li>
 *   <li>apply the response the way the client does (replace each leg by its segments, mint ids for
 *       new waypoints, inherit profile / custom model);</li>
 *   <li>simulate the client's re-route: route every followRoads segment exactly as the client
 *       sends it, and overwrite its start / end waypoint with GH's snapped route end (the client does
 *       this after every {@code /route} unless the neighbour is a direct / coordinates leg);</li>
 *   <li>fix again — every leg must now be {@code ok} with 0 added waypoints ({@code unroutable}
 *       legs: unroutable again with 0 added; unaligned / skipped stay as they are).</li>
 * </ol>
 * Step 4 routes each segment with the client's exact request, so passing it also proves parity:
 * what the client draws follows the saved track.
 */
public class FixRouteCorpusTest {

    static final String OUT_DIR = "../../data/fix-route-out/fix" + (System.getProperty("fix.engine") == null ? "" : "-" + System.getProperty("fix.engine"));
    private static GraphHopper hopper;
    private static ExecutorService pool;

    @BeforeAll
    static void setup() throws Exception {
        Assumptions.assumeTrue(FixRouteFixtures.available(), "fixtures not found");
        Assumptions.assumeTrue(FixRouteTestGraph.available(), "graph cache not found");
        hopper = FixRouteTestGraph.open();
        pool = Executors.newFixedThreadPool(Integer.getInteger("fix.threads",
                Math.max(2, Runtime.getRuntime().availableProcessors() - 2)));
    }

    @AfterAll
    static void teardown() {
        if (pool != null) pool.shutdown();
        if (hopper != null) hopper.close();
    }

    /** Fixture → request, exactly the fields the client sends. */
    static FixRouteRequest request(FixRouteFixtures.Route r) {
        FixRouteRequest q = new FixRouteRequest();
        q.reference = r.reference;
        for (int i = 0; i < r.waypoints.size(); i++) {
            q.waypoints.add(new FixRouteRequest.Waypoint(r.ids.get(i), r.waypoints.get(i)[0], r.waypoints.get(i)[1]));
        }
        for (FixRouteFixtures.Segment f : r.segments) {
            FixRouteRequest.Segment s = new FixRouteRequest.Segment();
            s.id = f.id;
            s.start = r.ids.get(f.startWaypointIndex);
            s.end = r.ids.get(f.endWaypointIndex);
            s.type = f.type;
            s.profile = f.profile;
            s.customModel = FixRouteTestGraph.customModel(f.customModel);
            for (double[] v : f.viaPoints) s.viaPoints.add(new LatLng(v[0], v[1]));
            s.initialHeading = f.initialHeading;
            s.headingPenalty = f.headingPenalty;
            if (!f.trackCoordinates.isEmpty()) {
                s.trackCoordinates = new ArrayList<>();
                for (double[] t : f.trackCoordinates) s.trackCoordinates.add(new LatLng(t[0], t[1]));
            }
            s.savedLengthM = f.savedLengthM;
            q.segments.add(s);
        }
        q.snapPreventions = r.snapPreventions;
        if (System.getProperty("fix.longLeg") != null) {
            q.options = q.options == null ? new FixRouteRequest.Options() : q.options;
            q.options.fixLongLegM = Double.parseDouble(System.getProperty("fix.longLeg"));
        }
        if (System.getProperty("fix.corridorGrowth") != null) {
            q.options = q.options == null ? new FixRouteRequest.Options() : q.options;
            q.options.fixLongCorridorGrowth = Double.parseDouble(System.getProperty("fix.corridorGrowth"));
        }
        return q;
    }

    /** Client-side application of a response + the client's re-route snap overwrite. */
    static FixRouteRequest applyAndReroute(FixRouteRequest q, FixRouteResponse rsp) {
        FixRouteRequest n = new FixRouteRequest();
        n.reference = q.reference;
        n.snapPreventions = q.snapPreventions;
        n.options = q.options;
        n.unroutablePolicy = q.unroutablePolicy;
        n.debug = q.debug;
        Map<String, FixRouteRequest.Waypoint> byId = new LinkedHashMap<>();
        for (FixRouteRequest.Waypoint w : q.waypoints) byId.put(w.id, w);
        int minted = 0;
        List<Integer> parents = new ArrayList<>();
        n.waypoints.add(copy(q.waypoints.get(0)));
        for (FixRouteResponse.Leg leg : rsp.legs) {
            FixRouteRequest.Segment parent = q.segments.get(leg.index);
            for (FixRouteResponse.Segment s : leg.segments) {
                FixRouteRequest.Waypoint end;
                if (s.end.waypointId != null) end = copy(byId.get(s.end.waypointId));
                else end = new FixRouteRequest.Waypoint("new-" + (minted++), s.end.newPoint.lat, s.end.newPoint.lng);
                FixRouteRequest.Segment ns = new FixRouteRequest.Segment();
                ns.id = "seg-" + n.segments.size();
                ns.start = n.waypoints.get(n.waypoints.size() - 1).id;
                ns.end = end.id;
                ns.type = s.type;
                ns.profile = parent.profile;
                ns.customModel = parent.customModel;
                ns.headingPenalty = s.initialHeading != null ? RouteFixer.CLIENT_HEADING_PENALTY : null;
                ns.initialHeading = s.initialHeading;
                if (s.viaPoints != null) ns.viaPoints = new ArrayList<>(s.viaPoints);
                ns.trackCoordinates = s.trackCoordinates;
                n.segments.add(ns);
                n.waypoints.add(end);
                parents.add(leg.index);
            }
        }
        // Keep the server-returned coordinates (before the client's snap overwrite) for diagnostics.
        List<double[][]> before = new ArrayList<>();
        for (int i = 0; i < n.segments.size(); i++) {
            FixRouteRequest.Waypoint a0 = n.waypoints.get(i), b0 = n.waypoints.get(i + 1);
            before.add(new double[][]{{a0.coordinates.lat, a0.coordinates.lng}, {b0.coordinates.lat, b0.coordinates.lng}});
        }
        LAST_BEFORE.set(before);
        LAST_PARENT.set(parents);
        List<List<double[]>> geoms = new ArrayList<>();
        List<ClientRoute.Leg> legs = new ArrayList<>();
        for (int i = 0; i < n.segments.size(); i++) {
            geoms.add(null);
            legs.add(null);
        }
        LAST_GEOM.set(geoms);
        LAST_LEGS.set(legs);
        LAST_NEWWP.set(n);
        // Client re-route: each followRoads segment routed as the client sends it; its end points
        // overwrite the waypoints (1e-6°), except next to direct / coordinates legs.
        for (int i = 0; i < n.segments.size(); i++) {
            FixRouteRequest.Segment s = n.segments.get(i);
            if (!s.isFollowRoads()) continue;
            FixRouteRequest.Waypoint a = n.waypoints.get(i), b = n.waypoints.get(i + 1);
            List<GHPoint> via = new ArrayList<>();
            for (LatLng v : s.viaPoints) via.add(new GHPoint(v.lat, v.lng));
            ClientRoute.Leg leg = ClientRoute.route(hopper,
                    new ClientRoute.Settings(s.profile, s.customModel, n.snapPreventions),
                    new GHPoint(a.coordinates.lat, a.coordinates.lng), via,
                    new GHPoint(b.coordinates.lat, b.coordinates.lng), s.initialHeading, s.headingPenalty);
            if (!leg.ok()) continue;
            s.savedLengthM = leg.distanceM();
            geoms.set(i, leg.points());
            legs.set(i, leg);
            boolean prevRouted = i == 0 || n.segments.get(i - 1).isFollowRoads();
            boolean nextRouted = i + 1 == n.segments.size() || n.segments.get(i + 1).isFollowRoads();
            double[] ps = leg.points().get(0), pe = leg.points().get(leg.points().size() - 1);
            if (prevRouted) a.coordinates = new LatLng(RouteFixer.round6(ps[0]), RouteFixer.round6(ps[1]));
            if (nextRouted) b.coordinates = new LatLng(RouteFixer.round6(pe[0]), RouteFixer.round6(pe[1]));
        }
        return n;
    }

    /**
     * The owner's verdicts from the review rounds (2026-09-24/25), as standing checks: leg → the
     * status it must get ("ok" / "fixed" / "unroutable", or "*" = any) and the number of waypoints
     * the owner approved (-1 = any). A mismatch fails the corpus run: it changes something the owner
     * has judged, and must go back to him with a before/after.
     */
    static final Object[][] OWNER_VERDICTS = {
            {16153L, 17, "*", 1}, {16153L, 18, "*", 1}, {34580L, 0, "*", 3}, {92304L, 2, "ok", 0},
            {44932L, 13, "ok", 0}, {42833L, 7, "*", 1}, {15171L, 61, "*", 1}, {20630L, 15, "*", 3},
            {34549L, 0, "*", 1}, {76851L, 6, "*", 1}, {10122L, 50, "*", 1}, {41405L, 37, "fixed", 1},
            {49302L, 176, "unroutable", -1}, {45039L, 55, "fixed", -1},
            {49302L, 106, "ok", 0}, {35670L, 14, "ok", 0}, {15171L, 64, "ok", 0}, {49302L, 206, "ok", 0},
            {13909L, 15, "ok", 0}, {32997L, 8, "fixed", -1}, {15603L, 44, "fixed", -1},
            {29326L, 9, "ok", 0}, {49302L, 158, "ok", 0}, {92304L, 80, "ok", 0}, {90688L, 313, "ok", 0},
            {44932L, 12, "ok", 0}, {49302L, 124, "ok", 0}, {49302L, 138, "ok", 0},
            // review of 2026-09-29 (ba4)
            {13909L, 14, "*", 1}, {15603L, 2, "ok", 0}, {49302L, 176, "unroutable", 1},
            {42235L, 0, "*", -1}, {44932L, 14, "*", 1},
            // long-leg review of 2026-09-29 (via points, no new waypoints): 63295:1 and 42235:3 "very good",
            // 72540:22 with the service/track fallback, 32997:0 ok (routing-profile evolution)
            {63295L, 1, "fixed", 0}, {42235L, 3, "fixed", 0}, {72540L, 22, "fixed", 0}, {32997L, 0, "fixed", 0},
    };

    /** Owner verdicts (2026-09-29): the route must not turn back at any waypoint the fix added. */
    static final String[] NO_UTURN_VERDICTS = {"32997:1", "42235:0", "44932:14"};
    static final Map<String, Integer> UTURNS_BY_LEG = new java.util.HashMap<>();

    static final double[] WALK = new double[2], UTURNS = new double[2];

    /** The quality measure's distance [m]: saved route farther than this from the route "diverges". */
    static final double QUALITY_TOL_M = 30;

    /**
     * The quality measure (the one the client team scores with): per aligned followRoads leg, the
     * length of saved route farther than {@link #QUALITY_TOL_M} from the client's route today and
     * from the route after applying the fix. One JSON line per leg; returns the route's totals [m].
     */
    static double[] quality(long id, GraphHopper hopper, FixRouteRequest q, FixRouteResponse r1, Writer out) throws java.io.IOException {
        ReferenceTrack ref = new ReferenceTrack(q.reference);
        List<Integer> parents = LAST_PARENT.get();
        List<List<double[]>> geoms = LAST_GEOM.get();
        List<ClientRoute.Leg> legs = LAST_LEGS.get();
        FixRouteRequest applied = LAST_NEWWP.get();
        double tb = 0, ta = 0;
        Map<Integer, ClientRoute.Leg> todayByLeg = new java.util.HashMap<>();
        for (FixRouteResponse.Leg l : r1.legs) {
            if (l.metrics.refFromM == null || l.metrics.refToM == null) continue;
            FixRouteRequest.Segment s = q.segments.get(l.index);
            if (!s.isFollowRoads() || l.metrics.refToM - l.metrics.refFromM < 1) continue;
            List<double[]> saved = ref.slice(l.metrics.refFromM, l.metrics.refToM);
            List<double[]> after = new ArrayList<>();
            for (int i = 0; i < parents.size(); i++) if (parents.get(i) == l.index && geoms.get(i) != null) after.addAll(geoms.get(i));
            FixRouteRequest.Waypoint a = q.waypoints.get(l.index), b = q.waypoints.get(l.index + 1);
            List<GHPoint> via = new ArrayList<>();
            for (LatLng v : s.viaPoints) via.add(new GHPoint(v.lat, v.lng));
            ClientRoute.Leg today = ClientRoute.route(hopper, new ClientRoute.Settings(s.profile, s.customModel, q.snapPreventions),
                    new GHPoint(a.coordinates.lat, a.coordinates.lng), via, new GHPoint(b.coordinates.lat, b.coordinates.lng),
                    s.initialHeading, s.headingPenalty);
            if (!today.ok() || after.size() < 2) continue;
            double db = RequestReplayTest.farLength(saved, today.points(), QUALITY_TOL_M);
            double da = RequestReplayTest.farLength(saved, after, QUALITY_TOL_M);
            double wb = RequestReplayTest.coWalkOff(today.points(), saved, QUALITY_TOL_M);
            double wa = RequestReplayTest.coWalkOff(after, saved, QUALITY_TOL_M);
            // U-turns at waypoints the fix added: the route arrives and leaves on the same road.
            int uturns = 0;
            double uturnM = 0;
            for (int i = 0; i + 1 < parents.size(); i++) {
                if (parents.get(i) != l.index || parents.get(i + 1) != l.index) continue;
                if (!applied.waypoints.get(i + 1).id.startsWith("new-") || legs.get(i) == null || legs.get(i + 1) == null) continue;
                double back = RequestReplayTest.retrace(legs.get(i), legs.get(i + 1));
                if (back > 0) {
                    uturns++;
                    uturnM += back;
                }
            }
            // U-turn at the leg's own start waypoint: the previous leg's arrival vs this leg's departure,
            // today and after the fix (legs are routed independently, so a fix can create one).
            double wpToday = -1, wpAfter = -1;
            ClientRoute.Leg prevToday = todayByLeg.get(l.index - 1);
            int firstSub = parents.indexOf(l.index), lastPrevSub = parents.lastIndexOf(l.index - 1);
            if (prevToday != null && firstSub >= 0 && lastPrevSub >= 0 && legs.get(firstSub) != null && legs.get(lastPrevSub) != null) {
                wpToday = RequestReplayTest.retrace(prevToday, today);
                wpAfter = RequestReplayTest.retrace(legs.get(lastPrevSub), legs.get(firstSub));
                if (wpAfter > wpToday + WP_UTURN_SLACK_M) {
                    WP_UTURNS[0]++;
                    WP_UTURNS[1] += wpAfter - wpToday;
                    WP_UTURN_LEGS.add(id + ":" + l.index);
                }
            }
            todayByLeg.put(l.index, today);
            tb += db;
            ta += da;
            WALK[0] += wb;
            WALK[1] += wa;
            UTURNS_BY_LEG.put(id + ":" + l.index, uturns);
            UTURNS[0] += uturns;
            UTURNS[1] += uturnM;
            out.write(String.format(Locale.ROOT, "{\"route\":%d,\"leg\":%d,\"status\":\"%s\",\"added\":%d,\"saved_m\":%.0f,\"div_today_m\":%.0f,\"div_after_m\":%.0f,\"walk_today_m\":%.0f,\"walk_after_m\":%.0f,\"uturns\":%d,\"uturn_m\":%.0f,\"wp_uturn_today_m\":%.0f,\"wp_uturn_after_m\":%.0f}%n",
                    id, l.index, l.status, l.metrics.addedWaypoints, l.metrics.refToM - l.metrics.refFromM, db, da, wb, wa, uturns, uturnM, wpToday, wpAfter));
        }
        return new double[]{tb, ta};
    }

    /** A U-turn at an original waypoint counts as created by the fix when it grows by more than this [m]. */
    static final double WP_UTURN_SLACK_M = 5;
    static final double[] WP_UTURNS = new double[2];
    static final List<String> WP_UTURN_LEGS = java.util.Collections.synchronizedList(new ArrayList<>());

    static final ThreadLocal<List<ClientRoute.Leg>> LAST_LEGS = new ThreadLocal<>();
    static final ThreadLocal<FixRouteRequest> LAST_NEWWP = new ThreadLocal<>();
    static final ThreadLocal<List<double[][]>> LAST_BEFORE = new ThreadLocal<>();
    static final ThreadLocal<List<Integer>> LAST_PARENT = new ThreadLocal<>();
    static final ThreadLocal<List<List<double[]>>> LAST_GEOM = new ThreadLocal<>();

    /** Per fixed / unroutable leg: one JSON line of facts + a GeoJSON (saved slice, final route, new waypoints). */
    static void exportLegs(long id, FixRouteRequest q, FixRouteResponse r1, FixRouteRequest q2, Writer jsonl) throws java.io.IOException {
        File dir = new File(OUT_DIR, "legs");
        dir.mkdirs();
        ReferenceTrack ref = new ReferenceTrack(q.reference);
        List<Integer> parents = LAST_PARENT.get();
        List<List<double[]>> geoms = LAST_GEOM.get();
        com.fasterxml.jackson.databind.ObjectMapper om = new com.fasterxml.jackson.databind.ObjectMapper();
        for (FixRouteResponse.Leg l : r1.legs) {
            if (!l.status.equals(FixRouteResponse.FIXED) && !l.status.equals(FixRouteResponse.UNROUTABLE)) continue;
            if (l.metrics.refFromM == null || l.metrics.refToM == null) continue;
            List<double[]> saved = ref.slice(l.metrics.refFromM, l.metrics.refToM);
            List<double[]> fin = new ArrayList<>();
            List<double[]> newWps = new ArrayList<>();
            for (int i = 0; i < parents.size(); i++) {
                if (parents.get(i) != l.index) continue;
                if (geoms.get(i) != null) fin.addAll(geoms.get(i));
                FixRouteRequest.Waypoint w = q2.waypoints.get(i + 1);
                if (w.id.startsWith("new-")) newWps.add(new double[]{w.coordinates.lat, w.coordinates.lng});
            }
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("route", id);
            m.put("leg", l.index);
            m.put("status", l.status);
            m.put("profile", q.segments.get(l.index).profile);
            m.put("added", l.metrics.addedWaypoints);
            m.put("saved_m", Math.round(PathSimilarity.length(saved)));
            m.put("final_m", Math.round(PathSimilarity.length(fin)));
            m.put("segments", l.segments.size());
            List<Map<String, Object>> un = new ArrayList<>();
            if (l.unroutable != null) for (FixRouteResponse.Unroutable u : l.unroutable) {
                Map<String, Object> um = new LinkedHashMap<>();
                um.put("from", u.refFromM);
                um.put("to", u.refToM);
                um.put("len", u.referenceLengthM);
                um.put("reroute", u.rerouteLengthM);
                um.put("cause", u.cause);
                un.add(um);
            }
            m.put("unroutable", un);
            if (l.debug != null && l.debug.get("steps") != null) m.put("steps", l.debug.get("steps"));
            jsonl.write(om.writeValueAsString(m) + "\n");
            StringBuilder g = new StringBuilder("{\"type\":\"FeatureCollection\",\"features\":[");
            g.append(line(saved, "saved")).append(',').append(line(fin, "final"));
            for (double[] w : newWps) g.append(',').append(point(w, "new_waypoint"));
            if (l.unroutable != null) for (FixRouteResponse.Unroutable u : l.unroutable)
                g.append(',').append(line(ref.slice(u.refFromM, u.refToM), "unroutable_" + u.cause));
            g.append("],\"properties\":").append(om.writeValueAsString(m)).append('}');
            try (Writer w = new FileWriter(new File(dir, "route-" + id + "_leg-" + l.index + ".geojson"))) {
                w.write(g.toString());
            }
        }
    }

    private static String line(List<double[]> pts, String name) {
        StringBuilder sb = new StringBuilder("{\"type\":\"Feature\",\"properties\":{\"name\":\"" + name + "\"},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
        for (int i = 0; i < pts.size(); i++) {
            if (i > 0) sb.append(',');
            sb.append(String.format(Locale.ROOT, "[%.6f,%.6f]", pts.get(i)[1], pts.get(i)[0]));
        }
        return sb.append("]}}").toString();
    }

    private static String point(double[] p, String name) {
        return String.format(Locale.ROOT, "{\"type\":\"Feature\",\"properties\":{\"name\":\"%s\"},\"geometry\":{\"type\":\"Point\",\"coordinates\":[%.6f,%.6f]}}", name, p[1], p[0]);
    }

    private static FixRouteRequest.Waypoint copy(FixRouteRequest.Waypoint w) {
        return new FixRouteRequest.Waypoint(w.id, w.coordinates.lat, w.coordinates.lng);
    }

    @Test
    void fixApplyRerouteFixAgain() throws Exception {
        String only = System.getProperty("fix.routes");
        List<Long> ids = new ArrayList<>();
        if (only != null && !only.isBlank()) for (String s : only.split(",")) ids.add(Long.parseLong(s.trim()));
        else ids = FixRouteFixtures.allIds();
        new File(OUT_DIR).mkdirs();

        Map<String, Integer> firstTotals = new TreeMap<>(), secondTotals = new TreeMap<>(), causes = new TreeMap<>();
        Map<String, Integer> acceptedTotals = new java.util.TreeMap<>();
        int addedTotal = 0, violations = 0, legsTotal = 0;
        double divBefore = 0, divAfter = 0;
        Map<String, FixRouteResponse.Leg> firstPass = new java.util.HashMap<>();
        long msTotal = 0, msMax = 0;
        long maxRoute = -1;
        StringBuilder bad = new StringBuilder();
        try (Writer w = new FileWriter(new File(OUT_DIR, "routes.csv"));
             Writer jsonl = new FileWriter(new File(OUT_DIR, "legs.jsonl"));
             Writer qual = new FileWriter(new File(OUT_DIR, "quality.jsonl"))) {
            w.write("route,legs,ok,fixed,unroutable,unaligned,skipped,not_processed,added,ms,second_ok,second_violations\n");
            for (long id : ids) {
                FixRouteFixtures.Route r = FixRouteFixtures.load(id);
                FixRouteRequest q = request(r);
                q.debug = Boolean.getBoolean("fix.verbose");
                if (System.getProperty("fix.tol") != null) q.options.materialityMaxM = Double.parseDouble(System.getProperty("fix.tol"));
                if (System.getProperty("fix.policy") != null) q.unroutablePolicy = System.getProperty("fix.policy");
                if (System.getProperty("fix.engine") != null) q.options.engine = System.getProperty("fix.engine");
                FixRouteResponse r1 = FixRouteEngine.fix(hopper, pool, q);
                for (FixRouteResponse.Leg l : r1.legs) firstPass.put(id + ":" + l.index, l);
                FixRouteRequest q2 = applyAndReroute(q, r1);
                List<double[][]> before = LAST_BEFORE.get();
                FixRouteResponse r2 = FixRouteEngine.fix(hopper, pool, q2);
                exportLegs(id, q, r1, q2, jsonl);
                double[] dq = quality(id, hopper, q, r1, qual);
                divBefore += dq[0];
                divAfter += dq[1];

                r1.stats.byStatus.forEach((k, v) -> firstTotals.merge(k, v, Integer::sum));
                for (FixRouteResponse.Leg l : r1.legs) acceptedTotals.merge(l.status + "/" + l.metrics.acceptedBy, 1, Integer::sum);
                r2.stats.byStatus.forEach((k, v) -> secondTotals.merge(k, v, Integer::sum));
                for (FixRouteResponse.Leg l : r1.legs)
                    if (l.unroutable != null) for (FixRouteResponse.Unroutable u : l.unroutable) causes.merge(u.cause, 1, Integer::sum);
                if (Boolean.getBoolean("fix.verbose")) {
                    com.fasterxml.jackson.databind.ObjectWriter ow = new com.fasterxml.jackson.databind.ObjectMapper()
                            .writerWithDefaultPrettyPrinter();
                    for (FixRouteResponse.Leg l : r1.legs)
                        if (!l.status.equals(FixRouteResponse.OK) && !l.status.equals(FixRouteResponse.SKIPPED))
                            System.out.println("R1 route-" + id + " " + ow.writeValueAsString(l));
                    for (FixRouteResponse.Leg l : r2.legs)
                        if (!l.status.equals(FixRouteResponse.OK) && !l.status.equals(FixRouteResponse.SKIPPED))
                            System.out.println("R2 route-" + id + " " + ow.writeValueAsString(l));
                }
                int v2 = 0;
                if (q.debug && r1.debug != null) System.out.println("TIMING route-" + id + " total=" + r1.stats.totalMs + " " + r1.debug);
                if (q.debug) {
                    com.fasterxml.jackson.databind.ObjectMapper om = new com.fasterxml.jackson.databind.ObjectMapper();
                    for (FixRouteResponse.Leg l : r1.legs) System.out.println("Q1 route-" + id + " leg " + l.index + " " + l.status + " " + om.writeValueAsString(l.debug));
                }
                for (FixRouteResponse.Leg l : r2.legs) {
                    boolean okLeg = l.status.equals(FixRouteResponse.OK)
                            || l.status.equals(FixRouteResponse.SKIPPED)
                            || l.status.equals(FixRouteResponse.UNALIGNED)
                            || l.status.equals(FixRouteResponse.NO_ROUTE)
                            || (l.status.equals(FixRouteResponse.UNROUTABLE) && l.metrics.addedWaypoints == 0);
                    if (!okLeg) {
                        v2++;
                        if (q.debug) {
                            FixRouteRequest.Segment s2 = q2.segments.get(l.index);
                            FixRouteRequest.Waypoint a2 = q2.waypoints.get(l.index), b2 = q2.waypoints.get(l.index + 1);
                            double[][] bf = before.get(l.index);
                            ClientRoute.Settings cs = new ClientRoute.Settings(s2.profile, s2.customModel, q2.snapPreventions);
                            ClientRoute.Leg asReturned = ClientRoute.route(hopper, cs, new GHPoint(bf[0][0], bf[0][1]), List.of(),
                                    new GHPoint(bf[1][0], bf[1][1]), s2.initialHeading, s2.headingPenalty);
                            ClientRoute.Leg afterSnap = ClientRoute.route(hopper, cs, new GHPoint(a2.coordinates.lat, a2.coordinates.lng), List.of(),
                                    new GHPoint(b2.coordinates.lat, b2.coordinates.lng), s2.initialHeading, s2.headingPenalty);
                            System.out.printf(Locale.ROOT, "DIAG route-%d leg %d heading=%s | returned %s -> %s len=%.1f | after-snap %s -> %s len=%.1f | moved start %.2fm end %.2fm%n",
                                    id, l.index, s2.initialHeading, java.util.Arrays.toString(bf[0]), java.util.Arrays.toString(bf[1]),
                                    asReturned.ok() ? asReturned.distanceM() : -1,
                                    a2.coordinates.lat + "," + a2.coordinates.lng, b2.coordinates.lat + "," + b2.coordinates.lng,
                                    afterSnap.ok() ? afterSnap.distanceM() : -1,
                                    com.graphhopper.util.DistanceCalcEarth.DIST_EARTH.calcDist(bf[0][0], bf[0][1], a2.coordinates.lat, a2.coordinates.lng),
                                    com.graphhopper.util.DistanceCalcEarth.DIST_EARTH.calcDist(bf[1][0], bf[1][1], b2.coordinates.lat, b2.coordinates.lng));
                        }
                        if (bad.length() < 20_000)
                            bad.append(String.format(Locale.ROOT, "  route-%d leg %d: %s (+%d wp) %s%n",
                                    id, l.index, l.status, l.metrics.addedWaypoints, l.debug == null ? "" : l.debug));
                    }
                }
                violations += v2;
                legsTotal += r1.legs.size();
                addedTotal += r1.stats.addedWaypoints;
                msTotal += r1.stats.totalMs;
                if (r1.stats.totalMs > msMax) {
                    msMax = r1.stats.totalMs;
                    maxRoute = id;
                }
                Map<String, Integer> b = r1.stats.byStatus;
                w.write(String.format(Locale.ROOT, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d%n", id, r1.legs.size(),
                        b.getOrDefault("ok", 0), b.getOrDefault("fixed", 0), b.getOrDefault("unroutable", 0),
                        b.getOrDefault("unaligned", 0), b.getOrDefault("skipped", 0), b.getOrDefault("not_processed", 0),
                        r1.stats.addedWaypoints, r1.stats.totalMs, r2.stats.byStatus.getOrDefault("ok", 0), v2));
                w.flush();
            }
        }
        System.out.println("FIXROUTE first pass  " + firstTotals + " added_waypoints=" + addedTotal + " legs=" + legsTotal);
        System.out.println("FIXROUTE first pass by rule " + acceptedTotals);
        System.out.println("FIXROUTE unroutable causes " + causes);
        System.out.println("FIXROUTE second pass " + secondTotals + " violations=" + violations);
        System.out.printf(Locale.ROOT, "FIXROUTE quality: saved route >%.0f m from the route: today %.1f km, after the fix %.1f km%n",
                QUALITY_TOL_M, divBefore / 1000, divAfter / 1000);
        System.out.printf(Locale.ROOT, "FIXROUTE quality: route off the saved route, walked in order: today %.1f km, after the fix %.1f km; U-turns at added waypoints: %d (%.0f m ridden back)%n",
                WALK[0] / 1000, WALK[1] / 1000, (int) UTURNS[0], UTURNS[1]);
        System.out.printf(Locale.ROOT, "FIXROUTE quality: U-turns the fix creates at original waypoints: %d (+%.0f m ridden back) %s%n",
                (int) WP_UTURNS[0], WP_UTURNS[1], WP_UTURN_LEGS);
        System.out.printf(Locale.ROOT, "FIXROUTE time total %.1fs, slowest route %d at %.1fs%n", msTotal / 1e3, maxRoute, msMax / 1e3);
        if (violations > 0) System.out.println("FIXROUTE violations:\n" + bad);
        int checked = 0;
        StringBuilder verdicts = new StringBuilder();
        for (Object[] v : OWNER_VERDICTS) {
            FixRouteResponse.Leg l = firstPass.get(v[0] + ":" + v[1]);
            if (l == null) continue; // route not in this run
            checked++;
            boolean st = "*".equals(v[2]) || v[2].equals(l.status);
            boolean wp = (Integer) v[3] < 0 || (Integer) v[3] == l.metrics.addedWaypoints;
            if (!st || !wp) verdicts.append(String.format(Locale.ROOT, "  %s:%s owner %s / %s waypoints, now %s / %d%n",
                    v[0], v[1], v[2], v[3], l.status, l.metrics.addedWaypoints));
        }
        for (String k : NO_UTURN_VERDICTS) {
            Integer u = UTURNS_BY_LEG.get(k);
            if (u == null) continue;
            checked++;
            if (u > 0) verdicts.append(String.format(Locale.ROOT, "  %s owner: no U-turn at added waypoints, now %d%n", k, u));
        }
        System.out.println("FIXROUTE owner verdicts: " + (checked - verdicts.toString().split("\n", -1).length + 1) + "/" + checked + " hold"
                + (verdicts.length() > 0 ? "\n" + verdicts : ""));
        if (only == null || only.isBlank()) {
            org.junit.jupiter.api.Assertions.assertEquals(0, violations, "second-pass violations");
            org.junit.jupiter.api.Assertions.assertEquals("", verdicts.toString(), "owner verdicts changed");
        }
    }
}
