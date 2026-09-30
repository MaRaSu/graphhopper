package com.graphhopper.trailmap.fixroute;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.graphhopper.GraphHopper;
import com.graphhopper.jackson.Jackson;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.io.File;

/**
 * Replays a saved {@code /trailmap/fix_route} request body ({@code -Dfix.request=path.json}) against
 * the local graph with debug on, printing each leg's status, stretches and deviation measurements —
 * to reproduce what the client saw on gh-test. Local diagnostic only.
 */
public class RequestReplayTest {

    @Test
    void replay() throws Exception {
        String path = System.getProperty("fix.request");
        Assumptions.assumeTrue(path != null && FixRouteTestGraph.available());
        ObjectMapper om = Jackson.newObjectMapper();
        FixRouteRequest q = om.readValue(new File(path), FixRouteRequest.class);
        q.debug = true;
        GraphHopper hopper = FixRouteTestGraph.open();
        try {
            FixRouteResponse rsp = FixRouteEngine.fix(hopper, null, q);
            for (FixRouteResponse.Leg l : rsp.legs) {
                if (FixRouteResponse.OK.equals(l.status) || FixRouteResponse.SKIPPED.equals(l.status)) continue;
                StringBuilder sb = new StringBuilder("REPLAY leg " + l.index + " " + l.status + " +" + l.metrics.addedWaypoints
                        + " ref " + l.metrics.refFromM + ".." + l.metrics.refToM);
                if (l.unroutable != null) for (FixRouteResponse.Unroutable u : l.unroutable)
                    sb.append(String.format(java.util.Locale.ROOT, " | stretch %.0f..%.0f len %.0f reroute %s %s",
                            u.refFromM, u.refToM, u.referenceLengthM, u.rerouteLengthM, u.cause));
                if (l.debug != null) {
                    sb.append(" | deviations ").append(l.debug.get("deviations"));
                    sb.append(" | steps ").append(l.debug.get("steps"));
                    sb.append(" | unmatched ").append(l.debug.get("unmatched_regions"));
                    sb.append(" | start_match ").append(l.debug.get("start_match"));
                    sb.append(" | prune ").append(l.debug.get("prune"));
                    if (l.waypointSnapMismatch != null) sb.append(" | MISMATCH offset ").append(l.waypointSnapMismatch.offsetM);
                }
                System.out.println(sb);
            }
            System.out.println("REPLAY stats added=" + rsp.stats.addedWaypoints + " " + rsp.stats.byStatus);
            // -Dfix.dump=legIndex: saved slice, the client's route of the leg, matched paths → GeoJSON.
            String dump = System.getProperty("fix.dump");
            if (dump != null) {
                int li = Integer.parseInt(dump);
                FixRouteResponse.Leg l = rsp.legs.get(li);
                FixRouteRequest.Segment s = q.segments.get(li);
                FixRouteRequest.Waypoint a = q.waypoints.get(li), b = q.waypoints.get(li + 1);
                java.util.List<com.graphhopper.util.shapes.GHPoint> via = new java.util.ArrayList<>();
                if (s.viaPoints != null) for (FixRouteRequest.LatLng v : s.viaPoints) via.add(new com.graphhopper.util.shapes.GHPoint(v.lat, v.lng));
                ClientRoute.Leg cr = ClientRoute.route(hopper, new ClientRoute.Settings(s.profile, s.customModel, q.snapPreventions),
                        new com.graphhopper.util.shapes.GHPoint(a.coordinates.lat, a.coordinates.lng), via,
                        new com.graphhopper.util.shapes.GHPoint(b.coordinates.lat, b.coordinates.lng), s.initialHeading, s.headingPenalty);
                java.util.List<double[]> saved = new ReferenceTrack(q.reference).slice(l.metrics.refFromM, l.metrics.refToM);
                StringBuilder g = new StringBuilder("{\"type\":\"FeatureCollection\",\"features\":[");
                g.append(line(saved, "saved"));
                if (cr.ok()) g.append(',').append(line(cr.points(), "client_route"));
                Object sl = l.debug == null ? null : l.debug.get("saved_leg");
                if (sl instanceof java.util.List<?>) {
                    @SuppressWarnings("unchecked") java.util.List<double[]> p = (java.util.List<double[]>) sl;
                    g.append(',').append(line(p, "engine_saved"));
                }
                System.out.println("REPLAY engine saved arcs " + (l.debug == null ? null : l.debug.get("saved_leg_arcs")));
                Object mp = l.debug == null ? null : l.debug.get("matched_paths");
                if (mp instanceof java.util.List<?> lists) for (Object o : lists) {
                    @SuppressWarnings("unchecked") java.util.List<double[]> p = (java.util.List<double[]>) o;
                    g.append(',').append(line(p, "matched"));
                }
                g.append(String.format(java.util.Locale.ROOT, ",{\"type\":\"Feature\",\"properties\":{\"name\":\"wp_start\"},\"geometry\":{\"type\":\"Point\",\"coordinates\":[%.6f,%.6f]}}", a.coordinates.lng, a.coordinates.lat));
                g.append(String.format(java.util.Locale.ROOT, ",{\"type\":\"Feature\",\"properties\":{\"name\":\"wp_end\"},\"geometry\":{\"type\":\"Point\",\"coordinates\":[%.6f,%.6f]}}", b.coordinates.lng, b.coordinates.lat));
                g.append("]}");
                String out = System.getProperty("fix.dumpfile", "replay-leg.geojson");
                try (java.io.Writer w = new java.io.FileWriter(out)) {
                    w.write(g.toString());
                }
                System.out.println("REPLAY dumped leg " + li + " -> " + out);
            }
        } finally {
            hopper.close();
        }
    }

    /**
     * {@code -Dfix.score=path1,path2,...}: per request, the saved-route length [km] farther than 30 m
     * from the returned route (per leg, both engines' kind of "divergent km"), plus waypoints added
     * and km reported unroutable. Run once per build configuration to compare them.
     */
    @Test
    void score() throws Exception {
        String prop = System.getProperty("fix.score");
        Assumptions.assumeTrue(prop != null && FixRouteTestGraph.available());
        ObjectMapper om = Jackson.newObjectMapper();
        GraphHopper hopper = FixRouteTestGraph.open();
        try {
            for (String path : prop.split(",")) {
                FixRouteRequest q = om.readValue(new File(path), FixRouteRequest.class);
                FixRouteResponse rsp = FixRouteEngine.fix(hopper, null, q);
                ReferenceTrack ref = new ReferenceTrack(q.reference);
                java.util.Map<String, double[]> byId = new java.util.HashMap<>();
                for (FixRouteRequest.Waypoint w : q.waypoints) byId.put(w.id, new double[]{w.coordinates.lat, w.coordinates.lng});
                double div = 0, un = 0, uturnM = 0, walkOff = 0;
                int uturns = 0;
                StringBuilder uturnList = new StringBuilder();
                StringBuilder worst = new StringBuilder();
                for (FixRouteResponse.Leg l : rsp.legs) {
                    if (l.unroutable != null) for (FixRouteResponse.Unroutable u : l.unroutable) un += u.referenceLengthM;
                    if (l.metrics.refFromM == null || l.metrics.refToM == null) continue;
                    FixRouteRequest.Segment s = q.segments.get(l.index);
                    if (!s.isFollowRoads()) continue;
                    ClientRoute.Settings st = new ClientRoute.Settings(s.profile, s.customModel, q.snapPreventions);
                    java.util.List<double[]> fin = new java.util.ArrayList<>();
                    java.util.List<ClientRoute.Leg> parts = new java.util.ArrayList<>();
                    for (FixRouteResponse.Segment g : l.segments) {
                        double[] a = g.start.newPoint != null ? new double[]{g.start.newPoint.lat, g.start.newPoint.lng} : byId.get(g.start.waypointId);
                        double[] b = g.end.newPoint != null ? new double[]{g.end.newPoint.lat, g.end.newPoint.lng} : byId.get(g.end.waypointId);
                        if (FixRouteRequest.TYPE_COORDINATES.equals(g.type)) {
                            for (FixRouteRequest.LatLng c : g.trackCoordinates) fin.add(new double[]{c.lat, c.lng});
                            continue;
                        }
                        java.util.List<com.graphhopper.util.shapes.GHPoint> via = new java.util.ArrayList<>();
                        if (g.viaPoints != null) for (FixRouteRequest.LatLng v : g.viaPoints) via.add(new com.graphhopper.util.shapes.GHPoint(v.lat, v.lng));
                        ClientRoute.Leg r = ClientRoute.route(hopper, st, new com.graphhopper.util.shapes.GHPoint(a[0], a[1]), via,
                                new com.graphhopper.util.shapes.GHPoint(b[0], b[1]), g.initialHeading,
                                s.headingPenalty != null ? s.headingPenalty : MatchPipelineFixer.CLIENT_HEADING_PENALTY);
                        if (r.ok()) fin.addAll(r.points());
                        parts.add(r);
                    }
                    if (fin.size() < 2) continue;
                    // U-turns at added waypoints: the route arrives and leaves on the same road.
                    for (int k = 0; k + 1 < parts.size(); k++) {
                        if (l.segments.get(k).end.newPoint == null) continue;
                        double back = retrace(parts.get(k), parts.get(k + 1));
                        if (back > 0) {
                            uturns++;
                            uturnM += back;
                            FixRouteRequest.LatLng w = l.segments.get(k).end.newPoint;
                            uturnList.append(String.format(java.util.Locale.ROOT, " leg%d@(%.6f,%.6f):%.0fm", l.index, w.lat, w.lng, back));
                        }
                    }
                    double legDiv = farLength(ref.slice(l.metrics.refFromM, l.metrics.refToM), fin, 30);
                    walkOff += coWalkOff(fin, ref.slice(l.metrics.refFromM, l.metrics.refToM), 30);
                    div += legDiv;
                    if (legDiv > 300) worst.append(String.format(java.util.Locale.ROOT, " leg%d:%.2fkm(+%d,%s)", l.index, legDiv / 1000, l.metrics.addedWaypoints, l.status));
                }
                System.out.printf(java.util.Locale.ROOT, "SCORE %s divergent %.2f km, walk-off %.2f km, added %d, unroutable %.2f km, uturns %d (%.0f m retraced) |%s | UTURNS%s%n",
                        new File(path).getName(), div / 1000, walkOff / 1000, rsp.stats.addedWaypoints, un / 1000, uturns, uturnM, worst, uturnList);
            }
        } finally {
            hopper.close();
        }
    }

    /**
     * Length [m] the route retraces at the waypoint between {@code in} and {@code out}: the end of
     * {@code in} ridden back at the start of {@code out} (same edges, opposite direction); 0 when
     * the route does not turn back there.
     */
    static double retrace(ClientRoute.Leg in, ClientRoute.Leg out) {
        if (!in.ok() || !out.ok() || in.edgeKeys() == null || out.edgeKeys() == null) return 0;
        int[] a = com.graphhopper.trailmap.shared.EdgeKeyMatching.dedupConsecutive(in.edgeKeys());
        int[] b = com.graphhopper.trailmap.shared.EdgeKeyMatching.dedupConsecutive(out.edgeKeys());
        double[] la = in.keyLengthsM(), lb = out.keyLengthsM();
        double m = 0;
        int i = a.length - 1, j = 0;
        while (i >= 0 && j < b.length && (a[i] >> 1) == (b[j] >> 1) && a[i] != b[j]) {
            m += Math.min(la != null && i < la.length ? la[i] : 0, lb != null && j < lb.length ? lb[j] : 0);
            i--;
            j++;
        }
        return m;
    }

    /**
     * Route-side, in-order measure (like the client team's co-walk scorer): walk the route in order
     * and keep a position on the saved route that may only move forward (20 m of slack for noise).
     * A route sample counts as walking with the saved route when a saved point within {@code tolM}
     * lies ahead of that position (within {@code WINDOW_M}, or anywhere further ahead after a way
     * round); otherwise it is off — a way round, or the return leg of an out-and-back that the
     * saved route does not make. Returns the off length [m] of the route.
     */
    static double coWalkOff(java.util.List<double[]> route, java.util.List<double[]> saved, double tolM) {
        java.util.List<double[]> r = PathSimilarity.resample(route), f = PathSimilarity.resample(saved);
        if (r.size() < 2 || f.size() < 2) return 0;
        double lat0 = f.get(0)[0], k = 111320 * Math.cos(Math.toRadians(lat0));
        double[][] fx = new double[f.size()][];
        for (int i = 0; i < fx.length; i++) fx[i] = new double[]{f.get(i)[1] * k, (f.get(i)[0] - lat0) * 111320};
        int back = (int) (20 / PathSimilarity.STEP_M), window = (int) (500 / PathSimilarity.STEP_M);
        int j = 0;
        double off = 0;
        for (double[] p : r) {
            double x = p[1] * k, y = (p[0] - lat0) * 111320;
            int found = -1;
            for (int t = Math.max(0, j - back); t < Math.min(fx.length, j + window) && found < 0; t++)
                if (Math.hypot(fx[t][0] - x, fx[t][1] - y) <= tolM) found = t;
            if (found < 0) // rejoined further ahead after a way round?
                for (int t = j + window; t < fx.length && found < 0; t++)
                    if (Math.hypot(fx[t][0] - x, fx[t][1] - y) <= tolM) found = t;
            if (found >= 0) j = Math.max(j, found);
            else off += PathSimilarity.STEP_M;
        }
        return off;
    }

    /** Length [m] of {@code line} (2 m samples) farther than {@code tolM} from {@code other}. */
    static double farLength(java.util.List<double[]> line, java.util.List<double[]> other, double tolM) {
        java.util.List<double[]> a = PathSimilarity.resample(line), b = PathSimilarity.resample(other);
        double lat0 = a.get(0)[0], k = 111320 * Math.cos(Math.toRadians(lat0));
        java.util.Map<Long, java.util.List<double[]>> grid = new java.util.HashMap<>();
        for (double[] p : b) {
            double x = p[1] * k, y = (p[0] - lat0) * 111320;
            grid.computeIfAbsent(((long) Math.floor(x / tolM) << 32) ^ ((long) Math.floor(y / tolM) & 0xffffffffL), z -> new java.util.ArrayList<>()).add(new double[]{x, y});
        }
        double far = 0;
        for (double[] p : a) {
            double x = p[1] * k, y = (p[0] - lat0) * 111320;
            long cx = (long) Math.floor(x / tolM), cy = (long) Math.floor(y / tolM);
            boolean near = false;
            for (long gx = cx - 1; gx <= cx + 1 && !near; gx++)
                for (long gy = cy - 1; gy <= cy + 1 && !near; gy++) {
                    java.util.List<double[]> c = grid.get((gx << 32) ^ (gy & 0xffffffffL));
                    if (c != null) for (double[] q : c) if (Math.hypot(q[0] - x, q[1] - y) <= tolM) { near = true; break; }
                }
            if (!near) far += PathSimilarity.STEP_M;
        }
        return far;
    }

    private static String line(java.util.List<double[]> pts, String name) {
        StringBuilder sb = new StringBuilder("{\"type\":\"Feature\",\"properties\":{\"name\":\"" + name + "\"},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
        for (int i = 0; i < pts.size(); i++) {
            if (i > 0) sb.append(',');
            sb.append(String.format(java.util.Locale.ROOT, "[%.6f,%.6f]", pts.get(i)[1], pts.get(i)[0]));
        }
        return sb.append("]}}").toString();
    }
}
