package com.graphhopper.trailmap.fixroute;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.graphhopper.GraphHopper;
import com.graphhopper.util.shapes.GHPoint;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.FileWriter;
import java.io.Writer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Exports, for chosen legs ({@code -Dfix.legs=route:leg:label,…}), one GeoJSON per leg for the
 * side-by-side review page: the saved slice, the client's CURRENT route of the leg (before any fix),
 * and — from a fresh run of the matching engine — the route after its fix, the added waypoints, the
 * reported stretches, and per fixed waypoint where the client's route snaps it and, when reported,
 * the suggested position of a waypoint snap mismatch. Local review only (real users' routes).
 */
public class EngineCompareExportTest {

    static final String OUT = System.getProperty("fix.out", "../../data/fix-route-out/compare");

    @Test
    void export() throws Exception {
        String prop = System.getProperty("fix.legs");
        Assumptions.assumeTrue(prop != null && FixRouteFixtures.available() && FixRouteTestGraph.available());
        new File(OUT).mkdirs();
        ObjectMapper om = new ObjectMapper();
        GraphHopper hopper = FixRouteTestGraph.open();
        Map<Long, FixRouteResponse> runs = new HashMap<>();
        try {
            for (String t : prop.split(",")) {
                String[] a = t.split(":", 3);
                long rid = Long.parseLong(a[0]);
                int leg = Integer.parseInt(a[1]);
                // -Dfix.request=path: the route from a saved client request body instead of a fixture.
                String reqPath = System.getProperty("fix.request");
                FixRouteFixtures.Route r = reqPath != null ? fromRequest(reqPath, rid) : FixRouteFixtures.load(rid);
                FixRouteRequest q = FixRouteCorpusTest.request(r);
                q.options.engine = "matching";
                q.debug = true;
                FixRouteResponse rsp = runs.computeIfAbsent(rid, x -> FixRouteEngine.fix(hopper, null, q));
                FixRouteFixtures.Segment seg = r.segments.get(leg);
                ClientRoute.Settings settings = new ClientRoute.Settings(seg.profile, FixRouteTestGraph.customModel(seg.customModel), r.snapPreventions);
                WaypointAligner.Result al = new WaypointAligner(30, 0.5).align(r.reference, r.waypoints, r.savedSegmentLengths);
                ReferenceTrack ref = new ReferenceTrack(r.reference);
                List<double[]> saved = ref.slice(al.placements().get(leg).arcM(), al.placements().get(leg + 1).arcM());
                double[] s0 = r.waypoints.get(leg), e0 = r.waypoints.get(leg + 1);
                List<GHPoint> via = new ArrayList<>();
                for (double[] v : seg.viaPoints) via.add(new GHPoint(v[0], v[1]));
                ClientRoute.Leg before = ClientRoute.route(hopper, settings, new GHPoint(s0[0], s0[1]), via,
                        new GHPoint(e0[0], e0[1]), seg.initialHeading, seg.headingPenalty);

                FixRouteResponse.Leg fl = rsp.legs.get(leg);
                Map<String, double[]> byId = new LinkedHashMap<>();
                for (FixRouteRequest.Waypoint w : q.waypoints) byId.put(w.id, new double[]{w.coordinates.lat, w.coordinates.lng});
                StringBuilder g = new StringBuilder("{\"type\":\"FeatureCollection\",\"features\":[");
                g.append(line(saved, "saved"));
                if (before.ok()) g.append(',').append(line(before.points(), "before"));
                List<double[]> after = new ArrayList<>();
                List<ClientRoute.Leg> parts = new ArrayList<>();
                List<double[]> partEnds = new ArrayList<>();
                int wrongWay = 0;
                StringBuilder flagFeatures = new StringBuilder();
                for (FixRouteResponse.Segment s : fl.segments) {
                    double[] p0 = s.start.newPoint != null ? new double[]{s.start.newPoint.lat, s.start.newPoint.lng} : byId.get(s.start.waypointId);
                    double[] p1 = s.end.newPoint != null ? new double[]{s.end.newPoint.lat, s.end.newPoint.lng} : byId.get(s.end.waypointId);
                    if (s.end.newPoint != null) g.append(',').append(point(p1, "match_new_waypoint", null));
                    if (FixRouteRequest.TYPE_COORDINATES.equals(s.type)) {
                        for (FixRouteRequest.LatLng c : s.trackCoordinates) after.add(new double[]{c.lat, c.lng});
                        continue;
                    }
                    List<GHPoint> sv = new ArrayList<>();
                    if (s.viaPoints != null) for (FixRouteRequest.LatLng v : s.viaPoints) sv.add(new GHPoint(v.lat, v.lng));
                    ClientRoute.Leg l = ClientRoute.route(hopper, settings, new GHPoint(p0[0], p0[1]), sv, new GHPoint(p1[0], p1[1]),
                            s.initialHeading, MatchPipelineFixer.CLIENT_HEADING_PENALTY);
                    if (l.ok()) after.addAll(l.points());
                    parts.add(l);
                    partEnds.add(p1);
                    // Report any edge this segment rides against its allowed direction (a one-way the
                    // profile only allows at a heavy penalty): weight this way ≫ the other way.
                    if (l.ok()) {
                        var w = hopper.createWeighting(hopper.getProfile(seg.profile), new com.graphhopper.util.PMap());
                        for (int key : com.graphhopper.trailmap.shared.EdgeKeyMatching.dedupConsecutive(l.edgeKeys())) {
                            var e = hopper.getBaseGraph().getEdgeIteratorStateForKey(key);
                            double fwd = w.calcEdgeWeight(e, false), bwd = w.calcEdgeWeight(e, true);
                            if (Double.isFinite(fwd) && Double.isFinite(bwd) && Math.min(fwd, bwd) > 0 && fwd > 10 * bwd) {
                                System.out.printf(Locale.ROOT, "WRONGWAY route-%d leg %d: edge %d (%.0f m) ridden at %.0fx the other direction's cost%n",
                                        rid, leg, e.getEdge(), e.getDistance(), fwd / bwd);
                                wrongWay++;
                                com.graphhopper.util.PointList eg = e.fetchWayGeometry(com.graphhopper.util.FetchMode.ALL);
                                flagFeatures.append(',').append(point(new double[]{eg.getLat(eg.size() / 2), eg.getLon(eg.size() / 2)}, "wrongway", null));
                            }
                        }
                    }
                }
                if (!after.isEmpty()) g.append(',').append(line(after, "match_final"));
                // The checks I can make myself, so the reviewer does not have to: U-turns at added
                // waypoints, edges ridden against their direction, and both quality measures.
                int uturns = 0;
                double uturnM = 0;
                for (int k = 0; k + 1 < parts.size(); k++) {
                    if (fl.segments.get(k).end.newPoint == null) continue;
                    double back = RequestReplayTest.retrace(parts.get(k), parts.get(k + 1));
                    if (back > 0) {
                        uturns++;
                        uturnM += back;
                        flagFeatures.append(',').append(point(partEnds.get(k), "uturn", String.format(Locale.ROOT, "%.0f", back)));
                    }
                }
                g.append(flagFeatures);
                double savedLeftAfter = after.size() < 2 ? -1 : RequestReplayTest.farLength(saved, after, 30);
                double walkOffAfter = after.size() < 2 ? -1 : RequestReplayTest.coWalkOff(after, saved, 30);
                double savedLeftToday = before.ok() ? RequestReplayTest.farLength(saved, before.points(), 30) : -1;
                double walkOffToday = before.ok() ? RequestReplayTest.coWalkOff(before.points(), saved, 30) : -1;
                String checks = String.format(Locale.ROOT,
                        "{\"uturns\":%d,\"uturn_m\":%.0f,\"wrongway\":%d,\"saved_left_m\":%.0f,\"walk_off_m\":%.0f,\"saved_left_today_m\":%.0f,\"walk_off_today_m\":%.0f}",
                        uturns, uturnM, wrongWay, savedLeftAfter, walkOffAfter, savedLeftToday, walkOffToday);
                if (fl.unroutable != null) for (FixRouteResponse.Unroutable u : fl.unroutable)
                    g.append(',').append(line(ref.slice(u.refFromM, u.refToM), "match_unroutable_" + u.cause));
                // The two fixed waypoints: as placed, and where the client's route snaps them.
                g.append(',').append(point(s0, "fixed_waypoint", null)).append(',').append(point(e0, "fixed_waypoint", null));
                if (before.ok()) {
                    g.append(',').append(point(before.points().get(0), "waypoint_snap", null));
                    g.append(',').append(point(before.points().get(before.points().size() - 1), "waypoint_snap", null));
                }
                // Snap-mismatch reports for either waypoint: this leg (its start), the next leg (its start = our end).
                for (int k = leg; k <= leg + 1 && k < rsp.legs.size(); k++) {
                    FixRouteResponse.SnapMismatch m = rsp.legs.get(k).waypointSnapMismatch;
                    if (m == null) continue;
                    if (!m.waypointId.equals(r.ids.get(leg)) && !m.waypointId.equals(r.ids.get(leg + 1))) continue;
                    System.out.println("MISMATCH route-" + rid + " leg " + leg + ": reported by leg " + k + " for waypoint "
                            + (m.waypointId.equals(r.ids.get(leg)) ? "start" : "end") + " offset " + m.offsetM + " debug " + rsp.legs.get(k).debug);
                    g.append(',').append(point(new double[]{m.suggested.lat, m.suggested.lng}, "snap_suggested",
                            String.format(Locale.ROOT, "%.1f", m.offsetM)));
                }
                g.append("],\"properties\":{\"route\":").append(rid).append(",\"leg\":").append(leg)
                        .append(",\"label\":").append(om.writeValueAsString(a.length > 2 ? a[2] : ""))
                        .append(",\"deviations\":").append(om.writeValueAsString(fl.debug == null ? List.of() : fl.debug.getOrDefault("deviations", List.of())))
                        .append(",\"checks\":").append(checks)
                        .append("}}");
                try (Writer w = new FileWriter(new File(OUT, "route-" + rid + "_leg-" + leg + ".geojson"))) {
                    w.write(g.toString());
                }
            }
        } finally {
            hopper.close();
        }
    }

    /** A client request body as a fixture-shaped route (live waypoint positions, as the client sent them). */
    static FixRouteFixtures.Route fromRequest(String path, long rid) throws Exception {
        ObjectMapper jm = com.graphhopper.jackson.Jackson.newObjectMapper();
        FixRouteRequest q = jm.readValue(new File(path), FixRouteRequest.class);
        FixRouteFixtures.Route r = new FixRouteFixtures.Route();
        r.routeId = rid;
        r.reference = q.reference;
        r.snapPreventions = q.snapPreventions == null ? List.of() : q.snapPreventions;
        Map<String, Integer> idx = new HashMap<>();
        for (FixRouteRequest.Waypoint w : q.waypoints) {
            idx.put(w.id, r.ids.size());
            r.ids.add(w.id);
            r.waypoints.add(new double[]{w.coordinates.lat, w.coordinates.lng});
        }
        r.savedSegmentLengths = new double[q.segments.size()];
        for (int i = 0; i < q.segments.size(); i++) {
            FixRouteRequest.Segment s = q.segments.get(i);
            FixRouteFixtures.Segment f = new FixRouteFixtures.Segment();
            f.index = i;
            f.id = s.id;
            f.type = s.type;
            f.startWaypointIndex = idx.get(s.start);
            f.endWaypointIndex = idx.get(s.end);
            f.profile = s.profile;
            f.customModel = s.customModel == null ? null : jm.valueToTree(s.customModel);
            f.initialHeading = s.initialHeading;
            f.headingPenalty = s.headingPenalty;
            List<double[]> via = new ArrayList<>();
            if (s.viaPoints != null) for (FixRouteRequest.LatLng v : s.viaPoints) via.add(new double[]{v.lat, v.lng});
            f.viaPoints = via;
            List<double[]> tc = new ArrayList<>();
            if (s.trackCoordinates != null) for (FixRouteRequest.LatLng v : s.trackCoordinates) tc.add(new double[]{v.lat, v.lng});
            f.trackCoordinates = tc;
            f.savedLengthM = s.savedLengthM;
            r.savedSegmentLengths[i] = s.savedLengthM == null ? Double.NaN : s.savedLengthM;
            r.segments.add(f);
        }
        return r;
    }

    private static String line(List<double[]> pts, String name) {
        StringBuilder sb = new StringBuilder("{\"type\":\"Feature\",\"properties\":{\"name\":\"" + name + "\"},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
        for (int i = 0; i < pts.size(); i++) {
            if (i > 0) sb.append(',');
            sb.append(String.format(Locale.ROOT, "[%.6f,%.6f]", pts.get(i)[1], pts.get(i)[0]));
        }
        return sb.append("]}}").toString();
    }

    private static String point(double[] p, String name, String offset) {
        return String.format(Locale.ROOT, "{\"type\":\"Feature\",\"properties\":{\"name\":\"%s\"%s},\"geometry\":{\"type\":\"Point\",\"coordinates\":[%.6f,%.6f]}}",
                name, offset == null ? "" : ",\"offset_m\":" + offset, p[1], p[0]);
    }
}
