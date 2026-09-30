package com.graphhopper.trailmap.fixroute;

import com.graphhopper.GraphHopper;
import com.graphhopper.matching.EdgeMatch;
import com.graphhopper.matching.MapMatching;
import com.graphhopper.matching.MatchResult;
import com.graphhopper.matching.Observation;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.ev.RoadClass;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.trailmap.convert.ConvertTrackResponse;
import com.graphhopper.trailmap.convert.TrackToRouteConverter;
import com.graphhopper.trailmap.matching.MatcherConfig;
import com.graphhopper.trailmap.matching.ObservationDensifier;
import com.graphhopper.trailmap.matching.TrailmapMapMatching;
import com.graphhopper.util.FetchMode;
import com.graphhopper.util.PMap;
import com.graphhopper.util.PointList;
import com.graphhopper.util.shapes.GHPoint;
import org.junit.jupiter.api.Assumptions;
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

/**
 * Diagnostic ({@code -Dfix.legs=route:leg,…}): runs the REAL {@code /convert_track} pipeline, configured
 * exactly as the client's request and {@code TrailmapConvertResource} configure it, on the saved slice
 * of a leg (plus context), next to the matching engine of {@code /fix_route}. Prints, per line, how
 * closely it follows the saved path and its length per road class; exports one GeoJSON per leg to
 * {@code data/fix-route-out/parity}. Local only (real users' routes).
 */
public class ConvertParityTest {

    static final String OUT = "../../data/fix-route-out/parity";
    static final double CONTEXT_M = 300;

    @Test
    void parity() throws Exception {
        String prop = System.getProperty("fix.legs");
        Assumptions.assumeTrue(prop != null && FixRouteFixtures.available() && FixRouteTestGraph.available());
        new File(OUT).mkdirs();
        GraphHopper hopper = FixRouteTestGraph.open();
        try {
            for (String t : prop.split(",")) {
                String[] a = t.split(":");
                long rid = Long.parseLong(a[0]);
                int leg = Integer.parseInt(a[1]);
                run(hopper, rid, leg);
            }
        } finally {
            hopper.close();
        }
    }

    private void run(GraphHopper hopper, long rid, int leg) throws Exception {
        FixRouteFixtures.Route r = FixRouteFixtures.load(rid);
        FixRouteFixtures.Segment seg = r.segments.get(leg);
        WaypointAligner.Result al = new WaypointAligner(30, 0.5).align(r.reference, r.waypoints, r.savedSegmentLengths);
        ReferenceTrack ref = new ReferenceTrack(r.reference);
        double from = al.placements().get(leg).arcM(), to = al.placements().get(leg + 1).arcM();
        List<double[]> saved = ref.slice(from, to);
        ClientRoute.Settings settings = new ClientRoute.Settings(seg.profile, FixRouteTestGraph.customModel(seg.customModel), r.snapPreventions);

        // --- /fix_route matching engine ---
        FixRouteRequest q = FixRouteCorpusTest.request(r);
        q.options.engine = "matching";
        q.debug = true;
        FixRouteResponse rsp = FixRouteEngine.fix(hopper, null, q);
        FixRouteResponse.Leg fl = rsp.legs.get(leg);
        Map<String, double[]> wpById = new LinkedHashMap<>();
        for (FixRouteRequest.Waypoint w : q.waypoints) wpById.put(w.id, new double[]{w.coordinates.lat, w.coordinates.lng});
        List<ClientRoute.Leg> fixRoute = new ArrayList<>();
        List<double[]> fixWps = new ArrayList<>();
        for (FixRouteResponse.Segment s : fl.segments) {
            double[] p0 = pt(s.start, wpById), p1 = pt(s.end, wpById);
            if (s.end.newPoint != null) fixWps.add(p1);
            List<GHPoint> via = new ArrayList<>();
            if (s.viaPoints != null) for (FixRouteRequest.LatLng v : s.viaPoints) via.add(new GHPoint(v.lat, v.lng));
            fixRoute.add(ClientRoute.route(hopper, settings, new GHPoint(p0[0], p0[1]), via, new GHPoint(p1[0], p1[1]), s.initialHeading, 60.0));
        }
        @SuppressWarnings("unchecked")
        List<List<double[]>> fixMatched = fl.debug == null ? List.of() : (List<List<double[]>>) fl.debug.getOrDefault("matched_paths", List.of());

        // --- /convert_track, client request verbatim, on the saved slice plus context ---
        List<double[]> track = ref.slice(Math.max(0, from - CONTEXT_M), Math.min(ref.lengthM(), to + CONTEXT_M));
        List<Observation> obs = new ArrayList<>();
        for (double[] p : track) obs.add(new Observation(new GHPoint(p[0], p[1])));
        MatcherConfig cfg = new MatcherConfig();
        cfg.measurementErrorSigma = 20;
        cfg.transitionProbabilityBeta = 4.0;
        cfg.candidateRadiusSigmaMult = 3.0;
        cfg.autoSigma = true;
        cfg.autoSigmaMaxM = 10;
        cfg.densifyMaxGapM = Double.parseDouble(System.getProperty("parity.densify", "40"));
        cfg.emissionDesirabilityLambda = Double.parseDouble(System.getProperty("parity.lambda", "1.0"));
        cfg.adaptiveSigma = !"false".equals(System.getProperty("parity.adaptive"));
        cfg.transitionProbabilityBeta = Double.parseDouble(System.getProperty("parity.beta", "4.0"));
        obs = ObservationDensifier.densify(obs, cfg.densifyMaxGapM);
        PMap hints = new PMap();
        hints.putObject("profile", System.getProperty("parity.mprofile", seg.profile)); // the resource's matching hints: profile only
        TrailmapMapMatching mm = new TrailmapMapMatching(hopper.getBaseGraph(), (LocationIndexTree) hopper.getLocationIndex(),
                MapMatching.routerFromGraphHopper(hopper, hints), cfg);
        MatchResult match = mm.match(obs);
        double sigma = ((Number) mm.getStatistics().get("autoSigmaEstimatedM")).doubleValue();
        TrackToRouteConverter conv = new TrackToRouteConverter(hopper);
        conv.setSegmentationV2Enabled(true);
        conv.setOptimizerKeptObsOnly(true);
        conv.setOptimizerTwinEdgeTolerance(true);
        ConvertTrackResponse cr = conv.convert(match, obs, seg.profile, settings.customModel(),
                TrackToRouteConverter.AUTO_SIGMA_SNAP_THRESHOLD_MULT * sigma, 40, 3.0, 75, 2.0,
                TrackToRouteConverter.AUTO_SIGMA_DRIFT_FLOOR_MULT * sigma, false);
        Map<String, double[]> cwp = new LinkedHashMap<>();
        for (ConvertTrackResponse.Waypoint w : cr.getWaypoints()) cwp.put(w.getId(), new double[]{w.getCoordinates().getLat(), w.getCoordinates().getLng()});
        List<ClientRoute.Leg> convRoute = new ArrayList<>();
        List<List<double[]>> convCoords = new ArrayList<>();
        for (ConvertTrackResponse.Segment s : cr.getSegments()) {
            double[] p0 = cwp.get(s.getStart()), p1 = cwp.get(s.getEnd());
            if (ConvertTrackResponse.Segment.TYPE_FOLLOW_ROADS.equals(s.getType())) {
                convRoute.add(ClientRoute.route(hopper, settings, new GHPoint(p0[0], p0[1]), List.of(), new GHPoint(p1[0], p1[1]), s.getInitialHeading(), 60.0));
            } else {
                List<double[]> c = new ArrayList<>();
                for (ConvertTrackResponse.Coordinates x : s.getTrackCoordinates()) c.add(new double[]{x.getLat(), x.getLng()});
                convCoords.add(c);
            }
        }
        List<double[]> convMatched = new ArrayList<>();
        for (EdgeMatch em : match.getEdgeMatches()) {
            PointList g = em.getEdgeState().fetchWayGeometry(FetchMode.ALL);
            for (int p = convMatched.isEmpty() ? 0 : 1; p < g.size(); p++) convMatched.add(new double[]{g.getLat(p), g.getLon(p)});
        }

        // --- report, restricted to the leg's saved slice ---
        EnumEncodedValue<RoadClass> rc = hopper.getEncodingManager().getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        System.out.printf(Locale.ROOT, "PARITY route-%d leg %d (%s, saved %.0f m): fix=%s +%d wp, convert: %d wp, %d coords segs, sigma %.1f%n",
                rid, leg, seg.profile, to - from, fl.status, fl.metrics.addedWaypoints, cr.getWaypoints().size(), convCoords.size(), sigma);
        System.out.println("  saved->line within 3 m / max:  fix_matched " + follow(saved, flat(fixMatched)) + "  fix_route " + follow(saved, flatRoute(fixRoute))
                + "  convert_matched " + follow(saved, convMatched) + "  convert_route " + follow(saved, flatRoute(convRoute)));
        System.out.println("  road classes fix_route:     " + classes(hopper, rc, fixRoute, null));
        System.out.println("  road classes convert_route: " + classes(hopper, rc, convRoute, saved));
        for (int k = 0; k < fixRoute.size(); k++) {
            FixRouteResponse.Segment s = fl.segments.get(k);
            double[] p0 = pt(s.start, wpById), p1 = pt(s.end, wpById);
            System.out.printf(Locale.ROOT, "    fix seg %d %s(%.6f,%.6f) -> %s(%.6f,%.6f) h=%s: %s%n", k,
                    s.start.newPoint != null ? "NEW" : "wp", p0[0], p0[1], s.end.newPoint != null ? "NEW" : "wp", p1[0], p1[1],
                    s.initialHeading, classes(hopper, rc, List.of(fixRoute.get(k)), null));
        }
        for (double[] w : new double[][]{r.waypoints.get(leg), r.waypoints.get(leg + 1)}) {
            var sn = hopper.getLocationIndex().findClosest(w[0], w[1], com.graphhopper.routing.util.EdgeFilter.ALL_EDGES);
            System.out.printf(Locale.ROOT, "    fixed waypoint (%.6f,%.6f): nearest way %s at %.1f m, saved path %.1f m away%n", w[0], w[1],
                    sn.isValid() ? sn.getClosestEdge().get(rc) : "-", sn.isValid() ? sn.getQueryDistance() : -1, dist(w, saved));
        }

        StringBuilder g = new StringBuilder("{\"type\":\"FeatureCollection\",\"features\":[");
        g.append(line(saved, "saved"));
        for (List<double[]> m : fixMatched) g.append(',').append(line(m, "fix_matched"));
        for (ClientRoute.Leg l : fixRoute) if (l.ok()) g.append(',').append(line(l.points(), "fix_route"));
        for (double[] w : fixWps) g.append(',').append(point(w, "fix_waypoint"));
        g.append(',').append(line(convMatched, "convert_matched"));
        for (ClientRoute.Leg l : convRoute) if (l.ok()) g.append(',').append(line(l.points(), "convert_route"));
        for (List<double[]> c : convCoords) g.append(',').append(line(c, "convert_coords"));
        for (double[] w : cwp.values()) g.append(',').append(point(w, "convert_waypoint"));
        g.append("],\"properties\":{\"route\":").append(rid).append(",\"leg\":").append(leg).append("}}");
        try (Writer w = new FileWriter(new File(OUT, "route-" + rid + "_leg-" + leg + ".geojson"))) {
            w.write(g.toString());
        }
    }

    private static double[] pt(FixRouteResponse.Endpoint e, Map<String, double[]> byId) {
        return e.newPoint != null ? new double[]{e.newPoint.lat, e.newPoint.lng} : byId.get(e.waypointId);
    }

    private static List<double[]> flat(List<List<double[]>> ls) {
        List<double[]> out = new ArrayList<>();
        for (List<double[]> l : ls) out.addAll(l);
        return out;
    }

    private static List<double[]> flatRoute(List<ClientRoute.Leg> ls) {
        List<double[]> out = new ArrayList<>();
        for (ClientRoute.Leg l : ls) if (l.ok()) out.addAll(l.points());
        return out;
    }

    /** Share of saved-path samples (every 2 m) within 3 m of the line, and the max distance. */
    private static String follow(List<double[]> saved, List<double[]> line) {
        if (line.size() < 2) return "-";
        List<double[]> s = PathSimilarity.resample(saved);
        int in = 0;
        double max = 0;
        for (double[] p : s) {
            double d = dist(p, line);
            if (d <= 3) in++;
            max = Math.max(max, d);
        }
        return String.format(Locale.ROOT, "%.0f%%/%.0fm", 100.0 * in / s.size(), max);
    }

    private static double dist(double[] p, List<double[]> line) {
        double best = Double.POSITIVE_INFINITY, k = Math.cos(Math.toRadians(p[0])) * 111320;
        for (int i = 0; i + 1 < line.size(); i++) {
            double ax = (line.get(i)[1] - p[1]) * k, ay = (line.get(i)[0] - p[0]) * 111320;
            double bx = (line.get(i + 1)[1] - p[1]) * k, by = (line.get(i + 1)[0] - p[0]) * 111320;
            double dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
            double t = l2 == 0 ? 0 : Math.max(0, Math.min(1, -(ax * dx + ay * dy) / l2));
            best = Math.min(best, Math.hypot(ax + t * dx, ay + t * dy));
        }
        return best;
    }

    /** Metres per road class; with {@code within}, only edges whose midpoint lies within 30 m of it. */
    private static String classes(GraphHopper hopper, EnumEncodedValue<RoadClass> rc, List<ClientRoute.Leg> legs, List<double[]> within) {
        Map<String, Double> m = new TreeMap<>();
        for (ClientRoute.Leg l : legs) {
            if (!l.ok() || l.keyLengthsM() == null) continue;
            int[] keys = com.graphhopper.trailmap.shared.EdgeKeyMatching.dedupConsecutive(l.edgeKeys());
            for (int i = 0; i < keys.length && i < l.keyLengthsM().length; i++) {
                var e = hopper.getBaseGraph().getEdgeIteratorStateForKey(keys[i]);
                if (within != null) {
                    PointList g = e.fetchWayGeometry(FetchMode.ALL);
                    double[] mid = {g.getLat(g.size() / 2), g.getLon(g.size() / 2)};
                    if (dist(mid, within) > 30) continue;
                }
                m.merge(e.get(rc).toString(), l.keyLengthsM()[i], Double::sum);
            }
        }
        StringBuilder sb = new StringBuilder();
        m.forEach((k, v) -> sb.append(k).append(' ').append(Math.round(v)).append("m  "));
        return sb.toString();
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
}
