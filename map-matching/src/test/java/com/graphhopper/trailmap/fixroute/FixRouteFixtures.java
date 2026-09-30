package com.graphhopper.trailmap.fixroute;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Loader for the real-route {@code /fix_route} fixtures exported offline by the client team
 * ({@code data/fix-route-fixtures/route-<id>.json}, schema {@code fix-route-fixture/1}).
 * Test-side only. The folder is untracked (under {@code data/}); tests skip when it is absent.
 */
public final class FixRouteFixtures {

    /** Relative to map-matching/ (Maven working directory). */
    public static final String DIR = "../../data/fix-route-fixtures";

    /** Boundary mis-anchoring cases the client's windowed alignment gets wrong. */
    public static final List<Long> ALIGNMENT_CASES = List.of(11286L, 42017L, 42831L, 45039L, 116370L);

    private static final ObjectMapper MAPPER = new ObjectMapper()
            .configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);

    private FixRouteFixtures() {
    }

    public static boolean available() {
        return new File(DIR).isDirectory();
    }

    public static final class Segment {
        public int index;
        public String id;
        public String type;
        public int startWaypointIndex;
        public int endWaypointIndex;
        public String profile;
        public JsonNode customModel;
        public Double initialHeading;
        public Double headingPenalty;
        public List<double[]> viaPoints = List.of();
        public List<double[]> trackCoordinates = List.of();
        public boolean noStartSnapping;
        public boolean noEndSnapping;
        public Double savedLengthM;
    }

    public static final class Route {
        public long routeId;
        public String mode;
        public List<String> ids = new ArrayList<>();
        public List<double[]> waypoints = new ArrayList<>();
        public List<Segment> segments = new ArrayList<>();
        public double[] savedSegmentLengths;
        public List<double[]> reference = new ArrayList<>();
        public List<String> snapPreventions = List.of();
        public JsonNode knownIssues;
        public JsonNode tags;

        @Override
        public String toString() {
            return "route-" + routeId;
        }
    }

    public static List<Long> allIds() throws IOException {
        JsonNode index = MAPPER.readTree(new File(DIR, "index.json"));
        List<Long> ids = new ArrayList<>();
        for (JsonNode r : index.get("routes")) ids.add(r.get("route_id").asLong());
        return ids;
    }

    public static Route load(long routeId) throws IOException {
        JsonNode n = MAPPER.readTree(new File(DIR, "route-" + routeId + ".json"));
        Route r = new Route();
        r.routeId = n.get("route_id").asLong();
        r.mode = n.path("mode").asText(null);
        for (JsonNode w : n.get("waypoints")) {
            r.ids.add(w.path("id").asText());
            r.waypoints.add(new double[]{w.get("lat").asDouble(), w.get("lng").asDouble()});
        }
        for (JsonNode s : n.get("segments")) {
            Segment seg = new Segment();
            seg.index = s.path("index").asInt();
            seg.id = s.path("id").asText(null);
            seg.type = s.path("type").asText();
            seg.startWaypointIndex = s.path("start_waypoint_index").asInt();
            seg.endWaypointIndex = s.path("end_waypoint_index").asInt();
            seg.profile = s.path("profile").isNull() ? null : s.path("profile").asText(null);
            seg.customModel = s.path("custom_model").isMissingNode() || s.path("custom_model").isNull()
                    ? null : s.get("custom_model");
            seg.initialHeading = s.path("initial_heading").isNumber() ? s.get("initial_heading").asDouble() : null;
            seg.headingPenalty = s.path("heading_penalty").isNumber() ? s.get("heading_penalty").asDouble() : null;
            seg.viaPoints = latLngList(s.path("via_points"));
            seg.trackCoordinates = latLngList(s.path("track_coordinates"));
            seg.noStartSnapping = s.path("no_start_snapping").asBoolean(false);
            seg.noEndSnapping = s.path("no_end_snapping").asBoolean(false);
            seg.savedLengthM = s.path("saved_length_m").isNumber() ? s.get("saved_length_m").asDouble() : null;
            r.segments.add(seg);
        }
        JsonNode lens = n.path("saved_segment_lengths");
        if (lens.isArray()) {
            r.savedSegmentLengths = new double[lens.size()];
            for (int i = 0; i < lens.size(); i++) {
                r.savedSegmentLengths[i] = lens.get(i).isNumber() ? lens.get(i).asDouble() : Double.NaN;
            }
        }
        r.reference = latLngList(n.get("reference"));
        JsonNode sp = n.path("request_defaults").path("snap_preventions");
        if (sp.isArray()) {
            List<String> l = new ArrayList<>();
            for (JsonNode x : sp) l.add(x.asText());
            r.snapPreventions = l;
        }
        r.knownIssues = n.get("known_issues");
        r.tags = n.get("tags");
        return r;
    }

    /** Accepts {@code [[lat,lng],…]} or {@code [{lat,lng},…]}. */
    private static List<double[]> latLngList(JsonNode arr) {
        if (arr == null || !arr.isArray()) return List.of();
        List<double[]> out = new ArrayList<>(arr.size());
        for (JsonNode p : arr) {
            if (p.isArray()) out.add(new double[]{p.get(0).asDouble(), p.get(1).asDouble()});
            else out.add(new double[]{p.get("lat").asDouble(), p.get("lng").asDouble()});
        }
        return out;
    }

    public static String fmt(double v) {
        return Double.isNaN(v) ? "NaN" : String.format("%.1f", v);
    }

    public static String arr(double[] xs) {
        return Arrays.toString(xs);
    }
}
