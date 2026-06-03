package com.graphhopper.trailmap.convert;

import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.annotation.JsonProperty;

import java.util.List;

/**
 * Response DTO for {@code POST /trailmap/convert_track}.
 *
 * <p>Shape mirrors the client-side {@code RouteSchemaType}:
 * a flat {@code waypoints} array with unique ids, and a {@code segments}
 * array whose {@code start}/{@code end} fields reference waypoints by id.
 */
@JsonInclude(JsonInclude.Include.NON_NULL)
public class ConvertTrackResponse {

    @JsonProperty("waypoints")
    private List<Waypoint> waypoints;

    @JsonProperty("segments")
    private List<Segment> segments;

    @JsonProperty("stats")
    private Stats stats;

    /** Present only when the request set {@code debug: true}. See API doc §4. */
    @JsonProperty("debug")
    private Debug debug;

    public List<Waypoint> getWaypoints() { return waypoints; }
    public void setWaypoints(List<Waypoint> waypoints) { this.waypoints = waypoints; }

    public List<Segment> getSegments() { return segments; }
    public void setSegments(List<Segment> segments) { this.segments = segments; }

    public Stats getStats() { return stats; }
    public void setStats(Stats stats) { this.stats = stats; }

    public Debug getDebug() { return debug; }
    public void setDebug(Debug debug) { this.debug = debug; }

    public static class Waypoint {
        @JsonProperty("id")
        private String id;

        @JsonProperty("coordinates")
        private Coordinates coordinates;

        public Waypoint() {}
        public Waypoint(String id, double lat, double lng) {
            this.id = id;
            this.coordinates = new Coordinates(lat, lng);
        }

        public String getId() { return id; }
        public void setId(String id) { this.id = id; }

        public Coordinates getCoordinates() { return coordinates; }
        public void setCoordinates(Coordinates coordinates) { this.coordinates = coordinates; }
    }

    public static class Coordinates {
        @JsonProperty("lat")
        private double lat;

        @JsonProperty("lng")
        private double lng;

        public Coordinates() {}
        public Coordinates(double lat, double lng) {
            this.lat = lat;
            this.lng = lng;
        }

        public double getLat() { return lat; }
        public void setLat(double lat) { this.lat = lat; }

        public double getLng() { return lng; }
        public void setLng(double lng) { this.lng = lng; }
    }

    @JsonInclude(JsonInclude.Include.NON_NULL)
    public static class Segment {
        public static final String TYPE_FOLLOW_ROADS = "followRoads";
        public static final String TYPE_COORDINATES = "coordinates";

        @JsonProperty("type")
        private String type;

        @JsonProperty("start")
        private String start;

        @JsonProperty("end")
        private String end;

        /** Only present for {@code coordinates} segments. */
        @JsonProperty("track_coordinates")
        private List<Coordinates> trackCoordinates;

        @JsonProperty("distance_m")
        private double distanceM;

        /** Start-of-segment travel-direction heading (degrees, 0–360, 0=north, clockwise) the
         *  client must pass as {@code headings=[initialHeading, null]} when rendering this
         *  {@code followRoads} segment, so its {@code /route} reproduces the path the server
         *  validated. Emitted ONLY when the segment is not the route's first and its previous
         *  segment is also {@code followRoads}; omitted otherwise (a missing field is the
         *  explicit "no heading constraint" signal the client relies on). Class-level
         *  {@code NON_NULL} keeps it out of the JSON when null. The client applies its own
         *  global heading_penalty (60); the server does not send a penalty. */
        @JsonProperty("initialHeading")
        private Double initialHeading;

        public Segment() {}

        public static Segment routed(String start, String end, double distanceM) {
            Segment s = new Segment();
            s.type = TYPE_FOLLOW_ROADS;
            s.start = start;
            s.end = end;
            s.distanceM = distanceM;
            return s;
        }

        public static Segment routed(String start, String end, double distanceM, Double initialHeading) {
            Segment s = routed(start, end, distanceM);
            s.initialHeading = initialHeading;
            return s;
        }

        public static Segment coordinates(String start, String end, List<Coordinates> track, double distanceM) {
            Segment s = new Segment();
            s.type = TYPE_COORDINATES;
            s.start = start;
            s.end = end;
            s.trackCoordinates = track;
            s.distanceM = distanceM;
            return s;
        }

        public String getType() { return type; }
        public void setType(String type) { this.type = type; }

        public String getStart() { return start; }
        public void setStart(String start) { this.start = start; }

        public String getEnd() { return end; }
        public void setEnd(String end) { this.end = end; }

        public List<Coordinates> getTrackCoordinates() { return trackCoordinates; }
        public void setTrackCoordinates(List<Coordinates> trackCoordinates) { this.trackCoordinates = trackCoordinates; }

        public double getDistanceM() { return distanceM; }
        public void setDistanceM(double distanceM) { this.distanceM = distanceM; }

        public Double getInitialHeading() { return initialHeading; }
        public void setInitialHeading(Double initialHeading) { this.initialHeading = initialHeading; }
    }

    /** Diagnostic block. Present only when the request opted into debug mode. Shape is
     *  intentionally not part of the public versioned contract — see API doc §4.6. */
    @JsonInclude(JsonInclude.Include.NON_NULL)
    public static class Debug {
        @JsonProperty("matcher")
        public MatcherDebug matcher;

        @JsonProperty("detours")
        public List<DetourDebug> detours;

        /** Region-by-region segmentation decisions from {@link RegionSegmenter}.
         *  Lets the client render matched-vs-coordinates regions for visual verification. */
        @JsonProperty("segmentation")
        public SegmentationDebug segmentation;
    }

    @JsonInclude(JsonInclude.Include.NON_NULL)
    public static class SegmentationDebug {
        @JsonProperty("regions")
        public List<RegionDebug> regions;
    }

    @JsonInclude(JsonInclude.Include.NON_NULL)
    public static class RegionDebug {
        /** One of {@code "matched"} or {@code "coordinates"}. */
        @JsonProperty("type")
        public String type;

        @JsonProperty("first_obs")
        public int firstObs;

        @JsonProperty("last_obs")
        public int lastObs;
    }

    @JsonInclude(JsonInclude.Include.NON_NULL)
    public static class MatcherDebug {
        @JsonProperty("polyline_encoded")
        public String polylineEncoded;

        @JsonProperty("polyline_length_m")
        public double polylineLengthM;

        @JsonProperty("edge_count")
        public int edgeCount;

        @JsonProperty("tracepoints")
        public List<TracepointDebug> tracepoints;
    }

    @JsonInclude(JsonInclude.Include.NON_NULL)
    public static class TracepointDebug {
        @JsonProperty("index")
        public int index;

        @JsonProperty("original")
        public Coordinates original;

        @JsonProperty("snap")
        public Coordinates snap;

        @JsonProperty("snap_distance_m")
        public Double snapDistanceM;

        @JsonProperty("edge_id")
        public Integer edgeId;

        @JsonProperty("matched")
        public boolean matched;

        @JsonProperty("filtered")
        public boolean filtered;
    }

    @JsonInclude(JsonInclude.Include.NON_NULL)
    public static class DetourDebug {
        @JsonProperty("from_obs")
        public int fromObs;

        @JsonProperty("to_obs")
        public int toObs;

        @JsonProperty("matched_length_m")
        public double matchedLengthM;

        @JsonProperty("straight_m")
        public double straightM;

        @JsonProperty("ratio")
        public double ratio;
    }

    public static class Stats {
        @JsonProperty("input_points")
        public int inputPoints;

        @JsonProperty("matched_points")
        public int matchedPoints;

        @JsonProperty("unmatched_points")
        public int unmatchedPoints;

        @JsonProperty("routed_segments")
        public int routedSegments;

        @JsonProperty("coordinates_segments")
        public int coordinatesSegments;

        @JsonProperty("total_distance_m")
        public double totalDistanceM;

        @JsonProperty("matching_ms")
        public long matchingMs;

        @JsonProperty("optimization_ms")
        public long optimizationMs;
    }
}
