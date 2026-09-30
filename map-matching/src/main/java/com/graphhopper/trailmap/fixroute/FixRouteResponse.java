package com.graphhopper.trailmap.fixroute;

import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.graphhopper.trailmap.fixroute.FixRouteRequest.LatLng;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** {@code /trailmap/fix_route} response (wire format: design doc §4.1). Null fields are omitted. */
@JsonInclude(JsonInclude.Include.NON_NULL)
public class FixRouteResponse {

    public static final String ALGORITHM_VERSION = "fix_route/1";

    public static final String OK = "ok";
    public static final String FIXED = "fixed";
    public static final String UNROUTABLE = "unroutable";
    public static final String SKIPPED = "skipped";
    public static final String UNALIGNED = "unaligned";
    public static final String NOT_PROCESSED = "not_processed";
    /** No route exists at all between the leg's own waypoints (e.g. outside the routing map): left untouched, nothing reported. */
    public static final String NO_ROUTE = "no_route";

    @JsonProperty("algorithm_version")
    public String algorithmVersion = ALGORITHM_VERSION;

    @JsonProperty("legs")
    public List<Leg> legs = new ArrayList<>();

    @JsonProperty("stats")
    public Stats stats = new Stats();

    @JsonProperty("debug")
    public Map<String, Object> debug;

    @JsonInclude(JsonInclude.Include.NON_NULL)
    public static class Leg {
        @JsonProperty("index")
        public int index;
        @JsonProperty("segment_id")
        public String segmentId;
        @JsonProperty("status")
        public String status;
        @JsonProperty("segments")
        public List<Segment> segments = new ArrayList<>();
        @JsonProperty("alternatives")
        public Map<String, Alternative> alternatives;
        @JsonProperty("unroutable")
        public List<Unroutable> unroutable;
        @JsonProperty("waypoint_snap_mismatch")
        public SnapMismatch waypointSnapMismatch;
        @JsonProperty("metrics")
        public Metrics metrics = new Metrics();
        @JsonProperty("debug")
        public Map<String, Object> debug;
    }

    @JsonInclude(JsonInclude.Include.NON_NULL)
    public static class Alternative {
        @JsonProperty("segments")
        public List<Segment> segments = new ArrayList<>();
    }

    @JsonInclude(JsonInclude.Include.NON_NULL)
    public static class Segment {
        @JsonProperty("type")
        public String type;
        @JsonProperty("start")
        public Endpoint start;
        @JsonProperty("end")
        public Endpoint end;
        @JsonProperty("initial_heading")
        public Double initialHeading;
        @JsonProperty("via_points")
        public List<LatLng> viaPoints;
        @JsonProperty("track_coordinates")
        public List<LatLng> trackCoordinates;
        @JsonProperty("distance_m")
        public double distanceM;
    }

    /** Either a fixed waypoint by id, or a new waypoint position (1e-6°). */
    @JsonInclude(JsonInclude.Include.NON_NULL)
    public static class Endpoint {
        @JsonProperty("waypoint_id")
        public String waypointId;
        @JsonProperty("new")
        public LatLng newPoint;

        public static Endpoint fixed(String id) {
            Endpoint e = new Endpoint();
            e.waypointId = id;
            return e;
        }

        public static Endpoint added(double lat, double lng) {
            Endpoint e = new Endpoint();
            e.newPoint = new LatLng(lat, lng);
            return e;
        }
    }

    @JsonInclude(JsonInclude.Include.NON_NULL)
    public static class Unroutable {
        @JsonProperty("ref_from_m")
        public double refFromM;
        @JsonProperty("ref_to_m")
        public double refToM;
        @JsonProperty("cause")
        public String cause;
        @JsonProperty("reference_length_m")
        public double referenceLengthM;
        @JsonProperty("reroute_length_m")
        public Double rerouteLengthM;
    }

    @JsonInclude(JsonInclude.Include.NON_NULL)
    public static class SnapMismatch {
        @JsonProperty("waypoint_id")
        public String waypointId;
        @JsonProperty("suggested")
        public LatLng suggested;
        @JsonProperty("offset_m")
        public double offsetM;
    }

    @JsonInclude(JsonInclude.Include.NON_NULL)
    public static class Metrics {
        @JsonProperty("ref_from_m")
        public Double refFromM;
        @JsonProperty("ref_to_m")
        public Double refToM;
        /** geometry | heading | none */
        @JsonProperty("accepted_by")
        public String acceptedBy;
        @JsonProperty("probes")
        public int probes;
        @JsonProperty("added_waypoints")
        public int addedWaypoints;
        @JsonProperty("ms")
        public long ms;
    }

    public static class Stats {
        @JsonProperty("matching_ms")
        public long matchingMs;
        @JsonProperty("fixing_ms")
        public long fixingMs;
        @JsonProperty("total_ms")
        public long totalMs;
        @JsonProperty("probes")
        public int probes;
        @JsonProperty("added_waypoints")
        public int addedWaypoints;
        @JsonProperty("by_status")
        public Map<String, Integer> byStatus = new LinkedHashMap<>();
    }
}
