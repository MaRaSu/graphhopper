package com.graphhopper.trailmap.fixroute;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.graphhopper.util.CustomModel;

import java.util.ArrayList;
import java.util.List;

/**
 * {@code POST /trailmap/fix_route} request (wire format: design doc §4.1). The route is given in
 * the {@code /instructions} shape — waypoints with ids, segments referencing them by id — plus the
 * saved reference track. Unknown fields are ignored.
 */
@JsonIgnoreProperties(ignoreUnknown = true)
public class FixRouteRequest {

    public static final String TYPE_FOLLOW_ROADS = "followRoads";
    public static final String TYPE_DIRECT = "direct";
    public static final String TYPE_COORDINATES = "coordinates";

    public static final String POLICY_REROUTE = "reroute";
    public static final String POLICY_COORDINATES = "coordinates";
    public static final String POLICY_BOTH = "both";

    /** Saved track, {@code [[lat, lng], …]}. */
    @JsonProperty("reference")
    public List<double[]> reference = new ArrayList<>();

    @JsonProperty("waypoints")
    public List<Waypoint> waypoints = new ArrayList<>();

    @JsonProperty("segments")
    public List<Segment> segments = new ArrayList<>();

    @JsonProperty("snap_preventions")
    public List<String> snapPreventions;

    @JsonProperty("unroutable_policy")
    public String unroutablePolicy = POLICY_REROUTE;

    @JsonProperty("debug")
    public boolean debug;

    @JsonProperty("options")
    public Options options = new Options();

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class LatLng {
        @JsonProperty("lat")
        public double lat;
        @JsonProperty("lng")
        public double lng;

        public LatLng() {
        }

        public LatLng(double lat, double lng) {
            this.lat = lat;
            this.lng = lng;
        }
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class Waypoint {
        @JsonProperty("id")
        public String id;
        @JsonProperty("coordinates")
        public LatLng coordinates;

        public Waypoint() {
        }

        public Waypoint(String id, double lat, double lng) {
            this.id = id;
            this.coordinates = new LatLng(lat, lng);
        }
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class Segment {
        @JsonProperty("id")
        public String id;
        @JsonProperty("start")
        public String start;
        @JsonProperty("end")
        public String end;
        @JsonProperty("type")
        public String type = TYPE_FOLLOW_ROADS;
        @JsonProperty("profile")
        public String profile;
        @JsonProperty("custom_model")
        public CustomModel customModel;
        @JsonProperty("via_points")
        public List<LatLng> viaPoints = new ArrayList<>();
        @JsonProperty("initial_heading")
        public Double initialHeading;
        @JsonProperty("heading_penalty")
        public Double headingPenalty;
        @JsonProperty("track_coordinates")
        public List<LatLng> trackCoordinates;
        @JsonProperty("saved_length_m")
        public Double savedLengthM;
        /** Per-leg override of {@link Options#fixMinDeviationM} (e.g. stricter for an MTB leg). */
        @JsonProperty("fix_min_deviation_m")
        public Double fixMinDeviationM;
        /** Per-leg override of {@link Options#fixMinDeviationLengthM}. */
        @JsonProperty("fix_min_deviation_length_m")
        public Double fixMinDeviationLengthM;
        /** Per-leg overrides of the detour and waypoint-end rules (see {@link Options}). */
        @JsonProperty("fix_detour_min_m")
        public Double fixDetourMinM;
        @JsonProperty("fix_detour_min_ratio")
        public Double fixDetourMinRatio;
        @JsonProperty("fix_waypoint_zone_m")
        public Double fixWaypointZoneM;
        @JsonProperty("fix_waypoint_arm_m")
        public Double fixWaypointArmM;

        public boolean isFollowRoads() {
            return TYPE_FOLLOW_ROADS.equals(type);
        }
    }

    /** Tunables (design doc §6). All optional; defaults are the starting values. */
    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class Options {
        /** "Close enough counts as OK" [m] — owner decision 2026-09-24: request option, default 12. */
        @JsonProperty("materiality_max_m")
        public double materialityMaxM = 12.0;
        @JsonProperty("materiality_len_ratio")
        public double materialityLenRatio = 0.03;
        @JsonProperty("materiality_len_slack_m")
        public double materialityLenSlackM = 20.0;
        @JsonProperty("waypoint_align_max_m")
        public double waypointAlignMaxM = 30.0;
        @JsonProperty("min_unroutable_m")
        public double minUnroutableM = 25.0;
        /** Reference-to-road distance beyond which a reference point is not a waypoint candidate. */
        @JsonProperty("offnet_snap_m")
        public double offnetSnapM = 15.0;
        @JsonProperty("time_budget_ms")
        public long timeBudgetMs = 10_000;
        @JsonProperty("max_probes")
        public int maxProbes = 50_000;
        /**
         * "matching" (default since 2026-09-25, owner decision: map matching + deviation rule) |
         * "geometry" (the earlier distance-verdict engine, kept for comparison).
         */
        @JsonProperty("engine")
        public String engine = "matching";
        /**
         * Matching engine: only a DEVIATION is fixed — a stretch where the leg's route is away from
         * the saved route by at least this much at its peak [m]. Smaller differences (a cycleway
         * beside the road, OSM redrawn, routing-profile drift) are left as the router draws them.
         * Owner calibration 2026-09-25: 11–23 m leave, 25 m either, 35–39 m fix.
         */
        @JsonProperty("fix_min_deviation_m")
        public double fixMinDeviationM = 30.0;
        /** ...and that stays more than {@link Deviation#BASE_M} away over at least this length [m]. */
        @JsonProperty("fix_min_deviation_length_m")
        public double fixMinDeviationLengthM = 25.0;
        /**
         * A DETOUR is a deviation too, however close it stays: a stretch at least this much longer [m]
         * than the part of the saved route it replaces (e.g. round a closed bridge)...
         */
        @JsonProperty("fix_detour_min_m")
        public double fixDetourMinM = 100.0;
        /** ...and by at least this share of that part. */
        @JsonProperty("fix_detour_min_ratio")
        public double fixDetourMinRatio = 0.3;
        /** Not judged: this far [m] from each fixed waypoint — the waypoint's placement decides there. */
        @JsonProperty("fix_waypoint_zone_m")
        public double fixWaypointZoneM = 30.0;
        /** A stretch leaving that zone and over within this far [m] of the waypoint is its effect too. */
        @JsonProperty("fix_waypoint_arm_m")
        public double fixWaypointArmM = 150.0;

        /*
         * LONG LEGS (owner, 2026-09-29). Up to this saved length a leg is fixed as above: the planner
         * picked its trails. Longer legs are "Google Maps" legs: today's best route is wanted, and
         * only a corridor change (via another city / valley) is pulled back — with a few hidden via
         * points on the saved route, the leg is not split and gets no new waypoints.
         */
        /** Saved leg length [m] from which a leg is a long leg. */
        @JsonProperty("fix_long_leg_m")
        public double fixLongLegM = 20_000;
        /** Corridor width [m] at {@link #fixLongLegM}... */
        @JsonProperty("fix_long_corridor_min_m")
        public double fixLongCorridorMinM = 1_000;
        /** ...growing linearly to this share of the leg's length at {@link #fixLongCorridorFullM}, then that share. */
        @JsonProperty("fix_long_corridor_ratio")
        public double fixLongCorridorRatio = 0.10;
        @JsonProperty("fix_long_corridor_full_m")
        public double fixLongCorridorFullM = 100_000;
        /**
         * Shape of that growth: 1 = linear, above 1 = slower just above {@link #fixLongLegM} (owner,
         * 2026-09-29, 42235:0: a ~30 km leg should follow the saved route more closely). With 1.5 the
         * width is 1.4 km at 30 km, 3.1 km at 50 km, 5.4 km at 70 km, 10 km at 100 km.
         */
        @JsonProperty("fix_long_corridor_growth")
        public double fixLongCorridorGrowth = 1.5;
        /** At most this many via points per corridor deviation (one per corridor width). */
        @JsonProperty("fix_long_max_vias_per_deviation")
        public int fixLongMaxViasPerDeviation = 3;
    }
}
