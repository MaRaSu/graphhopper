package com.graphhopper.trailmap.tbt;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.graphhopper.util.CustomModel;

import java.util.List;

/**
 * Request DTO for the Trailmap TbT instruction generation endpoint.
 * Mirrors the client-side Waypoint/Segment structures in compact form.
 */
public class TrailmapInstructionRequest {

    @JsonProperty("waypoints")
    private List<Waypoint> waypoints;

    @JsonProperty("segments")
    private List<Segment> segments;

    @JsonProperty("instruction_profile")
    private String instructionProfile;

    @JsonProperty("locale")
    private String locale = "en";

    @JsonProperty("snap_preventions")
    private List<String> snapPreventions;

    public List<Waypoint> getWaypoints() { return waypoints; }
    public void setWaypoints(List<Waypoint> waypoints) { this.waypoints = waypoints; }

    public List<Segment> getSegments() { return segments; }
    public void setSegments(List<Segment> segments) { this.segments = segments; }

    public String getInstructionProfile() { return instructionProfile; }
    public void setInstructionProfile(String instructionProfile) { this.instructionProfile = instructionProfile; }

    public String getLocale() { return locale; }
    public void setLocale(String locale) { this.locale = locale; }

    public List<String> getSnapPreventions() { return snapPreventions; }
    public void setSnapPreventions(List<String> snapPreventions) { this.snapPreventions = snapPreventions; }

    public static class Waypoint {
        @JsonProperty("id")
        private String id;

        @JsonProperty("coordinates")
        private Coordinates coordinates;

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

        public double getLat() { return lat; }
        public void setLat(double lat) { this.lat = lat; }

        public double getLng() { return lng; }
        public void setLng(double lng) { this.lng = lng; }
    }

    /**
     * Segment types matching client-side RoutingMode enum.
     */
    public static final String TYPE_FOLLOW_ROADS = "followRoads";
    public static final String TYPE_DIRECT = "direct";
    public static final String TYPE_COORDINATES = "coordinates";

    public static class Segment {
        @JsonProperty("start")
        private String start;

        @JsonProperty("end")
        private String end;

        @JsonProperty("type")
        private String type = TYPE_FOLLOW_ROADS;

        @JsonProperty("profile")
        private String profile;

        @JsonProperty("custom_model")
        private CustomModel customModel;

        @JsonProperty("via_points")
        private List<Coordinates> viaPoints;

        @JsonProperty("initial_heading")
        private Double initialHeading;

        @JsonProperty("track_coordinates")
        private List<Coordinates> trackCoordinates;

        @JsonProperty("heading_penalty")
        private Double headingPenalty;

        public String getStart() { return start; }
        public void setStart(String start) { this.start = start; }

        public String getEnd() { return end; }
        public void setEnd(String end) { this.end = end; }

        public String getType() { return type; }
        public void setType(String type) { this.type = type; }

        public String getProfile() { return profile; }
        public void setProfile(String profile) { this.profile = profile; }

        public CustomModel getCustomModel() { return customModel; }
        public void setCustomModel(CustomModel customModel) { this.customModel = customModel; }

        public List<Coordinates> getViaPoints() { return viaPoints; }
        public void setViaPoints(List<Coordinates> viaPoints) { this.viaPoints = viaPoints; }

        public Double getInitialHeading() { return initialHeading; }
        public void setInitialHeading(Double initialHeading) { this.initialHeading = initialHeading; }

        public List<Coordinates> getTrackCoordinates() { return trackCoordinates; }
        public void setTrackCoordinates(List<Coordinates> trackCoordinates) { this.trackCoordinates = trackCoordinates; }

        public Double getHeadingPenalty() { return headingPenalty; }
        public void setHeadingPenalty(Double headingPenalty) { this.headingPenalty = headingPenalty; }

        public boolean isRoutable() {
            return TYPE_FOLLOW_ROADS.equals(type);
        }
    }
}
