/*
 * Trailmap - Route Conversion API
 *
 * Request object for converting exploration waypoints to normalized waypoints.
 */
package com.graphhopper;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.graphhopper.util.CustomModel;
import com.graphhopper.util.shapes.GHPoint;

import java.util.ArrayList;
import java.util.List;

/**
 * Request object for the /convert endpoint.
 *
 * <p>Converts exploration waypoints (from a non-finalized exploration round-trip)
 * into normalized waypoints suitable for client editing.
 */
public class ConvertRequest {

    private String profile = "";
    private List<GHPoint> waypoints = new ArrayList<>();

    @JsonProperty("custom_model")
    private CustomModel customModel;

    @JsonProperty("points_encoded")
    private boolean pointsEncoded = true;

    @JsonProperty("points_encoded_multiplier")
    private double pointsEncodedMultiplier = 1e5;

    public ConvertRequest() {
    }

    public ConvertRequest(String profile, List<GHPoint> waypoints) {
        this.profile = profile;
        this.waypoints = waypoints != null ? new ArrayList<>(waypoints) : new ArrayList<>();
    }

    /**
     * Get the routing profile name (e.g., "gravel", "mtb").
     */
    public String getProfile() {
        return profile;
    }

    public ConvertRequest setProfile(String profile) {
        this.profile = profile;
        return this;
    }

    /**
     * Get the exploration waypoints to convert.
     *
     * <p>These should be the snapped exploration waypoints from a
     * non-finalized exploration round-trip response.
     */
    public List<GHPoint> getWaypoints() {
        return waypoints;
    }

    public ConvertRequest setWaypoints(List<GHPoint> waypoints) {
        this.waypoints = waypoints != null ? new ArrayList<>(waypoints) : new ArrayList<>();
        return this;
    }

    /**
     * Whether points in response should be encoded as polyline.
     */
    public boolean isPointsEncoded() {
        return pointsEncoded;
    }

    public ConvertRequest setPointsEncoded(boolean pointsEncoded) {
        this.pointsEncoded = pointsEncoded;
        return this;
    }

    /**
     * Multiplier for polyline encoding precision.
     */
    public double getPointsEncodedMultiplier() {
        return pointsEncodedMultiplier;
    }

    public ConvertRequest setPointsEncodedMultiplier(double multiplier) {
        this.pointsEncodedMultiplier = multiplier;
        return this;
    }

    /**
     * Get the custom model for additional routing customization.
     */
    public CustomModel getCustomModel() {
        return customModel;
    }

    public ConvertRequest setCustomModel(CustomModel customModel) {
        this.customModel = customModel;
        return this;
    }

    /**
     * Validate the request.
     *
     * @throws IllegalArgumentException if request is invalid
     */
    public void validate() {
        if (profile == null || profile.isEmpty()) {
            throw new IllegalArgumentException("Profile is required");
        }
        if (waypoints == null || waypoints.size() < 2) {
            throw new IllegalArgumentException("At least 2 waypoints are required");
        }
    }

    @Override
    public String toString() {
        return "ConvertRequest{" +
            "profile='" + profile + '\'' +
            ", waypoints=" + waypoints.size() +
            ", pointsEncoded=" + pointsEncoded +
            '}';
    }
}
