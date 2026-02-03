/*
 * Trailmap - Route Conversion API
 *
 * Response object for converting exploration waypoints to normalized waypoints.
 */
package com.graphhopper;

import com.graphhopper.util.PointList;
import com.graphhopper.util.shapes.GHPoint;

import java.util.ArrayList;
import java.util.List;

/**
 * Response object for the /convert endpoint.
 *
 * <p>Contains the normalized waypoints and final route details after
 * converting exploration waypoints.
 */
public class ConvertResponse {

    private final List<Throwable> errors = new ArrayList<>();
    private List<GHPoint> normalizedWaypoints = new ArrayList<>();
    private PointList points = PointList.EMPTY;
    private double distance;
    private long time;
    private double matchPercentage;
    private int originalWaypointCount;
    private int normalizedWaypointCount;

    public ConvertResponse() {
    }

    /**
     * Create a successful response.
     */
    public static ConvertResponse success(List<GHPoint> normalizedWaypoints, PointList points,
                                           double distance, long time, double matchPercentage,
                                           int originalWaypointCount) {
        ConvertResponse response = new ConvertResponse();
        response.normalizedWaypoints = normalizedWaypoints != null
            ? new ArrayList<>(normalizedWaypoints) : new ArrayList<>();
        response.points = points;
        response.distance = distance;
        response.time = time;
        response.matchPercentage = matchPercentage;
        response.originalWaypointCount = originalWaypointCount;
        response.normalizedWaypointCount = response.normalizedWaypoints.size();
        return response;
    }

    /**
     * Create a failed response.
     */
    public static ConvertResponse failure(String errorMessage) {
        ConvertResponse response = new ConvertResponse();
        response.addError(new IllegalArgumentException(errorMessage));
        return response;
    }

    public void addError(Throwable error) {
        errors.add(error);
    }

    public boolean hasErrors() {
        return !errors.isEmpty();
    }

    public List<Throwable> getErrors() {
        return errors;
    }

    /**
     * Get the normalized waypoints.
     *
     * <p>These waypoints, when routed with the standard profile, will
     * reproduce the original exploration route.
     */
    public List<GHPoint> getNormalizedWaypoints() {
        return normalizedWaypoints;
    }

    /**
     * Get the route geometry points.
     */
    public PointList getPoints() {
        return points;
    }

    /**
     * Get the total route distance in meters.
     */
    public double getDistance() {
        return distance;
    }

    /**
     * Get the total route time in milliseconds.
     */
    public long getTime() {
        return time;
    }

    /**
     * Get the edge match percentage from normalization.
     *
     * <p>Indicates how well the normalized waypoints reproduce the
     * original exploration route (0-100%).
     */
    public double getMatchPercentage() {
        return matchPercentage;
    }

    /**
     * Get the number of input (exploration) waypoints.
     */
    public int getOriginalWaypointCount() {
        return originalWaypointCount;
    }

    /**
     * Get the number of output (normalized) waypoints.
     */
    public int getNormalizedWaypointCount() {
        return normalizedWaypointCount;
    }

    @Override
    public String toString() {
        if (hasErrors()) {
            return "ConvertResponse{errors=" + errors + "}";
        }
        return String.format("ConvertResponse{waypoints=%d->%d, match=%.1f%%, distance=%.0fm}",
            originalWaypointCount, normalizedWaypointCount, matchPercentage, distance);
    }
}
