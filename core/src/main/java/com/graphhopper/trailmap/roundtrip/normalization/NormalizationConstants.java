/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Constants for waypoint normalization algorithm.
 */
package com.graphhopper.trailmap.roundtrip.normalization;

/**
 * Configuration constants for the waypoint normalization algorithm.
 *
 * <p>These constants control the behavior of the binary search algorithm
 * that finds minimal waypoints to reproduce an exploration route.
 */
public final class NormalizationConstants {

    private NormalizationConstants() {
        // Utility class
    }

    /**
     * Maximum length in meters for virtual/connector edges.
     * Edges shorter than this are considered virtual and can be skipped
     * during edge sequence comparison.
     *
     * <p>Virtual edges are short connector edges that GraphHopper inserts
     * at via-points. When a waypoint is skipped during binary search,
     * these edges don't appear, causing false comparison failures.
     */
    public static final double VIRTUAL_EDGE_MAX_LENGTH_M = 5.0;

    /**
     * Minimum acceptable edge match percentage.
     * Routes with lower match percentage are considered failures.
     */
    public static final double MIN_MATCH_PERCENTAGE = 90.0;

    /**
     * Default distance interval for sampling candidate waypoints in meters.
     */
    public static final double DEFAULT_SAMPLE_INTERVAL_M = 500.0;

    /**
     * Maximum number of normalization attempts before accepting best-effort result.
     */
    public static final int MAX_NORMALIZATION_ATTEMPTS = 3;

    /**
     * Maximum waypoints per kilometer of route.
     * Used to cap the number of waypoints to prevent excessive waypoint counts.
     *
     * <p>For a 50km route, this would allow up to 10 waypoints (50 * 0.2).
     */
    public static final double MAX_WAYPOINTS_PER_KM = 0.2;

    /**
     * Minimum number of waypoints for any route (including start and end).
     */
    public static final int MIN_WAYPOINTS = 2;

    /**
     * Absolute maximum waypoints regardless of route length.
     */
    public static final int ABSOLUTE_MAX_WAYPOINTS = 25;

    /**
     * Calculate the maximum allowed waypoints for a given route distance.
     *
     * @param distanceMeters Route distance in meters
     * @return Maximum allowed waypoints
     */
    public static int calculateMaxWaypoints(double distanceMeters) {
        double distanceKm = distanceMeters / 1000.0;
        int calculated = (int) Math.ceil(distanceKm * MAX_WAYPOINTS_PER_KM);
        return Math.min(ABSOLUTE_MAX_WAYPOINTS, Math.max(MIN_WAYPOINTS, calculated));
    }
}
