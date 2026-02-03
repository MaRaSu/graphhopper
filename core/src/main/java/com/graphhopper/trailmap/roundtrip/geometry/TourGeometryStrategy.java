/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Strategy interface for generating waypoints with different geometric shapes.
 */
package com.graphhopper.trailmap.roundtrip.geometry;

import com.graphhopper.util.shapes.GHPoint;

import java.util.List;

/**
 * Strategy for generating waypoints in a round-trip route.
 * Different implementations create different route shapes (circle, diamond, etc.).
 */
public interface TourGeometryStrategy {

    /**
     * Generate waypoints for a round-trip route.
     * The returned list should include the start point at the beginning and end.
     *
     * @param start Starting point
     * @param distanceMeters Target total route distance in meters
     * @param initialHeading Initial direction (0-360°, north-based clockwise)
     * @param seed Random seed for reproducibility
     * @return List of waypoints including start at beginning and end (closing the loop)
     */
    List<GHPoint> generateWaypoints(GHPoint start, double distanceMeters,
                                     double initialHeading, long seed);

    /**
     * @return Shape name for API parameter (e.g., "circle", "diamond")
     */
    String getName();

    /**
     * @return Human-readable description of this shape
     */
    default String getDescription() {
        return getName() + " tour geometry";
    }
}
