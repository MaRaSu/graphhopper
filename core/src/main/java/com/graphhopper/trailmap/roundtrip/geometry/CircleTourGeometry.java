/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Circle geometry - ports the existing GH MultiPointTour behavior.
 * Generates waypoints in a roughly circular pattern with evenly-spaced angles.
 */
package com.graphhopper.trailmap.roundtrip.geometry;

import com.graphhopper.util.shapes.GHPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Circle tour geometry - evenly distributed waypoints around a circular path.
 *
 * This is equivalent to GraphHopper's original MultiPointTour behavior:
 * - Waypoints are projected from the previous point (not from start)
 * - Angles are evenly distributed (360° / N)
 * - Distance per leg is totalDistance / (N+1) with ±10% variation
 *
 * Example with 4 waypoints, heading=0°:
 * <pre>
 *                          P1
 *                         /  \
 *                        /    \
 *              heading  /      \ heading
 *                0°    /        \  90°
 *                     /          \
 *                    S            P2
 *                     \          /
 *             heading  \        / heading
 *               270°    \      /  180°
 *                        \    /
 *                         \  /
 *                          P3
 *
 *     Route: S → P1 → P2 → P3 → S
 * </pre>
 */
public class CircleTourGeometry extends AbstractTourGeometry {

    /** Default number of waypoints (not counting duplicated start at end) */
    private final int defaultWaypointCount;

    /**
     * Create with default waypoint count of 4 (creates a roughly square loop).
     */
    public CircleTourGeometry() {
        this(4);
    }

    /**
     * Create with specified default waypoint count.
     *
     * @param defaultWaypointCount Number of waypoints (minimum 3 for a triangle)
     */
    public CircleTourGeometry(int defaultWaypointCount) {
        this.defaultWaypointCount = Math.max(3, defaultWaypointCount);
    }

    @Override
    public String getName() {
        return "circle";
    }

    @Override
    public String getDescription() {
        return "Circular tour with evenly distributed waypoints";
    }

    @Override
    public List<GHPoint> generateWaypoints(GHPoint start, double distanceMeters,
                                            double initialHeading, long seed) {
        Random random = new Random(seed);
        initVariation(seed);  // Initialize seed-based variation

        try {
            // Calculate number of waypoints based on distance (same logic as GH)
            // Minimum 3, scales with distance
            int waypointCount = Math.max(3, Math.min(20, 2 + (int) (distanceMeters / 50000)));

            // Number of intermediate points (not counting start duplicated at end)
            int intermediatePoints = waypointCount - 1;

            // Distance per leg: total / (waypoints + 1) to account for return leg
            double baseDistancePerLeg = distanceMeters / (waypointCount + 1);

            List<GHPoint> waypoints = new ArrayList<>(waypointCount + 1);
            waypoints.add(start);

            // Determine base heading once
            double baseHeading = Double.isNaN(initialHeading) ? random.nextInt(360) : initialHeading;

            // Generate intermediate waypoints
            GHPoint lastPoint = start;
            for (int i = 0; i < intermediatePoints; i++) {
                // Rotate heading: baseHeading + 360° * i / waypointCount
                double heading = normalizeBearing(baseHeading + 360.0 * i / waypointCount);

                // Distance with standard ±10% variation and quadrant stretch
                double legDistance = modifyDistanceWithVariation(baseDistancePerLeg, random, heading);

                // Project point with full variation (radial, angular, perpendicular)
                GHPoint nextPoint = projectWithVariation(lastPoint, heading, legDistance, i);
                waypoints.add(nextPoint);
                lastPoint = nextPoint;
            }

            // Close the loop by returning to start
            waypoints.add(start);

            return waypoints;
        } finally {
            clearVariation();  // Clean up
        }
    }

    /**
     * Generate waypoints with explicit waypoint count.
     *
     * @param start Starting point
     * @param distanceMeters Target total route distance
     * @param initialHeading Initial direction (0-360° or NaN for random)
     * @param seed Random seed
     * @param waypointCount Number of waypoints (minimum 3)
     * @return List of waypoints
     */
    public List<GHPoint> generateWaypoints(GHPoint start, double distanceMeters,
                                            double initialHeading, long seed,
                                            int waypointCount) {
        Random random = new Random(seed);
        initVariation(seed);  // Initialize seed-based variation

        try {
            waypointCount = Math.max(3, waypointCount);

            int intermediatePoints = waypointCount - 1;
            double baseDistancePerLeg = distanceMeters / (waypointCount + 1);

            List<GHPoint> waypoints = new ArrayList<>(waypointCount + 1);
            waypoints.add(start);

            GHPoint lastPoint = start;
            double baseHeading = Double.isNaN(initialHeading) ? random.nextInt(360) : initialHeading;

            for (int i = 0; i < intermediatePoints; i++) {
                double heading = normalizeBearing(baseHeading + 360.0 * i / waypointCount);
                double legDistance = modifyDistanceWithVariation(baseDistancePerLeg, random, heading);

                GHPoint nextPoint = projectWithVariation(lastPoint, heading, legDistance, i);
                waypoints.add(nextPoint);
                lastPoint = nextPoint;
            }

            waypoints.add(start);
            return waypoints;
        } finally {
            clearVariation();  // Clean up
        }
    }
}
