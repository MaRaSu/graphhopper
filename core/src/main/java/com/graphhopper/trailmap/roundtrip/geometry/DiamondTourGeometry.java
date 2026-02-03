/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Diamond/Lozenge geometry - elongated in one direction.
 * Good for wind-aware routes (start into wind, return with wind).
 */
package com.graphhopper.trailmap.roundtrip.geometry;

import com.graphhopper.util.shapes.GHPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Diamond tour geometry - elongated shape good for wind-aware routes.
 *
 * The diamond is elongated along the initial heading direction.
 * With elongation ratio of 2.0, the route extends twice as far in
 * the heading direction as in the perpendicular direction.
 *
 * Example with heading=0° (north) and elongationRatio=2.0:
 * <pre>
 *                          P1 (far north - tip)
 *                         /  \
 *                        /    \
 *                       /      \
 *                      /        \
 *                    P4          P2
 *                      \        /
 *                       \      /
 *                        \    /
 *                         \  /
 *                          S (start/P3)
 *
 *     Route: S → P1 → P2 → S (4-point diamond)
 * </pre>
 *
 * Use cases:
 * - Wind-aware cycling: start into headwind (harder), return with tailwind (easier)
 * - Exploring a corridor in one direction
 * - Out-and-back with scenic loop at the far end
 */
public class DiamondTourGeometry extends AbstractTourGeometry {

    /** Ratio of long axis to short axis (e.g., 2.0 = twice as long as wide) */
    private final double elongationRatio;

    /**
     * Create with default elongation ratio of 2.0.
     */
    public DiamondTourGeometry() {
        this(2.0);
    }

    /**
     * Create with specified elongation ratio.
     *
     * @param elongationRatio Ratio of long axis to short axis (1.0 = square, 2.0+ = elongated)
     */
    public DiamondTourGeometry(double elongationRatio) {
        this.elongationRatio = Math.max(1.0, elongationRatio);
    }

    @Override
    public String getName() {
        return "diamond";
    }

    @Override
    public String getDescription() {
        return "Diamond/lozenge shape elongated in heading direction (ratio: " + elongationRatio + ")";
    }

    @Override
    public List<GHPoint> generateWaypoints(GHPoint start, double distanceMeters,
                                            double initialHeading, long seed) {
        Random random = new Random(seed);
        initVariation(seed);  // Initialize seed-based variation

        try {
            // Resolve heading
            double heading = Double.isNaN(initialHeading) ? random.nextInt(360) : initialHeading;

            // For a diamond shape with elongation
            // Total perimeter = 4 * side_length, where sides connect tips
            // Using diagonals: long diagonal along heading, short diagonal perpendicular
            double e = elongationRatio;
            double x = distanceMeters / (4.0 * Math.sqrt(1 + e * e));
            double longHalf = x * e;  // Distance to far tip along heading
            double shortHalf = x;      // Perpendicular distance

            // Apply variation to base dimensions
            longHalf = modifyDistanceWithVariation(longHalf, random, heading);
            shortHalf = modifyDistanceWithVariation(shortHalf, random, normalizeBearing(heading + 90));

            List<GHPoint> waypoints = new ArrayList<>(5);
            waypoints.add(start);

            // Center of diamond
            GHPoint center = project(start, heading, longHalf);

            // P1: Far tip - with variation
            GHPoint p1 = projectWithVariation(center, heading, longHalf, 0);
            waypoints.add(p1);

            // P2: Right side tip - with variation
            double perpBearing = normalizeBearing(heading + 90);
            GHPoint p2 = projectWithVariation(center, perpBearing, shortHalf, 1);
            waypoints.add(p2);

            // Return to start (which is the near tip)
            waypoints.add(start);

            return waypoints;
        } finally {
            clearVariation();
        }
    }

    /**
     * Generate a 5-point diamond (more symmetric).
     * Route: Start → Left → Far → Right → Start
     *
     * @param start Starting point
     * @param distanceMeters Target total distance
     * @param initialHeading Initial direction
     * @param seed Random seed
     * @return List of waypoints
     */
    public List<GHPoint> generateSymmetricDiamond(GHPoint start, double distanceMeters,
                                                   double initialHeading, long seed) {
        Random random = new Random(seed);
        initVariation(seed);  // Initialize seed-based variation

        try {
            double heading = Double.isNaN(initialHeading) ? random.nextInt(360) : initialHeading;

            // Calculate diagonals based on elongation ratio
            double e = elongationRatio;
            double d1 = distanceMeters / (2.0 * Math.sqrt(1 + 1.0 / (e * e)));
            double d2 = d1 / e;

            double halfLong = d1 / 2.0;
            double halfShort = d2 / 2.0;

            // Apply variation
            halfLong = modifyDistanceWithVariation(halfLong, random, heading);
            halfShort = modifyDistanceWithVariation(halfShort, random, normalizeBearing(heading + 90));

            // Center point for calculating tips
            GHPoint center = project(start, heading, halfLong);

            // P1: Far tip - with variation
            GHPoint p1 = projectWithVariation(center, heading, halfLong, 0);

            // P2: Right side tip - with variation
            double rightBearing = normalizeBearing(heading + 90);
            GHPoint p2 = projectWithVariation(center, rightBearing, halfShort, 1);

            // P3: Left side tip - with variation
            double leftBearing = normalizeBearing(heading - 90);
            GHPoint p3 = projectWithVariation(center, leftBearing, halfShort, 2);

            // For non-crossing route: Start → Left → Far → Right → Start
            List<GHPoint> waypoints = new ArrayList<>(6);
            waypoints.add(start);
            waypoints.add(p3);  // Left
            waypoints.add(p1);  // Far
            waypoints.add(p2);  // Right
            waypoints.add(start);

            return waypoints;
        } finally {
            clearVariation();
        }
    }
}
