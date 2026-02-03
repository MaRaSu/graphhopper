/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Teardrop geometry - pointed at start, rounded at far end.
 * Good for gradual exploration with a wide turn-around area.
 */
package com.graphhopper.trailmap.roundtrip.geometry;

import com.graphhopper.util.shapes.GHPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Teardrop tour geometry - pointed at start, rounded at far end.
 *
 * The route spreads out gradually from the start point, reaches
 * maximum width at the far end, and narrows back to start.
 * Creates a teardrop or balloon shape.
 *
 * Example with heading=0° (north):
 * <pre>
 *                    .--P3--.
 *                   /        \
 *                  /          \
 *                 P2          P4
 *                  \          /
 *                   \        /
 *                    P1    P5
 *                     \    /
 *                      \  /
 *                       \/
 *                       S (start - pointed tip)
 *
 *     Route: S → P1 → P2 → P3 → P4 → P5 → S
 * </pre>
 *
 * Use cases:
 * - Scenic routes that gradually open up
 * - When start area is constrained (narrow valley opening)
 * - Natural progression from urban to rural areas
 */
public class TeardropTourGeometry extends AbstractTourGeometry {

    /** How pointed the teardrop is at start (1.0 = moderate, 2.0 = very pointed) */
    private final double pointedness;

    /** Number of waypoints along the teardrop */
    private final int waypointCount;

    /**
     * Create with default parameters (moderate pointedness, 5 waypoints).
     */
    public TeardropTourGeometry() {
        this(1.5, 5);
    }

    /**
     * Create with specified parameters.
     *
     * @param pointedness How pointed at start (1.0-3.0, higher = more pointed)
     * @param waypointCount Number of waypoints (4-8)
     */
    public TeardropTourGeometry(double pointedness, int waypointCount) {
        this.pointedness = Math.max(1.0, Math.min(3.0, pointedness));
        this.waypointCount = Math.max(4, Math.min(8, waypointCount));
    }

    @Override
    public String getName() {
        return "teardrop";
    }

    @Override
    public String getDescription() {
        return "Teardrop shape, pointed at start (pointedness: " + pointedness + ")";
    }

    @Override
    public List<GHPoint> generateWaypoints(GHPoint start, double distanceMeters,
                                            double initialHeading, long seed) {
        Random random = new Random(seed);
        initVariation(seed);  // Initialize seed-based variation

        try {
            // Resolve heading
            double heading = Double.isNaN(initialHeading) ? random.nextInt(360) : initialHeading;

            // Calculate teardrop dimensions:
            // Approximate perimeter ≈ L * (1 + π/(2p))
            double length = distanceMeters / (1.0 + Math.PI / (2.0 * pointedness));
            double maxWidth = length / pointedness;

            // Apply variation
            length = modifyDistanceWithVariation(length, random, heading);
            maxWidth = modifyDistanceWithVariation(maxWidth, random, normalizeBearing(heading + 90));

            List<GHPoint> waypoints = new ArrayList<>(waypointCount + 2);
            waypoints.add(start);

            // Generate points along the teardrop
            int sidePoints = waypointCount / 2;
            int wpIndex = 0;

            // Left side (going out from start)
            for (int i = 1; i <= sidePoints; i++) {
                double progress = (double) i / (sidePoints + 1);
                double dist = length * progress;

                // Width follows a sine-like curve
                double widthFactor = Math.sin(progress * Math.PI / 2);
                double width = maxWidth * widthFactor;

                // Calculate point position with variation
                GHPoint alongHeading = project(start, heading, dist);
                double leftBearing = normalizeBearing(heading - 90);
                GHPoint point = projectWithVariation(alongHeading, leftBearing, width / 2, wpIndex++);
                waypoints.add(point);
            }

            // Far end point (center, at maximum distance) - with variation
            GHPoint farEnd = projectWithVariation(start, heading, length, wpIndex++);
            waypoints.add(farEnd);

            // Right side (coming back to start)
            for (int i = sidePoints; i >= 1; i--) {
                double progress = (double) i / (sidePoints + 1);
                double dist = length * progress;
                double widthFactor = Math.sin(progress * Math.PI / 2);
                double width = maxWidth * widthFactor;

                GHPoint alongHeading = project(start, heading, dist);
                double rightBearing = normalizeBearing(heading + 90);
                GHPoint point = projectWithVariation(alongHeading, rightBearing, width / 2, wpIndex++);
                waypoints.add(point);
            }

            // Close back to start
            waypoints.add(start);

            return waypoints;
        } finally {
            clearVariation();
        }
    }

    /**
     * Generate an asymmetric teardrop that bulges more to one side.
     * Creates a comma or paisley shape.
     *
     * @param start Starting point
     * @param distanceMeters Target total distance
     * @param initialHeading Initial direction
     * @param seed Random seed
     * @param asymmetry Asymmetry factor (0 = symmetric, 0.5 = 50% wider on right)
     * @return List of waypoints
     */
    public List<GHPoint> generateAsymmetricTeardrop(GHPoint start, double distanceMeters,
                                                      double initialHeading, long seed,
                                                      double asymmetry) {
        Random random = new Random(seed);
        initVariation(seed);  // Initialize seed-based variation

        try {
            double heading = Double.isNaN(initialHeading) ? random.nextInt(360) : initialHeading;

            double length = distanceMeters / (1.0 + Math.PI / (2.0 * pointedness));
            double maxWidth = length / pointedness;

            length = modifyDistanceWithVariation(length, random, heading);
            maxWidth = modifyDistanceWithVariation(maxWidth, random, normalizeBearing(heading + 90));

            // Asymmetry: left side gets (1 - asymmetry), right side gets (1 + asymmetry)
            double leftFactor = 1.0 - Math.max(-0.5, Math.min(0.5, asymmetry));
            double rightFactor = 1.0 + Math.max(-0.5, Math.min(0.5, asymmetry));

            List<GHPoint> waypoints = new ArrayList<>(waypointCount + 2);
            waypoints.add(start);

            int sidePoints = waypointCount / 2;
            int wpIndex = 0;

            // Left side
            for (int i = 1; i <= sidePoints; i++) {
                double progress = (double) i / (sidePoints + 1);
                double dist = length * progress;
                double widthFactor = Math.sin(progress * Math.PI / 2);
                double width = maxWidth * widthFactor * leftFactor;

                GHPoint alongHeading = project(start, heading, dist);
                double leftBearing = normalizeBearing(heading - 90);
                GHPoint point = projectWithVariation(alongHeading, leftBearing, width / 2, wpIndex++);
                waypoints.add(point);
            }

            // Far end - with variation
            GHPoint farEnd = projectWithVariation(start, heading, length, wpIndex++);
            waypoints.add(farEnd);

            // Right side (with asymmetry)
            for (int i = sidePoints; i >= 1; i--) {
                double progress = (double) i / (sidePoints + 1);
                double dist = length * progress;
                double widthFactor = Math.sin(progress * Math.PI / 2);
                double width = maxWidth * widthFactor * rightFactor;

                GHPoint alongHeading = project(start, heading, dist);
                double rightBearing = normalizeBearing(heading + 90);
                GHPoint point = projectWithVariation(alongHeading, rightBearing, width / 2, wpIndex++);
                waypoints.add(point);
            }

            waypoints.add(start);

            return waypoints;
        } finally {
            clearVariation();
        }
    }
}
