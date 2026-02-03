/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Figure-8 geometry - two connected loops.
 * Good for exploring two areas in opposite directions from start.
 */
package com.graphhopper.trailmap.roundtrip.geometry;

import com.graphhopper.util.shapes.GHPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Figure-8 tour geometry - two connected loops meeting at start.
 *
 * The route makes two loops in opposite directions from the start point,
 * creating a figure-8 pattern. Good for maximizing exploration while
 * starting and ending at the same point.
 *
 * Example with heading=0° (north):
 * <pre>
 *           P1--P2              P5--P6
 *          /      \            /      \
 *         |        |          |        |
 *          \      /            \      /
 *           P4--P3      S      P8--P7
 *                  \    |    /
 *                   \   |   /
 *                    \  |  /
 *                     Start
 *
 *     Route: S → P1 → P2 → P3 → P4 → S → P5 → P6 → P7 → P8 → S
 * </pre>
 *
 * Use cases:
 * - Exploring two distinct areas from a central point
 * - Maximizing variety while maintaining a central base
 * - When terrain allows loops on both sides of start
 */
public class Figure8TourGeometry extends AbstractTourGeometry {

    /** Ratio of first loop size to second loop (1.0 = equal, 1.5 = first loop 50% larger) */
    private final double loopRatio;

    /** Angle between the two loops (180 = opposite directions) */
    private final double loopAngle;

    /** Points per loop */
    private final int pointsPerLoop;

    /**
     * Create with default parameters (equal loops, opposite directions).
     */
    public Figure8TourGeometry() {
        this(1.0, 180.0, 4);
    }

    /**
     * Create with specified parameters.
     *
     * @param loopRatio Ratio of first loop to second (0.5-2.0)
     * @param loopAngle Angle between loop centers from start (90-180 degrees)
     * @param pointsPerLoop Number of waypoints per loop (3-5)
     */
    public Figure8TourGeometry(double loopRatio, double loopAngle, int pointsPerLoop) {
        this.loopRatio = Math.max(0.5, Math.min(2.0, loopRatio));
        this.loopAngle = Math.max(90, Math.min(180, loopAngle));
        this.pointsPerLoop = Math.max(3, Math.min(5, pointsPerLoop));
    }

    @Override
    public String getName() {
        return "figure8";
    }

    @Override
    public String getDescription() {
        return "Figure-8 with two loops (ratio: " + loopRatio + ", angle: " + loopAngle + "°)";
    }

    @Override
    public List<GHPoint> generateWaypoints(GHPoint start, double distanceMeters,
                                            double initialHeading, long seed) {
        Random random = new Random(seed);
        initVariation(seed);  // Initialize seed-based variation

        try {
            // Resolve heading (direction of first loop)
            double heading = Double.isNaN(initialHeading) ? random.nextInt(360) : initialHeading;

            // Calculate loop sizes:
            // Total distance = circumference_1 + circumference_2 = π * (d1 + d2)
            double d2 = distanceMeters / (Math.PI * (loopRatio + 1));
            double d1 = d2 * loopRatio;

            double radius1 = d1 / 2.0;
            double radius2 = d2 / 2.0;

            // Apply variation
            radius1 = modifyDistanceWithVariation(radius1, random, heading);
            double heading2 = normalizeBearing(heading + loopAngle);
            radius2 = modifyDistanceWithVariation(radius2, random, heading2);

            List<GHPoint> waypoints = new ArrayList<>(pointsPerLoop * 2 + 3);
            waypoints.add(start);

            // First loop - in the heading direction
            GHPoint center1 = project(start, heading, radius1);
            generateLoopPointsWithVariation(waypoints, center1, radius1, heading, random, true, 0);

            // Second loop - in the opposite direction (or at loopAngle)
            GHPoint center2 = project(start, heading2, radius2);
            generateLoopPointsWithVariation(waypoints, center2, radius2, heading2, random, false, pointsPerLoop);

            // Close back to start
            waypoints.add(start);

            return waypoints;
        } finally {
            clearVariation();
        }
    }

    /**
     * Generate waypoints around a loop with variation applied.
     *
     * @param waypoints List to add points to
     * @param center Center of the loop
     * @param radius Radius of the loop
     * @param heading Direction from start to loop center
     * @param random Random source
     * @param clockwise Direction around the loop
     * @param baseIndex Base waypoint index for variation lookup
     */
    private void generateLoopPointsWithVariation(List<GHPoint> waypoints, GHPoint center,
                                                  double radius, double heading, Random random,
                                                  boolean clockwise, int baseIndex) {
        double angleStep = 360.0 / pointsPerLoop;
        // Start from the point closest to start
        double startAngle = normalizeBearing(heading + 180);

        for (int i = 0; i < pointsPerLoop; i++) {
            double angle;
            if (clockwise) {
                angle = normalizeBearing(startAngle + 90 + i * angleStep);
            } else {
                angle = normalizeBearing(startAngle - 90 - i * angleStep);
            }
            GHPoint point = projectWithVariation(center, angle, radius, baseIndex + i);
            waypoints.add(point);
        }
    }

    /**
     * Generate waypoints around a loop (legacy method without variation).
     */
    private void generateLoopPoints(List<GHPoint> waypoints, GHPoint center,
                                     double radius, double heading, Random random,
                                     boolean clockwise) {
        double angleStep = 360.0 / pointsPerLoop;
        double startAngle = normalizeBearing(heading + 180);

        for (int i = 0; i < pointsPerLoop; i++) {
            double angle;
            if (clockwise) {
                angle = normalizeBearing(startAngle + 90 + i * angleStep);
            } else {
                angle = normalizeBearing(startAngle - 90 - i * angleStep);
            }
            GHPoint point = project(center, angle, slightlyModifyDistance(radius, random) * 0.1 + radius * 0.9);
            waypoints.add(point);
        }
    }

    /**
     * Generate a tilted figure-8 where the loops are not perfectly opposite.
     * Creates a more asymmetric and interesting route.
     *
     * @param start Starting point
     * @param distanceMeters Target total distance
     * @param initialHeading Initial direction for first loop
     * @param seed Random seed
     * @param tiltAngle Additional tilt for second loop (0-45 degrees)
     * @return List of waypoints
     */
    public List<GHPoint> generateTiltedFigure8(GHPoint start, double distanceMeters,
                                                 double initialHeading, long seed,
                                                 double tiltAngle) {
        Random random = new Random(seed);
        initVariation(seed);  // Initialize seed-based variation

        try {
            double heading = Double.isNaN(initialHeading) ? random.nextInt(360) : initialHeading;

            double d2 = distanceMeters / (Math.PI * (loopRatio + 1));
            double d1 = d2 * loopRatio;

            double radius1 = modifyDistanceWithVariation(d1 / 2.0, random, heading);
            double heading2 = normalizeBearing(heading + loopAngle + tiltAngle);
            double radius2 = modifyDistanceWithVariation(d2 / 2.0, random, heading2);

            List<GHPoint> waypoints = new ArrayList<>(pointsPerLoop * 2 + 3);
            waypoints.add(start);

            // First loop
            GHPoint center1 = project(start, heading, radius1);
            generateLoopPointsWithVariation(waypoints, center1, radius1, heading, random, true, 0);

            // Second loop with tilt
            GHPoint center2 = project(start, heading2, radius2);
            generateLoopPointsWithVariation(waypoints, center2, radius2, heading2, random, false, pointsPerLoop);

            waypoints.add(start);

            return waypoints;
        } finally {
            clearVariation();
        }
    }
}
