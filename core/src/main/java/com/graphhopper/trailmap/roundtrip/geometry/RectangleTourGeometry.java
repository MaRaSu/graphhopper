/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Rectangle geometry - out-and-back with slight perpendicular variation.
 * Creates a narrow rectangle shape avoiding exact backtracking.
 */
package com.graphhopper.trailmap.roundtrip.geometry;

import com.graphhopper.util.shapes.GHPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Rectangle tour geometry - out-and-back with perpendicular offset.
 *
 * Creates a narrow rectangle where most of the distance is along the main axis,
 * with small perpendicular offsets to avoid exact backtracking.
 *
 * Example with heading=0° (north):
 * <pre>
 *     P1 ─────────────────── P2
 *     │                       │
 *     │    (narrow width)     │
 *     │                       │
 *     S ─────────────────── P3
 *
 *     Route: S → P1 → P2 → P3 → S
 * </pre>
 *
 * With aspect ratio of 5.0:
 * - 70% of distance along heading direction
 * - 30% as perpendicular offset
 *
 * Use cases:
 * - Exploring a specific corridor or valley
 * - Out-and-back along a coast with inland return
 * - Following a river/road outbound, different route return
 */
public class RectangleTourGeometry extends AbstractTourGeometry {

    /** Ratio of length to width (e.g., 5.0 = 5x longer than wide) */
    private final double aspectRatio;

    /**
     * Create with default aspect ratio of 5.0 (very elongated).
     */
    public RectangleTourGeometry() {
        this(5.0);
    }

    /**
     * Create with specified aspect ratio.
     *
     * @param aspectRatio Ratio of length to width (minimum 2.0)
     */
    public RectangleTourGeometry(double aspectRatio) {
        this.aspectRatio = Math.max(2.0, aspectRatio);
    }

    @Override
    public String getName() {
        return "rectangle";
    }

    @Override
    public String getDescription() {
        return "Narrow rectangle for out-and-back with variation (aspect: " + aspectRatio + ")";
    }

    @Override
    public List<GHPoint> generateWaypoints(GHPoint start, double distanceMeters,
                                            double initialHeading, long seed) {
        Random random = new Random(seed);
        initVariation(seed);  // Initialize seed-based variation

        try {
            double heading = Double.isNaN(initialHeading) ? random.nextInt(360) : initialHeading;

            // Rectangle perimeter = 2*length + 2*width
            // With aspect ratio r = length/width:
            // perimeter = 2*r*w + 2*w = 2w(r+1)
            // w = perimeter / (2*(r+1))
            // length = r * w

            double width = distanceMeters / (2.0 * (aspectRatio + 1.0));
            double length = aspectRatio * width;

            // Apply variation
            length = modifyDistanceWithVariation(length, random, heading);
            width = modifyDistanceWithVariation(width, random, normalizeBearing(heading + 90));

            List<GHPoint> waypoints = new ArrayList<>(5);
            waypoints.add(start);

            // Far midpoint along heading
            GHPoint farMid = project(start, heading, length);

            // P1: Far left - with variation
            double leftBearing = normalizeBearing(heading - 90);
            GHPoint p1 = projectWithVariation(farMid, leftBearing, width / 2.0, 0);
            waypoints.add(p1);

            // P2: Far right - with variation
            double rightBearing = normalizeBearing(heading + 90);
            GHPoint p2 = projectWithVariation(farMid, rightBearing, width / 2.0, 1);
            waypoints.add(p2);

            // P3: Near right - with variation
            GHPoint p3 = projectWithVariation(start, rightBearing, width / 2.0, 2);
            waypoints.add(p3);

            // Return to start
            waypoints.add(start);

            return waypoints;
        } finally {
            clearVariation();
        }
    }

    /**
     * Generate a simpler 3-point rectangle (triangle-ish).
     * Out along heading, return via parallel offset path.
     *
     * @param start Starting point
     * @param distanceMeters Target total distance
     * @param initialHeading Initial direction
     * @param seed Random seed
     * @return List of waypoints
     */
    public List<GHPoint> generateSimpleRectangle(GHPoint start, double distanceMeters,
                                                  double initialHeading, long seed) {
        Random random = new Random(seed);
        initVariation(seed);  // Initialize seed-based variation

        try {
            double heading = Double.isNaN(initialHeading) ? random.nextInt(360) : initialHeading;

            // Simple: out, across, back
            // Total = 2*length + width
            double width = distanceMeters * 0.1;
            double length = (distanceMeters - width) / 2.0;

            length = modifyDistanceWithVariation(length, random, heading);
            width = modifyDistanceWithVariation(width, random, normalizeBearing(heading + 90));

            List<GHPoint> waypoints = new ArrayList<>(4);
            waypoints.add(start);

            // P1: Far point along heading - with variation
            GHPoint p1 = projectWithVariation(start, heading, length, 0);
            waypoints.add(p1);

            // P2: Offset perpendicular - with variation
            double perpBearing = normalizeBearing(heading + 90);
            GHPoint p2 = projectWithVariation(p1, perpBearing, width, 1);
            waypoints.add(p2);

            // Return to start
            waypoints.add(start);

            return waypoints;
        } finally {
            clearVariation();
        }
    }
}
