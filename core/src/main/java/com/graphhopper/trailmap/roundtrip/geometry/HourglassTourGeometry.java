/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Hourglass geometry - wide at top and bottom, narrow in the middle.
 * Start is at the bottom, route traverses through the narrow middle.
 */
package com.graphhopper.trailmap.roundtrip.geometry;

import com.graphhopper.util.shapes.GHPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Hourglass tour geometry - wide ends, narrow middle, start at bottom.
 *
 * Unlike figure-8 where start is at the crossing point, hourglass has
 * start at the bottom corner. The route goes up one side, across the
 * wide top, through the narrow middle, across the wide bottom, back to start.
 *
 * Example with heading=0° (north):
 * <pre>
 *         P2─────────────────P3
 *           \               /
 *            \             /
 *             \           /
 *              \         /
 *               \       /
 *                \_____/  (narrow middle)
 *               /       \
 *              /         \
 *             /           \
 *            /             \
 *           /               \
 *         P1─────────────────P4
 *         |
 *         S (start at bottom-left)
 *
 *     Route: S → P1 → P2 → P3 → P4 → S
 * </pre>
 *
 * Use cases:
 * - Exploring two areas connected by a narrow corridor
 * - When terrain has a natural pinch point (valley, bridge, pass)
 * - Interior traversal with crossing through the middle
 */
public class HourglassTourGeometry extends AbstractTourGeometry {

    /** How pinched the middle is (0.1 = very narrow, 0.5 = moderate) */
    private final double pinchRatio;

    /** Ratio of top half to bottom half (1.0 = symmetric) */
    private final double heightRatio;

    /**
     * Create with default parameters (moderate pinch, symmetric).
     */
    public HourglassTourGeometry() {
        this(0.2, 1.0);
    }

    /**
     * Create with specified parameters.
     *
     * @param pinchRatio How narrow the middle is relative to width (0.1-0.5)
     * @param heightRatio Ratio of top half to bottom half (0.5-2.0)
     */
    public HourglassTourGeometry(double pinchRatio, double heightRatio) {
        this.pinchRatio = Math.max(0.1, Math.min(0.5, pinchRatio));
        this.heightRatio = Math.max(0.5, Math.min(2.0, heightRatio));
    }

    @Override
    public String getName() {
        return "hourglass";
    }

    @Override
    public String getDescription() {
        return "Hourglass with narrow middle (pinch: " + pinchRatio + ", height ratio: " + heightRatio + ")";
    }

    @Override
    public List<GHPoint> generateWaypoints(GHPoint start, double distanceMeters,
                                            double initialHeading, long seed) {
        Random random = new Random(seed);
        initVariation(seed);

        try {
            double heading = Double.isNaN(initialHeading) ? random.nextInt(360) : initialHeading;

            // Calculate dimensions:
            // Perimeter ≈ 2*height + 2*topWidth + 2*bottomWidth + 4*diagonals
            // Simplified: total ≈ 2*height + 4*avgWidth (rough approximation)
            // With heightRatio r, bottomHeight = height/(1+r), topHeight = height*r/(1+r)

            double totalHeight = distanceMeters / 4.0; // Rough approximation
            double bottomHeight = totalHeight / (1.0 + heightRatio);
            double maxWidth = totalHeight * 0.6; // Width at top and bottom
            double minWidth = maxWidth * pinchRatio; // Width at middle

            // Apply variation
            totalHeight = modifyDistanceWithVariation(totalHeight, random, heading);
            maxWidth = modifyDistanceWithVariation(maxWidth, random, normalizeBearing(heading + 90));

            List<GHPoint> waypoints = new ArrayList<>();
            waypoints.add(start);

            // Bearings
            double perpLeft = normalizeBearing(heading - 90);
            double perpRight = normalizeBearing(heading + 90);

            // P1: Bottom-left corner (start is at bottom-left, P1 is bottom-right)
            // Actually, let's make start truly at bottom-left corner
            // S is at origin, P1 is to the right of S
            GHPoint p1 = projectWithVariation(start, perpRight, maxWidth, 0);
            waypoints.add(p1);

            // P2: Top-right corner (go up from P1, but angled inward then outward)
            // First go to the pinch point on the right side
            GHPoint pinchRight = project(start, heading, bottomHeight);
            pinchRight = project(pinchRight, perpRight, minWidth / 2);

            // Then to top-right
            GHPoint p2 = project(start, heading, totalHeight);
            p2 = projectWithVariation(p2, perpRight, maxWidth / 2, 1);
            waypoints.add(p2);

            // P3: Top-left corner
            GHPoint p3 = project(start, heading, totalHeight);
            p3 = projectWithVariation(p3, perpLeft, maxWidth / 2, 2);
            waypoints.add(p3);

            // P4: Back down to bottom, passing through pinch
            // The route naturally goes through the narrow middle when going from P3 to start

            // Add pinch point as explicit waypoint for better routing through middle
            GHPoint pinchLeft = project(start, heading, bottomHeight);
            pinchLeft = projectWithVariation(pinchLeft, perpLeft, minWidth / 2, 3);
            waypoints.add(pinchLeft);

            // Return to start
            waypoints.add(start);

            return waypoints;
        } finally {
            clearVariation();
        }
    }

    /**
     * Generate a more detailed hourglass with explicit pinch waypoints on both sides.
     * This ensures the route actually goes through the narrow middle.
     *
     * @param start Starting point
     * @param distanceMeters Target total distance
     * @param initialHeading Initial direction
     * @param seed Random seed
     * @return List of waypoints
     */
    public List<GHPoint> generateDetailedHourglass(GHPoint start, double distanceMeters,
                                                     double initialHeading, long seed) {
        Random random = new Random(seed);
        initVariation(seed);

        try {
            double heading = Double.isNaN(initialHeading) ? random.nextInt(360) : initialHeading;

            double totalHeight = distanceMeters / 5.0;
            double bottomHeight = totalHeight / (1.0 + heightRatio);
            double maxWidth = totalHeight * 0.5;
            double minWidth = maxWidth * pinchRatio;

            totalHeight = modifyDistanceWithVariation(totalHeight, random, heading);
            maxWidth = modifyDistanceWithVariation(maxWidth, random, normalizeBearing(heading + 90));

            List<GHPoint> waypoints = new ArrayList<>();
            waypoints.add(start);

            double perpLeft = normalizeBearing(heading - 90);
            double perpRight = normalizeBearing(heading + 90);
            int wpIndex = 0;

            // Bottom-right corner
            GHPoint bottomRight = projectWithVariation(start, perpRight, maxWidth, wpIndex++);
            waypoints.add(bottomRight);

            // Right pinch point (narrow middle, right side)
            GHPoint rightPinch = project(start, heading, bottomHeight);
            rightPinch = projectWithVariation(rightPinch, perpRight, minWidth / 2, wpIndex++);
            waypoints.add(rightPinch);

            // Top-right corner
            GHPoint topRight = project(start, heading, totalHeight);
            topRight = projectWithVariation(topRight, perpRight, maxWidth / 2, wpIndex++);
            waypoints.add(topRight);

            // Top-left corner
            GHPoint topLeft = project(start, heading, totalHeight);
            topLeft = projectWithVariation(topLeft, perpLeft, maxWidth / 2, wpIndex++);
            waypoints.add(topLeft);

            // Left pinch point (narrow middle, left side)
            GHPoint leftPinch = project(start, heading, bottomHeight);
            leftPinch = projectWithVariation(leftPinch, perpLeft, minWidth / 2, wpIndex++);
            waypoints.add(leftPinch);

            // Return to start
            waypoints.add(start);

            return waypoints;
        } finally {
            clearVariation();
        }
    }
}
