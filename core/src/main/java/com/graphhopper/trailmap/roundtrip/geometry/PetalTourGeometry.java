/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Petal/Lollipop geometry - stem out, loop at far end, stem back.
 * Good for reaching a scenic area and exploring it before returning.
 */
package com.graphhopper.trailmap.roundtrip.geometry;

import com.graphhopper.util.shapes.GHPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Petal/Lollipop tour geometry - stem + loop at the far end.
 *
 * The route goes out along a stem, makes a loop (petal) at the far end,
 * and returns along the same general corridor.
 *
 * Example with heading=0° (north):
 * <pre>
 *                    .--P2--.
 *                   /        \
 *                  P1        P3
 *                   \        /
 *                    '--P4--'
 *                       |
 *                       | (stem)
 *                       |
 *                       S (start)
 *
 *     Route: S → P1 → P2 → P3 → P4 → S
 * </pre>
 *
 * Use cases:
 * - Reaching a scenic lake/viewpoint and exploring around it
 * - Out-and-back with variety at the destination
 * - When there's only one good corridor to a destination
 */
public class PetalTourGeometry extends AbstractTourGeometry {

    /** Ratio of stem length to petal diameter (e.g., 1.0 = equal stem and petal) */
    private final double stemRatio;

    /** Number of points around the petal (more = rounder loop) */
    private final int petalPoints;

    /**
     * Create with default parameters (stem = petal size, 4 petal points).
     */
    public PetalTourGeometry() {
        this(1.0, 4);
    }

    /**
     * Create with specified parameters.
     *
     * @param stemRatio Ratio of stem length to petal diameter (0.5 = short stem, 2.0 = long stem)
     * @param petalPoints Number of waypoints around the petal (3-6)
     */
    public PetalTourGeometry(double stemRatio, int petalPoints) {
        this.stemRatio = Math.max(0.2, Math.min(3.0, stemRatio));
        this.petalPoints = Math.max(3, Math.min(6, petalPoints));
    }

    @Override
    public String getName() {
        return "petal";
    }

    @Override
    public String getDescription() {
        return "Lollipop shape with stem and loop (stem ratio: " + stemRatio + ")";
    }

    @Override
    public List<GHPoint> generateWaypoints(GHPoint start, double distanceMeters,
                                            double initialHeading, long seed) {
        Random random = new Random(seed);
        initVariation(seed);  // Initialize seed-based variation

        try {
            // Resolve heading
            double heading = Double.isNaN(initialHeading) ? random.nextInt(360) : initialHeading;

            // Calculate dimensions:
            // Total distance = 2 * stem + petal_circumference
            // petal_circumference ≈ π * petal_diameter
            double petalDiameter = distanceMeters / (2.0 * stemRatio + Math.PI);
            double stemLength = stemRatio * petalDiameter;
            double petalRadius = petalDiameter / 2.0;

            // Apply variation
            stemLength = modifyDistanceWithVariation(stemLength, random, heading);
            petalRadius = modifyDistanceWithVariation(petalRadius, random, normalizeBearing(heading + 90));

            List<GHPoint> waypoints = new ArrayList<>(petalPoints + 2);
            waypoints.add(start);

            // Calculate petal center (at end of stem)
            GHPoint petalCenter = project(start, heading, stemLength);

            // Generate petal waypoints in a circle around the petal center
            double angleStep = 360.0 / petalPoints;
            double startAngle = normalizeBearing(heading + 180); // Start facing back toward start

            for (int i = 0; i < petalPoints; i++) {
                // Go around the petal with variation applied
                double angle = normalizeBearing(startAngle + 90 + i * angleStep);
                GHPoint petalPoint = projectWithVariation(petalCenter, angle, petalRadius, i);
                waypoints.add(petalPoint);
            }

            // Close back to start
            waypoints.add(start);

            return waypoints;
        } finally {
            clearVariation();
        }
    }

    /**
     * Generate an asymmetric petal with different entry and exit angles.
     * This creates a more interesting route that doesn't exactly retrace the stem.
     *
     * @param start Starting point
     * @param distanceMeters Target total distance
     * @param initialHeading Initial direction
     * @param seed Random seed
     * @param entryOffset Angle offset for entry point on petal (degrees)
     * @return List of waypoints
     */
    public List<GHPoint> generateAsymmetricPetal(GHPoint start, double distanceMeters,
                                                   double initialHeading, long seed,
                                                   double entryOffset) {
        Random random = new Random(seed);
        initVariation(seed);  // Initialize seed-based variation

        try {
            double heading = Double.isNaN(initialHeading) ? random.nextInt(360) : initialHeading;

            double petalDiameter = distanceMeters / (2.0 * stemRatio + Math.PI);
            double stemLength = stemRatio * petalDiameter;
            double petalRadius = petalDiameter / 2.0;

            stemLength = modifyDistanceWithVariation(stemLength, random, heading);
            petalRadius = modifyDistanceWithVariation(petalRadius, random, normalizeBearing(heading + 90));

            List<GHPoint> waypoints = new ArrayList<>(petalPoints + 2);
            waypoints.add(start);

            // Petal center
            GHPoint petalCenter = project(start, heading, stemLength);

            // Entry point on petal (offset from direct line) - with variation
            double entryAngle = normalizeBearing(heading + 180 + entryOffset);
            GHPoint entryPoint = projectWithVariation(petalCenter, entryAngle, petalRadius, 0);
            waypoints.add(entryPoint);

            // Generate remaining petal points
            double angleStep = (360.0 - Math.abs(entryOffset) * 2) / (petalPoints - 1);
            for (int i = 1; i < petalPoints; i++) {
                double angle = normalizeBearing(entryAngle - i * angleStep);
                GHPoint petalPoint = projectWithVariation(petalCenter, angle, petalRadius, i);
                waypoints.add(petalPoint);
            }

            // Exit point and back to start
            waypoints.add(start);

            return waypoints;
        } finally {
            clearVariation();
        }
    }
}
