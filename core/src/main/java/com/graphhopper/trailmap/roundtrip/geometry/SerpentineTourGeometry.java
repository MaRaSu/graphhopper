/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Serpentine geometry - parallel sweeps covering interior area.
 * Like mowing a lawn - goes out, sweeps back and forth, returns.
 */
package com.graphhopper.trailmap.roundtrip.geometry;

import com.graphhopper.util.shapes.GHPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Serpentine/Zigzag tour geometry - parallel sweeps through interior.
 *
 * Unlike perimeter-tracing geometries (circle, diamond), this pattern
 * traverses the interior of the area with parallel sweeping paths.
 *
 * Example with heading=0° (north), 3 sweeps:
 * <pre>
 *     ←─────────────────P4
 *     │                  │
 *     P3─────────────────→
 *     ←─────────────────P2
 *     │                  │
 *     S──────────────────P1
 *
 *     Route: S → P1 → P2 → P3 → P4 → S
 * </pre>
 *
 * Use cases:
 * - Systematically exploring an area
 * - Covering ground that perimeter shapes miss
 * - When you want to traverse "through" rather than "around"
 */
public class SerpentineTourGeometry extends AbstractTourGeometry {

    /** Number of parallel sweeps (more = tighter coverage) */
    private final int sweepCount;

    /** Width of the sweep area relative to length */
    private final double widthRatio;

    /**
     * Create with default parameters (3 sweeps, moderate width).
     */
    public SerpentineTourGeometry() {
        this(3, 0.5);
    }

    /**
     * Create with specified parameters.
     *
     * @param sweepCount Number of parallel sweeps (2-6)
     * @param widthRatio Width relative to length (0.3-0.8)
     */
    public SerpentineTourGeometry(int sweepCount, double widthRatio) {
        this.sweepCount = Math.max(2, Math.min(6, sweepCount));
        this.widthRatio = Math.max(0.3, Math.min(0.8, widthRatio));
    }

    @Override
    public String getName() {
        return "serpentine";
    }

    @Override
    public String getDescription() {
        return "Serpentine sweeps through interior (sweeps: " + sweepCount + ", width: " + widthRatio + ")";
    }

    @Override
    public List<GHPoint> generateWaypoints(GHPoint start, double distanceMeters,
                                            double initialHeading, long seed) {
        Random random = new Random(seed);
        initVariation(seed);

        try {
            double heading = Double.isNaN(initialHeading) ? random.nextInt(360) : initialHeading;

            // Calculate dimensions:
            // Total distance ≈ sweepCount * length + (sweepCount + 1) * width
            // Solve for length given widthRatio = width/length
            // d = s*L + (s+1)*w = s*L + (s+1)*r*L = L*(s + (s+1)*r)
            double length = distanceMeters / (sweepCount + (sweepCount + 1) * widthRatio);
            double width = length * widthRatio;

            // Apply variation to overall dimensions
            length = modifyDistanceWithVariation(length, random, heading);
            width = modifyDistanceWithVariation(width, random, normalizeBearing(heading + 90));

            List<GHPoint> waypoints = new ArrayList<>();
            waypoints.add(start);

            // Perpendicular direction for sweeps
            double perpLeft = normalizeBearing(heading - 90);
            double perpRight = normalizeBearing(heading + 90);

            int wpIndex = 0;
            boolean goingRight = true;

            // Generate sweep waypoints
            for (int sweep = 0; sweep < sweepCount; sweep++) {
                // Distance along main heading for this sweep
                double sweepDist = length * (sweep + 1) / sweepCount;

                // Point along main heading
                GHPoint alongHeading = project(start, heading, sweepDist);

                if (goingRight) {
                    // Go to right edge
                    GHPoint rightPoint = projectWithVariation(alongHeading, perpRight, width / 2, wpIndex++);
                    waypoints.add(rightPoint);

                    // If not last sweep, add left point for next sweep start
                    if (sweep < sweepCount - 1) {
                        double nextSweepDist = length * (sweep + 1.5) / sweepCount;
                        GHPoint nextAlong = project(start, heading, nextSweepDist);
                        GHPoint leftPoint = projectWithVariation(nextAlong, perpLeft, width / 2, wpIndex++);
                        waypoints.add(leftPoint);
                    }
                } else {
                    // Go to left edge
                    GHPoint leftPoint = projectWithVariation(alongHeading, perpLeft, width / 2, wpIndex++);
                    waypoints.add(leftPoint);

                    // If not last sweep, add right point for next sweep start
                    if (sweep < sweepCount - 1) {
                        double nextSweepDist = length * (sweep + 1.5) / sweepCount;
                        GHPoint nextAlong = project(start, heading, nextSweepDist);
                        GHPoint rightPoint = projectWithVariation(nextAlong, perpRight, width / 2, wpIndex++);
                        waypoints.add(rightPoint);
                    }
                }

                goingRight = !goingRight;
            }

            // Return to start
            waypoints.add(start);

            return waypoints;
        } finally {
            clearVariation();
        }
    }

    /**
     * Generate a denser serpentine with more waypoints per sweep.
     * Creates a more detailed zigzag pattern.
     *
     * @param start Starting point
     * @param distanceMeters Target total distance
     * @param initialHeading Initial direction
     * @param seed Random seed
     * @param pointsPerSweep Additional intermediate points per sweep (1-3)
     * @return List of waypoints
     */
    public List<GHPoint> generateDenseSerpentine(GHPoint start, double distanceMeters,
                                                   double initialHeading, long seed,
                                                   int pointsPerSweep) {
        Random random = new Random(seed);
        initVariation(seed);

        try {
            double heading = Double.isNaN(initialHeading) ? random.nextInt(360) : initialHeading;
            pointsPerSweep = Math.max(1, Math.min(3, pointsPerSweep));

            // Adjust for extra points
            int totalSegments = sweepCount * (1 + pointsPerSweep) + sweepCount;
            double segmentLength = distanceMeters / totalSegments;

            double length = segmentLength * sweepCount * (1 + pointsPerSweep) / (1 + pointsPerSweep);
            double width = length * widthRatio;

            length = modifyDistanceWithVariation(length, random, heading);
            width = modifyDistanceWithVariation(width, random, normalizeBearing(heading + 90));

            List<GHPoint> waypoints = new ArrayList<>();
            waypoints.add(start);

            double perpLeft = normalizeBearing(heading - 90);
            double perpRight = normalizeBearing(heading + 90);

            int wpIndex = 0;
            boolean goingRight = true;

            for (int sweep = 0; sweep < sweepCount; sweep++) {
                double sweepStartDist = length * sweep / sweepCount;
                double sweepEndDist = length * (sweep + 1) / sweepCount;

                // Intermediate points along sweep
                for (int p = 0; p <= pointsPerSweep; p++) {
                    double progress = (double) (p + 1) / (pointsPerSweep + 1);
                    double dist = sweepStartDist + (sweepEndDist - sweepStartDist) * progress;

                    GHPoint alongHeading = project(start, heading, dist);

                    if (goingRight) {
                        // Interpolate from left to right across sweep
                        double perpProgress = (double) p / pointsPerSweep;
                        double perpOffset = -width / 2 + width * perpProgress;
                        GHPoint point = projectWithVariation(alongHeading,
                            perpOffset >= 0 ? perpRight : perpLeft,
                            Math.abs(perpOffset), wpIndex++);
                        waypoints.add(point);
                    } else {
                        // Interpolate from right to left
                        double perpProgress = (double) p / pointsPerSweep;
                        double perpOffset = width / 2 - width * perpProgress;
                        GHPoint point = projectWithVariation(alongHeading,
                            perpOffset >= 0 ? perpRight : perpLeft,
                            Math.abs(perpOffset), wpIndex++);
                        waypoints.add(point);
                    }
                }

                goingRight = !goingRight;
            }

            waypoints.add(start);
            return waypoints;
        } finally {
            clearVariation();
        }
    }
}
