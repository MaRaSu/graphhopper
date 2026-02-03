/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Base class for tour geometry implementations with common utility methods.
 */
package com.graphhopper.trailmap.roundtrip.geometry;

import com.graphhopper.util.AngleCalc;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.PointList;
import com.graphhopper.util.shapes.GHPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Abstract base class for tour geometry strategies.
 * Provides common utility methods for coordinate projection and distance calculation.
 *
 * Supports seed-based variation for route diversity:
 * - Radial amplitude variation (±35% of base distance)
 * - Angular offset variation (±18°)
 * - Perpendicular displacement (±15% of base distance)
 * - Quadrant-based asymmetric stretching
 */
public abstract class AbstractTourGeometry implements TourGeometryStrategy {

    protected static final DistanceCalcEarth DIST_CALC = DistanceCalcEarth.DIST_EARTH;
    protected static final AngleCalc ANGLE_CALC = AngleCalc.ANGLE_CALC;

    /**
     * Seed-based variation generator.
     * Initialized via {@link #initVariation(long)} at start of waypoint generation.
     */
    protected SeededVariation variation;

    /**
     * Project a point at a given distance and bearing from a starting point.
     *
     * @param from Starting point
     * @param bearingDegrees Bearing in degrees (0 = north, 90 = east, clockwise)
     * @param distanceMeters Distance in meters
     * @return Projected point
     */
    protected GHPoint project(GHPoint from, double bearingDegrees, double distanceMeters) {
        return DIST_CALC.projectCoordinate(
            from.getLat(), from.getLon(),
            distanceMeters, bearingDegrees
        );
    }

    /**
     * Calculate bearing from one point to another.
     *
     * @param from Starting point
     * @param to Destination point
     * @return Bearing in degrees (0-360)
     */
    protected double bearing(GHPoint from, GHPoint to) {
        return ANGLE_CALC.calcAzimuth(from.getLat(), from.getLon(), to.getLat(), to.getLon());
    }

    /**
     * Calculate distance between two points.
     *
     * @param from Starting point
     * @param to Destination point
     * @return Distance in meters
     */
    protected double distance(GHPoint from, GHPoint to) {
        return DIST_CALC.calcDist(from.getLat(), from.getLon(), to.getLat(), to.getLon());
    }

    /**
     * Calculate the midpoint between two points.
     *
     * @param p1 First point
     * @param p2 Second point
     * @return Midpoint
     */
    protected GHPoint midpoint(GHPoint p1, GHPoint p2) {
        return new GHPoint(
            (p1.getLat() + p2.getLat()) / 2.0,
            (p1.getLon() + p2.getLon()) / 2.0
        );
    }

    /**
     * Slightly modify a distance by ±10% for variation.
     *
     * @param distance Original distance
     * @param random Random source
     * @return Modified distance
     */
    protected double slightlyModifyDistance(double distance, Random random) {
        double modification = random.nextDouble() * 0.1 * distance;
        if (random.nextBoolean()) {
            modification = -modification;
        }
        return distance + modification;
    }

    /**
     * Normalize a bearing to 0-360 range.
     *
     * @param bearing Bearing in degrees
     * @return Normalized bearing (0-360)
     */
    protected double normalizeBearing(double bearing) {
        bearing = bearing % 360;
        if (bearing < 0) {
            bearing += 360;
        }
        return bearing;
    }

    /**
     * Interpolate a point along the line between two points.
     *
     * @param from Starting point
     * @param to Ending point
     * @param ratio Interpolation ratio (0 = from, 1 = to)
     * @return Interpolated point
     */
    protected GHPoint interpolate(GHPoint from, GHPoint to, double ratio) {
        return new GHPoint(
            from.getLat() + (to.getLat() - from.getLat()) * ratio,
            from.getLon() + (to.getLon() - from.getLon()) * ratio
        );
    }

    // ========== Variation Support ==========

    /**
     * Initialize variation generator for this waypoint generation.
     * Call at the start of generateWaypoints() to enable seed-based variation.
     *
     * @param seed Random seed for variation
     */
    protected void initVariation(long seed) {
        this.variation = new SeededVariation(seed);
    }

    /**
     * Clear variation generator.
     * Called after waypoint generation completes.
     */
    protected void clearVariation() {
        this.variation = null;
    }

    /**
     * Check if variation is enabled.
     *
     * @return true if variation generator is initialized
     */
    protected boolean hasVariation() {
        return this.variation != null;
    }

    /**
     * Project a waypoint with seed-based variation applied.
     *
     * Applies three types of variation:
     * 1. Radial: Distance multiplied by factor in [0.65, 1.35]
     * 2. Angular: Bearing offset in [-18°, +18°]
     * 3. Perpendicular: Offset perpendicular to bearing direction
     *
     * @param from Starting point
     * @param baseBearing Base bearing (from geometry formula)
     * @param baseDistance Base distance (from geometry formula)
     * @param waypointIndex Index of this waypoint (for variation lookup)
     * @return Varied waypoint position
     */
    protected GHPoint projectWithVariation(GHPoint from, double baseBearing,
                                            double baseDistance, int waypointIndex) {
        if (variation == null) {
            return project(from, baseBearing, baseDistance);
        }

        // Apply radial variation
        double variedDistance = baseDistance * variation.getRadialFactor(waypointIndex);

        // Apply angular variation
        double variedBearing = normalizeBearing(
            baseBearing + variation.getAngularOffsetDegrees(waypointIndex));

        // Project to varied position
        GHPoint varied = project(from, variedBearing, variedDistance);

        // Apply perpendicular offset
        double perpOffset = baseDistance * variation.getPerpendicularFactor(waypointIndex);
        if (Math.abs(perpOffset) > 1.0) {  // Only apply if significant
            double perpBearing = normalizeBearing(variedBearing + 90);
            varied = project(varied, perpBearing, perpOffset);
        }

        return varied;
    }

    /**
     * Project a waypoint from the start point with variation.
     * Similar to projectWithVariation but projects from a fixed start,
     * useful for shapes that define waypoints relative to center.
     *
     * @param start Starting/center point
     * @param baseBearing Base bearing from start
     * @param baseDistance Base distance from start
     * @param waypointIndex Index of this waypoint
     * @return Varied waypoint position
     */
    protected GHPoint projectFromStartWithVariation(GHPoint start, double baseBearing,
                                                     double baseDistance, int waypointIndex) {
        if (variation == null) {
            return project(start, baseBearing, baseDistance);
        }

        // Apply quadrant-based stretch first
        double stretchedDistance = applyQuadrantStretch(baseDistance, baseBearing);

        // Then apply per-waypoint variation
        return projectWithVariation(start, baseBearing, stretchedDistance, waypointIndex);
    }

    /**
     * Apply quadrant-based stretch to a distance.
     * Quadrant determined by bearing (0-90° = Q0, 90-180° = Q1, etc.)
     *
     * This creates asymmetric shapes by stretching different angular sectors
     * by different amounts (0.75x to 1.25x).
     *
     * @param distance Base distance
     * @param bearing Bearing in degrees (determines quadrant)
     * @return Stretched distance
     */
    protected double applyQuadrantStretch(double distance, double bearing) {
        if (variation == null) {
            return distance;
        }
        int quadrant = ((int) Math.floor(normalizeBearing(bearing) / 90.0)) % 4;
        return distance * variation.getQuadrantStretch(quadrant);
    }

    /**
     * Modify distance with both standard random variation and seed-based variation.
     * Combines ±10% random variation with quadrant stretch.
     *
     * @param distance Base distance
     * @param random Random source for ±10% variation
     * @param bearing Bearing for quadrant stretch
     * @return Modified distance
     */
    protected double modifyDistanceWithVariation(double distance, Random random, double bearing) {
        // Standard ±10% variation
        double modified = slightlyModifyDistance(distance, random);

        // Apply quadrant stretch if variation enabled
        if (variation != null) {
            modified = applyQuadrantStretch(modified, bearing);
        }

        return modified;
    }

    // ========== Path Sampling Utilities ==========

    /**
     * Sample points along a polyline at regular distance intervals.
     * Used by waypoint normalization to generate candidate waypoints from a route.
     *
     * <p>Always includes the first and last points of the polyline.
     * Intermediate points are sampled at approximately the specified interval.
     *
     * @param points The polyline to sample from
     * @param intervalMeters Distance between samples in meters
     * @return List of sampled points including first and last
     */
    public static List<GHPoint> samplePointsAlongPath(PointList points, double intervalMeters) {
        List<GHPoint> sampled = new ArrayList<>();
        if (points == null || points.isEmpty()) {
            return sampled;
        }

        // Always include first point
        sampled.add(new GHPoint(points.getLat(0), points.getLon(0)));

        if (points.size() == 1) {
            return sampled;
        }

        double accumulatedDistance = 0;
        double nextSampleDistance = intervalMeters;

        for (int i = 1; i < points.size(); i++) {
            double segmentDist = DIST_CALC.calcDist(
                points.getLat(i - 1), points.getLon(i - 1),
                points.getLat(i), points.getLon(i)
            );

            // Check if we should sample within this segment
            while (accumulatedDistance + segmentDist >= nextSampleDistance) {
                // Calculate interpolation ratio within this segment
                double distanceIntoSegment = nextSampleDistance - accumulatedDistance;
                double ratio = distanceIntoSegment / segmentDist;

                // Interpolate point
                double lat = points.getLat(i - 1) + ratio * (points.getLat(i) - points.getLat(i - 1));
                double lon = points.getLon(i - 1) + ratio * (points.getLon(i) - points.getLon(i - 1));
                sampled.add(new GHPoint(lat, lon));

                nextSampleDistance += intervalMeters;
            }

            accumulatedDistance += segmentDist;
        }

        // Always include last point if not already included (or very close to it)
        GHPoint lastSampled = sampled.get(sampled.size() - 1);
        GHPoint lastPoint = new GHPoint(
            points.getLat(points.size() - 1),
            points.getLon(points.size() - 1)
        );

        double distToLast = DIST_CALC.calcDist(
            lastSampled.getLat(), lastSampled.getLon(),
            lastPoint.getLat(), lastPoint.getLon()
        );

        // Add last point if it's more than 10% of interval away from last sampled
        if (distToLast > intervalMeters * 0.1) {
            sampled.add(lastPoint);
        }

        return sampled;
    }

    /**
     * Sample points along a polyline at regular distance intervals,
     * returning indices into the original PointList.
     *
     * <p>This variant returns the indices of the closest original points
     * to each sampled position, which is useful when you need to reference
     * edge positions along the path.
     *
     * @param points The polyline to sample from
     * @param intervalMeters Distance between samples in meters
     * @return List of indices into the original PointList
     */
    public static List<Integer> samplePointIndicesAlongPath(PointList points, double intervalMeters) {
        List<Integer> indices = new ArrayList<>();
        if (points == null || points.isEmpty()) {
            return indices;
        }

        // Always include first point
        indices.add(0);

        if (points.size() == 1) {
            return indices;
        }

        double accumulatedDistance = 0;
        double nextSampleDistance = intervalMeters;

        for (int i = 1; i < points.size(); i++) {
            double segmentDist = DIST_CALC.calcDist(
                points.getLat(i - 1), points.getLon(i - 1),
                points.getLat(i), points.getLon(i)
            );

            // Check if we cross a sample threshold in this segment
            while (accumulatedDistance + segmentDist >= nextSampleDistance) {
                double distanceIntoSegment = nextSampleDistance - accumulatedDistance;
                // Use the closer of the two segment endpoints
                if (distanceIntoSegment < segmentDist / 2) {
                    if (indices.isEmpty() || indices.get(indices.size() - 1) != i - 1) {
                        indices.add(i - 1);
                    }
                } else {
                    if (indices.isEmpty() || indices.get(indices.size() - 1) != i) {
                        indices.add(i);
                    }
                }
                nextSampleDistance += intervalMeters;
            }

            accumulatedDistance += segmentDist;
        }

        // Always include last point
        int lastIdx = points.size() - 1;
        if (indices.isEmpty() || indices.get(indices.size() - 1) != lastIdx) {
            indices.add(lastIdx);
        }

        return indices;
    }
}
