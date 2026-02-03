/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Base class for route fixers with common utility methods.
 */
package com.graphhopper.trailmap.roundtrip.fixing;

import com.graphhopper.util.AngleCalc;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.shapes.GHPoint;

/**
 * Abstract base class for route fixers.
 * Provides common utility methods for coordinate calculations.
 */
public abstract class AbstractRouteFixer implements RouteFixer {

    protected static final DistanceCalcEarth DIST_CALC = DistanceCalcEarth.DIST_EARTH;
    protected static final AngleCalc ANGLE_CALC = AngleCalc.ANGLE_CALC;

    /**
     * Project a point at a given distance and bearing from a starting point.
     */
    protected GHPoint project(GHPoint from, double bearingDegrees, double distanceMeters) {
        return DIST_CALC.projectCoordinate(
            from.getLat(), from.getLon(),
            distanceMeters, bearingDegrees
        );
    }

    /**
     * Calculate bearing from one point to another.
     */
    protected double bearing(GHPoint from, GHPoint to) {
        return ANGLE_CALC.calcAzimuth(from.getLat(), from.getLon(), to.getLat(), to.getLon());
    }

    /**
     * Calculate distance between two points.
     */
    protected double distance(GHPoint from, GHPoint to) {
        return DIST_CALC.calcDist(from.getLat(), from.getLon(), to.getLat(), to.getLon());
    }

    /**
     * Calculate the midpoint between two points.
     */
    protected GHPoint midpoint(GHPoint p1, GHPoint p2) {
        return new GHPoint(
            (p1.getLat() + p2.getLat()) / 2.0,
            (p1.getLon() + p2.getLon()) / 2.0
        );
    }

    /**
     * Normalize a bearing to 0-360 range.
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
     */
    protected GHPoint interpolate(GHPoint from, GHPoint to, double ratio) {
        return new GHPoint(
            from.getLat() + (to.getLat() - from.getLat()) * ratio,
            from.getLon() + (to.getLon() - from.getLon()) * ratio
        );
    }

    /**
     * Find the index of the point furthest from start.
     */
    protected int findFurthestFromStart(java.util.List<GHPoint> waypoints) {
        if (waypoints.size() < 2) return 0;

        GHPoint start = waypoints.get(0);
        int furthestIdx = 1;
        double maxDist = 0;

        for (int i = 1; i < waypoints.size() - 1; i++) {
            double dist = distance(start, waypoints.get(i));
            if (dist > maxDist) {
                maxDist = dist;
                furthestIdx = i;
            }
        }

        return furthestIdx;
    }
}
