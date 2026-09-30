package com.graphhopper.trailmap.fixroute;

import com.graphhopper.util.AngleCalc;

/** Small geometry helpers shared by the {@code /fix_route} engines. */
final class RouteFixerMath {

    private RouteFixerMath() {
    }

    /** Azimuth [deg, 0 = north, clockwise] from {@code from} to {@code to} ({@code [lat, lng]}). */
    static double bearing(double[] from, double[] to) {
        return AngleCalc.ANGLE_CALC.calcAzimuth(from[0], from[1], to[0], to[1]);
    }

    static double round6(double v) {
        return Math.round(v * 1e6) / 1e6;
    }

    static double round1(double v) {
        return Math.round(v * 10) / 10.0;
    }
}
