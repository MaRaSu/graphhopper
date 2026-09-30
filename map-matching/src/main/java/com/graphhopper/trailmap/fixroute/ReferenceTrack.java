package com.graphhopper.trailmap.fixroute;

import java.util.ArrayList;
import java.util.List;

/**
 * The saved reference track with cumulative arc length, and slicing by arc position.
 * Positions are metres along the track (haversine, cumulative over its vertices) — the unit of
 * {@code ref_from_m} / {@code ref_to_m} in the response.
 */
public class ReferenceTrack {

    private final List<double[]> points;
    private final double[] cum;

    public ReferenceTrack(List<double[]> points) {
        if (points == null || points.size() < 2) {
            throw new IllegalArgumentException("reference must contain at least 2 points");
        }
        this.points = points;
        this.cum = WaypointAligner.cumulative(points);
    }

    public List<double[]> points() {
        return points;
    }

    public double[] cumulative() {
        return cum;
    }

    public double lengthM() {
        return cum[cum.length - 1];
    }

    /** Point on the track at arc position {@code s} (clamped to the track). */
    public double[] pointAt(double s) {
        if (s <= 0) return points.get(0);
        if (s >= lengthM()) return points.get(points.size() - 1);
        int i = segmentIndex(s);
        return interpolate(i, s);
    }

    /**
     * The part of the track between arc positions {@code from} and {@code to} (inclusive,
     * interpolated end points). A zero-length slice returns the single point twice.
     */
    public List<double[]> slice(double from, double to) {
        double a = Math.max(0, Math.min(from, to));
        double b = Math.min(lengthM(), Math.max(from, to));
        List<double[]> out = new ArrayList<>();
        out.add(pointAt(a));
        int i = segmentIndex(a);
        for (int v = i + 1; v < points.size() && cum[v] < b; v++) {
            if (cum[v] > a) out.add(points.get(v));
        }
        out.add(pointAt(b));
        return out;
    }

    /**
     * Slice {@code [from, to]} with each end moved (inward, by at most {@code windowM}) to the
     * track point closest to the new route's own start / end.
     *
     * <p>Why: legacy OSRM-era tracks contain a straight "spike" from the road out to the exact
     * waypoint coordinate and back at every waypoint (e.g. route 22708: road → waypoint → same
     * road vertex, 20 m each way). The client's GH route starts at the waypoint's road snap and has
     * no spike, so without anchoring every leg next to an off-road waypoint would look deviating.
     * The window is bounded by the alignment offset limit, so a genuine difference is never hidden.
     */
    public List<double[]> anchoredSlice(double from, double to, double[] routeStart, double[] routeEnd,
                                        double windowM) {
        return anchoredSlice(from, to, routeStart, routeEnd, windowM, windowM);
    }

    /** As above with separate windows per end (a spike arm is as long as its waypoint is off-road). */
    public List<double[]> anchoredSlice(double from, double to, double[] routeStart, double[] routeEnd,
                                        double startWindowM, double endWindowM) {
        double a = Math.min(from, to), b = Math.max(from, to);
        // Each window may span the whole slice (a leg can be shorter than a spike arm); the two ends
        // are only kept in order.
        double a2 = closestArc(a, Math.min(b, a + startWindowM), routeStart);
        double b2 = closestArc(Math.max(a, b - endWindowM), b, routeEnd);
        if (b2 < a2) b2 = a2;
        return slice(a2, b2);
    }

    /** Arc position in [lo, hi] closest to point p, sampled every metre. */
    private double closestArc(double lo, double hi, double[] p) {
        double best = lo, bestD = Double.POSITIVE_INFINITY;
        for (double s = lo; s <= hi + 1e-9; s += 1.0) {
            double[] q = pointAt(Math.min(s, hi));
            double d = com.graphhopper.util.DistanceCalcEarth.DIST_EARTH.calcDist(p[0], p[1], q[0], q[1]);
            if (d < bestD) {
                bestD = d;
                best = Math.min(s, hi);
            }
        }
        return best;
    }

    /**
     * Arc position of the track point closest to {@code p}, searching only the part of the track
     * within {@code windowM} of {@code aroundArc} (so a route passing the same place twice resolves
     * to the pass being worked on). Returns {@code {arc, distance}}.
     */
    public double[] projectNear(double[] p, double aroundArc, double windowM) {
        return projectBetween(p, aroundArc - windowM, aroundArc + windowM);
    }

    /** As {@link #projectNear} over the explicit arc range {@code [loArc, hiArc]}. */
    public double[] projectBetween(double[] p, double loArc, double hiArc) {
        double aroundArc = (loArc + hiArc) / 2;
        double lo = Math.max(0, loArc), hi = Math.min(lengthM(), hiArc);
        int i0 = segmentIndex(lo), i1 = segmentIndex(hi);
        double bestArc = aroundArc, bestD = Double.POSITIVE_INFINITY;
        for (int i = i0; i <= i1 && i + 1 < points.size(); i++) {
            double[] tp = WaypointAligner.project(p[0], p[1], points.get(i), points.get(i + 1));
            double arc = cum[i] + tp[1] * (cum[i + 1] - cum[i]);
            double d = tp[0];
            // A segment reaching past the window counts with its part inside it: the closest point
            // is then the window's end (not skipped — a waypoint right at the window start, where
            // a run begins, projects a hair before it).
            if (arc < lo || arc > hi) {
                arc = arc < lo ? lo : hi;
                double[] q = pointAt(arc);
                d = com.graphhopper.util.DistanceCalcEarth.DIST_EARTH.calcDist(p[0], p[1], q[0], q[1]);
            }
            if (d < bestD) {
                bestD = d;
                bestArc = arc;
            }
        }
        return new double[]{bestArc, bestD};
    }

    /** A point of the track with its arc position; {@code grid} marks global-grid points. */
    public record ArcPoint(double arcM, double[] point, boolean grid) {
    }

    /**
     * The slice {@code [from, to]} as its own vertices plus points on a GLOBAL arc grid (every
     * {@code stepM} metres from the start of the whole track), in track order. The grid does not
     * depend on where the slice starts, so two calls over overlapping slices offer the same grid
     * positions — the basis of stable waypoint candidates across runs.
     */
    public List<ArcPoint> slicePointsWithGrid(double from, double to, double stepM) {
        double a = Math.max(0, Math.min(from, to)), b = Math.min(lengthM(), Math.max(from, to));
        List<ArcPoint> out = new ArrayList<>();
        out.add(new ArcPoint(a, pointAt(a), false));
        int v = segmentIndex(a) + 1;
        double g = Math.floor(a / stepM) * stepM + stepM;
        while (true) {
            double nextV = v < points.size() ? cum[v] : Double.POSITIVE_INFINITY;
            double next = Math.min(nextV, g);
            if (next >= b) break;
            if (g <= nextV) {
                out.add(new ArcPoint(g, pointAt(g), true));
                if (g == nextV) v++;
                g += stepM;
            } else {
                if (cum[v] > a) out.add(new ArcPoint(cum[v], points.get(v), false));
                v++;
            }
        }
        out.add(new ArcPoint(b, pointAt(b), false));
        return out;
    }

    /** Largest segment index i with cum[i] <= s (so s lies on segment i → i+1). */
    private int segmentIndex(double s) {
        int lo = 0, hi = cum.length - 2;
        while (lo < hi) {
            int mid = (lo + hi + 1) >>> 1;
            if (cum[mid] <= s) lo = mid;
            else hi = mid - 1;
        }
        return lo;
    }

    private double[] interpolate(int i, double s) {
        double len = cum[i + 1] - cum[i];
        double t = len <= 0 ? 0 : (s - cum[i]) / len;
        double[] a = points.get(i), b = points.get(i + 1);
        return new double[]{a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])};
    }
}
