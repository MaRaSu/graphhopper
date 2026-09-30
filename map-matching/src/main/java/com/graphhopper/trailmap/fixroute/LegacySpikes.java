package com.graphhopper.trailmap.fixroute;

import com.graphhopper.util.DistanceCalcEarth;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

/**
 * Removes legacy OSRM "waypoint spikes" from a reference slice before comparison.
 *
 * <p>Corpus finding (design doc §3.2): OSRM-era saved tracks contain, at every off-road waypoint, a
 * straight excursion from the road out to the exact waypoint coordinate and straight back to the
 * same road vertex — {@code …, A, (points out), tip, (points back), A, …}. The out and back halves
 * retrace each other, and the client may have inserted extra points along them (route 32997: a
 * 150 m arm with 4 points each way). The client's GH route has no such excursion. Once the client
 * has moved the waypoint onto the road (it overwrites waypoints with GH's snapped route ends), the
 * spike sits in the MIDDLE of a slice where end-anchoring cannot absorb it.
 *
 * <p>An excursion {@code line[i..j]} is a spike when it returns to its base
 * ({@code line[j] ≈ line[i]}, within {@link #BASE_EPS_M}), reaches at least {@link #MIN_ARM_M} from
 * it, every point of the outbound half has a matching point on the return half (a retrace, not a
 * loop around a block), and its tip is NOT on a road the leg may use ({@code offRoad}): a real
 * dead-end out-and-back on a usable road is kept.
 *
 * <p>Only the OFF-ROAD part is the spike: when the excursion first runs along a dead-end road and
 * only its last metres leave it for the old waypoint position (route 32997: 250 m along a spur,
 * then 15 m off its end — the client has since moved the waypoint onto the road), the on-road
 * out-and-back is a real part of the route and stays; only the part beyond the road is dropped.
 */
final class LegacySpikes {

    static final double BASE_EPS_M = 3.0;
    static final double MIN_ARM_M = 5.0;
    static final double RETRACE_EPS_M = 3.0;
    /** How far ahead (vertices) the return to the base is searched. */
    static final int MAX_SPIKE_VERTICES = 40;

    private static final DistanceCalcEarth DIST = DistanceCalcEarth.DIST_EARTH;

    private LegacySpikes() {
    }

    static List<double[]> remove(List<double[]> line, Predicate<double[]> offRoad) {
        int n = line.size();
        if (n < 3) return line;
        List<double[]> out = new ArrayList<>(n);
        int i = 0;
        while (i < n) {
            int[] sp = spike(line, i, offRoad);
            if (sp == null) {
                out.add(line.get(i++));
                continue;
            }
            // Keep up to the spike's own base, skip the excursion (vertices sp[0]+1 .. sp[1]-1).
            for (int k = i; k <= sp[0]; k++) out.add(line.get(k));
            i = sp[1];
        }
        // Never drop the true end of the slice.
        double[] last = line.get(n - 1);
        if (out.get(out.size() - 1) != last && d(out.get(out.size() - 1), last) > BASE_EPS_M) out.add(last);
        return out.size() >= 2 ? out : line;
    }

    /** Same detection as {@link #remove}, as a per-vertex keep mask (spike vertices = false). */
    static boolean[] keepMask(List<double[]> line, Predicate<double[]> offRoad) {
        int n = line.size();
        boolean[] keep = new boolean[n];
        java.util.Arrays.fill(keep, true);
        int i = 0;
        while (i < n) {
            int[] sp = spike(line, i, offRoad);
            if (sp != null) {
                for (int k = sp[0] + 1; k < sp[1]; k++) keep[k] = false;
                i = sp[1];
            } else {
                i++;
            }
        }
        keep[0] = true;
        keep[n - 1] = true;
        return keep;
    }

    /**
     * The spike starting at i as {@code {keptBefore, keptAfter}}: vertices strictly between are
     * dropped. A plain spike drops the excursion and its returning base vertex (≈ the base). When
     * the excursion first runs along a usable road, only the part off the road is dropped: from the
     * last outbound point still on a road to the first returning point back on one (they are close
     * together at the road's end).
     */
    private static int[] spike(List<double[]> line, int i, Predicate<double[]> offRoad) {
        int j = spikeEnd(line, i, offRoad);
        if (j < 0) return null;
        int tip = i + 1;
        for (int k = i + 1; k < j; k++) if (d(line.get(i), line.get(k)) > d(line.get(i), line.get(tip))) tip = k;
        int a = i;
        while (a + 1 < tip && !offRoad.test(line.get(a + 1))) a++;
        int b = j;
        while (b - 1 > tip && !offRoad.test(line.get(b - 1))) b--;
        if (SHRINK && a > i && b < j && d(line.get(a), line.get(b)) <= ROAD_END_EPS_M) return new int[]{a, b};
        return new int[]{i, j + 1};
    }

    /** Comparison switch (before/after reviews): {@code -Dfix.spikeShrink=false} drops whole excursions. */
    static boolean SHRINK = !"false".equals(System.getProperty("fix.spikeShrink"));

    /** Where the on-road part of an excursion ends, its out and back points are this close [m]. */
    static final double ROAD_END_EPS_M = 40.0;

    /** Index j of the returning base vertex if a spike starts at i, else -1. */
    private static int spikeEnd(List<double[]> line, int i, Predicate<double[]> offRoad) {
        double[] base = line.get(i);
        int limit = Math.min(line.size() - 1, i + MAX_SPIKE_VERTICES);
        for (int j = i + 2; j <= limit; j++) {
            if (d(base, line.get(j)) > BASE_EPS_M) continue;
            int tip = i + 1;
            for (int k = i + 1; k < j; k++) if (d(base, line.get(k)) > d(base, line.get(tip))) tip = k;
            if (d(base, line.get(tip)) < MIN_ARM_M) return -1;
            if (!retraces(line, i, tip, j)) return -1;
            return offRoad.test(line.get(tip)) ? j : -1;
        }
        return -1;
    }

    /** Every outbound point (i, tip) is within RETRACE_EPS_M of the return polyline (tip..j). */
    private static boolean retraces(List<double[]> line, int i, int tip, int j) {
        for (int k = i + 1; k < tip; k++) {
            double best = Double.POSITIVE_INFINITY;
            for (int m = tip; m < j; m++) {
                best = Math.min(best, WaypointAligner.project(line.get(k)[0], line.get(k)[1],
                        line.get(m), line.get(m + 1))[0]);
            }
            if (best > RETRACE_EPS_M) return false;
        }
        return true;
    }

    private static double d(double[] a, double[] b) {
        return DIST.calcDist(a[0], a[1], b[0], b[1]);
    }
}
