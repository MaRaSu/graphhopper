package com.graphhopper.trailmap.fixroute;

import com.carrotsearch.hppc.IntArrayList;
import com.carrotsearch.hppc.LongObjectHashMap;
import com.graphhopper.util.DistanceCalcEarth;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.List;

/**
 * Order-aware comparison of two lines — "does the new route follow the saved reference?"
 * (design doc §3.2 / §3.6, the materiality test).
 *
 * <p>Both lines are resampled every {@link #STEP_M} metres and tested for a <b>discrete Fréchet
 * coupling within {@code toleranceM}</b>: an order-preserving pairing of the two sample sequences
 * (both walked from start to end, neither going backwards) in which every paired sample is within
 * the tolerance. This is what "the route follows the reference" means once order matters: unlike a
 * plain closest-point test it cannot be fooled by an out-and-back or a loop passing the same place
 * twice, and it checks both directions at once (every route point is near the reference AND every
 * reference point is near the route, in order).
 *
 * <p>The decision is computed on a sparse free-space grid (only sample pairs within the tolerance are
 * ever visited, found through a spatial hash), so the cost is linear in line length for lines that
 * are actually similar.
 *
 * <p>The one-way maxima (route→reference, reference→route) are reported for diagnostics only; the
 * verdict is the Fréchet decision plus the length-ratio guard (zig-zags close to the line pass the
 * distance test but not the length test).
 */
public final class PathSimilarity {

    /** Resampling step [m]. Discretisation error of the decision is at most STEP_M / 2. */
    public static final double STEP_M = 2.0;

    private static final double M_PER_DEG_LAT = 111_320.0;
    private static final DistanceCalcEarth DIST = DistanceCalcEarth.DIST_EARTH;

    private PathSimilarity() {
    }

    /**
     * @param frechetWithin       an order-preserving coupling within the tolerance exists
     * @param failedAtRouteM      when not within: arc position on the route [m] where coupling was
     *                            lost (NaN when within)
     * @param routeToRefMaxM      max distance from a route sample to the nearest reference sample
     * @param refToRouteMaxM      max distance from a reference sample to the nearest route sample
     * @param frechetBumpWithin   an order-preserving coupling within the (looser) bump tolerance exists
     * @param longestExcessRunM   longest stretch [m], in either direction, where one line is further
     *                            than the tolerance from the other
     */
    public record Result(boolean frechetWithin, double failedAtRouteM, double routeLengthM,
                         double referenceLengthM, double routeToRefMaxM, double refToRouteMaxM,
                         boolean frechetBumpWithin, double longestExcessRunM) {

        public double lengthRatio() {
            return referenceLengthM <= 0 ? Double.POSITIVE_INFINITY : routeLengthM / referenceLengthM;
        }

        private boolean lengthOk(double maxLenRatioDev, double minLenSlackM) {
            double slack = Math.max(minLenSlackM, maxLenRatioDev * referenceLengthM);
            return Math.abs(routeLengthM - referenceLengthM) <= slack;
        }

        /** Strict verdict: coupling within tolerance AND length within max(relative, absolute) slack. */
        public boolean follows(double maxLenRatioDev, double minLenSlackM) {
            return frechetWithin && lengthOk(maxLenRatioDev, minLenSlackM);
        }

        /**
         * Tiered verdict: as {@link #follows}, but a short local bump (within the bump tolerance,
         * no longer than {@code maxBumpM}) does not fail the leg — e.g. a junction corner drawn
         * differently by the old engine, or a road re-drawn a few metres over a short stretch.
         * A sustained offset (a parallel way) still fails.
         */
        public boolean followsTiered(double maxBumpM, double maxLenRatioDev, double minLenSlackM) {
            boolean shape = frechetWithin || (frechetBumpWithin && longestExcessRunM <= maxBumpM);
            return shape && lengthOk(maxLenRatioDev, minLenSlackM);
        }
    }

    public static Result compare(List<double[]> route, List<double[]> reference, double toleranceM) {
        return compare(route, reference, toleranceM, toleranceM);
    }

    public static Result compare(List<double[]> route, List<double[]> reference, double toleranceM,
                                 double bumpToleranceM) {
        List<double[]> r = resample(route);
        List<double[]> f = resample(reference);
        double routeLen = length(route), refLen = length(reference);

        // Local metric projection (equirectangular around the first reference point).
        double lat0 = f.get(0)[0];
        double mPerDegLon = M_PER_DEG_LAT * Math.cos(Math.toRadians(lat0));
        double[][] rp = project(r, lat0, mPerDegLon);
        double[][] fp = project(f, lat0, mPerDegLon);

        // One-way distances. Nearest within a generous radius; beyond it report the radius.
        double radius = Math.max(4 * Math.max(toleranceM, bumpToleranceM), 100);
        SpatialHash wideF = new SpatialHash(fp, radius), wideR = new SpatialHash(rp, radius);
        double[] dr = new double[rp.length], df = new double[fp.length];
        double r2f = 0, f2r = 0;
        for (int i = 0; i < rp.length; i++) r2f = Math.max(r2f, dr[i] = wideF.nearest(rp[i], radius, fp));
        for (int j = 0; j < fp.length; j++) f2r = Math.max(f2r, df[j] = wideR.nearest(fp[j], radius, rp));
        double run = Math.max(longestRun(dr, toleranceM), longestRun(df, toleranceM)) * STEP_M;

        double failedAt = frechet(rp, fp, toleranceM, routeLen);
        boolean within = Double.isNaN(failedAt);
        boolean bumpWithin = within || (bumpToleranceM > toleranceM
                && Double.isNaN(frechet(rp, fp, bumpToleranceM, routeLen)));
        return new Result(within, failedAt, routeLen, refLen, r2f, f2r, bumpWithin, run);
    }

    /** Discrete Fréchet decision on the sparse free space; NaN when within, else where it failed [m]. */
    private static double frechet(double[][] rp, double[][] fp, double tol, double routeLen) {
        SpatialHash hashF = new SpatialHash(fp, tol);
        double tol2 = tol * tol;
        int n = rp.length, m = fp.length;
        BitSet prev = new BitSet(m), cur = new BitSet(m);
        for (int i = 0; i < n; i++) {
            cur.clear();
            int[] js = hashF.within(rp[i], tol, tol2, fp).toArray();
            Arrays.sort(js); // ascending, so (i, j-1) is decided before (i, j)
            boolean any = false;
            for (int j : js) {
                boolean reach = (i == 0)
                        ? (j == 0) || cur.get(j - 1)
                        : prev.get(j) || (j > 0 && (prev.get(j - 1) || cur.get(j - 1)));
                if (reach) {
                    cur.set(j);
                    any = true;
                }
            }
            if (!any) return i * STEP_M;
            BitSet t = prev;
            prev = cur;
            cur = t;
        }
        return prev.get(m - 1) ? Double.NaN : routeLen;
    }

    /** Longest run of consecutive samples above the threshold, in samples. */
    private static int longestRun(double[] d, double threshold) {
        int best = 0, cur = 0;
        for (double v : d) {
            cur = v > threshold ? cur + 1 : 0;
            best = Math.max(best, cur);
        }
        return best;
    }

    /**
     * Resampling every {@link #STEP_M} along each segment, always keeping every original vertex.
     * Keeping the vertices makes the measurement independent of where sampling starts: the peak
     * distance between two polylines sits at or next to a vertex, and a phase shift (e.g. the slice
     * start moving by a metre after the client snaps a waypoint) must not move the samples there.
     */
    static List<double[]> resample(List<double[]> line) {
        List<double[]> out = new ArrayList<>();
        out.add(line.get(0));
        for (int i = 1; i < line.size(); i++) {
            double[] a = line.get(i - 1), b = line.get(i);
            double seg = DIST.calcDist(a[0], a[1], b[0], b[1]);
            if (seg <= 0) continue;
            int n = (int) Math.ceil(seg / STEP_M);
            for (int k = 1; k < n; k++) {
                double t = (double) k / n;
                out.add(new double[]{a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])});
            }
            out.add(b);
        }
        return out;
    }

    static double length(List<double[]> line) {
        double s = 0;
        for (int i = 1; i < line.size(); i++) {
            double[] a = line.get(i - 1), b = line.get(i);
            s += DIST.calcDist(a[0], a[1], b[0], b[1]);
        }
        return s;
    }

    private static double[][] project(List<double[]> pts, double lat0, double mPerDegLon) {
        double[][] out = new double[pts.size()][];
        for (int i = 0; i < pts.size(); i++) {
            double[] p = pts.get(i);
            out[i] = new double[]{p[1] * mPerDegLon, (p[0] - lat0) * M_PER_DEG_LAT};
        }
        return out;
    }

    /** Uniform grid over projected points; cell size = query radius. */
    private static final class SpatialHash {
        private final double cell;
        private final LongObjectHashMap<IntArrayList> cells = new LongObjectHashMap<>();

        SpatialHash(double[][] pts, double cellSize) {
            this.cell = cellSize;
            for (int i = 0; i < pts.length; i++) {
                long key = key(cx(pts[i][0]), cy(pts[i][1]));
                IntArrayList l = cells.get(key);
                if (l == null) {
                    l = new IntArrayList(4);
                    cells.put(key, l);
                }
                l.add(i);
            }
        }

        private long cx(double x) {
            return (long) Math.floor(x / cell);
        }

        private long cy(double y) {
            return (long) Math.floor(y / cell);
        }

        private static long key(long x, long y) {
            return (x << 32) ^ (y & 0xffffffffL);
        }

        IntArrayList within(double[] p, double r, double r2, double[][] pts) {
            IntArrayList out = new IntArrayList();
            long x = cx(p[0]), y = cy(p[1]);
            for (long dx = -1; dx <= 1; dx++) {
                for (long dy = -1; dy <= 1; dy++) {
                    IntArrayList l = cells.get(key(x + dx, y + dy));
                    if (l == null) continue;
                    for (int k = 0; k < l.size(); k++) {
                        int j = l.get(k);
                        double ex = pts[j][0] - p[0], ey = pts[j][1] - p[1];
                        if (ex * ex + ey * ey <= r2) out.add(j);
                    }
                }
            }
            return out;
        }

        /** Distance to the nearest point within {@code r}, or {@code r} when none is closer. */
        double nearest(double[] p, double r, double[][] pts) {
            double best = r * r;
            long x = cx(p[0]), y = cy(p[1]);
            for (long dx = -1; dx <= 1; dx++) {
                for (long dy = -1; dy <= 1; dy++) {
                    IntArrayList l = cells.get(key(x + dx, y + dy));
                    if (l == null) continue;
                    for (int k = 0; k < l.size(); k++) {
                        int j = l.get(k);
                        double ex = pts[j][0] - p[0], ey = pts[j][1] - p[1];
                        double d2 = ex * ex + ey * ey;
                        if (d2 < best) best = d2;
                    }
                }
            }
            return Math.sqrt(best);
        }
    }
}
