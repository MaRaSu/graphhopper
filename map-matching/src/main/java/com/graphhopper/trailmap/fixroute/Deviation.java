package com.graphhopper.trailmap.fixroute;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * What counts as a DEVIATION of a route from the saved route (owner decision 2026-09-25): a
 * stretch where one line is more than {@link #BASE_M} from the other, reaching at least the peak
 * threshold and lasting at least the minimum length. Measured both ways — the route going
 * elsewhere, and the saved route going where the route does not — at 2 m resolution.
 *
 * <p>Deliberately corridor-scale: a long run close alongside (a cycleway beside the old road, a
 * trail redrawn in OSM, the saved line's own simplification) never reaches the peak, so it is not
 * a deviation however long it is. Next to a FIXED waypoint the first / last {@link Rule#zoneM} of
 * each line are not judged: there the waypoint's own placement decides where the route goes
 * (reported separately as a snap mismatch). Next to an added waypoint — on the matched road —
 * everything is. The numbers are request options ({@link Rule}), defaults owner-calibrated.
 */
final class Deviation {

    /** A stretch starts where one line is further than this [m] from the other. */
    static final double BASE_M = 8.0;
    /**
     * The rule's numbers (request options, per-leg overridable; see {@link FixRouteRequest.Options}):
     *
     * @param peakM        a stretch reaching this far off [m]...
     * @param minLengthM   ...over at least this length [m] is a deviation;
     * @param detourMinM   so is a DETOUR: a stretch at least this much longer [m] than the part of
     *                     the other line it replaces...
     * @param detourMinRatio ...and by at least this share of it. Catches a way round that stays close
     *                     to the saved line (a closed bridge: along one bank to the next bridge and
     *                     back); a parallel way of about the same length is not one however long.
     * @param zoneM        not judged: this far [m] along each line from an end at a fixed waypoint;
     * @param armM         a stretch running out of that zone and over within this far [m] of the fixed
     *                     waypoint is the waypoint's own effect too (the old router's "arm" out of
     *                     the waypoint, today's route leaving it another way for a moment).
     */
    record Rule(double peakM, double minLengthM, double detourMinM, double detourMinRatio, double zoneM, double armM) {
        /** Stricter peak for created pieces (a margin, so a re-run does not judge them the other way). */
        Rule tighter(double marginM) {
            return new Rule(peakM - marginM, minLengthM, detourMinM, detourMinRatio, zoneM, armM);
        }
    }

    private static final double STEP_M = PathSimilarity.STEP_M;
    private static final double M_PER_DEG_LAT = 111_320.0;

    /**
     * One stretch: where along its line [m], how far off at most, and the length of the other
     * line's part between where it leaves and rejoins (its counterpart).
     */
    record Stretch(boolean onRoute, double fromM, double toM, double peakM, double counterpartM) {
        double lengthM() {
            return toM - fromM;
        }

        double extraM() {
            return lengthM() - counterpartM;
        }

        boolean isDeviation(Rule r) {
            return (peakM >= r.peakM() && lengthM() >= r.minLengthM())
                    || extraM() >= Math.max(r.detourMinM(), r.detourMinRatio() * counterpartM);
        }
    }

    private Deviation() {
    }

    /**
     * Metres not judged at each end of each line. At a fixed waypoint {@link Rule#zoneM}; on the route
     * side more when the waypoint's snap forces a connector (a loop off a wrong-way carriageway, a
     * footway run): that part is the waypoint's placement — reported as a snap mismatch, and no
     * added waypoint can change it.
     */
    record Ends(double routeStartM, double routeEndM, double savedStartM, double savedEndM) {
        static Ends of(Rule r, boolean fixedStart, boolean fixedEnd) {
            double z = r.zoneM();
            return new Ends(fixedStart ? z : 0, fixedEnd ? z : 0, fixedStart ? z : 0, fixedEnd ? z : 0);
        }

        Ends withConnectors(double startConnectorM, double endConnectorM) {
            return new Ends(Math.max(routeStartM, startConnectorM), Math.max(routeEndM, endConnectorM), savedStartM, savedEndM);
        }
    }

    /** True when the route has no deviation from the saved line (both {@code [lat, lng]}). */
    static boolean within(List<double[]> route, List<double[]> saved, Rule rule, Ends ends) {
        for (Stretch s : stretches(route, saved, rule, ends)) {
            if (s.isDeviation(rule)) return false;
        }
        return true;
    }

    /** All stretches beyond {@link #BASE_M}, both directions; distances capped at {@code 2 × peakM}. */
    static List<Stretch> stretches(List<double[]> route, List<double[]> saved, Rule rule, Ends ends) {
        List<Stretch> out = new ArrayList<>();
        if (route == null || saved == null || route.size() < 2 || saved.size() < 2) return out;
        double lat0 = saved.get(0)[0], mLon = M_PER_DEG_LAT * Math.cos(Math.toRadians(lat0));
        double[][] r = project(PathSimilarity.resample(route), lat0, mLon);
        double[][] f = project(PathSimilarity.resample(saved), lat0, mLon);
        double cap = Math.max(2 * rule.peakM(), 2 * BASE_M);
        collect(r, f, cap, true, samples(ends.routeStartM()), samples(ends.routeEndM()), rule.armM(), out);
        collect(f, r, cap, false, samples(ends.savedStartM()), samples(ends.savedEndM()), rule.armM(), out);
        return out;
    }

    /**
     * Length [m] of {@code line} farther than {@code tolM} from {@code other} (2 m samples) — the
     * quality measure: how much of the saved route a route leaves.
     */
    static double farLength(List<double[]> line, List<double[]> other, double tolM) {
        if (line == null || line.size() < 2) return 0;
        if (other == null || other.size() < 2) return PathSimilarity.length(line);
        double lat0 = line.get(0)[0], mLon = M_PER_DEG_LAT * Math.cos(Math.toRadians(lat0));
        double[][] a = project(PathSimilarity.resample(line), lat0, mLon);
        Hash h = new Hash(project(PathSimilarity.resample(other), lat0, mLon), tolM);
        double far = 0;
        for (double[] p : a) if (h.nearest(p, tolM) >= tolM) far += STEP_M;
        return far;
    }

    private static int samples(double m) {
        return (int) Math.ceil(m / STEP_M);
    }

    private static void collect(double[][] line, double[][] other, double cap, boolean onRoute,
                                int skipA, int skipB, double armM, List<Stretch> out) {
        Hash h = new Hash(other, cap);
        int n = line.length, from = -1;
        double peak = 0;
        for (int i = 0; i <= n; i++) {
            boolean judged = i < n && i >= skipA && i < n - skipB;
            double d = judged ? h.nearest(line[i], cap) : 0;
            if (judged && d > BASE_M) {
                if (from < 0) {
                    from = i;
                    peak = 0;
                }
                peak = Math.max(peak, d);
            } else if (from >= 0 && ((skipA > 0 && from == skipA && i * STEP_M <= armM)
                    || (skipB > 0 && i == n - skipB && (n - from) * STEP_M <= armM))) {
                from = -1; // an arm at a fixed waypoint
            } else if (from >= 0) {
                // Counterpart: between the other line's points nearest to where this line leaves
                // (the last close sample before) and rejoins (the first close sample after).
                int a = h.nearestIndex(line[Math.max(0, from - 1)], cap), b = h.nearestIndex(line[Math.min(n - 1, i)], cap);
                double counterpart = a < 0 || b < 0 ? Double.POSITIVE_INFINITY : Math.abs(b - a) * STEP_M;
                out.add(new Stretch(onRoute, from * STEP_M, i * STEP_M, peak, counterpart));
                from = -1;
            }
        }
    }

    private static double[][] project(List<double[]> pts, double lat0, double mLon) {
        double[][] p = new double[pts.size()][];
        for (int i = 0; i < p.length; i++) {
            p[i] = new double[]{(pts.get(i)[1]) * mLon, (pts.get(i)[0] - lat0) * M_PER_DEG_LAT};
        }
        return p;
    }

    /** Uniform grid over the other line's samples; nearest sample within a capped radius. */
    private static final class Hash {
        private final Map<Long, List<double[]>> cells = new HashMap<>();
        private final double size;

        Hash(double[][] pts, double size) {
            this.size = size;
            for (int i = 0; i < pts.length; i++) {
                double[] p = pts[i];
                cells.computeIfAbsent(key((long) Math.floor(p[0] / size), (long) Math.floor(p[1] / size)), k -> new ArrayList<>())
                        .add(new double[]{p[0], p[1], i});
            }
        }

        /** Index of the nearest sample within the cap, -1 if none. */
        int nearestIndex(double[] p, double cap) {
            long cx = (long) Math.floor(p[0] / size), cy = (long) Math.floor(p[1] / size);
            double best = cap * cap;
            int bi = -1;
            for (long x = cx - 1; x <= cx + 1; x++) {
                for (long y = cy - 1; y <= cy + 1; y++) {
                    List<double[]> c = cells.get(key(x, y));
                    if (c == null) continue;
                    for (double[] q : c) {
                        double dx = q[0] - p[0], dy = q[1] - p[1], d2 = dx * dx + dy * dy;
                        if (d2 < best) {
                            best = d2;
                            bi = (int) q[2];
                        }
                    }
                }
            }
            return bi;
        }

        double nearest(double[] p, double cap) {
            long cx = (long) Math.floor(p[0] / size), cy = (long) Math.floor(p[1] / size);
            double best = cap * cap;
            for (long x = cx - 1; x <= cx + 1; x++) {
                for (long y = cy - 1; y <= cy + 1; y++) {
                    List<double[]> c = cells.get(key(x, y));
                    if (c == null) continue;
                    for (double[] q : c) {
                        double dx = q[0] - p[0], dy = q[1] - p[1], d2 = dx * dx + dy * dy;
                        if (d2 < best) best = d2;
                    }
                }
            }
            return Math.sqrt(best);
        }

        private static long key(long x, long y) {
            return (x << 32) ^ (y & 0xffffffffL);
        }
    }
}
