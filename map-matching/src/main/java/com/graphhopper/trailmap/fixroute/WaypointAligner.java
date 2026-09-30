package com.graphhopper.trailmap.fixroute;

import com.graphhopper.util.DistanceCalcEarth;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

/**
 * Step 1 of {@code /fix_route}: place every waypoint on the saved reference track, cutting the
 * reference into one slice per leg (design doc §3.1).
 *
 * <p>One <b>global, order-preserving</b> assignment (dynamic programming) over per-waypoint
 * candidate positions. A candidate is a local minimum of the waypoint-to-reference distance within
 * {@code maxOffsetM} — one per pass of the reference near the waypoint, so a route that comes back
 * near itself offers one candidate per pass and the order constraint (plus optional saved leg
 * lengths) picks the right one. This replaces the client's forward/windowed anchoring, which can
 * lock onto the wrong pass with a confident residual and skip everything after it.
 *
 * <p>Cost of an assignment = Σ waypoint offsets [m] + {@code lengthWeight} × Σ |slice arc length −
 * saved leg length| [m]. With {@code lengthWeight = 0} the assignment is purely geometric; the saved
 * lengths then serve only as an independent consistency check.
 *
 * <p>First and last waypoints are pinned to the reference ends (their offsets are still reported).
 * An interior waypoint with no candidate within {@code maxOffsetM}, or one that cannot be placed
 * in order, is returned unaligned ({@code arcM = NaN}).
 *
 * <p>Pure geometry: no graph, no routing.
 */
public class WaypointAligner {

    private static final DistanceCalcEarth DIST = DistanceCalcEarth.DIST_EARTH;
    private static final double M_PER_DEG_LAT = 111_320.0;

    /** Candidates closer than this along the reference collapse into one. */
    private static final double CANDIDATE_MERGE_M = 1.0;
    /** Upper bound on candidates kept per waypoint (best offsets first). */
    private static final int MAX_CANDIDATES = 24;
    /** Cost [m-equivalent] of leaving one interior waypoint unaligned. */
    private static final double SKIP_PENALTY_M = 500.0;
    /** How many consecutive unaligned waypoints the DP normally bridges. */
    private static final int MAX_SKIP = 3;
    /** A waypoint may sit this far BEFORE its predecessor along the reference; it is clamped. */
    private static final double ORDER_SLACK_M = 30.0;

    private final double maxOffsetM;
    private final double lengthWeight;

    public WaypointAligner(double maxOffsetM, double lengthWeight) {
        this.maxOffsetM = maxOffsetM;
        this.lengthWeight = lengthWeight;
    }

    /** Where one waypoint landed on the reference. {@code arcM} is NaN when unaligned. */
    public record Placement(double arcM, double offsetM, int candidateCount) {
        public boolean aligned() {
            return !Double.isNaN(arcM);
        }
    }

    public record Result(List<Placement> placements, double referenceLengthM, double[] cumulativeM) {
    }

    private record Candidate(double arcM, double offsetM) {
    }

    /**
     * @param reference      reference track, {@code [lat, lng]} per vertex, at least 2 vertices
     * @param waypoints      waypoints in route order, {@code [lat, lng]}, at least 2
     * @param savedLegLength saved length [m] of leg k (waypoint k → k+1), or null / NaN entries when
     *                       unknown; ignored when {@code lengthWeight == 0}
     */
    public Result align(List<double[]> reference, List<double[]> waypoints, double[] savedLegLength) {
        int n = reference.size();
        int m = waypoints.size();
        if (n < 2) throw new IllegalArgumentException("reference must have at least 2 points");
        if (m < 2) throw new IllegalArgumentException("need at least 2 waypoints");

        double[] cum = cumulative(reference);
        double total = cum[n - 1];

        // Candidates per waypoint. Ends are pinned.
        List<List<Candidate>> cands = new ArrayList<>(m);
        for (int k = 0; k < m; k++) {
            double[] w = waypoints.get(k);
            if (k == 0) {
                cands.add(List.of(new Candidate(0, dist(w, reference.get(0)))));
            } else if (k == m - 1) {
                cands.add(List.of(new Candidate(total, dist(w, reference.get(n - 1)))));
            } else {
                cands.add(candidates(reference, cum, w));
            }
        }

        // Skip-aware DP. best[k][c] = min cost of an assignment of waypoints 0..k that places k at
        // candidate c, where any waypoints between k and its predecessor layer j are left
        // unaligned at SKIP_PENALTY_M each. Making a skip cost something (rather than being free)
        // lets the DP prefer a predecessor position that keeps the next waypoint placeable.
        double[][] best = new double[m][];
        int[][] fromLayer = new int[m][];
        int[][] fromCand = new int[m][];
        best[0] = new double[]{cands.get(0).get(0).offsetM()};
        fromLayer[0] = new int[]{-1};
        fromCand[0] = new int[]{-1};
        for (int k = 1; k < m; k++) {
            List<Candidate> ck = cands.get(k);
            int size = ck.size();
            best[k] = new double[size];
            fromLayer[k] = new int[size];
            fromCand[k] = new int[size];
            Arrays.fill(best[k], Double.POSITIVE_INFINITY);
            Arrays.fill(fromLayer[k], -1);
            if (size == 0) continue;
            // Normally only a short skip span is searched; if nothing is reachable within it
            // (rare), widen to all earlier layers so the pinned last waypoint is always placed.
            boolean found = relax(k, Math.max(0, k - 1 - MAX_SKIP), cands, best, fromLayer, fromCand, savedLegLength);
            if (!found) relax(k, 0, cands, best, fromLayer, fromCand, savedLegLength);
        }

        int[] chosen = new int[m];
        Arrays.fill(chosen, -1);
        int k = m - 1;
        int c = argMin(best[k]);
        if (Double.isInfinite(best[k][c])) throw new IllegalStateException("last waypoint not placed");
        while (k >= 0) {
            chosen[k] = c;
            int pl = fromLayer[k][c];
            int pc = fromCand[k][c];
            if (pl < 0) break;
            k = pl;
            c = pc;
        }

        List<Placement> out = new ArrayList<>(m);
        double runningArc = 0; // clamp order-slack placements so slices never go negative
        for (int i = 0; i < m; i++) {
            List<Candidate> ci = cands.get(i);
            if (chosen[i] >= 0) {
                Candidate cc = ci.get(chosen[i]);
                runningArc = Math.max(runningArc, cc.arcM());
                out.add(new Placement(runningArc, cc.offsetM(), ci.size()));
            } else {
                out.add(new Placement(Double.NaN, Double.NaN, ci.size()));
            }
        }
        return new Result(out, total, cum);
    }

    /**
     * Relax layer {@code k} from every earlier layer {@code j ∈ [jMin, k-1]} (layers strictly
     * between j and k are skipped). Returns whether any candidate of k became reachable.
     */
    private boolean relax(int k, int jMin, List<List<Candidate>> cands, double[][] best,
                          int[][] fromLayer, int[][] fromCand, double[] saved) {
        List<Candidate> ck = cands.get(k);
        boolean found = false;
        for (int j = k - 1; j >= jMin; j--) {
            List<Candidate> cj = cands.get(j);
            if (cj.isEmpty()) continue;
            double skipCost = SKIP_PENALTY_M * (k - j - 1);
            double expected = expectedLength(saved, j, k);
            for (int p = 0; p < cj.size(); p++) {
                if (Double.isInfinite(best[j][p])) continue;
                Candidate from = cj.get(p);
                for (int c = 0; c < ck.size(); c++) {
                    Candidate to = ck.get(c);
                    // Order-preserving, with a small slack for waypoints whose order along the
                    // reference is locally inconsistent by a few metres (coincident waypoints,
                    // spur bases); the placement is clamped to the predecessor below.
                    if (to.arcM() < from.arcM() - ORDER_SLACK_M) continue;
                    double slice = Math.max(0, to.arcM() - from.arcM());
                    double cost = best[j][p] + skipCost + to.offsetM();
                    if (lengthWeight > 0 && !Double.isNaN(expected)) {
                        cost += lengthWeight * Math.abs(slice - expected);
                    }
                    if (cost < best[k][c]) {
                        best[k][c] = cost;
                        fromLayer[k][c] = j;
                        fromCand[k][c] = p;
                        found = true;
                    }
                }
            }
        }
        return found;
    }

    /** Local minima of the waypoint-to-reference distance within {@link #maxOffsetM}. */
    private List<Candidate> candidates(List<double[]> ref, double[] cum, double[] w) {
        int segs = ref.size() - 1;
        double[] d = new double[segs];
        double[] s = new double[segs];
        double[] t = new double[segs];
        double wLat = w[0], wLon = w[1];
        double latPad = maxOffsetM / M_PER_DEG_LAT;
        double lonPad = maxOffsetM / (M_PER_DEG_LAT * Math.max(0.01, Math.cos(Math.toRadians(wLat))));
        for (int i = 0; i < segs; i++) {
            double[] a = ref.get(i), b = ref.get(i + 1);
            // Cheap bbox reject before projecting.
            if (Math.min(a[0], b[0]) - latPad > wLat || Math.max(a[0], b[0]) + latPad < wLat
                    || Math.min(a[1], b[1]) - lonPad > wLon || Math.max(a[1], b[1]) + lonPad < wLon) {
                d[i] = Double.POSITIVE_INFINITY;
                continue;
            }
            double[] proj = project(wLat, wLon, a, b);
            d[i] = proj[0];
            t[i] = proj[1];
            s[i] = cum[i] + proj[1] * (cum[i + 1] - cum[i]);
        }
        // Local minima of the CONTINUOUS distance-along-the-reference function. A segment's
        // closest point is a local minimum when it lies strictly inside the segment, or when it is
        // a shared vertex that the neighbouring segment also clamps to. Comparing per-segment
        // minimum values instead would miss a spur: the base of a short out-and-back is equally
        // close on the outbound and the return segment, with a maximum at the apex between them.
        List<Candidate> out = new ArrayList<>();
        for (int i = 0; i < segs; i++) {
            if (d[i] > maxOffsetM) continue;
            boolean interior = t[i] > 0 && t[i] < 1;
            boolean atStart = t[i] <= 0 && (i == 0 || (Double.isFinite(d[i - 1]) && t[i - 1] >= 1));
            boolean atEnd = t[i] >= 1 && (i == segs - 1 || (Double.isFinite(d[i + 1]) && t[i + 1] <= 0));
            if (interior || atStart || atEnd) {
                out.add(new Candidate(s[i], d[i]));
            }
        }
        // Collapse near-duplicates along the reference (e.g. a projection landing on a shared vertex).
        List<Candidate> merged = new ArrayList<>();
        for (Candidate cand : out) {
            if (!merged.isEmpty() && cand.arcM() - merged.get(merged.size() - 1).arcM() < CANDIDATE_MERGE_M) {
                if (cand.offsetM() < merged.get(merged.size() - 1).offsetM()) {
                    merged.set(merged.size() - 1, cand);
                }
                continue;
            }
            merged.add(cand);
        }
        if (merged.size() > MAX_CANDIDATES) {
            merged.sort(Comparator.comparingDouble(Candidate::offsetM));
            merged = new ArrayList<>(merged.subList(0, MAX_CANDIDATES));
            merged.sort(Comparator.comparingDouble(Candidate::arcM));
        }
        return merged;
    }

    /** Sum of saved leg lengths over legs [from, to), or NaN if any is unknown. */
    private static double expectedLength(double[] saved, int fromWp, int toWp) {
        if (saved == null) return Double.NaN;
        double sum = 0;
        for (int leg = fromWp; leg < toWp; leg++) {
            if (leg >= saved.length || Double.isNaN(saved[leg])) return Double.NaN;
            sum += saved[leg];
        }
        return sum;
    }

    private static int argMin(double[] xs) {
        int best = 0;
        for (int i = 1; i < xs.length; i++) if (xs[i] < xs[best]) best = i;
        return best;
    }

    /** Cumulative haversine arc length along the reference [m]. */
    public static double[] cumulative(List<double[]> ref) {
        double[] cum = new double[ref.size()];
        for (int i = 1; i < ref.size(); i++) {
            cum[i] = cum[i - 1] + dist(ref.get(i - 1), ref.get(i));
        }
        return cum;
    }

    private static double dist(double[] a, double[] b) {
        return DIST.calcDist(a[0], a[1], b[0], b[1]);
    }

    /** Perpendicular distance [m] from a point to segment a→b and the clamped fraction t ∈ [0,1]. */
    static double[] project(double pLat, double pLon, double[] a, double[] b) {
        double mPerDegLon = M_PER_DEG_LAT * Math.cos(Math.toRadians((a[0] + b[0]) * 0.5));
        double ax = a[1] * mPerDegLon, ay = a[0] * M_PER_DEG_LAT;
        double bx = b[1] * mPerDegLon, by = b[0] * M_PER_DEG_LAT;
        double px = pLon * mPerDegLon, py = pLat * M_PER_DEG_LAT;
        double dx = bx - ax, dy = by - ay;
        double len2 = dx * dx + dy * dy;
        double t = len2 == 0 ? 0 : ((px - ax) * dx + (py - ay) * dy) / len2;
        if (t < 0) t = 0;
        else if (t > 1) t = 1;
        double ex = px - (ax + t * dx), ey = py - (ay + t * dy);
        return new double[]{Math.sqrt(ex * ex + ey * ey), t};
    }
}
