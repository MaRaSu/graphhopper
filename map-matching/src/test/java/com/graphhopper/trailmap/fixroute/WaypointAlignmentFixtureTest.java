package com.graphhopper.trailmap.fixroute;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.util.List;

/**
 * Waypoint alignment (step 1) over the real-route fixture corpus. Pure geometry — no graph.
 *
 * <p>Independent check: a leg is <i>length-inconsistent</i> when the slice the aligner cut for it
 * differs from the leg's saved length by more than {@link #MISMATCH_ABS_M} and
 * {@link #MISMATCH_REL}. The saved lengths come from the client independently of the reference
 * geometry, so under the purely geometric assignment ({@code lengthWeight = 0}) they are a genuine
 * cross-check of the cut, not a restatement of it.
 */
public class WaypointAlignmentFixtureTest {

    static final double MAX_OFFSET_M = 30.0;
    static final double MISMATCH_ABS_M = 50.0;
    static final double MISMATCH_REL = 0.10;

    record Stats(int legs, int unaligned, int inconsistent, double worstMismatchM, int worstLeg,
                 int multiCandidate, double refKm, double savedKm) {
    }

    static Stats evaluate(FixRouteFixtures.Route r, double lengthWeight) {
        WaypointAligner aligner = new WaypointAligner(MAX_OFFSET_M, lengthWeight);
        WaypointAligner.Result res = aligner.align(r.reference, r.waypoints, r.savedSegmentLengths);
        List<WaypointAligner.Placement> p = res.placements();
        int legs = r.waypoints.size() - 1;
        int unaligned = 0, inconsistent = 0, multi = 0, worstLeg = -1;
        double worst = 0, saved = 0;
        for (WaypointAligner.Placement pl : p) {
            if (!pl.aligned()) unaligned++;
            if (pl.candidateCount() > 1) multi++;
        }
        for (int k = 0; k < legs; k++) {
            double sl = r.savedSegmentLengths != null && k < r.savedSegmentLengths.length
                    ? r.savedSegmentLengths[k] : Double.NaN;
            if (!Double.isNaN(sl)) saved += sl;
            WaypointAligner.Placement a = p.get(k), b = p.get(k + 1);
            if (!a.aligned() || !b.aligned() || Double.isNaN(sl)) continue;
            double mis = Math.abs((b.arcM() - a.arcM()) - sl);
            if (mis > MISMATCH_ABS_M && mis > MISMATCH_REL * sl) inconsistent++;
            if (mis > worst) {
                worst = mis;
                worstLeg = k;
            }
        }
        return new Stats(legs, unaligned, inconsistent, worst, worstLeg, multi,
                res.referenceLengthM() / 1000, saved / 1000);
    }

    /**
     * Routes whose fixture data is itself inconsistent, excluded from the standing assertion:
     * 116370 — saved track lacks the out-and-back spurs to ~313 waypoints (track 2098 km vs saved
     * leg lengths 2427 km); 17232 — two saved leg lengths contradict the waypoint geometry (a leg
     * saved as 86 m between waypoints 221 m apart).
     */
    static final List<Long> KNOWN_DATA_ISSUES = List.of(116370L, 17232L);

    /**
     * Standing regression assertion for step 1: with the purely geometric assignment, every
     * waypoint of every corpus route is placed and every leg's slice agrees with its independently
     * saved length — in particular on the routes where the client's windowed anchoring fails.
     */
    @Test
    void everyWaypointPlacedAndEverySliceConsistent() throws Exception {
        Assumptions.assumeTrue(FixRouteFixtures.available(), "fixtures not found");
        StringBuilder failures = new StringBuilder();
        for (long id : FixRouteFixtures.allIds()) {
            if (KNOWN_DATA_ISSUES.contains(id)) continue;
            Stats s = evaluate(FixRouteFixtures.load(id), 0.0);
            if (s.unaligned > 0 || s.inconsistent > 0) {
                failures.append(String.format("route-%d: unaligned=%d inconsistent=%d worst=%.0fm(leg %d)%n",
                        id, s.unaligned, s.inconsistent, s.worstMismatchM, s.worstLeg));
            }
        }
        if (failures.length() > 0) throw new AssertionError("alignment regressions:\n" + failures);
    }

    /** Per-waypoint dump around unaligned / length-inconsistent spots. {@code -Dfix.routes=id,id}. */
    @Test
    void detail() throws Exception {
        Assumptions.assumeTrue(FixRouteFixtures.available(), "fixtures not found");
        String prop = System.getProperty("fix.routes");
        Assumptions.assumeTrue(prop != null && !prop.isBlank(), "set -Dfix.routes");
        for (String s : prop.split(",")) {
            FixRouteFixtures.Route r = FixRouteFixtures.load(Long.parseLong(s.trim()));
            WaypointAligner.Result res = new WaypointAligner(MAX_OFFSET_M, 0).align(
                    r.reference, r.waypoints, r.savedSegmentLengths);
            List<WaypointAligner.Placement> p = res.placements();
            System.out.println("== " + r + " refKm=" + FixRouteFixtures.fmt(res.referenceLengthM() / 1000));
            for (int k = 0; k < p.size(); k++) {
                WaypointAligner.Placement pl = p.get(k);
                double sl = k < r.savedSegmentLengths.length ? r.savedSegmentLengths[k] : Double.NaN;
                double slice = (k + 1 < p.size() && pl.aligned() && p.get(k + 1).aligned())
                        ? p.get(k + 1).arcM() - pl.arcM() : Double.NaN;
                boolean bad = !pl.aligned() || (!Double.isNaN(slice) && Math.abs(slice - sl) > MISMATCH_ABS_M
                        && Math.abs(slice - sl) > MISMATCH_REL * sl);
                boolean near = bad || (k > 0 && !p.get(k - 1).aligned()) || (k + 1 < p.size() && !p.get(k + 1).aligned());
                if (!near && !Boolean.getBoolean("fix.all")) continue;
                String type = k < r.segments.size() ? r.segments.get(k).type : "-";
                System.out.printf("  wp%-4d arc=%9s off=%6s cands=%2d | leg %s saved=%8s slice=%8s%s%n",
                        k, FixRouteFixtures.fmt(pl.arcM()), FixRouteFixtures.fmt(pl.offsetM()), pl.candidateCount(),
                        type, FixRouteFixtures.fmt(sl), FixRouteFixtures.fmt(slice), bad ? "  <<" : "");
            }
        }
    }

    @Test
    void corpusReport() throws Exception {
        Assumptions.assumeTrue(FixRouteFixtures.available(), "fixtures not found");
        double[] weights = {0.0, 0.5};
        System.out.println("route      mode              legs  refKm savedKm | w=0: unal incons worst(leg) multi | w=0.5: unal incons worst(leg)");
        int[] totIncons = new int[weights.length];
        int[] totUnal = new int[weights.length];
        long t0 = System.nanoTime();
        for (long id : FixRouteFixtures.allIds()) {
            FixRouteFixtures.Route r = FixRouteFixtures.load(id);
            Stats a = evaluate(r, weights[0]);
            Stats b = evaluate(r, weights[1]);
            totIncons[0] += a.inconsistent;
            totIncons[1] += b.inconsistent;
            totUnal[0] += a.unaligned;
            totUnal[1] += b.unaligned;
            String flag = FixRouteFixtures.ALIGNMENT_CASES.contains(id) ? " *" : "";
            System.out.printf("%-10d %-17s %4d %6.1f %7.1f | %4d %6d %7.0f(%d) %5d | %4d %6d %7.0f(%d)%s%n",
                    id, r.mode, a.legs, a.refKm, a.savedKm,
                    a.unaligned, a.inconsistent, a.worstMismatchM, a.worstLeg, a.multiCandidate,
                    b.unaligned, b.inconsistent, b.worstMismatchM, b.worstLeg, flag);
        }
        System.out.printf("TOTAL  w=0: unaligned=%d inconsistent=%d | w=0.5: unaligned=%d inconsistent=%d  (%.0f ms)%n",
                totUnal[0], totIncons[0], totUnal[1], totIncons[1], (System.nanoTime() - t0) / 1e6);
    }
}
