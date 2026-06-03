package com.graphhopper.trailmap.matching;

import com.graphhopper.matching.Observation;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.shapes.GHPoint;

import java.util.ArrayList;
import java.util.List;

/**
 * Phase 5 (P1): GPX gap-filling / densification.
 *
 * <p>Trailmap's convert_track inputs are frequently NOT GPS recordings but route-engine /
 * hand-drawn polylines that have passed through a Douglas-Peucker simplification
 * (simplify.js). On straight runs that collapses many metres into a single chord between
 * two distant points. Those gaps hurt two stages:
 * <ol>
 *   <li><b>Matching</b> — long sparse legs leave the HMM with almost no emission anchoring
 *       in the middle, so a parallel connector whose length ≈ the chord can win
 *       (learnings doc §5).</li>
 *   <li><b>The downstream {@code /route} fit</b> — too-coarse snapped points let {@code /route}
 *       pick a different way between them, turning a routable segment into a coordinates
 *       segment unnecessarily.</li>
 * </ol>
 *
 * <p>This utility inserts linearly-interpolated observations so that no two consecutive
 * observations are farther apart than {@code maxGapM}. All original points are preserved in
 * order; only intermediate points are added. The result is intended to be used uniformly by
 * the matcher AND the converter (same list passed to both) so observation indices stay 1:1
 * across the pipeline.
 *
 * <p>Interpolation is planar in lat/lon, which is accurate to well under a metre at the gap
 * scales involved (tens to a few hundred metres) — adequate for seeding candidate snaps.
 */
public final class ObservationDensifier {

    private static final DistanceCalcEarth DIST = DistanceCalcEarth.DIST_EARTH;

    /**
     * Consecutive input points closer than this [m] are treated as coincident DUPLICATES and the
     * later one is dropped. Hand-drawn / route-engine GPX frequently contains exact-duplicate
     * vertices (gap = 0 m). A duplicate is filtered out of the matcher's Viterbi (2σ pre-filter)
     * but still rides along in the observation list, where the downstream converter/optimizer see
     * it as a degenerate, zero-length waypoint with an ambiguous snap — which can force a spurious
     * coordinates escalation at that spot. Removing duplicates here (the single input-conditioning
     * step applied before BOTH match and convert) eliminates them from the whole pipeline at once
     * while keeping observation indices 1:1 across stages. Sub-metre spacing carries no real GPS
     * information, so 1 m is safe.
     */
    public static final double COINCIDENT_EPS_M = 1.0;

    private ObservationDensifier() {
    }

    /**
     * Returns a densified copy of {@code observations} in which every consecutive pair is at
     * most {@code maxGapM} apart. Original observations are preserved in order; interpolated
     * observations are inserted between pairs that exceed the gap.
     *
     * @param observations input observations (not mutated)
     * @param maxGapM       maximum allowed gap [m]; must be &gt; 0
     * @return a new list (size &ge; input size)
     */
    public static List<Observation> densify(List<Observation> observations, double maxGapM) {
        if (maxGapM <= 0) {
            throw new IllegalArgumentException("maxGapM must be > 0, got " + maxGapM);
        }
        if (observations == null || observations.size() < 2) {
            return observations == null ? null : new ArrayList<>(observations);
        }

        List<Observation> out = new ArrayList<>(observations.size());
        out.add(observations.get(0));
        // Anchor for gap/interpolation is the last KEPT point, so dropping a duplicate doesn't
        // distort the gap to the following point.
        GHPoint lastKept = observations.get(0).getPoint();

        for (int i = 1; i < observations.size(); i++) {
            GHPoint b = observations.get(i).getPoint();
            double gap = DIST.calcDist(lastKept.lat, lastKept.lon, b.lat, b.lon);

            // Drop coincident duplicates (gap < COINCIDENT_EPS_M) — they add no information and
            // become degenerate zero-length waypoints downstream.
            if (gap < COINCIDENT_EPS_M) {
                continue;
            }

            if (gap > maxGapM) {
                // Number of sub-segments needed so each piece <= maxGapM.
                int pieces = (int) Math.ceil(gap / maxGapM);
                for (int k = 1; k < pieces; k++) {
                    double t = (double) k / pieces;
                    double lat = lastKept.lat + (b.lat - lastKept.lat) * t;
                    double lon = lastKept.lon + (b.lon - lastKept.lon) * t;
                    out.add(new Observation(new GHPoint(lat, lon)));
                }
            }
            out.add(observations.get(i));
            lastKept = b;
        }
        return out;
    }
}
