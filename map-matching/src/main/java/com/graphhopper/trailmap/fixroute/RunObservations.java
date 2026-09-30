package com.graphhopper.trailmap.fixroute;

import com.graphhopper.matching.Observation;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.shapes.GHPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

/**
 * The saved reference of one RUN (consecutive followRoads legs with the same routing settings) as
 * map-matcher observations — the input the {@code /convert_track} chain expects.
 *
 * <ul>
 *   <li>Points: the reference's own vertices plus a global 20 m arc grid (a clean routed line needs
 *       chord densification only), each with its arc position on the whole reference.</li>
 *   <li>Legacy OSRM spikes (road → off-road waypoint → road) are dropped first: they are not a path
 *       any road network can reproduce, and the matcher would turn each into an off-network region.</li>
 *   <li>Each fixed waypoint is an observation at its road snap, placed at its aligned arc position
 *       in the sequence ({@link #waypointObs}). It
 *       must survive the matcher's 2σ thinning, which keeps a point only if it is further than 2σ
 *       from the previously KEPT point: so every other observation within 2σ before a waypoint
 *       observation is removed. (First and last observations are always kept by the matcher.)</li>
 * </ul>
 */
final class RunObservations {

    private static final DistanceCalcEarth DIST = DistanceCalcEarth.DIST_EARTH;
    static final double GRID_M = 20.0;
    /** Experiment switch: arc grid step [m]; 0 = the client's /convert_track input instead (the
     *  track's own vertices, densified to a {@link #DENSIFY_MAX_GAP_M} max gap). */
    static double GRID_EXPERIMENT_M = Double.parseDouble(System.getProperty("fix.grid", "0"));
    /** The client's /convert_track {@code cm_densify_max_gap_m}. */
    static final double DENSIFY_MAX_GAP_M = 40.0;
    /** Experiment switch: clear the matcher's thinning distance in front of each waypoint obs. */
    static boolean CLEAR_EXPERIMENT = !"false".equals(System.getProperty("fix.clear"));

    /** Vertices plus evenly spaced points in every gap longer than {@code maxGapM} (ObservationDensifier's rule). */
    private static List<ReferenceTrack.ArcPoint> densified(List<ReferenceTrack.ArcPoint> v, double maxGapM) {
        List<ReferenceTrack.ArcPoint> out = new ArrayList<>();
        for (int i = 0; i < v.size(); i++) {
            if (i > 0) {
                ReferenceTrack.ArcPoint a = v.get(i - 1), b = v.get(i);
                double gap = d(a.point(), b.point());
                int n = (int) Math.ceil(gap / maxGapM);
                for (int k = 1; k < n; k++) {
                    double f = (double) k / n;
                    out.add(new ReferenceTrack.ArcPoint(a.arcM() + f * (b.arcM() - a.arcM()),
                            new double[]{a.point()[0] + f * (b.point()[0] - a.point()[0]), a.point()[1] + f * (b.point()[1] - a.point()[1])}, true));
                }
            }
            out.add(v.get(i));
        }
        return out;
    }

    final List<Observation> observations = new ArrayList<>();
    final List<Double> arcs = new ArrayList<>();
    /** Arc positions [m] of the reference points dropped as legacy spikes (not part of any road path). */
    final java.util.TreeSet<Double> spikeArcs = new java.util.TreeSet<>();
    /** Observation index of each waypoint of the run, in order (run's first … last waypoint). */
    final int[] waypointObs;

    /**
     * @param waypointPos where each waypoint is routed from: its road snap (what GH / the client
     *                    route starts at) — for a legacy off-road waypoint this is on the road at the
     *                    spike base, not at the spike tip the reference passes through.
     */
    RunObservations(ReferenceTrack ref, double[] waypointArcs, double[][] waypointPos, double sigmaM,
                    Predicate<double[]> offRoad, double contextBeforeM, double contextAfterM) {
        this.waypointObs = new int[waypointArcs.length];
        // Context beyond an artificial run cut: plain reference points (no forced waypoints), so the
        // match at the cut does not depend on where the cut fell.
        double from = Math.max(0, waypointArcs[0] - contextBeforeM);
        double to = Math.min(ref.lengthM(), waypointArcs[waypointArcs.length - 1] + contextAfterM);

        // 1. Reference points with arcs, legacy spikes removed.
        List<ReferenceTrack.ArcPoint> pts = GRID_EXPERIMENT_M > 0 ? ref.slicePointsWithGrid(from, to, GRID_EXPERIMENT_M)
                : densified(ref.slicePointsWithGrid(from, to, Double.MAX_VALUE), DENSIFY_MAX_GAP_M);
        List<double[]> line = new ArrayList<>(pts.size());
        for (ReferenceTrack.ArcPoint p : pts) line.add(p.point());
        boolean[] keep = LegacySpikes.keepMask(line, offRoad);
        for (int k = 0; k < keep.length; k++) if (!keep[k]) spikeArcs.add(pts.get(k).arcM());

        // 2. Merge waypoint observations in arc order; clear 2σ in front of each.
        double clear = CLEAR_EXPERIMENT ? 2 * sigmaM + 1 : 0;
        List<double[]> outPts = new ArrayList<>();
        List<Double> outArcs = new ArrayList<>();
        List<Integer> wpAt = new ArrayList<>();
        int w = 0;
        for (int k = 0; k <= pts.size(); k++) {
            double arc = k < pts.size() ? pts.get(k).arcM() : Double.POSITIVE_INFINITY;
            while (w < waypointArcs.length && waypointArcs[w] <= arc + 1e-6) {
                double[] wp = waypointPos[w] != null ? waypointPos[w] : ref.pointAt(waypointArcs[w]);
                while (!outPts.isEmpty() && !wpAt.contains(outPts.size() - 1)
                        && d(outPts.get(outPts.size() - 1), wp) <= clear) {
                    outPts.remove(outPts.size() - 1);
                    outArcs.remove(outArcs.size() - 1);
                }
                wpAt.add(outPts.size());
                outPts.add(wp);
                outArcs.add(waypointArcs[w]);
                w++;
            }
            if (k == pts.size()) break;
            if (!keep[k]) continue;
            double[] p = pts.get(k).point();
            // Skip points coinciding with the previous one (e.g. the waypoint just placed).
            if (!outPts.isEmpty() && d(outPts.get(outPts.size() - 1), p) < 1.0) continue;
            outPts.add(p);
            outArcs.add(pts.get(k).arcM());
        }
        for (int i = 0; i < wpAt.size(); i++) waypointObs[i] = wpAt.get(i);
        for (int i = 0; i < outPts.size(); i++) {
            observations.add(new Observation(new GHPoint(outPts.get(i)[0], outPts.get(i)[1])));
            arcs.add(outArcs.get(i));
        }
    }

    private static double d(double[] a, double[] b) {
        return DIST.calcDist(a[0], a[1], b[0], b[1]);
    }
}
