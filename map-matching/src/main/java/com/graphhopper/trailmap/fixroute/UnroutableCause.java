package com.graphhopper.trailmap.fixroute;

import com.graphhopper.GraphHopper;
import com.graphhopper.routing.ev.Subnetwork;
import com.graphhopper.routing.util.DefaultSnapFilter;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.util.CustomModel;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.PMap;

/**
 * Why a stretch of the reference cannot be reproduced (design doc §3.7), from what exists next to
 * it: {@code removed} (no way runs along it), {@code restricted} (a way runs along it but the leg's
 * profile / custom model cannot travel it in the reference's direction), {@code disconnected}
 * (usable ways exist but no connection follows the reference), else {@code unknown}. Shared by both
 * {@code /fix_route} engines.
 */
public class UnroutableCause {

    private static final double CAUSE_SAMPLE_M = 20.0;
    /** A way counts as running along the reference within this angle [deg]. */
    private static final double ALIGN_BEARING_DEG = 35.0;

    private final GraphHopper hopper;

    public UnroutableCause(GraphHopper hopper) {
        this.hopper = hopper;
    }

    public String cause(ReferenceTrack ref, double from, double to, ClientRoute.Settings s, boolean rerouted,
                        double offnetSnapM) {
        try {
            PMap hints = new PMap();
            if (s.customModel() != null) hints.putObject(CustomModel.KEY, s.customModel());
            Weighting w = hopper.createWeighting(hopper.getProfile(s.profile()), hints);
            EdgeFilter usable = new DefaultSnapFilter(w,
                    hopper.getEncodingManager().getBooleanEncodedValue(Subnetwork.key(s.profile())));
            int samples = 0, noRoad = 0, noUsable = 0;
            for (double arc = from + CAUSE_SAMPLE_M / 2; arc < to; arc += CAUSE_SAMPLE_M) {
                double[] p = ref.pointAt(arc);
                double refBearing = RouteFixerMath.bearing(ref.pointAt(Math.max(0, arc - 5)), ref.pointAt(Math.min(ref.lengthM(), arc + 5)));
                samples++;
                int verdict = alongReference(p, refBearing, w, usable, offnetSnapM);
                if (verdict == 0) noRoad++;
                else if (verdict == 1) noUsable++;
            }
            if (samples == 0) return "unknown";
            if (noRoad * 2 >= samples) return "removed";
            if ((noRoad + noUsable) * 2 >= samples) return "restricted";
            return rerouted ? "disconnected" : "unknown";
        } catch (Exception e) {
            return "unknown";
        }
    }

    /**
     * What runs ALONG the reference at {@code p}: 0 = no way at all (a crossing road nearby does not
     * count — owner review, route 32997, a missing way was labelled "restricted" because of one);
     * 1 = a way exists but the leg's profile cannot travel it in the reference's direction (access,
     * one-way); 2 = a usable way in that direction exists.
     */
    private int alongReference(double[] p, double refBearing, Weighting w, EdgeFilter usable, double radiusM) {
        double rLat = radiusM / 111_320.0, rLon = radiusM / (111_320.0 * Math.cos(Math.toRadians(p[0])));
        int[] best = {0};
        hopper.getLocationIndex().query(new com.graphhopper.util.shapes.BBox(p[1] - rLon, p[1] + rLon, p[0] - rLat, p[0] + rLat), edgeId -> {
            if (best[0] == 2) return;
            EdgeIteratorState e = hopper.getBaseGraph().getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            com.graphhopper.util.PointList g = e.fetchWayGeometry(com.graphhopper.util.FetchMode.ALL);
            for (int k = 0; k + 1 < g.size(); k++) {
                double[] a = {g.getLat(k), g.getLon(k)}, b = {g.getLat(k + 1), g.getLon(k + 1)};
                if (WaypointAligner.project(p[0], p[1], a, b)[0] > radiusM) continue;
                double segBearing = RouteFixerMath.bearing(a, b);
                double diff = Math.abs(((segBearing - refBearing) % 360 + 540) % 360 - 180); // 0 = same direction, 180 = opposite
                boolean sameDir = diff <= ALIGN_BEARING_DEG;
                boolean oppDir = diff >= 180 - ALIGN_BEARING_DEG;
                if (!sameDir && !oppDir) continue; // crosses the reference: not "the way"
                best[0] = Math.max(best[0], 1);
                // Along the reference in its direction of travel: forward on the edge when the
                // geometry runs the same way, else reverse.
                boolean reverse = oppDir;
                if (usable.accept(e) && Double.isFinite(w.calcEdgeWeight(e, reverse))) {
                    best[0] = 2;
                    return;
                }
            }
        });
        return best[0];
    }

}
