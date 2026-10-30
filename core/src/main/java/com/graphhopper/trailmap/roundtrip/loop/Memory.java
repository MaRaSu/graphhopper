package com.graphhopper.trailmap.roundtrip.loop;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Round 4 (m4): the ways a user has already been shown in this session, as z19 Web-Mercator cells (~37 m at 61 deg N).
 * The same kind of input as personal_routing's ridden cells (z20), so a known user's last rides can later fill it
 * instead. Cells within {@code freeM} of the start are never remembered: every route leaves home on the same ways.
 * <ul>
 *   <li>{@link #edgeMult}: the loop searches' cost factor per edge, 1 + strength x (share of the edge on remembered
 *       cells), so a remembered way costs more but is never forbidden (like PersonalWeighting's avoid).</li>
 *   <li>{@link #newShare}: the share of a route on cells not remembered, for the pick.</li>
 * </ul>
 */
public final class Memory {

    static final int Z = 19;
    static final double STEP_M = 10;

    final Set<Long> cells = new HashSet<>();
    // the search's cells: the same at Z, or coarser (searchZ < Z) so a whole area costs more, not single ways
    final int searchZ;
    final Set<Long> searchCells;
    final double lat0, lon0, freeM;
    private final double mPerLat, mPerLon;

    public Memory(double lat0, double lon0, double freeM) {
        this(lat0, lon0, freeM, Z);
    }

    public Memory(double lat0, double lon0, double freeM, int searchZ) {
        this.lat0 = lat0;
        this.lon0 = lon0;
        this.freeM = freeM;
        this.searchZ = searchZ;
        this.searchCells = searchZ == Z ? cells : new HashSet<>();
        this.mPerLat = 110_540;
        this.mPerLon = 111_320 * Math.cos(Math.toRadians(lat0));
    }

    static long cell(double lat, double lon) {
        return cell(lat, lon, Z);
    }

    static long cell(double lat, double lon, int z) {
        double n = 1L << z;
        long x = (long) Math.floor((lon + 180) / 360 * n);
        double r = Math.toRadians(lat);
        long y = (long) Math.floor((1 - Math.log(Math.tan(r) + 1 / Math.cos(r)) / Math.PI) / 2 * n);
        return x * (1L << z) + y;
    }

    boolean free(double lat, double lon) {
        double dx = (lon - lon0) * mPerLon, dy = (lat - lat0) * mPerLat;
        return dx * dx + dy * dy < freeM * freeM;
    }

    boolean isEmpty() {
        return cells.isEmpty();
    }

    /** Remembers a shown route, coords as [lat, lon]. */
    void add(List<double[]> coords) {
        walk(coords, (lat, lon, len) -> {
            if (free(lat, lon)) return;
            cells.add(cell(lat, lon));
            if (searchCells != cells) searchCells.add(cell(lat, lon, searchZ));
        });
    }

    /** Share of the route's length (outside the free zone) on cells not remembered; 1 with an empty memory. */
    double newShare(List<double[]> coords) {
        double[] acc = new double[2];
        walk(coords, (lat, lon, len) -> {
            if (free(lat, lon)) return;
            acc[0] += len;
            if (!cells.contains(cell(lat, lon))) acc[1] += len;
        });
        return acc[0] > 0 ? acc[1] / acc[0] : 1;
    }

    /**
     * Per-edge cost factor for a local graph whose edge e has geometry points gStart[e] .. gStart[e+1]-1 in a local
     * metric frame; f maps that frame back to lat / lon. Null when nothing is remembered or strength is 0.
     */
    double[] edgeMult(int m, int[] gStart, double[] gx, double[] gy, Frame f, double strength) {
        if (cells.isEmpty() || strength <= 0) return null;
        double[] mult = new double[m];
        for (int e = 0; e < m; e++) {
            double total = 0, hit = 0;
            for (int i = gStart[e]; i < gStart[e + 1] - 1; i++) {
                double seg = Math.hypot(gx[i + 1] - gx[i], gy[i + 1] - gy[i]);
                if (seg <= 0) continue;
                int k = Math.max(1, (int) Math.ceil(seg / STEP_M));
                for (int s = 0; s < k; s++) {
                    double t = (s + 0.5) / k;
                    double lat = f.lat(gy[i] + (gy[i + 1] - gy[i]) * t), lon = f.lon(gx[i] + (gx[i + 1] - gx[i]) * t);
                    if (free(lat, lon)) continue;
                    total += seg / k;
                    if (searchCells.contains(cell(lat, lon, searchZ))) hit += seg / k;
                }
            }
            mult[e] = 1 + strength * (total > 0 ? hit / total : 0);
        }
        return mult;
    }

    interface Frame {
        double lat(double y);

        double lon(double x);
    }

    private interface Step {
        void at(double lat, double lon, double len);
    }

    private void walk(List<double[]> coords, Step step) {
        for (int i = 1; i < coords.size(); i++) {
            double[] a = coords.get(i - 1), b = coords.get(i);
            double seg = Math.hypot((b[0] - a[0]) * mPerLat, (b[1] - a[1]) * mPerLon);
            if (seg <= 0) continue;
            int k = Math.max(1, (int) Math.ceil(seg / STEP_M));
            for (int s = 0; s < k; s++) {
                double t = (s + 0.5) / k;
                step.at(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, seg / k);
            }
        }
    }
}
