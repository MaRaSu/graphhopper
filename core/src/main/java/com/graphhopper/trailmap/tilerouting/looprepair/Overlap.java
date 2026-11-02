package com.graphhopper.trailmap.tilerouting.looprepair;

import java.util.*;

/**
 * Out-and-back (reverse-corridor) metres of a polyline: a Java port of the antiparallel part of gh-autoroute's
 * {@code selfOverlap} (route-metrics.mjs, params.mjs overlap block), which the verifier reports as "O&B km". Resample
 * at 10 m; a sample is antiparallel if a sample ≥ 300 m away along the route lies within 25 m with direction ≥ 135°
 * off; runs of one kind separated by ≤ 300 m are merged; runs ≥ 100 m count.
 */
final class Overlap {
    static final double STEP = 10, CORRIDOR = 25, MIN_ARC_GAP = 300, MIN_RUN = 100, MERGE_GAP = 300;
    static final int SPAN = 2;
    static final double COS_ANTI = Math.cos(Math.toRadians(135)), COS_PAR = Math.cos(Math.toRadians(45));
    private static final double R = 6371008.8, DEG = Math.PI / 180;

    static double outAndBackM(List<double[]> coords) {
        double lat0 = coords.get(0)[0], lon0 = coords.get(0)[1];
        double kx = Math.cos(lat0 * DEG) * R * DEG, ky = R * DEG;
        int m = coords.size();
        double[] px = new double[m], py = new double[m], cum = new double[m];
        for (int i = 0; i < m; i++) {
            px[i] = (coords.get(i)[1] - lon0) * kx;
            py[i] = (coords.get(i)[0] - lat0) * ky;
            if (i > 0) cum[i] = cum[i - 1] + Math.hypot(px[i] - px[i - 1], py[i] - py[i - 1]);
        }
        double length = cum[m - 1];
        int n = Math.max(1, (int) Math.floor(length / STEP) + 1);
        double[] x = new double[n], y = new double[n];
        int seg = 0;
        for (int k = 0; k < n; k++) {
            double target = k * STEP;
            while (seg < m - 2 && cum[seg + 1] < target) seg++;
            int nb = Math.min(seg + 1, m - 1);
            double segLen = cum[nb] - cum[seg];
            double t = segLen > 0 ? Math.min(1, Math.max(0, (target - cum[seg]) / segLen)) : 0;
            x[k] = px[seg] + (px[nb] - px[seg]) * t;
            y[k] = py[seg] + (py[nb] - py[seg]) * t;
        }
        Map<Long, List<Integer>> grid = new HashMap<>();
        for (int k = 0; k < n; k++) grid.computeIfAbsent(cell(x[k], y[k]), q -> new ArrayList<>()).add(k);
        double[] dx = new double[n], dy = new double[n];
        for (int k = 0; k < n; k++) {
            int a = Math.max(0, k - SPAN), b = Math.min(n - 1, k + SPAN);
            double ddx = x[b] - x[a], ddy = y[b] - y[a], l = Math.hypot(ddx, ddy);
            if (l > 0) {
                dx[k] = ddx / l;
                dy[k] = ddy / l;
            }
        }
        int minGap = (int) Math.ceil(MIN_ARC_GAP / STEP);
        double r2 = CORRIDOR * CORRIDOR;
        byte[] kind = new byte[n];
        for (int k = 0; k < n; k++) {
            boolean anti = false, par = false;
            long cx = (long) Math.floor(x[k] / CORRIDOR), cy = (long) Math.floor(y[k] / CORRIDOR);
            for (long ox = -1; ox <= 1 && !anti; ox++)
                for (long oy = -1; oy <= 1 && !anti; oy++) {
                    List<Integer> l = grid.get((cx + ox) * 1_000_003L + (cy + oy));
                    if (l == null) continue;
                    for (int j : l) {
                        if (Math.abs(j - k) < minGap) continue;
                        double ex = x[j] - x[k], ey = y[j] - y[k];
                        if (ex * ex + ey * ey > r2) continue;
                        double c = dx[k] * dx[j] + dy[k] * dy[j];
                        if (c <= COS_ANTI) {
                            anti = true;
                            break;
                        } else if (c >= COS_PAR) par = true;
                    }
                }
            kind[k] = (byte) (anti ? 1 : par ? 2 : 0);
        }
        int maxGap = (int) Math.floor(MERGE_GAP / STEP);
        for (int a = 0; a < n; ) {
            if (kind[a] == 0) {
                a++;
                continue;
            }
            int e = a;
            while (e + 1 < n && kind[e + 1] == kind[a]) e++;
            int g = e + 1;
            while (g < n && kind[g] == 0) g++;
            if (g < n && kind[g] == kind[a] && g - e - 1 <= maxGap) for (int q = e + 1; q < g; q++) kind[q] = kind[a];
            a = g;
        }
        double ob = 0;
        for (int k = 0; k < n; ) {
            if (kind[k] == 0) {
                k++;
                continue;
            }
            int e = k;
            while (e + 1 < n && kind[e + 1] == kind[k]) e++;
            double lenM = (e - k + 1) * STEP;
            if (lenM >= MIN_RUN && kind[k] == 1) ob += lenM;
            k = e + 1;
        }
        return ob;
    }

    private static long cell(double x, double y) {
        return (long) Math.floor(x / CORRIDOR) * 1_000_003L + (long) Math.floor(y / CORRIDOR);
    }
}
