/*
 * Trailmap - Personal routing, "bit of new" variations
 */
package com.graphhopper.trailmap.personal.variations;

import com.carrotsearch.hppc.IntArrayList;
import com.carrotsearch.hppc.LongObjectHashMap;

import java.util.ArrayList;
import java.util.List;

/**
 * Round-4 geometry judge: an exact Java port of the round-3 guard scorer ({@code personal-routing/r3/guard.mjs}),
 * so the round-4 generator judges candidates the way the guard does (projector at main[0], 5 m resampling).
 * Copied unchanged from the round-4 prototype ({@code R4Shared.Guard} and {@code R4Shared.PtIndex}).
 */
public final class R4Guard {
    static final double STEP = 5;
    final double kx, ky;
    public final double[][] mainP;
    public final double L;
    final PtIndex nearMain;
    final double[][] coarse;
    static final double CC = 100;
    final LongObjectHashMap<IntArrayList> coarseGrid = new LongObjectHashMap<>();

    public R4Guard(List<double[]> main) {
        kx = 111320 * Math.cos(main.get(0)[1] * Math.PI / 180);
        ky = 110540;
        mainP = resample(main);
        L = polyLen(mainP, 0, mainP.length - 1);
        nearMain = new PtIndex(mainP, 25);
        int nc = (mainP.length + 3) / 4;
        coarse = new double[nc][];
        for (int i = 0, c = 0; i < mainP.length; i += 4) coarse[c++] = mainP[i];
        for (int c = 0; c < nc; c++) {
            long k = PtIndex.key((long) Math.floor(coarse[c][0] / CC), (long) Math.floor(coarse[c][1] / CC));
            IntArrayList l = coarseGrid.get(k);
            if (l == null) coarseGrid.put(k, l = new IntArrayList());
            l.add(c);
        }
    }

    public double[][] resample(List<double[]> coords) {
        List<double[]> out = new ArrayList<>();
        for (int i = 0; i < coords.size() - 1; i++) {
            double ax = coords.get(i)[0] * kx, ay = coords.get(i)[1] * ky, bx = coords.get(i + 1)[0] * kx, by = coords.get(i + 1)[1] * ky;
            int n = (int) Math.max(1, Math.ceil(Math.hypot(bx - ax, by - ay) / STEP));
            for (int s = 0; s < n; s++) out.add(new double[]{ax + (bx - ax) * s / n, ay + (by - ay) * s / n});
        }
        if (!coords.isEmpty()) {
            double[] c = coords.get(coords.size() - 1);
            out.add(new double[]{c[0] * kx, c[1] * ky});
        }
        return out.toArray(new double[0][]);
    }

    static double polyLen(double[][] P, int s, int e) {
        double L = 0;
        for (int i = s + 1; i <= e; i++) L += Math.hypot(P[i][0] - P[i - 1][0], P[i][1] - P[i - 1][1]);
        return L;
    }

    /** distance of a projected point to main (the guard's nearest-sample rule). */
    public double distToMain(double[] p) {
        return farDist(p);
    }

    double farDist(double[] p) {
        double[] q = nearMain.near(p);
        if (q[1] >= 0 && q[0] <= 25) return q[0];
        // exact nearest of the coarse set (same answer as the guard's full scan), by rings of 100 m cells
        long gx = (long) Math.floor(p[0] / CC), gy = (long) Math.floor(p[1] / CC);
        double best = Double.POSITIVE_INFINITY;
        for (int r = 0; r < 400; r++) {
            for (long dx = -r; dx <= r; dx++)
                for (long dy = -r; dy <= r; dy++) {
                    if (Math.max(Math.abs(dx), Math.abs(dy)) != r) continue;
                    IntArrayList l = coarseGrid.get(PtIndex.key(gx + dx, gy + dy));
                    if (l == null) continue;
                    for (int t = 0; t < l.size(); t++) {
                        double[] m = coarse[l.get(t)];
                        best = Math.min(best, Math.hypot(m[0] - p[0], m[1] - p[1]));
                    }
                }
            if (best <= r * CC) break;
        }
        return best;
    }

    /** true when a projected point lies more than 20 m from main (the guard's "off main" rule). */
    public boolean offMain(double[] p) {
        return nearMain.near(p)[0] > 20;
    }

    public static final class Dev {
        public int s, e, exitI, rejoinI;
        public double len, span, maxD;
        public boolean good;
        public List<String> why = new ArrayList<>();
    }

    public List<Dev> deviations(double[][] altP) {
        int n = altP.length;
        double[][] info = new double[n][];
        for (int i = 0; i < n; i++) info[i] = nearMain.near(altP[i]);
        List<int[]> runs = new ArrayList<>();
        int cur = -1;
        for (int i = 0; i < n; i++) {
            boolean off = info[i][0] > 20;
            if (off && cur < 0) cur = Math.max(0, i - 1);
            if (!off && cur >= 0) {
                runs.add(new int[]{cur, i});
                cur = -1;
            }
        }
        if (cur >= 0) runs.add(new int[]{cur, n - 1});
        List<int[]> merged = new ArrayList<>();
        for (int[] r : runs) {
            if ((r[1] - r[0]) * STEP < 60) continue;
            int[] last = merged.isEmpty() ? null : merged.get(merged.size() - 1);
            if (last != null && (r[0] - last[1]) * STEP < 100) last[1] = r[1];
            else merged.add(new int[]{r[0], r[1]});
        }
        List<Dev> out = new ArrayList<>();
        for (int[] r : merged) {
            Dev d = new Dev();
            d.s = r[0];
            d.e = r[1];
            d.exitI = (int) info[r[0]][1];
            d.rejoinI = (int) info[r[1]][1];
            judge(d, altP);
            out.add(d);
        }
        return out;
    }

    void judge(Dev d, double[][] altP) {
        d.len = polyLen(altP, d.s, d.e);
        d.span = Math.abs(d.rejoinI - d.exitI) * STEP;
        double maxD = 0;
        for (int i = d.s; i <= d.e; i++) maxD = Math.max(maxD, farDist(altP[i]));
        d.maxD = maxD;
        // same pairs as the guard (i, j even, j < i - 12), found with a 15 m hash of the inserted j points
        boolean back = false;
        int np = d.e - d.s + 1;
        LongObjectHashMap<IntArrayList> h = new LongObjectHashMap<>();
        int nextJ = 0;
        for (int i = 12; i < np && !back; i += 2) {
            while (nextJ < i - 12) {
                double[] c = altP[d.s + nextJ];
                long k = PtIndex.key((long) Math.floor(c[0] / 15), (long) Math.floor(c[1] / 15));
                IntArrayList l = h.get(k);
                if (l == null) h.put(k, l = new IntArrayList());
                l.add(nextJ);
                nextJ += 2;
            }
            double[] a = altP[d.s + i];
            long gx = (long) Math.floor(a[0] / 15), gy = (long) Math.floor(a[1] / 15);
            for (long dx = -1; dx <= 1 && !back; dx++)
                for (long dy = -1; dy <= 1 && !back; dy++) {
                    IntArrayList l = h.get(PtIndex.key(gx + dx, gy + dy));
                    if (l == null) continue;
                    for (int t = 0; t < l.size(); t++) {
                        double[] c = altP[d.s + l.get(t)];
                        if (Math.hypot(a[0] - c[0], a[1] - c[1]) < 15) {
                            back = true;
                            break;
                        }
                    }
                }
        }
        if (d.len < 0.1 * L) d.why.add("short");
        if (d.len > 0.4 * L) d.why.add("long");
        if (d.span < 0.5 * d.len) d.why.add("spur");
        if (maxD < Math.max(150, 0.1 * d.len)) d.why.add("hugs");
        if (maxD > Math.max(500, 0.15 * L)) d.why.add("corridor");
        if (d.len > 2 * Math.max(d.span, 1)) d.why.add("detour");
        if (back) d.why.add("outback");
        d.good = d.why.isEmpty();
    }

    public static final class AltScore {
        public List<Dev> devs;
        public boolean real;
        public List<String> why = new ArrayList<>();
        public double reuse, extra, lenAlt;
        public double[][] devPts; // points of all deviations (for the distinct test)
    }

    public AltScore score(List<double[]> alt) {
        double[][] altP = resample(alt);
        AltScore a = new AltScore();
        a.lenAlt = polyLen(altP, 0, altP.length - 1);
        a.devs = deviations(altP);
        double spanSum = 0;
        int good = 0;
        List<double[]> pts = new ArrayList<>();
        for (Dev d : a.devs) {
            spanSum += d.span;
            if (d.good) good++;
            for (int i = d.s; i <= d.e; i++) pts.add(altP[i]);
        }
        a.devPts = pts.toArray(new double[0][]);
        int bad = a.devs.size() - good;
        a.reuse = Math.max(0, 1 - spanSum / L);
        a.extra = a.lenAlt - L;
        int need = L >= 6000 ? 2 : 1;
        if (good < need) a.why.add("good " + good + "<" + need);
        if (bad > 0) a.why.add("bad " + bad);
        if (a.reuse < 0.4 || a.reuse > 0.85) a.why.add("reuse " + Math.round(a.reuse * 100) + "%");
        if (a.extra > 0.2 * L) a.why.add("extra " + Math.round(a.extra) + "m");
        a.real = a.why.isEmpty();
        return a;
    }

    /** guard: second is distinct from first when <= 30 % of its deviation points lie within 30 m of the first's. */
    public static boolean distinct(double[][] d1, double[][] d2) {
        if (d1.length == 0 || d2.length == 0) return d2.length > 0;
        return overlapFrac(d1, d2) <= 0.3;
    }

    public static double overlapFrac(double[][] d1, double[][] d2) {
        if (d2.length == 0) return 0;
        PtIndex n1 = new PtIndex(d1, 30);
        int c = 0;
        for (double[] p : d2) if (n1.near(p)[0] <= 30) c++;
        return c / (double) d2.length;
    }

    /** Nearest-sample grid index (guard.mjs pointIndex, reach 1). near() returns {d, i}; i = -1 when nothing in reach. */
    public static final class PtIndex {
        final double cell;
        final double[][] P;
        final LongObjectHashMap<IntArrayList> grid = new LongObjectHashMap<>();

        public PtIndex(double[][] P, double cell) {
            this.P = P;
            this.cell = cell;
            for (int i = 0; i < P.length; i++) {
                long k = key((long) Math.floor(P[i][0] / cell), (long) Math.floor(P[i][1] / cell));
                IntArrayList l = grid.get(k);
                if (l == null) grid.put(k, l = new IntArrayList());
                l.add(i);
            }
        }

        static long key(long gx, long gy) {
            return (gx << 32) ^ (gy & 0xffffffffL);
        }

        public double[] near(double[] p) {
            double best = Double.POSITIVE_INFINITY;
            int at = -1;
            long gx = (long) Math.floor(p[0] / cell), gy = (long) Math.floor(p[1] / cell);
            for (long dx = -1; dx <= 1; dx++)
                for (long dy = -1; dy <= 1; dy++) {
                    IntArrayList l = grid.get(key(gx + dx, gy + dy));
                    if (l == null) continue;
                    for (int t = 0; t < l.size(); t++) {
                        int i = l.get(t);
                        double d = Math.hypot(P[i][0] - p[0], P[i][1] - p[1]);
                        if (d < best) {
                            best = d;
                            at = i;
                        }
                    }
                }
            return new double[]{best, at};
        }
    }
}
