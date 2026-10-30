/*
 * Trailmap - Personal routing, "bit of new" variations
 */
package com.graphhopper.trailmap.personal.variations;

import com.carrotsearch.hppc.IntArrayList;
import com.carrotsearch.hppc.LongObjectHashMap;

import java.util.ArrayList;
import java.util.List;

/**
 * Round-3 A's geometry judge, copied unchanged from the prototype ({@code R3AGuard}): an exact Java port of the
 * geometry part of {@code personal-routing/r3/guard.mjs} (round-3 guard scorer), so the A3 generator checks a spliced
 * alternative the way the guard does. Also provides the main-route index used by the face walk and the banded search
 * (lateral offset, side, along).
 */
final class A3Guard {
    static final double STEP = 5;

    final double kx, ky;
    final double[][] mainP;     // projected 5 m samples of main
    final double[] cum;         // along metres at each sample
    final int[] sampleOfPoint;  // first sample index of each original main point
    final double L;
    private final LongObjectHashMap<IntArrayList> grid25 = new LongObjectHashMap<>();
    private final LongObjectHashMap<IntArrayList> grid50 = new LongObjectHashMap<>();
    private final LongObjectHashMap<IntArrayList> grid250 = new LongObjectHashMap<>();
    private final LongObjectHashMap<double[]> farCell = new LongObjectHashMap<>();
    private final LongObjectHashMap<int[]> nearList = new LongObjectHashMap<>();
    private final double[][] coarse;

    A3Guard(List<double[]> main) {
        double lat0 = main.get(0)[1];
        kx = 111320 * Math.cos(lat0 * Math.PI / 180);
        ky = 110540;
        sampleOfPoint = new int[main.size()];
        mainP = resample(main, sampleOfPoint);
        cum = new double[mainP.length];
        for (int i = 1; i < mainP.length; i++)
            cum[i] = cum[i - 1] + Math.hypot(mainP[i][0] - mainP[i - 1][0], mainP[i][1] - mainP[i - 1][1]);
        L = cum[cum.length - 1];
        for (int i = 0; i < mainP.length; i++) {
            add(grid25, key((long) Math.floor(mainP[i][0] / 25), (long) Math.floor(mainP[i][1] / 25)), i);
            add(grid50, key((long) Math.floor(mainP[i][0] / 50), (long) Math.floor(mainP[i][1] / 50)), i);
            add(grid250, key((long) Math.floor(mainP[i][0] / 250), (long) Math.floor(mainP[i][1] / 250)), i);
        }
        int nc = (mainP.length + 3) / 4;
        coarse = new double[nc][];
        for (int i = 0, c = 0; i < mainP.length; i += 4) coarse[c++] = mainP[i];
    }

    private static void add(LongObjectHashMap<IntArrayList> g, long k, int i) {
        IntArrayList l = g.get(k);
        if (l == null) g.put(k, l = new IntArrayList());
        l.add(i);
    }

    static long key(long gx, long gy) {
        return (gx << 32) ^ (gy & 0xffffffffL);
    }

    double[] proj(double lon, double lat) {
        return new double[]{lon * kx, lat * ky};
    }

    /** guard.mjs resample: projected points every ~5 m; optionally the first sample index of each input point. */
    double[][] resample(List<double[]> coords, int[] firstSample) {
        List<double[]> out = new ArrayList<>();
        for (int i = 0; i < coords.size() - 1; i++) {
            if (firstSample != null) firstSample[i] = out.size();
            double[] a = proj(coords.get(i)[0], coords.get(i)[1]), b = proj(coords.get(i + 1)[0], coords.get(i + 1)[1]);
            int n = (int) Math.max(1, Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]) / STEP));
            for (int s = 0; s < n; s++) out.add(new double[]{a[0] + (b[0] - a[0]) * s / n, a[1] + (b[1] - a[1]) * s / n});
        }
        if (!coords.isEmpty()) {
            if (firstSample != null) firstSample[coords.size() - 1] = out.size();
            double[] p = coords.get(coords.size() - 1);
            out.add(proj(p[0], p[1]));
        }
        return out.toArray(new double[0][]);
    }

    static double polyLen(double[][] P, int s, int e) {
        double L = 0;
        for (int i = s + 1; i <= e; i++) L += Math.hypot(P[i][0] - P[i - 1][0], P[i][1] - P[i - 1][1]);
        return L;
    }

    /** guard nearMain(p, reach 1) on the 25 m grid: {d, i} (d = +inf, i = -1 when no sample nearby). */
    double[] near(double x, double y) {
        double best = Double.POSITIVE_INFINITY;
        int at = -1;
        long gx = (long) Math.floor(x / 25), gy = (long) Math.floor(y / 25);
        for (long dx = -1; dx <= 1; dx++)
            for (long dy = -1; dy <= 1; dy++) {
                IntArrayList l = grid25.get(key(gx + dx, gy + dy));
                if (l == null) continue;
                for (int t = 0; t < l.size(); t++) {
                    int i = l.get(t);
                    double d = Math.hypot(mainP[i][0] - x, mainP[i][1] - y);
                    if (d < best) {
                        best = d;
                        at = i;
                    }
                }
            }
        return new double[]{best, at};
    }

    double farDist(double[] p) {
        double[] q = near(p[0], p[1]);
        if (q[1] >= 0 && q[0] <= 25) return q[0];
        double best = Double.POSITIVE_INFINITY;
        for (double[] m : coarse) best = Math.min(best, Math.hypot(m[0] - p[0], m[1] - p[1]));
        return best;
    }

    /**
     * Nearest main sample within {@code maxR} (50 m ring search): {d, along, side(+1 left / -1 right), sampleIdx};
     * d = +inf when nothing within maxR.
     */
    double[] lateral(double x, double y, double maxR) {
        // far points (cell centre > 260 m from main): nearest sample of the 50 m cell centre, cached; the
        // distance is then within ~70 m of exact, which only matters for the corridor reach, never the band
        long ck = key((long) Math.floor(x / 50), (long) Math.floor(y / 50));
        double[] fc = farCell.get(ck);
        if (fc == null) {
            double cx = (Math.floor(x / 50) + 0.5) * 50, cy = (Math.floor(y / 50) + 0.5) * 50;
            double[] q = ring(grid50, 50, cx, cy, 300);
            if (q[1] < 0) q = ring(grid250, 250, cx, cy, maxR + 400);
            fc = q;
            farCell.put(ck, fc);
        }
        double[] q;
        if (fc[1] < 0) return new double[]{Double.POSITIVE_INFINITY, 0, 0, -1};
        if (fc[0] > 260) {
            int i = (int) fc[1];
            q = new double[]{Math.hypot(mainP[i][0] - x, mainP[i][1] - y), i};
        } else {
            // exact: the nearest sample to any point of the cell lies within dCentre + 36 + 36 m of the centre
            int[] cand = nearList.get(ck);
            if (cand == null) {
                double cx = (Math.floor(x / 50) + 0.5) * 50, cy = (Math.floor(y / 50) + 0.5) * 50, rr = fc[0] + 72;
                IntArrayList l = new IntArrayList();
                long gx = (long) Math.floor(cx / 50), gy = (long) Math.floor(cy / 50);
                int R = (int) Math.ceil(rr / 50) + 1;
                for (long dx = -R; dx <= R; dx++)
                    for (long dy = -R; dy <= R; dy++) {
                        IntArrayList c = grid50.get(key(gx + dx, gy + dy));
                        if (c == null) continue;
                        for (int t = 0; t < c.size(); t++) {
                            int i = c.get(t);
                            if (Math.hypot(mainP[i][0] - cx, mainP[i][1] - cy) <= rr) l.add(i);
                        }
                    }
                cand = l.toArray();
                nearList.put(ck, cand);
            }
            double best = Double.POSITIVE_INFINITY;
            int at = -1;
            for (int i : cand) {
                double d = Math.hypot(mainP[i][0] - x, mainP[i][1] - y);
                if (d < best) {
                    best = d;
                    at = i;
                }
            }
            q = new double[]{best, at};
        }
        int at = (int) q[1];
        double best = q[0];
        if (at < 0 || best > maxR) return new double[]{Double.POSITIVE_INFINITY, 0, 0, -1};
        int a = Math.max(0, at - 2), b = Math.min(mainP.length - 1, at + 2);
        double tx = mainP[b][0] - mainP[a][0], ty = mainP[b][1] - mainP[a][1];
        double cr = tx * (y - mainP[at][1]) - ty * (x - mainP[at][0]);
        return new double[]{best, cum[at], cr >= 0 ? 1 : -1, at};
    }

    /** Ring search on one grid: {d, idx} of the nearest sample, idx = -1 when none within maxR. */
    private double[] ring(LongObjectHashMap<IntArrayList> grid, double cell, double x, double y, double maxR) {
        long gx = (long) Math.floor(x / cell), gy = (long) Math.floor(y / cell);
        int R = (int) Math.ceil(maxR / cell) + 1;
        double best = Double.POSITIVE_INFINITY;
        int at = -1;
        for (int r = 0; r <= R; r++) {
            if (best <= (r - 1) * cell) break;
            for (long dx = -r; dx <= r; dx++)
                for (long dy = -r; dy <= r; dy++) {
                    if (Math.max(Math.abs(dx), Math.abs(dy)) != r) continue;
                    IntArrayList l = grid.get(key(gx + dx, gy + dy));
                    if (l == null) continue;
                    for (int t = 0; t < l.size(); t++) {
                        int i = l.get(t);
                        double d = Math.hypot(mainP[i][0] - x, mainP[i][1] - y);
                        if (d < best) {
                            best = d;
                            at = i;
                        }
                    }
                }
        }
        if (at < 0 || best > maxR) return new double[]{Double.POSITIVE_INFINITY, -1};
        return new double[]{best, at};
    }

    /** Fast distance to main (exact nearest 5 m sample), for local pre-checks; +inf beyond maxR. */
    double fastDist(double[] p, double maxR) {
        double[] q = near(p[0], p[1]);
        if (q[1] >= 0 && q[0] <= 25) return q[0];
        return lateral(p[0], p[1], maxR)[0];
    }

    // ------------------------------------------------------------------ guard port

    static final class Dev {
        int s, e, exitI, rejoinI;
        double len, span, maxD;
        boolean good;
        List<String> why = new ArrayList<>();
    }

    List<Dev> deviations(double[][] altP) {
        int n = altP.length;
        double[][] info = new double[n][];
        for (int i = 0; i < n; i++) info[i] = near(altP[i][0], altP[i][1]);
        List<int[]> runs = new ArrayList<>();
        int[] cur = null;
        for (int i = 0; i < n; i++) {
            boolean off = info[i][0] > 20;
            if (off && cur == null) cur = new int[]{Math.max(0, i - 1), -1};
            if (!off && cur != null) {
                cur[1] = i;
                runs.add(cur);
                cur = null;
            }
        }
        if (cur != null) {
            cur[1] = n - 1;
            runs.add(cur);
        }
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
            out.add(d);
        }
        return out;
    }

    void judge(Dev d, double[][] altP) {
        judge(d, altP, false);
    }

    void judge(Dev d, double[][] altP, boolean fast) {
        d.len = polyLen(altP, d.s, d.e);
        d.span = Math.abs(d.rejoinI - d.exitI) * STEP;
        double maxD = 0, cap = Math.max(500, 0.15 * L) + 50;
        for (int i = d.s; i <= d.e; i++) maxD = Math.max(maxD, fast ? Math.min(cap, fastDist(altP[i], cap)) : farDist(altP[i]));
        d.maxD = maxD;
        boolean back = false;
        int np = d.e - d.s + 1;
        for (int i = 12; i < np && !back; i += 2)
            for (int j = 0; j < i - 12; j += 2) {
                double[] a = altP[d.s + i], b = altP[d.s + j];
                if (Math.hypot(a[0] - b[0], a[1] - b[1]) < 15) {
                    back = true;
                    break;
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

    static final class Alt {
        double[][] altP;
        List<Dev> devs;
        double reuse, extra, Lalt;
        int good, bad;
        boolean real;
        List<String> why = new ArrayList<>();
    }

    Alt judgeAlt(List<double[]> coords) {
        Alt a = new Alt();
        a.altP = resample(coords, null);
        a.Lalt = polyLen(a.altP, 0, a.altP.length - 1);
        a.devs = deviations(a.altP);
        double spanSum = 0;
        for (Dev d : a.devs) {
            judge(d, a.altP);
            spanSum += d.span;
            if (d.good) a.good++;
        }
        a.bad = a.devs.size() - a.good;
        a.reuse = Math.max(0, 1 - spanSum / L);
        a.extra = a.Lalt - L;
        int need = L >= 6000 ? 2 : 1;
        if (a.good < need) a.why.add("good " + a.good + "<" + need);
        if (a.bad > 0) a.why.add("bad " + a.bad);
        if (a.reuse < 0.4 || a.reuse > 0.85) a.why.add("reuse " + Math.round(a.reuse * 100) + "%");
        if (a.extra > 0.2 * L) a.why.add("extra " + Math.round(a.extra) + "m");
        a.real = a.why.isEmpty();
        return a;
    }

    /** Deviation points of an alternative (as the guard collects them for the distinct test). */
    static List<double[]> devPoints(Alt a) {
        List<double[]> out = new ArrayList<>();
        for (Dev d : a.devs) for (int i = d.s; i <= d.e; i++) out.add(a.altP[i]);
        return out;
    }

    /** Share of {@code pts} within 30 m of {@code ref} (guard pointIndex(cell 30), reach 1). */
    static double nearShare(List<double[]> ref, List<double[]> pts) {
        if (pts.isEmpty()) return 0;
        if (ref.isEmpty()) return 0;
        LongObjectHashMap<IntArrayList> g = new LongObjectHashMap<>();
        for (int i = 0; i < ref.size(); i++)
            add(g, key((long) Math.floor(ref.get(i)[0] / 30), (long) Math.floor(ref.get(i)[1] / 30)), i);
        int c = 0;
        for (double[] p : pts) {
            long gx = (long) Math.floor(p[0] / 30), gy = (long) Math.floor(p[1] / 30);
            double best = Double.POSITIVE_INFINITY;
            for (long dx = -1; dx <= 1; dx++)
                for (long dy = -1; dy <= 1; dy++) {
                    IntArrayList l = g.get(key(gx + dx, gy + dy));
                    if (l == null) continue;
                    for (int t = 0; t < l.size(); t++) {
                        double[] q = ref.get(l.get(t));
                        best = Math.min(best, Math.hypot(q[0] - p[0], q[1] - p[1]));
                    }
                }
            if (best <= 30) c++;
        }
        return c / (double) pts.size();
    }

    /** guard distinct rule for (alt1, alt2). */
    static boolean distinct(Alt a1, Alt a2) {
        List<double[]> d1 = devPoints(a1), d2 = devPoints(a2);
        if (d1.isEmpty() || d2.isEmpty()) return !d2.isEmpty();
        return nearShare(d1, d2) <= 0.3;
    }
}
