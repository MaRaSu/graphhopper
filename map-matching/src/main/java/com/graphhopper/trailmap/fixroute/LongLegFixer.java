package com.graphhopper.trailmap.fixroute;

import com.graphhopper.GraphHopper;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.ev.RoadClass;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.FetchMode;
import com.graphhopper.util.PointList;
import com.graphhopper.util.shapes.GHPoint;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * LONG LEGS (owner, 2026-09-29): on a leg of 20+ km the planner wants today's best route from A to B,
 * "as Google Maps"; a deviation is a corridor change — last time via this city, now via another
 * one — not a parallel road. So: no matching and no iteration. The leg is routed once as the client
 * does; where the saved route lies outside today's corridor, a few via points are put on the saved
 * route's roads; the leg is routed once more with them (and once again without any via point that
 * made the route turn back). The leg is not split: the via points ride in its segment, hidden from
 * the user and dropped when the user edits the leg — both wanted (owner).
 *
 * <p>Via-point spots: in a window around each target on the saved route, a spot on a road or
 * cycleway (footway for walk profiles) — falling back to a service road, then a track, when the
 * window has none (owner) — mid-edge (not at a junction), not on a dead end, rideable both ways
 * (a one-way carriageway would force a U-turn), where the saved route follows that road for a
 * while; bigger roads and spots near the target preferred.
 */
final class LongLegFixer {

    /** Samples along the lines for the corridor check [m]. */
    static final double CORRIDOR_STEP_M = 25;
    /** A spot's road must be this close to the saved route [m]... */
    static final double SPOT_SNAP_M = 10;
    /** ...which follows it for at least this long [m] (looked at up to SPOT_ALIGN_CAP_M each way). */
    static final double SPOT_ALIGN_MIN_M = 150, SPOT_ALIGN_CAP_M = 500;
    /** A spot at least this far [m] from both ends of its edge (not at a junction). */
    static final double SPOT_EDGE_END_M = 30;
    /** Window around a target, as a share of the via spacing each way. */
    static final double SPOT_WINDOW = 0.2;
    /** At most this many spots looked at per window. */
    static final int SPOT_TRIES = 150;
    /** A via point the route turns back at by this much [m] is dropped. */
    static final double UTURN_M = 30;

    record Result(ClientRoute.Leg today, List<GHPoint> vias, ClientRoute.Leg fixed, double offTodayM, double offFixedM,
                  int probes, String note) {
    }

    private final GraphHopper hopper;
    private final EnumEncodedValue<RoadClass> roadClass;

    LongLegFixer(GraphHopper hopper) {
        this.hopper = hopper;
        this.roadClass = hopper.getEncodingManager().getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
    }

    /** Corridor width [m] for a leg of {@code len} m. */
    static double corridorWidth(FixRouteRequest.Options o, double len) {
        double full = o.fixLongCorridorRatio * o.fixLongCorridorFullM;
        if (len >= o.fixLongCorridorFullM) return o.fixLongCorridorRatio * len;
        if (len <= o.fixLongLegM || o.fixLongCorridorFullM <= o.fixLongLegM) return o.fixLongCorridorMinM;
        double t = (len - o.fixLongLegM) / (o.fixLongCorridorFullM - o.fixLongLegM);
        return o.fixLongCorridorMinM + Math.pow(t, o.fixLongCorridorGrowth) * (full - o.fixLongCorridorMinM);
    }

    Result fix(ClientRoute.Settings st, EdgeFilter usable, Weighting w, GHPoint a, GHPoint b, Double heading, Double penalty,
               List<double[]> saved, boolean walk, FixRouteRequest.Options o) {
        int probes = 1;
        ClientRoute.Leg today = ClientRoute.route(hopper, st, a, List.of(), b, heading, penalty);
        if (!today.ok()) return new Result(today, List.of(), null, 0, 0, probes, "no_route");
        double len = PathSimilarity.length(saved);
        double width = corridorWidth(o, len);
        List<double[]> dev = outside(saved, today.points(), width);
        double offToday = total(dev);
        if (dev.isEmpty()) return new Result(today, List.of(), null, 0, 0, probes, "in_corridor");

        double[] cum = cumulative(saved);
        List<double[]> spots = new ArrayList<>(); // [lat, lon, arc]
        for (double[] d : dev) {
            double span = d[1] - d[0];
            int k = Math.max(1, Math.min(o.fixLongMaxViasPerDeviation, (int) Math.round(span / width)));
            double step = span / k;
            for (int j = 0; j < k; j++) {
                double t = d[0] + (j + 0.5) * step;
                double[] s = pick(saved, cum, Math.max(0, t - SPOT_WINDOW * step), Math.min(len, t + SPOT_WINDOW * step), usable, w, walk);
                if (s != null) spots.add(s);
            }
        }
        spots.sort((x, y) -> Double.compare(x[2], y[2]));
        List<GHPoint> vias = new ArrayList<>();
        for (double[] s : spots) vias.add(new GHPoint(s[0], s[1]));
        if (vias.isEmpty()) return new Result(today, List.of(), null, offToday, offToday, probes, "no_spot");

        ClientRoute.Leg fixed = ClientRoute.route(hopper, st, a, vias, b, heading, penalty);
        probes++;
        if (fixed.ok()) {
            List<GHPoint> keep = new ArrayList<>();
            for (GHPoint v : vias) if (turnBackAt(fixed.points(), v) < UTURN_M) keep.add(v);
            if (keep.size() < vias.size()) {
                vias = keep;
                fixed = vias.isEmpty() ? null : ClientRoute.route(hopper, st, a, vias, b, heading, penalty);
                probes++;
            }
        }
        if (fixed == null || !fixed.ok()) return new Result(today, List.of(), null, offToday, offToday, probes, "not_better");
        double offFixed = total(outside(saved, fixed.points(), width));
        // Only a real improvement is returned (else today's route stays).
        if (offFixed >= offToday) return new Result(today, List.of(), null, offToday, offFixed, probes, "not_better");
        return new Result(today, vias, fixed, offToday, offFixed, probes, "vias");
    }

    /**
     * Stretches [from, to] [m along saved] where the saved route is more than half a corridor width
     * from the route, for at least one corridor width.
     */
    static List<double[]> outside(List<double[]> saved, List<double[]> route, double width) {
        double half = width / 2;
        List<double[]> s = resample(saved, CORRIDOR_STEP_M), r = resample(route, CORRIDOR_STEP_M);
        double lat0 = saved.get(0)[0], mLon = 111_320.0 * Math.cos(Math.toRadians(lat0));
        Map<Long, List<double[]>> grid = new HashMap<>();
        for (double[] p : r) {
            double x = p[1] * mLon, y = p[0] * 111_320.0;
            grid.computeIfAbsent(key((long) Math.floor(x / half), (long) Math.floor(y / half)), k -> new ArrayList<>()).add(new double[]{x, y});
        }
        List<double[]> out = new ArrayList<>();
        int from = -1;
        for (int i = 0; i <= s.size(); i++) {
            boolean far = i < s.size() && !near(grid, s.get(i)[1] * mLon, s.get(i)[0] * 111_320.0, half);
            if (far && from < 0) from = i;
            if (!far && from >= 0) {
                double a = from * CORRIDOR_STEP_M, b = i * CORRIDOR_STEP_M;
                if (b - a >= width) out.add(new double[]{a, b});
                from = -1;
            }
        }
        return out;
    }

    private static boolean near(Map<Long, List<double[]>> grid, double x, double y, double half) {
        long cx = (long) Math.floor(x / half), cy = (long) Math.floor(y / half);
        double h2 = half * half;
        for (long gx = cx - 1; gx <= cx + 1; gx++) {
            for (long gy = cy - 1; gy <= cy + 1; gy++) {
                List<double[]> c = grid.get(key(gx, gy));
                if (c == null) continue;
                for (double[] q : c) {
                    double dx = q[0] - x, dy = q[1] - y;
                    if (dx * dx + dy * dy <= h2) return true;
                }
            }
        }
        return false;
    }

    private static long key(long x, long y) {
        return (x << 32) ^ (y & 0xffffffffL);
    }

    private static double total(List<double[]> stretches) {
        double t = 0;
        for (double[] d : stretches) t += d[1] - d[0];
        return t;
    }

    /** Road tiers: 1 roads / cycleways (footways for walk), 2 + service roads, 3 + tracks. 0 = never. */
    static int tier(RoadClass c, boolean walk) {
        switch (c) {
            case MOTORWAY: case TRUNK: case PRIMARY: case SECONDARY: case TERTIARY:
            case UNCLASSIFIED: case ROAD: case RESIDENTIAL: case LIVING_STREET: case CYCLEWAY:
                return 1;
            case FOOTWAY: case PEDESTRIAN:
                return walk ? 1 : 0;
            case SERVICE:
                return 2;
            case TRACK:
                return 3;
            default:
                return 0;
        }
    }

    /** Bigger roads first: "the bigger the road, the less it alters the routing" (owner). */
    static int size(RoadClass c) {
        switch (c) {
            case MOTORWAY: case TRUNK: return 7;
            case PRIMARY: return 6;
            case SECONDARY: return 5;
            case TERTIARY: return 4;
            case UNCLASSIFIED: case ROAD: return 3;
            case RESIDENTIAL: case CYCLEWAY: return 2;
            default: return 1;
        }
    }

    /** The best spot [lat, lon, arc] in [lo, hi] along the saved route: tier 1 first, then 2, then 3. */
    private double[] pick(List<double[]> saved, double[] cum, double lo, double hi, EdgeFilter usable, Weighting w, boolean walk) {
        for (int t = 1; t <= 3; t++) {
            double[] s = pick(saved, cum, lo, hi, usable, w, walk, t);
            if (s != null) return s;
        }
        return null;
    }

    private double[] pick(List<double[]> saved, double[] cum, double lo, double hi, EdgeFilter usable, Weighting w, boolean walk, int maxTier) {
        EdgeExplorer ex = hopper.getBaseGraph().createEdgeExplorer();
        NodeAccess na = hopper.getBaseGraph().getNodeAccess();
        DistanceCalcEarth dc = DistanceCalcEarth.DIST_EARTH;
        EdgeFilter allowed = e -> usable.accept(e) && tier(e.get(roadClass), walk) > 0 && tier(e.get(roadClass), walk) <= maxTier;
        double stepM = Math.max(50, (hi - lo) / SPOT_TRIES), mid = (lo + hi) / 2, half = Math.max(1, (hi - lo) / 2);
        double[] best = null;
        double bestScore = 0;
        for (double arc = lo; arc <= hi; arc += stepM) {
            double[] p = at(saved, cum, arc);
            Snap sn = hopper.getLocationIndex().findClosest(p[0], p[1], allowed);
            if (!sn.isValid() || sn.getQueryDistance() > SPOT_SNAP_M || sn.getSnappedPosition() != Snap.Position.EDGE) continue;
            EdgeIteratorState e = sn.getClosestEdge();
            int base = e.getBaseNode(), adj = e.getAdjNode();
            GHPoint sp = sn.getSnappedPoint();
            if (dc.calcDist(sp.lat, sp.lon, na.getLat(base), na.getLon(base)) < SPOT_EDGE_END_M
                    || dc.calcDist(sp.lat, sp.lon, na.getLat(adj), na.getLon(adj)) < SPOT_EDGE_END_M) continue;
            if (degree(ex, base) < 2 || degree(ex, adj) < 2) continue;
            if (Double.isInfinite(w.calcEdgeWeight(e, false)) || Double.isInfinite(w.calcEdgeWeight(e, true))) continue;
            PointList g = e.fetchWayGeometry(FetchMode.ALL);
            double align = 0;
            for (int dir = -1; dir <= 1; dir += 2) {
                for (double d = 10; d <= SPOT_ALIGN_CAP_M; d += 10) {
                    double a2 = arc + dir * d;
                    if (a2 < 0 || a2 > cum[cum.length - 1] || distToLine(at(saved, cum, a2), g) > SPOT_SNAP_M) break;
                    align += 10;
                }
            }
            if (align < SPOT_ALIGN_MIN_M) continue;
            double score = align * (1 + 0.25 * size(e.get(roadClass))) * (1 - 0.7 * Math.abs(arc - mid) / half);
            if (score > bestScore) {
                bestScore = score;
                best = new double[]{sp.lat, sp.lon, arc};
            }
        }
        return best;
    }

    private static int degree(EdgeExplorer ex, int node) {
        int n = 0;
        EdgeIterator it = ex.setBaseNode(node);
        while (it.next()) n++;
        return n;
    }

    static double distToLine(double[] p, PointList g) {
        double best = Double.POSITIVE_INFINITY, mLon = 111_320 * Math.cos(Math.toRadians(p[0]));
        for (int i = 1; i < g.size(); i++) {
            double ax = (g.getLon(i - 1) - p[1]) * mLon, ay = (g.getLat(i - 1) - p[0]) * 111_320;
            double bx = (g.getLon(i) - p[1]) * mLon, by = (g.getLat(i) - p[0]) * 111_320;
            double dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
            double t = l2 == 0 ? 0 : Math.max(0, Math.min(1, -(ax * dx + ay * dy) / l2));
            double x = ax + t * dx, y = ay + t * dy;
            best = Math.min(best, Math.sqrt(x * x + y * y));
        }
        return best;
    }

    /** Metres the route rides back at via point {@code v}: arriving and leaving along the same line. */
    static double turnBackAt(List<double[]> route, GHPoint v) {
        double[] rc = cumulative(route);
        int k = 0;
        double bd = Double.POSITIVE_INFINITY;
        for (int i = 0; i < route.size(); i++) {
            double d = DistanceCalcEarth.DIST_EARTH.calcDist(route.get(i)[0], route.get(i)[1], v.lat, v.lon);
            if (d < bd) {
                bd = d;
                k = i;
            }
        }
        double back = 0;
        for (double d = 5; d <= 300; d += 5) {
            double ta = rc[k] + d, tb = rc[k] - d;
            if (ta > rc[rc.length - 1] || tb < 0) break;
            double[] pa = at(route, rc, ta), pb = at(route, rc, tb);
            if (DistanceCalcEarth.DIST_EARTH.calcDist(pa[0], pa[1], pb[0], pb[1]) > 8) break;
            back = d;
        }
        return back;
    }

    static double[] cumulative(List<double[]> line) {
        double[] c = new double[line.size()];
        for (int i = 1; i < line.size(); i++)
            c[i] = c[i - 1] + DistanceCalcEarth.DIST_EARTH.calcDist(line.get(i - 1)[0], line.get(i - 1)[1], line.get(i)[0], line.get(i)[1]);
        return c;
    }

    static double[] at(List<double[]> line, double[] cum, double arc) {
        int i = java.util.Arrays.binarySearch(cum, arc);
        if (i >= 0) return line.get(i);
        i = -i - 1;
        if (i <= 0) return line.get(0);
        if (i >= cum.length) return line.get(line.size() - 1);
        double t = (arc - cum[i - 1]) / Math.max(1e-9, cum[i] - cum[i - 1]);
        double[] a = line.get(i - 1), b = line.get(i);
        return new double[]{a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])};
    }

    static List<double[]> resample(List<double[]> line, double step) {
        List<double[]> out = new ArrayList<>();
        if (line.isEmpty()) return out;
        double[] cum = cumulative(line);
        for (double d = 0; d <= cum[cum.length - 1]; d += step) out.add(at(line, cum, d));
        return out;
    }
}
