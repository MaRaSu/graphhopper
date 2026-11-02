package com.graphhopper.trailmap.tilerouting;

import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.ResponsePath;
import com.graphhopper.json.Statement;
import com.graphhopper.trailmap.tilerouting.foundation.Foundation;
import com.graphhopper.trailmap.tilerouting.foundation.Via;
import com.graphhopper.util.CustomModel;
import com.graphhopper.util.JsonFeature;
import com.graphhopper.util.JsonFeatureCollection;
import com.graphhopper.util.PointList;
import com.graphhopper.util.details.PathDetail;
import com.graphhopper.util.shapes.GHPoint;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Polygon;

import java.util.*;

/**
 * Re-use polish (owner's idea, 2026-10-29; Java port of tile-routing/polish/polish.mjs): keep the tour's tile visits,
 * re-route each leg with roads the rest of the loop rides made dearer, then pin each changed leg with "shape" vias so
 * plain-via routing (the app) reproduces it. A leg change is kept only when it rides less re-used road, adds no trail
 * or hard km and every tile the loop fetched is still fetched.
 *
 * <p>Ridden roads are marked as in the Node tool: tiny squares at the middle of the other legs' geometry segments, an
 * area in a per-request custom model, so a road that only crosses or branches off a ridden road is not penalised.
 */
public class ReusePolish {

    // the Node tool's P
    static final double REUSE_M = 20, MARK_EVERY_M = 25, MARK_HALF_M = 1, MATCH_M = 25, MIN_DIV_M = 60, TRAIL_TOL_KM = 0.02;
    static final int MAX_SHAPE_VIAS = 6, PASSES = 2;

    public static final class Stats {
        public int tried, changed;
        public double kmBefore, kmAfter;
        public final List<String> legs = new ArrayList<>();
    }

    private final Foundation f;
    /** Priority factor on a road another leg rides (0.2 = 5× dearer). */
    private final double penalty;
    /** Priority factor on trail / hard sections during the polish; null = not priced. */
    private final Double trailFactor;
    public final Stats stats = new Stats();
    private static final GeometryFactory GF = new GeometryFactory();

    public ReusePolish(Foundation f, double penalty, Double trailFactor) {
        this.f = f;
        this.penalty = penalty;
        this.trailFactor = trailFactor;
    }

    static final class Leg {
        List<double[]> coords;
        double km, trailHardKm;
    }

    public Foundation.Plan polish(Foundation.Plan plan, List<Foundation.TileInfo> tiles) {
        List<Via> wp = plan.vias;
        int n = wp.size();
        List<List<double[]>> legs = new ArrayList<>();
        List<List<Via>> shape = new ArrayList<>();
        double[] th0 = new double[n - 1];
        for (int i = 0; i + 1 < n; i++) {
            shape.add(new ArrayList<>());
            if (!road(plan, i)) legs.add(List.of(new double[]{wp.get(i).lat, wp.get(i).lon}, new double[]{wp.get(i + 1).lat, wp.get(i + 1).lon}));
            else {
                Leg r = route(List.of(wp.get(i), wp.get(i + 1)), f.customModel);
                if (r == null) return plan;
                legs.add(r.coords);
                th0[i] = r.trailHardKm;
            }
        }
        Set<String> before = fetched(tiles, legs);
        stats.kmBefore = totalKm(legs);
        for (int pass = 0; pass < PASSES; pass++) {
            boolean any = false;
            for (int i = 0; i + 1 < n; i++) {
                if (!road(plan, i)) continue;
                List<List<double[]>> others = new ArrayList<>(legs);
                others.remove(i);
                Index idx = new Index(others);
                double cur = reuseM(legs.get(i), idx);
                if (cur < MIN_DIV_M) continue;
                stats.tried++;
                Leg alt = route(List.of(wp.get(i), wp.get(i + 1)), penaltyModel(others));
                if (alt == null) continue;
                if (reuseM(alt.coords, idx) > cur - MIN_DIV_M) continue;
                if (alt.trailHardKm > th0[i] + TRAIL_TOL_KM) continue;
                Pinned pinned = pin(wp.get(i), wp.get(i + 1), alt.coords);
                if (pinned == null) continue;
                double newReuse = reuseM(pinned.leg.coords, idx);
                if (pinned.leg.trailHardKm > th0[i] + TRAIL_TOL_KM || newReuse > cur - MIN_DIV_M) continue;
                List<List<double[]>> trial = new ArrayList<>(legs);
                trial.set(i, pinned.leg.coords);
                if (!fetched(tiles, trial).containsAll(before)) continue;
                stats.legs.add(String.format(Locale.ROOT, "leg%d %.2f→%.2fkm reuse %.0f→%.0fm vias+%d", i, segLen(legs.get(i)) / 1000,
                        pinned.leg.km, cur, newReuse, pinned.vias.size()));
                legs.set(i, pinned.leg.coords);
                shape.set(i, pinned.vias);
                stats.changed++;
                any = true;
            }
            if (!any) break;
        }
        stats.kmAfter = totalKm(legs);
        if (stats.changed == 0) return plan;
        Foundation.Plan out = new Foundation.Plan();
        out.walks.addAll(plan.walks);
        for (int i = 0; i < n; i++) {
            out.vias.add(wp.get(i));
            if (i + 1 < n) {
                out.segments.add(plan.segments.get(i));
                for (Via s : shape.get(i)) {
                    out.vias.add(s);
                    out.segments.add(Foundation.FOLLOW_ROADS);
                }
            }
        }
        return out;
    }

    private static boolean road(Foundation.Plan plan, int i) {
        return !Foundation.DIRECT.equals(plan.segments.get(i));
    }

    /** One plain-via route, the given custom model merged onto the profile's (null = the profile as is). */
    Leg route(List<Via> pts, CustomModel model) {
        List<GHPoint> ps = new ArrayList<>();
        for (Via v : pts) ps.add(new GHPoint(v.lat, v.lon));
        GHRequest req = new GHRequest(ps).setProfile(f.profile).setPathDetails(List.of("gravel_scale"));
        if (model != null) req.setCustomModel(model);
        GHResponse res = f.hopper.route(req);
        if (res.hasErrors()) return null;
        ResponsePath p = res.getBest();
        Leg l = new Leg();
        l.coords = new ArrayList<>();
        PointList pl = p.getPoints();
        for (int i = 0; i < pl.size(); i++) l.coords.add(new double[]{pl.getLat(i), pl.getLon(i)});
        l.km = p.getDistance() / 1000;
        double m = 0;
        for (PathDetail d : p.getPathDetails().getOrDefault("gravel_scale", List.of())) {
            String v = String.valueOf(d.getValue());
            if (!v.equals("2") && !v.equals("3") && !v.equals("4") && !v.equals("unknown")) continue;
            for (int i = d.getFirst(); i < d.getLast(); i++) m += dist(l.coords.get(i), l.coords.get(i + 1));
        }
        l.trailHardKm = m / 1000;
        return l;
    }

    /** The request's own custom model plus one area of squares on the given legs (and the optional trail price). */
    CustomModel penaltyModel(List<List<double[]>> legs) {
        List<Polygon> polys = new ArrayList<>();
        for (List<double[]> c : legs) {
            double since = Double.POSITIVE_INFINITY;
            for (int i = 1; i < c.size(); i++) {
                double L = dist(c.get(i - 1), c.get(i));
                since += L;
                if (L < 2 || since < MARK_EVERY_M) continue;
                since = 0;
                double lat = (c.get(i - 1)[0] + c.get(i)[0]) / 2, lon = (c.get(i - 1)[1] + c.get(i)[1]) / 2;
                double dLat = MARK_HALF_M / M_PER_DEG, dLon = MARK_HALF_M / (M_PER_DEG * Math.cos(Math.toRadians(lat)));
                Coordinate[] ring = {new Coordinate(lon - dLon, lat - dLat), new Coordinate(lon + dLon, lat - dLat),
                        new Coordinate(lon + dLon, lat + dLat), new Coordinate(lon - dLon, lat + dLat), new Coordinate(lon - dLon, lat - dLat)};
                polys.add(GF.createPolygon(ring));
            }
        }
        CustomModel m = f.customModel == null ? new CustomModel() : new CustomModel(f.customModel);
        m.addToPriority(Statement.If("in_used", Statement.Op.MULTIPLY, String.valueOf(penalty)));
        if (trailFactor != null)
            m.addToPriority(Statement.If("gravel_scale == TWO || gravel_scale == THREE || gravel_scale == FOUR || gravel_scale == UNKNOWN",
                    Statement.Op.MULTIPLY, String.valueOf(trailFactor)));
        JsonFeatureCollection areas = new JsonFeatureCollection();
        areas.getFeatures().add(new JsonFeature("used", "Feature", null, GF.createMultiPolygon(polys.toArray(new Polygon[0])), new HashMap<>()));
        m.addAreas(areas);
        return m;
    }

    static final class Pinned {
        Leg leg;
        List<Via> vias = new ArrayList<>();
    }

    /** Shape vias on target so a plain route a → … → b follows it; null when 6 vias are not enough. */
    Pinned pin(Via a, Via b, List<double[]> target) {
        List<double[]> vias = new ArrayList<>();
        List<Integer> at = new ArrayList<>();
        Leg got = route(List.of(a, b), f.customModel);
        for (int k = 0; k < MAX_SHAPE_VIAS && got != null; k++) {
            List<int[]> div = divergences(target, got.coords);
            if (div.isEmpty()) return pinned(got, vias);
            int[] d = div.get(0);
            double dLen = segLen(target.subList(d[0], d[1] + 1));
            for (int[] x : div) {
                double l = segLen(target.subList(x[0], x[1] + 1));
                if (l > dLen) {
                    d = x;
                    dLen = l;
                }
            }
            // the target point halfway along the divergent stretch
            double s = 0;
            int idx = d[0];
            for (int i = d[0] + 1; i <= d[1]; i++) {
                s += dist(target.get(i - 1), target.get(i));
                idx = i;
                if (s >= dLen / 2) break;
            }
            double[] v = target.get(idx);
            int pos = nearest(target, v);
            int ins = 0;
            while (ins < at.size() && at.get(ins) <= pos) ins++;
            vias.add(ins, v);
            at.add(ins, pos);
            List<Via> pts = new ArrayList<>();
            pts.add(a);
            for (double[] p : vias) pts.add(shapeVia(p));
            pts.add(b);
            got = route(pts, f.customModel);
        }
        if (got == null || !divergences(target, got.coords).isEmpty()) return null;
        return pinned(got, vias);
    }

    private static Pinned pinned(Leg got, List<double[]> vias) {
        Pinned p = new Pinned();
        p.leg = got;
        for (double[] v : vias) p.vias.add(shapeVia(v));
        return p;
    }

    private static Via shapeVia(double[] p) {
        return new Via(Math.round(p[0] * 1e6) / 1e6, Math.round(p[1] * 1e6) / 1e6, Double.NaN, "shape", null);
    }

    private static int nearest(List<double[]> c, double[] p) {
        int best = 0;
        double bd = Double.POSITIVE_INFINITY;
        for (int i = 0; i < c.size(); i++) {
            double d = dist(p, c.get(i));
            if (d < bd) {
                bd = d;
                best = i;
            }
        }
        return best;
    }

    /** Stretches of target farther than MATCH_M from got: [i0, i1] (target indices), longer than MIN_DIV_M. */
    static List<int[]> divergences(List<double[]> target, List<double[]> got) {
        Index idx = new Index(List.of(got));
        List<int[]> out = new ArrayList<>();
        int[] cur = null;
        for (int i = 0; i < target.size(); i++) {
            boolean far = idx.distance(target.get(i)[0], target.get(i)[1]) > MATCH_M;
            if (far && cur == null) cur = new int[]{i, i};
            else if (far) cur[1] = i;
            else if (cur != null) {
                out.add(cur);
                cur = null;
            }
        }
        if (cur != null) out.add(cur);
        out.removeIf(d -> segLen(target.subList(d[0], d[1] + 1)) <= MIN_DIV_M);
        return out;
    }

    /** Metres of c within REUSE_M of idx (sampled every ≤ 10 m). */
    static double reuseM(List<double[]> c, Index idx) {
        double m = 0;
        for (int i = 1; i < c.size(); i++) {
            double[] p = c.get(i - 1), q = c.get(i);
            double L = dist(p, q);
            int k = Math.max(1, (int) Math.ceil(L / 10));
            for (int j = 0; j < k; j++) {
                double t = (j + 0.5) / k;
                if (idx.distance(p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])) <= REUSE_M) m += L / k;
            }
        }
        return m;
    }

    /** Tiles whose inset any leg enters. */
    static Set<String> fetched(List<Foundation.TileInfo> tiles, List<List<double[]>> legs) {
        Set<String> got = new HashSet<>();
        for (Foundation.TileInfo ti : tiles)
            for (List<double[]> c : legs)
                if (Foundation.fetched(ti.tile, c)) {
                    got.add(ti.tile.key);
                    break;
                }
        return got;
    }

    static double totalKm(List<List<double[]>> legs) {
        double m = 0;
        for (List<double[]> c : legs) m += segLen(c);
        return m / 1000;
    }

    static double segLen(List<double[]> c) {
        double s = 0;
        for (int i = 1; i < c.size(); i++) s += dist(c.get(i - 1), c.get(i));
        return s;
    }

    static double dist(double[] a, double[] b) {
        return Foundation.dist(a[0], a[1], b[0], b[1]);
    }

    static final double M_PER_DEG = 111_195;

    /** Point-to-polylines distance in local metres, segments bucketed in a 50 m grid. */
    static final class Index {
        static final double CELL = 50;
        final double lat0, kx;
        final Map<Long, List<double[]>> grid = new HashMap<>();

        Index(List<List<double[]>> lines) {
            double[] first = null;
            for (List<double[]> c : lines) if (!c.isEmpty()) {
                first = c.get(0);
                break;
            }
            lat0 = first == null ? 0 : first[0];
            kx = M_PER_DEG * Math.cos(Math.toRadians(lat0));
            for (List<double[]> c : lines)
                for (int i = 1; i < c.size(); i++) {
                    double x0 = x(c.get(i - 1)[1]), y0 = y(c.get(i - 1)[0]), x1 = x(c.get(i)[1]), y1 = y(c.get(i)[0]);
                    double[] s = {x0, y0, x1, y1};
                    long ca = (long) Math.floor(Math.min(x0, x1) / CELL), cb = (long) Math.floor(Math.max(x0, x1) / CELL);
                    long ra = (long) Math.floor(Math.min(y0, y1) / CELL), rb = (long) Math.floor(Math.max(y0, y1) / CELL);
                    for (long cx = ca; cx <= cb; cx++)
                        for (long cy = ra; cy <= rb; cy++) grid.computeIfAbsent(key(cx, cy), k -> new ArrayList<>()).add(s);
                }
        }

        double x(double lon) {
            return lon * kx;
        }

        double y(double lat) {
            return lat * M_PER_DEG;
        }

        static long key(long cx, long cy) {
            return cx * 4_000_000_003L + cy;
        }

        /** Distance to the nearest segment; queries reach one cell around (exact up to 50 m, which covers every use). */
        double distance(double lat, double lon) {
            double px = x(lon), py = y(lat);
            long cx = (long) Math.floor(px / CELL), cy = (long) Math.floor(py / CELL);
            double best = Double.POSITIVE_INFINITY;
            for (long dx = -1; dx <= 1; dx++)
                for (long dy = -1; dy <= 1; dy++) {
                    List<double[]> l = grid.get(key(cx + dx, cy + dy));
                    if (l == null) continue;
                    for (double[] s : l) best = Math.min(best, segDist(px, py, s));
                }
            return best;
        }

        static double segDist(double px, double py, double[] s) {
            double dx = s[2] - s[0], dy = s[3] - s[1], L2 = dx * dx + dy * dy;
            double t = L2 == 0 ? 0 : Math.max(0, Math.min(1, ((px - s[0]) * dx + (py - s[1]) * dy) / L2));
            return Math.hypot(px - (s[0] + t * dx), py - (s[1] + t * dy));
        }
    }
}
