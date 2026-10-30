package com.graphhopper.trailmap.roundtrip.loop;

import com.graphhopper.ConvertRequest;
import com.graphhopper.ConvertResponse;
import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.ResponsePath;
import com.graphhopper.util.PointList;
import com.graphhopper.util.shapes.GHPoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

import static com.graphhopper.util.Parameters.Routing.CALC_POINTS;
import static com.graphhopper.util.Parameters.Routing.INSTRUCTIONS;

/**
 * Hands one {@link LoopAutoRoute} loop to the client the way the client will rebuild it: its waypoints are snapped and
 * routed on {@code <profile>_explore} (as the exploration round_trip returns them), then {@code /convert} re-routes them
 * and the final route runs on the standard profile. That final route is what the user gets.
 * <p>
 * The engine's waypoint cover predicts /convert's legs, but not exactly (algo/autoroute/rounds/4/fidelity/report.md:
 * near-tie alternatives within the cover's slack, the x5 avoid penalty priced differently, the start snap). So the
 * hand-over checks the final route against the loop: where the loop runs at least {@link #MIN_DIVERGENCE_M} outside a
 * {@link #CORRIDOR_M} corridor of the final route, it adds one waypoint on the loop in the middle of that stretch and
 * rebuilds, at most {@link #MAX_INSERTS} times. A loop the final route already follows is returned unchanged.
 */
public class LoopHandover {

    private static final Logger LOGGER = LoggerFactory.getLogger(LoopHandover.class);

    /** The loop counts as followed within this distance of the final route (the harness's fidelity corridor). */
    static final double CORRIDOR_M = 30;
    /** A stretch of the loop shorter than this outside the corridor is left alone. */
    static final double MIN_DIVERGENCE_M = 100;
    /** Sampling step along the loop. */
    static final double STEP_M = 25;
    /** At most this many waypoints are added per loop. */
    public static final int MAX_INSERTS = 4;

    /** The route as the client gets it. */
    public static final class Result {
        /** the response path: snapped waypoints (start, loop waypoints, start) + the final route */
        public final ResponsePath path;
        /** waypoints added to the engine's cover */
        public final int inserted;
        /** metres of the loop still outside the corridor of the final route (0 = faithful) */
        public final double offLoopM;
        /** false when /convert or its final route failed and the path is the explore via-route */
        public final boolean converted;

        Result(ResponsePath path, int inserted, double offLoopM, boolean converted) {
            this.path = path;
            this.inserted = inserted;
            this.offLoopM = offLoopM;
            this.converted = converted;
        }
    }

    private final GraphHopper hopper;
    private final int maxInserts;

    public LoopHandover(GraphHopper hopper) {
        this(hopper, MAX_INSERTS);
    }

    /** maxInserts: 0 hands the engine's own cover over unchanged. */
    public LoopHandover(GraphHopper hopper, int maxInserts) {
        this.hopper = hopper;
        this.maxInserts = maxInserts;
    }

    /**
     * @param request carries the custom model, path details, snap preventions and locale of the press
     * @return null when routing the loop's waypoints on the explore profile fails (the loop is dropped)
     */
    public Result handOver(String profile, GHPoint start, LoopAutoRoute.Route route, GHRequest request) {
        List<double[]> waypoints = new ArrayList<>(route.waypoints);
        Result best = null;
        for (int inserts = 0; ; inserts++) {
            ResponsePath via = viaRoute(profile, start, waypoints, request);
            if (via == null) {
                if (best == null) return null;
                LOGGER.warn("autoroute: routing a loop with {} added waypoints failed, keeping {}", inserts, best.inserted);
                return best;
            }
            ResponsePath fin = converted(via, profile, request);
            if (fin == null) {
                // the client keeps the explore route too; nothing to compare
                return best != null ? best : new Result(via, 0, Double.NaN, false);
            }
            Divergence d = divergence(route.coords, fin.getPoints());
            Result r = new Result(fin, inserts, d.offM, true);
            if (best == null || r.offLoopM < best.offLoopM) best = r;
            if (d.longest == null || inserts >= maxInserts) {
                if (d.longest != null)
                    LOGGER.info("autoroute: {} m of a {} loop still off the final route after {} added waypoints",
                            Math.round(d.offM), route.source, inserts);
                return best;
            }
            insert(waypoints, route.coords, d.longest);
        }
    }

    /** The client's exploration result: start, waypoints, start, snapped and routed on the explore profile. */
    private ResponsePath viaRoute(String profile, GHPoint start, List<double[]> waypoints, GHRequest request) {
        GHRequest leg = new GHRequest();
        leg.addPoint(start);
        for (double[] p : waypoints) leg.addPoint(new GHPoint(p[0], p[1]));
        leg.addPoint(start);
        leg.setProfile(profile + "_explore");
        leg.setCustomModel(request.getCustomModel());
        leg.setPathDetails(request.getPathDetails());
        leg.setSnapPreventions(request.getSnapPreventions());
        leg.setLocale(request.getLocale());
        leg.getHints().putObject(INSTRUCTIONS, false);
        leg.getHints().putObject(CALC_POINTS, true);
        GHResponse rsp = hopper.route(leg);
        if (rsp.hasErrors()) {
            LOGGER.warn("autoroute: routing the waypoints of a loop failed: {}", rsp.getErrors());
            return null;
        }
        return rsp.getBest();
    }

    /**
     * The loop as the user will get it: the client sends {@code snapped_waypoints} to {@code /convert}, which re-routes
     * them with the x5 avoid-reuse penalty (Stage A), normalises them and routes the result on the standard profile
     * (Stage C). Same call, same inputs, plus the details the press asked for; {@code snapped_waypoints} stay the via
     * route's, so the client's /convert gives exactly this route. Null when the conversion fails.
     */
    private ResponsePath converted(ResponsePath via, String profile, GHRequest request) {
        PointList wps = via.getWaypoints();
        List<GHPoint> waypoints = new ArrayList<>(wps.size());
        for (int i = 0; i < wps.size(); i++) waypoints.add(new GHPoint(wps.getLat(i), wps.getLon(i)));
        ConvertRequest convertRequest = new ConvertRequest(profile, waypoints);
        if (request.getCustomModel() != null) convertRequest.setCustomModel(request.getCustomModel());
        ConvertResponse converted = hopper.convert(convertRequest);
        if (converted.hasErrors()) {
            LOGGER.warn("autoroute: /convert of a loop failed, showing the via-route: {}", converted.getErrors());
            return null;
        }
        // the same final request as RouteConversionService's Stage C, plus the details the client asked for
        GHRequest finalReq = new GHRequest(new ArrayList<>(converted.getNormalizedWaypoints()));
        finalReq.setProfile(profile);
        if (request.getCustomModel() != null) finalReq.setCustomModel(request.getCustomModel());
        finalReq.setPathDetails(request.getPathDetails());
        finalReq.putHint(INSTRUCTIONS, false);
        finalReq.putHint(CALC_POINTS, true);
        GHResponse finalRsp = hopper.route(finalReq);
        if (finalRsp.hasErrors()) {
            LOGGER.warn("autoroute: final route of a converted loop failed, showing the via-route: {}", finalRsp.getErrors());
            return null;
        }
        ResponsePath f = finalRsp.getBest();
        ResponsePath out = new ResponsePath();
        out.setWaypoints(wps);
        out.setPoints(f.getPoints());
        out.setDistance(f.getDistance());
        out.setTime(f.getTime());
        out.setAscend(f.getAscend());
        out.setDescend(f.getDescend());
        out.setRouteWeight(f.getRouteWeight());
        out.addPathDetails(f.getPathDetails());
        return out;
    }

    /** One stretch of the loop outside the corridor: positions along the loop in metres. */
    static final class Stretch {
        final double fromM, toM;

        Stretch(double fromM, double toM) {
            this.fromM = fromM;
            this.toM = toM;
        }
    }

    static final class Divergence {
        /** the longest stretch of at least MIN_DIVERGENCE_M, or null */
        Stretch longest;
        /** metres of the loop outside the corridor, in stretches of at least MIN_DIVERGENCE_M */
        double offM;
    }

    /** Where the loop ([lat, lon], in order) leaves the corridor of the final route. */
    static Divergence divergence(List<double[]> loop, PointList fin) {
        Proj proj = new Proj(loop.get(0)[0], loop.get(0)[1]);
        SegmentGrid grid = new SegmentGrid(proj, fin);
        Divergence d = new Divergence();
        double runFrom = -1, last = 0;
        for (double[] s : samples(proj, loop)) {
            boolean off = grid.distance(s[0], s[1]) > CORRIDOR_M;
            if (off && runFrom < 0) runFrom = s[2];
            if (!off && runFrom >= 0) {
                close(d, runFrom, s[2]);
                runFrom = -1;
            }
            last = s[2];
        }
        if (runFrom >= 0) close(d, runFrom, last + STEP_M);
        return d;
    }

    private static void close(Divergence d, double from, double to) {
        if (to - from < MIN_DIVERGENCE_M) return;
        d.offM += to - from;
        if (d.longest == null || to - from > d.longest.toM - d.longest.fromM) d.longest = new Stretch(from, to);
    }

    /** Points every STEP_M along the loop: x, y (metres) and the position along the loop. */
    static List<double[]> samples(Proj proj, List<double[]> loop) {
        List<double[]> out = new ArrayList<>();
        double pos = 0, next = 0;
        double[] prev = proj.xy(loop.get(0));
        for (int i = 1; i < loop.size(); i++) {
            double[] cur = proj.xy(loop.get(i));
            double len = Math.hypot(cur[0] - prev[0], cur[1] - prev[1]);
            while (next <= pos + len) {
                double t = len == 0 ? 0 : (next - pos) / len;
                out.add(new double[]{prev[0] + t * (cur[0] - prev[0]), prev[1] + t * (cur[1] - prev[1]), next});
                next += STEP_M;
            }
            pos += len;
            prev = cur;
        }
        return out;
    }

    /**
     * Adds the loop's point in the middle of the stretch to the waypoints, between the waypoints around it along the
     * loop. The point lies inside a road segment of the loop, so it snaps onto the loop's own edge.
     */
    static void insert(List<double[]> waypoints, List<double[]> loop, Stretch s) {
        Proj proj = new Proj(loop.get(0)[0], loop.get(0)[1]);
        double mid = (s.fromM + s.toM) / 2;
        double[] at = pointAt(proj, loop, mid);
        double[] positions = positions(proj, loop, waypoints);
        int k = 0;
        while (k < waypoints.size() && positions[k] < mid) k++;
        waypoints.add(k, at);
    }

    /** The [lat, lon] point at this position along the loop. */
    static double[] pointAt(Proj proj, List<double[]> loop, double at) {
        double pos = 0;
        for (int i = 1; i < loop.size(); i++) {
            double[] a = loop.get(i - 1), b = loop.get(i);
            double[] pa = proj.xy(a), pb = proj.xy(b);
            double len = Math.hypot(pb[0] - pa[0], pb[1] - pa[1]);
            if (pos + len >= at && len > 0) {
                double t = (at - pos) / len;
                return new double[]{a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])};
            }
            pos += len;
        }
        return loop.get(loop.size() - 1).clone();
    }

    /**
     * Each waypoint's position along the loop. The waypoints lie on the loop in order, so each is matched from the
     * previous one's position onwards: the first loop segment within a metre, else the nearest one ahead.
     */
    static double[] positions(Proj proj, List<double[]> loop, List<double[]> waypoints) {
        int n = loop.size();
        double[][] xy = new double[n][];
        double[] cum = new double[n];
        for (int i = 0; i < n; i++) {
            xy[i] = proj.xy(loop.get(i));
            if (i > 0) cum[i] = cum[i - 1] + Math.hypot(xy[i][0] - xy[i - 1][0], xy[i][1] - xy[i - 1][1]);
        }
        double[] out = new double[waypoints.size()];
        int from = 1;
        double after = -1;
        for (int k = 0; k < waypoints.size(); k++) {
            double[] p = proj.xy(waypoints.get(k));
            double bestDist = Double.MAX_VALUE, bestPos = cum[n - 1];
            int bestSeg = from;
            for (int i = from; i < n; i++) {
                double[] r = segment(p, xy[i - 1], xy[i]);
                double pos = cum[i - 1] + r[1] * (cum[i] - cum[i - 1]);
                // strictly past the previous waypoint: on a road the loop rides twice, the next pass
                if (pos <= after + 1) continue;
                if (r[0] < bestDist) {
                    bestDist = r[0];
                    bestPos = pos;
                    bestSeg = i;
                }
                if (r[0] < 1) break;
            }
            out[k] = bestPos;
            after = bestPos;
            from = bestSeg;
        }
        return out;
    }

    /** Distance from p to segment a-b and the position t (0-1) of the nearest point on it. */
    static double[] segment(double[] p, double[] a, double[] b) {
        double dx = b[0] - a[0], dy = b[1] - a[1];
        double len2 = dx * dx + dy * dy;
        double t = len2 == 0 ? 0 : Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2));
        return new double[]{Math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy), t};
    }

    /** Equirectangular metres around a reference point (loops span at most ~60 km). */
    static final class Proj {
        final double lat0, lon0, kx;
        static final double KY = 111_320;

        Proj(double lat0, double lon0) {
            this.lat0 = lat0;
            this.lon0 = lon0;
            this.kx = KY * Math.cos(Math.toRadians(lat0));
        }

        double[] xy(double[] latLon) {
            return xy(latLon[0], latLon[1]);
        }

        double[] xy(double lat, double lon) {
            return new double[]{(lon - lon0) * kx, (lat - lat0) * KY};
        }
    }

    /** The final route's segments in 100 m cells, for nearest-distance queries within the corridor. */
    static final class SegmentGrid {
        static final double CELL = 100;
        final Map<Long, List<double[]>> cells = new HashMap<>();

        SegmentGrid(Proj proj, PointList pl) {
            double[] prev = null;
            for (int i = 0; i < pl.size(); i++) {
                double[] cur = proj.xy(pl.getLat(i), pl.getLon(i));
                if (prev != null) {
                    double[] seg = {prev[0], prev[1], cur[0], cur[1]};
                    long x0 = cell(Math.min(prev[0], cur[0]) - CORRIDOR_M), x1 = cell(Math.max(prev[0], cur[0]) + CORRIDOR_M);
                    long y0 = cell(Math.min(prev[1], cur[1]) - CORRIDOR_M), y1 = cell(Math.max(prev[1], cur[1]) + CORRIDOR_M);
                    for (long x = x0; x <= x1; x++)
                        for (long y = y0; y <= y1; y++)
                            cells.computeIfAbsent(key(x, y), k -> new ArrayList<>()).add(seg);
                }
                prev = cur;
            }
        }

        /** The distance to the nearest segment, exact up to CORRIDOR_M; beyond it only "more than CORRIDOR_M". */
        double distance(double x, double y) {
            List<double[]> segs = cells.get(key(cell(x), cell(y)));
            if (segs == null) return Double.MAX_VALUE;
            double best = Double.MAX_VALUE;
            double[] p = {x, y};
            for (double[] s : segs)
                best = Math.min(best, segment(p, new double[]{s[0], s[1]}, new double[]{s[2], s[3]})[0]);
            return best;
        }

        static long cell(double v) {
            return (long) Math.floor(v / CELL);
        }

        static long key(long x, long y) {
            return (x << 32) ^ (y & 0xffffffffL);
        }
    }
}
