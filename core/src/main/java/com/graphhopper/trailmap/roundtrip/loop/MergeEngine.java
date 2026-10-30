package com.graphhopper.trailmap.roundtrip.loop;

import com.graphhopper.GraphHopper;
import com.graphhopper.util.CustomModel;

import java.util.*;

/**
 * AutoRoute round 3, candidate m: one engine that merges a2 (shaped, joined holes: variety) and c2 (the client's
 * templates as a band: availability and heading via templates). Prototype, test package only.
 * <ol>
 *   <li>Both sources run on the same start, length, heading and generation seed: a2's {@link MultiHoleEngine} draws
 *       its shape configurations and keeps every feasible loop (its pool); c2's {@link BandLoopEngine} runs the
 *       client's generation loop and keeps every raw candidate.</li>
 *   <li>One pool: every loop within {@code showTol} of the requested length.</li>
 *   <li>Pick 3, greedily: each next route maximises heading score + {@code distinctWeight} x distinctness, where
 *       heading = the centre of mass's angle off the requested heading (1 up to 30 deg, 0 at 120 deg) and
 *       distinctness = 1 - the largest corridor similarity (share within 50 m, both ways) to the routes already
 *       picked.</li>
 *   <li>Each picked loop gets its own source's /convert-faithful cover (both sources use the same one).</li>
 * </ol>
 * {@code m.sources=a2|c2} with {@code m.select=source} runs one source with its own pick, unchanged (copy gate).
 */
public class MergeEngine {

    public static final class Params {
        // production: the round-4 area16 settings, fixed (prototype flags in comments)
        String sources = "a2,c2";                   // m.sources
        String select = "merged";                   // m.select: merged | source
        double showTol = 0.12;
        double distinctWeight = 0.5;
        double maxSim = 1;                          // i2: distinctness floor
        double fullDeg = 30, zeroDeg = 120, minCentreFrac = 0.1;
        double stepM = 25, corridorM = 50;
        // m4 (round 4), all off by default = m unchanged:
        //   memStrength: the session's shown ways cost 1 + memStrength x their share in both loop searches
        //   newWeight:   the pick adds newWeight x the share of the route on ways not shown in this session
        //   roles:       slots 1-2 follow the heading, slot 3 is the "surprise": heading ignored, most new wins
        //   memFreeM:    ways this close to the start are never remembered (every route leaves home the same way)
        // production = area16: memStrength 1, newWeight 1, roles on, memZ 16, guidedMaxOff 45
        double memStrength = 1;
        double newWeight = 1;
        boolean roles = true;
        double memFreeM = 1000;
        //   memZ:        zoom of the search's memory cells (19 ~37 m = the ways themselves; 16 ~300 m = whole areas)
        //   guidedMaxOff: a guided slot takes a route more than this off the heading only when nothing else is left
        int memZ = 16;
        double guidedMaxOff = 45;

        boolean session() { return memStrength > 0 || newWeight > 0 || roles; }

        boolean a2() { return sources.contains("a2"); }

        boolean c2() { return sources.contains("c2"); }

        boolean merged() { return select.equals("merged"); }

        @Override
        public String toString() {
            return "sources=" + sources + " select=" + select + " showTol=" + showTol + " distinctWeight=" + distinctWeight
                    + " maxSim=" + maxSim
                    + (session() ? " m4: memStrength=" + memStrength + " newWeight=" + newWeight + " roles=" + roles + " memFreeM=" + memFreeM
                    + " memZ=" + memZ + " guidedMaxOff=" + guidedMaxOff : "")
                    + " heading ramp " + fullDeg + "-" + zeroDeg + "deg corridor=" + corridorM + "m step=" + stepM;
        }
    }

    /** One shown route, whichever source made it. */
    static final class Route {
        String id, source, label;
        double distance;
        List<double[]> coords;
        List<double[]> waypoints;
        MultiHoleEngine.Loop a2;
        BandLoopEngine.Candidate c2;
        double headingScore = Double.NaN, offDeg = Double.NaN;
        double newShare = 1;
        String role = "";
        Prepared prep;
    }

    final Params params = new Params();
    final MultiHoleEngine holes;
    final BandLoopEngine band;

    public MergeEngine(GraphHopper hopper, String profile, CustomModel requestModel) {
        holes = new MultiHoleEngine(hopper, profile + "_explore", requestModel);
        band = new BandLoopEngine(hopper, profile, requestModel);
    }

    List<Route> generate(MultiHoleEngine.LocalGraph lg, Topology topo, double lat, double lon, double targetM,
                         double heading, long seed) {
        return generate(lg, topo, lat, lon, targetM, heading, seed, null);
    }

    /** lg / topo may be null (start not snapped by that source). memory = the session's shown ways (m4), or null. */
    List<Route> generate(MultiHoleEngine.LocalGraph lg, Topology topo, double lat, double lon, double targetM,
                         double heading, long seed, Memory memory) {
        List<Route> pool = new ArrayList<>();
        boolean merged = params.merged();
        Memory mem = merged && memory != null && !memory.isEmpty() ? memory : null;
        holes.memory = params.memStrength > 0 ? mem : null;
        holes.memStrength = params.memStrength;
        band.memory = params.memStrength > 0 ? mem : null;
        band.memStrength = params.memStrength;
        if (params.a2() && lg != null) {
            holes.skipPick = merged;
            List<MultiHoleEngine.Loop> loops = holes.generate(lg, targetM, Double.isNaN(heading) ? 0 : heading, seed);
            List<MultiHoleEngine.Loop> from = merged ? holes.lastPool : loops;
            int i = 0;
            for (MultiHoleEngine.Loop l : from) {
                String id = "a" + i++;
                if (l.failure != null) continue;
                Route r = new Route();
                r.id = id;
                r.source = "a2";
                r.label = l.family + " " + Math.round(l.bearingDeg) + "°";
                r.distance = l.distanceM;
                r.coords = l.coords;
                r.waypoints = l.waypoints;
                r.a2 = l;
                pool.add(r);
            }
        }
        if (params.c2()) {
            band.topo = topo;
            band.skipCover = merged;
            band.stats.reset();
            BandLoopEngine.Generation gen = band.generate(lat, lon, targetM, heading, seed);
            for (BandLoopEngine.Candidate c : merged ? gen.raw : gen.shown) {
                Route r = new Route();
                r.id = c.id;
                r.source = "c2";
                r.label = c.shape;
                r.distance = c.distance;
                r.coords = c.coords;
                r.waypoints = c.waypoints;
                r.c2 = c;
                pool.add(r);
            }
        }
        if (!merged) return pool;

        List<Route> ok = new ArrayList<>();
        for (Route r : pool) if (Math.abs(r.distance / targetM - 1) <= params.showTol) ok.add(r);
        for (Route r : ok) {
            r.prep = prepare(r.coords, lat, lon);
            heading(r, heading);
            if (mem != null) r.newShare = mem.newShare(r.coords);
        }
        List<Route> out = new ArrayList<>();
        while (out.size() < 3) {
            // m4 roles: the third slot is the surprise, heading left out of its score
            boolean surprise = params.roles && out.size() == 2;
            // i2: a route sharing more than maxSim of its corridor with a pick is taken only when nothing else is left
            Route pick = null;
            int pickTier = -1;
            double bestScore = -1;
            for (Route r : ok) {
                if (out.contains(r)) continue;
                double maxSim = 0;
                for (Route o : out) maxSim = Math.max(maxSim, similarity(r.prep, o.prep));
                boolean distinct = maxSim <= params.maxSim;
                // m4: a guided slot prefers routes within guidedMaxOff of the heading (all are, by default)
                boolean inCone = surprise || Double.isNaN(r.offDeg) || r.offDeg <= params.guidedMaxOff;
                int tier = (inCone ? 2 : 0) + (distinct ? 1 : 0);
                double h = surprise ? 0 : Double.isNaN(r.headingScore) ? 0.5 : r.headingScore;
                double score = h + params.distinctWeight * (1 - maxSim) + params.newWeight * r.newShare;
                if (tier > pickTier || (tier == pickTier && score > bestScore)) {
                    bestScore = score;
                    pick = r;
                    pickTier = tier;
                }
            }
            if (pick == null) break;
            if (params.roles) pick.role = surprise ? "surprise" : "guided";
            out.add(pick);
        }
        for (Route r : out) {
            if (r.a2 != null) {
                holes.coverWaypoints(lg, r.a2);
                r.waypoints = r.a2.waypoints;
            } else {
                band.coverWaypoints(topo, r.c2);
                r.waypoints = r.c2.waypoints;
            }
        }
        return out;
    }

    // ------------------------------------------------------------------ heading and corridor similarity
    // mirrors gh-autoroute quality.mjs headingMeasures and variety.mjs corridor similarity

    static final class Prepared {
        double[] x, y;
        Map<Long, List<Integer>> index;
        double cellM;
    }

    Prepared prepare(List<double[]> coords, double lat0, double lon0) {
        double mPerLat = 110_540, mPerLon = 111_320 * Math.cos(Math.toRadians(lat0));
        List<double[]> pts = new ArrayList<>();
        double carry = 0;
        double[] prev = null;
        for (double[] c : coords) {
            double[] p = {(c[1] - lon0) * mPerLon, (c[0] - lat0) * mPerLat};
            if (prev == null) {
                pts.add(p);
            } else {
                double seg = Math.hypot(p[0] - prev[0], p[1] - prev[1]);
                double t = params.stepM - carry;
                while (t <= seg) {
                    pts.add(new double[]{prev[0] + (p[0] - prev[0]) * t / seg, prev[1] + (p[1] - prev[1]) * t / seg});
                    t += params.stepM;
                }
                carry = seg - (t - params.stepM);
            }
            prev = p;
        }
        Prepared pr = new Prepared();
        pr.cellM = params.corridorM;
        pr.x = new double[pts.size()];
        pr.y = new double[pts.size()];
        for (int i = 0; i < pts.size(); i++) {
            pr.x[i] = pts.get(i)[0];
            pr.y[i] = pts.get(i)[1];
        }
        pr.index = new HashMap<>();
        for (int i = 0; i < pr.x.length; i++)
            pr.index.computeIfAbsent(cellKey(pr.x[i], pr.y[i], pr.cellM), k -> new ArrayList<>()).add(i);
        return pr;
    }

    static long cellKey(double x, double y, double cellM) {
        return (long) Math.floor(x / cellM) * 1_000_003L + (long) Math.floor(y / cellM);
    }

    double coverage(Prepared a, Prepared b) {
        if (a.x.length == 0) return 0;
        double r2 = params.corridorM * params.corridorM;
        int hit = 0;
        for (int i = 0; i < a.x.length; i++) {
            long cx = (long) Math.floor(a.x[i] / b.cellM), cy = (long) Math.floor(a.y[i] / b.cellM);
            boolean found = false;
            for (long dx = -1; dx <= 1 && !found; dx++)
                for (long dy = -1; dy <= 1 && !found; dy++) {
                    List<Integer> l = b.index.get((cx + dx) * 1_000_003L + (cy + dy));
                    if (l == null) continue;
                    for (int k : l) {
                        double ddx = b.x[k] - a.x[i], ddy = b.y[k] - a.y[i];
                        if (ddx * ddx + ddy * ddy <= r2) { found = true; break; }
                    }
                }
            if (found) hit++;
        }
        return (double) hit / a.x.length;
    }

    double similarity(Prepared a, Prepared b) {
        return (coverage(a, b) + coverage(b, a)) / 2;
    }

    void heading(Route r, double headingDeg) {
        Prepared p = r.prep;
        if (Double.isNaN(headingDeg) || p.x.length == 0) return;
        double cx = 0, cy = 0, maxR = 0;
        for (int i = 0; i < p.x.length; i++) {
            cx += p.x[i];
            cy += p.y[i];
            maxR = Math.max(maxR, Math.hypot(p.x[i], p.y[i]));
        }
        cx /= p.x.length;
        cy /= p.x.length;
        if (maxR <= 0 || Math.hypot(cx, cy) / maxR < params.minCentreFrac) return;
        double bearing = (Math.toDegrees(Math.atan2(cx, cy)) % 360 + 360) % 360;
        double d = Math.abs(((bearing - headingDeg) % 360 + 360) % 360);
        double off = d > 180 ? 360 - d : d;
        r.offDeg = off;
        r.headingScore = off <= params.fullDeg ? 1 : off >= params.zeroDeg ? 0 : 1 - (off - params.fullDeg) / (params.zeroDeg - params.fullDeg);
    }
}
