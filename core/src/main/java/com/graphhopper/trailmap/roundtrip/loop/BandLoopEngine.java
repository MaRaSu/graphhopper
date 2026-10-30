package com.graphhopper.trailmap.roundtrip.loop;

import com.graphhopper.GraphHopper;
import com.graphhopper.config.Profile;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.trailmap.roundtrip.geometry.*;
import com.graphhopper.util.CustomModel;
import com.graphhopper.util.PMap;
import com.graphhopper.util.shapes.GHPoint;

import java.util.*;

/**
 * AutoRoute round 2, candidate c2: C's templates as a band + A's one-loop winding search (prototype, test
 * package only).
 * <ol>
 *   <li>The web client's generation loop (3 slots x 3 draws, retries, cross-slot fallback, mulberry32 in the
 *       client's order), copied from round-1 C. Figure-8 and hourglass draws become one-loop shapes (C).</li>
 *   <li>Each draw: the template's corners (production geometry) are moved by C's placement (main 2-edge block,
 *       network-distance band, water rotation). The corner polygon is the band's centre line.</li>
 *   <li>The band: edges near the outline ride at their own weight, edges farther out cost {@code outPenalty}x,
 *       edges beyond {@code outBlockFactor} band widths are removed, and the polygon's deep interior is a hole
 *       (removed).</li>
 *   <li>A's parity Dijkstra (copied): the cheapest ride from the start back to the start that winds once around
 *       the hole centre. A walk that winds once cannot be an out-and-back, and a figure-8 or a mini-loop costs
 *       more than the simple loop, so the shortest answer is one loop.</li>
 *   <li>Length: the template is rescaled by the loop's own length (C's resize), at most {@code rescaleMax} times.</li>
 *   <li>Waypoints: A's shortest-path cover on the true weight, so /convert reproduces the loop.</li>
 * </ol>
 * Weights are the exploration profile's (what /convert re-routes legs with), without turn costs.
 */
public class BandLoopEngine {

    public static final class Params {
        // C's placement
        // production: the round-4 area16 settings, fixed (prototype flags in comments)
        double bandWidth = 0.2;                     // c2.cornerBand
        double factor = 0.75;                       // c2.factor
        double radiusFactor = 0.5;
        // the band around the template outline
        double holeFrac = 0.5;                      // hole = interior deeper than holeFrac * max depth
        double bandFrac = 0.5;                      // band half width outside = bandFrac * max depth
        double minBandM = 1500;
        double outPenalty = 3;
        double outBlockFactor = 3;                  // c2.outBlock
        double minDepthM = 300;
        // i2: hole centre on land (a 1 km cell with an allowed node), and rotate the template when no loop exists
        boolean landCentre = false;
        double[] rotations = {0, 30, -30, 60, -60, 90, -90};
        // i3: reject loops riding more than this many metres twice (-1 = off); the cover replays /convert's
        // avoid-edges penalty on the loop edges already passed
        double maxReuseM = 300;
        boolean coverAvoid = true;
        double avoidFactor = 5.0;
        // length
        double rescaleTol = 0.05;
        int rescaleMax = 3;
        // A's variation noise and waypoint cover
        double noise = 0.3;
        double noiseCellM = 2000;
        double coverSlack = 1e-3;

        @Override
        public String toString() {
            return "cornerBand=+-" + bandWidth + " factor=" + factor + " hole>" + holeFrac + "*depth band=max(" + minBandM
                    + "m," + bandFrac + "*depth) out x" + outPenalty + " block>" + outBlockFactor + "*band rescale(tol "
                    + rescaleTol + ", max " + rescaleMax + ") noise=+-" + noise + "/" + noiseCellM + "m cover slack=" + coverSlack
                    + " landCentre=" + landCentre + " rotations=" + Arrays.toString(rotations)
                    + " maxReuseM=" + maxReuseM + " coverAvoid=" + coverAvoid;
        }
    }

    static double[] parseList(String s) {
        String[] parts = s.split(",");
        double[] out = new double[parts.length];
        for (int i = 0; i < parts.length; i++) out[i] = Double.parseDouble(parts[i].trim());
        return out;
    }

    // client constants (client-replay.mjs CLIENT, SHAPES), as round-1 C
    static final int NUM_CANDIDATES = 3, RAW_PER_SLOT = 3, MAX_SLOT_RETRIES = 2, RETRY_COUNT = 2, VIA_POINT_COUNT = 3;
    static final double LENGTH_TOLERANCE = 0.2;
    static final int[] EXTRA_VIA_WEIGHTS = {4, 3, 2, 1};
    static final String[] SHAPES = {"circle", "diamond", "rectangle", "petal", "figure8", "teardrop", "serpentine", "hourglass"};
    static final double[] SHAPE_W = {1.0, 1.0, 1.0, 1.0, 0.6, 1.0, 0.6, 1.0};
    static final String[] ONE_LOOP_SHAPES = {"circle", "diamond", "rectangle", "petal", "teardrop", "serpentine"};
    static final int MAX_ATTEMPTS = 3;

    final Params params = new Params();
    private final Weighting exploreWeighting;
    private final Map<String, TourGeometryStrategy> geometryStrategies = new LinkedHashMap<>();

    // per generation
    Topology topo;
    final Stats stats = new Stats();
    // m: the caller picks from gen.raw and covers only what it shows
    boolean skipCover;
    // m4: set by the caller per press; null / 0 = off (c2 i5 unchanged)
    Memory memory;
    double memStrength;
    private double[] memMult;

    static final class Stats {
        int calls, solves, noLoop, flat, rescales, reuse;

        void reset() { calls = solves = noLoop = flat = rescales = reuse = 0; }

        @Override
        public String toString() {
            return "solves=" + solves + " noLoop=" + noLoop + " flat=" + flat + " reuse=" + reuse + " rescales=" + rescales;
        }
    }

    static final class Candidate {
        String id, shape;
        int slot, callIndex, points, seed;
        double distance, requestDistance;
        List<double[]> waypoints = new ArrayList<>();   // lat, lon, without start/end
        List<double[]> coords = new ArrayList<>();
        int[] arcs;
    }

    static final class Call {
        int slot, retry, points, seed;
        String shape, drawnShape, error;
        boolean ok;
        long ms;
    }

    static final class Generation {
        List<Call> calls = new ArrayList<>();
        List<Candidate> raw = new ArrayList<>();
        List<Candidate> shown = new ArrayList<>();
    }

    public BandLoopEngine(GraphHopper hopper, String profileName, CustomModel requestModel) {
        Profile explore = hopper.getProfile(profileName + "_explore");
        PMap hints = new PMap();
        hints.putObject(CustomModel.KEY, requestModel);
        this.exploreWeighting = hopper.createWeighting(explore, hints, false);
        geometryStrategies.put("circle", new CircleTourGeometry());
        geometryStrategies.put("diamond", new DiamondTourGeometry(2.0));
        geometryStrategies.put("rectangle", new RectangleTourGeometry(5.0));
        geometryStrategies.put("petal", new PetalTourGeometry());
        geometryStrategies.put("figure8", new Figure8TourGeometry());
        geometryStrategies.put("teardrop", new TeardropTourGeometry());
        geometryStrategies.put("serpentine", new SerpentineTourGeometry());
        geometryStrategies.put("hourglass", new HourglassTourGeometry());
    }

    Weighting exploreWeighting() {
        return exploreWeighting;
    }

    // ------------------------------------------------------------------ client generation loop (copy of C)

    Generation generate(double lat, double lon, double targetM, double heading, long genSeed) {
        Mulberry32 rand = new Mulberry32(genSeed);
        double apiLength = targetM * params.factor;
        final Topology t = topo;
        memMult = memory == null || t == null ? null : memory.edgeMult(t.m, t.gStart, t.gx, t.gy, new Memory.Frame() {
            public double lat(double y) { return t.lat(y); }

            public double lon(double x) { return t.lon(x); }
        }, memStrength);
        Generation gen = new Generation();
        Set<Candidate> selected = new HashSet<>();
        int unfilled = 0;
        for (int slot = 0; slot < NUM_CANDIDATES; slot++) {
            List<Candidate> pool = new ArrayList<>();
            int retry = 0;
            Candidate best;
            while (true) {
                int batch = retry == 0 ? RAW_PER_SLOT : RETRY_COUNT;
                for (int j = 0; j < batch; j++) {
                    String drawn = pickShape(rand);
                    int points = VIA_POINT_COUNT + pickExtraViaPoints(rand);
                    int seed = (int) Math.floor(rand.next() * 10000);
                    String shape = drawn;
                    if (drawn.equals("figure8") || drawn.equals("hourglass"))
                        shape = ONE_LOOP_SHAPES[seed % ONE_LOOP_SHAPES.length];
                    Call call = new Call();
                    call.slot = slot;
                    call.retry = retry;
                    call.shape = shape;
                    call.drawnShape = drawn;
                    call.points = points;
                    call.seed = seed;
                    gen.calls.add(call);
                    long t0 = System.nanoTime();
                    Candidate c = null;
                    try {
                        c = bandLoop(apiLength, targetM, heading, seed, shape, lat, lon);
                    } catch (Exception e) {
                        call.error = e.toString();
                    }
                    call.ms = (System.nanoTime() - t0) / 1_000_000;
                    call.ok = c != null;
                    if (c == null) {
                        if (call.error == null) call.error = "no loop in the band";
                        continue;
                    }
                    c.id = "s" + slot + "c" + (gen.calls.size() - 1);
                    c.callIndex = gen.calls.size() - 1;
                    c.slot = slot;
                    c.shape = shape;
                    c.points = points;
                    c.seed = seed;
                    pool.add(c);
                }
                best = closest(pool, targetM);
                if (best == null) break;
                if (Math.abs(best.distance - targetM) / targetM <= LENGTH_TOLERANCE) break;
                retry++;
                if (retry >= MAX_SLOT_RETRIES) break;
            }
            gen.raw.addAll(pool);
            if (best != null) {
                gen.shown.add(best);
                selected.add(best);
            } else unfilled++;
        }
        for (int i = 0; i < unfilled; i++) {
            List<Candidate> pool = new ArrayList<>();
            for (Candidate c : gen.raw) if (!selected.contains(c)) pool.add(c);
            if (pool.isEmpty()) break;
            Candidate fb = closest(pool, targetM);
            gen.shown.add(fb);
            selected.add(fb);
        }
        if (!skipCover) for (Candidate c : gen.shown) coverWaypoints(topo, c);
        return gen;
    }

    private static Candidate closest(List<Candidate> pool, double targetM) {
        Candidate best = null;
        for (Candidate c : pool)
            if (best == null || Math.abs(c.distance - targetM) < Math.abs(best.distance - targetM)) best = c;
        return best;
    }

    static String pickShape(Mulberry32 rand) {
        double total = 0;
        for (double w : SHAPE_W) total += w;
        double r = rand.next() * total;
        for (int i = 0; i < SHAPES.length; i++) {
            r -= SHAPE_W[i];
            if (r <= 0) return SHAPES[i];
        }
        return SHAPES[SHAPES.length - 1];
    }

    static int pickExtraViaPoints(Mulberry32 rand) {
        int sum = 0;
        for (int w : EXTRA_VIA_WEIGHTS) sum += w;
        double r = rand.next() * sum;
        for (int i = 0; i < EXTRA_VIA_WEIGHTS.length; i++) {
            r -= EXTRA_VIA_WEIGHTS[i];
            if (r <= 0) return i;
        }
        return 0;
    }

    // ------------------------------------------------------------------ c2: template band + wind-once loop

    /** One draw: template -> C's corners -> band -> parity loop, rescaled by its own length. Null = no loop. */
    Candidate bandLoop(double distance, double targetM, double heading, long seed, String shapeName, double lat, double lon) {
        if (topo == null) return null;
        TourGeometryStrategy geometry = geometryStrategies.getOrDefault(shapeName, geometryStrategies.get("circle"));
        GHPoint start = new GHPoint(lat, lon);
        double d = distance;
        Candidate best = null;
        for (int round = 0; round <= params.rescaleMax; round++) {
            Candidate c = null;
            for (double rot : params.rotations) {
                double h = Double.isNaN(heading) ? heading : heading + rot;
                for (int attempt = 0; attempt < MAX_ATTEMPTS && c == null; attempt++) {
                    List<GHPoint> wps = geometry.generateWaypoints(start, d, h, seed + attempt);
                    c = solve(placeCorners(wps), seed + attempt);
                }
                if (c != null) break;
            }
            if (c == null) return best;
            c.requestDistance = d;
            if (best == null || Math.abs(c.distance - targetM) < Math.abs(best.distance - targetM)) best = c;
            double ratio = c.distance / targetM;
            if (Math.abs(ratio - 1) <= params.rescaleTol) break;
            d = d / ratio;
            stats.rescales++;
        }
        return best;
    }

    /** C's corner placement (mask + water + network-distance band), copied; returns projected polygon x,y. */
    double[][] placeCorners(List<GHPoint> wps) {
        Topology t = topo;
        double detour = Double.NaN;
        double[][] poly = new double[wps.size() - 1][];
        poly[0] = new double[]{t.nx[t.startLocal], t.ny[t.startLocal]};
        for (int i = 1; i < wps.size() - 1; i++) {
            GHPoint p = wps.get(i);
            double px = t.x(p.getLon()), py = t.y(p.getLat());
            double chord = Math.hypot(px, py);
            if (chord >= 300) {
                if (!t.occupied(p.getLat(), p.getLon())) {
                    double ang = Math.atan2(px, py);
                    for (int k = 1; k <= 6; k++) {
                        boolean found = false;
                        for (int sgn = -1; sgn <= 1; sgn += 2) {
                            double a = ang + sgn * Math.toRadians(10 * k);
                            double qx = chord * Math.sin(a), qy = chord * Math.cos(a);
                            if (t.occupiedCells.contains(t.cell(qx, qy))) {
                                px = qx;
                                py = qy;
                                found = true;
                                break;
                            }
                        }
                        if (found) break;
                    }
                }
                if (Double.isNaN(detour)) detour = t.detourRatio(0.5 * chord, 1.5 * chord);
                int v = t.nearestInBand(px, py, chord * detour, params.bandWidth, Math.max(2000, 0.5 * chord), 0);
                if (v >= 0) {
                    px = t.nx[v];
                    py = t.ny[v];
                }
            }
            poly[i] = new double[]{px, py};
        }
        return poly;
    }

    /** A's parity Dijkstra inside the band around the closed polygon {@code poly} (vertex 0 = the start). */
    Candidate solve(double[][] poly, long seed) {
        Topology t = topo;
        stats.solves++;
        int k = poly.length;
        // hole centre: the polygon's deepest interior point (grid sample)
        double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE, minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        for (double[] p : poly) {
            minX = Math.min(minX, p[0]);
            maxX = Math.max(maxX, p[0]);
            minY = Math.min(minY, p[1]);
            maxY = Math.max(maxY, p[1]);
        }
        double cx = 0, cy = 0, depth = -1;
        int gridN = 40;
        for (int i = 0; i <= gridN; i++)
            for (int j = 0; j <= gridN; j++) {
                double x = minX + (maxX - minX) * i / gridN, y = minY + (maxY - minY) * j / gridN;
                if (!inside(poly, x, y)) continue;
                if (params.landCentre && !t.occupiedCells.contains(t.cell(x, y))) continue;
                double dd = outlineDist(poly, x, y);
                if (dd > depth) {
                    depth = dd;
                    cx = x;
                    cy = y;
                }
            }
        if (depth < params.minDepthM) {
            stats.flat++;
            return null;
        }
        double holeD = params.holeFrac * depth;
        double band = Math.max(params.minBandM, params.bandFrac * depth);
        double blockD = params.outBlockFactor * band;
        double bx0 = minX - blockD, bx1 = maxX + blockD, by0 = minY - blockD, by1 = maxY + blockD;

        // ray from the hole centre, pointing away from the start
        double rx = cx - poly[0][0], ry = cy - poly[0][1], rl = Math.hypot(rx, ry);
        double ux = rx / rl, uy = ry / rl;

        int m = t.m;
        boolean[] blocked = new boolean[m];
        boolean[] flip = new boolean[m];
        double[] mult = new double[m];
        for (int e = 0; e < m; e++) {
            int from = t.gStart[e], to = t.gStart[e + 1] - 1;
            boolean outBox = false;
            for (int i = from; i <= to; i++)
                if (t.gx[i] < bx0 || t.gx[i] > bx1 || t.gy[i] < by0 || t.gy[i] > by1) { outBox = true; break; }
            if (outBox || t.hard4[e]) { blocked[e] = true; continue; }
            boolean hole = false;
            double maxOut = 0;
            for (int i = from; i <= to; i++) {
                double dd = outlineDist(poly, t.gx[i], t.gy[i]);
                boolean in = inside(poly, t.gx[i], t.gy[i]);
                // inset hole (i1); with a land centre, a disk around it, so a lake elsewhere in the polygon may stay
                // on either side of the loop
                if (params.landCentre ? Math.hypot(t.gx[i] - cx, t.gy[i] - cy) < holeD : in && dd > holeD) { hole = true; break; }
                if (!in) maxOut = Math.max(maxOut, dd);
            }
            if (hole || maxOut > blockD) { blocked[e] = true; continue; }
            mult[e] = maxOut > band ? params.outPenalty : 1;
            if (memMult != null) mult[e] *= memMult[e];
            int crossings = 0;
            for (int i = from; i < to; i++) {
                double ax = t.gx[i] - cx, ay = t.gy[i] - cy, bx = t.gx[i + 1] - cx, by = t.gy[i + 1] - cy;
                double sa = ux * ay - uy * ax, sb = ux * by - uy * bx;
                if ((sa <= 0 && sb > 0) || (sb <= 0 && sa > 0)) {
                    double ta = ax * ux + ay * uy, tb = bx * ux + by * uy;
                    if (ta + (tb - ta) * sa / (sa - sb) > 0) crossings++;
                }
            }
            flip[e] = (crossings & 1) == 1;
            if (params.noise > 0) {
                int mid = (from + to) / 2;
                long qx = (long) Math.floor(t.gx[mid] / params.noiseCellM), qy = (long) Math.floor(t.gy[mid] / params.noiseCellM);
                long h = seed * 0x9E3779B97F4A7C15L + qx * 0xC2B2AE3D27D4EB4FL + qy * 0x165667B19E3779F9L;
                h ^= h >>> 33;
                h *= 0xFF51AFD7ED558CCDL;
                h ^= h >>> 33;
                double u = (h >>> 11) / (double) (1L << 53);
                mult[e] *= 1 + params.noise * (2 * u - 1);
            }
        }

        int states = 2 * t.n;
        double[] best = new double[states];
        Arrays.fill(best, Double.POSITIVE_INFINITY);
        int[] parentEdge = new int[states];
        int[] parentState = new int[states];
        PriorityQueue<double[]> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> a[0]));
        int src = 2 * t.startLocal, dst = 2 * t.startLocal + 1;
        best[src] = 0;
        pq.add(new double[]{0, src});
        while (!pq.isEmpty()) {
            double[] top = pq.poll();
            int s = (int) top[1];
            if (top[0] > best[s]) continue;
            if (s == dst) break;
            int node = s >> 1, par = s & 1;
            for (int q = t.adjStart[node]; q < t.adjStart[node + 1]; q++) {
                int a = t.adjEdge[q];
                int e = a >= 0 ? a : ~a;
                if (blocked[e]) continue;
                double w = a >= 0 ? t.wF[e] : t.wB[e];
                if (!Double.isFinite(w)) continue;
                w *= mult[e];
                int other = a >= 0 ? t.ev[e] : t.eu[e];
                int tt = 2 * other + (flip[e] ? par ^ 1 : par);
                double nd = top[0] + w;
                if (nd < best[tt]) {
                    best[tt] = nd;
                    parentEdge[tt] = a;
                    parentState[tt] = s;
                    pq.add(new double[]{nd, tt});
                }
            }
        }
        if (!Double.isFinite(best[dst])) {
            stats.noLoop++;
            return null;
        }
        ArrayDeque<Integer> path = new ArrayDeque<>();
        int s = dst;
        do {
            path.push(parentEdge[s]);
            s = parentState[s];
        } while (s != src);
        Candidate c = new Candidate();
        double total = 0;
        for (int a : path) {
            int e = a >= 0 ? a : ~a;
            total += t.dist[e];
            int from = t.gStart[e], to = t.gStart[e + 1] - 1;
            if (a >= 0) for (int i = from; i <= to; i++) addCoord(c.coords, t.lat(t.gy[i]), t.lon(t.gx[i]));
            else for (int i = to; i >= from; i--) addCoord(c.coords, t.lat(t.gy[i]), t.lon(t.gx[i]));
        }
        c.distance = total;
        c.arcs = path.stream().mapToInt(Integer::intValue).toArray();
        if (params.maxReuseM >= 0) {
            // i3: a loop that rides an edge twice is an out-and-back stretch, and /convert's avoid-edges re-route
            // would detour around its second pass: not a loop
            double reused = 0;
            BitSet seen = new BitSet(t.m);
            for (int a : c.arcs) {
                int e = a >= 0 ? a : ~a;
                if (seen.get(e)) reused += t.dist[e];
                seen.set(e);
            }
            if (reused > params.maxReuseM) {
                stats.reuse++;
                return null;
            }
        }
        return c;
    }

    /** A's shortest-path cover (copied): every leg between two waypoints is a true-weight shortest path along the loop. */
    void coverWaypoints(Topology t, Candidate loop) {
        int k = loop.arcs.length;
        int[] nodes = new int[k + 1];
        double[] cum = new double[k + 1];
        nodes[0] = t.startLocal;
        for (int i = 0; i < k; i++) {
            int a = loop.arcs[i];
            int e = a >= 0 ? a : ~a;
            nodes[i + 1] = a >= 0 ? t.ev[e] : t.eu[e];
            cum[i + 1] = cum[i] + (a >= 0 ? t.wF[e] : t.wB[e]);
        }
        loop.waypoints.clear();
        int cur = 0;
        BitSet avoided = new BitSet(t.m);
        while (cur < k) {
            // /convert's Stage A routes every leg with the edges of the earlier legs penalised (AvoidEdgesWeighting)
            if (params.coverAvoid) {
                avoided.clear();
                for (int i = 0; i < cur; i++) avoided.set(loop.arcs[i] >= 0 ? loop.arcs[i] : ~loop.arcs[i]);
            }
            double[] d = dijkstra(t, nodes[cur], (cum[k] - cum[cur]) * (1 + params.coverSlack) + 1, avoided, params.avoidFactor);
            int j = cur + 1;
            while (j + 1 <= k && d[nodes[j + 1]] >= (cum[j + 1] - cum[cur]) * (1 - params.coverSlack) - 1e-6)
                j++;
            if (j == k) break;
            int a = loop.arcs[j - 1];
            int e = a >= 0 ? a : ~a;
            int from = t.gStart[e], to = t.gStart[e + 1] - 1;
            double mx, my;
            if (to - from >= 2) {
                int mid = (from + to) / 2;
                mx = t.gx[mid];
                my = t.gy[mid];
            } else {
                mx = (t.gx[from] + t.gx[to]) / 2;
                my = (t.gy[from] + t.gy[to]) / 2;
            }
            loop.waypoints.add(new double[]{t.lat(my), t.lon(mx)});
            cur = j - 1 > cur ? j - 1 : j;
        }
    }

    static double[] dijkstra(Topology t, int src, double maxWeight, BitSet avoided, double avoidFactor) {
        double[] best = new double[t.n];
        Arrays.fill(best, Double.POSITIVE_INFINITY);
        best[src] = 0;
        PriorityQueue<double[]> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> a[0]));
        pq.add(new double[]{0, src});
        while (!pq.isEmpty()) {
            double[] top = pq.poll();
            int u = (int) top[1];
            if (top[0] > best[u]) continue;
            if (top[0] > maxWeight) break;
            for (int q = t.adjStart[u]; q < t.adjStart[u + 1]; q++) {
                int a = t.adjEdge[q];
                int e = a >= 0 ? a : ~a;
                double w = a >= 0 ? t.wF[e] : t.wB[e];
                if (!Double.isFinite(w)) continue;
                if (avoided.get(e)) w *= avoidFactor;
                int v = a >= 0 ? t.ev[e] : t.eu[e];
                double nd = top[0] + w;
                if (nd < best[v]) {
                    best[v] = nd;
                    pq.add(new double[]{nd, v});
                }
            }
        }
        return best;
    }

    /** Even-odd point-in-polygon for the closed polygon. */
    static boolean inside(double[][] poly, double x, double y) {
        boolean in = false;
        for (int i = 0, j = poly.length - 1; i < poly.length; j = i++) {
            double xi = poly[i][0], yi = poly[i][1], xj = poly[j][0], yj = poly[j][1];
            if ((yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) in = !in;
        }
        return in;
    }

    /** Distance from (x, y) to the closed polygon outline. */
    static double outlineDist(double[][] poly, double x, double y) {
        double best = Double.MAX_VALUE;
        for (int i = 0, j = poly.length - 1; i < poly.length; j = i++)
            best = Math.min(best, Topology.segDist(poly[j][0] - x, poly[j][1] - y, poly[i][0] - x, poly[i][1] - y));
        return best;
    }

    private static void addCoord(List<double[]> list, double lat, double lon) {
        if (!list.isEmpty()) {
            double[] last = list.get(list.size() - 1);
            if (last[0] == lat && last[1] == lon) return;
        }
        list.add(new double[]{lat, lon});
    }

    /** mulberry32, as gh-autoroute prng.mjs. */
    static final class Mulberry32 {
        private int a;

        Mulberry32(long seed) {
            this.a = (int) seed;
        }

        double next() {
            a = a + 0x6d2b79f5;
            int t = a;
            t = (t ^ (t >>> 15)) * (t | 1);
            t ^= t + (t ^ (t >>> 7)) * (t | 61);
            return Integer.toUnsignedLong(t ^ (t >>> 14)) / 4294967296.0;
        }
    }
}
