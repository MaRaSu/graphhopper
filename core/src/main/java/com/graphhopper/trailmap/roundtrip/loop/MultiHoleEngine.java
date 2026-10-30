package com.graphhopper.trailmap.roundtrip.loop;

import com.graphhopper.GraphHopper;
import com.graphhopper.routing.ev.BooleanEncodedValue;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.ev.Subnetwork;
import com.graphhopper.routing.ev.RoadEnvironment;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.trailmap.shared.GravelScale;
import com.graphhopper.util.*;

import java.util.*;

/**
 * AutoRoute round 2, candidate a2: round-1 A (wind-once loop) with variation in the loop's shape (prototype, test
 * package only).
 * <p>
 * A's core is kept: the rideable core is copied into parity sheets and one Dijkstra from (S, 0) finds the cheapest
 * closed ride through S with the required winding. What a2 adds is the obstacle the loop winds around:
 * <ul>
 *   <li><b>Shaped holes.</b> The hole is a union of capsules along a polyline (a disk, a sausage, a bend, a U, an L),
 *       so several "holes" joined by a spine act as one obstacle. The loop must wind once around all of it; it cannot
 *       slip between the parts, so it cannot split into two loops around them.</li>
 *   <li><b>Excluded points.</b> Small disks the loop must <i>not</i> enclose (one more parity bit each, target 0).
 *       Placed in the pocket of a U or an L, they make the loop concave: a kidney, an L.</li>
 *   <li><b>Shape families.</b> Each generation draws configurations from different families, scales each one to the
 *       requested length, and shows 3 routes of different families and small edge overlap.</li>
 * </ul>
 * With {@code a2.mode=a} every addition is off and the engine is round-1 A's i5, line for line (isolation baseline).
 * {@code a2.cover=c2} swaps in c2's /convert-faithful waypoint cover (gravel_scale 4 shortcuts kept in the graph and
 * excluded from the loop search; /convert's avoid-edges x5 replayed; loops riding more than maxReuseM twice dropped).
 */
public class MultiHoleEngine {

    public static final class Params {
        // round-1 A (i5) parameters, unchanged
        double rho = 0.75;
        double dLoFactor = 0.25;
        double dHiFactor = 1.2;
        int maxSearches = 6;
        // production: the round-4 area16 settings, fixed (prototype flags in comments)
        double noise = 0.3;                         // a2.noise
        double noiseCellM = 2000;
        double lengthTol = 0.05;
        double fanHalfDeg = 90, fanStepDeg = 15, firstPickHalfDeg = 30, minSepDeg = 30;
        double showTol = 0.12;
        boolean debug = false;
        double coverSlack = 1e-3;
        // a2
        String mode = "shapes";                     // a2.mode: a | shapes
        String cover = "c2";                        // a2.cover: a | c2
        double maxReuseM = 300;                     // a2.maxReuseM
        double avoidFactor = 5.0;
        String families = "round,long,wide,side,u,l";
        int configsPerFamily = 2;
        double vertexJitter = 0.15;                 // in units of L/2pi
        double detour = 1.25;                       // network / hull length guess
        double maxOverlap = 0.5;                    // between shown routes
        // i2: lake-district availability
        boolean stemAware = true;                   // reuse filter ignores the start stem
        boolean landCheck = true;                   // drop configs whose spine is off the network
        double cellM = 1000;
        int minLandNeighbours = 5;                  // of 8, around each spine vertex
        int maxConfigs = 18;                        // > 0: draw until this many configs pass the land check
        // i3: speed, stop drawing when the pool has this many families and loops (0 = off)
        int enoughFamilies = 4;
        int enoughLoops = 6;
        // m: the shapes' rotation fan around the heading (a2 i3: +-90)
        double shapeFanDeg = 90;                    // m.a2Fan

        boolean aMode() { return mode.equals("a"); }

        boolean c2Cover() { return cover.equals("c2"); }

        @Override
        public String toString() {
            String base = "rho=" + rho + " d=[" + dLoFactor + "," + dHiFactor + "]*L/2pi searches<=" + maxSearches
                    + " tol=" + lengthTol + " fan=+-" + fanHalfDeg + "/" + fanStepDeg + " first=+-" + firstPickHalfDeg
                    + " minSep=" + minSepDeg + " showTol=" + showTol + " noise=+-" + noise + "/" + noiseCellM + "m"
                    + " waypoints=shortest-path-cover slack=" + coverSlack;
            return "mode=" + mode + " cover=" + cover + (cover.equals("c2") ? " maxReuseM=" + maxReuseM + " avoid=x" + avoidFactor : "")
                    + (aMode() ? "" : " families=" + families + " configsPerFamily=" + configsPerFamily + " jitter=" + vertexJitter
                    + " detour=" + detour + " maxOverlap=" + maxOverlap + " stemAware=" + stemAware + " landCheck=" + landCheck
                    + (landCheck ? "(" + cellM + "m cells, >=" + minLandNeighbours + "/8 around vertices)" : "")
                    + " maxConfigs=" + maxConfigs + " enough=" + enoughFamilies + "fam/" + enoughLoops + "loops"
                    + " shapeFan=+-" + shapeFanDeg) + " | " + base;
        }
    }

    /** One loop result. */
    public static final class Loop {
        public double bearingDeg, centreDistM, radiusM, distanceM, headingOffsetDeg;
        public int searches;
        public String family = "round", failure;
        public double scale;
        public List<double[]> coords = new ArrayList<>();      // lat, lon
        public List<double[]> waypoints = new ArrayList<>();   // lat, lon, without start/end
        public double roundness, elongation;                   // of the engine's own loop
        int[] arcs;
    }

    /** The rideable core around one start, in a local metric projection (A's, plus gravel_scale 4 flags). */
    static final class LocalGraph {
        double lat0, lon0, mPerLat, mPerLon;
        int startLocal;
        int n;
        int[] eu, ev;
        double[] wF, wB, dist;
        int[] gStart;
        double[] gx, gy;
        int[] adjStart, adjEdge;
        boolean[] hard4;          // gravel_scale 4 (only present with the c2 cover)
        int m;
        Set<Long> occupied;       // cells (params.cellM) with core geometry, built on first use

        static long cell(double x, double y, double cellM) {
            return (long) Math.floor(x / cellM) * 1_000_003L + (long) Math.floor(y / cellM);
        }

        double x(double lon) { return (lon - lon0) * mPerLon; }
        double y(double lat) { return (lat - lat0) * mPerLat; }
        double lon(double x) { return lon0 + x / mPerLon; }
        double lat(double y) { return lat0 + y / mPerLat; }
    }

    /**
     * The obstacle of one configuration, in metres around the start: capsules (polyline vertices + radius) the loop
     * winds around once, and excluded disks it must not enclose.
     */
    static final class Shape {
        String family;
        List<double[]> spine = new ArrayList<>();   // x, y
        double radius;
        List<double[]> excluded = new ArrayList<>(); // x, y, r
        double rotDeg;
        double[] rayDir;   // A's ray direction (the bearing), so a=mode reproduces A to the last bit
    }

    private final GraphHopper hopper;
    private final Weighting weighting;
    private final EnumEncodedValue<GravelScale> gravelScale;
    private final EnumEncodedValue<RoadEnvironment> roadEnv;
    private final BooleanEncodedValue subnetwork;
    final Params params = new Params();
    private double[] edgeNoise;
    // m4: set by the caller per press; null / 0 = off (a2 i3 unchanged)
    Memory memory;
    double memStrength;
    // m: every feasible loop of the last generateShapes call; skipPick leaves the pick (and the cover) to the caller
    List<Loop> lastPool = new ArrayList<>();
    boolean skipPick;

    public MultiHoleEngine(GraphHopper hopper, String profile, CustomModel requestModel) {
        this.hopper = hopper;
        if (requestModel == null) {
            this.weighting = hopper.createWeighting(hopper.getProfile(profile), new PMap());
        } else {
            PMap hints = new PMap();
            hints.putObject(CustomModel.KEY, requestModel);
            this.weighting = hopper.createWeighting(hopper.getProfile(profile), hints, false);
        }
        this.gravelScale = hopper.getEncodingManager().getEnumEncodedValue(GravelScale.KEY, GravelScale.class);
        this.roadEnv = hopper.getEncodingManager().getEnumEncodedValue(RoadEnvironment.KEY, RoadEnvironment.class);
        this.subnetwork = hopper.getEncodingManager().getBooleanEncodedValue(Subnetwork.key(profile));
    }

    /** The start snaps like /route does: never onto an edge of a small island (subnetwork) of this profile. */
    boolean startable(EdgeIteratorState e) {
        return rideable(e) && !e.get(subnetwork);
    }

    boolean rideable(EdgeIteratorState e) {
        GravelScale gs = e.get(gravelScale);
        if (gs == GravelScale.FOUR || gs == GravelScale.FERRY) return false;
        if (e.get(roadEnv) == RoadEnvironment.FERRY) return false;
        return Double.isFinite(weighting.calcEdgeWeight(e, false)) || Double.isFinite(weighting.calcEdgeWeight(e, true));
    }

    /** c2 cover: gravel_scale 4 edges stay in the graph (flagged) so the cover sees the shortcuts /convert takes. */
    boolean traversable(EdgeIteratorState e) {
        if (e.get(gravelScale) == GravelScale.FERRY || e.get(roadEnv) == RoadEnvironment.FERRY) return false;
        return Double.isFinite(weighting.calcEdgeWeight(e, false)) || Double.isFinite(weighting.calcEdgeWeight(e, true));
    }

    /** Collect the core reachable from the start within radiusM (straight line). A's, with the c2 option. */
    LocalGraph build(double lat, double lon, double radiusM) {
        boolean keep4 = params.c2Cover();
        BaseGraph g = hopper.getBaseGraph();
        NodeAccess na = g.getNodeAccess();
        Snap snap = hopper.getLocationIndex().findClosest(lat, lon, this::startable);
        if (!snap.isValid()) return null;
        EdgeIteratorState se = snap.getClosestEdge();
        int sNode = dist(lat, lon, na.getLat(se.getBaseNode()), na.getLon(se.getBaseNode()))
                <= dist(lat, lon, na.getLat(se.getAdjNode()), na.getLon(se.getAdjNode())) ? se.getBaseNode() : se.getAdjNode();

        LocalGraph lg = new LocalGraph();
        lg.lat0 = lat;
        lg.lon0 = lon;
        lg.mPerLat = 110_540;
        lg.mPerLon = 111_320 * Math.cos(Math.toRadians(lat));

        Map<Integer, Integer> local = new HashMap<>();
        BitSet seenEdge = new BitSet(g.getEdges());
        IntList eu = new IntList(), ev = new IntList(), gStart = new IntList();
        DoubleList wF = new DoubleList(), wB = new DoubleList(), dd = new DoubleList(), gx = new DoubleList(), gy = new DoubleList();
        List<Boolean> hard4 = new ArrayList<>();
        ArrayDeque<Integer> queue = new ArrayDeque<>();
        local.put(sNode, 0);
        queue.add(sNode);
        EdgeExplorer explorer = g.createEdgeExplorer();
        while (!queue.isEmpty()) {
            int u = queue.poll();
            EdgeIterator it = explorer.setBaseNode(u);
            while (it.next()) {
                int edge = it.getEdge();
                if (seenEdge.get(edge)) continue;
                int v = it.getAdjNode();
                if (v == u) { seenEdge.set(edge); continue; }
                if (dist(lat, lon, na.getLat(v), na.getLon(v)) > radiusM) continue;
                seenEdge.set(edge);
                if (!(keep4 ? traversable(it) : rideable(it))) continue;
                Integer lv = local.get(v);
                if (lv == null) {
                    lv = local.size();
                    local.put(v, lv);
                    queue.add(v);
                }
                eu.add(local.get(u));
                ev.add(lv);
                wF.add(weighting.calcEdgeWeight(it, false));
                wB.add(weighting.calcEdgeWeight(it, true));
                dd.add(it.getDistance());
                hard4.add(it.get(gravelScale) == GravelScale.FOUR);
                gStart.add(gx.size());
                PointList pl = it.fetchWayGeometry(FetchMode.ALL);
                for (int i = 0; i < pl.size(); i++) {
                    gx.add(lg.x(pl.getLon(i)));
                    gy.add(lg.y(pl.getLat(i)));
                }
            }
        }
        lg.n = local.size();
        lg.m = eu.size();
        lg.startLocal = 0;
        lg.eu = eu.toArray();
        lg.ev = ev.toArray();
        lg.wF = wF.toArray();
        lg.wB = wB.toArray();
        lg.dist = dd.toArray();
        lg.hard4 = new boolean[lg.m];
        for (int e = 0; e < lg.m; e++) lg.hard4[e] = hard4.get(e);
        gStart.add(gx.size());
        lg.gStart = gStart.toArray();
        lg.gx = gx.toArray();
        lg.gy = gy.toArray();
        int[] deg = new int[lg.n + 1];
        for (int e = 0; e < lg.m; e++) {
            deg[lg.eu[e]]++;
            deg[lg.ev[e]]++;
        }
        lg.adjStart = new int[lg.n + 1];
        for (int i = 0; i < lg.n; i++) lg.adjStart[i + 1] = lg.adjStart[i] + deg[i];
        int[] fill = Arrays.copyOf(lg.adjStart, lg.n);
        lg.adjEdge = new int[2 * lg.m];
        for (int e = 0; e < lg.m; e++) {
            lg.adjEdge[fill[lg.eu[e]]++] = e;
            lg.adjEdge[fill[lg.ev[e]]++] = ~e;
        }
        return lg;
    }

    void makeNoise(LocalGraph lg, long seed) {
        edgeNoise = null;
        if (params.noise > 0) {
            edgeNoise = new double[lg.m];
            for (int e = 0; e < lg.m; e++) {
                int mid = (lg.gStart[e] + lg.gStart[e + 1] - 1) / 2;
                long cx = (long) Math.floor(lg.gx[mid] / params.noiseCellM), cy = (long) Math.floor(lg.gy[mid] / params.noiseCellM);
                long h = seed * 0x9E3779B97F4A7C15L + cx * 0xC2B2AE3D27D4EB4FL + cy * 0x165667B19E3779F9L;
                h ^= h >>> 33;
                h *= 0xFF51AFD7ED558CCDL;
                h ^= h >>> 33;
                double u = (h >>> 11) / (double) (1L << 53);
                edgeNoise[e] = 1 + params.noise * (2 * u - 1);
            }
        }
    }

    /** One generation: up to 3 loops. */
    public List<Loop> generate(LocalGraph lg, double targetM, double headingDeg, long seed) {
        Random rnd = new Random(seed);
        makeNoise(lg, seed);
        // m4: the session's shown ways cost more in the loop search (folded into the noise factor)
        double[] mem = memory == null ? null : memory.edgeMult(lg.m, lg.gStart, lg.gx, lg.gy, new Memory.Frame() {
            public double lat(double y) { return lg.lat(y); }

            public double lon(double x) { return lg.lon(x); }
        }, memStrength);
        if (mem != null) {
            if (edgeNoise == null) {
                edgeNoise = new double[lg.m];
                Arrays.fill(edgeNoise, 1);
            }
            for (int e = 0; e < lg.m; e++) edgeNoise[e] *= mem[e];
        }
        List<Loop> out = params.aMode() ? generateA(lg, targetM, headingDeg, rnd) : generateShapes(lg, targetM, headingDeg, rnd);
        while (out.size() < 3) {
            Loop f = new Loop();
            f.failure = "search_miss";
            f.bearingDeg = headingDeg;
            out.add(f);
        }
        return out;
    }

    // ------------------------------------------------------------------ round-1 A (i5), unchanged

    List<Loop> generateA(LocalGraph lg, double targetM, double headingDeg, Random rnd) {
        double jitter = (rnd.nextDouble() * 2 - 1) * params.fanStepDeg / 2;
        List<Loop> feasible = new ArrayList<>();
        for (double off = -params.fanHalfDeg; off <= params.fanHalfDeg + 1e-9; off += params.fanStepDeg) {
            Loop l = loopForBearing(lg, targetM, headingDeg + jitter + off);
            l.headingOffsetDeg = off + jitter;
            if (params.debug)
                System.out.printf(Locale.ROOT, "  fan b=%.0f %s d=%.0f r=%.0f%n", l.bearingDeg,
                        l.failure != null ? l.failure : String.format(Locale.ROOT, "len=%.2f", l.distanceM / targetM), l.centreDistM, l.radiusM);
            if (l.failure == null && Math.abs(l.distanceM / targetM - 1) <= params.showTol) feasible.add(l);
        }
        List<Loop> out = new ArrayList<>();
        List<Loop> near = new ArrayList<>();
        for (Loop l : feasible) if (Math.abs(l.headingOffsetDeg) <= params.firstPickHalfDeg) near.add(l);
        if (near.isEmpty()) near = feasible;
        if (!near.isEmpty()) out.add(near.get(rnd.nextInt(near.size())));
        while (out.size() < 3) {
            Loop pick = null;
            double bestSep = params.minSepDeg;
            for (Loop l : feasible) {
                if (out.contains(l)) continue;
                double sep = Double.MAX_VALUE;
                for (Loop o : out) sep = Math.min(sep, Math.abs(l.headingOffsetDeg - o.headingOffsetDeg));
                if (sep >= bestSep) {
                    bestSep = sep;
                    pick = l;
                }
            }
            if (pick == null) break;
            out.add(pick);
        }
        for (Loop l : out) coverWaypoints(lg, l);
        return out;
    }

    Loop loopForBearing(LocalGraph lg, double targetM, double bearingDeg) {
        double unit = targetM / (2 * Math.PI);
        double lo = params.dLoFactor * unit, hi = params.dHiFactor * unit;
        double d = 0.8 * unit;
        Loop best = null;
        double bestErr = Double.MAX_VALUE;
        int searches = 0;
        for (int i = 0; i < params.maxSearches; i++) {
            searches++;
            double br = Math.toRadians(bearingDeg);
            Shape disk = new Shape();
            disk.family = "round";
            double ux = Math.sin(br), uy = Math.cos(br);
            disk.spine.add(new double[]{ux * d, uy * d});
            disk.rayDir = new double[]{ux, uy};
            disk.radius = params.rho * d;
            Loop l = solve(lg, disk);
            l.bearingDeg = ((bearingDeg % 360) + 360) % 360;
            l.centreDistM = d;
            l.radiusM = params.rho * d;
            double err;
            if (l.failure != null) {
                err = Double.MAX_VALUE / 2;
                hi = d;
            } else {
                err = Math.abs(l.distanceM / targetM - 1);
                if (l.distanceM > targetM) hi = d; else lo = d;
            }
            if (best == null || err < bestErr) {
                best = l;
                bestErr = err;
            }
            if (err <= params.lengthTol) break;
            d = (lo + hi) / 2;
        }
        best.searches = searches;
        return best;
    }

    // ------------------------------------------------------------------ a2: shape families

    /**
     * The family templates, in units of u = L/2pi, in a frame where +y is the configuration's bearing and the start
     * is the origin. Returns the spine (polyline), the capsule radius and the excluded disks (x, y, r).
     */
    static Shape template(String family, boolean mirror) {
        Shape s = new Shape();
        s.family = family;
        double m = mirror ? -1 : 1;
        switch (family) {
            case "round": // A's balloon: one disk ahead
                s.spine.add(new double[]{0, 0.8});
                s.radius = 0.6;
                break;
            case "long": // a long oval pointing away from the start (start at its tip)
                s.spine.add(new double[]{0, 0.5});
                s.spine.add(new double[]{0, 1.9});
                s.radius = 0.3;
                break;
            case "wide": // a wide oval, the start in the middle of its near long side
                s.spine.add(new double[]{-1.0, 0.6});
                s.spine.add(new double[]{1.0, 0.6});
                s.radius = 0.3;
                break;
            case "side": // a long oval off to one side, the start near one end of its long side
                s.spine.add(new double[]{m * 0.3, 0.55});
                s.spine.add(new double[]{m * 1.9, 0.55});
                s.radius = 0.3;
                break;
            case "u": // a U opening away from the start, its pocket excluded: a kidney / heart
                s.spine.add(new double[]{-0.9, 1.6});
                s.spine.add(new double[]{-0.7, 0.6});
                s.spine.add(new double[]{0.7, 0.6});
                s.spine.add(new double[]{0.9, 1.6});
                s.radius = 0.25;
                s.excluded.add(new double[]{0, 1.55, 0.3});
                break;
            case "l": // an L, its inner corner excluded: an L-shaped loop
                s.spine.add(new double[]{m * 0.3, 0.45});
                s.spine.add(new double[]{m * 0.3, 1.7});
                s.spine.add(new double[]{m * 1.6, 1.7});
                s.radius = 0.25;
                s.excluded.add(new double[]{m * 1.15, 0.85, 0.3});
                break;
            default:
                throw new IllegalArgumentException("unknown family " + family);
        }
        return s;
    }

    /** The template rotated to bearingDeg, jittered and scaled to metres (scale = metres per unit). */
    static Shape place(Shape t, double bearingDeg, double scale, double[] jitter) {
        Shape s = new Shape();
        s.family = t.family;
        s.rotDeg = bearingDeg;
        double br = Math.toRadians(bearingDeg);
        double fx = Math.sin(br), fy = Math.cos(br);   // forward
        double tx = fy, ty = -fx;                      // right
        int j = 0;
        for (double[] p : t.spine) {
            double px = p[0] + jitter[j++], py = p[1] + jitter[j++];
            s.spine.add(new double[]{scale * (px * tx + py * fx), scale * (px * ty + py * fy)});
        }
        s.radius = scale * t.radius;
        for (double[] p : t.excluded) {
            double px = p[0] + jitter[j++], py = p[1] + jitter[j++];
            s.excluded.add(new double[]{scale * (px * tx + py * fx), scale * (px * ty + py * fy), scale * p[2]});
        }
        return s;
    }

    /** Rough loop length per metre of scale: the convex hull of the start and the capsules, times the detour guess. */
    double guessLengthPerScale(Shape unitShape) {
        List<double[]> pts = new ArrayList<>();
        pts.add(new double[]{0, 0});
        for (double[] p : unitShape.spine)
            for (int k = 0; k < 16; k++) {
                double a = 2 * Math.PI * k / 16;
                pts.add(new double[]{p[0] + unitShape.radius * Math.cos(a), p[1] + unitShape.radius * Math.sin(a)});
            }
        return params.detour * hullPerimeter(pts);
    }

    List<Loop> generateShapes(LocalGraph lg, double targetM, double headingDeg, Random rnd) {
        double unit = targetM / (2 * Math.PI);
        List<String> fams = new ArrayList<>(Arrays.asList(params.families.split(",")));
        Collections.shuffle(fams, rnd);
        List<Loop> feasible = new ArrayList<>();
        // i2 (maxConfigs > 0): keep drawing until maxConfigs configurations passed the land check (or 6x as many draws)
        int rounds = params.maxConfigs > 0 ? 6 * params.maxConfigs : params.configsPerFamily * fams.size();
        int solved = 0;
        for (int draw = 0; draw < rounds; draw++) {
            int c = draw / fams.size();
            String fam = fams.get(draw % fams.size());
            if (params.maxConfigs > 0 && solved >= params.maxConfigs) break;
            if (params.enoughFamilies > 0) {
                // i3: stop drawing once the pool already holds enough different shapes to pick 3 from
                Set<String> fs = new HashSet<>();
                for (Loop l : feasible) fs.add(l.family);
                if (fs.size() >= params.enoughFamilies && feasible.size() >= params.enoughLoops) break;
            }
            {
                boolean mirror = rnd.nextBoolean();
                // the first configuration of the first family is near the heading; the rest spread over the fan
                double off = c == 0 && feasible.isEmpty() && fam.equals(fams.get(0))
                        ? (rnd.nextDouble() * 2 - 1) * params.firstPickHalfDeg
                        : (rnd.nextDouble() * 2 - 1) * params.shapeFanDeg;
                Shape t = template(fam, mirror);
                double[] jitter = new double[2 * (t.spine.size() + t.excluded.size())];
                for (int i = 0; i < jitter.length; i++) jitter[i] = (rnd.nextDouble() * 2 - 1) * params.vertexJitter;
                if (params.landCheck) {
                    double s0 = targetM / guessLengthPerScale(place(t, 0, 1, jitter));
                    if (!onLand(lg, place(t, headingDeg + off, s0, jitter))) {
                        if (params.debug) System.out.printf(Locale.ROOT, "  %s off=%.0f off the network%n", fam, off);
                        continue;
                    }
                }
                solved++;
                Loop l = loopForShape(lg, targetM, t, headingDeg + off, jitter, unit);
                l.headingOffsetDeg = off;
                if (params.debug)
                    System.out.printf(Locale.ROOT, "  %s off=%.0f %s s=%.2f searches=%d%n", fam, off,
                            l.failure != null ? l.failure : String.format(Locale.ROOT, "len=%.2f", l.distanceM / targetM), l.scale / unit, l.searches);
                if (l.failure == null && Math.abs(l.distanceM / targetM - 1) <= params.showTol) feasible.add(l);
            }
        }
        lastPool = feasible;
        if (skipPick) return new ArrayList<>();
        // pick 3: first = the first feasible near the heading (else any); then the most different by family and edges
        List<Loop> out = new ArrayList<>();
        Loop first = null;
        for (Loop l : feasible) if (Math.abs(l.headingOffsetDeg) <= params.firstPickHalfDeg) { first = l; break; }
        if (first == null && !feasible.isEmpty()) first = feasible.get(0);
        if (first != null) out.add(first);
        while (out.size() < 3) {
            Loop pick = null;
            double bestScore = -1;
            for (Loop l : feasible) {
                if (out.contains(l)) continue;
                double maxOv = 0;
                boolean sameFamily = false;
                for (Loop o : out) {
                    maxOv = Math.max(maxOv, edgeOverlap(lg, l, o));
                    sameFamily |= o.family.equals(l.family);
                }
                if (maxOv > params.maxOverlap) continue;
                double score = (1 - maxOv) + (sameFamily ? 0 : 1);
                if (score > bestScore) {
                    bestScore = score;
                    pick = l;
                }
            }
            if (pick == null) break;
            out.add(pick);
        }
        for (Loop l : out) coverWaypoints(lg, l);
        return out;
    }

    /**
     * i2: the obstacle's spine must lie on the network: every 250 m along it in a cell with core geometry, and each
     * vertex with at least minLandNeighbours of its 8 neighbour cells occupied. A spine on a lake makes the lake part
     * of the hole, and around Näsijärvi no loop of the requested length encloses it.
     */
    boolean onLand(LocalGraph lg, Shape s) {
        double cm = params.cellM;
        // production: a cached LocalGraph is shared by concurrent requests, so the lazy init is published once
        synchronized (lg) {
            if (lg.occupied == null) {
                Set<Long> occ = new HashSet<>();
                for (int i = 0; i < lg.gx.length; i++) occ.add(LocalGraph.cell(lg.gx[i], lg.gy[i], cm));
                lg.occupied = occ;
            }
        }
        List<double[]> sp = s.spine;
        for (int i = 0; i < sp.size(); i++) {
            double[] p = sp.get(i);
            int nb = 0;
            for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                    if ((dx != 0 || dy != 0) && lg.occupied.contains(LocalGraph.cell(p[0] + dx * cm, p[1] + dy * cm, cm))) nb++;
            if (nb < params.minLandNeighbours) return false;
            if (i + 1 < sp.size()) {
                double[] q = sp.get(i + 1);
                int steps = (int) Math.max(1, Math.ceil(Math.hypot(q[0] - p[0], q[1] - p[1]) / 250));
                for (int k = 0; k <= steps; k++) {
                    double t = (double) k / steps;
                    if (!lg.occupied.contains(LocalGraph.cell(p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1]), cm))) return false;
                }
            } else if (!lg.occupied.contains(LocalGraph.cell(p[0], p[1], cm))) return false;
        }
        return true;
    }

    /** Scale a template to the requested length: secant steps on the scale, safeguarded by a bracket. */
    Loop loopForShape(LocalGraph lg, double targetM, Shape t, double bearingDeg, double[] jitter, double unit) {
        Shape unitShape = place(t, 0, 1, jitter);
        double s = targetM / guessLengthPerScale(unitShape);  // metres per template unit
        double lo = 0, hi = Double.MAX_VALUE;
        Loop best = null;
        double bestErr = Double.MAX_VALUE;
        int searches = 0;
        for (int i = 0; i < params.maxSearches; i++) {
            searches++;
            Shape shape = place(t, bearingDeg, s, jitter);
            Loop l = solve(lg, shape);
            l.scale = s;
            l.bearingDeg = ((bearingDeg % 360) + 360) % 360;
            l.family = t.family;
            l.centreDistM = Math.hypot(shape.spine.get(0)[0], shape.spine.get(0)[1]);
            l.radiusM = shape.radius;
            double err;
            double next;
            if (l.failure != null) {
                err = Double.MAX_VALUE / 2;
                hi = Math.min(hi, s);
                next = lo > 0 ? (lo + s) / 2 : 0.7 * s;
            } else {
                err = Math.abs(l.distanceM / targetM - 1);
                if (l.distanceM > targetM) hi = Math.min(hi, s); else lo = Math.max(lo, s);
                next = s * targetM / l.distanceM;
                if (next <= lo || next >= hi) next = hi < Double.MAX_VALUE && lo > 0 ? (lo + hi) / 2 : next;
            }
            if (best == null || err < bestErr) {
                best = l;
                bestErr = err;
            }
            if (err <= params.lengthTol) break;
            s = next;
        }
        best.searches = searches;
        return best;
    }

    // ------------------------------------------------------------------ the winding search

    /**
     * Parity Dijkstra around the obstacle: edges touching the capsules or the excluded disks are removed; sheet bit 0
     * flips on crossing a ray from the first spine vertex (target 1, winds once around the obstacle), bit k flips on a
     * ray from excluded disk k (target 0, does not enclose it).
     */
    Loop solve(LocalGraph lg, Shape shape) {
        Loop loop = new Loop();
        int nEx = shape.excluded.size();
        int sheets = 1 << (1 + nEx);
        double[][] rays = new double[1 + nEx][];   // ox, oy, ux, uy
        rays[0] = shape.rayDir != null
                ? new double[]{shape.spine.get(0)[0], shape.spine.get(0)[1], shape.rayDir[0], shape.rayDir[1]}
                : ray(shape.spine.get(0)[0], shape.spine.get(0)[1]);
        for (int k = 0; k < nEx; k++) rays[k + 1] = ray(shape.excluded.get(k)[0], shape.excluded.get(k)[1]);

        boolean[] blocked = new boolean[lg.m];
        int[] flip = new int[lg.m];
        for (int e = 0; e < lg.m; e++) {
            if (lg.hard4 != null && lg.hard4[e]) { blocked[e] = true; continue; }
            int from = lg.gStart[e], to = lg.gStart[e + 1] - 1;
            boolean blk = false;
            for (int i = from; i < to && !blk; i++) {
                double ax = lg.gx[i], ay = lg.gy[i], bx = lg.gx[i + 1], by = lg.gy[i + 1];
                blk = capsuleHit(shape, ax, ay, bx, by);
            }
            blocked[e] = blk;
            if (blk) continue;
            int mask = 0;
            for (int r = 0; r <= nEx; r++) {
                double ox = rays[r][0], oy = rays[r][1], ux = rays[r][2], uy = rays[r][3];
                int crossings = 0;
                for (int i = from; i < to; i++) {
                    double ax = lg.gx[i] - ox, ay = lg.gy[i] - oy, bx = lg.gx[i + 1] - ox, by = lg.gy[i + 1] - oy;
                    double sa = ux * ay - uy * ax, sb = ux * by - uy * bx;
                    if ((sa <= 0 && sb > 0) || (sb <= 0 && sa > 0)) {
                        double ta = ax * ux + ay * uy, tb = bx * ux + by * uy;
                        if (ta + (tb - ta) * sa / (sa - sb) > 0) crossings++;
                    }
                }
                if ((crossings & 1) == 1) mask |= 1 << r;
            }
            flip[e] = mask;
        }

        int states = sheets * lg.n;
        double[] best = new double[states];
        Arrays.fill(best, Double.POSITIVE_INFINITY);
        int[] parentEdge = new int[states];
        int[] parentState = new int[states];
        PriorityQueue<double[]> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> a[0]));
        int src = sheets * lg.startLocal, dst = sheets * lg.startLocal + 1;
        best[src] = 0;
        pq.add(new double[]{0, src});
        while (!pq.isEmpty()) {
            double[] top = pq.poll();
            int s = (int) top[1];
            if (top[0] > best[s]) continue;
            if (s == dst) break;
            int node = s / sheets, sh = s % sheets;
            for (int k = lg.adjStart[node]; k < lg.adjStart[node + 1]; k++) {
                int a = lg.adjEdge[k];
                int e = a >= 0 ? a : ~a;
                if (blocked[e]) continue;
                double w = a >= 0 ? lg.wF[e] : lg.wB[e];
                if (!Double.isFinite(w)) continue;
                if (edgeNoise != null) w *= edgeNoise[e];
                int other = a >= 0 ? lg.ev[e] : lg.eu[e];
                int t = sheets * other + (sh ^ flip[e]);
                double nd = top[0] + w;
                if (nd < best[t]) {
                    best[t] = nd;
                    parentEdge[t] = a;
                    parentState[t] = s;
                    pq.add(new double[]{nd, t});
                }
            }
        }
        if (!Double.isFinite(best[dst])) {
            loop.failure = "no_loop";
            return loop;
        }
        ArrayDeque<Integer> path = new ArrayDeque<>();
        int s = dst;
        do {
            path.push(parentEdge[s]);
            s = parentState[s];
        } while (s != src);
        double total = 0;
        for (int a : path) {
            int e = a >= 0 ? a : ~a;
            total += lg.dist[e];
            int from = lg.gStart[e], to = lg.gStart[e + 1] - 1;
            if (a >= 0) {
                for (int i = from; i <= to; i++)
                    addCoord(loop.coords, lg.lat(lg.gy[i]), lg.lon(lg.gx[i]));
            } else {
                for (int i = to; i >= from; i--)
                    addCoord(loop.coords, lg.lat(lg.gy[i]), lg.lon(lg.gx[i]));
            }
        }
        loop.distanceM = total;
        loop.arcs = path.stream().mapToInt(Integer::intValue).toArray();
        if (params.c2Cover() && params.maxReuseM >= 0) {
            // c2 i3: a loop riding an edge twice; /convert's avoid-edges would detour around the second pass
            double reused = 0;
            BitSet seen = new BitSet(lg.m);
            int k = loop.arcs.length, stem = 0;
            if (params.stemAware) {
                // i2: the start's own access road ridden out and back (a stem) is not counted: the editor rides it
                // the same way, there is no other way to the start
                while (stem < k - 1 - stem && edgeOf(loop.arcs[stem]) == edgeOf(loop.arcs[k - 1 - stem])) stem++;
                for (int i = 0; i < stem; i++) seen.set(edgeOf(loop.arcs[i]));
            }
            for (int i = stem; i < k - stem; i++) {
                int a = loop.arcs[i];
                int e = a >= 0 ? a : ~a;
                if (seen.get(e)) reused += lg.dist[e];
                seen.set(e);
            }
            if (reused > params.maxReuseM) {
                loop.failure = "reuse";
                return loop;
            }
        }
        shapeMeasures(lg, loop);
        return loop;
    }

    static int edgeOf(int arc) {
        return arc >= 0 ? arc : ~arc;
    }

    /** A ray from (ox, oy) pointing away from the start. */
    static double[] ray(double ox, double oy) {
        double l = Math.hypot(ox, oy);
        return l == 0 ? new double[]{ox, oy, 0, 1} : new double[]{ox, oy, ox / l, oy / l};
    }

    static boolean capsuleHit(Shape shape, double ax, double ay, double bx, double by) {
        List<double[]> sp = shape.spine;
        if (sp.size() == 1) {
            if (segDist(ax - sp.get(0)[0], ay - sp.get(0)[1], bx - sp.get(0)[0], by - sp.get(0)[1]) < shape.radius) return true;
        } else {
            for (int i = 0; i + 1 < sp.size(); i++)
                if (segSegDist(ax, ay, bx, by, sp.get(i)[0], sp.get(i)[1], sp.get(i + 1)[0], sp.get(i + 1)[1]) < shape.radius) return true;
        }
        for (double[] x : shape.excluded)
            if (segDist(ax - x[0], ay - x[1], bx - x[0], by - x[1]) < x[2]) return true;
        return false;
    }

    /** Roundness = 4 pi A / P^2 (1 = circle) and elongation = principal axis ratio (1 = round), of the own loop. */
    static void shapeMeasures(LocalGraph lg, Loop loop) {
        List<double[]> c = loop.coords;
        int n = c.size();
        if (n < 3) return;
        double area = 0, per = 0, mx = 0, my = 0;
        double[] xs = new double[n], ys = new double[n];
        for (int i = 0; i < n; i++) {
            xs[i] = lg.x(c.get(i)[1]);
            ys[i] = lg.y(c.get(i)[0]);
            mx += xs[i];
            my += ys[i];
        }
        mx /= n;
        my /= n;
        double sxx = 0, syy = 0, sxy = 0;
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            area += xs[i] * ys[j] - xs[j] * ys[i];
            per += Math.hypot(xs[j] - xs[i], ys[j] - ys[i]);
            double dx = xs[i] - mx, dy = ys[i] - my;
            sxx += dx * dx;
            syy += dy * dy;
            sxy += dx * dy;
        }
        area = Math.abs(area) / 2;
        loop.roundness = per > 0 ? 4 * Math.PI * area / (per * per) : 0;
        double tr = sxx + syy, det = sxx * syy - sxy * sxy, disc = Math.sqrt(Math.max(0, tr * tr / 4 - det));
        double l1 = tr / 2 + disc, l2 = tr / 2 - disc;
        loop.elongation = l2 > 0 ? Math.sqrt(l1 / l2) : 99;
    }

    /** Share of the shorter loop's length on edges the other loop also rides. */
    static double edgeOverlap(LocalGraph lg, Loop a, Loop b) {
        BitSet ea = new BitSet(lg.m);
        for (int x : a.arcs) ea.set(x >= 0 ? x : ~x);
        double common = 0, lb = 0;
        BitSet seen = new BitSet(lg.m);
        for (int x : b.arcs) {
            int e = x >= 0 ? x : ~x;
            if (seen.get(e)) continue;
            seen.set(e);
            lb += lg.dist[e];
            if (ea.get(e)) common += lg.dist[e];
        }
        double la = 0;
        BitSet seenA = new BitSet(lg.m);
        for (int x : a.arcs) {
            int e = x >= 0 ? x : ~x;
            if (!seenA.get(e)) la += lg.dist[e];
            seenA.set(e);
        }
        return common / Math.max(1, Math.min(la, lb));
    }

    // ------------------------------------------------------------------ waypoints

    /** A's shortest-path cover; with the c2 cover, /convert's avoid-edges x5 on the loop edges already passed. */
    void coverWaypoints(LocalGraph lg, Loop loop) {
        if (loop.failure != null) return;
        int k = loop.arcs.length;
        int[] nodes = new int[k + 1];
        double[] cum = new double[k + 1];
        nodes[0] = lg.startLocal;
        for (int i = 0; i < k; i++) {
            int a = loop.arcs[i];
            int e = a >= 0 ? a : ~a;
            nodes[i + 1] = a >= 0 ? lg.ev[e] : lg.eu[e];
            cum[i + 1] = cum[i] + (a >= 0 ? lg.wF[e] : lg.wB[e]);
        }
        loop.waypoints.clear();
        int cur = 0;
        BitSet avoided = new BitSet(lg.m);
        while (cur < k) {
            if (params.c2Cover()) {
                avoided.clear();
                for (int i = 0; i < cur; i++) avoided.set(loop.arcs[i] >= 0 ? loop.arcs[i] : ~loop.arcs[i]);
            }
            double[] d = dijkstra(lg, nodes[cur], (cum[k] - cum[cur]) * (1 + params.coverSlack) + 1, avoided, params.avoidFactor);
            int j = cur + 1;
            while (j + 1 <= k && d[nodes[j + 1]] >= (cum[j + 1] - cum[cur]) * (1 - params.coverSlack) - 1e-6)
                j++;
            if (j == k) break;
            int a = loop.arcs[j - 1];
            int e = a >= 0 ? a : ~a;
            int from = lg.gStart[e], to = lg.gStart[e + 1] - 1;
            double mx, my;
            if (to - from >= 2) {
                int mid = (from + to) / 2;
                mx = lg.gx[mid];
                my = lg.gy[mid];
            } else {
                mx = (lg.gx[from] + lg.gx[to]) / 2;
                my = (lg.gy[from] + lg.gy[to]) / 2;
            }
            loop.waypoints.add(new double[]{lg.lat(my), lg.lon(mx)});
            cur = j - 1 > cur ? j - 1 : j;
        }
    }

    /** One-to-all Dijkstra on the core (no hole), bounded by maxWeight; avoided edges cost avoidFactor x. */
    static double[] dijkstra(LocalGraph lg, int src, double maxWeight, BitSet avoided, double avoidFactor) {
        double[] best = new double[lg.n];
        Arrays.fill(best, Double.POSITIVE_INFINITY);
        best[src] = 0;
        PriorityQueue<double[]> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> a[0]));
        pq.add(new double[]{0, src});
        while (!pq.isEmpty()) {
            double[] top = pq.poll();
            int u = (int) top[1];
            if (top[0] > best[u]) continue;
            if (top[0] > maxWeight) break;
            for (int k = lg.adjStart[u]; k < lg.adjStart[u + 1]; k++) {
                int a = lg.adjEdge[k];
                int e = a >= 0 ? a : ~a;
                double w = a >= 0 ? lg.wF[e] : lg.wB[e];
                if (!Double.isFinite(w)) continue;
                if (avoided.get(e)) w *= avoidFactor;
                int v = a >= 0 ? lg.ev[e] : lg.eu[e];
                double nd = top[0] + w;
                if (nd < best[v]) {
                    best[v] = nd;
                    pq.add(new double[]{nd, v});
                }
            }
        }
        return best;
    }

    // ------------------------------------------------------------------ geometry helpers

    static double hullPerimeter(List<double[]> pts) {
        List<double[]> p = new ArrayList<>(pts);
        p.sort((a, b) -> a[0] != b[0] ? Double.compare(a[0], b[0]) : Double.compare(a[1], b[1]));
        int n = p.size();
        double[][] h = new double[2 * n][];
        int k = 0;
        for (int i = 0; i < n; i++) {
            while (k >= 2 && cross(h[k - 2], h[k - 1], p.get(i)) <= 0) k--;
            h[k++] = p.get(i);
        }
        for (int i = n - 2, t = k + 1; i >= 0; i--) {
            while (k >= t && cross(h[k - 2], h[k - 1], p.get(i)) <= 0) k--;
            h[k++] = p.get(i);
        }
        double per = 0;
        for (int i = 0; i + 1 < k; i++) per += Math.hypot(h[i + 1][0] - h[i][0], h[i + 1][1] - h[i][1]);
        return per;
    }

    static double cross(double[] o, double[] a, double[] b) {
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    }

    /** Distance between segments ab and cd. */
    static double segSegDist(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy) {
        if (segmentsIntersect(ax, ay, bx, by, cx, cy, dx, dy)) return 0;
        return Math.min(Math.min(segDist(cx - ax, cy - ay, dx - ax, dy - ay), segDist(cx - bx, cy - by, dx - bx, dy - by)),
                Math.min(segDist(ax - cx, ay - cy, bx - cx, by - cy), segDist(ax - dx, ay - dy, bx - dx, by - dy)));
    }

    static boolean segmentsIntersect(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy) {
        double d1 = (dx - cx) * (ay - cy) - (dy - cy) * (ax - cx);
        double d2 = (dx - cx) * (by - cy) - (dy - cy) * (bx - cx);
        double d3 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
        double d4 = (bx - ax) * (dy - ay) - (by - ay) * (dx - ax);
        return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0));
    }

    private static void addCoord(List<double[]> list, double lat, double lon) {
        if (!list.isEmpty()) {
            double[] last = list.get(list.size() - 1);
            if (last[0] == lat && last[1] == lon) return;
        }
        list.add(new double[]{lat, lon});
    }

    /** Distance from the origin to segment ab. */
    static double segDist(double ax, double ay, double bx, double by) {
        double dx = bx - ax, dy = by - ay;
        double len2 = dx * dx + dy * dy;
        double t = len2 == 0 ? 0 : Math.max(0, Math.min(1, -(ax * dx + ay * dy) / len2));
        double px = ax + t * dx, py = ay + t * dy;
        return Math.sqrt(px * px + py * py);
    }

    static double dist(double lat1, double lon1, double lat2, double lon2) {
        double dy = (lat2 - lat1) * 110_540, dx = (lon2 - lon1) * 111_320 * Math.cos(Math.toRadians(lat1));
        return Math.sqrt(dx * dx + dy * dy);
    }

    static final class IntList {
        int[] a = new int[1024];
        int n;
        void add(int v) { if (n == a.length) a = Arrays.copyOf(a, n * 2); a[n++] = v; }
        int size() { return n; }
        int[] toArray() { return Arrays.copyOf(a, n); }
    }

    static final class DoubleList {
        double[] a = new double[1024];
        int n;
        void add(double v) { if (n == a.length) a = Arrays.copyOf(a, n * 2); a[n++] = v; }
        int size() { return n; }
        double[] toArray() { return Arrays.copyOf(a, n); }
    }
}
