package com.graphhopper.trailmap.tilerouting.looprepair;

import com.graphhopper.storage.BaseGraph;
import com.graphhopper.trailmap.tilerouting.foundation.*;
import com.graphhopper.util.EdgeIteratorState;

import java.util.*;

/**
 * Candidate 2: loop repair by section replacement (ruin-and-recreate on the real route).
 * <ul>
 * <li>State: one closed route from home, an ordered list of PLAIN via points (no headings, no pass_through), each a
 * location inside a tile's inset square (the foundation's portal points). Legs are routed in-process by GraphHopper
 * exactly as the verifier routes them, and cached.</li>
 * <li>Objective on the assembled geometry: length + obLambda · out-and-back metres (the verifier's 25 m reverse
 * corridor) + uTurnM · (u-turn instructions + via reversals) + missM · tiles not entered.</li>
 * <li>Proposals use a plain-via road-distance table between all locations ({@link PlainCostTable}).</li>
 * <li>Seed: Viterbi over the given order (and its reverse) on that table. Moves: ruin a contiguous section of 2–5
 * stops, release the tiles only it fetched, recreate by cheapest insertion + Viterbi location re-pick against the
 * frozen rest; reverse a section; swap a stop's location; drop a stop. Greedy acceptance; simulated annealing once
 * greedy plateaus. Stops at the time budget.</li>
 * </ul>
 */
public class LoopRepairCandidate implements Candidate {

    // the prototype's tr.c2.* defaults
    /** Wall-clock search budget: results of a budget-limited search depend on the machine's speed and load. */
    public long budgetMs = 10_000;
    /**
     * The clock of the budget and of the annealing schedule (the temperature falls with the elapsed share of the
     * budget). Production: the wall clock. Tests: a virtual clock that advances per call, so a run is reproducible.
     */
    public java.util.function.LongSupplier clock = System::nanoTime;
    final int plateauMoves = 120;
    final int saPlateauMoves = 600;
    final boolean anneal = true;
    final int maxLocsPerTile = 6;
    final double locDedupM = 50;
    final long seed = 1;
    private final String variety;

    /** @param variety off | mild | strong (the prototype's tr.c2.variety, default mild) */
    public LoopRepairCandidate(String variety) {
        this.variety = variety;
    }

    @Override
    public String name() {
        return "c2";
    }

    protected String variety() {
        return variety;
    }

    /**
     * Optional external seed (round 2, c4): the via list of another candidate's solution for the same problem. Its
     * tile stops are mapped to this candidate's nearest location in the same tile. Null = only the given-order seeds.
     */
    protected List<Via> seedVias(Problem pb) {
        return null;
    }

    /** Whether the given-order seeds also compete with the external seed (c4: false, the free-order seed only). */
    protected boolean useGivenOrderSeeds() {
        return true;
    }

    /**
     * Merge note (round 1b): c2 was built on the foundation before the plain-via change. It keeps the old portal
     * placement (no junction clearance), which {@code midChord} reconstructs from placeM. Its own searches never used
     * EdgeDijkstra's reentry penalty.
     */
    @Override
    public void configure(Foundation.Params params) {
        params.nodeClearM = 0;
    }

    private List<PlainCostTable.Loc> locs;
    private int[] locTile;
    private List<List<Integer>> tileLocs;
    private double[][] D;
    private RouteGeom geom;
    private Random rnd;
    private final Map<String, int[]> moveStats = new TreeMap<>(); // tried, accepted(improving)
    final int maxShapes = 200;
    final int tableThreads = 4;
    /** A through-road portal with at least this much road inside the inset also gets a via location mid-chord. */
    final double throughChordM = 150;
    final int maxThroughLocs = 2;
    final double shapeP = 0.15;
    private PlainCostTable table;
    private Foundation f;
    private Problem pb;
    private int shapes;

    @Override
    public Solution solve(Problem pb) {
        Foundation f = pb.foundation;
        Solution sol = new Solution();
        sol.passThrough = false;
        rnd = new Random(seed * 31 + pb.jobId.hashCode());
        moveStats.clear();
        long t0 = System.nanoTime();

        // ---- locations: home + up to maxLocsPerTile distinct portal points per tile
        locs = new ArrayList<>();
        tileLocs = new ArrayList<>();
        PlainCostTable.Loc home = new PlainCostTable.Loc();
        home.id = 0;
        home.lat = pb.home.lat;
        home.lon = pb.home.lon;
        home.states.addAll(pb.home.states);
        locs.add(home);
        for (Foundation.TileInfo ti : pb.tiles) {
            List<Integer> ids = new ArrayList<>();
            for (Portal p : ti.portals) {
                if (ids.size() >= maxLocsPerTile) break;
                boolean near = false;
                for (int id : ids) near |= Foundation.dist(p.lat, p.lon, locs.get(id).lat, locs.get(id).lon) < locDedupM;
                if (near) continue;
                PlainCostTable.Loc l = new PlainCostTable.Loc();
                l.id = locs.size();
                l.lat = p.lat;
                l.lon = p.lon;
                l.tileKey = ti.tile.key;
                l.states.add(p);
                Portal r = reverse(f, p);
                if (r != null) l.states.add(r);
                locs.add(l);
                ids.add(l.id);
            }
            // through-road locations: the middle of a long chord, so a plain via there sits on a road the route can
            // ride straight through (a via just inside the border invites a duck-in-and-back)
            for (Portal p : ti.portals) {
                if (ids.size() >= maxLocsPerTile + maxThroughLocs || p.deadEnd || p.chordM < throughChordM) continue;
                PlainCostTable.Loc l = midChord(f, p);
                if (l == null) continue;
                boolean near = false;
                for (int id : ids) near |= Foundation.dist(l.lat, l.lon, locs.get(id).lat, locs.get(id).lon) < locDedupM;
                if (near) continue;
                l.id = locs.size();
                l.tileKey = ti.tile.key;
                locs.add(l);
                ids.add(l.id);
            }
            tileLocs.add(ids);
        }
        locTile = new int[locs.size() + maxShapes + 1];
        Arrays.fill(locTile, -1);
        for (int t = 0; t < tileLocs.size(); t++) for (int id : tileLocs.get(t)) locTile[id] = t;

        table = new PlainCostTable(f.graph, f.weighting);
        try {
            D = table.buildParallel(locs, f.params.maxSettled, locs.size() + maxShapes + 1, tableThreads);
        } catch (Exception ex) {
            throw new RuntimeException(ex);
        }
        this.f = f;
        this.pb = pb;
        shapes = 0;
        sol.tableMs = (System.nanoTime() - t0) / 1_000_000;
        sol.searches = table.searches;
        sol.settled = table.settled;
        if (table.anyBudgetHit) sol.unresolvedLegs++;

        // ---- seed: Viterbi on the given order, and on its reverse
        long t1 = clock.getAsLong();
        RouteGeom.Weights w = new RouteGeom.Weights(variety());
        geom = new RouteGeom(f, locs, pb.tiles, w);
        List<List<Integer>> seeds = new ArrayList<>();
        List<String> seedNames = new ArrayList<>();
        long ts0 = clock.getAsLong();
        List<Via> ext = seedVias(pb);
        long extMs = (clock.getAsLong() - ts0) / 1_000_000;
        if (ext != null) {
            List<Integer> s = stopsOf(ext);
            if (s != null) {
                seeds.add(s);
                seedNames.add("external");
            }
        }
        if (useGivenOrderSeeds() || seeds.isEmpty()) {
            List<Integer> order = new ArrayList<>();
            for (int t = 0; t < pb.tiles.size(); t++) order.add(t);
            // ordered groups: the given order, stably sorted by group rank
            order.sort(Comparator.comparingInt(t -> pb.rank(pb.tiles.get(t).tile.key)));
            seeds.add(viterbiOrder(order));
            seedNames.add("given");
            if (!pb.grouped()) {
                Collections.reverse(order);
                seeds.add(viterbiOrder(order));
                seedNames.add("reversed");
            }
        }
        // the reference weight per metre comes from the first seed (the external one when present)
        RouteGeom.Eval first = geom.calibrate(seeds.get(0), geom.eval(seeds.get(0)));
        List<Integer> cur = seeds.get(0);
        RouteGeom.Eval ce = first;
        int ci = 0;
        for (int k = 1; k < seeds.size(); k++) {
            RouteGeom.Eval e = geom.eval(seeds.get(k));
            if (e.objective < ce.objective) {
                cur = seeds.get(k);
                ce = e;
                ci = k;
            }
        }
        String seedNote = "seed(" + seedNames.get(ci) + (ext != null ? " extMs=" + extMs : "") + ") " + ce;
        List<Integer> best = cur;
        RouteGeom.Eval be = ce;

        // ---- ruin-and-recreate search
        // the external seed's own solve time does not eat the repair budget
        long deadline = t1 + budgetMs * 1_000_000L + extMs * 1_000_000L;
        int noImprove = 0, moves = 0, accepted = 0;
        boolean sa = false;
        double temp = 0, temp0 = 0;
        long saStart = 0;
        String stopReason = "budget";
        while (clock.getAsLong() < deadline) {
            if (pb.tiles.isEmpty()) {
                stopReason = "no tiles";
                break;
            }
            if (!sa && noImprove >= plateauMoves) {
                if (!anneal) {
                    stopReason = "plateau";
                    break;
                }
                sa = true;
                temp0 = 0.01 * ce.objective;
                saStart = clock.getAsLong();
                noImprove = 0;
            }
            if (sa && noImprove >= saPlateauMoves) {
                stopReason = "plateau(sa)";
                break;
            }
            if (sa) {
                double frac = (double) (clock.getAsLong() - saStart) / Math.max(1, deadline - saStart);
                temp = temp0 * Math.max(0, 1 - frac);
            }
            double r = rnd.nextDouble();
            String type = r < shapeP ? "shape" : r < shapeP + 0.4 ? "ruin" : r < shapeP + 0.55 ? "reverse" : r < 0.9 ? "swapLoc" : "drop";
            List<Integer> cand;
            switch (type) {
                case "shape":
                    cand = shape(cur, ce);
                    break;
                case "ruin":
                    cand = ruinRecreate(cur);
                    break;
                case "reverse":
                    cand = reverse(cur);
                    break;
                case "swapLoc":
                    cand = swapLoc(cur);
                    break;
                default:
                    cand = drop(cur);
            }
            if (cand == null || cand.equals(cur) || !orderOk(cand)) continue;
            moves++;
            int[] st = moveStats.computeIfAbsent(type, k -> new int[2]);
            st[0]++;
            RouteGeom.Eval e = geom.eval(cand);
            double delta = e.objective - ce.objective;
            boolean take = delta < -1e-6 || (sa && Double.isFinite(e.objective) && temp > 0 && rnd.nextDouble() < Math.exp(-delta / temp));
            if (take) {
                cur = cand;
                ce = e;
                accepted++;
            }
            if (e.objective < be.objective - 1e-6) {
                best = cand;
                be = e;
                st[1]++;
                noImprove = 0;
            } else noImprove++;
        }
        // ---- final clean-up: drop any stop whose tile the rest of the route already fetches
        for (int i = best.size() - 2; i >= 1; i--) {
            List<Integer> c = new ArrayList<>(best);
            c.remove(i);
            if (!orderOk(c)) continue;
            RouteGeom.Eval e = geom.eval(c);
            if (e.objective < be.objective - 1e-6) {
                best = c;
                be = e;
            }
        }
        sol.solveMs = (clock.getAsLong() - t1) / 1_000_000;

        for (int id : best) {
            PlainCostTable.Loc l = locs.get(id);
            sol.vias.add(id == 0 ? new Via(pb.home.lat, pb.home.lon, Double.NaN, "home", null)
                    : new Via(l.lat, l.lon, Double.NaN, l.tileKey == null ? "shape" : "portal", l.tileKey));
        }
        sol.searches = table.searches;
        sol.settled = table.settled;
        sol.keysHeld = table.keysHeldAll + table.peakKeys();
        StringBuilder ms = new StringBuilder();
        for (Map.Entry<String, int[]> en : moveStats.entrySet())
            ms.append(en.getKey()).append('=').append(en.getValue()[1]).append('/').append(en.getValue()[0]).append(' ');
        sol.note = String.format(Locale.ROOT, name() + " %s budgetMs=%d plateau=%d/%d anneal=%s shapeP=%.2f locs=%d shapes=%d | %s | best %s | moves=%d accepted=%d "
                        + "improving: %s| stop=%s sa=%s legsRouted=%d legMs=%d evals=%d", w, budgetMs, plateauMoves, saPlateauMoves,
                anneal, shapeP, locs.size(), shapes,
                seedNote, be, moves, accepted, ms, stopReason, sa, geom.legsRouted, geom.routeNanos / 1_000_000, geom.evals);
        return sol;
    }

    /**
     * Ordered groups (round 2): the tile stops' group ranks never decrease along the route (shape stops don't count),
     * and every grouped tile keeps a stop of its own, so it is not left to be fetched in passing late in the loop.
     * Always true for a job without groups.
     */
    private boolean orderOk(List<Integer> c) {
        if (!pb.grouped()) return true;
        int last = 0;
        Set<Integer> stopped = new HashSet<>();
        for (int id : c) {
            int t = locTile[id];
            if (t < 0) continue;
            int r = pb.rank(pb.tiles.get(t).tile.key);
            if (r < last) return false;
            last = r;
            stopped.add(t);
        }
        for (int t = 0; t < pb.tiles.size(); t++)
            if (pb.rank(pb.tiles.get(t).tile.key) < pb.groupCount && !stopped.contains(t)) return false;
        return true;
    }

    // ------------------------------------------------------------------ moves

    private List<Integer> ruinRecreate(List<Integer> cur) {
        int inner = cur.size() - 2;
        if (inner < 1) return null;
        int kMin = Math.min(2, inner), kMax = Math.min(5, inner);
        int k = kMin + rnd.nextInt(kMax - kMin + 1);
        int i = 1 + rnd.nextInt(inner - k + 1);
        List<Integer> rest = new ArrayList<>(cur.subList(0, i));
        rest.addAll(cur.subList(i + k, cur.size()));
        RouteGeom.Eval re = geom.eval(rest);
        if (re.coords == null) return null;
        List<Integer> released = new ArrayList<>();
        for (int t = 0; t < tileLocs.size(); t++) if (!re.fetched.get(t)) released.add(t);
        if (released.isEmpty()) return rest;
        // insertion order: random (diversity) or far-first
        if (rnd.nextBoolean()) Collections.shuffle(released, rnd);
        else {
            int a = rest.get(i - 1);
            released.sort(Comparator.comparingDouble(t -> -minD(a, t)));
        }
        boolean global = rnd.nextDouble() < 0.3;
        List<Integer> c = new ArrayList<>(rest);
        int lo = i - 1, hi = i; // the gap: insert between c[lo] and c[hi] (hi moves as we insert)
        for (int t : released) {
            int fromP = global ? 1 : lo + 1, toP = global ? c.size() - 1 : hi;
            double bestD = Double.POSITIVE_INFINITY;
            int bp = -1, bl = -1;
            for (int p = fromP; p <= toP; p++) {
                int a = c.get(p - 1), b = c.get(p);
                for (int l : tileLocs.get(t)) {
                    double d = D[a][l] + D[l][b] - D[a][b];
                    if (d < bestD) {
                        bestD = d;
                        bp = p;
                        bl = l;
                    }
                }
            }
            if (bp < 0) return null;
            c.add(bp, bl);
            if (bp <= hi) hi++;
            if (bp <= lo) lo++;
        }
        viterbiWindow(c, lo, hi);
        return c;
    }

    private List<Integer> reverse(List<Integer> cur) {
        int inner = cur.size() - 2;
        if (inner < 2) return null;
        int i = 1 + rnd.nextInt(inner), j = 1 + rnd.nextInt(inner);
        if (i == j) return null;
        if (i > j) {
            int x = i;
            i = j;
            j = x;
        }
        List<Integer> c = new ArrayList<>(cur);
        Collections.reverse(c.subList(i, j + 1));
        viterbiWindow(c, i - 1, j + 1);
        return c;
    }

    private List<Integer> swapLoc(List<Integer> cur) {
        int inner = cur.size() - 2;
        if (inner < 1) return null;
        int i = 1 + rnd.nextInt(inner);
        int t = locTile[cur.get(i)];
        if (t < 0 || tileLocs.get(t).size() < 2) return null;
        List<Integer> alts = new ArrayList<>(tileLocs.get(t));
        alts.remove(cur.get(i));
        List<Integer> c = new ArrayList<>(cur);
        c.set(i, alts.get(rnd.nextInt(alts.size())));
        return c;
    }

    /**
     * Insert a shape point (a plain via outside the tiles) into one leg, chosen by length: the leg's midpoint pushed
     * sideways by 10–50 % of the leg length (300 m … 5 km), snapped to the middle of the nearest rideable edge. Lets
     * the route return by another corridor instead of riding back along its own track.
     */
    private List<Integer> shape(List<Integer> cur, RouteGeom.Eval ce) {
        if (shapes >= maxShapes) return null;
        int legs = cur.size() - 1;
        double[] len = new double[legs];
        double tot = 0;
        for (int i = 0; i < legs; i++) tot += len[i] = geom.leg(cur.get(i), cur.get(i + 1)).distM;
        double pick = rnd.nextDouble() * tot;
        int i = 0;
        while (i < legs - 1 && pick > len[i]) pick -= len[i++];
        List<double[]> lc = geom.leg(cur.get(i), cur.get(i + 1)).coords;
        if (lc.size() < 2 || len[i] < 500) return null;
        // midpoint along the leg
        double half = len[i] / 2, acc = 0;
        double[] mid = lc.get(lc.size() / 2);
        for (int k = 0; k + 1 < lc.size(); k++) {
            double d = Foundation.dist(lc.get(k)[0], lc.get(k)[1], lc.get(k + 1)[0], lc.get(k + 1)[1]);
            if (acc + d >= half) {
                mid = lc.get(k);
                break;
            }
            acc += d;
        }
        double[] a = lc.get(0), b = lc.get(lc.size() - 1);
        double brg = Foundation.dist(a[0], a[1], b[0], b[1]) > 200 ? Foundation.bearing(a[0], a[1], b[0], b[1])
                : Foundation.bearing(lc.get(lc.size() / 4)[0], lc.get(lc.size() / 4)[1], lc.get(3 * lc.size() / 4)[0], lc.get(3 * lc.size() / 4)[1]);
        double off = Math.max(300, Math.min(5000, len[i] * (0.1 + 0.4 * rnd.nextDouble())));
        double side = rnd.nextBoolean() ? 90 : -90;
        double h = Math.toRadians(brg + side);
        double lat = mid[0] + off * Math.cos(h) / TileMath.M_PER_DEG_LAT;
        double lon = mid[1] + off * Math.sin(h) / (TileMath.M_PER_DEG_LON_EQ * Math.cos(Math.toRadians(mid[0])));
        PlainCostTable.Loc l = shapeLoc(lat, lon);
        if (l == null) return null;
        List<Integer> c = new ArrayList<>(cur);
        c.add(i + 1, l.id);
        return c;
    }

    /** A plain via at the middle of the rideable edge nearest to (lat, lon), reachable from and back to home. */
    private PlainCostTable.Loc shapeLoc(double lat, double lon) {
        com.graphhopper.storage.index.Snap snap = f.hopper.getLocationIndex().findClosest(lat, lon, f::rideable);
        if (!snap.isValid()) return null;
        EdgeIteratorState e = snap.getClosestEdge();
        com.graphhopper.util.PointList pl = e.fetchWayGeometry(com.graphhopper.util.FetchMode.ALL);
        double total = 0;
        for (int k = 0; k + 1 < pl.size(); k++) total += Foundation.dist(pl.getLat(k), pl.getLon(k), pl.getLat(k + 1), pl.getLon(k + 1));
        if (total < 40) return null;
        double acc = 0, mlat = pl.getLat(0), mlon = pl.getLon(0);
        for (int k = 0; k + 1 < pl.size(); k++) {
            double d = Foundation.dist(pl.getLat(k), pl.getLon(k), pl.getLat(k + 1), pl.getLon(k + 1));
            if (acc + d >= total / 2) {
                double t = d > 0 ? (total / 2 - acc) / d : 0;
                mlat = pl.getLat(k) + t * (pl.getLat(k + 1) - pl.getLat(k));
                mlon = pl.getLon(k) + t * (pl.getLon(k + 1) - pl.getLon(k));
                break;
            }
            acc += d;
        }
        // ordered groups: the verifier ranks every via by the tile it lies in, so a shape point stays out of the tiles
        if (pb.grouped())
            for (Foundation.TileInfo ti : pb.tiles)
                if (mlat >= ti.tile.minLat && mlat <= ti.tile.maxLat && mlon >= ti.tile.minLon && mlon <= ti.tile.maxLon) return null;
        PlainCostTable.Loc l = new PlainCostTable.Loc();
        l.lat = mlat;
        l.lon = mlon;
        boolean fwd = false, back = false;
        for (int key : new int[]{e.getEdgeKey(), e.getReverseEdgeKey()}) {
            EdgeIteratorState st = f.graph.getEdgeIteratorStateForKey(key);
            double w = f.weighting.calcEdgeWeight(st, false);
            if (!Double.isFinite(w)) continue;
            fwd |= pb.home.fwd.get(key);
            back |= pb.home.back.get(key);
            Portal p = new Portal();
            p.tileKey = "shape";
            p.edge = st.getEdge();
            p.key = key;
            p.fromNode = st.getBaseNode();
            p.toNode = st.getAdjNode();
            p.lat = mlat;
            p.lon = mlon;
            p.frac = 0.5;
            p.edgeWeight = w;
            p.edgeLenM = total;
            l.states.add(p);
        }
        if (!fwd || !back || l.states.isEmpty()) return null;
        l.id = locs.size();
        locs.add(l);
        locTile[l.id] = -1;
        shapes++;
        // proposals only (a shape stop is always judged on its routed legs): straight line × detour, both ways
        for (int k = 0; k < locs.size(); k++) {
            double d = k == l.id ? 0 : PlainCostTable.DETOUR * Foundation.dist(l.lat, l.lon, locs.get(k).lat, locs.get(k).lon);
            D[l.id][k] = d;
            D[k][l.id] = d;
        }
        return l;
    }

    private List<Integer> drop(List<Integer> cur) {
        int inner = cur.size() - 2;
        if (inner < 1) return null;
        List<Integer> c = new ArrayList<>(cur);
        c.remove(1 + rnd.nextInt(inner));
        return c;
    }

    // ------------------------------------------------------------------ Viterbi on the plain table

    /** Home → the tiles in this order → home, best location per tile. */
    private List<Integer> viterbiOrder(List<Integer> tileOrder) {
        List<Integer> c = new ArrayList<>();
        c.add(0);
        for (int t : tileOrder) c.add(tileLocs.get(t).get(0));
        c.add(0);
        viterbiWindow(c, 0, c.size() - 1);
        return c;
    }

    /** Re-pick the location of every tile stop strictly between the frozen stops c[lo] and c[hi]. */
    private void viterbiWindow(List<Integer> c, int lo, int hi) {
        if (hi - lo < 2) return;
        int m = hi - lo - 1;
        List<List<Integer>> opts = new ArrayList<>();
        for (int q = 0; q < m; q++) {
            int id = c.get(lo + 1 + q);
            opts.add(locTile[id] >= 0 ? tileLocs.get(locTile[id]) : List.of(id));
        }
        double[][] sc = new double[m][];
        int[][] bp = new int[m][];
        for (int q = 0; q < m; q++) {
            List<Integer> o = opts.get(q);
            sc[q] = new double[o.size()];
            bp[q] = new int[o.size()];
            for (int s = 0; s < o.size(); s++) {
                int l = o.get(s);
                if (q == 0) {
                    sc[q][s] = D[c.get(lo)][l];
                    continue;
                }
                double bv = Double.POSITIVE_INFINITY;
                int bs = 0;
                List<Integer> po = opts.get(q - 1);
                for (int ps = 0; ps < po.size(); ps++) {
                    double v = sc[q - 1][ps] + D[po.get(ps)][l];
                    if (v < bv) {
                        bv = v;
                        bs = ps;
                    }
                }
                sc[q][s] = bv;
                bp[q][s] = bs;
            }
        }
        int end = c.get(hi);
        double bv = Double.POSITIVE_INFINITY;
        int bs = 0;
        for (int s = 0; s < opts.get(m - 1).size(); s++) {
            double v = sc[m - 1][s] + D[opts.get(m - 1).get(s)][end];
            if (v < bv) {
                bv = v;
                bs = s;
            }
        }
        if (!Double.isFinite(bv)) return;
        for (int q = m - 1; q >= 0; q--) {
            c.set(lo + 1 + q, opts.get(q).get(bs));
            bs = bp[q][bs];
        }
    }

    /** Map an external via list to stops: home → 0, each tile via → the nearest location of that tile. */
    private List<Integer> stopsOf(List<Via> vias) {
        Map<String, Integer> tileIdx = new HashMap<>();
        for (int t = 0; t < pb.tiles.size(); t++) tileIdx.put(pb.tiles.get(t).tile.key, t);
        List<Integer> s = new ArrayList<>();
        for (Via v : vias) {
            if ("home".equals(v.role)) {
                if (s.isEmpty() || s.get(s.size() - 1) != 0 || s.size() == 1) s.add(0);
                continue;
            }
            if (v.tileKey == null) continue; // a via outside every tile (e.g. a stroke point): c2 has no location for it
            Integer t = tileIdx.get(v.tileKey);
            if (t == null || tileLocs.get(t).isEmpty()) continue;
            int best = -1;
            double bd = Double.POSITIVE_INFINITY;
            for (int id : tileLocs.get(t)) {
                double d = Foundation.dist(v.lat, v.lon, locs.get(id).lat, locs.get(id).lon);
                if (d < bd) {
                    bd = d;
                    best = id;
                }
            }
            s.add(best);
        }
        if (s.size() < 2 || s.get(0) != 0 || s.get(s.size() - 1) != 0) return null;
        return s;
    }

    private double minD(int a, int t) {
        double m = Double.POSITIVE_INFINITY;
        for (int l : tileLocs.get(t)) m = Math.min(m, D[a][l]);
        return m;
    }

    /** A location at the middle of the portal's chord inside the inset (on the same directed edge), both directions. */
    private static PlainCostTable.Loc midChord(Foundation f, Portal p) {
        EdgeIteratorState st = f.graph.getEdgeIteratorStateForKey(p.key);
        com.graphhopper.util.PointList pl = st.fetchWayGeometry(com.graphhopper.util.FetchMode.ALL);
        double len = p.edgeLenM;
        if (len <= 0) return null;
        double s = p.frac * len - Math.min(f.params.placeM, p.chordM / 2) + p.chordM / 2;
        if (s <= 20 || s >= len - 20) return null; // keep clear of the junctions at the edge ends
        double acc = 0, lat = Double.NaN, lon = Double.NaN;
        for (int k = 0; k + 1 < pl.size(); k++) {
            double d = Foundation.dist(pl.getLat(k), pl.getLon(k), pl.getLat(k + 1), pl.getLon(k + 1));
            if (acc + d >= s) {
                double t = d > 0 ? (s - acc) / d : 0;
                lat = pl.getLat(k) + t * (pl.getLat(k + 1) - pl.getLat(k));
                lon = pl.getLon(k) + t * (pl.getLon(k + 1) - pl.getLon(k));
                break;
            }
            acc += d;
        }
        if (Double.isNaN(lat)) return null;
        Portal q = new Portal();
        q.tileKey = p.tileKey;
        q.edge = p.edge;
        q.key = p.key;
        q.fromNode = p.fromNode;
        q.toNode = p.toNode;
        q.lat = lat;
        q.lon = lon;
        q.heading = p.heading;
        q.frac = s / len;
        q.edgeWeight = p.edgeWeight;
        q.edgeLenM = len;
        q.chordM = p.chordM;
        q.name = p.name;
        PlainCostTable.Loc l = new PlainCostTable.Loc();
        l.lat = lat;
        l.lon = lon;
        l.states.add(q);
        Portal r = reverse(f, q);
        if (r != null) l.states.add(r);
        return l;
    }

    private static Portal reverse(Foundation f, Portal p) {
        BaseGraph g = f.graph;
        EdgeIteratorState st = g.getEdgeIteratorStateForKey(p.key ^ 1);
        double w = f.weighting.calcEdgeWeight(st, false);
        if (!Double.isFinite(w)) return null;
        Portal r = new Portal();
        r.tileKey = p.tileKey;
        r.edge = p.edge;
        r.key = p.key ^ 1;
        r.fromNode = st.getBaseNode();
        r.toNode = st.getAdjNode();
        r.lat = p.lat;
        r.lon = p.lon;
        r.heading = (p.heading + 180) % 360;
        r.frac = 1 - p.frac;
        r.edgeWeight = w;
        r.edgeLenM = p.edgeLenM;
        r.side = p.side;
        r.name = p.name;
        return r;
    }
}
