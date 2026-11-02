package com.graphhopper.trailmap.tilerouting.strokes;

import com.graphhopper.trailmap.tilerouting.foundation.*;

import java.util.*;
import java.util.concurrent.ForkJoinPool;
import java.util.stream.IntStream;

/**
 * Candidate 3 — street-stroke cover.
 * <ol>
 * <li>Strokes: rideable edges near the tiles chained across smooth junctions ({@link Strokes}); each records where it
 * runs inside a requested tile's inset ("hits").</li>
 * <li>Options: a <em>section</em> = ride one stroke from a hit in tile A to a hit in tile B (both directions), which
 * fetches every tile hit on the way; a <em>single</em> = one plain via point inside one tile (a truncated spur when the
 * road is a dead end; the foundation's portals are singles too).</li>
 * <li>Costs between option endpoints: edge-based Dijkstra with plain-via semantics ({@link PathDijkstra}), which
 * also records the tiles each connector crosses (free coverage) and the riding direction at both ends, so a
 * turnaround at a via point is priced ({@link #uTurnPenalty}).</li>
 * <li>Greedy cover: insert the option with the least added cost per newly fetched tile at its best tour position,
 * until every tile is fetched; then drop redundant options, relocate / flip options and 2-opt the tour.</li>
 * <li>Output: plain via points — entry and exit of each section (plus mid points where the shortest path between
 * them would leave the stroke and miss a tile), one point per single.</li>
 * </ol>
 * The given order is not used (free order). No Euler tour.
 */
public class StrokeCoverCandidate implements Candidate {

    // the prototype's tr.c3.* defaults
    public double uTurnPenalty = 100;
    private String rrNote = "";
    public int keepHitsPerTile = 4;
    public int maxSettled = 3_000_000;
    public int threads = 6;
    public int repairRounds = 2;
    private static final ForkJoinPool POOL = new ForkJoinPool(6);
    private static final int HOME = -1;

    @Override
    public String name() {
        return "c3";
    }

    /** Merge note (round 1b): c3 was built on the foundation before the plain-via change; keep its old portal placement. */
    @Override
    public void configure(Foundation.Params params) {
        params.nodeClearM = 0;
    }

    // ------------------------------------------------------------------ state of one solve

    static final class Opt {
        int id, twin = -1;
        boolean section;
        int[] pts;
        byte entryDir, exitDir;
        double internal;
        int[] tiles;
        Strokes.Stroke stroke;
        Strokes.Hit[] hits; // section: hits in riding order
        /** Ordered groups: the group rank of the first and last tile the option's via points sit in. */
        int rLo, rHi;
        /** Ordered groups: a section whose tiles' ranks fall along its riding direction can never be used. */
        boolean banned;
    }

    /** Ordered groups: rank per tile index (null = free order, the default). */
    private int[] tileRank;

    /** Fills each option's rank span; a section is banned unless its hits' ranks are non-decreasing in riding order. */
    private void rankOptions(Problem pb) {
        tileRank = null;
        if (!pb.grouped()) return;
        tileRank = new int[nTiles];
        for (int t = 0; t < nTiles; t++) tileRank[t] = pb.rank(pb.tiles.get(t).tile.key);
        for (Opt o : opts) {
            if (!o.section) {
                o.rLo = o.rHi = tileRank[o.tiles[0]];
                continue;
            }
            int prev = Integer.MIN_VALUE;
            for (Strokes.Hit h : o.hits) {
                int r = tileRank[h.tile];
                if (r < prev) o.banned = true;
                prev = Math.max(prev, r);
            }
            o.rLo = tileRank[o.hits[0].tile];
            o.rHi = tileRank[o.hits[o.hits.length - 1].tile];
        }
        // a grouped tile must be fetched by its own via (in rank order), never only in passing by a connector
        groupCount = pb.groupCount;
        for (int t = 0; t < nTiles; t++) if (tileRank[t] < groupCount) ownRequired[t] = true;
    }

    private int groupCount;

    /** Initial force set for greedy: grouped tiles (they need an own option); all false in free order. */
    private boolean[] groupForce() {
        boolean[] f = new boolean[nTiles];
        if (tileRank != null) for (int t = 0; t < nTiles; t++) f[t] = tileRank[t] < groupCount;
        return f;
    }

    /** Ordered groups: every option allowed and consecutive options in non-decreasing rank. */
    private boolean orderOk(List<Integer> tour) {
        if (tileRank == null) return true;
        int prev = Integer.MIN_VALUE;
        for (int e : tour) {
            Opt o = opts.get(e);
            if (o.banned || o.rLo < prev) return false;
            prev = o.rHi;
        }
        return true;
    }

    /** Ordered groups: may option o go between tour positions p and p+1 of the full sequence s (home at both ends)? */
    private boolean fits(int[] s, int p, Opt o) {
        if (tileRank == null) return true;
        if (o.banned) return false;
        if (p > 0 && opts.get(s[p]).rHi > o.rLo) return false;
        return p + 1 >= s.length - 1 || o.rHi <= opts.get(s[p + 1]).rLo;
    }

    private Foundation f;
    private Strokes strokes;
    private final List<PathDijkstra.Pt> pts = new ArrayList<>();
    private final List<Double> ptS = new ArrayList<>();
    private final List<Opt> opts = new ArrayList<>();
    private double[][] C;
    private byte[][] A, D;
    private int[][][] T;
    private int nTiles;
    private EdgeTiles et;
    /**
     * One searcher per thread, owned by this candidate (one per request), so it is freed with the request: a
     * ThreadLocal would stay referenced from the shared pool's threads.
     */
    private final Map<Thread, PathDijkstra> dij = new java.util.concurrent.ConcurrentHashMap<>();

    private PathDijkstra dijkstra() {
        PathDijkstra d = dij.get(Thread.currentThread());
        if (d == null) {
            d = new PathDijkstra(f.graph, f.weighting, et, weights);
            dij.put(Thread.currentThread(), d);
        } else if (d.edgeTiles() != et) d.setEdgeTiles(et);
        return d;
    }

    /** Edge weights shared by this request's searchers (PathDijkstra); made in {@link #solve}. */
    private PagedWeights weights;

    private long keysHeld() {
        long n = weights.keysHeld();
        for (PathDijkstra d : dij.values()) n += d.keysHeld();
        return n;
    }

    @Override
    public Solution solve(Problem pb) {
        Solution sol = new Solution();
        sol.passThrough = false;
        f = pb.foundation;
        weights = new PagedWeights(2 * f.graph.getEdges());
        pts.clear();
        ptS.clear();
        opts.clear();
        K = null;
        if (pb.tiles.isEmpty()) {
            sol.vias.add(new Via(pb.home.lat, pb.home.lon, Double.NaN, "home", null));
            sol.vias.add(new Via(pb.home.lat, pb.home.lon, Double.NaN, "home", null));
            return sol;
        }
        long t0 = System.nanoTime();
        int z = pb.tiles.get(0).tile.z;
        boolean small = z >= 17;
        double minChord = small ? 10 : 20, place = small ? 10 : 15, maxGap = small ? 300 : 1500,
                maxSpan = small ? 3000 : 12000, buffer = small ? 300 : 1500;
        // a hit gets an entry point at each end only when it is long; short hits get one point in the middle
        double twoPointMinM = small ? 120 : 2 * place + 10;

        List<TileMath.Tile> tiles = new ArrayList<>();
        for (Foundation.TileInfo ti : pb.tiles) tiles.add(ti.tile);
        nTiles = tiles.size();
        ownRequired = new boolean[nTiles];
        midS.clear();
        et = new EdgeTiles(f, tiles);
        strokes = new Strokes(f, et);
        strokes.build(buffer);
        strokes.hits(minChord, maxGap);

        // home point
        Portal hs = pb.home.states.get(0);
        PathDijkstra.Pt home = new PathDijkstra.Pt();
        home.edge = hs.edge;
        home.f = (hs.key & 1) == 0 ? hs.frac : 1 - hs.frac;
        home.lat = hs.lat;
        home.lon = hs.lon;
        home.label = "home";
        addPt(home, Double.NaN);

        // keep the best hits per tile as option endpoints
        Map<Integer, List<Strokes.Hit>> byTile = new HashMap<>();
        for (Strokes.Stroke s : strokes.strokes) for (Strokes.Hit h : s.hits) byTile.computeIfAbsent(h.tile, k -> new ArrayList<>()).add(h);
        Map<Strokes.Hit, int[]> hitPts = new HashMap<>();
        int hitsTotal = 0;
        for (int t = 0; t < nTiles; t++) {
            List<Strokes.Hit> l = byTile.getOrDefault(t, List.of());
            hitsTotal += l.size();
            List<Strokes.Hit> sorted = new ArrayList<>(l);
            sorted.sort(Comparator.<Strokes.Hit>comparingInt(h -> -h.clusterTiles).thenComparingDouble(h -> h.weightPerM)
                    .thenComparingDouble(h -> -h.len()));
            int kept = 0;
            for (Strokes.Hit h : sorted) {
                if (kept >= keepHitsPerTile) break;
                double p = Math.min(place, h.len() / 2);
                int[] ids;
                if (h.len() < twoPointMinM) {
                    double c = (h.s0 + h.s1) / 2;
                    PathDijkstra.Pt q = strokes.point(h.stroke, c);
                    if (!valid(pb.home, q)) continue;
                    int id = addPt(q, c);
                    ids = new int[]{id, id};
                } else {
                    PathDijkstra.Pt a = strokes.point(h.stroke, h.s0 + p), b = strokes.point(h.stroke, h.s1 - p);
                    if (!valid(pb.home, a) || !valid(pb.home, b)) continue;
                    ids = new int[]{addPt(a, h.s0 + p), addPt(b, h.s1 - p)};
                }
                pts.get(ids[0]).label = "hit:t" + t;
                pts.get(ids[1]).label = "hit:t" + t;
                h.kept = true;
                hitPts.put(h, ids);
                kept++;
                single(ids[0], t);
                if (ids[1] != ids[0]) single(ids[1], t);
            }
            // r4: with the via swap, the foundation's portals are offered for every tile (the tile-10 case: the good
            // via was a portal, but the tile had enough stroke hits that its portals were never added)
            if (kept < 2 || (SWAP_VIA && PORTALS_ALWAYS)) {
                for (Portal po : pb.tiles.get(t).portals) {
                    PathDijkstra.Pt q = new PathDijkstra.Pt();
                    q.edge = po.edge;
                    q.f = (po.key & 1) == 0 ? po.frac : 1 - po.frac;
                    q.lat = po.lat;
                    q.lon = po.lon;
                    q.label = "portal:t" + t;
                    single(addPt(q, Double.NaN), t);
                    // walk portal (round 3): the walk in and out, priced per metre well above any road
                    if (po.walkM > 0) opts.get(opts.size() - 1).internal = 2 * po.walkM * f.params.walkWeightPerM;
                }
            }
        }
        // sections
        int sections = 0;
        for (Strokes.Stroke s : strokes.strokes) {
            List<Strokes.Hit> hs2 = s.hits;
            for (int i = 0; i < hs2.size(); i++) {
                Strokes.Hit hi = hs2.get(i);
                if (!hi.kept) continue;
                Set<Integer> ts = new LinkedHashSet<>();
                ts.add(hi.tile);
                for (int j = i + 1; j < hs2.size(); j++) {
                    Strokes.Hit hj = hs2.get(j);
                    if (hj.s0 - hs2.get(j - 1).s1 > maxGap || hj.s1 - hi.s0 > maxSpan) break;
                    ts.add(hj.tile);
                    if (!hj.kept || ts.size() < 2 || hj.tile == hi.tile) continue;
                    int[] pi = hitPts.get(hi), pj = hitPts.get(hj);
                    int[] tileArr = ts.stream().mapToInt(Integer::intValue).toArray();
                    Strokes.Hit[] fwdHits = hs2.subList(i, j + 1).toArray(new Strokes.Hit[0]);
                    Strokes.Hit[] bwdHits = fwdHits.clone();
                    Collections.reverse(Arrays.asList(bwdHits));
                    Opt fw = section(s, pi[0], pj[1], true, tileArr, fwdHits);
                    Opt bw = section(s, pj[1], pi[0], false, tileArr, bwdHits);
                    if (fw != null && bw != null) {
                        fw.twin = bw.id;
                        bw.twin = fw.id;
                    }
                    if (fw != null) sections++;
                    if (bw != null) sections++;
                }
            }
        }
        rankOptions(pb);
        long tPrep = System.nanoTime();

        // table over all points
        int n = pts.size();
        PathDijkstra.Targets tg = new PathDijkstra.Targets(pts, f.weighting, f.graph, pb.home.fwd);
        C = new double[n][];
        A = new byte[n][];
        D = new byte[n][];
        T = new int[n][][];
        this.targets = tg;
        exact = new boolean[n];
        stSettled = 0;
        stSearches = 0;
        // home row exact; it also calibrates weight per metre for the bounded rows and the estimates
        row(0, Double.POSITIVE_INFINITY);
        List<Double> ratios = new ArrayList<>();
        for (int j = 1; j < n; j++) {
            double d = Foundation.dist(pts.get(0).lat, pts.get(0).lon, pts.get(j).lat, pts.get(j).lon);
            if (d > 300 && Double.isFinite(C[0][j])) ratios.add(C[0][j] / d);
        }
        Collections.sort(ratios);
        wpm = ratios.isEmpty() ? 0.3 : ratios.get(ratios.size() / 2);
        double bound = wpm * (small ? boundSmallM : boundBigM);
        POOL.submit(() -> IntStream.range(1, n).parallel().forEach(i -> row(i, bound))).join();
        long tTable = System.nanoTime();

        // greedy cover + improvement; legs still priced by estimate get an exact row and the tour is improved again
        List<Integer> tour = new ArrayList<>();
        greedy(tour, groupForce());
        improve(tour);
        double costGreedy = tourCost(tour);
        int ruins = ruinRecreate(tour);
        rrNote = String.format(Locale.ROOT, " greedyCost=%.0f ruinAccepted=%d", costGreedy, ruins);
        int lazyRows = 0;
        for (int round = 0; round < 8; round++) {
            Set<Integer> need = new TreeSet<>();
            int[] s = full(tour);
            for (int k = 0; k + 1 < s.length; k++) {
                int a = exit(s[k]), b = entry(s[k + 1]);
                if (!known(a, b) && !exact[a]) need.add(a);
            }
            if (need.isEmpty()) break;
            lazyRows += need.size();
            POOL.submit(() -> need.parallelStream().forEach(i -> row(i, Double.POSITIVE_INFINITY))).join();
            greedy(tour, new boolean[nTiles]);
            improve(tour);
        }
        tLazyMs = lazyRows;
        // vias, then a GraphHopper check of the actual plain-via route with repair of missed tiles
        List<Via> vias = vias(tour, pb);
        String repairNote = "";
        for (int round = 0; round < repairRounds; round++) {
            Foundation.Route route = f.assemble(vias, false);
            if (route.error != null) {
                repairNote += " routeError";
                break;
            }
            boolean[] force = new boolean[nTiles];
            int missed = 0;
            for (int t = 0; t < nTiles; t++)
                // a walk tile (round 3) is fetched by the walk from its single, never by the routed line
                if (!pb.tiles.get(t).walk && !Foundation.fetched(tiles.get(t), route.coords)) {
                    force[t] = true;
                    ownRequired[t] = true;
                    missed++;
                }
            if (missed == 0) break;
            repairNote += " repair" + round + "=" + missed;
            greedy(tour, force);
            improve(tour);
            vias = vias(tour, pb);
        }
        sol.vias = vias;
        long tEnd = System.nanoTime();

        // stats
        Set<Integer> multi = new HashSet<>();
        int nSec = 0, nSingle = 0;
        for (int o : tour) {
            Opt op = opts.get(o);
            if (op.section) {
                nSec++;
                for (int t : op.tiles) multi.add(t);
            } else nSingle++;
        }
        sol.tableMs = (tTable - tPrep) / 1_000_000;
        sol.solveMs = (tPrep - t0 + tEnd - tTable) / 1_000_000;
        sol.searches = stSearches;
        sol.settled = stSettled;
        sol.note = String.format(Locale.ROOT,
                "strokes=%d hits=%d points=%d options=%d (sections %d) tour=%d (sections %d, singles %d) multiTileShare=%.2f cost=%.0f prepMs=%d lazyRows=%d wpm=%.3f uTurnPenalty=%.0f keep=%d bound=%.0f/%.0fm%s",
                strokes.strokes.size(), hitsTotal, n, opts.size(), sections, tour.size(), nSec, nSingle,
                (double) multi.size() / nTiles, tourCost(tour), (tPrep - t0) / 1_000_000, tLazyMs, wpm, uTurnPenalty, keepHitsPerTile,
                boundSmallM, boundBigM, repairNote) + rrNote
                + (tileRank != null ? " groups=" + pb.groupCount + " bannedSections=" + opts.stream().filter(o -> o.banned).count()
                + " orderOk=" + orderOk(tour) : "");
        multiShare = (double) multi.size() / nTiles;
        sol.keysHeld = keysHeld();
        return sol;
    }

    // ------------------------------------------------------------------ sparse cost table with lazy exact rows

    /** Bounded rows: exact costs only within this riding distance (as weight via the home row's weight per metre). */
    public double boundSmallM = Double.POSITIVE_INFINITY, boundBigM = Double.POSITIVE_INFINITY;
    /** Estimate for pairs beyond the bound: straight line × weight per metre (median of the home row) × this factor. */
    public double detourFactor = 1.0;
    private PathDijkstra.Targets targets;
    private boolean[] exact;
    private boolean[][] K;
    private double wpm;
    private long stSettled, stSearches;
    private int tLazyMs;
    double multiShare;

    private void row(int i, double bound) {
        PathDijkstra.Row r = dijkstra().search(pts.get(i), targets, maxSettled, true, bound);
        int n = pts.size();
        double[] c = new double[n];
        boolean[] k = new boolean[n];
        byte[] a = new byte[n], d = new byte[n];
        for (int j = 0; j < n; j++) {
            if (Double.isFinite(r.cost[j])) {
                c[j] = r.cost[j];
                k[j] = true;
                a[j] = r.arr[j];
                d[j] = r.dep[j];
            } else if (!Double.isFinite(bound) && !r.budgetHit) {
                c[j] = Double.POSITIVE_INFINITY; // truly unreachable
                k[j] = true;
            } else {
                c[j] = detourFactor * wpm * Foundation.dist(pts.get(i).lat, pts.get(i).lon, pts.get(j).lat, pts.get(j).lon);
                a[j] = -1;
                d[j] = -1;
            }
        }
        if (!Double.isFinite(bound)) exact[i] = true;
        synchronized (this) {
            C[i] = c;
            A[i] = a;
            D[i] = d;
            T[i] = r.tiles;
            if (K == null || K.length != n) K = new boolean[n][];
            K[i] = k;
            stSettled += r.settled;
            stSearches++;
        }
    }

    private boolean known(int a, int b) {
        return K[a][b];
    }

    private boolean valid(Foundation.Home h, PathDijkstra.Pt p) {
        boolean in = h.fwd.get(2 * p.edge) || h.fwd.get(2 * p.edge + 1);
        boolean out = h.back.get(2 * p.edge) || h.back.get(2 * p.edge + 1);
        return in && out;
    }

    private int addPt(PathDijkstra.Pt p, double s) {
        p.id = pts.size();
        pts.add(p);
        ptS.add(s);
        return p.id;
    }

    private void single(int pt, int tile) {
        Opt o = new Opt();
        o.id = opts.size();
        o.pts = new int[]{pt};
        o.tiles = new int[]{tile};
        opts.add(o);
    }

    private Opt section(Strokes.Stroke s, int entry, int exit, boolean fwd, int[] tiles, Strokes.Hit[] hits) {
        double a = ptS.get(entry), b = ptS.get(exit);
        double c = strokes.cost(s, a, b);
        if (!Double.isFinite(c)) return null;
        Opt o = new Opt();
        o.id = opts.size();
        o.section = true;
        o.pts = new int[]{entry, exit};
        o.entryDir = Strokes.dirAt(s, a, fwd);
        o.exitDir = Strokes.dirAt(s, b, fwd);
        o.internal = c;
        o.tiles = tiles;
        o.stroke = s;
        o.hits = hits;
        opts.add(o);
        return o;
    }

    // ------------------------------------------------------------------ tour cost model

    private int entry(int o) {
        return o == HOME ? 0 : opts.get(o).pts[0];
    }

    private int exit(int o) {
        if (o == HOME) return 0;
        int[] p = opts.get(o).pts;
        return p[p.length - 1];
    }

    private double leg(int a, int b) {
        return C[exit(a)][entry(b)];
    }

    private int arr(int a, int b) {
        return A[exit(a)][entry(b)];
    }

    private int dep(int a, int b) {
        return D[exit(a)][entry(b)];
    }

    private int rev(int e, int arrIn, int depOut) {
        if (e == HOME) return 0;
        Opt o = opts.get(e);
        if (!o.section) return arrIn != depOut ? 1 : 0;
        return (arrIn != o.entryDir ? 1 : 0) + (o.exitDir != depOut ? 1 : 0);
    }

    /** Cost of riding the elements in seq, legs between them and turnarounds at each, with the boundary directions given. */
    private double segCost(int[] seq, int arrIntoFirst, int depOutLast) {
        double c = 0;
        int revs = 0;
        for (int k = 0; k < seq.length; k++) {
            int e = seq[k];
            if (k > 0 && k < seq.length - 1 && e != HOME) c += opts.get(e).internal;
            if (k + 1 < seq.length) {
                double l = leg(e, seq[k + 1]);
                if (!Double.isFinite(l)) return Double.POSITIVE_INFINITY;
                c += l;
            }
            int ai = k == 0 ? arrIntoFirst : arr(seq[k - 1], e);
            int dO = k == seq.length - 1 ? depOutLast : dep(e, seq[k + 1]);
            if (ai >= 0 && dO >= 0) revs += rev(e, ai, dO);
        }
        return c + uTurnPenalty * revs;
    }

    private int[] full(List<Integer> tour) {
        int[] s = new int[tour.size() + 2];
        s[0] = HOME;
        for (int i = 0; i < tour.size(); i++) s[i + 1] = tour.get(i);
        s[s.length - 1] = HOME;
        return s;
    }

    double tourCost(List<Integer> tour) {
        if (!orderOk(tour)) return Double.POSITIVE_INFINITY;
        int[] s = full(tour);
        double c = segCost(s, -1, -1);
        for (int o : tour) c += 0; // internal already summed for inner elements
        return c;
    }

    private int[] coverage(int[] s) {
        int[] cov = new int[nTiles];
        for (int k = 0; k < s.length; k++) {
            if (s[k] != HOME) for (int t : opts.get(s[k]).tiles) cov[t]++;
            if (k + 1 < s.length) {
                int[] lt = T[exit(s[k])][entry(s[k + 1])];
                if (lt != null) for (int t : lt) cov[t]++;
            }
        }
        return cov;
    }

    /** Tiles the real route missed although a connector "covered" them: from now on an option must fetch them. */
    private boolean[] ownRequired;

    private boolean complete(int[] cov, boolean[] unused, int[] s) {
        for (int c : cov) if (c == 0) return false;
        if (ownRequired != null && s != null) {
            boolean[] own = new boolean[nTiles];
            for (int e : s) if (e != HOME) for (int t : opts.get(e).tiles) own[t] = true;
            for (int t = 0; t < nTiles; t++) if (ownRequired[t] && !own[t]) return false;
        }
        return true;
    }

    // ------------------------------------------------------------------ greedy cover

    /** Insert options until every tile is covered; tiles in {@code force} must be covered by an option's own tiles. */
    private void greedy(List<Integer> tour, boolean[] force) {
        int[] mark = new int[nTiles];
        int markRun = 0;
        Set<Integer> inTour = new HashSet<>(tour);
        for (int iter = 0; iter < 10 * nTiles + 50; iter++) {
            int[] s = full(tour);
            int[] cov = coverage(s);
            boolean[] need = new boolean[nTiles];
            boolean any = false;
            for (int t = 0; t < nTiles; t++) {
                need[t] = cov[t] == 0 || force[t];
                any |= need[t];
            }
            if (!any) return;
            double bestRatio = Double.POSITIVE_INFINITY;
            int bestO = -1, bestP = -1;
            for (Opt o : opts) {
                if (inTour.contains(o.id)) continue;
                int own = 0;
                for (int t : o.tiles) if (need[t]) own++;
                if (own == 0) continue;
                for (int p = 0; p + 1 < s.length; p++) {
                    if (!fits(s, p, o)) continue;
                    int a = s[p], b = s[p + 1];
                    int arrA = p == 0 ? -1 : arr(s[p - 1], a);
                    int depB = p + 2 < s.length ? dep(b, s[p + 2]) : -1;
                    double nw = segCost(new int[]{a, o.id, b}, arrA, depB);
                    if (!Double.isFinite(nw)) continue;
                    double old = segCost(new int[]{a, b}, arrA, depB);
                    double delta = nw - old;
                    // net newly covered tiles: own + new connectors − tiles only the replaced leg covered
                    markRun++;
                    int gain = 0;
                    for (int t : o.tiles) {
                        mark[t] = markRun;
                        if (need[t]) gain++;
                    }
                    int[] l1 = T[exit(a)][entry(o.id)], l2 = T[exit(o.id)][entry(b)], l0 = T[exit(a)][entry(b)];
                    for (int[] l : new int[][]{l1, l2})
                        if (l != null)
                            for (int t : l)
                                if (mark[t] != markRun) {
                                    mark[t] = markRun;
                                    if (cov[t] == 0) gain++;
                                }
                    if (l0 != null) for (int t : l0) if (cov[t] == 1 && mark[t] != markRun) gain--;
                    // ordered groups: the order may leave only positions whose replaced leg loses more tiles than
                    // the option adds; keep such an insertion as a last resort (the lost tiles are re-covered by
                    // the next iterations) instead of stalling with the tile unfetched. Free order: unchanged.
                    double ratio;
                    if (gain <= 0) {
                        if (tileRank == null) continue;
                        ratio = 1e9 + delta;
                    } else ratio = delta / gain;
                    if (ratio < bestRatio) {
                        bestRatio = ratio;
                        bestO = o.id;
                        bestP = p;
                    }
                }
            }
            if (bestO < 0) return;
            tour.add(bestP, bestO);
            inTour.add(bestO);
            for (int t : opts.get(bestO).tiles) force[t] = false;
        }
    }

    // ------------------------------------------------------------------ ruin and recreate

    public int ruinMaxLen = 3;

    /** Cut 1..ruinMaxLen consecutive options, re-cover greedily, keep if cheaper; then the local moves again. */
    private int ruinRecreate(List<Integer> tour) {
        if (ruinMaxLen <= 0) return 0;
        int accepted = 0;
        double cur = tourCost(tour);
        for (int pass = 0; pass < 3; pass++) {
            boolean any = false;
            for (int len = 1; len <= ruinMaxLen; len++)
                for (int k = 0; k + len <= tour.size(); k++) {
                    List<Integer> t2 = new ArrayList<>(tour);
                    for (int i = 0; i < len; i++) t2.remove(k);
                    greedy(t2, new boolean[nTiles]);
                    double c = tourCost(t2);
                    if (c < cur - 1e-6 && complete(coverage(full(t2)), null, full(t2))) {
                        tour.clear();
                        tour.addAll(t2);
                        cur = c;
                        any = true;
                        accepted++;
                    }
                }
            if (!any) break;
            improve(tour);
            cur = tourCost(tour);
        }
        return accepted;
    }

    // ------------------------------------------------------------------ improvement: drop, relocate/flip, 2-opt

    /** Single-tile options per tile: the other via points a relocate may swap a tile's via for. */
    private List<List<Integer>> singlesByTile() {
        List<List<Integer>> by = new ArrayList<>();
        for (int t = 0; t < nTiles; t++) by.add(new ArrayList<>());
        for (Opt o : opts) if (!o.section && !o.banned) by.get(o.tiles[0]).add(o.id);
        return by;
    }

    /** r4: relocate also tries the tile's other via points (a tile's via was never swapped before). */
    static final boolean SWAP_VIA = true;
    static final boolean PORTALS_ALWAYS = true;

    private void improve(List<Integer> tour) {
        List<List<Integer>> byTile = SWAP_VIA ? singlesByTile() : null;
        double cur = tourCost(tour);
        boolean improved = true;
        int passes = 0;
        while (improved && passes++ < 20) {
            improved = false;
            // drop redundant
            for (int k = 0; k < tour.size(); k++) {
                List<Integer> t2 = new ArrayList<>(tour);
                t2.remove(k);
                double c = tourCost(t2);
                if (c < cur - 1e-6 && complete(coverage(full(t2)), null, full(t2))) {
                    tour.clear();
                    tour.addAll(t2);
                    cur = c;
                    improved = true;
                    k--;
                }
            }
            // relocate (and flip a section to its twin)
            for (int k = 0; k < tour.size(); k++) {
                int x = tour.get(k);
                List<Integer> base = new ArrayList<>(tour);
                base.remove(k);
                int bestP = -1, bestX = -1;
                double best = cur - 1e-6;
                List<Integer> cands = new ArrayList<>(List.of(x, opts.get(x).twin));
                if (byTile != null && !opts.get(x).section)
                    for (int alt : byTile.get(opts.get(x).tiles[0])) if (alt != x && !base.contains(alt)) cands.add(alt);
                for (int cand : cands) {
                    if (cand < 0) continue;
                    for (int p = 0; p <= base.size(); p++) {
                        if (p == k && cand == x) continue;
                        base.add(p, cand);
                        double c = tourCost(base);
                        if (c < best && complete(coverage(full(base)), null, full(base))) {
                            best = c;
                            bestP = p;
                            bestX = cand;
                        }
                        base.remove(p);
                    }
                }
                if (bestX >= 0) {
                    base.add(bestP, bestX);
                    tour.clear();
                    tour.addAll(base);
                    cur = best;
                    improved = true;
                }
            }
            // 2-opt: reverse a run, flipping sections
            for (int i = 0; i < tour.size(); i++)
                for (int j = i + 1; j < tour.size(); j++) {
                    List<Integer> t2 = new ArrayList<>(tour);
                    boolean ok = true;
                    for (int a = i, b = j; a <= j; a++, b--) {
                        int o = tour.get(b);
                        Opt op = opts.get(o);
                        if (op.section) {
                            if (op.twin < 0) {
                                ok = false;
                                break;
                            }
                            o = op.twin;
                        }
                        t2.set(a, o);
                    }
                    if (!ok) continue;
                    double c = tourCost(t2);
                    if (c < cur - 1e-6 && complete(coverage(full(t2)), null, full(t2))) {
                        tour.clear();
                        tour.addAll(t2);
                        cur = c;
                        improved = true;
                    }
                }
        }
    }

    // ------------------------------------------------------------------ via points

    private final List<PathDijkstra.Pt> viaPts = new ArrayList<>();

    private List<Via> vias(List<Integer> tour, Problem pb) {
        List<Via> out = new ArrayList<>();
        viaPts.clear();
        out.add(new Via(pb.home.lat, pb.home.lon, Double.NaN, "home", null));
        viaPts.add(pts.get(0));
        for (int o : tour) {
            Opt op = opts.get(o);
            String tk = pb.tiles.get(op.tiles[0]).tile.key;
            if (!op.section) {
                PathDijkstra.Pt p = pts.get(op.pts[0]);
                out.add(new Via(p.lat, p.lon, Double.NaN, p.label.startsWith("portal") ? "portal" : "single", tk));
                viaPts.add(p);
                continue;
            }
            List<PathDijkstra.Pt> chain = sectionChain(op);
            for (int i = 0; i < chain.size(); i++) {
                PathDijkstra.Pt p = chain.get(i);
                String role = i == 0 ? "stroke-entry" : i == chain.size() - 1 ? "stroke-exit" : "stroke-mid";
                out.add(new Via(p.lat, p.lon, Double.NaN, role, tk));
                viaPts.add(p);
            }
        }
        out.add(new Via(pb.home.lat, pb.home.lon, Double.NaN, "home", null));
        viaPts.add(pts.get(0));
        return out;
    }

    /** Own plain-via cost per leg of the final via list (for the runner's -Dtr.legParity check). */
    private double[] legCosts() {
        double[] c = new double[viaPts.size() - 1];
        for (int i = 0; i + 1 < viaPts.size(); i++) {
            PathDijkstra.Targets tg = new PathDijkstra.Targets(List.of(viaPts.get(i + 1)), f.weighting, f.graph, null);
            c[i] = dijkstra().search(viaPts.get(i), tg, maxSettled, false).cost[0];
        }
        return c;
    }

    /**
     * Entry, exit and the mid points needed so the plain-via shortest path between consecutive points fetches every
     * tile of the section: where a sub-leg misses a tile, a mid point at that tile's hit is added.
     */
    private List<PathDijkstra.Pt> sectionChain(Opt op) {
        List<PathDijkstra.Pt> chain = new ArrayList<>();
        chain.add(pts.get(op.pts[0]));
        chain.add(pts.get(op.pts[1]));
        // tile -> the hit to use as a mid point (first in riding order)
        Map<Integer, Strokes.Hit> mid = new LinkedHashMap<>();
        for (Strokes.Hit h : op.hits) mid.putIfAbsent(h.tile, h);
        for (int guard = 0; guard < op.tiles.length + 2; guard++) {
            Set<Integer> got = new HashSet<>();
            int missAfter = -1;
            for (int i = 0; i + 1 < chain.size(); i++) {
                PathDijkstra.Targets tg = new PathDijkstra.Targets(List.of(chain.get(i + 1)), f.weighting, f.graph, null);
                PathDijkstra.Row r = dijkstra().search(chain.get(i), tg, maxSettled, true);
                if (r.tiles[0] != null) for (int t : r.tiles[0]) got.add(t);
            }
            Strokes.Hit miss = null;
            for (Map.Entry<Integer, Strokes.Hit> e : mid.entrySet()) if (!got.contains(e.getKey())) {
                miss = e.getValue();
                break;
            }
            if (miss == null) break;
            double c = (miss.s0 + miss.s1) / 2;
            PathDijkstra.Pt p = strokes.point(miss.stroke, c);
            p.label = "mid";
            // insert in riding order along the stroke
            double sEntry = ptS.get(op.pts[0]);
            double dist = Math.abs(c - sEntry);
            int pos = chain.size() - 1;
            List<Double> along = new ArrayList<>();
            for (int i = 1; i < chain.size() - 1; i++) along.add(chainS(chain.get(i), op));
            for (int i = 0; i < along.size(); i++) if (Math.abs(along.get(i) - sEntry) > dist) {
                pos = i + 1;
                break;
            }
            midS.put(p, c);
            chain.add(pos, p);
            mid.remove(miss.tile);
            if (missAfter >= 0) break;
        }
        return chain;
    }

    private final Map<PathDijkstra.Pt, Double> midS = new IdentityHashMap<>();

    private double chainS(PathDijkstra.Pt p, Opt op) {
        Double s = midS.get(p);
        return s == null ? ptS.get(p.id) : s;
    }
}
