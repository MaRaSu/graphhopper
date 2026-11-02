package com.graphhopper.trailmap.tilerouting.portaltour;

import com.graphhopper.trailmap.tilerouting.foundation.EdgeDijkstra;
import com.graphhopper.trailmap.tilerouting.foundation.Foundation;
import com.graphhopper.trailmap.tilerouting.foundation.Portal;
import com.graphhopper.trailmap.tilerouting.foundation.TileMath;

import java.util.*;

/**
 * Leg costs between plain via points, as GraphHopper routes them: each leg on its own, leaving a point in either
 * direction and arriving in either direction, with the profile's weighting and turn costs inside the leg and no
 * turn cost across a via. Computed lazily with goal-directed {@link EdgeDijkstra} searches and cached.
 * <p>
 * Nodes: 0 = home, 1..N = the tiles. A point is one portal position; its directions are the portal and its twin.
 * A tile point's first search covers all its k nearest tiles at once (the neighbour bundle); any other pair is
 * searched on demand.
 */
final class LegOracle {

    static final class Point {
        int node, idx;
        /** The directed variants (1 or 2) of the same position. */
        List<Portal> dirs = new ArrayList<>();

        Portal primary() {
            return dirs.get(0);
        }
    }

    /** The leg GraphHopper takes from point a to point b. */
    static final class Leg {
        double cost = Double.POSITIVE_INFINITY;
        /** Edge keys of the leg's departure from a and arrival at b. */
        int depKey = -1, arrKey = -1;
        /** Interior edge keys (both end edges left out, so coverage from it is conservative). */
        List<Integer> path = List.of();
    }

    final Foundation f;
    final List<List<Point>> points = new ArrayList<>();
    final List<TileMath.Tile> tiles = new ArrayList<>();
    final List<Set<Integer>> neighbours = new ArrayList<>();
    private final Map<Long, Leg[]> cache = new HashMap<>();
    long searches, settled, tableNs;
    /** Goal-directed searches (exact either way; a lever for the speed comparison). */
    boolean aStar = true; // prototype tr.aStar
    int budgetHits;

    LegOracle(Foundation f, Foundation.Home home, List<Foundation.TileInfo> tileInfos, int k) {
        this.f = f;
        Point hp = new Point();
        hp.dirs.addAll(home.states);
        points.add(List.of(hp));
        tiles.add(null);
        for (Foundation.TileInfo ti : tileInfos) {
            int node = points.size();
            List<Point> pts = new ArrayList<>();
            Set<String> seen = new HashSet<>();
            for (Portal p : ti.portals) {
                // a portal and the opposite entry on the same edge at the same spot are one plain via point
                if (!seen.add(p.edge + "@" + Math.round(p.lat * 1e6) + "," + Math.round(p.lon * 1e6))) continue;
                Point pt = new Point();
                pt.node = node;
                pt.idx = pts.size();
                pt.dirs.add(p);
                // the twin only if it is in home's directed reach, so no search waits for an unreachable target
                Portal tw = f.twin(p);
                if (tw != null && home.fwd.get(tw.key) && home.back.get(tw.key)) pt.dirs.add(tw);
                pts.add(pt);
            }
            points.add(pts);
            tiles.add(ti.tile);
        }
        // k nearest tiles by centre distance
        neighbours.add(Set.of());
        for (int a = 1; a < tiles.size(); a++) {
            List<Integer> others = new ArrayList<>();
            for (int b = 1; b < tiles.size(); b++) if (b != a) others.add(b);
            final int aa = a;
            others.sort(Comparator.comparingDouble(b -> centreDist(aa, b)));
            neighbours.add(new HashSet<>(others.subList(0, Math.min(k, others.size()))));
        }
    }

    double centreDist(int a, int b) {
        double[] pa = centre(a), pb = centre(b);
        return Foundation.dist(pa[0], pa[1], pb[0], pb[1]);
    }

    double[] centre(int node) {
        if (node == 0) {
            Portal h = points.get(0).get(0).primary();
            return new double[]{h.lat, h.lon};
        }
        return new double[]{tiles.get(node).centreLat(), tiles.get(node).centreLon()};
    }

    boolean near(int a, int b) {
        return a == 0 || b == 0 || neighbours.get(a).contains(b) || neighbours.get(b).contains(a);
    }

    /** Legs from point a to every point of node b. */
    Leg[] legs(Point a, int b) {
        Leg[] c = cache.get(cacheKey(a, b));
        if (c != null) return c;
        if (a.node != 0 && neighbours.get(a.node).contains(b)) {
            search(a, new ArrayList<>(neighbours.get(a.node)));
            c = cache.get(cacheKey(a, b));
            if (c != null) return c;
        }
        search(a, List.of(b));
        return cache.get(cacheKey(a, b));
    }

    private long cacheKey(Point a, int b) {
        return ((long) a.node * 64 + a.idx) * 4096 + b;
    }

    /** One search from both directions of point a to all points of the given nodes. */
    private void search(Point a, List<Integer> nodes) {
        List<Portal> targets = new ArrayList<>();
        List<int[]> who = new ArrayList<>(); // node, point index
        for (int b : nodes)
            for (Point q : points.get(b))
                for (Portal d : q.dirs) {
                    targets.add(d);
                    who.add(new int[]{b, q.idx});
                }
        // both directions of one point form a group: GraphHopper's leg takes the cheaper arrival
        int[] group = new int[targets.size()];
        for (int t = 0; t < group.length; t++) group[t] = who.get(t)[0] * 64 + who.get(t)[1];
        long t0 = System.nanoTime();
        EdgeDijkstra.Result r = f.dijkstra.search(a.dirs, null, targets, group, f.params.maxSettled, aStar);
        searches++;
        settled += r.settled;
        if (r.budgetHit) budgetHits++;
        Map<Integer, Leg[]> out = new HashMap<>();
        for (int b : nodes) {
            Leg[] arr = new Leg[points.get(b).size()];
            for (int i = 0; i < arr.length; i++) arr[i] = new Leg();
            out.put(b, arr);
        }
        for (int t = 0; t < targets.size(); t++) {
            if (!Double.isFinite(r.cost[t])) continue;
            Leg l = out.get(who.get(t)[0])[who.get(t)[1]];
            if (r.cost[t] < l.cost) {
                l.cost = r.cost[t];
                l.depKey = a.dirs.get(r.origin[t]).key;
                l.arrKey = targets.get(t).key;
                List<Integer> keys = f.dijkstra.pathKeys(r, t);
                l.path = keys.size() <= 1 ? List.of() : new ArrayList<>(keys.subList(0, keys.size() - 1));
            }
        }
        tableNs += System.nanoTime() - t0;
        for (Map.Entry<Integer, Leg[]> e : out.entrySet()) cache.put(cacheKey(a, e.getKey()), e.getValue());
    }
}
