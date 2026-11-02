package com.graphhopper.trailmap.tilerouting.strokes;

import com.graphhopper.storage.BaseGraph;
import com.graphhopper.trailmap.tilerouting.foundation.Foundation;
import com.graphhopper.trailmap.tilerouting.foundation.TileMath;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.FetchMode;
import com.graphhopper.util.PointList;
import com.graphhopper.util.shapes.BBox;

import java.util.*;

/**
 * For every rideable edge near the requested tiles: the parts of its geometry inside each tile's inset square, as
 * storage-direction fractions. Also caches edge geometry (cumulative metres) for the stroke builder.
 */
final class EdgeTiles {

    /** A path must ride at least this far inside the inset for the tile to count (GPS / snap robustness). */
    static final double MIN_OVERLAP_M = 2;

    static final class Iv {
        int tile;
        double f0, f1;
        double edgeLen;
    }

    static final class Geo {
        PointList pl;
        double[] cum;
        double len;
    }

    private final Map<Integer, Iv[]> byEdge = new HashMap<>();
    private final Map<Integer, Geo> geo = new HashMap<>();
    private final BaseGraph graph;
    final List<TileMath.Tile> tiles;

    EdgeTiles(Foundation f, List<TileMath.Tile> tiles) {
        this.graph = f.graph;
        this.tiles = tiles;
        Map<Integer, List<Iv>> m = new HashMap<>();
        for (int ti = 0; ti < tiles.size(); ti++) {
            TileMath.Tile t = tiles.get(ti);
            Set<Integer> edges = new TreeSet<>();
            f.hopper.getLocationIndex().query(new BBox(t.minLon, t.maxLon, t.minLat, t.maxLat), edges::add);
            for (int e : edges) {
                EdgeIteratorState st = graph.getEdgeIteratorState(e, Integer.MIN_VALUE);
                if (!f.rideable(st)) continue;
                Geo g = geo(e);
                for (double[] iv : inside(t, g)) {
                    Iv v = new Iv();
                    v.tile = ti;
                    v.f0 = g.len > 0 ? iv[0] / g.len : 0;
                    v.f1 = g.len > 0 ? iv[1] / g.len : 0;
                    v.edgeLen = g.len;
                    m.computeIfAbsent(e, k -> new ArrayList<>()).add(v);
                }
            }
        }
        for (Map.Entry<Integer, List<Iv>> e : m.entrySet()) byEdge.put(e.getKey(), e.getValue().toArray(new Iv[0]));
    }

    int tileCount() {
        return tiles.size();
    }

    Iv[] get(int edge) {
        return byEdge.get(edge);
    }

    synchronized Geo geo(int edge) {
        Geo g = geo.get(edge);
        if (g != null) return g;
        g = new Geo();
        g.pl = graph.getEdgeIteratorState(edge, Integer.MIN_VALUE).fetchWayGeometry(FetchMode.ALL);
        g.cum = new double[g.pl.size()];
        for (int i = 1; i < g.pl.size(); i++)
            g.cum[i] = g.cum[i - 1] + Foundation.dist(g.pl.getLat(i - 1), g.pl.getLon(i - 1), g.pl.getLat(i), g.pl.getLon(i));
        g.len = g.cum[g.pl.size() - 1];
        geo.put(edge, g);
        return g;
    }

    /** Inside-inset intervals (metres from the base node), merged when touching. */
    static List<double[]> inside(TileMath.Tile t, Geo g) {
        List<double[]> out = new ArrayList<>();
        for (int i = 0; i + 1 < g.pl.size(); i++) {
            double[] c = t.clip(g.pl.getLat(i), g.pl.getLon(i), g.pl.getLat(i + 1), g.pl.getLon(i + 1));
            if (c == null) continue;
            double seg = g.cum[i + 1] - g.cum[i];
            double a = g.cum[i] + c[0] * seg, b = g.cum[i] + c[1] * seg;
            if (!out.isEmpty() && a - out.get(out.size() - 1)[1] < 0.5) out.get(out.size() - 1)[1] = b;
            else out.add(new double[]{a, b});
        }
        return out;
    }

    /** lat, lon, bearing at metres s from the base node. */
    static double[] pointAt(Geo g, double s) {
        PointList pl = g.pl;
        for (int i = 0; i + 1 < pl.size(); i++) {
            if (s <= g.cum[i + 1] || i + 2 == pl.size()) {
                double len = g.cum[i + 1] - g.cum[i];
                double f = len > 0 ? Math.max(0, Math.min(1, (s - g.cum[i]) / len)) : 0;
                double la = pl.getLat(i) + f * (pl.getLat(i + 1) - pl.getLat(i));
                double lo = pl.getLon(i) + f * (pl.getLon(i + 1) - pl.getLon(i));
                return new double[]{la, lo, Foundation.bearing(pl.getLat(i), pl.getLon(i), pl.getLat(i + 1), pl.getLon(i + 1))};
            }
        }
        return new double[]{pl.getLat(0), pl.getLon(0), 0};
    }
}
