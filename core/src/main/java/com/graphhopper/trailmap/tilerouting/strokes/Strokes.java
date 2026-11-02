package com.graphhopper.trailmap.tilerouting.strokes;

import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.trailmap.tilerouting.foundation.Foundation;
import com.graphhopper.trailmap.tilerouting.foundation.TileMath;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.shapes.BBox;

import java.util.*;

/**
 * Street strokes (Thomson &amp; Richardson): rideable edges near the requested tiles chained across junctions where the
 * road continues smoothly. At each node the incident edges are paired greedily by smallest deflection (+ a penalty
 * when the road name changes), within {@link #maxDeflectDeg} at real junctions and {@link #maxBendDeg} at plain
 * degree-2 nodes, and only where the turn is legal at least one way. Each stroke records where it crosses the inset
 * squares of the requested tiles ("hits").
 */
final class Strokes {

    double maxDeflectDeg = 35, maxBendDeg = 110, nameChangePenalty = 15, bearingProbeM = 15;

    static final class Stroke {
        int id;
        int[] edges;
        boolean[] fwd;
        double[] start;
        double[] len;
        double length;
        List<Hit> hits = new ArrayList<>();
    }

    /** Where a stroke runs inside one tile's inset: stroke metres [s0, s1]. */
    static final class Hit {
        Stroke stroke;
        int tile;
        double s0, s1;
        int idx;
        /** Distinct tiles in this hit's chain on the stroke (gaps ≤ maxGap). */
        int clusterTiles;
        double weightPerM;
        boolean kept;

        double len() {
            return s1 - s0;
        }
    }

    final List<Stroke> strokes = new ArrayList<>();
    private final Foundation f;
    private final BaseGraph graph;
    private final Weighting w;
    private final EdgeTiles et;

    Strokes(Foundation f, EdgeTiles et) {
        this.f = f;
        this.graph = f.graph;
        this.w = f.weighting;
        this.et = et;
    }

    /** Area = every rideable edge within {@code bufferM} of a requested tile. */
    void build(double bufferM) {
        Set<Integer> area = new TreeSet<>();
        for (TileMath.Tile t : et.tiles) {
            double dLat = bufferM / TileMath.M_PER_DEG_LAT;
            double dLon = bufferM / (TileMath.M_PER_DEG_LON_EQ * Math.cos(Math.toRadians(t.centreLat())));
            f.hopper.getLocationIndex().query(new BBox(t.minLon - dLon, t.maxLon + dLon, t.minLat - dLat, t.maxLat + dLat), area::add);
        }
        // node -> incident (edge, end): end 0 = edge's base node, 1 = adj node
        Map<Integer, List<int[]>> inc = new HashMap<>();
        Map<Integer, String> names = new HashMap<>();
        List<Integer> edges = new ArrayList<>();
        for (int e : area) {
            EdgeIteratorState st = graph.getEdgeIteratorState(e, Integer.MIN_VALUE);
            if (!f.rideable(st) || st.getBaseNode() == st.getAdjNode()) continue;
            edges.add(e);
            names.put(e, st.getName() == null ? "" : st.getName());
            inc.computeIfAbsent(st.getBaseNode(), k -> new ArrayList<>()).add(new int[]{e, 0});
            inc.computeIfAbsent(st.getAdjNode(), k -> new ArrayList<>()).add(new int[]{e, 1});
        }
        Map<Long, Long> partner = new HashMap<>();
        for (Map.Entry<Integer, List<int[]>> en : inc.entrySet()) {
            int node = en.getKey();
            List<int[]> l = en.getValue();
            if (l.size() < 2) continue;
            double[] brg = new double[l.size()];
            for (int i = 0; i < l.size(); i++) brg[i] = outBearing(l.get(i)[0], l.get(i)[1]);
            double limit = l.size() == 2 ? maxBendDeg : maxDeflectDeg;
            List<double[]> pairs = new ArrayList<>();
            for (int i = 0; i < l.size(); i++)
                for (int j = i + 1; j < l.size(); j++) {
                    int a = l.get(i)[0], b = l.get(j)[0];
                    if (a == b) continue;
                    double diff = Math.abs(brg[i] - brg[j]) % 360;
                    if (diff > 180) diff = 360 - diff;
                    double defl = 180 - diff;
                    if (defl > limit) continue;
                    if (!Double.isFinite(w.calcTurnWeight(a, node, b)) && !Double.isFinite(w.calcTurnWeight(b, node, a))) continue;
                    String na = names.get(a), nb = names.get(b);
                    double score = defl + (!na.isEmpty() && na.equals(nb) ? 0 : nameChangePenalty);
                    pairs.add(new double[]{score, i, j});
                }
            pairs.sort(Comparator.comparingDouble(p -> p[0]));
            boolean[] used = new boolean[l.size()];
            for (double[] p : pairs) {
                int i = (int) p[1], j = (int) p[2];
                if (used[i] || used[j]) continue;
                used[i] = used[j] = true;
                long ki = code(l.get(i)[0], l.get(i)[1]), kj = code(l.get(j)[0], l.get(j)[1]);
                partner.put(ki, kj);
                partner.put(kj, ki);
            }
        }
        Set<Integer> visited = new HashSet<>();
        for (int e : edges) {
            if (visited.contains(e)) continue;
            visited.add(e);
            Deque<int[]> chain = new ArrayDeque<>(); // {edge, fwd?1:0}
            chain.add(new int[]{e, 1});
            // forward: leave e at its adj end
            int cur = e, endAtNode = 1;
            while (true) {
                Long p = partner.get(code(cur, endAtNode));
                if (p == null) break;
                int e2 = (int) (p >> 1), end2 = (int) (p & 1);
                if (visited.contains(e2)) break;
                visited.add(e2);
                boolean fwd2 = end2 == 0;
                chain.addLast(new int[]{e2, fwd2 ? 1 : 0});
                cur = e2;
                endAtNode = fwd2 ? 1 : 0;
            }
            cur = e;
            endAtNode = 0;
            while (true) {
                Long p = partner.get(code(cur, endAtNode));
                if (p == null) break;
                int e2 = (int) (p >> 1), end2 = (int) (p & 1);
                if (visited.contains(e2)) break;
                visited.add(e2);
                boolean fwd2 = end2 == 1;
                chain.addFirst(new int[]{e2, fwd2 ? 1 : 0});
                cur = e2;
                endAtNode = fwd2 ? 0 : 1;
            }
            Stroke s = new Stroke();
            s.id = strokes.size();
            int n = chain.size();
            s.edges = new int[n];
            s.fwd = new boolean[n];
            s.start = new double[n];
            s.len = new double[n];
            int i = 0;
            double acc = 0;
            for (int[] c : chain) {
                s.edges[i] = c[0];
                s.fwd[i] = c[1] == 1;
                s.start[i] = acc;
                s.len[i] = et.geo(c[0]).len;
                acc += s.len[i];
                i++;
            }
            s.length = acc;
            strokes.add(s);
        }
    }

    /** Find hits; minChordM = least inside length for a hit to count. */
    void hits(double minChordM, double maxGapM) {
        for (Stroke s : strokes) {
            Map<Integer, List<double[]>> byTile = new TreeMap<>();
            for (int i = 0; i < s.edges.length; i++) {
                EdgeTiles.Iv[] ivs = et.get(s.edges[i]);
                if (ivs == null) continue;
                for (EdgeTiles.Iv iv : ivs) {
                    double a = s.fwd[i] ? iv.f0 : 1 - iv.f1, b = s.fwd[i] ? iv.f1 : 1 - iv.f0;
                    byTile.computeIfAbsent(iv.tile, k -> new ArrayList<>()).add(new double[]{s.start[i] + a * s.len[i], s.start[i] + b * s.len[i]});
                }
            }
            for (Map.Entry<Integer, List<double[]>> en : byTile.entrySet()) {
                List<double[]> l = en.getValue();
                l.sort(Comparator.comparingDouble(x -> x[0]));
                double[] cur = null;
                List<double[]> merged = new ArrayList<>();
                for (double[] x : l) {
                    if (cur != null && x[0] - cur[1] < 5) cur[1] = Math.max(cur[1], x[1]);
                    else {
                        cur = x.clone();
                        merged.add(cur);
                    }
                }
                for (double[] m : merged) {
                    if (m[1] - m[0] < minChordM) continue;
                    Hit h = new Hit();
                    h.stroke = s;
                    h.tile = en.getKey();
                    h.s0 = m[0];
                    h.s1 = m[1];
                    s.hits.add(h);
                }
            }
            s.hits.sort(Comparator.comparingDouble(h -> h.s0));
            for (int i = 0; i < s.hits.size(); i++) s.hits.get(i).idx = i;
            // chains of hits with gaps ≤ maxGap: distinct tiles per chain
            int a = 0;
            while (a < s.hits.size()) {
                int b = a;
                while (b + 1 < s.hits.size() && s.hits.get(b + 1).s0 - s.hits.get(b).s1 <= maxGapM) b++;
                Set<Integer> ts = new HashSet<>();
                for (int k = a; k <= b; k++) ts.add(s.hits.get(k).tile);
                for (int k = a; k <= b; k++) s.hits.get(k).clusterTiles = ts.size();
                a = b + 1;
            }
            for (Hit h : s.hits) {
                int i = edgeAt(s, (h.s0 + h.s1) / 2);
                EdgeIteratorState st = graph.getEdgeIteratorState(s.edges[i], Integer.MIN_VALUE);
                double wf = w.calcEdgeWeight(st, false), wb = w.calcEdgeWeight(st, true);
                double best = Math.min(wf, wb);
                h.weightPerM = s.len[i] > 0 ? best / s.len[i] : Double.POSITIVE_INFINITY;
            }
        }
    }

    static int edgeAt(Stroke s, double sPos) {
        for (int i = 0; i < s.edges.length; i++) if (sPos < s.start[i] + s.len[i]) return i;
        return s.edges.length - 1;
    }

    /** Point at stroke metres sPos as (edge, storage fraction), and the storage direction of riding the stroke forward there. */
    PathDijkstra.Pt point(Stroke s, double sPos) {
        int i = edgeAt(s, sPos);
        double t = s.len[i] > 0 ? (sPos - s.start[i]) / s.len[i] : 0.5;
        t = Math.max(0, Math.min(1, t));
        // keep vias off nodes: at least 3 m (or 20 %) from either end of the edge
        double off = s.len[i] > 0 ? Math.min(0.2, 3 / s.len[i]) : 0.5;
        t = Math.max(off, Math.min(1 - off, t));
        PathDijkstra.Pt p = new PathDijkstra.Pt();
        p.edge = s.edges[i];
        p.f = s.fwd[i] ? t : 1 - t;
        EdgeTiles.Geo g = et.geo(p.edge);
        double[] at = EdgeTiles.pointAt(g, p.f * g.len);
        p.lat = at[0];
        p.lon = at[1];
        return p;
    }

    /** Storage direction (0/1) of riding the stroke forward (strokeFwd) or backward at sPos. */
    static byte dirAt(Stroke s, double sPos, boolean strokeFwd) {
        int i = edgeAt(s, sPos);
        boolean storageFwd = s.fwd[i] == strokeFwd;
        return (byte) (storageFwd ? 0 : 1);
    }

    /** Weight of riding the stroke from a to b (b &gt; a forward, b &lt; a backward), turn costs at its joints included. */
    double cost(Stroke s, double a, double b) {
        boolean fwd = b >= a;
        int ia = edgeAt(s, a), ib = edgeAt(s, b);
        double c = 0;
        if (ia == ib) return Math.abs(b - a) / Math.max(1e-9, s.len[ia]) * dirWeight(s, ia, fwd);
        if (fwd) {
            c += (s.start[ia] + s.len[ia] - a) / s.len[ia] * dirWeight(s, ia, true);
            for (int k = ia; k < ib; k++) {
                if (k > ia) c += dirWeight(s, k, true);
                c += turn(s, k, k + 1);
            }
            c += (b - s.start[ib]) / s.len[ib] * dirWeight(s, ib, true);
        } else {
            c += (a - s.start[ia]) / s.len[ia] * dirWeight(s, ia, false);
            for (int k = ia; k > ib; k--) {
                if (k < ia) c += dirWeight(s, k, false);
                c += turn(s, k, k - 1);
            }
            c += (s.start[ib] + s.len[ib] - b) / s.len[ib] * dirWeight(s, ib, false);
        }
        return c;
    }

    private double dirWeight(Stroke s, int i, boolean strokeFwd) {
        EdgeIteratorState st = graph.getEdgeIteratorState(s.edges[i], Integer.MIN_VALUE);
        return w.calcEdgeWeight(st, s.fwd[i] != strokeFwd);
    }

    /** Turn weight from stroke edge i to its neighbour j at their shared node. */
    private double turn(Stroke s, int i, int j) {
        EdgeIteratorState a = graph.getEdgeIteratorState(s.edges[i], Integer.MIN_VALUE);
        EdgeIteratorState b = graph.getEdgeIteratorState(s.edges[j], Integer.MIN_VALUE);
        int node = (a.getBaseNode() == b.getBaseNode() || a.getBaseNode() == b.getAdjNode()) ? a.getBaseNode() : a.getAdjNode();
        return w.calcTurnWeight(s.edges[i], node, s.edges[j]);
    }

    private double outBearing(int edge, int end) {
        EdgeTiles.Geo g = et.geo(edge);
        double probe = Math.min(bearingProbeM, g.len);
        double[] from = end == 0 ? EdgeTiles.pointAt(g, 0) : EdgeTiles.pointAt(g, g.len);
        double[] to = end == 0 ? EdgeTiles.pointAt(g, probe) : EdgeTiles.pointAt(g, g.len - probe);
        return Foundation.bearing(from[0], from[1], to[0], to[1]);
    }

    private static long code(int edge, int end) {
        return ((long) edge << 1) | end;
    }
}
