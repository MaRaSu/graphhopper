/*
 * Trailmap - Gravel Segment Analysis
 *
 * Phase 2 pre-pass (between B and C): admit weighted connector corridors. See spec §3.
 */
package com.graphhopper.trailmap.analysis.gravel.prune;

import com.graphhopper.trailmap.analysis.gravel.graph.AnalysisGraph;
import com.graphhopper.trailmap.analysis.gravel.role.EdgeRole;

import java.util.Arrays;
import java.util.PriorityQueue;

/**
 * Admits {@link EdgeRole#CONNECTOR} corridors that bridge the real network, judged by a
 * <b>weighted</b> cost rather than a flat length. Runs <b>before</b> the network filter so admitted
 * connectors are part of the connectivity the pruner sees.
 *
 * <p><b>Cost model.</b> Each connector edge costs {@code lengthM × connectorWeight} — its quality
 * weight (good-but-not-Target gravel is light, hike-a-bike is heavy). A corridor is the cheapest
 * weighted path between two <i>distinct</i> attachment points (nodes incident to a TARGET or ANCHOR
 * edge); it is admitted iff that cost ≤ {@code costBudget}. So a light band can stretch far and a
 * heavy band only a few metres, yet both may appear in one corridor — each spends from the shared
 * budget in proportion to how bad it is. This is the per-corridor model: only the actual bridge is
 * costed, not the whole connected blob, so a short crossing threaded through a larger mesh is still
 * found.</p>
 *
 * <p><b>Algorithm.</b> A multi-source Dijkstra over the CONNECTOR subgraph seeded at every
 * connector-incident attachment node (each its own region). For a connector edge whose two endpoints
 * lie in <i>different</i> regions, the cheapest corridor through it costs
 * {@code dist[u] + cost(e) + dist[w]}; if ≤ budget, the edge and the shortest-path-tree paths back to
 * both sources are admitted (the full corridor). Admitted edges keep their CONNECTOR role; all other
 * connector edges are removed (stay effectively IGNORED).</p>
 */
public class ConnectorResolver {

    private final AnalysisGraph g;
    private final double costBudget;

    public ConnectorResolver(AnalysisGraph g, double costBudget) {
        this.g = g;
        this.costBudget = costBudget;
    }

    /** Outcome counts. {@code corridors} = bridging edges admitted (≈ corridors found). */
    public static final class Result {
        public final int corridors, admittedEdges, removedEdges;
        Result(int corridors, int admittedEdges, int removedEdges) {
            this.corridors = corridors;
            this.admittedEdges = admittedEdges;
            this.removedEdges = removedEdges;
        }
    }

    public Result resolve() {
        final int n = g.nodeCount();
        final int m = g.edgeCount();

        // Attachment nodes: incident to a non-removed TARGET or ANCHOR edge (the real network a
        // connector must bridge between).
        boolean[] attach = new boolean[n];
        boolean anyConnector = false;
        for (int e = 0; e < m; e++) {
            if (g.isRemoved(e)) continue;
            EdgeRole r = g.role(e);
            if (r == EdgeRole.TARGET || r == EdgeRole.ANCHOR) {
                attach[g.nodeA(e)] = true;
                attach[g.nodeB(e)] = true;
            } else if (r == EdgeRole.CONNECTOR) {
                anyConnector = true;
            }
        }
        if (!anyConnector) return new Result(0, 0, 0);

        // Multi-source Dijkstra over the connector subgraph from every connector-incident attachment.
        double[] dist = new double[n];
        int[] src = new int[n];        // region label = nearest attachment node
        int[] predEdge = new int[n];   // connector edge used to reach this node from src
        Arrays.fill(dist, Double.POSITIVE_INFINITY);
        Arrays.fill(src, -1);
        Arrays.fill(predEdge, -1);

        PriorityQueue<double[]> pq = new PriorityQueue<>((a, b) -> Double.compare(a[0], b[0]));
        for (int e = 0; e < m; e++) {
            if (g.isRemoved(e) || g.role(e) != EdgeRole.CONNECTOR) continue;
            for (int node : new int[]{g.nodeA(e), g.nodeB(e)}) {
                if (attach[node] && src[node] == -1) {
                    dist[node] = 0;
                    src[node] = node;
                    pq.add(new double[]{0, node});
                }
            }
        }

        while (!pq.isEmpty()) {
            double[] top = pq.poll();
            double d = top[0];
            int u = (int) top[1];
            if (d > dist[u]) continue;                       // stale heap entry
            for (int i = g.adjBegin(u); i < g.adjEnd(u); i++) {
                int e = g.adjEdge(i);
                if (g.isRemoved(e) || g.role(e) != EdgeRole.CONNECTOR) continue;
                int w = g.other(e, u);
                double nd = d + edgeCost(e);
                if (nd < dist[w]) {
                    dist[w] = nd;
                    src[w] = src[u];
                    predEdge[w] = e;
                    pq.add(new double[]{nd, w});
                }
            }
        }

        // Admit every corridor whose cheapest bridge between two distinct regions is within budget.
        // No redundancy pruning here — connectivity must never be severed; redundant connectors are
        // distinguished later as an OUTPUT label only (ConnectorOutputClassifier).
        boolean[] admit = new boolean[m];
        int corridors = 0;
        for (int e = 0; e < m; e++) {
            if (g.isRemoved(e) || g.role(e) != EdgeRole.CONNECTOR) continue;
            int u = g.nodeA(e), w = g.nodeB(e);
            if (src[u] < 0 || src[w] < 0 || src[u] == src[w]) continue;   // not a bridge
            if (dist[u] + edgeCost(e) + dist[w] <= costBudget) {
                if (!admit[e]) corridors++;
                markBackToSource(u, predEdge, admit);
                markBackToSource(w, predEdge, admit);
                admit[e] = true;
            }
        }

        int admittedEdges = 0, removedEdges = 0;
        for (int e = 0; e < m; e++) {
            if (g.isRemoved(e) || g.role(e) != EdgeRole.CONNECTOR) continue;
            if (admit[e]) admittedEdges++;
            else {
                g.remove(e);
                removedEdges++;
            }
        }
        return new Result(corridors, admittedEdges, removedEdges);
    }

    private double edgeCost(int e) {
        return g.lengthM(e) * g.connectorWeight(e);
    }

    /** Walk the shortest-path tree from {@code node} back to its source, admitting each edge. */
    private void markBackToSource(int node, int[] predEdge, boolean[] admit) {
        int v = node;
        while (predEdge[v] != -1) {
            int e = predEdge[v];
            admit[e] = true;
            v = g.other(e, v);
        }
    }

    /**
     * Output-correctness pass — run AFTER the network filter has pruned failed gravel. Removes
     * admitted CONNECTOR edges that serve no <b>retained</b> gravel: a connector is kept only if its
     * connected component over (retained TARGET ∪ CONNECTOR) contains at least one retained TARGET
     * edge. This drops road-to-road connectors (they bridge ANCHOR regions but no gravel) and
     * connectors orphaned when the gravel they glued was pruned — both would otherwise leak into
     * qualifying output. Safe: such connectors are by definition not connecting any retained gravel,
     * so removing them cannot disconnect kept gravel. Returns the number of connector edges removed.
     */
    public static int removeConnectorsNotServingGravel(AnalysisGraph g) {
        final int n = g.nodeCount(), m = g.edgeCount();
        DisjointSet ds = new DisjointSet(n);
        for (int e = 0; e < m; e++) {
            if (g.isRemoved(e)) continue;
            EdgeRole r = g.role(e);
            if (r == EdgeRole.TARGET || r == EdgeRole.CONNECTOR) ds.union(g.nodeA(e), g.nodeB(e));
        }
        boolean[] hasRetainedTarget = new boolean[n];
        for (int e = 0; e < m; e++)
            if (!g.isRemoved(e) && g.role(e) == EdgeRole.TARGET) hasRetainedTarget[ds.find(g.nodeA(e))] = true;
        int removed = 0;
        for (int e = 0; e < m; e++) {
            if (g.isRemoved(e) || g.role(e) != EdgeRole.CONNECTOR) continue;
            if (!hasRetainedTarget[ds.find(g.nodeA(e))]) { g.remove(e); removed++; }
        }
        return removed;
    }
}
