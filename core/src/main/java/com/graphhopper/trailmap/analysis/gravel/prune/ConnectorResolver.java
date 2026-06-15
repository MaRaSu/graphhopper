/*
 * Trailmap - Gravel Segment Analysis
 *
 * Phase 2 pre-pass (between B and C): resolve bounded connector chains. See spec §3.
 */
package com.graphhopper.trailmap.analysis.gravel.prune;

import com.graphhopper.trailmap.analysis.gravel.graph.AnalysisGraph;
import com.graphhopper.trailmap.analysis.gravel.role.EdgeRole;

/**
 * Resolves {@link EdgeRole#CONNECTOR} edges (normally-IGNORED paths / gravel-4 tracks, admitted as
 * connector candidates) into either ANCHOR connectivity or removal — a length-bounded connector
 * chain (spec §3). It must run <b>before</b> the network filter so the admitted connectors are part
 * of the connectivity graph the pruner sees.
 *
 * <p>Algorithm (one pass, O(V+E)). Connector candidates form connected components ("chains") over
 * CONNECTOR edges. A chain is <b>admitted</b> (every edge promoted to ANCHOR — non-backbone
 * connectivity, never output) iff:</p>
 * <ul>
 *   <li>its total length ≤ {@code maxChainLenM} — a short crossing, not a route leg (per spec §3 the
 *       bound is on the cumulative <i>chain</i>, not a single edge); and</li>
 *   <li>it touches ≥ {@code minAttachmentPoints} distinct attachment nodes (nodes also incident to a
 *       TARGET or ANCHOR edge) — i.e. it actually bridges separate gravel/road, rather than
 *       dead-ending.</li>
 * </ul>
 * <p>Otherwise every edge of the chain is removed (it stays effectively IGNORED). The giant rough
 * mesh, being one long component, is dropped; only genuinely short bridging chains are admitted.</p>
 */
public class ConnectorResolver {

    private final AnalysisGraph g;
    private final double maxChainLenM;
    private final int minAttachmentPoints;

    public ConnectorResolver(AnalysisGraph g, double maxChainLenM, int minAttachmentPoints) {
        this.g = g;
        this.maxChainLenM = maxChainLenM;
        this.minAttachmentPoints = minAttachmentPoints;
    }

    /** Outcome counts. */
    public static final class Result {
        public final int admittedChains, admittedEdges, removedEdges;
        Result(int admittedChains, int admittedEdges, int removedEdges) {
            this.admittedChains = admittedChains;
            this.admittedEdges = admittedEdges;
            this.removedEdges = removedEdges;
        }
    }

    public Result resolve() {
        final int n = g.nodeCount();
        final int m = g.edgeCount();

        // Chains = connected components over CONNECTOR edges.
        DisjointSet chain = new DisjointSet(n);
        boolean anyConnector = false;
        for (int e = 0; e < m; e++) {
            if (g.isRemoved(e) || g.role(e) != EdgeRole.CONNECTOR) continue;
            chain.union(g.nodeA(e), g.nodeB(e));
            anyConnector = true;
        }
        if (!anyConnector) return new Result(0, 0, 0);

        // Attachment nodes: incident to a non-removed TARGET or ANCHOR edge.
        boolean[] attach = new boolean[n];
        for (int e = 0; e < m; e++) {
            if (g.isRemoved(e)) continue;
            EdgeRole r = g.role(e);
            if (r == EdgeRole.TARGET || r == EdgeRole.ANCHOR) {
                attach[g.nodeA(e)] = true;
                attach[g.nodeB(e)] = true;
            }
        }

        // Per-chain total length.
        double[] len = new double[n];
        for (int e = 0; e < m; e++) {
            if (g.isRemoved(e) || g.role(e) != EdgeRole.CONNECTOR) continue;
            len[chain.find(g.nodeA(e))] += g.lengthM(e);
        }

        // Per-chain distinct attachment-node count. Visit each connector-incident node once.
        int[] attachCount = new int[n];
        boolean[] seenNode = new boolean[n];
        for (int e = 0; e < m; e++) {
            if (g.isRemoved(e) || g.role(e) != EdgeRole.CONNECTOR) continue;
            for (int node : new int[]{g.nodeA(e), g.nodeB(e)}) {
                if (seenNode[node]) continue;
                seenNode[node] = true;
                if (attach[node]) attachCount[chain.find(node)]++;
            }
        }

        // Admit or drop each chain.
        boolean[] chainAdmitted = new boolean[n];
        boolean[] chainDecided = new boolean[n];
        int admittedChains = 0, admittedEdges = 0, removedEdges = 0;
        for (int e = 0; e < m; e++) {
            if (g.isRemoved(e) || g.role(e) != EdgeRole.CONNECTOR) continue;
            int rep = chain.find(g.nodeA(e));
            if (!chainDecided[rep]) {
                chainDecided[rep] = true;
                chainAdmitted[rep] = len[rep] <= maxChainLenM && attachCount[rep] >= minAttachmentPoints;
                if (chainAdmitted[rep]) admittedChains++;
            }
            if (chainAdmitted[rep]) {
                g.setRole(e, EdgeRole.ANCHOR);   // non-backbone connectivity; never output
                admittedEdges++;
            } else {
                g.remove(e);
                removedEdges++;
            }
        }
        return new Result(admittedChains, admittedEdges, removedEdges);
    }
}
