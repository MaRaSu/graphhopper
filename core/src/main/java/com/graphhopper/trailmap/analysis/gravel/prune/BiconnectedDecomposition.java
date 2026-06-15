/*
 * Trailmap - Gravel Segment Analysis
 *
 * Phase C primitive: iterative Hopcroft–Tarjan biconnected-component decomposition over
 * an AnalysisGraph. Produces a block id per edge and identifies cut vertices, from which
 * the block-cut tree is built. See docs/gravel_segments_design.md §7.1.
 *
 * GraphHopper provides no bridge / articulation-point / biconnected decomposition (its
 * subnetwork code is SCC-only), so this is new code. The DFS is explicitly stacked
 * (never recursive) because the national-scale input graph is deep. Complexity O(V + E).
 */
package com.graphhopper.trailmap.analysis.gravel.prune;

import com.graphhopper.trailmap.analysis.gravel.graph.AnalysisGraph;

import java.util.Arrays;

/**
 * Assigns each (non-removed) analysis edge a biconnected-component ("block") id. A bridge
 * is a block of one edge; a maximal 2-connected subgraph (e.g. a turning loop) is one
 * block. Cut vertices are nodes belonging to ≥2 distinct blocks.
 *
 * <p>The block-cut tree is: nodes = blocks ∪ cut vertices;
 * a block is adjacent to each cut vertex it contains. Tails and lollipops are leaf blocks.</p>
 */
public class BiconnectedDecomposition {

    private final AnalysisGraph g;
    private final int[] blockIdOfEdge;
    private int blockCount;

    private BiconnectedDecomposition(AnalysisGraph g) {
        this.g = g;
        this.blockIdOfEdge = new int[g.edgeCount()];
        Arrays.fill(blockIdOfEdge, -1);
    }

    public static BiconnectedDecomposition decompose(AnalysisGraph g) {
        BiconnectedDecomposition d = new BiconnectedDecomposition(g);
        d.run();
        return d;
    }

    /** Block id of analysis edge {@code e}, or -1 if the edge is removed/unvisited. */
    public int blockOf(int e) {
        return blockIdOfEdge[e];
    }

    public int blockCount() {
        return blockCount;
    }

    private void run() {
        int n = g.nodeCount();
        int[] disc = new int[n];
        int[] low = new int[n];
        Arrays.fill(disc, -1);

        // DFS frame stack (parallel arrays).
        int[] frameNode = new int[n];
        int[] frameInEdge = new int[n];   // analysis-edge id used to enter the node (-1 for a root)
        int[] frameCursor = new int[n];   // current position in the node's CSR adjacency
        int top;

        // Edge stack for block extraction.
        int[] edgeStack = new int[g.edgeCount()];
        int edgeTop;

        int time = 0;
        for (int start = 0; start < n; start++) {
            if (disc[start] != -1)
                continue;

            top = 0;
            edgeTop = 0;
            disc[start] = low[start] = time++;
            frameNode[0] = start;
            frameInEdge[0] = -1;
            frameCursor[0] = g.adjBegin(start);

            while (top >= 0) {
                int u = frameNode[top];
                boolean descended = false;

                while (frameCursor[top] < g.adjEnd(u)) {
                    int e = g.adjEdge(frameCursor[top]++);
                    if (g.isRemoved(e))
                        continue;
                    if (e == frameInEdge[top])   // skip the single edge we entered through
                        continue;
                    int v = g.other(e, u);
                    if (disc[v] == -1) {
                        // tree edge: push it and descend into v
                        edgeStack[edgeTop++] = e;
                        disc[v] = low[v] = time++;
                        top++;
                        frameNode[top] = v;
                        frameInEdge[top] = e;
                        frameCursor[top] = g.adjBegin(v);
                        descended = true;
                        break;
                    } else if (disc[v] < disc[u]) {
                        // back edge to an ancestor: push once (from the descendant side)
                        edgeStack[edgeTop++] = e;
                        if (disc[v] < low[u])
                            low[u] = disc[v];
                    }
                    // disc[v] > disc[u]: already handled from v's side — skip
                }

                if (descended)
                    continue;

                // u is fully explored: pop it and, if it closes a block at its parent, emit.
                int teEdge = frameInEdge[top];   // tree edge into u (-1 if u is a root)
                int uLow = low[u];
                top--;
                if (top >= 0) {
                    int p = frameNode[top];
                    if (uLow < low[p])
                        low[p] = uLow;
                    if (uLow >= disc[p]) {
                        int blockId = blockCount++;
                        while (edgeTop > 0) {
                            int be = edgeStack[--edgeTop];
                            blockIdOfEdge[be] = blockId;
                            if (be == teEdge)
                                break;
                        }
                    }
                }
            }
        }
    }
}
