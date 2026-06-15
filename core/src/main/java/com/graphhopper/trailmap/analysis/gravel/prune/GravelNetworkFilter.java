/*
 * Trailmap - Gravel Segment Analysis
 *
 * Phase C/D: whole-appendage dead-end pruning. Keep gravel that forms a real network reaching the
 * real-road backbone; drop dead-end clusters and isolated islands — each as ONE unit, never partly.
 * See docs/gravel_segments_design.md / docs/gravel_segment_spec.md §1, §C, §D.
 */
package com.graphhopper.trailmap.analysis.gravel.prune;

import com.graphhopper.trailmap.analysis.gravel.graph.AnalysisGraph;
import com.graphhopper.trailmap.analysis.gravel.role.EdgeRole;

/**
 * Dead-end pruning at the granularity of a <b>whole appendage</b> (spec §C), not individual edges.
 *
 * <p><b>The model.</b> The real-road backbone (ANCHOR edges of a drivable/cycleway class, see
 * {@link AnalysisGraph#isBackbone(int)}) is contracted to a single super-node S. In that contracted
 * view of the working graph (TARGET ∪ ANCHOR, undirected):</p>
 * <ul>
 *   <li>Gravel that is <b>2-edge-connected to S</b> (reachable two edge-independent ways — a
 *       through-route, or a loop that touches the backbone at ≥2 points) is a genuine network and is
 *       <b>always kept</b>.</li>
 *   <li>Gravel separated from S by a single bridge is a <b>pendant appendage</b> (a dead-end cluster
 *       reachable only one way in): the entire appendage — the access stick <i>and</i> any terminal
 *       loop/lollipop — is kept or dropped as <b>one unit</b>, by whether its total qualifying
 *       (TARGET) length meets the threshold. A substantial gravel area attached at one point is kept
 *       whole; a thin dead-end is pruned whole. Never a half-way split (e.g. keep the loop, drop the
 *       stick that is its only access).</li>
 *   <li>Gravel touching no backbone at all is an <b>island</b>: same whole-unit size test.</li>
 * </ul>
 *
 * <p>Rough/ground ANCHOR tracks are NOT backbone: they still carry connectivity (a cluster woven
 * into the network across them is on a cycle and survives), but they cannot by themselves terminate
 * a dead-end — so a gravel loop reachable only through rough tracks is correctly a closed cluster
 * judged by its own size. Only TARGET edges are ever removed; ANCHOR edges are never pruned.</p>
 *
 * <p>One pass suffices: a pendant lies on no cycle through S, so removing it cannot change any other
 * cluster's connectivity to S. The whole appendage is measured at once, so there is no leaf-by-leaf
 * cascade to iterate.</p>
 */
public class GravelNetworkFilter {

    private final AnalysisGraph g;
    private final double minClusterLenM;

    /** @param minClusterLenM minimum total qualifying (TARGET) length to keep a dead-end cluster / island */
    public GravelNetworkFilter(AnalysisGraph g, double minClusterLenM) {
        this.g = g;
        this.minClusterLenM = minClusterLenM;
    }

    /** Run the filter. Returns the number of TARGET analysis edges removed. */
    public int filter() {
        final int n = g.nodeCount();
        final int m = g.edgeCount();
        final int superNode = n;   // the contracted real-road backbone

        // 1) Backbone-anchored nodes: incident to a non-removed real-road backbone (ANCHOR) edge.
        boolean[] anchored = new boolean[n];
        for (int e = 0; e < m; e++) {
            if (g.isRemoved(e) || !g.isBackbone(e)) continue;
            anchored[g.nodeA(e)] = true;
            anchored[g.nodeB(e)] = true;
        }

        // 2) Contracted working graph: every non-removed edge, anchored endpoints folded into S.
        //    Edges fully inside S (both endpoints anchored) are dropped — they sit on the backbone
        //    itself and are trivially in the core. The original analysis-edge id is stored in the
        //    contracted edge's ghEdgeId slot so we can map verdicts back.
        AnalysisGraph.Builder cb = new AnalysisGraph.Builder();
        boolean[] addedAsContracted = new boolean[m];
        for (int e = 0; e < m; e++) {
            if (g.isRemoved(e)) continue;
            int ca = anchored[g.nodeA(e)] ? superNode : g.nodeA(e);
            int cbn = anchored[g.nodeB(e)] ? superNode : g.nodeB(e);
            if (ca == cbn) continue;   // edge inside the backbone (or a degenerate self-loop)
            cb.addEdge(e, ca, cbn, EdgeRole.TARGET, g.lengthM(e), e, null, null, 0);
            addedAsContracted[e] = true;
        }
        AnalysisGraph cg = cb.build(superNode + 1);
        final int cm = cg.edgeCount();

        // 3) Bridges of the contracted graph (a biconnected block of one edge is a bridge).
        BiconnectedDecomposition decomp = BiconnectedDecomposition.decompose(cg);
        int[] blockSize = new int[Math.max(1, decomp.blockCount())];
        for (int ce = 0; ce < cm; ce++) {
            int b = decomp.blockOf(ce);
            if (b >= 0) blockSize[b]++;
        }

        // 4) 2-edge-connected components = union over NON-bridge contracted edges. The component of
        //    S is the core (kept unconditionally).
        DisjointSet twoEC = new DisjointSet(superNode + 1);
        for (int ce = 0; ce < cm; ce++) {
            int b = decomp.blockOf(ce);
            boolean bridge = b < 0 || blockSize[b] < 2;
            if (!bridge) twoEC.union(cg.nodeA(ce), cg.nodeB(ce));
        }
        int coreRoot = twoEC.find(superNode);

        // 5) Group the non-core part into maximal appendages / islands: union over contracted edges
        //    whose BOTH endpoints are non-core (i.e. everything beyond the bridge that attaches it
        //    to the core stays together). Bridges core↔pendant join nothing here; the pendant side
        //    carries the group.
        DisjointSet group = new DisjointSet(superNode + 1);
        for (int ce = 0; ce < cm; ce++) {
            int a = cg.nodeA(ce), b = cg.nodeB(ce);
            boolean aCore = twoEC.find(a) == coreRoot;
            boolean bCore = twoEC.find(b) == coreRoot;
            if (!aCore && !bCore) group.union(a, b);
        }

        // 6) Per-group qualifying length (TARGET only), then drop groups below the threshold.
        double[] groupLen = new double[superNode + 1];
        for (int ce = 0; ce < cm; ce++) {
            int orig = cg.ghEdgeId(ce);
            if (g.role(orig) != EdgeRole.TARGET) continue;
            int a = cg.nodeA(ce), b = cg.nodeB(ce);
            boolean aCore = twoEC.find(a) == coreRoot;
            boolean bCore = twoEC.find(b) == coreRoot;
            if (aCore && bCore) continue;            // core through-gravel — kept
            int rep = group.find(aCore ? b : a);     // the non-core side owns the appendage
            groupLen[rep] += g.lengthM(orig);
        }

        int removed = 0;
        for (int ce = 0; ce < cm; ce++) {
            int orig = cg.ghEdgeId(ce);
            if (g.role(orig) != EdgeRole.TARGET || g.isRemoved(orig)) continue;
            int a = cg.nodeA(ce), b = cg.nodeB(ce);
            boolean aCore = twoEC.find(a) == coreRoot;
            boolean bCore = twoEC.find(b) == coreRoot;
            if (aCore && bCore) continue;            // core — keep
            int rep = group.find(aCore ? b : a);
            if (groupLen[rep] < minClusterLenM) {
                g.remove(orig);
                removed++;
            }
        }
        return removed;
    }
}
