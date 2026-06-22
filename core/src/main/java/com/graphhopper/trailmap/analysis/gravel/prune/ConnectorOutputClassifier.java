/*
 * Trailmap - Gravel Segment Analysis
 *
 * Output-only classification of admitted connectors into "needed" vs "redundant". This NEVER mutates
 * connectivity — it runs after the network filter and only labels edges for the output.
 */
package com.graphhopper.trailmap.analysis.gravel.prune;

import com.graphhopper.trailmap.analysis.gravel.graph.AnalysisGraph;
import com.graphhopper.trailmap.analysis.gravel.role.EdgeRole;

import java.util.ArrayList;
import java.util.List;

/**
 * Decides, for the OUTPUT only, which admitted CONNECTOR edges are <b>needed</b> vs <b>redundant</b>.
 *
 * <p>Build the kept network without connectors (kept TARGET ∪ ANCHOR backbone) and pre-merge its
 * components. Then walk the admitted connector edges cheapest (best-quality / shortest weighted) first
 * as a <b>spanning forest</b>: a connector that joins two still-separate components is <b>needed</b>
 * (it establishes a link that nothing else provides — exactly the connectors that rescue gravel); a
 * connector whose two ends are already connected is <b>redundant</b> (a loop beside gravel that is
 * already linked). A rescuing connector <i>mesh</i> thus yields one clean spanning path of needed
 * connectors, with the parallel loops marked redundant.</p>
 *
 * <p>This is a pure labelling pass over the final graph; it removes nothing, so it can never sever a
 * needed link. Connectivity for the filter was already provided by every admitted connector.</p>
 */
public final class ConnectorOutputClassifier {

    private ConnectorOutputClassifier() {
    }

    /**
     * @return {@code needed[e]} = true for each admitted CONNECTOR analysis-edge that lies on the
     *         spanning forest joining the kept network; false for redundant connectors and all
     *         non-connector edges.
     */
    public static boolean[] classify(AnalysisGraph g) {
        final int n = g.nodeCount(), m = g.edgeCount();
        DisjointSet ds = new DisjointSet(n);
        List<Integer> connectors = new ArrayList<>();
        for (int e = 0; e < m; e++) {
            if (g.isRemoved(e)) continue;
            EdgeRole r = g.role(e);
            if (r == EdgeRole.TARGET || r == EdgeRole.ANCHOR) ds.union(g.nodeA(e), g.nodeB(e));
            else if (r == EdgeRole.CONNECTOR) connectors.add(e);
        }
        connectors.sort((a, b) -> Double.compare(cost(g, a), cost(g, b)));

        boolean[] needed = new boolean[m];
        for (int e : connectors) {
            int ra = ds.find(g.nodeA(e)), rb = ds.find(g.nodeB(e));
            if (ra != rb) {
                ds.union(ra, rb);
                needed[e] = true;
            }
        }
        return needed;
    }

    private static double cost(AnalysisGraph g, int e) {
        double w = g.connectorWeight(e);
        return g.lengthM(e) * (w > 0 ? w : 1.0);
    }
}
