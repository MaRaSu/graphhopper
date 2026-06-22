/*
 * Trailmap - Gravel Segment Analysis
 *
 * Network filter. Realizes docs/gravel_segments_goal.md with standard graph algorithms only
 * (Tarjan bridges / 2-edge-connected components) — no contraction, no bespoke constructs.
 *
 * Implements both keep rules from the goal:
 *   (A) gravel 2-edge-connected to the road grid (a through-route), any length;
 *   (B) gravel in a standalone cluster whose looped core (2-edge-connected structure) totals ≥ τ —
 *       the looped core and its access to the kept network are kept, the cluster's tails are not.
 * Pendant tails are dropped uniformly. τ is measured on the looped core, which is invariant under
 * pendant removal, so the whole thing is a fixed number of standard passes with no fixpoint.
 */
package com.graphhopper.trailmap.analysis.gravel.prune;

import com.graphhopper.trailmap.analysis.gravel.graph.AnalysisGraph;
import com.graphhopper.trailmap.analysis.gravel.role.EdgeRole;

/**
 * Keeps a GRAVEL edge iff it is <b>2-edge-connected to the road grid</b> (goal §6 case A): you can
 * reach it from the grid and leave a different way. Everything standalone (reachable only one way,
 * or isolated) is dropped here and handled by Step 2.
 *
 * <p><b>Model (goal §3).</b> The base graph is ROAD ∪ GRAVEL. ROAD = real-road (backbone) ANCHOR
 * edges; GRAVEL = TARGET. Rough/ground tracks and connectors are excluded from the base — they are
 * a later phase.</p>
 *
 * <p><b>Algorithm.</b> Three standard passes:</p>
 * <ol>
 *   <li><b>Road grid</b> (goal §5) = the 2-edge-connected core of ROAD: road edges on a cycle, or
 *       on a path between two cycles. Computed as the road edges 2-edge-connected to the set of
 *       "on a road cycle" vertices.</li>
 *   <li>The gravel <b>anchor vertices</b> = vertices incident to a grid edge.</li>
 *   <li><b>Keep</b> gravel that is 2-edge-connected to those anchor vertices, over GRID ∪ GRAVEL.</li>
 * </ol>
 *
 * <p><b>"2-edge-connected to a set of anchor vertices" — the standard primitive.</b> Add one virtual
 * super-node S joined to each anchor vertex by a single edge, then an edge is 2-edge-connected to
 * the anchor set iff it is 2-edge-connected to S (in S's 2-edge-connected component). This is the
 * textbook super-terminal construction. It is <i>not</i> a contraction: anchor vertices are joined
 * to S by edges, not merged — so a loop attached at a single anchor vertex stays beyond a bridge
 * (its single S-edge) and is correctly NOT 2-edge-connected, while a through-route between two
 * anchors forms a real cycle with S and is.</p>
 */
public class GravelNetworkFilter {

    private final AnalysisGraph g;
    /** τ — the minimum looped-core length for a standalone gravel cluster to survive (goal §7). */
    private final double minStandaloneLenM;
    /** Minimum cyclic-core length for a ROAD component to count as real grid (goal §5); 0 disables. */
    private final double minGridComponentCoreM;

    public GravelNetworkFilter(AnalysisGraph g, double minStandaloneLenM) {
        this(g, minStandaloneLenM, 0);
    }

    /**
     * @param minGridComponentCoreM a road connected-component contributes to the grid only if its
     *        2-edge-connected (cyclic) core totals at least this many metres. Excludes isolated tiny
     *        road loops (parking aisles, turning circles, yards) that are 2-edge-connected in isolation
     *        yet are not the real through-road grid, so they cannot anchor a dead-end gravel road as a
     *        false junction (goal §5). 0 disables the floor (grid = full 2-edge-connected core).
     */
    public GravelNetworkFilter(AnalysisGraph g, double minStandaloneLenM, double minGridComponentCoreM) {
        this.g = g;
        this.minStandaloneLenM = minStandaloneLenM;
        this.minGridComponentCoreM = minGridComponentCoreM;
    }

    /** Per-edge verdicts, exposed for diagnosis (no graph mutation). */
    public static final class Verdict {
        public final boolean[] road, gravel, gridEdge, keptA, keptB;
        /** Standalone (case-B) diagnosis: the looped-core mask, and per-node the core length of the
         *  standalone gravel component the node belongs to. Null when standalone rescue did not run. */
        public final boolean[] standaloneCore;
        public final double[] standaloneCoreLenByNode;
        Verdict(boolean[] road, boolean[] gravel, boolean[] gridEdge, boolean[] keptA, boolean[] keptB,
                boolean[] standaloneCore, double[] standaloneCoreLenByNode) {
            this.road = road; this.gravel = gravel; this.gridEdge = gridEdge;
            this.keptA = keptA; this.keptB = keptB;
            this.standaloneCore = standaloneCore;
            this.standaloneCoreLenByNode = standaloneCoreLenByNode;
        }
    }

    /** Compute the grid and the keep verdicts without mutating the graph (used by {@link #filter()}). */
    public Verdict analyze() {
        final int m = g.edgeCount();

        boolean[] road = new boolean[m];      // R: real-road backbone
        boolean[] gravel = new boolean[m];    // T
        boolean[] connector = new boolean[m]; // admitted Step-3 connectors (carry paths, never anchor)
        for (int e = 0; e < m; e++) {
            if (g.isRemoved(e)) continue;
            if (g.role(e) == EdgeRole.TARGET) gravel[e] = true;
            else if (g.role(e) == EdgeRole.ANCHOR && g.isBackbone(e)) road[e] = true;
            else if (g.role(e) == EdgeRole.CONNECTOR) connector[e] = true;
        }

        // (1) Road grid (goal §5) = the 2-edge-connected (cyclic) core of ROAD — but only within road
        // connected-components that are a real through-road network. The full 2-edge-connected core
        // alone treats every road cycle as grid, including an ISOLATED tiny service loop (a parking
        // aisle / turning circle / yard) whose only link to the rest of the world is through gravel,
        // not roads. Such a stray loop would falsely anchor a dead-end gravel road as a "through-route"
        // (real bug: way 329963439 via the ~63 m service loop 1202926746). So a road component counts
        // as grid only if its own cyclic-core length ≥ minGridComponentCoreM; the main road network
        // (one huge component) always clears it, the stray loops do not.
        boolean[] onCycleRoadVertex = onCycleVertices(road);
        boolean[] gridCore = keep2ecToAnchor(road, onCycleRoadVertex);
        boolean[] gridEdge = restrictGridToRealComponents(road, gridCore);

        // (2) Gravel anchor vertices = vertices incident to a grid edge.
        boolean[] gridVertex = new boolean[g.nodeCount()];
        for (int e = 0; e < m; e++) {
            if (!gridEdge[e]) continue;
            gridVertex[g.nodeA(e)] = true;
            gridVertex[g.nodeB(e)] = true;
        }

        // (3) Case A: keep gravel that lies on a through-route between two DISTINCT grid junctions
        // (goal §6) — i.e. 2-vertex-connected (biconnected) to the grid. Connectivity spans ALL of
        // R ∪ T (any road edge may carry a path between gravel), but only GRID vertices anchor: a
        // road on a pendant of R carries paths yet its endpoints are not grid vertices, so it adds
        // no anchor junction. A structure sharing a single grid junction is separated by that
        // cut-vertex and is NOT case A.
        boolean[] activeA = new boolean[m];
        for (int e = 0; e < m; e++) activeA[e] = road[e] || gravel[e] || connector[e];
        boolean[] caseA = keepBiconnectedToAnchor(activeA, gridVertex);
        boolean[] keptA = new boolean[m];
        for (int e = 0; e < m; e++) keptA[e] = gravel[e] && caseA[e];

        // (4) Case B: rescue standalone gravel clusters whose looped core totals ≥ τ.
        dbgCore = null;
        dbgCoreLenByNode = null;
        boolean[] keptB = standaloneRescue(gravel, keptA, gridEdge);

        return new Verdict(road, gravel, gridEdge, keptA, keptB, dbgCore, dbgCoreLenByNode);
    }

    // Diagnosis-only snapshots from the last standaloneRescue (not used by the algorithm).
    private boolean[] dbgCore;
    private double[] dbgCoreLenByNode;

    /** Run the filter. Returns the number of GRAVEL analysis edges removed. */
    public int filter() {
        return filter(analyze());
    }

    /**
     * Apply a precomputed verdict: remove every GRAVEL edge kept by neither case. Exposed so a caller
     * that already called {@link #analyze()} (e.g. to read per-edge {@code keptA}/{@code keptB} for
     * output tagging) can reuse the same verdict instead of recomputing it.
     */
    public int filter(Verdict v) {
        int removed = 0;
        for (int e = 0; e < g.edgeCount(); e++) {
            if (v.gravel[e] && !v.keptA[e] && !v.keptB[e]) {
                g.remove(e);
                removed++;
            }
        }
        return removed;
    }

    /**
     * Keep grid-core edges only where their ROAD connected-component has a cyclic-core length of at
     * least {@code minGridComponentCoreM}. The main road network (a huge connected component) always
     * qualifies; an isolated tiny road loop (parking aisle / turning circle / yard) does not, so it
     * never becomes a grid junction. With the floor at 0 this is a no-op (grid = full cyclic core).
     */
    private boolean[] restrictGridToRealComponents(boolean[] road, boolean[] gridCore) {
        final int m = g.edgeCount();
        if (minGridComponentCoreM <= 0) return gridCore;
        final int n = g.nodeCount();
        DisjointSet roadComp = new DisjointSet(n);
        for (int e = 0; e < m; e++) if (road[e]) roadComp.union(g.nodeA(e), g.nodeB(e));
        double[] compCoreLen = new double[n];
        for (int e = 0; e < m; e++) if (gridCore[e]) compCoreLen[roadComp.find(g.nodeA(e))] += g.lengthM(e);
        boolean[] gridEdge = new boolean[m];
        for (int e = 0; e < m; e++)
            if (gridCore[e] && compCoreLen[roadComp.find(g.nodeA(e))] >= minGridComponentCoreM)
                gridEdge[e] = true;
        return gridEdge;
    }

    /**
     * Case B (goal §6/§7). The standalone gravel = gravel not kept by case A. For each standalone
     * cluster, measure its <b>looped core</b> (gravel 2-edge-connected to its own loops). If the core
     * totals ≥ τ, keep the core and its access to the already-kept network (grid ∪ case-A gravel);
     * drop the cluster's tails. The looped core is invariant under pendant removal, so this is the
     * same primitive applied a fixed number of times — no fixpoint.
     */
    private boolean[] standaloneRescue(boolean[] gravel, boolean[] keptA, boolean[] gridEdge) {
        final int m = g.edgeCount();
        final int n = g.nodeCount();
        boolean[] keptB = new boolean[m];

        boolean[] r2 = new boolean[m];   // standalone gravel
        boolean any = false;
        for (int e = 0; e < m; e++) {
            r2[e] = gravel[e] && !keptA[e];
            any |= r2[e];
        }
        if (!any || Double.isInfinite(minStandaloneLenM)) return keptB;

        // Vertices of the already-kept network a standalone cluster may attach to for access.
        boolean[] anchoredVertex = new boolean[n];
        for (int e = 0; e < m; e++) {
            if (!(gridEdge[e] || keptA[e])) continue;
            anchoredVertex[g.nodeA(e)] = true;
            anchoredVertex[g.nodeB(e)] = true;
        }

        // Looped core = the genuinely CYCLIC standalone gravel — edges that actually lie on a gravel
        // cycle (non-bridges). Bridges — dead-end roads, and roads that merely link loops — are NOT
        // counted, so a tiny turnaround loop yields a tiny core and cannot rescue a dead-end (goal
        // §8). τ is measured on this cyclic length only.
        boolean[] cyclic = cyclicEdges(r2);
        boolean[] loopVertex = new boolean[n];
        for (int e = 0; e < m; e++) if (cyclic[e]) {
            loopVertex[g.nodeA(e)] = true;
            loopVertex[g.nodeB(e)] = true;
        }

        // Total cyclic (loop) length per standalone cluster (gravel-connected component).
        DisjointSet comp = new DisjointSet(n);
        for (int e = 0; e < m; e++) if (r2[e]) comp.union(g.nodeA(e), g.nodeB(e));
        double[] coreLen = new double[n];
        for (int e = 0; e < m; e++) if (cyclic[e]) coreLen[comp.find(g.nodeA(e))] += g.lengthM(e);

        // Anchor = the loop vertices of qualifying clusters, plus the kept-network attachment points.
        boolean[] anchorB = new boolean[n];
        for (int v = 0; v < n; v++) {
            if (loopVertex[v] && coreLen[comp.find(v)] >= minStandaloneLenM) anchorB[v] = true;
            if (anchoredVertex[v]) anchorB[v] = true;
        }

        boolean[] kb = keep2ecToAnchor(r2, anchorB);
        for (int e = 0; e < m; e++) keptB[e] = r2[e] && kb[e];

        // Diagnosis snapshot: cyclic-core mask + per-node component cyclic length.
        dbgCore = cyclic;
        dbgCoreLenByNode = new double[n];
        for (int v = 0; v < n; v++) dbgCoreLenByNode[v] = coreLen[comp.find(v)];

        return keptB;
    }

    /** Edges of the active subgraph that lie on a cycle (non-bridges), by analysis-edge id. */
    private boolean[] cyclicEdges(boolean[] active) {
        AnalysisGraph sub = buildSub(active, null);
        boolean[] bridge = bridgesOf(sub);
        boolean[] cyclic = new boolean[g.edgeCount()];
        for (int i = 0; i < sub.edgeCount(); i++) {
            int orig = sub.ghEdgeId(i);
            if (orig < 0 || bridge[i]) continue;
            cyclic[orig] = true;
        }
        return cyclic;
    }

    /** Vertices incident to a non-bridge edge of the active subgraph (i.e. lying on a cycle). */
    private boolean[] onCycleVertices(boolean[] active) {
        AnalysisGraph sub = buildSub(active, null);
        boolean[] bridge = bridgesOf(sub);
        boolean[] onCycle = new boolean[g.nodeCount()];
        for (int i = 0; i < sub.edgeCount(); i++) {
            int orig = sub.ghEdgeId(i);
            if (orig < 0 || bridge[i]) continue;
            onCycle[g.nodeA(orig)] = true;
            onCycle[g.nodeB(orig)] = true;
        }
        return onCycle;
    }

    /**
     * Active edges <b>2-vertex-connected (biconnected)</b> to {@code anchorVertex}, via a virtual
     * super-node S joined to each anchor vertex. An active edge is kept iff its biconnected block
     * contains a virtual S-edge — equivalently, it lies on a cycle through S that uses two distinct
     * anchor vertices (a through-route between two distinct grid junctions, or a cycle that includes
     * a grid edge). A structure attached to the anchors at a single cut-vertex is a separate block
     * and is NOT kept. Used for gravel case A.
     */
    private boolean[] keepBiconnectedToAnchor(boolean[] active, boolean[] anchorVertex) {
        AnalysisGraph sub = buildSub(active, anchorVertex);
        BiconnectedDecomposition d = BiconnectedDecomposition.decompose(sub);
        boolean[] blockHasSuper = new boolean[Math.max(1, d.blockCount())];
        for (int i = 0; i < sub.edgeCount(); i++) {
            if (sub.ghEdgeId(i) != -1) continue;             // only virtual S-edges
            int bl = d.blockOf(i);
            if (bl >= 0) blockHasSuper[bl] = true;
        }
        boolean[] kept = new boolean[g.edgeCount()];
        for (int i = 0; i < sub.edgeCount(); i++) {
            int orig = sub.ghEdgeId(i);
            if (orig < 0) continue;
            int bl = d.blockOf(i);
            if (bl >= 0 && blockHasSuper[bl]) kept[orig] = true;
        }
        return kept;
    }

    /**
     * Active edges 2-edge-connected to {@code anchorVertex}, via a virtual super-node S joined to
     * each anchor vertex. Returns kept[origEdge] for active edges (indexed by analysis-edge id).
     */
    private boolean[] keep2ecToAnchor(boolean[] active, boolean[] anchorVertex) {
        final int superNode = g.nodeCount();
        AnalysisGraph sub = buildSub(active, anchorVertex);
        boolean[] bridge = bridgesOf(sub);

        DisjointSet ds = new DisjointSet(superNode + 1);
        for (int i = 0; i < sub.edgeCount(); i++)
            if (!bridge[i]) ds.union(sub.nodeA(i), sub.nodeB(i));
        int sRoot = ds.find(superNode);

        boolean[] kept = new boolean[g.edgeCount()];
        for (int i = 0; i < sub.edgeCount(); i++) {
            int orig = sub.ghEdgeId(i);
            if (orig < 0 || bridge[i]) continue;                 // virtual, or a bridge → never core
            if (ds.find(sub.nodeA(i)) == sRoot) kept[orig] = true;
        }
        return kept;
    }

    /**
     * Build the subgraph induced by {@code active} edges (analysis-edge id stored in ghEdgeId), plus
     * — if {@code anchorVertex != null} — a virtual super-node {@code g.nodeCount()} joined to each
     * anchor vertex by a single edge (stored with ghEdgeId -1).
     */
    private AnalysisGraph buildSub(boolean[] active, boolean[] anchorVertex) {
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        for (int e = 0; e < g.edgeCount(); e++)
            if (active[e])
                b.addEdge(e, g.nodeA(e), g.nodeB(e), EdgeRole.TARGET, g.lengthM(e), e, null, null, 0, false);
        int superNode = g.nodeCount();
        if (anchorVertex != null)
            for (int v = 0; v < anchorVertex.length; v++)
                if (anchorVertex[v])
                    b.addEdge(-1, superNode, v, EdgeRole.TARGET, 0, -1, null, null, 0, false);
        return b.build(superNode + 1);
    }

    /** bridge[i] for every edge i of {@code sub}: true iff it lies on no cycle (a bridge). */
    private static boolean[] bridgesOf(AnalysisGraph sub) {
        BiconnectedDecomposition d = BiconnectedDecomposition.decompose(sub);
        int[] blockSize = new int[Math.max(1, d.blockCount())];
        for (int i = 0; i < sub.edgeCount(); i++) {
            int bl = d.blockOf(i);
            if (bl >= 0) blockSize[bl]++;
        }
        boolean[] bridge = new boolean[sub.edgeCount()];
        for (int i = 0; i < sub.edgeCount(); i++) {
            int bl = d.blockOf(i);
            bridge[i] = bl < 0 || blockSize[bl] < 2;
        }
        return bridge;
    }
}
