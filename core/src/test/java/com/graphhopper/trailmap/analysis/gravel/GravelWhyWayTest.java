package com.graphhopper.trailmap.analysis.gravel;

import com.graphhopper.routing.ev.BooleanEncodedValue;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.ev.IntEncodedValue;
import com.graphhopper.routing.ev.RoadClass;
import com.graphhopper.routing.ev.VehicleAccess;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.trailmap.TrailmapGraphHopper;
import com.graphhopper.trailmap.analysis.gravel.graph.AnalysisGraph;
import com.graphhopper.trailmap.analysis.gravel.prune.ConnectorResolver;
import com.graphhopper.trailmap.analysis.gravel.prune.GravelNetworkFilter;
import com.graphhopper.trailmap.analysis.gravel.role.EdgeRole;
import com.graphhopper.trailmap.shared.GravelScale;
import com.graphhopper.trailmap.shared.PredictedSurface;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import static org.junit.jupiter.api.Assumptions.assumeTrue;

/**
 * PRODUCTISED DIAGNOSTIC — "why is way X kept or dropped by the gravel extraction?"
 *
 * <p>Run it: {@code bash gravel-viz/why_way.sh <wayId> [moreIds...]} (or directly with
 * {@code -Dgravel.diag.ways=26216742,26216737}). For each way it prints ONE grounded verdict:
 * the authoritative keptA/keptB, whether it is 2-edge-connected to the road grid (Test 1) with the
 * specific cut edge if not, the standalone looped core vs τ (Test 2), and the bottom-line reason.</p>
 *
 * <p>This is the reliable diagnostic referenced by the CLAUDE.md "gravel diagnostic protocol".
 * Do NOT reason from any other derived metric (e.g. a TARGET-only "grid junctions" count) — those
 * are misleading. The {@code /route} endpoint at localhost:8989 is ground truth for real
 * connectivity; the user's observation outranks this tool if they disagree.</p>
 */
class GravelWhyWayTest {

    private static final String DATA = "/Users/suomimar/Dropbox/dev/map-server/routing_v2/data";
    private static final String PBF = DATA + "/finland_2.osm.pbf";
    private static final String GRAPH = DATA + "/finland-analysis-gh";
    private static final double THRESHOLD = 2000;

    @Test
    void explainWays() {
        assumeTrue(new File(PBF).exists(), "PBF not present — skipping");
        long[] ways = parseWays(System.getProperty("gravel.diag.ways", "26216742,26216737"));

        GravelAnalysisConfig cfg = new GravelAnalysisConfig();
        cfg.gravelSizeThresholdM = THRESHOLD;
        cfg.enableConnectors = true;
        cfg.validate();

        TrailmapGraphHopper hopper = GravelSegmentTool.buildAndImport(PBF, GRAPH);
        try {
            BaseGraph graph = hopper.getBaseGraph();
            EncodingManager em = hopper.getEncodingManager();
            AnalysisGraph ag = GravelSegmentTool.buildAnalysisGraph(graph, em, cfg, new GravelSegmentTool.Stats());
            if (cfg.enableConnectors) new ConnectorResolver(ag, cfg.connectorCostBudget).resolve();
            double tau = cfg.enableStandaloneRescue ? cfg.effectiveIslandThresholdM() : Double.POSITIVE_INFINITY;
            GravelNetworkFilter.Verdict vd = new GravelNetworkFilter(ag, tau, cfg.gridMinComponentCoreLenM).analyze();

            EnumEncodedValue<GravelScale> gsEnc = em.getEnumEncodedValue(GravelScale.KEY, GravelScale.class);
            EnumEncodedValue<RoadClass> rcEnc = em.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
            EnumEncodedValue<PredictedSurface> psEnc = em.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class);
            IntEncodedValue widEnc = em.getIntEncodedValue("osm_way_id");
            String bikeKey = VehicleAccess.key("bike");
            BooleanEncodedValue bikeEnc = em.hasEncodedValue(bikeKey) ? em.getBooleanEncodedValue(bikeKey) : null;

            // grid vertices (the anchor Test 1 attaches to)
            boolean[] gridNode = new boolean[ag.nodeCount()];
            for (int e = 0; e < ag.edgeCount(); e++)
                if (vd.gridEdge[e]) { gridNode[ag.nodeA(e)] = true; gridNode[ag.nodeB(e)] = true; }

            System.out.println("\n################  WHY-WAY DIAGNOSTIC (τ=" + (int) THRESHOLD + "m)  ################");
            for (long w : ways)
                explain(graph, ag, vd, em, gsEnc, rcEnc, psEnc, widEnc, bikeEnc, gridNode, tau, w);
            System.out.println("################################################################\n");
        } finally {
            hopper.close();
        }
    }

    private void explain(BaseGraph graph, AnalysisGraph ag, GravelNetworkFilter.Verdict vd, EncodingManager em,
                         EnumEncodedValue<GravelScale> gsEnc, EnumEncodedValue<RoadClass> rcEnc,
                         EnumEncodedValue<PredictedSurface> psEnc, IntEncodedValue widEnc,
                         BooleanEncodedValue bikeEnc, boolean[] gridNode, double tau, long wayId) {
        final int N = ag.nodeCount();
        Set<Integer> src = new HashSet<>();
        int tEdges = 0, kA = 0, kB = 0; double len = 0; String name = "", rc = "", surf = "";
        for (int e = 0; e < ag.edgeCount(); e++) {
            if ((ag.osmWayId(e) & 0xFFFFFFFFL) != wayId || ag.role(e) != EdgeRole.TARGET) continue;
            src.add(ag.nodeA(e)); src.add(ag.nodeB(e));
            tEdges++; len += ag.lengthM(e); if (vd.keptA[e]) kA++; if (vd.keptB[e]) kB++;
            if (name.isEmpty() && ag.name(e) != null) name = ag.name(e);
            rc = graph.getEdgeIteratorState(ag.ghEdgeId(e), ag.nodeB(e)).get(rcEnc).toString();
            surf = graph.getEdgeIteratorState(ag.ghEdgeId(e), ag.nodeB(e)).get(psEnc).toString();
        }
        System.out.println("\n=== way " + wayId + " '" + name + "' ===");
        if (tEdges == 0) {
            // Not a TARGET way — say why (role at classification).
            for (int e = 0; e < ag.edgeCount(); e++)
                if ((ag.osmWayId(e) & 0xFFFFFFFFL) == wayId) {
                    System.out.println("  NOT TARGET: classified " + ag.role(e) + " (road_class="
                            + graph.getEdgeIteratorState(ag.ghEdgeId(e), ag.nodeB(e)).get(rcEnc)
                            + ", gravel_scale=" + graph.getEdgeIteratorState(ag.ghEdgeId(e), ag.nodeB(e)).get(gsEnc) + ")");
                    return;
                }
            System.out.println("  not present in the analysis graph (IGNORED at classification, or absent in PBF)");
            return;
        }
        boolean kept = kA > 0 || kB > 0;
        System.out.printf("  %d TARGET edges (%.0fm), road_class=%s, surface=%s%n", tEdges, len, rc, surf);
        System.out.printf("  VERDICT: %s   (keptA=%d/%d via Test 1, keptB=%d/%d via Test 2)%n",
                kept ? "KEPT" : "DROPPED", kA, tEdges, kB, tEdges);

        // Test 1: edge-disjoint routes from the whole way to the road grid.
        int[] pe = new int[N], pn = new int[N];
        Set<Integer> blk = new HashSet<>();
        int g1 = bfs(ag, null, null, bikeEnc, widEnc, src, gridNode, blk, pe, pn);
        int routesAG;
        Integer cutWay = null; int attachNode = -1;
        if (g1 < 0) { routesAG = 0; }
        else {
            attachNode = g1;
            // find the single cut edge (a bridge) on path1, if path2 is blocked
            java.util.List<Integer> path1Edges = new java.util.ArrayList<>();
            for (int v = g1; pe[v] != -1; v = pn[v]) path1Edges.add(pe[v]);
            for (int e : path1Edges) blk.add(e);
            int g2 = bfs(ag, null, null, bikeEnc, widEnc, src, gridNode, blk, pe, pn);
            routesAG = (g2 < 0) ? 1 : 2;
            if (routesAG == 1) {
                for (int e : path1Edges) {       // the bridge = an edge whose removal alone severs src->grid
                    Set<Integer> one = new HashSet<>(); one.add(e);
                    if (bfs(ag, null, null, bikeEnc, widEnc, src, gridNode, one, pe, pn) < 0) { cutWay = (int) (ag.osmWayId(e) & 0xFFFFFFFFL); break; }
                }
            }
        }
        // reality (full bike graph) — distinguishes a real single-access from a classification severance
        int[] pe2 = new int[graph.getNodes()], pn2 = new int[graph.getNodes()];
        Set<Integer> blk2 = new HashSet<>();
        int b1 = bfs(null, graph, em, bikeEnc, widEnc, src, gridNode, blk2, pe2, pn2);
        int routesReal;
        if (b1 < 0) routesReal = 0;
        else { for (int v = b1; pe2[v] != -1; v = pn2[v]) blk2.add(pe2[v]);
               routesReal = bfs(null, graph, em, bikeEnc, widEnc, src, gridNode, blk2, pe2, pn2) < 0 ? 1 : 2; }

        System.out.printf("  Test 1 (2-edge-connected to grid): analysis=%d route(s), reality(bike)=%d route(s)%n",
                routesAG, routesReal);
        if (routesAG < 2 && attachNode >= 0) {
            NodeAccess na = graph.getNodeAccess();
            System.out.printf("    grid attachment at node %d (%.6f,%.6f)%s%n", attachNode,
                    na.getLat(attachNode), na.getLon(attachNode),
                    cutWay != null ? "  | single route gated by cut-edge way " + cutWay : "");
            if (routesAG == 1 && routesReal >= 2)
                System.out.println("    -> reality has 2 routes but analysis has 1: an IGNORED edge severs it (classification/severance).");
            if (routesAG == 1 && routesReal == 1)
                System.out.println("    -> single-access in the routable network too (a 2nd route would need a currently non-routable link, e.g. a closed bridge).");
        }

        // Test 2: standalone looped core vs τ.
        int anyNode = src.iterator().next();
        double core = vd.standaloneCoreLenByNode == null ? -1 : vd.standaloneCoreLenByNode[anyNode];
        System.out.printf("  Test 2 (standalone looped core ≥ τ): core=%.0fm  τ=%.0fm%n", core, tau);

        // bottom line
        if (kept) System.out.println("  >>> KEPT" + (kA > 0 ? " by Test 1 (through-route to the grid)." : " by Test 2 (standalone loop ≥ τ)."));
        else {
            String why = routesAG < 2 ? "only " + routesAG + " independent route to the grid (Test 1 needs 2)" : "fails Test 1";
            String why2 = core >= 0 && core < tau ? "looped core " + (int) core + "m < τ " + (int) tau + "m (Test 2)" : "no qualifying standalone loop";
            System.out.println("  >>> DROPPED — " + why + "; " + why2 + ".");
            System.out.println("      To qualify: a 2nd independent route to the grid, or a standalone looped core ≥ τ.");
        }
    }

    /** Multi-source edge-disjoint BFS over the analysis graph (ag != null) or base bike graph (graph != null),
     *  from sources to any grid node, avoiding blocked edges. Fills pe/pn; returns the reached grid node or -1. */
    private int bfs(AnalysisGraph ag, BaseGraph graph, EncodingManager em, BooleanEncodedValue bikeEnc,
                    IntEncodedValue widEnc, Set<Integer> sources, boolean[] gridNode,
                    Set<Integer> blockedEdges, int[] pe, int[] pn) {
        Arrays.fill(pe, -2);
        ArrayDeque<Integer> q = new ArrayDeque<>();
        for (int s : sources) if (pe[s] == -2) { pe[s] = -1; pn[s] = -1; q.add(s); }
        com.graphhopper.util.EdgeExplorer ex = graph != null ? graph.createEdgeExplorer() : null;
        while (!q.isEmpty()) {
            int u = q.poll();
            if (ag != null) {
                for (int i = ag.adjBegin(u); i < ag.adjEnd(u); i++) {
                    int e = ag.adjEdge(i);
                    if (ag.isRemoved(e) || blockedEdges.contains(e)) continue;
                    int v = ag.other(e, u);
                    if (pe[v] != -2) continue;
                    pe[v] = e; pn[v] = u;
                    if (gridNode[v]) return v;
                    q.add(v);
                }
            } else {
                com.graphhopper.util.EdgeIterator it = ex.setBaseNode(u);
                while (it.next()) {
                    if (bikeEnc != null && !(it.get(bikeEnc) || it.getReverse(bikeEnc))) continue;
                    if (blockedEdges.contains(it.getEdge())) continue;
                    int v = it.getAdjNode();
                    if (pe[v] != -2 || v >= gridNode.length) continue;
                    pe[v] = it.getEdge(); pn[v] = u;
                    if (gridNode[v]) return v;
                    q.add(v);
                }
            }
        }
        return -1;
    }

    private static long[] parseWays(String s) {
        String[] p = s.split(",");
        long[] out = new long[p.length];
        for (int i = 0; i < p.length; i++) out[i] = Long.parseLong(p[i].trim());
        return out;
    }
}
