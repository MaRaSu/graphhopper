package com.graphhopper.trailmap.analysis.gravel;

import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.ev.IntEncodedValue;
import com.graphhopper.routing.ev.RoadClass;
import com.graphhopper.routing.util.AllEdgesIterator;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.trailmap.TrailmapGraphHopper;
import com.graphhopper.trailmap.analysis.gravel.graph.AnalysisGraph;
import com.graphhopper.trailmap.analysis.gravel.prune.BiconnectedDecomposition;
import com.graphhopper.trailmap.analysis.gravel.prune.GravelNetworkFilter;
import com.graphhopper.trailmap.analysis.gravel.role.EdgeRole;
import com.graphhopper.trailmap.analysis.gravel.role.EdgeRoleClassifier;
import com.graphhopper.trailmap.shared.GravelScale;
import com.graphhopper.trailmap.shared.MtbScale;
import com.graphhopper.trailmap.shared.PredictedHighway;
import com.graphhopper.trailmap.shared.PredictedSurface;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assumptions.assumeTrue;

/**
 * Diagnostic trace for specific OSM way IDs: prints each way's role + raw EVs (Phase B),
 * whether it survives the network filter (Phase C/D), and a summary of the gravel cluster it
 * belongs to (looped length + how many points it touches the paved network). Not an assertion
 * test — a tool for understanding why a way is in or out.
 */
class GravelWayDiagnosticTest {

    private static final String DATA = "/Users/suomimar/Dropbox/dev/map-server/routing_v2/data";
    private static final String PBF = DATA + "/tampere.osm.pbf";
    private static final double THRESHOLD = 4000;

    private static final Map<String, long[]> WAYS = new LinkedHashMap<>();
    static {
        // Oracles (must stay correct under any backbone change).
        WAYS.put("ORACLE Hirviniemenranta 41428429 (PRUNED)", new long[]{41428429L});
        WAYS.put("ORACLE Latohuhdantie 27060078 (mostly KEPT)", new long[]{27060078L});
        WAYS.put("ORACLE Varsamaentie 1254246199 (KEPT)", new long[]{1254246199L});
        // Dead-ends that should be gone but survive.
        WAYS.put("DEADEND 79735420", new long[]{79735420L});
        WAYS.put("DEADEND 986643062", new long[]{986643062L});
        WAYS.put("DEADEND 211468847/849/843 (one road, Y-fork one end, dead other)",
                new long[]{211468847L, 211468849L, 211468843L});
        WAYS.put("DEADEND 41035427", new long[]{41035427L});
        // 'Missing' cluster parts.
        WAYS.put("MISSING 39011150 (part of Salmuksentie cluster?)", new long[]{39011150L});
        WAYS.put("MISSING 255421697 + 235886987 (connect target 235886985?)",
                new long[]{255421697L, 235886987L});
        WAYS.put("the target they connect: 235886985", new long[]{235886985L});
    }

    private BaseGraph graph;
    private EncodingManager em;
    private GravelAnalysisConfig cfg;
    private java.util.Set<Long> trackWays;

    @Test
    void diagnose() {
        assumeTrue(new File(PBF).exists(), "tampere.osm.pbf not present — skipping");

        cfg = new GravelAnalysisConfig();
        cfg.gravelSizeThresholdM = THRESHOLD;
        cfg.validate();
        trackWays = new java.util.HashSet<>();
        for (long[] ids : WAYS.values()) for (long id : ids) trackWays.add(id);

        TrailmapGraphHopper hopper =
                GravelSegmentTool.buildAndImport(PBF, new File(DATA, "tampere-analysis-gh").getAbsolutePath());
        try {
            graph = hopper.getBaseGraph();
            em = hopper.getEncodingManager();
            EnumEncodedValue<GravelScale> gravelEnc = em.getEnumEncodedValue(GravelScale.KEY, GravelScale.class);
            EnumEncodedValue<RoadClass> roadClassEnc = em.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
            EnumEncodedValue<PredictedHighway> predictedHighwayEnc =
                    em.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
            EnumEncodedValue<PredictedSurface> predictedSurfaceEnc =
                    em.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class);
            IntEncodedValue wayIdEnc = em.getIntEncodedValue("osm_way_id");
            EdgeRoleClassifier classifier = EdgeRoleClassifier.create(cfg, em);

            // --- Phase B: raw EVs + role for every edge of each way ---
            Map<Long, List<String>> bInfo = new LinkedHashMap<>();
            for (long id : trackWays) bInfo.put(id, new ArrayList<>());
            AllEdgesIterator iter = graph.getAllEdges();
            while (iter.next()) {
                long wid = iter.get(wayIdEnc) & 0xFFFFFFFFL;
                List<String> sink = bInfo.get(wid);
                if (sink == null) continue;
                sink.add(String.format("edge#%d len=%.0fm role=%s gravel_scale=%s surface=%s road_class=%s predHwy=%s name='%s'",
                        iter.getEdge(), iter.getDistance(), classifier.classify(iter),
                        iter.get(gravelEnc), iter.get(predictedSurfaceEnc), iter.get(roadClassEnc),
                        iter.get(predictedHighwayEnc), iter.getName()));
            }

            // --- Phase C/D: survival under the network filter ---
            Map<Long, int[]> surv = runPipeline(THRESHOLD);   // {survived, removed}

            // A/B: does dropping SERVICE from the backbone fix the false-core dead-ends?
            java.util.Set<RoadClass> orig = cfg.backboneRoadClasses;
            java.util.Set<RoadClass> noSvc = java.util.EnumSet.copyOf(orig);
            noSvc.remove(RoadClass.SERVICE);
            cfg.backboneRoadClasses = noSvc;
            Map<Long, int[]> survNoSvc = runPipeline(THRESHOLD);
            cfg.backboneRoadClasses = orig;
            System.out.println("\n--- A/B backbone WITH service vs WITHOUT (survived/removed) ---");
            for (long id : trackWays)
                System.out.printf("  way %d: WITH-svc %d/%d | NO-svc %d/%d%n",
                        id, surv.get(id)[0], surv.get(id)[1], survNoSvc.get(id)[0], survNoSvc.get(id)[1]);

            AnalysisGraph ag0 = GravelSegmentTool.buildAnalysisGraph(graph, em, cfg, new GravelSegmentTool.Stats());

            System.out.println("\n================ WAY DIAGNOSTIC (threshold " + (int) THRESHOLD + " m) ================");
            for (Map.Entry<String, long[]> cat : WAYS.entrySet()) {
                System.out.println("\n## " + cat.getKey());
                for (long id : cat.getValue()) {
                    List<String> edges = bInfo.get(id);
                    int[] s = surv.get(id);
                    String verdict;
                    if (edges.isEmpty()) verdict = "NOT IN GRAPH";
                    else if (s[0] > 0) verdict = "PRESENT (" + s[0] + " TARGET edge(s) survived)";
                    else if (s[1] > 0) verdict = "ABSENT — dropped by network filter (" + s[1] + " edges)";
                    else verdict = "ABSENT — classified ANCHOR/IGNORED at B";
                    System.out.println("  way " + id + " -> " + verdict);
                    for (String line : edges) System.out.println("        " + line);
                    dumpComponentAnchoring(graph, ag0, id, gravelEnc, roadClassEnc);
                    dumpBaseNeighborhood(graph, ag0, id, classifier, gravelEnc, predictedSurfaceEnc,
                            roadClassEnc, predictedHighwayEnc, wayIdEnc);
                }
            }
            System.out.println("\n=====================================================================");
        } finally {
            hopper.close();
        }
    }

    /** BFS the whole TARGET cluster of a way; report its looped length and where it touches ANCHORs. */
    private void dumpComponentAnchoring(BaseGraph graph, AnalysisGraph ag, long wayId,
                                        EnumEncodedValue<GravelScale> gravelEnc,
                                        EnumEncodedValue<RoadClass> rcEnc) {
        int seed = -1;
        for (int e = 0; e < ag.edgeCount(); e++)
            if ((ag.osmWayId(e) & 0xFFFFFFFFL) == wayId && ag.role(e) == EdgeRole.TARGET) { seed = ag.nodeA(e); break; }
        if (seed < 0) return;

        BiconnectedDecomposition d = BiconnectedDecomposition.decompose(ag);
        int[] blockSize = new int[d.blockCount()];
        for (int e = 0; e < ag.edgeCount(); e++) { int b = d.blockOf(e); if (b >= 0) blockSize[b]++; }

        boolean[] nv = new boolean[ag.nodeCount()];
        boolean[] es = new boolean[ag.edgeCount()];
        int[] q = new int[ag.nodeCount()];
        int head = 0, tail = 0; nv[seed] = true; q[tail++] = seed;
        int tEdges = 0; double totalLen = 0, loopedLen = 0;
        boolean[] anchorSeen = new boolean[ag.edgeCount()];
        java.util.Map<RoadClass, Integer> connByClass = new java.util.TreeMap<>();
        while (head < tail) {
            int u = q[head++];
            for (int i = ag.adjBegin(u); i < ag.adjEnd(u); i++) {
                int e = ag.adjEdge(i);
                if (ag.role(e) == EdgeRole.ANCHOR) {
                    if (!anchorSeen[e]) {
                        anchorSeen[e] = true;
                        RoadClass rc = graph.getEdgeIteratorState(ag.ghEdgeId(e), ag.other(e, u)).get(rcEnc);
                        connByClass.merge(rc, 1, Integer::sum);
                    }
                    continue;
                }
                if (!es[e]) { es[e] = true; tEdges++; double L = ag.lengthM(e); totalLen += L; if (blockSize[d.blockOf(e)] >= 2) loopedLen += L; }
                int v = ag.other(e, u);
                if (!nv[v]) { nv[v] = true; q[tail++] = v; }
            }
        }
        System.out.printf("        => CLUSTER: %d TARGET edges, total=%.0fm, looped=%.0fm; connects to ANCHOR roads by class: %s%n",
                tEdges, totalLen, loopedLen, connByClass);
        for (int e = 0; e < ag.edgeCount(); e++)
            if ((ag.osmWayId(e) & 0xFFFFFFFFL) == wayId && ag.role(e) == EdgeRole.TARGET)
                System.out.printf("           this way edge#%d len=%.0fm -> %s%n", ag.ghEdgeId(e), ag.lengthM(e),
                        blockSize[d.blockOf(e)] >= 2 ? "ON-CYCLE (kept)" : "bridge (pruned)");
    }

    /** Experiment: build connectivity from every road EXCEPT the given road_classes; report whether
     *  each tracked way's TARGET edges lie on a cycle. Shows what excluding paths etc. would do. */
    private void experimentConnectivity(String label, java.util.Set<RoadClass> excluded,
                                        BaseGraph graph, EdgeRoleClassifier classifier,
                                        EnumEncodedValue<RoadClass> rcEnc, IntEncodedValue wayIdEnc) {
        AnalysisGraph.Builder b = new AnalysisGraph.Builder();
        AllEdgesIterator it = graph.getAllEdges();
        while (it.next()) {
            boolean target = classifier.classify(it) == EdgeRole.TARGET;
            if (!target && excluded.contains(it.get(rcEnc))) continue;   // excluded from connectivity
            b.addEdge(it.getEdge(), it.getBaseNode(), it.getAdjNode(),
                    target ? EdgeRole.TARGET : EdgeRole.ANCHOR, it.getDistance(),
                    it.get(wayIdEnc), null, null, 0);
        }
        AnalysisGraph ag = b.build(graph.getNodes());
        BiconnectedDecomposition d = BiconnectedDecomposition.decompose(ag);
        int[] bs = new int[d.blockCount()];
        for (int e = 0; e < ag.edgeCount(); e++) { int bl = d.blockOf(e); if (bl >= 0) bs[bl]++; }
        System.out.println("  [" + label + "]");
        for (long id : trackWays) {
            int looped = 0, bridge = 0; double loopLen = 0;
            for (int e = 0; e < ag.edgeCount(); e++) {
                if ((ag.osmWayId(e) & 0xFFFFFFFFL) != id || ag.role(e) != EdgeRole.TARGET) continue;
                if (bs[d.blockOf(e)] >= 2) { looped++; loopLen += ag.lengthM(e); } else bridge++;
            }
            System.out.printf("      way %d: %d on-cycle + %d bridge (%.0fm on-cycle)%n", id, looped, bridge, loopLen);
        }
    }

    /** List ALL incident base-graph edges (incl. IGNORED) at each node the way touches. */
    private void dumpBaseNeighborhood(BaseGraph graph, AnalysisGraph ag, long wayId,
                                      EdgeRoleClassifier classifier,
                                      EnumEncodedValue<GravelScale> gravelEnc,
                                      EnumEncodedValue<PredictedSurface> surfEnc,
                                      EnumEncodedValue<RoadClass> rcEnc,
                                      EnumEncodedValue<PredictedHighway> phEnc,
                                      IntEncodedValue wayIdEnc) {
        java.util.Set<Integer> nodes = new java.util.LinkedHashSet<>();
        for (int e = 0; e < ag.edgeCount(); e++)
            if ((ag.osmWayId(e) & 0xFFFFFFFFL) == wayId) { nodes.add(ag.nodeA(e)); nodes.add(ag.nodeB(e)); }
        com.graphhopper.util.EdgeExplorer ex = graph.createEdgeExplorer();
        for (int node : nodes) {
            System.out.println("        node " + node + " — all incident base edges:");
            com.graphhopper.util.EdgeIterator it = ex.setBaseNode(node);
            while (it.next()) {
                System.out.printf("            %-7s way=%d '%s' len=%.0fm gravel=%s surface=%s road_class=%s predHwy=%s%n",
                        classifier.classify(it), it.get(wayIdEnc) & 0xFFFFFFFFL, it.getName(), it.getDistance(),
                        it.get(gravelEnc), it.get(surfEnc), it.get(rcEnc), it.get(phEnc));
            }
        }
    }

    /** Build a fresh working graph, run the network filter, return per-way {survived, removed}. */
    private Map<Long, int[]> runPipeline(double threshold) {
        AnalysisGraph ag = GravelSegmentTool.buildAnalysisGraph(graph, em, cfg, new GravelSegmentTool.Stats());
        Map<Integer, Long> tracked = new LinkedHashMap<>();
        for (int e = 0; e < ag.edgeCount(); e++) {
            long wid = ag.osmWayId(e) & 0xFFFFFFFFL;
            if (trackWays.contains(wid) && ag.role(e) == EdgeRole.TARGET) tracked.put(e, wid);
        }
        new GravelNetworkFilter(ag, threshold).filter();
        Map<Long, int[]> stage = new LinkedHashMap<>();
        for (long id : trackWays) stage.put(id, new int[2]);
        for (Map.Entry<Integer, Long> en : tracked.entrySet()) {
            int[] s = stage.get(en.getValue());
            if (ag.isRemoved(en.getKey())) s[1]++; else s[0]++;
        }
        return stage;
    }
}
