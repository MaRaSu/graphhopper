package com.graphhopper.trailmap.analysis.gravel;

import com.graphhopper.routing.ev.BooleanEncodedValue;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.ev.IntEncodedValue;
import com.graphhopper.routing.ev.RoadAccess;
import com.graphhopper.routing.ev.RoadClass;
import com.graphhopper.routing.ev.VehicleAccess;
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
    private static final String PBF = DATA + "/finland_2.osm.pbf";
    private static final String GRAPH = DATA + "/finland-analysis-gh";
    private static final double THRESHOLD = 2000;

    private static final Map<String, long[]> WAYS = new LinkedHashMap<>();
    static {
        // Two ways the user reports qualify but are NOT marked as gravel segments.
        WAYS.put("missing 26216737", new long[]{26216737L});
        WAYS.put("missing 26216742", new long[]{26216742L});
    }

    private BaseGraph graph;
    private EncodingManager em;
    private GravelAnalysisConfig cfg;
    private java.util.Set<Long> trackWays;

    @Test
    void diagnose() {
        assumeTrue(new File(PBF).exists(), "finland_2.osm.pbf not present — skipping");

        cfg = new GravelAnalysisConfig();
        cfg.gravelSizeThresholdM = THRESHOLD;
        cfg.enableConnectors = true;        // Step 3 on, to inspect connector candidacy/admission
        cfg.validate();
        trackWays = new java.util.HashSet<>();
        for (long[] ids : WAYS.values()) for (long id : ids) trackWays.add(id);

        TrailmapGraphHopper hopper =
                GravelSegmentTool.buildAndImport(PBF, GRAPH);
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

            // Access EVs to understand restrictions (guarded — may be absent in some builds).
            String bikeAccessKey = VehicleAccess.key("bike");
            BooleanEncodedValue bikeAccessEnc = em.hasEncodedValue(bikeAccessKey)
                    ? em.getBooleanEncodedValue(bikeAccessKey) : null;
            EnumEncodedValue<RoadAccess> bikeRoadAccessEnc = em.hasEncodedValue("bike_road_access")
                    ? em.getEnumEncodedValue("bike_road_access", RoadAccess.class) : null;
            EnumEncodedValue<RoadAccess> roadAccessEnc = em.hasEncodedValue(RoadAccess.KEY)
                    ? em.getEnumEncodedValue(RoadAccess.KEY, RoadAccess.class) : null;

            // --- Phase B: raw EVs + role for every edge of each way ---
            Map<Long, List<String>> bInfo = new LinkedHashMap<>();
            for (long id : trackWays) bInfo.put(id, new ArrayList<>());
            AllEdgesIterator iter = graph.getAllEdges();
            while (iter.next()) {
                long wid = iter.get(wayIdEnc) & 0xFFFFFFFFL;
                List<String> sink = bInfo.get(wid);
                if (sink == null) continue;
                sink.add(String.format("edge#%d len=%.0fm role=%s gravel_scale=%s surface=%s road_class=%s predHwy=%s bike_access=%s bike_road_access=%s road_access=%s name='%s'",
                        iter.getEdge(), iter.getDistance(), classifier.classify(iter),
                        iter.get(gravelEnc), iter.get(predictedSurfaceEnc), iter.get(roadClassEnc),
                        iter.get(predictedHighwayEnc),
                        bikeAccessEnc == null ? "n/a" : iter.get(bikeAccessEnc),
                        bikeRoadAccessEnc == null ? "n/a" : iter.get(bikeRoadAccessEnc),
                        roadAccessEnc == null ? "n/a" : iter.get(roadAccessEnc),
                        iter.getName()));
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
            if (cfg.enableConnectors) {
                new com.graphhopper.trailmap.analysis.gravel.prune.ConnectorResolver(
                        ag0, cfg.connectorCostBudget).resolve();
                System.out.println("\n--- CONNECTOR status per tracked way (role in graph + admitted?) ---");
                for (long id : trackWays) {
                    boolean found = false;
                    for (int e = 0; e < ag0.edgeCount(); e++) {
                        if ((ag0.osmWayId(e) & 0xFFFFFFFFL) != id) continue;
                        found = true;
                        System.out.printf("  way %d edge#%d role=%s len=%.0fm weight=%.1f -> %s%n",
                                id, ag0.ghEdgeId(e), ag0.role(e), ag0.lengthM(e), ag0.connectorWeight(e),
                                ag0.isRemoved(e) ? "REMOVED" : "kept");
                    }
                    if (!found) System.out.printf("  way %d: no TARGET/ANCHOR/CONNECTOR edge in graph "
                            + "(IGNORED at classify)%n", id);
                }
            }

            // --- Grid connectivity per way: how many distinct road-grid junctions does the way's
            //     gravel component touch? (case A needs >= 2.) ---
            GravelNetworkFilter.Verdict vd =
                    new GravelNetworkFilter(ag0, THRESHOLD, cfg.gridMinComponentCoreLenM).analyze();
            boolean[] gridVtx = new boolean[ag0.nodeCount()];
            for (int e = 0; e < ag0.edgeCount(); e++)
                if (vd.gridEdge[e]) { gridVtx[ag0.nodeA(e)] = true; gridVtx[ag0.nodeB(e)] = true; }
            int[] gcomp = new int[ag0.nodeCount()];
            for (int i = 0; i < gcomp.length; i++) gcomp[i] = i;
            for (int e = 0; e < ag0.edgeCount(); e++)
                if (vd.gravel[e]) ufUnion(gcomp, ag0.nodeA(e), ag0.nodeB(e));
            System.out.println("\n--- GRID CONNECTIVITY per way (case A needs >= 2 distinct grid junctions) ---");
            for (long id : trackWays) {
                int seed = -1;
                for (int e = 0; e < ag0.edgeCount(); e++)
                    if ((ag0.osmWayId(e) & 0xFFFFFFFFL) == id && vd.gravel[e]) { seed = ag0.nodeA(e); break; }
                if (seed < 0) { System.out.printf("  way %d: not a TARGET gravel way%n", id); continue; }
                int root = ufFind(gcomp, seed);
                int junctions = 0;
                for (int v2 = 0; v2 < ag0.nodeCount(); v2++)
                    if (gridVtx[v2] && ufFind(gcomp, v2) == root) junctions++;
                int kA = 0, kB = 0, tot = 0, inCore = 0;
                for (int e = 0; e < ag0.edgeCount(); e++)
                    if ((ag0.osmWayId(e) & 0xFFFFFFFFL) == id && vd.gravel[e]) {
                        tot++; if (vd.keptA[e]) kA++; if (vd.keptB[e]) kB++;
                        if (vd.standaloneCore != null && vd.standaloneCore[e]) inCore++;
                    }
                double coreLen = vd.standaloneCoreLenByNode == null ? -1 : vd.standaloneCoreLenByNode[seed];
                System.out.printf("  way %d: grid junctions=%d; edges=%d keptA=%d keptB=%d inLoopedCore=%d; cluster core=%.0fm%n",
                        id, junctions, tot, kA, kB, inCore, coreLen);
                // dead-tip check: any node of this way with degree 1 in the TARGET gravel graph.
                java.util.Map<Integer,Integer> deg = new java.util.HashMap<>();
                for (int e = 0; e < ag0.edgeCount(); e++) if (vd.gravel[e]) {
                    deg.merge(ag0.nodeA(e), 1, Integer::sum); deg.merge(ag0.nodeB(e), 1, Integer::sum);
                }
                java.util.Set<Integer> wnodes = new java.util.HashSet<>();
                for (int e = 0; e < ag0.edgeCount(); e++)
                    if ((ag0.osmWayId(e) & 0xFFFFFFFFL) == id && vd.gravel[e]) { wnodes.add(ag0.nodeA(e)); wnodes.add(ag0.nodeB(e)); }
                int deadTips = 0;
                for (int nd : wnodes) if (deg.getOrDefault(nd, 0) == 1) deadTips++;
                System.out.printf("        gravel-graph dead tips on this way: %d (of %d nodes)%n", deadTips, wnodes.size());
            }

            // --- LEVER EXPERIMENT: which lever drops these? Vary grid floor / connectors / tau. ---
            System.out.println("\n--- LEVER EXPERIMENT (survived/removed per tracked way) ---");
            double sFloor = cfg.gridMinComponentCoreLenM;
            boolean sConn = cfg.enableConnectors;
            Object[][] exps = {
                    {"baseline floor=3000 conn=on  tau=2000", 3000.0, true,  2000.0},
                    {"floor=0     conn=on  tau=2000        ", 0.0,    true,  2000.0},
                    {"floor=0     conn=off tau=2000        ", 0.0,    false, 2000.0},
                    {"floor=0     conn=off tau=INF (caseA) ", 0.0,    false, Double.POSITIVE_INFINITY},
                    {"floor=3000  conn=on  tau=50  (caseB) ", 3000.0, true,  50.0},
            };
            for (Object[] ex : exps) {
                cfg.gridMinComponentCoreLenM = (double) ex[1];
                cfg.enableConnectors = (boolean) ex[2];
                Map<Long, int[]> r = runPipeline((double) ex[3]);
                StringBuilder sb = new StringBuilder();
                for (long id : trackWays) sb.append(String.format("  %d:%d/%d", id, r.get(id)[0], r.get(id)[1]));
                System.out.println("  " + ex[0] + " ->" + sb);
            }
            cfg.gridMinComponentCoreLenM = sFloor;
            cfg.enableConnectors = sConn;

            // Global grid size under each floor (is the floor killing a real local grid?).
            for (double fl : new double[]{0, 300, 3000}) {
                AnalysisGraph agf = GravelSegmentTool.buildAnalysisGraph(graph, em, cfg, new GravelSegmentTool.Stats());
                GravelNetworkFilter.Verdict vf = new GravelNetworkFilter(agf, THRESHOLD, fl).analyze();
                int ge = 0; for (int e = 0; e < agf.edgeCount(); e++) if (vf.gridEdge[e]) ge++;
                System.out.printf("  grid edges globally @ floor=%.0f : %d%n", fl, ge);
            }

            // --- REACHABILITY: what does Morsfjärdintie's analysis component actually reach? ---
            // BFS over the analysis graph (TARGET ∪ ANCHOR ∪ CONNECTOR) from a 26216742 node, and
            // see whether it reaches a real road grid; then dump the IGNORED base edges on its
            // boundary (edges the route rides but the classifier drops, severing connectivity).
            EnumEncodedValue<PredictedSurface> surfEnc2 = predictedSurfaceEnc;
            int rseed = -1;
            for (int e = 0; e < ag0.edgeCount(); e++)
                if ((ag0.osmWayId(e) & 0xFFFFFFFFL) == 26216742L) { rseed = ag0.nodeA(e); break; }
            System.out.println("\n--- REACHABILITY from Morsfjärdintie (26216742) over the analysis graph ---");
            if (rseed >= 0) {
                boolean[] vis = new boolean[ag0.nodeCount()];
                int[] q = new int[ag0.nodeCount()];
                int head = 0, tail = 0; vis[rseed] = true; q[tail++] = rseed;
                int anchorEdges = 0, targetEdges = 0, connEdges = 0, gridEdges = 0;
                double anchorLen = 0, asphaltAnchorLen = 0, targetLen = 0;
                while (head < tail) {
                    int u = q[head++];
                    for (int i = ag0.adjBegin(u); i < ag0.adjEnd(u); i++) {
                        int e = ag0.adjEdge(i);
                        if (ag0.isRemoved(e)) continue;
                        int v = ag0.other(e, u);
                        if (!vis[v]) { vis[v] = true; q[tail++] = v; }
                    }
                }
                boolean[] edgeSeen = new boolean[ag0.edgeCount()];
                for (int e = 0; e < ag0.edgeCount(); e++) {
                    if (ag0.isRemoved(e) || edgeSeen[e]) continue;
                    if (!(vis[ag0.nodeA(e)] || vis[ag0.nodeB(e)])) continue;
                    edgeSeen[e] = true;
                    EdgeRole r = ag0.role(e);
                    if (vd.gridEdge[e]) gridEdges++;
                    if (r == EdgeRole.ANCHOR) {
                        anchorEdges++; anchorLen += ag0.lengthM(e);
                        if (graph.getEdgeIteratorState(ag0.ghEdgeId(e), ag0.nodeB(e)).get(surfEnc2)
                                == PredictedSurface.ASPHALT) asphaltAnchorLen += ag0.lengthM(e);
                    } else if (r == EdgeRole.TARGET) { targetEdges++; targetLen += ag0.lengthM(e); }
                    else if (r == EdgeRole.CONNECTOR) connEdges++;
                }
                int nodes = 0; for (boolean b : vis) if (b) nodes++;
                System.out.printf("  component: %d nodes | TARGET %d (%.0fm) ANCHOR %d (%.0fm, asphalt %.0fm) "
                        + "CONNECTOR %d | GRID edges reached = %d%n",
                        nodes, targetEdges, targetLen, anchorEdges, anchorLen, asphaltAnchorLen,
                        connEdges, gridEdges);
                // Boundary IGNORED base edges (only worth dumping if the component is small/severed).
                if (nodes < 8000) {
                    java.util.Map<Long, String> sever = new java.util.LinkedHashMap<>();
                    com.graphhopper.util.EdgeExplorer ex = graph.createEdgeExplorer();
                    for (int u = 0; u < ag0.nodeCount(); u++) {
                        if (!vis[u]) continue;
                        com.graphhopper.util.EdgeIterator it = ex.setBaseNode(u);
                        while (it.next()) {
                            if (classifier.classify(it) != EdgeRole.IGNORED) continue;
                            long w = it.get(wayIdEnc) & 0xFFFFFFFFL;
                            sever.putIfAbsent(w, String.format("way=%d '%s' len=%.0fm gs=%s surf=%s rc=%s bike=%s",
                                    w, it.getName(), it.getDistance(), it.get(gravelEnc),
                                    it.get(surfEnc2), it.get(roadClassEnc),
                                    bikeAccessEnc == null ? "n/a" : (it.get(bikeAccessEnc) || it.getReverse(bikeAccessEnc))));
                        }
                    }
                    System.out.println("  IGNORED base edges on the component boundary (severing candidates): " + sever.size());
                    int shown = 0;
                    for (String s : sever.values()) { System.out.println("      " + s); if (++shown >= 40) break; }
                } else {
                    System.out.println("  component is LARGE -> it DOES reach the main network (not severed); "
                            + "look at case-A/floor/2EC, not connectivity.");
                }
            }

            // --- PROVE TEST 1: two independent paths from each road to ANCHOR Hirsalantie ---
            proveTest1(graph, ag0, vd, 26216737L, wayIdEnc, bikeAccessEnc);
            proveTest1(graph, ag0, vd, 26216742L, wayIdEnc, bikeAccessEnc);

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

    /**
     * Prove (or disprove) Test 1 for a road: are there TWO node-independent paths from the road to
     * ANCHOR "Hirsalantie", (a) in the analysis graph the algo uses, and (b) in the full bike graph
     * (reality)? Also report whether Hirsalantie counts as GRID. This pinpoints where case A loses
     * the connection the user observes.
     */
    private void proveTest1(BaseGraph graph, AnalysisGraph ag, GravelNetworkFilter.Verdict vd,
                            long roadId, IntEncodedValue wayIdEnc, BooleanEncodedValue bikeAccessEnc) {
        final int N = graph.getNodes();
        System.out.println("\n--- PROVE TEST 1 for way " + roadId + " -> ANCHOR Hirsalantie ---");
        java.util.Set<Integer> target = new java.util.HashSet<>();
        int hAnchor = 0, hTarget = 0, hGrid = 0;
        for (int e = 0; e < ag.edgeCount(); e++) {
            String nm = ag.name(e);
            if (nm == null || !nm.toLowerCase().contains("hirsalantie")) continue;
            if (ag.role(e) == EdgeRole.ANCHOR) hAnchor++; else if (ag.role(e) == EdgeRole.TARGET) hTarget++;
            if (vd.gridEdge[e]) hGrid++;
            target.add(ag.nodeA(e)); target.add(ag.nodeB(e));
        }
        System.out.printf("  Hirsalantie in analysis graph: ANCHOR=%d TARGET=%d | GRID(vd.gridEdge)=%d "
                + "(0 => floor excluded it from the grid) | target nodes=%d%n",
                hAnchor, hTarget, hGrid, target.size());
        // Sources = ALL nodes of the road (both ends and everything between).
        java.util.Set<Integer> src = new java.util.HashSet<>();
        for (int e = 0; e < ag.edgeCount(); e++) if ((ag.osmWayId(e) & 0xFFFFFFFFL) == roadId) { src.add(ag.nodeA(e)); src.add(ag.nodeB(e)); }
        if (src.isEmpty() || target.isEmpty()) { System.out.println("  cannot run (road/Hirsalantie missing)"); return; }

        com.graphhopper.storage.NodeAccess na = graph.getNodeAccess();
        int anyRoadNode = src.iterator().next();
        System.out.printf("  COORD road node %d = %.6f,%.6f | Hirsalantie junctions: 3475545=%.6f,%.6f  3475810=%.6f,%.6f%n",
                anyRoadNode, na.getLat(anyRoadNode), na.getLon(anyRoadNode),
                na.getLat(3475545), na.getLon(3475545), na.getLat(3475810), na.getLon(3475810));

        int[] pe = new int[N], pn = new int[N];
        // (a) analysis graph: two EDGE-disjoint paths road -> Hirsalantie.
        java.util.Set<Integer> blkE = new java.util.HashSet<>();
        int t1 = bfsMulti(ag, null, bikeAccessEnc, wayIdEnc, src, target, blkE, pe, pn);
        if (t1 < 0) { System.out.println("  ANALYSIS: NO path road->Hirsalantie (severed in analysis graph!)"); }
        else {
            System.out.println("  ANALYSIS path #1 -> Hirsalantie node " + t1 + " : " + wayChain(ag, null, wayIdEnc, graph, pe, pn, t1));
            for (int v = t1; pe[v] != -1; v = pn[v]) blkE.add(pe[v]);   // block path-1 edges (edge-disjoint)
            int t2 = bfsMulti(ag, null, bikeAccessEnc, wayIdEnc, src, target, blkE, pe, pn);
            if (t2 < 0) System.out.println("  ANALYSIS path #2: NONE -> only ONE independent route -> case A treats it as a dead-end branch");
            else System.out.println("  ANALYSIS path #2 -> Hirsalantie node " + t2 + " : " + wayChain(ag, null, wayIdEnc, graph, pe, pn, t2)
                    + "\n  => TWO edge-disjoint routes to Hirsalantie EXIST in the analysis graph (Test 1 should pass)");
        }
        // (b) full bike graph (reality, like routing): two edge-disjoint paths.
        blkE.clear();
        int b1 = bfsMulti(null, graph, bikeAccessEnc, wayIdEnc, src, target, blkE, pe, pn);
        if (b1 < 0) System.out.println("  BASE(reality): NO bike path road->Hirsalantie?!");
        else {
            for (int v = b1; pe[v] != -1; v = pn[v]) blkE.add(pe[v]);
            int b2 = bfsMulti(null, graph, bikeAccessEnc, wayIdEnc, src, target, blkE, pe, pn);
            System.out.println("  BASE(reality): path#1 -> Hirsalantie node " + b1 + " ; path#2 -> "
                    + (b2 < 0 ? "NONE (only one route in reality too)" : "node " + b2 + " => TWO edge-disjoint routes exist in reality"));
        }
    }

    /** Multi-source BFS over EITHER the analysis graph (ag != null) or the base bike graph (graph != null),
     *  road sources -> any target node, avoiding blocked edges. Fills pe/pn. Returns the reached target. */
    private int bfsMulti(AnalysisGraph ag, BaseGraph graph, BooleanEncodedValue bikeAccessEnc, IntEncodedValue wayIdEnc,
                         java.util.Set<Integer> sources, java.util.Set<Integer> targets,
                         java.util.Set<Integer> blockedEdges, int[] pe, int[] pn) {
        java.util.Arrays.fill(pe, -2);
        java.util.ArrayDeque<Integer> q = new java.util.ArrayDeque<>();
        for (int s : sources) { if (pe[s] == -2) { pe[s] = -1; pn[s] = -1; q.add(s); } }
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
                    if (targets.contains(v)) return v;
                    q.add(v);
                }
            } else {
                com.graphhopper.util.EdgeIterator it = ex.setBaseNode(u);
                while (it.next()) {
                    if (bikeAccessEnc != null && !(it.get(bikeAccessEnc) || it.getReverse(bikeAccessEnc))) continue;
                    if (blockedEdges.contains(it.getEdge())) continue;
                    int v = it.getAdjNode();
                    if (pe[v] != -2) continue;
                    pe[v] = it.getEdge(); pn[v] = u;
                    if (targets.contains(v)) return v;
                    q.add(v);
                }
            }
        }
        return -1;
    }

    /** Distinct way ids + roles along the path to target (analysis graph if ag != null, else base). */
    private String wayChain(AnalysisGraph ag, Object unused, IntEncodedValue wayIdEnc, BaseGraph graph,
                            int[] pe, int[] pn, int target) {
        java.util.List<String> chain = new java.util.ArrayList<>();
        long last = -1;
        for (int v = target; pe[v] != -1 && pe[v] != -2; v = pn[v]) {
            int e = pe[v];
            long w; String role;
            if (ag != null) { w = ag.osmWayId(e) & 0xFFFFFFFFL; role = ag.role(e).toString(); }
            else { com.graphhopper.util.EdgeIteratorState es = graph.getEdgeIteratorState(e, v);
                   w = es.get(wayIdEnc) & 0xFFFFFFFFL; role = "base"; }
            if (w != last) { chain.add(w + "(" + role + ")"); last = w; }
            if (chain.size() >= 16) { chain.add("..."); break; }
        }
        return String.join(" <- ", chain);
    }

    private int bfsAG(AnalysisGraph ag, int src, java.util.Set<Integer> targets,
                      java.util.Set<Integer> blockedNodes, java.util.Set<Integer> blockedEdges,
                      int[] predEdge, int[] predNode) {
        java.util.Arrays.fill(predEdge, -2);
        java.util.ArrayDeque<Integer> q = new java.util.ArrayDeque<>();
        predEdge[src] = -1; predNode[src] = -1; q.add(src);
        while (!q.isEmpty()) {
            int u = q.poll();
            for (int i = ag.adjBegin(u); i < ag.adjEnd(u); i++) {
                int e = ag.adjEdge(i);
                if (ag.isRemoved(e) || blockedEdges.contains(e)) continue;
                int v = ag.other(e, u);
                if (predEdge[v] != -2 || blockedNodes.contains(v)) continue;
                predEdge[v] = e; predNode[v] = u;
                if (targets.contains(v)) return v;
                q.add(v);
            }
        }
        return -1;
    }

    private int bfsBase(BaseGraph graph, int src, java.util.Set<Integer> targets,
                        java.util.Set<Integer> blockedNodes, BooleanEncodedValue bikeAccessEnc,
                        int[] predEdge, int[] predNode, IntEncodedValue wayIdEnc) {
        java.util.Arrays.fill(predEdge, -2);
        com.graphhopper.util.EdgeExplorer ex = graph.createEdgeExplorer();
        java.util.ArrayDeque<Integer> q = new java.util.ArrayDeque<>();
        predEdge[src] = -1; predNode[src] = -1; q.add(src);
        while (!q.isEmpty()) {
            int u = q.poll();
            com.graphhopper.util.EdgeIterator it = ex.setBaseNode(u);
            while (it.next()) {
                if (bikeAccessEnc != null && !(it.get(bikeAccessEnc) || it.getReverse(bikeAccessEnc))) continue;
                int v = it.getAdjNode();
                if (predEdge[v] != -2 || blockedNodes.contains(v)) continue;
                predEdge[v] = it.getEdge(); predNode[v] = u;
                if (targets.contains(v)) return v;
                q.add(v);
            }
        }
        return -1;
    }

    /** Distinct OSM way ids + roles along the analysis-graph path from src to target (capped). */
    private String wayChainAG(AnalysisGraph ag, int[] predEdge, int[] predNode, int target, int src) {
        java.util.List<String> chain = new java.util.ArrayList<>();
        long last = -1;
        for (int v = target; v != src && v != -1; v = predNode[v]) {
            int e = predEdge[v];
            if (e < 0) break;
            long w = ag.osmWayId(e) & 0xFFFFFFFFL;
            if (w != last) { chain.add(w + "(" + ag.role(e) + ")"); last = w; }
            if (chain.size() >= 14) { chain.add("..."); break; }
        }
        return String.join(" <- ", chain);
    }

    private static int ufFind(int[] p, int x) { while (p[x] != x) { p[x] = p[p[x]]; x = p[x]; } return x; }

    private static void ufUnion(int[] p, int a, int b) {
        int ra = ufFind(p, a), rb = ufFind(p, b);
        if (ra != rb) p[ra] = rb;
    }

    /** Build a fresh working graph, run the network filter, return per-way {survived, removed}. */
    private Map<Long, int[]> runPipeline(double threshold) {
        AnalysisGraph ag = GravelSegmentTool.buildAnalysisGraph(graph, em, cfg, new GravelSegmentTool.Stats());
        Map<Integer, Long> tracked = new LinkedHashMap<>();
        for (int e = 0; e < ag.edgeCount(); e++) {
            long wid = ag.osmWayId(e) & 0xFFFFFFFFL;
            if (trackWays.contains(wid) && ag.role(e) == EdgeRole.TARGET) tracked.put(e, wid);
        }
        new GravelNetworkFilter(ag, threshold, cfg.gridMinComponentCoreLenM).filter();
        Map<Long, int[]> stage = new LinkedHashMap<>();
        for (long id : trackWays) stage.put(id, new int[2]);
        for (Map.Entry<Integer, Long> en : tracked.entrySet()) {
            int[] s = stage.get(en.getValue());
            if (ag.isRemoved(en.getKey())) s[1]++; else s[0]++;
        }
        return stage;
    }
}
