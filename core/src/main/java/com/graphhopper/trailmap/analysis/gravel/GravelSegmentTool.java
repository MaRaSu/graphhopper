/*
 * Trailmap - Gravel Segment Analysis
 *
 * CLI entry point and orchestration for the well-connected gravel-network extractor.
 * Offline batch job: build a dedicated analysis graph from a PBF, run Phases B–F in
 * process, write two JSON artifacts, exit. See docs/gravel_segments_design.md §2, §16.
 *
 * Run (no shell pipes/redirects needed; the tool writes files directly):
 *   java -cp <classpath> com.graphhopper.trailmap.analysis.gravel.GravelSegmentTool \
 *        datareader.file=finland.osm.pbf graph.location=gravel-analysis-gh \
 *        gravel.size_threshold_m=300 out.dir=gravel-out
 */
package com.graphhopper.trailmap.analysis.gravel;

import com.graphhopper.GraphHopperConfig;
import com.graphhopper.config.Profile;
import com.graphhopper.json.Statement;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.ev.IntEncodedValue;
import com.graphhopper.routing.ev.RoadClass;
import com.graphhopper.trailmap.shared.GravelScale;
import com.graphhopper.trailmap.shared.PredictedHighway;
import com.graphhopper.trailmap.shared.PredictedSurface;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.trailmap.TrailmapGraphHopper;
import com.graphhopper.trailmap.analysis.gravel.graph.AnalysisGraph;
import com.graphhopper.trailmap.analysis.gravel.group.LogicalRoadGrouper;
import com.graphhopper.trailmap.analysis.gravel.output.GravelOutputWriter;
import com.graphhopper.trailmap.analysis.gravel.prune.ConnectorOutputClassifier;
import com.graphhopper.trailmap.analysis.gravel.prune.ConnectorResolver;
import com.graphhopper.trailmap.analysis.gravel.prune.GravelNetworkFilter;
import com.graphhopper.trailmap.analysis.gravel.role.EdgeRole;
import com.graphhopper.trailmap.analysis.gravel.role.EdgeRoleClassifier;
import com.graphhopper.routing.util.AllEdgesIterator;
import com.graphhopper.trailmap.shared.TrailmapImportRegistry;
import com.graphhopper.util.CustomModel;
import com.graphhopper.util.PMap;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.util.Arrays;
import java.util.Collections;

/**
 * Orchestrates the gravel-network extraction. All tuning lives in {@link GravelAnalysisConfig};
 * scalar/boolean levers can be overridden on the command line, the enum-set levers are edited
 * on the config object.
 */
public class GravelSegmentTool {

    private static final Logger LOGGER = LoggerFactory.getLogger(GravelSegmentTool.class);

    /**
     * Encoded values the analysis reads (§3.2 / §4.2). {@code osm_way_id} is added here only;
     * {@code predicted_highway} is read only to emit per-way styling attributes (Phase F sugar).
     */
    private static final String ENCODED_VALUES =
            "road_class, surface, track_type, gravel_scale, mtb_scale, predicted_highway, "
            + "predicted_surface, osm_way_id, road_name_hash, bike_access";

    /** KVStorage key under which OSMReader stores the street ref (Parameters.Details.STREET_REF). */
    private static final String STREET_REF = "street_ref";

    /** Run statistics, surfaced for logging and tests. */
    public static class Stats {
        public int nodes, workingEdges;
        public long targetEdges, anchorEdges, ignoredEdges, connectorEdges;
        public int connectorChainsAdmitted, connectorEdgesAdmitted, connectorEdgesRemoved;
        public int prunedNotNetwork, logicalRoads, qualifyingWays;
        public File waysFile, roadsFile, attrsFile, segmentsFile;
        /** way id → {gravel_scale, predicted_highway, predicted_surface} for TARGET edges (styling sugar). */
        public final java.util.Map<Long, String[]> wayAttrs = new java.util.HashMap<>();
        /** way id → "through" (case A: 2-vertex-connected to the road grid) or "island" (case B:
         *  standalone cluster rescued by τ). A way with both is labelled "through". */
        public final java.util.Map<Long, String> wayConnectivity = new java.util.HashMap<>();
        /** Ways carrying both a through-route edge and an island edge (labelled "through"). */
        public int mixedConnectivityWays;
    }

    public static void main(String[] args) throws Exception {
        PMap pm = PMap.read(args);

        String pbf = pm.getString("datareader.file", "");
        String graphLocation = pm.getString("graph.location", "gravel-analysis-gh");
        String outDir = pm.getString("out.dir", "gravel-out");
        if (pbf.isEmpty())
            throw new IllegalArgumentException("datareader.file=<path to OSM PBF> is required");

        GravelAnalysisConfig cfg = new GravelAnalysisConfig();
        cfg.gravelSizeThresholdM = pm.getDouble("gravel.size_threshold_m", Double.NaN);
        cfg.islandSizeThresholdM = pm.getDouble("gravel.island_threshold_m", Double.NaN);
        cfg.requireSameRef = pm.getBool("gravel.require_same_ref", cfg.requireSameRef);
        cfg.emitUnnamed = pm.getBool("gravel.emit_unnamed", cfg.emitUnnamed);
        cfg.enableConnectors = pm.getBool("gravel.enable_connectors", cfg.enableConnectors);
        cfg.connectorCostBudget = pm.getDouble("gravel.connector_cost_budget", cfg.connectorCostBudget);
        cfg.validate();

        run(cfg, pbf, graphLocation, new File(outDir));
    }

    /**
     * Run the full extraction (Phases A–F) and write the two JSON artifacts. Returns
     * statistics. The config's {@link GravelAnalysisConfig#validate()} should be called first.
     */
    public static Stats run(GravelAnalysisConfig cfg, String pbf, String graphLocation, File outDir)
            throws java.io.IOException {
        Stats st = new Stats();
        TrailmapGraphHopper hopper = buildAndImport(pbf, graphLocation);
        try {
            BaseGraph graph = hopper.getBaseGraph();
            EncodingManager em = hopper.getEncodingManager();

            // Phase A→B: project the BaseGraph to the working AnalysisGraph (TARGET ∪ ANCHOR).
            AnalysisGraph ag = buildAnalysisGraph(graph, em, cfg, st);
            st.nodes = ag.nodeCount();
            st.workingEdges = ag.edgeCount();
            LOGGER.info("Analysis graph: {} nodes, {} working edges (TARGET ∪ ANCHOR)",
                    ag.nodeCount(), ag.edgeCount());

            // STEP 3 (toggle): resolve bounded connector chains, so admitted connectors join the
            // connectivity the filter sees. No-op when disabled.
            if (cfg.enableConnectors) {
                ConnectorResolver.Result cr =
                        new ConnectorResolver(ag, cfg.connectorCostBudget).resolve();
                st.connectorChainsAdmitted = cr.corridors;
                st.connectorEdgesAdmitted = cr.admittedEdges;
                st.connectorEdgesRemoved = cr.removedEdges;
                LOGGER.info("Step 3 connectors (budget {}): admitted {} corridors ({} edges), "
                        + "dropped {} edges", cfg.connectorCostBudget, cr.corridors,
                        cr.admittedEdges, cr.removedEdges);
            }

            // Steps 1 (+2): keep gravel 2-vertex-connected to the road grid; rescue standalone
            // clusters by τ when Step 2 is on (τ = ∞ disables it).
            double tau = cfg.enableStandaloneRescue ? cfg.effectiveIslandThresholdM()
                    : Double.POSITIVE_INFINITY;
            GravelNetworkFilter filter = new GravelNetworkFilter(ag, tau, cfg.gridMinComponentCoreLenM);
            GravelNetworkFilter.Verdict verdict = filter.analyze();
            st.prunedNotNetwork = filter.filter(verdict);
            // Output-correctness: drop admitted connectors that serve no RETAINED gravel (road-to-road
            // connectors, or connectors orphaned when their gravel was pruned) so they cannot leak into
            // qualifying output. Runs after the filter; cannot disconnect kept gravel.
            if (cfg.enableConnectors) {
                int orphans = ConnectorResolver.removeConnectorsNotServingGravel(ag);
                st.connectorEdgesRemoved += orphans;
                LOGGER.info("Step 3 connectors: dropped {} that serve no retained gravel", orphans);
            }
            // Output-only: split admitted connectors into needed (bridge) vs redundant. Never mutates
            // connectivity — it runs on the already-filtered graph.
            boolean[] neededConnector = cfg.enableConnectors && cfg.connectorMarkRedundant
                    ? ConnectorOutputClassifier.classify(ag) : null;
            collectConnectivity(ag, verdict, neededConnector, st);
            LOGGER.info("Network filter (Step 2 {}): dropped {} TARGET edges; connectivity tagged "
                    + "({} ways carry both through & island)",
                    cfg.enableStandaloneRescue ? "on, τ=" + cfg.effectiveIslandThresholdM() + "m" : "off",
                    st.prunedNotNetwork, st.mixedConnectivityWays);

            // Phase E: contiguous same-name logical-road grouping.
            LogicalRoadGrouper.Result result =
                    new LogicalRoadGrouper(ag, cfg.requireSameRef, cfg.emitUnnamed).group();
            st.logicalRoads = result.roads.size();
            st.qualifyingWays = result.allWayIds.size();
            LOGGER.info("Phase E: {} logical roads over {} qualifying way IDs",
                    st.logicalRoads, st.qualifyingWays);

            // Phase F: emit JSON.
            if (!outDir.exists() && !outDir.mkdirs())
                throw new IllegalStateException("Could not create output directory: " + outDir.getAbsolutePath());
            GravelOutputWriter writer = new GravelOutputWriter();
            st.waysFile = new File(outDir, "qualifying_ways.json");
            st.roadsFile = new File(outDir, "logical_roads.json");
            st.attrsFile = new File(outDir, "way_attributes.json");
            st.segmentsFile = new File(outDir, "retained_segments.json");
            writer.writeQualifyingWays(st.waysFile, result);
            writer.writeLogicalRoads(st.roadsFile, result);
            writer.writeWayAttributes(st.attrsFile, result.allWayIds, st.wayAttrs, st.wayConnectivity);
            writer.writeRetainedSegments(st.segmentsFile, collectRetainedSegments(graph, ag));
            LOGGER.info("Wrote {}, {}, {} and {}", st.waysFile.getAbsolutePath(),
                    st.roadsFile.getAbsolutePath(), st.attrsFile.getAbsolutePath(),
                    st.segmentsFile.getAbsolutePath());
            return st;
        } finally {
            hopper.close();
        }
    }

    /** Bootstrap a dedicated, routing-decoupled analysis graph and import the PBF. */
    static TrailmapGraphHopper buildAndImport(String pbf, String graphLocation) {
        GraphHopperConfig config = new GraphHopperConfig();
        config.putObject("datareader.file", pbf);
        config.putObject("graph.location", graphLocation);
        config.putObject("graph.encoded_values", ENCODED_VALUES);
        // GraphHopper requires this be set explicitly (empty = exclude nothing). We keep all
        // highways so service/track dead-ends are present for Phase C to classify and prune.
        config.putObject("import.osm.ignored_highways", "");
        // No CH/LM (we never route), no elevation (irrelevant to connectivity), keep every
        // component (we do our own size accounting). A constant-speed dummy profile satisfies
        // init()'s ≥1-profile rule and the per-profile weighting needed by subnetwork prep;
        // it is never used for routing.
        config.setProfiles(Collections.singletonList(dummyProfile()));
        config.putObject("prepare.min_network_size", 0);

        TrailmapGraphHopper hopper = new TrailmapGraphHopper();
        hopper.setImportRegistry(new TrailmapImportRegistry());
        hopper.init(config);
        try {
            hopper.importOrLoad();
        } catch (RuntimeException e) {
            if (indicatesWayIdOverflow(e))
                throw new IllegalStateException(
                        "OSM way ID exceeds the 31-bit osm_way_id encoded value during import. "
                        + "Today's OSM way IDs fit in 31 bits; if this fires, osm_way_id must be widened "
                        + "(e.g. a Trailmap 40-bit IntEncodedValue variant) before re-importing.", e);
            throw e;
        }
        return hopper;
    }

    /** A trivial constant-speed custom profile: enough to satisfy init() and weighting prep. */
    private static Profile dummyProfile() {
        CustomModel cm = new CustomModel();
        cm.addToSpeed(Statement.If("true", Statement.Op.LIMIT, "60"));
        Profile p = new Profile("gravel_analysis");
        p.setCustomModel(cm);
        return p;
    }

    private static boolean indicatesWayIdOverflow(Throwable e) {
        for (Throwable t = e; t != null; t = t.getCause()) {
            String m = t.getMessage();
            if (t instanceof ArithmeticException && m != null && m.contains("overflow"))
                return true;
            if (m != null && m.contains("osm_way_id"))
                return true;
        }
        return false;
    }

    /**
     * Collect, per OSM way, the retained gravel edges as their two junction (tower-node)
     * coordinates [latA, lonA, latB, lonB], rounded to 7 decimals (= OSM precision). A way with
     * only some segments retained appears with just those segments — partial-way output. Downstream
     * matches these coords to OSM node ids / cuts the way's linestring (GraphHopper drops OSM node
     * ids at import, so coordinates are the interchange).
     */
    private static java.util.Map<Long, java.util.List<double[]>> collectRetainedSegments(
            BaseGraph graph, AnalysisGraph ag) {
        java.util.Map<Long, java.util.List<double[]>> segs =
                new java.util.TreeMap<>();
        for (int e = 0; e < ag.edgeCount(); e++) {
            if (ag.isRemoved(e)
                    || (ag.role(e) != EdgeRole.TARGET && ag.role(e) != EdgeRole.CONNECTOR)) continue;
            com.graphhopper.util.PointList pts = graph.getEdgeIteratorState(ag.ghEdgeId(e), ag.nodeB(e))
                    .fetchWayGeometry(com.graphhopper.util.FetchMode.TOWER_ONLY);
            int last = pts.size() - 1;
            double[] seg = {round7(pts.getLat(0)), round7(pts.getLon(0)),
                    round7(pts.getLat(last)), round7(pts.getLon(last))};
            segs.computeIfAbsent((long) ag.osmWayId(e), k -> new java.util.ArrayList<>()).add(seg);
        }
        return segs;
    }

    private static double round7(double v) {
        return Math.round(v * 1e7) / 1e7;
    }

    /**
     * Tag each retained OSM way as {@code "through"} (has a case-A edge: 2-vertex-connected to the
     * road grid — a through-route you can ride into and out of) or {@code "island"} (only case-B
     * edges: a standalone gravel cluster rescued by τ — a loop reached from a single point). A way
     * carrying both kinds is "through" (it does reach the grid); such ways are counted separately.
     */
    private static void collectConnectivity(AnalysisGraph ag, GravelNetworkFilter.Verdict v,
                                            boolean[] neededConnector, Stats st) {
        java.util.Set<Long> hasThrough = new java.util.HashSet<>();
        java.util.Set<Long> hasIsland = new java.util.HashSet<>();
        java.util.Set<Long> hasNeededConn = new java.util.HashSet<>();
        java.util.Set<Long> hasRedundantConn = new java.util.HashSet<>();
        for (int e = 0; e < ag.edgeCount(); e++) {
            if (ag.isRemoved(e)) continue;
            long wid = ag.osmWayId(e);
            if (ag.role(e) == EdgeRole.TARGET) {
                if (v.keptA[e]) hasThrough.add(wid);
                else if (v.keptB[e]) hasIsland.add(wid);
            } else if (ag.role(e) == EdgeRole.CONNECTOR) {
                // surviving CONNECTOR = admitted by the resolver; needed (bridge) vs redundant (loop).
                if (neededConnector == null || neededConnector[e]) hasNeededConn.add(wid);
                else hasRedundantConn.add(wid);
            }
        }
        // Precedence (lowest first; later puts win): redundant connector < connector < island < through.
        for (Long wid : hasRedundantConn) st.wayConnectivity.put(wid, "connector_redundant");
        for (Long wid : hasNeededConn) st.wayConnectivity.put(wid, "connector");
        for (Long wid : hasIsland) {
            if (hasThrough.contains(wid)) st.mixedConnectivityWays++;
            else st.wayConnectivity.put(wid, "island");
        }
        for (Long wid : hasThrough) st.wayConnectivity.put(wid, "through");
    }

    /** Phase A→B: single pass over all edges, classify, keep TARGET ∪ ANCHOR. */
    static AnalysisGraph buildAnalysisGraph(BaseGraph graph, EncodingManager em,
                                            GravelAnalysisConfig cfg, Stats st) {
        EdgeRoleClassifier classifier = EdgeRoleClassifier.create(cfg, em);
        IntEncodedValue osmWayIdEnc = em.getIntEncodedValue("osm_way_id");
        IntEncodedValue roadNameHashEnc = em.getIntEncodedValue("road_name_hash");
        EnumEncodedValue<RoadClass> roadClassEnc =
                em.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<GravelScale> gravelScaleEnc =
                em.getEnumEncodedValue(GravelScale.KEY, GravelScale.class);
        EnumEncodedValue<com.graphhopper.trailmap.shared.MtbScale> mtbScaleEnc =
                em.getEnumEncodedValue(com.graphhopper.trailmap.shared.MtbScale.KEY,
                        com.graphhopper.trailmap.shared.MtbScale.class);
        EnumEncodedValue<PredictedHighway> predictedHighwayEnc =
                em.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
        EnumEncodedValue<PredictedSurface> predictedSurfaceEnc =
                em.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class);

        AnalysisGraph.Builder builder = new AnalysisGraph.Builder();
        java.util.List<Double> connectorWeights = new java.util.ArrayList<>();
        AllEdgesIterator iter = graph.getAllEdges();
        while (iter.next()) {
            EdgeRole role = classifier.classify(iter);
            boolean backboneAnchor = role == EdgeRole.ANCHOR
                    && cfg.backboneRoadClasses.contains(iter.get(roadClassEnc));
            // STEP 3 candidate = a near-Target gravel way (gravel_scale just below Target, by band),
            // that is neither Target nor real-road backbone. The connector pre-pass resolves which
            // weighted corridors are admitted into connectivity; the rest are dropped.
            if (cfg.enableConnectors && role != EdgeRole.TARGET && !backboneAnchor
                    && classifier.isConnectorCandidate(iter))
                role = EdgeRole.CONNECTOR;
            if (role == EdgeRole.IGNORED) {
                st.ignoredEdges++;
                continue;
            }

            double connWeight = 0;
            if (role == EdgeRole.TARGET) {
                st.targetEdges++;
                // Capture styling attributes once per way (a way's edges share its OSM tags).
                st.wayAttrs.putIfAbsent((long) iter.get(osmWayIdEnc), new String[]{
                        iter.get(gravelScaleEnc).toString(), iter.get(predictedHighwayEnc).toString(),
                        iter.get(predictedSurfaceEnc).toString()});
            } else if (role == EdgeRole.CONNECTOR) {
                st.connectorEdges++;
                connWeight = cfg.connectorWeight(iter.get(gravelScaleEnc), iter.get(mtbScaleEnc));
                // Connectors are emitted too (distinct connectivity tag) — capture their attrs.
                st.wayAttrs.putIfAbsent((long) iter.get(osmWayIdEnc), new String[]{
                        iter.get(gravelScaleEnc).toString(), iter.get(predictedHighwayEnc).toString(),
                        iter.get(predictedSurfaceEnc).toString()});
            } else {
                st.anchorEdges++;
            }

            // Real-road backbone: an ANCHOR edge of a drivable/cycleway class anchors a non-dead-end.
            boolean backbone = backboneAnchor;
            Object refVal = iter.getValue(STREET_REF);
            builder.addEdge(iter.getEdge(), iter.getBaseNode(), iter.getAdjNode(), role,
                    iter.getDistance(), iter.get(osmWayIdEnc), iter.getName(),
                    refVal == null ? null : refVal.toString(), iter.get(roadNameHashEnc), backbone);
            connectorWeights.add(connWeight);
        }
        LOGGER.info("Role classification: {} TARGET, {} ANCHOR, {} IGNORED, {} CONNECTOR candidate",
                st.targetEdges, st.anchorEdges, st.ignoredEdges, st.connectorEdges);
        AnalysisGraph ag = builder.build(graph.getNodes());
        for (int i = 0; i < connectorWeights.size(); i++)
            ag.setConnectorWeight(i, connectorWeights.get(i));
        return ag;
    }

    private GravelSegmentTool() {
    }
}
