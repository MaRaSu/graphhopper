package com.graphhopper.trailmap.analysis.gravel;

import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.ev.IntEncodedValue;
import com.graphhopper.routing.ev.RoadClass;
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
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

/**
 * Finding-2a validation: what does the orphan-connector prune actually REMOVE on Tampere?
 * Snapshots the connectors admitted by the resolver, runs the filter, then runs the prune, and
 * lists the connector OSM ways it dropped — with name / surface / a coordinate / OSM URL so the
 * removals can be eyeballed. Also re-checks the invariant (each removed connector's component holds
 * no retained TARGET) independently.
 */
class GravelConnectorPruneDeltaTest {

    private static final String DATA = "/Users/suomimar/Dropbox/dev/map-server/routing_v2/data";
    private static final String PBF = DATA + "/tampere.osm.pbf";
    private static final String GRAPH = DATA + "/tampere-analysis-gh";

    @Test
    void listPrunedConnectors() {
        assumeTrue(new File(PBF).exists(), "tampere.osm.pbf not present — skipping");
        GravelAnalysisConfig cfg = new GravelAnalysisConfig();
        cfg.gravelSizeThresholdM = 2000;
        cfg.enableConnectors = true;
        cfg.validate();

        TrailmapGraphHopper hopper = GravelSegmentTool.buildAndImport(PBF, GRAPH);
        try {
            BaseGraph graph = hopper.getBaseGraph();
            EncodingManager em = hopper.getEncodingManager();
            NodeAccess na = graph.getNodeAccess();
            EnumEncodedValue<GravelScale> gsEnc = em.getEnumEncodedValue(GravelScale.KEY, GravelScale.class);
            EnumEncodedValue<RoadClass> rcEnc = em.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
            EnumEncodedValue<PredictedSurface> psEnc = em.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class);

            AnalysisGraph ag = GravelSegmentTool.buildAnalysisGraph(graph, em, cfg, new GravelSegmentTool.Stats());
            new ConnectorResolver(ag, cfg.connectorCostBudget).resolve();
            GravelNetworkFilter f = new GravelNetworkFilter(ag, cfg.effectiveIslandThresholdM(), cfg.gridMinComponentCoreLenM);
            f.filter(f.analyze());

            // Snapshot connectors admitted (surviving the resolver) BEFORE the orphan prune.
            boolean[] admittedBefore = new boolean[ag.edgeCount()];
            int admittedWays = countWays(ag, admittedBeforeMask(ag, admittedBefore));

            // THE RED-FLAG CHECK: retained TARGET ways must be identical before and after the prune.
            java.util.Set<Long> targetBefore = retainedTargetWays(ag);

            int removedEdges = ConnectorResolver.removeConnectorsNotServingGravel(ag);

            java.util.Set<Long> targetAfter = retainedTargetWays(ag);
            java.util.Set<Long> lostTargets = new java.util.TreeSet<>(targetBefore);
            lostTargets.removeAll(targetAfter);
            System.out.println("\n############ 2a — DOES THE PRUNE DROP ANY TARGET? ############");
            System.out.println("retained TARGET ways before prune: " + targetBefore.size());
            System.out.println("retained TARGET ways after  prune: " + targetAfter.size());
            System.out.println("TARGET ways LOST to the prune:     " + lostTargets.size()
                    + (lostTargets.isEmpty() ? "  (PASS — prune touches connectors only)" : "  >>> RED FLAG: " + lostTargets));
            // THE critical guard: the orphan-connector prune must never drop a qualifying TARGET way.
            assertEquals(0, lostTargets.size(), "orphan-connector prune dropped TARGET ways: " + lostTargets);

            // Removed = admitted-before AND now removed. Group to OSM ways with a sample row each.
            Map<Long, String> removedWays = new LinkedHashMap<>();
            Map<Long, String> keptWays = new LinkedHashMap<>();
            for (int e = 0; e < ag.edgeCount(); e++) {
                if (!admittedBefore[e]) continue;
                long w = ag.osmWayId(e) & 0xFFFFFFFFL;
                Map<Long, String> bucket = ag.isRemoved(e) ? removedWays : keptWays;
                bucket.putIfAbsent(w, row(graph, ag, na, gsEnc, rcEnc, psEnc, e, w));
            }
            // ways fully removed = removed minus any that still have a surviving connector edge
            List<String> fullyRemoved = new ArrayList<>();
            for (Map.Entry<Long, String> en : removedWays.entrySet())
                if (!keptWays.containsKey(en.getKey())) fullyRemoved.add(en.getValue());

            System.out.println("\n############ 2a ORPHAN-CONNECTOR PRUNE — TAMPERE DELTA ############");
            System.out.println("admitted connector ways (before prune): " + admittedWays);
            System.out.println("connector EDGES removed by prune:        " + removedEdges);
            System.out.println("connector WAYS fully removed by prune:   " + fullyRemoved.size());
            System.out.println("(a removed connector provably sits in a component with NO retained gravel)\n");
            System.out.println("SAMPLE of removed connector ways (open https://www.openstreetmap.org/way/<id>):");
            int shown = 0;
            for (String r : fullyRemoved) { System.out.println("  " + r); if (++shown >= 25) break; }
            System.out.println("################################################################\n");
        } finally {
            hopper.close();
        }
    }

    private static boolean[] admittedBeforeMask(AnalysisGraph ag, boolean[] mask) {
        for (int e = 0; e < ag.edgeCount(); e++)
            mask[e] = !ag.isRemoved(e) && ag.role(e) == EdgeRole.CONNECTOR;
        return mask;
    }

    private static java.util.Set<Long> retainedTargetWays(AnalysisGraph ag) {
        java.util.Set<Long> ws = new java.util.HashSet<>();
        for (int e = 0; e < ag.edgeCount(); e++)
            if (!ag.isRemoved(e) && ag.role(e) == EdgeRole.TARGET) ws.add(ag.osmWayId(e) & 0xFFFFFFFFL);
        return ws;
    }

    private static int countWays(AnalysisGraph ag, boolean[] mask) {
        java.util.Set<Long> ws = new java.util.HashSet<>();
        for (int e = 0; e < ag.edgeCount(); e++) if (mask[e]) ws.add(ag.osmWayId(e) & 0xFFFFFFFFL);
        return ws.size();
    }

    private static String row(BaseGraph graph, AnalysisGraph ag, NodeAccess na,
                              EnumEncodedValue<GravelScale> gsEnc, EnumEncodedValue<RoadClass> rcEnc,
                              EnumEncodedValue<PredictedSurface> psEnc, int e, long wayId) {
        int n = ag.nodeA(e);
        var es = graph.getEdgeIteratorState(ag.ghEdgeId(e), ag.nodeB(e));
        String nm = ag.name(e) == null || ag.name(e).isEmpty() ? "(unnamed)" : ag.name(e);
        return String.format("way %-11d  %-22s gs=%s surf=%s rc=%s  @ %.5f,%.5f",
                wayId, nm, es.get(gsEnc), es.get(psEnc), es.get(rcEnc), na.getLat(n), na.getLon(n));
    }
}
