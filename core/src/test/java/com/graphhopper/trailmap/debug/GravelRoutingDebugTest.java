package com.graphhopper.trailmap.debug;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.ResponsePath;
import com.graphhopper.jackson.Jackson;
import com.graphhopper.routing.ev.*;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.trailmap.shared.*;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.Parameters;
import com.graphhopper.util.details.PathDetail;
import com.graphhopper.util.shapes.GHPoint;
import org.junit.jupiter.api.*;

import java.io.File;
import java.util.*;

/**
 * Debug test for investigating weird gravel routing behavior near Tampere, Finland.
 *
 * A short route between two close points takes a bizarre detour instead of going straight
 * along cycleways. This test inspects edge-by-edge encoded values to understand why.
 *
 * Uses the existing graph-cache built by the server (no re-import).
 * Run from graphhopper/core directory.
 */
public class GravelRoutingDebugTest {

    private static final String CONFIG_FILE = "../trailmap-config.yml";

    // Tampere coordinates (lon,lat in API = lat,lon in GHPoint)
    private static final GHPoint START = new GHPoint(61.491359, 23.746356);
    private static final GHPoint END = new GHPoint(61.49182182021838, 23.745932492901375);

    private static GraphHopper hopper;
    private static boolean isInitialized = false;

    @BeforeAll
    static void setupGraphHopper() {
        System.out.println("=== Gravel Routing Debug Test ===");

        File configFile = new File(CONFIG_FILE);
        System.out.println("Config file: " + configFile.getAbsolutePath());
        System.out.println("Config exists: " + configFile.exists());

        if (!configFile.exists()) {
            System.err.println("ERROR: Config file not found at " + configFile.getAbsolutePath());
            isInitialized = false;
            return;
        }

        try {
            ObjectMapper mapper = Jackson.initObjectMapper(new ObjectMapper(new YAMLFactory()));
            var rootNode = mapper.readTree(configFile);
            var ghNode = rootNode.get("graphhopper");
            if (ghNode == null) {
                throw new IllegalStateException("Config file missing 'graphhopper:' section");
            }
            GraphHopperConfig config = mapper.treeToValue(ghNode, GraphHopperConfig.class);

            String graphLocation = config.getString("graph.location", "../data/graph-cache");
            String fixedGraphLocation = "../" + graphLocation;
            config.putObject("graph.location", fixedGraphLocation);

            System.out.println("Graph location (fixed): " + fixedGraphLocation);

            hopper = new GraphHopper();
            hopper.init(config);
            hopper.load();

            isInitialized = true;
            System.out.println("GraphHopper loaded successfully");
            System.out.println("Base graph nodes: " + hopper.getBaseGraph().getNodes());
            System.out.println("Base graph edges: " + hopper.getBaseGraph().getEdges());
        } catch (Exception e) {
            System.err.println("Failed to load GraphHopper: " + e.getMessage());
            e.printStackTrace();
            isInitialized = false;
        }
    }

    @AfterAll
    static void cleanup() {
        if (hopper != null) {
            hopper.close();
        }
    }

    /**
     * Test 1: Route with gravel profile and print edge-by-edge diagnostic table
     * showing all encoded values for each edge the router chose.
     */
    @Test
    void testTampereWeirdDetour() {
        Assumptions.assumeTrue(isInitialized, "GraphHopper not initialized");

        System.out.println("\n=== Test 1: Tampere Weird Detour - Edge-by-Edge Diagnostics ===");
        System.out.println("Start: " + START);
        System.out.println("End:   " + END);

        List<String> pathDetailKeys = Arrays.asList(
                "edge_id", "road_class", "road_environment",
                "gravel_scale", "gravel_scale_num", "gravel_base_priority",
                "mtb_scale", "mtb_scale_num",
                "predicted_surface", "predicted_highway", "average_speed"
        );

        GHRequest request = new GHRequest(Arrays.asList(START, END))
                .setProfile("gravel")
                .putHint(Parameters.Routing.INSTRUCTIONS, false)
                .setPathDetails(pathDetailKeys);

        GHResponse response = hopper.route(request);

        if (response.hasErrors()) {
            System.err.println("Routing errors: " + response.getErrors());
            Assertions.fail("Routing failed: " + response.getErrors());
            return;
        }

        ResponsePath path = response.getBest();
        System.out.println("\nRoute summary:");
        System.out.println("  Distance: " + String.format("%.1f", path.getDistance()) + " m");
        System.out.println("  Time:     " + (path.getTime() / 1000) + " s");

        Map<String, List<PathDetail>> details = path.getPathDetails();
        List<PathDetail> edgeIdDetails = details.get("edge_id");

        if (edgeIdDetails == null || edgeIdDetails.isEmpty()) {
            System.out.println("  No edge_id details returned!");
            return;
        }

        int numEdges = edgeIdDetails.size();
        System.out.println("  Edges:    " + numEdges);

        // Print header
        System.out.println();
        System.out.printf("%-4s  %-8s  %-14s  %-12s  %-14s  %-10s  %-12s  %-10s  %-10s  %-16s  %-16s  %-8s%n",
                "#", "edge_id", "road_class", "road_env", "gravel_scale", "gs_num", "gs_priority", "mtb_scale", "mtb_num",
                "pred_surface", "pred_highway", "avg_spd");
        System.out.println("-".repeat(160));

        // For each edge, find corresponding values from other detail lists
        for (int i = 0; i < numEdges; i++) {
            PathDetail edgeDetail = edgeIdDetails.get(i);
            int first = edgeDetail.getFirst();
            int last = edgeDetail.getLast();

            String edgeId = String.valueOf(edgeDetail.getValue());
            String roadClass = findDetailValue(details.get("road_class"), first, last);
            String roadEnv = findDetailValue(details.get("road_environment"), first, last);
            String gravelScale = findDetailValue(details.get("gravel_scale"), first, last);
            String gravelScaleNum = findDetailValue(details.get("gravel_scale_num"), first, last);
            String gravelPriority = findDetailValue(details.get("gravel_base_priority"), first, last);
            String mtbScale = findDetailValue(details.get("mtb_scale"), first, last);
            String mtbScaleNum = findDetailValue(details.get("mtb_scale_num"), first, last);
            String predSurface = findDetailValue(details.get("predicted_surface"), first, last);
            String predHighway = findDetailValue(details.get("predicted_highway"), first, last);
            String avgSpeed = findDetailValue(details.get("average_speed"), first, last);

            System.out.printf("%-4d  %-8s  %-14s  %-12s  %-14s  %-10s  %-12s  %-10s  %-10s  %-16s  %-16s  %-8s%n",
                    i, edgeId, roadClass, roadEnv, gravelScale, gravelScaleNum, gravelPriority,
                    mtbScale, mtbScaleNum, predSurface, predHighway, avgSpeed);
        }
    }

    /**
     * Test 2: Inspect raw graph edges around the snap points.
     * Shows all adjacent edges and their encoded values at start and end locations.
     */
    @Test
    void testTampereEdgeInspection() {
        Assumptions.assumeTrue(isInitialized, "GraphHopper not initialized");

        System.out.println("\n=== Test 2: Tampere Edge Inspection - Raw Graph Data ===");

        EncodingManager em = hopper.getEncodingManager();
        BaseGraph graph = hopper.getBaseGraph();

        // Get encoded values
        EnumEncodedValue<GravelScale> gravelScaleEV = em.getEnumEncodedValue(GravelScale.KEY, GravelScale.class);
        DecimalEncodedValue gravelScaleNumEV = em.getDecimalEncodedValue(GravelScaleNum.KEY);
        DecimalEncodedValue gravelBasePriorityEV = em.getDecimalEncodedValue(GravelBasePriority.KEY);
        BooleanEncodedValue bikeAccessEV = em.getBooleanEncodedValue("bike_access");
        EnumEncodedValue<RoadClass> roadClassEV = em.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<RoadEnvironment> roadEnvEV = em.getEnumEncodedValue(RoadEnvironment.KEY, RoadEnvironment.class);
        EnumEncodedValue<PredictedSurface> predSurfaceEV = em.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class);
        EnumEncodedValue<PredictedHighway> predHighwayEV = em.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
        EnumEncodedValue<MtbScale> mtbScaleEV = em.getEnumEncodedValue(MtbScale.KEY, MtbScale.class);
        DecimalEncodedValue mtbScaleNumEV = em.getDecimalEncodedValue(MtbScaleNum.KEY);
        DecimalEncodedValue avgSpeedEV = em.getDecimalEncodedValue(VehicleSpeed.key("bike"));

        // Snap both points
        Snap startSnap = hopper.getLocationIndex().findClosest(START.getLat(), START.getLon(), EdgeFilter.ALL_EDGES);
        Snap endSnap = hopper.getLocationIndex().findClosest(END.getLat(), END.getLon(), EdgeFilter.ALL_EDGES);

        System.out.println("Start snap: node=" + startSnap.getClosestNode() +
                " valid=" + startSnap.isValid() +
                " snappedPoint=" + startSnap.getSnappedPoint());
        System.out.println("End snap:   node=" + endSnap.getClosestNode() +
                " valid=" + endSnap.isValid() +
                " snappedPoint=" + endSnap.getSnappedPoint());

        // Inspect edges around each snap point
        EdgeExplorer explorer = graph.createEdgeExplorer();

        for (String label : new String[]{"START", "END"}) {
            Snap snap = label.equals("START") ? startSnap : endSnap;
            int nodeId = snap.getClosestNode();

            System.out.println("\n--- Adjacent edges for " + label + " node " + nodeId + " ---");
            System.out.printf("%-8s  %-6s  %-6s  %-10s  %-14s  %-12s  %-14s  %-10s  %-12s  %-10s  %-10s  %-16s  %-16s  %-8s%n",
                    "edge_id", "base", "adj", "bike_acc", "road_class", "road_env", "gravel_scale", "gs_num", "gs_priority",
                    "mtb_scale", "mtb_num", "pred_surface", "pred_highway", "avg_spd");
            System.out.println("-".repeat(180));

            EdgeIterator iter = explorer.setBaseNode(nodeId);
            while (iter.next()) {
                boolean fwdAccess = iter.get(bikeAccessEV);
                boolean revAccess = iter.getReverse(bikeAccessEV);
                String accessStr = (fwdAccess ? "F" : "-") + (revAccess ? "R" : "-");

                System.out.printf("%-8d  %-6d  %-6d  %-10s  %-14s  %-12s  %-14s  %-10.1f  %-12.2f  %-10s  %-10.1f  %-16s  %-16s  %-8.1f%n",
                        iter.getEdge(),
                        iter.getBaseNode(),
                        iter.getAdjNode(),
                        accessStr,
                        iter.get(roadClassEV),
                        iter.get(roadEnvEV),
                        iter.get(gravelScaleEV),
                        iter.get(gravelScaleNumEV),
                        iter.get(gravelBasePriorityEV),
                        iter.get(mtbScaleEV),
                        iter.get(mtbScaleNumEV),
                        iter.get(predSurfaceEV),
                        iter.get(predHighwayEV),
                        iter.get(avgSpeedEV));
            }
        }
    }

    /**
     * Test 3: Route with ALL bike-like profiles and compare distances/edges.
     * Shows whether the weird detour is specific to the gravel model or a graph structure issue.
     */
    @Test
    void testTampereStraightVsActual() {
        Assumptions.assumeTrue(isInitialized, "GraphHopper not initialized");

        System.out.println("\n=== Test 3: Tampere - All Profiles Comparison ===");
        System.out.println("Start: " + START);
        System.out.println("End:   " + END);

        String[] profiles = {"gravel", "gravel_easy", "groad", "gravel_mtb", "safebike", "roadbike", "mtb"};

        System.out.println();
        System.out.printf("%-14s  %-10s  %-6s  %s%n", "Profile", "Distance", "Edges", "Edge IDs");
        System.out.println("-".repeat(120));

        for (String profile : profiles) {
            try {
                GHRequest request = new GHRequest(Arrays.asList(START, END))
                        .setProfile(profile)
                        .putHint(Parameters.Routing.INSTRUCTIONS, false)
                        .setPathDetails(Collections.singletonList("edge_id"));

                GHResponse response = hopper.route(request);

                if (response.hasErrors()) {
                    System.out.printf("%-14s  ERROR: %s%n", profile, response.getErrors().get(0).getMessage());
                    continue;
                }

                ResponsePath path = response.getBest();
                List<PathDetail> edgeIdDetails = path.getPathDetails().get("edge_id");

                StringBuilder edgeIds = new StringBuilder();
                if (edgeIdDetails != null) {
                    for (int i = 0; i < edgeIdDetails.size(); i++) {
                        if (i > 0) edgeIds.append(", ");
                        edgeIds.append(edgeIdDetails.get(i).getValue());
                    }
                }

                System.out.printf("%-14s  %-10.1f  %-6d  %s%n",
                        profile,
                        path.getDistance(),
                        edgeIdDetails != null ? edgeIdDetails.size() : 0,
                        edgeIds);

            } catch (Exception e) {
                System.out.printf("%-14s  EXCEPTION: %s%n", profile, e.getMessage());
            }
        }
    }

    /**
     * Test 4: Deep investigation — snap details, edge geometry, and why the direct edge is avoided.
     * Prints the snapped edge, distance to each edge endpoint, and walks the detour vs direct path.
     */
    @Test
    void testTampereSnapAndConnectivity() {
        Assumptions.assumeTrue(isInitialized, "GraphHopper not initialized");

        System.out.println("\n=== Test 4: Snap Details & Connectivity ===");

        BaseGraph graph = hopper.getBaseGraph();
        var na = graph.getNodeAccess();

        // Show snap details
        Snap startSnap = hopper.getLocationIndex().findClosest(START.getLat(), START.getLon(), EdgeFilter.ALL_EDGES);
        Snap endSnap = hopper.getLocationIndex().findClosest(END.getLat(), END.getLon(), EdgeFilter.ALL_EDGES);

        System.out.println("\nSTART snap details:");
        System.out.println("  Closest node: " + startSnap.getClosestNode());
        System.out.println("  Snapped edge: " + startSnap.getClosestEdge().getEdge() +
                " (base=" + startSnap.getClosestEdge().getBaseNode() +
                " adj=" + startSnap.getClosestEdge().getAdjNode() + ")");
        System.out.println("  Snapped point: " + startSnap.getSnappedPoint());
        System.out.println("  Query point:   " + START);
        System.out.println("  Snap type: " + startSnap.getSnappedPosition());

        System.out.println("\nEND snap details:");
        System.out.println("  Closest node: " + endSnap.getClosestNode());
        System.out.println("  Snapped edge: " + endSnap.getClosestEdge().getEdge() +
                " (base=" + endSnap.getClosestEdge().getBaseNode() +
                " adj=" + endSnap.getClosestEdge().getAdjNode() + ")");
        System.out.println("  Snapped point: " + endSnap.getSnappedPoint());
        System.out.println("  Query point:   " + END);
        System.out.println("  Snap type: " + endSnap.getSnappedPosition());

        // Print node coordinates for all nodes involved
        System.out.println("\n--- Node coordinates ---");
        int[] nodes = {1994845, 1994849, 1994851, 1994852, 1994853, 1994854, 1994878};
        for (int n : nodes) {
            if (n < graph.getNodes()) {
                System.out.printf("  Node %d: lat=%.6f, lon=%.6f, ele=%.1f%n",
                        n, na.getLat(n), na.getLon(n), na.getEle(n));
            }
        }

        // Print edge distances and slopes
        System.out.println("\n--- Edge distances and slopes ---");
        DecimalEncodedValue avgSlopeEV = hopper.getEncodingManager().getDecimalEncodedValue("average_slope");
        EdgeExplorer explorer = graph.createEdgeExplorer();
        for (int nodeId : new int[]{1994849, 1994853}) {
            EdgeIterator iter = explorer.setBaseNode(nodeId);
            while (iter.next()) {
                System.out.printf("  Edge %d: %d→%d  dist=%.1fm  slope_fwd=%.1f  slope_rev=%.1f  name='%s'%n",
                        iter.getEdge(), iter.getBaseNode(), iter.getAdjNode(),
                        iter.getDistance(), iter.get(avgSlopeEV), iter.getReverse(avgSlopeEV), iter.getName());
            }
        }

        // The direct edge between start and end
        System.out.println("\n--- Direct edge 2459014 details ---");
        EdgeIteratorState directEdge = graph.getEdgeIteratorState(2459014, Integer.MIN_VALUE);
        if (directEdge != null) {
            System.out.println("  Edge ID: " + directEdge.getEdge());
            System.out.println("  Base→Adj: " + directEdge.getBaseNode() + "→" + directEdge.getAdjNode());
            System.out.println("  Distance: " + directEdge.getDistance() + "m");
            System.out.println("  Name: '" + directEdge.getName() + "'");
            System.out.println("  Geometry points: " + directEdge.fetchWayGeometry(com.graphhopper.util.FetchMode.ALL));

            // Check access in both directions
            BooleanEncodedValue bikeAccessEV = hopper.getEncodingManager().getBooleanEncodedValue("bike_access");
            System.out.println("  bike_access fwd: " + directEdge.get(bikeAccessEV));
            System.out.println("  bike_access rev: " + directEdge.getReverse(bikeAccessEV));
        }

        // Also check: does the gravel custom model weighting for the direct edge differ?
        // Route start→end forcing through the direct edge by using intermediate point
        System.out.println("\n--- Route geometry (gravel, 4 edges) ---");
        GHRequest req = new GHRequest(Arrays.asList(START, END))
                .setProfile("gravel")
                .putHint(Parameters.Routing.INSTRUCTIONS, false);
        GHResponse resp = hopper.route(req);
        if (!resp.hasErrors()) {
            var pts = resp.getBest().getPoints();
            for (int i = 0; i < pts.size(); i++) {
                System.out.printf("  [%d] lat=%.6f, lon=%.6f%n", i, pts.getLat(i), pts.getLon(i));
            }
        }

        System.out.println("\n--- Route geometry (mtb, 2 edges / direct) ---");
        GHRequest req2 = new GHRequest(Arrays.asList(START, END))
                .setProfile("mtb")
                .putHint(Parameters.Routing.INSTRUCTIONS, false);
        GHResponse resp2 = hopper.route(req2);
        if (!resp2.hasErrors()) {
            var pts = resp2.getBest().getPoints();
            for (int i = 0; i < pts.size(); i++) {
                System.out.printf("  [%d] lat=%.6f, lon=%.6f%n", i, pts.getLat(i), pts.getLon(i));
            }
        }
    }

    /**
     * Find the value from a detail list that covers the given point range [first, last).
     * Path details have first/last indices into the point list; we match by overlap.
     */
    private String findDetailValue(List<PathDetail> details, int first, int last) {
        if (details == null) return "N/A";
        for (PathDetail d : details) {
            // Details overlap if they share any point index range
            if (d.getFirst() < last && d.getLast() > first) {
                Object val = d.getValue();
                if (val instanceof Number) {
                    double dval = ((Number) val).doubleValue();
                    if (dval == Math.floor(dval) && !Double.isInfinite(dval)) {
                        return String.valueOf((int) dval);
                    }
                    return String.format("%.2f", dval);
                }
                return String.valueOf(val);
            }
        }
        return "?";
    }
}
