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
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.FetchMode;
import com.graphhopper.util.Parameters;
import com.graphhopper.util.PointList;
import com.graphhopper.util.details.PathDetail;
import com.graphhopper.util.shapes.GHPoint;
import org.junit.jupiter.api.*;

import java.io.File;
import java.util.*;

/**
 * Debug test for the "phantom edge" routing problem near Tampere (Lielahti / lake shore).
 *
 * Two client /route requests (gravel profile) produce paths that follow a lake coastline
 * and a field/housing-area boundary that do NOT correspond to real highway OSM ways. The
 * suspicion is that these are AREA-ROUTING synthetic edges: closed polygons (parks,
 * recreation_ground, camp_site, etc.) accepted by area_routing_rules.json, whose boundary
 * is imported as a routable edge.
 *
 * Such synthetic edges have NO highway tag, so road_class == OTHER, yet bike_access is set —
 * that combination is the fingerprint that separates them from genuine cycleways/paths.
 * Note: osm_way_id is NOT stored in this graph (not in graph.encoded_values), so we identify
 * the culprit edges by road_class==OTHER + bike_access + geometry coordinates (paste into
 * openstreetmap.org to find the originating polygon).
 *
 * Uses the existing graph-cache (no re-import). Run from graphhopper/core.
 */
public class AreaEdgeRoutingDebugTest {

    private static final String CONFIG_FILE = "../trailmap-config.yml";

    // ---- Request 1: points [[lon,lat],[lon,lat]] = [[23.7391...,61.4740...],[23.7420...,61.4732...]]
    private static final GHPoint R1_START = new GHPoint(61.47403768942567, 23.739121681698435);
    private static final GHPoint R1_END   = new GHPoint(61.473210560825095, 23.742051666509866);

    // ---- Request 2: points [[23.742048,61.473208],[23.743051...,61.471458...]] with headings + heading_penalty
    private static final GHPoint R2_START = new GHPoint(61.473208, 23.742048);
    private static final GHPoint R2_END   = new GHPoint(61.471458152564026, 23.743051147309643);
    private static final double R2_HEADING = 124.91418826292033;
    private static final int R2_HEADING_PENALTY = 60;

    private static GraphHopper hopper;
    private static boolean isInitialized = false;

    @BeforeAll
    static void setupGraphHopper() {
        System.out.println("=== Area Edge Routing Debug Test ===");

        File configFile = new File(CONFIG_FILE);
        System.out.println("Config: " + configFile.getAbsolutePath() + " exists=" + configFile.exists());
        if (!configFile.exists()) { isInitialized = false; return; }

        try {
            ObjectMapper mapper = Jackson.initObjectMapper(new ObjectMapper(new YAMLFactory()));
            var rootNode = mapper.readTree(configFile);
            var ghNode = rootNode.get("graphhopper");
            if (ghNode == null) throw new IllegalStateException("Config missing 'graphhopper:' section");
            GraphHopperConfig config = mapper.treeToValue(ghNode, GraphHopperConfig.class);

            String graphLocation = config.getString("graph.location", "../data/graph-cache");
            String fixed = "../" + graphLocation;
            config.putObject("graph.location", fixed);
            System.out.println("Graph location: " + fixed);

            hopper = new GraphHopper();
            hopper.init(config);
            hopper.load();
            isInitialized = true;
            System.out.println("Loaded. nodes=" + hopper.getBaseGraph().getNodes()
                    + " edges=" + hopper.getBaseGraph().getEdges());
        } catch (Exception e) {
            e.printStackTrace();
            isInitialized = false;
        }
    }

    @AfterAll
    static void cleanup() {
        if (hopper != null) hopper.close();
    }

    @Test
    void testRequest1_lakeShore() {
        Assumptions.assumeTrue(isInitialized, "GraphHopper not initialized");
        GHRequest req = new GHRequest(Arrays.asList(R1_START, R1_END))
                .setProfile("gravel")
                .setAlgorithm(Parameters.Algorithms.ALT_ROUTE)
                .setSnapPreventions(List.of("ferry"))
                .putHint(Parameters.Routing.INSTRUCTIONS, false)
                .putHint(Parameters.Algorithms.AltRoute.MAX_PATHS, 3)
                .setPathDetails(detailKeys());
        dumpRequest("REQUEST 1 — reportedly follows lake shore (no OSM ways)", req, R1_START, R1_END);
    }

    @Test
    void testRequest2_fieldHousingEdge() {
        Assumptions.assumeTrue(isInitialized, "GraphHopper not initialized");
        GHRequest req = new GHRequest(Arrays.asList(R2_START, R2_END))
                .setProfile("gravel")
                .setAlgorithm(Parameters.Algorithms.ALT_ROUTE)
                .setSnapPreventions(List.of("ferry"))
                .setHeadings(Arrays.asList(R2_HEADING, Double.NaN))
                .putHint(Parameters.Routing.INSTRUCTIONS, false)
                .putHint(Parameters.Routing.HEADING_PENALTY, R2_HEADING_PENALTY)
                .putHint(Parameters.Algorithms.AltRoute.MAX_PATHS, 3)
                .setPathDetails(detailKeys());
        dumpRequest("REQUEST 2 — reportedly follows field/housing-area boundary", req, R2_START, R2_END);
    }

    private static List<String> detailKeys() {
        return Arrays.asList(
                "edge_id", "road_class", "road_environment", "average_speed",
                "gravel_scale", "gravel_scale_num", "gravel_base_priority",
                "predicted_surface", "predicted_highway");
    }

    private void dumpRequest(String label, GHRequest request, GHPoint start, GHPoint end) {
        System.out.println("\n##################################################");
        System.out.println("# " + label);
        System.out.println("##################################################");
        System.out.println("start = " + start + "   end = " + end);

        GHResponse response = hopper.route(request);
        if (response.hasErrors()) {
            System.err.println("Routing errors: " + response.getErrors());
            Assertions.fail("Routing failed: " + response.getErrors());
            return;
        }

        int altCount = response.getAll().size();
        System.out.println("Returned " + altCount + " path(s).");

        // Only the BEST path matters for the reported behaviour, but dump all so we can
        // see whether the area-edge detour is the chosen one or an alternative.
        for (int p = 0; p < altCount; p++) {
            ResponsePath path = response.getAll().get(p);
            System.out.println("\n--- Path " + p + (p == 0 ? " (BEST)" : "") + " ---");
            System.out.printf("Distance: %.1f m   Time: %d s   Weight: %.3f%n",
                    path.getDistance(), path.getTime() / 1000, path.getRouteWeight());

            Map<String, List<PathDetail>> details = path.getPathDetails();
            List<PathDetail> edgeIdDetails = details.get("edge_id");
            if (edgeIdDetails == null || edgeIdDetails.isEmpty()) {
                System.out.println("  (no edge_id details)");
                continue;
            }

            System.out.println("Edges: " + edgeIdDetails.size());
            System.out.printf("%-4s %-8s %-12s %-10s %-7s %-7s %-22s %-16s %-16s %-10s%n",
                    "#", "edge_id", "road_class", "road_env", "AREA?", "bike", "name", "pred_surf", "pred_hwy", "dist_m");
            System.out.println("-".repeat(130));

            int areaEdgeCount = 0;
            double areaEdgeMeters = 0;
            List<Integer> areaEdgeIds = new ArrayList<>();

            for (int i = 0; i < edgeIdDetails.size(); i++) {
                PathDetail ed = edgeIdDetails.get(i);
                int first = ed.getFirst();
                int last = ed.getLast();
                int edgeId = ((Number) ed.getValue()).intValue();

                EdgeInfo info = rawEdgeInfo(edgeId);
                boolean isArea = "other".equalsIgnoreCase(info.roadClass) && (info.bikeFwd || info.bikeRev);
                if (isArea) {
                    areaEdgeCount++;
                    areaEdgeMeters += info.distance;
                    areaEdgeIds.add(edgeId);
                }

                System.out.printf("%-4d %-8d %-12s %-10s %-7s %-7s %-22s %-16s %-16s %-10.1f%n",
                        i, edgeId,
                        info.roadClass,
                        findDetailValue(details.get("road_environment"), first, last),
                        isArea ? "*** YES" : "-",
                        (info.bikeFwd ? "F" : "-") + (info.bikeRev ? "R" : "-"),
                        truncate(info.name, 22),
                        findDetailValue(details.get("predicted_surface"), first, last),
                        findDetailValue(details.get("predicted_highway"), first, last),
                        info.distance);
            }

            System.out.printf("%nSUMMARY path %d: %d/%d edges are AREA-ROUTING synthetic edges (road_class=OTHER + bike_access), totalling %.1f m of %.1f m (%.0f%%).%n",
                    p, areaEdgeCount, edgeIdDetails.size(), areaEdgeMeters, path.getDistance(),
                    path.getDistance() > 0 ? 100.0 * areaEdgeMeters / path.getDistance() : 0);

            // For the BEST path, dump full geometry of each suspect (area) edge so the
            // originating polygon can be located on openstreetmap.org.
            if (p == 0 && !areaEdgeIds.isEmpty()) {
                System.out.println("\n>>> Geometry of suspect AREA edges (paste a coord into openstreetmap.org to find the polygon):");
                for (int edgeId : areaEdgeIds) {
                    dumpEdgeGeometry(edgeId);
                }
            }
        }
    }

    private static class EdgeInfo {
        String roadClass = "?";
        String name = "";
        double distance = 0;
        boolean bikeFwd, bikeRev;
    }

    private EdgeInfo rawEdgeInfo(int edgeId) {
        EncodingManager em = hopper.getEncodingManager();
        BaseGraph graph = hopper.getBaseGraph();
        EnumEncodedValue<RoadClass> roadClassEV = em.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        BooleanEncodedValue bikeAccessEV = em.getBooleanEncodedValue("bike_access");

        EdgeInfo info = new EdgeInfo();
        EdgeIteratorState edge = graph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
        if (edge == null) return info;
        info.roadClass = String.valueOf(edge.get(roadClassEV));
        info.bikeFwd = edge.get(bikeAccessEV);
        info.bikeRev = edge.getReverse(bikeAccessEV);
        info.distance = edge.getDistance();
        String n = edge.getName();
        info.name = (n == null || n.isEmpty()) ? "(no name)" : n;
        return info;
    }

    private void dumpEdgeGeometry(int edgeId) {
        BaseGraph graph = hopper.getBaseGraph();
        EdgeIteratorState edge = graph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
        if (edge == null) return;
        PointList pts = edge.fetchWayGeometry(FetchMode.ALL);
        System.out.printf("  edge %d  base=%d adj=%d  dist=%.1fm  name='%s'  points=%d%n",
                edgeId, edge.getBaseNode(), edge.getAdjNode(), edge.getDistance(), edge.getName(), pts.size());
        for (int i = 0; i < pts.size(); i++) {
            System.out.printf("      [%d] %.7f, %.7f%n", i, pts.getLat(i), pts.getLon(i));
        }
    }

    private static String truncate(String s, int max) {
        if (s == null) return "";
        return s.length() > max ? s.substring(0, max) : s;
    }

    private String findDetailValue(List<PathDetail> details, int first, int last) {
        if (details == null) return "N/A";
        for (PathDetail d : details) {
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
