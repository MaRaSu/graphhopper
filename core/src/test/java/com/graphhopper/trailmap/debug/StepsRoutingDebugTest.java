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
import com.graphhopper.trailmap.shared.*;
import com.graphhopper.util.CustomModel;
import com.graphhopper.util.Parameters;
import com.graphhopper.json.Statement;
import com.graphhopper.util.details.PathDetail;
import com.graphhopper.util.shapes.GHPoint;
import org.junit.jupiter.api.*;

import java.io.File;
import java.util.*;

/**
 * Diagnostic test for highway=steps weighting in the gravel profile.
 *
 * Compares two route requests near Tampere with nearly-identical endpoints — one chooses
 * a path that crosses steps, the other detours around them. Dumps per-edge encoded values
 * (gravel_scale, predicted_highway, bike_access, average_speed, etc.) and Trailmap's
 * custom-model inputs so we can see exactly what numbers the router consumed.
 *
 * Mirrors the request bodies the client issued:
 *   profile=gravel, algorithm=alternative_route, alternative_route.max_paths=3,
 *   snap_preventions=[ferry], custom_model={priority:[MAJOR_ROAD→1.45], speed:[]}
 *
 * Uses the existing graph-cache (no re-import). Run from graphhopper/core.
 */
public class StepsRoutingDebugTest {

    private static final String CONFIG_FILE = "../trailmap-config.yml";

    // Request A — original (router picks the steps path)
    private static final GHPoint A_START = new GHPoint(61.47452882034372, 23.775485055063683);
    private static final GHPoint A_END   = new GHPoint(61.47480084316851, 23.77628252988214);

    // Request B — slightly shifted endpoints (router avoids steps)
    private static final GHPoint B_START = new GHPoint(61.474528, 23.775492);
    private static final GHPoint B_END   = new GHPoint(61.47488724992141, 23.776148500500454);

    private static GraphHopper hopper;
    private static boolean isInitialized = false;

    @BeforeAll
    static void setupGraphHopper() {
        System.out.println("=== Steps Routing Debug Test ===");

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
    void testRequestA_usesSteps() {
        Assumptions.assumeTrue(isInitialized);
        dumpRequest("A — original endpoints (reportedly routes through steps)", A_START, A_END);
    }

    @Test
    void testRequestB_avoidsSteps() {
        Assumptions.assumeTrue(isInitialized);
        dumpRequest("B — slightly shifted endpoints (reportedly avoids steps)", B_START, B_END);
    }

    private void dumpRequest(String label, GHPoint start, GHPoint end) {
        System.out.println("\n##################################################");
        System.out.println("# " + label);
        System.out.println("##################################################");
        System.out.println("start = " + start);
        System.out.println("end   = " + end);

        // Match the request body custom_model: priority: MAJOR_ROAD * 1.45
        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If("predicted_highway == MAJOR_ROAD", Statement.Op.MULTIPLY, "1.45"));

        List<String> pathDetailKeys = Arrays.asList(
                "edge_id", "road_class", "road_environment", "average_speed",
                "gravel_scale", "gravel_scale_num", "gravel_base_priority",
                "mtb_scale", "mtb_scale_num", "mtb_base_priority",
                "predicted_surface", "predicted_highway",
                "issue_biking_blocked", "issue_foot_blocked", "issue_narrow",
                "issue_poor_visibility", "issue_vegetation", "issue_mud",
                "issue_unknown_path", "issue_unknown_track", "issue_ferry"
        );

        GHRequest request = new GHRequest(Arrays.asList(start, end))
                .setProfile("gravel")
                .setAlgorithm(Parameters.Algorithms.ALT_ROUTE)
                .setSnapPreventions(List.of("ferry"))
                .setPathDetails(pathDetailKeys)
                .putHint(Parameters.Routing.INSTRUCTIONS, false)
                .putHint(Parameters.Routing.CALC_POINTS, true)
                .putHint(Parameters.Algorithms.AltRoute.MAX_PATHS, 3)
                .putHint("timeout_ms", 10000);
        request.setCustomModel(cm);

        GHResponse response = hopper.route(request);
        if (response.hasErrors()) {
            System.err.println("Routing errors: " + response.getErrors());
            Assertions.fail("Routing failed: " + response.getErrors());
            return;
        }

        int altCount = response.getAll().size();
        System.out.println("Returned " + altCount + " path(s).");

        for (int p = 0; p < altCount; p++) {
            ResponsePath path = response.getAll().get(p);
            System.out.println("\n--- Path " + p + (p == 0 ? " (BEST)" : "") + " ---");
            System.out.println("Distance: " + String.format("%.1f", path.getDistance()) + " m");
            System.out.println("Time:     " + (path.getTime() / 1000) + " s");
            System.out.println("Weight:   " + String.format("%.3f", path.getRouteWeight()));

            Map<String, List<PathDetail>> details = path.getPathDetails();
            List<PathDetail> edgeIdDetails = details.get("edge_id");
            if (edgeIdDetails == null || edgeIdDetails.isEmpty()) {
                System.out.println("  (no edge_id details)");
                continue;
            }

            System.out.println("Edges:    " + edgeIdDetails.size());
            System.out.printf("%-4s %-8s %-12s %-10s %-12s %-6s %-6s %-12s %-16s %-16s %-7s %-7s%n",
                    "#", "edge_id", "road_class", "road_env", "gravel_sc", "gs#", "gp", "mtb_sc",
                    "pred_surf", "pred_hwy", "avg_spd", "issues");
            System.out.println("-".repeat(140));

            for (int i = 0; i < edgeIdDetails.size(); i++) {
                PathDetail edgeDetail = edgeIdDetails.get(i);
                int first = edgeDetail.getFirst();
                int last = edgeDetail.getLast();
                String issueFlags = issueFlags(details, first, last);

                System.out.printf("%-4d %-8s %-12s %-10s %-12s %-6s %-6s %-12s %-16s %-16s %-7s %-7s%n",
                        i,
                        String.valueOf(edgeDetail.getValue()),
                        findDetailValue(details.get("road_class"), first, last),
                        findDetailValue(details.get("road_environment"), first, last),
                        findDetailValue(details.get("gravel_scale"), first, last),
                        findDetailValue(details.get("gravel_scale_num"), first, last),
                        findDetailValue(details.get("gravel_base_priority"), first, last),
                        findDetailValue(details.get("mtb_scale"), first, last),
                        findDetailValue(details.get("predicted_surface"), first, last),
                        findDetailValue(details.get("predicted_highway"), first, last),
                        findDetailValue(details.get("average_speed"), first, last),
                        issueFlags);
            }

            // Also fetch raw encoded-value snapshot (incl. bike_access fwd/rev and
            // gravel_base_priority) for each edge — these are NOT path details for some
            // booleans but we can read them from the graph by edge id.
            dumpRawEdgeValues(edgeIdDetails);
        }
    }

    private void dumpRawEdgeValues(List<PathDetail> edgeIdDetails) {
        EncodingManager em = hopper.getEncodingManager();
        var graph = hopper.getBaseGraph();
        BooleanEncodedValue bikeAccessEV = em.getBooleanEncodedValue("bike_access");
        DecimalEncodedValue avgSpeedEV = em.getDecimalEncodedValue(VehicleSpeed.key("bike"));
        DecimalEncodedValue avgSlopeEV = em.getDecimalEncodedValue("average_slope");
        EnumEncodedValue<RoadClass> roadClassEV = em.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<PredictedHighway> predHwyEV = em.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
        EnumEncodedValue<GravelScale> gravelScaleEV = em.getEnumEncodedValue(GravelScale.KEY, GravelScale.class);

        System.out.println();
        System.out.println("Raw graph values per edge (fwd/rev for direction-sensitive):");
        System.out.printf("  %-8s %-7s %-8s %-12s %-14s %-12s %-9s %-9s %-7s %-6s%n",
                "edge_id", "dist_m", "name", "road_class", "pred_hwy", "gravel_sc",
                "spd_fwd", "spd_rev", "access", "slope");
        System.out.println("  " + "-".repeat(120));

        for (PathDetail d : edgeIdDetails) {
            int edgeId = ((Number) d.getValue()).intValue();
            var edge = graph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            if (edge == null) continue;
            boolean accF = edge.get(bikeAccessEV);
            boolean accR = edge.getReverse(bikeAccessEV);
            double spdF = edge.get(avgSpeedEV);
            double spdR = edge.getReverse(avgSpeedEV);
            double slope = edge.get(avgSlopeEV);
            String acc = (accF ? "F" : "-") + (accR ? "R" : "-");
            String name = edge.getName();
            if (name != null && name.length() > 8) name = name.substring(0, 8);

            System.out.printf("  %-8d %-7.1f %-8s %-12s %-14s %-12s %-9.2f %-9.2f %-7s %-6.1f%n",
                    edgeId,
                    edge.getDistance(),
                    name == null || name.isEmpty() ? "-" : name,
                    edge.get(roadClassEV),
                    edge.get(predHwyEV),
                    edge.get(gravelScaleEV),
                    spdF, spdR, acc, slope);
        }
    }

    /** Compact summary of which issue_* flags are set on this edge. */
    private String issueFlags(Map<String, List<PathDetail>> details, int first, int last) {
        String[] keys = {
                "issue_biking_blocked", "issue_foot_blocked", "issue_narrow",
                "issue_poor_visibility", "issue_vegetation", "issue_mud",
                "issue_unknown_path", "issue_unknown_track", "issue_ferry"
        };
        StringBuilder sb = new StringBuilder();
        for (String k : keys) {
            String v = findDetailValue(details.get(k), first, last);
            if (v != null && !v.equals("?") && !v.equals("N/A") && !v.equalsIgnoreCase("false") && !v.equals("0")) {
                if (sb.length() > 0) sb.append(',');
                sb.append(k.replace("issue_", ""));
            }
        }
        return sb.length() == 0 ? "-" : sb.toString();
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
