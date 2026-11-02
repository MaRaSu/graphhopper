package com.graphhopper.trailmap.fixroute;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.graphhopper.GraphHopper;
import com.graphhopper.util.shapes.GHPoint;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.nio.file.Files;
import java.util.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * W26 S4 /fix_route gate check (test_lead_routing, CATALOG §8), in one JVM on the S1 graph, every fixture route:
 * <ol>
 * <li>no flag: pass 1 + pass 2 responses (timings removed) equal the baseline file {@code -Dfr.baseline} line by line
 * (the responses.jsonl of a FixRouteCorpusTest run before the W26 edits);</li>
 * <li>flag vs no flag, ignoring only metrics.probes: the legs that differ are exactly {@link #EXPECTED_PASS1} (pass 1)
 * and the routes in {@link #EXPECTED_PASS2_ROUTES} (pass 2); pass 2 with the flag has 0 violations;</li>
 * <li>every flagged leg whose own route needs a crossing (the routeAsApp retry) equals plain /route + barriers "v1"
 * on the same waypoints and profile, as an edge_key sequence.</li>
 * </ol>
 * <pre>-Dfix.graph=DIR -Dfix.config=FILE -Dfix.fixtures=DIR -Dfr.baseline=FILE</pre>
 */
public class BarrierFixRouteFlagDiffTest {

    /** the legs that differ with the flag in pass 1 (16178: its legs through and behind the closed ask gate) */
    static final Set<String> EXPECTED_PASS1 = Set.of("16178:15", "16178:16");
    static final Set<Long> EXPECTED_PASS2_ROUTES = Set.of(16178L);

    static final ObjectMapper JSON = new ObjectMapper();

    static List<String> legs(String dump, boolean dropProbes) throws Exception {
        List<String> out = new ArrayList<>();
        for (JsonNode l : JSON.readTree(dump).get("legs")) {
            ObjectNode c = l.deepCopy();
            if (dropProbes && c.get("metrics") instanceof ObjectNode) ((ObjectNode) c.get("metrics")).remove("probes");
            out.add(c.toString());
        }
        return out;
    }

    @Test
    void flagVsNoFlag() throws Exception {
        Assumptions.assumeTrue(System.getProperty("fr.baseline") != null, "fr.baseline not set");
        GraphHopper hopper = FixRouteTestGraph.open();
        FixRouteCorpusTest.hopper = hopper; // its applyAndReroute re-routes with this graph
        ExecutorService pool = Executors.newFixedThreadPool(Math.max(2, Runtime.getRuntime().availableProcessors() - 2));
        Map<Long, String> baseline = new HashMap<>();
        for (String line : Files.readAllLines(new File(System.getProperty("fr.baseline")).toPath())) {
            JsonNode n = JSON.readTree(line);
            baseline.put(n.get("route").asLong(), line);
        }
        int baselineMismatch = 0, violations = 0, replays = 0, replayMismatch = 0;
        Set<String> pass1 = new TreeSet<>();
        Set<Long> pass2 = new TreeSet<>();
        for (long id : FixRouteFixtures.allIds()) {
            FixRouteFixtures.Route r = FixRouteFixtures.load(id);
            FixRouteRequest q = FixRouteCorpusTest.request(r);
            q.barriers = null;
            FixRouteResponse a1 = FixRouteEngine.fix(hopper, pool, q);
            FixRouteResponse a2 = FixRouteEngine.fix(hopper, pool, FixRouteCorpusTest.applyAndReroute(q, a1));
            String line = "{\"route\":" + id + ",\"r1\":" + FixRouteCorpusTest.dump(a1) + ",\"r2\":" + FixRouteCorpusTest.dump(a2) + "}";
            if (!line.equals(baseline.get(id))) {
                baselineMismatch++;
                String b = baseline.get(id) == null ? "" : baseline.get(id);
                int k = 0;
                while (k < Math.min(b.length(), line.length()) && b.charAt(k) == line.charAt(k)) k++;
                System.out.println("FRD no-flag differs from the baseline: route " + id + " at char " + k + ": baseline …"
                        + b.substring(Math.max(0, k - 80), Math.min(b.length(), k + 80)) + "… now …" + line.substring(Math.max(0, k - 80), Math.min(line.length(), k + 80)) + "…");
            }
            FixRouteRequest qf = FixRouteCorpusTest.request(r);
            qf.barriers = "v1";
            FixRouteResponse f1 = FixRouteEngine.fix(hopper, pool, qf);
            FixRouteRequest qf2 = FixRouteCorpusTest.applyAndReroute(qf, f1);
            FixRouteResponse f2 = FixRouteEngine.fix(hopper, pool, qf2);
            List<String> la = legs(FixRouteCorpusTest.dump(a1), true), lf = legs(FixRouteCorpusTest.dump(f1), true);
            for (int i = 0; i < la.size(); i++) if (!la.get(i).equals(lf.get(i))) pass1.add(id + ":" + i);
            if (!legs(FixRouteCorpusTest.dump(a2), true).equals(legs(FixRouteCorpusTest.dump(f2), true))) pass2.add(id);
            for (FixRouteResponse.Leg l : f2.legs) {
                boolean ok = l.status.equals(FixRouteResponse.OK) || l.status.equals(FixRouteResponse.SKIPPED)
                        || l.status.equals(FixRouteResponse.UNALIGNED) || l.status.equals(FixRouteResponse.NO_ROUTE)
                        || (l.status.equals(FixRouteResponse.UNROUTABLE) && l.metrics.addedWaypoints == 0);
                if (!ok) violations++;
            }
            // the routeAsApp retry vs plain /route + flag, edge_key sequence
            for (int i = 0; i < qf.segments.size(); i++) {
                FixRouteRequest.Segment s = qf.segments.get(i);
                if (!s.isFollowRoads()) continue;
                FixRouteRequest.Waypoint wa = qf.waypoints.get(i), wb = qf.waypoints.get(i + 1);
                GHPoint pa = new GHPoint(wa.coordinates.lat, wa.coordinates.lng), pb = new GHPoint(wb.coordinates.lat, wb.coordinates.lng);
                List<GHPoint> via = new ArrayList<>();
                for (FixRouteRequest.LatLng v : s.viaPoints) via.add(new GHPoint(v.lat, v.lng));
                double pen = s.headingPenalty != null ? s.headingPenalty : MatchPipelineFixer.CLIENT_HEADING_PENALTY;
                ClientRoute.Settings st = new ClientRoute.Settings(s.profile, s.customModel, qf.snapPreventions, true);
                if (ClientRoute.route(hopper, st, pa, via, pb, s.initialHeading, pen).ok()) continue;
                ClientRoute.Leg app = ClientRoute.routeAsApp(hopper, st, pa, via, pb, s.initialHeading, pen);
                if (!app.ok()) continue;
                replays++;
                com.graphhopper.GHResponse plain = hopper.route(ClientRoute.request(st, pa, via, pb, s.initialHeading, pen));
                int[] keys = plain.hasErrors() ? null : com.graphhopper.trailmap.shared.EdgeKeyMatching.edgeKeysFromDetails(plain.getBest().getPathDetails().get("edge_key"));
                boolean same = Arrays.equals(keys, app.edgeKeys());
                if (!same) replayMismatch++;
                System.out.println("FRD retry " + id + ":" + i + (same ? " == /route+flag" : " DIFFERS from /route+flag") + ", " + app.edgeKeys().length + " edges");
            }
        }
        System.out.println("FRD no-flag vs baseline mismatches " + baselineMismatch + "; flag pass-1 differing legs " + pass1
                + "; pass-2 differing routes " + pass2 + "; pass-2 violations " + violations + "; retries " + replays + ", mismatches " + replayMismatch);
        pool.shutdown();
        hopper.close();
        assertEquals(0, baselineMismatch, "no flag must be byte-identical to the baseline");
        assertEquals(EXPECTED_PASS1, pass1, "legs that differ with the flag (pass 1, probes ignored)");
        assertEquals(EXPECTED_PASS2_ROUTES, pass2, "routes that differ with the flag (pass 2, probes ignored)");
        assertEquals(0, violations, "flag pass-2 violations");
        assertTrue(replays > 0 && replayMismatch == 0, "routeAsApp retries must equal /route + flag");
    }
}
