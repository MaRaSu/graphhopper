package com.graphhopper.resources;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.graphhopper.GHRequest;
import com.graphhopper.GraphHopper;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.http.ProfileResolver;
import com.graphhopper.jackson.Jackson;
import jakarta.ws.rs.core.Response;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;

import java.io.File;
import java.util.*;

import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

/**
 * {@code POST /personal_route} with {@code algorithm=bit_of_new}, in-JVM on the experiment's graph clone and the 30
 * round-3 legs: the main route is exactly the plain request's route (distance, time, weight, points, details), the
 * variations follow it with their source and deviations, history avoid/explore
 * work, and bad requests are refused. Also prints the endpoint timing (took and the variations' own share).
 *
 * <p>Runs only with the data given as system properties (skipped when unset): {@code -Dbon.root=ABS/routing_v2
 * [-Dbon.graph=ABS/graphClone] [-Dbon.prod_config=ABS/trailmap_prod_config.yml]}; the user's cells come from the
 * config's {@code personal_routing.cells_dir}.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
public class TrailmapBitOfNewResourceTest {

    static final String ROOT = System.getProperty("bon.root", "");
    static final String GRAPH = System.getProperty("bon.graph", ROOT + "/data/experiments/personal_routing/integ/graph");
    static final String LEGS = ROOT + "/data/personal-routing/r3/legs.json";

    GraphHopper hopper;
    TrailmapPersonalRouteResource resource;
    final ObjectMapper om = Jackson.newObjectMapper();
    JsonNode legs;

    @BeforeAll
    void setUp() throws Exception {
        assumeTrue(new File(GRAPH, "properties").exists() && new File(LEGS).exists(), "graph or legs not here");
        ObjectMapper yaml = Jackson.initObjectMapper(new ObjectMapper(new YAMLFactory()));
        // the repo's trailmap-config.yml: ../ of the module when forked, else found from the compiled test classes
        File cfg = new File("../trailmap-config.yml");
        if (!cfg.exists())
            cfg = new File(new File(getClass().getProtectionDomain().getCodeSource().getLocation().toURI()), "../../../trailmap-config.yml");
        GraphHopperConfig config = yaml.treeToValue(yaml.readTree(cfg).get("graphhopper"), GraphHopperConfig.class);
        config.putObject("graph.location", GRAPH);
        config.putObject("graph.elevation.cache_dir", ROOT + "/data/loop/elevation");
        config.putObject("datareader.file", "");
        hopper = new GraphHopper();
        hopper.init(config);
        hopper.setAllowWrites(false);
        assertTrue(hopper.load(), "graph did not load");
        resource = new TrailmapPersonalRouteResource(config, hopper, new ProfileResolver(hopper.getProfiles()), r -> r);
        legs = om.readTree(new File(LEGS));
    }

    @AfterAll
    void tearDown() {
        if (hopper != null) hopper.close();
    }

    ObjectNode post(JsonNode leg, Map<String, Object> extra) throws Exception {
        Map<String, Object> body = new LinkedHashMap<>();
        // legs.json holds [lat, lon]; the API takes [lon, lat]
        body.put("points", List.of(List.of(leg.get("from").get(1).asDouble(), leg.get("from").get(0).asDouble()),
                List.of(leg.get("to").get(1).asDouble(), leg.get("to").get(0).asDouble())));
        body.put("profile", "gravel");
        body.put("points_encoded", false);
        body.put("instructions", false);
        body.put("personal_mode", "favor");
        body.put("personal_user", "owner");
        body.put("personal_cells", "cycling_lifetime");
        body.put("personal_strength", 1.0);
        body.put("personal_switch_penalty", 30);
        body.put("personal_rides_saturation", 10);
        body.putAll(extra);
        GHRequest req = om.readValue(om.writeValueAsString(body), GHRequest.class);
        Response r = resource.doPost(req, null);
        assertEquals(200, r.getStatus());
        // through JSON text, as a client sees it (points and hints are POJOs in the response object)
        return (ObjectNode) om.readTree(om.writeValueAsString(r.getEntity()));
    }

    static List<String> coords(JsonNode path) {
        List<String> out = new ArrayList<>();
        for (JsonNode c : path.get("points").get("coordinates")) out.add(c.get(0).asText() + "," + c.get(1).asText());
        return out;
    }

    static long pct(List<Long> xs, double q) {
        List<Long> s = new ArrayList<>(xs);
        Collections.sort(s);
        return s.get((int) Math.ceil(q * s.size()) - 1);
    }

    @Test
    void mainIsThePlainRouteAndVariationsFollow() throws Exception {
        List<Long> took = new ArrayList<>(), varMs = new ArrayList<>();
        int legsDone = 0, alts = 0, identicalPoints = 0;
        Map<String, Integer> bySource = new TreeMap<>();
        for (int pass = 0; pass < 2; pass++)
            for (JsonNode leg : legs) {
                ObjectNode plain = post(leg, Map.of());
                ObjectNode bon = post(leg, Map.of("algorithm", "bit_of_new"));
                if (pass == 0) continue;
                legsDone++;
                String id = leg.get("id").asText();
                JsonNode p0 = plain.get("paths").get(0), b0 = bon.get("paths").get(0);
                assertEquals(1, plain.get("paths").size(), id);
                assertNull(p0.get("variation_source"), id + ": plain request unchanged");
                assertEquals(p0.get("distance").asDouble(), b0.get("distance").asDouble(), 0, id);
                assertEquals(p0.get("time").asLong(), b0.get("time").asLong(), id);
                assertEquals(p0.get("weight").asDouble(), b0.get("weight").asDouble(), 0, id);
                List<String> pc = coords(p0), bc = coords(b0);
                assertEquals(pc, bc, id + ": main route points exactly as the plain request's");
                assertEquals(om.writeValueAsString(p0.get("details")), om.writeValueAsString(b0.get("details")), id);
                if (pc.equals(bc)) identicalPoints++;
                assertFalse(b0.get("details").has("edge_id"), id + ": edge_id detail not leaked");
                assertEquals("main", b0.get("variation_source").asText(), id);
                JsonNode info = bon.get("hints").get("bit_of_new");
                assertNotNull(info, id);
                int n4 = info.get("round4").asInt(), na = info.get("a3").asInt(), dup = info.get("a3_dropped_as_duplicate").asInt();
                assertEquals(1 + n4 + na - dup, bon.get("paths").size(), id);
                String prev = "main";
                for (int k = 1; k < bon.get("paths").size(); k++) {
                    JsonNode p = bon.get("paths").get(k);
                    String src = p.get("variation_source").asText();
                    assertTrue(src.equals("round4") || src.equals("a3"), id);
                    assertFalse(prev.equals("a3") && src.equals("round4"), id + ": round 4 first, then A3");
                    prev = src;
                    bySource.merge(src, 1, Integer::sum);
                    assertTrue(p.get("variation").get("deviations").size() >= 1, id);
                    assertEquals(p.get("distance").asDouble(), p.get("variation").get("length_m").asDouble(), 1, id);
                    List<String> c = coords(p);
                    assertEquals(bc.get(0), c.get(0), id + ": starts at A");
                    assertEquals(bc.get(bc.size() - 1), c.get(c.size() - 1), id + ": ends at B");
                    alts++;
                }
                took.add(bon.get("info").get("took").asLong());
                varMs.add(info.get("ms_total").asLong());
            }
        System.out.printf(Locale.ROOT, "SUMMARY endpoint none: %d legs, %d alternatives %s, main points identical to the plain request on %d/%d legs, took p50 %d p95 %d ms, variations alone p50 %d p95 %d ms%n",
                legsDone, alts, bySource, identicalPoints, legsDone, pct(took, 0.5), pct(took, 0.95), pct(varMs, 0.5), pct(varMs, 0.95));
    }

    /**
     * Through the resource, the variations are exactly the prototypes' (round 4 = r4 i3's alternatives, A3 = r3a i5's
     * minus duplicates, in order), whatever the client's own instructions flag and path details. The prototypes routed
     * without snap preventions, so this asks for none; with the config default (tunnel, bridge, ferry) the main route
     * of l4 and l12 starts differently and so do their variations.
     */
    @Test
    void variationsAreThePrototypesWhateverTheClientAsks() throws Exception {
        String exp = ROOT + "/data/experiments/personal_routing/";
        assumeTrue(new File(exp + "r4/runs/i3/l1.json").exists() && new File(exp + "r3a/runs/i5/l1.json").exists(), "prototype runs not here");
        List<Map<String, Object>> clients = List.of(Map.of("instructions", false), Map.of("instructions", true),
                Map.of("details", List.of("road_class", "road_environment")));
        for (Map<String, Object> client : clients)
            for (JsonNode leg : legs) {
                String id = leg.get("id").asText() + " " + client;
                Map<String, Object> extra = new LinkedHashMap<>(client);
                extra.put("algorithm", "bit_of_new");
                extra.put("snap_preventions", List.of());
                ObjectNode bon = post(leg, extra);
                List<JsonNode> got4 = new ArrayList<>(), gotA = new ArrayList<>();
                for (int k = 1; k < bon.get("paths").size(); k++) {
                    JsonNode p = bon.get("paths").get(k);
                    (p.get("variation_source").asText().equals("round4") ? got4 : gotA).add(p.get("points").get("coordinates"));
                }
                JsonNode exp4 = om.readTree(new File(exp + "r4/runs/i3/" + leg.get("id").asText() + ".json")).get("alts");
                JsonNode expA = om.readTree(new File(exp + "r3a/runs/i5/" + leg.get("id").asText() + ".json")).get("alts");
                assertEquals(exp4.size(), got4.size(), id + ": round 4 count");
                for (int k = 0; k < got4.size(); k++)
                    assertTrue(sameLine(got4.get(k), exp4.get(k)), id + ": round 4 alternative " + (k + 1) + " (points "
                            + got4.get(k).size() + " vs " + exp4.get(k).size() + ", first " + got4.get(k).get(0) + " vs " + exp4.get(k).get(0) + ")");
                int j = 0;
                for (JsonNode g : gotA) {
                    while (j < expA.size() && !sameLine(g, expA.get(j))) j++;
                    assertTrue(j < expA.size(), id + ": A3 alternative not one of the prototype's, in order");
                    j++;
                }
            }
    }

    static boolean sameLine(JsonNode a, JsonNode b) {
        if (a.size() != b.size()) return false;
        for (int i = 0; i < a.size(); i++)
            if (Math.abs(a.get(i).get(0).asDouble() - b.get(i).get(0).asDouble()) > 1e-6
                    || Math.abs(a.get(i).get(1).asDouble() - b.get(i).get(1).asDouble()) > 1e-6)
                return false;
        return true;
    }

    @Test
    void historyFlavours() throws Exception {
        JsonNode leg = legs.get(0);
        for (String h : List.of("avoid", "explore")) {
            ObjectNode bon = post(leg, Map.of("algorithm", "bit_of_new", "bit_of_new_history", h));
            assertEquals(h, bon.get("hints").get("bit_of_new").get("history").asText());
            assertTrue(bon.get("paths").size() >= 2, h);
        }
        // a client that asks for edge ids itself gets them on the main route, as with the plain request
        ObjectNode plain = post(leg, Map.of("details", List.of("edge_id")));
        ObjectNode bon = post(leg, Map.of("algorithm", "bit_of_new", "details", List.of("edge_id")));
        assertEquals(om.writeValueAsString(plain.get("paths").get(0).get("details")), om.writeValueAsString(bon.get("paths").get(0).get("details")));
        assertEquals(coords(plain.get("paths").get(0)), coords(bon.get("paths").get(0)));
        assertTrue(bon.get("paths").size() >= 2);
    }

    /**
     * The variations follow the request's custom_model as the main route does: a small area blocked by the custom
     * model, put on the point of a variation farthest from the main route, is not used by any variation any more.
     */
    @Test
    void variationsFollowTheRequestsCustomModel() throws Exception {
        int checked = 0;
        for (JsonNode leg : legs) {
            ObjectNode bon = post(leg, Map.of("algorithm", "bit_of_new"));
            if (bon.get("paths").size() < 2) continue;
            List<String> main = coords(bon.get("paths").get(0));
            JsonNode alt = bon.get("paths").get(1).get("points").get("coordinates");
            double bestD = -1, lon = 0, lat = 0;
            for (JsonNode c : alt) {
                double d = Double.MAX_VALUE;
                for (String m : main) {
                    String[] p = m.split(",");
                    d = Math.min(d, Math.hypot(c.get(0).asDouble() - Double.parseDouble(p[0]), c.get(1).asDouble() - Double.parseDouble(p[1])));
                }
                if (d > bestD) {
                    bestD = d;
                    lon = c.get(0).asDouble();
                    lat = c.get(1).asDouble();
                }
            }
            if (bestD < 0.003) continue; // the variation must leave the main route clearly (~200 m)
            double r = 0.0006;
            List<List<Double>> ring = List.of(List.of(lon - r, lat - r), List.of(lon + r, lat - r), List.of(lon + r, lat + r),
                    List.of(lon - r, lat + r), List.of(lon - r, lat - r));
            Map<String, Object> cm = Map.of(
                    "areas", Map.of("type", "FeatureCollection", "features", List.of(Map.of("type", "Feature", "id", "blk",
                            "properties", Map.of(), "geometry", Map.of("type", "Polygon", "coordinates", List.of(ring))))),
                    "priority", List.of(Map.of("if", "in_blk", "multiply_by", "0")));
            ObjectNode with = post(leg, Map.of("algorithm", "bit_of_new", "custom_model", cm, "ch.disable", true));
            String id = leg.get("id").asText();
            for (int k = 1; k < with.get("paths").size(); k++)
                for (JsonNode c : with.get("paths").get(k).get("points").get("coordinates"))
                    assertFalse(Math.abs(c.get(0).asDouble() - lon) < r && Math.abs(c.get(1).asDouble() - lat) < r,
                            id + ": variation " + k + " goes through the area the custom model blocks");
            if (++checked == 5) break;
        }
        assertTrue(checked >= 3, "too few legs with a clear variation: " + checked);
    }

    @Test
    void tooLongRouteGetsOnlyTheMainRoute() throws Exception {
        JsonNode leg = legs.get(0);
        ObjectNode plain = post(leg, Map.of());
        ObjectNode bon = post(leg, Map.of("algorithm", "bit_of_new", "bit_of_new_max_km", 1));
        assertEquals(1, bon.get("paths").size());
        assertEquals("main", bon.get("paths").get(0).get("variation_source").asText());
        assertEquals(coords(plain.get("paths").get(0)), coords(bon.get("paths").get(0)));
        assertTrue(bon.get("hints").get("bit_of_new").get("skipped").asText().contains("longer than"));
        assertThrows(IllegalArgumentException.class, () -> post(leg, Map.of("algorithm", "bit_of_new", "bit_of_new_max_km", 500)));
    }

    @Test
    void userWithoutTheHistoryDataSetGetsOnlyTheMainRoute() throws Exception {
        JsonNode leg = legs.get(0);
        for (String h : List.of("avoid", "explore")) {
            ObjectNode plain = post(leg, Map.of("personal_user", "nobody_here"));
            ObjectNode bon = post(leg, Map.of("algorithm", "bit_of_new", "bit_of_new_history", h, "personal_user", "nobody_here"));
            assertEquals(1, bon.get("paths").size(), h);
            assertEquals(coords(plain.get("paths").get(0)), coords(bon.get("paths").get(0)), h);
            assertTrue(bon.get("hints").get("bit_of_new").get("skipped").asText().contains("data set"), h);
        }
    }

    /**
     * The production config: cells from the activity service, no local files, no previewer. Without a token nothing
     * personal is served (the body's personal_user is ignored, personal modes are 401), plain requests route as
     * /route, and the previewer's GET endpoints are 404.
     */
    @Test
    void productionConfigIsTokenOnly() throws Exception {
        File prod = new File(System.getProperty("bon.prod_config", ROOT + "/gh-src/trailmap_prod_config.yml"));
        assumeTrue(prod.exists(), "production config not here");
        ObjectMapper yaml = Jackson.initObjectMapper(new ObjectMapper(new YAMLFactory()));
        GraphHopperConfig prodConfig = yaml.treeToValue(yaml.readTree(prod).get("graphhopper"), GraphHopperConfig.class);
        assertEquals("", prodConfig.getString("personal_routing.cells_dir", ""), "no local cells in production");
        assertFalse(prodConfig.getString("personal_routing.cells_source", "").isEmpty(), "production fetches cells");
        assertFalse(prodConfig.getBool("personal_routing.previewer", false), "no previewer in production");
        TrailmapPersonalRouteResource prodRes = new TrailmapPersonalRouteResource(prodConfig, hopper, new ProfileResolver(hopper.getProfiles()), r -> r);
        JsonNode leg = legs.get(0);
        Map<String, Object> body = new LinkedHashMap<>();
        body.put("points", List.of(List.of(leg.get("from").get(1).asDouble(), leg.get("from").get(0).asDouble()),
                List.of(leg.get("to").get(1).asDouble(), leg.get("to").get(0).asDouble())));
        body.put("profile", "gravel");
        body.put("points_encoded", false);
        body.put("instructions", false);
        for (Map<String, Object> extra : List.<Map<String, Object>>of(Map.of("personal_mode", "favor", "personal_user", "owner", "personal_cells", "cycling_lifetime"),
                Map.of("algorithm", "bit_of_new", "bit_of_new_history", "avoid", "personal_user", "owner"))) {
            Map<String, Object> b = new LinkedHashMap<>(body);
            b.putAll(extra);
            GHRequest req = om.readValue(om.writeValueAsString(b), GHRequest.class);
            jakarta.ws.rs.WebApplicationException e = assertThrows(jakarta.ws.rs.WebApplicationException.class, () -> prodRes.doPost(req, null), extra.toString());
            assertEquals(401, e.getResponse().getStatus(), extra.toString());
        }
        // neutral without a token: exactly the route the previewer-mode resource gives for a plain request
        Map<String, Object> b = new LinkedHashMap<>(body);
        b.put("personal_user", "owner");
        Response r = prodRes.doPost(om.readValue(om.writeValueAsString(b), GHRequest.class), null);
        ObjectNode got = (ObjectNode) om.readTree(om.writeValueAsString(r.getEntity()));
        assertFalse(got.get("hints").get("personal").get("active").asBoolean());
        ObjectNode plain = post(leg, Map.of("personal_mode", "neutral"));
        assertEquals(coords(plain.get("paths").get(0)), coords(got.get("paths").get(0)));
        assertThrows(jakarta.ws.rs.NotFoundException.class, () -> prodRes.dataSets("owner"));
        assertThrows(jakarta.ws.rs.NotFoundException.class, () -> prodRes.cells("owner", "cycling_lifetime", "23,61,24,62"));
    }

    @Test
    void badRequests() {
        JsonNode leg = legs.get(0);
        assertThrows(IllegalArgumentException.class, () -> post(leg, Map.of("algorithm", "bit_of_new", "bit_of_new_history", "sideways")));
        Map<String, Object> three = new HashMap<>();
        three.put("algorithm", "bit_of_new");
        three.put("points", List.of(List.of(leg.get("from").get(1).asDouble(), leg.get("from").get(0).asDouble()),
                List.of(leg.get("to").get(1).asDouble(), leg.get("to").get(0).asDouble()),
                List.of(leg.get("from").get(1).asDouble(), leg.get("from").get(0).asDouble())));
        assertThrows(IllegalArgumentException.class, () -> post(leg, three));
    }
}
