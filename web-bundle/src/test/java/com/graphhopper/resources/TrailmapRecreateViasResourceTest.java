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
import com.graphhopper.jackson.ResponsePathDeserializerHelper;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.PointList;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.ws.rs.core.Response;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;

import java.io.File;
import java.lang.reflect.Proxy;
import java.nio.file.Files;
import java.util.*;

import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

/**
 * Re-create via points (W20): every path {@code POST /personal_route} generates must come back from plain
 * {@code POST /route} [A, ...via_points, B] sent the way the app rebuilds a saved segment (CATALOG §1 "Re-create by
 * plain /route"). In-JVM on a graph clone: the shown request through {@link TrailmapPersonalRouteResource#doPost},
 * the replay through {@link RouteResource#doPost} with the same JSON body minus everything personal or generating,
 * via_points decoded from the polyline at the response's multiplier, headings [h, NaN, ...]. Pass: the collapsed
 * edge_id sequence is identical, or the path is flagged in {@code hints.personal.recreate} with the replay's own
 * missing metres. Unflagged misses fail.
 *
 * <p>Runs only with {@code -Dbon.root=ABS/routing_v2 [-Dbon.graph=ABS/graphClone]} (skipped when unset), cells from
 * the config's {@code personal_routing.cells_dir}. {@code -Drv.all=true} runs every leg × mode × algorithm and prints
 * vias per path and the pinning time; {@code -Drv.out=FILE} writes the per-path results as JSON.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
public class TrailmapRecreateViasResourceTest {

    static final String ROOT = System.getProperty("bon.root", "");
    static final String GRAPH = System.getProperty("bon.graph", ROOT + "/data/experiments/personal_routing/w20/graph");
    static final String LEGS = ROOT + "/data/personal-routing/r3/legs.json";
    static final double MULT = 1e6;

    GraphHopper hopper;
    TrailmapPersonalRouteResource personal;
    RouteResource route;
    final ObjectMapper om = Jackson.newObjectMapper();
    JsonNode legs, pairs;
    final List<Map<String, Object>> results = new ArrayList<>();

    GraphHopperConfig config() throws Exception {
        ObjectMapper yaml = Jackson.initObjectMapper(new ObjectMapper(new YAMLFactory()));
        File cfg = new File("../trailmap-config.yml");
        if (!cfg.exists())
            cfg = new File(new File(getClass().getProtectionDomain().getCodeSource().getLocation().toURI()), "../../../trailmap-config.yml");
        GraphHopperConfig config = yaml.treeToValue(yaml.readTree(cfg).get("graphhopper"), GraphHopperConfig.class);
        config.putObject("graph.location", GRAPH);
        config.putObject("graph.elevation.cache_dir", ROOT + "/data/loop/elevation");
        config.putObject("datareader.file", "");
        return config;
    }

    /** The same resource with re-create via points off (max vias 0): what /personal_route answered before them. */
    TrailmapPersonalRouteResource withoutVias;

    @BeforeAll
    void setUp() throws Exception {
        assumeTrue(new File(GRAPH, "properties").exists() && new File(LEGS).exists(), "graph or legs not here");
        GraphHopperConfig config = config();
        // the levers, for measuring (defaults: the server's)
        if (System.getProperty("rv.max_vias") != null)
            config.putObject("personal_routing.recreate_max_vias", Integer.getInteger("rv.max_vias"));
        if (System.getProperty("rv.budget_ms") != null)
            config.putObject("personal_routing.recreate_budget_ms", Long.getLong("rv.budget_ms"));
        hopper = new GraphHopper();
        hopper.init(config);
        hopper.setAllowWrites(false);
        assertTrue(hopper.load(), "graph did not load");
        ProfileResolver pr = new ProfileResolver(hopper.getProfiles());
        personal = new TrailmapPersonalRouteResource(config, hopper, pr, r -> r);
        GraphHopperConfig off = config();
        off.putObject("personal_routing.recreate_max_vias", 0);
        withoutVias = new TrailmapPersonalRouteResource(off, hopper, pr, r -> r);
        route = new RouteResource(config, hopper, pr, r -> r, true);
        legs = om.readTree(new File(LEGS));
        pairs = om.readTree(new File(ROOT + "/route-check/pairs.json"));
    }

    @AfterAll
    void tearDown() throws Exception {
        String out = System.getProperty("rv.out", "");
        if (!out.isEmpty() && !results.isEmpty())
            Files.writeString(new File(out).toPath(), om.writerWithDefaultPrettyPrinter().writeValueAsString(results));
        if (hopper != null) hopper.close();
    }

    /** The app's /route body for this leg (gh-routing-service.ts getRouteOutcome), details edge_id only. */
    Map<String, Object> appBody(JsonNode leg, String profile) {
        Map<String, Object> body = new LinkedHashMap<>();
        // legs.json holds [lat, lon]; the API takes [lon, lat]
        body.put("points", new ArrayList<>(List.of(List.of(leg.get("from").get(1).asDouble(), leg.get("from").get(0).asDouble()),
                List.of(leg.get("to").get(1).asDouble(), leg.get("to").get(0).asDouble()))));
        body.put("profile", profile);
        body.put("elevation", true);
        body.put("instructions", false);
        body.put("locale", "fi_FI");
        body.put("points_encoded", true);
        body.put("points_encoded_multiplier", MULT);
        body.put("details", List.of("edge_id"));
        body.put("snap_preventions", List.of("ferry"));
        return body;
    }

    static Map<String, Object> personalFields(String mode) {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("personal_mode", mode);
        m.put("personal_user", "owner");
        m.put("personal_cells", "cycling_lifetime");
        m.put("personal_strength", 1.0);
        m.put("personal_switch_penalty", 30);
        m.put("personal_rides_saturation", 10);
        return m;
    }

    static GHRequest request(ObjectMapper om, Map<String, Object> body, Double heading, int nPoints) throws Exception {
        GHRequest req = om.readValue(om.writeValueAsString(body), GHRequest.class);
        if (heading != null) {
            List<Double> hs = new ArrayList<>();
            hs.add(heading);
            for (int i = 1; i < nPoints; i++) hs.add(Double.NaN);
            req.setHeadings(hs);
            req.putHint("heading_penalty", 120);
        }
        return req;
    }

    static final HttpServletRequest HTTP = (HttpServletRequest) Proxy.newProxyInstance(
            TrailmapRecreateViasResourceTest.class.getClassLoader(), new Class<?>[]{HttpServletRequest.class}, (p, m, a) -> null);

    static List<Integer> edges(JsonNode path) {
        List<Integer> out = new ArrayList<>();
        for (JsonNode d : path.get("details").get("edge_id")) {
            int e = d.get(2).asInt();
            if (out.isEmpty() || out.get(out.size() - 1) != e) out.add(e);
        }
        return out;
    }

    /** Metres per edge id along the path's decoded points (the replay tool's definition). */
    static Map<Integer, Double> metresByEdge(JsonNode path) {
        PointList pts = ResponsePathDeserializerHelper.decodePolyline(path.get("points").asText(), 100, true, MULT);
        Map<Integer, Double> out = new HashMap<>();
        for (JsonNode d : path.get("details").get("edge_id")) {
            double m = 0;
            for (int i = d.get(0).asInt(); i < d.get(1).asInt(); i++)
                m += DistanceCalcEarth.DIST_EARTH.calcDist(pts.getLat(i), pts.getLon(i), pts.getLat(i + 1), pts.getLon(i + 1));
            out.merge(d.get(2).asInt(), m, Double::sum);
        }
        return out;
    }

    /** Metres of a's edge ids that b does not ride at all. */
    static double missingM(JsonNode a, JsonNode b) {
        Map<Integer, Double> ma = metresByEdge(a), mb = metresByEdge(b);
        double m = 0;
        for (Map.Entry<Integer, Double> e : ma.entrySet()) if (!mb.containsKey(e.getKey())) m += e.getValue();
        return m;
    }

    static PointList vias(JsonNode path) {
        String vp = path.get("via_points").asText();
        return vp.isEmpty() ? new PointList() : ResponsePathDeserializerHelper.decodePolyline(vp, 4, true, MULT);
    }

    /**
     * The app's rebuild of a shown path: plain /route from the path's own first to last point (the app moves the
     * segment's waypoints there, route-members.ts:787-796) over its via_points, with the app's body.
     */
    JsonNode replay(JsonNode path, JsonNode leg, String profile, Double heading, Map<String, Object> extra, String label) throws Exception {
        PointList shownPts = ResponsePathDeserializerHelper.decodePolyline(path.get("points").asText(), 100, true, MULT);
        PointList vias = vias(path);
        Map<String, Object> replay = appBody(leg, profile);
        if (extra != null) replay.putAll(extra);
        List<Object> pts = new ArrayList<>();
        pts.add(List.of(shownPts.getLon(0), shownPts.getLat(0)));
        for (int i = 0; i < vias.size(); i++) pts.add(List.of(vias.getLon(i), vias.getLat(i)));
        pts.add(List.of(shownPts.getLon(shownPts.size() - 1), shownPts.getLat(shownPts.size() - 1)));
        replay.put("points", pts);
        Response rr = route.doPost(request(om, replay, heading, pts.size()), HTTP);
        assertEquals(200, rr.getStatus(), label);
        return om.readTree(om.writeValueAsString(rr.getEntity())).get("paths").get(0);
    }

    /**
     * W23: plain /route alternative_route, the app's body; every alternative replayed as the app rebuilds it. Prints
     * the misses; -Drv.out gets the rows. Gate (0 regressions vs the build before) is the 8999 replay; here: misses
     * listed, and a pass_through=true replay of re-created paths (signal).
     */
    @Test
    void routeAlternatives() throws Exception {
        assumeTrue(Boolean.getBoolean("rv.alt"), "-Drv.alt=true runs /route alternative_route");
        int n = 0, same = 0, sameThrough = 0;
        List<JsonNode> all = new ArrayList<>();
        pairs.forEach(all::add);
        legs.forEach(all::add);
        for (JsonNode leg : all)
            for (String profile : List.of("gravel", "gravel_no_tc")) {
                String label = leg.get("id").asText() + " " + profile + " /route alternatives";
                Map<String, Object> body = appBody(leg, profile);
                body.put("algorithm", "alternative_route");
                body.put("alternative_route.max_paths", 3);
                long t0 = System.nanoTime();
                Response r = route.doPost(request(om, body, null, 2), HTTP);
                long ms = (System.nanoTime() - t0) / 1_000_000;
                if (r.getStatus() != 200) continue;
                JsonNode shown = om.readTree(om.writeValueAsString(r.getEntity()));
                for (int k = 0; k < shown.get("paths").size(); k++) {
                    JsonNode p = shown.get("paths").get(k);
                    JsonNode rp = replay(p, leg, profile, null, null, label);
                    boolean ok = edges(p).equals(edges(rp));
                    n++;
                    if (ok) {
                        same++;
                        JsonNode through = replay(p, leg, profile, null, Map.of("pass_through", true), label);
                        if (edges(p).equals(edges(through))) sameThrough++;
                    } else
                        System.out.println("ALTMISS " + label + " path " + k + " vias " + vias(p).size() + " missing "
                                + Math.round(missingM(p, rp)) + " extra " + Math.round(missingM(rp, p)));
                    Map<String, Object> res = new LinkedHashMap<>();
                    res.put("case", label);
                    res.put("path", k);
                    res.put("source", "route_alternative");
                    res.put("identical", ok);
                    res.put("vias", vias(p).size());
                    res.put("via_points", p.get("via_points").asText());
                    res.put("missing_m", Math.round(missingM(p, rp)));
                    res.put("extra_m", Math.round(missingM(rp, p)));
                    res.put("flag", null);
                    res.put("took_ms", ms);
                    results.add(res);
                }
            }
        System.out.println("ROUTEALT paths " + n + " identical " + same + " pass_through identical " + sameThrough + " of " + same);
    }

    /** One shown request; every returned path replayed as the app would. Returns the number of unflagged misses. */
    int check(String label, JsonNode leg, String profile, String mode, String algorithm, Double heading) throws Exception {
        Map<String, Object> body = appBody(leg, profile);
        body.putAll(personalFields(mode));
        if (algorithm != null) body.put("algorithm", algorithm);
        Response r = personal.doPost(request(om, body, heading, 2), null);
        assertEquals(200, r.getStatus(), label);
        ObjectNode shown = (ObjectNode) om.readTree(om.writeValueAsString(r.getEntity()));
        JsonNode recreate = shown.get("hints").get("personal").get("recreate");
        int unflagged = 0;
        for (int k = 0; k < shown.get("paths").size(); k++) {
            JsonNode p = shown.get("paths").get(k);
            PointList vias = vias(p);
            JsonNode rp = replay(p, leg, profile, heading, null, label);
            List<Integer> want = edges(p), got = edges(rp);
            boolean same = want.equals(got);
            JsonNode flag = null;
            if (recreate != null)
                for (JsonNode f : recreate) if (f.get("path").asInt() == k) flag = f;
            Map<String, Object> res = new LinkedHashMap<>();
            res.put("case", label);
            res.put("path", k);
            res.put("source", p.has("variation_source") ? p.get("variation_source").asText() : k == 0 ? "main" : "alternative");
            res.put("identical", same);
            res.put("vias", vias.size());
            res.put("missing_m", Math.round(missingM(p, rp)));
            res.put("extra_m", Math.round(missingM(rp, p)));
            res.put("flag", flag == null ? null : om.convertValue(flag, Map.class));
            res.put("took_ms", shown.get("info").get("took").asLong());
            results.add(res);
            if (same) {
                assertTrue(flag == null || flag.get("identical").asBoolean(), label + " path " + k + ": identical but flagged as not");
                continue;
            }
            if (flag == null || flag.get("identical").asBoolean()) {
                unflagged++;
                System.out.println("UNFLAGGED " + label + " path " + k + " vias " + vias.size() + " missing " + res.get("missing_m"));
            } else if (!flag.has("missing_m")) {
                // not pinned (too long, or out of time before it started): flagged, no metres claimed
                assertTrue(Set.of("too long", "time budget").contains(flag.get("stopped").asText()), label + " path " + k);
            } else {
                assertEquals(Math.round(missingM(p, rp)), flag.get("missing_m").asLong(), 1, label + " path " + k + ": flagged missing metres");
                assertEquals(Math.round(missingM(rp, p)), flag.get("extra_m").asLong(), 1, label + " path " + k + ": flagged extra metres");
            }
        }
        return unflagged;
    }

    @Test
    void sixCases() throws Exception {
        JsonNode l1 = legs.get(0), l10 = legs.get(9), l16 = legs.get(15);
        int unflagged = 0;
        unflagged += check("l1 gravel favor", l1, "gravel", "favor", null, null);
        unflagged += check("l10 gravel_no_tc avoid", l10, "gravel_no_tc", "avoid", null, null);
        unflagged += check("l16 gravel explore alternatives", l16, "gravel", "explore", "alternative_route", null);
        unflagged += check("l1 gravel favor bit_of_new", l1, "gravel", "favor", "bit_of_new", null);
        unflagged += check("l10 gravel_no_tc favor bit_of_new avoid", l10, "gravel_no_tc", "favor", "bit_of_new", null);
        unflagged += check("l16 gravel favor heading 90", l16, "gravel", "favor", null, 90.0);
        unflagged += check("l16 gravel favor heading 270", l16, "gravel", "favor", null, 270.0);
        // plain /route alternative_route does not re-create these paths from its own vias (test lead, 8999)
        for (JsonNode pair : pairs)
            if (Set.of("home-m3a", "espoo-hml13").contains(pair.get("id").asText()))
                unflagged += check(pair.get("id").asText() + " gravel favor alternatives", pair, "gravel", "favor", "alternative_route", null);
        assertEquals(0, unflagged, "paths plain /route does not re-create and the response does not flag");
    }

    /**
     * Second gate: the vias change nothing else. The app's own body (its default details, no edge_id) answers the same
     * with and without them, except via_points, hints.personal.recreate and the timings.
     */
    @Test
    void shownPathsUnchanged() throws Exception {
        for (int i : new int[]{0, 9, 15})
            for (String alg : Arrays.asList(null, "alternative_route", "bit_of_new")) {
                JsonNode leg = legs.get(i);
                Map<String, Object> body = appBody(leg, "gravel");
                body.put("details", List.of("road_class", "road_environment", "predicted_surface", "gravel_scale"));
                body.putAll(personalFields("favor"));
                if (alg != null) body.put("algorithm", alg);
                ObjectNode on = (ObjectNode) om.readTree(om.writeValueAsString(personal.doPost(request(om, body, null, 2), null).getEntity()));
                ObjectNode off = (ObjectNode) om.readTree(om.writeValueAsString(withoutVias.doPost(request(om, body, null, 2), null).getEntity()));
                String label = leg.get("id").asText() + " " + alg;
                assertNotNull(on.get("hints").get("personal").get("recreate"), label + ": pinned");
                System.out.println("TOOK " + label + " with vias " + on.get("info").get("took").asLong() + " ms, without "
                        + off.get("info").get("took").asLong() + " ms");
                for (ObjectNode o : List.of(on, off)) {
                    o.remove("info");
                    ((ObjectNode) o.get("hints").get("personal")).remove(List.of("recreate", "load_ms", "cached", "edges_measured_now", "edges_measured_total"));
                    if (o.get("hints").has("bit_of_new"))
                        ((ObjectNode) o.get("hints").get("bit_of_new")).remove(List.of("ms_round4", "ms_a3", "ms_total"));
                    for (JsonNode p : o.get("paths")) ((ObjectNode) p).remove("via_points");
                }
                assertEquals(om.writeValueAsString(off), om.writeValueAsString(on), label);
            }
    }

    /**
     * Long personal alternatives (the app's own body: its details, no edge_id) answer well inside the app's 15 s:
     * paths over the length limit are not pinned and the alternatives need no second search. Shown points equal the
     * answer without vias.
     */
    @Test
    void longPersonalAlternativesFinishInTime() throws Exception {
        for (JsonNode pair : pairs) {
            if (!Set.of("espoo-hml13", "hml7-espoo").contains(pair.get("id").asText())) continue;
            Map<String, Object> body = appBody(pair, "gravel");
            body.put("details", List.of("road_class", "road_environment", "predicted_surface", "gravel_scale"));
            body.putAll(personalFields("favor"));
            body.put("algorithm", "alternative_route");
            long t0 = System.nanoTime();
            ObjectNode on = (ObjectNode) om.readTree(om.writeValueAsString(personal.doPost(request(om, body, null, 2), null).getEntity()));
            long ms = (System.nanoTime() - t0) / 1_000_000;
            long t1 = System.nanoTime();
            ObjectNode off = (ObjectNode) om.readTree(om.writeValueAsString(withoutVias.doPost(request(om, body, null, 2), null).getEntity()));
            long msOff = (System.nanoTime() - t1) / 1_000_000;
            System.out.println("LONG " + pair.get("id").asText() + " with vias " + ms + " ms, without " + msOff + " ms, recreate "
                    + on.get("hints").get("personal").get("recreate"));
            assertTrue(ms < 10_000, pair.get("id").asText() + ": " + ms + " ms");
            for (int k = 0; k < on.get("paths").size(); k++)
                assertEquals(off.get("paths").get(k).get("points").asText(), on.get("paths").get(k).get("points").asText(),
                        pair.get("id").asText() + " path " + k);
        }
    }

    /** A user without the data set gets /route's alternatives, with /route's (W23) via points. */
    @Test
    void noHistoryAlternativesAreRoutesOwn() throws Exception {
        for (int i : new int[]{0, 15, 24}) {
            Map<String, Object> body = appBody(legs.get(i), "gravel");
            body.put("algorithm", "alternative_route");
            JsonNode plain = om.readTree(om.writeValueAsString(route.doPost(request(om, body, null, 2), HTTP).getEntity()));
            body.putAll(personalFields("favor"));
            body.put("personal_cells", "no_such_set");
            JsonNode pers = om.readTree(om.writeValueAsString(personal.doPost(request(om, body, null, 2), null).getEntity()));
            assertEquals(plain.get("paths").size(), pers.get("paths").size(), "leg " + i);
            for (int k = 0; k < plain.get("paths").size(); k++)
                assertEquals(plain.get("paths").get(k).get("via_points").asText(), pers.get("paths").get(k).get("via_points").asText(),
                        "leg " + i + " path " + k);
        }
    }

    /** Out of time before anything is pinned: an alternative keeps its own vias, never falls back to the main route. */
    @Test
    void alternativesKeepTheirViasWhenTheBudgetIsGone() throws Exception {
        GraphHopperConfig noTime = config();
        noTime.putObject("personal_routing.recreate_budget_ms", 0);
        TrailmapPersonalRouteResource r = new TrailmapPersonalRouteResource(noTime, hopper, new ProfileResolver(hopper.getProfiles()), x -> x);
        Map<String, Object> body = appBody(legs.get(15), "gravel");
        body.putAll(personalFields("explore"));
        body.put("algorithm", "alternative_route");
        JsonNode rsp = om.readTree(om.writeValueAsString(r.doPost(request(om, body, null, 2), null).getEntity()));
        assertTrue(rsp.get("paths").size() > 1, "l16 has a personal alternative");
        for (int k = 1; k < rsp.get("paths").size(); k++)
            assertFalse(rsp.get("paths").get(k).get("via_points").asText().isEmpty(), "alternative " + k + " keeps its vias");
    }

    @Test
    void allLegs() throws Exception {
        assumeTrue(Boolean.getBoolean("rv.all"), "-Drv.all=true runs every leg");
        int unflagged = 0, n = 0;
        Set<String> only = new HashSet<>(Arrays.asList(System.getProperty("rv.legs", "").split(",")));
        only.remove("");
        for (JsonNode leg : legs)
            if (only.isEmpty() || only.contains(leg.get("id").asText()))
            for (String profile : List.of("gravel", "gravel_no_tc"))
                for (String mode : List.of("favor", "avoid", "explore"))
                    for (String alg : Arrays.asList(null, "alternative_route", "bit_of_new")) {
                        unflagged += check(leg.get("id").asText() + " " + profile + " " + mode + " " + alg, leg, profile, mode, alg, null);
                        n++;
                    }
        Map<Integer, Integer> viaHist = new TreeMap<>();
        int flagged = 0, paths = 0;
        List<Long> took = new ArrayList<>();
        for (Map<String, Object> r : results) {
            paths++;
            viaHist.merge((Integer) r.get("vias"), 1, Integer::sum);
            if (!(Boolean) r.get("identical")) flagged++;
            if ((Integer) r.get("path") == 0) took.add((Long) r.get("took_ms"));
        }
        Collections.sort(took);
        System.out.println("RECREATE requests " + n + " paths " + paths + " not identical " + flagged + " unflagged " + unflagged
                + " vias/path " + viaHist + " took p50 " + took.get(took.size() / 2) + " p95 " + took.get((int) (0.95 * took.size())));
        assertEquals(0, unflagged);
    }
}
