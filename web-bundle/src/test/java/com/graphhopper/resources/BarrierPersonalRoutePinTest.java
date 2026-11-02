package com.graphhopper.resources;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.graphhopper.GHRequest;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.config.Profile;
import com.graphhopper.http.ProfileResolver;
import com.graphhopper.jackson.Jackson;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.util.AllEdgesIterator;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.trailmap.TrailmapGraphHopper;
import com.graphhopper.trailmap.barrier.BarrierClass;
import com.graphhopper.trailmap.barrier.BarrierProfiles;
import com.graphhopper.util.*;
import com.graphhopper.util.shapes.GHPoint;
import jakarta.ws.rs.core.Response;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;

import java.io.File;
import java.io.FileWriter;
import java.io.Writer;
import java.nio.file.Files;
import java.util.*;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

/**
 * W26 S4 /personal_route PIN (test_lead_routing, CATALOG §8): the no-flag answers of POST /personal_route through
 * {@code doPost}, in-JVM on the S1 graph, captured before the change and compared after it, byte for byte, ignoring
 * only timing keys and hints.personal.cached / edges_measured_*.
 * <ul>
 * <li>pairs: route-check/pairs.json + pairs at changed-set barriers (ask, refuse, and gates within 12.5 m of a
 * junction): 300 m outside → 50 m inside, and 300 m outside → 300 m outside past the gate;</li>
 * <li>modes neutral / favor / avoid / explore × single / alternative_route / bit_of_new (history avoid, explore) ×
 * profiles gravel / trailmap_foot (the S1 import has no gravel_no_tc).</li>
 * </ul>
 * Errors are recorded as their status and body. Personal data: the capture stays in data/, never on a page.
 * <pre>-Dpin.root=ABS/routing_v2 -Dpin.graph=DIR -Dpin.config=FILE -Dpin.cells=DIR (-Dpin.write=FILE | -Dpin.compare=FILE)
 * [-Dpin.gates=N]</pre>
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
public class BarrierPersonalRoutePinTest {

    static final String[] PROFILES = {"gravel", "trailmap_foot"};
    static final String[] MODES = {"neutral", "favor", "avoid", "explore"};
    static final String[][] ALGOS = {{"single", null}, {"alternative_route", null}, {"bit_of_new", "avoid"}, {"bit_of_new", "explore"}};

    TrailmapGraphHopper hopper;
    TrailmapPersonalRouteResource resource;
    final ObjectMapper om = Jackson.newObjectMapper();

    @BeforeAll
    void setUp() throws Exception {
        assumeTrue(System.getProperty("pin.graph") != null, "pin.graph not set");
        ObjectMapper yaml = Jackson.initObjectMapper(new ObjectMapper(new YAMLFactory()));
        GraphHopperConfig config = yaml.treeToValue(yaml.readTree(new File(System.getProperty("pin.config"))).get("graphhopper"), GraphHopperConfig.class);
        config.putObject("graph.location", System.getProperty("pin.graph"));
        config.putObject("datareader.file", "");
        // local cells instead of the production cells service
        config.putObject("personal_routing.cells_source", "");
        config.putObject("personal_routing.cells_dir", System.getProperty("pin.cells"));
        hopper = new TrailmapGraphHopper();
        hopper.init(config);
        hopper.setAllowWrites(false);
        hopper.load();
        resource = new TrailmapPersonalRouteResource(config, hopper, new ProfileResolver(hopper.getProfiles()), r -> r);
    }

    @AfterAll
    void tearDown() {
        if (hopper != null) hopper.close();
    }

    // ------------------------------------------------------------------------------------------------ pairs

    static boolean open(Weighting w, EdgeIteratorState e) {
        return Double.isFinite(w.calcEdgeWeight(e, false)) || Double.isFinite(w.calcEdgeWeight(e, true));
    }

    static int find(int[] p, int x) {
        while (p[x] != x) {
            p[x] = p[p[x]];
            x = p[x];
        }
        return x;
    }

    GHPoint along(Weighting w, int start, int skip, double d) {
        BaseGraph g = hopper.getBaseGraph();
        EdgeExplorer ex = g.createEdgeExplorer();
        Map<Integer, Double> dist = new HashMap<>();
        PriorityQueue<double[]> q = new PriorityQueue<>(Comparator.comparingDouble(x -> x[0]));
        dist.put(start, 0.0);
        q.add(new double[]{0, start});
        while (!q.isEmpty()) {
            double[] cur = q.poll();
            int n = (int) cur[1];
            if (cur[0] > dist.getOrDefault(n, Double.MAX_VALUE)) continue;
            EdgeIterator it = ex.setBaseNode(n);
            while (it.next()) {
                if (it.getEdge() == skip || !open(w, it)) continue;
                double nd = cur[0] + it.getDistance();
                if (cur[0] <= d && nd >= d && it.getDistance() > 0.5) {
                    PointList pl = it.fetchWayGeometry(FetchMode.ALL);
                    double rest = d - cur[0];
                    for (int i = 0; i + 1 < pl.size(); i++) {
                        double seg = DistanceCalcEarth.DIST_EARTH.calcDist(pl.getLat(i), pl.getLon(i), pl.getLat(i + 1), pl.getLon(i + 1));
                        if (seg >= rest || i + 2 == pl.size()) {
                            double f = seg <= 0 ? 0 : Math.min(1, rest / seg);
                            return new GHPoint(pl.getLat(i) + f * (pl.getLat(i + 1) - pl.getLat(i)), pl.getLon(i) + f * (pl.getLon(i + 1) - pl.getLon(i)));
                        }
                        rest -= seg;
                    }
                }
                if (nd < dist.getOrDefault(it.getAdjNode(), Double.MAX_VALUE)) {
                    dist.put(it.getAdjNode(), nd);
                    q.add(new double[]{nd, it.getAdjNode()});
                }
            }
        }
        return null;
    }

    static int degree(EdgeExplorer ex, int node) {
        int d = 0;
        EdgeIterator it = ex.setBaseNode(node);
        while (it.next()) d++;
        return d;
    }

    /** within 12.5 m of a junction on either side (over open or closed edges, as the crossing placement sees it) */
    boolean nearJunction(int node, int skip) {
        BaseGraph g = hopper.getBaseGraph();
        EdgeExplorer ex = g.createEdgeExplorer();
        EdgeIterator it = ex.setBaseNode(node);
        while (it.next()) {
            if (it.getEdge() == skip) continue;
            if (it.getDistance() <= 12.5 && degree(ex, it.getAdjNode()) >= 3) return true;
        }
        return false;
    }

    /** {id, profile-or-null, fromLat, fromLon, toLat, toLon}; deterministic */
    List<Object[]> pairs() throws Exception {
        List<Object[]> out = new ArrayList<>();
        JsonNode rc = om.readTree(new File(System.getProperty("pin.root"), "route-check/pairs.json"));
        for (JsonNode p : rc)
            out.add(new Object[]{p.get("id").asText(), null, p.get("from").get(0).asDouble(), p.get("from").get(1).asDouble(),
                    p.get("to").get(0).asDouble(), p.get("to").get(1).asDouble()});
        int per = Integer.getInteger("pin.gates", 4);
        BaseGraph g = hopper.getBaseGraph();
        NodeAccess na = g.getNodeAccess();
        for (String prof : PROFILES) {
            Profile p = hopper.getProfile(prof);
            Weighting w = hopper.createWeighting(p, new PMap());
            EnumEncodedValue<BarrierClass> enc = BarrierProfiles.barrierEnc(hopper.getEncodingManager(), p);
            int[] par = new int[g.getNodes()];
            for (int i = 0; i < par.length; i++) par[i] = i;
            AllEdgesIterator it = g.getAllEdges();
            while (it.next()) if (open(w, it)) {
                int a = find(par, it.getBaseNode()), b = find(par, it.getAdjNode());
                if (a != b) par[a] = b;
            }
            Map<Integer, Integer> size = new HashMap<>();
            it = g.getAllEdges();
            while (it.next()) if (open(w, it)) size.merge(find(par, it.getBaseNode()), 1, Integer::sum);
            int main = size.entrySet().stream().max(Map.Entry.comparingByValue()).get().getKey();
            Map<String, Integer> got = new HashMap<>();
            it = g.getAllEdges();
            while (it.next()) {
                if (it.getDistance() >= 0.01 || !it.get(enc).closes() || open(w, it)) continue;
                if (na.getLat(it.getBaseNode()) != na.getLat(it.getAdjNode()) || na.getLon(it.getBaseNode()) != na.getLon(it.getAdjNode())) continue;
                int ca = find(par, it.getBaseNode()), cb = find(par, it.getAdjNode());
                if (ca == cb || (ca != main && cb != main)) continue;
                int e = it.getEdge();
                int outside = ca == main ? it.getBaseNode() : it.getAdjNode();
                int inside = outside == it.getBaseNode() ? it.getAdjNode() : it.getBaseNode();
                String kind = it.get(enc).name().toLowerCase(Locale.ROOT) + (nearJunction(outside, e) || nearJunction(inside, e) ? "-junction" : "");
                if (got.getOrDefault(kind, 0) >= per) continue;
                GHPoint a = along(w, outside, e, 300), in = along(w, inside, e, 50), far = along(w, inside, e, 300);
                if (a == null || in == null) continue;
                got.merge(kind, 1, Integer::sum);
                out.add(new Object[]{prof + "-" + kind + "-" + e + "-in", prof, a.lat, a.lon, in.lat, in.lon});
                if (far != null && far.lat != in.lat)
                    out.add(new Object[]{prof + "-" + kind + "-" + e + "-far", prof, a.lat, a.lon, far.lat, far.lon});
            }
        }
        return out;
    }

    // ------------------------------------------------------------------------------------------------ requests

    String post(Object[] pair, String prof, String mode, String[] algo) throws Exception {
        return post(pair, prof, mode, algo, Map.of());
    }

    String post(Object[] pair, String prof, String mode, String[] algo, Map<String, Object> extra) throws Exception {
        Map<String, Object> body = new LinkedHashMap<>();
        body.put("points", List.of(List.of(pair[3], pair[2]), List.of(pair[5], pair[4])));
        body.put("profile", prof);
        body.put("points_encoded", false);
        body.put("instructions", false);
        body.put("personal_mode", mode);
        if (!"neutral".equals(mode)) {
            body.put("personal_user", "owner");
            body.put("personal_cells", "gravel_last50");
            body.put("personal_strength", 1.0);
        }
        if (!"single".equals(algo[0])) body.put("algorithm", algo[0]);
        if (algo[1] != null) {
            body.put("bit_of_new_history", algo[1]);
            body.put("personal_user", "owner");
        }
        if (System.getProperty("pin.flag") != null) body.put("barriers", System.getProperty("pin.flag"));
        body.putAll(extra);
        GHRequest req = om.readValue(om.writeValueAsString(body), GHRequest.class);
        ObjectNode rec = om.createObjectNode();
        try {
            Response r = resource.doPost(req, null);
            rec.put("status", r.getStatus());
            rec.set("body", om.readTree(om.writeValueAsString(r.getEntity())));
        } catch (Exception ex) {
            rec.put("status", "exception");
            rec.put("class", ex.getClass().getName());
            rec.put("message", String.valueOf(ex.getMessage()));
            if (ex instanceof com.graphhopper.jackson.MultiException) {
                List<Throwable> errs = ((com.graphhopper.jackson.MultiException) ex).getErrors();
                rec.put("message", errs.isEmpty() ? "" : String.valueOf(errs.get(0).getMessage()));
            }
        }
        strip(rec);
        return rec.toString();
    }

    /** timing keys and the personal cache state out, everything else stays */
    static void strip(JsonNode n) {
        if (n instanceof ObjectNode) {
            ObjectNode o = (ObjectNode) n;
            List<String> drop = new ArrayList<>();
            o.fieldNames().forEachRemaining(k -> {
                if (k.equals("took") || k.equals("ms") || k.startsWith("ms_") || k.endsWith("_ms") || k.equals("cached")
                        || k.startsWith("edges_measured")) drop.add(k);
            });
            drop.forEach(o::remove);
            o.elements().forEachRemaining(BarrierPersonalRoutePinTest::strip);
        } else if (n instanceof ArrayNode) n.elements().forEachRemaining(BarrierPersonalRoutePinTest::strip);
    }

    // ------------------------------------------------------------------------------------------------ flag checks

    static String key(double lat, double lon) {
        return Math.round(lat * 1e6) + "," + Math.round(lon * 1e6);
    }

    static List<Integer> edgeIds(JsonNode path) {
        List<Integer> out = new ArrayList<>();
        JsonNode d = path.path("details").path("edge_id");
        for (JsonNode x : d) {
            int id = x.get(2).asInt();
            if (out.isEmpty() || out.get(out.size() - 1) != id) out.add(id);
        }
        return out;
    }

    static List<double[]> coordsOf(JsonNode lineString) {
        List<double[]> out = new ArrayList<>();
        if (lineString == null) return out;
        JsonNode c = lineString.has("coordinates") ? lineString.get("coordinates") : lineString;
        for (JsonNode p : c) out.add(new double[]{p.get(1).asDouble(), p.get(0).asDouble()});
        return out;
    }

    static int nearestIndex(List<double[]> pts, double lat, double lon) {
        int best = 0;
        double bd = Double.MAX_VALUE;
        for (int i = 0; i < pts.size(); i++) {
            double d = DistanceCalcEarth.DIST_EARTH.calcDist(pts.get(i)[0], pts.get(i)[1], lat, lon);
            if (d < bd) {
                bd = d;
                best = i;
            }
        }
        return best;
    }

    /** the app's re-create of one piece: plain /route + flag, edge_id; null on an error */
    List<Integer> replay(String prof, List<double[]> pts) {
        GHRequest r = new GHRequest();
        for (double[] p : pts) r.addPoint(new GHPoint(p[0], p[1]));
        r.setProfile(prof);
        r.setSnapPreventions(List.of("ferry"));
        r.setPathDetails(List.of("edge_id"));
        r.putHint("instructions", false);
        r.putHint("barriers", "v1");
        com.graphhopper.GHResponse rsp = hopper.route(r);
        if (rsp.hasErrors()) return null;
        List<Integer> out = new ArrayList<>();
        for (com.graphhopper.util.details.PathDetail d : rsp.getBest().getPathDetails().get("edge_id")) {
            int id = ((Number) d.getValue()).intValue();
            if (out.isEmpty() || out.get(out.size() - 1) != id) out.add(id);
        }
        return out;
    }

    /**
     * W26 flag checks (test_lead_routing, CATALOG §8 S4 /personal_route), on the same pairs and modes as the pin, with
     * "barriers":"v1" and the app's snap_preventions ["ferry"]:
     * (i) neutral single == /route + flag (points, distance, hints.barriers; a refusal: the same message);
     * (ii) every closed barrier the shown path passes (its node on the path, from the graph's own barrier edges) is
     * listed in hints.barriers; no refuse barrier is passed;
     * (iii) hints.personal.alternatives_skipped exactly when a crossing exists and alternatives / bit_of_new were asked;
     * (iv) route-check pairs (no barrier): flag == no flag, hints.barriers aside;
     * (v) CRITICAL: the app's pieces of a crossing path (A → W1, W2 → B, the path's via_points split at W1 / W2),
     * re-routed by plain /route + flag, ride the path's own edges, or the path carries a recreate entry (not identical).
     */
    @Test
    void flagChecks() throws Exception {
        assumeTrue(Boolean.getBoolean("pin.flagchecks"), "pin.flagchecks not set");
        BaseGraph g = hopper.getBaseGraph();
        NodeAccess na = g.getNodeAccess();
        Map<String, Map<String, BarrierClass>> closed = new HashMap<>();
        for (String prof : PROFILES) {
            EnumEncodedValue<BarrierClass> enc = BarrierProfiles.barrierEnc(hopper.getEncodingManager(), hopper.getProfile(prof));
            Map<String, BarrierClass> m = new HashMap<>();
            AllEdgesIterator it = g.getAllEdges();
            while (it.next())
                if (it.getDistance() < 0.01 && na.getLat(it.getBaseNode()) == na.getLat(it.getAdjNode())
                        && na.getLon(it.getBaseNode()) == na.getLon(it.getAdjNode()) && it.get(enc).closes())
                    m.put(key(na.getLat(it.getBaseNode()), na.getLon(it.getBaseNode())), it.get(enc));
            closed.put(prof, m);
        }
        // (ii) works on edges, not on the shown points (simplification can drop a gate node on a straight): the
        // graph's closed barrier edges per profile, edge id → [class, "lat,lon" key]
        Map<String, Map<Integer, Object[]>> closedEdges = new HashMap<>();
        for (String prof : PROFILES) {
            EnumEncodedValue<BarrierClass> enc = BarrierProfiles.barrierEnc(hopper.getEncodingManager(), hopper.getProfile(prof));
            Map<Integer, Object[]> m = new HashMap<>();
            AllEdgesIterator it = g.getAllEdges();
            while (it.next())
                if (it.getDistance() < 0.01 && na.getLat(it.getBaseNode()) == na.getLat(it.getAdjNode())
                        && na.getLon(it.getBaseNode()) == na.getLon(it.getAdjNode()) && it.get(enc).closes())
                    m.put(it.getEdge(), new Object[]{it.get(enc), key(na.getLat(it.getBaseNode()), na.getLon(it.getBaseNode()))});
            closedEdges.put(prof, m);
        }
        Map<String, Integer> c = new TreeMap<>();
        List<String> problems = new ArrayList<>();
        Map<String, Object> flag = Map.of("barriers", "v1", "snap_preventions", List.of("ferry"));
        for (Object[] pair : pairs())
            for (String prof : PROFILES) {
                if (pair[1] != null && !pair[1].equals(prof)) continue;
                for (String mode : MODES)
                    for (String[] algo : ALGOS) {
                        String id = pair[0] + "|" + prof + "|" + mode + "|" + algo[0] + (algo[1] == null ? "" : ":" + algo[1]);
                        // -Dpin.onlyI: re-run only the (i) cases (neutral single) after a harness fix there
                        if (Boolean.getBoolean("pin.onlyI") && !("neutral".equals(mode) && "single".equals(algo[0]))) continue;
                        JsonNode rec = om.readTree(post(pair, prof, mode, algo, flag));
                        c.merge("cases", 1, Integer::sum);
                        boolean ok = rec.get("status").asInt(0) == 200;
                        JsonNode body = rec.get("body");
                        JsonNode crossings = ok ? body.path("hints").path("barriers").path("paths").path(0).path("crossings") : null;
                        boolean crossing = crossings != null && crossings.size() > 0;
                        if (!ok) c.merge(rec.path("message").asText().contains("closed barrier") ? "refusal 400" : "other error", 1, Integer::sum);
                        if (crossing) c.merge("with a crossing", 1, Integer::sum);
                        // (i)
                        if ("neutral".equals(mode) && "single".equals(algo[0]) && !Boolean.getBoolean("pin.onlyII")) {
                            GHRequest r = new GHRequest(new GHPoint((Double) pair[2], (Double) pair[3]), new GHPoint((Double) pair[4], (Double) pair[5]));
                            r.setProfile(prof);
                            r.setSnapPreventions(List.of("ferry"));
                            r.putHint("instructions", false);
                            r.putHint("barriers", "v1");
                            com.graphhopper.GHResponse rr = hopper.route(r);
                            boolean same;
                            if (rr.hasErrors()) same = !ok && rec.path("message").asText().equals(rr.getErrors().get(0).getMessage());
                            else {
                                List<double[]> a = coordsOf(body.path("paths").path(0).path("points"));
                                PointList b = rr.getBest().getPoints();
                                // the response JSON rounds distance to 3 decimals and points to 1e-6 (ResponsePathSerializer)
                                boolean sizeOk = ok && a.size() == b.size();
                                boolean distOk = ok && Math.abs(body.path("paths").path(0).path("distance").asDouble() - rr.getBest().getDistance()) < 0.01;
                                boolean hintOk = om.readTree(om.writeValueAsString(rr.getHints().getObject("barriers", null))).equals(body.path("hints").path("barriers"));
                                boolean ptsOk = sizeOk;
                                for (int i = 0; ptsOk && i < a.size(); i++)
                                    ptsOk = Math.abs(a.get(i)[0] - b.getLat(i)) < 1.5e-6 && Math.abs(a.get(i)[1] - b.getLon(i)) < 1.5e-6;
                                same = sizeOk && distOk && hintOk && ptsOk;
                                if (!same && problems.size() < 30)
                                    problems.add("(i) detail " + id + " size " + sizeOk + " (" + a.size() + "/" + b.size() + ") dist " + distOk + " hints " + hintOk + " points " + ptsOk);
                            }
                            c.merge(same ? "(i) neutral == /route+flag" : "(i) NEUTRAL DIFFERS FROM /route+flag", 1, Integer::sum);
                            if (!same && problems.size() < 30) problems.add("(i) " + id);
                        }
                        if (!ok) continue;
                        // (ii)
                        Set<String> listed = new HashSet<>();
                        for (JsonNode x : crossings) for (JsonNode bar : x.path("barriers")) listed.add(key(bar.get("lat").asDouble(), bar.get("lon").asDouble()));
                        // the same request with the edge_id detail: every edge of every path, simplification aside
                        JsonNode withEdges = om.readTree(post(pair, prof, mode, algo,
                                Map.of("barriers", "v1", "snap_preventions", List.of("ferry"), "details", List.of("edge_id"))));
                        int pathNo = 0;
                        for (JsonNode p : withEdges.path("body").path("paths")) {
                            boolean main = pathNo++ == 0;
                            for (int e : edgeIds(p)) {
                                Object[] bc = closedEdges.get(prof).get(e);
                                if (bc == null) continue;
                                if (bc[0] == BarrierClass.REFUSE) {
                                    c.merge("(ii) REFUSE BARRIER PASSED", 1, Integer::sum);
                                    if (problems.size() < 30) problems.add("(ii) refuse passed " + id);
                                } else if (!main || !listed.contains((String) bc[1])) {
                                    c.merge("(ii) CLOSED BARRIER PASSED, NOT LISTED", 1, Integer::sum);
                                    if (problems.size() < 30) problems.add("(ii) unlisted " + id + " path " + (main ? 0 : "alt"));
                                } else c.merge("(ii) passed barrier listed", 1, Integer::sum);
                            }
                        }
                        // the opening-hours notices: one hints.barriers.paths entry per path (review F1)
                        int entries = body.path("hints").path("barriers").path("paths").size(), shown = body.path("paths").size();
                        c.merge(entries == shown ? "(ii) one hints.barriers entry per path" : "(ii) HINTS.BARRIERS ENTRIES != PATHS", 1, Integer::sum);
                        if (entries != shown && problems.size() < 30) problems.add("(ii) entries " + entries + " paths " + shown + " " + id);
                        if (Boolean.getBoolean("pin.onlyII")) continue;
                        // (iii)
                        boolean skipped = body.path("hints").path("personal").has("alternatives_skipped");
                        boolean expect = crossing && !"single".equals(algo[0]);
                        c.merge(skipped == expect ? "(iii) skip hint right" : "(iii) SKIP HINT WRONG", 1, Integer::sum);
                        if (skipped != expect && problems.size() < 30) problems.add("(iii) " + id + " skipped=" + skipped + " crossing=" + crossing);
                        // (iv)
                        if (pair[1] == null) {
                            ObjectNode a = (ObjectNode) rec.deepCopy();
                            ObjectNode b = (ObjectNode) om.readTree(post(pair, prof, mode, algo, Map.of("snap_preventions", List.of("ferry"))));
                            // the shown lines, flag vs no flag (same snap preventions)
                            boolean same = a.path("body").path("paths").size() == b.path("body").path("paths").size();
                            for (int k = 0; same && k < a.path("body").path("paths").size(); k++)
                                same = a.path("body").path("paths").get(k).path("points").equals(b.path("body").path("paths").get(k).path("points"));
                            c.merge(same ? "(iv) no-barrier pair: flag == no flag (lines)" : "(iv) no-barrier pair: flag differs", 1, Integer::sum);
                            if (!same && problems.size() < 30) problems.add("(iv) " + id);
                        }
                        // (v)
                        if (crossing) {
                            JsonNode withIds = om.readTree(post(pair, prof, mode, algo, Map.of("barriers", "v1", "snap_preventions", List.of("ferry"), "details", List.of("edge_id"))));
                            JsonNode p0 = body.path("paths").get(0);
                            List<Integer> edges = edgeIds(withIds.path("body").path("paths").path(0));
                            List<double[]> pts = coordsOf(p0.path("points"));
                            List<double[]> vias = coordsOf(p0.path("via_points"));
                            JsonNode x0 = crossings.get(0), xl = crossings.get(crossings.size() - 1);
                            double[] w1 = {x0.get("w1").get(1).asDouble(), x0.get("w1").get(0).asDouble()};
                            double[] w2 = {xl.get("w2").get(1).asDouble(), xl.get("w2").get(0).asDouble()};
                            int i1 = x0.get("w1_index").asInt(), i2 = xl.get("w2_index").asInt();
                            List<double[]> piece1 = new ArrayList<>(), piece2 = new ArrayList<>();
                            piece1.add(pts.get(0));
                            piece2.add(w2);
                            for (double[] v : vias) {
                                int k = nearestIndex(pts, v[0], v[1]);
                                if (k <= i1) piece1.add(v);
                                else if (k > i2) piece2.add(v);
                            }
                            piece1.add(w1);
                            piece2.add(pts.get(pts.size() - 1));
                            List<Integer> r1 = crossings.size() == 1 ? replay(prof, piece1) : null, r2 = crossings.size() == 1 ? replay(prof, piece2) : null;
                            boolean same1 = r1 != null && edges.size() >= r1.size() && edges.subList(0, r1.size()).equals(r1);
                            boolean same2 = r2 != null && edges.size() >= r2.size() && edges.subList(edges.size() - r2.size(), edges.size()).equals(r2);
                            boolean flagged = false;
                            for (JsonNode e : body.path("hints").path("personal").path("recreate"))
                                if (e.path("path").asInt(-1) == 0 && !e.path("identical").asBoolean(true)) flagged = true;
                            String v = crossings.size() > 1 ? "(v) several crossings: not replayed"
                                    : same1 && same2 ? "(v) pieces re-create" : flagged ? "(v) pieces differ, flagged" : "(v) UNFLAGGED MISS";
                            c.merge(v, 1, Integer::sum);
                            if (v.contains("UNFLAGGED") && problems.size() < 30) problems.add("(v) " + id + " piece1 " + same1 + " piece2 " + same2);
                        }
                    }
            }
        System.out.println("FLAGCHECK " + c);
        for (String p : problems) System.out.println("FLAGCHECK problem " + p);
    }

    @Test
    void pin() throws Exception {
        assumeTrue(!Boolean.getBoolean("pin.flagchecks"), "flag checks only");
        List<String> lines = new ArrayList<>();
        Map<String, Integer> statuses = new TreeMap<>();
        for (Object[] pair : pairs())
            for (String prof : PROFILES) {
                if (pair[1] != null && !pair[1].equals(prof)) continue;
                for (String mode : MODES)
                    for (String[] algo : ALGOS) {
                        String rec = post(pair, prof, mode, algo);
                        String key = pair[0] + "|" + prof + "|" + mode + "|" + algo[0] + (algo[1] == null ? "" : ":" + algo[1]);
                        lines.add(key + "\t" + rec);
                        statuses.merge(om.readTree(rec).get("status").asText(), 1, Integer::sum);
                    }
            }
        System.out.println("PIN cases " + lines.size() + " statuses " + statuses);
        if (System.getProperty("pin.write") != null) {
            try (Writer w = new FileWriter(System.getProperty("pin.write"))) {
                for (String l : lines) w.write(l + "\n");
            }
        }
        if (System.getProperty("pin.compare") != null) {
            List<String> before = Files.readAllLines(new File(System.getProperty("pin.compare")).toPath());
            int diff = 0;
            for (int i = 0; i < Math.max(before.size(), lines.size()); i++) {
                String b = i < before.size() ? before.get(i) : "", a = i < lines.size() ? lines.get(i) : "";
                if (!b.equals(a)) {
                    diff++;
                    if (diff <= 20) System.out.println("PIN DIFF " + (a.isEmpty() ? b : a).split("\t")[0]);
                }
            }
            System.out.println("PIN compare: " + diff + " of " + lines.size() + " differ");
            assertEquals(0, diff, "no-flag /personal_route must be byte-identical to the capture");
        }
    }
}
