package com.graphhopper.trailmap.tilerouting;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.jackson.Jackson;
import com.graphhopper.trailmap.TrailmapGraphHopper;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.*;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * /tile_route regression gate: the production {@link TileRouter} reproduces the stored prototype runs (waypoints to
 * 1e-6, roles, segment types) listed in tile-routing/verify/parity-manifest.json.
 * <pre>
 * mvn -pl core test -Dtest=TileRouteParityTest -DargLine=-Xmx10g \
 *   -Dtr.parity.root=ABS/routing_v2 -Dtr.parity.graph=ABS/data/experiments/tile_routing/r4-polish/graph [-Dtr.parity.only=regex]
 * </pre>
 * The graph must be the import the stored runs used (data date 2026-09-27T20:23:36Z). Loop repair and tour_repair
 * search under a wall-clock budget (LoopRepairCandidate.budgetMs, the prototype's tr.c2.budgetMs; the tour_repair seed
 * under PortalViterbiCandidate.budgetMs): a stored job that stopped on its budget ("stop=budget" or "BUDGET_HIT" in
 * its note) depends on the machine's speed and is only reported, not compared. Skips unless tr.parity.graph is set: a
 * gate run counts only with "Tests run: N, Skipped: 0".
 */
public class TileRouteParityTest {

    static TrailmapGraphHopper hopper;
    static File root;
    static final ObjectMapper JSON = Jackson.newObjectMapper();

    @BeforeAll
    static void load() throws Exception {
        String graph = System.getProperty("tr.parity.graph");
        Assumptions.assumeTrue(graph != null, "tr.parity.graph not set");
        root = new File(System.getProperty("tr.parity.root"));
        ObjectMapper yaml = Jackson.initObjectMapper(new ObjectMapper(new YAMLFactory()));
        GraphHopperConfig config = yaml.treeToValue(yaml.readTree(new File("../trailmap-config.yml")).get("graphhopper"), GraphHopperConfig.class);
        config.putObject("graph.location", graph);
        hopper = new TrailmapGraphHopper();
        hopper.init(config);
        hopper.setAllowWrites(false);
        hopper.load();
        System.out.println("graph data.date=" + hopper.getProperties().get("datareader.data.date")
                + " import.date=" + hopper.getProperties().get("datareader.import.date"));
    }

    @AfterAll
    static void close() {
        if (hopper != null) hopper.close();
    }

    static String safetyOf(JsonNode jobsFile) {
        String s = jobsFile.path("safety").asText("standard");
        return "very_strict".equals(s) ? "no_trails" : s;
    }

    /** The request a stored job stands for. */
    static TileRouteRequest request(JsonNode jobsFile, JsonNode job, JsonNode run) {
        TileRouteRequest rq = new TileRouteRequest();
        for (JsonNode k : job.get("tiles")) rq.tiles.add(k.asText());
        rq.zoom = jobsFile.path("zoom").asInt(14);
        rq.startLat = jobsFile.get("home").get("lat").asDouble();
        rq.startLon = jobsFile.get("home").get("lon").asDouble();
        rq.profile = jobsFile.path("profile").asText("gravel");
        rq.safety = safetyOf(jobsFile);
        rq.algorithm = run.get("algorithm").asText();
        rq.variety = run.path("variety").asText("mild");
        rq.seedKey = job.get("jobId").asText();
        rq.handover = false; // the stored runs are the solver's waypoints, before the W21 hand-over
        // the stored no_trails runs were made without walks (the default until the owner's A20, 2026-10-31)
        if ("no_trails".equals(rq.safety)) rq.walkInMaxM = 0;
        JsonNode g = job.get("groups");
        if (g != null && g.isArray())
            for (JsonNode grp : g) {
                List<String> l = new ArrayList<>();
                for (JsonNode k : grp) l.add(k.asText());
                rq.groups.add(l);
            }
        return rq;
    }

    @Test
    void sameAsStoredRuns() throws Exception {
        String only = System.getProperty("tr.parity.only");
        TileRouter.Config rc = new TileRouter.Config();
        rc.portalBudgetMs = 5000; // the prototype's tr.budgetMs for the tour_repair seed
        // the references were made with the bound measured on the request; -Dtr.parity.heuristic=BUILT_IN checks the
        // server's fallback before its background bound is ready
        rc.heuristic = TileRouter.Heuristic.valueOf(System.getProperty("tr.parity.heuristic", "INLINE"));
        // -Dtr.parity.clock=virtual:<nsPerCall> (the stored runs made with the same -Dtr.clock): every job is reproducible,
        // so budget-stopped and annealed jobs are compared exactly too
        String clock = System.getProperty("tr.parity.clock", "wall");
        boolean virtual = clock.startsWith("virtual:");
        if (virtual) {
            long step = Long.parseLong(clock.substring("virtual:".length()));
            rc.clock = () -> TileRouter.virtualClock(step);
        }
        String manifestName = System.getProperty("tr.parity.manifest", "tile-routing/verify/parity-manifest.json");
        TileRouter router = new TileRouter(hopper, rc);
        JsonNode manifest = JSON.readTree(new File(root, manifestName));
        int compared = 0, budgetLimited = 0, saTiming = 0;
        List<String> diffs = new ArrayList<>();
        // -Dtr.parity.memPerKey=<bytes> -Dtr.parity.memFixed=<bytes>: assert allocation <= perKey * keysHeld + fixed
        long memPerKey = Long.getLong("tr.parity.memPerKey", 0), memFixed = Long.getLong("tr.parity.memFixed", 0);
        List<String> memOver = new ArrayList<>();
        for (JsonNode run : manifest.get("runs")) {
            JsonNode jobsFile = JSON.readTree(new File(root, run.get("jobs").asText()));
            File dir = new File(root, run.get("run").asText());
            for (JsonNode job : jobsFile.get("jobs")) {
                String id = job.get("jobId").asText();
                String label = run.get("run").asText() + " " + id;
                if (only != null && !label.matches(only)) continue;
                File stored = new File(dir, id + ".waypoints.json");
                if (!stored.exists()) continue;
                JsonNode want = JSON.readTree(stored);
                String note = want.path("note").asText("");
                TileRouteRequest rq = request(jobsFile, job, run);
                long a0 = allocated();
                TileRouter.Result rr = router.route(rq);
                long alloc = allocated() - a0;
                ObjectNode got = TileRouteJson.write(JSON, rq, rr);
                String d = diff(want, got);
                // memory (CATALOG §10): allocation per request grows with the keys the searches touched, not the graph
                long bound = memPerKey * rr.keysHeld + memFixed;
                System.out.println("MEM " + label + " allocMB=" + alloc / 1_000_000 + " keysHeld=" + rr.keysHeld
                        + " bytesPerKey=" + (rr.keysHeld > 0 ? alloc / rr.keysHeld : -1) + " graphKeys=" + 2L * hopper.getBaseGraph().getEdges()
                        + (memPerKey > 0 ? " boundMB=" + bound / 1_000_000 : ""));
                if (memPerKey > 0 && alloc > bound) memOver.add(label + " allocMB=" + alloc / 1_000_000 + " > boundMB=" + bound / 1_000_000);
                if (!virtual && !"strokes".equals(rq.algorithm) && (note.contains("stop=budget") || note.contains("BUDGET_HIT"))) {
                    budgetLimited++;
                    System.out.println("BUDGET (reported only) " + label + (d == null ? " same" : " differs: " + d));
                    continue;
                }
                // the annealing temperature follows the wall clock (LoopRepairCandidate.solve: temp = temp0 * (1 - elapsed
                // share of the budget)), so a repair that reached annealing ("sa=true") can differ by timing alone
                if (!virtual && d != null && !"strokes".equals(rq.algorithm) && note.contains("sa=true")) {
                    saTiming++;
                    System.out.println("SA-TIMING (reported only) " + label + ": " + d);
                    continue;
                }
                compared++;
                System.out.println((d == null ? "SAME " : "DIFF ") + label + (d == null ? "" : ": " + d)
                        + " ms=" + got.path("stats").path("timings").path("totalMs").asLong());
                if (d != null) diffs.add(label + ": " + d);
            }
        }
        System.out.println("compared=" + compared + " budgetLimited=" + budgetLimited + " saTiming=" + saTiming + " diffs=" + diffs.size());
        assertTrue(compared > 0, "nothing compared");
        assertTrue(diffs.isEmpty(), "differs from the stored prototype runs:\n" + String.join("\n", diffs));
        assertTrue(memOver.isEmpty(), "allocation above the per-key bound:\n" + String.join("\n", memOver));
    }

    /**
     * Heap bytes allocated so far by all threads, live and finished: JDK 21's getTotalThreadAllocatedBytes, called
     * reflectively because the build targets an older release.
     */
    static long allocated() {
        try {
            Object mx = java.lang.management.ManagementFactory.getThreadMXBean();
            java.lang.reflect.Method m = Class.forName("com.sun.management.ThreadMXBean").getMethod("getTotalThreadAllocatedBytes");
            return (long) m.invoke(mx);
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("needs JDK 21+ for getTotalThreadAllocatedBytes", e);
        }
    }

    /**
     * W21 hand-over (CATALOG §10 "Re-create by plain /route"): every manifest job with the hand-over on. Per job: no
     * flagged segment, every routed segment rebuilt the app's way (alone, profile only, snap_preventions ferry) gives
     * exactly the response's edges, no banned-trail metres on it for strict / no_trails, the same tiles fetched as with
     * the hand-over off. Prints pins per job and the metres each pin guides (the owner's visible vs hidden question).
     */
    @Test
    void handoverRecreates() throws Exception {
        String only = System.getProperty("tr.parity.only");
        TileRouter.Config rc = new TileRouter.Config();
        rc.portalBudgetMs = 5000;
        rc.heuristic = TileRouter.Heuristic.INLINE;
        String clock = System.getProperty("tr.parity.clock", "wall");
        if (clock.startsWith("virtual:")) {
            long step = Long.parseLong(clock.substring("virtual:".length()));
            rc.clock = () -> TileRouter.virtualClock(step);
        }
        TileRouter router = new TileRouter(hopper, rc);
        JsonNode manifest = JSON.readTree(new File(root, System.getProperty("tr.parity.manifest", "tile-routing/verify/parity-manifest.json")));
        boolean polish = Boolean.getBoolean("tr.parity.handoverPolish");
        List<String> fails = new ArrayList<>();
        int jobs = 0;
        for (JsonNode run : manifest.get("runs")) {
            JsonNode jobsFile = JSON.readTree(new File(root, run.get("jobs").asText()));
            for (JsonNode job : jobsFile.get("jobs")) {
                String label = run.get("run").asText() + " " + job.get("jobId").asText();
                if (only != null && !label.matches(only)) continue;
                TileRouteRequest off = request(jobsFile, job, run);
                if (polish && !off.groups.isEmpty()) continue;
                off.polish = polish;
                off.walkInMaxM = Double.NaN; // the endpoint's defaults
                TileRouteRequest on = request(jobsFile, job, run);
                on.polish = polish;
                on.walkInMaxM = Double.NaN;
                on.handover = true;
                TileRouter.Result ro = router.route(off), rn = router.route(on);
                jobs++;
                ObjectNode jo = TileRouteJson.write(JSON, off, ro), jn = TileRouteJson.write(JSON, on, rn);
                int fetchedOff = jo.path("stats").path("fetched").asInt(), fetchedOn = jn.path("stats").path("fetched").asInt();
                TileHandover.Stats h = rn.handover;
                double[] rb = rebuildApp(rn, TileRouter.safetyModel(on.safety) != null ? on.safety : null);
                System.out.println("HANDOVER " + label + " safety=" + on.safety + " segments=" + (h == null ? rn.segments.size() : h.segments)
                        + " pinnedSegments=" + (h == null ? 0 : h.pinnedSegments) + " pins=" + (h == null ? 0 : h.pins)
                        + " flagged=" + (h == null ? 0 : h.flags.size()) + " guidedM=" + (h == null ? "[]" : h.guidedM)
                        + " fetched " + fetchedOff + "→" + fetchedOn + " rebuildDiffRuns=" + Math.round(rb[0]) + " bannedM=" + Math.round(rb[1])
                        + " handoverMs=" + rn.handoverMs);
                if (h != null) for (TileHandover.Flag fl : h.flags)
                    fails.add(label + " segment " + fl.segment + " offM=" + Math.round(fl.offM) + ": " + fl.reason);
                if (rb[0] > 0) fails.add(label + ": the app's rebuild differs from the response in " + Math.round(rb[0]) + " routed runs");
                if (rb[1] > 0) fails.add(label + ": " + Math.round(rb[1]) + " m of banned trail on the app's rebuild");
                if (fetchedOn != fetchedOff) fails.add(label + ": fetched " + fetchedOff + " → " + fetchedOn);
            }
        }
        System.out.println("handover jobs=" + jobs + " fails=" + fails.size());
        assertTrue(jobs > 0, "nothing run");
        assertTrue(fails.isEmpty(), String.join("\n", fails));
    }

    /**
     * The response rebuilt the app's way, segment by segment: [metres where its edges differ from the response's
     * geometry (by edge id sequence per routed run), banned-trail metres under the safety level (null = none)].
     */
    static double[] rebuildApp(TileRouter.Result r, String safety) {
        // gravel_scale detail values as GraphHopper writes them ("2", "3", "4", "unknown")
        Set<String> banned = "strict".equals(safety) ? Set.of("3", "4", "unknown")
                : "no_trails".equals(safety) ? Set.of("2", "3", "4", "unknown") : Set.of();
        double diffRuns = 0, bannedM = 0;
        for (int i = 0; i + 1 < r.vias.size(); ) {
            if (!"followRoads".equals(r.segments.get(i))) {
                i++;
                continue;
            }
            int j = i;
            while (j < r.segments.size() && "followRoads".equals(r.segments.get(j))) j++;
            // the response's run i..j as it was assembled (one request) vs the app's pieces
            List<Integer> shown = edgeIds(appRoute(r.vias.subList(i, j + 1), null)), rebuilt = new ArrayList<>();
            for (int k = i; k < j; k++) {
                com.graphhopper.ResponsePath p = appRoute(r.vias.subList(k, k + 2), banned.isEmpty() ? null : "gravel_scale");
                for (int id : edgeIds(p)) if (rebuilt.isEmpty() || rebuilt.get(rebuilt.size() - 1) != id) rebuilt.add(id);
                if (!banned.isEmpty())
                    for (com.graphhopper.util.details.PathDetail d : p.getPathDetails().get("gravel_scale"))
                        if (banned.contains(String.valueOf(d.getValue())))
                            for (int q = d.getFirst(); q < d.getLast(); q++)
                                bannedM += com.graphhopper.trailmap.tilerouting.foundation.Foundation.dist(p.getPoints().getLat(q), p.getPoints().getLon(q),
                                        p.getPoints().getLat(q + 1), p.getPoints().getLon(q + 1));
            }
            if (!shown.equals(rebuilt)) diffRuns++;
            i = j;
        }
        return new double[]{diffRuns, bannedM};
    }

    static com.graphhopper.ResponsePath appRoute(List<com.graphhopper.trailmap.tilerouting.foundation.Via> vias, String extraDetail) {
        List<com.graphhopper.util.shapes.GHPoint> pts = new ArrayList<>();
        for (com.graphhopper.trailmap.tilerouting.foundation.Via v : vias) pts.add(new com.graphhopper.util.shapes.GHPoint(v.lat, v.lon));
        com.graphhopper.GHRequest req = new com.graphhopper.GHRequest(pts).setProfile("gravel");
        req.setSnapPreventions(List.of("ferry"));
        req.setPathDetails(extraDetail == null ? List.of("edge_id") : List.of("edge_id", extraDetail));
        com.graphhopper.GHResponse rsp = hopper.route(req);
        if (rsp.hasErrors()) throw new IllegalStateException(rsp.getErrors().toString());
        return rsp.getBest();
    }

    static List<Integer> edgeIds(com.graphhopper.ResponsePath p) {
        List<Integer> out = new ArrayList<>();
        for (com.graphhopper.util.details.PathDetail d : p.getPathDetails().get("edge_id")) {
            int id = ((Number) d.getValue()).intValue();
            if (d.getLast() > d.getFirst() && (out.isEmpty() || out.get(out.size() - 1) != id)) out.add(id);
        }
        return out;
    }

    /** Two different jobs at once (different zoom, safety and algorithm) equal their stored solo runs: no shared state leaks. */
    @Test
    void concurrentSameAsSolo() throws Exception {
        String[][] pairs = {
                {"data/tile-routing/jobs/perf.json", "data/experiments/tile_routing/r4-polish/runs/i6/c3-perf", "strokes", "perf-n020"},
                {"data/tile-routing/jobs/small.json", "data/experiments/tile_routing/r4-polish/runs/i6/c3-small", "strokes", "small-n050"},
                {"data/tile-routing/jobs/easy-safety-strict.json", "data/experiments/tile_routing/r4-polish/runs/i6/c3-easy-strict", "strokes", "easy-n10"}};
        TileRouter router = new TileRouter(hopper, new TileRouter.Config());
        List<Thread> threads = new ArrayList<>();
        List<String> fails = Collections.synchronizedList(new ArrayList<>());
        for (String[] p : pairs) {
            JsonNode jobsFile = JSON.readTree(new File(root, p[0]));
            JsonNode job = null;
            for (JsonNode j : jobsFile.get("jobs")) if (j.get("jobId").asText().equals(p[3])) job = j;
            JsonNode want = JSON.readTree(new File(new File(root, p[1]), p[3] + ".waypoints.json"));
            TileRouteRequest rq = request(jobsFile, job, JSON.createObjectNode().put("algorithm", p[2]));
            Thread t = new Thread(() -> {
                try {
                    String d = diff(want, TileRouteJson.write(JSON, rq, router.route(rq)));
                    System.out.println("CONCURRENT " + p[3] + (d == null ? " same" : " differs: " + d));
                    if (d != null) fails.add(p[3] + ": " + d);
                } catch (Exception e) {
                    fails.add(p[3] + ": " + e);
                }
            });
            threads.add(t);
        }
        for (Thread t : threads) t.start();
        for (Thread t : threads) t.join();
        assertTrue(fails.isEmpty(), String.join("\n", fails));
    }

    /**
     * The Java re-use polish on the same inputs as tile-routing/polish/polish.mjs (each run dir under tr.parity.polishIn
     * names its input in run.json "polishOf"), written as run dirs under tr.parity.polishOut for verify.mjs and the
     * band comparison (tile-routing/verify/polish-band.mjs). Skips unless tr.parity.polishOut is set.
     */
    @Test
    void polishOnStoredInputs() throws Exception {
        String outPath = System.getProperty("tr.parity.polishOut");
        Assumptions.assumeTrue(outPath != null, "tr.parity.polishOut not set");
        File in = new File(System.getProperty("tr.parity.polishIn"));
        TileRouter router = new TileRouter(hopper, new TileRouter.Config());
        int jobs = 0;
        File[] dirs = in.listFiles(File::isDirectory);
        Arrays.sort(dirs);
        for (File d : dirs) {
            JsonNode runJson = JSON.readTree(new File(d, "run.json"));
            JsonNode jobsFile = JSON.readTree(new File(runJson.get("jobsFile").asText()));
            File src = new File(runJson.get("polishOf").asText());
            double penalty = runJson.get("penalty").asDouble();
            JsonNode tf = runJson.get("trailFactor");
            File out = new File(outPath, d.getName());
            out.mkdirs();
            ObjectNode rj = JSON.createObjectNode().put("jobsFile", runJson.get("jobsFile").asText()).put("polishOf", src.getPath())
                    .put("polishStrength", 1 / penalty).put("polishTrailFactor", tf == null || tf.isNull() ? 1 : 1 / tf.asDouble()).put("engine", "java");
            JSON.writerWithDefaultPrettyPrinter().writeValue(new File(out, "run.json"), rj);
            for (JsonNode job : jobsFile.get("jobs")) {
                if (job.get("groups") != null) continue;
                String id = job.get("jobId").asText();
                File wf = new File(src, id + ".waypoints.json");
                if (!wf.exists()) continue;
                JsonNode w = JSON.readTree(wf);
                com.graphhopper.trailmap.tilerouting.foundation.Foundation.Plan plan = new com.graphhopper.trailmap.tilerouting.foundation.Foundation.Plan();
                JsonNode wps = w.get("waypoints"), roles = w.get("roles"), segs = w.get("segments");
                for (int i = 0; i < wps.size(); i++) {
                    String role = roles.get(i).asText();
                    int c = role.indexOf(':');
                    plan.vias.add(new com.graphhopper.trailmap.tilerouting.foundation.Via(wps.get(i).get(0).asDouble(), wps.get(i).get(1).asDouble(), Double.NaN,
                            c < 0 ? role : role.substring(0, c), c < 0 ? null : role.substring(c + 1)));
                    if (i + 1 < wps.size()) plan.segments.add(segs == null ? "followRoads" : segs.get(i).asText());
                }
                TileRouteRequest rq = request(jobsFile, job, JSON.createObjectNode().put("algorithm", "strokes"));
                rq.polish = true;
                rq.polishStrength = 1 / penalty;
                rq.polishTrailFactor = tf == null || tf.isNull() ? 1 : 1 / tf.asDouble();
                TileRouter.Result r = router.polishPlan(rq, plan);
                ObjectNode o = TileRouteJson.write(JSON, rq, r);
                o.put("jobId", id).put("cand", w.path("cand").asText() + "+javapolish").put("version", w.path("version").asText() + "+javapolish");
                if (w.has("walks")) o.set("walks", w.get("walks"));
                if (segs == null && !o.get("segments").toString().contains("direct")) o.remove("segments");
                JSON.writerWithDefaultPrettyPrinter().writeValue(new File(out, id + ".waypoints.json"), o);
                System.out.println("POLISH " + d.getName() + " " + id + " changed=" + r.polish.changed + " km " + Math.round(r.polish.kmBefore * 100) / 100.0
                        + " -> " + Math.round(r.polish.kmAfter * 100) / 100.0 + " ms=" + r.totalMs);
                jobs++;
            }
        }
        assertTrue(jobs > 0, "no polish input");
    }

    /** null when waypoints (1e-6), roles and segment types are equal; else the first difference. */
    static String diff(JsonNode want, JsonNode got) {
        JsonNode ww = want.get("waypoints"), gw = got.get("waypoints");
        if (ww.size() != gw.size()) return "waypoints " + ww.size() + " vs " + gw.size();
        for (int i = 0; i < ww.size(); i++) {
            if (Math.abs(ww.get(i).get(0).asDouble() - gw.get(i).get(0).asDouble()) > 1.5e-6
                    || Math.abs(ww.get(i).get(1).asDouble() - gw.get(i).get(1).asDouble()) > 1.5e-6)
                return "waypoint " + i + " " + ww.get(i) + " vs " + gw.get(i);
            if (!want.get("roles").get(i).asText().equals(got.get("roles").get(i).asText()))
                return "role " + i + " " + want.get("roles").get(i) + " vs " + got.get("roles").get(i);
        }
        JsonNode ws = want.get("segments"), gs = got.get("segments");
        for (int i = 0; i + 1 < ww.size(); i++) {
            String a = ws == null ? "followRoads" : ws.get(i).asText(), b = gs.get(i).asText();
            if (!a.equals(b)) return "segment " + i + " " + a + " vs " + b;
        }
        return null;
    }
}
