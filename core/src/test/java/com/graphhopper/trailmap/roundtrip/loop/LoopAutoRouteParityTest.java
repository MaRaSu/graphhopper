package com.graphhopper.trailmap.roundtrip.loop;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.jackson.Jackson;
import com.graphhopper.trailmap.TrailmapGraphHopper;
import com.graphhopper.util.CustomModel;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.*;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * AutoRoute v2 regression gate: {@link LoopAutoRoute} reproduces the round-4 "area16" prototype run exactly — the same
 * 3 routes per press, waypoints and coords to the stored 1e-6, every session's presses in order with the earlier
 * presses' loops as its memory.
 * <pre>
 * mvn -pl core test -Dtest=LoopAutoRouteParityTest -DargLine=-Xmx10g \
 *   -Dloop.jobs=ABS/eval55/runs/jobs.json -Dloop.gens=ABS/r4/runs/gen-area16 -Dloop.graph=ABS/graphClone
 * </pre>
 * The graph must be the import the stored run used (data date 2026-09-27T20:23:36Z). Skips unless loop.jobs is set:
 * a gate run counts only with "Tests run: N, Skipped: 0".
 */
public class LoopAutoRouteParityTest {

    static TrailmapGraphHopper hopper;
    static JsonNode jobs;
    static File gens;
    static final ObjectMapper JSON = Jackson.newObjectMapper();

    @BeforeAll
    static void load() throws Exception {
        String jobsPath = System.getProperty("loop.jobs");
        Assumptions.assumeTrue(jobsPath != null, "loop.jobs not set");
        gens = new File(System.getProperty("loop.gens"));
        ObjectMapper yaml = Jackson.initObjectMapper(new ObjectMapper(new YAMLFactory()));
        GraphHopperConfig config = yaml.treeToValue(yaml.readTree(new File("../trailmap-config.yml")).get("graphhopper"), GraphHopperConfig.class);
        config.putObject("graph.location", System.getProperty("loop.graph"));
        hopper = new TrailmapGraphHopper();
        hopper.init(config);
        hopper.setAllowWrites(false);
        hopper.load();
        System.out.println("graph data.date=" + hopper.getProperties().get("datareader.data.date")
                + " import.date=" + hopper.getProperties().get("datareader.import.date"));
        jobs = JSON.readTree(new File(jobsPath)).get("jobs");
    }

    @AfterAll
    static void close() {
        if (hopper != null) hopper.close();
    }

    /** Every session solo, in job order. */
    @Test
    void sameAsStoredRun() throws Exception {
        LoopAutoRoute engine = new LoopAutoRoute(hopper);
        int presses = run(engine, order(null));
        assertTrue(presses > 0);
    }

    /** Two sessions interleaved (A1, B1, A2, B2 ...) on one engine: each equals its solo run, nothing leaks. */
    @Test
    void sessionsInterleavedSameAsSolo() throws Exception {
        List<String> ids = new ArrayList<>();
        for (JsonNode j : jobs) if (!ids.contains(j.get("scenarioId").asText())) ids.add(j.get("scenarioId").asText());
        Assumptions.assumeTrue(ids.size() >= 2);
        // a cache of 1 start forces eviction and rebuild between the two sessions on every press
        LoopAutoRoute engine = new LoopAutoRoute(hopper, 1);
        int presses = run(engine, order(List.of(ids.get(0), ids.get(1))));
        assertTrue(presses > 0);
    }

    /** Jobs in session order; with two scenario ids, their presses alternate. */
    private List<JsonNode> order(List<String> interleave) {
        List<JsonNode> out = new ArrayList<>();
        if (interleave == null) {
            for (JsonNode j : jobs) out.add(j);
            return out;
        }
        List<List<JsonNode>> lists = new ArrayList<>();
        for (String id : interleave) {
            List<JsonNode> l = new ArrayList<>();
            for (JsonNode j : jobs) if (j.get("scenarioId").asText().equals(id)) l.add(j);
            l.sort(Comparator.comparingInt(j -> j.get("regen").asInt()));
            lists.add(l);
        }
        for (int i = 0; i < Math.max(lists.get(0).size(), lists.get(1).size()); i++)
            for (List<JsonNode> l : lists) if (i < l.size()) out.add(l.get(i));
        return out;
    }

    private int run(LoopAutoRoute engine, List<JsonNode> order) throws Exception {
        Map<String, List<List<double[]>>> shown = new HashMap<>();
        int presses = 0;
        for (JsonNode job : order) {
            String scenarioId = job.get("scenarioId").asText();
            JsonNode cmNode = job.get("customModel").get("convert");
            CustomModel cm = cmNode == null || cmNode.isNull() ? null : JSON.treeToValue(cmNode, CustomModel.class);
            double lat = job.get("start").get("lat").asDouble(), lon = job.get("start").get("lng").asDouble();
            double heading = job.get("heading").isNull() ? Double.NaN : job.get("heading").asDouble();
            List<List<double[]>> mem = shown.computeIfAbsent(scenarioId, k -> new ArrayList<>());
            long t0 = System.nanoTime();
            List<LoopAutoRoute.Route> routes = engine.generate(job.get("profile").asText(), cm, lat, lon,
                    job.get("targetM").asDouble(), heading, job.get("genSeed").asLong(), mem);
            long ms = (System.nanoTime() - t0) / 1_000_000;
            for (LoopAutoRoute.Route r : routes) mem.add(r.coords);

            String where = scenarioId + " r" + job.get("regen").asInt();
            JsonNode stored = JSON.readTree(new File(gens, job.get("outputFile").asText())).get("routes");
            assertEquals(stored.size(), routes.size(), where + ": route count");
            for (int i = 0; i < routes.size(); i++) {
                LoopAutoRoute.Route r = routes.get(i);
                JsonNode s = stored.get(i);
                String at = where + " slot " + (i + 1);
                assertEquals(s.get("label").asText(), r.source + " " + r.label, at + ": label");
                assertEquals(s.get("info").get("role").asText(), r.role, at + ": role");
                assertEquals(Math.round(r.distance), s.get("distance").asLong(), at + ": distance");
                List<double[]> wp = new ArrayList<>();
                wp.add(new double[]{lat, lon});
                wp.addAll(r.waypoints);
                wp.add(new double[]{lat, lon});
                assertSame(s.get("waypoints"), wp, at + ": waypoints");
                assertSame(s.get("coords"), r.coords, at + ": coords");
            }
            presses++;
            System.out.println(where + " " + ms + " ms: " + routes.size() + " routes identical");
        }
        return presses;
    }

    private static void assertSame(JsonNode stored, List<double[]> actual, String at) {
        assertEquals(stored.size(), actual.size(), at + " count");
        for (int k = 0; k < actual.size(); k++) {
            assertEquals(stored.get(k).get(0).asDouble(), round(actual.get(k)[0]), 0, at + " [" + k + "] lat");
            assertEquals(stored.get(k).get(1).asDouble(), round(actual.get(k)[1]), 0, at + " [" + k + "] lon");
        }
    }

    private static double round(double v) {
        return Math.round(v * 1e6) / 1e6;
    }
}
