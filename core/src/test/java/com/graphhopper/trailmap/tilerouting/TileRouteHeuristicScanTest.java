package com.graphhopper.trailmap.tilerouting;

import com.fasterxml.jackson.databind.JsonNode;
import com.graphhopper.trailmap.tilerouting.foundation.Foundation;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.io.File;

import static org.junit.jupiter.api.Assertions.*;

/**
 * W21 F1 measurement: time and allocation of {@link Foundation#tightenHeuristic()} (the full edge scan behind
 * tour_repair's A* bound) per profile × safety, with the graph's edge count, so it can be scaled to the production
 * graph. Prints only; asserts nothing.
 * <pre>
 * mvn -pl core test -Dtest=TileRouteHeuristicScanTest -DargLine=-Xmx12g -Dtr.parity.root=ABS/routing_v2 -Dtr.parity.graph=ABS/graph
 * </pre>
 */
public class TileRouteHeuristicScanTest {

    @BeforeAll
    static void load() throws Exception {
        TileRouteParityTest.load();
    }

    @AfterAll
    static void close() {
        TileRouteParityTest.close();
    }

    /**
     * Server mode: a request never measures the bound itself. The first tour_repair request routes with the weighting's
     * own bound (or a ready one) and queues the measurement; the measured bounds for its profile's three safety levels
     * appear in the background and equal the inline measurement.
     */
    @Test
    void backgroundBound() throws Exception {
        TileRouter.Config rc = new TileRouter.Config();
        assertEquals(TileRouter.Heuristic.BACKGROUND, rc.heuristic, "the server default");
        rc.portalBudgetMs = 5000;
        TileRouter router = new TileRouter(TileRouteParityTest.hopper, rc);
        File root = TileRouteParityTest.root;
        JsonNode manifest = TileRouteParityTest.JSON.readTree(new File(root, "tile-routing/verify/parity-manifest-virtual.json"));
        TileRouteRequest rq = null;
        for (JsonNode run : manifest.get("runs")) {
            JsonNode jobsFile = TileRouteParityTest.JSON.readTree(new File(root, run.get("jobs").asText()));
            TileRouteRequest r = TileRouteParityTest.request(jobsFile, jobsFile.get("jobs").get(0), run);
            if ("tour_repair".equals(r.algorithm)) {
                rq = r;
                break;
            }
        }
        assertNotNull(rq, "a tour_repair job in the manifest");
        TileRouter.Result res = router.route(rq);
        assertNotNull(res.route, "the first tour_repair request routes");
        long t0 = System.nanoTime();
        for (String safety : new String[]{"standard", "strict", "no_trails"}) {
            while (!TileRouter.boundReady(TileRouteParityTest.hopper, rq.profile, safety)) {
                assertTrue(System.nanoTime() - t0 < 60_000_000_000L, "bound " + rq.profile + " " + safety + " within 60 s");
                Thread.sleep(50);
            }
        }
        System.out.println("SCAN background: 3 bounds of " + rq.profile + " ready " + (System.nanoTime() - t0) / 1_000_000 + " ms after the first request (" + res.totalMs + " ms)");
        // a ready bound is the inline one: the next tour_repair request equals an INLINE run
        TileRouter.Config inline = new TileRouter.Config();
        inline.portalBudgetMs = 5000;
        inline.heuristic = TileRouter.Heuristic.INLINE;
        inline.clock = rc.clock = () -> TileRouter.virtualClock(2_000_000);
        String a = TileRouteJson.write(TileRouteParityTest.JSON, rq, new TileRouter(TileRouteParityTest.hopper, rc).route(rq)).get("waypoints").toString();
        String b = TileRouteJson.write(TileRouteParityTest.JSON, rq, new TileRouter(TileRouteParityTest.hopper, inline).route(rq)).get("waypoints").toString();
        assertEquals(b, a, "BACKGROUND with its bound ready == INLINE");
    }

    @Test
    void scanCost() {
        int edges = TileRouteParityTest.hopper.getBaseGraph().getEdges();
        System.out.println("SCAN graph edges=" + edges + " nodes=" + TileRouteParityTest.hopper.getBaseGraph().getNodes());
        String[] profiles = System.getProperty("tr.scan.profiles", "gravel,roadbike,mtb,safebike").split(",");
        for (String profile : profiles) {
            for (String safety : new String[]{"standard", "strict", "no_trails"}) {
                StringBuilder sb = new StringBuilder("SCAN " + profile + " " + safety + ":");
                for (int run = 0; run < 3; run++) {
                    Foundation f = new Foundation(TileRouteParityTest.hopper, profile, TileRouter.safetyModel(safety), new Foundation.Params());
                    long a0 = TileRouteParityTest.allocated();
                    double[] r = f.tightenHeuristic();
                    long alloc = TileRouteParityTest.allocated() - a0;
                    sb.append(String.format(" run%d %.0f ms alloc %.1f MB bound %.6f (own %.6f);", run, r[2], alloc / 1e6, r[1], r[0]));
                }
                System.out.println(sb);
            }
        }
    }
}
