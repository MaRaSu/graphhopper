package com.graphhopper.trailmap.tilerouting;

import com.fasterxml.jackson.databind.JsonNode;
import com.graphhopper.trailmap.tilerouting.foundation.Candidate;
import com.graphhopper.trailmap.tilerouting.foundation.Foundation;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.util.*;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * /tile_route memory bar (CATALOG §10, test lead 2026-10-31): the search state a request keeps alive is bounded by the
 * searches' own caps (settle caps, the table radius, the home flood's 40 km), not by the graph. Per manifest job: solve
 * the job as TileRouter does, keep the foundation, home and candidate alive, and weigh them: used heap after GC with
 * them minus after releasing them. Asserts retained ≤ {@code tr.mem.maxMB} (300) per request, and prints keysHeld and
 * the measured bytes per held key. Allocation per request is printed as a GC-pressure signal only.
 * <pre>
 * mvn -pl core test -Dtest=TileRouteMemoryTest -DargLine=-Xmx12g -Dtr.parity.root=ABS/routing_v2 \
 *   -Dtr.parity.graph=ABS/data/experiments/tile_routing/r4-polish/graph [-Dtr.parity.only=regex]
 * </pre>
 */
public class TileRouteMemoryTest {

    @BeforeAll
    static void load() throws Exception {
        TileRouteParityTest.load();
    }

    @AfterAll
    static void close() {
        TileRouteParityTest.close();
    }

    static final MemoryMXBean MEM = ManagementFactory.getMemoryMXBean();

    /** Used heap after a few full collections. */
    static long usedAfterGc() throws InterruptedException {
        long used = Long.MAX_VALUE;
        for (int i = 0; i < 4; i++) {
            MEM.gc();
            Thread.sleep(50);
            used = Math.min(used, MEM.getHeapMemoryUsage().getUsed());
        }
        return used;
    }

    @Test
    void retainedPerRequest() throws Exception {
        long maxMB = Long.getLong("tr.mem.maxMB", 300);
        String only = System.getProperty("tr.parity.only");
        TileRouter.Config rc = new TileRouter.Config();
        rc.portalBudgetMs = 5000;
        rc.heuristic = TileRouter.Heuristic.INLINE;
        rc.clock = () -> TileRouter.virtualClock(2_000_000);
        TileRouter router = new TileRouter(TileRouteParityTest.hopper, rc);
        File root = TileRouteParityTest.root;
        JsonNode manifest = TileRouteParityTest.JSON.readTree(new File(root, "tile-routing/verify/parity-manifest-virtual.json"));
        // the first request per profile × safety measures the A* bound once (kept for the JVM's life, one double)
        List<String> over = new ArrayList<>();
        int jobs = 0;
        for (JsonNode run : manifest.get("runs")) {
            JsonNode jobsFile = TileRouteParityTest.JSON.readTree(new File(root, run.get("jobs").asText()));
            for (JsonNode job : jobsFile.get("jobs")) {
                String label = run.get("run").asText() + " " + job.get("jobId").asText();
                if (only != null && !label.matches(only)) continue;
                TileRouteRequest rq = TileRouteParityTest.request(jobsFile, job, run);
                if (jobs == 0) router.route(rq); // class loading and JIT outside the measurement
                long before = usedAfterGc();
                long a0 = TileRouteParityTest.allocated();
                Held h = solve(router, rq);
                long alloc = TileRouteParityTest.allocated() - a0;
                long with = usedAfterGc();
                long keys = h.keysHeld();
                String caps = h.caps();
                h = null;
                long after = usedAfterGc();
                long retained = Math.max(0, with - Math.min(before, after));
                jobs++;
                System.out.println("RETAINED " + label + " MB=" + retained / 1_000_000 + " keysHeld=" + keys
                        + " bytesPerKey=" + (keys > 0 ? retained / keys : -1) + " allocMB=" + alloc / 1_000_000 + " " + caps);
                if (retained > maxMB * 1_000_000) over.add(label + " retained " + retained / 1_000_000 + " MB");
            }
        }
        System.out.println("memory jobs=" + jobs + " over=" + over.size());
        assertTrue(jobs > 0, "nothing run");
        assertTrue(over.isEmpty(), "retained above " + maxMB + " MB:\n" + String.join("\n", over));
    }

    /** What a request keeps alive while it runs. */
    static final class Held {
        Foundation f;
        Foundation.Home home;
        Candidate cand;
        Candidate.Solution sol;

        long keysHeld() {
            return home.fwd.size() + home.back.size() + f.dijkstra.peakKeys() + sol.keysHeld;
        }

        String caps() {
            return "homeFloodKeys=" + (home.fwd.size() + home.back.size()) + " floodBoundedAt" + Math.round(f.params.reachRadiusM / 1000) + "km="
                    + (home.fwdBounded || home.backBounded) + " foundationPeak=" + f.dijkstra.peakKeys() + " candidateKeys=" + sol.keysHeld
                    + " settleCap=" + f.params.maxSettled + " unresolvedLegs=" + sol.unresolvedLegs;
        }
    }

    /** The A* bound per profile × safety, as TileRouter keeps it. */
    static final Map<String, Double> HEURISTIC = new HashMap<>();

    /** TileRouter.route up to the solve, holding its state. */
    static Held solve(TileRouter router, TileRouteRequest rq) {
        Held h = new Held();
        Candidate cand = router.candidate(rq);
        Foundation.Params params = new Foundation.Params();
        params.marginM = rq.zoom == 17 ? 20 : 40;
        params.walkMaxM = !Double.isNaN(rq.walkInMaxM) ? rq.walkInMaxM : 300;
        cand.configure(params);
        Foundation f = new Foundation(TileRouteParityTest.hopper, rq.profile, TileRouter.safetyModel(rq.safety), params);
        f.dijkstra.minWeightPerM = HEURISTIC.computeIfAbsent(rq.profile + "|" + rq.safety, k -> f.tightenHeuristic()[1]);
        Foundation.Home home = f.home(rq.startLat, rq.startLon);
        Candidate.Problem pb = new Candidate.Problem();
        pb.jobId = rq.seedKey;
        pb.foundation = f;
        pb.home = home;
        pb.ordered = true;
        pb.loop = true;
        if (!rq.groups.isEmpty()) {
            pb.groupRank = new HashMap<>();
            pb.groupCount = rq.groups.size();
            for (int g = 0; g < rq.groups.size(); g++)
                for (String k : rq.groups.get(g)) pb.groupRank.putIfAbsent(k, g);
        }
        for (String k : rq.tiles) {
            Foundation.TileInfo ti = f.analyse(k, rq.zoom, home);
            if ("reachable".equals(ti.status) || ti.walk) pb.tiles.add(ti);
        }
        h.f = f;
        h.home = home;
        h.cand = cand;
        h.sol = cand.solve(pb);
        return h;
    }
}
