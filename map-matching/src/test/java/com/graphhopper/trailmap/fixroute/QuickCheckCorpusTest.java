package com.graphhopper.trailmap.fixroute;

import com.graphhopper.GraphHopper;
import com.graphhopper.trailmap.shared.EdgeKeyMatching;
import com.graphhopper.util.CustomModel;
import com.graphhopper.util.shapes.GHPoint;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 * Step 3: the per-leg quick check over the real-route corpus, with the side-by-side comparison of
 * the two candidate verdicts (owner decision 2026-09-24: "A for starters, switch to B if A is not
 * proven").
 * <ul>
 *   <li><b>A (geometry)</b> — route the leg exactly as the client does ({@link ClientRoute}) and test
 *       whether it follows the leg's slice of the saved track ({@link PathSimilarity}).</li>
 *   <li><b>B (edges)</b> — map-match the slice onto today's graph with the leg's own settings
 *       ({@link ReferenceMatcher}) and compare road edges with the same route.</li>
 * </ul>
 * Every A/B disagreement is written as GeoJSON (saved slice blue, new route red) to
 * {@code data/fix-route-out/quickcheck/} for visual review, plus a per-leg CSV of all legs.
 *
 * <p>Graph-dependent: skipped when the local graph cache is absent. Legs outside the graph's
 * coverage (routing error on the client-identical request) are counted separately.
 */
public class QuickCheckCorpusTest {

    static final double TOL_M = 12.0;           // materiality_max_m
    static final double LEN_RATIO = 0.03;        // materiality_len_ratio
    static final double LEN_SLACK_M = 20.0;      // absolute floor for short legs
    static final double BUMP_TOL_M = 25.0;       // tiered variant: local bump tolerance
    static final double MAX_BUMP_M = 30.0;       // tiered variant: longest allowed bump
    static final double ANCHOR_WINDOW_M = 30.0; // = waypoint_align_max_m
    static final String OUT_DIR = "../../data/fix-route-out/quickcheck";

    private static GraphHopper hopper;

    @BeforeAll
    static void setup() throws Exception {
        Assumptions.assumeTrue(FixRouteFixtures.available(), "fixtures not found");
        Assumptions.assumeTrue(FixRouteTestGraph.available(), "graph cache not found");
        hopper = FixRouteTestGraph.open();
    }

    @AfterAll
    static void teardown() {
        if (hopper != null) hopper.close();
    }

    record LegOutcome(long routeId, int leg, String profile, boolean routedOk, String error,
                      boolean aPass, boolean aTier, boolean bPass, boolean bGap, PathSimilarity.Result sim,
                      double routeM, double sliceM, int routeEdges, int matchedEdges,
                      long routeMs, long matchMs, List<double[]> slice, List<double[]> route, String bDiff) {
    }

    @Test
    void corpus() throws Exception {
        String only = System.getProperty("fix.routes");
        List<Long> ids = new ArrayList<>();
        if (only != null && !only.isBlank()) {
            for (String s : only.split(",")) ids.add(Long.parseLong(s.trim()));
        } else {
            ids = FixRouteFixtures.allIds();
        }
        File out = new File(OUT_DIR);
        out.mkdirs();
        for (File f : out.listFiles()) if (f.getName().endsWith(".geojson")) f.delete();

        int threads = Integer.getInteger("fix.threads", Math.max(2, Runtime.getRuntime().availableProcessors() - 2));
        ExecutorService pool = Executors.newFixedThreadPool(threads);
        List<Future<LegOutcome>> futures = new ArrayList<>();
        long t0 = System.nanoTime();
        for (long id : ids) {
            FixRouteFixtures.Route r = FixRouteFixtures.load(id);
            WaypointAligner.Result al = new WaypointAligner(30.0, 0.5)
                    .align(r.reference, r.waypoints, r.savedSegmentLengths);
            ReferenceTrack ref = new ReferenceTrack(r.reference);
            for (FixRouteFixtures.Segment seg : r.segments) {
                if (!"followRoads".equals(seg.type)) continue;
                WaypointAligner.Placement a = al.placements().get(seg.startWaypointIndex);
                WaypointAligner.Placement b = al.placements().get(seg.endWaypointIndex);
                if (!a.aligned() || !b.aligned()) continue;
                futures.add(pool.submit(() -> checkLeg(r, seg, ref, a.arcM(), b.arcM())));
            }
        }
        List<LegOutcome> outcomes = new ArrayList<>();
        for (Future<LegOutcome> f : futures) outcomes.add(f.get());
        pool.shutdown();
        double wallS = (System.nanoTime() - t0) / 1e9;

        int total = 0, routeErr = 0, both = 0, aOnly = 0, bOnly = 0, neither = 0, gaps = 0;
        int tBoth = 0, tAOnly = 0, tBOnly = 0, tNeither = 0;
        long routeMs = 0, matchMs = 0;
        try (Writer csv = new FileWriter(new File(out, "legs.csv"))) {
            csv.write("route,leg,profile,routed,a_pass,a_tier,b_pass,b_gap,frechet_within,failed_at_m,route_to_ref_max_m,"
                    + "ref_to_route_max_m,route_m,slice_m,route_edges,matched_edges,route_ms,match_ms,b_diff,error\n");
            for (LegOutcome o : outcomes) {
                total++;
                routeMs += o.routeMs;
                matchMs += o.matchMs;
                if (!o.routedOk) {
                    routeErr++;
                } else {
                    if (o.bGap) gaps++;
                    if (o.aPass && o.bPass) both++;
                    else if (o.aPass) aOnly++;
                    else if (o.bPass) bOnly++;
                    else neither++;
                    if (o.aTier && o.bPass) tBoth++;
                    else if (o.aTier) tAOnly++;
                    else if (o.bPass) tBOnly++;
                    else tNeither++;
                    if (o.aPass != o.bPass) writeGeoJson(out, o, o.aPass ? "A-only" : "B-only");
                    else if (o.aTier && !o.aPass) writeGeoJson(out, o, "bump");
                }
                PathSimilarity.Result s = o.sim;
                csv.write(String.format(Locale.ROOT, "%d,%d,%s,%b,%b,%b,%b,%b,%s,%s,%s,%s,%.1f,%.1f,%d,%d,%d,%d,%s,%s%n",
                        o.routeId, o.leg, o.profile, o.routedOk, o.aPass, o.aTier, o.bPass, o.bGap,
                        s == null ? "" : s.frechetWithin(), s == null ? "" : f1(s.failedAtRouteM()),
                        s == null ? "" : f1(s.routeToRefMaxM()), s == null ? "" : f1(s.refToRouteMaxM()),
                        o.routeM, o.sliceM, o.routeEdges, o.matchedEdges, o.routeMs, o.matchMs, o.bDiff,
                        o.error == null ? "" : o.error.replace(',', ';').replace('\n', ' ')));
            }
        }
        System.out.printf(Locale.ROOT, "QUICKCHECK legs=%d routeErrors=%d | A&B pass=%d  A-only=%d  B-only=%d  "
                        + "neither=%d | B matcher gaps=%d | routing %.1fs, matching %.1fs (cpu), wall %.1fs on %d threads%n",
                total, routeErr, both, aOnly, bOnly, neither, gaps, routeMs / 1e3, matchMs / 1e3, wallS, threads);
        System.out.printf(Locale.ROOT, "QUICKCHECK tiered(bump<=%.0fm within %.0fm): A&B=%d A-only=%d B-only=%d neither=%d%n",
                MAX_BUMP_M, BUMP_TOL_M, tBoth, tAOnly, tBOnly, tNeither);
    }

    static LegOutcome checkLeg(FixRouteFixtures.Route r, FixRouteFixtures.Segment seg, ReferenceTrack ref,
                               double fromArc, double toArc) {
        double[] s = r.waypoints.get(seg.startWaypointIndex), e = r.waypoints.get(seg.endWaypointIndex);
        List<GHPoint> via = new ArrayList<>();
        for (double[] v : seg.viaPoints) via.add(new GHPoint(v[0], v[1]));
        CustomModel cm = FixRouteTestGraph.customModel(seg.customModel);
        ClientRoute.Settings settings = new ClientRoute.Settings(seg.profile, cm, r.snapPreventions);

        long t1 = System.nanoTime();
        ClientRoute.Leg leg = ClientRoute.route(hopper, settings, new GHPoint(s[0], s[1]), via,
                new GHPoint(e[0], e[1]), seg.initialHeading, seg.headingPenalty);
        long routeMs = (System.nanoTime() - t1) / 1_000_000;
        List<double[]> slice;
        if (leg.ok()) {
            double[] rs = leg.points().get(0), re = leg.points().get(leg.points().size() - 1);
            // A legacy spike arm is as long as the waypoint is off the road, i.e. about the
            // distance from the waypoint to the route's own (snapped) end point.
            double ws = Math.max(ANCHOR_WINDOW_M, dist(s, rs) + 10);
            double we = Math.max(ANCHOR_WINDOW_M, dist(e, re) + 10);
            slice = ref.anchoredSlice(fromArc, toArc, rs, re, ws, we);
        } else {
            slice = ref.slice(fromArc, toArc);
        }
        double sliceM = PathSimilarity.length(slice);
        if (!leg.ok()) {
            return new LegOutcome(r.routeId, seg.index, seg.profile, false, leg.error(), false, false, false, false,
                    null, 0, sliceM, 0, 0, routeMs, 0, slice, null, "");
        }
        PathSimilarity.Result sim = PathSimilarity.compare(leg.points(), slice, TOL_M, BUMP_TOL_M);
        boolean aPass = sim.follows(LEN_RATIO, LEN_SLACK_M);
        boolean aTier = sim.followsTiered(MAX_BUMP_M, LEN_RATIO, LEN_SLACK_M);

        long t2 = System.nanoTime();
        boolean bPass = false, bGap = false;
        int matched = 0;
        String err = null;
        String bDiff = "";
        try {
            ReferenceMatcher.Match m = new ReferenceMatcher(hopper).match(slice, seg.profile, cm);
            bGap = m.hasGap();
            matched = m.edgeKeys().length;
            EdgeKeyMatching ekm = new EdgeKeyMatching(hopper.getBaseGraph());
            bPass = ekm.matches(m.edgeKeys(), leg.edgeKeys(), true);
            if (!bPass) bDiff = edgeDiff(ekm.twinCanonicalize(m.edgeKeys()), ekm.twinCanonicalize(leg.edgeKeys()));
        } catch (Exception ex) {
            err = "match: " + ex.getClass().getSimpleName() + ": " + ex.getMessage();
        }
        long matchMs = (System.nanoTime() - t2) / 1_000_000;
        return new LegOutcome(r.routeId, seg.index, seg.profile, true, err, aPass, aTier, bPass, bGap, sim,
                leg.distanceM(), sliceM, leg.edgeKeys().length, matched, routeMs, matchMs, slice, leg.points(), bDiff);
    }

    /**
     * How the matched and routed edge sequences differ: "ends" when one is the other plus extra
     * edges only at the start and/or end (a boundary artifact of where the slice / route begins),
     * otherwise "interior:<n>" with the number of differing interior edges (by longest common
     * contiguous run).
     */
    static String edgeDiff(int[] matched, int[] route) {
        int[] big = matched.length >= route.length ? matched : route;
        int[] small = big == matched ? route : matched;
        if (small.length == 0) return "empty";
        for (int off = 0; off + small.length <= big.length; off++) {
            if (java.util.Arrays.equals(java.util.Arrays.copyOfRange(big, off, off + small.length), small)) {
                return "ends:" + (big.length - small.length);
            }
        }
        int pre = 0;
        while (pre < matched.length && pre < route.length && matched[pre] == route[pre]) pre++;
        int suf = 0;
        while (suf < matched.length - pre && suf < route.length - pre
                && matched[matched.length - 1 - suf] == route[route.length - 1 - suf]) suf++;
        return "interior:" + Math.max(matched.length, route.length - 0) + "-" + (pre + suf);
    }

    private static double dist(double[] a, double[] b) {
        return com.graphhopper.util.DistanceCalcEarth.DIST_EARTH.calcDist(a[0], a[1], b[0], b[1]);
    }

    private static void writeGeoJson(File dir, LegOutcome o, String cat) throws IOException {
        File f = new File(dir, String.format("%s_route-%d_leg-%d.geojson", cat, o.routeId, o.leg));
        PathSimilarity.Result s = o.sim;
        String props = String.format(Locale.ROOT,
                "\"route\":%d,\"leg\":%d,\"profile\":\"%s\",\"a_pass\":%b,\"a_tier\":%b,\"b_pass\":%b,\"b_gap\":%b,\"excess_run_m\":%.0f,"
                        + "\"route_to_ref_max_m\":%.1f,\"ref_to_route_max_m\":%.1f,\"route_m\":%.1f,\"slice_m\":%.1f",
                o.routeId, o.leg, o.profile, o.aPass, o.aTier, o.bPass, o.bGap, s.longestExcessRunM(), s.routeToRefMaxM(), s.refToRouteMaxM(),
                o.routeM, o.sliceM);
        try (Writer w = new FileWriter(f)) {
            w.write("{\"type\":\"FeatureCollection\",\"features\":[");
            w.write(feature(o.slice, "saved", "#1f6feb", props));
            w.write(",");
            w.write(feature(o.route, "new_route", "#d1242f", props));
            w.write("]}");
        }
    }

    private static String feature(List<double[]> line, String name, String color, String props) {
        StringBuilder sb = new StringBuilder();
        sb.append("{\"type\":\"Feature\",\"properties\":{\"name\":\"").append(name)
                .append("\",\"stroke\":\"").append(color).append("\",\"stroke-width\":3,").append(props)
                .append("},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
        for (int i = 0; i < line.size(); i++) {
            if (i > 0) sb.append(',');
            sb.append(String.format(Locale.ROOT, "[%.7f,%.7f]", line.get(i)[1], line.get(i)[0]));
        }
        return sb.append("]}}").toString();
    }

    private static String f1(double v) {
        return Double.isNaN(v) ? "" : String.format(Locale.ROOT, "%.1f", v);
    }
}
