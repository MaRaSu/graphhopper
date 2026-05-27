package com.graphhopper.trailmap.convert;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.PropertyNamingStrategies;
import com.fasterxml.jackson.databind.SerializationFeature;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.ResponsePath;
import com.graphhopper.jackson.GraphHopperModule;
import com.graphhopper.matching.EdgeMatch;
import com.graphhopper.matching.MapMatching;
import com.graphhopper.matching.MatchResult;
import com.graphhopper.matching.Observation;
import com.graphhopper.matching.Tracepoint;
import com.graphhopper.trailmap.TrailmapGraphHopper;
import com.graphhopper.trailmap.shared.TrailmapImportRegistry;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.FetchMode;
import com.graphhopper.util.PMap;
import com.graphhopper.util.PointList;
import com.graphhopper.util.details.PathDetail;
import com.graphhopper.util.shapes.GHPoint;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import java.io.File;
import java.io.FileInputStream;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Diagnostic / debugging harness for {@link TrackToRouteConverter}.
 *
 * <p>Verbose stdout, minimal assertions. Used for ad-hoc exploration when adding a new
 * GPX case or investigating a quality issue. <b>Not</b> a regression suite — see
 * {@code TrackConvertValidationTest} for that.
 *
 * <p>Run with:
 * <pre>
 *   mvn -f /path/to/graphhopper/pom.xml -pl map-matching test \
 *       -Dtest=TrackConvertDiagnosticTest#convertGpx \
 *       -Dconvert.gpx=/path/to/track.gpx \
 *       -Dconvert.profile=gravel \
 *       -DargLine=-Xmx16g -Dsurefire.useFile=false
 * </pre>
 *
 * <p>Workflow when adding a new GPX case:
 * <ol>
 *   <li>Run {@link #convertGpx()} with the new GPX. Inspect printed response.</li>
 *   <li>Iterate on the algorithm if needed (still in diagnostic mode).</li>
 *   <li>Once satisfied, copy the printed {@link #emitFixtureBlock(ConvertTrackResponse) fixture block}
 *       into {@code data/gpx-for-testing/test_cases.json}.</li>
 *   <li>Run {@code TrackConvertValidationTest} to lock the baseline in.</li>
 * </ol>
 */
public class TrackConvertDiagnosticTest {

    // Paths relative to map-matching/ (Maven working directory)
    private static final String GRAPH_LOCATION = "../../data/graph-cache";
    private static final String OSM_FILE = "../../data/finland_3.osm.pbf";
    private static final String CONFIG_FILE = "../trailmap-config.yml";

    private static GraphHopper hopper;

    @BeforeAll
    static void setup() throws Exception {
        File graphDir = new File(GRAPH_LOCATION);
        Assumptions.assumeTrue(graphDir.exists() && graphDir.isDirectory(),
                "Graph cache not found at " + graphDir.getAbsolutePath() + " — skipping convert test");

        ObjectMapper yamlMapper = new ObjectMapper(new YAMLFactory());
        yamlMapper.setPropertyNamingStrategy(PropertyNamingStrategies.SNAKE_CASE);
        yamlMapper.registerModule(new GraphHopperModule());

        JsonNode root = yamlMapper.readTree(new FileInputStream(new File(CONFIG_FILE)));
        JsonNode ghNode = root.get("graphhopper");
        assertNotNull(ghNode, "trailmap-config.yml must have a 'graphhopper' top-level key");

        GraphHopperConfig config = yamlMapper.treeToValue(ghNode, GraphHopperConfig.class);
        config.putObject("graph.location", GRAPH_LOCATION);
        config.putObject("datareader.file", OSM_FILE);

        hopper = new TrailmapGraphHopper();
        hopper.setImportRegistry(new TrailmapImportRegistry());
        hopper.setAllowWrites(false);
        hopper.init(config);
        hopper.importOrLoad();
    }

    @AfterAll
    static void teardown() {
        if (hopper != null) {
            hopper.close();
        }
    }

    /**
     * Run the converter against a GPX file. Pretty-prints the full response plus a
     * compact segment summary and (if computable) faithfulness metrics. Use this for
     * exploring new GPX cases and debugging quality issues.
     *
     * <p>Pass {@code -Dconvert.gpx=/path/to/file.gpx} and optionally
     * {@code -Dconvert.profile=...} (default {@code gravel}).
     */
    @Test
    void convertGpx() throws Exception {
        String gpxPath = System.getProperty("convert.gpx");
        Assumptions.assumeTrue(gpxPath != null,
                "Pass -Dconvert.gpx=/abs/path/to/file.gpx to run this test");
        String profile = System.getProperty("convert.profile", "gravel");
        String matchingProfile = System.getProperty("convert.matchingProfile", profile);
        double gpsAccuracy = Double.parseDouble(System.getProperty("convert.gpsAccuracy", "5"));
        double snapThreshold = Double.parseDouble(System.getProperty("convert.snapThreshold", "35"));

        List<Observation> observations = parseGpx(new File(gpxPath));
        System.out.println("Loaded " + observations.size() + " observations from " + gpxPath
                + " (profile=" + profile
                + (matchingProfile.equals(profile) ? "" : ", matching_profile=" + matchingProfile)
                + ")");

        PMap hints = new PMap();
        hints.putObject("profile", matchingProfile);
        MapMatching matching = MapMatching.fromGraphHopper(hopper, hints);
        matching.setMeasurementErrorSigma(gpsAccuracy);

        long t0 = System.currentTimeMillis();
        MatchResult matchResult = matching.match(observations);
        long tMatched = System.currentTimeMillis();
        System.out.println("MapMatching: " + (tMatched - t0) + "ms, "
                + matchResult.getEdgeMatches().size() + " edge matches, "
                + (matchResult.getTracepoints() != null ? matchResult.getTracepoints().size() : 0) + " tracepoints, "
                + "match length " + Math.round(matchResult.getMatchLength()) + "m");

        TrackToRouteConverter converter = new TrackToRouteConverter(hopper);
        boolean debug = Boolean.parseBoolean(System.getProperty("convert.debug", "false"));
        ConvertTrackResponse response = converter.convert(
                matchResult, observations, profile, null,
                snapThreshold,
                TrackToRouteConverter.DEFAULT_MIN_ROUTED_SEGMENT_M,
                TrackToRouteConverter.DEFAULT_COORDINATES_SIMPLIFY_EPS_M,
                TrackToRouteConverter.DEFAULT_MIN_DETOUR_M,
                TrackToRouteConverter.DEFAULT_MAX_DETOUR_RATIO,
                debug);
        long t1 = System.currentTimeMillis();

        // --- Summary ---
        System.out.println("Conversion total: " + (t1 - t0) + "ms");
        System.out.println(ConvertValidationHelpers.summarizeSegments(response));

        // --- Full JSON ---
        ObjectMapper out = new ObjectMapper();
        out.enable(SerializationFeature.INDENT_OUTPUT);
        System.out.println("=== FULL RESPONSE ===");
        System.out.println(out.writeValueAsString(response));

        // --- Faithfulness metrics ---
        PointList realized = ConvertValidationHelpers.realizePolyline(response, hopper, profile, null);
        double[] devs = ConvertValidationHelpers.perPointDeviations(observations, realized);
        double maxDev = ConvertValidationHelpers.maxDeviation(devs);
        double meanDev = ConvertValidationHelpers.meanDeviation(devs);
        int worstIdx = ConvertValidationHelpers.worstDeviationIndex(devs);
        GHPoint worstP = observations.get(worstIdx).getPoint();
        System.out.printf("Faithfulness: realized polyline %d points; "
                        + "max deviation %.2fm (gpx[%d]=%.6f,%.6f), mean %.2fm%n",
                realized.size(), maxDev, worstIdx, worstP.lat, worstP.lon, meanDev);

        // --- Paste-ready fixture block ---
        System.out.println();
        System.out.println("=== FIXTURE BLOCK (paste into data/gpx-for-testing/test_cases.json) ===");
        System.out.println(emitFixtureBlock(response, maxDev, meanDev, gpxPath, profile));
        System.out.println("=== END FIXTURE BLOCK ===");

        // Shape sanity only — diagnostic test, not a regression check.
        assertNotNull(response.getWaypoints());
        assertNotNull(response.getSegments());
        assertFalse(response.getSegments().isEmpty(), "Should produce at least one segment");
        assertTrue(response.getStats().totalDistanceM > 0, "Total distance should be positive");
    }

    /**
     * Deep diagnostic dump for investigating waypoint-optimization issues.
     *
     * <p>Replicates the segmenter + optimizer logic locally with full per-step printing —
     * does NOT call the production converter, so we can see edge-by-edge what the
     * algorithm "sees" without touching production code.
     *
     * <p>What it prints:
     * <ol>
     *   <li>Per-observation: original GPS, snap position, snap distance, edge id, matched/filtered flags</li>
     *   <li>The full EdgeMatch list (edge ids + distances)</li>
     *   <li>The obs → EdgeMatch index mapping (with notes when forward/backward search was needed)</li>
     *   <li>Snap-based good/bad classification, with explicit notes when de-isolation flipped a value</li>
     *   <li>For each Matched region: full edge slice + per-obs slice-index mapping</li>
     *   <li>For each Matched region's optimizer: for every probe, the expected edge slice vs the
     *       actual route's edge sequence, dedup'd, with a clear pass/fail verdict and reason</li>
     * </ol>
     *
     * <p>Run: {@code -Dtest=TrackConvertDiagnosticTest#diagnoseProbes
     *   -Dconvert.gpx=/abs/path.gpx -Dconvert.profile=... }
     */
    @Test
    void diagnoseProbes() throws Exception {
        String gpxPath = System.getProperty("convert.gpx");
        Assumptions.assumeTrue(gpxPath != null,
                "Pass -Dconvert.gpx=/abs/path/to/file.gpx to run this test");
        String profile = System.getProperty("convert.profile", "gravel");
        double gpsAccuracy = Double.parseDouble(System.getProperty("convert.gpsAccuracy", "5"));
        double snapThreshold = Double.parseDouble(System.getProperty("convert.snapThreshold", "35"));

        List<Observation> observations = parseGpx(new File(gpxPath));
        System.out.println("=== INPUT ===");
        System.out.println("gpx=" + gpxPath);
        System.out.println("profile=" + profile + " gpsAccuracy=" + gpsAccuracy + " snapThreshold=" + snapThreshold);
        System.out.println("observations=" + observations.size());

        PMap hints = new PMap();
        hints.putObject("profile", profile);
        MapMatching matching = MapMatching.fromGraphHopper(hopper, hints);
        matching.setMeasurementErrorSigma(gpsAccuracy);
        MatchResult mr = matching.match(observations);
        System.out.println();
        System.out.println("=== MAPMATCHING RESULT ===");
        System.out.println("edgeMatches=" + mr.getEdgeMatches().size()
                + " tracepoints=" + (mr.getTracepoints() != null ? mr.getTracepoints().size() : 0)
                + " matchLengthM=" + Math.round(mr.getMatchLength()));

        // --- Per-observation tracepoint dump ---
        List<Tracepoint> tps = mr.getTracepoints();
        System.out.println();
        System.out.println("=== TRACEPOINTS (one row per input observation) ===");
        System.out.println("idx |  origin (lat,lon)       |  snap (lat,lon)         | snap_dist | distFromPrev |  edge_id | matched | filtered");
        System.out.println("----|-------------------------|-------------------------|-----------|--------------|----------|---------|---------");
        for (int i = 0; i < tps.size(); i++) {
            Tracepoint tp = tps.get(i);
            String snap = tp.getSnappedPoint() != null
                    ? String.format("(%.6f, %.6f)", tp.getSnappedPoint().lat, tp.getSnappedPoint().lon)
                    : "       N/A          ";
            String d = tp.getDistance() != null ? String.format("%6.1fm", tp.getDistance()) : "    N/A";
            String dfp = tp.getDistanceFromPrevious() != null
                    ? String.format("%9.1fm", tp.getDistanceFromPrevious()) : "      N/A";
            System.out.printf("%3d | (%.6f, %.6f) | %s |  %s  | %s   | %8s | %7s | %8s%n",
                    i,
                    tp.getOriginalPoint().lat, tp.getOriginalPoint().lon,
                    snap, d, dfp,
                    tp.getEdgeId() != null ? tp.getEdgeId().toString() : "    null",
                    tp.isMatched(), tp.isFiltered());
        }

        // --- EdgeMatch list ---
        System.out.println();
        System.out.println("=== EDGE MATCHES (the matched path's edge sequence) ===");
        for (int e = 0; e < mr.getEdgeMatches().size(); e++) {
            EdgeMatch em = mr.getEdgeMatches().get(e);
            PointList g = em.getEdgeState().fetchWayGeometry(FetchMode.ALL);
            System.out.printf("em[%2d] edgeId=%-8d dist=%6.1fm geomPts=%d states=%d%n",
                    e, em.getEdgeState().getEdge(),
                    em.getEdgeState().getDistance(),
                    g.size(), em.getStates().size());
        }

        // --- Replicate obsToEdgeMatch (annotate forward/backward) ---
        int n = tps.size();
        int[] obsToEm = new int[n];
        String[] obsToEmNote = new String[n];
        for (int i = 0; i < n; i++) obsToEm[i] = -1;
        int searchFrom = 0;
        for (int i = 0; i < n; i++) {
            Tracepoint tp = tps.get(i);
            if (!tp.isMatched() || tp.getEdgeId() == null) {
                obsToEmNote[i] = "no edge id";
                continue;
            }
            int target = tp.getEdgeId();
            int found = -1;
            for (int j = searchFrom; j < mr.getEdgeMatches().size(); j++) {
                if (mr.getEdgeMatches().get(j).getEdgeState().getEdge() == target) {
                    found = j; break;
                }
            }
            if (found < 0) {
                for (int j = searchFrom - 1; j >= 0; j--) {
                    if (mr.getEdgeMatches().get(j).getEdgeState().getEdge() == target) {
                        found = j;
                        obsToEmNote[i] = "BACKWARD scan";
                        break;
                    }
                }
            } else {
                obsToEmNote[i] = "forward";
            }
            if (found >= 0) {
                obsToEm[i] = found;
                searchFrom = found;
            } else {
                obsToEmNote[i] = "NOT FOUND in EdgeMatches";
            }
        }

        // --- Snap-based classification + de-isolation ---
        boolean[] good0 = new boolean[n];
        for (int i = 0; i < n; i++) {
            Tracepoint tp = tps.get(i);
            good0[i] = tp.isMatched() && tp.getDistance() != null && tp.getDistance() < snapThreshold;
        }
        boolean[] good = good0.clone();
        boolean[] flipped = new boolean[n];
        if (n >= 3) {
            boolean[] orig = good.clone();
            for (int i = 1; i < n - 1; i++) {
                if (orig[i - 1] == orig[i + 1] && orig[i] != orig[i - 1]) {
                    good[i] = orig[i - 1];
                    flipped[i] = true;
                }
            }
            if (n >= 2 && good[0] != good[1]) { good[0] = good[1]; flipped[0] = true; }
            if (n >= 2 && good[n - 1] != good[n - 2]) { good[n - 1] = good[n - 2]; flipped[n - 1] = true; }
        }
        System.out.println();
        System.out.println("=== OBS → EDGEMATCH MAPPING + CLASSIFICATION ===");
        System.out.println("idx | snap_dist | edgeId   | obsToEm | scan note          | good0 (pre-deiso) | good (final) | flipped?");
        System.out.println("----|-----------|----------|---------|--------------------|--------------------|--------------|----------");
        for (int i = 0; i < n; i++) {
            Tracepoint tp = tps.get(i);
            String d = tp.getDistance() != null ? String.format("%6.1fm", tp.getDistance()) : "    N/A";
            System.out.printf("%3d | %s | %-8s |  em[%3d] | %-18s |   %-15s |    %-9s |   %s%n",
                    i, d,
                    tp.getEdgeId() != null ? tp.getEdgeId().toString() : "null",
                    obsToEm[i], obsToEmNote[i],
                    good0[i] ? "good" : "BAD",
                    good[i] ? "good" : "BAD",
                    flipped[i] ? "*FLIPPED*" : "");
        }

        // --- Region building (mimic) ---
        System.out.println();
        System.out.println("=== REGIONS (snap-based only — detour-split not replayed here) ===");
        int i = 0;
        List<int[]> regs = new ArrayList<>();  // [start, end, isMatched? 1:0]
        while (i < n) {
            int start = i;
            boolean run = good[i];
            int j = i;
            while (j + 1 < n && good[j + 1] == run) j++;
            regs.add(new int[]{start, j, run ? 1 : 0});
            i = j + 1;
        }
        for (int[] r : regs) {
            System.out.printf("  %s [%d..%d]  (%d obs)%n",
                    r[2] == 1 ? "Matched  " : "Unmatched",
                    r[0], r[1], r[1] - r[0] + 1);
        }

        // --- Per matched region: full edge slice + per-probe instrumentation ---
        for (int[] r : regs) {
            if (r[2] != 1) continue;
            int startObs = r[0], endObs = r[1];
            // Build edge slice
            int firstEm = -1, lastEm = -1;
            for (int k = startObs; k <= endObs; k++) {
                if (obsToEm[k] >= 0) {
                    if (firstEm < 0) firstEm = obsToEm[k];
                    lastEm = obsToEm[k];
                }
            }
            if (firstEm < 0) continue;

            // Collect well-mapped obs in region
            List<int[]> wellMapped = new ArrayList<>(); // [obsIdx, edgeIdxInSlice]
            for (int k = startObs; k <= endObs; k++) {
                if (obsToEm[k] < 0) continue;
                Tracepoint tp = tps.get(k);
                if (tp.getSnappedPoint() == null) continue;
                wellMapped.add(new int[]{k, obsToEm[k] - firstEm});
            }
            if (wellMapped.size() < 2) {
                System.out.printf("%n=== REGION Matched [%d..%d]: only %d well-mapped obs, skipping probe diagnostic%n",
                        startObs, endObs, wellMapped.size());
                continue;
            }

            // Print the slice
            System.out.printf("%n=== REGION Matched [%d..%d]  edges em[%d..%d]  (%d edges in slice) ===%n",
                    startObs, endObs, firstEm, lastEm, lastEm - firstEm + 1);
            System.out.print("Edge slice IDs: [");
            for (int e = firstEm; e <= lastEm; e++) {
                if (e > firstEm) System.out.print(", ");
                System.out.print(mr.getEdgeMatches().get(e).getEdgeState().getEdge());
            }
            System.out.println("]");

            System.out.println("Candidates (obs index, edge-idx-in-slice, snap lat,lon):");
            for (int k = 0; k < wellMapped.size(); k++) {
                int obsIdx = wellMapped.get(k)[0];
                int edgeIdx = wellMapped.get(k)[1];
                GHPoint s = tps.get(obsIdx).getSnappedPoint();
                System.out.printf("  c[%2d] obs[%d] edgeIdxInSlice=%d snap=(%.6f, %.6f)%n",
                        k, obsIdx, edgeIdx, s.lat, s.lon);
            }

            // For every consecutive pair AND every long-range probe combination tried by exponential
            // extension, run the probe and print full diagnostics. We don't run the actual optimizer
            // here — just enumerate the probes its loop would issue.
            System.out.println("Probes (replicating exponential-extension + binary-refinement):");
            int cursor = 0;
            int lastCand = wellMapped.size() - 1;
            while (cursor < lastCand) {
                int step = 1;
                int lastOk = -1;
                int lastTried = cursor;
                while (cursor + step <= lastCand) {
                    int probeIdx = Math.min(cursor + step, lastCand);
                    lastTried = probeIdx;
                    boolean ok = probeAndPrint(profile, wellMapped, mr.getEdgeMatches(), firstEm,
                            tps, cursor, probeIdx);
                    if (ok) {
                        lastOk = probeIdx;
                        if (probeIdx == lastCand) break;
                        step *= 2;
                    } else {
                        break;
                    }
                }
                if (lastOk < 0) {
                    // forced single step
                    int next = cursor + 1;
                    probeAndPrint(profile, wellMapped, mr.getEdgeMatches(), firstEm,
                            tps, cursor, next);
                    System.out.printf("  → forced step: cursor %d → %d%n", cursor, next);
                    cursor = next;
                    continue;
                }
                int lo = lastOk + 1;
                int hi = lastTried;
                int best = lastOk;
                while (lo <= hi) {
                    int mid = (lo + hi) / 2;
                    boolean ok = probeAndPrint(profile, wellMapped, mr.getEdgeMatches(), firstEm,
                            tps, cursor, mid);
                    if (ok) { best = mid; lo = mid + 1; }
                    else { hi = mid - 1; }
                }
                System.out.printf("  → cursor %d → %d (best successful probe)%n", cursor, best);
                cursor = best;
            }
        }
    }

    /**
     * Run a probe and print its full input + output. Returns the pass/fail verdict using
     * the same comparison rule the production optimizer uses (interior edges match,
     * up to 1 leading + 2 trailing virtual-edge tolerance).
     */
    private boolean probeAndPrint(String profile,
                                  List<int[]> wellMapped,
                                  List<EdgeMatch> em,
                                  int firstEm,
                                  List<Tracepoint> tps,
                                  int startCand, int endCand) {
        int startObs = wellMapped.get(startCand)[0];
        int endObs = wellMapped.get(endCand)[0];
        int eFrom = Math.min(wellMapped.get(startCand)[1], wellMapped.get(endCand)[1]);
        int eTo = Math.max(wellMapped.get(startCand)[1], wellMapped.get(endCand)[1]);
        GHPoint pStart = tps.get(startObs).getSnappedPoint();
        GHPoint pEnd = tps.get(endObs).getSnappedPoint();

        GHRequest req = new GHRequest(pStart, pEnd);
        req.setProfile(profile);
        req.setPathDetails(List.of("edge_id"));
        req.putHint("instructions", false);
        req.putHint("calc_points", true);

        StringBuilder header = new StringBuilder();
        header.append(String.format("  probe c[%d]→c[%d] (obs[%d]→obs[%d], expected slice[%d..%d]):",
                startCand, endCand, startObs, endObs, eFrom, eTo));

        GHResponse rsp;
        try { rsp = hopper.route(req); }
        catch (Exception ex) {
            System.out.println(header + " ROUTING THREW: " + ex.getMessage() + "  → FAIL");
            return false;
        }
        if (rsp.hasErrors()) {
            System.out.println(header + " ROUTE ERROR: " + rsp.getErrors() + "  → FAIL");
            return false;
        }
        ResponsePath path = rsp.getBest();
        List<PathDetail> details = path.getPathDetails().get("edge_id");
        if (details == null || details.isEmpty()) {
            System.out.println(header + " NO edge_id DETAILS  → FAIL");
            return false;
        }

        // Dedup consecutive identical edge IDs in route
        List<Integer> routeEdges = new ArrayList<>(details.size());
        int prev = Integer.MIN_VALUE;
        for (PathDetail d : details) {
            int eid = ((Number) d.getValue()).intValue();
            if (eid != prev) { routeEdges.add(eid); prev = eid; }
        }

        // Expected
        List<Integer> expected = new ArrayList<>();
        for (int e = eFrom; e <= eTo; e++) expected.add(em.get(firstEm + e).getEdgeState().getEdge());

        // Verdict using same rule as RoutedRegionOptimizer.probe
        boolean ok;
        String reason;
        if (eFrom == eTo) {
            // same edge — accept if target appears anywhere in route
            int target = expected.get(0);
            ok = routeEdges.contains(target);
            reason = ok ? "same-edge target found" : "same-edge target NOT in route";
        } else {
            // find expected[0] in first 2 of route
            int rIdx = -1;
            for (int k = 0; k < Math.min(2, routeEdges.size()); k++) {
                if (routeEdges.get(k).equals(expected.get(0))) { rIdx = k; break; }
            }
            if (rIdx < 0) {
                ok = false;
                reason = "expected[0]=" + expected.get(0) + " not in route[0..1]";
            } else {
                int ei = 0;
                boolean diverged = false;
                while (ei < expected.size() && rIdx < routeEdges.size()) {
                    if (!routeEdges.get(rIdx).equals(expected.get(ei))) {
                        diverged = true;
                        break;
                    }
                    rIdx++; ei++;
                }
                if (diverged) {
                    ok = false;
                    reason = "mismatch at expected[" + ei + "]=" + expected.get(ei)
                            + " vs route[" + rIdx + "]=" + routeEdges.get(rIdx);
                } else if (ei < expected.size()) {
                    ok = false;
                    reason = "ran out of route at expected[" + ei + "]";
                } else if (routeEdges.size() - rIdx > 2) {
                    ok = false;
                    reason = "too many trailing extras: " + (routeEdges.size() - rIdx);
                } else {
                    ok = true;
                    reason = "exact sequence match (trailing extras: " + (routeEdges.size() - rIdx) + ")";
                }
            }
        }

        // ---- Geometric instrumentation (independent of strict verdict) ----
        // Build the matched polyline by walking slice edges' geometries.
        PointList matchedPoly = new PointList(64, false);
        for (int e = eFrom; e <= eTo; e++) {
            PointList g = em.get(firstEm + e).getEdgeState().fetchWayGeometry(
                    e == eFrom ? FetchMode.ALL : FetchMode.PILLAR_AND_ADJ);
            for (int j = 0; j < g.size(); j++) {
                matchedPoly.add(g.getLat(j), g.getLon(j));
            }
        }
        // Sum of slice edge full lengths (what production code uses as "expectedLen")
        double sliceFullLen = 0;
        for (int e = eFrom; e <= eTo; e++) {
            sliceFullLen += em.get(firstEm + e).getEdgeState().getDistance();
        }
        double routeLen = path.getDistance();
        double ratioDiffPct = sliceFullLen > 0
                ? Math.abs(routeLen - sliceFullLen) / Math.max(routeLen, sliceFullLen) * 100.0
                : 0;

        PointList routePoly = path.getPoints();
        // Max deviation: every /route point → matched polyline (Hausdorff one-way, the
        // metric the production geometric fallback uses).
        double maxRouteToMatched = 0;
        if (matchedPoly.size() >= 2 && routePoly.size() >= 1) {
            for (int j = 0; j < routePoly.size(); j++) {
                double d = diagPointToPolylineDistance(
                        routePoly.getLat(j), routePoly.getLon(j), matchedPoly);
                if (d > maxRouteToMatched) maxRouteToMatched = d;
            }
        }
        // Max deviation: every matched-polyline point → route polyline (the reverse, to
        // catch the case where matched goes through a section /route skips entirely).
        double maxMatchedToRoute = 0;
        if (routePoly.size() >= 2 && matchedPoly.size() >= 1) {
            for (int j = 0; j < matchedPoly.size(); j++) {
                double d = diagPointToPolylineDistance(
                        matchedPoly.getLat(j), matchedPoly.getLon(j), routePoly);
                if (d > maxMatchedToRoute) maxMatchedToRoute = d;
            }
        }

        System.out.println(header);
        System.out.println("    expected (" + expected.size() + "): " + expected);
        System.out.println("    actual   (" + routeEdges.size() + "): " + routeEdges);
        System.out.println("    distance=" + Math.round(path.getDistance()) + "m → " + (ok ? "PASS" : "FAIL")
                + " (" + reason + ")");
        System.out.printf("    [GEO] sliceFullLen=%.1fm routeLen=%.1fm lenRatioDiff=%.1f%% "
                        + "| maxDev route→matched=%.2fm matched→route=%.2fm%n",
                sliceFullLen, routeLen, ratioDiffPct, maxRouteToMatched, maxMatchedToRoute);
        return ok;
    }

    /** Test-side helper: min distance from a single point to a polyline. */
    private static double diagPointToPolylineDistance(double pLat, double pLon, PointList line) {
        double min = Double.POSITIVE_INFINITY;
        if (line.size() == 1) {
            return DistanceCalcEarth.DIST_EARTH.calcDist(pLat, pLon, line.getLat(0), line.getLon(0));
        }
        for (int i = 0; i + 1 < line.size(); i++) {
            double d = diagPointToSegmentDistance(pLat, pLon,
                    line.getLat(i), line.getLon(i),
                    line.getLat(i + 1), line.getLon(i + 1));
            if (d < min) min = d;
        }
        return min;
    }

    /** Test-side helper: perpendicular point-to-segment distance, clamped to endpoints. */
    private static double diagPointToSegmentDistance(double pLat, double pLon,
                                                     double aLat, double aLon,
                                                     double bLat, double bLon) {
        double meanLatRad = Math.toRadians((aLat + bLat) * 0.5);
        double mPerDegLat = 111_320.0;
        double mPerDegLon = 111_320.0 * Math.cos(meanLatRad);
        double ax = aLon * mPerDegLon, ay = aLat * mPerDegLat;
        double bx = bLon * mPerDegLon, by = bLat * mPerDegLat;
        double px = pLon * mPerDegLon, py = pLat * mPerDegLat;
        double dx = bx - ax, dy = by - ay;
        double segLenSq = dx * dx + dy * dy;
        if (segLenSq == 0) {
            return DistanceCalcEarth.DIST_EARTH.calcDist(pLat, pLon, aLat, aLon);
        }
        double t = ((px - ax) * dx + (py - ay) * dy) / segLenSq;
        if (t < 0) t = 0;
        else if (t > 1) t = 1;
        double cx = ax + t * dx;
        double cy = ay + t * dy;
        return Math.sqrt((px - cx) * (px - cx) + (py - cy) * (py - cy));
    }

    /**
     * Build a paste-ready JSON fixture entry from a converted response. Caller still
     * needs to fill in {@code name} (suggested below) and may want to tune the deviation
     * caps if the measured values are unusual.
     */
    private static String emitFixtureBlock(ConvertTrackResponse rsp,
                                           double measuredMaxDev,
                                           double measuredMeanDev,
                                           String gpxPath,
                                           String profile) {
        StringBuilder sb = new StringBuilder();
        String name = new File(gpxPath).getName().replaceFirst("\\.gpx$", "");
        // Default deviation caps: small slack above measured.
        double maxDevCap = Math.ceil(measuredMaxDev + 5);
        double meanDevCap = Math.ceil(measuredMeanDev + 3);

        sb.append("{\n");
        sb.append("  \"name\": \"").append(name).append("\",\n");
        sb.append("  \"gpx\": \"data/gpx-for-testing/").append(new File(gpxPath).getName()).append("\",\n");
        sb.append("  \"profile\": \"").append(profile).append("\",\n");
        sb.append(ConvertValidationHelpers.emitFixtureSegmentsBlock(rsp, 5.0, 15.0));
        sb.append(",\n");
        sb.append(String.format("  \"max_deviation_from_input_gpx_m\":  { \"max\": %.1f },%n", maxDevCap));
        sb.append(String.format("  \"mean_deviation_from_input_gpx_m\": { \"max\": %.1f }%n", meanDevCap));
        sb.append("}");
        return sb.toString();
    }

    // ----------------------------------------------------------------------
    // SIMULATION: alternative optimizer probe using only polyline-deviation.
    //
    // Drives the same exponential-extension + binary-refinement loop as production,
    // but the probe verdict is: "max distance from /route's polyline to the matcher's
    // matched polyline (over the slice between the two snap candidates) ≤ tolM".
    // No edge-ID comparison, no length-ratio gate, no same-edge containment shortcut.
    //
    // Runs alongside production convert and prints both side-by-side.
    //
    // -Dconvert.tolM=10.0  (tolerance in meters; default 10)
    // ----------------------------------------------------------------------

    @Test
    void simulateGeoOnly() throws Exception {
        String gpxPath = System.getProperty("convert.gpx");
        Assumptions.assumeTrue(gpxPath != null,
                "Pass -Dconvert.gpx=/abs/path/to/file.gpx to run this test");
        String profile = System.getProperty("convert.profile", "gravel");
        String matchingProfile = System.getProperty("convert.matchingProfile", profile);
        double gpsAccuracy = Double.parseDouble(System.getProperty("convert.gpsAccuracy", "10"));
        double snapThreshold = Double.parseDouble(System.getProperty("convert.snapThreshold", "35"));
        double tolM = Double.parseDouble(System.getProperty("convert.tolM", "10.0"));

        List<Observation> observations = parseGpx(new File(gpxPath));
        System.out.println("=== INPUT ===");
        System.out.println("gpx=" + gpxPath + " profile=" + profile
                + " matchingProfile=" + matchingProfile
                + " sigma=" + gpsAccuracy + " tolM=" + tolM);

        // --- MapMatching ---
        PMap hints = new PMap();
        hints.putObject("profile", matchingProfile);
        MapMatching matching = MapMatching.fromGraphHopper(hopper, hints);
        matching.setMeasurementErrorSigma(gpsAccuracy);
        MatchResult mr = matching.match(observations);
        System.out.printf("MapMatching: %d edges, %d tracepoints, matchLen=%.0fm%n",
                mr.getEdgeMatches().size(),
                mr.getTracepoints() != null ? mr.getTracepoints().size() : 0,
                mr.getMatchLength());

        // --- Production converter (for comparison) ---
        TrackToRouteConverter prodConverter = new TrackToRouteConverter(hopper);
        ConvertTrackResponse prodRsp = prodConverter.convert(
                mr, observations, profile, null,
                snapThreshold,
                TrackToRouteConverter.DEFAULT_MIN_ROUTED_SEGMENT_M,
                TrackToRouteConverter.DEFAULT_COORDINATES_SIMPLIFY_EPS_M,
                TrackToRouteConverter.DEFAULT_MIN_DETOUR_M,
                TrackToRouteConverter.DEFAULT_MAX_DETOUR_RATIO);

        PointList prodRealized = ConvertValidationHelpers.realizePolyline(prodRsp, hopper, profile, null);
        double[] prodDevs = ConvertValidationHelpers.perPointDeviations(observations, prodRealized);
        double prodMaxDev = ConvertValidationHelpers.maxDeviation(prodDevs);
        double prodMeanDev = ConvertValidationHelpers.meanDeviation(prodDevs);

        // --- Simulated geo-only: same segmenter, alternative optimizer ---
        RegionSegmenter segmenter = new RegionSegmenter();
        CoordinatesRegionBuilder coordsBuilder = new CoordinatesRegionBuilder();
        List<TrackRegion> regions = segmenter.segment(mr, observations, snapThreshold,
                TrackToRouteConverter.DEFAULT_MIN_DETOUR_M,
                TrackToRouteConverter.DEFAULT_MAX_DETOUR_RATIO,
                TrackToRouteConverter.DEFAULT_MIN_ROUTED_SEGMENT_M);

        // Per-segment realization for the simulation
        PointList simRealized = new PointList(256, false);
        int simRoutedSegs = 0;
        int simCoordsSegs = 0;
        int simWaypoints = 0;
        int simForcedSteps = 0;
        int simProbes = 0;
        List<Integer> simChosenObsAll = new ArrayList<>();

        for (TrackRegion region : regions) {
            if (region instanceof TrackRegion.Matched matched) {
                GeoSimResult opt = simulateOptimize(matched, profile, tolM);
                simProbes += opt.probes;
                simForcedSteps += opt.forcedSteps;
                simWaypoints += opt.waypoints.size();
                simRoutedSegs += Math.max(0, opt.waypoints.size() - 1);
                simChosenObsAll.addAll(opt.chosenObsIdx);
                // Realize: re-route between consecutive chosen waypoints
                for (int i = 0; i + 1 < opt.waypoints.size(); i++) {
                    PointList legPts = routePolyline(opt.waypoints.get(i), opt.waypoints.get(i + 1), profile);
                    appendDedupingBoundary(simRealized, legPts);
                }
            } else if (region instanceof TrackRegion.Unmatched unmatched) {
                List<ConvertTrackResponse.Coordinates> coords = coordsBuilder.build(
                        observations, unmatched,
                        TrackToRouteConverter.DEFAULT_COORDINATES_SIMPLIFY_EPS_M);
                simCoordsSegs++;
                simWaypoints += 2;
                PointList legPts = new PointList(coords.size(), false);
                for (ConvertTrackResponse.Coordinates c : coords) {
                    legPts.add(c.getLat(), c.getLng());
                }
                appendDedupingBoundary(simRealized, legPts);
            }
        }

        double[] simDevs = ConvertValidationHelpers.perPointDeviations(observations, simRealized);
        double simMaxDev = ConvertValidationHelpers.maxDeviation(simDevs);
        double simMeanDev = ConvertValidationHelpers.meanDeviation(simDevs);

        // Find worst dev index for each
        int prodWorstIdx = ConvertValidationHelpers.worstDeviationIndex(prodDevs);
        int simWorstIdx = ConvertValidationHelpers.worstDeviationIndex(simDevs);

        // --- Report ---
        System.out.println();
        System.out.println("=== COMPARISON ===");
        System.out.printf("%-14s %10s %10s %10s %10s %10s %14s%n",
                "", "waypoints", "routed", "coords", "maxDev", "meanDev", "worstAt");
        System.out.printf("%-14s %10d %10d %10d %9.2fm %9.2fm %12s%n",
                "production:",
                prodRsp.getWaypoints().size(),
                prodRsp.getStats().routedSegments,
                prodRsp.getStats().coordinatesSegments,
                prodMaxDev, prodMeanDev,
                "gpx[" + prodWorstIdx + "]");
        System.out.printf("%-14s %10d %10d %10d %9.2fm %9.2fm %12s   (forced=%d probes=%d)%n",
                "sim geo-only:",
                simWaypoints, simRoutedSegs, simCoordsSegs,
                simMaxDev, simMeanDev,
                "gpx[" + simWorstIdx + "]",
                simForcedSteps, simProbes);

        // Sim chosen obs indices (matched regions only)
        System.out.println("Sim chosen matched-region obs indices: " + simChosenObsAll);

        // Print top-5 worst per-point deviations for both, side-by-side, with their obs index
        System.out.println("Top 5 worst-deviation points (production vs simulation):");
        int[] prodTopIdx = topKIndices(prodDevs, 5);
        int[] simTopIdx = topKIndices(simDevs, 5);
        for (int k = 0; k < 5; k++) {
            int pi = prodTopIdx[k], si = simTopIdx[k];
            System.out.printf("  #%d  prod gpx[%3d]=%6.2fm     sim gpx[%3d]=%6.2fm%n",
                    k + 1, pi, prodDevs[pi], si, simDevs[si]);
        }

        // Shape sanity
        assertTrue(simWaypoints > 0, "Simulation must produce waypoints");
    }

    private static int[] topKIndices(double[] devs, int k) {
        int[] out = new int[Math.min(k, devs.length)];
        boolean[] used = new boolean[devs.length];
        for (int n = 0; n < out.length; n++) {
            int best = -1;
            double bestVal = -1;
            for (int i = 0; i < devs.length; i++) {
                if (used[i]) continue;
                if (devs[i] > bestVal) { bestVal = devs[i]; best = i; }
            }
            out[n] = best;
            used[best] = true;
        }
        return out;
    }

    /** Result of one matched-region simulation. */
    private static class GeoSimResult {
        final List<GHPoint> waypoints = new ArrayList<>();
        final List<Integer> chosenObsIdx = new ArrayList<>();
        int probes;
        int forcedSteps;
    }

    /**
     * Simulated optimizer: same exp+binary loop, polyline-deviation probe only.
     */
    private GeoSimResult simulateOptimize(TrackRegion.Matched region, String profile, double tolM) {
        GeoSimResult out = new GeoSimResult();
        List<GHPoint> candidates = region.obsSnapPoints();
        int[] candEdgeIdx = region.obsEdgeIdxInSlice();
        List<EdgeMatch> edges = region.edgeMatches();

        if (candidates.size() < 2 || edges.isEmpty()) {
            out.waypoints.add(region.startSnap());
            out.waypoints.add(region.endSnap());
            return out;
        }

        out.waypoints.add(candidates.get(0));
        out.chosenObsIdx.add(region.matchedObsIndices().get(0));
        int cursor = 0;
        final int lastCand = candidates.size() - 1;

        while (cursor < lastCand) {
            int step = 1;
            int lastOk = -1;
            int lastTried = cursor;
            while (true) {
                int probeIdx;
                boolean clampedToEnd;
                if (cursor + step <= lastCand) {
                    probeIdx = cursor + step;
                    clampedToEnd = false;
                } else if (lastTried < lastCand) {
                    probeIdx = lastCand;
                    clampedToEnd = true;
                } else {
                    break;
                }
                lastTried = probeIdx;
                boolean ok = geoProbeOk(
                        candidates.get(cursor), candidates.get(probeIdx),
                        edges, candEdgeIdx[cursor], candEdgeIdx[probeIdx], profile, tolM);
                out.probes++;
                if (ok) {
                    lastOk = probeIdx;
                    if (probeIdx == lastCand) break;
                    step *= 2;
                } else {
                    break;
                }
                if (clampedToEnd) break;
            }
            if (lastOk < 0) {
                // Forced step in the simulation too — adjacent probe failed
                out.forcedSteps++;
                int next = cursor + 1;
                out.waypoints.add(candidates.get(next));
                out.chosenObsIdx.add(region.matchedObsIndices().get(next));
                cursor = next;
                continue;
            }
            int lo = lastOk + 1, hi = lastTried, best = lastOk;
            while (lo <= hi) {
                int mid = (lo + hi) / 2;
                boolean ok = geoProbeOk(
                        candidates.get(cursor), candidates.get(mid),
                        edges, candEdgeIdx[cursor], candEdgeIdx[mid], profile, tolM);
                out.probes++;
                if (ok) { best = mid; lo = mid + 1; }
                else { hi = mid - 1; }
            }
            out.waypoints.add(candidates.get(best));
            out.chosenObsIdx.add(region.matchedObsIndices().get(best));
            cursor = best;
        }
        return out;
    }

    /**
     * Simulated probe: pass iff every /route polyline vertex is within tolM of the
     * matched-edge polyline over the slice [eFromInclusive .. eToInclusive].
     */
    private boolean geoProbeOk(GHPoint pStart, GHPoint pEnd,
                               List<EdgeMatch> sliceEdges, int eStart, int eEnd,
                               String profile, double tolM) {
        int eFrom = Math.min(eStart, eEnd);
        int eTo = Math.max(eStart, eEnd);
        GHRequest req = new GHRequest(pStart, pEnd);
        req.setProfile(profile);
        req.putHint("instructions", false);
        req.putHint("calc_points", true);
        GHResponse rsp;
        try { rsp = hopper.route(req); }
        catch (Exception ex) { return false; }
        if (rsp.hasErrors()) return false;
        ResponsePath path = rsp.getBest();
        PointList routePoly = path.getPoints();
        if (routePoly.size() < 1) return false;

        // Build matched polyline from slice edges
        PointList matchedPoly = new PointList(64, false);
        for (int e = eFrom; e <= eTo; e++) {
            PointList g = sliceEdges.get(e).getEdgeState().fetchWayGeometry(
                    e == eFrom ? FetchMode.ALL : FetchMode.PILLAR_AND_ADJ);
            for (int j = 0; j < g.size(); j++) {
                matchedPoly.add(g.getLat(j), g.getLon(j));
            }
        }
        if (matchedPoly.size() < 2) return false;

        for (int j = 0; j < routePoly.size(); j++) {
            double d = diagPointToPolylineDistance(routePoly.getLat(j), routePoly.getLon(j), matchedPoly);
            if (d > tolM) return false;
        }
        return true;
    }

    private PointList routePolyline(GHPoint a, GHPoint b, String profile) {
        GHRequest req = new GHRequest(a, b);
        req.setProfile(profile);
        req.putHint("instructions", false);
        req.putHint("calc_points", true);
        GHResponse rsp = hopper.route(req);
        if (rsp.hasErrors()) {
            throw new IllegalStateException("routePolyline failed: " + rsp.getErrors());
        }
        return rsp.getBest().getPoints();
    }

    private static void appendDedupingBoundary(PointList target, PointList src) {
        int startIdx = 0;
        if (target.size() > 0 && src.size() > 0) {
            double lastLat = target.getLat(target.size() - 1);
            double lastLon = target.getLon(target.size() - 1);
            double firstLat = src.getLat(0);
            double firstLon = src.getLon(0);
            if (DistanceCalcEarth.DIST_EARTH.calcDist(lastLat, lastLon, firstLat, firstLon) < 1.0) {
                startIdx = 1;
            }
        }
        for (int i = startIdx; i < src.size(); i++) {
            target.add(src.getLat(i), src.getLon(i));
        }
    }

    // ----------------------------------------------------------------------
    // STUDY: compare matcher's getEdgeMatches() against /route's edge_id
    // PathDetails when /route is given ALL matched snap points as via-points
    // (a single multi-via /route call — the "ground truth" approach used by
    // the client-side OSRM optimizer).
    //
    // For each matched region:
    //   - dump matcher's edge ID sequence
    //   - dump /route-through-all-snaps' edge ID sequence (after dedup)
    //   - report identical / differ where / differ how
    // ----------------------------------------------------------------------
    @Test
    void studyMatcherVsRouteAllSnaps() throws Exception {
        String gpxPath = System.getProperty("convert.gpx");
        Assumptions.assumeTrue(gpxPath != null,
                "Pass -Dconvert.gpx=/abs/path/to/file.gpx to run this test");
        String profile = System.getProperty("convert.profile", "gravel");
        String matchingProfile = System.getProperty("convert.matchingProfile", profile);
        double gpsAccuracy = Double.parseDouble(System.getProperty("convert.gpsAccuracy", "10"));
        double snapThreshold = Double.parseDouble(System.getProperty("convert.snapThreshold", "35"));

        List<Observation> observations = parseGpx(new File(gpxPath));
        System.out.println("=== INPUT ===");
        System.out.println("gpx=" + gpxPath + " profile=" + profile
                + " matchingProfile=" + matchingProfile + " sigma=" + gpsAccuracy);

        PMap hints = new PMap();
        hints.putObject("profile", matchingProfile);
        MapMatching matching = MapMatching.fromGraphHopper(hopper, hints);
        matching.setMeasurementErrorSigma(gpsAccuracy);
        MatchResult mr = matching.match(observations);
        System.out.printf("MapMatching: %d edges, %d tracepoints, matchLen=%.0fm%n",
                mr.getEdgeMatches().size(),
                mr.getTracepoints() != null ? mr.getTracepoints().size() : 0,
                mr.getMatchLength());

        // Segmenter — only to identify matched regions and their snap points
        RegionSegmenter segmenter = new RegionSegmenter();
        List<TrackRegion> regions = segmenter.segment(mr, observations, snapThreshold,
                TrackToRouteConverter.DEFAULT_MIN_DETOUR_M,
                TrackToRouteConverter.DEFAULT_MAX_DETOUR_RATIO,
                TrackToRouteConverter.DEFAULT_MIN_ROUTED_SEGMENT_M);

        for (TrackRegion region : regions) {
            if (!(region instanceof TrackRegion.Matched matched)) continue;

            // 1. Matcher's edge ID sequence for this region
            List<EdgeMatch> matcherEdges = matched.edgeMatches();
            List<Integer> matcherIds = new ArrayList<>(matcherEdges.size());
            for (EdgeMatch em : matcherEdges) {
                matcherIds.add(em.getEdgeState().getEdge());
            }

            // 2. Build a multi-via /route through ALL matched snap points
            List<GHPoint> snaps = matched.obsSnapPoints();
            if (snaps.size() < 2) {
                System.out.printf("Region [%d..%d]: only %d snap points — skipping%n",
                        matched.firstObservation(), matched.lastObservation(), snaps.size());
                continue;
            }
            GHRequest req = new GHRequest(new ArrayList<>(snaps));
            req.setProfile(profile);
            req.setPathDetails(List.of("edge_id"));
            req.putHint("instructions", false);
            req.putHint("calc_points", true);

            GHResponse rsp;
            try { rsp = hopper.route(req); }
            catch (Exception e) {
                System.out.printf("Region [%d..%d]: multi-via /route threw: %s%n",
                        matched.firstObservation(), matched.lastObservation(), e.getMessage());
                continue;
            }
            if (rsp.hasErrors()) {
                System.out.printf("Region [%d..%d]: multi-via /route errors: %s%n",
                        matched.firstObservation(), matched.lastObservation(),
                        rsp.getErrors());
                continue;
            }
            ResponsePath path = rsp.getBest();
            List<PathDetail> details = path.getPathDetails().get("edge_id");

            // Raw and dedup'd /route edge sequences
            List<Integer> routeIdsRaw = new ArrayList<>(details.size());
            for (PathDetail d : details) {
                routeIdsRaw.add(((Number) d.getValue()).intValue());
            }
            List<Integer> routeIds = new ArrayList<>(routeIdsRaw.size());
            int prev = Integer.MIN_VALUE;
            for (int id : routeIdsRaw) {
                if (id != prev) { routeIds.add(id); prev = id; }
            }

            // 3. Print comparison
            System.out.println();
            System.out.printf("=== Region Matched [%d..%d]  (%d snap points) ===%n",
                    matched.firstObservation(), matched.lastObservation(), snaps.size());
            System.out.printf("Matcher edges (%d): %s%n", matcherIds.size(), matcherIds);
            System.out.printf("/route raw    (%d): %s%n", routeIdsRaw.size(), routeIdsRaw);
            System.out.printf("/route dedup  (%d): %s%n", routeIds.size(), routeIds);

            // Strict element-wise equality
            if (matcherIds.equals(routeIds)) {
                System.out.println("VERDICT: IDENTICAL after dedup");
            } else {
                // Try aligning: where does the first divergence appear?
                int n = Math.min(matcherIds.size(), routeIds.size());
                int firstDiff = -1;
                for (int i = 0; i < n; i++) {
                    if (!matcherIds.get(i).equals(routeIds.get(i))) {
                        firstDiff = i;
                        break;
                    }
                }
                if (firstDiff < 0) {
                    System.out.printf("VERDICT: PREFIX MATCHES; sizes differ (matcher=%d /route=%d)%n",
                            matcherIds.size(), routeIds.size());
                } else {
                    System.out.printf("VERDICT: diverge at index %d (matcher=%d /route=%d)%n",
                            firstDiff, matcherIds.get(firstDiff), routeIds.get(firstDiff));
                }

                // Set difference (what's in matcher but not /route, and vice versa)
                java.util.Set<Integer> matcherSet = new java.util.LinkedHashSet<>(matcherIds);
                java.util.Set<Integer> routeSet = new java.util.LinkedHashSet<>(routeIds);
                java.util.Set<Integer> onlyMatcher = new java.util.LinkedHashSet<>(matcherSet);
                onlyMatcher.removeAll(routeSet);
                java.util.Set<Integer> onlyRoute = new java.util.LinkedHashSet<>(routeSet);
                onlyRoute.removeAll(matcherSet);
                System.out.println("  edges only in matcher: " + onlyMatcher);
                System.out.println("  edges only in /route:  " + onlyRoute);
            }
        }
    }

    // ----------------------------------------------------------------------
    // STAGE 1 — New segmenter (test-side only; no production code changes)
    //
    // Mirrors the OSRM client's segmentation logic (route-convert.ts) using the
    // matcher's authoritative signals only:
    //   - tracepoint.getDistance()          — per-observation snap distance
    //   - tracepoint.getDistanceFromPrevious() — per-Viterbi-transition matched-leg distance
    //
    // No span-based detour detection (seed-and-extend), no DetourValidator. Detour is
    // a strictly per-consecutive-Viterbi-pair signal: matched > 2× straight AND matched > 75 m.
    //
    // Side-by-side diagnostic against the current production RegionSegmenter.
    //
    // Usage:
    //   -Dtest=TrackConvertDiagnosticTest#segmentationCompare
    //   -Dconvert.gpx=/abs/path.gpx -Dconvert.profile=... -Dconvert.gpsAccuracy=...
    // ----------------------------------------------------------------------

    @Test
    void segmentationCompare() throws Exception {
        String gpxPath = System.getProperty("convert.gpx");
        Assumptions.assumeTrue(gpxPath != null,
                "Pass -Dconvert.gpx=/abs/path/to/file.gpx to run this test");
        String profile = System.getProperty("convert.profile", "gravel");
        String matchingProfile = System.getProperty("convert.matchingProfile", profile);
        double gpsAccuracy = Double.parseDouble(System.getProperty("convert.gpsAccuracy", "10"));
        double snapThreshold = Double.parseDouble(System.getProperty("convert.snapThreshold", "35"));

        List<Observation> observations = parseGpx(new File(gpxPath));
        System.out.println("=== INPUT ===");
        System.out.println("gpx=" + gpxPath
                + " profile=" + profile + " matchingProfile=" + matchingProfile
                + " sigma=" + gpsAccuracy);

        // --- MapMatching ---
        PMap hints = new PMap();
        hints.putObject("profile", matchingProfile);
        MapMatching matching = MapMatching.fromGraphHopper(hopper, hints);
        matching.setMeasurementErrorSigma(gpsAccuracy);
        MatchResult mr = matching.match(observations);
        System.out.printf("MapMatching: %d edges, %d tracepoints, matchLen=%.0fm%n",
                mr.getEdgeMatches().size(),
                mr.getTracepoints() != null ? mr.getTracepoints().size() : 0,
                mr.getMatchLength());

        // --- Current production segmenter ---
        RegionSegmenter prodSegmenter = new RegionSegmenter();
        List<TrackRegion> prodRegions = prodSegmenter.segment(mr, observations, snapThreshold,
                TrackToRouteConverter.DEFAULT_MIN_DETOUR_M,
                TrackToRouteConverter.DEFAULT_MAX_DETOUR_RATIO,
                TrackToRouteConverter.DEFAULT_MIN_ROUTED_SEGMENT_M);

        // --- New segmenter ---
        NewSegmenter newSeg = new NewSegmenter();
        List<NewSegmenter.Region> newRegions = newSeg.segment(mr.getTracepoints());

        // --- Side-by-side report ---
        System.out.println();
        System.out.println("=== CURRENT PRODUCTION SEGMENTER ===");
        for (int i = 0; i < prodRegions.size(); i++) {
            TrackRegion r = prodRegions.get(i);
            String type;
            if (r instanceof TrackRegion.Matched) type = "MATCHED  ";
            else if (r instanceof TrackRegion.Unmatched) type = "COORDS   ";
            else type = "????     ";
            System.out.printf("  [%2d] %s  obs[%3d..%3d]  (%d obs)%n",
                    i, type, r.firstObservation(), r.lastObservation(),
                    r.lastObservation() - r.firstObservation() + 1);
        }

        System.out.println();
        System.out.println("=== NEW SEGMENTER (OSRM-style) ===");
        for (int i = 0; i < newRegions.size(); i++) {
            NewSegmenter.Region r = newRegions.get(i);
            String type = r.type() == NewSegmenter.RegionType.MATCHED ? "MATCHED  " : "COORDS   ";
            System.out.printf("  [%2d] %s  obs[%3d..%3d]  (%d obs)%n",
                    i, type, r.firstObs(), r.lastObs(),
                    r.lastObs() - r.firstObs() + 1);
        }

        // --- Brief per-detour signals from the new segmenter, for context ---
        System.out.println();
        System.out.println("=== NEW SEGMENTER — DETOUR SIGNAL DETAILS ===");
        newSeg.dumpDetourSignals(mr.getTracepoints());

        assertTrue(!newRegions.isEmpty(), "New segmenter must produce at least one region");
    }

    /**
     * New segmenter — OSRM-style. Test-side reference implementation; not yet promoted to
     * production. Operates on the Tracepoint list produced by MapMatching.
     */
    private static class NewSegmenter {

        /** Snap-distance threshold (m). Observations with snap distance >= this become "bad". */
        static final double TRACEPOINT_MAX_DISTANCE = 35.0;

        /** Per-pair detour ratio threshold: matched-leg / straight-line. */
        static final double MAX_DETOUR_FACTOR = 2.0;

        /** Per-pair detour minimum matched-leg distance (m). Pairs shorter than this never
         *  flag as detour even at high ratios — guards against tiny noise spans. */
        static final double MIN_DETOUR_DISTANCE = 75.0;

        /** Minimum matched-path length for a MATCHED region. Shorter ones are demoted to
         *  COORDINATES because the routed-mode rendering advantage disappears below this
         *  threshold — see OSRM client route-convert.ts:27 (MIN_ROUTED_SEGMENT_LENGTH). */
        static final double MIN_ROUTED_SEGMENT_LENGTH = 40.0;

        enum RegionType { MATCHED, COORDINATES }

        record Region(RegionType type, int firstObs, int lastObs) {}

        private enum Status { GOOD, BAD, DETOUR_BOUNDARY }

        List<Region> segment(List<Tracepoint> tps) {
            int n = tps.size();
            if (n == 0) return new ArrayList<>();

            // Stage A — detour flags from per-Viterbi-transition signal
            boolean[] direct = computeDirect(tps, n);

            // Stage B — snap-distance classification
            Status[] status = new Status[n];
            for (int i = 0; i < n; i++) {
                Tracepoint tp = tps.get(i);
                if (tp.isMatched() && tp.getDistance() != null
                        && tp.getDistance() < TRACEPOINT_MAX_DISTANCE) {
                    status[i] = Status.GOOD;
                } else {
                    status[i] = Status.BAD;
                }
            }

            // Stage C — de-singleton (matches OSRM rules)
            //   boundary GOOD with BAD neighbor → BAD
            //   interior GOOD with BAD on both sides → BAD
            Status[] orig = status.clone();
            if (n >= 2) {
                if (orig[0] == Status.GOOD && orig[1] == Status.BAD) status[0] = Status.BAD;
                if (orig[n - 1] == Status.GOOD && orig[n - 2] == Status.BAD) status[n - 1] = Status.BAD;
            }
            for (int i = 1; i < n - 1; i++) {
                if (orig[i] == Status.GOOD && orig[i - 1] == Status.BAD && orig[i + 1] == Status.BAD) {
                    status[i] = Status.BAD;
                }
            }

            // Stage D — promote GOOD to DETOUR_BOUNDARY where the per-pair signal fires
            for (int i = 0; i < n; i++) {
                if (direct[i] && status[i] == Status.GOOD) {
                    status[i] = Status.DETOUR_BOUNDARY;
                }
            }

            // Stage E — build regions
            List<Region> regions = buildRegions(status, n);

            // Stage F — post-process: drop single-obs MATCHED regions (they have no
            // meaningful edge slice and can't be optimized), then merge any consequently
            // adjacent COORDINATES regions. A MATCHED region of length 1 typically arises
            // when a single well-snapped observation sits between an off-network run and a
            // detour-boundary transition — semantically it belongs to the coords on either
            // side.
            regions = postProcess(regions);

            // Stage G — demote MATCHED regions shorter than MIN_ROUTED_SEGMENT_LENGTH to
            // COORDINATES. Routed-mode rendering provides no value at sub-40m stretches;
            // such short matched regions are noise. Then re-merge consecutive coords.
            return demoteShortMatched(regions, tps);
        }

        private static List<Region> postProcess(List<Region> regions) {
            // 1. Drop single-obs MATCHED regions
            List<Region> filtered = new ArrayList<>();
            for (Region r : regions) {
                if (r.type() == RegionType.MATCHED && r.firstObs() == r.lastObs()) continue;
                filtered.add(r);
            }
            // 2. Merge consecutive COORDINATES regions (they share a boundary obs)
            return mergeConsecutiveCoords(filtered);
        }

        private static List<Region> mergeConsecutiveCoords(List<Region> regions) {
            List<Region> merged = new ArrayList<>();
            for (Region r : regions) {
                if (!merged.isEmpty()) {
                    Region prev = merged.get(merged.size() - 1);
                    if (prev.type() == RegionType.COORDINATES && r.type() == RegionType.COORDINATES) {
                        merged.set(merged.size() - 1,
                                new Region(RegionType.COORDINATES, prev.firstObs(), r.lastObs()));
                        continue;
                    }
                }
                merged.add(r);
            }
            return merged;
        }

        /** Stage G: demote MATCHED regions whose matched-path length is below
         *  MIN_ROUTED_SEGMENT_LENGTH to COORDINATES, then merge consecutive coords. */
        private static List<Region> demoteShortMatched(List<Region> regions, List<Tracepoint> tps) {
            List<Region> out = new ArrayList<>();
            for (Region r : regions) {
                if (r.type() == RegionType.MATCHED) {
                    double matchedLen = computeMatchedPathLength(tps, r.firstObs(), r.lastObs());
                    if (matchedLen < MIN_ROUTED_SEGMENT_LENGTH) {
                        out.add(new Region(RegionType.COORDINATES, r.firstObs(), r.lastObs()));
                        continue;
                    }
                }
                out.add(r);
            }
            return mergeConsecutiveCoords(out);
        }

        /** Sum tracepoint.distanceFromPrevious across [firstObs..lastObs] for non-filtered
         *  matched observations. distanceFromPrevious is the matcher's HMM transition
         *  distance from the previous Viterbi participant into this tracepoint; we sum the
         *  transitions for obs strictly after firstObs (the first obs has no incoming
         *  transition within this region). */
        private static double computeMatchedPathLength(List<Tracepoint> tps, int firstObs, int lastObs) {
            double total = 0;
            for (int i = firstObs + 1; i <= lastObs; i++) {
                Tracepoint tp = tps.get(i);
                if (!tp.isMatched() || tp.isFiltered()) continue;
                Double dfp = tp.getDistanceFromPrevious();
                if (dfp != null) total += dfp;
            }
            return total;
        }

        /** Mark direct[i] when the matcher's HMM transition into obs[i] from the previous
         *  non-filtered observation exceeds the OSRM detour gate. */
        private static boolean[] computeDirect(List<Tracepoint> tps, int n) {
            boolean[] direct = new boolean[n];
            Tracepoint prevNonFiltered = null;
            for (int i = 0; i < n; i++) {
                Tracepoint tp = tps.get(i);
                if (!tp.isMatched() || tp.isFiltered()) {
                    // Not a Viterbi participant; skip but don't reset (Viterbi seq jumps over filtered)
                    continue;
                }
                Double dfp = tp.getDistanceFromPrevious();
                if (dfp != null && prevNonFiltered != null) {
                    double straight = DistanceCalcEarth.DIST_EARTH.calcDist(
                            tp.getOriginalPoint().getLat(), tp.getOriginalPoint().getLon(),
                            prevNonFiltered.getOriginalPoint().getLat(), prevNonFiltered.getOriginalPoint().getLon());
                    if (dfp > MAX_DETOUR_FACTOR * straight && dfp > MIN_DETOUR_DISTANCE) {
                        direct[i] = true;
                    }
                }
                prevNonFiltered = tp;
            }
            return direct;
        }

        private static List<Region> buildRegions(Status[] status, int n) {
            List<Region> regions = new ArrayList<>();

            // Find first non-BAD obs
            int i = 0;
            while (i < n && status[i] == Status.BAD) i++;
            if (i > 0) {
                // Leading BAD run becomes coords [0, firstGood]
                regions.add(new Region(RegionType.COORDINATES, 0, Math.min(i, n - 1)));
            }
            if (i >= n) {
                if (regions.isEmpty()) {
                    regions.add(new Region(RegionType.COORDINATES, 0, n - 1));
                }
                return regions;
            }

            int matchedStart = i;
            while (i < n) {
                Status s = status[i];
                if (s == Status.GOOD || s == Status.DETOUR_BOUNDARY) {
                    if (s == Status.DETOUR_BOUNDARY && i > matchedStart) {
                        // Detour boundary in mid-run: close matched at i-1, emit coords [i-1, i],
                        // start a new matched at i.
                        regions.add(new Region(RegionType.MATCHED, matchedStart, i - 1));
                        regions.add(new Region(RegionType.COORDINATES, i - 1, i));
                        matchedStart = i;
                    }
                    i++;
                } else {
                    // BAD — close current matched at i-1
                    regions.add(new Region(RegionType.MATCHED, matchedStart, i - 1));
                    int badStart = i;
                    while (i < n && status[i] == Status.BAD) i++;
                    if (i >= n) {
                        // Trailing BAD — coords from (last good = badStart-1) to n-1
                        regions.add(new Region(RegionType.COORDINATES, badStart - 1, n - 1));
                        return regions;
                    }
                    // BAD run sandwiched: coords [last good, next good]
                    regions.add(new Region(RegionType.COORDINATES, badStart - 1, i));
                    matchedStart = i;
                }
            }
            // Close any final matched run
            if (matchedStart < n) {
                regions.add(new Region(RegionType.MATCHED, matchedStart, n - 1));
            }
            return regions;
        }

        /** For diagnostic clarity: print the per-Viterbi-transition detour-check
         *  numbers (matched leg, straight-line, ratio, verdict) for each non-filtered
         *  observation that has a distFromPrev. */
        void dumpDetourSignals(List<Tracepoint> tps) {
            int n = tps.size();
            System.out.println("idx | dist_from_prev | straight | ratio | detour?");
            System.out.println("----|----------------|----------|-------|---------");
            Tracepoint prevNonFiltered = null;
            for (int i = 0; i < n; i++) {
                Tracepoint tp = tps.get(i);
                if (!tp.isMatched() || tp.isFiltered()) continue;
                Double dfp = tp.getDistanceFromPrevious();
                if (dfp != null && prevNonFiltered != null) {
                    double straight = DistanceCalcEarth.DIST_EARTH.calcDist(
                            tp.getOriginalPoint().getLat(), tp.getOriginalPoint().getLon(),
                            prevNonFiltered.getOriginalPoint().getLat(), prevNonFiltered.getOriginalPoint().getLon());
                    double ratio = straight > 0 ? dfp / straight : Double.POSITIVE_INFINITY;
                    boolean isDetour = dfp > MAX_DETOUR_FACTOR * straight && dfp > MIN_DETOUR_DISTANCE;
                    System.out.printf("%3d | %12.1fm | %7.1fm | %5.2f | %s%n",
                            i, dfp, straight, ratio, isDetour ? "YES" : "no");
                }
                prevNonFiltered = tp;
            }
        }
    }

    // ----------------------------------------------------------------------
    // GPX parser (minimal — extracts <trkpt lat=... lon=.../> in order)
    // ----------------------------------------------------------------------

    static List<Observation> parseGpx(File gpxFile) throws Exception {
        DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
        dbf.setNamespaceAware(false);
        DocumentBuilder db = dbf.newDocumentBuilder();
        Document doc = db.parse(gpxFile);

        NodeList trkpts = doc.getElementsByTagName("trkpt");
        List<Observation> out = new ArrayList<>(trkpts.getLength());
        for (int i = 0; i < trkpts.getLength(); i++) {
            Node n = trkpts.item(i);
            Node latAttr = n.getAttributes().getNamedItem("lat");
            Node lonAttr = n.getAttributes().getNamedItem("lon");
            if (latAttr == null || lonAttr == null) continue;
            double lat = Double.parseDouble(latAttr.getNodeValue());
            double lon = Double.parseDouble(lonAttr.getNodeValue());
            out.add(new Observation(new GHPoint(lat, lon)));
        }
        if (out.isEmpty()) {
            throw new IllegalArgumentException("No <trkpt> entries found in " + gpxFile);
        }
        return out;
    }
}
