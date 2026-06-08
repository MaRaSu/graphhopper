package com.graphhopper.trailmap.convert;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.PropertyNamingStrategies;
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
import com.graphhopper.matching.State;
import com.graphhopper.matching.Tracepoint;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.ev.RoadClass;
import com.graphhopper.routing.ev.RoadEnvironment;
import com.graphhopper.routing.ev.Surface;
import com.graphhopper.routing.querygraph.VirtualEdgeIteratorState;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.storage.Graph;
import com.graphhopper.trailmap.TrailmapGraphHopper;
import com.graphhopper.trailmap.shared.TrailmapImportRegistry;
import com.graphhopper.util.EdgeIteratorState;
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
import java.util.Arrays;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertNotNull;

/**
 * Optimizer-stage signal study. Read-only diagnostic, no production code touched.
 *
 * <p>Goal: empirically settle which topological-identity signals between
 * MapMatching's {@code EdgeMatch[]} slice and {@code /route(snap_i, snap_j)} are
 * reliable as Optimizer probe verdicts. Three signal channels are collected and
 * compared per probe:
 *
 * <ol>
 *   <li>{@code edge_id} sequence — undirected, junction-level (current optimizer signal).</li>
 *   <li>{@code edge_key} sequence — directed (edge_id * 2 + reverseFlag),
 *       resolves back-traversal ambiguity, junction-level.</li>
 *   <li>Interior pillar (lat,lng) sequence — dense (every OSM way vertex within
 *       each edge), the GH equivalent of OSRM's node-id channel. Compared bit-exact.</li>
 * </ol>
 *
 * <p>The test does its own minimal observation→region classification (contiguous
 * runs of well-snapped, non-filtered tracepoints under a snap-distance threshold)
 * so it does not depend on {@code RegionSegmenter} or {@code TrackToRouteConverter},
 * both of which are being refactored.
 *
 * <p>Run:
 * <pre>
 *   mvn -f .../graphhopper/pom.xml -pl map-matching test \
 *       -Dtest=TrackConvertOptimizerStudyTest#studyOptimizerSignals \
 *       -Dconvert.gpx=/abs/path/to/file.gpx \
 *       -Dconvert.profile=gravel \
 *       -Dconvert.gpsAccuracy=5 \
 *       -DargLine=-Xmx16g -q -Dsurefire.useFile=false
 * </pre>
 */
public class TrackConvertOptimizerStudyTest {

    private static final String GRAPH_LOCATION = "../../data/graph-cache";
    private static final String OSM_FILE = "../../data/finland_4.osm.pbf";
    private static final String CONFIG_FILE = "../trailmap-config.yml";

    private static GraphHopper hopper;

    @BeforeAll
    static void setup() throws Exception {
        File graphDir = new File(GRAPH_LOCATION);
        Assumptions.assumeTrue(graphDir.exists() && graphDir.isDirectory(),
                "Graph cache not found at " + graphDir.getAbsolutePath());

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
        if (hopper != null) hopper.close();
    }

    /**
     * Per-region probe study. For each contiguous well-snapped region in the matched
     * track, runs a few snap-pair {@code /route} probes and prints the three signals
     * side-by-side: edge_id sequences, edge_key sequences, and bit-exact interior
     * pillar identity per shared interior edge.
     */
    @Test
    void studyOptimizerSignals() throws Exception {
        String gpxPath = System.getProperty("convert.gpx");
        Assumptions.assumeTrue(gpxPath != null,
                "Pass -Dconvert.gpx=/abs/path/to/file.gpx to run this test");
        String profile = System.getProperty("convert.profile", "gravel");
        String matchingProfile = System.getProperty("convert.matchingProfile", profile);
        double gpsAccuracy = Double.parseDouble(System.getProperty("convert.gpsAccuracy", "5"));
        double snapThreshold = Double.parseDouble(System.getProperty("convert.snapThreshold", "35"));
        int maxProbesPerRegion = Integer.parseInt(System.getProperty("convert.probesPerRegion", "5"));
        int maxInteriorPrint = Integer.parseInt(System.getProperty("convert.interiorPrint", "6"));

        File gpxFile = new File(gpxPath);
        List<Observation> observations = parseGpx(gpxFile);
        System.out.println();
        System.out.println("================================================================");
        System.out.println("FIXTURE: " + gpxFile.getName() + "  profile=" + profile
                + (matchingProfile.equals(profile) ? "" : "  matching=" + matchingProfile)
                + "  sigma=" + gpsAccuracy
                + "  snapThreshold=" + snapThreshold);
        System.out.println("Observations: " + observations.size());
        System.out.println("================================================================");

        PMap hints = new PMap();
        hints.putObject("profile", matchingProfile);
        MapMatching matching = MapMatching.fromGraphHopper(hopper, hints);
        matching.setMeasurementErrorSigma(gpsAccuracy);

        MatchResult matchResult = matching.match(observations);
        List<EdgeMatch> edgeMatches = matchResult.getEdgeMatches();
        List<Tracepoint> tracepoints = matchResult.getTracepoints();
        System.out.printf("Matched: %d edges, %d tracepoints, match length %.0fm%n",
                edgeMatches.size(),
                tracepoints != null ? tracepoints.size() : 0,
                matchResult.getMatchLength());

        // Map each observation → its EdgeMatch index (by walking EdgeMatch.getStates()).
        // First-occurrence wins when an obs ends up on multiple EdgeMatches (rare).
        Map<Observation, Integer> obsToEm = new IdentityHashMap<>();
        for (int emIdx = 0; emIdx < edgeMatches.size(); emIdx++) {
            for (State s : edgeMatches.get(emIdx).getStates()) {
                obsToEm.putIfAbsent(s.getEntry(), emIdx);
            }
        }

        // Classify observations: well-snapped iff non-filtered + matched + snap_dist below threshold.
        // Tracepoints are 1-to-1 with input observations.
        List<WellSnap> wellSnapped = new ArrayList<>();
        if (tracepoints != null) {
            for (int i = 0; i < tracepoints.size(); i++) {
                Tracepoint tp = tracepoints.get(i);
                if (!tp.isMatched() || tp.isFiltered()) continue;
                if (tp.getDistance() == null || tp.getDistance() > snapThreshold) continue;
                Integer emIdx = obsToEm.get(observations.get(i));
                if (emIdx == null) continue;
                wellSnapped.add(new WellSnap(i, tp.getSnappedPoint(), emIdx, tp.getDistance()));
            }
        }

        // Contiguous runs in obs-index order.
        List<List<WellSnap>> regions = groupContiguous(wellSnapped);
        System.out.printf("Well-snapped runs: %d  (regions of contiguous well-snapped obs)%n",
                regions.size());

        for (int r = 0; r < regions.size(); r++) {
            List<WellSnap> region = regions.get(r);
            if (region.size() < 2) {
                System.out.printf("%n--- Region %d: obs[%d] (singleton, skipping) ---%n",
                        r, region.get(0).obsIdx);
                continue;
            }
            studyRegion(r, region, edgeMatches, profile, maxProbesPerRegion, maxInteriorPrint);
        }
        System.out.println();
        System.out.println("================================================================");
        System.out.println("END FIXTURE: " + gpxFile.getName());
        System.out.println("================================================================");
    }

    // ---- region study ---------------------------------------------------------

    private void studyRegion(int rIdx, List<WellSnap> region, List<EdgeMatch> allEdges,
                             String profile, int maxProbes, int maxInteriorPrint) {
        WellSnap first = region.get(0);
        WellSnap last = region.get(region.size() - 1);
        int emLo = first.emIdx, emHi = last.emIdx;
        for (WellSnap w : region) {
            if (w.emIdx < emLo) emLo = w.emIdx;
            if (w.emIdx > emHi) emHi = w.emIdx;
        }
        System.out.println();
        System.out.printf("--- Region %d: obs[%d..%d] (%d obs), em[%d..%d]%n",
                rIdx, first.obsIdx, last.obsIdx, region.size(), emLo, emHi);
        double maxSnap = 0;
        for (WellSnap w : region) if (w.snapDist > maxSnap) maxSnap = w.snapDist;
        System.out.printf("    max snap dist in region: %.2fm%n", maxSnap);

        List<int[]> probePairs = pickProbePairs(region.size(), maxProbes);
        for (int[] pair : probePairs) {
            WellSnap a = region.get(pair[0]);
            WellSnap b = region.get(pair[1]);
            if (a.obsIdx == b.obsIdx) continue;
            int emA = Math.min(a.emIdx, b.emIdx);
            int emB = Math.max(a.emIdx, b.emIdx);
            studyProbe(rIdx, a, b, emA, emB, allEdges, profile, maxInteriorPrint);
        }
    }

    private void studyProbe(int rIdx, WellSnap a, WellSnap b, int emA, int emB,
                            List<EdgeMatch> allEdges, String profile, int maxInteriorPrint) {
        System.out.println();
        System.out.printf("  PROBE region %d  obs[%d]→obs[%d]  em[%d..%d]  start=(%.6f,%.6f)  end=(%.6f,%.6f)%n",
                rIdx, a.obsIdx, b.obsIdx, emA, emB,
                a.snap.lat, a.snap.lon, b.snap.lat, b.snap.lon);

        // Expected (matcher) signals.
        int[] expEdgeIds = new int[emB - emA + 1];
        int[] expEdgeKeys = new int[emB - emA + 1];
        double expDist = 0;
        for (int i = emA; i <= emB; i++) {
            expEdgeIds[i - emA] = allEdges.get(i).getEdgeState().getEdge();
            expEdgeKeys[i - emA] = allEdges.get(i).getEdgeState().getEdgeKey();
            expDist += allEdges.get(i).getEdgeState().getDistance();
        }
        int[] expEdgeIdsDedup = dedupConsecutive(expEdgeIds);
        int[] expEdgeKeysDedup = dedupConsecutive(expEdgeKeys);
        System.out.printf("    expected: %d edges, dedup_id=%d, dedup_key=%d, matched_len=%.1fm%n",
                expEdgeIds.length, expEdgeIdsDedup.length, expEdgeKeysDedup.length, expDist);
        System.out.println("    exp edge_id:  " + previewIntArr(expEdgeIdsDedup, 14));
        System.out.println("    exp edge_key: " + previewKeyArr(expEdgeKeysDedup, 14));

        // Call /route.
        GHRequest req = new GHRequest(a.snap, b.snap);
        req.setProfile(profile);
        req.setPathDetails(List.of("edge_id", "edge_key"));
        req.putHint("instructions", false);
        req.putHint("calc_points", true);

        GHResponse rsp;
        try {
            rsp = hopper.route(req);
        } catch (Exception ex) {
            System.out.println("    /route threw: " + ex.getMessage());
            return;
        }
        if (rsp.hasErrors()) {
            System.out.println("    /route errors: " + rsp.getErrors());
            return;
        }
        ResponsePath path = rsp.getBest();
        List<PathDetail> eidDetails = path.getPathDetails().get("edge_id");
        List<PathDetail> ekeyDetails = path.getPathDetails().get("edge_key");
        if (eidDetails == null || ekeyDetails == null) {
            System.out.println("    /route missing PathDetails");
            return;
        }
        int[] actEdgeIds = pdValuesInt(eidDetails);
        int[] actEdgeKeys = pdValuesInt(ekeyDetails);
        int[] actEdgeIdsDedup = dedupConsecutive(actEdgeIds);
        int[] actEdgeKeysDedup = dedupConsecutive(actEdgeKeys);
        PointList routePolyline = path.getPoints();
        System.out.printf("    actual: %d edge_id details, %d edge_key details, dedup_id=%d, dedup_key=%d, route_len=%.1fm, polyline_pts=%d%n",
                actEdgeIds.length, actEdgeKeys.length, actEdgeIdsDedup.length, actEdgeKeysDedup.length,
                path.getDistance(), routePolyline.size());
        System.out.println("    act edge_id:  " + previewIntArr(actEdgeIdsDedup, 14));
        System.out.println("    act edge_key: " + previewKeyArr(actEdgeKeysDedup, 14));

        // Structural diffs.
        String idDiff = describeStructuralDiff(expEdgeIdsDedup, actEdgeIdsDedup);
        String keyDiff = describeStructuralDiff(expEdgeKeysDedup, actEdgeKeysDedup);
        System.out.println("    edge_id  structural: " + idDiff);
        System.out.println("    edge_key structural: " + keyDiff);

        // Interior pillar identity per shared interior edge.
        // Interior of matcher slice: positions [1 .. n-2] of expEdgeIdsDedup.
        // Interior of /route slice: positions [1 .. m-2] of actEdgeIdsDedup.
        // For each interior matcher edge, find the same edge_id in /route's interior and compare pillar geometries.
        if (expEdgeIdsDedup.length < 3 || actEdgeIdsDedup.length < 3) {
            System.out.println("    interior pillar check: SKIPPED (slice too short — only boundary edges)");
            return;
        }
        // Map edge_id → list of indices into PathDetail list (handles repeated edges in /route).
        java.util.Map<Integer, java.util.Queue<Integer>> actIdToPdIndices = new java.util.HashMap<>();
        for (int i = 0; i < eidDetails.size(); i++) {
            int eid = ((Number) eidDetails.get(i).getValue()).intValue();
            actIdToPdIndices.computeIfAbsent(eid, k -> new java.util.ArrayDeque<>()).add(i);
        }

        int interiorChecked = 0;
        int interiorIdentical = 0;
        int firstDivergenceEdgeId = -1;
        String firstDivergenceDetail = null;
        // Walk matcher's edges from position 1 to n-2 (interior of dedup edge_id list).
        // For pillar identity we use the ORIGINAL non-dedup'd allEdges slice, indexing into the
        // EdgeMatch list. Find the EdgeMatch for each interior dedup'd edge by scanning the
        // original slice for the first matching edge_id between boundary positions.
        int origIdx = 0;
        // skip leading boundary (first dedup edge)
        while (origIdx < expEdgeIds.length && expEdgeIds[origIdx] == expEdgeIdsDedup[0]) origIdx++;
        int interiorPrinted = 0;
        for (int dedupPos = 1; dedupPos <= expEdgeIdsDedup.length - 2; dedupPos++) {
            int curEid = expEdgeIdsDedup[dedupPos];
            // origIdx now points at the first allEdges position whose edge_id == curEid.
            int emPos = emA + origIdx;
            if (emPos < 0 || emPos >= allEdges.size() || allEdges.get(emPos).getEdgeState().getEdge() != curEid) {
                System.out.printf("    interior[%d]=%d : matcher position scan lost track (origIdx=%d) — skipping%n",
                        dedupPos, curEid, origIdx);
                // advance past this dedup group
                while (origIdx < expEdgeIds.length && expEdgeIds[origIdx] == curEid) origIdx++;
                continue;
            }
            EdgeMatch em = allEdges.get(emPos);
            int expEdgeKey = em.getEdgeState().getEdgeKey();
            PointList expGeom = em.getEdgeState().fetchWayGeometry(FetchMode.PILLAR_AND_ADJ);

            // Find this edge in /route's PathDetails.
            java.util.Queue<Integer> q = actIdToPdIndices.get(curEid);
            Integer pdIdx = (q == null || q.isEmpty()) ? null : q.poll();
            if (pdIdx == null) {
                if (interiorPrinted < maxInteriorPrint) {
                    System.out.printf("    interior[%d] edge_id=%d em[%d] : NOT FOUND in /route%n",
                            dedupPos, curEid, emPos);
                    interiorPrinted++;
                }
                interiorChecked++;
                if (firstDivergenceEdgeId < 0) {
                    firstDivergenceEdgeId = curEid;
                    firstDivergenceDetail = "expected interior edge missing from /route";
                }
                while (origIdx < expEdgeIds.length && expEdgeIds[origIdx] == curEid) origIdx++;
                continue;
            }
            PathDetail pd = eidDetails.get(pdIdx);
            int pdFirst = pd.getFirst();
            int pdLast = pd.getLast();
            int actEdgeKey = ((Number) ekeyDetails.get(pdIdx).getValue()).intValue();
            // PathDetail interval [first, last] is INCLUSIVE on both ends and shares its first
            // index with the previous edge's last index (the boundary tower node between edges).
            // So this edge's distinct contribution to the merged polyline is indices (first+1)..last,
            // which is exactly fetchWayGeometry(PILLAR_AND_ADJ).size() points: pillars + adj tower.
            int actSliceLen = pdLast - pdFirst; // = number of (pillars + adj) points contributed by this edge
            boolean sameDirection = (actEdgeKey == expEdgeKey);
            boolean reverseDirection = !sameDirection && (actEdgeKey == reverseEdgeKey(expEdgeKey));

            boolean identical = false;
            int firstDiv = -1;
            double sampleLat1 = 0, sampleLon1 = 0, sampleLat2 = 0, sampleLon2 = 0;
            if (actSliceLen == expGeom.size()) {
                identical = true;
                for (int k = 0; k < expGeom.size(); k++) {
                    int expK = k;
                    int actK = sameDirection ? k : (reverseDirection ? (expGeom.size() - 1 - k) : k);
                    double lat1 = expGeom.getLat(expK);
                    double lon1 = expGeom.getLon(expK);
                    // Route polyline: skip the shared leading boundary (index pdFirst), so the i-th
                    // PILLAR_AND_ADJ point is at pdFirst + 1 + i.
                    double lat2 = routePolyline.getLat(pdFirst + 1 + actK);
                    double lon2 = routePolyline.getLon(pdFirst + 1 + actK);
                    if (k == 0) {
                        sampleLat1 = lat1; sampleLon1 = lon1; sampleLat2 = lat2; sampleLon2 = lon2;
                    }
                    if (Double.doubleToLongBits(lat1) != Double.doubleToLongBits(lat2)
                            || Double.doubleToLongBits(lon1) != Double.doubleToLongBits(lon2)) {
                        identical = false;
                        firstDiv = k;
                        sampleLat1 = lat1; sampleLon1 = lon1; sampleLat2 = lat2; sampleLon2 = lon2;
                        break;
                    }
                }
            }
            interiorChecked++;
            if (identical) interiorIdentical++;
            if (!identical && firstDivergenceEdgeId < 0) {
                firstDivergenceEdgeId = curEid;
                firstDivergenceDetail = "edge_key " + actEdgeKey + " vs " + expEdgeKey
                        + ", sizes " + actSliceLen + " vs " + expGeom.size()
                        + (firstDiv >= 0 ? (", first diff at pillar " + firstDiv) : "");
            }
            if (interiorPrinted < maxInteriorPrint) {
                String verdict = identical ? "IDENTICAL"
                        : firstDiv >= 0 ? ("DIFF@pillar " + firstDiv + " exp=(" + sampleLat1 + "," + sampleLon1 + ") act=(" + sampleLat2 + "," + sampleLon2 + ")")
                        : "SIZE_DIFF";
                System.out.printf("    interior[%d] eid=%d em[%d] pd[%d] expKey=%d actKey=%d %s pillars=%d/%d → %s%n",
                        dedupPos, curEid, emPos, pdIdx,
                        expEdgeKey, actEdgeKey,
                        (sameDirection ? "(same dir)" : reverseDirection ? "(reverse)" : "(KEY MISMATCH)"),
                        expGeom.size(), actSliceLen,
                        verdict);
                interiorPrinted++;
            }
            // Advance origIdx past this dedup group.
            while (origIdx < expEdgeIds.length && expEdgeIds[origIdx] == curEid) origIdx++;
        }
        System.out.printf("    INTERIOR SUMMARY: %d/%d interior edges bit-exact identical pillar sequences",
                interiorIdentical, interiorChecked);
        if (interiorIdentical == interiorChecked && interiorChecked > 0) {
            System.out.println("  ✓");
        } else {
            System.out.println();
            if (firstDivergenceEdgeId >= 0) {
                System.out.printf("    first divergence: edge_id=%d → %s%n",
                        firstDivergenceEdgeId, firstDivergenceDetail);
            }
        }
    }

    // ----------------------------------------------------------------------
    // DIAGNOSTIC: hypothesis check — virtual vs real edge_keys on matcher slice
    // ----------------------------------------------------------------------

    /**
     * Targeted diagnostic for the "short routed section becomes coords" failure.
     *
     * <p>Hypothesis: my optimizer reads {@code slice.get(i).getEdgeState().getEdgeKey()}
     * which returns the VIRTUAL edge_key when the matcher's EdgeMatch.getEdgeState() is a
     * {@link VirtualEdgeIteratorState} (matcher uses QueryGraph internally to handle
     * snaps). Meanwhile {@code /route}'s {@code edge_key} PathDetail returns the
     * UNDERLYING REAL key (see {@code EdgeKeyDetails.java}). So my E_keys and A_keys come
     * from non-equivalent sources whenever a virtual edge is in the matcher's slice.
     *
     * <p>In long matched regions, virtual edges are a small fraction and the boundary
     * tolerance absorbs them. In SHORT regions (1–2 edges), virtual entries dominate the
     * slice; mismatch is unrecoverable and the optimizer escalates.
     *
     * <p>This method:
     * <ol>
     *   <li>Runs the real Segmenter.</li>
     *   <li>For each matched region: walks the edge slice; for each EdgeMatch, prints
     *       whether it is a virtual edge, its raw {@code getEdgeKey()}, and (if virtual)
     *       its {@code getOriginalEdgeKey()}.</li>
     *   <li>For each consecutive-candidate probe (cursor → cursor+1, the smallest probe
     *       whose failure triggers escalation), runs {@code /route} and compares:
     *       (a) E built from raw matcher keys vs A; (b) E built from RESOLVED keys
     *       (virtual → original) vs A. Reports which one matches.</li>
     * </ol>
     *
     * <p>If hypothesis is correct, every short-region smallest-probe will have:
     * E_raw != A, E_resolved == A. If false, the data will show a different pattern.
     */
    @Test
    void diagnoseVirtualEdgeKeys() throws Exception {
        String gpxPath = System.getProperty("convert.gpx");
        Assumptions.assumeTrue(gpxPath != null,
                "Pass -Dconvert.gpx=/abs/path/to/file.gpx to run this test");
        String profile = System.getProperty("convert.profile", "gravel");
        String matchingProfile = System.getProperty("convert.matchingProfile", profile);
        double gpsAccuracy = Double.parseDouble(System.getProperty("convert.gpsAccuracy", "5"));
        double snapThreshold = Double.parseDouble(System.getProperty("convert.snapThreshold", "35"));
        double minDetourM = Double.parseDouble(System.getProperty("convert.minDetourM", "75.0"));
        double maxDetourRatio = Double.parseDouble(System.getProperty("convert.maxDetourRatio", "2.0"));
        double minRoutedSegmentM = Double.parseDouble(System.getProperty("convert.minRoutedSegmentM", "40.0"));

        File gpxFile = new File(gpxPath);
        List<Observation> observations = parseGpx(gpxFile);
        System.out.println();
        System.out.println("================================================================");
        System.out.println("DIAGNOSE VIRTUAL-EDGE KEYS: " + gpxFile.getName()
                + "  profile=" + profile
                + (matchingProfile.equals(profile) ? "" : "  matching=" + matchingProfile)
                + "  sigma=" + gpsAccuracy);
        System.out.println("================================================================");

        PMap hints = new PMap();
        hints.putObject("profile", matchingProfile);
        MapMatching matching = MapMatching.fromGraphHopper(hopper, hints);
        matching.setMeasurementErrorSigma(gpsAccuracy);
        MatchResult mr = matching.match(observations);

        RegionSegmenter segmenter = new RegionSegmenter();
        List<TrackRegion> regions = segmenter.segment(mr, observations,
                snapThreshold, minDetourM, maxDetourRatio, minRoutedSegmentM);

        int totalRegions = 0;
        int totalShortMismatches = 0;
        int rawMatches = 0;
        int resolvedMatches = 0;
        int totalProbes = 0;

        for (int rIdx = 0; rIdx < regions.size(); rIdx++) {
            TrackRegion tr = regions.get(rIdx);
            if (!(tr instanceof TrackRegion.Matched m)) continue;
            totalRegions++;
            List<EdgeMatch> slice = m.edgeMatches();
            List<GHPoint> snaps = m.obsSnapPoints();
            List<Integer> obsIdx = m.matchedObsIndices();
            int[] candEdge = m.obsEdgeIdxInSlice();

            System.out.println();
            System.out.printf("--- Region %d: obs[%d..%d], %d candidates, slice has %d edges ---%n",
                    rIdx, m.firstObservation(), m.lastObservation(), snaps.size(), slice.size());

            // Dump every slice edge with virtual-vs-real classification.
            int virtualEdgeCount = 0;
            for (int i = 0; i < slice.size(); i++) {
                EdgeIteratorState es = slice.get(i).getEdgeState();
                boolean isVirtual = es instanceof VirtualEdgeIteratorState;
                int rawKey = es.getEdgeKey();
                int rawEdge = es.getEdge();
                String origStr;
                if (isVirtual) {
                    int origKey = ((VirtualEdgeIteratorState) es).getOriginalEdgeKey();
                    origStr = String.format("origKey=%d (edge_id=%d %s)",
                            origKey, origKey / 2, (origKey & 1) == 0 ? "+" : "-");
                    virtualEdgeCount++;
                } else {
                    origStr = "(real, no origKey)";
                }
                System.out.printf("  slice[%d] %s rawEdge=%d rawKey=%d  %s%n",
                        i, isVirtual ? "VIRT" : "REAL", rawEdge, rawKey, origStr);
            }
            System.out.printf("  → %d virtual / %d real in slice%n",
                    virtualEdgeCount, slice.size() - virtualEdgeCount);

            // For every consecutive-candidate probe (cursor → cursor+1), test the
            // hypothesis. These are the probes that, when they fail, trigger escalation.
            for (int c = 0; c < snaps.size() - 1; c++) {
                int eA = Math.min(candEdge[c], candEdge[c + 1]);
                int eB = Math.max(candEdge[c], candEdge[c + 1]);

                // Build E_raw and E_resolved
                int[] rawKeys = new int[eB - eA + 1];
                int[] resolvedKeys = new int[eB - eA + 1];
                for (int i = eA; i <= eB; i++) {
                    EdgeIteratorState es = slice.get(i).getEdgeState();
                    rawKeys[i - eA] = es.getEdgeKey();
                    resolvedKeys[i - eA] = (es instanceof VirtualEdgeIteratorState ves)
                            ? ves.getOriginalEdgeKey() : es.getEdgeKey();
                }
                int[] eRaw = dedupConsecutive(rawKeys);
                int[] eResolved = dedupConsecutive(resolvedKeys);

                // Call /route between the two snap points
                GHRequest req = new GHRequest(snaps.get(c), snaps.get(c + 1));
                req.setProfile(profile);
                req.setPathDetails(List.of("edge_key"));
                req.putHint("instructions", false);
                req.putHint("calc_points", true);
                GHResponse rsp = hopper.route(req);
                if (rsp.hasErrors()) {
                    System.out.printf("  PROBE cand[%d]→[%d] obs[%d]→[%d]: /route ERRORS%n",
                            c, c + 1, obsIdx.get(c), obsIdx.get(c + 1));
                    continue;
                }
                List<PathDetail> ek = rsp.getBest().getPathDetails().get("edge_key");
                int[] a = dedupConsecutive(pdValuesInt(ek));
                boolean ruleRaw = com.graphhopper.trailmap.convert.RoutedRegionOptimizer
                        .applyEdgeKeyRule(eRaw, a);
                boolean ruleResolved = com.graphhopper.trailmap.convert.RoutedRegionOptimizer
                        .applyEdgeKeyRule(eResolved, a);
                totalProbes++;
                if (ruleRaw) rawMatches++;
                if (ruleResolved) resolvedMatches++;
                if (!ruleRaw && ruleResolved) totalShortMismatches++;

                // Print only the interesting cases (failures or virtual involvement)
                boolean anyVirt = false;
                for (int i = eA; i <= eB; i++) {
                    if (slice.get(i).getEdgeState() instanceof VirtualEdgeIteratorState) {
                        anyVirt = true; break;
                    }
                }
                if (anyVirt || !ruleRaw) {
                    System.out.printf("  PROBE cand[%d]→[%d] obs[%d]→[%d]  em[%d..%d]  virt=%s%n",
                            c, c + 1, obsIdx.get(c), obsIdx.get(c + 1), eA, eB, anyVirt);
                    System.out.printf("    E_raw      = %s%n", previewKeyArr(eRaw, 12));
                    System.out.printf("    E_resolved = %s%n", previewKeyArr(eResolved, 12));
                    System.out.printf("    A          = %s%n", previewKeyArr(a, 12));
                    System.out.printf("    rule(raw)=%s   rule(resolved)=%s%n",
                            ruleRaw ? "PASS" : "FAIL",
                            ruleResolved ? "PASS" : "FAIL");
                }
            }
        }

        System.out.println();
        System.out.println("================================================================");
        System.out.printf("VIRTUAL-EDGE HYPOTHESIS SUMMARY for %s:%n", gpxFile.getName());
        System.out.printf("  matched regions: %d%n", totalRegions);
        System.out.printf("  consecutive-pair probes tested: %d%n", totalProbes);
        System.out.printf("  rule(raw)      PASS count: %d%n", rawMatches);
        System.out.printf("  rule(resolved) PASS count: %d%n", resolvedMatches);
        System.out.printf("  cases where raw FAILS but resolved PASSES: %d   ← hypothesis-fit count%n",
                totalShortMismatches);
        System.out.println("================================================================");
    }

    // ----------------------------------------------------------------------
    // DIAGNOSTIC: isolate divergent edges between matcher and /route on the
    // failing probes (MATCHER_EDGES mode), then query the routing graph for
    // each divergent edge's properties — to design node-pair tolerance from
    // real data, not guesses.
    // ----------------------------------------------------------------------

    @Test
    void diagnoseDivergentEdge() throws Exception {
        String gpxPath = System.getProperty("convert.gpx");
        Assumptions.assumeTrue(gpxPath != null, "Pass -Dconvert.gpx=...");
        String profile = System.getProperty("convert.profile", "gravel");
        String matchingProfile = System.getProperty("convert.matchingProfile", profile);
        double gpsAccuracy = Double.parseDouble(System.getProperty("convert.gpsAccuracy", "10"));
        double snapThreshold = Double.parseDouble(System.getProperty("convert.snapThreshold", "25"));
        double minDetourM = Double.parseDouble(System.getProperty("convert.minDetourM", "75.0"));
        double maxDetourRatio = Double.parseDouble(System.getProperty("convert.maxDetourRatio", "2.0"));
        double minRoutedSegmentM = Double.parseDouble(System.getProperty("convert.minRoutedSegmentM", "40.0"));
        int maxCases = Integer.parseInt(System.getProperty("convert.maxCases", "5"));

        File gpxFile = new File(gpxPath);
        List<Observation> observations = parseGpx(gpxFile);
        System.out.println();
        System.out.println("================================================================");
        System.out.println("DIAGNOSE DIVERGENT EDGE: " + gpxFile.getName()
                + "  profile=" + profile
                + (matchingProfile.equals(profile) ? "" : "  matching=" + matchingProfile)
                + "  sigma=" + gpsAccuracy);
        System.out.println("================================================================");

        PMap hints = new PMap();
        hints.putObject("profile", matchingProfile);
        MapMatching matching = MapMatching.fromGraphHopper(hopper, hints);
        matching.setMeasurementErrorSigma(gpsAccuracy);
        MatchResult mr = matching.match(observations);

        RegionSegmenter segmenter = new RegionSegmenter();
        List<TrackRegion> regions = segmenter.segment(mr, observations,
                snapThreshold, minDetourM, maxDetourRatio, minRoutedSegmentM);

        EncodingManager em = hopper.getEncodingManager();
        EnumEncodedValue<RoadClass> roadClassEnc = em.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<RoadEnvironment> roadEnvEnc = em.getEnumEncodedValue(RoadEnvironment.KEY, RoadEnvironment.class);
        EnumEncodedValue<Surface> surfaceEnc = null;
        try {
            surfaceEnc = em.getEnumEncodedValue(Surface.KEY, Surface.class);
        } catch (Exception ignored) {}
        Graph graph = hopper.getBaseGraph();

        int casesFound = 0;
        for (int rIdx = 0; rIdx < regions.size() && casesFound < maxCases; rIdx++) {
            TrackRegion tr = regions.get(rIdx);
            if (!(tr instanceof TrackRegion.Matched m)) continue;
            List<GHPoint> snaps = m.obsSnapPoints();
            List<Integer> obsIdx = m.matchedObsIndices();
            int[] candEdgeIdx = m.obsEdgeIdxInSlice();
            List<EdgeMatch> slice = m.edgeMatches();

            for (int c = 0; c < snaps.size() - 1 && casesFound < maxCases; c++) {
                // Matcher's E for this smallest-step probe.
                int eAraw = Math.min(candEdgeIdx[c], candEdgeIdx[c + 1]);
                int eBraw = Math.max(candEdgeIdx[c], candEdgeIdx[c + 1]);
                int[] expKeysRaw = new int[eBraw - eAraw + 1];
                for (int i = eAraw; i <= eBraw; i++) {
                    expKeysRaw[i - eAraw] = slice.get(i).getEdgeState().getEdgeKey();
                }
                int[] E = dedupConsecutive(expKeysRaw);

                // Actual A from /route.
                GHRequest req = new GHRequest(snaps.get(c), snaps.get(c + 1));
                req.setProfile(profile);
                req.setPathDetails(List.of("edge_key"));
                req.putHint("instructions", false);
                req.putHint("calc_points", true);
                GHResponse rsp = hopper.route(req);
                if (rsp.hasErrors()) continue;
                List<PathDetail> ek = rsp.getBest().getPathDetails().get("edge_key");
                if (ek == null) continue;
                int[] A = dedupConsecutive(pdValuesInt(ek));

                // Skip when the existing boundary-tolerant rule already passes.
                if (com.graphhopper.trailmap.convert.RoutedRegionOptimizer.applyEdgeKeyRule(E, A)) continue;

                casesFound++;
                System.out.println();
                System.out.printf("--- Divergence case [%d]: Region %d, cand[%d]→[%d] obs[%d]→obs[%d] ---%n",
                        casesFound, rIdx, c, c + 1, obsIdx.get(c), obsIdx.get(c + 1));
                System.out.println("  snap_start=(" + snaps.get(c).lat + "," + snaps.get(c).lon + ")");
                System.out.println("  snap_end  =(" + snaps.get(c + 1).lat + "," + snaps.get(c + 1).lon + ")");
                System.out.println("  E (matcher dedup_key): " + previewKeyArr(E, 20));
                System.out.println("  A (/route dedup_key):  " + previewKeyArr(A, 20));

                // Common prefix and suffix.
                int prefixLen = 0;
                while (prefixLen < E.length && prefixLen < A.length && E[prefixLen] == A[prefixLen]) {
                    prefixLen++;
                }
                int suffixLen = 0;
                while (suffixLen < E.length - prefixLen
                        && suffixLen < A.length - prefixLen
                        && E[E.length - 1 - suffixLen] == A[A.length - 1 - suffixLen]) {
                    suffixLen++;
                }
                int[] eMid = Arrays.copyOfRange(E, prefixLen, E.length - suffixLen);
                int[] aMid = Arrays.copyOfRange(A, prefixLen, A.length - suffixLen);

                System.out.printf("  common prefix=%d, common suffix=%d%n", prefixLen, suffixLen);
                System.out.println("  divergent middle (matcher): " + previewKeyArr(eMid, 20));
                System.out.println("  divergent middle (/route):  " + previewKeyArr(aMid, 20));

                if (prefixLen > 0) {
                    int boundaryKey = E[prefixLen - 1];
                    System.out.println("  last common prefix edge (context):");
                    printEdgeDetails("    PRE", boundaryKey, graph, roadClassEnc, roadEnvEnc, surfaceEnc);
                }
                System.out.println("  MATCHER divergent edges:");
                for (int k : eMid) printEdgeDetails("    matcher", k, graph, roadClassEnc, roadEnvEnc, surfaceEnc);
                System.out.println("  /ROUTE divergent edges:");
                for (int k : aMid) printEdgeDetails("    /route ", k, graph, roadClassEnc, roadEnvEnc, surfaceEnc);
                if (suffixLen > 0) {
                    int boundaryKey = E[E.length - suffixLen];
                    System.out.println("  first common suffix edge (context):");
                    printEdgeDetails("    POST", boundaryKey, graph, roadClassEnc, roadEnvEnc, surfaceEnc);
                }

                // Node-pair tolerance verification: do divergent middles share entry and exit nodes?
                if (eMid.length > 0 && aMid.length > 0) {
                    EdgeIteratorState eFirst = graph.getEdgeIteratorStateForKey(eMid[0]);
                    EdgeIteratorState eLast = graph.getEdgeIteratorStateForKey(eMid[eMid.length - 1]);
                    EdgeIteratorState aFirst = graph.getEdgeIteratorStateForKey(aMid[0]);
                    EdgeIteratorState aLast = graph.getEdgeIteratorStateForKey(aMid[aMid.length - 1]);
                    int eEntry = eFirst.getBaseNode();
                    int eExit = eLast.getAdjNode();
                    int aEntry = aFirst.getBaseNode();
                    int aExit = aLast.getAdjNode();
                    boolean sameEntry = eEntry == aEntry;
                    boolean sameExit = eExit == aExit;
                    System.out.println("  NODE-PAIR ANALYSIS:");
                    System.out.printf("    matcher: entry_node=%d  exit_node=%d%n", eEntry, eExit);
                    System.out.printf("    /route:  entry_node=%d  exit_node=%d%n", aEntry, aExit);
                    System.out.printf("    same entry: %s, same exit: %s%n", sameEntry, sameExit);
                    if (sameEntry && sameExit) {
                        System.out.println("    → MICRO-ALTERNATIVE AT JUNCTION (node-pair tolerance would accept)");
                    } else {
                        System.out.println("    → DIFFERENT entry/exit nodes (real path divergence)");
                    }
                }
            }
        }

        System.out.println();
        System.out.printf("Divergence cases found: %d%n", casesFound);
    }

    private static void printEdgeDetails(String label, int edgeKey, Graph graph,
                                         EnumEncodedValue<RoadClass> roadClassEnc,
                                         EnumEncodedValue<RoadEnvironment> roadEnvEnc,
                                         EnumEncodedValue<Surface> surfaceEnc) {
        try {
            EdgeIteratorState es = graph.getEdgeIteratorStateForKey(edgeKey);
            int edgeId = edgeKey / 2;
            String dir = (edgeKey & 1) == 0 ? "fwd" : "rev";
            double dist = es.getDistance();
            int baseNode = es.getBaseNode();
            int adjNode = es.getAdjNode();
            RoadClass rc = roadClassEnc != null ? es.get(roadClassEnc) : null;
            RoadEnvironment re = roadEnvEnc != null ? es.get(roadEnvEnc) : null;
            Surface sf = surfaceEnc != null ? es.get(surfaceEnc) : null;
            PointList geom = es.fetchWayGeometry(FetchMode.ALL);
            System.out.printf("%s edge_id=%d %s base=%d adj=%d len=%.1fm road_class=%s road_env=%s surface=%s geom_pts=%d%n",
                    label, edgeId, dir, baseNode, adjNode, dist, rc, re, sf, geom.size());
            if (geom.size() > 0) {
                System.out.printf("%s   geom_first=(%.6f,%.6f) geom_last=(%.6f,%.6f)%n",
                        label,
                        geom.getLat(0), geom.getLon(0),
                        geom.getLat(geom.size() - 1), geom.getLon(geom.size() - 1));
            }
        } catch (Exception e) {
            System.out.printf("%s edge_key=%d (lookup failed: %s)%n", label, edgeKey, e.getMessage());
        }
    }

    // ----------------------------------------------------------------------
    // DIAGNOSTIC: U-turn apex detection from the matcher slice (now that the
    // segmenter dedups by edge_key, the back-traversal signature is preserved)
    // ----------------------------------------------------------------------

    /**
     * Scan each matched region's edge slice for U-turn apex patterns: adjacent slice
     * entries with the same {@code edge_id} but flipped direction (their edge_keys
     * differ by exactly 1). For each detected apex, identify the "apex observation"
     * as the LAST candidate whose {@code obsEdgeIdxInSlice} maps to the forward
     * half of the pair — that's the observation the optimizer should force as a
     * waypoint to preserve the user's actual turn position.
     *
     * <p>Verifies on out-n-back (one apex expected at obs[7]) and confirms no apex
     * is detected on fixtures without U-turns.
     */
    @Test
    void diagnoseUTurnApex() throws Exception {
        String gpxPath = System.getProperty("convert.gpx");
        Assumptions.assumeTrue(gpxPath != null, "Pass -Dconvert.gpx=...");
        String profile = System.getProperty("convert.profile", "gravel");
        String matchingProfile = System.getProperty("convert.matchingProfile", profile);
        double gpsAccuracy = Double.parseDouble(System.getProperty("convert.gpsAccuracy", "10"));
        double snapThreshold = Double.parseDouble(System.getProperty("convert.snapThreshold", "25"));
        double minDetourM = Double.parseDouble(System.getProperty("convert.minDetourM", "75.0"));
        double maxDetourRatio = Double.parseDouble(System.getProperty("convert.maxDetourRatio", "2.0"));
        double minRoutedSegmentM = Double.parseDouble(System.getProperty("convert.minRoutedSegmentM", "40.0"));

        File gpxFile = new File(gpxPath);
        List<Observation> observations = parseGpx(gpxFile);
        System.out.println();
        System.out.println("================================================================");
        System.out.println("U-TURN APEX DETECTION: " + gpxFile.getName()
                + "  profile=" + profile
                + (matchingProfile.equals(profile) ? "" : "  matching=" + matchingProfile)
                + "  sigma=" + gpsAccuracy);
        System.out.println("================================================================");

        PMap hints = new PMap();
        hints.putObject("profile", matchingProfile);
        MapMatching matching = MapMatching.fromGraphHopper(hopper, hints);
        matching.setMeasurementErrorSigma(gpsAccuracy);
        MatchResult mr = matching.match(observations);

        RegionSegmenter segmenter = new RegionSegmenter();
        List<TrackRegion> regions = segmenter.segment(mr, observations,
                snapThreshold, minDetourM, maxDetourRatio, minRoutedSegmentM);

        int totalApexes = 0;
        for (int rIdx = 0; rIdx < regions.size(); rIdx++) {
            TrackRegion tr = regions.get(rIdx);
            if (!(tr instanceof TrackRegion.Matched m)) continue;
            List<EdgeMatch> slice = m.edgeMatches();
            int[] candEdgeIdx = m.obsEdgeIdxInSlice();
            List<Integer> obsIdx = m.matchedObsIndices();

            System.out.printf("%n--- Region %d: obs[%d..%d], %d candidates, slice %d edges ---%n",
                    rIdx, m.firstObservation(), m.lastObservation(),
                    candEdgeIdx.length, slice.size());

            int regionApexes = 0;
            for (int i = 0; i + 1 < slice.size(); i++) {
                int kA = slice.get(i).getEdgeState().getEdgeKey();
                int kB = slice.get(i + 1).getEdgeState().getEdgeKey();
                if ((kA ^ 1) != kB) continue;
                // U-turn apex pair found at slice positions (i, i+1).
                // Apex obs = last candidate whose obsEdgeIdxInSlice == i.
                int apexCand = -1;
                for (int k = 0; k < candEdgeIdx.length; k++) {
                    if (candEdgeIdx[k] == i) apexCand = k;
                }
                int alternateCand = -1; // first candidate mapped to slice[i+1]
                for (int k = 0; k < candEdgeIdx.length; k++) {
                    if (candEdgeIdx[k] == i + 1) { alternateCand = k; break; }
                }
                System.out.printf("  APEX at slice[%d..%d]: edge_id=%d, %s → %s%n",
                        i, i + 1, kA / 2,
                        (kA & 1) == 0 ? "fwd" : "rev",
                        (kB & 1) == 0 ? "fwd" : "rev");
                if (apexCand >= 0) {
                    System.out.printf("     ← forced waypoint candidate: cand[%d] = obs[%d] (last in forward half)%n",
                            apexCand, obsIdx.get(apexCand));
                } else {
                    System.out.println("     ← no candidate maps to forward half (apex internal to filtered obs)");
                }
                if (alternateCand >= 0 && alternateCand != apexCand) {
                    System.out.printf("     alternate: cand[%d] = obs[%d] (first in reverse half)%n",
                            alternateCand, obsIdx.get(alternateCand));
                }
                regionApexes++;
                totalApexes++;
            }
            if (regionApexes == 0) {
                System.out.println("  (no U-turn apex in slice)");
            }
        }
        System.out.println();
        System.out.println("================================================================");
        System.out.printf("SUMMARY for %s: %d U-turn apex pair(s) detected across all matched regions%n",
                gpxFile.getName(), totalApexes);
        System.out.println("================================================================");
    }

    // ----------------------------------------------------------------------
    // DIAGNOSTIC: OSRM-style /route(all_snaps) as ground truth
    // ----------------------------------------------------------------------

    /**
     * Hypothesis check for the "matcher edges vs /route edges disagree" failure pattern
     * observed when matching_profile != profile.
     *
     * <p>Architecture question: should the Optimizer's reference be the matcher's edge
     * slice (current rule) or {@code /route(all_snaps_in_region)} (OSRM-style)?
     *
     * <p>For each Matched region, this diagnostic:
     * <ol>
     *   <li>Runs {@code /route(all snaps as via-points)} with the RENDERING profile.</li>
     *   <li>Walks per-leg edges using {@code waypointIndices} + {@code edge_key} details.</li>
     *   <li>For each consecutive-pair probe that the current rule REJECTS against the
     *       matcher reference, checks whether the corresponding ground-truth leg's edges
     *       match the probe's /route edges.</li>
     * </ol>
     *
     * <p>If the ground-truth comparison passes the cases that the matcher-reference
     * rejected, switching the reference fixes them without weakening the rule on real
     * disagreements.
     */
    @Test
    void diagnoseRouteGroundTruth() throws Exception {
        String gpxPath = System.getProperty("convert.gpx");
        Assumptions.assumeTrue(gpxPath != null, "Pass -Dconvert.gpx=...");
        String profile = System.getProperty("convert.profile", "gravel");
        String matchingProfile = System.getProperty("convert.matchingProfile", profile);
        double gpsAccuracy = Double.parseDouble(System.getProperty("convert.gpsAccuracy", "10"));
        double snapThreshold = Double.parseDouble(System.getProperty("convert.snapThreshold", "25"));
        double minDetourM = Double.parseDouble(System.getProperty("convert.minDetourM", "75.0"));
        double maxDetourRatio = Double.parseDouble(System.getProperty("convert.maxDetourRatio", "2.0"));
        double minRoutedSegmentM = Double.parseDouble(System.getProperty("convert.minRoutedSegmentM", "40.0"));

        File gpxFile = new File(gpxPath);
        List<Observation> observations = parseGpx(gpxFile);
        System.out.println();
        System.out.println("================================================================");
        System.out.println("ROUTE-GROUND-TRUTH DIAGNOSTIC: " + gpxFile.getName()
                + "  profile=" + profile
                + (matchingProfile.equals(profile) ? "" : "  matching=" + matchingProfile)
                + "  sigma=" + gpsAccuracy + "  snapThr=" + snapThreshold);
        System.out.println("================================================================");

        PMap hints = new PMap();
        hints.putObject("profile", matchingProfile);
        MapMatching matching = MapMatching.fromGraphHopper(hopper, hints);
        matching.setMeasurementErrorSigma(gpsAccuracy);
        MatchResult mr = matching.match(observations);
        RegionSegmenter segmenter = new RegionSegmenter();
        List<TrackRegion> regions = segmenter.segment(mr, observations,
                snapThreshold, minDetourM, maxDetourRatio, minRoutedSegmentM);

        int totalMatcherRejected = 0;
        int matcherRejectedButGroundTruthPasses = 0;
        int matcherRejectedAndGroundTruthAlsoFails = 0;

        for (int rIdx = 0; rIdx < regions.size(); rIdx++) {
            TrackRegion tr = regions.get(rIdx);
            if (!(tr instanceof TrackRegion.Matched m)) continue;
            List<GHPoint> snaps = m.obsSnapPoints();
            List<Integer> obsIdx = m.matchedObsIndices();
            int[] candEdge = m.obsEdgeIdxInSlice();
            List<EdgeMatch> slice = m.edgeMatches();
            if (snaps.size() < 2) continue;

            System.out.println();
            System.out.printf("--- Region %d: obs[%d..%d], %d candidates ---%n",
                    rIdx, m.firstObservation(), m.lastObservation(), snaps.size());

            // ---- Step 1: build the /route(all_snaps) ground truth ----
            GHRequest gtReq = new GHRequest(snaps);
            gtReq.setProfile(profile);
            gtReq.setPathDetails(List.of("edge_key"));
            gtReq.putHint("instructions", false);
            gtReq.putHint("calc_points", true);
            GHResponse gtRsp;
            try {
                gtRsp = hopper.route(gtReq);
            } catch (Exception ex) {
                System.out.printf("  /route(all_snaps) THREW: %s%n", ex.getMessage());
                continue;
            }
            if (gtRsp.hasErrors()) {
                System.out.printf("  /route(all_snaps) errors: %s%n", gtRsp.getErrors());
                continue;
            }
            ResponsePath gtPath = gtRsp.getBest();
            List<Integer> wpIndices = gtPath.getWaypointIndices();
            List<PathDetail> gtEk = gtPath.getPathDetails().get("edge_key");
            System.out.printf("  /route(all_snaps): %d points, %d edges_key details, %d waypointIndices%n",
                    gtPath.getPoints().size(), gtEk.size(), wpIndices.size());

            // Build per-via-leg ground-truth edge_key sequences (from via[i] to via[i+1]).
            // Use waypointIndices to find which PathDetail interval indices fall in each leg.
            int legCount = wpIndices.size() - 1;
            int[][] gtLegKeys = new int[legCount][];
            int pdIdx = 0;
            for (int leg = 0; leg < legCount; leg++) {
                int legStart = wpIndices.get(leg);
                int legEnd = wpIndices.get(leg + 1);
                List<Integer> keys = new ArrayList<>();
                while (pdIdx < gtEk.size() && gtEk.get(pdIdx).getLast() <= legEnd) {
                    keys.add(((Number) gtEk.get(pdIdx).getValue()).intValue());
                    if (gtEk.get(pdIdx).getLast() == legEnd) {
                        pdIdx++;
                        break;
                    }
                    pdIdx++;
                }
                // Edges fully inside this leg, plus the one spanning the via boundary if any.
                int[] arr = new int[keys.size()];
                for (int k = 0; k < keys.size(); k++) arr[k] = keys.get(k);
                gtLegKeys[leg] = dedupConsecutive(arr);
            }

            // ---- Step 2: for each consecutive-pair probe, compare matcher rule vs ground truth ----
            for (int c = 0; c < snaps.size() - 1; c++) {
                // Matcher-reference E (current rule)
                int eA = Math.min(candEdge[c], candEdge[c + 1]);
                int eB = Math.max(candEdge[c], candEdge[c + 1]);
                int[] expRaw = new int[eB - eA + 1];
                for (int i = eA; i <= eB; i++) expRaw[i - eA] = slice.get(i).getEdgeState().getEdgeKey();
                int[] eMatcher = dedupConsecutive(expRaw);

                // /route(snap_c, snap_c+1) directly
                GHRequest req = new GHRequest(snaps.get(c), snaps.get(c + 1));
                req.setProfile(profile);
                req.setPathDetails(List.of("edge_key"));
                req.putHint("instructions", false);
                req.putHint("calc_points", true);
                GHResponse rsp = hopper.route(req);
                if (rsp.hasErrors()) continue;
                List<PathDetail> ek = rsp.getBest().getPathDetails().get("edge_key");
                int[] a = dedupConsecutive(pdValuesInt(ek));

                // Ground-truth leg c: edges /route(all_snaps) traversed for via_c → via_{c+1}
                int[] gtLeg = (c < gtLegKeys.length) ? gtLegKeys[c] : new int[0];

                boolean ruleMatcher = com.graphhopper.trailmap.convert.RoutedRegionOptimizer
                        .applyEdgeKeyRule(eMatcher, a);
                boolean ruleGroundTruth = com.graphhopper.trailmap.convert.RoutedRegionOptimizer
                        .applyEdgeKeyRule(gtLeg, a);

                if (!ruleMatcher) {
                    totalMatcherRejected++;
                    if (ruleGroundTruth) matcherRejectedButGroundTruthPasses++;
                    else matcherRejectedAndGroundTruthAlsoFails++;
                    System.out.printf("  PROBE cand[%d]→[%d] obs[%d]→[%d]:%n", c, c + 1, obsIdx.get(c), obsIdx.get(c + 1));
                    System.out.printf("    E (matcher)     = %s%n", previewKeyArr(eMatcher, 12));
                    System.out.printf("    GT leg edges    = %s%n", previewKeyArr(gtLeg, 12));
                    System.out.printf("    A (probe /route)= %s%n", previewKeyArr(a, 12));
                    System.out.printf("    rule(matcher) = %s    rule(ground_truth) = %s%n",
                            ruleMatcher ? "PASS" : "FAIL", ruleGroundTruth ? "PASS" : "FAIL");
                }
            }
        }

        System.out.println();
        System.out.println("================================================================");
        System.out.printf("ROUTE-GROUND-TRUTH SUMMARY for %s:%n", gpxFile.getName());
        System.out.printf("  total smallest-probe failures vs matcher reference: %d%n", totalMatcherRejected);
        System.out.printf("  ... of which ground-truth reference would PASS:    %d   ← fix-fits%n",
                matcherRejectedButGroundTruthPasses);
        System.out.printf("  ... of which ground-truth reference also FAILS:    %d%n",
                matcherRejectedAndGroundTruthAlsoFails);
        System.out.println("================================================================");
    }

    // ----------------------------------------------------------------------
    // PROTOTYPE: full Optimizer algorithm simulation
    // ----------------------------------------------------------------------

    /**
     * Prototype the proposed Optimizer algorithm on the test side. No production
     * code touched. Per region, runs exponential extension + binary refinement with
     * the edge_key-based comparison rule:
     *
     *   PASS iff A == E[x .. len(E)-y]  for some x ∈ {0,1}, y ∈ {0,1}
     *
     * where E is the matcher's dedup-by-edge_key slice and A is /route's
     * dedup-by-edge_key response. Escalates to a coords-segment marker when the
     * smallest probe (cursor → cursor+1) fails.
     *
     * <p>Prints per-probe verdict (rule + pillar identity cross-check) and per-region
     * final waypoint list + leg distances + escalations. Used to validate the
     * algorithm before any production change.
     */
    @Test
    void simulateOptimizer() throws Exception {
        String gpxPath = System.getProperty("convert.gpx");
        Assumptions.assumeTrue(gpxPath != null,
                "Pass -Dconvert.gpx=/abs/path/to/file.gpx to run this test");
        String profile = System.getProperty("convert.profile", "gravel");
        String matchingProfile = System.getProperty("convert.matchingProfile", profile);
        double gpsAccuracy = Double.parseDouble(System.getProperty("convert.gpsAccuracy", "5"));
        double snapThreshold = Double.parseDouble(System.getProperty("convert.snapThreshold", "35"));
        boolean verboseProbe = Boolean.parseBoolean(System.getProperty("convert.verboseProbe", "true"));

        File gpxFile = new File(gpxPath);
        List<Observation> observations = parseGpx(gpxFile);
        System.out.println();
        System.out.println("================================================================");
        System.out.println("FIXTURE (simulate): " + gpxFile.getName() + "  profile=" + profile
                + (matchingProfile.equals(profile) ? "" : "  matching=" + matchingProfile)
                + "  sigma=" + gpsAccuracy);
        System.out.println("Observations: " + observations.size());
        System.out.println("================================================================");

        PMap hints = new PMap();
        hints.putObject("profile", matchingProfile);
        MapMatching matching = MapMatching.fromGraphHopper(hopper, hints);
        matching.setMeasurementErrorSigma(gpsAccuracy);

        MatchResult matchResult = matching.match(observations);
        List<EdgeMatch> edgeMatches = matchResult.getEdgeMatches();
        List<Tracepoint> tracepoints = matchResult.getTracepoints();
        System.out.printf("Matched: %d edges, %d tracepoints, match length %.0fm%n",
                edgeMatches.size(),
                tracepoints != null ? tracepoints.size() : 0,
                matchResult.getMatchLength());

        Map<Observation, Integer> obsToEm = new IdentityHashMap<>();
        for (int emIdx = 0; emIdx < edgeMatches.size(); emIdx++) {
            for (State s : edgeMatches.get(emIdx).getStates()) {
                obsToEm.putIfAbsent(s.getEntry(), emIdx);
            }
        }

        List<WellSnap> wellSnapped = new ArrayList<>();
        if (tracepoints != null) {
            for (int i = 0; i < tracepoints.size(); i++) {
                Tracepoint tp = tracepoints.get(i);
                if (!tp.isMatched() || tp.isFiltered()) continue;
                if (tp.getDistance() == null || tp.getDistance() > snapThreshold) continue;
                Integer emIdx = obsToEm.get(observations.get(i));
                if (emIdx == null) continue;
                wellSnapped.add(new WellSnap(i, tp.getSnappedPoint(), emIdx, tp.getDistance()));
            }
        }
        // Use the REAL production RegionSegmenter (other agent's rewrite).
        // The optimizer prototype works on Matched regions exactly as the production
        // converter would deliver them.
        double minDetourM = Double.parseDouble(System.getProperty("convert.minDetourM", "75.0"));
        double maxDetourRatio = Double.parseDouble(System.getProperty("convert.maxDetourRatio", "2.0"));
        double minRoutedSegmentM = Double.parseDouble(System.getProperty("convert.minRoutedSegmentM", "40.0"));
        RegionSegmenter segmenter = new RegionSegmenter();
        List<TrackRegion> segRegions = segmenter.segment(matchResult, observations,
                snapThreshold, minDetourM, maxDetourRatio, minRoutedSegmentM);
        long matchedCount = segRegions.stream().filter(r -> r instanceof TrackRegion.Matched).count();
        long unmatchedCount = segRegions.stream().filter(r -> r instanceof TrackRegion.Unmatched).count();
        System.out.printf("Segmenter (REAL): %d regions (%d matched, %d unmatched)%n",
                segRegions.size(), matchedCount, unmatchedCount);

        // Report unmatched (coords) regions for context (the optimizer doesn't process them).
        for (TrackRegion tr : segRegions) {
            if (tr instanceof TrackRegion.Unmatched u) {
                System.out.printf("  Unmatched (coords) region: obs[%d..%d]%n",
                        u.firstObservation(), u.lastObservation());
            }
        }

        // Build prototype input from each Matched region: WellSnap entries indexed into
        // the region's DEDUP'D edge slice (m.edgeMatches()), not into the global match.
        List<RegionInput> regions = new ArrayList<>();
        for (TrackRegion tr : segRegions) {
            if (!(tr instanceof TrackRegion.Matched m)) continue;
            List<WellSnap> wsRegion = new ArrayList<>();
            List<Integer> obsIdx = m.matchedObsIndices();
            List<GHPoint> snaps = m.obsSnapPoints();
            int[] edgeIdxInSlice = m.obsEdgeIdxInSlice();
            for (int k = 0; k < obsIdx.size(); k++) {
                int oi = obsIdx.get(k);
                Double snapDist = tracepoints != null && oi < tracepoints.size()
                        ? tracepoints.get(oi).getDistance() : null;
                wsRegion.add(new WellSnap(oi, snaps.get(k),
                        edgeIdxInSlice[k],
                        snapDist != null ? snapDist : 0.0));
            }
            regions.add(new RegionInput(wsRegion, m.edgeMatches()));
        }

        int totalProbes = 0;
        int totalEscalations = 0;
        int totalRuleVsPillarDisagreements = 0;
        int totalWaypoints = 0;
        int totalCoordsSegments = 0;
        int totalRoutedSegments = 0;

        for (int r = 0; r < regions.size(); r++) {
            RegionInput region = regions.get(r);
            if (region.candidates.size() < 2) {
                System.out.printf("%n--- Region %d: obs[%d] (singleton, skipping) ---%n",
                        r, region.candidates.get(0).obsIdx);
                continue;
            }
            OptResult res = simulateRegion(r, region.candidates, region.edgeSlice, profile, verboseProbe);
            totalProbes += res.probes;
            totalEscalations += res.coordsEscalations;
            totalRuleVsPillarDisagreements += res.ruleVsPillarDisagreements;
            totalWaypoints += res.chosen.size();
            totalCoordsSegments += res.coordsEscalations;
            totalRoutedSegments += (res.chosen.size() - 1) - res.coordsEscalations;
        }

        System.out.println();
        System.out.println("================================================================");
        System.out.printf("SUMMARY: %d total waypoints across %d regions, %d routed legs, %d coords escalations, %d probes total%n",
                totalWaypoints, regions.size(), totalRoutedSegments, totalCoordsSegments, totalProbes);
        if (totalRuleVsPillarDisagreements == 0) {
            System.out.println("RULE-vs-PILLAR cross-check: 0 disagreements ✓ (rule never said PASS while pillars disagreed, or vice versa)");
        } else {
            System.out.printf("RULE-vs-PILLAR cross-check: %d DISAGREEMENTS — investigate%n", totalRuleVsPillarDisagreements);
        }
        System.out.println("END FIXTURE (simulate): " + gpxFile.getName());
        System.out.println("================================================================");
    }

    /** One Matched region's input for the optimizer: candidates + dedup'd edge slice. */
    private record RegionInput(List<WellSnap> candidates, List<EdgeMatch> edgeSlice) {}

    /** Outcome of one region's optimization. */
    private static final class OptResult {
        final List<WellSnap> chosen = new ArrayList<>();
        final List<Double> legsM = new ArrayList<>();
        final List<Boolean> legIsCoords = new ArrayList<>(); // true → coords escalation between chosen[i] and chosen[i+1]
        int probes = 0;
        int coordsEscalations = 0;
        int ruleVsPillarDisagreements = 0;
    }

    private static final int MAX_PROBES_PER_REGION = 400;

    private OptResult simulateRegion(int rIdx, List<WellSnap> region,
                                     List<EdgeMatch> allEdges, String profile,
                                     boolean verbose) {
        OptResult res = new OptResult();
        int n = region.size();
        WellSnap first = region.get(0);
        WellSnap last = region.get(n - 1);

        System.out.println();
        System.out.printf("--- Region %d: obs[%d..%d] (%d cand), em-range[%d..%d] ---%n",
                rIdx, first.obsIdx, last.obsIdx, n,
                Math.min(first.emIdx, last.emIdx), Math.max(first.emIdx, last.emIdx));

        res.chosen.add(first);
        int cursor = 0;
        final int lastCand = n - 1;

        while (cursor < lastCand) {
            int step = 1;
            int lastOk = -1;
            double lastOkDist = 0;
            int lastTried = cursor;

            // Exponential extension
            while (true) {
                int probeIdx = Math.min(cursor + step, lastCand);
                if (probeIdx == lastTried && probeIdx != cursor + step) break; // clamped repeatedly
                lastTried = probeIdx;
                ProbeReport pr = runProbe(region, cursor, probeIdx, allEdges, profile, verbose);
                res.probes++;
                if (pr.ruleVsPillarDisagreed) res.ruleVsPillarDisagreements++;
                if (pr.pass) {
                    lastOk = probeIdx;
                    lastOkDist = pr.routeDistanceM;
                    if (probeIdx == lastCand) break;
                    if (cursor + step >= lastCand) break; // we already probed lastCand
                    step *= 2;
                } else {
                    break;
                }
                if (res.probes >= MAX_PROBES_PER_REGION) break;
            }

            if (lastOk < 0) {
                // Smallest probe (cursor → cursor+1) failed: ESCALATE to coords segment.
                int next = cursor + 1;
                WellSnap nextSnap = region.get(next);
                System.out.printf("  → ESCALATE: smallest probe cand[%d]→cand[%d] (obs[%d]→obs[%d]) failed; emitting COORDS segment%n",
                        cursor, next, region.get(cursor).obsIdx, nextSnap.obsIdx);
                res.chosen.add(nextSnap);
                res.legsM.add(0.0); // placeholder — production would use GPX-based coord length
                res.legIsCoords.add(true);
                res.coordsEscalations++;
                cursor = next;
                if (res.probes >= MAX_PROBES_PER_REGION) {
                    System.out.printf("  ! probe cap reached at region %d%n", rIdx);
                    break;
                }
                continue;
            }

            // Binary refine between lastOk+1 and lastTried (if exp-extension overshot).
            int best = lastOk;
            double bestDist = lastOkDist;
            if (lastTried > lastOk) {
                int lo = lastOk + 1;
                int hi = lastTried;
                while (lo <= hi) {
                    int mid = (lo + hi) / 2;
                    ProbeReport pr = runProbe(region, cursor, mid, allEdges, profile, verbose);
                    res.probes++;
                    if (pr.ruleVsPillarDisagreed) res.ruleVsPillarDisagreements++;
                    if (pr.pass) {
                        best = mid;
                        bestDist = pr.routeDistanceM;
                        lo = mid + 1;
                    } else {
                        hi = mid - 1;
                    }
                    if (res.probes >= MAX_PROBES_PER_REGION) break;
                }
            }

            WellSnap bestSnap = region.get(best);
            System.out.printf("  ✓ chose cand[%d] = obs[%d] (leg %.1fm via /route)%n",
                    best, bestSnap.obsIdx, bestDist);
            res.chosen.add(bestSnap);
            res.legsM.add(bestDist);
            res.legIsCoords.add(false);
            cursor = best;

            if (res.probes >= MAX_PROBES_PER_REGION) {
                System.out.printf("  ! probe cap reached at region %d%n", rIdx);
                break;
            }
        }

        // Summary for region
        System.out.printf("  REGION RESULT: %d waypoints, %d legs (%d routed, %d coords)%n",
                res.chosen.size(), res.legsM.size(),
                (int) res.legIsCoords.stream().filter(b -> !b).count(),
                (int) res.legIsCoords.stream().filter(b -> b).count());
        StringBuilder wptStr = new StringBuilder("    waypoints (obs idx): [");
        for (int i = 0; i < res.chosen.size(); i++) {
            if (i > 0) wptStr.append(',');
            wptStr.append(res.chosen.get(i).obsIdx);
        }
        wptStr.append("]");
        System.out.println(wptStr);
        return res;
    }

    /** One probe's full output. */
    private static final class ProbeReport {
        final boolean pass;
        final String ruleReason;
        final double routeDistanceM;
        final boolean ruleVsPillarDisagreed;

        ProbeReport(boolean pass, String ruleReason, double routeDistanceM, boolean ruleVsPillarDisagreed) {
            this.pass = pass;
            this.ruleReason = ruleReason;
            this.routeDistanceM = routeDistanceM;
            this.ruleVsPillarDisagreed = ruleVsPillarDisagreed;
        }
    }

    private ProbeReport runProbe(List<WellSnap> region, int startIdx, int endIdx,
                                 List<EdgeMatch> allEdges, String profile,
                                 boolean verbose) {
        WellSnap a = region.get(startIdx);
        WellSnap b = region.get(endIdx);
        int emA = Math.min(a.emIdx, b.emIdx);
        int emB = Math.max(a.emIdx, b.emIdx);

        // Build expected edge_key sequence E (dedup'd).
        int[] expKeysFull = new int[emB - emA + 1];
        for (int i = emA; i <= emB; i++) {
            expKeysFull[i - emA] = allEdges.get(i).getEdgeState().getEdgeKey();
        }
        int[] E = dedupConsecutive(expKeysFull);

        // Call /route.
        GHRequest req = new GHRequest(a.snap, b.snap);
        req.setProfile(profile);
        req.setPathDetails(List.of("edge_id", "edge_key"));
        req.putHint("instructions", false);
        req.putHint("calc_points", true);

        GHResponse rsp;
        try {
            rsp = hopper.route(req);
        } catch (Exception ex) {
            String msg = "ROUTE THREW: " + ex.getMessage();
            if (verbose) System.out.printf("  probe cand[%d]→cand[%d] : %s%n", startIdx, endIdx, msg);
            return new ProbeReport(false, msg, 0, false);
        }
        if (rsp.hasErrors()) {
            String msg = "ROUTE ERRORS: " + rsp.getErrors();
            if (verbose) System.out.printf("  probe cand[%d]→cand[%d] : %s%n", startIdx, endIdx, msg);
            return new ProbeReport(false, msg, 0, false);
        }
        ResponsePath path = rsp.getBest();
        List<PathDetail> ekDetails = path.getPathDetails().get("edge_key");
        List<PathDetail> eidDetails = path.getPathDetails().get("edge_id");
        if (ekDetails == null || eidDetails == null) {
            String msg = "NO PATH DETAILS";
            if (verbose) System.out.printf("  probe cand[%d]→cand[%d] : %s%n", startIdx, endIdx, msg);
            return new ProbeReport(false, msg, 0, false);
        }
        int[] A = dedupConsecutive(pdValuesInt(ekDetails));

        // Apply the comparison rule.
        RuleVerdict v = applyEdgeKeyRule(E, A);

        // Cross-check: pillar identity for shared interior edges (mostly diagnostic).
        PillarCheck pc = pillarCrossCheck(allEdges, emA, emB, eidDetails, ekDetails, path.getPoints());
        // Only flag a real falsification: rule said EXACT PASS but some shared interior
        // pillar pair was not bit-identical. Boundary-tolerance PASS cases have snap-truncated
        // edges and naturally show fewer-than-all pillar matches; that's expected.
        boolean ruleVsPillarDisagreed = false;
        if (v.pass && v.reason.startsWith("PASS exact")
                && pc.interiorChecked > 0 && pc.interiorIdentical < pc.interiorChecked) {
            ruleVsPillarDisagreed = true;
        }

        if (verbose) {
            System.out.printf("  probe cand[%d]→cand[%d] (obs[%d]→obs[%d])  em[%d..%d]  E_keys=%s  A_keys=%s  rule=%s  dist=%.1fm  pillars=%d/%d%s%n",
                    startIdx, endIdx, a.obsIdx, b.obsIdx, emA, emB,
                    previewKeyArr(E, 10), previewKeyArr(A, 10),
                    v.reason, path.getDistance(),
                    pc.interiorIdentical, pc.interiorChecked,
                    ruleVsPillarDisagreed ? "  !DISAGREEMENT" : "");
        }

        return new ProbeReport(v.pass, v.reason, path.getDistance(), ruleVsPillarDisagreed);
    }

    private static final class RuleVerdict {
        final boolean pass;
        final String reason;
        RuleVerdict(boolean pass, String reason) {
            this.pass = pass;
            this.reason = reason;
        }
    }

    /** Apply A == E[x..len(E)-y] for x,y ∈ {0,1}. Classify FAIL into Pattern B or C. */
    private static RuleVerdict applyEdgeKeyRule(int[] E, int[] A) {
        if (Arrays.equals(E, A)) return new RuleVerdict(true, "PASS exact");
        for (int x = 0; x <= 1; x++) {
            for (int y = 0; y <= 1; y++) {
                if (x == 0 && y == 0) continue;
                if (E.length - x - y < 0) continue;
                if (E.length - x - y == 0) {
                    if (A.length == 0) return new RuleVerdict(true, "PASS empty (x=" + x + ",y=" + y + ")");
                    continue;
                }
                int[] sub = Arrays.copyOfRange(E, x, E.length - y);
                if (Arrays.equals(sub, A)) {
                    return new RuleVerdict(true, "PASS boundary x=" + x + ",y=" + y);
                }
            }
        }
        // Failure classification.
        boolean hasBackTraversal = hasDirectionFlipSamePair(E);
        if (A.length + 2 < E.length && hasBackTraversal) {
            return new RuleVerdict(false, "FAIL Pattern B (matcher loop / back-traversal, route shortcut)");
        }
        if (A.length > E.length) {
            return new RuleVerdict(false, "FAIL Pattern C (route has extra edges)");
        }
        if (A.length + 2 < E.length) {
            return new RuleVerdict(false, "FAIL Pattern B-like (route much shorter, no clear back-traversal in E)");
        }
        return new RuleVerdict(false, "FAIL Pattern C (sequence differs in middle)");
    }

    /** True if E contains a consecutive pair (k, k^1) — same edge id, opposite directions. */
    private static boolean hasDirectionFlipSamePair(int[] keys) {
        for (int i = 0; i + 1 < keys.length; i++) {
            if ((keys[i] ^ 1) == keys[i + 1]) return true;
        }
        return false;
    }

    private static final class PillarCheck {
        final int interiorChecked;
        final int interiorIdentical;
        PillarCheck(int checked, int identical) {
            this.interiorChecked = checked;
            this.interiorIdentical = identical;
        }
    }

    /**
     * For shared interior edges (present in both matcher slice and /route), verify
     * pillar (lat,lng) bit-identity between EdgeMatch.fetchWayGeometry(PILLAR_AND_ADJ)
     * and the corresponding /route polyline sub-range.
     */
    private static PillarCheck pillarCrossCheck(List<EdgeMatch> allEdges, int emA, int emB,
                                                List<PathDetail> eidDetails,
                                                List<PathDetail> ekDetails,
                                                PointList routePoly) {
        // Map edge_id → ordered list of PathDetail indices in /route response
        Map<Integer, java.util.Deque<Integer>> eidToPd = new java.util.HashMap<>();
        for (int i = 0; i < eidDetails.size(); i++) {
            int eid = ((Number) eidDetails.get(i).getValue()).intValue();
            eidToPd.computeIfAbsent(eid, k -> new java.util.ArrayDeque<>()).add(i);
        }
        int checked = 0;
        int identical = 0;
        for (int e = emA; e <= emB; e++) {
            // Skip boundary positions (only true interior is meaningful for this check).
            if (e == emA || e == emB) continue;
            int eid = allEdges.get(e).getEdgeState().getEdge();
            int expKey = allEdges.get(e).getEdgeState().getEdgeKey();
            java.util.Deque<Integer> q = eidToPd.get(eid);
            if (q == null || q.isEmpty()) continue; // edge not in /route — handled by rule
            int pdIdx = q.poll();
            int actKey = ((Number) ekDetails.get(pdIdx).getValue()).intValue();
            boolean sameDir = (actKey == expKey);
            boolean reverseDir = !sameDir && (actKey == (expKey ^ 1));
            PathDetail pd = eidDetails.get(pdIdx);
            int sliceLen = pd.getLast() - pd.getFirst();
            PointList g = allEdges.get(e).getEdgeState().fetchWayGeometry(FetchMode.PILLAR_AND_ADJ);
            checked++;
            if (sliceLen != g.size()) continue;
            boolean ok = true;
            for (int k = 0; k < g.size(); k++) {
                int actK = sameDir ? k : (reverseDir ? (g.size() - 1 - k) : k);
                double lat1 = g.getLat(k), lon1 = g.getLon(k);
                double lat2 = routePoly.getLat(pd.getFirst() + 1 + actK);
                double lon2 = routePoly.getLon(pd.getFirst() + 1 + actK);
                if (Double.doubleToLongBits(lat1) != Double.doubleToLongBits(lat2)
                        || Double.doubleToLongBits(lon1) != Double.doubleToLongBits(lon2)) {
                    ok = false;
                    break;
                }
            }
            if (ok) identical++;
        }
        return new PillarCheck(checked, identical);
    }

    // ----------------------------------------------------------------------

    private static int reverseEdgeKey(int key) {
        return key ^ 1; // toggle the LSB (reverse flag)
    }

    private static List<int[]> pickProbePairs(int n, int maxProbes) {
        List<int[]> probes = new ArrayList<>();
        probes.add(new int[]{0, n - 1});
        if (n >= 4) {
            int q = n / 4;
            probes.add(new int[]{0, n / 2});
            probes.add(new int[]{n / 2, n - 1});
            if (maxProbes >= 4) probes.add(new int[]{0, q});
            if (maxProbes >= 5) probes.add(new int[]{n - 1 - q, n - 1});
        } else if (n >= 3) {
            probes.add(new int[]{0, 1});
            probes.add(new int[]{1, n - 1});
        }
        while (probes.size() > maxProbes) probes.remove(probes.size() - 1);
        return probes;
    }

    private static int[] dedupConsecutive(int[] xs) {
        if (xs.length == 0) return xs;
        int[] out = new int[xs.length];
        int n = 0;
        int prev = Integer.MIN_VALUE;
        for (int x : xs) {
            if (x != prev) {
                out[n++] = x;
                prev = x;
            }
        }
        int[] trimmed = new int[n];
        System.arraycopy(out, 0, trimmed, 0, n);
        return trimmed;
    }

    private static int[] pdValuesInt(List<PathDetail> details) {
        int[] out = new int[details.size()];
        for (int i = 0; i < details.size(); i++) {
            out[i] = ((Number) details.get(i).getValue()).intValue();
        }
        return out;
    }

    private static String previewIntArr(int[] xs, int maxShow) {
        StringBuilder sb = new StringBuilder("[");
        int show = Math.min(maxShow, xs.length);
        for (int i = 0; i < show; i++) {
            if (i > 0) sb.append(',');
            sb.append(xs[i]);
        }
        if (xs.length > maxShow) sb.append(",… (total ").append(xs.length).append(")");
        sb.append(']');
        return sb.toString();
    }

    private static String previewKeyArr(int[] xs, int maxShow) {
        StringBuilder sb = new StringBuilder("[");
        int show = Math.min(maxShow, xs.length);
        for (int i = 0; i < show; i++) {
            if (i > 0) sb.append(',');
            int k = xs[i];
            sb.append(k / 2).append((k & 1) == 0 ? "+" : "-");
        }
        if (xs.length > maxShow) sb.append(",… (total ").append(xs.length).append(")");
        sb.append(']');
        return sb.toString();
    }

    private static String describeStructuralDiff(int[] exp, int[] act) {
        // First find shared sub-sequence: does `exp` appear contiguously inside `act`?
        // Report (a) leading extras in act, (b) trailing extras in act, (c) whether exp
        // is contiguously contained, (d) length deltas.
        if (java.util.Arrays.equals(exp, act)) return "EXACT MATCH";
        // Leading match prefix.
        int lp = 0;
        while (lp < exp.length && lp < act.length && exp[lp] == act[lp]) lp++;
        // Trailing match suffix.
        int tp = 0;
        while (tp < exp.length - lp && tp < act.length - lp && exp[exp.length - 1 - tp] == act[act.length - 1 - tp]) tp++;
        // Try sub-sequence containment.
        boolean contained = isSubsequence(exp, act);
        StringBuilder sb = new StringBuilder();
        sb.append("DIFF; exp_len=").append(exp.length).append(" act_len=").append(act.length);
        sb.append(", common_prefix=").append(lp).append(", common_suffix=").append(tp);
        sb.append(", exp⊂act_contiguous=").append(contained);
        return sb.toString();
    }

    private static boolean isSubsequence(int[] needle, int[] haystack) {
        if (needle.length == 0) return true;
        if (needle.length > haystack.length) return false;
        outer:
        for (int s = 0; s + needle.length <= haystack.length; s++) {
            for (int i = 0; i < needle.length; i++) {
                if (haystack[s + i] != needle[i]) continue outer;
            }
            return true;
        }
        return false;
    }

    private static List<List<WellSnap>> groupContiguous(List<WellSnap> ws) {
        List<List<WellSnap>> out = new ArrayList<>();
        if (ws.isEmpty()) return out;
        List<WellSnap> cur = new ArrayList<>();
        cur.add(ws.get(0));
        for (int i = 1; i < ws.size(); i++) {
            if (ws.get(i).obsIdx == ws.get(i - 1).obsIdx + 1) {
                cur.add(ws.get(i));
            } else {
                out.add(cur);
                cur = new ArrayList<>();
                cur.add(ws.get(i));
            }
        }
        out.add(cur);
        return out;
    }

    private static final class WellSnap {
        final int obsIdx;
        final GHPoint snap;
        final int emIdx;
        final double snapDist;
        WellSnap(int obsIdx, GHPoint snap, int emIdx, double snapDist) {
            this.obsIdx = obsIdx;
            this.snap = snap;
            this.emIdx = emIdx;
            this.snapDist = snapDist;
        }
    }

    // ---- GPX parsing (duplicated from TrackConvertDiagnosticTest to avoid coupling) ----

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
