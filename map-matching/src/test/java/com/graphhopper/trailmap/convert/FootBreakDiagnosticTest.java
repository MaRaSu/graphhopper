package com.graphhopper.trailmap.convert;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.PropertyNamingStrategies;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.jackson.GraphHopperModule;
import com.graphhopper.config.Profile;
import com.graphhopper.matching.MapMatching;
import com.graphhopper.matching.MatchResult;
import com.graphhopper.matching.Observation;
import com.graphhopper.matching.State;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.ev.BooleanEncodedValue;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.querygraph.QueryGraph;
import com.graphhopper.routing.querygraph.VirtualEdgeIteratorState;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.trailmap.TrailmapGraphHopper;
import com.graphhopper.trailmap.matching.MatcherConfig;
import com.graphhopper.trailmap.matching.ObservationDensifier;
import com.graphhopper.trailmap.matching.TrailmapMapMatching;
import com.graphhopper.trailmap.shared.TrailmapImportRegistry;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.PMap;
import com.graphhopper.util.Parameters;
import com.graphhopper.util.details.PathDetail;
import com.graphhopper.util.shapes.GHPoint;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.FileInputStream;
import java.util.ArrayList;
import java.util.List;

/**
 * Diagnostic harness for the "Sequence is broken for submitted track at time step 2827"
 * failure the user hits when the /convert_track request uses profile=trailmap_foot (the
 * exact same track + cm_* params succeed with profile=mtb/gravel).
 *
 * <p>NOT a regression test — verbose stdout, minimal assertions. No production code touched.
 *
 * <p>The error is thrown by {@link TrailmapMapMatching}'s Viterbi when, from every candidate
 * of some time step, the transition router (using the profile's weighting) cannot reach ANY
 * candidate of the next time step. That is a routing-connectivity fact about the profile, not
 * a segmentation issue. This test reproduces the throw, then pinpoints the exact densified
 * observation pair that is unroutable under trailmap_foot and shows the same pair routes fine
 * under mtb.
 *
 * <p>Run:
 * <pre>
 *   mvn -f graphhopper/pom.xml -pl map-matching test \
 *       -Dtest=FootBreakDiagnosticTest -Dsurefire.useFile=false -DargLine=-Xmx16g
 * </pre>
 */
public class FootBreakDiagnosticTest {

    private static final String GRAPH_LOCATION = "../../data/graph-cache";
    private static final String OSM_FILE = "../../data/finland_2.osm.pbf";
    private static final String CONFIG_FILE = "../trailmap-config.yml";
    private static final String TRACK_JSON = "src/test/resources/trailmap/foot_break_track.json";

    private static final DistanceCalcEarth DIST = DistanceCalcEarth.DIST_EARTH;

    private static GraphHopper hopper;

    @BeforeAll
    static void setup() throws Exception {
        File graphDir = new File(GRAPH_LOCATION);
        Assumptions.assumeTrue(graphDir.exists() && graphDir.isDirectory(),
                "Graph cache not found at " + graphDir.getAbsolutePath() + " — skipping");

        ObjectMapper yamlMapper = new ObjectMapper(new YAMLFactory());
        yamlMapper.setPropertyNamingStrategy(PropertyNamingStrategies.SNAKE_CASE);
        yamlMapper.registerModule(new GraphHopperModule());

        JsonNode root = yamlMapper.readTree(new FileInputStream(new File(CONFIG_FILE)));
        JsonNode ghNode = root.get("graphhopper");
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

    /** Mirror the request's cm_* config exactly. */
    private static MatcherConfig requestConfig() {
        MatcherConfig cfg = new MatcherConfig();
        cfg.measurementErrorSigma = 20.0;            // gps_accuracy_m
        cfg.transitionProbabilityBeta = 4.0;          // cm_beta
        cfg.candidateRadiusSigmaMult = 3.0;           // cm_candidate_radius_sigma_mult
        cfg.autoSigma = true;                         // cm_auto_sigma
        cfg.autoSigmaMaxM = 10.0;                     // cm_auto_sigma_max_m
        cfg.adaptiveSigma = true;                     // cm_adaptive_sigma
        cfg.densifyMaxGapM = 40.0;                    // cm_densify_max_gap_m
        cfg.emissionDesirabilityLambda = 1.0;         // cm_emission_desirability_lambda
        // cm_segmentation_v2 is a segmenter flag, irrelevant to the match() throw.
        return cfg;
    }

    private static List<Observation> loadTrack() throws Exception {
        return loadTrack(TRACK_JSON);
    }

    private static List<Observation> loadTrack(String path) throws Exception {
        ObjectMapper om = new ObjectMapper();
        JsonNode arr = om.readTree(new File(path));
        List<Observation> obs = new ArrayList<>(arr.size());
        for (JsonNode pt : arr) {
            obs.add(new Observation(new GHPoint(pt.get(0).asDouble(), pt.get(1).asDouble())));
        }
        return obs;
    }

    /** Try to match the (already-densified) observations under the given profile. */
    private static String tryMatch(String profile, List<Observation> densified) {
        PMap hints = new PMap();
        hints.putObject("profile", profile);
        MatcherConfig cfg = requestConfig();
        cfg.densifyMaxGapM = null; // already densified by caller — don't double densify
        TrailmapMapMatching fork = TrailmapMapMatching.fromGraphHopper(hopper, hints, cfg);
        try {
            MatchResult mr = fork.match(densified);
            return "OK: " + mr.getEdgeMatches().size() + " edge matches, matchLength="
                    + Math.round(mr.getMatchLength()) + "m";
        } catch (Exception e) {
            return "THREW " + e.getClass().getSimpleName() + ": " + e.getMessage();
        }
    }

    /**
     * Route between two points under a profile (Dijkstra, CH+LM disabled — same engine family
     * as the matcher's transition router). Returns distance in meters, or -1 if no path.
     */
    private static double routeDist(String profile, GHPoint a, GHPoint b) {
        GHRequest req = new GHRequest(a.lat, a.lon, b.lat, b.lon);
        req.setProfile(profile);
        req.putHint(Parameters.CH.DISABLE, true);
        req.putHint(Parameters.Landmark.DISABLE, true);
        req.putHint("instructions", false);
        req.putHint("calc_points", false);
        GHResponse rr = hopper.route(req);
        if (rr.hasErrors()) return -1;
        return rr.getBest().getDistance();
    }

    @Test
    void diagnose() throws Exception {
        List<Observation> raw = loadTrack();
        List<Observation> densified = ObservationDensifier.densify(raw, 40.0);
        System.out.println("=== INPUT ===");
        System.out.println("raw observations:       " + raw.size());
        System.out.println("densified (maxGap=40m): " + densified.size());

        // 1) Reproduce: trailmap_foot throws, mtb succeeds — same track, same cm params.
        System.out.println("\n=== MATCH REPRODUCTION (custom matcher, request cm_* params) ===");
        System.out.println("profile=trailmap_foot -> " + tryMatch("trailmap_foot", densified));
        System.out.println("profile=mtb           -> " + tryMatch("mtb", densified));

        // 2) Pinpoint the unroutable transition under trailmap_foot.
        // The Viterbi break means: no candidate->candidate path exists across some consecutive
        // pair. Probe every consecutive densified pair with /route (ground-truth connectivity).
        // Report every foot-unroutable pair, and whether mtb can route the same pair.
        System.out.println("\n=== PER-PAIR CONNECTIVITY PROBE (consecutive densified points) ===");
        System.out.println("Listing pairs where trailmap_foot has NO path (route between the two "
                + "raw densified points, CH/LM off):");
        int footBreaks = 0;
        for (int i = 0; i + 1 < densified.size(); i++) {
            GHPoint a = densified.get(i).getPoint();
            GHPoint b = densified.get(i + 1).getPoint();
            double footD = routeDist("trailmap_foot", a, b);
            if (footD < 0) {
                footBreaks++;
                double straight = DIST.calcDist(a.lat, a.lon, b.lat, b.lon);
                double mtbD = routeDist("mtb", a, b);
                System.out.printf(
                        "  [%d->%d] FOOT=NO PATH  mtb=%s  straight=%.1fm  a=%.6f,%.6f  b=%.6f,%.6f%n",
                        i, i + 1, (mtbD < 0 ? "NO PATH" : String.format("%.1fm", mtbD)),
                        straight, a.lat, a.lon, b.lat, b.lon);
            }
        }
        System.out.println("total foot-unroutable consecutive pairs: " + footBreaks);

        // 3) Zoom in on the observation named in the user's error (67.552678, 24.234854).
        GHPoint errPt = new GHPoint(67.552678, 24.234854);
        int nearest = nearestDensifiedIndex(densified, errPt);
        System.out.println("\n=== ZOOM AT ERROR OBSERVATION 67.552678,24.234854 ===");
        System.out.println("nearest densified index: " + nearest);
        int lo = Math.max(0, nearest - 6);
        int hi = Math.min(densified.size() - 1, nearest + 6);
        for (int i = lo; i < hi; i++) {
            GHPoint a = densified.get(i).getPoint();
            GHPoint b = densified.get(i + 1).getPoint();
            double footD = routeDist("trailmap_foot", a, b);
            double mtbD = routeDist("mtb", a, b);
            double straight = DIST.calcDist(a.lat, a.lon, b.lat, b.lon);
            System.out.printf("  [%d->%d] straight=%5.1fm  foot=%-9s  mtb=%-9s   a=%.6f,%.6f%n",
                    i, i + 1, straight,
                    footD < 0 ? "NO PATH" : String.format("%.0fm", footD),
                    mtbD < 0 ? "NO PATH" : String.format("%.0fm", mtbD),
                    a.lat, a.lon);
        }

        // 4) For the first foot break near the error, measure how far foot must skip to reconnect
        // (reveals the size of the foot-impassable gap along the track).
        Integer firstBreak = firstFootBreakAtOrAfter(densified, lo);
        if (firstBreak != null) {
            GHPoint from = densified.get(firstBreak).getPoint();
            System.out.println("\n=== FOOT RECONNECTION SCAN from densified[" + firstBreak + "] ==="
                    + " (" + from.lat + "," + from.lon + ")");
            for (int j = firstBreak + 1; j <= Math.min(densified.size() - 1, firstBreak + 25); j++) {
                GHPoint to = densified.get(j).getPoint();
                double footD = routeDist("trailmap_foot", from, to);
                double straight = DIST.calcDist(from.lat, from.lon, to.lat, to.lon);
                System.out.printf("  foot [%d->%d] straight=%6.1fm  foot=%s%n",
                        firstBreak, j, straight, footD < 0 ? "NO PATH" : String.format("%.0fm", footD));
                if (footD >= 0) {
                    System.out.println("  ^ foot reconnects here.");
                    break;
                }
            }
        }

        // 5) Root cause: enumerate the edges mtb traverses across the failing transition
        // (2836 -> 2837) and, for each, report whether trailmap_foot can traverse it and why not.
        System.out.println("\n=== BLOCKING-EDGE ANALYSIS (mtb path 2836->2837) ===");
        GHPoint a = densified.get(2836).getPoint();
        GHPoint b = densified.get(2837).getPoint();
        inspectMtbEdgesAcross(a, b);
    }

    private void inspectMtbEdgesAcross(GHPoint a, GHPoint b) {
        GHRequest req = new GHRequest(a.lat, a.lon, b.lat, b.lon);
        req.setProfile("mtb");
        req.putHint(Parameters.CH.DISABLE, true);
        req.putHint(Parameters.Landmark.DISABLE, true);
        req.setPathDetails(java.util.Collections.singletonList("edge_id"));
        GHResponse rr = hopper.route(req);
        if (rr.hasErrors()) {
            System.out.println("  mtb route errored: " + rr.getErrors());
            return;
        }
        EncodingManager em = hopper.getEncodingManager();
        BooleanEncodedValue footAccess = em.hasEncodedValue("foot_access")
                ? em.getBooleanEncodedValue("foot_access") : null;
        BooleanEncodedValue footSubnet = em.hasEncodedValue("trailmap_foot_subnetwork")
                ? em.getBooleanEncodedValue("trailmap_foot_subnetwork") : null;
        BooleanEncodedValue mtbSubnet = em.hasEncodedValue("mtb_subnetwork")
                ? em.getBooleanEncodedValue("mtb_subnetwork") : null;
        EnumEncodedValue<?> roadClass = em.hasEncodedValue("road_class")
                ? em.getEnumEncodedValue("road_class", (Class) enumClass(em, "road_class")) : null;
        EnumEncodedValue<?> surface = em.hasEncodedValue("surface")
                ? em.getEnumEncodedValue("surface", (Class) enumClass(em, "surface")) : null;
        EnumEncodedValue<?> footRoadAccess = em.hasEncodedValue("foot_road_access")
                ? em.getEnumEncodedValue("foot_road_access", (Class) enumClass(em, "foot_road_access")) : null;

        Weighting footW = hopper.createWeighting(hopper.getProfile("trailmap_foot"), new PMap());
        Weighting mtbW = hopper.createWeighting(hopper.getProfile("mtb"), new PMap());

        List<PathDetail> details = rr.getBest().getPathDetails().get("edge_id");
        System.out.println("  mtb path edges: " + (details == null ? 0 : details.size()));
        if (details == null) return;
        for (PathDetail d : details) {
            int edgeId = ((Number) d.getValue()).intValue();
            EdgeIteratorState e = hopper.getBaseGraph().getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            double footFwd = footW.calcEdgeWeight(e, false);
            double footRev = footW.calcEdgeWeight(e, true);
            double mtbFwd = mtbW.calcEdgeWeight(e, false);
            StringBuilder sb = new StringBuilder();
            sb.append(String.format("  edge %-8d len=%6.1fm", edgeId, e.getDistance()));
            if (roadClass != null) sb.append("  road_class=").append(e.get(roadClass));
            if (surface != null) sb.append("  surface=").append(e.get(surface));
            if (footAccess != null) sb.append("  foot_access=").append(e.get(footAccess));
            if (footRoadAccess != null) sb.append("  foot_road_access=").append(e.get(footRoadAccess));
            if (footSubnet != null) sb.append("  foot_subnetwork=").append(e.get(footSubnet));
            if (mtbSubnet != null) sb.append("  mtb_subnetwork=").append(e.get(mtbSubnet));
            sb.append(String.format("  | footW(fwd/rev)=%s/%s  mtbW(fwd)=%s",
                    fmtW(footFwd), fmtW(footRev), fmtW(mtbFwd)));
            boolean footBlocked = Double.isInfinite(footFwd) && Double.isInfinite(footRev);
            System.out.println(sb + (footBlocked ? "   <== FOOT-BLOCKED" : ""));
        }
    }

    private static String fmtW(double w) {
        return Double.isInfinite(w) ? "INF" : String.format("%.1f", w);
    }

    /**
     * Reproduce the matcher's OWN directed, edge-based transition (same candidate snaps, same
     * QueryGraph, same router as {@code computeViterbiSequence}) for each consecutive densified
     * pair in a window, and report for each: the candidate snaps (are they on ALLOWED edges near
     * the track?) and whether ANY candidate->candidate transition finds a path. The pair where
     * ALL transitions fail is the literal "Sequence is broken" point.
     */
    @Test
    void probeTransitionMechanism() throws Exception {
        List<Observation> raw = loadTrack();
        List<Observation> densified = ObservationDensifier.densify(raw, 40.0);

        String profile = "trailmap_foot";
        PMap hints = new PMap();
        hints.putObject("profile", profile);
        MatcherConfig cfg = requestConfig();
        cfg.densifyMaxGapM = null;
        cfg.adaptiveSigma = false;   // fixed representative σ so findCandidateSnaps is deterministic here
        cfg.autoSigma = false;
        cfg.measurementErrorSigma = 20.0;
        TrailmapMapMatching fork = TrailmapMapMatching.fromGraphHopper(hopper, hints, cfg);
        MapMatching.Router router = MapMatching.routerFromGraphHopper(hopper, hints);

        Weighting footW = hopper.createWeighting(hopper.getProfile("trailmap_foot"), new PMap());

        System.out.println("=== MATCHER TRANSITION PROBE (" + profile + ", directed edge-based, real router) ===");
        System.out.println("Window around the break. 'candA/candB' = #candidate snaps; 'nearest' = closest "
                + "snap dist (m) + is its edge foot-traversable; 'trans' = found/total candidate transitions.");
        int lo = 2832, hi = 2841;
        for (int i = lo; i < hi; i++) {
            Observation a = densified.get(i);
            Observation b = densified.get(i + 1);
            List<Snap> snapsA = fork.findCandidateSnaps(a.getPoint().lat, a.getPoint().lon);
            List<Snap> snapsB = fork.findCandidateSnaps(b.getPoint().lat, b.getPoint().lon);

            List<Snap> all = new ArrayList<>();
            all.addAll(snapsA);
            all.addAll(snapsB);
            QueryGraph qg = QueryGraph.create(hopper.getBaseGraph(), all);

            List<State> fromStates = buildStates(qg, a, snapsA);
            List<State> toStates = buildStates(qg, b, snapsB);
            int[] toNodes = toStates.stream().mapToInt(s -> s.getSnap().getClosestNode()).toArray();
            int[] toInEdges = toStates.stream()
                    .mapToInt(s -> s.isOnDirectedEdge() ? s.getIncomingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE)
                    .toArray();

            int total = 0, found = 0;
            double minDist = Double.MAX_VALUE;
            for (State from : fromStates) {
                int fromNode = from.getSnap().getClosestNode();
                int fromOutEdge = from.isOnDirectedEdge() ? from.getOutgoingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE;
                List<Path> paths = router.calcPaths(qg, fromNode, fromOutEdge, toNodes, toInEdges);
                for (Path p : paths) {
                    total++;
                    if (p.isFound()) {
                        found++;
                        minDist = Math.min(minDist, p.getDistance());
                    }
                }
            }
            System.out.printf(
                    "  [%d->%d] candA=%d candB=%d  nearestA=%s  nearestB=%s  trans=%d/%d  %s%n",
                    i, i + 1, snapsA.size(), snapsB.size(),
                    describeNearest(snapsA, footW), describeNearest(snapsB, footW),
                    found, total,
                    found == 0 ? "<== ALL TRANSITIONS FAIL (Viterbi cannot advance here)"
                            : String.format("minPath=%.0fm", minDist));
        }
    }

    /**
     * Show, for the observations in the failing stretch, how far the track is from ANY edge vs
     * from a FOOT-LEGAL edge, and whether the matcher's candidate search comes up EMPTY at small
     * (adaptive-floor) sigma — the exact way the Viterbi loses a timestep and aborts the track.
     */
    @Test
    void probeOffNetwork() throws Exception {
        List<Observation> raw = loadTrack();
        List<Observation> densified = ObservationDensifier.densify(raw, 40.0);

        PMap footHints = new PMap();
        footHints.putObject("profile", "trailmap_foot");
        MapMatching.Router footRouter = MapMatching.routerFromGraphHopper(hopper, footHints);
        com.graphhopper.routing.util.EdgeFilter footFilter = footRouter.getSnapFilter();
        com.graphhopper.routing.util.EdgeFilter allFilter = com.graphhopper.routing.util.EdgeFilter.ALL_EDGES;

        EncodingManager em = hopper.getEncodingManager();
        EnumEncodedValue<?> roadClass = em.getEnumEncodedValue("road_class", (Class) enumClass(em, "road_class"));
        BooleanEncodedValue footAccess = em.getBooleanEncodedValue("foot_access");

        System.out.println("=== OFF-NETWORK PROBE: track vs nearest ALL edge vs nearest FOOT-LEGAL edge ===");
        for (int i = 2833; i <= 2842; i++) {
            GHPoint p = densified.get(i).getPoint();
            com.graphhopper.storage.index.Snap sAll =
                    hopper.getLocationIndex().findClosest(p.lat, p.lon, allFilter);
            com.graphhopper.storage.index.Snap sFoot =
                    hopper.getLocationIndex().findClosest(p.lat, p.lon, footFilter);
            String allDesc = sAll.isValid()
                    ? String.format("%.1fm edge %d road_class=%s foot_access=%s", sAll.getQueryDistance(),
                            sAll.getClosestEdge().getEdge(), sAll.getClosestEdge().get(roadClass),
                            sAll.getClosestEdge().get(footAccess))
                    : "none";
            String footDesc = sFoot.isValid()
                    ? String.format("%.1fm edge %d", sFoot.getQueryDistance(), sFoot.getClosestEdge().getEdge())
                    : "none";
            System.out.printf("  obs[%d] %.6f,%.6f  nearestANY=[%s]  nearestFOOT-LEGAL=[%s]%n",
                    i, p.lat, p.lon, allDesc, footDesc);
        }

        // Candidate emptiness vs sigma (adaptive σ is clamped to [min≈3, max≈10-20]; radius=3·σ,
        // grown up to 50× before giving up). If the nearest foot-legal edge is beyond that reach,
        // the observation gets ZERO candidates and the Viterbi cannot place it -> track aborts.
        System.out.println("\n=== CANDIDATE COUNT vs SIGMA at obs[2837] (nearest foot-legal edge is far) ===");
        int probeIdx = 2837;
        GHPoint pp = densified.get(probeIdx).getPoint();
        for (double sigma : new double[]{3, 4, 5, 8, 10, 15, 20}) {
            MatcherConfig cfg = requestConfig();
            cfg.densifyMaxGapM = null;
            cfg.adaptiveSigma = false;
            cfg.autoSigma = false;
            cfg.measurementErrorSigma = sigma;
            TrailmapMapMatching fork = TrailmapMapMatching.fromGraphHopper(hopper, footHints, cfg);
            List<Snap> snaps = fork.findCandidateSnaps(pp.lat, pp.lon, sigma);
            double radius = Math.min(200, 3 * sigma);
            System.out.printf("  sigma=%-4.0f radius=3σ=%-5.0fm (reach≈50×=%.0fm)  candidates=%d  %s%n",
                    sigma, radius, 50 * radius, snaps.size(),
                    snaps.isEmpty() ? "<== EMPTY: this obs cannot be placed -> Viterbi break" : "");
        }
    }

    private static final String GRAVEL_TRACK_JSON = "src/test/resources/trailmap/gravel_break_track.json";

    /**
     * END-TO-END VALIDATION of the gap-splitting fix. For both confirmed triggers, drive the FULL
     * request config (adaptive+auto σ, densify, segmentation v2) through the matcher and the
     * {@link TrackToRouteConverter}, and assert: (1) the matcher no longer throws; (2) the
     * MatchResult contains gap tracepoints (matched=false) across the unmatchable stretch; (3) the
     * final response has a coordinates section AND routed segments (routed → coords → routed).
     */
    @Test
    void validateGapSplitting() throws Exception {
        runFullPipeline("T1/trailmap_foot", TRACK_JSON, "trailmap_foot");
        runFullPipeline("T2/gravel", GRAVEL_TRACK_JSON, "gravel");
    }

    private void runFullPipeline(String label, String trackFile, String profile) throws Exception {
        List<Observation> raw = loadTrack(trackFile);
        List<Observation> densified = ObservationDensifier.densify(raw, 40.0);

        PMap hints = new PMap();
        hints.putObject("profile", profile);
        MatcherConfig cfg = requestConfig();
        cfg.densifyMaxGapM = null; // already densified
        TrailmapMapMatching fork = TrailmapMapMatching.fromGraphHopper(hopper, hints, cfg);

        // (1) no throw
        MatchResult mr = fork.match(densified);

        // (2) gap tracepoints present
        long gapTps = mr.getTracepoints().stream().filter(tp -> !tp.isMatched()).count();
        long matchedTps = mr.getTracepoints().stream().filter(com.graphhopper.matching.Tracepoint::isMatched).count();

        // Derive segmenter thresholds the way the resource does in adaptive/auto-σ mode.
        double estSigma = ((Number) fork.getStatistics().get("autoSigmaEstimatedM")).doubleValue();
        double snapThreshold = TrackToRouteConverter.AUTO_SIGMA_SNAP_THRESHOLD_MULT * estSigma;
        double driftFloor = TrackToRouteConverter.AUTO_SIGMA_DRIFT_FLOOR_MULT * estSigma;

        TrackToRouteConverter conv = new TrackToRouteConverter(hopper);
        conv.setSegmentationV2Enabled(true);
        ConvertTrackResponse resp = conv.convert(mr, densified, profile, null, snapThreshold,
                TrackToRouteConverter.DEFAULT_MIN_ROUTED_SEGMENT_M,
                TrackToRouteConverter.DEFAULT_COORDINATES_SIMPLIFY_EPS_M,
                TrackToRouteConverter.DEFAULT_MIN_DETOUR_M,
                TrackToRouteConverter.DEFAULT_MAX_DETOUR_RATIO,
                driftFloor, false);

        StringBuilder types = new StringBuilder();
        for (ConvertTrackResponse.Segment s : resp.getSegments()) {
            types.append(ConvertTrackResponse.Segment.TYPE_COORDINATES.equals(s.getType()) ? "C" : "R");
        }
        System.out.printf("%n=== VALIDATE %s ===%n", label);
        System.out.printf("  match OK: %d tracepoints (%d matched, %d gap)%n",
                mr.getTracepoints().size(), matchedTps, gapTps);
        System.out.printf("  edgeMatches=%d matchLength=%.0fm%n", mr.getEdgeMatches().size(), mr.getMatchLength());
        System.out.printf("  response: %d segments [%s], %d routed, %d coords, total=%.0fm%n",
                resp.getSegments().size(), types,
                resp.getStats().routedSegments, resp.getStats().coordinatesSegments,
                resp.getStats().totalDistanceM);

        assertTrue(gapTps > 0, label + ": expected gap tracepoints (matched=false)");
        assertTrue(resp.getStats().coordinatesSegments >= 1, label + ": expected a coordinates section");
        assertTrue(resp.getStats().routedSegments >= 1, label + ": expected routed segments around the gap");
    }

    /**
     * Second failure case (profile=gravel): "Sequence is broken at time step 42.
     * observation:Observation{point=63.5781741875,26.837352312500002}". The observation has
     * interpolated coordinates -> a DENSIFIED point. Diagnose whether the mechanism is the same
     * as the foot case (off-network observation -> matcher aborts whole track) or different.
     */
    @Test
    void diagnoseGravelCase() throws Exception {
        List<Observation> raw = loadTrack(GRAVEL_TRACK_JSON);
        List<Observation> densified = ObservationDensifier.densify(raw, 40.0);
        System.out.println("=== GRAVEL CASE INPUT ===");
        System.out.println("raw observations:       " + raw.size());
        System.out.println("densified (maxGap=40m): " + densified.size());

        System.out.println("\n=== MATCH REPRODUCTION ===");
        System.out.println("profile=gravel -> " + tryMatch("gravel", densified));

        // The break point is interpolated. Show the RAW segment it was densified from.
        GHPoint errPt = new GHPoint(63.5781741875, 26.837352312500002);
        int rawIdx = nearestDensifiedIndex(raw, errPt);
        System.out.println("\n=== WHERE THE BREAK POINT CAME FROM ===");
        System.out.printf("error obs 63.5781741875,26.837352 is NOT a raw point.%n");
        System.out.printf("nearest raw points: raw[%d]=%.6f,%.6f and raw[%d]=%.6f,%.6f%n",
                rawIdx, raw.get(rawIdx).getPoint().lat, raw.get(rawIdx).getPoint().lon,
                rawIdx + 1, raw.get(rawIdx + 1).getPoint().lat, raw.get(rawIdx + 1).getPoint().lon);
        double rawGap = DIST.calcDist(raw.get(rawIdx).getPoint().lat, raw.get(rawIdx).getPoint().lon,
                raw.get(rawIdx + 1).getPoint().lat, raw.get(rawIdx + 1).getPoint().lon);
        System.out.printf("straight-line gap between those raw points: %.0f m (densified into ~%d synthetic pts)%n",
                rawGap, (int) (rawGap / 40));

        int di = nearestDensifiedIndex(densified, errPt);
        System.out.println("nearest densified index to break point: " + di
                + " = " + densified.get(di).getPoint().lat + "," + densified.get(di).getPoint().lon);

        // Off-network probe: nearest ANY edge vs nearest GRAVEL-LEGAL edge, and road_class.
        PMap gravelHints = new PMap();
        gravelHints.putObject("profile", "gravel");
        MapMatching.Router gravelRouter = MapMatching.routerFromGraphHopper(hopper, gravelHints);
        com.graphhopper.routing.util.EdgeFilter gravelFilter = gravelRouter.getSnapFilter();
        com.graphhopper.routing.util.EdgeFilter allFilter = com.graphhopper.routing.util.EdgeFilter.ALL_EDGES;
        EncodingManager em = hopper.getEncodingManager();
        EnumEncodedValue<?> roadClass = em.getEnumEncodedValue("road_class", (Class) enumClass(em, "road_class"));
        Weighting gravelW = hopper.createWeighting(hopper.getProfile("gravel"), new PMap());

        System.out.println("\n=== OFF-NETWORK PROBE (gravel): densified obs vs nearest ANY vs nearest GRAVEL-LEGAL edge ===");
        int lo = Math.max(0, di - 5), hi = Math.min(densified.size() - 1, di + 5);
        for (int i = lo; i <= hi; i++) {
            GHPoint p = densified.get(i).getPoint();
            com.graphhopper.storage.index.Snap sAll = hopper.getLocationIndex().findClosest(p.lat, p.lon, allFilter);
            com.graphhopper.storage.index.Snap sG = hopper.getLocationIndex().findClosest(p.lat, p.lon, gravelFilter);
            String allDesc = sAll.isValid()
                    ? String.format("%.1fm edge %d road_class=%s gravelW=%s", sAll.getQueryDistance(),
                            sAll.getClosestEdge().getEdge(), sAll.getClosestEdge().get(roadClass),
                            fmtW(gravelW.calcEdgeWeight(sAll.getClosestEdge(), false)))
                    : "none";
            String gDesc = sG.isValid()
                    ? String.format("%.1fm edge %d", sG.getQueryDistance(), sG.getClosestEdge().getEdge())
                    : "none";
            System.out.printf("  d[%d] %.6f,%.6f  nearestANY=[%s]  nearestGRAVEL=[%s]%n",
                    i, p.lat, p.lon, allDesc, gDesc);
        }

        // Candidate count vs sigma across the drift (mode A = empty candidates). The break is
        // reported at the LAST reached obs; the NEXT ones drift farther and lose candidates.
        System.out.println("\n=== CANDIDATE COUNT vs SIGMA across the interpolated drift ===");
        for (int idx = di; idx <= Math.min(densified.size() - 1, di + 5); idx++) {
            GHPoint pp = densified.get(idx).getPoint();
            StringBuilder sb = new StringBuilder(String.format("  densified[%d]: ", idx));
            for (double sigma : new double[]{3, 4, 5, 8, 10}) {
                MatcherConfig cfg = requestConfig();
                cfg.densifyMaxGapM = null;
                cfg.adaptiveSigma = false;
                cfg.autoSigma = false;
                cfg.measurementErrorSigma = sigma;
                TrailmapMapMatching fork = TrailmapMapMatching.fromGraphHopper(hopper, gravelHints, cfg);
                int c = fork.findCandidateSnaps(pp.lat, pp.lon, sigma).size();
                sb.append(String.format("σ%.0f=%d ", sigma, c));
            }
            System.out.println(sb);
        }

        // Per-pair gravel routing across the interpolated jump (mode B check).
        System.out.println("\n=== PER-PAIR GRAVEL ROUTE across the densified jump ===");
        for (int i = lo; i < hi; i++) {
            GHPoint a = densified.get(i).getPoint();
            GHPoint b = densified.get(i + 1).getPoint();
            double d = routeDist("gravel", a, b);
            double straight = DIST.calcDist(a.lat, a.lon, b.lat, b.lon);
            System.out.printf("  [%d->%d] straight=%.1fm gravel=%s%n",
                    i, i + 1, straight, d < 0 ? "NO PATH" : String.format("%.0fm", d));
        }
    }

    /** Nearest candidate snap: distance + whether its edge is foot-traversable (finite weight). */
    private static String describeNearest(List<Snap> snaps, Weighting footW) {
        if (snaps.isEmpty()) return "NONE";
        Snap s = snaps.get(0); // findCandidateSnaps returns sorted by distance
        boolean traversable = !Double.isInfinite(footW.calcEdgeWeight(s.getClosestEdge(), false))
                || !Double.isInfinite(footW.calcEdgeWeight(s.getClosestEdge(), true));
        return String.format("%.1fm(edge %d,%s)", s.getQueryDistance(), s.getClosestEdge().getEdge(),
                traversable ? "footOK" : "footINF");
    }

    /** Mirror {@code TrailmapMapMatching.createTimeSteps} candidate construction. */
    private static List<State> buildStates(QueryGraph qg, Observation obs, List<Snap> snaps) {
        List<State> states = new ArrayList<>();
        for (Snap split : snaps) {
            if (qg.isVirtualNode(split.getClosestNode())) {
                List<VirtualEdgeIteratorState> ve = new ArrayList<>();
                EdgeIterator it = qg.createEdgeExplorer().setBaseNode(split.getClosestNode());
                while (it.next()) {
                    if (!qg.isVirtualEdge(it.getEdge())) continue;
                    ve.add((VirtualEdgeIteratorState) qg.getEdgeIteratorState(it.getEdge(), it.getAdjNode()));
                }
                if (ve.size() == 2) {
                    states.add(new State(obs, split, ve.get(0), ve.get(1)));
                    states.add(new State(obs, split, ve.get(1), ve.get(0)));
                }
            } else {
                states.add(new State(obs, split));
            }
        }
        return states;
    }

    private static Class<? extends Enum> enumClass(EncodingManager em, String key) {
        // road_class -> com.graphhopper.routing.ev.RoadClass, etc.
        String camel = toCamel(key);
        for (String pkg : new String[]{"com.graphhopper.routing.ev.", "com.graphhopper.trailmap.routing.ev."}) {
            try {
                return (Class<? extends Enum>) Class.forName(pkg + camel);
            } catch (ClassNotFoundException ignored) {
            }
        }
        throw new RuntimeException("enum class not found for " + key);
    }

    private static String toCamel(String snake) {
        StringBuilder sb = new StringBuilder();
        boolean up = true;
        for (char c : snake.toCharArray()) {
            if (c == '_') { up = true; continue; }
            sb.append(up ? Character.toUpperCase(c) : c);
            up = false;
        }
        return sb.toString();
    }

    private static int nearestDensifiedIndex(List<Observation> obs, GHPoint p) {
        int best = -1;
        double bestD = Double.MAX_VALUE;
        for (int i = 0; i < obs.size(); i++) {
            GHPoint o = obs.get(i).getPoint();
            double d = DIST.calcDist(o.lat, o.lon, p.lat, p.lon);
            if (d < bestD) { bestD = d; best = i; }
        }
        return best;
    }

    private static Integer firstFootBreakAtOrAfter(List<Observation> obs, int start) {
        for (int i = start; i + 1 < obs.size(); i++) {
            if (routeDist("trailmap_foot", obs.get(i).getPoint(), obs.get(i + 1).getPoint()) < 0) {
                return i;
            }
        }
        return null;
    }
}
