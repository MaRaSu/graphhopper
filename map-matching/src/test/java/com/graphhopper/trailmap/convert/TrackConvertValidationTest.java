package com.graphhopper.trailmap.convert;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.PropertyNamingStrategies;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.graphhopper.GraphHopper;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.jackson.GraphHopperModule;
import com.graphhopper.matching.MapMatching;
import com.graphhopper.matching.MatchResult;
import com.graphhopper.matching.Observation;
import com.graphhopper.trailmap.TrailmapGraphHopper;
import com.graphhopper.trailmap.shared.TrailmapImportRegistry;
import com.graphhopper.util.PMap;
import com.graphhopper.util.PointList;
import com.graphhopper.util.shapes.GHPoint;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import java.io.File;
import java.io.FileInputStream;
import java.util.List;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Regression validation suite for {@link TrackToRouteConverter}.
 *
 * <p>Reads {@code data/gpx-for-testing/test_cases.json}, runs the converter against each
 * named GPX, and asserts that the output matches the recorded baseline within small
 * tolerances. Zero stdout on success; failure messages embed a full segment summary
 * and (for deviation failures) the worst-deviating input GPX point.
 *
 * <p>Four levels of check per case:
 * <ol>
 *   <li>Segment <b>sequence</b> — exact ordered list of {@code (type, distance ± tol)}.
 *       The critical check: catches algorithm regressions that change segment count or
 *       boundaries even when totals stay similar.</li>
 *   <li>{@code total_distance_m} — sum across segments, with small tolerance.</li>
 *   <li>{@code max_deviation_from_input_gpx_m} — hard ceiling on the realized polyline's
 *       worst per-point distance from the original GPX (faithfulness metric).</li>
 *   <li>{@code mean_deviation_from_input_gpx_m} — hard ceiling on the mean per-point
 *       deviation.</li>
 * </ol>
 *
 * <p>Workflow:
 * <ul>
 *   <li>New case: develop in {@code TrackConvertDiagnosticTest}, copy its emitted fixture
 *       block into the fixture file once the result is satisfactory.</li>
 *   <li>Algorithm change: run this test; failures point at the case + diff.</li>
 *   <li>Intentional behavior change: rewrite the fixture entry (re-record from a diagnostic
 *       run) and add a {@code notes} explaining why the baseline moved.</li>
 * </ul>
 */
public class TrackConvertValidationTest {

    // Paths relative to map-matching/ (Maven working directory)
    private static final String GRAPH_LOCATION = "../../data/graph-cache";
    private static final String OSM_FILE = "../../data/finland_4.osm.pbf";
    private static final String CONFIG_FILE = "../trailmap-config.yml";
    private static final String FIXTURE_FILE = "../../data/gpx-for-testing/test_cases.json";

    private static GraphHopper hopper;

    @BeforeAll
    static void setup() throws Exception {
        File graphDir = new File(GRAPH_LOCATION);
        Assumptions.assumeTrue(graphDir.exists() && graphDir.isDirectory(),
                "Graph cache not found — skipping convert validation");

        File fixture = new File(FIXTURE_FILE);
        Assumptions.assumeTrue(fixture.exists() && fixture.isFile(),
                "Fixture file not found at " + fixture.getAbsolutePath() + " — skipping convert validation");

        ObjectMapper yamlMapper = new ObjectMapper(new YAMLFactory());
        yamlMapper.setPropertyNamingStrategy(PropertyNamingStrategies.SNAKE_CASE);
        yamlMapper.registerModule(new GraphHopperModule());

        JsonNode root = yamlMapper.readTree(new FileInputStream(new File(CONFIG_FILE)));
        JsonNode ghNode = root.get("graphhopper");
        assertNotNull(ghNode);

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

    /** Loads test cases for parameterization. */
    static Stream<ConvertValidationHelpers.TestCase> loadCases() throws Exception {
        File fixture = new File(FIXTURE_FILE);
        if (!fixture.exists()) return Stream.empty();
        ObjectMapper m = new ObjectMapper();
        m.setPropertyNamingStrategy(PropertyNamingStrategies.SNAKE_CASE);
        ConvertValidationHelpers.TestCases tcs = m.readValue(fixture,
                ConvertValidationHelpers.TestCases.class);
        return tcs.cases.stream();
    }

    @ParameterizedTest(name = "{0}")
    @MethodSource("loadCases")
    void validate(ConvertValidationHelpers.TestCase c) throws Exception {
        // Resolve GPX path: fixture lists paths relative to the project root
        // (data/...), but the test runs from map-matching/. Strip leading "data/" if
        // present and resolve under ../../data/.
        String gpxRel = c.gpx;
        if (gpxRel.startsWith("data/")) gpxRel = gpxRel.substring("data/".length());
        File gpxFile = new File("../../data/" + gpxRel);
        Assumptions.assumeTrue(gpxFile.exists(),
                "GPX file not found for case '" + c.name + "': " + gpxFile.getAbsolutePath());

        // Read tunables from request_overrides (if present)
        double gpsAccuracy = readDouble(c, "gps_accuracy_m", 5.0);
        double snapThreshold = readDouble(c, "snap_threshold_m",
                TrackToRouteConverter.DEFAULT_SNAP_THRESHOLD_M);
        double minRouted = readDouble(c, "min_routed_segment_m",
                TrackToRouteConverter.DEFAULT_MIN_ROUTED_SEGMENT_M);
        double simplifyEps = readDouble(c, "coordinates_simplify_eps",
                TrackToRouteConverter.DEFAULT_COORDINATES_SIMPLIFY_EPS_M);

        List<Observation> observations = TrackConvertDiagnosticTest.parseGpx(gpxFile);
        assertTrue(observations.size() >= 2, "GPX must have at least 2 points");

        String matchingProfile = c.matchingProfile != null ? c.matchingProfile : c.profile;
        PMap hints = new PMap();
        hints.putObject("profile", matchingProfile);
        MapMatching matching = MapMatching.fromGraphHopper(hopper, hints);
        matching.setMeasurementErrorSigma(gpsAccuracy);
        MatchResult matchResult = matching.match(observations);

        TrackToRouteConverter converter = new TrackToRouteConverter(hopper);
        ConvertTrackResponse rsp = converter.convert(matchResult, observations,
                c.profile, null, snapThreshold, minRouted, simplifyEps);

        // ---- Check 1: segment sequence (type + distance per segment) ----
        if (c.expectedSegments != null) {
            List<ConvertTrackResponse.Segment> actual = rsp.getSegments();
            boolean sequenceOk = true;
            if (actual.size() != c.expectedSegments.size()) {
                sequenceOk = false;
            } else {
                for (int i = 0; i < actual.size(); i++) {
                    ConvertValidationHelpers.ExpectedSegment e = c.expectedSegments.get(i);
                    ConvertTrackResponse.Segment a = actual.get(i);
                    if (!e.type.equals(a.getType())) { sequenceOk = false; break; }
                    if (Math.abs(e.distanceM - a.getDistanceM()) > e.toleranceM) {
                        sequenceOk = false; break;
                    }
                }
            }
            if (!sequenceOk) {
                fail(failureMessage(c, rsp, "Segment sequence mismatch:\n"
                        + ConvertValidationHelpers.segmentSequenceDiff(c.expectedSegments, actual)));
            }
        }

        // ---- Check 2: total distance ----
        if (c.totalDistanceM != null) {
            double actualTotal = rsp.getStats().totalDistanceM;
            if (Math.abs(actualTotal - c.totalDistanceM.value) > c.totalDistanceM.toleranceM) {
                fail(failureMessage(c, rsp, String.format(
                        "Total distance %.1fm differs from expected %.1fm (tol ±%.1fm)",
                        actualTotal, c.totalDistanceM.value, c.totalDistanceM.toleranceM)));
            }
        }

        // ---- Check 3 + 4: faithfulness to input GPX ----
        if (c.maxDeviationFromInputGpxM != null || c.meanDeviationFromInputGpxM != null) {
            PointList realized = ConvertValidationHelpers.realizePolyline(rsp, hopper, c.profile, null);
            double[] devs = ConvertValidationHelpers.perPointDeviations(observations, realized);
            double maxDev = ConvertValidationHelpers.maxDeviation(devs);
            double meanDev = ConvertValidationHelpers.meanDeviation(devs);

            if (c.maxDeviationFromInputGpxM != null
                    && maxDev > c.maxDeviationFromInputGpxM.max) {
                int worst = ConvertValidationHelpers.worstDeviationIndex(devs);
                GHPoint worstP = observations.get(worst).getPoint();
                fail(failureMessage(c, rsp, String.format(
                        "Max deviation %.2fm exceeds ceiling %.1fm. "
                                + "Worst at gpx[%d]=(%.6f, %.6f).",
                        maxDev, c.maxDeviationFromInputGpxM.max,
                        worst, worstP.lat, worstP.lon)));
            }
            if (c.meanDeviationFromInputGpxM != null
                    && meanDev > c.meanDeviationFromInputGpxM.max) {
                fail(failureMessage(c, rsp, String.format(
                        "Mean deviation %.2fm exceeds ceiling %.1fm.",
                        meanDev, c.meanDeviationFromInputGpxM.max)));
            }
        }
    }

    private static double readDouble(ConvertValidationHelpers.TestCase c, String key, double fallback) {
        if (c.requestOverrides == null) return fallback;
        Object v = c.requestOverrides.get(key);
        if (v instanceof Number n) return n.doubleValue();
        return fallback;
    }

    private static String failureMessage(ConvertValidationHelpers.TestCase c,
                                         ConvertTrackResponse rsp,
                                         String reason) {
        return "Case '" + c.name + "' (" + c.profile + "): " + reason + "\n"
                + ConvertValidationHelpers.summarizeSegments(rsp);
    }
}
