package com.graphhopper.trailmap.matching;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.PropertyNamingStrategies;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.graphhopper.GraphHopper;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.jackson.GraphHopperModule;
import com.graphhopper.matching.MatchResult;
import com.graphhopper.matching.Observation;
import com.graphhopper.trailmap.TrailmapGraphHopper;
import com.graphhopper.trailmap.shared.TrailmapImportRegistry;
import com.graphhopper.util.PMap;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.FileInputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import static org.junit.jupiter.api.Assertions.assertNotNull;

/**
 * PERFORMANCE diagnostic harness (test-side only, no production effect).
 *
 * <p>Times the Trailmap matcher fork on representative larger GPX tracks while toggling
 * ONE {@link MatcherConfig} lever at a time vs the canonical (Phase-0) baseline, and reports
 * wall-clock match time plus candidate-count evidence (total candidate snaps across all
 * observations, and the number of Viterbi candidate-pair routings — the expensive inner
 * loop {@code router.calcPaths}). This isolates which lever drives the reported regression.
 *
 * <p>Skipped unless the graph cache exists at {@code ../../data/graph-cache}.
 */
public class TrailmapMatcherPerfTest {

    private static final String GRAPH_LOCATION = "../../data/graph-cache";
    private static final String OSM_FILE = "../../data/finland_4.osm.pbf";
    private static final String CONFIG_FILE = "../trailmap-config.yml";
    private static final String GPX_DIR = "../../data/gpx-for-testing/";

    private static final String PROFILE = "gravel";

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

    @Test
    void timeLevers() throws Exception {
        // sigma=20 mirrors the client "default" (hand-drawn / recording) preset, which is the
        // regime the regression report is about. Densify gap and radius mults chosen to match
        // the docs' example values (mult=3 at sigma 20 -> 60 m radius; densify 30 m).
        double sigma = 20.0;

        String[] fixtures = {"l_93km.gpx", "sbc_76.gpx", "l_16km.gpx", "sbc_23.gpx"};

        for (String fixture : fixtures) {
            File gpx = new File(GPX_DIR + fixture);
            if (!gpx.exists()) {
                System.out.println("SKIP (missing): " + fixture);
                continue;
            }
            List<Observation> obs = TrailmapMatcherEquivalenceTest.parseGpx(gpx);
            System.out.println();
            System.out.println("############################################################");
            System.out.printf("# %s  (%d raw observations)  sigma=%.0f profile=%s%n",
                    fixture, obs.size(), sigma, PROFILE);
            System.out.println("############################################################");
            System.out.printf("%-34s %8s %8s %10s %12s %8s%n",
                    "case", "obs", "match_ms", "totCands", "candPairs", "edges");

            // (a) baseline — canonical Phase 0
            runCase("(a) baseline (canonical)", obs, () -> {
                MatcherConfig c = new MatcherConfig();
                c.measurementErrorSigma = sigma;
                return c;
            }, false, 0);

            // (b) +M1 radius mult=3
            runCase("(b) +M1 radius mult=3", obs, () -> {
                MatcherConfig c = new MatcherConfig();
                c.measurementErrorSigma = sigma;
                c.candidateRadiusSigmaMult = 3.0;
                return c;
            }, false, 0);

            // (b2) +M1 radius mult=2 (a milder common setting)
            runCase("(b2) +M1 radius mult=2", obs, () -> {
                MatcherConfig c = new MatcherConfig();
                c.measurementErrorSigma = sigma;
                c.candidateRadiusSigmaMult = 2.0;
                return c;
            }, false, 0);

            // (c) +auto-sigma (two-pass)
            runCase("(c) +auto-sigma", obs, () -> {
                MatcherConfig c = new MatcherConfig();
                c.measurementErrorSigma = sigma;
                c.autoSigma = true;
                return c;
            }, false, 0);

            // (d) +densify 30m
            runCase("(d) +densify 30m", obs, () -> {
                MatcherConfig c = new MatcherConfig();
                c.measurementErrorSigma = sigma;
                return c;
            }, true, 30.0);

            // (e) +M2a lambda=1
            runCase("(e) +M2a lambda=1", obs, () -> {
                MatcherConfig c = new MatcherConfig();
                c.measurementErrorSigma = sigma;
                c.emissionDesirabilityLambda = 1.0;
                return c;
            }, false, 0);

            // (f) ALL ON — the likely user config (radius=3 + auto-sigma + densify + M2a)
            runCase("(f) ALL (M1=3+auto+densify30+M2a1)", obs, () -> {
                MatcherConfig c = new MatcherConfig();
                c.measurementErrorSigma = sigma;
                c.candidateRadiusSigmaMult = 3.0;
                c.autoSigma = true;
                c.emissionDesirabilityLambda = 1.0;
                return c;
            }, true, 30.0);

            // (g) auto-sigma + M1=3 (the compounding pair without densify)
            runCase("(g) +auto-sigma +M1=3", obs, () -> {
                MatcherConfig c = new MatcherConfig();
                c.measurementErrorSigma = sigma;
                c.candidateRadiusSigmaMult = 3.0;
                c.autoSigma = true;
                return c;
            }, false, 0);
        }
    }

    /**
     * Runs one config case (with optional caller-side densification) and prints timing +
     * candidate-count evidence. Each case is run twice; the second (warm) run is reported to
     * remove first-call JIT/cache noise.
     */
    private void runCase(String label, List<Observation> rawObs,
                         Supplier<MatcherConfig> cfgSupplier,
                         boolean densify, double densifyGapM) {
        PMap hints = new PMap();
        hints.putObject("profile", PROFILE);

        // warm
        measure(cfgSupplier.get(), rawObs, densify, densifyGapM, hints);
        // measured
        Stats s = measure(cfgSupplier.get(), rawObs, densify, densifyGapM, hints);

        System.out.printf("%-34s %8d %8d %10d %12d %8d%n",
                label, s.obsAfterDensify, s.matchMs, s.totalCandidates, s.candidatePairs, s.edges);
    }

    private static class Stats {
        long matchMs;
        int obsAfterDensify;
        int totalCandidates;
        long candidatePairs;
        int edges;
    }

    private Stats measure(MatcherConfig cfg, List<Observation> rawObs,
                          boolean densify, double densifyGapM, PMap hints) {
        List<Observation> obs = new ArrayList<>(rawObs);
        if (densify) {
            obs = ObservationDensifier.densify(obs, densifyGapM);
        }
        TrailmapMapMatching m = TrailmapMapMatching.fromGraphHopper(hopper, hints, cfg);

        long t0 = System.nanoTime();
        MatchResult r = m.match(new ArrayList<>(obs));
        long t1 = System.nanoTime();

        Stats s = new Stats();
        s.matchMs = (t1 - t0) / 1_000_000L;
        s.obsAfterDensify = obs.size();
        s.edges = r.getEdgeMatches().size();

        // Candidate-count evidence from the matcher's own statistics. For auto-sigma the
        // stats reflect the FINAL pass; the candidate-pair count is a per-pass figure, and
        // auto-sigma runs the whole thing twice, so its true work is ~2x the reported pairs.
        Object spo = m.getStatistics().get("snapsPerObservation");
        int[] snaps = (spo instanceof int[]) ? (int[]) spo : new int[0];
        int total = 0;
        for (int x : snaps) total += x;
        s.totalCandidates = total;

        // Viterbi inner-loop work proxy: number of candidate-pair routings = sum over
        // consecutive observation pairs of (candidates_i_directed * candidates_{i+1}_directed).
        // Directed candidate count ~= 2x snap count for virtual nodes; we approximate with
        // snap counts which is the dominant factor and matches "snapsPerObservation".
        long pairs = 0;
        for (int i = 0; i + 1 < snaps.length; i++) {
            pairs += (long) snaps[i] * snaps[i + 1];
        }
        s.candidatePairs = pairs;
        return s;
    }
}
