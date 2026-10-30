package com.graphhopper.trailmap.personal.variations;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.jackson.Jackson;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.trailmap.personal.PersonalCells;
import com.graphhopper.trailmap.personal.PersonalFamiliarity;
import com.graphhopper.trailmap.personal.PersonalParams;
import com.graphhopper.trailmap.personal.PersonalRouter;
import com.graphhopper.util.PMap;
import com.graphhopper.util.Parameters;
import com.graphhopper.util.shapes.GHPoint;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.IOException;
import java.io.Reader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;

/**
 * Critical validation of the "bit of new" port: on the 30 round-3 legs, in-JVM on the experiment's graph clone, the
 * ported round-4 alternatives equal round-4's run files (i3 none, i6 avoid, i7 explore) and the ported A3
 * alternatives equal round-3 A's (i5 none, i6 avoid, i7 explore), coordinate for coordinate. Main route exactly as
 * the prototypes routed it (favor, strength 1, switch 30, saturation 10, cycling_lifetime, gravel profile).
 *
 * <p>Runs only where the graph, legs, cells and reference runs exist, given as system properties (skipped when
 * unset): {@code -Dtest=BitOfNewParityTest -Dbon.root=ABS/routing_v2 -Dbon.cells=ABS/cells/<user>/csv
 * [-Dbon.graph=ABS/graphClone] [-Dbon.flavours=none,avoid,explore] [-Dbon.legs=l1,l2]}. Also prints the timing
 * of the history-free run (p50 / p95 of the whole computation, and the round-4 / A3 shares).
 */
public class BitOfNewParityTest {

    static final String ROOT = System.getProperty("bon.root", "");
    static final String EXP = ROOT + "/data/experiments/personal_routing";
    static final String GRAPH = System.getProperty("bon.graph", EXP + "/integ/graph");
    static final String CELLS_DIR = System.getProperty("bon.cells", "");
    static final String LEGS = ROOT + "/data/personal-routing/r3/legs.json";
    static final String PROFILE = "gravel";
    static final Map<String, String> R4_REF = Map.of("none", "i3", "avoid", "i6", "explore", "i7");
    static final Map<String, String> A3_REF = Map.of("none", "i5", "avoid", "i6", "explore", "i7");

    static GraphHopper loadGraph() throws IOException {
        ObjectMapper mapper = Jackson.initObjectMapper(new ObjectMapper(new YAMLFactory()));
        JsonNode gh = mapper.readTree(new File("../trailmap-config.yml")).get("graphhopper");
        GraphHopperConfig config = mapper.treeToValue(gh, GraphHopperConfig.class);
        config.putObject("graph.location", GRAPH);
        config.putObject("graph.elevation.cache_dir", ROOT + "/data/loop/elevation");
        config.putObject("datareader.file", "");
        GraphHopper hopper = new GraphHopper();
        hopper.init(config);
        hopper.setAllowWrites(false);
        if (!hopper.load()) throw new IllegalStateException("graph did not load from " + GRAPH);
        return hopper;
    }

    static PersonalCells readCells(String name) throws IOException {
        try (Reader r = Files.newBufferedReader(Path.of(CELLS_DIR, name + ".csv"), StandardCharsets.UTF_8)) {
            return PersonalCells.read(r);
        }
    }

    static PersonalParams favorParams() {
        PMap h = new PMap();
        h.putObject(PersonalParams.MODE, "favor");
        h.putObject(PersonalParams.USER, "owner");
        h.putObject(PersonalParams.CELLS, "cycling_lifetime");
        h.putObject(PersonalParams.STRENGTH, 1.0);
        h.putObject(PersonalParams.SWITCH_PENALTY, 30.0);
        h.putObject(PersonalParams.RIDES_SATURATION, 10.0);
        return PersonalParams.from(h);
    }

    static boolean same(List<double[]> a, JsonNode ref) {
        if (ref == null || a.size() != ref.size()) return false;
        for (int i = 0; i < a.size(); i++)
            if (a.get(i)[0] != ref.get(i).get(0).asDouble() || a.get(i)[1] != ref.get(i).get(1).asDouble()) return false;
        return true;
    }

    /** "" when equal, else a short description of the first difference. */
    static String compare(List<BitOfNew.Variation> got, JsonNode refAlts) {
        if (refAlts == null) return "no reference";
        if (got.size() != refAlts.size()) return "count " + got.size() + " vs " + refAlts.size();
        for (int k = 0; k < got.size(); k++)
            if (!same(got.get(k).coords, refAlts.get(k)))
                return "alt" + (k + 1) + " geometry (" + got.get(k).coords.size() + " vs " + refAlts.get(k).size() + " points)";
        return "";
    }

    static long pct(List<Double> xs, double q) {
        List<Double> s = new ArrayList<>(xs);
        Collections.sort(s);
        return s.isEmpty() ? 0 : Math.round(s.get((int) Math.ceil(q * s.size()) - 1));
    }

    @Test
    void parityWithPrototypes() throws Exception {
        Assumptions.assumeTrue(new File(GRAPH, "properties").exists() && new File(LEGS).exists()
                && new File(CELLS_DIR, "cycling_lifetime.csv").exists(), "graph, legs or cells not here");
        GraphHopper hopper = loadGraph();
        try {
            Weighting profileW = hopper.createWeighting(hopper.getProfile(PROFILE), new PMap());
            PersonalFamiliarity lifetime = new PersonalFamiliarity(readCells("cycling_lifetime"), hopper.getBaseGraph());
            PersonalFamiliarity last12 = new PersonalFamiliarity(readCells("cycling_last12m"), hopper.getBaseGraph());
            ObjectMapper om = new ObjectMapper();
            JsonNode legs = om.readTree(new File(LEGS));
            String legFilter = System.getProperty("bon.legs", "");
            List<String> flavours = Arrays.asList(System.getProperty("bon.flavours", "none,avoid,explore").split(","));
            List<String> failures = new ArrayList<>();
            for (String flavour : flavours) {
                BitOfNew.History h = BitOfNew.History.of(flavour);
                List<String> r4bad = new ArrayList<>(), a3bad = new ArrayList<>();
                List<Double> tTot = new ArrayList<>(), t4 = new ArrayList<>(), tA = new ArrayList<>();
                int n = 0, offeredTotal = 0, a3dup = 0;
                for (int pass = 0; pass < 2; pass++) {
                    for (JsonNode t : legs) {
                        String id = t.get("id").asText();
                        if (!legFilter.isEmpty() && !Arrays.asList(legFilter.split(",")).contains(id)) continue;
                        GHRequest req = new GHRequest(new GHPoint(t.get("from").get(0).asDouble(), t.get("from").get(1).asDouble()),
                                new GHPoint(t.get("to").get(0).asDouble(), t.get("to").get(1).asDouble()))
                                .setProfile(PROFILE).setPathDetails(List.of("edge_id"));
                        req.putHint(Parameters.Routing.INSTRUCTIONS, false);
                        GHResponse rsp = new PersonalRouter(hopper, lifetime, favorParams()).route(req);
                        if (rsp.hasErrors()) throw new IllegalStateException(id + ": " + rsp.getErrors());
                        BitOfNew.Result r = BitOfNew.compute(hopper.getBaseGraph(), hopper.getEncodingManager(), hopper.getLocationIndex(),
                                profileW, rsp.getBest(), h, lifetime, last12);
                        if (pass == 0) continue;
                        n++;
                        tTot.add(r.msTotal);
                        t4.add(r.msRound4);
                        tA.add(r.msA3);
                        offeredTotal += r.offered.size();
                        a3dup += r.a3Duplicates;
                        File f4 = new File(EXP + "/r4/runs/" + R4_REF.get(flavour), id + ".json");
                        File fa = new File(EXP + "/r3a/runs/" + A3_REF.get(flavour), id + ".json");
                        String d4 = compare(r.round4, f4.exists() ? om.readTree(f4).get("alts") : null);
                        String da = compare(r.a3, fa.exists() ? om.readTree(fa).get("alts") : null);
                        if (!d4.isEmpty()) r4bad.add(id + " (" + d4 + ")");
                        if (!da.isEmpty()) a3bad.add(id + " (" + da + ")");
                        System.out.printf(Locale.ROOT, "%-8s %-4s round4 %d alt %s · a3 %d alt %s · offered %d (a3 dup %d) · %.0f ms (round4 %.0f, a3 %.0f)%n",
                                flavour, id, r.round4.size(), d4.isEmpty() ? "SAME" : "DIFF " + d4, r.a3.size(), da.isEmpty() ? "SAME" : "DIFF " + da,
                                r.offered.size(), r.a3Duplicates, r.msTotal, r.msRound4, r.msA3);
                    }
                }
                double s4 = 0, sA = 0, sT = 0;
                for (int i = 0; i < tTot.size(); i++) {
                    s4 += t4.get(i);
                    sA += tA.get(i);
                    sT += tTot.get(i);
                }
                System.out.printf(Locale.ROOT, "SUMMARY %s: round4 vs r4 %s %d/%d%s · a3 vs r3a %s %d/%d%s · offered %d alts (a3 dropped as duplicate %d) · total ms p50 %d p95 %d · round4 p50 %d p95 %d · a3 p50 %d p95 %d · shares round4 %.0f %% a3 %.0f %%%n",
                        flavour, R4_REF.get(flavour), n - r4bad.size(), n, r4bad.isEmpty() ? "" : " DIFF " + r4bad, A3_REF.get(flavour), n - a3bad.size(), n,
                        a3bad.isEmpty() ? "" : " DIFF " + a3bad, offeredTotal, a3dup, pct(tTot, 0.5), pct(tTot, 0.95), pct(t4, 0.5), pct(t4, 0.95),
                        pct(tA, 0.5), pct(tA, 0.95), 100 * s4 / Math.max(1e-9, sT), 100 * sA / Math.max(1e-9, sT));
                if (!r4bad.isEmpty()) failures.add(flavour + " round4: " + r4bad);
                if (!a3bad.isEmpty()) failures.add(flavour + " a3: " + a3bad);
            }
            Assertions.assertTrue(failures.isEmpty(), "port differs from the prototypes: " + failures);
        } finally {
            hopper.close();
        }
    }
}
