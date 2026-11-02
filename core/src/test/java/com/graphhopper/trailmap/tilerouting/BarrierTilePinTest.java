package com.graphhopper.trailmap.tilerouting;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.jackson.Jackson;
import com.graphhopper.trailmap.TrailmapGraphHopper;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.FileWriter;
import java.io.Writer;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * W26 S4 /tile_route PIN (test_lead_routing, CATALOG §8): every parity-manifest job routed in-JVM on the S1 graph
 * with a virtual clock (reproducible), its response JSON (timings removed) written before the change and compared
 * after it, line by line. The stored prototype runs predate S1, so this fresh capture is the before.
 * {@code -Dtb.flag=v1} routes with the W26 flag (the after-with-flag run; compared only on request).
 * <pre>-Dtb.root=ABS/routing_v2 -Dtb.graph=DIR -Dtb.config=FILE (-Dtb.write=FILE | -Dtb.compare=FILE) [-Dtb.only=REGEX]</pre>
 */
public class BarrierTilePinTest {

    static final ObjectMapper JSON = Jackson.newObjectMapper();

    static void strip(JsonNode n) {
        if (n instanceof ObjectNode) {
            ObjectNode o = (ObjectNode) n;
            o.remove("timings");
            // the note carries the solver's wall-clock ms (prepMs=…): timing, not output
            if (o.has("note") && o.get("note").isTextual())
                o.put("note", o.get("note").asText().replaceAll("(\\w*Ms)=\\d+", "$1=*"));
            o.elements().forEachRemaining(BarrierTilePinTest::strip);
        } else if (n != null && n.isArray()) n.elements().forEachRemaining(BarrierTilePinTest::strip);
    }

    @Test
    void pin() throws Exception {
        Assumptions.assumeTrue(System.getProperty("tb.graph") != null, "tb.graph not set");
        File root = new File(System.getProperty("tb.root"));
        ObjectMapper yaml = Jackson.initObjectMapper(new ObjectMapper(new YAMLFactory()));
        GraphHopperConfig config = yaml.treeToValue(yaml.readTree(new File(System.getProperty("tb.config"))).get("graphhopper"), GraphHopperConfig.class);
        config.putObject("graph.location", System.getProperty("tb.graph"));
        TrailmapGraphHopper hopper = new TrailmapGraphHopper();
        hopper.init(config);
        hopper.setAllowWrites(false);
        hopper.load();
        TileRouter.Config rc = new TileRouter.Config();
        rc.portalBudgetMs = 5000;
        rc.clock = () -> TileRouter.virtualClock(1000);
        TileRouter router = new TileRouter(hopper, rc);
        JsonNode manifest = JSON.readTree(new File(root, "tile-routing/verify/parity-manifest.json"));
        String only = System.getProperty("tb.only");
        List<String> lines = new ArrayList<>();
        for (JsonNode run : manifest.get("runs")) {
            JsonNode jobsFile = JSON.readTree(new File(root, run.get("jobs").asText()));
            for (JsonNode job : jobsFile.get("jobs")) {
                String label = run.get("run").asText() + " " + job.get("jobId").asText();
                if (only != null && !label.matches(only)) continue;
                TileRouteRequest rq = TileRouteParityTest.request(jobsFile, job, run);
                if (System.getProperty("tb.flag") != null) rq.barriers = System.getProperty("tb.flag");
                String out;
                try {
                    ObjectNode got = TileRouteJson.write(JSON, rq, router.route(rq));
                    strip(got);
                    out = got.toString();
                } catch (Exception e) {
                    out = "{\"exception\":" + JSON.writeValueAsString(e.getClass().getName() + ": " + e.getMessage()) + "}";
                }
                lines.add(label + "\t" + out);
            }
        }
        hopper.close();
        System.out.println("TILEPIN jobs " + lines.size());
        if (System.getProperty("tb.write") != null)
            try (Writer w = new FileWriter(System.getProperty("tb.write"))) {
                for (String l : lines) w.write(l + "\n");
            }
        if (System.getProperty("tb.compare") != null) {
            List<String> before = Files.readAllLines(new File(System.getProperty("tb.compare")).toPath());
            int diff = 0;
            for (int i = 0; i < Math.max(before.size(), lines.size()); i++) {
                String b = i < before.size() ? before.get(i) : "", a = i < lines.size() ? lines.get(i) : "";
                if (b.contains("\t")) { // a capture written before the note normalisation: normalise it the same way
                    JsonNode bn = JSON.readTree(b.split("\t", 2)[1]);
                    strip(bn);
                    b = b.split("\t", 2)[0] + "\t" + bn;
                }
                if (!a.equals(b)) {
                    diff++;
                    System.out.println("TILEPIN DIFF " + (a.isEmpty() ? b : a).split("\t")[0]);
                }
            }
            System.out.println("TILEPIN compare: " + diff + " of " + lines.size() + " differ");
            assertEquals(0, diff, "no-flag /tile_route must be identical to the capture");
        }
    }
}
