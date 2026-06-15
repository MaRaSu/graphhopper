package com.graphhopper.trailmap.analysis.gravel;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.File;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * End-to-end smoke test (design §15): import the small Monaco fixture through the real
 * Trailmap import pipeline, run all phases, and assert the two JSON artifacts are produced
 * and internally consistent. This exercises the bootstrap that the pure-algorithm unit
 * tests cannot: the dedicated analysis-graph config, EV resolution, and the projection.
 */
class GravelSegmentToolMonacoTest {

    private static final String MONACO = "../core/files/monaco.osm.gz";

    @Test
    void importsMonacoAndEmitsConsistentArtifacts(@TempDir Path tmp) throws Exception {
        File graphDir = tmp.resolve("gh").toFile();
        File outDir = tmp.resolve("out").toFile();

        GravelSegmentTool.main(new String[]{
                "datareader.file=" + MONACO,
                "graph.location=" + graphDir.getAbsolutePath(),
                "out.dir=" + outDir.getAbsolutePath(),
                "gravel.size_threshold_m=50"
        });

        File waysFile = new File(outDir, "qualifying_ways.json");
        File roadsFile = new File(outDir, "logical_roads.json");
        assertTrue(waysFile.exists(), "qualifying_ways.json written");
        assertTrue(roadsFile.exists(), "logical_roads.json written");

        ObjectMapper mapper = new ObjectMapper();
        JsonNode ways = mapper.readTree(waysFile);
        JsonNode roads = mapper.readTree(roadsFile);

        assertTrue(ways.has("way_ids") && ways.get("way_ids").isArray(), "way_ids array present");
        assertTrue(roads.has("roads") && roads.get("roads").isArray(), "roads array present");

        // Invariant: every way id appearing in a logical road must also be in the flat list.
        Set<Long> flat = new HashSet<>();
        ways.get("way_ids").forEach(n -> flat.add(n.asLong()));
        for (JsonNode road : roads.get("roads")) {
            assertTrue(road.has("name"));
            for (JsonNode wid : road.get("way_ids"))
                assertTrue(flat.contains(wid.asLong()),
                        "way id " + wid.asLong() + " in a logical road must be in qualifying_ways");
        }
    }
}
