package com.graphhopper.trailmap.fixroute;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.PropertyNamingStrategies;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.graphhopper.GraphHopper;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.jackson.GraphHopperModule;
import com.graphhopper.jackson.Jackson;
import com.graphhopper.trailmap.TrailmapGraphHopper;
import com.graphhopper.trailmap.shared.TrailmapImportRegistry;
import com.graphhopper.util.CustomModel;

import java.io.File;
import java.io.FileInputStream;

/**
 * Opens the local graph cache READ-ONLY for {@code /fix_route} tests (never imports, never writes —
 * the cache belongs to the owner). Same setup as the {@code /convert_track} validation tests.
 */
final class FixRouteTestGraph {

    static final String GRAPH_LOCATION = "../../data/graph-cache";
    static final String OSM_FILE = "../../data/finland_4.osm.pbf";
    static final String CONFIG_FILE = "../trailmap-config.yml";

    private static final ObjectMapper GH_JSON = Jackson.newObjectMapper();

    private FixRouteTestGraph() {
    }

    static boolean available() {
        return new File(GRAPH_LOCATION, "properties").isFile();
    }

    static GraphHopper open() throws Exception {
        ObjectMapper yaml = new ObjectMapper(new YAMLFactory());
        yaml.setPropertyNamingStrategy(PropertyNamingStrategies.SNAKE_CASE);
        yaml.registerModule(new GraphHopperModule());
        JsonNode gh = yaml.readTree(new FileInputStream(CONFIG_FILE)).get("graphhopper");
        GraphHopperConfig config = yaml.treeToValue(gh, GraphHopperConfig.class);
        config.putObject("graph.location", GRAPH_LOCATION);
        config.putObject("datareader.file", OSM_FILE);
        GraphHopper hopper = new TrailmapGraphHopper();
        hopper.setImportRegistry(new TrailmapImportRegistry());
        hopper.setAllowWrites(false);
        hopper.init(config);
        if (!hopper.load()) throw new IllegalStateException("graph cache could not be loaded (never imported by tests)");
        return hopper;
    }

    /** Client custom model JSON → GH {@link CustomModel}, via GH's own Jackson setup. */
    static CustomModel customModel(JsonNode json) {
        if (json == null || json.isNull()) return null;
        try {
            return GH_JSON.treeToValue(json, CustomModel.class);
        } catch (Exception e) {
            throw new IllegalArgumentException("bad custom_model: " + e.getMessage(), e);
        }
    }
}
