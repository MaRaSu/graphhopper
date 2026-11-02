/*
 * Trailmap - Extended GraphHopper with area routing support
 *
 * Extends GraphHopper to provide Trailmap-specific functionality:
 * - Area routing through parking lots, plazas, etc.
 */
package com.graphhopper.trailmap;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.graphhopper.GraphHopper;
import com.graphhopper.routing.OSMReaderConfig;
import com.graphhopper.routing.ev.BooleanEncodedValue;
import com.graphhopper.routing.ev.ImportUnit;
import com.graphhopper.routing.ev.VehicleAccess;
import com.graphhopper.routing.util.AreaWayFilter;
import com.graphhopper.routing.util.OSMParsers;
import com.graphhopper.routing.util.parsers.TagParser;
import com.graphhopper.trailmap.areas.AreaAccessParser;
import com.graphhopper.trailmap.areas.AreaRoutingRules;
import com.graphhopper.trailmap.areas.TrailmapAreaWayFilter;
import com.graphhopper.trailmap.mtk.MtkWayAttrsParser;
import com.graphhopper.trailmap.shared.TrailmapPathDetailsBuilderFactory;
import com.graphhopper.util.PMap;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.config.Profile;
import com.graphhopper.reader.osm.OSMReader;
import com.graphhopper.routing.Router;
import com.graphhopper.routing.RouterConfig;
import com.graphhopper.routing.WeightingFactory;
import com.graphhopper.routing.ev.DefaultImportRegistry;
import com.graphhopper.routing.ev.ImportRegistry;
import com.graphhopper.routing.lm.LandmarkStorage;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.RoutingCHGraph;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.trailmap.barrier.*;
import com.graphhopper.util.TranslationMap;
import com.graphhopper.util.details.PathDetailsBuilderFactory;
import java.util.ArrayList;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.util.List;
import java.util.Map;

/**
 * Extended GraphHopper with Trailmap-specific features.
 * <p>
 * Use this class instead of GraphHopper to enable:
 * - Area routing through parking lots, pedestrian plazas, parks, etc.
 * - Trailmap path details (predicted_highway projected to its client-facing value)
 * <p>
 * Configuration:
 * <pre>
 * import.osm.area_routing.enabled: true
 * import.osm.area_routing.rules_file: area_routing_rules.json
 * </pre>
 */
public class TrailmapGraphHopper extends GraphHopper {
    private static final Logger LOGGER = LoggerFactory.getLogger(TrailmapGraphHopper.class);

    private TrailmapAreaWayFilter areaWayFilter;
    private AreaRoutingRules areaRoutingRules;

    public TrailmapGraphHopper() {
        // predicted_highway must cross the wire as its coarse, client-facing value; the
        // stock factory would emit internal-only refinements (e.g. SERVICE_DRIVEWAY).
        setPathDetailsBuilderFactory(new TrailmapPathDetailsBuilderFactory());
        setImportRegistry(new DefaultImportRegistry());
    }

    // ---- W26 barrier classes (trailmap/barrier, docs/gh_barrier_gate_implementation.md) ----

    private final BarrierTable barrierTable = new BarrierTable();
    private int barrierMinNetworkSize = 200, barrierSubnetworkThreads = 1;

    @Override
    public GraphHopper init(GraphHopperConfig ghConfig) {
        super.init(ghConfig);
        barrierMinNetworkSize = ghConfig.getInt("prepare.min_network_size", barrierMinNetworkSize);
        barrierSubnetworkThreads = ghConfig.getInt("prepare.subnetworks.threads", barrierSubnetworkThreads);
        return this;
    }

    /** bike_barrier / foot_barrier come from the barrier registry, everything else from the given one. */
    @Override
    public GraphHopper setImportRegistry(ImportRegistry importRegistry) {
        return super.setImportRegistry(importRegistry instanceof BarrierImportRegistry ? importRegistry
                : new BarrierImportRegistry(importRegistry, barrierTable));
    }

    @Override
    protected OSMReader createOSMReader(BaseGraph baseGraph, OSMParsers osmParsers, OSMReaderConfig osmReaderConfig) {
        return new TrailmapOSMReader(baseGraph, osmParsers, osmReaderConfig);
    }

    /** Adds {@code <profile>_subnetwork_b} for every profile; the stock {@code <profile>_subnetwork} is unchanged. */
    @Override
    protected List<BooleanEncodedValue> createSubnetworkEncodedValues() {
        List<BooleanEncodedValue> evs = new ArrayList<>(super.createSubnetworkEncodedValues());
        for (Profile profile : getProfiles())
            evs.add(BarrierProfiles.createSubnetworkEV(profile.getName()));
        return evs;
    }

    @Override
    protected void cleanUp() {
        super.cleanUp();
        BarrierSubnetworks.prepare(this, barrierMinNetworkSize, barrierSubnetworkThreads);
        if (getEncodingManager().hasEncodedValue(BarrierClass.KEY_BIKE) || getEncodingManager().hasEncodedValue(BarrierClass.KEY_FOOT)) {
            try {
                barrierTable.write(new File(getGraphHopperLocation()));
                loadedBarrierTable = barrierTable; // the table just built: never re-read after an import
                LOGGER.info("barrier table: {} barriers with a class other than none", barrierTable.size());
            } catch (IOException e) {
                throw new IllegalStateException("cannot write the barrier table", e);
            }
        }
    }

    @Override
    protected Router doCreateRouter(BaseGraph baseGraph, EncodingManager encodingManager, LocationIndex locationIndex,
                                    Map<String, Profile> profilesByName, PathDetailsBuilderFactory pathBuilderFactory,
                                    TranslationMap trMap, RouterConfig routerConfig, WeightingFactory weightingFactory,
                                    Map<String, RoutingCHGraph> chGraphs, Map<String, LandmarkStorage> landmarks) {
        return new BarrierRouter(baseGraph, encodingManager, locationIndex, profilesByName, pathBuilderFactory, trMap,
                routerConfig, weightingFactory, chGraphs, landmarks, loadedBarrierTable());
    }

    private volatile BarrierTable loadedBarrierTable;

    /**
     * Reads the barrier table once, when the graph loads. A malformed or truncated trailmap_barriers.json fails the load,
     * so the server doesn't start, instead of every request failing later (a rollout's health check then keeps the old
     * pod serving). A missing file is an empty table (a graph imported without barrier classes).
     */
    @Override
    public boolean load() {
        boolean loaded = super.load();
        if (loaded)
            loadedBarrierTable = BarrierTable.read(new File(getGraphHopperLocation()));
        return loaded;
    }

    /** The graph's barrier table, the same one every router of this instance uses; empty without barrier classes. */
    public BarrierTable barrierTable() {
        return loadedBarrierTable();
    }

    /** The graph's barrier table, read once (a router is created for every request). */
    private BarrierTable loadedBarrierTable() {
        BarrierTable t = loadedBarrierTable;
        if (t == null) {
            synchronized (this) {
                if (loadedBarrierTable == null)
                    loadedBarrierTable = BarrierTable.read(new File(getGraphHopperLocation()));
                t = loadedBarrierTable;
            }
        }
        return t;
    }

    @Override
    protected AreaWayFilter createAreaWayFilter(OSMReaderConfig config) {
        if (!config.isAreaRoutingEnabled()) {
            LOGGER.info("Area routing is disabled");
            return AreaWayFilter.NONE;
        }

        areaRoutingRules = loadAreaRoutingRules(config.getAreaRoutingRulesFile());
        areaWayFilter = new TrailmapAreaWayFilter(areaRoutingRules);

        LOGGER.info("Area routing enabled with {} rules", areaRoutingRules.getRules().size());
        for (AreaRoutingRules.Rule rule : areaRoutingRules.getRules()) {
            LOGGER.debug("  - {}: {}", rule.getName(), rule.getDescription());
        }

        return areaWayFilter;
    }

    @Override
    protected OSMParsers buildOSMParsers(Map<String, PMap> encodedValuesWithProps,
                                         Map<String, ImportUnit> activeImportUnits,
                                         Map<String, List<String>> restrictionVehicleTypesByProfile,
                                         List<String> ignoredHighways,
                                         List<String> trailmapExtraWays,
                                         OSMReaderConfig osmReaderConfig) {
        // Call parent to build standard parsers
        OSMParsers osmParsers = super.buildOSMParsers(encodedValuesWithProps, activeImportUnits,
                restrictionVehicleTypesByProfile, ignoredHighways, trailmapExtraWays, osmReaderConfig);

        // Add AreaAccessParser if area routing is enabled
        if (osmReaderConfig.isAreaRoutingEnabled() && areaRoutingRules != null) {
            addAreaAccessParser(osmParsers);
        }

        return osmParsers;
    }

    @Override
    protected void postImportOSM() {
        // the MTK overlay keeps per-import counters (applied, stale, hazard skips); log them once
        for (TagParser parser : getOSMParsers().getWayTagParsers())
            if (parser instanceof MtkWayAttrsParser)
                ((MtkWayAttrsParser) parser).logSummary();
        super.postImportOSM();
    }

    /**
     * Adds the AreaAccessParser to set access flags for area edges.
     */
    private void addAreaAccessParser(OSMParsers osmParsers) {
        // Get bike_access and foot_access encoded values if they exist
        BooleanEncodedValue bikeAccessEnc = null;
        BooleanEncodedValue footAccessEnc = null;

        try {
            bikeAccessEnc = getEncodingManager().getBooleanEncodedValue(VehicleAccess.key("bike"));
        } catch (IllegalArgumentException e) {
            LOGGER.debug("bike_access not configured, area edges won't have bike access set");
        }

        try {
            footAccessEnc = getEncodingManager().getBooleanEncodedValue(VehicleAccess.key("foot"));
        } catch (IllegalArgumentException e) {
            LOGGER.debug("foot_access not configured, area edges won't have foot access set");
        }

        if (bikeAccessEnc != null || footAccessEnc != null) {
            AreaAccessParser areaAccessParser = new AreaAccessParser(bikeAccessEnc, footAccessEnc, areaRoutingRules);
            osmParsers.addWayTagParser(areaAccessParser);
            LOGGER.info("Added AreaAccessParser for area edge access (bike={}, foot={})",
                    bikeAccessEnc != null, footAccessEnc != null);
        } else {
            LOGGER.warn("Neither bike_access nor foot_access configured, AreaAccessParser not added");
        }
    }

    /**
     * Loads area routing rules from file or uses defaults.
     */
    private AreaRoutingRules loadAreaRoutingRules(String rulesFile) {
        if (rulesFile == null || rulesFile.isEmpty()) {
            LOGGER.info("No area routing rules file specified, using defaults");
            return AreaRoutingRules.createDefault();
        }

        // Try loading from file path
        File file = new File(rulesFile);
        if (file.exists()) {
            try {
                ObjectMapper mapper = new ObjectMapper();
                AreaRoutingRules rules = mapper.readValue(file, AreaRoutingRules.class);
                LOGGER.info("Loaded area routing rules from: {}", rulesFile);
                return rules;
            } catch (IOException e) {
                LOGGER.warn("Failed to load area routing rules from {}: {}, using defaults",
                        rulesFile, e.getMessage());
                return AreaRoutingRules.createDefault();
            }
        }

        // Try loading from classpath
        try (InputStream is = getClass().getClassLoader().getResourceAsStream(rulesFile)) {
            if (is != null) {
                ObjectMapper mapper = new ObjectMapper();
                AreaRoutingRules rules = mapper.readValue(is, AreaRoutingRules.class);
                LOGGER.info("Loaded area routing rules from classpath: {}", rulesFile);
                return rules;
            }
        } catch (IOException e) {
            LOGGER.warn("Failed to load area routing rules from classpath {}: {}",
                    rulesFile, e.getMessage());
        }

        LOGGER.warn("Area routing rules file not found: {}, using defaults", rulesFile);
        return AreaRoutingRules.createDefault();
    }

    /**
     * Returns the area way filter for statistics/debugging.
     */
    public TrailmapAreaWayFilter getAreaWayFilter() {
        return areaWayFilter;
    }

    /**
     * Returns the area routing rules.
     */
    public AreaRoutingRules getAreaRoutingRules() {
        return areaRoutingRules;
    }

    /**
     * Logs area routing statistics after import.
     */
    public void logAreaRoutingStats() {
        if (areaWayFilter != null) {
            LOGGER.info(areaWayFilter.getStatistics());
        }
    }
}
