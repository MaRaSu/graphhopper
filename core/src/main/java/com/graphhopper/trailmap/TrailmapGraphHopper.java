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
import com.graphhopper.trailmap.areas.AreaAccessParser;
import com.graphhopper.trailmap.areas.AreaRoutingRules;
import com.graphhopper.trailmap.areas.TrailmapAreaWayFilter;
import com.graphhopper.util.PMap;
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
