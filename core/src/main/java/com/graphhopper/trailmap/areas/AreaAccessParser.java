/*
 * Trailmap - Area Access Parser
 *
 * Sets bike_access and foot_access flags for area edges that were accepted
 * by the AreaWayFilter but skipped by standard access parsers (which require
 * highway tags).
 */
package com.graphhopper.trailmap.areas;

import com.graphhopper.reader.ReaderWay;
import com.graphhopper.routing.ev.BooleanEncodedValue;
import com.graphhopper.routing.ev.EdgeIntAccess;
import com.graphhopper.routing.util.parsers.TagParser;
import com.graphhopper.storage.IntsRef;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * Sets access flags for area edges.
 * <p>
 * Standard access parsers (BikeCommonAccessParser, FootAccessParser) skip ways
 * without highway tags, leaving access flags at false. This parser fills in
 * access flags for area edges based on the area routing rules.
 * <p>
 * This parser respects explicit OSM access restrictions (access=no, bicycle=no,
 * foot=no, etc.) and will not override them.
 * <p>
 * This parser should run AFTER the standard access parsers.
 */
public class AreaAccessParser implements TagParser {
    private static final Logger LOGGER = LoggerFactory.getLogger(AreaAccessParser.class);

    // Values that indicate access is denied
    private static final Set<String> RESTRICTED_VALUES = new HashSet<>(Arrays.asList(
            "no", "private", "restricted", "military", "emergency", "agricultural", "forestry", "delivery"
    ));

    private final BooleanEncodedValue bikeAccessEnc;
    private final BooleanEncodedValue footAccessEnc;
    private final AreaRoutingRules rules;

    /**
     * Creates an AreaAccessParser.
     *
     * @param bikeAccessEnc the bike_access encoded value (may be null if not configured)
     * @param footAccessEnc the foot_access encoded value (may be null if not configured)
     * @param rules the area routing rules
     */
    public AreaAccessParser(BooleanEncodedValue bikeAccessEnc, BooleanEncodedValue footAccessEnc, AreaRoutingRules rules) {
        this.bikeAccessEnc = bikeAccessEnc;
        this.footAccessEnc = footAccessEnc;
        this.rules = rules;
    }

    @Override
    public void handleWayTags(int edgeId, EdgeIntAccess edgeIntAccess, ReaderWay way, IntsRef relationFlags) {
        // Skip if way has a highway tag - standard parsers handle those
        if (way.getTag("highway") != null) {
            return;
        }

        // Check if this is a closed polygon area
        if (!isClosedPolygonArea(way)) {
            return;
        }

        // Check global forbidden tags
        if (hasGlobalForbiddenTag(way)) {
            return;
        }

        // Find matching rule and apply access
        for (AreaRoutingRules.Rule rule : rules.getRules()) {
            if (matchesRule(way, rule)) {
                applyAccess(edgeId, edgeIntAccess, way, rule);
                if (LOGGER.isDebugEnabled()) {
                    LOGGER.debug("Set access for area edge {} (way {}), rule '{}': bike={}, foot={}",
                            edgeId, way.getId(), rule.getName(),
                            rule.getAccess().getOrDefault("bike", false),
                            rule.getAccess().getOrDefault("foot", false));
                }
                return;
            }
        }
    }

    /**
     * Checks if the way is a closed polygon that should be treated as an area.
     * Requires the way to be a closed polygon (first node == last node).
     * The area=yes tag alone is not sufficient for non-closed ways.
     */
    private boolean isClosedPolygonArea(ReaderWay way) {
        var nodes = way.getNodes();
        if (nodes.size() < 4) {  // Need at least 4 nodes for a valid closed polygon (A-B-C-A)
            return false;
        }

        // Must be a closed polygon
        boolean isClosed = nodes.get(0) == nodes.get(nodes.size() - 1);
        if (!isClosed) {
            return false;
        }

        // Either has explicit area=yes tag, or is a closed way with area-indicating tags
        // (the tag matching is done in matchesRule, here we just confirm it's closed)
        return true;
    }

    private boolean hasGlobalForbiddenTag(ReaderWay way) {
        for (AreaRoutingRules.TagMatcher matcher : rules.getGlobalForbiddenTags()) {
            String actualValue = way.getTag(matcher.getKey());
            if (matcher.matches(actualValue)) {
                return true;
            }
        }
        return false;
    }

    private boolean matchesRule(ReaderWay way, AreaRoutingRules.Rule rule) {
        // Check area_check condition
        boolean hasExplicitArea = "yes".equals(way.getTag("area"));

        switch (rule.getAreaCheck()) {
            case "explicit":
                // Only accept if area=yes is present
                if (!hasExplicitArea) {
                    return false;
                }
                break;
            case "closed_only":
                // Only accept closed polygons without area=yes
                if (hasExplicitArea) {
                    return false;
                }
                break;
            case "explicit_or_closed":
            default:
                // Accept closed polygons (already verified in isClosedPolygonArea)
                break;
        }

        // Check required tags
        for (Map.Entry<String, String> req : rule.getRequiredTags().entrySet()) {
            String actualValue = way.getTag(req.getKey());
            if (actualValue == null || !actualValue.equals(req.getValue())) {
                return false;
            }
        }

        // Check forbidden tags
        for (AreaRoutingRules.TagMatcher matcher : rule.getForbiddenTags()) {
            String actualValue = way.getTag(matcher.getKey());
            if (matcher.matches(actualValue)) {
                return false;
            }
        }

        return true;
    }

    /**
     * Applies access flags based on rule configuration, but respects explicit OSM restrictions.
     */
    private void applyAccess(int edgeId, EdgeIntAccess edgeIntAccess, ReaderWay way, AreaRoutingRules.Rule rule) {
        Map<String, Boolean> access = rule.getAccess();

        // Set bike access if allowed by rule AND not explicitly restricted in OSM
        if (bikeAccessEnc != null && access.getOrDefault("bike", false)) {
            if (!isBikeAccessRestricted(way)) {
                bikeAccessEnc.setBool(false, edgeId, edgeIntAccess, true);
                bikeAccessEnc.setBool(true, edgeId, edgeIntAccess, true);
            } else {
                LOGGER.debug("Bike access restricted by OSM tags on way {}", way.getId());
            }
        }

        // Set foot access if allowed by rule AND not explicitly restricted in OSM
        if (footAccessEnc != null && access.getOrDefault("foot", false)) {
            if (!isFootAccessRestricted(way)) {
                footAccessEnc.setBool(false, edgeId, edgeIntAccess, true);
                footAccessEnc.setBool(true, edgeId, edgeIntAccess, true);
            } else {
                LOGGER.debug("Foot access restricted by OSM tags on way {}", way.getId());
            }
        }
    }

    /**
     * Checks if bike access is explicitly restricted by OSM tags.
     */
    private boolean isBikeAccessRestricted(ReaderWay way) {
        // Check bicycle-specific tag first
        String bicycle = way.getTag("bicycle");
        if (bicycle != null && RESTRICTED_VALUES.contains(bicycle)) {
            return true;
        }

        // Check vehicle tag (applies to bikes)
        String vehicle = way.getTag("vehicle");
        if (vehicle != null && RESTRICTED_VALUES.contains(vehicle)) {
            // But bicycle tag can override vehicle
            if (bicycle == null) {
                return true;
            }
        }

        // Check general access tag
        String access = way.getTag("access");
        if (access != null && RESTRICTED_VALUES.contains(access)) {
            // But more specific tags can override
            if (bicycle == null && vehicle == null) {
                return true;
            }
        }

        return false;
    }

    /**
     * Checks if foot access is explicitly restricted by OSM tags.
     */
    private boolean isFootAccessRestricted(ReaderWay way) {
        // Check foot-specific tag first
        String foot = way.getTag("foot");
        if (foot != null && RESTRICTED_VALUES.contains(foot)) {
            return true;
        }

        // Check general access tag
        String access = way.getTag("access");
        if (access != null && RESTRICTED_VALUES.contains(access)) {
            // But foot tag can override access
            if (foot == null) {
                return true;
            }
        }

        return false;
    }
}
