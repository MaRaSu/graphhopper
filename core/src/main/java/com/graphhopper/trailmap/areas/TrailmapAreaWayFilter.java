/*
 * Trailmap - Area Way Filter Implementation
 *
 * Determines which OSM area polygons should be included in the routing graph
 * based on configurable rules.
 */
package com.graphhopper.trailmap.areas;

import com.graphhopper.reader.ReaderWay;
import com.graphhopper.routing.util.AreaWayFilter;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Map;

/**
 * Filters OSM ways to identify routable area polygons.
 * <p>
 * An area is included in routing if:
 * 1. It is a closed polygon (first node == last node) - required
 * 2. It does not have any globally forbidden tags (buildings, water, etc.)
 * 3. It matches at least one passability rule
 * <p>
 * Note: The area=yes tag alone is NOT sufficient. The way must be a closed polygon.
 * This prevents spurious edges from invalid/linear ways with area=yes.
 */
public class TrailmapAreaWayFilter implements AreaWayFilter {
    private static final Logger LOGGER = LoggerFactory.getLogger(TrailmapAreaWayFilter.class);

    private final AreaRoutingRules rules;
    private long acceptedCount = 0;
    private long rejectedCount = 0;

    public TrailmapAreaWayFilter(AreaRoutingRules rules) {
        this.rules = rules;
    }

    @Override
    public boolean acceptAreaWay(ReaderWay way) {
        // Step 1: Must be a closed polygon
        if (!isClosedPolygon(way)) {
            return false;
        }

        // Step 2: Check global forbidden tags (water, buildings, etc.)
        if (hasGlobalForbiddenTag(way)) {
            rejectedCount++;
            return false;
        }

        // Step 3: Check each rule for a match
        for (AreaRoutingRules.Rule rule : rules.getRules()) {
            if (matchesRule(way, rule)) {
                acceptedCount++;
                if (LOGGER.isDebugEnabled()) {
                    LOGGER.debug("Way {} accepted as area type '{}': {}",
                            way.getId(), rule.getName(), summarizeTags(way));
                }
                return true;
            }
        }

        rejectedCount++;
        return false;
    }

    /**
     * Checks if the way is a closed polygon (first node == last node).
     * This is required for area routing - the area=yes tag alone is not sufficient.
     */
    private boolean isClosedPolygon(ReaderWay way) {
        var nodes = way.getNodes();
        if (nodes.size() < 4) {  // Need at least 4 nodes for a valid closed polygon (A-B-C-A)
            return false;
        }
        return nodes.get(0) == nodes.get(nodes.size() - 1);
    }

    /**
     * Checks if any global forbidden tag is present.
     */
    private boolean hasGlobalForbiddenTag(ReaderWay way) {
        for (AreaRoutingRules.TagMatcher matcher : rules.getGlobalForbiddenTags()) {
            String actualValue = way.getTag(matcher.getKey());
            if (matcher.matches(actualValue)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Checks if the way matches a specific rule.
     */
    private boolean matchesRule(ReaderWay way, AreaRoutingRules.Rule rule) {
        // Check area_check condition
        boolean hasExplicitArea = "yes".equals(way.getTag("area"));

        switch (rule.getAreaCheck()) {
            case "explicit":
                // Only accept if area=yes is present (and it's closed - already verified)
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
                // Accept any closed polygon (already verified in isClosedPolygon)
                break;
        }

        // Check required tags - ALL must be present with matching values
        for (Map.Entry<String, String> req : rule.getRequiredTags().entrySet()) {
            String tagKey = req.getKey();
            String requiredValue = req.getValue();
            String actualValue = way.getTag(tagKey);

            if (actualValue == null || !actualValue.equals(requiredValue)) {
                return false;
            }
        }

        // Check forbidden tags - NONE must match
        for (AreaRoutingRules.TagMatcher matcher : rule.getForbiddenTags()) {
            String actualValue = way.getTag(matcher.getKey());
            if (matcher.matches(actualValue)) {
                return false;
            }
        }

        return true;
    }

    /**
     * Creates a summary of key tags for logging.
     */
    private String summarizeTags(ReaderWay way) {
        StringBuilder sb = new StringBuilder();
        String[] interestingTags = {"amenity", "highway", "leisure", "tourism", "place", "area", "access"};
        for (String tag : interestingTags) {
            String value = way.getTag(tag);
            if (value != null) {
                if (sb.length() > 0) sb.append(", ");
                sb.append(tag).append("=").append(value);
            }
        }
        return sb.toString();
    }

    /**
     * Returns statistics about accepted/rejected areas.
     */
    public String getStatistics() {
        return String.format("AreaWayFilter: accepted=%d, rejected=%d", acceptedCount, rejectedCount);
    }

    public long getAcceptedCount() {
        return acceptedCount;
    }

    public long getRejectedCount() {
        return rejectedCount;
    }
}
