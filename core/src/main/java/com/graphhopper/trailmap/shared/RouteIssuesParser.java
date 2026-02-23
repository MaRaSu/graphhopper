/*
 * Trailmap - RouteIssuesParser
 *
 * Detects route issues from OSM tags and sets boolean encoded values.
 * Implements typedIssueRules from route-profile-rules.ts.
 *
 * Issues detected:
 * - BIKING_BLOCKED: bicycle=no/private or access=no/private without bicycle override
 * - BIKING_BLOCKED_RISK: access tags suggest possible restriction (unknown, agricultural,
 *   forestry, delivery, service, permit) but routing is still allowed
 * - FOOT_BLOCKED: foot=no/private or access restriction without foot override
 * - NARROW: highway=path with width < 0.5m
 * - POOR_VISIBILITY: highway=path with trail_visibility=bad/horrible/no
 * - VEGETATION: obstacle=vegetation
 * - MUD: highway=path/track with surface=mud
 * - UNKNOWN_PATH: highway=path without mtb:scale and no good indicators
 * - UNKNOWN_TRACK: highway=track without mtb:scale/name and no good indicators
 * - FERRY: route=ferry
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.reader.ReaderWay;
import com.graphhopper.routing.ev.BooleanEncodedValue;
import com.graphhopper.routing.ev.EdgeIntAccess;
import com.graphhopper.routing.util.parsers.TagParser;
import com.graphhopper.storage.IntsRef;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

/**
 * Parser that detects route issues from OSM tags.
 * All issue flags are set in a single pass over the way tags.
 */
public class RouteIssuesParser implements TagParser {

    private final BooleanEncodedValue bikingBlockedEnc;
    private final BooleanEncodedValue bikingBlockedRiskEnc;
    private final BooleanEncodedValue footBlockedEnc;
    private final BooleanEncodedValue narrowEnc;
    private final BooleanEncodedValue poorVisibilityEnc;
    private final BooleanEncodedValue vegetationEnc;
    private final BooleanEncodedValue mudEnc;
    private final BooleanEncodedValue unknownPathEnc;
    private final BooleanEncodedValue unknownTrackEnc;
    private final BooleanEncodedValue ferryEnc;

    // Good surfaces that indicate a path/track is likely safe
    private static final Set<String> GOOD_SURFACES = new HashSet<>(
        Arrays.asList("paved", "unpaved", "fine_gravel", "compacted", "gravel", "sand")
    );

    // Good tracktypes that indicate quality
    private static final Set<String> GOOD_TRACKTYPES = new HashSet<>(
        Arrays.asList("grade1", "grade2", "grade3", "grade4")
    );

    // Bad visibility values
    private static final Set<String> BAD_VISIBILITY = new HashSet<>(
        Arrays.asList("bad", "horrible", "no")
    );

    // Access values that suggest possible restriction but don't fully block biking
    private static final Set<String> RISK_ACCESS_VALUES = new HashSet<>(
        Arrays.asList("unknown", "agricultural", "forestry", "delivery", "service", "permit")
    );

    // Allowed bicycle values that override access risk
    private static final Set<String> BICYCLE_OVERRIDES = new HashSet<>(
        Arrays.asList("yes", "designated", "official", "permissive", "destination")
    );

    public RouteIssuesParser(
            BooleanEncodedValue bikingBlockedEnc,
            BooleanEncodedValue bikingBlockedRiskEnc,
            BooleanEncodedValue footBlockedEnc,
            BooleanEncodedValue narrowEnc,
            BooleanEncodedValue poorVisibilityEnc,
            BooleanEncodedValue vegetationEnc,
            BooleanEncodedValue mudEnc,
            BooleanEncodedValue unknownPathEnc,
            BooleanEncodedValue unknownTrackEnc,
            BooleanEncodedValue ferryEnc) {
        this.bikingBlockedEnc = bikingBlockedEnc;
        this.bikingBlockedRiskEnc = bikingBlockedRiskEnc;
        this.footBlockedEnc = footBlockedEnc;
        this.narrowEnc = narrowEnc;
        this.poorVisibilityEnc = poorVisibilityEnc;
        this.vegetationEnc = vegetationEnc;
        this.mudEnc = mudEnc;
        this.unknownPathEnc = unknownPathEnc;
        this.unknownTrackEnc = unknownTrackEnc;
        this.ferryEnc = ferryEnc;
    }

    @Override
    public void handleWayTags(int edgeId, EdgeIntAccess edgeIntAccess,
                              ReaderWay way, IntsRef relationFlags) {
        // Set all issue flags in one pass
        bikingBlockedEnc.setBool(false, edgeId, edgeIntAccess, checkBikingBlocked(way));
        bikingBlockedRiskEnc.setBool(false, edgeId, edgeIntAccess, checkBikingBlockedRisk(way));
        footBlockedEnc.setBool(false, edgeId, edgeIntAccess, checkFootBlocked(way));
        narrowEnc.setBool(false, edgeId, edgeIntAccess, checkNarrow(way));
        poorVisibilityEnc.setBool(false, edgeId, edgeIntAccess, checkPoorVisibility(way));
        vegetationEnc.setBool(false, edgeId, edgeIntAccess, checkVegetation(way));
        mudEnc.setBool(false, edgeId, edgeIntAccess, checkMud(way));
        unknownPathEnc.setBool(false, edgeId, edgeIntAccess, checkUnknownPath(way));
        unknownTrackEnc.setBool(false, edgeId, edgeIntAccess, checkUnknownTrack(way));
        ferryEnc.setBool(false, edgeId, edgeIntAccess, checkFerry(way));
    }

    // =========================================================================
    // BIKING_NOT_PERMITTED
    // From TypeScript: BIKING_NOT_PERMITTED_RULE
    // =========================================================================
    private boolean checkBikingBlocked(ReaderWay way) {
        String bicycle = way.getTag("bicycle");

        // Case 1: Direct bicycle restriction
        if ("no".equals(bicycle) || "private".equals(bicycle)) {
            return true;
        }

        // Case 2: General access restriction without bicycle override
        String access = way.getTag("access");
        if ("no".equals(access) || "private".equals(access)) {
            // Check if bicycle explicitly allowed
            return !"yes".equals(bicycle) && !"permissive".equals(bicycle);
        }

        return false;
    }

    // =========================================================================
    // BIKING_BLOCKED_RISK
    // Access tags suggest possible restriction but routing is still allowed.
    // Triggers for: unknown, agricultural, forestry, delivery, service, permit
    // =========================================================================
    private boolean checkBikingBlockedRisk(ReaderWay way) {
        String bicycle = way.getTag("bicycle");

        // Direct bicycle risk tag
        if (bicycle != null && RISK_ACCESS_VALUES.contains(bicycle)) {
            return true;
        }

        // General access risk without bicycle override
        String access = way.getTag("access");
        if (access != null && RISK_ACCESS_VALUES.contains(access)) {
            return !BICYCLE_OVERRIDES.contains(bicycle);
        }

        return false;
    }

    // =========================================================================
    // FOOT_NOT_PERMITTED
    // From TypeScript: FOOT_NOT_PERMITTED_RULE
    // =========================================================================
    private boolean checkFootBlocked(ReaderWay way) {
        String foot = way.getTag("foot");

        // Case 1: Direct foot restriction
        if ("no".equals(foot) || "private".equals(foot)) {
            return true;
        }

        // Case 2: General access restriction without foot override
        String access = way.getTag("access");
        if ("no".equals(access) || "private".equals(access) || "permit".equals(access)) {
            // Check if foot explicitly allowed
            return !"yes".equals(foot) && !"permissive".equals(foot);
        }

        return false;
    }

    // =========================================================================
    // NARROW_PATH
    // From TypeScript: NARROW_PATH_RULE
    // Required: highway=path AND width < 0.5
    // =========================================================================
    private boolean checkNarrow(ReaderWay way) {
        if (!"path".equals(way.getTag("highway"))) {
            return false;
        }

        String width = way.getTag("width");
        if (width == null) {
            return false;
        }

        try {
            double w = parseWidth(width);
            return w < 0.5;
        } catch (NumberFormatException e) {
            return false;
        }
    }

    // =========================================================================
    // POOR_VISIBILITY
    // From TypeScript: POOR_VISIBILITY_RULE
    // Required: highway=path AND trail_visibility=bad/horrible/no
    // =========================================================================
    private boolean checkPoorVisibility(ReaderWay way) {
        if (!"path".equals(way.getTag("highway"))) {
            return false;
        }

        String visibility = way.getTag("trail_visibility");
        return visibility != null && BAD_VISIBILITY.contains(visibility);
    }

    // =========================================================================
    // VEGETATION
    // From TypeScript: typedIssueRules with VEGETATION pattern
    // Required: obstacle=vegetation
    // =========================================================================
    private boolean checkVegetation(ReaderWay way) {
        return "vegetation".equals(way.getTag("obstacle"));
    }

    // =========================================================================
    // MUD
    // From TypeScript: typedIssueRules with MUD_PATH_TRACK pattern
    // Required: highway=path/track AND surface=mud
    // =========================================================================
    private boolean checkMud(ReaderWay way) {
        String highway = way.getTag("highway");
        if (!"path".equals(highway) && !"track".equals(highway)) {
            return false;
        }
        return "mud".equals(way.getTag("surface"));
    }

    // =========================================================================
    // DIFFICULT_UNKNOWN_PATH
    // From TypeScript: DIFFICULT_UNKNOWN_PATH_RULE
    // Required: highway=path, absent mtb:scale
    // NoneOf: bicycle=designated/yes, good surface, any smoothness, good tracktype
    // =========================================================================
    private boolean checkUnknownPath(ReaderWay way) {
        if (!"path".equals(way.getTag("highway"))) {
            return false;
        }

        // Must not have mtb:scale tag
        if (way.hasTag("mtb:scale")) {
            return false;
        }

        // Check noneOf conditions (good indicators that make it safe)
        String bicycle = way.getTag("bicycle");
        if ("designated".equals(bicycle) || "yes".equals(bicycle)) {
            return false;
        }

        String surface = way.getTag("surface");
        if (surface != null && GOOD_SURFACES.contains(surface)) {
            return false;
        }

        // Any smoothness tag indicates some quality information
        if (way.hasTag("smoothness")) {
            return false;
        }

        String tracktype = way.getTag("tracktype");
        if (tracktype != null && GOOD_TRACKTYPES.contains(tracktype)) {
            return false;
        }

        // Path without quality indicators = unknown difficulty
        return true;
    }

    // =========================================================================
    // DIFFICULT_UNKNOWN_TRACK
    // From TypeScript: DIFFICULT_UNKNOWN_TRACK_RULE
    // Required: highway=track, absent mtb:scale AND absent name
    // NoneOf: bicycle=designated/yes, good surface, any smoothness, good tracktype
    // =========================================================================
    private boolean checkUnknownTrack(ReaderWay way) {
        if (!"track".equals(way.getTag("highway"))) {
            return false;
        }

        // Must not have mtb:scale tag
        if (way.hasTag("mtb:scale")) {
            return false;
        }

        // Must not have name (named tracks are usually maintained)
        if (way.hasTag("name")) {
            return false;
        }

        // Check noneOf conditions (good indicators that make it safe)
        String bicycle = way.getTag("bicycle");
        if ("designated".equals(bicycle) || "yes".equals(bicycle)) {
            return false;
        }

        String surface = way.getTag("surface");
        if (surface != null && GOOD_SURFACES.contains(surface)) {
            return false;
        }

        // Any smoothness tag indicates some quality information
        if (way.hasTag("smoothness")) {
            return false;
        }

        String tracktype = way.getTag("tracktype");
        if (tracktype != null && GOOD_TRACKTYPES.contains(tracktype)) {
            return false;
        }

        // Track without name and quality indicators = unknown difficulty
        return true;
    }

    // =========================================================================
    // FERRY_CROSSING
    // From TypeScript: typedIssueRules with FERRY_ROUTE pattern
    // Required: route=ferry
    // =========================================================================
    private boolean checkFerry(ReaderWay way) {
        return "ferry".equals(way.getTag("route"));
    }

    // =========================================================================
    // HELPER METHODS
    // =========================================================================

    /**
     * Parse width tag value to meters.
     * Handles formats: "0.5", "0.5 m", "50 cm", "0.5m", "50cm"
     */
    private double parseWidth(String width) {
        if (width == null) {
            throw new NumberFormatException("null width");
        }

        width = width.trim().toLowerCase();

        // Handle centimeters
        if (width.endsWith("cm")) {
            String numPart = width.substring(0, width.length() - 2).trim();
            return Double.parseDouble(numPart) / 100.0;
        }

        // Handle meters (with or without space)
        if (width.endsWith("m")) {
            String numPart = width.substring(0, width.length() - 1).trim();
            return Double.parseDouble(numPart);
        }

        // Assume meters if no unit
        return Double.parseDouble(width);
    }
}
