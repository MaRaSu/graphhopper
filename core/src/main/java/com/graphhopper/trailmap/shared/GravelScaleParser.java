/*
 * Trailmap - GravelScaleParser
 *
 * Computes the GravelScale value for each edge based on OSM tags.
 * This is a full port of gravelScaleRules from route-profile-rules.ts.
 *
 * The rules are evaluated in order - first matching rule wins.
 * Each rule can have:
 * - required: TagPattern that MUST match
 * - anyOf: Array of TagPatterns where at least one MUST match
 * - noneOf: Array of TagPatterns where NONE must match (exclusions)
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.reader.ReaderWay;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.ev.EdgeIntAccess;
import com.graphhopper.routing.util.parsers.TagParser;
import com.graphhopper.storage.IntsRef;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

/**
 * Parser that computes GravelScale from OSM tags.
 * Implements the gravelScaleRules logic from route-profile-rules.ts.
 */
public class GravelScaleParser implements TagParser {

    private final EnumEncodedValue<GravelScale> gravelScaleEnc;

    public GravelScaleParser(EnumEncodedValue<GravelScale> gravelScaleEnc) {
        this.gravelScaleEnc = gravelScaleEnc;
    }

    @Override
    public void handleWayTags(int edgeId, EdgeIntAccess edgeIntAccess, ReaderWay way, IntsRef relationFlags) {
        GravelScale scale = computeGravelScale(way);
        gravelScaleEnc.setEnum(false, edgeId, edgeIntAccess, scale);
    }

    /**
     * Compute GravelScale from OSM tags.
     * Rules are evaluated in order - first match wins.
     */
    public GravelScale computeGravelScale(ReaderWay way) {
        // =================================================================
        // RULE 1: Ferry routes -> FERRY
        // =================================================================
        if (matchesFerryRoute(way)) {
            return GravelScale.FERRY;
        }

        // =================================================================
        // RULE 2: Asphalt surfaces -> ZERO_MINUS (paved)
        // anyOf: [ASPHALT, MOTORWAY, MAJOR_ROAD, SECONDARY_ROAD_NO_SURFACE, CYCLEWAY_PAVED]
        // noneOf: [UNPAVED_SURFACE]
        // =================================================================
        if (matchesAsphaltRule(way) && !matchesUnpavedSurface(way)) {
            return GravelScale.ZERO_MINUS;
        }

        // =================================================================
        // RULE 3: Very well-maintained unpaved -> ZERO
        // anyOf: [cycleway without surface, grade1 tracktype]
        // =================================================================
        if (matchesZeroRule(way)) {
            return GravelScale.ZERO;
        }

        // =================================================================
        // RULE 4: Good gravel conditions -> ZERO_PLUS
        // anyOf: [footway, fine_gravel surface, mtb:scale 0-, highway compacted,
        //         unpaved on tertiary/unclassified/residential, unpaved service,
        //         grade2 tracktype, mid smoothness, compacted surface,
        //         nordic track with mtb:scale 0]
        // noneOf: [horrible smoothness, risky mtb:scale, vegetation, mud, path, service]
        // =================================================================
        if (matchesZeroPlusRule(way)) {
            return GravelScale.ZERO_PLUS;
        }

        // =================================================================
        // RULE 5: Basic gravel, slightly bigger risk -> ONE
        // anyOf: [highway compacted, unpaved on tertiary/unclassified/residential,
        //         unpaved service, compacted surface]
        // noneOf: [horrible smoothness, vegetation, mud, path, service with gravel]
        // =================================================================
        if (matchesOneRuleBasic(way)) {
            return GravelScale.ONE;
        }

        // =================================================================
        // RULE 6: Service road with Finnish name ending in "tie" -> ONE
        // anyOf: [service highway with name ending in "tie"]
        // noneOf: [horrible smoothness, risky mtb:scale, vegetation, mud]
        // =================================================================
        if (matchesOneRuleFinnishService(way)) {
            return GravelScale.ONE;
        }

        // =================================================================
        // RULE 7: Standard gravel with medium risk -> ONE (risk TWO)
        // anyOf: [service, compacted, mtb:scale 0-, grade2, mid smoothness,
        //         track with mtb:scale 0, grade3, track unpaved without other tags]
        // noneOf: [horrible smoothness, risky mtb:scale, vegetation, mud, service gravel]
        // =================================================================
        if (matchesOneRuleWithRisk(way)) {
            return GravelScale.ONE;
        }

        // =================================================================
        // RULE 8: Rougher gravel -> TWO (risk THREE)
        // anyOf: [service, track/path/service with mtb:scale 0/0+/1, grade4, track unpaved]
        // noneOf: [horrible smoothness, big risk mtb:scale, vegetation, mud, visibility problem]
        // =================================================================
        if (matchesTwoRule(way)) {
            return GravelScale.TWO;
        }

        // =================================================================
        // RULE 9: Track with Finnish name ending in "tie" -> TWO (risk THREE)
        // anyOf: [track with name ending in "tie"]
        // noneOf: [horrible smoothness, risky mtb:scale, vegetation, mud]
        // =================================================================
        if (matchesTwoRuleFinnishTrack(way)) {
            return GravelScale.TWO;
        }

        // =================================================================
        // RULE 10: Difficult track -> THREE (risk FOUR)
        // required: track highway
        // anyOf: [gravel surface, no surface, horrible smoothness]
        // noneOf: [big risk mtb:scale, very horrible smoothness, vegetation, mud, visibility problem]
        // =================================================================
        if (matchesThreeRule(way)) {
            return GravelScale.THREE;
        }

        // =================================================================
        // RULE 11: Not rideable -> FOUR
        // anyOf: [path, big risk mtb:scale, very horrible smoothness,
        //         mud/sand/ground/dirt surface, vegetation]
        // =================================================================
        if (matchesFourRule(way)) {
            return GravelScale.FOUR;
        }

        // =================================================================
        // DEFAULT: Unknown
        // =================================================================
        return GravelScale.UNKNOWN;
    }

    // =====================================================================
    // TAG PATTERN MATCHERS
    // Each method implements the matching logic for a specific pattern
    // =====================================================================

    // --- FERRY_ROUTE pattern ---
    private boolean matchesFerryRoute(ReaderWay way) {
        return "ferry".equals(way.getTag("route"));
    }

    // --- ASPHALT pattern (includes motorway, major road, secondary no surface, cycleway paved) ---
    private boolean matchesAsphaltRule(ReaderWay way) {
        String surface = way.getTag("surface");
        String highway = way.getTag("highway");

        // ASPHALT: surface in [asphalt, paved, concrete, paving_stones]
        if (surface != null && isAsphaltSurface(surface)) {
            return true;
        }

        // MOTORWAY: highway in [motorway, motorway_link]
        if (highway != null && isMotorway(highway)) {
            return true;
        }

        // MAJOR_ROAD: highway in [trunk, trunk_link, primary, primary_link]
        if (highway != null && isMajorRoad(highway)) {
            return true;
        }

        // SECONDARY_ROAD_NO_SURFACE: highway in [secondary, secondary_link] AND no surface tag
        if (highway != null && isSecondaryRoad(highway) && surface == null) {
            return true;
        }

        // CYCLEWAY_PAVED: highway in [cycleway, footway] AND surface in [paved, asphalt]
        if (highway != null && isCyclewayOrFootway(highway) && surface != null && isPavedSurface(surface)) {
            return true;
        }

        return false;
    }

    // --- UNPAVED_SURFACE pattern ---
    private boolean matchesUnpavedSurface(ReaderWay way) {
        String surface = way.getTag("surface");
        return surface != null && isUnpavedSurface(surface);
    }

    // --- ZERO rule: well-maintained unpaved cycleways ---
    private boolean matchesZeroRule(ReaderWay way) {
        String highway = way.getTag("highway");
        String tracktype = way.getTag("tracktype");

        // cycleway without considering surface (any surface cycleway is at least ZERO)
        if ("cycleway".equals(highway)) {
            return true;
        }

        // FIRST_TRACKTYPE: grade1
        if ("grade1".equals(tracktype)) {
            return true;
        }

        return false;
    }

    // --- ZERO_PLUS rule ---
    private boolean matchesZeroPlusRule(ReaderWay way) {
        // First check exclusions (noneOf) EXCEPT isRiskService
        if (hasHorribleSmoothness(way) ||
            hasGravelBikeRiskMtbScale(way) ||
            hasVegetation(way) ||
            hasMud(way) ||
            isPath(way)) {
            return false;
        }

        String highway = way.getTag("highway");
        String surface = way.getTag("surface");

        // Special case: service road with compacted surface → ZERO_PLUS
        if ("service".equals(highway) && "compacted".equals(surface)) {
            return true;
        }

        // Now exclude remaining service roads
        if (isRiskService(way)) {
            return false;
        }
        String tracktype = way.getTag("tracktype");
        String smoothness = way.getTag("smoothness");
        String mtbScale = way.getTag("mtb:scale");
        String pisteType = way.getTag("piste:type");

        // footway
        if ("footway".equals(highway)) {
            return true;
        }

        // fine_gravel surface
        if ("fine_gravel".equals(surface)) {
            return true;
        }

        // MTB_SCALE_0MINUS: mtb:scale = 0-
        if ("0-".equals(mtbScale)) {
            return true;
        }

        // LIKELY_HIGHWAY_COMPACT: tertiary/unclassified/residential without surface
        if (isLikelyHighwayCompact(highway, surface)) {
            return true;
        }

        // LIKELY_UNPAVED_COMPACT: tertiary/unclassified/residential with unpaved/gravel/sand/mud/fine_gravel
        if (isLikelyUnpavedCompact(highway, surface)) {
            return true;
        }

        // LIKELY_UNPAVED_COMPACT2: service with unpaved surface
        if ("service".equals(highway) && "unpaved".equals(surface)) {
            return true;
        }

        // SECOND_TRACKTYPE: grade2
        if ("grade2".equals(tracktype)) {
            return true;
        }

        // MID_SMOOTHNESS: intermediate or bad
        if (isMidSmoothness(smoothness)) {
            return true;
        }

        // compacted surface
        if ("compacted".equals(surface)) {
            return true;
        }

        return false;
    }

    // --- ONE rule (basic) ---
    private boolean matchesOneRuleBasic(ReaderWay way) {
        // Check exclusions
        if (hasHorribleSmoothness(way) ||
            hasVegetation(way) ||
            hasMud(way) ||
            isPath(way) ||
            isRiskServiceGravel(way)) {
            return false;
        }

        String highway = way.getTag("highway");
        String surface = way.getTag("surface");

        // LIKELY_HIGHWAY_COMPACT
        if (isLikelyHighwayCompact(highway, surface)) {
            return true;
        }

        // LIKELY_UNPAVED_COMPACT
        if (isLikelyUnpavedCompact(highway, surface)) {
            return true;
        }

        // LIKELY_UNPAVED_COMPACT2: service with unpaved
        if ("service".equals(highway) && "unpaved".equals(surface)) {
            return true;
        }

        // compacted surface
        if ("compacted".equals(surface)) {
            return true;
        }

        return false;
    }

    // --- ONE rule (Finnish service road with name ending in "tie") ---
    private boolean matchesOneRuleFinnishService(ReaderWay way) {
        // Check exclusions
        if (hasHorribleSmoothness(way) ||
            hasGravelBikeRiskMtbScale(way) ||
            hasVegetation(way) ||
            hasMud(way)) {
            return false;
        }

        String highway = way.getTag("highway");
        String name = way.getTag("name");

        // service with Finnish name ending in "tie" (road)
        return "service".equals(highway) && name != null && name.endsWith("tie");
    }

    // --- ONE rule with risk TWO ---
    private boolean matchesOneRuleWithRisk(ReaderWay way) {
        // Check exclusions
        if (hasHorribleSmoothness(way) ||
            hasGravelBikeRiskMtbScale(way) ||
            hasVegetation(way) ||
            hasMud(way) ||
            isRiskServiceGravel(way)) {
            return false;
        }

        String highway = way.getTag("highway");
        String surface = way.getTag("surface");
        String tracktype = way.getTag("tracktype");
        String smoothness = way.getTag("smoothness");
        String mtbScale = way.getTag("mtb:scale");

        // service highway
        if ("service".equals(highway)) {
            return true;
        }

        // compacted surface
        if ("compacted".equals(surface)) {
            return true;
        }

        // MTB_SCALE_0MINUS
        if ("0-".equals(mtbScale)) {
            return true;
        }

        // SECOND_TRACKTYPE: grade2
        if ("grade2".equals(tracktype)) {
            return true;
        }

        // MID_SMOOTHNESS
        if (isMidSmoothness(smoothness)) {
            return true;
        }

        // TRACK_MTB_SCALE_0: track with mtb:scale 0
        if ("track".equals(highway) && "0".equals(mtbScale)) {
            return true;
        }

        // THIRD_TRACKTYPE: grade3
        if ("grade3".equals(tracktype)) {
            return true;
        }

        // TRACK_UNPAVED_NO_OTHER_TAGS: track with unpaved/compacted, no smoothness or mtb:scale
        if ("track".equals(highway) &&
            (surface != null && ("unpaved".equals(surface) || "compacted".equals(surface))) &&
            smoothness == null && mtbScale == null) {
            return true;
        }

        return false;
    }

    // --- TWO rule ---
    private boolean matchesTwoRule(ReaderWay way) {
        // Check exclusions
        if (hasHorribleSmoothness(way) ||
            hasGravelBigRiskMtbScale(way) ||
            hasVegetation(way) ||
            hasMud(way) ||
            hasVisibilityProblem(way)) {
            return false;
        }

        String highway = way.getTag("highway");
        String surface = way.getTag("surface");
        String tracktype = way.getTag("tracktype");
        String mtbScale = way.getTag("mtb:scale");

        // service highway
        if ("service".equals(highway)) {
            return true;
        }

        // track/path/service with mtb:scale 0, 0+, or 1
        if (highway != null && isTrackPathOrService(highway) &&
            mtbScale != null && isMtbScaleZeroToOne(mtbScale)) {
            return true;
        }

        // FOURTH_TRACKTYPE: grade4
        if ("grade4".equals(tracktype)) {
            return true;
        }

        // TRACK_UNPAVED: track with unpaved or compacted surface
        if ("track".equals(highway) &&
            surface != null && ("unpaved".equals(surface) || "compacted".equals(surface))) {
            return true;
        }

        return false;
    }

    // --- TWO rule (Finnish track with name ending in "tie") ---
    private boolean matchesTwoRuleFinnishTrack(ReaderWay way) {
        // Check exclusions
        if (hasHorribleSmoothness(way) ||
            hasGravelBikeRiskMtbScale(way) ||
            hasVegetation(way) ||
            hasMud(way)) {
            return false;
        }

        String highway = way.getTag("highway");
        String name = way.getTag("name");

        // track with Finnish name ending in "tie"
        return "track".equals(highway) && name != null && name.endsWith("tie");
    }

    // --- THREE rule ---
    private boolean matchesThreeRule(ReaderWay way) {
        String highway = way.getTag("highway");

        // required: track
        if (!"track".equals(highway)) {
            return false;
        }

        // Check exclusions
        if (hasGravelBigRiskMtbScale(way) ||
            hasVeryHorribleSmoothness(way) ||
            hasVegetation(way) ||
            hasMud(way) ||
            hasVisibilityProblem(way)) {
            return false;
        }

        String surface = way.getTag("surface");
        String smoothness = way.getTag("smoothness");

        // anyOf: gravel surface, no surface, or horrible smoothness
        if ("gravel".equals(surface)) {
            return true;
        }

        if (surface == null) {
            return true;
        }

        if ("horrible".equals(smoothness)) {
            return true;
        }

        return false;
    }

    // --- FOUR rule (not rideable) ---
    private boolean matchesFourRule(ReaderWay way) {
        String highway = way.getTag("highway");
        String surface = way.getTag("surface");
        String smoothness = way.getTag("smoothness");
        String mtbScale = way.getTag("mtb:scale");
        String obstacle = way.getTag("obstacle");

        // path
        if ("path".equals(highway)) {
            return true;
        }

        // GRAVEL_BIG_RISK_MTB_SCALE: mtb:scale 2, 3, 4, 5
        if (mtbScale != null && isGravelBigRiskMtbScale(mtbScale)) {
            return true;
        }

        // VERY_HORRIBLE_SMOOTHNESS
        if ("very_horrible".equals(smoothness)) {
            return true;
        }

        // mud, sand, ground, dirt surface
        if (surface != null && isDifficultSurface(surface)) {
            return true;
        }

        // vegetation obstacle
        if ("vegetation".equals(obstacle)) {
            return true;
        }

        return false;
    }

    // =====================================================================
    // HELPER METHODS FOR TAG VALUE CHECKS
    // =====================================================================

    private static final Set<String> ASPHALT_SURFACES = new HashSet<>(
        Arrays.asList("asphalt", "paved", "concrete", "paving_stones")
    );

    private static final Set<String> PAVED_SURFACES = new HashSet<>(
        Arrays.asList("paved", "asphalt")
    );

    private static final Set<String> UNPAVED_SURFACES = new HashSet<>(
        Arrays.asList("unpaved", "compacted", "gravel", "fine_gravel")
    );

    private static final Set<String> DIFFICULT_SURFACES = new HashSet<>(
        Arrays.asList("mud", "sand", "ground", "dirt")
    );

    private static final Set<String> LIKELY_COMPACT_HIGHWAYS = new HashSet<>(
        Arrays.asList("tertiary", "unclassified", "residential")
    );

    private static final Set<String> UNPAVED_COMPACT_SURFACES = new HashSet<>(
        Arrays.asList("unpaved", "gravel", "sand", "mud", "fine_gravel")
    );

    private boolean isAsphaltSurface(String surface) {
        return ASPHALT_SURFACES.contains(surface);
    }

    private boolean isPavedSurface(String surface) {
        return PAVED_SURFACES.contains(surface);
    }

    private boolean isUnpavedSurface(String surface) {
        return UNPAVED_SURFACES.contains(surface);
    }

    private boolean isDifficultSurface(String surface) {
        return DIFFICULT_SURFACES.contains(surface);
    }

    private boolean isMotorway(String highway) {
        return "motorway".equals(highway) || "motorway_link".equals(highway);
    }

    private boolean isMajorRoad(String highway) {
        return "trunk".equals(highway) || "trunk_link".equals(highway) ||
               "primary".equals(highway) || "primary_link".equals(highway);
    }

    private boolean isSecondaryRoad(String highway) {
        return "secondary".equals(highway) || "secondary_link".equals(highway);
    }

    private boolean isCyclewayOrFootway(String highway) {
        return "cycleway".equals(highway) || "footway".equals(highway);
    }

    private boolean isPath(ReaderWay way) {
        return "path".equals(way.getTag("highway"));
    }

    private boolean isRiskService(ReaderWay way) {
        return "service".equals(way.getTag("highway"));
    }

    private boolean isRiskServiceGravel(ReaderWay way) {
        return "service".equals(way.getTag("highway")) &&
               "gravel".equals(way.getTag("surface"));
    }

    private boolean isLikelyHighwayCompact(String highway, String surface) {
        return highway != null &&
               LIKELY_COMPACT_HIGHWAYS.contains(highway) &&
               surface == null;
    }

    private boolean isLikelyUnpavedCompact(String highway, String surface) {
        return highway != null &&
               LIKELY_COMPACT_HIGHWAYS.contains(highway) &&
               surface != null &&
               UNPAVED_COMPACT_SURFACES.contains(surface);
    }

    private boolean isMidSmoothness(String smoothness) {
        return "intermediate".equals(smoothness) || "bad".equals(smoothness);
    }

    private boolean isTrackPathOrService(String highway) {
        return "track".equals(highway) || "path".equals(highway) || "service".equals(highway);
    }

    private boolean isMtbScaleZeroToOne(String mtbScale) {
        return "0".equals(mtbScale) || "0+".equals(mtbScale) || "1".equals(mtbScale);
    }

    private boolean hasHorribleSmoothness(ReaderWay way) {
        String smoothness = way.getTag("smoothness");
        return "horrible".equals(smoothness) || "very_horrible".equals(smoothness);
    }

    private boolean hasVeryHorribleSmoothness(ReaderWay way) {
        return "very_horrible".equals(way.getTag("smoothness"));
    }

    private boolean hasGravelBikeRiskMtbScale(ReaderWay way) {
        String mtbScale = way.getTag("mtb:scale");
        if (mtbScale == null) return false;
        return "1".equals(mtbScale) || "1+".equals(mtbScale) ||
               "2".equals(mtbScale) || "3".equals(mtbScale) || "4".equals(mtbScale);
    }

    private boolean hasGravelBigRiskMtbScale(ReaderWay way) {
        String mtbScale = way.getTag("mtb:scale");
        return mtbScale != null && isGravelBigRiskMtbScale(mtbScale);
    }

    private boolean isGravelBigRiskMtbScale(String mtbScale) {
        return "2".equals(mtbScale) || "3".equals(mtbScale) ||
               "4".equals(mtbScale) || "5".equals(mtbScale);
    }

    private boolean hasVegetation(ReaderWay way) {
        return "vegetation".equals(way.getTag("obstacle"));
    }

    private boolean hasMud(ReaderWay way) {
        return "mud".equals(way.getTag("surface"));
    }

    private boolean hasVisibilityProblem(ReaderWay way) {
        String highway = way.getTag("highway");
        String visibility = way.getTag("trail_visibility");
        if (visibility == null) return false;
        if (!"path".equals(highway) && !"track".equals(highway)) return false;
        return "intermediate".equals(visibility) || "bad".equals(visibility) ||
               "horrible".equals(visibility) || "no".equals(visibility);
    }
}
