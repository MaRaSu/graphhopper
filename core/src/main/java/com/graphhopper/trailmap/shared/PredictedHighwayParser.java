/*
 * Trailmap - PredictedHighwayParser
 *
 * Full port of predictedHighwayRules from route-profile-rules.ts (lines 488-684).
 * Profile-independent - used by all routing profiles for UI display and analysis.
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
 * Parser that computes PredictedHighway from OSM tags.
 * Implements the predictedHighwayRules logic from route-profile-rules.ts.
 */
public class PredictedHighwayParser implements TagParser {

    private final EnumEncodedValue<PredictedHighway> predictedHighwayEnc;

    public PredictedHighwayParser(EnumEncodedValue<PredictedHighway> predictedHighwayEnc) {
        this.predictedHighwayEnc = predictedHighwayEnc;
    }

    @Override
    public void handleWayTags(int edgeId, EdgeIntAccess edgeIntAccess, ReaderWay way, IntsRef relationFlags) {
        PredictedHighway highway = computePredictedHighway(way);
        predictedHighwayEnc.setEnum(false, edgeId, edgeIntAccess, highway);
    }

    /**
     * Single definition of "this way is a driveway": {@code highway=service} +
     * {@code service=driveway}. Other service values (parking_aisle, alley,
     * drive-through, ...) are not driveways.
     *
     * <p>Shared so the routing category (SERVICE_DRIVEWAY, RULE 6 below) and the
     * client-facing issue flag ({@code issue_driveway}, set by RouteIssuesParser) can
     * never drift apart — the two run as independent tag parsers and are never
     * compared at runtime.</p>
     */
    public static boolean isDriveway(ReaderWay way) {
        return "service".equals(way.getTag("highway"))
                && "driveway".equals(way.getTag("service"));
    }

    /**
     * Compute PredictedHighway from OSM tags.
     * Full port of predictedHighwayRules from route-profile-rules.ts
     */
    public PredictedHighway computePredictedHighway(ReaderWay way) {
        String route = way.getTag("route");
        String highway = way.getTag("highway");

        // =================================================================
        // RULE 1: Ferry -> FERRY
        // =================================================================
        if ("ferry".equals(route)) {
            return PredictedHighway.FERRY;
        }

        // =================================================================
        // RULE 2: Motorway -> MOTORWAY
        // =================================================================
        if (highway != null && MOTORWAY_HIGHWAYS.contains(highway)) {
            return PredictedHighway.MOTORWAY;
        }

        // =================================================================
        // RULE 3: Major road -> MAJOR_ROAD
        // =================================================================
        if (highway != null && MAJOR_ROAD_HIGHWAYS.contains(highway)) {
            return PredictedHighway.MAJOR_ROAD;
        }

        // =================================================================
        // RULE 4: Secondary/tertiary roads -> MINOR_ROAD
        // =================================================================
        if (highway != null && SECONDARY_TERTIARY_HIGHWAYS.contains(highway)) {
            return PredictedHighway.MINOR_ROAD;
        }

        // =================================================================
        // RULE 5: Residential/unclassified -> MINOR_ROAD
        // =================================================================
        if (highway != null && RESIDENTIAL_UNCLASSIFIED_HIGHWAYS.contains(highway)) {
            return PredictedHighway.MINOR_ROAD;
        }

        // =================================================================
        // RULE 6: Service roads -> SERVICE_ROAD, driveways split out as an
        // internal-only refinement (highway=service + service=driveway).
        // SERVICE_DRIVEWAY lets routing profiles weight driveways separately; it
        // projects back to SERVICE_ROAD for clients and TbT classification via
        // PredictedHighway.toExternal().
        // =================================================================
        if ("service".equals(highway)) {
            if (isDriveway(way)) {
                return PredictedHighway.SERVICE_DRIVEWAY;
            }
            return PredictedHighway.SERVICE_ROAD;
        }

        // =================================================================
        // RULE 7: Cycleways and designated cycling -> CYCLEWAY
        // anyOf: [highway=cycleway, CYCLEWAY_AS_FOOTWAY, BICYCLE_DESIGNATED]
        // =================================================================
        if (matchesCyclewayRule(way)) {
            return PredictedHighway.CYCLEWAY;
        }

        // =================================================================
        // RULE 7.5: Bare footways -> FOOTWAY
        // required: highway=footway
        // noneOf: [bicycle in designated, yes, permissive, official]
        // =================================================================
        if (matchesBareFootwayRule(way)) {
            return PredictedHighway.FOOTWAY;
        }

        // =================================================================
        // RULE 8: Nordic outdoor ways (track) -> OUTDOOR_WAY
        // required: NORDIC_TRACK
        // anyOf: [GOOD_SURFACE, GOOD_TRACKTYPE, FOURTH_TRACKTYPE, GOOD_SMOOTHNESS,
        //         OK_SMOOTHNESS, GOOD_MTB_SCALE, LIKELY_GOOD_SURFACE, BICYCLE_OK]
        // =================================================================
        if (matchesNordicTrackOutdoorWayRule(way)) {
            return PredictedHighway.OUTDOOR_WAY;
        }

        // =================================================================
        // RULE 9: Nordic outdoor ways (path) -> OUTDOOR_WAY
        // required: NORDIC_PATH (highway=path, width>2, piste:type=nordic)
        // anyOf: [GOOD_SURFACE, GOOD_TRACKTYPE, FOURTH_TRACKTYPE, GOOD_SMOOTHNESS,
        //         OK_SMOOTHNESS, GOOD_MTB_SCALE, LIKELY_GOOD_SURFACE, BICYCLE_OK]
        // =================================================================
        if (matchesNordicPathOutdoorWayRule(way)) {
            return PredictedHighway.OUTDOOR_WAY;
        }

        // =================================================================
        // RULE 10: Paths with good conditions -> OUTDOOR_PATH
        // required: highway=path
        // anyOf: [GOOD_SURFACE, GOOD_TRACKTYPE, FOURTH_TRACKTYPE, GOOD_SMOOTHNESS,
        //         OK_SMOOTHNESS, BICYCLE_DESIGNATED]
        // =================================================================
        if (matchesOutdoorPathRule(way)) {
            return PredictedHighway.OUTDOOR_PATH;
        }

        // =================================================================
        // RULE 11: Good tracks -> GOOD_TRACK
        // required: highway=track
        // anyOf: [GOOD_SURFACE, GOOD_TRACKTYPE, FOURTH_TRACKTYPE, GOOD_SMOOTHNESS,
        //         OK_SMOOTHNESS, GOOD_MTB_SCALE, LIKELY_GOOD_SURFACE, BICYCLE_OK]
        // =================================================================
        if (matchesGoodTrackRule(way)) {
            return PredictedHighway.GOOD_TRACK;
        }

        // =================================================================
        // RULE 12: Track-like paths with good conditions -> GOOD_TRACK (risk ROUGH_TRACK)
        // required: TRACK_LIKE_PATH (highway=path, width>2)
        // anyOf: [GOOD_SURFACE, GOOD_TRACKTYPE, FOURTH_TRACKTYPE, GOOD_SMOOTHNESS,
        //         OK_SMOOTHNESS, GOOD_MTB_SCALE, LIKELY_GOOD_SURFACE, BICYCLE_OK]
        // =================================================================
        if (matchesTrackLikePathGoodRule(way)) {
            return PredictedHighway.GOOD_TRACK;
        }

        // =================================================================
        // RULE 13: Unknown tracks -> ROUGH_TRACK
        // required: highway=track (with low predictive power)
        // =================================================================
        if ("track".equals(highway)) {
            return PredictedHighway.ROUGH_TRACK;
        }

        // =================================================================
        // RULE 14: Paths -> PATH
        // required: highway=path
        // =================================================================
        if ("path".equals(highway)) {
            return PredictedHighway.PATH;
        }

        // =================================================================
        // DEFAULT: Unknown
        // =================================================================
        return PredictedHighway.UNKNOWN;
    }

    // =====================================================================
    // RULE MATCHERS - Full port from route-profile-rules.ts
    // =====================================================================

    // --- CYCLEWAY rule ---
    // anyOf: [highway=cycleway, CYCLEWAY_AS_FOOTWAY, BICYCLE_DESIGNATED]
    private boolean matchesCyclewayRule(ReaderWay way) {
        String highway = way.getTag("highway");
        String bicycle = way.getTag("bicycle");

        // highway=cycleway
        if ("cycleway".equals(highway)) {
            return true;
        }

        // CYCLEWAY_AS_FOOTWAY: highway=footway AND bicycle in [designated, yes]
        if ("footway".equals(highway) && bicycle != null && BICYCLE_DESIGNATED_YES.contains(bicycle)) {
            return true;
        }

        // BICYCLE_DESIGNATED: bicycle=designated
        if ("designated".equals(bicycle)) {
            return true;
        }

        return false;
    }

    // --- Bare footway rule ---
    // required: highway=footway
    // noneOf: [bicycle in designated, yes, permissive, official]
    private boolean matchesBareFootwayRule(ReaderWay way) {
        String highway = way.getTag("highway");
        if (!"footway".equals(highway)) {
            return false;
        }

        String bicycle = way.getTag("bicycle");
        // If bicycle tag grants access, this was handled by cycleway rule
        if (bicycle != null && BICYCLE_ALLOWED.contains(bicycle)) {
            return false;
        }

        return true;
    }

    // --- Nordic track outdoor way rule ---
    // required: NORDIC_TRACK (highway=track, piste:type=nordic)
    // anyOf: [GOOD_SURFACE, GOOD_TRACKTYPE, FOURTH_TRACKTYPE, GOOD_SMOOTHNESS,
    //         OK_SMOOTHNESS, GOOD_MTB_SCALE, LIKELY_GOOD_SURFACE, BICYCLE_OK]
    private boolean matchesNordicTrackOutdoorWayRule(ReaderWay way) {
        String highway = way.getTag("highway");
        String pisteType = way.getTag("piste:type");

        // required: highway=track AND piste:type=nordic
        if (!"track".equals(highway) || !"nordic".equals(pisteType)) {
            return false;
        }

        return matchesGoodConditions(way);
    }

    // --- Nordic path outdoor way rule ---
    // required: NORDIC_PATH (highway=path, width>2, piste:type=nordic)
    // anyOf: [GOOD_SURFACE, GOOD_TRACKTYPE, FOURTH_TRACKTYPE, GOOD_SMOOTHNESS,
    //         OK_SMOOTHNESS, GOOD_MTB_SCALE, LIKELY_GOOD_SURFACE, BICYCLE_OK]
    private boolean matchesNordicPathOutdoorWayRule(ReaderWay way) {
        String highway = way.getTag("highway");
        String pisteType = way.getTag("piste:type");
        String widthStr = way.getTag("width");

        // required: highway=path AND piste:type=nordic AND width>2
        if (!"path".equals(highway) || !"nordic".equals(pisteType)) {
            return false;
        }

        // Check width > 2
        if (!isWidthGreaterThan(widthStr, 2.0)) {
            return false;
        }

        return matchesGoodConditions(way);
    }

    // --- Outdoor path rule ---
    // required: highway=path
    // anyOf: [GOOD_SURFACE, GOOD_TRACKTYPE, FOURTH_TRACKTYPE, GOOD_SMOOTHNESS,
    //         OK_SMOOTHNESS, BICYCLE_DESIGNATED]
    private boolean matchesOutdoorPathRule(ReaderWay way) {
        String highway = way.getTag("highway");

        // required: highway=path
        if (!"path".equals(highway)) {
            return false;
        }

        String surface = way.getTag("surface");
        String tracktype = way.getTag("tracktype");
        String smoothness = way.getTag("smoothness");
        String bicycle = way.getTag("bicycle");

        // GOOD_SURFACE
        if (surface != null && GOOD_SURFACES.contains(surface)) {
            return true;
        }

        // GOOD_TRACKTYPE: grade1, grade2, grade3
        if (tracktype != null && GOOD_TRACKTYPES.contains(tracktype)) {
            return true;
        }

        // FOURTH_TRACKTYPE: grade4
        if ("grade4".equals(tracktype)) {
            return true;
        }

        // GOOD_SMOOTHNESS: excellent, good, intermediate
        if (smoothness != null && GOOD_SMOOTHNESS_VALUES.contains(smoothness)) {
            return true;
        }

        // OK_SMOOTHNESS: bad
        if ("bad".equals(smoothness)) {
            return true;
        }

        // BICYCLE_DESIGNATED
        if ("designated".equals(bicycle)) {
            return true;
        }

        return false;
    }

    // --- Good track rule ---
    // required: highway=track
    // anyOf: [GOOD_SURFACE, GOOD_TRACKTYPE, FOURTH_TRACKTYPE, GOOD_SMOOTHNESS,
    //         OK_SMOOTHNESS, GOOD_MTB_SCALE, LIKELY_GOOD_SURFACE, BICYCLE_OK]
    private boolean matchesGoodTrackRule(ReaderWay way) {
        String highway = way.getTag("highway");

        // required: highway=track
        if (!"track".equals(highway)) {
            return false;
        }

        return matchesGoodConditions(way);
    }

    // --- Track-like path good rule ---
    // required: TRACK_LIKE_PATH (highway=path, width>2)
    // anyOf: [GOOD_SURFACE, GOOD_TRACKTYPE, FOURTH_TRACKTYPE, GOOD_SMOOTHNESS,
    //         OK_SMOOTHNESS, GOOD_MTB_SCALE, LIKELY_GOOD_SURFACE, BICYCLE_OK]
    private boolean matchesTrackLikePathGoodRule(ReaderWay way) {
        String highway = way.getTag("highway");
        String widthStr = way.getTag("width");

        // required: highway=path AND width>2
        if (!"path".equals(highway)) {
            return false;
        }

        if (!isWidthGreaterThan(widthStr, 2.0)) {
            return false;
        }

        return matchesGoodConditions(way);
    }

    // --- Shared good conditions matcher ---
    // anyOf: [GOOD_SURFACE, GOOD_TRACKTYPE, FOURTH_TRACKTYPE, GOOD_SMOOTHNESS,
    //         OK_SMOOTHNESS, GOOD_MTB_SCALE, LIKELY_GOOD_SURFACE, BICYCLE_OK]
    private boolean matchesGoodConditions(ReaderWay way) {
        String surface = way.getTag("surface");
        String tracktype = way.getTag("tracktype");
        String smoothness = way.getTag("smoothness");
        String mtbScale = way.getTag("mtb:scale");
        String bicycle = way.getTag("bicycle");

        // GOOD_SURFACE: asphalt, paved, concrete, compacted, fine_gravel
        if (surface != null && GOOD_SURFACES.contains(surface)) {
            return true;
        }

        // GOOD_TRACKTYPE: grade1, grade2, grade3
        if (tracktype != null && GOOD_TRACKTYPES.contains(tracktype)) {
            return true;
        }

        // FOURTH_TRACKTYPE: grade4
        if ("grade4".equals(tracktype)) {
            return true;
        }

        // GOOD_SMOOTHNESS: excellent, good, intermediate
        if (smoothness != null && GOOD_SMOOTHNESS_VALUES.contains(smoothness)) {
            return true;
        }

        // OK_SMOOTHNESS: bad
        if ("bad".equals(smoothness)) {
            return true;
        }

        // GOOD_MTB_SCALE: 0-, 0, 0+
        if (mtbScale != null && GOOD_MTB_SCALES.contains(mtbScale)) {
            return true;
        }

        // LIKELY_GOOD_SURFACE: surface=unpaved
        if ("unpaved".equals(surface)) {
            return true;
        }

        // BICYCLE_OK: bicycle=yes
        if ("yes".equals(bicycle)) {
            return true;
        }

        return false;
    }

    // =====================================================================
    // HELPER METHODS
    // =====================================================================

    private boolean isWidthGreaterThan(String widthStr, double threshold) {
        if (widthStr == null) {
            return false;
        }
        try {
            // Handle comma decimals that start with 0, (common typo in OSM data)
            if (widthStr.startsWith("0,")) {
                widthStr = widthStr.replaceFirst("^0,", "0.");
            }
            // Handle various formats: "2", "2m", "2.5", "2.5 m"
            String cleaned = widthStr.replaceAll("[^0-9.]", "");
            if (cleaned.isEmpty()) {
                return false;
            }
            double width = Double.parseDouble(cleaned);
            return width > threshold;
        } catch (NumberFormatException e) {
            return false;
        }
    }

    // =====================================================================
    // TAG VALUE SETS
    // =====================================================================

    private static final Set<String> MOTORWAY_HIGHWAYS = new HashSet<>(
        Arrays.asList("motorway", "motorway_link")
    );

    private static final Set<String> MAJOR_ROAD_HIGHWAYS = new HashSet<>(
        Arrays.asList("trunk", "trunk_link", "primary", "primary_link")
    );

    private static final Set<String> SECONDARY_TERTIARY_HIGHWAYS = new HashSet<>(
        Arrays.asList("secondary", "secondary_link", "tertiary", "tertiary_link")
    );

    private static final Set<String> RESIDENTIAL_UNCLASSIFIED_HIGHWAYS = new HashSet<>(
        Arrays.asList("residential", "unclassified")
    );

    private static final Set<String> BICYCLE_DESIGNATED_YES = new HashSet<>(
        Arrays.asList("designated", "yes")
    );

    private static final Set<String> BICYCLE_ALLOWED = new HashSet<>(
        Arrays.asList("yes", "designated", "permissive", "official")
    );

    private static final Set<String> GOOD_SURFACES = new HashSet<>(
        Arrays.asList("asphalt", "paved", "concrete", "compacted", "fine_gravel")
    );

    private static final Set<String> GOOD_TRACKTYPES = new HashSet<>(
        Arrays.asList("grade1", "grade2", "grade3")
    );

    private static final Set<String> GOOD_SMOOTHNESS_VALUES = new HashSet<>(
        Arrays.asList("excellent", "good", "intermediate")
    );

    private static final Set<String> GOOD_MTB_SCALES = new HashSet<>(
        Arrays.asList("0-", "0", "0+")
    );
}
