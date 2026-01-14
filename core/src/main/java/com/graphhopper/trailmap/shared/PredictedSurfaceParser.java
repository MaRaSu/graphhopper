/*
 * Trailmap - PredictedSurfaceParser
 *
 * Full port of predictedSurfaceRules from route-profile-rules.ts (lines 690-830).
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
 * Parser that computes PredictedSurface from OSM tags.
 * Implements the predictedSurfaceRules logic from route-profile-rules.ts.
 */
public class PredictedSurfaceParser implements TagParser {

    private final EnumEncodedValue<PredictedSurface> predictedSurfaceEnc;

    public PredictedSurfaceParser(EnumEncodedValue<PredictedSurface> predictedSurfaceEnc) {
        this.predictedSurfaceEnc = predictedSurfaceEnc;
    }

    @Override
    public void handleWayTags(int edgeId, EdgeIntAccess edgeIntAccess, ReaderWay way, IntsRef relationFlags) {
        PredictedSurface surface = computePredictedSurface(way);
        predictedSurfaceEnc.setEnum(false, edgeId, edgeIntAccess, surface);
    }

    /**
     * Compute PredictedSurface from OSM tags.
     * Full port of predictedSurfaceRules from route-profile-rules.ts
     */
    public PredictedSurface computePredictedSurface(ReaderWay way) {
        String highway = way.getTag("highway");
        String surface = way.getTag("surface");
        String tracktype = way.getTag("tracktype");
        String smoothness = way.getTag("smoothness");
        String mtbScale = way.getTag("mtb:scale");
        String route = way.getTag("route");

        // =================================================================
        // RULE 1: Ferry route -> FERRY
        // anyOf: [FERRY_ROUTE]
        // =================================================================
        if ("ferry".equals(route)) {
            return PredictedSurface.FERRY;
        }

        // =================================================================
        // RULE 2: Asphalt surfaces -> ASPHALT
        // anyOf: [ASPHALT, MOTORWAY, MAJOR_ROAD, SECONDARY_ROAD_NO_SURFACE]
        // =================================================================
        if (matchesAsphaltRule(way)) {
            return PredictedSurface.ASPHALT;
        }

        // =================================================================
        // RULE 3: Compacted surfaces -> COMPACTED
        // anyOf: [surface=compacted, LIKELY_HIGHWAY_COMPACT, LIKELY_UNPAVED_COMPACT,
        //         LIKELY_UNPAVED_COMPACT2, EXCELLENT_GOOD_SMOOTHNESS, MID_SMOOTHNESS,
        //         FIRST_TRACKTYPE, SECOND_TRACKTYPE, LIKELY_SERVICE_COMPACT]
        // =================================================================
        if (matchesCompactedRule(way)) {
            return PredictedSurface.COMPACTED;
        }

        // =================================================================
        // RULE 4: Ground surfaces -> GROUND
        // anyOf: [surface in [dirt, ground], PATH_NO_SURFACE]
        // =================================================================
        if (matchesGroundRule(way)) {
            return PredictedSurface.GROUND;
        }

        // =================================================================
        // RULE 5: Fine gravel -> FINE_GRAVEL
        // anyOf: [surface=fine_gravel, CYCLEWAY_UNPAVED_OR_GRAVEL, CYCLEWAY_NO_SURFACE,
        //         THIRD_TRACKTYPE, MTB_SCALE_0MINUS]
        // =================================================================
        if (matchesFineGravelRule(way)) {
            return PredictedSurface.FINE_GRAVEL;
        }

        // =================================================================
        // RULE 6: Medium gravel -> MEDIUM_GRAVEL
        // anyOf: [FOURTH_TRACKTYPE, MTB_SCALE_0]
        // noneOf: [PATH]
        // =================================================================
        if (matchesMediumGravelRule(way)) {
            return PredictedSurface.MEDIUM_GRAVEL;
        }

        // =================================================================
        // RULE 7: Rough gravel (track) -> ROUGH_GRAVEL
        // required: highway=track
        // anyOf: [surface=gravel, TRACK_NO_SURFACE, BAD_TRACKTYPE, HORRIBLE_AND_VERY_SMOOTHNESS]
        // =================================================================
        if (matchesRoughGravelRule(way)) {
            return PredictedSurface.ROUGH_GRAVEL;
        }

        // =================================================================
        // RULE 8: Sand -> SAND
        // required: highway in [path, track]
        // anyOf: [surface=sand]
        // =================================================================
        if (matchesSandRule(way)) {
            return PredictedSurface.SAND;
        }

        // =================================================================
        // RULE 9: Mud -> MUD
        // required: highway in [path, track]
        // anyOf: [surface=mud]
        // =================================================================
        if (matchesMudRule(way)) {
            return PredictedSurface.MUD;
        }

        // =================================================================
        // DEFAULT: Unknown
        // =================================================================
        return PredictedSurface.UNKNOWN;
    }

    // =====================================================================
    // RULE MATCHERS - Full port from route-profile-rules.ts
    // =====================================================================

    // --- ASPHALT rule ---
    // anyOf: [ASPHALT, MOTORWAY, MAJOR_ROAD, SECONDARY_ROAD_NO_SURFACE]
    private boolean matchesAsphaltRule(ReaderWay way) {
        String surface = way.getTag("surface");
        String highway = way.getTag("highway");

        // ASPHALT: surface in [asphalt, paved, concrete, paving_stones]
        if (surface != null && ASPHALT_SURFACES.contains(surface)) {
            return true;
        }

        // MOTORWAY: highway in [motorway, motorway_link]
        if (highway != null && MOTORWAY_HIGHWAYS.contains(highway)) {
            return true;
        }

        // MAJOR_ROAD: highway in [trunk, trunk_link, primary, primary_link]
        if (highway != null && MAJOR_ROAD_HIGHWAYS.contains(highway)) {
            return true;
        }

        // SECONDARY_ROAD_NO_SURFACE: highway in [secondary, secondary_link] AND no surface
        if (highway != null && SECONDARY_HIGHWAYS.contains(highway) && surface == null) {
            return true;
        }

        return false;
    }

    // --- COMPACTED rule ---
    // anyOf: [surface=compacted, LIKELY_HIGHWAY_COMPACT, LIKELY_UNPAVED_COMPACT,
    //         LIKELY_UNPAVED_COMPACT2, EXCELLENT_GOOD_SMOOTHNESS, MID_SMOOTHNESS,
    //         FIRST_TRACKTYPE, SECOND_TRACKTYPE, LIKELY_SERVICE_COMPACT]
    private boolean matchesCompactedRule(ReaderWay way) {
        String surface = way.getTag("surface");
        String highway = way.getTag("highway");
        String tracktype = way.getTag("tracktype");
        String smoothness = way.getTag("smoothness");

        // surface=compacted
        if ("compacted".equals(surface)) {
            return true;
        }

        // LIKELY_HIGHWAY_COMPACT: highway in [tertiary, unclassified, residential] AND no surface
        if (highway != null && LIKELY_COMPACT_HIGHWAYS.contains(highway) && surface == null) {
            return true;
        }

        // LIKELY_UNPAVED_COMPACT: highway in [tertiary, unclassified, residential]
        //                         AND surface in [unpaved, gravel, sand, mud, fine_gravel]
        if (highway != null && LIKELY_COMPACT_HIGHWAYS.contains(highway) &&
            surface != null && UNPAVED_COMPACT_SURFACES.contains(surface)) {
            return true;
        }

        // LIKELY_UNPAVED_COMPACT2: highway=service AND surface=unpaved
        if ("service".equals(highway) && "unpaved".equals(surface)) {
            return true;
        }

        // EXCELLENT_GOOD_SMOOTHNESS: smoothness in [excellent, good]
        if (smoothness != null && EXCELLENT_GOOD_SMOOTHNESS_VALUES.contains(smoothness)) {
            return true;
        }

        // MID_SMOOTHNESS: smoothness in [intermediate, bad]
        if (smoothness != null && MID_SMOOTHNESS_VALUES.contains(smoothness)) {
            return true;
        }

        // FIRST_TRACKTYPE: tracktype=grade1
        if ("grade1".equals(tracktype)) {
            return true;
        }

        // SECOND_TRACKTYPE: tracktype=grade2
        if ("grade2".equals(tracktype)) {
            return true;
        }

        // LIKELY_SERVICE_COMPACT: highway=service AND no surface
        if ("service".equals(highway) && surface == null) {
            return true;
        }

        return false;
    }

    // --- GROUND rule ---
    // anyOf: [surface in [dirt, ground], PATH_NO_SURFACE]
    private boolean matchesGroundRule(ReaderWay way) {
        String surface = way.getTag("surface");
        String highway = way.getTag("highway");

        // surface in [dirt, ground]
        if (surface != null && GROUND_SURFACES.contains(surface)) {
            return true;
        }

        // PATH_NO_SURFACE: highway=path AND no surface
        if ("path".equals(highway) && surface == null) {
            return true;
        }

        return false;
    }

    // --- FINE_GRAVEL rule ---
    // anyOf: [surface=fine_gravel, CYCLEWAY_UNPAVED_OR_GRAVEL, CYCLEWAY_NO_SURFACE,
    //         THIRD_TRACKTYPE, MTB_SCALE_0MINUS]
    private boolean matchesFineGravelRule(ReaderWay way) {
        String surface = way.getTag("surface");
        String highway = way.getTag("highway");
        String tracktype = way.getTag("tracktype");
        String mtbScale = way.getTag("mtb:scale");

        // surface=fine_gravel
        if ("fine_gravel".equals(surface)) {
            return true;
        }

        // CYCLEWAY_UNPAVED_OR_GRAVEL: highway in [cycleway, footway] AND surface in [unpaved, gravel]
        if (highway != null && CYCLEWAY_FOOTWAY.contains(highway) &&
            surface != null && UNPAVED_GRAVEL_SURFACES.contains(surface)) {
            return true;
        }

        // CYCLEWAY_NO_SURFACE: highway in [cycleway, footway] AND no surface
        if (highway != null && CYCLEWAY_FOOTWAY.contains(highway) && surface == null) {
            return true;
        }

        // THIRD_TRACKTYPE: tracktype=grade3
        if ("grade3".equals(tracktype)) {
            return true;
        }

        // MTB_SCALE_0MINUS: mtb:scale=0-
        if ("0-".equals(mtbScale)) {
            return true;
        }

        return false;
    }

    // --- MEDIUM_GRAVEL rule ---
    // anyOf: [FOURTH_TRACKTYPE, MTB_SCALE_0]
    // noneOf: [PATH]
    private boolean matchesMediumGravelRule(ReaderWay way) {
        String highway = way.getTag("highway");
        String tracktype = way.getTag("tracktype");
        String mtbScale = way.getTag("mtb:scale");

        // noneOf: PATH (highway=path)
        if ("path".equals(highway)) {
            return false;
        }

        // FOURTH_TRACKTYPE: tracktype=grade4
        if ("grade4".equals(tracktype)) {
            return true;
        }

        // MTB_SCALE_0: mtb:scale=0
        if ("0".equals(mtbScale)) {
            return true;
        }

        return false;
    }

    // --- ROUGH_GRAVEL rule ---
    // required: highway=track
    // anyOf: [surface=gravel, TRACK_NO_SURFACE, BAD_TRACKTYPE, HORRIBLE_AND_VERY_SMOOTHNESS]
    private boolean matchesRoughGravelRule(ReaderWay way) {
        String highway = way.getTag("highway");
        String surface = way.getTag("surface");
        String tracktype = way.getTag("tracktype");
        String smoothness = way.getTag("smoothness");

        // required: highway=track
        if (!"track".equals(highway)) {
            return false;
        }

        // surface=gravel
        if ("gravel".equals(surface)) {
            return true;
        }

        // TRACK_NO_SURFACE: highway=track AND no surface (already have highway=track)
        if (surface == null) {
            return true;
        }

        // BAD_TRACKTYPE: tracktype=grade5
        if ("grade5".equals(tracktype)) {
            return true;
        }

        // HORRIBLE_AND_VERY_SMOOTHNESS: smoothness in [horrible, very_horrible]
        if (smoothness != null && HORRIBLE_SMOOTHNESS_VALUES.contains(smoothness)) {
            return true;
        }

        return false;
    }

    // --- SAND rule ---
    // required: highway in [path, track]
    // anyOf: [surface=sand]
    private boolean matchesSandRule(ReaderWay way) {
        String highway = way.getTag("highway");
        String surface = way.getTag("surface");

        // required: highway in [path, track]
        if (highway == null || !PATH_TRACK_HIGHWAYS.contains(highway)) {
            return false;
        }

        // surface=sand
        return "sand".equals(surface);
    }

    // --- MUD rule ---
    // required: highway in [path, track]
    // anyOf: [surface=mud]
    private boolean matchesMudRule(ReaderWay way) {
        String highway = way.getTag("highway");
        String surface = way.getTag("surface");

        // required: highway in [path, track]
        if (highway == null || !PATH_TRACK_HIGHWAYS.contains(highway)) {
            return false;
        }

        // surface=mud
        return "mud".equals(surface);
    }

    // =====================================================================
    // TAG VALUE SETS
    // =====================================================================

    private static final Set<String> ASPHALT_SURFACES = new HashSet<>(
        Arrays.asList("asphalt", "paved", "concrete", "paving_stones")
    );

    private static final Set<String> MOTORWAY_HIGHWAYS = new HashSet<>(
        Arrays.asList("motorway", "motorway_link")
    );

    private static final Set<String> MAJOR_ROAD_HIGHWAYS = new HashSet<>(
        Arrays.asList("trunk", "trunk_link", "primary", "primary_link")
    );

    private static final Set<String> SECONDARY_HIGHWAYS = new HashSet<>(
        Arrays.asList("secondary", "secondary_link")
    );

    private static final Set<String> LIKELY_COMPACT_HIGHWAYS = new HashSet<>(
        Arrays.asList("tertiary", "unclassified", "residential")
    );

    private static final Set<String> UNPAVED_COMPACT_SURFACES = new HashSet<>(
        Arrays.asList("unpaved", "gravel", "sand", "mud", "fine_gravel")
    );

    private static final Set<String> EXCELLENT_GOOD_SMOOTHNESS_VALUES = new HashSet<>(
        Arrays.asList("excellent", "good")
    );

    private static final Set<String> MID_SMOOTHNESS_VALUES = new HashSet<>(
        Arrays.asList("intermediate", "bad")
    );

    private static final Set<String> GROUND_SURFACES = new HashSet<>(
        Arrays.asList("dirt", "ground")
    );

    private static final Set<String> CYCLEWAY_FOOTWAY = new HashSet<>(
        Arrays.asList("cycleway", "footway")
    );

    private static final Set<String> UNPAVED_GRAVEL_SURFACES = new HashSet<>(
        Arrays.asList("unpaved", "gravel")
    );

    private static final Set<String> HORRIBLE_SMOOTHNESS_VALUES = new HashSet<>(
        Arrays.asList("horrible", "very_horrible")
    );

    private static final Set<String> PATH_TRACK_HIGHWAYS = new HashSet<>(
        Arrays.asList("path", "track")
    );
}
