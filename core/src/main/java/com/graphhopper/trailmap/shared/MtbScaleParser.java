/*
 * Trailmap - MtbScaleParser
 *
 * Computes MtbScale from mtb:scale tag with inference for paths/tracks
 * that lack explicit mtb:scale tagging.
 *
 * Based on design doc gh_profile_migration_design.md section 3.3
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
 * Parser that computes MtbScale from OSM tags.
 * Uses mtb:scale tag when present, with inference fallback for paths/tracks.
 */
public class MtbScaleParser implements TagParser {

    private final EnumEncodedValue<MtbScale> mtbScaleEnc;

    public MtbScaleParser(EnumEncodedValue<MtbScale> mtbScaleEnc) {
        this.mtbScaleEnc = mtbScaleEnc;
    }

    @Override
    public void handleWayTags(int edgeId, EdgeIntAccess edgeIntAccess, ReaderWay way, IntsRef relationFlags) {
        MtbScale scale = computeMtbScale(way);
        mtbScaleEnc.setEnum(false, edgeId, edgeIntAccess, scale);
    }

    /**
     * Compute MtbScale from OSM tags.
     * Priority: mtb:scale tag > inference from highway/surface/tracktype
     */
    public MtbScale computeMtbScale(ReaderWay way) {
        String route = way.getTag("route");
        String highway = way.getTag("highway");
        String mtbScaleTag = way.getTag("mtb:scale");

        // =================================================================
        // RULE 1: Ferry -> FERRY
        // =================================================================
        if ("ferry".equals(route)) {
            return MtbScale.FERRY;
        }

        // =================================================================
        // RULE 2: Use mtb:scale tag if present
        // =================================================================
        if (mtbScaleTag != null) {
            MtbScale fromTag = MtbScale.fromOsmTag(mtbScaleTag);
            if (fromTag != null) {
                return fromTag;
            }
        }

        // =================================================================
        // INFERENCE for paths without mtb:scale
        // =================================================================
        if ("path".equals(highway)) {
            return inferPathScale(way);
        }

        // =================================================================
        // INFERENCE for tracks without mtb:scale
        // =================================================================
        if ("track".equals(highway)) {
            return inferTrackScale(way);
        }

        // =================================================================
        // Roads, cycleways, footways -> ZERO_MINUS (easy)
        // =================================================================
        if (highway != null && EASY_HIGHWAYS.contains(highway)) {
            return MtbScale.ZERO_MINUS;
        }

        // =================================================================
        // DEFAULT: Unknown
        // =================================================================
        return MtbScale.UNKNOWN;
    }

    /**
     * Infer MtbScale for paths without explicit mtb:scale tag.
     */
    private MtbScale inferPathScale(ReaderWay way) {
        String surface = way.getTag("surface");
        String bicycle = way.getTag("bicycle");
        String smoothness = way.getTag("smoothness");

        // Bicycle designated paths are usually easy
        if ("designated".equals(bicycle)) {
            return MtbScale.ZERO;
        }

        // Good surface indicates easier path
        if (surface != null && GOOD_SURFACES.contains(surface)) {
            return MtbScale.ZERO;
        }

        // Good smoothness
        if (smoothness != null && GOOD_SMOOTHNESS.contains(smoothness)) {
            return MtbScale.ZERO;
        }

        // Difficult surfaces
        if (surface != null && DIFFICULT_SURFACES.contains(surface)) {
            return MtbScale.TWO;
        }

        // Bad/horrible smoothness
        if (smoothness != null && BAD_SMOOTHNESS.contains(smoothness)) {
            return MtbScale.TWO;
        }

        // Horrible smoothness
        if (smoothness != null && HORRIBLE_SMOOTHNESS.contains(smoothness)) {
            return MtbScale.THREE;
        }

        // No quality indicators - truly unknown
        return MtbScale.UNKNOWN;
    }

    /**
     * Infer MtbScale for tracks without explicit mtb:scale tag.
     */
    private MtbScale inferTrackScale(ReaderWay way) {
        String tracktype = way.getTag("tracktype");
        String surface = way.getTag("surface");
        String smoothness = way.getTag("smoothness");

        // Use tracktype for inference
        if (tracktype != null) {
            switch (tracktype) {
                case "grade1":
                    return MtbScale.ZERO_MINUS;
                case "grade2":
                    return MtbScale.ZERO;
                case "grade3":
                    return MtbScale.ZERO_PLUS;
                case "grade4":
                    return MtbScale.ONE;
                case "grade5":
                    return MtbScale.TWO;
            }
        }

        // Good surface
        if (surface != null && GOOD_SURFACES.contains(surface)) {
            return MtbScale.ZERO;
        }

        // Good smoothness
        if (smoothness != null && GOOD_SMOOTHNESS.contains(smoothness)) {
            return MtbScale.ZERO;
        }

        // Difficult surfaces
        if (surface != null && DIFFICULT_SURFACES.contains(surface)) {
            return MtbScale.ONE;
        }

        // Tracks without quality info assumed rideable
        return MtbScale.ZERO;
    }

    // =====================================================================
    // TAG VALUE SETS
    // =====================================================================

    private static final Set<String> EASY_HIGHWAYS = new HashSet<>(Arrays.asList(
        "motorway", "motorway_link",
        "trunk", "trunk_link",
        "primary", "primary_link",
        "secondary", "secondary_link",
        "tertiary", "tertiary_link",
        "unclassified", "residential",
        "service", "cycleway", "footway",
        "living_street", "pedestrian"
    ));

    private static final Set<String> GOOD_SURFACES = new HashSet<>(Arrays.asList(
        "asphalt", "paved", "concrete", "paving_stones",
        "compacted", "fine_gravel"
    ));

    private static final Set<String> GOOD_SMOOTHNESS = new HashSet<>(Arrays.asList(
        "excellent", "good", "intermediate"
    ));

    private static final Set<String> DIFFICULT_SURFACES = new HashSet<>(Arrays.asList(
        "mud", "sand", "grass"
    ));

    private static final Set<String> BAD_SMOOTHNESS = new HashSet<>(Arrays.asList(
        "bad", "very_bad"
    ));

    private static final Set<String> HORRIBLE_SMOOTHNESS = new HashSet<>(Arrays.asList(
        "horrible", "very_horrible", "impassable"
    ));
}
