/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Snapping preference patterns based on gravel_scale and mtb_scale.
 */
package com.graphhopper.trailmap.roundtrip.snapping;

import com.graphhopper.trailmap.shared.GravelScale;
import com.graphhopper.trailmap.shared.MtbScale;

/**
 * Snapping preference patterns for waypoint snapping.
 *
 * <p>
 * Each preference defines a scoring pattern that determines which edges are
 * preferred when snapping waypoints. The client selects the preference via
 * the API parameter {@link #API_PARAM}.
 *
 * <p>
 * Scoring uses the appropriate scale for each preference:
 * <ul>
 * <li>PAVED, LIGHT_GRAVEL, GRAVEL: gravel_scale only</li>
 * <li>ROUGH_GRAVEL: gravel_scale (primary) + mtb_scale (light weight)</li>
 * <li>MTB: mtb_scale (primary), gravel_scale as fallback</li>
 * </ul>
 */
public enum SnappingPreference {

    /**
     * Paved roads preferred (road bike).
     * Uses gravel_scale: strongly prefer ZERO_MINUS (paved).
     */
    PAVED,

    /**
     * Light gravel / smooth unpaved (gravel bike, easy).
     * Uses gravel_scale: prefer ZERO to ONE (well-maintained unpaved).
     */
    LIGHT_GRAVEL,

    /**
     * Standard gravel (gravel bike, normal).
     * Uses gravel_scale: prefer ZERO_PLUS to TWO (typical gravel).
     */
    GRAVEL,

    /**
     * Rough gravel / easy MTB terrain (adventurous gravel).
     * Uses gravel_scale (primary): prefer ONE to THREE.
     */
    ROUGH_GRAVEL,

    /**
     * MTB trails preferred.
     * Uses mtb_scale (primary): prefer ONE to THREE (technical but rideable).
     * Falls back to gravel_scale when mtb_scale not available.
     */
    MTB;

    /**
     * API parameter name for selecting snapping preference.
     */
    public static final String API_PARAM = "round_trip.snapping_preference";

    /**
     * Score an edge based on this preference pattern.
     * Higher score = better match for this preference.
     *
     * <p>
     * Scoring strategy per preference:
     * <ul>
     * <li>PAVED, LIGHT_GRAVEL, GRAVEL, ROUGH_GRAVEL: Use gravel_scale only</li>
     * <li>MTB: Use mtb_scale (primary), gravel_scale as fallback when mtb_scale
     * missing</li>
     * </ul>
     *
     * @param gravelScale The gravel_scale value of the edge (may be null)
     * @param mtbScale    The mtb_scale value of the edge (may be null)
     * @return Score value (can be negative for poor matches)
     */
    public double scoreEdge(GravelScale gravelScale, MtbScale mtbScale) {
        switch (this) {
            case PAVED:
                return scorePaved(gravelScale);
            case LIGHT_GRAVEL:
                return scoreLightGravel(gravelScale);
            case GRAVEL:
                return scoreGravel(gravelScale);
            case ROUGH_GRAVEL:
                return scoreRoughGravel(gravelScale);
            case MTB:
                return scoreMtb(gravelScale, mtbScale);
            default:
                return 0;
        }
    }

    /**
     * PAVED: gravel_scale only - strongly prefer paved roads.
     */
    private double scorePaved(GravelScale gravelScale) {
        if (gravelScale == null) {
            return 0;
        }
        switch (gravelScale) {
            case FERRY:
                return -200; // Avoid ferries
            case ZERO_MINUS:
                return 25; // Perfect - paved
            case ZERO:
                return 5; // OK - smooth unpaved
            case ZERO_PLUS:
            case ONE:
                return -15; // Mild penalty
            case TWO:
            case THREE:
            case FOUR:
                return -40; // Strong penalty
            case UNKNOWN:
                return -40;
            default:
                return -40;
        }
    }

    /**
     * LIGHT_GRAVEL: gravel_scale only - prefer smooth unpaved.
     */
    private double scoreLightGravel(GravelScale gravelScale) {
        if (gravelScale == null) {
            return 0;
        }
        switch (gravelScale) {
            case FERRY:
                return -200; // Avoid ferries
            case ZERO:
                return 25; // Best - smooth unpaved
            case ZERO_PLUS:
                return 20; // Good
            case ONE:
                return 10; // OK
            case ZERO_MINUS:
                return 3; // Paved is acceptable
            case TWO:
                return -15;
            case THREE:
            case FOUR:
                return -35;
            case UNKNOWN:
                return -40;
            default:
                return -40;
        }
    }

    /**
     * GRAVEL: gravel_scale only - sweet spot for gravel bikes.
     */
    private double scoreGravel(GravelScale gravelScale) {
        if (gravelScale == null) {
            return 0;
        }
        switch (gravelScale) {
            case FERRY:
                return -200; // Avoid ferries
            case ZERO_PLUS:
            case ONE:
                return 25; // Ideal
            case TWO:
                return 3; // Good
            case ZERO:
                return 15; // OK
            case ZERO_MINUS:
                return 5; // paved
            case THREE:
                return -15;
            case FOUR:
                return -35;
            case UNKNOWN:
                return -40;
            default:
                return -40;
        }
    }

    /**
     * ROUGH_GRAVEL: gravel_scale (primary)
     */
    private double scoreRoughGravel(GravelScale gravelScale) {
        if (gravelScale == null) {
            return 0;
        }

        // Primary: gravel_scale
        switch (gravelScale) {
            case FERRY:
                return -200; // Avoid ferries
            case ZERO_PLUS:
            case ONE:
                return 15; // Ideal
            case TWO:
                return 13; // Good
            case ZERO:
                return 10; // OK
            case ZERO_MINUS:
                return 5; // paved
            case THREE:
                return -15;
            case FOUR:
                return -35;
            case UNKNOWN:
                return -40;
            default:
                return -40;
        }

    }

    /**
     * MTB: mtb_scale (primary), gravel_scale as fallback when mtb_scale missing.
     */
    private double scoreMtb(GravelScale gravelScale, MtbScale mtbScale) {
        // If mtb_scale is available, use it as primary
        if (mtbScale != null && mtbScale != MtbScale.UNKNOWN) {
            switch (mtbScale) {
                case FERRY:
                    return -200; // Avoid ferries
                case TWO:
                    return 15; // Ideal
                case ONE:
                    return 12;
                case THREE:
                    return 10;
                case ZERO_PLUS:
                    return 5;
                case ZERO:
                    return 2;
                case ZERO_MINUS:
                    return -5;
                case FOUR:
                    return -3;
                case FIVE:
                case SIX:
                    return -8;
                default:
                    break;
            }
        }

        // Fallback: use gravel_scale when mtb_scale not available
        if (gravelScale != null) {
            switch (gravelScale) {
                case FERRY:
                    return -200; // Avoid ferries
                case THREE:
                    return 12;
                case TWO:
                    return 10;
                case FOUR:
                    return 5;
                case ONE:
                    return 5;
                case ZERO_PLUS:
                    return 2;
                case ZERO:
                    return -3;
                case ZERO_MINUS:
                    return -15; // Avoid paved
                case UNKNOWN:
                    return -1;
                default:
                    break;
            }
        }

        return 0;
    }

    /**
     * Parse from string (case-insensitive).
     *
     * @param value String value to parse (PAVED, LIGHT_GRAVEL, GRAVEL,
     *              ROUGH_GRAVEL, MTB)
     * @return Matching preference, or GRAVEL as default
     */
    public static SnappingPreference fromString(String value) {
        if (value == null || value.isEmpty()) {
            return GRAVEL;
        }
        try {
            return valueOf(value.toUpperCase().replace("-", "_"));
        } catch (IllegalArgumentException e) {
            return GRAVEL;
        }
    }

    /**
     * Get a human-readable description of this preference.
     */
    public String getDescription() {
        switch (this) {
            case PAVED:
                return "Paved roads (road bike)";
            case LIGHT_GRAVEL:
                return "Light gravel, smooth unpaved";
            case GRAVEL:
                return "Standard gravel bike terrain";
            case ROUGH_GRAVEL:
                return "Rough gravel, adventurous";
            case MTB:
                return "Mountain bike trails";
            default:
                return name();
        }
    }
}
