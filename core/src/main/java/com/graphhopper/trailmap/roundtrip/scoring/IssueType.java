/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Types of quality issues that can be detected in a route.
 */
package com.graphhopper.trailmap.roundtrip.scoring;

/**
 * Types of quality issues that can be identified in a round-trip route.
 * Used by scorers to categorize problems and by fixers to determine applicable fixes.
 */
public enum IssueType {

    /** No issue detected */
    NONE,

    /** Route is shorter than target distance */
    TOO_SHORT,

    /** Route is longer than target distance */
    TOO_LONG,

    /** Route is too direct (not enough exploration) */
    TOO_DIRECT,

    /** Too much asphalt/paved surface for the profile */
    HIGH_ASPHALT_RATIO,

    /** Not enough unpaved surface for the profile */
    LOW_UNPAVED_RATIO,

    /** Too many edges used multiple times (backtracking) */
    HIGH_REPETITION,

    /** Dead-end backtrack: route goes out and returns on same edges near waypoint */
    DEAD_END_BACKTRACK,

    /** Distant repetition: edges repeated from non-adjacent segments (out-and-back pattern) */
    DISTANT_REPETITION,

    /** Route stuck on main roads when profile prefers smaller roads */
    MAIN_ROAD_TRAP,

    /** A specific leg has poor quality */
    LOW_QUALITY_LEG,

    /** Route calculation failed completely */
    ROUTE_FAILED,

    /** Route has an unreasonably long detour */
    HUGE_DETOUR,

    /** Route blocked by geographical feature (lake, mountain, etc.) */
    BLOCKED_CORRIDOR,

    /** Generic low score issue */
    LOW_SCORE;

    /**
     * Check if this issue type can potentially be fixed by waypoint adjustment.
     */
    public boolean isFixable() {
        switch (this) {
            case NONE:
            case ROUTE_FAILED:
                return false;
            default:
                return true;
        }
    }

    /**
     * Get a human-readable description of this issue.
     */
    public String getDescription() {
        switch (this) {
            case NONE:
                return "No issues";
            case TOO_SHORT:
                return "Route is shorter than requested distance";
            case TOO_LONG:
                return "Route is longer than requested distance";
            case TOO_DIRECT:
                return "Route is too direct, not exploring the area";
            case HIGH_ASPHALT_RATIO:
                return "Too much paved road for this profile";
            case LOW_UNPAVED_RATIO:
                return "Not enough unpaved surface for this profile";
            case HIGH_REPETITION:
                return "Route reuses too many roads (backtracking)";
            case DEAD_END_BACKTRACK:
                return "Route backtracks on a dead-end road";
            case DISTANT_REPETITION:
                return "Route has out-and-back sections from non-adjacent segments";
            case MAIN_ROAD_TRAP:
                return "Route stuck on main roads";
            case LOW_QUALITY_LEG:
                return "One or more route sections have poor quality";
            case ROUTE_FAILED:
                return "Could not calculate a valid route";
            case HUGE_DETOUR:
                return "Route contains an unreasonably long detour";
            case BLOCKED_CORRIDOR:
                return "Route blocked by geographical obstacle";
            case LOW_SCORE:
                return "Route quality score below threshold";
            default:
                return "Unknown issue";
        }
    }
}
