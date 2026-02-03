/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Generates deterministic variations from a seed for waypoint placement.
 * All variation derives from the base seed, ensuring reproducibility.
 */
package com.graphhopper.trailmap.roundtrip.geometry;

/**
 * Generates deterministic variations from a seed.
 *
 * Used to create route variety while maintaining reproducibility:
 * - Same seed produces identical variations
 * - Different seeds produce genuinely different waypoint placements
 * - Variations are orthogonal to client-controlled parameters (heading, distance, shape)
 */
public class SeededVariation {

    private final long baseSeed;

    // Golden ratio based mixing constant for better hash distribution
    private static final long GOLDEN_RATIO = 0x9E3779B97F4A7C15L;

    public SeededVariation(long seed) {
        this.baseSeed = seed;
    }

    /**
     * Get the base seed.
     */
    public long getBaseSeed() {
        return baseSeed;
    }

    /**
     * Radial amplitude factor per waypoint.
     * Range: [0.65, 1.35] - significant variation to hit different road catchments.
     *
     * @param waypointIndex Index of the waypoint
     * @return Factor to multiply base distance by
     */
    public double getRadialFactor(int waypointIndex) {
        return 0.65 + 0.70 * deterministicRandom("radial", waypointIndex);
    }

    /**
     * Angular offset per waypoint in degrees.
     * Range: [-18, +18] - enough to shift into adjacent road corridors
     * without fundamentally changing the shape.
     *
     * @param waypointIndex Index of the waypoint
     * @return Offset in degrees to add to base bearing
     */
    public double getAngularOffsetDegrees(int waypointIndex) {
        return -18.0 + 36.0 * deterministicRandom("angular", waypointIndex);
    }

    /**
     * Perpendicular offset factor per waypoint.
     * Range: [-0.15, +0.15] of base distance.
     * Creates "wobbly" shapes where waypoints don't lie on clean radial lines.
     *
     * @param waypointIndex Index of the waypoint
     * @return Factor of base distance for perpendicular offset
     */
    public double getPerpendicularFactor(int waypointIndex) {
        return -0.15 + 0.30 * deterministicRandom("perp", waypointIndex);
    }

    /**
     * Quadrant stretch factor.
     * Range: [0.75, 1.25]
     * Creates asymmetric shapes by stretching different angular sectors differently.
     *
     * @param quadrant Quadrant index (0-3, based on bearing / 90)
     * @return Factor to multiply distance by for this quadrant
     */
    public double getQuadrantStretch(int quadrant) {
        return 0.75 + 0.50 * deterministicRandom("stretch", quadrant % 4);
    }

    /**
     * Generic variation value for custom use cases.
     * Range: [0, 1)
     *
     * @param aspect String identifier for this variation aspect
     * @param index Numeric index
     * @return Deterministic value in [0, 1)
     */
    public double getVariation(String aspect, int index) {
        return deterministicRandom(aspect, index);
    }

    /**
     * Generic variation value in a custom range.
     *
     * @param aspect String identifier for this variation aspect
     * @param index Numeric index
     * @param min Minimum value (inclusive)
     * @param max Maximum value (exclusive)
     * @return Deterministic value in [min, max)
     */
    public double getVariationInRange(String aspect, int index, double min, double max) {
        return min + (max - min) * deterministicRandom(aspect, index);
    }

    /**
     * Hash-based deterministic random value in [0, 1).
     * Uses golden ratio mixing for good distribution.
     *
     * @param aspect String identifier for variation type
     * @param index Numeric index
     * @return Value in [0, 1)
     */
    private double deterministicRandom(String aspect, int index) {
        // Combine seed with aspect and index
        long combined = baseSeed;
        combined ^= aspect.hashCode() * 31L;
        combined ^= index * 127L;

        // Mix using golden ratio for better distribution
        combined *= GOLDEN_RATIO;
        combined ^= combined >>> 33;
        combined *= GOLDEN_RATIO;

        // Convert to [0, 1) range
        return (Math.abs(combined) % 1000000) / 1000000.0;
    }
}
