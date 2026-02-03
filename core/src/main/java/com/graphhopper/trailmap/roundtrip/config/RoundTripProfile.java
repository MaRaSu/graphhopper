/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Configuration POJO for round-trip quality profiles.
 */
package com.graphhopper.trailmap.roundtrip.config;

import java.util.ArrayList;
import java.util.List;

/**
 * Configuration for a round-trip quality profile.
 * Defines snapping preferences and scoring thresholds for a specific routing
 * profile
 * (e.g., gravel, roadbike, mtb).
 */
public class RoundTripProfile {

    private String name;
    private SnappingConfig snapping;
    private ScoringConfig scoring;
    private WeightsConfig weights;

    public RoundTripProfile() {
        this.snapping = new SnappingConfig();
        this.scoring = new ScoringConfig();
        this.weights = new WeightsConfig();
    }

    public RoundTripProfile(String name) {
        this();
        this.name = name;
    }

    // Getters and setters

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public SnappingConfig getSnapping() {
        return snapping;
    }

    public void setSnapping(SnappingConfig snapping) {
        this.snapping = snapping;
    }

    public ScoringConfig getScoring() {
        return scoring;
    }

    public void setScoring(ScoringConfig scoring) {
        this.scoring = scoring;
    }

    public WeightsConfig getWeights() {
        return weights;
    }

    public void setWeights(WeightsConfig weights) {
        this.weights = weights;
    }

    /**
     * Snapping configuration - preferences for waypoint road selection.
     */
    public static class SnappingConfig {
        private double searchRadiusMeters = 500;
        private List<String> preferredSurfaces = new ArrayList<>();
        private List<String> avoidSurfaces = new ArrayList<>();
        private List<String> preferHighways = new ArrayList<>();
        private List<String> avoidHighways = new ArrayList<>();
        private boolean fallbackToAny = true;

        public double getSearchRadiusMeters() {
            return searchRadiusMeters;
        }

        public void setSearchRadiusMeters(double searchRadiusMeters) {
            this.searchRadiusMeters = searchRadiusMeters;
        }

        public List<String> getPreferredSurfaces() {
            return preferredSurfaces;
        }

        public void setPreferredSurfaces(List<String> preferredSurfaces) {
            this.preferredSurfaces = preferredSurfaces;
        }

        public List<String> getAvoidSurfaces() {
            return avoidSurfaces;
        }

        public void setAvoidSurfaces(List<String> avoidSurfaces) {
            this.avoidSurfaces = avoidSurfaces;
        }

        public List<String> getPreferHighways() {
            return preferHighways;
        }

        public void setPreferHighways(List<String> preferHighways) {
            this.preferHighways = preferHighways;
        }

        public List<String> getAvoidHighways() {
            return avoidHighways;
        }

        public void setAvoidHighways(List<String> avoidHighways) {
            this.avoidHighways = avoidHighways;
        }

        public boolean isFallbackToAny() {
            return fallbackToAny;
        }

        public void setFallbackToAny(boolean fallbackToAny) {
            this.fallbackToAny = fallbackToAny;
        }
    }

    /**
     * Scoring configuration - thresholds for route quality assessment.
     */
    public static class ScoringConfig {
        private double minOverallScore = 60;
        private double minLegScore = 40;
        private double maxAsphaltRatio = 0.3;
        private double minUnpavedRatio = 0.5;
        private double maxRepetitionRatio = 0.1;
        private double maxHighwayRatio = 0.15;
        private double minDirectnessRatio = 0.4;
        private double maxDirectnessRatio = 0.9;

        public double getMinOverallScore() {
            return minOverallScore;
        }

        public void setMinOverallScore(double minOverallScore) {
            this.minOverallScore = minOverallScore;
        }

        public double getMinLegScore() {
            return minLegScore;
        }

        public void setMinLegScore(double minLegScore) {
            this.minLegScore = minLegScore;
        }

        public double getMaxAsphaltRatio() {
            return maxAsphaltRatio;
        }

        public void setMaxAsphaltRatio(double maxAsphaltRatio) {
            this.maxAsphaltRatio = maxAsphaltRatio;
        }

        public double getMinUnpavedRatio() {
            return minUnpavedRatio;
        }

        public void setMinUnpavedRatio(double minUnpavedRatio) {
            this.minUnpavedRatio = minUnpavedRatio;
        }

        public double getMaxRepetitionRatio() {
            return maxRepetitionRatio;
        }

        public void setMaxRepetitionRatio(double maxRepetitionRatio) {
            this.maxRepetitionRatio = maxRepetitionRatio;
        }

        public double getMaxHighwayRatio() {
            return maxHighwayRatio;
        }

        public void setMaxHighwayRatio(double maxHighwayRatio) {
            this.maxHighwayRatio = maxHighwayRatio;
        }

        public double getMinDirectnessRatio() {
            return minDirectnessRatio;
        }

        public void setMinDirectnessRatio(double minDirectnessRatio) {
            this.minDirectnessRatio = minDirectnessRatio;
        }

        public double getMaxDirectnessRatio() {
            return maxDirectnessRatio;
        }

        public void setMaxDirectnessRatio(double maxDirectnessRatio) {
            this.maxDirectnessRatio = maxDirectnessRatio;
        }
    }

    /**
     * Weights configuration - importance of different scoring factors.
     */
    public static class WeightsConfig {
        private double surfaceQuality = 0.4;
        private double roadTypeQuality = 0.3;
        private double uniqueness = 0.2;
        private double directness = 0.1;

        public double getSurfaceQuality() {
            return surfaceQuality;
        }

        public void setSurfaceQuality(double surfaceQuality) {
            this.surfaceQuality = surfaceQuality;
        }

        public double getRoadTypeQuality() {
            return roadTypeQuality;
        }

        public void setRoadTypeQuality(double roadTypeQuality) {
            this.roadTypeQuality = roadTypeQuality;
        }

        public double getUniqueness() {
            return uniqueness;
        }

        public void setUniqueness(double uniqueness) {
            this.uniqueness = uniqueness;
        }

        public double getDirectness() {
            return directness;
        }

        public void setDirectness(double directness) {
            this.directness = directness;
        }
    }

    /**
     * Create a default gravel profile.
     */
    public static RoundTripProfile createGravelProfile() {
        RoundTripProfile profile = new RoundTripProfile("gravel");

        profile.getSnapping().setSearchRadiusMeters(1500);
        profile.getSnapping().setPreferredSurfaces(List.of("gravel", "compacted", "fine_gravel", "ground", "dirt"));
        profile.getSnapping().setAvoidSurfaces(List.of("asphalt", "concrete", "paved"));
        profile.getSnapping().setPreferHighways(List.of("track", "path", "cycleway", "unclassified", "service"));
        profile.getSnapping().setAvoidHighways(List.of("primary", "secondary", "trunk", "motorway"));

        profile.getScoring().setMinOverallScore(60);
        profile.getScoring().setMaxAsphaltRatio(0.3);
        profile.getScoring().setMinUnpavedRatio(0.5);

        return profile;
    }

    /**
     * Create a default road bike profile.
     */
    public static RoundTripProfile createRoadBikeProfile() {
        RoundTripProfile profile = new RoundTripProfile("roadbike");

        profile.getSnapping().setSearchRadiusMeters(2000);
        profile.getSnapping().setPreferredSurfaces(List.of("asphalt", "paved", "concrete"));
        profile.getSnapping().setAvoidSurfaces(List.of("gravel", "ground", "dirt", "grass", "mud"));
        profile.getSnapping().setPreferHighways(List.of("cycleway", "secondary", "tertiary", "residential"));
        profile.getSnapping().setAvoidHighways(List.of("track", "path", "footway"));

        profile.getScoring().setMinOverallScore(70);
        profile.getScoring().setMaxAsphaltRatio(1.0); // No limit for road bike
        profile.getScoring().setMinUnpavedRatio(0.0); // No requirement

        return profile;
    }

    /**
     * Create a default MTB profile.
     */
    public static RoundTripProfile createMtbProfile() {
        RoundTripProfile profile = new RoundTripProfile("mtb");

        profile.getSnapping().setSearchRadiusMeters(800);
        profile.getSnapping().setPreferredSurfaces(List.of("ground", "dirt", "grass", "gravel", "earth"));
        profile.getSnapping().setAvoidSurfaces(List.of("asphalt", "concrete", "paved"));
        profile.getSnapping().setPreferHighways(List.of("path", "track", "bridleway"));
        profile.getSnapping().setAvoidHighways(List.of("primary", "secondary", "trunk", "motorway", "tertiary"));

        profile.getScoring().setMinOverallScore(55);
        profile.getScoring().setMaxAsphaltRatio(0.2);
        profile.getScoring().setMinUnpavedRatio(0.6);

        return profile;
    }
}
