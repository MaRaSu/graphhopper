/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Route quality score result.
 */
package com.graphhopper.trailmap.roundtrip.scoring;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Result of scoring a round-trip route.
 * Contains overall score, per-leg breakdown, and detailed metrics.
 */
public class RouteScore {

    private double overallScore;
    private boolean acceptable;
    private List<LegScore> legScores;
    private Map<String, Double> metrics;
    private List<String> issues;
    private List<String> fixesApplied;
    private int attemptsUsed;

    // Distance tracking for unified quality model
    private double actualDistance;
    private double targetDistance;

    /** Distance tolerance (±15%) */
    public static final double DISTANCE_TOLERANCE = 0.15;

    public RouteScore() {
        this.legScores = new ArrayList<>();
        this.metrics = new LinkedHashMap<>();
        this.issues = new ArrayList<>();
        this.fixesApplied = new ArrayList<>();
        this.attemptsUsed = 1;
    }

    /**
     * Create a score indicating success.
     */
    public static RouteScore acceptable(double score) {
        RouteScore rs = new RouteScore();
        rs.setOverallScore(score);
        rs.setAcceptable(true);
        return rs;
    }

    /**
     * Create a score indicating failure.
     */
    public static RouteScore unacceptable(double score, String issue) {
        RouteScore rs = new RouteScore();
        rs.setOverallScore(score);
        rs.setAcceptable(false);
        rs.getIssues().add(issue);
        return rs;
    }

    // Getters and setters

    public double getOverallScore() {
        return overallScore;
    }

    public void setOverallScore(double overallScore) {
        this.overallScore = overallScore;
    }

    public boolean isAcceptable() {
        return acceptable;
    }

    public void setAcceptable(boolean acceptable) {
        this.acceptable = acceptable;
    }

    public List<LegScore> getLegScores() {
        return legScores;
    }

    public void setLegScores(List<LegScore> legScores) {
        this.legScores = legScores;
    }

    public Map<String, Double> getMetrics() {
        return metrics;
    }

    public void setMetrics(Map<String, Double> metrics) {
        this.metrics = metrics;
    }

    public List<String> getIssues() {
        return issues;
    }

    public void setIssues(List<String> issues) {
        this.issues = issues;
    }

    public List<String> getFixesApplied() {
        return fixesApplied;
    }

    public void setFixesApplied(List<String> fixesApplied) {
        this.fixesApplied = fixesApplied;
    }

    public int getAttemptsUsed() {
        return attemptsUsed;
    }

    public void setAttemptsUsed(int attemptsUsed) {
        this.attemptsUsed = attemptsUsed;
    }

    public double getActualDistance() {
        return actualDistance;
    }

    public void setActualDistance(double actualDistance) {
        this.actualDistance = actualDistance;
    }

    public double getTargetDistance() {
        return targetDistance;
    }

    public void setTargetDistance(double targetDistance) {
        this.targetDistance = targetDistance;
    }

    /**
     * Get the distance ratio (actual / target).
     */
    public double getDistanceRatio() {
        if (targetDistance <= 0) return 1.0;
        return actualDistance / targetDistance;
    }

    /**
     * Check if distance is within acceptable tolerance.
     */
    public boolean isDistanceWithinTolerance() {
        double ratio = getDistanceRatio();
        return ratio >= (1 - DISTANCE_TOLERANCE) && ratio <= (1 + DISTANCE_TOLERANCE);
    }

    /**
     * Get the distance issue type, if any.
     */
    public IssueType getDistanceIssue() {
        double ratio = getDistanceRatio();
        if (ratio > 1 + DISTANCE_TOLERANCE) {
            return IssueType.TOO_LONG;
        } else if (ratio < 1 - DISTANCE_TOLERANCE) {
            return IssueType.TOO_SHORT;
        }
        return IssueType.NONE;
    }

    /**
     * Add a metric value.
     */
    public RouteScore withMetric(String name, double value) {
        this.metrics.put(name, value);
        return this;
    }

    /**
     * Add an issue.
     */
    public RouteScore withIssue(String issue) {
        this.issues.add(issue);
        return this;
    }

    /**
     * Add a fix description.
     */
    public RouteScore withFix(String fixDescription) {
        this.fixesApplied.add(fixDescription);
        return this;
    }

    /**
     * Find the worst leg (lowest score).
     * @return Index of worst leg, or -1 if no legs
     */
    public int getWorstLegIndex() {
        if (legScores.isEmpty()) {
            return -1;
        }
        int worstIndex = 0;
        double worstScore = legScores.get(0).getScore();
        for (int i = 1; i < legScores.size(); i++) {
            if (legScores.get(i).getScore() < worstScore) {
                worstScore = legScores.get(i).getScore();
                worstIndex = i;
            }
        }
        return worstIndex;
    }

    /**
     * Get the worst leg's issue type.
     */
    public IssueType getWorstLegIssue() {
        int idx = getWorstLegIndex();
        if (idx < 0) {
            return IssueType.NONE;
        }
        return legScores.get(idx).getMainIssue();
    }

    /**
     * Convert to a map for JSON serialization.
     */
    public Map<String, Object> toMap() {
        Map<String, Object> map = new LinkedHashMap<>();
        map.put("overallScore", overallScore);
        map.put("acceptable", acceptable);
        map.put("metrics", metrics);

        List<Map<String, Object>> legList = new ArrayList<>();
        for (LegScore leg : legScores) {
            legList.add(leg.toMap());
        }
        map.put("legScores", legList);

        if (!issues.isEmpty()) {
            map.put("issues", issues);
        }
        if (!fixesApplied.isEmpty()) {
            map.put("fixesApplied", fixesApplied);
        }
        map.put("attemptsUsed", attemptsUsed);

        if (targetDistance > 0) {
            map.put("targetDistance", Math.round(targetDistance));
            map.put("actualDistance", Math.round(actualDistance));
            map.put("distanceRatio", Math.round(getDistanceRatio() * 100) / 100.0);
        }

        return map;
    }

    @Override
    public String toString() {
        return String.format("RouteScore{score=%.1f, acceptable=%s, issues=%d, legs=%d}",
            overallScore, acceptable, issues.size(), legScores.size());
    }
}
