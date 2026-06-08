/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Per-leg quality score.
 */
package com.graphhopper.trailmap.roundtrip.scoring;

import com.graphhopper.util.shapes.GHPoint;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Quality score for a single leg of a round-trip route.
 */
public class LegScore {

    private int legIndex;
    private double score;
    private double surfaceQuality;
    private double roadTypeQuality;
    private double uniquenessRatio;
    private double directnessRatio;
    private double distance;
    private IssueType mainIssue;

    // Repetition breakdown
    private double deadEndDistance;              // Meters of dead-end backtrack
    private double distantRepetitionDistance;    // Meters of distant (non-adjacent) repetition
    private int forkEdgeIndex = -1;      // Edge index where backtrack starts (-1 if none)
    private GHPoint forkPoint;           // Coordinate for dead-end fixer

    // Geospatial corridor overlap (different-edge parallel/antiparallel reuse)
    private double corridorOverlapDistance;      // Meters this leg runs alongside another part of the route
    private GHPoint corridorAnchor;              // Point on this (return) corridor for the fixer to divert
    private GHPoint corridorPartner;             // Nearby point on the other (outbound) corridor; fixer pushes away from it

    public LegScore() {
        this.mainIssue = IssueType.NONE;
    }

    public LegScore(int legIndex) {
        this();
        this.legIndex = legIndex;
    }

    // Getters and setters

    public int getLegIndex() {
        return legIndex;
    }

    public void setLegIndex(int legIndex) {
        this.legIndex = legIndex;
    }

    public double getScore() {
        return score;
    }

    public void setScore(double score) {
        this.score = score;
    }

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

    public double getUniquenessRatio() {
        return uniquenessRatio;
    }

    public void setUniquenessRatio(double uniquenessRatio) {
        this.uniquenessRatio = uniquenessRatio;
    }

    public double getDirectnessRatio() {
        return directnessRatio;
    }

    public void setDirectnessRatio(double directnessRatio) {
        this.directnessRatio = directnessRatio;
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public IssueType getMainIssue() {
        return mainIssue;
    }

    public void setMainIssue(IssueType mainIssue) {
        this.mainIssue = mainIssue;
    }

    public double getDeadEndDistance() {
        return deadEndDistance;
    }

    public void setDeadEndDistance(double deadEndDistance) {
        this.deadEndDistance = deadEndDistance;
    }

    public double getDistantRepetitionDistance() {
        return distantRepetitionDistance;
    }

    public void setDistantRepetitionDistance(double distantRepetitionDistance) {
        this.distantRepetitionDistance = distantRepetitionDistance;
    }

    public int getForkEdgeIndex() {
        return forkEdgeIndex;
    }

    public void setForkEdgeIndex(int forkEdgeIndex) {
        this.forkEdgeIndex = forkEdgeIndex;
    }

    public GHPoint getForkPoint() {
        return forkPoint;
    }

    public void setForkPoint(GHPoint forkPoint) {
        this.forkPoint = forkPoint;
    }

    public double getCorridorOverlapDistance() {
        return corridorOverlapDistance;
    }

    public void setCorridorOverlapDistance(double corridorOverlapDistance) {
        this.corridorOverlapDistance = corridorOverlapDistance;
    }

    public GHPoint getCorridorAnchor() {
        return corridorAnchor;
    }

    public void setCorridorAnchor(GHPoint corridorAnchor) {
        this.corridorAnchor = corridorAnchor;
    }

    public GHPoint getCorridorPartner() {
        return corridorPartner;
    }

    public void setCorridorPartner(GHPoint corridorPartner) {
        this.corridorPartner = corridorPartner;
    }

    /**
     * Convert to map for JSON serialization.
     */
    public Map<String, Object> toMap() {
        Map<String, Object> map = new LinkedHashMap<>();
        map.put("leg", legIndex);
        map.put("score", Math.round(score * 10) / 10.0);
        map.put("surfaceQuality", Math.round(surfaceQuality * 100) / 100.0);
        map.put("roadTypeQuality", Math.round(roadTypeQuality * 100) / 100.0);
        map.put("uniqueness", Math.round(uniquenessRatio * 100) / 100.0);
        map.put("directness", Math.round(directnessRatio * 100) / 100.0);
        map.put("distance", Math.round(distance));
        if (mainIssue != IssueType.NONE) {
            map.put("issue", mainIssue.name());
        }
        if (deadEndDistance > 0) {
            map.put("deadEndDistance", Math.round(deadEndDistance));
        }
        if (distantRepetitionDistance > 0) {
            map.put("distantRepetitionDistance", Math.round(distantRepetitionDistance));
        }
        if (forkPoint != null) {
            map.put("forkPoint", forkPoint.lat + "," + forkPoint.lon);
        }
        if (corridorOverlapDistance > 0) {
            map.put("corridorOverlapDistance", Math.round(corridorOverlapDistance));
        }
        if (corridorAnchor != null) {
            map.put("corridorAnchor", corridorAnchor.lat + "," + corridorAnchor.lon);
        }
        return map;
    }

    @Override
    public String toString() {
        return String.format("Leg[%d]: score=%.1f, surface=%.0f%%, issue=%s",
            legIndex, score, surfaceQuality * 100, mainIssue);
    }
}
