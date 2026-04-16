package com.graphhopper.trailmap.tbt;

import com.fasterxml.jackson.annotation.JsonProperty;

import java.util.List;
import java.util.Map;

/**
 * Response DTO for the Trailmap TbT instruction generation endpoint.
 * Returns a single flat instruction list for the entire route with a reference polyline.
 */
public class TrailmapInstructionResponse {

    @JsonProperty("instructions")
    private List<Map<String, Object>> instructions;

    @JsonProperty("points")
    private String points;

    @JsonProperty("points_encoded")
    private boolean pointsEncoded = true;

    @JsonProperty("points_encoded_multiplier")
    private double pointsEncodedMultiplier = 1e6;

    public TrailmapInstructionResponse() {}

    public List<Map<String, Object>> getInstructions() { return instructions; }
    public void setInstructions(List<Map<String, Object>> instructions) { this.instructions = instructions; }

    public String getPoints() { return points; }
    public void setPoints(String points) { this.points = points; }

    public boolean isPointsEncoded() { return pointsEncoded; }
    public void setPointsEncoded(boolean pointsEncoded) { this.pointsEncoded = pointsEncoded; }

    public double getPointsEncodedMultiplier() { return pointsEncodedMultiplier; }
    public void setPointsEncodedMultiplier(double pointsEncodedMultiplier) { this.pointsEncodedMultiplier = pointsEncodedMultiplier; }
}
