package com.graphhopper.trailmap.tbt;

import com.graphhopper.util.PointList;
import com.graphhopper.util.RoundaboutInstruction;

import java.util.HashMap;
import java.util.Map;

/**
 * Extends {@link RoundaboutInstruction} to preserve Trailmap extraInfo fields
 * alongside roundabout-specific fields in {@link #getExtraInfoJSON()}.
 * <p>
 * The upstream {@code RoundaboutInstruction.getExtraInfoJSON()} creates a fresh map
 * containing only {@code exit_number}, {@code exited}, and {@code turn_angle},
 * discarding everything stored via {@code setExtraInfo()} (which writes to the
 * inherited {@code extraInfo} map). This subclass merges both.
 */
public class TrailmapRoundaboutInstruction extends RoundaboutInstruction {

    public TrailmapRoundaboutInstruction(int sign, String name, PointList pl) {
        super(sign, name, pl);
    }

    @Override
    public Map<String, Object> getExtraInfoJSON() {
        // Start with Trailmap extraInfo (road_class, predicted_highway, etc.)
        Map<String, Object> merged = new HashMap<>(this.extraInfo);
        // Overlay roundabout-specific fields (exit_number, exited, turn_angle)
        merged.putAll(super.getExtraInfoJSON());
        return merged;
    }
}
