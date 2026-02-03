/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Result of a route fix attempt.
 */
package com.graphhopper.trailmap.roundtrip.fixing;

import com.graphhopper.util.shapes.GHPoint;

import java.util.List;

/**
 * Result of attempting to fix a route quality issue.
 */
public class FixResult {

    private final boolean success;
    private final List<GHPoint> modifiedWaypoints;
    private final String description;
    private final String fixerName;

    private FixResult(boolean success, List<GHPoint> modifiedWaypoints,
                      String description, String fixerName) {
        this.success = success;
        this.modifiedWaypoints = modifiedWaypoints;
        this.description = description;
        this.fixerName = fixerName;
    }

    /**
     * Create a successful fix result.
     *
     * @param waypoints Modified waypoint list
     * @param description Human-readable description of the fix
     * @param fixerName Name of the fixer that created this result
     * @return Success result
     */
    public static FixResult success(List<GHPoint> waypoints, String description, String fixerName) {
        return new FixResult(true, waypoints, description, fixerName);
    }

    /**
     * Create a successful fix result.
     *
     * @param waypoints Modified waypoint list
     * @param description Human-readable description of the fix
     * @return Success result
     */
    public static FixResult success(List<GHPoint> waypoints, String description) {
        return new FixResult(true, waypoints, description, null);
    }

    /**
     * Create a failure result.
     *
     * @param reason Why the fix failed
     * @return Failure result
     */
    public static FixResult failure(String reason) {
        return new FixResult(false, null, reason, null);
    }

    /**
     * Create a failure result.
     *
     * @param reason Why the fix failed
     * @param fixerName Name of the fixer that failed
     * @return Failure result
     */
    public static FixResult failure(String reason, String fixerName) {
        return new FixResult(false, null, reason, fixerName);
    }

    // Getters

    public boolean isSuccess() {
        return success;
    }

    public List<GHPoint> getModifiedWaypoints() {
        return modifiedWaypoints;
    }

    public String getDescription() {
        return description;
    }

    public String getFixerName() {
        return fixerName;
    }

    @Override
    public String toString() {
        if (success) {
            return String.format("FixResult{success=true, waypoints=%d, desc='%s'}",
                modifiedWaypoints != null ? modifiedWaypoints.size() : 0, description);
        } else {
            return String.format("FixResult{success=false, reason='%s'}", description);
        }
    }
}
