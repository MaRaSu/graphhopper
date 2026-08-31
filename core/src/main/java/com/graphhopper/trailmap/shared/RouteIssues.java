/*
 * Trailmap - RouteIssues
 *
 * Defines encoded values for route issue detection.
 * Each issue is a boolean flag computed at import time from OSM tags.
 * These are returned as path details to the client for warning display.
 *
 * Implements typedIssueRules from route-profile-rules.ts.
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.routing.ev.BooleanEncodedValue;
import com.graphhopper.routing.ev.SimpleBooleanEncodedValue;

/**
 * Factory class for route issue encoded values.
 * Each issue is a boolean flag indicating a potential problem on the way.
 */
public class RouteIssues {

    // Key constants for encoded values
    public static final String KEY_BIKING_BLOCKED = "issue_biking_blocked";
    public static final String KEY_BIKING_BLOCKED_RISK = "issue_biking_blocked_risk";
    public static final String KEY_FOOT_BLOCKED = "issue_foot_blocked";
    public static final String KEY_NARROW = "issue_narrow";
    public static final String KEY_POOR_VISIBILITY = "issue_poor_visibility";
    public static final String KEY_VEGETATION = "issue_vegetation";
    public static final String KEY_MUD = "issue_mud";
    public static final String KEY_UNKNOWN_PATH = "issue_unknown_path";
    public static final String KEY_UNKNOWN_TRACK = "issue_unknown_track";
    public static final String KEY_FERRY = "issue_ferry";
    public static final String KEY_DRIVEWAY = "issue_driveway";

    /**
     * Biking not permitted: bicycle=no/private or access=no/private without bicycle override.
     */
    public static BooleanEncodedValue createBikingBlocked() {
        return new SimpleBooleanEncodedValue(KEY_BIKING_BLOCKED);
    }

    /**
     * Biking access risk: access tags suggest possible restriction (unknown, agricultural,
     * forestry, delivery, service, permit) but routing is still allowed.
     */
    public static BooleanEncodedValue createBikingBlockedRisk() {
        return new SimpleBooleanEncodedValue(KEY_BIKING_BLOCKED_RISK);
    }

    /**
     * Walking not permitted: foot=no/private or access=no/private without foot override.
     */
    public static BooleanEncodedValue createFootBlocked() {
        return new SimpleBooleanEncodedValue(KEY_FOOT_BLOCKED);
    }

    /**
     * Narrow path: highway=path with width < 0.5m.
     */
    public static BooleanEncodedValue createNarrow() {
        return new SimpleBooleanEncodedValue(KEY_NARROW);
    }

    /**
     * Poor visibility: highway=path with trail_visibility=bad/horrible/no.
     */
    public static BooleanEncodedValue createPoorVisibility() {
        return new SimpleBooleanEncodedValue(KEY_POOR_VISIBILITY);
    }

    /**
     * Vegetation obstacle: obstacle=vegetation.
     */
    public static BooleanEncodedValue createVegetation() {
        return new SimpleBooleanEncodedValue(KEY_VEGETATION);
    }

    /**
     * Muddy surface: highway=path/track with surface=mud.
     */
    public static BooleanEncodedValue createMud() {
        return new SimpleBooleanEncodedValue(KEY_MUD);
    }

    /**
     * Unknown path difficulty: highway=path without mtb:scale and no good condition indicators.
     */
    public static BooleanEncodedValue createUnknownPath() {
        return new SimpleBooleanEncodedValue(KEY_UNKNOWN_PATH);
    }

    /**
     * Unknown track difficulty: highway=track without mtb:scale or name and no good condition indicators.
     */
    public static BooleanEncodedValue createUnknownTrack() {
        return new SimpleBooleanEncodedValue(KEY_UNKNOWN_TRACK);
    }

    /**
     * Ferry crossing: route=ferry.
     */
    public static BooleanEncodedValue createFerry() {
        return new SimpleBooleanEncodedValue(KEY_FERRY);
    }

    /**
     * Driveway: highway=service + service=driveway.
     * <p>
     * The client-facing counterpart of the internal-only SERVICE_DRIVEWAY routing
     * category — in path details and TbT instructions, predicted_highway reports these
     * ways as SERVICE_ROAD, so this flag is how a client learns it is on a driveway.
     */
    public static BooleanEncodedValue createDriveway() {
        return new SimpleBooleanEncodedValue(KEY_DRIVEWAY);
    }

    /**
     * Check if a key is a route issue key.
     */
    public static boolean isRouteIssueKey(String key) {
        return KEY_BIKING_BLOCKED.equals(key) ||
               KEY_BIKING_BLOCKED_RISK.equals(key) ||
               KEY_FOOT_BLOCKED.equals(key) ||
               KEY_NARROW.equals(key) ||
               KEY_POOR_VISIBILITY.equals(key) ||
               KEY_VEGETATION.equals(key) ||
               KEY_MUD.equals(key) ||
               KEY_UNKNOWN_PATH.equals(key) ||
               KEY_UNKNOWN_TRACK.equals(key) ||
               KEY_FERRY.equals(key) ||
               KEY_DRIVEWAY.equals(key);
    }
}
