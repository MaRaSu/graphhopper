/*
 * Trailmap - Personal routing
 */
package com.graphhopper.trailmap.personal;

import com.graphhopper.util.PMap;

import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;
import java.util.regex.Pattern;

/**
 * The personal settings of one request, read from its hints ({@code personal_*}).
 */
public final class PersonalParams {

    public enum Mode {NEUTRAL, FAVOR, AVOID, EXPLORE}

    public static final String USER = "personal_user";
    public static final String CELLS = "personal_cells";
    public static final String MODE = "personal_mode";
    public static final String STRENGTH = "personal_strength";
    public static final String SWITCH_PENALTY = "personal_switch_penalty";
    public static final String RIDES_SATURATION = "personal_rides_saturation";
    public static final String KNOWN_SHARE = "personal_known_share";

    static final double MAX_STRENGTH = 3.0;
    static final double MAX_SWITCH_PENALTY = 600;

    private static final Pattern NAME = Pattern.compile("[A-Za-z0-9_-]{1,64}");

    public final String user;
    public final String cells;
    public final Mode mode;
    /** Extra cost factor: an edge costs at most 1 + strength times its profile cost. */
    public final double strength;
    /** Added at a junction where the route switches between ridden and unridden edges (turn-cost units). */
    public final double switchPenalty;
    /** Ride count at which an edge counts as fully habitual (favor/avoid). */
    public final double ridesSaturation;
    /** Share of an edge's length in the user's cells from which it counts as ridden (switch penalty). */
    public final double knownShare;

    private PersonalParams(String user, String cells, Mode mode, double strength, double switchPenalty,
                           double ridesSaturation, double knownShare) {
        this.user = user;
        this.cells = cells;
        this.mode = mode;
        this.strength = strength;
        this.switchPenalty = switchPenalty;
        this.ridesSaturation = ridesSaturation;
        this.knownShare = knownShare;
    }

    public static PersonalParams from(PMap hints) {
        Mode mode;
        try {
            mode = Mode.valueOf(hints.getString(MODE, "neutral").trim().toUpperCase(Locale.ROOT));
        } catch (IllegalArgumentException e) {
            throw new IllegalArgumentException(MODE + " must be one of neutral, favor, avoid, explore");
        }
        String user = hints.getString(USER, "");
        String cells = hints.getString(CELLS, "");
        // Empty names mean "no history" (routes as /route); only malformed names are an error.
        if (!user.isEmpty() && !NAME.matcher(user).matches())
            throw new IllegalArgumentException(USER + " must be 1-64 characters of A-Z a-z 0-9 _ -");
        if (!cells.isEmpty() && !NAME.matcher(cells).matches())
            throw new IllegalArgumentException(CELLS + " must be 1-64 characters of A-Z a-z 0-9 _ -");
        double strength = clamp(hints.getDouble(STRENGTH, 0.5), 0, MAX_STRENGTH);
        double switchPenalty = clamp(hints.getDouble(SWITCH_PENALTY, 30), 0, MAX_SWITCH_PENALTY);
        double saturation = clamp(hints.getDouble(RIDES_SATURATION, 10), 1, 1000);
        double knownShare = clamp(hints.getDouble(KNOWN_SHARE, 0.5), 0.05, 1);
        return new PersonalParams(user, cells, mode, strength, switchPenalty, saturation, knownShare);
    }

    /** True when the request may change anything: a mode other than neutral, a non-zero effect and a named history. */
    public boolean isActive() {
        return mode != Mode.NEUTRAL && (strength > 0 || switchPenalty > 0) && !user.isEmpty() && !cells.isEmpty();
    }

    public Map<String, Object> toMap() {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("mode", mode.name().toLowerCase(Locale.ROOT));
        m.put("strength", strength);
        m.put("switch_penalty", switchPenalty);
        m.put("rides_saturation", ridesSaturation);
        m.put("known_share", knownShare);
        return m;
    }

    private static double clamp(double v, double lo, double hi) {
        if (Double.isNaN(v)) return lo;
        return Math.max(lo, Math.min(hi, v));
    }
}
