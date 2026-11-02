package com.graphhopper.trailmap.barrier;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import static com.graphhopper.trailmap.barrier.BarrierClass.*;

/**
 * The accepted barrier rule (docs/gh_barrier_gate_proposal.md, ACCEPTED 2026-11-01, owner's r3 changes in
 * algo/barrier_gates/owner-answers-r3.md): for one barrier node, one mode, the class and the reason. First match wins.
 * Inputs: the node's tags, the highway value of the way the barrier edge belongs to, and whether the node is part of a
 * mapped fence / wall / hedge way ({@link TrailmapOSMReader} sets {@code gh:fenced}).
 * <p>
 * Pure function, no graph access. The reason codes name the proposal's tag families; clients localise them.
 */
public final class BarrierClassRule {

    public static final String FENCED_TAG = "gh:fenced";

    static final Set<String> GATES = set("gate", "swing_gate", "lift_gate", "chain", "sliding_gate", "wicket_gate",
            "hampshire_gate", "bump_gate", "sliding_beam", "spikes");
    static final Set<String> BOOMS = set("swing_gate", "lift_gate", "chain");
    static final Set<String> PED_WAYS = set("footway", "cycleway", "pedestrian");
    static final Set<String> PATH_WAYS = set("path", "steps", "bridleway");
    static final Set<String> NO = set("no", "private", "restricted");
    static final Set<String> HARD = set("military", "emergency");
    static final Set<String> YES = set("yes", "designated", "official", "permissive", "destination");
    static final Set<String> STILES = set("stile", "kissing_gate", "turnstile", "motorcycle_barrier");
    static final Set<String> DOORS = set("door", "full-height_turnstile");
    static final Set<String> BORDER = set("border_control", "border_crossing");
    static final Set<String> WALLS = set("fence", "wall", "hedge");

    private BarrierClassRule() {
    }

    public static final class Result {
        public final BarrierClass cls;
        public final String reason;

        Result(BarrierClass cls, String reason) {
            this.cls = cls;
            this.reason = reason;
        }

        @Override
        public String toString() {
            return cls + ":" + reason;
        }
    }

    private static Result r(BarrierClass c, String reason) {
        return new Result(c, reason);
    }

    /**
     * @param nodeTags the barrier node's tags (values are strings; other objects are read with toString)
     * @param highway  the highway value of the way of the barrier edge (may be null)
     */
    public static Result classify(Map<String, Object> nodeTags, String highway, Mode mode) {
        String b = str(nodeTags, "barrier");
        if (b == null)
            return r(NONE, "NOT_A_BARRIER");
        String mine = str(nodeTags, mode.key);
        String other = str(nodeTags, mode.otherKey);
        String acc = str(nodeTags, "access");
        boolean locked = "yes".equals(str(nodeTags, "locked"));
        boolean fenced = Boolean.TRUE.equals(nodeTags.get(FENCED_TAG)) || "true".equals(str(nodeTags, FENCED_TAG));
        boolean restricted = acc != null && (NO.contains(acc) || HARD.contains(acc));
        boolean onPed = highway != null && PED_WAYS.contains(highway);
        boolean onPath = highway != null && PATH_WAYS.contains(highway);

        if (WALLS.contains(b))
            return r(REFUSE, "WALL_ACROSS_WAY");
        if (BORDER.contains(b) && (restricted || (mine != null && NO.contains(mine))))
            return r(REFUSE, "BORDER_CLOSED");
        if ((acc != null && HARD.contains(acc)) || (mine != null && HARD.contains(mine)))
            return r(REFUSE, "MILITARY_OR_EMERGENCY");
        if (DOORS.contains(b) && mode == Mode.BIKE)
            return r(REFUSE, "DOOR");
        if ((mine != null && YES.contains(mine)) || (mine == null && acc != null && YES.contains(acc)))
            return r(NONE, "MODE_ALLOWED");
        if (mine != null && NO.contains(mine)) {
            if ((other != null && NO.contains(other)) || (other == null && acc != null && NO.contains(acc)))
                return r(REFUSE, "BIKE_AND_FOOT_BANNED");
            return r(ASK, "MODE_BANNED_OTHER_ALLOWED");
        }
        // a value on the mode's own key that is neither a ban nor a yes (dismount, permit, customers…) still
        // overrides the general access key for this mode (OSM access hierarchy), as GraphHopper's stock rule does
        if (mine != null)
            acc = null;
        if (locked) {
            if (fenced)
                return r(REFUSE, "LOCKED_IN_FENCE");
            if (acc != null && NO.contains(acc))
                return r(REFUSE, "LOCKED_AND_PRIVATE");
            if (DOORS.contains(b) || "turnstile".equals(b))
                return r(REFUSE, "LOCKED_DOOR");
            if (nodeTags.containsKey("opening_hours"))
                return r(ASK, "LOCKED_WITH_OPENING_HOURS");
            if (other != null && YES.contains(other))
                return r(NONE, "LOCKED_OTHER_MODE_ALLOWED");
            String mv = str(nodeTags, "motor_vehicle"), mc = str(nodeTags, "motorcar"), v = str(nodeTags, "vehicle");
            if ((mv != null && (NO.contains(mv) || "destination".equals(mv))) || (mc != null && NO.contains(mc))
                    || (v != null && NO.contains(v)))
                return r(NONE, "LOCKED_MOTOR_VEHICLES_ONLY");
            if (onPed)
                return r(ASK, "LOCKED_ON_FOOTWAY");
            if (BOOMS.contains(b))
                return r(NONE, "LOCKED_BOOM");
            if (onPath)
                return r(ASK, "LOCKED_GATE_ON_PATH");
            return r(ASK, "LOCKED_PLAIN_GATE");
        }
        if (acc != null && NO.contains(acc)) {
            // only the other mode is allowed (e.g. access=no + foot=yes): passable, but not explicitly for this mode
            if (other != null && YES.contains(other))
                return r(ASK, "PRIVATE_OTHER_MODE_ALLOWED");
            if (DOORS.contains(b) || STILES.contains(b))
                return r(REFUSE, "PRIVATE_DOOR");
            if (fenced)
                return r(REFUSE, "PRIVATE_IN_FENCE");
            if (GATES.contains(b))
                return r(ASK, "PRIVATE_GATE");
            return r(ASK, "PRIVATE_BARRIER");
        }
        if (DOORS.contains(b))
            return r(NONE, "DOOR_FOOT");
        if (STILES.contains(b))
            return r(NONE, "STILE");
        if (nodeTags.containsKey("opening_hours"))
            return r(NOTICE, "OPENING_HOURS");
        return r(NONE, "NO_RESTRICTION");
    }

    static String str(Map<String, Object> tags, String key) {
        Object o = tags.get(key);
        return o == null ? null : o.toString();
    }

    private static Set<String> set(String... s) {
        return new HashSet<>(Arrays.asList(s));
    }
}
