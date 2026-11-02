package com.graphhopper.trailmap.barrier;

import com.graphhopper.routing.ev.EnumEncodedValue;

/**
 * What a barrier node means for one mode (bike or foot), decided at import by {@link BarrierClassRule}
 * (docs/gh_barrier_gate_proposal.md, ACCEPTED 2026-11-01). Stored on barrier edges only; every other edge is NONE.
 * <ul>
 * <li>NONE: allow, nothing to say.</li>
 * <li>NOTICE: allow, but the route result carries a notice (a gate with opening hours only).</li>
 * <li>ASK: closed for normal routing; with {@code barriers=v1} a point behind it is reached by a confirmed crossing.</li>
 * <li>REFUSE: closed; with {@code barriers=v1} a point behind it gets a named refusal.</li>
 * </ul>
 * The access bit of the barrier edge follows the class (ASK and REFUSE closed), so every profile and endpoint that
 * does not know about barrier classes keeps them closed.
 */
public enum BarrierClass {
    NONE, NOTICE, ASK, REFUSE;

    public static final String KEY_BIKE = "bike_barrier";
    public static final String KEY_FOOT = "foot_barrier";

    public static EnumEncodedValue<BarrierClass> create(String key) {
        return new EnumEncodedValue<>(key, BarrierClass.class);
    }

    public boolean closes() {
        return this == ASK || this == REFUSE;
    }

    public enum Mode {
        BIKE("bicycle", "foot", KEY_BIKE), FOOT("foot", "bicycle", KEY_FOOT);

        public final String key, otherKey, evKey;

        Mode(String key, String otherKey, String evKey) {
            this.key = key;
            this.otherKey = otherKey;
            this.evKey = evKey;
        }
    }
}
