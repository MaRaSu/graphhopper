package com.graphhopper.trailmap.barrier;

import org.junit.jupiter.api.Test;

import java.util.HashMap;
import java.util.Map;

import static com.graphhopper.trailmap.barrier.BarrierClass.*;
import static com.graphhopper.trailmap.barrier.BarrierClass.Mode.BIKE;
import static com.graphhopper.trailmap.barrier.BarrierClass.Mode.FOOT;
import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * One case per family of the accepted rule (docs/gh_barrier_gate_proposal.md §1), with the real tags of the nodes the
 * owner judged in rounds 1–2 (live OSM API, 2026-11-01) where there is one.
 */
class BarrierClassRuleTest {

    private static Map<String, Object> tags(String... kv) {
        Map<String, Object> m = new HashMap<>();
        for (int i = 0; i < kv.length; i += 2)
            m.put(kv[i], kv[i + 1]);
        return m;
    }

    private static Map<String, Object> fenced(Map<String, Object> m) {
        m.put(BarrierClassRule.FENCED_TAG, Boolean.TRUE);
        return m;
    }

    private static void check(BarrierClass bike, BarrierClass foot, Map<String, Object> t, String highway, String what) {
        assertEquals(bike, BarrierClassRule.classify(t, highway, BIKE).cls, what + " (bike)");
        assertEquals(foot, BarrierClassRule.classify(t, highway, FOOT).cls, what + " (foot)");
    }

    @Test
    void allow() {
        check(NONE, NONE, tags("barrier", "gate"), "track", "case 1, 5558583221");
        check(NONE, NONE, tags("barrier", "block"), "track", "case 2, 6708198025");
        check(NONE, NONE, tags("barrier", "swing_gate", "locked", "yes", "material", "metal"), "track", "case 4, 6258424701");
        check(NONE, NONE, tags("barrier", "lift_gate", "locked", "yes"), "track", "case 5, 6018277155");
        check(NONE, NONE, tags("barrier", "gate", "locked", "yes", "motor_vehicle", "private"), "service", "case 6, 9993620725");
        check(NONE, NONE, tags("barrier", "gate", "bicycle", "yes", "foot", "yes", "horse", "yes", "locked", "yes", "motor_vehicle", "private"), "unclassified", "case 9, 8695245912");
        check(NONE, NONE, tags("barrier", "swing_gate", "foot", "yes", "locked", "yes", "motor_vehicle", "no"), "track", "case 10, 13790331047");
        check(NONE, NONE, tags("access", "private", "barrier", "gate", "bicycle", "yes", "foot", "yes"), "unclassified", "case 15, 4881688716");
        check(NONE, NONE, tags("barrier", "stile"), "path", "case 18, 12050110548");
        check(NONE, NONE, tags("barrier", "chain", "locked", "yes"), "service", "locked chain");
        check(NONE, NONE, tags("barrier", "gate", "access", "permit"), "track", "access=permit is not a restriction");
        check(NONE, NONE, tags("access", "no", "barrier", "cycle_barrier", "bicycle", "dismount", "foot", "yes"), "cycleway", "bicycle=dismount overrides access=no, 11155584461");
        check(REFUSE, NONE, fenced(tags("access", "no", "barrier", "gate", "foot", "customers")), "service", "foot=customers overrides access=no for foot, 1320793481");
    }

    @Test
    void notice() {
        check(NOTICE, NOTICE, tags("barrier", "gate", "material", "metal", "opening_hours", "Mo-Su 10:00-17:00"), "unclassified", "r2-11, 12097535670");
    }

    @Test
    void ask() {
        check(ASK, ASK, tags("barrier", "gate", "locked", "yes"), "unclassified", "case 3, 12108613024");
        check(ASK, ASK, tags("barrier", "gate", "locked", "yes", "name", "Padan puomi"), "track", "r2-1, 13928560757");
        check(ASK, ASK, tags("barrier", "gate", "locked", "yes"), "cycleway", "case 8, 11853770186 (owner r3: ask)");
        check(ASK, ASK, tags("barrier", "gate", "locked", "yes"), "path", "locked gate on a path");
        check(ASK, ASK, tags("access", "private", "barrier", "gate"), "unclassified", "case 11, 6532693245");
        check(ASK, ASK, tags("access", "no", "barrier", "swing_gate", "wheelchair", "no"), "unclassified", "case 12, 7342586559");
        check(ASK, ASK, tags("access", "private", "barrier", "lift_gate"), "unclassified", "r2-2, 7794224127");
        check(ASK, ASK, tags("access", "private", "barrier", "gate"), "footway", "r2-8, 8862068354");
        check(ASK, ASK, tags("access", "private", "barrier", "yes"), "service", "r2-13, 11091126506");
        check(ASK, NONE, tags("access", "no", "barrier", "gate", "foot", "yes"), "track", "r2-12, 6987929857");
        check(NONE, ASK, tags("access", "no", "barrier", "gate", "bicycle", "yes"), "track", "only bikes allowed, 3121116204");
        check(ASK, NONE, tags("barrier", "gate", "bicycle", "no", "foot", "yes", "horse", "no", "motor_vehicle", "no"), "path", "case 17, 3214112404 (in a fence too)");
        check(ASK, ASK, tags("barrier", "gate", "locked", "yes", "opening_hours", "Mo-Fr 08:00-16:00"), "service", "locked + opening hours");
    }

    @Test
    void refuse() {
        check(REFUSE, REFUSE, fenced(tags("barrier", "gate", "locked", "yes")), "track", "case 7, 11796198227");
        check(REFUSE, REFUSE, tags("access", "private", "barrier", "gate", "locked", "yes", "material", "metal"), "unclassified", "case 13, 11144276973");
        check(REFUSE, REFUSE, fenced(tags("access", "private", "barrier", "gate")), "unclassified", "r2-7, 117569749");
        check(REFUSE, REFUSE, fenced(tags("access", "no", "barrier", "gate")), "unclassified", "case 14, 385704276");
        check(REFUSE, REFUSE, tags("barrier", "lift_gate", "bicycle", "no", "foot", "no"), "unclassified", "case 16, 3370600104");
        check(REFUSE, REFUSE, tags("barrier", "gate", "bicycle", "no", "foot", "no"), "unclassified", "r2-10, 811811893");
        check(REFUSE, NONE, tags("barrier", "full-height_turnstile"), "cycleway", "case 19, 1610736606");
        check(REFUSE, NONE, tags("barrier", "door"), "footway", "door");
        check(REFUSE, REFUSE, tags("barrier", "fence"), "track", "fence node across a way");
        check(REFUSE, REFUSE, tags("barrier", "border_control", "access", "private"), "primary", "border zone, 6629389639");
        check(REFUSE, REFUSE, tags("barrier", "gate", "access", "military"), "service", "military");
        check(REFUSE, REFUSE, tags("barrier", "door", "access", "private"), "service", "private door");
    }

    @Test
    void notABarrier() {
        check(NONE, NONE, tags("ford", "yes"), "track", "a ford node is not decided here");
    }
}
