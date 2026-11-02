package com.graphhopper.trailmap.barrier;

import com.graphhopper.config.Profile;
import com.graphhopper.json.Statement;
import com.graphhopper.routing.ev.BooleanEncodedValue;
import com.graphhopper.routing.ev.EncodedValueLookup;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.ev.SimpleBooleanEncodedValue;
import com.graphhopper.routing.ev.Subnetwork;
import com.graphhopper.util.CustomModel;

import java.util.List;

/**
 * Which barrier class applies to a profile, and the names of its barrier-aware island marking.
 * A profile follows {@code bike_barrier} when its custom model reads {@code bike_access}, and {@code foot_barrier} when
 * it reads {@code foot_access}; otherwise (nordic_ski, winter_mtb: their own access) it has none and is unchanged.
 */
public final class BarrierProfiles {

    private BarrierProfiles() {
    }

    public static BarrierClass.Mode modeOf(Profile profile) {
        CustomModel cm = profile.getCustomModel();
        if (cm == null)
            return null;
        boolean bike = reads(cm.getPriority(), "bike_access") || reads(cm.getSpeed(), "bike_access");
        boolean foot = reads(cm.getPriority(), "foot_access") || reads(cm.getSpeed(), "foot_access");
        if (bike == foot)
            return null;
        return bike ? BarrierClass.Mode.BIKE : BarrierClass.Mode.FOOT;
    }

    private static boolean reads(List<Statement> statements, String ev) {
        if (statements == null)
            return false;
        for (Statement s : statements) {
            if (s.condition() != null && s.condition().contains(ev))
                return true;
            if (s.isBlock() && reads(s.doBlock(), ev))
                return true;
        }
        return false;
    }

    /** {@code <profile>_subnetwork_b}: the island marking in which barriers of class ASK and REFUSE count as open. */
    public static String subnetworkKey(String profile) {
        return Subnetwork.key(profile) + "_b";
    }

    public static BooleanEncodedValue createSubnetworkEV(String profile) {
        return new SimpleBooleanEncodedValue(subnetworkKey(profile));
    }

    /** The barrier class value of this profile, or null if the graph has none for it. */
    public static EnumEncodedValue<BarrierClass> barrierEnc(EncodedValueLookup lookup, Profile profile) {
        BarrierClass.Mode mode = modeOf(profile);
        if (mode == null || !lookup.hasEncodedValue(mode.evKey))
            return null;
        return lookup.getEnumEncodedValue(mode.evKey, BarrierClass.class);
    }

    /** The barrier-aware marking of this profile, or the stock marking if the graph has none. */
    public static BooleanEncodedValue subnetworkEnc(EncodedValueLookup lookup, String profile) {
        String key = subnetworkKey(profile);
        return lookup.hasEncodedValue(key) ? lookup.getBooleanEncodedValue(key) : lookup.getBooleanEncodedValue(Subnetwork.key(profile));
    }
}
