package com.graphhopper.trailmap.barrier;

import com.graphhopper.GraphHopper;
import com.graphhopper.config.Profile;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.subnetwork.PrepareRoutingSubnetworks;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.util.PMap;
import com.graphhopper.util.Parameters;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;

/**
 * The second island marking {@code <profile>_subnetwork_b}: the same search as GraphHopper's
 * ({@link PrepareRoutingSubnetworks}, same minimum size, same u-turn rule), but barrier edges of class ASK and REFUSE
 * count as open. So an island behind a barrier is not marked and can take a route point, while every other island is
 * marked exactly as in {@code <profile>_subnetwork}, which stays untouched. Used only by requests with
 * {@code barriers=v1}.
 */
public final class BarrierSubnetworks {

    private BarrierSubnetworks() {
    }

    public static void prepare(GraphHopper hopper, int minNetworkSize, int threads) {
        List<PrepareRoutingSubnetworks.PrepareJob> jobs = new ArrayList<>();
        for (Profile profile : hopper.getProfiles()) {
            String key = BarrierProfiles.subnetworkKey(profile.getName());
            if (!hopper.getEncodingManager().hasEncodedValue(key))
                continue;
            Weighting w = hopper.createWeighting(profile, new PMap().putObject(Parameters.Routing.U_TURN_COSTS, 0));
            EnumEncodedValue<BarrierClass> enc = BarrierProfiles.barrierEnc(hopper.getEncodingManager(), profile);
            if (enc != null)
                w = new BarrierPassWeighting(w, enc, EnumSet.of(BarrierClass.ASK, BarrierClass.REFUSE), 1);
            jobs.add(new PrepareRoutingSubnetworks.PrepareJob(hopper.getEncodingManager().getBooleanEncodedValue(key), w));
        }
        if (jobs.isEmpty())
            return;
        PrepareRoutingSubnetworks preparation = new PrepareRoutingSubnetworks(hopper.getBaseGraph().getBaseGraph(), jobs);
        preparation.setMinNetworkSize(minNetworkSize);
        preparation.setThreads(threads);
        preparation.doWork();
    }
}
