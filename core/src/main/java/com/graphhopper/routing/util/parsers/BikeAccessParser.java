package com.graphhopper.routing.util.parsers;

import com.graphhopper.reader.ReaderNode;
import com.graphhopper.reader.ReaderWay;
import com.graphhopper.routing.ev.BooleanEncodedValue;
import com.graphhopper.routing.ev.EdgeIntAccess;
import com.graphhopper.routing.ev.EncodedValueLookup;
import com.graphhopper.trailmap.barrier.BarrierAccess;
import com.graphhopper.trailmap.barrier.BarrierClass;
import com.graphhopper.routing.ev.Roundabout;
import com.graphhopper.routing.ev.VehicleAccess;
import com.graphhopper.util.PMap;

public class BikeAccessParser extends BikeCommonAccessParser {

    public BikeAccessParser(EncodedValueLookup lookup, PMap properties) {
        this(lookup.getBooleanEncodedValue(VehicleAccess.key("bike")),
                lookup.getBooleanEncodedValue(Roundabout.KEY));
        blockPrivate(properties.getBool("block_private", true));
        blockFords(properties.getBool("block_fords", false));

        // Trailmap: bikes can pass through these access types
        restrictedValues.remove("unknown");
        restrictedValues.remove("agricultural");
        restrictedValues.remove("forestry");
        restrictedValues.remove("delivery");
        restrictedValues.remove("service");
        restrictedValues.remove("permit");
    }

    public BikeAccessParser(BooleanEncodedValue accessEnc, BooleanEncodedValue roundaboutEnc) {
        super(accessEnc, roundaboutEnc);
        barriers.add("kissing_gate");
        barriers.add("stile");
        barriers.add("turnstile");
    }

    // Trailmap (W26): barrier nodes follow the allow/ask/refuse rule (trailmap/barrier/BarrierClassRule)
    @Override
    public boolean isBarrier(ReaderNode node) {
        return !BarrierAccess.decidedByTrailmapRule(node) && super.isBarrier(node);
    }

    @Override
    public void handleWayTags(int edgeId, EdgeIntAccess edgeIntAccess, ReaderWay way) {
        super.handleWayTags(edgeId, edgeIntAccess, way);
        BarrierAccess.apply(getAccessEnc(), edgeId, edgeIntAccess, way, BarrierClass.Mode.BIKE);
    }
}
