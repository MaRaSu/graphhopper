package com.graphhopper.trailmap.barrier;

import com.graphhopper.reader.ReaderNode;
import com.graphhopper.reader.ReaderWay;
import com.graphhopper.routing.ev.BooleanEncodedValue;
import com.graphhopper.routing.ev.EdgeIntAccess;

import java.util.Collections;
import java.util.List;
import java.util.Map;

/**
 * The access side of the barrier rule, for the Trailmap bike and foot access parsers: on a barrier edge whose node has
 * a {@code barrier} tag, the class from {@link BarrierClassRule} replaces GraphHopper's node rule
 * (AbstractAccessParser.isBarrier): ASK and REFUSE close the edge both ways, NONE and NOTICE leave the way's own
 * access as it is. Nodes without a {@code barrier} tag (fords) keep the stock rule.
 */
public final class BarrierAccess {

    private BarrierAccess() {
    }

    /** True when the stock node rule must not decide this node (it carries a barrier tag). */
    public static boolean decidedByTrailmapRule(ReaderNode node) {
        return node.hasTag("barrier");
    }

    public static void apply(BooleanEncodedValue accessEnc, int edgeId, EdgeIntAccess edgeIntAccess, ReaderWay way,
                             BarrierClass.Mode mode) {
        if (!way.hasTag("gh:barrier_edge"))
            return;
        List<Map<String, Object>> nodeTags = way.getTag("node_tags", Collections.emptyList());
        if (nodeTags.isEmpty() || !nodeTags.get(0).containsKey("barrier"))
            return;
        if (BarrierClassRule.classify(nodeTags.get(0), way.getTag("highway"), mode).cls.closes()) {
            accessEnc.setBool(false, edgeId, edgeIntAccess, false);
            accessEnc.setBool(true, edgeId, edgeIntAccess, false);
        }
    }
}
