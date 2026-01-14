/*
 * Trailmap - MtbBasePriorityParser
 *
 * Computes the base priority for MTB routing profile.
 * Uses MtbScaleParser logic internally, converts to priority.
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.reader.ReaderWay;
import com.graphhopper.routing.ev.DecimalEncodedValue;
import com.graphhopper.routing.ev.EdgeIntAccess;
import com.graphhopper.routing.util.parsers.TagParser;
import com.graphhopper.storage.IntsRef;

/**
 * Parser that computes mtb_base_priority from OSM tags.
 * Reuses MtbScaleParser logic and converts to priority value.
 */
public class MtbBasePriorityParser implements TagParser {

    private final DecimalEncodedValue priorityEnc;
    private final MtbScaleParser mtbScaleParser;

    public MtbBasePriorityParser(DecimalEncodedValue priorityEnc) {
        this.priorityEnc = priorityEnc;
        // Null encoder is intentional - we only use computeMtbScale(), not handleWayTags()
        this.mtbScaleParser = new MtbScaleParser(null);
    }

    @Override
    public void handleWayTags(int edgeId, EdgeIntAccess edgeIntAccess, ReaderWay way, IntsRef relationFlags) {
        MtbScale scale = mtbScaleParser.computeMtbScale(way);
        double priority = MtbBasePriority.getPriority(scale);
        priorityEnc.setDecimal(false, edgeId, edgeIntAccess, priority);
    }
}
