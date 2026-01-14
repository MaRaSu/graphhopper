/*
 * Trailmap - GravelBasePriorityParser
 *
 * Computes the base priority for gravel routing profile.
 * Uses GravelScaleParser logic internally, converts to priority.
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.reader.ReaderWay;
import com.graphhopper.routing.ev.DecimalEncodedValue;
import com.graphhopper.routing.ev.EdgeIntAccess;
import com.graphhopper.routing.util.parsers.TagParser;
import com.graphhopper.storage.IntsRef;

/**
 * Parser that computes gravel_base_priority from OSM tags.
 * Reuses GravelScaleParser logic and converts to priority value.
 */
public class GravelBasePriorityParser implements TagParser {

    private final DecimalEncodedValue priorityEnc;
    private final GravelScaleParser gravelScaleParser;

    public GravelBasePriorityParser(DecimalEncodedValue priorityEnc) {
        this.priorityEnc = priorityEnc;
        // Null encoder is intentional - we only use computeGravelScale(), not handleWayTags()
        this.gravelScaleParser = new GravelScaleParser(null);
    }

    @Override
    public void handleWayTags(int edgeId, EdgeIntAccess edgeIntAccess, ReaderWay way, IntsRef relationFlags) {
        GravelScale scale = gravelScaleParser.computeGravelScale(way);
        double priority = GravelBasePriority.getPriority(scale);
        priorityEnc.setDecimal(false, edgeId, edgeIntAccess, priority);
    }
}
