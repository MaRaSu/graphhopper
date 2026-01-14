/*
 * Trailmap - GravelScaleNumParser
 *
 * Computes the numeric GravelScale value for custom model formulas.
 * Uses GravelScaleParser logic internally, converts result to numeric.
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.reader.ReaderWay;
import com.graphhopper.routing.ev.DecimalEncodedValue;
import com.graphhopper.routing.ev.EdgeIntAccess;
import com.graphhopper.routing.util.parsers.TagParser;
import com.graphhopper.storage.IntsRef;

/**
 * Parser that computes gravel_scale_num from OSM tags.
 * Reuses GravelScaleParser logic and converts to numeric value.
 */
public class GravelScaleNumParser implements TagParser {

    private final DecimalEncodedValue gravelScaleNumEnc;
    private final GravelScaleParser gravelScaleParser;

    public GravelScaleNumParser(DecimalEncodedValue gravelScaleNumEnc) {
        this.gravelScaleNumEnc = gravelScaleNumEnc;
        // Null encoder is intentional - we only use computeGravelScale(), not handleWayTags()
        this.gravelScaleParser = new GravelScaleParser(null);
    }

    @Override
    public void handleWayTags(int edgeId, EdgeIntAccess edgeIntAccess, ReaderWay way, IntsRef relationFlags) {
        GravelScale scale = gravelScaleParser.computeGravelScale(way);
        double numericValue = GravelScaleNum.getNumericValue(scale);
        gravelScaleNumEnc.setDecimal(false, edgeId, edgeIntAccess, numericValue);
    }
}
