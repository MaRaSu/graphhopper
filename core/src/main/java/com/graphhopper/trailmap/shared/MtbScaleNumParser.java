/*
 * Trailmap - MtbScaleNumParser
 *
 * Computes the numeric MtbScale value for custom model formulas.
 * Uses MtbScaleParser logic internally, converts result to numeric.
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.reader.ReaderWay;
import com.graphhopper.routing.ev.DecimalEncodedValue;
import com.graphhopper.routing.ev.EdgeIntAccess;
import com.graphhopper.routing.util.parsers.TagParser;
import com.graphhopper.storage.IntsRef;

/**
 * Parser that computes mtb_scale_num from OSM tags.
 * Reuses MtbScaleParser logic and converts to numeric value.
 */
public class MtbScaleNumParser implements TagParser {

    private final DecimalEncodedValue mtbScaleNumEnc;
    private final MtbScaleParser mtbScaleParser;

    public MtbScaleNumParser(DecimalEncodedValue mtbScaleNumEnc) {
        this.mtbScaleNumEnc = mtbScaleNumEnc;
        // Null encoder is intentional - we only use computeMtbScale(), not handleWayTags()
        this.mtbScaleParser = new MtbScaleParser(null);
    }

    @Override
    public void handleWayTags(int edgeId, EdgeIntAccess edgeIntAccess, ReaderWay way, IntsRef relationFlags) {
        MtbScale scale = mtbScaleParser.computeMtbScale(way);
        double numericValue = MtbScaleNum.getNumericValue(scale);
        mtbScaleNumEnc.setDecimal(false, edgeId, edgeIntAccess, numericValue);
    }
}
