/*
 * Trailmap - MtbWinterParser
 *
 * Parses the mtb:winter OSM tag and stores it as a MtbWinter encoded value.
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.reader.ReaderWay;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.ev.EdgeIntAccess;
import com.graphhopper.routing.util.parsers.TagParser;
import com.graphhopper.storage.IntsRef;

/**
 * Parser that reads mtb:winter OSM tag and encodes as MtbWinter.
 */
public class MtbWinterParser implements TagParser {

    private final EnumEncodedValue<MtbWinter> mtbWinterEnc;

    public MtbWinterParser(EnumEncodedValue<MtbWinter> mtbWinterEnc) {
        this.mtbWinterEnc = mtbWinterEnc;
    }

    @Override
    public void handleWayTags(int edgeId, EdgeIntAccess edgeIntAccess,
                              ReaderWay way, IntsRef relationFlags) {
        MtbWinter scale = computeMtbWinter(way);
        mtbWinterEnc.setEnum(false, edgeId, edgeIntAccess, scale);
    }

    public MtbWinter computeMtbWinter(ReaderWay way) {
        String mtbWinterTag = way.getTag("mtb:winter");
        return MtbWinter.fromOsmTag(mtbWinterTag);
    }
}
