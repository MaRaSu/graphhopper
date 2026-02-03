/*
 * Trailmap - PisteTypeParser
 *
 * Parses the piste:type OSM tag and stores it as a PisteType encoded value.
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.reader.ReaderWay;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.ev.EdgeIntAccess;
import com.graphhopper.routing.util.parsers.TagParser;
import com.graphhopper.storage.IntsRef;

/**
 * Parser that reads the piste:type OSM tag and encodes it as PisteType.
 */
public class PisteTypeParser implements TagParser {

    private final EnumEncodedValue<PisteType> pisteTypeEnc;

    public PisteTypeParser(EnumEncodedValue<PisteType> pisteTypeEnc) {
        this.pisteTypeEnc = pisteTypeEnc;
    }

    @Override
    public void handleWayTags(int edgeId, EdgeIntAccess edgeIntAccess,
                              ReaderWay way, IntsRef relationFlags) {
        PisteType type = computePisteType(way);
        pisteTypeEnc.setEnum(false, edgeId, edgeIntAccess, type);
    }

    public PisteType computePisteType(ReaderWay way) {
        String pisteType = way.getTag("piste:type");
        return PisteType.fromOsmTag(pisteType);
    }
}
