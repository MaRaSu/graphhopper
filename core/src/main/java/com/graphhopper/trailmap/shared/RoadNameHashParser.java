/*
 * Trailmap - RoadNameHashParser
 *
 * Computes a 16-bit hash of the OSM road name (preferred) or ref (fallback) and stores it
 * via the road_name_hash IntEncodedValue. Sentinel 0 = "no name and no ref".
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.reader.ReaderWay;
import com.graphhopper.routing.ev.EdgeIntAccess;
import com.graphhopper.routing.ev.IntEncodedValue;
import com.graphhopper.routing.util.parsers.TagParser;
import com.graphhopper.storage.IntsRef;

public class RoadNameHashParser implements TagParser {

    private final IntEncodedValue roadNameHashEnc;

    public RoadNameHashParser(IntEncodedValue roadNameHashEnc) {
        this.roadNameHashEnc = roadNameHashEnc;
    }

    @Override
    public void handleWayTags(int edgeId, EdgeIntAccess edgeIntAccess, ReaderWay way, IntsRef relationFlags) {
        String key = pickIdentity(way.getTag("name"), way.getTag("ref"));
        roadNameHashEnc.setInt(false, edgeId, edgeIntAccess, computeHash(key));
    }

    static String pickIdentity(String name, String ref) {
        if (name != null && !name.isEmpty()) return name;
        if (ref != null && !ref.isEmpty()) return ref;
        return null;
    }

    static int computeHash(String key) {
        if (key == null) return 0;
        int h = key.hashCode() & 0xFFFF;
        // reserve 0 for "unknown"; remap collisions to 1
        return h == 0 ? 1 : h;
    }
}
