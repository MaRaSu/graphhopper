/*
 * Trailmap - RoadNameHash encoded value
 *
 * Stores a 16-bit hash of the OSM road name (or ref as fallback) on each edge.
 * Used in turn-penalty evaluation to detect "same road continues" vs. "real road change":
 * matching non-zero hashes on inEdge and outEdge => same named road, no road-change penalty.
 *
 * Reserved value: 0 means "no name and no ref" (unknown). Two zero hashes are NOT considered
 * the same road, since we have no evidence of identity.
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.routing.ev.IntEncodedValue;
import com.graphhopper.routing.ev.IntEncodedValueImpl;

public final class RoadNameHash {
    public static final String KEY = "road_name_hash";
    private static final int BITS = 16;

    private RoadNameHash() {
    }

    public static IntEncodedValue create() {
        return new IntEncodedValueImpl(KEY, BITS, false);
    }
}
