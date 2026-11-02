package com.graphhopper.trailmap.barrier;

import com.carrotsearch.hppc.LongHashSet;
import com.carrotsearch.hppc.cursors.LongCursor;
import com.graphhopper.reader.ReaderNode;
import com.graphhopper.reader.ReaderWay;
import com.graphhopper.reader.osm.OSMReader;
import com.graphhopper.routing.util.OSMParsers;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.routing.OSMReaderConfig;

/**
 * OSMReader that tells the barrier rule two things GraphHopper doesn't read: the OSM id of a barrier node and whether
 * the node is part of a mapped fence, wall or hedge.
 * <ul>
 * <li>Pass 1 offers every way to {@link #acceptWay}: the node ids of barrier ways ({@code barrier=fence|wall|hedge|
 * city_wall|retaining_wall}) are remembered, then the way is judged as usual. Pass 1 reads all ways before pass 2 reads
 * any node, so the set is complete when it is needed.</li>
 * <li>Pass 2 asks {@link #isBarrierNode} for every mapped node before its tags are stored: a barrier node gets the
 * artificial tags {@code gh:osm_node_id} and, when it is on such a way, {@code gh:fenced=true}. The answer itself is
 * unchanged.</li>
 * </ul>
 * Nothing else changes: the same ways are accepted and the same nodes are split.
 */
public class TrailmapOSMReader extends OSMReader {

    public static final String NODE_ID_TAG = "gh:osm_node_id";
    private static final String[] FENCE_VALUES = {"fence", "wall", "hedge", "city_wall", "retaining_wall"};

    private final LongHashSet fenceNodes = new LongHashSet();

    public TrailmapOSMReader(BaseGraph baseGraph, OSMParsers osmParsers, OSMReaderConfig config) {
        super(baseGraph, osmParsers, config);
    }

    @Override
    protected boolean acceptWay(ReaderWay way) {
        if (way.hasTag("barrier", FENCE_VALUES))
            for (LongCursor c : way.getNodes())
                fenceNodes.add(c.value);
        return super.acceptWay(way);
    }

    @Override
    protected boolean isBarrierNode(ReaderNode node) {
        if (node.hasTag("barrier")) {
            node.setTag(NODE_ID_TAG, Long.toString(node.getId()));
            if (fenceNodes.contains(node.getId()))
                node.setTag(BarrierClassRule.FENCED_TAG, "true"); // node tags are stored as strings
        }
        return super.isBarrierNode(node);
    }

    public int fenceNodeCount() {
        return fenceNodes.size();
    }
}
