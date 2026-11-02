package com.graphhopper.trailmap.barrier;

import com.graphhopper.reader.ReaderWay;
import com.graphhopper.routing.ev.EdgeIntAccess;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.util.parsers.TagParser;
import com.graphhopper.storage.IntsRef;
import com.graphhopper.util.PointList;

import java.util.Collections;
import java.util.List;
import java.util.Map;

/**
 * Writes {@code bike_barrier} or {@code foot_barrier} (one parser per mode) on barrier edges, the 0 m edge GraphHopper
 * adds at a barrier node that is not a junction, and adds that mode's class to the {@link BarrierTable} row. The access
 * bits are set by the access parsers with the same rule ({@link BarrierAccess}); this parser only records the class.
 */
public class BarrierClassParser implements TagParser {

    private final EnumEncodedValue<BarrierClass> enc;
    private final BarrierClass.Mode mode;
    private final BarrierTable table;

    public BarrierClassParser(EnumEncodedValue<BarrierClass> enc, BarrierClass.Mode mode, BarrierTable table) {
        this.enc = enc;
        this.mode = mode;
        this.table = table;
    }

    @Override
    public void handleWayTags(int edgeId, EdgeIntAccess edgeIntAccess, ReaderWay way, IntsRef relationFlags) {
        if (!way.hasTag("gh:barrier_edge"))
            return;
        List<Map<String, Object>> nodeTags = way.getTag("node_tags", Collections.emptyList());
        if (nodeTags.isEmpty() || !nodeTags.get(0).containsKey("barrier"))
            return;
        Map<String, Object> tags = nodeTags.get(0);
        String highway = way.getTag("highway");
        BarrierClassRule.Result res = BarrierClassRule.classify(tags, highway, mode);
        enc.setEnum(false, edgeId, edgeIntAccess, res.cls);
        if (table == null || res.cls == BarrierClass.NONE)
            return;
        PointList pl = way.getTag("point_list", null);
        if (pl == null || pl.isEmpty())
            return;
        table.record(pl.getLat(0), pl.getLon(0), tags, highway, mode, res);
    }
}
