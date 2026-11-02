package com.graphhopper.trailmap.barrier;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.storage.NodeAccess;

import java.io.File;
import java.io.IOException;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

/**
 * What each non-trivial barrier is, for messages: one row per barrier edge whose class is not NONE for bike or foot,
 * written into the graph folder at import ({@value #FILE}) and read by the server. Rows are keyed by the barrier
 * node's coordinates (1e-6 degrees), not by edge id: the import sorts the graph after parsing, so edge ids change, but
 * the barrier node and its copy keep their coordinates.
 */
public class BarrierTable {

    public static final String FILE = "trailmap_barriers.json";
    static final List<String> KEPT_TAGS = Arrays.asList("barrier", "access", "bicycle", "foot", "motor_vehicle",
            "motorcar", "vehicle", "locked", "opening_hours", "name");

    public static final class Row {
        public long osm_node = -1;
        public double lat, lon;
        public Map<String, String> tags = new TreeMap<>();
        public String highway;
        public boolean fenced;
        public String bike, bike_reason, foot, foot_reason;
    }

    private final Map<Long, Row> rows = new ConcurrentHashMap<>();

    static long key(double lat, double lon) {
        long a = Math.round(lat * 1e6), b = Math.round(lon * 1e6);
        return a * 1_000_000_000L + b;
    }

    void add(Row row) {
        rows.put(key(row.lat, row.lon), row);
    }

    /** One mode's class for the barrier at (lat, lon); the row is created by whichever mode comes first. */
    void record(double lat, double lon, Map<String, Object> nodeTags, String highway, BarrierClass.Mode mode,
                BarrierClassRule.Result res) {
        Row row = rows.computeIfAbsent(key(lat, lon), k -> {
            Row r = new Row();
            r.lat = lat;
            r.lon = lon;
            Object id = nodeTags.get(TrailmapOSMReader.NODE_ID_TAG);
            if (id != null)
                r.osm_node = Long.parseLong(id.toString());
            for (String t : KEPT_TAGS)
                if (nodeTags.containsKey(t))
                    r.tags.put(t, nodeTags.get(t).toString());
            r.highway = highway;
            r.fenced = "true".equals(String.valueOf(nodeTags.get(BarrierClassRule.FENCED_TAG)));
            r.bike = r.foot = BarrierClass.NONE.name();
            return r;
        });
        synchronized (row) {
            if (mode == BarrierClass.Mode.BIKE) {
                row.bike = res.cls.name();
                row.bike_reason = res.reason;
            } else {
                row.foot = res.cls.name();
                row.foot_reason = res.reason;
            }
        }
    }

    public int size() {
        return rows.size();
    }

    public Collection<Row> rows() {
        return rows.values();
    }

    private volatile Map<Long, List<Row>> noticeCells;

    /** Rows with a NOTICE class (either mode) by 0.001° cell, built once. */
    public Map<Long, List<Row>> noticeCells() {
        Map<Long, List<Row>> m = noticeCells;
        if (m == null) {
            m = new HashMap<>();
            for (Row r : rows.values())
                if (BarrierClass.NOTICE.name().equals(r.bike) || BarrierClass.NOTICE.name().equals(r.foot))
                    m.computeIfAbsent(cell(r.lat, r.lon), k -> new ArrayList<>()).add(r);
            noticeCells = m;
        }
        return m;
    }

    public static long cell(double lat, double lon) {
        return (long) Math.floor(lat * 1000) * 1_000_000L + (long) Math.floor(lon * 1000);
    }

    /** The row of a barrier edge (its base node's coordinates), or null. */
    public Row lookup(EdgeIteratorState barrierEdge, NodeAccess na) {
        return rows.get(key(na.getLat(barrierEdge.getBaseNode()), na.getLon(barrierEdge.getBaseNode())));
    }

    public void write(File graphDir) throws IOException {
        List<Row> sorted = new ArrayList<>(rows.values());
        sorted.sort(Comparator.comparingDouble((Row r) -> r.lat).thenComparingDouble(r -> r.lon));
        new ObjectMapper().disable(SerializationFeature.INDENT_OUTPUT).writeValue(new File(graphDir, FILE), sorted);
    }

    /** Reads the table of a graph folder; an empty table if the graph has none (an import without barrier classes). */
    private static final Map<String, BarrierTable> BY_FOLDER = new ConcurrentHashMap<>();

    /**
     * The table of a GraphHopper's graph: a TrailmapGraphHopper's own (read once when the graph loads); for any other
     * GraphHopper (tests), read once per graph folder.
     */
    public static BarrierTable of(com.graphhopper.GraphHopper hopper) {
        if (hopper instanceof com.graphhopper.trailmap.TrailmapGraphHopper)
            return ((com.graphhopper.trailmap.TrailmapGraphHopper) hopper).barrierTable();
        return BY_FOLDER.computeIfAbsent(hopper.getGraphHopperLocation(), k -> read(new File(k)));
    }

    public static BarrierTable read(File graphDir) {
        BarrierTable t = new BarrierTable();
        File f = new File(graphDir, FILE);
        if (!f.exists())
            return t;
        try {
            for (Row r : new ObjectMapper().readValue(f, Row[].class))
                t.add(r);
        } catch (IOException e) {
            throw new IllegalStateException("malformed or truncated barrier table " + f
                    + " (written by the import; re-import the graph): " + e.getMessage(), e);
        }
        return t;
    }
}
