/*
 * Trailmap - Gravel Segment Analysis
 *
 * Phase F: emit the two JSON artifacts (way-IDs only). Geometry is joined downstream in
 * PostGIS keyed by OSM way ID. See docs/gravel_segments_design.md §11.
 */
package com.graphhopper.trailmap.analysis.gravel.output;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.graphhopper.trailmap.analysis.gravel.group.LogicalRoadGrouper;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

/**
 * Writes {@code qualifying_ways.json} (flat distinct way-ID list) and
 * {@code logical_roads.json} (contiguous same-name runs). Way IDs are emitted as JSON
 * numbers (stored as 31-bit ints; safe for today's OSM way IDs).
 */
public class GravelOutputWriter {

    private final ObjectMapper mapper = new ObjectMapper().enable(
            com.fasterxml.jackson.databind.SerializationFeature.INDENT_OUTPUT);

    /** Write the flat qualifying-ways artifact. */
    public void writeQualifyingWays(File file, LogicalRoadGrouper.Result result) throws IOException {
        Map<String, Object> root = new LinkedHashMap<>();
        root.put("way_ids", new ArrayList<>(result.allWayIds));
        mapper.writeValue(file, root);
    }

    /** Write the grouped logical-road artifact. */
    public void writeLogicalRoads(File file, LogicalRoadGrouper.Result result) throws IOException {
        // Deterministic order: by name, then ref, then first way id.
        List<LogicalRoadGrouper.LogicalRoad> roads = new ArrayList<>(result.roads);
        roads.sort(Comparator
                .comparing((LogicalRoadGrouper.LogicalRoad r) -> r.name == null ? "" : r.name)
                .thenComparing(r -> r.ref == null ? "" : r.ref)
                .thenComparing(r -> r.wayIds.isEmpty() ? Long.MIN_VALUE : r.wayIds.first()));

        List<Map<String, Object>> roadList = new ArrayList<>(roads.size());
        for (LogicalRoadGrouper.LogicalRoad r : roads) {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("name", r.name);
            m.put("ref", r.ref);
            m.put("way_ids", new ArrayList<>(r.wayIds));
            m.put("qualifying_length_m", round1(r.qualifyingLengthM));
            roadList.add(m);
        }
        Map<String, Object> root = new LinkedHashMap<>();
        root.put("roads", roadList);
        mapper.writeValue(file, root);
    }

    /**
     * Write per-way styling attributes (way id → {@code gravel_scale}, {@code predicted_highway},
     * {@code predicted_surface}), restricted to the retained qualifying ways. Consumed downstream
     * only for map styling / verification; not part of the way-ID contract.
     */
    public void writeWayAttributes(File file, Set<Long> wayIds, Map<Long, String[]> attrs)
            throws IOException {
        // Sorted by way id for deterministic, diff-friendly output.
        Map<String, Object> ways = new TreeMap<>(Comparator.comparingLong(Long::parseLong));
        for (Long wid : wayIds) {
            String[] a = attrs.get(wid);
            if (a == null) continue;
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("gravel_scale", a[0]);
            m.put("predicted_highway", a[1]);
            m.put("predicted_surface", a[2]);
            ways.put(Long.toString(wid), m);
        }
        Map<String, Object> root = new LinkedHashMap<>();
        root.put("ways", ways);
        mapper.writeValue(file, root);
    }

    /**
     * Write the retained sub-segments per way as tower-node coordinates (partial-way output).
     * Shape: {@code { "ways": { "<wayId>": [[latA,lonA,latB,lonB], ...] } }} — one entry per
     * retained gravel edge, its two junction (tower-node) coordinates at 7 decimals (= OSM
     * precision). A way is partially retained when only some of its segments survive; downstream
     * matches these coords to OSM node ids / cuts the way's linestring with them. GraphHopper
     * discards OSM node ids at import, hence coordinates rather than node ids.
     */
    public void writeRetainedSegments(File file, Map<Long, List<double[]>> segments) throws IOException {
        Map<String, Object> ways = new TreeMap<>(Comparator.comparingLong(Long::parseLong));
        for (Map.Entry<Long, List<double[]>> en : segments.entrySet()) {
            List<List<Double>> segs = new ArrayList<>(en.getValue().size());
            for (double[] s : en.getValue()) {
                List<Double> one = new ArrayList<>(4);
                for (double v : s) one.add(v);
                segs.add(one);
            }
            ways.put(Long.toString(en.getKey()), segs);
        }
        Map<String, Object> root = new LinkedHashMap<>();
        root.put("ways", ways);
        mapper.writeValue(file, root);
    }

    private static double round1(double v) {
        return Math.round(v * 10.0) / 10.0;
    }
}
