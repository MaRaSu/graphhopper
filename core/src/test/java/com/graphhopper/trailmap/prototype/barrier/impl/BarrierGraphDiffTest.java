package com.graphhopper.trailmap.prototype.barrier.impl;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.jackson.Jackson;
import com.graphhopper.routing.ev.*;
import com.graphhopper.routing.util.AllEdgesIterator;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.trailmap.TrailmapGraphHopper;
import com.graphhopper.trailmap.barrier.BarrierClass;
import com.graphhopper.trailmap.barrier.BarrierTable;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.FetchMode;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.*;

/**
 * W26 S1/S2 critical validation (CATALOG §8): the whole-graph encoded-value diff between a "before" graph (current
 * fork) and an "after" graph (barrier classes), same pbf, same config. Edges are matched by geometry (both end
 * coordinates at 1e-6, distance in cm, pillar count), never by edge id. Every encoded value of the before graph is
 * compared on every matched edge, both directions; the after graph's new values are summarised separately.
 * <pre>
 * -Dbd.before=GRAPHDIR -Dbd.after=GRAPHDIR -Dbd.config=import-config.yml -Dbd.out=DIR
 * </pre>
 * Read-only (setAllowWrites(false)). Prints a summary and writes diff.json.
 */
public class BarrierGraphDiffTest {

    static final ObjectMapper JSON = Jackson.newObjectMapper();

    static TrailmapGraphHopper open(String graph, String config) throws Exception {
        ObjectMapper yaml = Jackson.initObjectMapper(new ObjectMapper(new YAMLFactory()));
        GraphHopperConfig c = yaml.treeToValue(yaml.readTree(new File(config)).get("graphhopper"), GraphHopperConfig.class);
        c.putObject("graph.location", graph);
        TrailmapGraphHopper h = new TrailmapGraphHopper();
        h.init(c);
        h.setAllowWrites(false);
        h.load();
        return h;
    }

    static String key(EdgeIteratorState e, NodeAccess na) {
        int a = e.getBaseNode(), b = e.getAdjNode();
        String ka = Math.round(na.getLat(a) * 1e6) + "," + Math.round(na.getLon(a) * 1e6);
        String kb = Math.round(na.getLat(b) * 1e6) + "," + Math.round(na.getLon(b) * 1e6);
        int pillars = e.fetchWayGeometry(FetchMode.PILLAR_ONLY).size();
        long cm = Math.round(e.getDistance() * 100);
        // undirected key: the same edge may be stored the other way round
        return (ka.compareTo(kb) <= 0 ? ka + "|" + kb : kb + "|" + ka) + "|" + cm + "|" + pillars;
    }

    static boolean reversedVs(EdgeIteratorState e, NodeAccess na) {
        String ka = Math.round(na.getLat(e.getBaseNode()) * 1e6) + "," + Math.round(na.getLon(e.getBaseNode()) * 1e6);
        String kb = Math.round(na.getLat(e.getAdjNode()) * 1e6) + "," + Math.round(na.getLon(e.getAdjNode()) * 1e6);
        return ka.compareTo(kb) > 0;
    }

    /** value of ev in travel direction "forward along the key" (from the smaller to the larger end coordinate) */
    static String val(EdgeIteratorState e, EncodedValue ev, boolean keyReverse, boolean dir) {
        boolean rev = keyReverse ^ dir;
        if (ev instanceof BooleanEncodedValue)
            return String.valueOf(rev ? e.getReverse((BooleanEncodedValue) ev) : e.get((BooleanEncodedValue) ev));
        if (ev instanceof IntEncodedValue)
            return String.valueOf(rev ? e.getReverse((IntEncodedValue) ev) : e.get((IntEncodedValue) ev));
        if (ev instanceof DecimalEncodedValue)
            return String.valueOf(rev ? e.getReverse((DecimalEncodedValue) ev) : e.get((DecimalEncodedValue) ev));
        if (ev instanceof EnumEncodedValue) {
            @SuppressWarnings({"unchecked", "rawtypes"})
            EnumEncodedValue<?> en = (EnumEncodedValue) ev;
            return String.valueOf(rev ? e.getReverse(en) : e.get(en));
        }
        if (ev instanceof StringEncodedValue)
            return String.valueOf(rev ? e.getReverse((StringEncodedValue) ev) : e.get((StringEncodedValue) ev));
        return "?" + ev.getClass().getSimpleName();
    }

    @Test
    void diff() throws Exception {
        String before = System.getProperty("bd.before"), after = System.getProperty("bd.after");
        Assumptions.assumeTrue(before != null && after != null, "bd.before / bd.after not set");
        String config = System.getProperty("bd.config");
        File outDir = new File(System.getProperty("bd.out"));
        outDir.mkdirs();
        TrailmapGraphHopper hb = open(before, config), ha = open(after, config);
        BaseGraph gb = hb.getBaseGraph(), ga = ha.getBaseGraph();
        NodeAccess nb = gb.getNodeAccess(), na = ga.getNodeAccess();
        System.out.println("before " + hb.getProperties().get("datareader.data.date") + " nodes " + gb.getNodes() + " edges " + gb.getEdges());
        System.out.println("after  " + ha.getProperties().get("datareader.data.date") + " nodes " + ga.getNodes() + " edges " + ga.getEdges());

        Map<String, List<Integer>> beforeEdges = new HashMap<>();
        AllEdgesIterator it = gb.getAllEdges();
        while (it.next())
            beforeEdges.computeIfAbsent(key(it, nb), k -> new ArrayList<>()).add(it.getEdge());

        List<EncodedValue> evs = new ArrayList<>(hb.getEncodingManager().getEncodedValues());
        List<String> newEvs = new ArrayList<>();
        for (EncodedValue ev : ha.getEncodingManager().getEncodedValues())
            if (!hb.getEncodingManager().hasEncodedValue(ev.getName()))
                newEvs.add(ev.getName());
        Map<String, Integer> diffCount = new TreeMap<>();
        ArrayNode accessDiffs = JSON.createArrayNode();
        int matched = 0, unmatchedAfter = 0, idSame = 0;
        Set<Integer> usedBefore = new HashSet<>();
        EnumEncodedValue<BarrierClass> bikeEnc = ha.getEncodingManager().hasEncodedValue(BarrierClass.KEY_BIKE)
                ? ha.getEncodingManager().getEnumEncodedValue(BarrierClass.KEY_BIKE, BarrierClass.class) : null;
        EnumEncodedValue<BarrierClass> footEnc = ha.getEncodingManager().hasEncodedValue(BarrierClass.KEY_FOOT)
                ? ha.getEncodingManager().getEnumEncodedValue(BarrierClass.KEY_FOOT, BarrierClass.class) : null;
        BarrierTable table = BarrierTable.read(new File(after));
        Map<String, Integer> classCounts = new TreeMap<>();
        Map<String, Integer> kvDiff = new TreeMap<>();

        AllEdgesIterator ia = ga.getAllEdges();
        while (ia.next()) {
            String k = key(ia, na);
            List<Integer> cands = beforeEdges.get(k);
            Integer be = null;
            if (cands != null)
                for (Integer c : cands)
                    if (!usedBefore.contains(c)) {
                        be = c;
                        break;
                    }
            boolean barrierEdge = ia.getDistance() < 0.01 && nb != null && na.getLat(ia.getBaseNode()) == na.getLat(ia.getAdjNode())
                    && na.getLon(ia.getBaseNode()) == na.getLon(ia.getAdjNode());
            if (barrierEdge) {
                if (bikeEnc != null)
                    classCounts.merge("bike:" + ia.get(bikeEnc), 1, Integer::sum);
                if (footEnc != null)
                    classCounts.merge("foot:" + ia.get(footEnc), 1, Integer::sum);
                // a closing class whose access is open anyway (e.g. reopened by the area access parser)
                BooleanEncodedValue bA = ha.getEncodingManager().getBooleanEncodedValue("bike_access");
                BooleanEncodedValue fA = ha.getEncodingManager().getBooleanEncodedValue("foot_access");
                if (bikeEnc != null && ia.get(bikeEnc).closes() && (ia.get(bA) || ia.getReverse(bA)))
                    classCounts.merge("bike:CLOSING_CLASS_BUT_ACCESS_OPEN", 1, Integer::sum);
                if (footEnc != null && ia.get(footEnc).closes() && (ia.get(fA) || ia.getReverse(fA)))
                    classCounts.merge("foot:CLOSING_CLASS_BUT_ACCESS_OPEN", 1, Integer::sum);
            }
            if (be == null) {
                unmatchedAfter++;
                continue;
            }
            usedBefore.add(be);
            matched++;
            if (be == ia.getEdge())
                idSame++;
            EdgeIteratorState eb = gb.getEdgeIteratorState(be, Integer.MIN_VALUE);
            boolean rb = reversedVs(eb, nb), ra = reversedVs(ia, na);
            if (!String.valueOf(new TreeMap<>(eb.getKeyValues())).equals(String.valueOf(new TreeMap<>(ia.getKeyValues()))))
                kvDiff.merge("key_values", 1, Integer::sum);
            for (EncodedValue ev : evs) {
                EncodedValue evA = ha.getEncodingManager().getEncodedValue(ev.getName(), EncodedValue.class);
                for (boolean dir : new boolean[]{false, true}) {
                    if (dir && !ev.isStoreTwoDirections())
                        continue;
                    String vb = val(eb, ev, rb, dir), va = val(ia, evA, ra, dir);
                    if (!vb.equals(va)) {
                        diffCount.merge(ev.getName(), 1, Integer::sum);
                        if (ev.getName().endsWith("_access")) {
                            ObjectNode d = accessDiffs.addObject();
                            d.put("ev", ev.getName());
                            d.put("dir", dir ? "bwd" : "fwd");
                            d.put("before", vb);
                            d.put("after", va);
                            d.put("barrier_edge", barrierEdge);
                            d.put("lat", na.getLat(ia.getBaseNode()));
                            d.put("lon", na.getLon(ia.getBaseNode()));
                            if (bikeEnc != null)
                                d.put("bike_class", String.valueOf(ia.get(bikeEnc)));
                            if (footEnc != null)
                                d.put("foot_class", String.valueOf(ia.get(footEnc)));
                            BarrierTable.Row row = table.lookup(ia, na);
                            if (row != null) {
                                d.put("osm_node", row.osm_node);
                                d.put("bike_reason", row.bike_reason);
                                d.put("foot_reason", row.foot_reason);
                                d.putPOJO("tags", row.tags);
                            }
                        }
                    }
                }
            }
        }
        int unmatchedBefore = gb.getEdges() - usedBefore.size();

        // subnetwork markings: stock (must be identical, counted in diffCount) vs barrier-aware (new)
        Map<String, int[]> marking = new TreeMap<>();
        for (String ev : newEvs) {
            if (!ev.endsWith("_subnetwork_b"))
                continue;
            String stock = ev.substring(0, ev.length() - 2);
            BooleanEncodedValue b = ha.getEncodingManager().getBooleanEncodedValue(ev);
            BooleanEncodedValue s = ha.getEncodingManager().getBooleanEncodedValue(stock);
            // stock marked, b marked, marked in stock only, marked in b only, of those b-only: barrier edges
            int[] c = new int[5];
            AllEdgesIterator i2 = ga.getAllEdges();
            while (i2.next()) {
                boolean sm = i2.get(s), bm = i2.get(b);
                if (sm) c[0]++;
                if (bm) c[1]++;
                if (sm && !bm) c[2]++;
                if (bm && !sm) {
                    c[3]++;
                    if (i2.getDistance() < 0.01 && na.getLat(i2.getBaseNode()) == na.getLat(i2.getAdjNode())
                            && na.getLon(i2.getBaseNode()) == na.getLon(i2.getAdjNode()))
                        c[4]++;
                }
            }
            marking.put(ev, c);
        }

        // every change of a stock <profile>_subnetwork value must sit in a piece of network that touches a barrier
        // edge whose access changed (S1 moves islands only there). Edge ids are compared directly (checked identical).
        Map<String, int[]> stockExplained = new TreeMap<>();
        if (idSame == gb.getEdges() && gb.getEdges() == ga.getEdges()) {
            Set<Integer> changedBarrierEdges = new HashSet<>();
            Map<String, Set<Integer>> changedByEv = new HashMap<>();
            AllEdgesIterator i3 = ga.getAllEdges();
            for (String accessKey : new String[]{"bike_access", "foot_access"}) {
                BooleanEncodedValue accB = hb.getEncodingManager().getBooleanEncodedValue(accessKey);
                BooleanEncodedValue accA = ha.getEncodingManager().getBooleanEncodedValue(accessKey);
                Set<Integer> ch = new HashSet<>();
                i3 = ga.getAllEdges();
                while (i3.next()) {
                    EdgeIteratorState eb = gb.getEdgeIteratorState(i3.getEdge(), Integer.MIN_VALUE);
                    if (eb.get(accB) != i3.get(accA) || eb.getReverse(accB) != i3.getReverse(accA))
                        ch.add(i3.getEdge());
                }
                changedByEv.put(accessKey, ch);
                changedBarrierEdges.addAll(ch);
            }
            for (String profile : new String[]{"gravel", "trailmap_foot"}) {
                String accessKey = profile.contains("foot") ? "foot_access" : "bike_access";
                BooleanEncodedValue sB = hb.getEncodingManager().getBooleanEncodedValue(Subnetwork.key(profile));
                BooleanEncodedValue sA = ha.getEncodingManager().getBooleanEncodedValue(Subnetwork.key(profile));
                BooleanEncodedValue accB = hb.getEncodingManager().getBooleanEncodedValue(accessKey);
                BooleanEncodedValue accA = ha.getEncodingManager().getBooleanEncodedValue(accessKey);
                Set<Integer> targets = changedByEv.get(accessKey);
                int[] c = new int[3]; // changed, explained, unexplained
                com.graphhopper.util.EdgeExplorer ex = ga.createEdgeExplorer();
                i3 = ga.getAllEdges();
                while (i3.next()) {
                    EdgeIteratorState eb = gb.getEdgeIteratorState(i3.getEdge(), Integer.MIN_VALUE);
                    if (eb.get(sB) == i3.get(sA))
                        continue;
                    c[0]++;
                    // BFS over edges usable in either graph, up to 20,000 edges
                    ArrayDeque<Integer> q = new ArrayDeque<>(Arrays.asList(i3.getBaseNode(), i3.getAdjNode()));
                    Set<Integer> seenN = new HashSet<>(q), seenE = new HashSet<>();
                    boolean found = targets.contains(i3.getEdge());
                    while (!found && !q.isEmpty() && seenE.size() < 20000) {
                        com.graphhopper.util.EdgeIterator e2 = ex.setBaseNode(q.poll());
                        while (e2.next()) {
                            if (!seenE.add(e2.getEdge()))
                                continue;
                            if (targets.contains(e2.getEdge())) {
                                found = true;
                                break;
                            }
                            EdgeIteratorState o = gb.getEdgeIteratorState(e2.getEdge(), Integer.MIN_VALUE);
                            boolean usable = e2.get(accA) || e2.getReverse(accA) || o.get(accB) || o.getReverse(accB);
                            if (usable && seenN.add(e2.getAdjNode()))
                                q.add(e2.getAdjNode());
                        }
                    }
                    c[found ? 1 : 2]++;
                }
                stockExplained.put(Subnetwork.key(profile), c);
            }
        }

        // summary of access changes per class, opened vs closed
        Map<String, Integer> accessSummary = new TreeMap<>();
        for (int i = 0; i < accessDiffs.size(); i++) {
            ObjectNode d = (ObjectNode) accessDiffs.get(i);
            String ev = d.get("ev").asText();
            String cls = ev.startsWith("bike") ? d.path("bike_class").asText() : ev.startsWith("foot") ? d.path("foot_class").asText() : "-";
            String dirn = "true".equals(d.get("after").asText()) ? "opened" : "closed";
            accessSummary.merge(ev + " " + dirn + " " + cls + (d.get("barrier_edge").asBoolean() ? "" : " NOT-A-BARRIER-EDGE"), 1, Integer::sum);
        }

        ObjectNode out = JSON.createObjectNode();
        out.put("before_edges", gb.getEdges());
        out.put("after_edges", ga.getEdges());
        out.put("matched", matched);
        out.put("same_edge_id", idSame);
        out.put("unmatched_after", unmatchedAfter);
        out.put("unmatched_before", unmatchedBefore);
        out.putPOJO("ev_diff_counts", diffCount);
        out.putPOJO("key_value_diffs", kvDiff);
        out.putPOJO("new_evs", newEvs);
        out.putPOJO("barrier_class_counts", classCounts);
        out.putPOJO("access_summary", accessSummary);
        out.putPOJO("subnetwork_b [stock, b, stock_only, b_only, b_only_barrier_edges]", marking);
        out.putPOJO("stock_subnetwork_changes [changed, explained, unexplained]", stockExplained);
        out.put("table_rows", table.size());
        out.set("access_diffs", accessDiffs);
        JSON.writerWithDefaultPrettyPrinter().writeValue(new File(outDir, "diff.json"), out);
        ObjectNode summary = out.deepCopy();
        summary.remove("access_diffs");
        System.out.println("DIFF " + JSON.writerWithDefaultPrettyPrinter().writeValueAsString(summary));
        hb.close();
        ha.close();
    }
}
