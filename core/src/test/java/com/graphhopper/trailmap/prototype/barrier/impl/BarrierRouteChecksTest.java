package com.graphhopper.trailmap.prototype.barrier.impl;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.ResponsePath;
import com.graphhopper.config.Profile;
import com.graphhopper.jackson.Jackson;
import com.graphhopper.routing.Router;
import com.graphhopper.routing.ev.BooleanEncodedValue;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.ev.Subnetwork;
import com.graphhopper.routing.util.AllEdgesIterator;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.trailmap.TrailmapGraphHopper;
import com.graphhopper.trailmap.barrier.*;
import com.graphhopper.util.*;
import com.graphhopper.util.details.PathDetail;
import com.graphhopper.util.shapes.GHPoint;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.*;

import static com.graphhopper.util.DistanceCalcEarth.DIST_EARTH;

/**
 * W26 S2/S3 critical validations (CATALOG §8), in-JVM on the S1+S2 graph, profiles gravel and trailmap_foot.
 * <ul>
 * <li>no flag: BarrierRouter == stock Router, byte for byte (the path JSON), on every request below;</li>
 * <li>unchanged set: 300 islands sealed by closed ways only + 300 isolated ones (seed 26) + every stock-marked edge in
 * the main component; a point 5 m off the island edge's middle, routed to the nearest main-network node: the flagged
 * answer's path equals the unflagged one;</li>
 * <li>changed set: every barrier edge of class ASK/REFUSE with the main network on one side and an island on the
 * other; A 30 m inside, B 30 m outside: ASK → one crossing at that node, W1/W2 checks, re-create replay (headings
 * none and aligned); REFUSE → the refusal naming that node, or a crossing through an ASK barrier of the same island;</li>
 * <li>way round: ASK/REFUSE barriers with the main network on both sides → a normal route, no crossing, no refusal;</li>
 * <li>notices: listed exactly on paths whose edges include a NOTICE barrier edge.</li>
 * </ul>
 * <pre>-Dbc.graph=DIR -Dbc.config=import-config.yml -Dbc.out=DIR [-Dbc.limit=N]</pre>
 */
public class BarrierRouteChecksTest {

    static TrailmapGraphHopper hopper;
    static Router stock;
    static final ObjectMapper JSON = Jackson.newObjectMapper();

    @BeforeAll
    static void load() throws Exception {
        String graph = System.getProperty("bc.graph");
        Assumptions.assumeTrue(graph != null, "bc.graph not set");
        hopper = BarrierGraphDiffTest.open(graph, System.getProperty("bc.config"));
        java.lang.reflect.Method wf = com.graphhopper.GraphHopper.class.getDeclaredMethod("createWeightingFactory");
        wf.setAccessible(true);
        stock = new Router(hopper.getBaseGraph(), hopper.getEncodingManager(), hopper.getLocationIndex(),
                profiles(), hopper.getPathDetailsBuilderFactory(), hopper.getTranslationMap(), hopper.getRouterConfig(),
                (com.graphhopper.routing.WeightingFactory) wf.invoke(hopper), Collections.emptyMap(), Collections.emptyMap());
    }

    static Map<String, Profile> profiles() {
        Map<String, Profile> m = new LinkedHashMap<>();
        for (Profile p : hopper.getProfiles())
            m.put(p.getName(), p);
        return m;
    }

    @AfterAll
    static void close() {
        if (hopper != null)
            hopper.close();
    }

    // ------------------------------------------------------------------------------------------------ helpers

    static boolean isBarrierEdge(EdgeIteratorState e, NodeAccess na) {
        return e.getDistance() < 0.01 && na.getLat(e.getBaseNode()) == na.getLat(e.getAdjNode())
                && na.getLon(e.getBaseNode()) == na.getLon(e.getAdjNode());
    }

    static int find(int[] p, int x) {
        while (p[x] != x) {
            p[x] = p[p[x]];
            x = p[x];
        }
        return x;
    }

    /** component root per node over edges open (finite base weight either way); -1 for nodes without open edges */
    static int[] components(Weighting w, int[] sizeOut) {
        BaseGraph g = hopper.getBaseGraph();
        int[] p = new int[g.getNodes()];
        for (int i = 0; i < p.length; i++) p[i] = i;
        boolean[] touched = new boolean[g.getNodes()];
        AllEdgesIterator it = g.getAllEdges();
        while (it.next()) {
            if (!(Double.isFinite(w.calcEdgeWeight(it, false)) || Double.isFinite(w.calcEdgeWeight(it, true))))
                continue;
            touched[it.getBaseNode()] = touched[it.getAdjNode()] = true;
            int a = find(p, it.getBaseNode()), b = find(p, it.getAdjNode());
            if (a != b) p[a] = b;
        }
        int[] root = new int[g.getNodes()];
        for (int i = 0; i < root.length; i++)
            root[i] = touched[i] ? find(p, i) : -1;
        return root;
    }

    static GHRequest req(String profile, GHPoint a, GHPoint b, boolean flag, Double heading) {
        GHRequest r = new GHRequest(a, b).setProfile(profile);
        r.setSnapPreventions(Collections.singletonList("ferry"));
        r.setPathDetails(Collections.singletonList("edge_id"));
        if (heading != null)
            r.setHeadings(Arrays.asList(heading, Double.NaN));
        if (flag)
            r.putHint(BarrierRouter.HINT, BarrierRouter.VERSION);
        return r;
    }

    static String pathJson(GHResponse rsp) throws Exception {
        if (rsp.hasErrors())
            return "ERR " + rsp.getErrors().get(0).getClass().getSimpleName() + " " + rsp.getErrors().get(0).getMessage();
        ResponsePath p = rsp.getBest();
        return JSON.writeValueAsString(Arrays.asList(p.getDistance(), p.getTime(), p.getPoints().toLineString(false).toString(),
                p.getWaypoints().toString(), p.getPathDetails().toString(), p.getInstructions().size()));
    }

    static List<Integer> edgeIds(ResponsePath p) {
        List<Integer> out = new ArrayList<>();
        for (PathDetail d : p.getPathDetails().getOrDefault("edge_id", Collections.emptyList())) {
            int id = ((Number) d.getValue()).intValue();
            if (out.isEmpty() || out.get(out.size() - 1) != id)
                out.add(id);
        }
        return out;
    }

    /** point at network distance d from start (every edge except skip), along the geometry */
    static Weighting alongWeighting;

    /** point at network distance d from start, over edges the profile can use (alongWeighting), never the skip edge */
    static GHPoint along(int start, int skip, double d) {
        BaseGraph g = hopper.getBaseGraph();
        NodeAccess na = g.getNodeAccess();
        EdgeExplorer ex = g.createEdgeExplorer();
        Map<Integer, Double> dist = new HashMap<>();
        PriorityQueue<double[]> q = new PriorityQueue<>(Comparator.comparingDouble(x -> x[0]));
        dist.put(start, 0.0);
        q.add(new double[]{0, start});
        GHPoint far = new GHPoint(na.getLat(start), na.getLon(start));
        while (!q.isEmpty()) {
            double[] cur = q.poll();
            int n = (int) cur[1];
            if (cur[0] > dist.getOrDefault(n, Double.MAX_VALUE)) continue;
            far = new GHPoint(na.getLat(n), na.getLon(n));
            EdgeIterator it = ex.setBaseNode(n);
            while (it.next()) {
                if (it.getEdge() == skip) continue;
                if (alongWeighting != null && !Double.isFinite(alongWeighting.calcEdgeWeight(it, false))
                        && !Double.isFinite(alongWeighting.calcEdgeWeight(it, true))) continue;
                double nd = cur[0] + it.getDistance();
                if (cur[0] <= d && nd >= d && it.getDistance() > 0.5) {
                    PointList pl = it.fetchWayGeometry(FetchMode.ALL);
                    double rest = d - cur[0];
                    for (int i = 0; i + 1 < pl.size(); i++) {
                        double seg = DIST_EARTH.calcDist(pl.getLat(i), pl.getLon(i), pl.getLat(i + 1), pl.getLon(i + 1));
                        if (seg >= rest || i + 2 == pl.size()) {
                            double f = seg <= 0 ? 0 : Math.min(1, rest / seg);
                            return new GHPoint(pl.getLat(i) + f * (pl.getLat(i + 1) - pl.getLat(i)), pl.getLon(i) + f * (pl.getLon(i + 1) - pl.getLon(i)));
                        }
                        rest -= seg;
                    }
                }
                if (nd < dist.getOrDefault(it.getAdjNode(), Double.MAX_VALUE)) {
                    dist.put(it.getAdjNode(), nd);
                    q.add(new double[]{nd, it.getAdjNode()});
                }
            }
        }
        return far;
    }

    @SuppressWarnings("unchecked")
    static Map<String, Object> barriersHint(GHResponse rsp) {
        return (Map<String, Object>) rsp.getHints().getObject(BarrierRouter.HINT, null);
    }

    @SuppressWarnings("unchecked")
    static List<Map<String, Object>> list(Map<String, Object> barriers, int path, String key) {
        if (barriers == null) return Collections.emptyList();
        List<Map<String, Object>> paths = (List<Map<String, Object>>) barriers.get("paths");
        if (paths == null || paths.size() <= path) return Collections.emptyList();
        return (List<Map<String, Object>>) paths.get(path).get(key);
    }

    static double bearingAt(PointList pts, int i) {
        int j = Math.min(i + 1, pts.size() - 1), k = j == i ? i - 1 : i;
        if (j == i) return AngleCalc.ANGLE_CALC.calcAzimuth(pts.getLat(k), pts.getLon(k), pts.getLat(i), pts.getLon(i));
        return AngleCalc.ANGLE_CALC.calcAzimuth(pts.getLat(i), pts.getLon(i), pts.getLat(j), pts.getLon(j));
    }

    // ------------------------------------------------------------------------------------------------ the checks

    @Test
    void checks() throws Exception {
        File outDir = new File(System.getProperty("bc.out"));
        outDir.mkdirs();
        int limit = Integer.getInteger("bc.limit", Integer.MAX_VALUE);
        BaseGraph g = hopper.getBaseGraph();
        NodeAccess na = g.getNodeAccess();
        ObjectNode report = JSON.createObjectNode();
        for (String profile : Arrays.asList("gravel", "trailmap_foot")) {
            Profile prof = hopper.getProfile(profile);
            Weighting w = hopper.createWeighting(prof, new PMap(), true);
            alongWeighting = w;
            EnumEncodedValue<BarrierClass> enc = BarrierProfiles.barrierEnc(hopper.getEncodingManager(), prof);
            BooleanEncodedValue stockSub = hopper.getEncodingManager().getBooleanEncodedValue(Subnetwork.key(profile));
            int[] comp = components(w, null);
            Map<Integer, Integer> compEdges = new HashMap<>();
            AllEdgesIterator it = g.getAllEdges();
            while (it.next())
                if (comp[it.getBaseNode()] >= 0 && (Double.isFinite(w.calcEdgeWeight(it, false)) || Double.isFinite(w.calcEdgeWeight(it, true))))
                    compEdges.merge(comp[it.getBaseNode()], 1, Integer::sum);
            int main = compEdges.entrySet().stream().max(Map.Entry.comparingByValue()).get().getKey();
            // seals per island: barrier (closed barrier edge to another component) or closed way
            Map<Integer, Integer> seal = new HashMap<>(); // 1 barrier, 2 way
            List<Integer> changed = new ArrayList<>(), wayRound = new ArrayList<>(), notices = new ArrayList<>();
            it = g.getAllEdges();
            while (it.next()) {
                boolean open = Double.isFinite(w.calcEdgeWeight(it, false)) || Double.isFinite(w.calcEdgeWeight(it, true));
                boolean barrier = isBarrierEdge(it, na);
                BarrierClass c = barrier ? it.get(enc) : BarrierClass.NONE;
                if (barrier && c == BarrierClass.NOTICE)
                    notices.add(it.getEdge());
                if (open)
                    continue;
                int a = comp[it.getBaseNode()], b = comp[it.getAdjNode()];
                if (a < 0 || b < 0)
                    continue;
                if (barrier && c.closes()) {
                    if (a == b)
                        wayRound.add(it.getEdge());
                    else if (a == main || b == main)
                        changed.add(it.getEdge());
                }
                if (a != b)
                    for (int k : new int[]{a, b})
                        seal.merge(k, barrier ? 1 : 2, Math::min);
            }
            // unchanged set candidates: islands (not main) sealed by ways only / not sealed; one edge each
            Map<Integer, Integer> islandEdge = new HashMap<>();
            it = g.getAllEdges();
            while (it.next()) {
                int a = comp[it.getBaseNode()];
                if (a < 0 || a == main || it.getDistance() < 5) continue;
                if (!(Double.isFinite(w.calcEdgeWeight(it, false)) || Double.isFinite(w.calcEdgeWeight(it, true)))) continue;
                islandEdge.putIfAbsent(a, it.getEdge());
            }
            List<Integer> wayOnly = new ArrayList<>(), isolated = new ArrayList<>();
            for (Map.Entry<Integer, Integer> e : islandEdge.entrySet()) {
                Integer s = seal.get(e.getKey());
                if (s == null) isolated.add(e.getValue());
                else if (s == 2) wayOnly.add(e.getValue());
            }
            Collections.sort(wayOnly);
            Collections.sort(isolated);
            Random rnd = new Random(26);
            Collections.shuffle(wayOnly, rnd);
            Collections.shuffle(isolated, rnd);
            List<Integer> traps = new ArrayList<>();
            it = g.getAllEdges();
            while (it.next())
                if (it.get(stockSub) && comp[it.getBaseNode()] == main)
                    traps.add(it.getEdge());
            ObjectNode pr = report.putObject(profile);
            pr.put("islands_way_only", wayOnly.size());
            pr.put("islands_isolated", isolated.size());
            pr.put("traps", traps.size());
            pr.put("changed_set", changed.size());
            pr.put("way_round_set", wayRound.size());
            pr.put("notice_barriers", notices.size());

            // --- unchanged set
            int[] unchanged = new int[4]; // cases, flag==noflag, barrier==stock (no flag), differs but explained
            int unchangedUnexplained = 0;
            List<Integer> uset = new ArrayList<>(wayOnly.subList(0, Math.min(300, wayOnly.size())));
            uset.addAll(isolated.subList(0, Math.min(300, isolated.size())));
            uset.addAll(traps);
            ArrayNode udiffs = pr.putArray("unchanged_diffs");
            for (int e : uset) {
                EdgeIteratorState edge = g.getEdgeIteratorState(e, Integer.MIN_VALUE);
                PointList pl = edge.fetchWayGeometry(FetchMode.ALL);
                GHPoint mid = midpointOffset(pl, 5);
                GHPoint target = nearestMainNode(comp, main, mid);
                if (target == null) continue;
                unchanged[0]++;
                String noFlag = pathJson(hopper.route(req(profile, mid, target, false, null)));
                String stockAns = pathJson(stock.route(req(profile, mid, target, false, null)));
                GHResponse fr = hopper.route(req(profile, mid, target, true, null));
                String flag = pathJson(fr);
                if (noFlag.equals(stockAns)) unchanged[2]++;
                if (noFlag.equals(flag)) unchanged[1]++;
                else {
                    // explained only if the flagged point now lies in an island sealed off by a barrier
                    boolean explained = false;
                    if (fr.hasErrors() && fr.getErrors().get(0) instanceof BarrierRefusedException)
                        explained = true;
                    else if (!fr.hasErrors()) {
                        List<Integer> ids = edgeIds(fr.getBest());
                        if (!ids.isEmpty()) {
                            int c0 = comp[g.getEdgeIteratorState(ids.get(0), Integer.MIN_VALUE).getBaseNode()];
                            explained = c0 != main && Integer.valueOf(1).equals(seal.get(c0));
                        }
                    }
                    unchanged[explained ? 3 : 0] += explained ? 1 : 0;
                    if (!explained) unchangedUnexplained++;
                }
                if (!noFlag.equals(flag) && udiffs.size() < 20) {
                    ObjectNode d = udiffs.addObject();
                    d.put("edge", e);
                    d.put("from", mid.toString());
                    d.put("to", target.toString());
                    d.put("noflag", noFlag.length() > 300 ? noFlag.substring(0, 300) : noFlag);
                    d.put("flag", flag.length() > 300 ? flag.substring(0, 300) : flag);
                }
            }
            pr.putPOJO("unchanged [cases, flag==noflag, noflag==stock_router, differs_snapped_into_barrier_island, differs_unexplained]",
                    new int[]{unchanged[0], unchanged[1], unchanged[2], unchanged[3], unchangedUnexplained});

            // --- changed set
            Map<String, Integer> outcome = new TreeMap<>();
            ArrayNode problems = pr.putArray("changed_problems");
            ArrayNode samples = pr.putArray("changed_samples");
            List<Long> msFlag = new ArrayList<>(), msNoFlag = new ArrayList<>();
            int[] replay = new int[9]; // replays, identical, misses, aligned identical, aligned differ same side, aligned wrong side, aligned A→W1 differs, aligned W2→B differs
            int n = 0;
            int[] w12 = new int[4]; // limited by junction, w at route point, two-barrier groups, W1/W2 farther than 12.5 m+0.5
            for (int e : changed) {
                if (n++ >= limit) break;
                EdgeIteratorState be = g.getEdgeIteratorState(e, Integer.MIN_VALUE);
                BarrierClass c = be.get(enc);
                int inside = comp[be.getBaseNode()] == main ? be.getAdjNode() : be.getBaseNode();
                int outside = inside == be.getBaseNode() ? be.getAdjNode() : be.getBaseNode();
                GHPoint a = along(inside, e, 30), b = along(outside, e, 30);
                GHPoint gate = new GHPoint(na.getLat(be.getBaseNode()), na.getLon(be.getBaseNode()));
                long t0 = System.nanoTime();
                GHResponse rsp = hopper.route(req(profile, a, b, true, null));
                msFlag.add((System.nanoTime() - t0) / 1_000_000);
                t0 = System.nanoTime();
                GHResponse plain = hopper.route(req(profile, a, b, false, null));
                msNoFlag.add((System.nanoTime() - t0) / 1_000_000);
                if (!pathJson(plain).equals(pathJson(stock.route(req(profile, a, b, false, null)))))
                    outcome.merge("NOFLAG_DIFFERS_FROM_STOCK", 1, Integer::sum);
                String key;
                if (rsp.hasErrors()) {
                    Throwable t = rsp.getErrors().get(0);
                    if (t instanceof BarrierRefusedException) {
                        Map<String, Object> det = ((BarrierRefusedException) t).getDetails();
                        @SuppressWarnings("unchecked")
                        Map<String, Object> bar = (Map<String, Object>) det.get("barrier");
                        boolean atNode = near(bar, gate);
                        key = c + (atNode ? " -> refused at this node" : " -> refused at another node");
                        // the refused crossing in the ask-crossing shape (owner 2026-11-02, "Draw a direct line")
                        String bad = refuseCrossingProblem(det, bar);
                        outcome.merge(bad == null ? "REFUSE_CROSSING_OK" : "REFUSE_CROSSING_BAD " + bad, 1, Integer::sum);
                    } else {
                        key = c + " -> error " + t.getClass().getSimpleName();
                    }
                } else {
                    List<Map<String, Object>> crossings = list(barriersHint(rsp), 0, "crossings");
                    if (crossings.isEmpty()) key = c + " -> route without crossing";
                    else {
                        boolean atNode = false;
                        int nb = 0;
                        for (Map<String, Object> cr : crossings) {
                            @SuppressWarnings("unchecked")
                            List<Map<String, Object>> bars = (List<Map<String, Object>>) cr.get("barriers");
                            nb += bars.size();
                            for (Map<String, Object> bar : bars) atNode |= near(bar, gate);
                            if (bars.size() > 1) w12[2]++;
                            if (Boolean.TRUE.equals(cr.get("w1_is_route_point")) || Boolean.TRUE.equals(cr.get("w2_is_route_point"))) w12[1]++;
                            double len = ((Number) cr.get("length_m")).doubleValue();
                            if (bars.size() == 1 && len < 24.9 && !Boolean.TRUE.equals(cr.get("w1_is_route_point")) && !Boolean.TRUE.equals(cr.get("w2_is_route_point"))) w12[0]++;
                            if (bars.size() == 1 && len > 25.5) w12[3]++;
                        }
                        key = c + (crossings.size() == 1 && atNode ? " -> one crossing at this node" : " -> crossings " + crossings.size() + (atNode ? " incl. this node" : " elsewhere"));
                        if (crossings.size() == 1 && atNode && c == BarrierClass.ASK)
                            replayCheck(profile, a, b, rsp, crossings.get(0), replay, problems, comp, comp[inside], main);
                    }
                }
                outcome.merge(key, 1, Integer::sum);
                if (!key.endsWith("one crossing at this node") && !key.endsWith("refused at this node") && problems.size() < 40) {
                    ObjectNode pdesc = problems.addObject();
                    pdesc.put("case", key);
                    pdesc.put("gate", gate.toString());
                    pdesc.put("a", a.toString());
                    pdesc.put("b", b.toString());
                    BarrierTable.Row row = BarrierTable.read(new File(System.getProperty("bc.graph"))).lookup(be, na);
                    if (row != null) pdesc.put("osm_node", row.osm_node);
                }
                if (samples.size() < 3 && !rsp.hasErrors())
                    samples.addPOJO(barriersHint(rsp));
            }
            pr.putPOJO("changed_outcomes", outcome);
            pr.putPOJO("replay [replays, identical, misses, aligned_identical, aligned_differs_same_side, aligned_WRONG_SIDE, aligned_A_W1_differs, aligned_W2_B_differs, client_contract_identical]", replay);
            pr.putPOJO("w1w2 [junction_limited, at_route_point, multi_barrier, longer_than_25.5]", w12);
            pr.putPOJO("ms_flag [p50, p90, max]", pct(msFlag));
            pr.putPOJO("ms_noflag [p50, p90, max]", pct(msNoFlag));

            // --- way round
            Map<String, Integer> round = new TreeMap<>();
            n = 0;
            for (int e : wayRound) {
                if (n++ >= Math.min(limit, 2000)) break;
                EdgeIteratorState be = g.getEdgeIteratorState(e, Integer.MIN_VALUE);
                GHPoint a = along(be.getBaseNode(), e, 30), b = along(be.getAdjNode(), e, 30);
                GHResponse rsp = hopper.route(req(profile, a, b, true, null));
                String key = be.get(enc) + (rsp.hasErrors() ? " -> error " + rsp.getErrors().get(0).getClass().getSimpleName()
                        : list(barriersHint(rsp), 0, "crossings").isEmpty() ? " -> normal route" : " -> crossing");
                round.merge(key, 1, Integer::sum);
                if (rsp.hasErrors())
                    problems.addObject().put("case", "way round: " + key).put("a", a.toString()).put("b", b.toString())
                            .put("gate", na.getLat(be.getBaseNode()) + "," + na.getLon(be.getBaseNode()))
                            .put("error", rsp.getErrors().get(0).getMessage())
                            .put("same_without_flag", pathJson(hopper.route(req(profile, a, b, false, null))).equals(pathJson(rsp)));
            }
            pr.putPOJO("way_round_outcomes", round);

            // --- two closed barriers within 25 m of each other: one crossing listing both (strictest class)
            Map<String, Integer> pairs = new TreeMap<>();
            int pairCases = 0;
            List<Integer> closedBarriers = new ArrayList<>(changed);
            closedBarriers.addAll(wayRound);
            Set<Integer> closedSet = new HashSet<>(closedBarriers);
            EdgeExplorer pex = g.createEdgeExplorer();
            for (int e : closedBarriers) {
                if (pairCases >= 300) break;
                EdgeIteratorState be = g.getEdgeIteratorState(e, Integer.MIN_VALUE);
                for (int side : new int[]{be.getBaseNode(), be.getAdjNode()}) {
                    // nearest other closed barrier within 25 m over open edges from this side
                    Map<Integer, Double> dist = new HashMap<>();
                    PriorityQueue<double[]> q = new PriorityQueue<>(Comparator.comparingDouble(x -> x[0]));
                    dist.put(side, 0.0);
                    q.add(new double[]{0, side});
                    int second = -1, reached = -1;
                    while (!q.isEmpty() && second < 0) {
                        double[] cur = q.poll();
                        int nn = (int) cur[1];
                        if (cur[0] > 25 || cur[0] > dist.getOrDefault(nn, Double.MAX_VALUE)) continue;
                        EdgeIterator ei = pex.setBaseNode(nn);
                        while (ei.next()) {
                            if (ei.getEdge() == e) continue;
                            if (closedSet.contains(ei.getEdge())) {
                                second = ei.getEdge();
                                reached = nn;
                                break;
                            }
                            if (!Double.isFinite(w.calcEdgeWeight(ei, false)) && !Double.isFinite(w.calcEdgeWeight(ei, true))) continue;
                            double nd = cur[0] + ei.getDistance();
                            if (nd <= 25 && nd < dist.getOrDefault(ei.getAdjNode(), Double.MAX_VALUE)) {
                                dist.put(ei.getAdjNode(), nd);
                                q.add(new double[]{nd, ei.getAdjNode()});
                            }
                        }
                    }
                    if (second < 0 || second < e) continue; // each pair once
                    EdgeIteratorState se = g.getEdgeIteratorState(second, Integer.MIN_VALUE);
                    int outer1 = side == be.getBaseNode() ? be.getAdjNode() : be.getBaseNode();
                    int outer2 = reached == se.getBaseNode() ? se.getAdjNode() : se.getBaseNode();
                    GHPoint a = along(outer1, e, 30), b = along(outer2, second, 30);
                    pairCases++;
                    GHResponse rsp = hopper.route(req(profile, a, b, true, null));
                    String key;
                    if (rsp.hasErrors())
                        key = rsp.getErrors().get(0) instanceof BarrierRefusedException ? "refused" : "error " + rsp.getErrors().get(0).getClass().getSimpleName();
                    else {
                        List<Map<String, Object>> cs = list(barriersHint(rsp), 0, "crossings");
                        int nbar = 0;
                        for (Map<String, Object> c0 : cs) nbar += ((List<?>) c0.get("barriers")).size();
                        key = cs.size() + " crossing(s), " + nbar + " barrier(s)";
                        if (cs.size() == 1 && nbar == 2 && pairs.getOrDefault("example", 0) == 0) {
                            pr.putPOJO("pair_example", cs.get(0));
                            pairs.put("example", 1);
                        }
                    }
                    BarrierClass c1 = be.get(enc), c2 = se.get(enc);
                    pairs.merge((c1 == BarrierClass.REFUSE || c2 == BarrierClass.REFUSE ? "with refuse: " : "ask+ask: ") + key, 1, Integer::sum);
                }
            }
            pairs.remove("example");
            pr.putPOJO("two_barriers_within_25m", pairs);

            // --- notices: on exactly the paths whose edges include a NOTICE barrier edge
            int[] nt = new int[3]; // cases, notice present when expected, notice wrongly present/absent
            Set<Integer> noticeSet = new HashSet<>(notices);
            for (int e : notices) {
                EdgeIteratorState be = g.getEdgeIteratorState(e, Integer.MIN_VALUE);
                GHPoint a = along(be.getBaseNode(), e, 30), b = along(be.getAdjNode(), e, 30);
                for (GHPoint[] ab : new GHPoint[][]{{a, b}, {a, along(be.getBaseNode(), e, 80)}}) {
                    GHResponse rsp = hopper.route(req(profile, ab[0], ab[1], true, null));
                    if (rsp.hasErrors()) continue;
                    nt[0]++;
                    boolean passes = false;
                    for (int id : edgeIds(rsp.getBest())) passes |= noticeSet.contains(id);
                    boolean listed = !list(barriersHint(rsp), 0, "notices").isEmpty();
                    if (passes && listed) nt[1]++;
                    if (passes != listed) {
                        nt[2]++;
                        double md = Double.MAX_VALUE;
                        PointList pts = rsp.getBest().getPoints();
                        for (int i = 0; i < pts.size(); i++)
                            md = Math.min(md, DIST_EARTH.calcDist(pts.getLat(i), pts.getLon(i), na.getLat(be.getBaseNode()), na.getLon(be.getBaseNode())));
                        if (problems.size() < 60)
                            problems.addObject().put("case", "notice " + (passes ? "missing" : "extra"))
                                    .put("gate", na.getLat(be.getBaseNode()) + "," + na.getLon(be.getBaseNode()))
                                    .put("a", ab[0].toString()).put("b", ab[1].toString()).put("min_point_dist_m", md)
                                    .put("edges", edgeIds(rsp.getBest()).toString()).put("barrier_edge", e);
                    }
                }
            }
            pr.putPOJO("notices [cases, expected_and_present, mismatches]", nt);
            System.out.println("CHECK " + profile + " " + JSON.writerWithDefaultPrettyPrinter().writeValueAsString(pr));
        }
        JSON.writerWithDefaultPrettyPrinter().writeValue(new File(outDir, "route-checks.json"), report);
        // every refusal carries the refused crossing in the ask-crossing shape
        org.junit.jupiter.api.Assertions.assertFalse(report.toString().contains("REFUSE_CROSSING_BAD"),
                "a refusal without a well-formed crossing; see route-checks.json");
    }

    /** -Dbc.debug=lat,lon;lat,lon : one case, printed */
    @Test
    @SuppressWarnings("unchecked")
    void debugOne() throws Exception {
        String dbg = System.getProperty("bc.debug");
        Assumptions.assumeTrue(dbg != null, "bc.debug not set");
        String[] ab = dbg.split(";");
        GHPoint a = GHPoint.fromString(ab[0]), b = GHPoint.fromString(ab[1]);
        String profile = System.getProperty("bc.profile", "gravel");
        GHResponse rsp = hopper.route(req(profile, a, b, true, null));
        System.out.println("DEBUG main " + pathJson(rsp) + " hints " + barriersHint(rsp));
        Map<String, Object> cr = list(barriersHint(rsp), 0, "crossings").get(0);
        List<Number> w2 = (List<Number>) cr.get("w2");
        GHPoint p2 = new GHPoint(w2.get(1).doubleValue(), w2.get(0).doubleValue());
        PointList pts = rsp.getBest().getPoints();
        double h2 = bearingAt(pts, closestIndex(pts, p2));
        GHResponse r2 = hopper.route(req(profile, p2, b, true, h2));
        System.out.println("DEBUG w2 " + p2 + " heading " + h2 + " -> " + pathJson(r2));
        GHResponse r2n = hopper.route(req(profile, p2, b, true, null));
        System.out.println("DEBUG w2 no heading -> " + pathJson(r2n));
        NodeAccess na = hopper.getBaseGraph().getNodeAccess();
        for (int id : edgeIds(r2.getBest())) {
            EdgeIteratorState e = hopper.getBaseGraph().getEdgeIteratorState(id, Integer.MIN_VALUE);
            System.out.println("DEBUG edge " + id + " " + na.getLat(e.getBaseNode()) + "," + na.getLon(e.getBaseNode()) + " -> "
                    + na.getLat(e.getAdjNode()) + "," + na.getLon(e.getAdjNode()) + " d=" + e.getDistance());
        }
    }

    /**
     * null when the refusal's {@code crossing} is the ask-crossing shape: class refuse, w1 == nearest_approach, a
     * distinct w2, the refused barrier among its barriers, and at most ~25 m unless an end is the route point.
     */
    @SuppressWarnings("unchecked")
    static String refuseCrossingProblem(Map<String, Object> det, Map<String, Object> bar) {
        Map<String, Object> cr = (Map<String, Object>) det.get("crossing");
        if (cr == null) return "missing";
        if (!"refuse".equals(cr.get("class"))) return "class " + cr.get("class");
        if (cr.get("w1") == null || !cr.get("w1").equals(det.get("nearest_approach"))) return "w1 != nearest_approach";
        if (cr.get("w2") == null || cr.get("w2").equals(cr.get("w1"))) return "no distinct w2";
        List<Map<String, Object>> bars = (List<Map<String, Object>>) cr.get("barriers");
        if (bars == null || !bars.contains(bar)) return "refused barrier not listed";
        double len = ((Number) cr.get("length_m")).doubleValue();
        boolean routePoint = Boolean.TRUE.equals(cr.get("w1_is_route_point")) || Boolean.TRUE.equals(cr.get("w2_is_route_point"));
        if (bars.size() == 1 && len > 25.5 && !routePoint) return "length " + len;
        return null;
    }

    static boolean near(Map<String, Object> bar, GHPoint gate) {
        return DIST_EARTH.calcDist(((Number) bar.get("lat")).doubleValue(), ((Number) bar.get("lon")).doubleValue(), gate.lat, gate.lon) < 0.5;
    }

    static long[] pct(List<Long> v) {
        if (v.isEmpty()) return new long[3];
        List<Long> s = new ArrayList<>(v);
        Collections.sort(s);
        return new long[]{s.get(s.size() / 2), s.get((int) (s.size() * 0.9)), s.get(s.size() - 1)};
    }

    static GHPoint midpointOffset(PointList pl, double m) {
        double total = 0;
        for (int i = 1; i < pl.size(); i++) total += DIST_EARTH.calcDist(pl.getLat(i - 1), pl.getLon(i - 1), pl.getLat(i), pl.getLon(i));
        double half = total / 2, s = 0;
        for (int i = 1; i < pl.size(); i++) {
            double seg = DIST_EARTH.calcDist(pl.getLat(i - 1), pl.getLon(i - 1), pl.getLat(i), pl.getLon(i));
            if (s + seg >= half) {
                double f = seg <= 0 ? 0 : (half - s) / seg;
                double lat = pl.getLat(i - 1) + f * (pl.getLat(i) - pl.getLat(i - 1)), lon = pl.getLon(i - 1) + f * (pl.getLon(i) - pl.getLon(i - 1));
                double brg = AngleCalc.ANGLE_CALC.calcAzimuth(pl.getLat(i - 1), pl.getLon(i - 1), pl.getLat(i), pl.getLon(i)) + 90;
                double dLat = m * Math.cos(Math.toRadians(brg)) / 111320, dLon = m * Math.sin(Math.toRadians(brg)) / (111320 * Math.cos(Math.toRadians(lat)));
                return new GHPoint(lat + dLat, lon + dLon);
            }
            s += seg;
        }
        return new GHPoint(pl.getLat(0), pl.getLon(0));
    }

    static GHPoint nearestMainNode(int[] comp, int main, GHPoint p) {
        NodeAccess na = hopper.getBaseGraph().getNodeAccess();
        double best = 3000;
        GHPoint bp = null;
        double d = 0.03;
        int[] found = {0};
        List<Integer> edges = new ArrayList<>();
        hopper.getLocationIndex().query(new com.graphhopper.util.shapes.BBox(p.lon - d * 2, p.lon + d * 2, p.lat - d, p.lat + d), edges::add);
        for (int e : edges) {
            EdgeIteratorState es = hopper.getBaseGraph().getEdgeIteratorState(e, Integer.MIN_VALUE);
            for (int nd : new int[]{es.getBaseNode(), es.getAdjNode()}) {
                if (comp[nd] != main) continue;
                double dist = DIST_EARTH.calcDist(p.lat, p.lon, na.getLat(nd), na.getLon(nd));
                if (dist < best) {
                    best = dist;
                    bp = new GHPoint(na.getLat(nd), na.getLon(nd));
                }
            }
        }
        return bp;
    }

    /**
     * The client's re-creation: A→W1 and W2→B as plain /route with the flag (and the crossing as a straight line).
     * Their edge sequences must be the response's sequence before and after the crossing.
     */
    @SuppressWarnings("unchecked")
    static void replayCheck(String profile, GHPoint a, GHPoint b, GHResponse rsp, Map<String, Object> crossing,
                            int[] replay, ArrayNode problems, int[] comp, int insideComp, int mainComp) {
        List<Number> w1 = (List<Number>) crossing.get("w1"), w2 = (List<Number>) crossing.get("w2");
        GHPoint p1 = new GHPoint(w1.get(1).doubleValue(), w1.get(0).doubleValue());
        GHPoint p2 = new GHPoint(w2.get(1).doubleValue(), w2.get(0).doubleValue());
        List<Integer> full = edgeIds(rsp.getBest());
        for (int variant = 0; variant < 2; variant++) {
            Double h1 = null, h2 = null;
            if (variant == 1) {
                // headings "aligned": the generating request carries the path's bearing at A, the replay carries the
                // same at A and (against the client contract, the worst case) the path's bearing at W2
                PointList pts = rsp.getBest().getPoints();
                h1 = bearingAt(pts, 0);
                GHResponse gen = hopper.route(req(profile, a, b, true, h1));
                List<Map<String, Object>> cs = list(barriersHint(gen), 0, "crossings");
                if (gen.hasErrors() || cs.size() != 1) {
                    if (problems.size() < 40)
                        problems.addObject().put("case", "aligned generation without one crossing").put("a", a.toString()).put("b", b.toString());
                    continue;
                }
                crossing = cs.get(0);
                w1 = (List<Number>) crossing.get("w1");
                w2 = (List<Number>) crossing.get("w2");
                p1 = new GHPoint(w1.get(1).doubleValue(), w1.get(0).doubleValue());
                p2 = new GHPoint(w2.get(1).doubleValue(), w2.get(0).doubleValue());
                full = edgeIds(gen.getBest());
                PointList gp = gen.getBest().getPoints();
                h2 = bearingAt(gp, closestIndex(gp, p2));
            }
            GHResponse r1 = Boolean.TRUE.equals(crossing.get("w1_is_route_point")) ? null : hopper.route(req(profile, a, p1, true, h1));
            GHResponse r2 = Boolean.TRUE.equals(crossing.get("w2_is_route_point")) ? null : hopper.route(req(profile, p2, b, true, h2));
            if ((r1 != null && r1.hasErrors()) || (r2 != null && r2.hasErrors())) {
                replay[2]++;
                if (problems.size() < 40)
                    problems.addObject().put("case", "replay error " + (variant == 1 ? "aligned" : "none")).put("a", a.toString()).put("b", b.toString());
                continue;
            }
            List<Integer> s1 = r1 == null ? Collections.emptyList() : edgeIds(r1.getBest());
            List<Integer> s2 = r2 == null ? Collections.emptyList() : edgeIds(r2.getBest());
            if (variant == 1) {
                // the client contract: heading at A as generated, none on the segment after the crossing
                GHResponse r2c = Boolean.TRUE.equals(crossing.get("w2_is_route_point")) ? null : hopper.route(req(profile, p2, b, true, null));
                List<Integer> s2c = r2c == null || r2c.hasErrors() ? Collections.emptyList() : edgeIds(r2c.getBest());
                if ((r2c == null || !r2c.hasErrors()) && startsWith(full, s1) && endsWith(full, s2c))
                    replay[8]++;
                else
                    problems.addObject().put("case", "replay miss, client contract (heading at A only)").put("a", a.toString()).put("b", b.toString())
                            .put("full", full.toString()).put("s1", s1.toString()).put("s2", s2c.toString());
            }
            boolean ok = startsWith(full, s1) && endsWith(full, s2);
            // the edges between the replays belong to the crossing: each touches a point within 25 m of a crossed barrier
            if (ok) {
                int from = Math.max(0, s1.size()), to = full.size() - Math.max(0, s2.size());
                NodeAccess na = hopper.getBaseGraph().getNodeAccess();
                List<Map<String, Object>> bars = (List<Map<String, Object>>) crossing.get("barriers");
                for (int i = from; i < to && ok; i++) {
                    EdgeIteratorState ed = hopper.getBaseGraph().getEdgeIteratorState(full.get(i), Integer.MIN_VALUE);
                    boolean nearBarrier = false;
                    for (Map<String, Object> bar : bars)
                        for (int nd : new int[]{ed.getBaseNode(), ed.getAdjNode()})
                            nearBarrier |= DIST_EARTH.calcDist(na.getLat(nd), na.getLon(nd), ((Number) bar.get("lat")).doubleValue(), ((Number) bar.get("lon")).doubleValue()) <= 25;
                    ok = nearBarrier;
                }
            }
            if (variant == 0) {
                replay[0]++;
                if (ok) replay[1]++;
                else {
                    replay[2]++;
                    problems.addObject().put("case", "replay miss").put("a", a.toString()).put("b", b.toString())
                                .put("full", full.toString()).put("s1", s1.toString()).put("s2", s2.toString());
                }
            } else if (ok) replay[3]++;
            else {
                // which side of the barrier did the heading-snapped replays start on?
                // A is inside the island, B on the main network: A→W1 must end inside, W2→B must start on main
                boolean sideOk = true;
                if (!s2.isEmpty())
                    sideOk &= comp[hopper.getBaseGraph().getEdgeIteratorState(s2.get(0), Integer.MIN_VALUE).getBaseNode()] == mainComp;
                if (!s1.isEmpty())
                    sideOk &= comp[hopper.getBaseGraph().getEdgeIteratorState(s1.get(s1.size() - 1), Integer.MIN_VALUE).getBaseNode()] == insideComp;
                replay[sideOk ? 4 : 5]++;
                if (!startsWith(full, s1)) replay[6]++; // A→W1 (heading at A, as the client sends it)
                if (!endsWith(full, s2)) replay[7]++;   // W2→B (heading at W2, which the client must not send)
                if (problems.size() < 40)
                    problems.addObject().put("case", "replay miss aligned, " + (sideOk ? "same side (W25 start-edge swap)" : "WRONG SIDE OF THE BARRIER"))
                            .put("a", a.toString()).put("b", b.toString())
                            .put("full", full.toString()).put("s1", s1.toString()).put("s2", s2.toString());
            }
        }
    }

    static int closestIndex(PointList pts, GHPoint p) {
        int best = 0;
        double bd = Double.MAX_VALUE;
        for (int i = 0; i < pts.size(); i++) {
            double d = DIST_EARTH.calcDist(p.lat, p.lon, pts.getLat(i), pts.getLon(i));
            if (d < bd) {
                bd = d;
                best = i;
            }
        }
        return best;
    }

    /** s (with its last element allowed to be a partial edge) is the start of full */
    static boolean startsWith(List<Integer> full, List<Integer> s) {
        if (s.isEmpty()) return true;
        if (s.size() > full.size()) return false;
        for (int i = 0; i < s.size(); i++)
            if (!full.get(i).equals(s.get(i))) return false;
        return true;
    }

    static boolean endsWith(List<Integer> full, List<Integer> s) {
        if (s.isEmpty()) return true;
        if (s.size() > full.size()) return false;
        for (int i = 0; i < s.size(); i++)
            if (!full.get(full.size() - s.size() + i).equals(s.get(i))) return false;
        return true;
    }
}
