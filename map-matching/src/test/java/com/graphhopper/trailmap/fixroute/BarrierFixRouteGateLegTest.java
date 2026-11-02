package com.graphhopper.trailmap.fixroute;

import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.config.Profile;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.util.AllEdgesIterator;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.trailmap.barrier.BarrierClass;
import com.graphhopper.trailmap.barrier.BarrierProfiles;
import com.graphhopper.trailmap.barrier.BarrierTable;
import com.graphhopper.trailmap.shared.EdgeKeyMatching;
import com.graphhopper.util.*;
import com.graphhopper.util.details.PathDetail;
import com.graphhopper.util.shapes.GHPoint;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import static com.graphhopper.util.DistanceCalcEarth.DIST_EARTH;

/**
 * W26 S4 /fix_route, option (i) (test_lead_routing, CATALOG §8): saved one-leg routes through closed changed-set
 * barriers, per class (ASK / REFUSE) and profile, fixed without and with the flag; the transition table no flag →
 * flag. Shapes: "end behind" (outside → barrier → 400 m inside) and "through" (outside → barrier 1 → the island →
 * barrier 2 → outside; both ends outside). Every leg that comes out worse with the flag is traced (its unroutable
 * stretches and the nearest closed barrier to each). Every leg that takes the routeAsApp retry is replayed with plain
 * /route + flag on the same waypoints and profile and compared as an edge_key sequence. In-JVM on the S1 graph.
 * <pre>-Dfix.graph=DIR -Dfix.config=FILE -Dfr.gateleg=N</pre>
 */
public class BarrierFixRouteGateLegTest {

    static GraphHopper hopper;
    static ExecutorService pool;
    static BarrierTable table;

    static boolean open(Weighting w, EdgeIteratorState e) {
        return Double.isFinite(w.calcEdgeWeight(e, false)) || Double.isFinite(w.calcEdgeWeight(e, true));
    }

    static int find(int[] p, int x) {
        while (p[x] != x) {
            p[x] = p[p[x]];
            x = p[x];
        }
        return x;
    }

    static GHPoint along(Weighting w, int start, int skip, double d) {
        BaseGraph g = hopper.getBaseGraph();
        EdgeExplorer ex = g.createEdgeExplorer();
        Map<Integer, Double> dist = new HashMap<>();
        PriorityQueue<double[]> q = new PriorityQueue<>(Comparator.comparingDouble(x -> x[0]));
        dist.put(start, 0.0);
        q.add(new double[]{0, start});
        while (!q.isEmpty()) {
            double[] cur = q.poll();
            int n = (int) cur[1];
            if (cur[0] > dist.getOrDefault(n, Double.MAX_VALUE)) continue;
            EdgeIterator it = ex.setBaseNode(n);
            while (it.next()) {
                if (it.getEdge() == skip || !open(w, it)) continue;
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
        return null;
    }

    /** plain /route (no flag) between two points; inside an island the stock snap may move them, so the flag snaps */
    static GHResponse route(String prof, boolean flagSnap, GHPoint a, GHPoint b) {
        GHRequest r = new GHRequest(a, b).setProfile(prof);
        r.setSnapPreventions(Collections.singletonList("ferry"));
        r.putHint("instructions", false);
        if (flagSnap) r.putHint("barriers", "v1");
        return hopper.route(r);
    }

    static boolean worse(String no, FixRouteResponse.Leg f) {
        int rank = rank(f.status), r0 = rank(no);
        return rank > r0 || (rank == r0 && f.metrics.addedWaypoints > 0 && !FixRouteResponse.FIXED.equals(no));
    }

    static int rank(String s) {
        return switch (s) {
            case FixRouteResponse.OK -> 0;
            case FixRouteResponse.FIXED -> 1;
            case FixRouteResponse.UNROUTABLE -> 2;
            default -> 3; // no_route, not_processed
        };
    }

    /** one saved leg along `truth` from its first to its last point; returns [transition key, trace or ""] */
    static String[] run(String prof, PointList truth, EnumEncodedValue<BarrierClass> enc) {
        FixRouteRequest q = new FixRouteRequest();
        q.snapPreventions = List.of("ferry");
        double len = 0;
        for (int i = 0; i < truth.size(); i++) {
            if (i > 0) len += DIST_EARTH.calcDist(truth.getLat(i - 1), truth.getLon(i - 1), truth.getLat(i), truth.getLon(i));
            q.reference.add(new double[]{truth.getLat(i), truth.getLon(i)});
        }
        GHPoint pa = new GHPoint(RouteFixer.round6(truth.getLat(0)), RouteFixer.round6(truth.getLon(0)));
        GHPoint pb = new GHPoint(RouteFixer.round6(truth.getLat(truth.size() - 1)), RouteFixer.round6(truth.getLon(truth.size() - 1)));
        q.waypoints.add(new FixRouteRequest.Waypoint("a", pa.lat, pa.lon));
        q.waypoints.add(new FixRouteRequest.Waypoint("b", pb.lat, pb.lon));
        FixRouteRequest.Segment s = new FixRouteRequest.Segment();
        s.id = "s";
        s.start = "a";
        s.end = "b";
        s.profile = prof;
        s.savedLengthM = len;
        q.segments.add(s);
        FixRouteResponse.Leg l0 = FixRouteEngine.fix(hopper, pool, q).legs.get(0);
        String no = l0.status;
        q.barriers = "v1";
        FixRouteResponse.Leg lf = FixRouteEngine.fix(hopper, pool, q).legs.get(0);
        String key = "no flag " + no + " +" + l0.metrics.addedWaypoints + " wp -> flag " + lf.status + " +" + lf.metrics.addedWaypoints + " wp";
        // the routeAsApp retry: taken when the leg's own flagged route needs a crossing
        ClientRoute.Settings st = new ClientRoute.Settings(prof, null, q.snapPreventions, true);
        if (!ClientRoute.route(hopper, st, pa, List.of(), pb, null, MatchPipelineFixer.CLIENT_HEADING_PENALTY).ok()) {
            ClientRoute.Leg app = ClientRoute.routeAsApp(hopper, st, pa, List.of(), pb, null, MatchPipelineFixer.CLIENT_HEADING_PENALTY);
            if (app.ok()) {
                GHResponse plain = hopper.route(ClientRoute.request(st, pa, List.of(), pb, null, MatchPipelineFixer.CLIENT_HEADING_PENALTY));
                List<PathDetail> ek = plain.hasErrors() ? null : plain.getBest().getPathDetails().get("edge_key");
                int[] keys = ek == null ? null : EdgeKeyMatching.edgeKeysFromDetails(ek);
                key += Arrays.equals(keys, app.edgeKeys()) ? ", retry == /route+flag" : ", RETRY DIFFERS FROM /route+flag";
            } else key += ", retry fails too";
        }
        String trace = "";
        if (rank(lf.status) > rank(no) || (rank(lf.status) == rank(no) && lf.metrics.addedWaypoints > l0.metrics.addedWaypoints)) {
            StringBuilder sb = new StringBuilder();
            ReferenceTrack ref = new ReferenceTrack(q.reference);
            if (lf.unroutable != null)
                for (FixRouteResponse.Unroutable u : lf.unroutable) {
                    double[] mid = ref.pointAt((u.refFromM + u.refToM) / 2);
                    BarrierTable.Row best = null;
                    double bd = Double.MAX_VALUE;
                    for (BarrierTable.Row r : table.rows()) {
                        double d = DIST_EARTH.calcDist(mid[0], mid[1], r.lat, r.lon);
                        if (d < bd) {
                            bd = d;
                            best = r;
                        }
                    }
                    String mode = BarrierClass.KEY_FOOT.equals(enc.getName()) ? best.foot : best.bike;
                    sb.append(String.format(Locale.ROOT, "stretch %.0f..%.0f m (%s, %.0f m): nearest barrier osm %d %s %.0f m away; ",
                            u.refFromM, u.refToM, u.cause, u.referenceLengthM, best.osm_node, mode, bd));
                }
            trace = String.format(Locale.ROOT, "%s | from %s to %s | %s", key, pa, pb, sb);
        }
        return new String[]{key, trace};
    }

    @Test
    void gateLegs() throws Exception {
        Assumptions.assumeTrue(System.getProperty("fr.gateleg") != null, "fr.gateleg not set");
        int per = Integer.getInteger("fr.gateleg");
        hopper = FixRouteTestGraph.open();
        pool = Executors.newFixedThreadPool(4);
        table = BarrierTable.read(new File(hopper.getGraphHopperLocation()));
        BaseGraph g = hopper.getBaseGraph();
        NodeAccess na = g.getNodeAccess();
        for (String prof : new String[]{"gravel", "trailmap_foot"}) {
            Profile p = hopper.getProfile(prof);
            Weighting w = hopper.createWeighting(p, new PMap());
            EnumEncodedValue<BarrierClass> enc = BarrierProfiles.barrierEnc(hopper.getEncodingManager(), p);
            int[] par = new int[g.getNodes()];
            for (int i = 0; i < par.length; i++) par[i] = i;
            AllEdgesIterator it = g.getAllEdges();
            while (it.next()) if (open(w, it)) {
                int a = find(par, it.getBaseNode()), b = find(par, it.getAdjNode());
                if (a != b) par[a] = b;
            }
            Map<Integer, Integer> size = new HashMap<>();
            it = g.getAllEdges();
            while (it.next()) if (open(w, it)) size.merge(find(par, it.getBaseNode()), 1, Integer::sum);
            int main = size.entrySet().stream().max(Map.Entry.comparingByValue()).get().getKey();
            for (BarrierClass cls : new BarrierClass[]{BarrierClass.ASK, BarrierClass.REFUSE}) {
                // changed-set barriers of this class, and per island its exits of this class
                List<int[]> gates = new ArrayList<>(); // edge, outside node, inside node
                Map<Integer, List<int[]>> exits = new LinkedHashMap<>();
                it = g.getAllEdges();
                while (it.next()) {
                    if (it.getDistance() >= 0.01 || it.get(enc) != cls || open(w, it)) continue;
                    if (na.getLat(it.getBaseNode()) != na.getLat(it.getAdjNode()) || na.getLon(it.getBaseNode()) != na.getLon(it.getAdjNode())) continue;
                    int ca = find(par, it.getBaseNode()), cb = find(par, it.getAdjNode());
                    if (ca == cb || (ca != main && cb != main)) continue;
                    int outside = ca == main ? it.getBaseNode() : it.getAdjNode();
                    int inside = outside == it.getBaseNode() ? it.getAdjNode() : it.getBaseNode();
                    int[] gt = {it.getEdge(), outside, inside};
                    gates.add(gt);
                    exits.computeIfAbsent(find(par, inside), k -> new ArrayList<>()).add(gt);
                }
                // end behind
                Map<String, Integer> out = new TreeMap<>();
                List<String> traces = new ArrayList<>();
                int n = 0;
                for (int[] gt : gates) {
                    if (n >= per) break;
                    GHPoint a = along(w, gt[1], gt[0], 400), ai = along(w, gt[1], gt[0], 1), bi = along(w, gt[2], gt[0], 1), b = along(w, gt[2], gt[0], 400);
                    if (a == null || ai == null || bi == null || b == null) continue;
                    GHResponse r1 = route(prof, false, a, ai), r2 = route(prof, true, bi, b);
                    if (r1.hasErrors() || r2.hasErrors()) continue;
                    PointList truth = new PointList();
                    truth.add(r1.getBest().getPoints());
                    truth.add(na.getLat(gt[1]), na.getLon(gt[1]));
                    truth.add(r2.getBest().getPoints());
                    if (truth.size() < 3) continue;
                    n++;
                    String[] r = run(prof, truth, enc);
                    out.merge(r[0], 1, Integer::sum);
                    if (!r[1].isEmpty()) traces.add(r[1]);
                }
                System.out.println("FRGL end-behind " + prof + " " + cls + " " + n + " " + out);
                for (String t : traces) System.out.println("FRGL   worse: " + t);
                // through: both ends outside, in by one exit, out by another
                out = new TreeMap<>();
                traces = new ArrayList<>();
                n = 0;
                for (List<int[]> ex : exits.values()) {
                    if (n >= per) break;
                    if (ex.size() < 2) continue;
                    int[] g1 = ex.get(0), g2 = ex.get(1);
                    GHPoint a = along(w, g1[1], g1[0], 400), a1 = along(w, g1[1], g1[0], 1), i1 = along(w, g1[2], g1[0], 1);
                    GHPoint i2 = along(w, g2[2], g2[0], 1), b2 = along(w, g2[1], g2[0], 1), b = along(w, g2[1], g2[0], 400);
                    if (a == null || a1 == null || i1 == null || i2 == null || b2 == null || b == null) continue;
                    GHResponse r1 = route(prof, false, a, a1), rIn = route(prof, true, i1, i2), r3 = route(prof, false, b2, b);
                    if (r1.hasErrors() || rIn.hasErrors() || r3.hasErrors()) continue;
                    PointList truth = new PointList();
                    truth.add(r1.getBest().getPoints());
                    truth.add(na.getLat(g1[1]), na.getLon(g1[1]));
                    truth.add(rIn.getBest().getPoints());
                    truth.add(na.getLat(g2[1]), na.getLon(g2[1]));
                    truth.add(r3.getBest().getPoints());
                    n++;
                    String[] r = run(prof, truth, enc);
                    out.merge(r[0], 1, Integer::sum);
                    if (!r[1].isEmpty()) traces.add(r[1]);
                }
                System.out.println("FRGL through " + prof + " " + cls + " " + n + " " + out);
                for (String t : traces) System.out.println("FRGL   worse: " + t);
            }
        }
        pool.shutdown();
        hopper.close();
    }
}
