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
import com.graphhopper.util.PMap;
import com.graphhopper.util.PointList;
import com.graphhopper.util.shapes.GHPoint;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.util.*;
import java.util.concurrent.Executors;

import static com.graphhopper.util.DistanceCalcEarth.DIST_EARTH;

/**
 * W26 S4 /fix_route, two owner / web questions (lead_routing 2026-11-02), in-JVM on the graph given by -Dfix.graph:
 * <ul>
 * <li>{@code -Dfr.contract=corpus}: every fixture route, no flag (an old app): fix, apply as the app, then every
 * followRoads segment routed as the app routes it (plain /route). A segment that doesn't route = the saved route is
 * BROKEN for that app. Counted per route and leg, with examples. Run on the old map (pre-S1 graph) and on the new one.</li>
 * <li>{@code -Dfr.contract=crossing} (J300, gates_web): a saved crossing as the app stores it, A→W1 followRoads,
 * W1→W2 direct, W2→B followRoads, W1 / W2 from /route + flag's crossing; fixed without and with the flag, every
 * returned segment printed (type, ends moved from the stored ones, metres).</li>
 * </ul>
 * <pre>-Dfix.graph=DIR -Dfix.config=FILE -Dfix.fixtures=DIR -Dfr.contract=corpus|crossing [-Dfr.n=N]</pre>
 */
public class BarrierFixRouteContractTest {

    static GraphHopper hopper;

    @Test
    void contract() throws Exception {
        String what = System.getProperty("fr.contract");
        Assumptions.assumeTrue(what != null, "fr.contract not set");
        hopper = FixRouteTestGraph.open();
        FixRouteCorpusTest.hopper = hopper;
        BarrierFixRouteGateLegTest.hopper = hopper;
        BarrierFixRouteGateLegTest.pool = Executors.newFixedThreadPool(4);
        if ("corpus".equals(what)) corpus();
        else crossing();
        BarrierFixRouteGateLegTest.pool.shutdown();
        hopper.close();
    }

    void corpus() throws Exception {
        int routes = 0, broken = 0, legsBroken = 0, legs = 0;
        List<String> examples = new ArrayList<>();
        for (long id : FixRouteFixtures.allIds()) {
            FixRouteRequest q = FixRouteCorpusTest.request(FixRouteFixtures.load(id));
            q.barriers = null;
            routes++;
            for (String policy : new String[]{FixRouteRequest.POLICY_REROUTE, FixRouteRequest.POLICY_COORDINATES}) {
                q.unroutablePolicy = policy;
                FixRouteResponse r = FixRouteEngine.fix(hopper, BarrierFixRouteGateLegTest.pool, q);
                FixRouteRequest q2 = FixRouteCorpusTest.applyAndReroute(q, r);
                List<Integer> parents = FixRouteCorpusTest.LAST_PARENT.get();
                Set<Integer> bad = new TreeSet<>();
                for (int i = 0; i < q2.segments.size(); i++) {
                    FixRouteRequest.Segment s = q2.segments.get(i);
                    if (!s.isFollowRoads()) continue;
                    if (policy.equals(FixRouteRequest.POLICY_REROUTE)) legs++;
                    FixRouteRequest.Waypoint wa = q2.waypoints.get(i), wb = q2.waypoints.get(i + 1);
                    List<GHPoint> via = new ArrayList<>();
                    for (FixRouteRequest.LatLng v : s.viaPoints) via.add(new GHPoint(v.lat, v.lng));
                    ClientRoute.Leg leg = ClientRoute.route(hopper, new ClientRoute.Settings(s.profile, s.customModel, q2.snapPreventions),
                            new GHPoint(wa.coordinates.lat, wa.coordinates.lng), via, new GHPoint(wb.coordinates.lat, wb.coordinates.lng),
                            s.initialHeading, s.headingPenalty);
                    if (!leg.ok()) {
                        int parent = parents.get(i);
                        bad.add(parent);
                        if (examples.size() < 25)
                            examples.add(policy + " " + id + ":" + parent + " (" + r.legs.get(parent).status + ", +" + r.legs.get(parent).metrics.addedWaypoints
                                    + " wp) piece " + i + ": " + leg.error());
                    }
                }
                if (!bad.isEmpty()) {
                    if (policy.equals(FixRouteRequest.POLICY_REROUTE)) broken++;
                    legsBroken += bad.size();
                    System.out.println("CONTRACT broken " + policy + " route " + id + " legs " + bad);
                }
            }
        }
        System.out.println("CONTRACT corpus routes " + routes + ", followRoads pieces " + legs + ", routes with a piece the app can't route (reroute policy) "
                + broken + ", broken legs over both policies " + legsBroken);
        for (String e : examples) System.out.println("CONTRACT example " + e);
    }

    void crossing() throws Exception {
        int per = Integer.getInteger("fr.n", 10);
        BaseGraph g = hopper.getBaseGraph();
        NodeAccess na = g.getNodeAccess();
        for (String prof : new String[]{"gravel", "trailmap_foot"}) {
            Profile p = hopper.getProfile(prof);
            Weighting w = hopper.createWeighting(p, new PMap());
            EnumEncodedValue<BarrierClass> enc = BarrierProfiles.barrierEnc(hopper.getEncodingManager(), p);
            int[] par = new int[g.getNodes()];
            for (int i = 0; i < par.length; i++) par[i] = i;
            AllEdgesIterator it = g.getAllEdges();
            while (it.next()) if (BarrierFixRouteGateLegTest.open(w, it)) {
                int a = BarrierFixRouteGateLegTest.find(par, it.getBaseNode()), b = BarrierFixRouteGateLegTest.find(par, it.getAdjNode());
                if (a != b) par[a] = b;
            }
            Map<Integer, Integer> size = new HashMap<>();
            it = g.getAllEdges();
            while (it.next()) if (BarrierFixRouteGateLegTest.open(w, it)) size.merge(BarrierFixRouteGateLegTest.find(par, it.getBaseNode()), 1, Integer::sum);
            int main = size.entrySet().stream().max(Map.Entry.comparingByValue()).get().getKey();
            Map<String, Integer> table = new TreeMap<>();
            int n = 0;
            it = g.getAllEdges();
            while (it.next() && n < per) {
                if (it.getDistance() >= 0.01 || it.get(enc) != BarrierClass.ASK || BarrierFixRouteGateLegTest.open(w, it)) continue;
                if (na.getLat(it.getBaseNode()) != na.getLat(it.getAdjNode()) || na.getLon(it.getBaseNode()) != na.getLon(it.getAdjNode())) continue;
                int ca = BarrierFixRouteGateLegTest.find(par, it.getBaseNode()), cb = BarrierFixRouteGateLegTest.find(par, it.getAdjNode());
                if (ca == cb || (ca != main && cb != main)) continue;
                int outside = ca == main ? it.getBaseNode() : it.getAdjNode();
                int inside = outside == it.getBaseNode() ? it.getAdjNode() : it.getBaseNode();
                GHPoint a = BarrierFixRouteGateLegTest.along(w, outside, it.getEdge(), 400), b = BarrierFixRouteGateLegTest.along(w, inside, it.getEdge(), 400);
                if (a == null || b == null) continue;
                // the app's crossing: /route + flag from A to B, split at W1 / W2
                GHRequest rq = new GHRequest(a, b).setProfile(prof);
                rq.setSnapPreventions(List.of("ferry"));
                rq.putHint("instructions", false);
                rq.putHint("barriers", "v1");
                GHResponse rsp = hopper.route(rq);
                if (rsp.hasErrors()) continue;
                @SuppressWarnings("unchecked") Map<String, Object> hb = (Map<String, Object>) rsp.getHints().getObject("barriers", null);
                if (hb == null) continue;
                @SuppressWarnings("unchecked") List<Map<String, Object>> cs = (List<Map<String, Object>>) ((List<Map<String, Object>>) hb.get("paths")).get(0).get("crossings");
                if (cs == null || cs.size() != 1) continue;
                Map<String, Object> c = cs.get(0);
                @SuppressWarnings("unchecked") List<Number> w1 = (List<Number>) c.get("w1"), w2 = (List<Number>) c.get("w2");
                PointList pl = rsp.getBest().getPoints();
                int i1 = (Integer) c.get("w1_index"), i2 = (Integer) c.get("w2_index");
                n++;
                // the saved route: 3 segments, reference = the path's points
                FixRouteRequest q = new FixRouteRequest();
                q.snapPreventions = List.of("ferry");
                for (int i = 0; i < pl.size(); i++) q.reference.add(new double[]{pl.getLat(i), pl.getLon(i)});
                double[][] wp = {{pl.getLat(0), pl.getLon(0)}, {w1.get(1).doubleValue(), w1.get(0).doubleValue()},
                        {w2.get(1).doubleValue(), w2.get(0).doubleValue()}, {pl.getLat(pl.size() - 1), pl.getLon(pl.size() - 1)}};
                String[] type = {"followRoads", "direct", "followRoads"};
                double[] len = {lenOf(pl, 0, i1) + DIST_EARTH.calcDist(pl.getLat(i1), pl.getLon(i1), wp[1][0], wp[1][1]),
                        DIST_EARTH.calcDist(wp[1][0], wp[1][1], wp[2][0], wp[2][1]),
                        lenOf(pl, i2, pl.size() - 1) + DIST_EARTH.calcDist(wp[2][0], wp[2][1], pl.getLat(i2), pl.getLon(i2))};
                for (int k = 0; k < 4; k++) q.waypoints.add(new FixRouteRequest.Waypoint("p" + k, RouteFixer.round6(wp[k][0]), RouteFixer.round6(wp[k][1])));
                for (int k = 0; k < 3; k++) {
                    FixRouteRequest.Segment s = new FixRouteRequest.Segment();
                    s.id = "s" + k;
                    s.start = "p" + k;
                    s.end = "p" + (k + 1);
                    s.type = type[k];
                    s.profile = prof;
                    s.savedLengthM = len[k];
                    q.segments.add(s);
                }
                for (String flag : new String[]{null, "v1"})
                    for (String policy : new String[]{FixRouteRequest.POLICY_REROUTE, FixRouteRequest.POLICY_COORDINATES}) {
                        q.barriers = flag;
                        q.unroutablePolicy = policy;
                        FixRouteResponse r = FixRouteEngine.fix(hopper, BarrierFixRouteGateLegTest.pool, q);
                        StringBuilder sb = new StringBuilder();
                        boolean asStored = true;
                        for (FixRouteResponse.Leg l : r.legs) {
                            sb.append(" | leg ").append(l.index).append(' ').append(l.status).append(':');
                            for (FixRouteResponse.Segment s : l.segments) {
                                sb.append(' ').append(s.type).append(s.end.newPoint != null ? "(new end)" : "").append(Math.round(s.distanceM)).append('m');
                                if (s.end.newPoint != null || !s.type.equals(q.segments.get(l.index).type)) asStored = false;
                            }
                            if (l.segments.size() != 1) asStored = false;
                        }
                        String key = prof + " | " + (flag == null ? "no flag" : "flag") + " | " + policy;
                        table.merge(key + " | " + (asStored ? "AS STORED" : "CHANGED") + statuses(r), 1, Integer::sum);
                        if (n <= 3) System.out.println("CROSSING " + key + " gate " + na.getLat(it.getBaseNode()) + "," + na.getLon(it.getBaseNode()) + sb);
                    }
            }
            for (Map.Entry<String, Integer> e : table.entrySet()) System.out.println("CROSSING TABLE " + e.getKey() + " = " + e.getValue());
        }
    }

    static String statuses(FixRouteResponse r) {
        StringBuilder sb = new StringBuilder(" [");
        for (FixRouteResponse.Leg l : r.legs) sb.append(l.index == 0 ? "" : ",").append(l.status);
        return sb.append(']').toString();
    }

    static double lenOf(PointList pl, int from, int to) {
        double d = 0;
        for (int i = from + 1; i <= to; i++) d += DIST_EARTH.calcDist(pl.getLat(i - 1), pl.getLon(i - 1), pl.getLat(i), pl.getLon(i));
        return d;
    }
}
