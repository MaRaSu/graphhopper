package com.graphhopper.trailmap.fixroute;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
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
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.PMap;
import com.graphhopper.util.PointList;
import com.graphhopper.util.shapes.GHPoint;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.*;
import java.util.concurrent.Executors;

import static com.graphhopper.util.DistanceCalcEarth.DIST_EARTH;

/**
 * W26 owner question A48 (2026-11-02): what does the app end up showing for a saved route through a closed barrier,
 * after /fix_route on the NEW map (S1 graph), for an old app (no flag) and a new app (flag), each with
 * unroutable_policy reroute and coordinates? Saved routes (one leg, as the app stores them):
 * <ul>
 * <li>S1-closed: through each barrier S1 newly closes (open on the old map): 300 m before → barrier → 300 m after;</li>
 * <li>ask-behind / refuse-behind: 400 m outside → a changed-set barrier → 400 m inside (16178:16 shape);</li>
 * <li>ask-through: in at one ask exit of a gated area, out at another, both ends outside.</li>
 * </ul>
 * The app's view: the response applied as the app does (FixRouteCorpusTest.applyAndReroute), then every followRoads
 * segment routed as that app would (old: /route; new: /route + flag, where a crossing = the app's dialog and a
 * refusal = its refuse message). Outcome per case, worst segment first: BROKEN (a followRoads segment the app can't
 * route) / REFUSE MESSAGE / CROSSING DIALOG / DIFFERENT PATH (shown line > 30 m off the saved line somewhere) / AS SAVED.
 * <pre>-Dfix.graph=DIR -Dfix.config=FILE -Dfr.owner=N -Dfr.diff=ABS/i14/diff.json</pre>
 */
public class BarrierFixRouteOwnerCasesTest {

    static GraphHopper hopper;

    static PointList through(String prof, Weighting w, int e, int from, int to, boolean flagSnap) {
        BaseGraph g = hopper.getBaseGraph();
        NodeAccess na = g.getNodeAccess();
        GHPoint a = BarrierFixRouteGateLegTest.along(w, from, e, from == to ? 0 : 400), a1 = BarrierFixRouteGateLegTest.along(w, from, e, 1);
        GHPoint b1 = BarrierFixRouteGateLegTest.along(w, to, e, 1), b = BarrierFixRouteGateLegTest.along(w, to, e, 400);
        if (a == null || a1 == null || b1 == null || b == null) return null;
        GHResponse r1 = BarrierFixRouteGateLegTest.route(prof, flagSnap, a, a1), r2 = BarrierFixRouteGateLegTest.route(prof, flagSnap, b1, b);
        if (r1.hasErrors() || r2.hasErrors()) return null;
        PointList pl = new PointList();
        pl.add(r1.getBest().getPoints());
        pl.add(na.getLat(from), na.getLon(from));
        pl.add(r2.getBest().getPoints());
        return pl.size() < 3 ? null : pl;
    }

    static FixRouteRequest saved(String prof, PointList pl) {
        FixRouteRequest q = new FixRouteRequest();
        q.snapPreventions = List.of("ferry");
        double len = 0;
        for (int i = 0; i < pl.size(); i++) {
            if (i > 0) len += DIST_EARTH.calcDist(pl.getLat(i - 1), pl.getLon(i - 1), pl.getLat(i), pl.getLon(i));
            q.reference.add(new double[]{pl.getLat(i), pl.getLon(i)});
        }
        q.waypoints.add(new FixRouteRequest.Waypoint("a", RouteFixer.round6(pl.getLat(0)), RouteFixer.round6(pl.getLon(0))));
        q.waypoints.add(new FixRouteRequest.Waypoint("b", RouteFixer.round6(pl.getLat(pl.size() - 1)), RouteFixer.round6(pl.getLon(pl.size() - 1))));
        FixRouteRequest.Segment s = new FixRouteRequest.Segment();
        s.id = "s";
        s.start = "a";
        s.end = "b";
        s.profile = prof;
        s.savedLengthM = len;
        q.segments.add(s);
        return q;
    }

    static double toLine(double[] p, List<double[]> line) {
        double best = Double.MAX_VALUE, cos = Math.cos(Math.toRadians(p[0]));
        for (int i = 0; i + 1 < line.size(); i++) {
            double[] a = line.get(i), b = line.get(i + 1);
            double bx = (b[1] - a[1]) * cos * 111320, by = (b[0] - a[0]) * 111320;
            double px = (p[1] - a[1]) * cos * 111320, py = (p[0] - a[0]) * 111320;
            double l2 = bx * bx + by * by;
            double t = l2 <= 0 ? 0 : Math.max(0, Math.min(1, (px * bx + py * by) / l2));
            best = Math.min(best, Math.hypot(px - t * bx, py - t * by));
        }
        return best;
    }

    /** the app's view of one saved route after /fix_route: [outcome, fix status] */
    static String[] appView(String prof, PointList pl, boolean newApp, String policy) {
        FixRouteRequest q = saved(prof, pl);
        q.barriers = newApp ? "v1" : null;
        q.unroutablePolicy = policy;
        FixRouteResponse r = FixRouteEngine.fix(hopper, BarrierFixRouteGateLegTest.pool, q);
        String status = r.legs.get(0).status;
        FixRouteRequest q2 = FixRouteCorpusTest.applyAndReroute(q, r);
        int worst = 0; // 0 as saved, 1 different path, 2 crossing dialog, 3 refuse message, 4 broken
        List<double[]> shown = new ArrayList<>();
        ClientRoute.Settings st = new ClientRoute.Settings(prof, null, q.snapPreventions, newApp);
        for (int i = 0; i < q2.segments.size(); i++) {
            FixRouteRequest.Segment s = q2.segments.get(i);
            FixRouteRequest.Waypoint wa = q2.waypoints.get(i), wb = q2.waypoints.get(i + 1);
            GHPoint pa = new GHPoint(wa.coordinates.lat, wa.coordinates.lng), pb = new GHPoint(wb.coordinates.lat, wb.coordinates.lng);
            if (!s.isFollowRoads()) {
                if (s.trackCoordinates != null && !s.trackCoordinates.isEmpty())
                    for (FixRouteRequest.LatLng c : s.trackCoordinates) shown.add(new double[]{c.lat, c.lng});
                else {
                    shown.add(new double[]{pa.lat, pa.lon});
                    shown.add(new double[]{pb.lat, pb.lon});
                }
                continue;
            }
            List<GHPoint> via = new ArrayList<>();
            for (FixRouteRequest.LatLng v : s.viaPoints) via.add(new GHPoint(v.lat, v.lng));
            ClientRoute.Leg leg = newApp ? ClientRoute.routeAsApp(hopper, st, pa, via, pb, s.initialHeading, s.headingPenalty)
                    : ClientRoute.route(hopper, st, pa, via, pb, s.initialHeading, s.headingPenalty);
            if (!leg.ok()) {
                worst = Math.max(worst, leg.error() != null && leg.error().contains("BarrierRefused") || leg.error() != null && leg.error().contains("closed barrier") ? 3 : 4);
                continue;
            }
            if (newApp && !ClientRoute.crossedBarriers(hopper, st, pa, via, pb, s.initialHeading, s.headingPenalty).isEmpty())
                worst = Math.max(worst, 2);
            shown.addAll(leg.points());
        }
        if (worst < 2 && shown.size() > 1) {
            double dev = 0;
            for (int i = 0; i < pl.size(); i++) dev = Math.max(dev, toLine(new double[]{pl.getLat(i), pl.getLon(i)}, shown));
            if (dev > 30) worst = Math.max(worst, 1);
        }
        String[] names = {"AS SAVED", "DIFFERENT PATH", "CROSSING DIALOG", "REFUSE MESSAGE", "BROKEN"};
        return new String[]{names[worst], status};
    }

    @Test
    void ownerCases() throws Exception {
        Assumptions.assumeTrue(System.getProperty("fr.owner") != null, "fr.owner not set");
        int per = Integer.getInteger("fr.owner");
        hopper = FixRouteTestGraph.open();
        BarrierFixRouteGateLegTest.hopper = hopper;
        BarrierFixRouteGateLegTest.pool = Executors.newFixedThreadPool(4);
        FixRouteCorpusTest.hopper = hopper;
        BaseGraph g = hopper.getBaseGraph();
        NodeAccess na = g.getNodeAccess();
        JsonNode diff = new ObjectMapper().readTree(new File(System.getProperty("fr.diff")));
        Map<String, Map<String, Integer>> table = new TreeMap<>();
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
            // barrier edges by coordinate
            Map<String, Integer> barrierAt = new HashMap<>();
            it = g.getAllEdges();
            while (it.next())
                if (it.getDistance() < 0.01 && na.getLat(it.getBaseNode()) == na.getLat(it.getAdjNode()) && na.getLon(it.getBaseNode()) == na.getLon(it.getAdjNode()))
                    barrierAt.put(Math.round(na.getLat(it.getBaseNode()) * 1e6) + "," + Math.round(na.getLon(it.getBaseNode()) * 1e6), it.getEdge());
            Map<String, List<PointList>> groups = new LinkedHashMap<>();
            // S1-closed: open on the old map, closed now
            String ev = "gravel".equals(prof) ? "bike_access" : "foot_access";
            Set<Integer> seen = new HashSet<>();
            for (JsonNode d : diff.get("access_diffs")) {
                if (!ev.equals(d.get("ev").asText()) || !"true".equals(d.get("before").asText()) || !"false".equals(d.get("after").asText())) continue;
                Integer e = barrierAt.get(Math.round(d.get("lat").asDouble() * 1e6) + "," + Math.round(d.get("lon").asDouble() * 1e6));
                if (e == null || !seen.add(e)) continue;
                EdgeIteratorState be = g.getEdgeIteratorState(e, Integer.MIN_VALUE);
                PointList pl = through(prof, w, e, be.getBaseNode(), be.getAdjNode(), true);
                if (pl != null) groups.computeIfAbsent("S1-closed (open on the old map)", k -> new ArrayList<>()).add(pl);
            }
            // changed-set ask / refuse behind, and ask through
            Map<Integer, List<int[]>> exits = new LinkedHashMap<>();
            for (BarrierClass cls : new BarrierClass[]{BarrierClass.ASK, BarrierClass.REFUSE}) {
                int n = 0;
                it = g.getAllEdges();
                while (it.next()) {
                    if (it.getDistance() >= 0.01 || it.get(enc) != cls || BarrierFixRouteGateLegTest.open(w, it)) continue;
                    if (na.getLat(it.getBaseNode()) != na.getLat(it.getAdjNode()) || na.getLon(it.getBaseNode()) != na.getLon(it.getAdjNode())) continue;
                    int ca = BarrierFixRouteGateLegTest.find(par, it.getBaseNode()), cb = BarrierFixRouteGateLegTest.find(par, it.getAdjNode());
                    if (ca == cb || (ca != main && cb != main)) continue;
                    int outside = ca == main ? it.getBaseNode() : it.getAdjNode();
                    int inside = outside == it.getBaseNode() ? it.getAdjNode() : it.getBaseNode();
                    if (cls == BarrierClass.ASK) exits.computeIfAbsent(BarrierFixRouteGateLegTest.find(par, inside), k -> new ArrayList<>()).add(new int[]{it.getEdge(), outside, inside});
                    if (n >= per) continue;
                    PointList pl = through(prof, w, it.getEdge(), outside, inside, true);
                    if (pl == null) continue;
                    n++;
                    groups.computeIfAbsent(cls == BarrierClass.ASK ? "ask, end behind it" : "refuse, end behind it", k -> new ArrayList<>()).add(pl);
                }
            }
            int n = 0;
            for (List<int[]> ex : exits.values()) {
                if (n >= per || ex.size() < 2) continue;
                int[] g1 = ex.get(0), g2 = ex.get(1);
                GHPoint a = BarrierFixRouteGateLegTest.along(w, g1[1], g1[0], 400), a1 = BarrierFixRouteGateLegTest.along(w, g1[1], g1[0], 1);
                GHPoint i1 = BarrierFixRouteGateLegTest.along(w, g1[2], g1[0], 1), i2 = BarrierFixRouteGateLegTest.along(w, g2[2], g2[0], 1);
                GHPoint b2 = BarrierFixRouteGateLegTest.along(w, g2[1], g2[0], 1), b = BarrierFixRouteGateLegTest.along(w, g2[1], g2[0], 400);
                if (a == null || a1 == null || i1 == null || i2 == null || b2 == null || b == null) continue;
                GHResponse r1 = BarrierFixRouteGateLegTest.route(prof, false, a, a1), rIn = BarrierFixRouteGateLegTest.route(prof, true, i1, i2),
                        r3 = BarrierFixRouteGateLegTest.route(prof, false, b2, b);
                if (r1.hasErrors() || rIn.hasErrors() || r3.hasErrors()) continue;
                PointList pl = new PointList();
                pl.add(r1.getBest().getPoints());
                pl.add(na.getLat(g1[1]), na.getLon(g1[1]));
                pl.add(rIn.getBest().getPoints());
                pl.add(na.getLat(g2[1]), na.getLon(g2[1]));
                pl.add(r3.getBest().getPoints());
                n++;
                groups.computeIfAbsent("ask through (in one gate, out another)", k -> new ArrayList<>()).add(pl);
            }
            for (Map.Entry<String, List<PointList>> grp : groups.entrySet())
                for (PointList pl : grp.getValue())
                    for (boolean newApp : new boolean[]{false, true})
                        for (String policy : new String[]{FixRouteRequest.POLICY_REROUTE, FixRouteRequest.POLICY_COORDINATES}) {
                            String[] v = appView(prof, pl, newApp, policy);
                            String row = prof + " | " + grp.getKey() + " | " + (newApp ? "new app (flag)" : "old app (no flag)") + " | " + policy;
                            table.computeIfAbsent(row, k -> new TreeMap<>()).merge(v[0] + " (fix: " + v[1] + ")", 1, Integer::sum);
                        }
        }
        for (Map.Entry<String, Map<String, Integer>> e : table.entrySet()) System.out.println("OWNER " + e.getKey() + " | " + e.getValue());
        BarrierFixRouteGateLegTest.pool.shutdown();
        hopper.close();
    }
}
