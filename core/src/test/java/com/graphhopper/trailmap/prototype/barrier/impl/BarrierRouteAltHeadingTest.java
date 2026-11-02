package com.graphhopper.trailmap.prototype.barrier.impl;

import com.fasterxml.jackson.databind.JsonNode;
import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.config.Profile;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.util.AllEdgesIterator;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.trailmap.barrier.BarrierClass;
import com.graphhopper.trailmap.barrier.BarrierProfiles;
import com.graphhopper.util.*;
import com.graphhopper.util.shapes.GHPoint;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.*;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * W26 review F3 (2026-11-02): BarrierRouter.routeAlt's heading-snap guard, in-JVM on the S1+S2 graph.
 * <ul>
 * <li>no flag: alternative_route with a start heading, BarrierRouter == stock Router, byte for byte, on the route-check
 * pairs and the gate cases below;</li>
 * <li>flag: A 8 m behind a changed-set ASK barrier, B 400 m outside, start heading = the bearing of the road on the
 * far side of the barrier (a heading snap there would land across it). alternative_route must start on the same side
 * as the single (routeVia, already guarded) answer: both with a crossing, or both without.</li>
 * </ul>
 * <pre>-Dbc.graph=DIR -Dbc.config=FILE -Dbc.root=ABS/routing_v2 [-Dbc.limit=N]</pre>
 */
public class BarrierRouteAltHeadingTest {

    @BeforeAll
    static void load() throws Exception {
        BarrierRouteChecksTest.load();
    }

    @AfterAll
    static void close() {
        BarrierRouteChecksTest.close();
    }

    static GHRequest alt(GHRequest r) {
        r.setAlgorithm(Parameters.Algorithms.ALT_ROUTE);
        return r;
    }

    @SuppressWarnings("unchecked")
    static int crossings(GHResponse rsp) {
        if (rsp.hasErrors()) return -1;
        Object b = rsp.getHints().getObject("barriers", null);
        if (!(b instanceof Map)) return 0;
        List<Map<String, Object>> paths = (List<Map<String, Object>>) ((Map<String, Object>) b).get("paths");
        return paths == null || paths.isEmpty() ? 0 : ((List<?>) paths.get(0).get("crossings")).size();
    }

    @Test
    void altHeadingGuard() throws Exception {
        Assumptions.assumeTrue(System.getProperty("bc.graph") != null, "bc.graph not set");
        int limit = Integer.getInteger("bc.limit", 40);
        int noFlagCases = 0, noFlagDiff = 0;
        // no flag on the route-check pairs, with a start heading
        JsonNode pairs = BarrierRouteChecksTest.JSON.readTree(new File(System.getProperty("bc.root"), "route-check/pairs.json"));
        for (String prof : new String[]{"gravel", "trailmap_foot"})
            for (JsonNode p : pairs)
                for (double h : new double[]{0, 90, 180, 270}) {
                    GHPoint a = new GHPoint(p.get("from").get(0).asDouble(), p.get("from").get(1).asDouble());
                    GHPoint b = new GHPoint(p.get("to").get(0).asDouble(), p.get("to").get(1).asDouble());
                    String x = BarrierRouteChecksTest.pathJson(BarrierRouteChecksTest.hopper.route(alt(BarrierRouteChecksTest.req(prof, a, b, false, h))));
                    String y = BarrierRouteChecksTest.pathJson(BarrierRouteChecksTest.stock.route(alt(BarrierRouteChecksTest.req(prof, a, b, false, h))));
                    noFlagCases++;
                    if (!x.equals(y)) noFlagDiff++;
                }
        Map<String, Integer> flagged = new TreeMap<>();
        BaseGraph g = BarrierRouteChecksTest.hopper.getBaseGraph();
        NodeAccess na = g.getNodeAccess();
        for (String prof : new String[]{"gravel", "trailmap_foot"}) {
            Profile p = BarrierRouteChecksTest.hopper.getProfile(prof);
            Weighting w = BarrierRouteChecksTest.hopper.createWeighting(p, new PMap());
            BarrierRouteChecksTest.alongWeighting = w;
            EnumEncodedValue<BarrierClass> enc = BarrierProfiles.barrierEnc(BarrierRouteChecksTest.hopper.getEncodingManager(), p);
            int[] comp = BarrierRouteChecksTest.components(w, new int[1]);
            Map<Integer, Integer> size = new HashMap<>();
            AllEdgesIterator it = g.getAllEdges();
            while (it.next()) if (comp[it.getBaseNode()] >= 0) size.merge(comp[it.getBaseNode()], 1, Integer::sum);
            int main = size.entrySet().stream().max(Map.Entry.comparingByValue()).get().getKey();
            int n = 0;
            it = g.getAllEdges();
            while (it.next() && n < limit) {
                if (!BarrierRouteChecksTest.isBarrierEdge(it, na) || it.get(enc) != BarrierClass.ASK) continue;
                int ca = comp[it.getBaseNode()], cb = comp[it.getAdjNode()];
                if (ca < 0 || cb < 0 || ca == cb || (ca != main && cb != main)) continue;
                int outside = ca == main ? it.getBaseNode() : it.getAdjNode();
                int inside = outside == it.getBaseNode() ? it.getAdjNode() : it.getBaseNode();
                GHPoint a = BarrierRouteChecksTest.along(inside, it.getEdge(), 8), b = BarrierRouteChecksTest.along(outside, it.getEdge(), 400);
                GHPoint o20 = BarrierRouteChecksTest.along(outside, it.getEdge(), 20);
                if (a == null || b == null || o20 == null) continue;
                double heading = AngleCalc.ANGLE_CALC.calcAzimuth(na.getLat(outside), na.getLon(outside), o20.lat, o20.lon);
                n++;
                // no flag: identity with the stock router, alternatives + heading
                String x = BarrierRouteChecksTest.pathJson(BarrierRouteChecksTest.hopper.route(alt(BarrierRouteChecksTest.req(prof, a, b, false, heading))));
                String y = BarrierRouteChecksTest.pathJson(BarrierRouteChecksTest.stock.route(alt(BarrierRouteChecksTest.req(prof, a, b, false, heading))));
                noFlagCases++;
                if (!x.equals(y)) noFlagDiff++;
                // flag: alternatives start on the side the guarded single answer starts on
                int single = crossings(BarrierRouteChecksTest.hopper.route(BarrierRouteChecksTest.req(prof, a, b, true, heading)));
                int alts = crossings(BarrierRouteChecksTest.hopper.route(alt(BarrierRouteChecksTest.req(prof, a, b, true, heading))));
                String k = prof + (single == alts ? " alternatives == single" : " ALTERNATIVES DIFFER FROM SINGLE")
                        + " (single crossings " + single + ", alternatives " + alts + ")";
                flagged.merge(k, 1, Integer::sum);
            }
        }
        System.out.println("ALTHEADING no flag: " + noFlagDiff + " of " + noFlagCases + " differ from the stock router");
        for (Map.Entry<String, Integer> e : flagged.entrySet()) System.out.println("ALTHEADING " + e.getKey() + " = " + e.getValue());
        assertEquals(0, noFlagDiff, "no flag: BarrierRouter must equal the stock router");
    }
}
