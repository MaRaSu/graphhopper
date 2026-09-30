package com.graphhopper.trailmap.fixroute;

import com.graphhopper.GraphHopper;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.util.List;

import com.graphhopper.routing.ev.RoadClass;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.FetchMode;
import com.graphhopper.util.PMap;
import com.graphhopper.util.PointList;
import com.graphhopper.util.shapes.BBox;
import com.graphhopper.util.shapes.GHPoint;

/** Diagnostics for single legs: {@code -Dfix.leg=route:leg}. Graph-dependent; skipped otherwise. */
public class FixRouteLegDiagnosticTest {

    @Test
    void singleWaypointSolutions() throws Exception {
        String prop = System.getProperty("fix.leg");
        Assumptions.assumeTrue(prop != null && FixRouteFixtures.available() && FixRouteTestGraph.available());
        GraphHopper hopper = FixRouteTestGraph.open();
        try {
            for (String t : prop.split(",")) {
                String[] rl = t.split(":");
                FixRouteRequest q = FixRouteCorpusTest.request(FixRouteFixtures.load(Long.parseLong(rl[0])));
                if (System.getProperty("fix.tol") != null) q.options.materialityMaxM = Double.parseDouble(System.getProperty("fix.tol"));
                List<Double> arcs = new RouteFixer(hopper, null).singleWaypointSolutions(q, Integer.parseInt(rl[1]));
                System.out.println("SINGLE route-" + rl[0] + " leg " + rl[1] + ": " + arcs.size() + " single-waypoint solutions " + arcs);
            }
        } finally {
            hopper.close();
        }
    }

    /** {@code -Dfix.stretch=route:fromArc:toArc:profile}: ways within 15 m of each 20 m sample. */
    @Test
    void stretchEdges() throws Exception {
        String prop = System.getProperty("fix.stretch");
        Assumptions.assumeTrue(prop != null && FixRouteFixtures.available() && FixRouteTestGraph.available());
        GraphHopper hopper = FixRouteTestGraph.open();
        try {
            String[] a = prop.split(":");
            FixRouteFixtures.Route r = FixRouteFixtures.load(Long.parseLong(a[0]));
            ReferenceTrack ref = new ReferenceTrack(r.reference);
            Weighting w = hopper.createWeighting(hopper.getProfile(a[3]), new PMap());
            EnumEncodedValue<RoadClass> rc = hopper.getEncodingManager().getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
            for (double arc = Double.parseDouble(a[1]) + 10; arc < Double.parseDouble(a[2]); arc += 20) {
                double[] p = ref.pointAt(arc);
                double[] p0 = ref.pointAt(arc - 5), p1 = ref.pointAt(arc + 5);
                double rb = com.graphhopper.util.AngleCalc.ANGLE_CALC.calcAzimuth(p0[0], p0[1], p1[0], p1[1]);
                StringBuilder sb = new StringBuilder(String.format("arc %.0f refBearing %.0f:", arc, rb));
                double rLat = 15 / 111320.0, rLon = 15 / (111320.0 * Math.cos(Math.toRadians(p[0])));
                hopper.getLocationIndex().query(new BBox(p[1] - rLon, p[1] + rLon, p[0] - rLat, p[0] + rLat), id -> {
                    EdgeIteratorState e = hopper.getBaseGraph().getEdgeIteratorState(id, Integer.MIN_VALUE);
                    PointList g = e.fetchWayGeometry(FetchMode.ALL);
                    double best = 1e9, bb = 0;
                    for (int k = 0; k + 1 < g.size(); k++) {
                        double[] x = {g.getLat(k), g.getLon(k)}, y = {g.getLat(k + 1), g.getLon(k + 1)};
                        double d = WaypointAligner.project(p[0], p[1], x, y)[0];
                        if (d < best) { best = d; bb = com.graphhopper.util.AngleCalc.ANGLE_CALC.calcAzimuth(x[0], x[1], y[0], y[1]); }
                    }
                    if (best > 15) return;
                    sb.append(String.format(" [e%d %s d=%.0f brg=%.0f fwd=%s bwd=%s]", id, e.get(rc), best, bb,
                            Double.isFinite(w.calcEdgeWeight(e, false)) ? "ok" : "NO", Double.isFinite(w.calcEdgeWeight(e, true)) ? "ok" : "NO"));
                });
                System.out.println("STRETCH " + sb);
            }
        } finally {
            hopper.close();
        }
    }

    /**
     * {@code -Dfix.leg=route:leg -Dfix.wp=arc}: compare, for the leg's saved slice, (1) the map-matched
     * path, (2) the route with ONE added waypoint at the given arc (snapped like a candidate), and
     * (3) the smallest tolerance at which the geometric verdict accepts that route, against the saved
     * slice and against the matched path. Also edge equality route vs matched path.
     */
    @Test
    void matchVsRoute() throws Exception {
        String leg = System.getProperty("fix.leg"), wpArc = System.getProperty("fix.wp");
        Assumptions.assumeTrue(leg != null && wpArc != null && FixRouteFixtures.available() && FixRouteTestGraph.available());
        GraphHopper hopper = FixRouteTestGraph.open();
        try {
            String[] rl = leg.split(":");
            FixRouteFixtures.Route r = FixRouteFixtures.load(Long.parseLong(rl[0]));
            int i = Integer.parseInt(rl[1]);
            FixRouteFixtures.Segment seg = r.segments.get(i);
            WaypointAligner.Result al = new WaypointAligner(30, 0.5).align(r.reference, r.waypoints, r.savedSegmentLengths);
            ReferenceTrack ref = new ReferenceTrack(r.reference);
            double a = al.placements().get(i).arcM(), b = al.placements().get(i + 1).arcM();
            List<double[]> slice = ref.slice(a, b);
            com.graphhopper.util.CustomModel cm = FixRouteTestGraph.customModel(seg.customModel);

            ReferenceMatcher.Match m = new ReferenceMatcher(hopper).match(slice, seg.profile, cm);
            List<double[]> matched = new java.util.ArrayList<>();
            for (com.graphhopper.matching.EdgeMatch em : m.result().getEdgeMatches()) {
                PointList g = em.getEdgeState().fetchWayGeometry(FetchMode.ALL);
                for (int k = matched.isEmpty() ? 0 : 1; k < g.size(); k++) matched.add(new double[]{g.getLat(k), g.getLon(k)});
            }
            System.out.printf("MVR slice %.0f m, matched path %.0f m, matched edges %d, gaps %d, max snap %.1f m%n",
                    PathSimilarity.length(slice), PathSimilarity.length(matched), m.edgeKeys().length, m.gapObservations(), m.maxSnapM());
            // snap distance profile along the slice
            StringBuilder sp = new StringBuilder("MVR snap distances (every obs, m):");
            for (com.graphhopper.matching.Tracepoint tp : m.result().getTracepoints())
                sp.append(' ').append(tp.getDistance() == null ? "-" : String.format("%.0f", tp.getDistance()));
            System.out.println(sp);

            double[] w = ref.pointAt(Double.parseDouble(wpArc));
            com.graphhopper.storage.index.Snap sn = hopper.getLocationIndex().findClosest(w[0], w[1], com.graphhopper.routing.util.EdgeFilter.ALL_EDGES);
            GHPoint wp = sn.getSnappedPoint();
            ClientRoute.Settings st = new ClientRoute.Settings(seg.profile, cm, r.snapPreventions);
            double[] s0 = r.waypoints.get(i), e0 = r.waypoints.get(i + 1);
            ClientRoute.Leg l1 = ClientRoute.route(hopper, st, new GHPoint(s0[0], s0[1]), List.of(), wp, seg.initialHeading, 60.0);
            ClientRoute.Leg l2 = ClientRoute.route(hopper, st, wp, List.of(), new GHPoint(e0[0], e0[1]), null, 60.0);
            List<double[]> route = new java.util.ArrayList<>(l1.points());
            route.addAll(l2.points().subList(1, l2.points().size()));
            int[] keys = concat(l1.edgeKeys(), l2.edgeKeys());
            System.out.printf("MVR one-waypoint route %.0f m; edges equal to matched path: %s%n",
                    PathSimilarity.length(route), new com.graphhopper.trailmap.shared.EdgeKeyMatching(hopper.getBaseGraph())
                            .matches(m.edgeKeys(), com.graphhopper.trailmap.shared.EdgeKeyMatching.dedupConsecutive(keys), true));
            System.out.printf("MVR smallest passing tolerance: vs saved slice %.1f m, vs matched path %.1f m%n",
                    minTol(route, slice), minTol(route, matched));
            ClientRoute.Leg direct = ClientRoute.route(hopper, st, new GHPoint(s0[0], s0[1]), List.of(), new GHPoint(e0[0], e0[1]), seg.initialHeading, 60.0);
            System.out.printf("MVR no-waypoint route %.0f m: smallest tol vs saved %.1f m, vs matched %.1f m%n",
                    direct.distanceM(), minTol(direct.points(), slice), minTol(direct.points(), matched));
        } finally {
            hopper.close();
        }
    }

    private static int[] concat(int[] x, int[] y) {
        int[] o = java.util.Arrays.copyOf(x, x.length + y.length);
        System.arraycopy(y, 0, o, x.length, y.length);
        return o;
    }

    /** Smallest tolerance (0.5 m resolution) at which the discrete Fréchet coupling exists. */
    private static double minTol(List<double[]> a, List<double[]> b) {
        double lo = 0, hi = 200;
        while (hi - lo > 0.5) {
            double mid = (lo + hi) / 2;
            if (PathSimilarity.compare(a, b, mid).frechetWithin()) hi = mid; else lo = mid;
        }
        return hi;
    }

    /** {@code -Dfix.edges=id,id -Dfix.point=lat,lng -Dfix.profile=p}: edge geometry and the point's snaps. */
    @Test
    void edgesAndSnap() throws Exception {
        String edges = System.getProperty("fix.edges");
        Assumptions.assumeTrue(edges != null && FixRouteTestGraph.available());
        GraphHopper hopper = FixRouteTestGraph.open();
        try {
            String edgeProf = System.getProperty("fix.profile");
            Weighting ew = edgeProf == null ? null : hopper.createWeighting(hopper.getProfile(edgeProf), new PMap());
            for (String e : edges.split(",")) {
                EdgeIteratorState st = hopper.getBaseGraph().getEdgeIteratorState(Integer.parseInt(e.trim()), Integer.MIN_VALUE);
                PointList g = st.fetchWayGeometry(FetchMode.ALL);
                StringBuilder sb = new StringBuilder("EDGE " + e + " base=" + st.getBaseNode() + " adj=" + st.getAdjNode()
                        + " len=" + Math.round(st.getDistance()) + " name=" + st.getName()
                        + (ew == null ? "" : " w(base->adj)=" + ew.calcEdgeWeight(st, false) + " w(adj->base)=" + ew.calcEdgeWeight(st, true)) + " :");
                for (int k = 0; k < g.size(); k++) sb.append(String.format(" (%.6f,%.6f)", g.getLat(k), g.getLon(k)));
                System.out.println(sb);
            }
            String pt = System.getProperty("fix.point");
            if (pt != null) {
                String[] ll = pt.split(",");
                double lat = Double.parseDouble(ll[0]), lon = Double.parseDouble(ll[1]);
                com.graphhopper.storage.index.Snap any = hopper.getLocationIndex().findClosest(lat, lon, com.graphhopper.routing.util.EdgeFilter.ALL_EDGES);
                System.out.println("SNAP any: edge " + any.getClosestEdge().getEdge() + " at " + any.getSnappedPoint() + " d=" + Math.round(any.getQueryDistance() * 10) / 10.0 + " pos=" + any.getSnappedPosition());
                String prof = System.getProperty("fix.profile");
                if (prof != null) {
                    Weighting w = hopper.createWeighting(hopper.getProfile(prof), new PMap());
                    com.graphhopper.routing.util.EdgeFilter f = new com.graphhopper.routing.util.DefaultSnapFilter(w,
                            hopper.getEncodingManager().getBooleanEncodedValue(com.graphhopper.routing.ev.Subnetwork.key(prof)));
                    com.graphhopper.storage.index.Snap us = hopper.getLocationIndex().findClosest(lat, lon, f);
                    System.out.println("SNAP usable(" + prof + "): edge " + us.getClosestEdge().getEdge() + " at " + us.getSnappedPoint() + " d=" + Math.round(us.getQueryDistance() * 10) / 10.0 + " pos=" + us.getSnappedPosition());
                }
            }
        } finally {
            hopper.close();
        }
    }
}
