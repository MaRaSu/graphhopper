/*
 * Trailmap - Personal routing, "bit of new" variations
 */
package com.graphhopper.trailmap.personal.variations;

import com.carrotsearch.hppc.IntArrayList;
import com.carrotsearch.hppc.IntHashSet;
import com.carrotsearch.hppc.IntObjectHashMap;
import com.graphhopper.ResponsePath;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.PointList;
import com.graphhopper.util.details.PathDetail;

import java.util.ArrayList;
import java.util.List;

/**
 * The main route as a tower-node sequence, the form both generators start from: node[k] is the tower node between
 * main edges inEdge[k] and outEdge[k], at point index ptIdx[k] of the route's points. Built from a routed
 * {@link ResponsePath} that carries the {@code edge_id} path detail (copied from the prototypes' {@code routeMain} /
 * {@code routeBase}, which built it the same way). Coordinates are {lon, lat} (plus elevation when the route has it).
 */
public final class MainPath {
    public static final String EDGE_ID = "edge_id";

    public PointList points;
    public List<double[]> coords; // [lon, lat(, ele)]
    public int[] node, ptIdx, inEdge, outEdge;
    public double[] along;  // geodesic metres along the points at node[k] (round 4)
    public double[] cumPt;  // geodesic metres along the points at each point
    public IntHashSet edgeSet = new IntHashSet();
    public IntObjectHashMap<IntArrayList> occ = new IntObjectHashMap<>();
    public double distance;

    public int size() {
        return node.length;
    }

    public static MainPath from(BaseGraph g, ResponsePath p) {
        List<PathDetail> det = p.getPathDetails().get(EDGE_ID);
        if (det == null) throw new IllegalArgumentException("the main route needs the " + EDGE_ID + " path detail");
        MainPath b = new MainPath();
        b.points = p.getPoints();
        b.distance = p.getDistance();
        boolean ele = b.points.is3D();
        b.coords = new ArrayList<>(b.points.size());
        for (int i = 0; i < b.points.size(); i++)
            b.coords.add(ele ? new double[]{b.points.getLon(i), b.points.getLat(i), b.points.getEle(i)}
                    : new double[]{b.points.getLon(i), b.points.getLat(i)});
        b.cumPt = new double[b.points.size()];
        for (int i = 1; i < b.points.size(); i++)
            b.cumPt[i] = b.cumPt[i - 1] + DistanceCalcEarth.DIST_EARTH.calcDist(b.points.getLat(i - 1), b.points.getLon(i - 1),
                    b.points.getLat(i), b.points.getLon(i));
        NodeAccess na = g.getNodeAccess();
        int m = det.size();
        int n = Math.max(0, m - 1);
        b.node = new int[n];
        b.ptIdx = new int[n];
        b.inEdge = new int[n];
        b.outEdge = new int[n];
        b.along = new double[n];
        for (PathDetail d : det) b.edgeSet.add(((Number) d.getValue()).intValue());
        for (int k = 0; k < n; k++) {
            int e1 = ((Number) det.get(k).getValue()).intValue(), e2 = ((Number) det.get(k + 1).getValue()).intValue();
            int pi = det.get(k).getLast();
            b.ptIdx[k] = pi;
            b.inEdge[k] = e1;
            b.outEdge[k] = e2;
            b.along[k] = b.cumPt[pi];
            EdgeIteratorState s1 = g.getEdgeIteratorState(e1, Integer.MIN_VALUE), s2 = g.getEdgeIteratorState(e2, Integer.MIN_VALUE);
            int[] c1 = {s1.getBaseNode(), s1.getAdjNode()}, c2 = {s2.getBaseNode(), s2.getAdjNode()};
            int best = -1;
            double bestD = 1.0;
            for (int a : c1)
                for (int c : c2)
                    if (a == c) {
                        double dd = DistanceCalcEarth.DIST_EARTH.calcDist(na.getLat(a), na.getLon(a), b.points.getLat(pi), b.points.getLon(pi));
                        if (dd < bestD) {
                            bestD = dd;
                            best = a;
                        }
                    }
            b.node[k] = best;
            if (best >= 0) {
                IntArrayList l = b.occ.get(best);
                if (l == null) b.occ.put(best, l = new IntArrayList());
                l.add(k);
            }
        }
        return b;
    }

    /** {lon, lat(, ele)} of point t of an edge geometry. */
    static double[] coord(PointList pl, int t) {
        return pl.is3D() ? new double[]{pl.getLon(t), pl.getLat(t), pl.getEle(t)} : new double[]{pl.getLon(t), pl.getLat(t)};
    }
}
