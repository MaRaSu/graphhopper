package com.graphhopper.trailmap.tilerouting.portaltour;

import java.util.*;

/**
 * Exact portal choice for a fixed node order (home, tiles…, home) under plain via routing.
 * <p>
 * Each leg is the one GraphHopper takes ({@link LegOracle.Leg}), so the road cost is exactly what the app's route
 * will cost. On top, for ride quality: a u-turn at a via point (the leg leaves the point back along the edge it
 * arrived on — free for GraphHopper, an out-and-back for the rider) costs {@link #uTurnPenalty}, and a shallow
 * clip portal costs {@link #shallowPenalty}. Because the u-turn depends on the arriving and the leaving leg, a state
 * is (point, arrival direction): second-order Viterbi, at most 12 states per tile.
 */
final class PlainViterbi {

    double uTurnPenalty = 60;
    double shallowPenalty = 60;

    static final class Result {
        double score = Double.POSITIVE_INFINITY;
        /** Road cost only (sum of GraphHopper leg weights). */
        double roadCost;
        int uTurnsAtVias;
        /** Chosen point per position (position 0 and last = home). */
        List<LegOracle.Point> chosen = new ArrayList<>();
        List<LegOracle.Leg> legs = new ArrayList<>();
    }

    Result solve(LegOracle o, List<Integer> order) {
        int m = order.size();
        // states per position: (point index, arrival key); position 0 = home with no arrival
        List<List<int[]>> states = new ArrayList<>();
        List<double[]> score = new ArrayList<>();
        List<int[]> back = new ArrayList<>();
        List<List<LegOracle.Leg>> legOf = new ArrayList<>();
        states.add(List.of(new int[]{0, -1}));
        score.add(new double[]{0});
        back.add(new int[]{-1});
        legOf.add(Collections.singletonList(null));
        for (int i = 1; i < m; i++) {
            int node = order.get(i);
            boolean tileNode = node != 0;
            List<int[]> st = new ArrayList<>();
            Map<Long, Integer> index = new HashMap<>();
            List<Double> sc = new ArrayList<>();
            List<Integer> bp = new ArrayList<>();
            List<LegOracle.Leg> lg = new ArrayList<>();
            List<int[]> prevStates = states.get(i - 1);
            int prevNode = order.get(i - 1);
            for (int s = 0; s < prevStates.size(); s++) {
                double base = score.get(i - 1)[s];
                if (!Double.isFinite(base)) continue;
                int[] ps = prevStates.get(s);
                LegOracle.Point a = o.points.get(prevNode).get(ps[0]);
                LegOracle.Leg[] legs = o.legs(a, node);
                for (int q = 0; q < legs.length; q++) {
                    LegOracle.Leg l = legs[q];
                    if (!Double.isFinite(l.cost)) continue;
                    double v = base + l.cost;
                    if (ps[1] >= 0 && l.depKey == (ps[1] ^ 1)) v += uTurnPenalty;
                    if (tileNode && shallow(o, o.points.get(node).get(q))) v += shallowPenalty;
                    // walk portal (round 3): the walk in and out, priced per metre well above any road
                    double walkM = tileNode ? o.points.get(node).get(q).primary().walkM : 0;
                    if (walkM > 0) v += 2 * walkM * o.f.params.walkWeightPerM;
                    int arr = tileNode ? l.arrKey : -1;
                    long key = ((long) q << 32) | (arr & 0xffffffffL);
                    Integer at = index.get(key);
                    if (at == null) {
                        index.put(key, st.size());
                        st.add(new int[]{q, arr});
                        sc.add(v);
                        bp.add(s);
                        lg.add(l);
                    } else if (v < sc.get(at)) {
                        sc.set(at, v);
                        bp.set(at, s);
                        lg.set(at, l);
                    }
                }
            }
            states.add(st);
            score.add(sc.stream().mapToDouble(Double::doubleValue).toArray());
            back.add(bp.stream().mapToInt(Integer::intValue).toArray());
            legOf.add(lg);
        }
        Result res = new Result();
        double[] last = score.get(m - 1);
        int best = -1;
        for (int s = 0; s < last.length; s++) if (best < 0 || last[s] < last[best]) best = s;
        if (best < 0 || !Double.isFinite(last[best])) return res;
        res.score = last[best];
        LinkedList<LegOracle.Point> chosen = new LinkedList<>();
        LinkedList<LegOracle.Leg> legs = new LinkedList<>();
        int s = best;
        for (int i = m - 1; i >= 1; i--) {
            int[] st = states.get(i).get(s);
            chosen.addFirst(o.points.get(order.get(i)).get(st[0]));
            LegOracle.Leg l = legOf.get(i).get(s);
            legs.addFirst(l);
            res.roadCost += l.cost;
            int prev = back.get(i)[s];
            int[] ps = states.get(i - 1).get(prev);
            if (ps[1] >= 0 && l.depKey == (ps[1] ^ 1)) res.uTurnsAtVias++;
            s = prev;
        }
        chosen.addFirst(o.points.get(0).get(0));
        res.chosen = new ArrayList<>(chosen);
        res.legs = new ArrayList<>(legs);
        return res;
    }

    private static boolean shallow(LegOracle o, LegOracle.Point p) {
        return o.f.shallow(p.primary());
    }
}
