package com.graphhopper.trailmap.fixroute;

import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.ResponsePath;
import com.graphhopper.matching.EdgeMatch;
import com.graphhopper.storage.Graph;
import com.graphhopper.trailmap.convert.TrackRegion;
import com.graphhopper.trailmap.shared.EdgeKeyMatching;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.details.PathDetail;
import com.graphhopper.util.shapes.GHPoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Minimal-waypoint search for one leg over a map-matched region — the {@code /convert_track}
 * optimizer ({@code RoutedRegionOptimizer}) adapted for {@code /fix_route} as a separate copy, so the
 * {@code /convert_track} code stays untouched until this engine is validated.
 *
 * <p>Same verdict: a sub-leg is accepted iff the route the CLIENT would draw between its two points
 * takes exactly the matched roads — directed edge_key equality with the one-edge boundary tolerance,
 * the junction node-pair tolerance and coincident-twin tolerance. With that verdict a waypoint is
 * only ever added where the route would otherwise take other roads.
 *
 * <p>Differences for {@code /fix_route}: the search runs over the candidate sub-range between the
 * leg's two FIXED waypoints; the fixed ends are routed from their own coordinates (probe parity),
 * the first sub-leg with the leg's stored heading; client snap preventions are applied; a fixed end
 * whose own road snap is on a crossing road may add one stub edge (it cannot be removed by any
 * waypoint — reported separately); heading-aware start snaps must stay put.
 */
final class MatchedLegOptimizer {

    private static final DistanceCalcEarth DIST = DistanceCalcEarth.DIST_EARTH;
    private static final int MAX_PROBES_PER_CURSOR = 64;
    private static final int UTURN_CLEAN_PAIR_COUNT = 2;
    private static final double HEADING_SNAP_STABLE_M = 1.0;
    /** A route's first / last edge travelled for less than this [m] is a snap artefact. */
    static final double MICRO_EDGE_M = 3.0;

    /** One piece of the leg: an accepted sub-leg (routed) or a step no route reproduces. */
    record Piece(int fromCand, int toCand, boolean routed, Double heading, ClientRoute.Leg route) {
    }

    record TolerancePair(int[] matcherKeys, int[] routeKeys) {
    }

    private final GraphHopper hopper;
    private final TrackRegion.Matched region;
    private final ClientRoute.Settings settings;
    private final double penalty;
    private final EdgeKeyMatching ekm;
    private List<TolerancePair> tolerances;
    private final java.util.function.Supplier<List<TolerancePair>> toleranceSupplier;
    private final int[] sliceKeys;
    int probes;
    /** The saved route's own direction of travel leaving the leg's fixed start (heading repair). */
    private Double startDeparture;
    /** Stub allowance [m] at the leg's fixed start / end (waypoint snap off the matched path). */
    double startStubM = 1.0, endStubM = 1.0;

    void setStubs(double startM, double endM) {
        this.startStubM = startM;
        this.endStubM = endM;
    }
    /** Diagnostics: the most recent failed probe per starting candidate (set when non-null). */
    java.util.Map<Integer, String> failTrace;

    /** Second-chance verdict for a probe between two candidates (null = edge verdict only). */
    interface GeometryCheck {
        boolean follows(int candA, int candB, ClientRoute.Leg leg);
    }

    GeometryCheck geometryCheck;

    /**
     * @param tolerances junction node-pair tolerances, computed lazily on the first comparison the
     *                   cheaper rules cannot settle (it routes through every candidate of the range)
     */
    MatchedLegOptimizer(GraphHopper hopper, TrackRegion.Matched region, ClientRoute.Settings settings,
                        double penalty, java.util.function.Supplier<List<TolerancePair>> tolerances) {
        this.hopper = hopper;
        this.region = region;
        this.settings = settings;
        this.penalty = penalty;
        this.ekm = new EdgeKeyMatching(hopper.getBaseGraph());
        this.toleranceSupplier = tolerances;
        List<EdgeMatch> slice = region.edgeMatches();
        this.sliceKeys = new int[slice.size()];
        for (int i = 0; i < slice.size(); i++) sliceKeys[i] = ReferenceMatcher.originalEdgeKey(slice.get(i).getEdgeState());
    }

    // ------------------------------------------------------------------------------------------

    /** Expected (matched) directed edge keys between two candidates, dedup'd. */
    int[] expected(int candA, int candB) {
        int[] idx = region.obsEdgeIdxInSlice();
        int from = Math.min(idx[candA], idx[candB]), to = Math.max(idx[candA], idx[candB]);
        return EdgeKeyMatching.dedupConsecutive(Arrays.copyOfRange(sliceKeys, from, to + 1));
    }

    /**
     * The verdict for a routed leg: it takes exactly the matched roads, apart from
     * <ul>
     *   <li>a MICRO end piece (under {@link #MICRO_EDGE_M}): a snap a hair past a junction onto a twin
     *       or adjacent edge, making the route reach the junction and step back (route 41405);</li>
     *   <li>a STUB at a fixed waypoint whose own road snap is off the matched path (a side street, a
     *       crossing road): the edges from that snap to where the route joins the matched path, up
     *       to {@code stubStartMaxM} / {@code stubEndMaxM}. No added waypoint can remove it; it is
     *       reported as a waypoint snap mismatch instead.</li>
     * </ul>
     */
    boolean reproduces(int[] E, ClientRoute.Leg leg, double stubStartMaxM, double stubEndMaxM) {
        int[] A = EdgeKeyMatching.dedupConsecutive(leg.edgeKeys());
        double[] len = leg.keyLengthsM();
        if (len == null || len.length != A.length) {
            len = new double[A.length];
            Arrays.fill(len, Double.POSITIVE_INFINITY);
            if (A.length > 0) {
                len[0] = leg.firstEdgeM();
                len[A.length - 1] = leg.lastEdgeM();
            }
        }
        // At a fixed end (allowance > 0) the first piece is always droppable — the waypoint's snap
        // edge up to the junction where the matched path continues — plus further pieces within the
        // allowance. Elsewhere only a micro piece.
        double startMax = Math.max(MICRO_EDGE_M, stubStartMaxM), endMax = Math.max(MICRO_EDGE_M, stubEndMaxM);
        // The matched path is trimmed at a fixed end by the same allowance: next to a fixed
        // waypoint it has no evidence of its own (the waypoint's snap decides where the route
        // goes), e.g. it takes the other side of a junction triangle into the waypoint.
        int eStart = stubStartMaxM > 0 ? droppable(E, true, stubStartMaxM) : 0;
        int eEnd = stubEndMaxM > 0 ? droppable(E, false, stubEndMaxM) : 0;
        double pre = 0;
        for (int p = 0; p < A.length; p++) {
            if (p > 1 || (p == 1 && stubStartMaxM <= 0)) pre += len[p - 1];
            if (pre > startMax) break;
            double suf = 0;
            for (int q = 0; q < A.length - p; q++) {
                if (q > 1 || (q == 1 && stubEndMaxM <= 0)) suf += len[A.length - q];
                if (suf > endMax) break;
                if (A.length - p - q < 1) break;
                int[] Ac = Arrays.copyOfRange(A, p, A.length - q);
                for (int ep = 0; ep <= eStart; ep++) {
                    for (int eq = 0; eq <= eEnd && E.length - ep - eq >= 1; eq++) {
                        if (matches(ep == 0 && eq == 0 ? E : Arrays.copyOfRange(E, ep, E.length - eq), Ac)) return true;
                    }
                }
            }
        }
        return false;
    }

    /**
     * How many matched pieces may be dropped at one end: the end piece (the partial edge the
     * waypoint lies on) always, then whole edges while their total stays within {@code maxM}.
     */
    private int droppable(int[] E, boolean fromStart, double maxM) {
        int n = 0;
        double sum = 0;
        for (int k = 0; k < E.length - 1; k++) {
            int key = E[fromStart ? k : E.length - 1 - k];
            if (k > 0) {
                sum += hopper.getBaseGraph().getEdgeIteratorStateForKey(key).getDistance();
                if (sum > maxM) break;
            }
            n++;
        }
        return n;
    }

    private boolean matches(int[] E, int[] A) {
        if (ekm.matches(E, A, true)) return true; // basic rule, then coincident twins
        if (tolerances == null) tolerances = toleranceSupplier.get();
        if (!tolerances.isEmpty()) {
            int[] Es = applyTolerances(E, tolerances);
            return !Arrays.equals(E, Es) && EdgeKeyMatching.basicRule(Es, A);
        }
        return false;
    }

    // ------------------------------------------------------------------------------------------

    /**
     * Search the candidate range [{@code cFrom}, {@code cTo}]. {@code startPos}/{@code endPos}: the
     * fixed waypoints' own coordinates (null = the candidate's snap). {@code startHeading}: the leg's
     * stored heading for the first sub-leg.
     */
    List<Piece> optimize(int cFrom, int cTo, GHPoint startPos, GHPoint endPos, Double startHeading,
                         Double startDeparture) {
        this.startDeparture = startDeparture;
        // (stub allowances are set by the caller via setStubs before optimizing)
        List<Piece> out = new ArrayList<>();
        int[] forced = uTurnForcedCandidates(cFrom, cTo);
        int cursor = cFrom;
        Double heading = startHeading;
        while (cursor < cTo) {
            int cap = cTo;
            for (int f : forced) if (f > cursor) { cap = f; break; }
            Piece lastOk = null;
            int lastTried = cursor, perCursor = 0;
            // Exponential extension, then binary refinement (as /convert_track).
            for (int step = 1; ; step *= 2) {
                int k = Math.min(cursor + step, cap);
                if (k == lastTried) break;
                lastTried = k;
                Piece p = probe(cursor, k, heading, cFrom, cTo, startPos, endPos);
                perCursor++;
                if (!p.routed()) break;
                lastOk = p;
                if (k == cap || perCursor >= MAX_PROBES_PER_CURSOR) break;
            }
            if (lastOk != null && lastTried > lastOk.toCand()) {
                int lo = lastOk.toCand() + 1, hi = lastTried - 1;
                while (lo <= hi && perCursor < MAX_PROBES_PER_CURSOR) {
                    int mid = (lo + hi) >>> 1;
                    Piece p = probe(cursor, mid, heading, cFrom, cTo, startPos, endPos);
                    perCursor++;
                    if (p.routed()) {
                        lastOk = p;
                        lo = mid + 1;
                    } else {
                        hi = mid - 1;
                    }
                }
            }
            if (lastOk == null) {
                out.add(new Piece(cursor, cursor + 1, false, null, null)); // no route reproduces this step
                cursor++;
                heading = null;
            } else {
                out.add(lastOk);
                cursor = lastOk.toCand();
                heading = Double.isNaN(lastOk.route().exitHeading()) ? null : lastOk.route().exitHeading();
            }
        }
        return out;
    }

    /** Diagnostics: candidates' slice positions and the matched keys around them. */
    private String neighbourhood(int a, int b) {
        int[] idx = region.obsEdgeIdxInSlice();
        StringBuilder sb = new StringBuilder("candIdx:");
        for (int c = Math.max(0, a - 3); c <= Math.min(idx.length - 1, b + 3); c++) sb.append(' ').append(c).append('@').append(idx[c]);
        int lo = Math.max(0, Math.min(idx[a], idx[b]) - 3), hi = Math.min(sliceKeys.length - 1, Math.max(idx[a], idx[b]) + 3);
        sb.append(" slice[").append(lo).append("..]:");
        for (int k = lo; k <= hi; k++) sb.append(' ').append(sliceKeys[k]);
        return sb.toString();
    }

    /** Point the client routes from/to for a candidate. */
    GHPoint position(int cand, int cFrom, int cTo, GHPoint startPos, GHPoint endPos) {
        if (cand == cFrom && startPos != null) return startPos;
        if (cand == cTo && endPos != null) return endPos;
        GHPoint s = region.obsSnapPoints().get(cand);
        return new GHPoint(RouteFixerMath.round6(s.lat), RouteFixerMath.round6(s.lon));
    }

    Piece probe(int a, int b, Double heading, int cFrom, int cTo, GHPoint startPos, GHPoint endPos) {
        GHPoint pa = position(a, cFrom, cTo, startPos, endPos), pb = position(b, cFrom, cTo, startPos, endPos);
        int[] E = expected(a, b);
        double stubStart = a == cFrom && startPos != null ? startStubM : 0, stubEnd = b == cTo && endPos != null ? endStubM : 0;
        List<Double> tries = new ArrayList<>(2);
        if (a == cFrom) {
            tries.add(heading);
            if (heading != null) tries.add(null);
            // Heading repair: the saved route's departure direction — right also where the route
            // turns back at the waypoint, where the arrival direction would be wrong.
            if (startDeparture != null) tries.add(startDeparture);
        } else {
            tries.add(null);
            if (heading != null) tries.add(heading);
        }
        for (Double h : tries) {
            ClientRoute.Leg leg = ClientRoute.route(hopper, settings, pa, List.of(), pb, h, penalty);
            probes++;
            if (!leg.ok()) continue;
            boolean storedHeading = a == cFrom && h != null && h.equals(heading);
            if (h != null && !storedHeading) {
                double[] s = leg.points().get(0);
                if (DIST.calcDist(pa.lat, pa.lon, s[0], s[1]) > HEADING_SNAP_STABLE_M) continue;
            }
            // With a geometry check (the deviation rule) it alone decides, so a created piece is
            // judged exactly as the whole leg will be on the next run; otherwise the edge verdict.
            if (geometryCheck != null ? geometryCheck.follows(a, b, leg) : reproduces(E, leg, stubStart, stubEnd))
                return new Piece(a, b, true, h, leg);
            if (failTrace != null && b == a + 1) {
                failTrace.put(a, "cand " + a + "->" + b + " h=" + h + " E=" + Arrays.toString(E)
                        + " A=" + Arrays.toString(EdgeKeyMatching.dedupConsecutive(leg.edgeKeys()))
                        + " from=" + pa + " to=" + pb + " " + neighbourhood(a, b));
            }
        }
        return new Piece(a, b, false, null, null);
    }

    // ------------------------------------------------------------------------------------------
    // Copied from RoutedRegionOptimizer (junction node-pair tolerance, U-turn apex).
    // ------------------------------------------------------------------------------------------

    /**
     * At region start, compare the matched edge sequence with {@code /route(all region snaps)}; a
     * divergent middle spanning the same pair of graph nodes is a micro-alternative at a junction.
     */
    List<TolerancePair> nodePairTolerances(int cFrom, int cTo) {
        return nodePairTolerances(hopper, region, settings, cFrom, cTo, expected(cFrom, cTo));
    }

    /** As in /convert_track, but over the candidate range [cFrom, cTo] of one leg only. */
    static List<TolerancePair> nodePairTolerances(GraphHopper hopper, TrackRegion.Matched region,
                                                  ClientRoute.Settings s, int cFrom, int cTo, int[] E) {
        if (cTo - cFrom < 1) return List.of();
        GHRequest req = new GHRequest(new ArrayList<>(region.obsSnapPoints().subList(cFrom, cTo + 1)));
        req.setProfile(s.profile());
        if (s.customModel() != null) req.setCustomModel(s.customModel());
        req.setSnapPreventions(s.snapPreventions() == null ? List.of() : s.snapPreventions());
        req.setPathDetails(List.of("edge_key"));
        req.putHint("instructions", false);
        req.putHint("calc_points", true);
        GHResponse rsp;
        try {
            rsp = hopper.route(req);
        } catch (Exception e) {
            return List.of();
        }
        if (rsp.hasErrors()) return List.of();
        ResponsePath path = rsp.getBest();
        List<PathDetail> ek = path.getPathDetails().get("edge_key");
        if (ek == null) return List.of();
        int[] A = EdgeKeyMatching.edgeKeysFromDetails(ek);
        Graph graph = hopper.getBaseGraph();
        for (int x = 0; x <= 1; x++) {
            for (int y = 0; y <= 1; y++) {
                if (E.length - x - y < 1) continue;
                int[] Et = Arrays.copyOfRange(E, x, E.length - y);
                int pre = 0;
                while (pre < Et.length && pre < A.length && Et[pre] == A[pre]) pre++;
                int suf = 0;
                while (suf < Et.length - pre && suf < A.length - pre
                        && Et[Et.length - 1 - suf] == A[A.length - 1 - suf]) suf++;
                if (Et.length - pre - suf == 0 || A.length - pre - suf == 0) continue;
                int[] eMid = Arrays.copyOfRange(Et, pre, Et.length - suf);
                int[] aMid = Arrays.copyOfRange(A, pre, A.length - suf);
                try {
                    EdgeIteratorState eF = graph.getEdgeIteratorStateForKey(eMid[0]);
                    EdgeIteratorState eL = graph.getEdgeIteratorStateForKey(eMid[eMid.length - 1]);
                    EdgeIteratorState aF = graph.getEdgeIteratorStateForKey(aMid[0]);
                    EdgeIteratorState aL = graph.getEdgeIteratorStateForKey(aMid[aMid.length - 1]);
                    if (eF.getBaseNode() == aF.getBaseNode() && eL.getAdjNode() == aL.getAdjNode()) {
                        return List.of(new TolerancePair(eMid, aMid));
                    }
                } catch (Exception ignored) {
                    // skip unresolvable keys
                }
            }
        }
        return List.of();
    }

    private static int[] applyTolerances(int[] E, List<TolerancePair> tolerances) {
        List<Integer> r = new ArrayList<>(E.length);
        int i = 0;
        while (i < E.length) {
            boolean sub = false;
            for (TolerancePair tp : tolerances) {
                int[] pat = tp.matcherKeys();
                if (i + pat.length <= E.length && Arrays.equals(Arrays.copyOfRange(E, i, i + pat.length), pat)) {
                    for (int k : tp.routeKeys()) r.add(k);
                    i += pat.length;
                    sub = true;
                    break;
                }
            }
            if (!sub) r.add(E[i++]);
        }
        int[] out = new int[r.size()];
        for (int j = 0; j < out.length; j++) out[j] = r.get(j);
        return EdgeKeyMatching.dedupConsecutive(out);
    }

    /** U-turn apexes (same edge, flipped direction, exactly twice in the slice) inside the range. */
    private int[] uTurnForcedCandidates(int cFrom, int cTo) {
        List<EdgeMatch> slice = region.edgeMatches();
        int[] candEdgeIdx = region.obsEdgeIdxInSlice();
        List<Integer> forced = new ArrayList<>();
        for (int i = 0; i + 1 < slice.size(); i++) {
            int kA = sliceKeys[i], kB = sliceKeys[i + 1];
            if ((kA ^ 1) != kB) continue;
            int edgeId = kA >> 1, count = 0;
            for (int k : sliceKeys) if ((k >> 1) == edgeId) count++;
            if (count != UTURN_CLEAN_PAIR_COUNT) continue;
            int first = 0, second = 0, firstAtSecond = -1;
            for (int k = cFrom; k <= cTo; k++) {
                if (candEdgeIdx[k] == i) first++;
                else if (candEdgeIdx[k] == i + 1) {
                    second++;
                    if (firstAtSecond < 0) firstAtSecond = k;
                }
            }
            if (first == 0 || second == 0 || firstAtSecond <= cFrom || firstAtSecond >= cTo) continue;
            if (!forced.contains(firstAtSecond)) forced.add(firstAtSecond);
        }
        return forced.stream().mapToInt(Integer::intValue).sorted().toArray();
    }
}
