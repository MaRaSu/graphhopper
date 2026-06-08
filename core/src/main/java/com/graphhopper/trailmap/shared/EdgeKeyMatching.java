/*
 * Trailmap - Shared edge_key path matching
 *
 * Bullet-proof route-path equality based on directed edge_key sequences.
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.storage.Graph;
import com.graphhopper.util.AngleCalc;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.PointList;
import com.graphhopper.util.details.PathDetail;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

/**
 * Shared primitive for comparing route paths by their <i>directed edge_key</i> sequences.
 *
 * <p>Two routes follow the same physical path iff their dedup'd edge_key sequences are equal
 * (up to a one-edge boundary tolerance at each end for snap-at-junction artifacts). This is the
 * "bullet-proof" comparison the {@code /convert_track} optimizer uses; it is extracted here so
 * the {@code /convert} and {@code /route} round-trip normalization share exactly one
 * implementation.
 *
 * <p>Directed edge keys distinguish the two traversal directions of an edge ({@code X+} vs
 * {@code X-}), so an out-and-back or a return leg on the same road compares faithfully — unlike
 * undirected edge ids which collapse both directions.
 *
 * <p>The pure helpers ({@link #dedupConsecutive}, {@link #pdValuesInt}, {@link #basicRule},
 * {@link #exitHeadingOf}) are static. The twin-edge tolerance needs the graph (to resolve edge
 * keys and walk parallels) and a per-instance cache, so it lives on an instance constructed with
 * the base {@link Graph}.
 */
public class EdgeKeyMatching {

    private static final AngleCalc ANGLE_CALC = AngleCalc.ANGLE_CALC;

    /** Two edges sharing a node pair are twins only if the longer is &le; this &times; the shorter
     *  — guards against a genuine alternate path between the same junctions being mistaken for a
     *  coincident twin. */
    public static final double TWIN_RATIO_MAX = 1.5;
    /** Below this length the ratio test is skipped (very short edges can't deviate enough to
     *  matter); they count as twins on node-pair alone. */
    public static final double TWIN_MIN_LEN_M = 20.0;

    private final Graph graph;
    /** Cache: graph edge id → canonical (min) edge id of its geometry-guarded twin group. */
    private final HashMap<Integer, Integer> twinCanonCache = new HashMap<>();

    public EdgeKeyMatching(Graph graph) {
        this.graph = graph;
    }

    // ------------------------------------------------------------------------
    // Pure helpers
    // ------------------------------------------------------------------------

    /** Collapse consecutive duplicates in an edge_key (or edge id) sequence. */
    public static int[] dedupConsecutive(int[] xs) {
        if (xs.length == 0) return xs;
        int[] out = new int[xs.length];
        int n = 0;
        int prev = Integer.MIN_VALUE;
        for (int x : xs) {
            if (x != prev) {
                out[n++] = x;
                prev = x;
            }
        }
        return Arrays.copyOf(out, n);
    }

    /** Raw int values of an {@code edge_key} (or {@code edge_id}) path-detail list, no dedup. */
    public static int[] pdValuesInt(List<PathDetail> details) {
        int[] out = new int[details.size()];
        for (int i = 0; i < details.size(); i++) {
            out[i] = ((Number) details.get(i).getValue()).intValue();
        }
        return out;
    }

    /** Convenience: dedup'd edge_key sequence straight from a path-detail list. */
    public static int[] edgeKeysFromDetails(List<PathDetail> details) {
        return dedupConsecutive(pdValuesInt(details));
    }

    /**
     * The basic probe rule: PASS iff {@code A == E[x..len(E)-y]} for some {@code x, y ∈ {0,1}}.
     * The one-edge boundary tolerance absorbs the snap-at-junction case (the leading/trailing
     * edge of the expected sequence may be absent from the actual route when a waypoint snaps at
     * a junction). Identical to the {@code /convert_track} optimizer's edge_key rule.
     */
    public static boolean basicRule(int[] E, int[] A) {
        if (Arrays.equals(E, A)) return true;
        for (int x = 0; x <= 1; x++) {
            for (int y = 0; y <= 1; y++) {
                if (x == 0 && y == 0) continue;
                int remaining = E.length - x - y;
                if (remaining < 0) continue;
                if (remaining == 0) {
                    if (A.length == 0) return true;
                    continue;
                }
                int[] sub = Arrays.copyOfRange(E, x, E.length - y);
                if (Arrays.equals(sub, A)) return true;
            }
        }
        return false;
    }

    /**
     * Exit bearing of a route polyline — the azimuth of its final segment, mirroring how the
     * client (and {@code RouteInstructionGenerator}) derive the next leg's start heading. Returns
     * {@code NaN} when the polyline has fewer than two points.
     */
    public static double exitHeadingOf(PointList pts) {
        int n = pts.size();
        if (n < 2) return Double.NaN;
        return ANGLE_CALC.calcAzimuth(
                pts.getLat(n - 2), pts.getLon(n - 2),
                pts.getLat(n - 1), pts.getLon(n - 1));
    }

    // ------------------------------------------------------------------------
    // Matching with twin-edge tolerance
    // ------------------------------------------------------------------------

    /**
     * Whether actual sequence {@code A} reproduces expected {@code E}, trying {@link #basicRule}
     * first and — when {@code twinEnabled} — the geometry-guarded twin-edge canonicalization as a
     * last resort. Twin tolerance treats coincident parallel edges over the same node pair
     * (e.g. a cycleway and a footway mapped as separate ways over the same stripe) as equivalent,
     * so a path that picks the sibling twin still matches.
     */
    public boolean matches(int[] E, int[] A, boolean twinEnabled) {
        if (basicRule(E, A)) return true;
        if (twinEnabled) {
            return basicRule(twinCanonicalize(E), twinCanonicalize(A));
        }
        return false;
    }

    /**
     * Map an edge_key sequence to DIRECTION-PRESERVING twin-group symbols, then collapse
     * consecutive duplicates. Two edges belong to the same group when they share a node pair AND
     * pass the length guard ({@link #TWIN_MIN_LEN_M} / {@link #TWIN_RATIO_MAX}); the symbol is the
     * group's min edge id times two plus a direction bit. Same-direction twins collapse; a
     * same-edge reversal (a genuine U-turn / out-and-back) keeps two distinct symbols and is NOT
     * flattened.
     */
    public int[] twinCanonicalize(int[] edgeKeys) {
        int[] out = new int[edgeKeys.length];
        for (int i = 0; i < edgeKeys.length; i++) {
            out[i] = twinCanonical(edgeKeys[i]);
        }
        return dedupConsecutive(out);
    }

    private int twinCanonical(int edgeKey) {
        EdgeIteratorState st;
        try {
            st = graph.getEdgeIteratorStateForKey(edgeKey);
        } catch (Exception e) {
            return edgeKey; // unresolvable — keep distinct
        }
        int rep = twinGroupRep(st.getEdge(), st.getBaseNode(), st.getAdjNode(), st.getDistance());
        int dirBit = st.getBaseNode() < st.getAdjNode() ? 0 : 1;
        return rep * 2 + dirBit;
    }

    private int twinGroupRep(int edgeId, int base, int adj, double len) {
        Integer cached = twinCanonCache.get(edgeId);
        if (cached != null) return cached;
        int rep = edgeId;
        EdgeExplorer explorer = graph.createEdgeExplorer();
        EdgeIterator it = explorer.setBaseNode(base);
        while (it.next()) {
            if (it.getAdjNode() != adj) continue;          // only parallels between base↔adj
            if (it.getEdge() == edgeId) continue;          // skip self
            if (!isTwin(len, it.getDistance())) continue;  // geometry guard
            if (it.getEdge() < rep) rep = it.getEdge();
        }
        twinCanonCache.put(edgeId, rep);
        return rep;
    }

    /** Length guard for twin equivalence: very short edges (≤ {@link #TWIN_MIN_LEN_M}) are twins
     *  on node-pair alone; otherwise the longer must be ≤ {@link #TWIN_RATIO_MAX}× the shorter. */
    public static boolean isTwin(double lenA, double lenB) {
        double hi = Math.max(lenA, lenB), lo = Math.min(lenA, lenB);
        if (hi <= TWIN_MIN_LEN_M) return true;
        return lo > 0 && hi <= TWIN_RATIO_MAX * lo;
    }
}
