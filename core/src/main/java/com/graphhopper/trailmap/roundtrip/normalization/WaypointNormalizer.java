/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Waypoint normalization: choose the minimal via-points such that routing the standard profile
 * between them reproduces an exploration route, validated by directed edge_key comparison.
 */
package com.graphhopper.trailmap.roundtrip.normalization;

import com.carrotsearch.hppc.IntArrayList;
import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.ResponsePath;
import com.graphhopper.routing.Path;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.trailmap.shared.EdgeKeyMatching;
import com.graphhopper.util.CustomModel;
import com.graphhopper.util.PointList;
import com.graphhopper.util.details.PathDetail;
import com.graphhopper.util.shapes.GHPoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

/**
 * Normalizes exploration routes into the minimal waypoints that reproduce the same path when
 * routed with the standard profile.
 *
 * <p>This is the {@code /convert_track}-proven pattern applied to round-trip normalization:
 * candidate via-points are taken from the exploration path's edge boundaries; each probe routes
 * the standard profile via the <b>public routing API</b> ({@code router}) requesting the
 * {@code edge_key} path detail, and the resulting directed edge_key sequence is compared against
 * the exploration path's edge_key sequence with {@link EdgeKeyMatching}. Directed edge keys make
 * the comparison faithful to traversal direction (so out-and-back / return legs are preserved).
 *
 * <p>The exploration path itself is produced upstream with the anti-backtracking
 * {@code AvoidEdgesWeighting}; this class only <i>reproduces</i> that path — it does not re-derive
 * it — so the penalty-shaped route quality is preserved in the chosen waypoints by construction.
 *
 * <p>Unlike the map-matching optimizer, there is no "coordinates" fallback: a leg that cannot be
 * reproduced by a single routed step simply gets another waypoint inserted (worst case one per
 * exploration edge), keeping the whole result a routed path.
 */
public class WaypointNormalizer {

    private static final Logger logger = LoggerFactory.getLogger(WaypointNormalizer.class);

    /** Per-cursor probe guard (anti-thrash). A cursor's exponential+binary search needs only
     *  O(log span) probes; this caps a pathological cursor, after which best-so-far is committed
     *  and the search continues. Mirrors the optimizer's MAX_PROBES_PER_CURSOR. */
    private static final int MAX_PROBES_PER_CURSOR = 64;

    /** Whole-leg backstop (paranoia only; the cursor strictly advances so the loop is bounded). */
    private static final int MAX_PROBES_PER_LEG = 50_000;

    private final BaseGraph graph;
    private final EdgeKeyMatching ekm;

    /** Twin-edge tolerance for the probe comparison (coincident parallel cycleway/footway over the
     *  same node pair). Default on, mirroring the optimizer. */
    private boolean twinEdgeTolerance = true;

    public WaypointNormalizer(BaseGraph graph) {
        this.graph = graph;
        this.ekm = new EdgeKeyMatching(graph);
    }

    /** Toggle the twin-edge tolerance used during probe comparison (default on). */
    public void setTwinEdgeTolerance(boolean enabled) {
        this.twinEdgeTolerance = enabled;
    }

    /**
     * Normalize exploration routes to the minimal waypoints that reproduce them under the standard
     * profile.
     *
     * @param originalWaypoints  Original waypoints (start, intermediate, end) — one more than legs
     * @param explorationPaths   Exploration paths, one per leg between consecutive waypoints
     * @param profile            Standard (rendering) profile name to route/validate with
     * @param customModel        Optional custom model to pass to the router (may be null)
     * @param router             Public routing entry point (e.g. {@code Router::route})
     * @return Normalization result with waypoints that reproduce the exploration routes
     */
    public NormalizationResult normalize(List<GHPoint> originalWaypoints,
                                         List<Path> explorationPaths,
                                         String profile,
                                         CustomModel customModel,
                                         Function<GHRequest, GHResponse> router) {
        if (originalWaypoints == null || originalWaypoints.size() < 2) {
            return NormalizationResult.failure("Need at least 2 waypoints");
        }
        if (explorationPaths == null || explorationPaths.isEmpty()) {
            return NormalizationResult.failure("No exploration paths provided");
        }
        if (explorationPaths.size() != originalWaypoints.size() - 1) {
            return NormalizationResult.failure("Path count doesn't match waypoint count: "
                    + explorationPaths.size() + " paths for " + originalWaypoints.size() + " waypoints");
        }

        logger.info("Normalizing {} legs with {} total waypoints",
                explorationPaths.size(), originalWaypoints.size());

        List<GHPoint> allWaypoints = new ArrayList<>();
        IntArrayList refAll = new IntArrayList();   // exploration edge_keys (the reference)
        IntArrayList actAll = new IntArrayList();   // edge_keys the chosen routed legs actually produce

        for (int leg = 0; leg < explorationPaths.size(); leg++) {
            GHPoint legStart = originalWaypoints.get(leg);
            GHPoint legEnd = originalWaypoints.get(leg + 1);
            Path explorationPath = explorationPaths.get(leg);

            if (allWaypoints.isEmpty()) {
                allWaypoints.add(legStart);
            }

            int[] rawKeys = PathEdgeExtractor.extractOriginalEdgeKeys(explorationPath);
            if (rawKeys.length == 0) {
                logger.warn("Leg {}: no exploration edges", leg);
                allWaypoints.add(legEnd);
                continue;
            }

            List<EdgeWithIndices> ewi = PathEdgeExtractor.extractEdgesWithIndices(explorationPath);
            PointList polyline = explorationPath.calcPoints();
            int e = rawKeys.length;

            // Candidate via-points: cand[0]=leg start, cand[e]=leg end, cand[k]=boundary after edge k-1.
            GHPoint[] cand = new GHPoint[e + 1];
            cand[0] = legStart;
            cand[e] = legEnd;
            for (int k = 1; k < e; k++) {
                int idx = Math.min(ewi.get(k - 1).getEndPolylineIndex(), polyline.size() - 1);
                cand[k] = new GHPoint(polyline.getLat(idx), polyline.getLon(idx));
            }

            // Reference edge_keys for this leg (dedup'd).
            for (int v : EdgeKeyMatching.dedupConsecutive(rawKeys)) {
                refAll.add(v);
            }

            // Forced waypoints at genuine same-edge directional reversals (out-and-back apexes):
            // rawKeys[i] and rawKeys[i+1] are the two directions of one edge → force candidate i+1.
            // A plain route(A,B) across such an apex would cut the corner, so it must be a waypoint.
            IntArrayList forced = new IntArrayList();
            for (int i = 0; i + 1 < e; i++) {
                if ((rawKeys[i] ^ 1) == rawKeys[i + 1]) {
                    forced.add(i + 1);
                }
            }

            List<GHPoint> legWps = searchLeg(cand, rawKeys, forced, profile, customModel, router, actAll);
            for (int i = 1; i < legWps.size(); i++) {
                allWaypoints.add(legWps.get(i));
            }
        }

        if (allWaypoints.size() < 2) {
            return NormalizationResult.failure("Failed to generate waypoints");
        }

        int[] ref = EdgeKeyMatching.dedupConsecutive(refAll.toArray());
        int[] act = EdgeKeyMatching.dedupConsecutive(actAll.toArray());
        int matched = orderedOverlapCount(ref, act);
        double pct = ref.length > 0 ? matched * 100.0 / ref.length : 0;

        logger.info("Normalization complete: {} waypoints, {}% match ({}/{} edge_keys)",
                allWaypoints.size(), String.format("%.1f", pct), matched, ref.length);

        if (pct >= NormalizationConstants.MIN_MATCH_PERCENTAGE) {
            return NormalizationResult.success(allWaypoints, pct, ref.length, matched);
        }
        return NormalizationResult.bestEffort(allWaypoints, pct, ref.length, matched);
    }

    /**
     * Exponential-extension + binary-refinement search over one leg's candidate boundaries.
     * Returns the chosen waypoints (starting with {@code cand[0]} and ending with the leg end),
     * appending each accepted/forced step's actual edge_keys to {@code actAll} for match scoring.
     */
    private List<GHPoint> searchLeg(GHPoint[] cand, int[] rawKeys, IntArrayList forced,
                                    String profile, CustomModel customModel,
                                    Function<GHRequest, GHResponse> router, IntArrayList actAll) {
        List<GHPoint> chosen = new ArrayList<>();
        chosen.add(cand[0]);
        final int lastCand = rawKeys.length; // candidate index of the leg end
        int cursor = 0;
        int probes = 0;

        while (cursor < lastCand) {
            int step = 1;
            int lastOk = -1;
            int[] lastOkA = null;
            int[] firstProbeA = null; // A of the smallest probe (cursor→cursor+1), for the fail case
            int lastTried = cursor;
            int probesThisCursor = 0;

            int forcedCap = nextForcedAfter(forced, cursor);
            int searchLimit = (forcedCap >= 0) ? Math.min(lastCand, forcedCap) : lastCand;

            while (true) {
                int probeIdx = Math.min(cursor + step, searchLimit);
                if (probeIdx == lastTried && probeIdx != cursor + step) break;
                lastTried = probeIdx;
                ProbeResult pr = probe(cand, rawKeys, cursor, probeIdx, profile, customModel, router);
                probes++;
                probesThisCursor++;
                if (step == 1) firstProbeA = pr.actualKeys;
                if (pr.pass) {
                    lastOk = probeIdx;
                    lastOkA = pr.actualKeys;
                    if (probeIdx == searchLimit) break;
                    if (cursor + step >= searchLimit) break;
                    step *= 2;
                } else {
                    break;
                }
                if (probesThisCursor >= MAX_PROBES_PER_CURSOR) break;
            }

            if (lastOk < 0) {
                // Smallest step did not reproduce the exploration edge — insert a waypoint anyway
                // (no coords fallback). The route still follows SOME path here; record it for scoring.
                int next = cursor + 1;
                chosen.add(cand[next]);
                appendAll(actAll, firstProbeA);
                cursor = next;
                continue;
            }

            int best = lastOk;
            int[] bestA = lastOkA;
            if (lastTried > lastOk) {
                int lo = lastOk + 1, hi = lastTried;
                while (lo <= hi) {
                    int mid = (lo + hi) / 2;
                    ProbeResult pr = probe(cand, rawKeys, cursor, mid, profile, customModel, router);
                    probes++;
                    probesThisCursor++;
                    if (pr.pass) {
                        best = mid;
                        bestA = pr.actualKeys;
                        lo = mid + 1;
                    } else {
                        hi = mid - 1;
                    }
                    if (probesThisCursor >= MAX_PROBES_PER_CURSOR) break;
                }
            }

            chosen.add(cand[best]);
            appendAll(actAll, bestA);
            cursor = best;

            if (probes >= MAX_PROBES_PER_LEG) {
                logger.warn("Normalization hit per-leg probe backstop {}; committing leg end", MAX_PROBES_PER_LEG);
                if (cursor != lastCand) {
                    chosen.add(cand[lastCand]);
                }
                break;
            }
        }

        return chosen;
    }

    /**
     * Probe a standard-profile {@code /route} from {@code cand[start]} to {@code cand[end]} and
     * test whether its directed edge_key sequence reproduces the exploration sub-path.
     */
    private ProbeResult probe(GHPoint[] cand, int[] rawKeys, int start, int end,
                              String profile, CustomModel customModel,
                              Function<GHRequest, GHResponse> router) {
        int[] expected = EdgeKeyMatching.dedupConsecutive(copyOfRange(rawKeys, start, end));

        GHRequest req = new GHRequest(cand[start], cand[end]);
        req.setProfile(profile);
        if (customModel != null) req.setCustomModel(customModel);
        req.setPathDetails(List.of("edge_key"));
        req.putHint("instructions", false);
        req.putHint("calc_points", true);

        GHResponse rsp;
        try {
            rsp = router.apply(req);
        } catch (Exception ex) {
            return ProbeResult.FAIL;
        }
        if (rsp.hasErrors()) return ProbeResult.FAIL;

        ResponsePath path = rsp.getBest();
        List<PathDetail> ek = path.getPathDetails().get("edge_key");
        if (ek == null) return ProbeResult.FAIL;

        int[] actual = EdgeKeyMatching.edgeKeysFromDetails(ek);
        boolean pass = ekm.matches(expected, actual, twinEdgeTolerance);
        return new ProbeResult(pass, actual);
    }

    /** Smallest forced candidate index strictly greater than {@code cursor}, or -1 if none. */
    private static int nextForcedAfter(IntArrayList forced, int cursor) {
        for (int i = 0; i < forced.size(); i++) {
            int f = forced.get(i);
            if (f > cursor) return f;
        }
        return -1;
    }

    /** Count of {@code ref} keys found in {@code act} in order (ordered subsequence overlap). */
    private static int orderedOverlapCount(int[] ref, int[] act) {
        int matched = 0;
        int j = 0;
        for (int r : ref) {
            while (j < act.length) {
                if (act[j] == r) {
                    matched++;
                    j++;
                    break;
                }
                j++;
            }
        }
        return matched;
    }

    private static void appendAll(IntArrayList sink, int[] values) {
        if (values == null) return;
        for (int v : values) sink.add(v);
    }

    private static int[] copyOfRange(int[] arr, int from, int to) {
        int[] out = new int[Math.max(0, to - from)];
        System.arraycopy(arr, from, out, 0, out.length);
        return out;
    }

    private static final class ProbeResult {
        static final ProbeResult FAIL = new ProbeResult(false, new int[0]);
        final boolean pass;
        final int[] actualKeys;

        ProbeResult(boolean pass, int[] actualKeys) {
            this.pass = pass;
            this.actualKeys = actualKeys;
        }
    }
}
