package com.graphhopper.trailmap.convert;

import com.graphhopper.matching.EdgeMatch;
import com.graphhopper.matching.MatchResult;
import com.graphhopper.matching.Observation;
import com.graphhopper.matching.State;
import com.graphhopper.matching.Tracepoint;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.shapes.GHPoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;

/**
 * Stage 2 of the track-to-route conversion pipeline.
 *
 * <p>Walks the {@link MatchResult} and partitions the input track into a sequence of
 * {@link TrackRegion}s: alternating runs of matched (followable by road network) and
 * unmatched (raw GPX) observations.
 *
 * <p>The segmenter decides, per region of the track, which client-side rendering mode
 * applies: {@code Matched} → re-routed via {@code /route} between waypoints,
 * {@code Unmatched} → drawn as raw GPX coordinates. Detour detection is one tool the
 * segmenter uses — not its job. See {@code docs/gh_convert_track_implementation_v2.md}
 * §3 for the first-principles framing.
 *
 * <p>Three reasons a stretch becomes Unmatched (all producing coordinates regions):
 * <ol>
 *   <li><b>Off-network observations.</b> Snap distance over threshold.
 *     <i>Signal:</i> {@link Tracepoint#getDistance()}.</li>
 *   <li><b>Apparent detour at a transition.</b> The matcher's Viterbi transition between
 *     two consecutive non-filtered observations is much longer than the straight-line
 *     between them. Covers both real obstacle-detours (e.g. railway crossing) and
 *     off-network transition artifacts (one snap arbitrary, transition includes the
 *     artificial travel).
 *     <i>Signal:</i> {@link Tracepoint#getDistanceFromPrevious()}.</li>
 *   <li><b>Too-short matched stretch.</b> Even where matched output is faithful, a
 *     sub-{@code minRoutedSegmentM} matched region adds noise without value.
 *     <i>Signal:</i> sum of {@link Tracepoint#getDistanceFromPrevious()} across the
 *     region.</li>
 * </ol>
 *
 * <p>The algorithm mirrors the OSRM client implementation
 * ({@code trailmap/shared/models/route-convert.ts}) which is the reference design.
 */
public class RegionSegmenter {

    private static final Logger LOGGER = LoggerFactory.getLogger(RegionSegmenter.class);
    private static final DistanceCalcEarth DIST = DistanceCalcEarth.DIST_EARTH;

    /** Drift-trim tight-snap floor. At a matched-region boundary, the segmenter walks
     *  inward and demotes each GOOD observation whose snap exceeds this floor, stopping
     *  at the first observation with snap ≤ floor (confidently on the road).
     *
     *  <p>Replaces an earlier ratio-based rule (compare snap to immediate neighbor) which
     *  failed on gradual climbs: when both the boundary obs and its neighbor are already
     *  drifting, their ratio is small (~1.1–1.6×) even though the boundary is clearly
     *  off-road. The absolute floor handles immediate jumps AND gradual climbs uniformly.
     *
     *  <p>Per design principle (docs/gh_convert_track_implementation_v2.md §3): a slightly
     *  too-long coordinates region is cheap; a routed segment drifting onto a wrong OSM
     *  way is expensive. 5 m floor separates "on the road within GPS noise" from
     *  "measurably off the road". */
    private static final double DRIFT_TRIM_TIGHT_FLOOR_M = 5.0;

    /** Detour entries accepted by the most recent {@link #segment} call. Populated for
     *  debug/diagnostic consumers (the converter pulls these when the request opted into
     *  debug output). Cleared at the start of each segment() call. */
    private final List<DetourReport> lastDetours = new ArrayList<>();

    public List<DetourReport> getLastDetours() { return lastDetours; }

    /** Per-detour signal record for diagnostic consumers. */
    public record DetourReport(int fromObs, int toObs,
                               double matchedLengthM, double straightM, double ratio) {}

    private enum Status { GOOD, BAD, DETOUR_BOUNDARY }

    /** Intermediate region representation before converting to {@link TrackRegion}. */
    private record RawRegion(boolean isMatched, int firstObs, int lastObs) {}

    /**
     * Segment the track. Parameters:
     * <ul>
     *   <li>{@code observations}: the original observation list that was passed into
     *     {@link com.graphhopper.matching.MapMatching#match}. Required for
     *     direction-aware obs→EdgeMatch attribution via {@link State#getEntry()}
     *     reference identity. Must be the same list (or equivalent reference list)
     *     the matcher consumed.</li>
     *   <li>{@code snapThresholdM}: tracepoint snap distance ceiling (OSRM uses 35 m).</li>
     *   <li>{@code minDetourM}: per-Viterbi-transition minimum matched-leg length to be
     *     flagged as a detour (OSRM uses 75 m).</li>
     *   <li>{@code maxDetourRatio}: per-Viterbi-transition matched/straight ratio
     *     threshold (OSRM uses 2.0).</li>
     *   <li>{@code minRoutedSegmentM}: minimum matched-region length to remain matched;
     *     shorter regions get demoted to coordinates (OSRM uses 40 m).</li>
     * </ul>
     */
    public List<TrackRegion> segment(MatchResult matchResult,
                                     List<Observation> observations,
                                     double snapThresholdM,
                                     double minDetourM,
                                     double maxDetourRatio,
                                     double minRoutedSegmentM) {
        lastDetours.clear();
        List<Tracepoint> tps = matchResult.getTracepoints();
        if (tps == null || tps.isEmpty()) return List.of();
        int n = tps.size();
        List<EdgeMatch> edgeMatches = matchResult.getEdgeMatches();

        // Map each observation to its EdgeMatch index (needed downstream by buildMatched
        // to construct edge slices for the optimizer).
        int[] obsToEdgeMatch = mapObsToEdgeMatch(tps, observations, edgeMatches);

        // Stage A — per-Viterbi-transition detour flags.
        // For each non-filtered matched obs with distanceFromPrevious set, compare the
        // matcher's HMM transition distance to the straight-line distance from the
        // previous Viterbi participant. Flag the obs if both gates are met.
        boolean[] direct = computeDirectFlags(tps, n, minDetourM, maxDetourRatio);

        // Stage B — snap classification.
        Status[] status = new Status[n];
        for (int i = 0; i < n; i++) {
            Tracepoint tp = tps.get(i);
            if (tp.isMatched() && tp.getDistance() != null && tp.getDistance() < snapThresholdM) {
                status[i] = Status.GOOD;
            } else {
                status[i] = Status.BAD;
            }
        }

        // Stage B.5 — drift trim at matched-region boundaries.
        // Snap distance is a lagging indicator of road departure: a user starting to
        // leave the road has snap climb gradually (e.g. 2m → 8m → 12m → 19m → 30m → 35m).
        // The first observation crossing the absolute snap threshold isn't where the
        // user actually left — it's just the first one too far for the matcher to find
        // any snap. The actual departure is several observations earlier.
        //
        // Rule (symmetric, iterative): at a matched-region boundary (next or previous
        // observation is BAD), demote the boundary GOOD observation if its snap exceeds
        // DRIFT_TRIM_TIGHT_FLOOR_M. Iterate — demoting one boundary exposes its
        // predecessor as the new boundary, which then gets re-checked. Stops when the
        // boundary observation has tight snap (≤ floor), confidently on the road.
        //
        // Handles both immediate jumps and gradual climbs uniformly: the floor is an
        // absolute statement about "definitely on road" rather than a relative test
        // against an already-drifting neighbor.
        boolean driftChanged = true;
        while (driftChanged) {
            driftChanged = false;
            for (int i = 0; i < n; i++) {
                if (status[i] != Status.GOOD) continue;
                Double snap = tps.get(i).getDistance();
                if (snap == null || snap <= DRIFT_TRIM_TIGHT_FLOOR_M) continue;
                boolean atEndBoundary = (i == n - 1 || status[i + 1] == Status.BAD);
                boolean atStartBoundary = (i == 0 || status[i - 1] == Status.BAD);
                if (atEndBoundary || atStartBoundary) {
                    status[i] = Status.BAD;
                    driftChanged = true;
                }
            }
        }

        // Stage C — de-singleton (mirrors OSRM rules):
        //   boundary GOOD with BAD neighbor → BAD;
        //   interior GOOD with BAD on both sides → BAD.
        Status[] orig = status.clone();
        if (n >= 2) {
            if (orig[0] == Status.GOOD && orig[1] == Status.BAD) status[0] = Status.BAD;
            if (orig[n - 1] == Status.GOOD && orig[n - 2] == Status.BAD) status[n - 1] = Status.BAD;
        }
        for (int i = 1; i < n - 1; i++) {
            if (orig[i] == Status.GOOD && orig[i - 1] == Status.BAD && orig[i + 1] == Status.BAD) {
                status[i] = Status.BAD;
            }
        }

        // Stage D — promote GOOD to DETOUR_BOUNDARY where the per-pair signal fired.
        for (int i = 0; i < n; i++) {
            if (direct[i] && status[i] == Status.GOOD) {
                status[i] = Status.DETOUR_BOUNDARY;
            }
        }

        // Stage E — build raw regions from status array (obs ranges only).
        List<RawRegion> raw = buildRawRegions(status, n);

        // Stage F — post-process: drop single-obs MATCHED (they have no edge slice to
        // optimize against), then merge consecutive COORDS regions (they share a
        // boundary obs).
        raw = dropSingletonMatched(raw);
        raw = mergeConsecutiveCoords(raw);

        // Stage G — demote MATCHED regions whose matched-path length is below
        // minRoutedSegmentM to COORDS; merge consecutive COORDS again.
        raw = demoteShortMatched(raw, tps, minRoutedSegmentM);
        raw = mergeConsecutiveCoords(raw);

        // Convert raw regions to TrackRegion (Matched needs the edge slice + obs mapping
        // built up for the optimizer downstream).
        List<TrackRegion> regions = new ArrayList<>();
        for (RawRegion r : raw) {
            if (r.isMatched()) {
                regions.add(buildMatched(tps, edgeMatches, obsToEdgeMatch, r.firstObs(), r.lastObs()));
            } else {
                regions.add(new TrackRegion.Unmatched(r.firstObs(), r.lastObs()));
            }
        }

        LOGGER.info("RegionSegmenter: {} observations → {} regions ({} matched, {} unmatched, {} detour-splits)",
                n, regions.size(),
                regions.stream().filter(rr -> rr instanceof TrackRegion.Matched).count(),
                regions.stream().filter(rr -> rr instanceof TrackRegion.Unmatched).count(),
                lastDetours.size());
        return regions;
    }

    // ------------------------------------------------------------------------
    // Stage A — detour flags
    // ------------------------------------------------------------------------

    private boolean[] computeDirectFlags(List<Tracepoint> tps, int n,
                                         double minDetourM, double maxDetourRatio) {
        boolean[] direct = new boolean[n];
        Tracepoint prevNonFiltered = null;
        int prevNonFilteredIdx = -1;
        for (int i = 0; i < n; i++) {
            Tracepoint tp = tps.get(i);
            if (!tp.isMatched() || tp.isFiltered()) continue;
            Double dfp = tp.getDistanceFromPrevious();
            if (dfp != null && prevNonFiltered != null) {
                double straight = DIST.calcDist(
                        tp.getOriginalPoint().lat, tp.getOriginalPoint().lon,
                        prevNonFiltered.getOriginalPoint().lat, prevNonFiltered.getOriginalPoint().lon);
                if (straight > 0 && dfp > maxDetourRatio * straight && dfp > minDetourM) {
                    // U-turn suppression: when two consecutive Viterbi participants snap to
                    // the SAME undirected edge, a long matched-path length is a routing-graph
                    // directional artifact (the two states sit on different directed virtual
                    // edges of the same physical road; routing must go around to satisfy
                    // direction constraints) — not a real obstacle detour. The client's
                    // /route call between the same-edge snap points will produce a sensible
                    // partial-edge geometry, not the round-trip the matcher reports.
                    Integer edgeA = prevNonFiltered.getEdgeId();
                    Integer edgeB = tp.getEdgeId();
                    boolean sameEdge = edgeA != null && edgeA.equals(edgeB);
                    if (sameEdge) {
                        LOGGER.info("RegionSegmenter: detour obs[{}]→obs[{}] SUPPRESSED "
                                        + "(same edge {}, U-turn artifact): matched {}m, straight {}m, ratio {}",
                                prevNonFilteredIdx, i, edgeA,
                                String.format("%.1f", dfp), String.format("%.1f", straight),
                                String.format("%.2f", dfp / straight));
                    } else {
                        direct[i] = true;
                        double ratio = dfp / straight;
                        lastDetours.add(new DetourReport(prevNonFilteredIdx, i, dfp, straight, ratio));
                        LOGGER.info("RegionSegmenter: detour transition obs[{}]→obs[{}]: "
                                        + "matched {}m, straight {}m, ratio {}",
                                prevNonFilteredIdx, i,
                                String.format("%.1f", dfp), String.format("%.1f", straight),
                                String.format("%.2f", ratio));
                    }
                }
            }
            prevNonFiltered = tp;
            prevNonFilteredIdx = i;
        }
        return direct;
    }

    // ------------------------------------------------------------------------
    // Stage E — build raw regions from status array
    // ------------------------------------------------------------------------

    private static List<RawRegion> buildRawRegions(Status[] status, int n) {
        List<RawRegion> out = new ArrayList<>();

        // Skip leading BAD run.
        int i = 0;
        while (i < n && status[i] == Status.BAD) i++;
        if (i > 0) {
            // Leading BAD run becomes coords [0, firstGood]. The first good obs is shared
            // as the start of the next matched region (boundary convention).
            out.add(new RawRegion(false, 0, Math.min(i, n - 1)));
        }
        if (i >= n) {
            // All BAD.
            if (out.isEmpty()) out.add(new RawRegion(false, 0, n - 1));
            return out;
        }

        int matchedStart = i;
        while (i < n) {
            Status s = status[i];
            if (s == Status.GOOD || s == Status.DETOUR_BOUNDARY) {
                if (s == Status.DETOUR_BOUNDARY && i > matchedStart) {
                    // Detour boundary in mid-run: close matched at i-1, emit a 1-step
                    // coords [i-1, i], start a new matched at i.
                    out.add(new RawRegion(true, matchedStart, i - 1));
                    out.add(new RawRegion(false, i - 1, i));
                    matchedStart = i;
                }
                i++;
            } else {
                // BAD — close current matched at i-1.
                out.add(new RawRegion(true, matchedStart, i - 1));
                int badStart = i;
                while (i < n && status[i] == Status.BAD) i++;
                if (i >= n) {
                    // Trailing BAD — coords from (last good = badStart-1) to n-1.
                    out.add(new RawRegion(false, badStart - 1, n - 1));
                    return out;
                }
                // BAD run sandwiched — coords [last good, next good].
                out.add(new RawRegion(false, badStart - 1, i));
                matchedStart = i;
            }
        }
        if (matchedStart < n) {
            out.add(new RawRegion(true, matchedStart, n - 1));
        }
        return out;
    }

    // ------------------------------------------------------------------------
    // Stage F — drop single-obs matched + merge consecutive coords
    // ------------------------------------------------------------------------

    private static List<RawRegion> dropSingletonMatched(List<RawRegion> in) {
        List<RawRegion> out = new ArrayList<>(in.size());
        for (RawRegion r : in) {
            if (r.isMatched() && r.firstObs() == r.lastObs()) continue;
            out.add(r);
        }
        return out;
    }

    private static List<RawRegion> mergeConsecutiveCoords(List<RawRegion> in) {
        List<RawRegion> out = new ArrayList<>(in.size());
        for (RawRegion r : in) {
            if (!out.isEmpty()) {
                RawRegion prev = out.get(out.size() - 1);
                if (!prev.isMatched() && !r.isMatched()) {
                    out.set(out.size() - 1, new RawRegion(false, prev.firstObs(), r.lastObs()));
                    continue;
                }
            }
            out.add(r);
        }
        return out;
    }

    // ------------------------------------------------------------------------
    // Stage G — demote short matched regions
    // ------------------------------------------------------------------------

    private static List<RawRegion> demoteShortMatched(List<RawRegion> in,
                                                     List<Tracepoint> tps,
                                                     double minRoutedSegmentM) {
        List<RawRegion> out = new ArrayList<>(in.size());
        for (RawRegion r : in) {
            if (r.isMatched()) {
                double len = computeMatchedPathLength(tps, r.firstObs(), r.lastObs());
                if (len < minRoutedSegmentM) {
                    out.add(new RawRegion(false, r.firstObs(), r.lastObs()));
                    continue;
                }
            }
            out.add(r);
        }
        return out;
    }

    /** Sum {@link Tracepoint#getDistanceFromPrevious()} across {@code (firstObs, lastObs]}
     *  for non-filtered matched observations. The first observation in a region has no
     *  incoming transition within the region, so we sum from {@code firstObs + 1}. */
    private static double computeMatchedPathLength(List<Tracepoint> tps, int firstObs, int lastObs) {
        double total = 0;
        for (int i = firstObs + 1; i <= lastObs; i++) {
            Tracepoint tp = tps.get(i);
            if (!tp.isMatched() || tp.isFiltered()) continue;
            Double dfp = tp.getDistanceFromPrevious();
            if (dfp != null) total += dfp;
        }
        return total;
    }

    // ------------------------------------------------------------------------
    // obs → EdgeMatch mapping (preserved from prior implementation; needed by
    // buildMatched downstream).
    // ------------------------------------------------------------------------

    /**
     * Assign each observation to its EdgeMatch position via two passes:
     *
     * <ol>
     *   <li><b>Direction-aware (primary)</b>: walk EdgeMatches in order and read each
     *       em's {@link EdgeMatch#getStates()}. For each {@link State}, look up its
     *       Observation by reference identity in the original observations list and
     *       assign that index to the current em. This uses the matcher's own
     *       direction-aware Viterbi attribution — so for a U-turn where the same
     *       undirected edge appears twice (forward + reverse), the observations end
     *       up on the correct em.</li>
     *   <li><b>Undirected fallback</b>: filtered observations don't participate in
     *       Viterbi and aren't in any State's list. For these, fall back to the
     *       previous undirected {@link Tracepoint#getEdgeId()} forward/backward
     *       search.</li>
     * </ol>
     *
     * <p>Reference-identity requires that {@code observations} is the same list (same
     * Observation object references) that was passed into {@code MapMatching.match()}.
     * MapMatching's internal filter step returns a subList of the same references, and
     * State holds those references through to {@code getEntry()}, so identity holds.
     */
    private static int[] mapObsToEdgeMatch(List<Tracepoint> tracepoints,
                                           List<Observation> observations,
                                           List<EdgeMatch> edgeMatches) {
        int n = tracepoints.size();
        int[] obsToEdgeMatch = new int[n];
        for (int i = 0; i < n; i++) obsToEdgeMatch[i] = -1;

        // Pass 1: direction-aware attribution via State.getEntry().
        // Build identity-based Observation -> obs index map. Identity (not equality)
        // because two Observations can have identical coordinates but are distinct
        // entries in the input sequence (e.g. out-n-back's return through the same
        // location).
        Map<Observation, Integer> obsIdxByRef = new IdentityHashMap<>();
        for (int i = 0; i < observations.size(); i++) {
            obsIdxByRef.put(observations.get(i), i);
        }
        int pass1Assigned = 0;
        for (int emIdx = 0; emIdx < edgeMatches.size(); emIdx++) {
            for (State s : edgeMatches.get(emIdx).getStates()) {
                Integer obsIdx = obsIdxByRef.get(s.getEntry());
                if (obsIdx != null && obsIdx < n) {
                    obsToEdgeMatch[obsIdx] = emIdx;
                    pass1Assigned++;
                }
            }
        }
        if (LOGGER.isDebugEnabled()) {
            StringBuilder sb = new StringBuilder("mapObsToEdgeMatch pass1 (direction-aware via State.getEntry()): assigned ").append(pass1Assigned).append("/").append(n).append(" obs. obsToEm=[");
            for (int i = 0; i < n; i++) {
                if (i > 0) sb.append(",");
                sb.append(obsToEdgeMatch[i]);
            }
            sb.append("]");
            LOGGER.debug(sb.toString());
        }

        // Pass 2: undirected fallback for filtered observations (not in any State's
        // list). Walks forward from the previous assignment's em index, with a
        // backward-scan fallback for out-of-order cases.
        int searchFrom = 0;
        for (int i = 0; i < n; i++) {
            if (obsToEdgeMatch[i] >= 0) {
                searchFrom = obsToEdgeMatch[i];
                continue;
            }
            Tracepoint tp = tracepoints.get(i);
            if (!tp.isMatched() || tp.getEdgeId() == null) continue;
            int targetEdge = tp.getEdgeId();
            int found = -1;
            for (int j = searchFrom; j < edgeMatches.size(); j++) {
                if (edgeMatches.get(j).getEdgeState().getEdge() == targetEdge) { found = j; break; }
            }
            if (found < 0) {
                for (int j = searchFrom - 1; j >= 0; j--) {
                    if (edgeMatches.get(j).getEdgeState().getEdge() == targetEdge) { found = j; break; }
                }
            }
            if (found >= 0) {
                obsToEdgeMatch[i] = found;
                searchFrom = found;
            }
        }
        return obsToEdgeMatch;
    }

    // ------------------------------------------------------------------------
    // buildMatched — preserved from prior implementation. Constructs the rich
    // TrackRegion.Matched with edge slice, obs→edge mapping in slice coords,
    // snap points, and matched length. Required by RoutedRegionOptimizer.
    // ------------------------------------------------------------------------

    private TrackRegion.Matched buildMatched(List<Tracepoint> tracepoints,
                                             List<EdgeMatch> edgeMatches,
                                             int[] obsToEdgeMatch,
                                             int startObs, int endObs) {
        // EdgeMatch index range covering all matched obs in this region. Use min/max
        // because obsToEdgeMatch can be non-monotonic when MapMatching's path includes
        // loops or back-and-forth segments.
        int firstEm = Integer.MAX_VALUE, lastEm = -1;
        for (int i = startObs; i <= endObs; i++) {
            if (obsToEdgeMatch[i] >= 0) {
                if (obsToEdgeMatch[i] < firstEm) firstEm = obsToEdgeMatch[i];
                if (obsToEdgeMatch[i] > lastEm) lastEm = obsToEdgeMatch[i];
            }
        }
        if (lastEm < 0) {
            Tracepoint tp = tracepoints.get(startObs);
            GHPoint snap = tp.getSnappedPoint() != null ? tp.getSnappedPoint() : tp.getOriginalPoint();
            return new TrackRegion.Matched(startObs, endObs,
                    List.of(startObs), List.of(snap), new int[]{0},
                    List.of(), snap, snap, 0.0);
        }

        // Raw slice; dedup consecutive same-edge entries by DIRECTED edge key
        // (getEdgeKey(), which encodes both edge id and traversal direction).
        //
        // The matcher (prepareEdgeMatches in MapMatching) already merges consecutive
        // same-directed-edge entries, so under normal conditions this dedup is a no-op.
        // It's retained as a defensive safety net.
        //
        // Critically: we must NOT dedup by undirected getEdge(), because that collapses
        // U-turn patterns where the user traversed the same physical road forward and
        // then back. The matcher correctly emits these as two consecutive entries with
        // the same undirected edge id but different edge keys (forward/reverse). The
        // direction-change carries real information the optimizer needs — collapsing it
        // here destroys the U-turn signal and the optimizer cannot recover it.
        // See out-n-back fixture: em[4]=3003674 (forward) + em[5]=3003674 (reverse) at
        // the turn-around point.
        List<EdgeMatch> rawSlice = edgeMatches.subList(firstEm, lastEm + 1);
        List<EdgeMatch> slice = new ArrayList<>(rawSlice.size());
        int[] rawToDedupIdx = new int[rawSlice.size()];
        int dedupIdx = -1;
        int prevEdgeKey = Integer.MIN_VALUE;
        for (int i = 0; i < rawSlice.size(); i++) {
            int eKey = rawSlice.get(i).getEdgeState().getEdgeKey();
            if (eKey != prevEdgeKey) {
                slice.add(rawSlice.get(i));
                dedupIdx++;
                prevEdgeKey = eKey;
            }
            rawToDedupIdx[i] = dedupIdx;
        }

        List<Integer> matchedObsIndices = new ArrayList<>();
        List<GHPoint> obsSnapPoints = new ArrayList<>();
        List<Integer> obsEdgeIdxInSliceL = new ArrayList<>();
        for (int i = startObs; i <= endObs; i++) {
            int emIdx = obsToEdgeMatch[i];
            if (emIdx < 0) continue;
            Tracepoint tp = tracepoints.get(i);
            GHPoint snap = tp.getSnappedPoint();
            if (snap == null) continue;
            matchedObsIndices.add(i);
            obsSnapPoints.add(snap);
            obsEdgeIdxInSliceL.add(rawToDedupIdx[emIdx - firstEm]);
        }
        int[] obsEdgeIdxInSlice = new int[obsEdgeIdxInSliceL.size()];
        for (int k = 0; k < obsEdgeIdxInSlice.length; k++) {
            obsEdgeIdxInSlice[k] = obsEdgeIdxInSliceL.get(k);
        }

        GHPoint startSnap = obsSnapPoints.isEmpty()
                ? tracepoints.get(startObs).getOriginalPoint() : obsSnapPoints.get(0);
        GHPoint endSnap = obsSnapPoints.isEmpty()
                ? tracepoints.get(endObs).getOriginalPoint() : obsSnapPoints.get(obsSnapPoints.size() - 1);

        double lengthM = 0;
        for (EdgeMatch em : slice) lengthM += em.getEdgeState().getDistance();

        return new TrackRegion.Matched(startObs, endObs,
                matchedObsIndices, obsSnapPoints, obsEdgeIdxInSlice,
                slice, startSnap, endSnap, lengthM);
    }
}
