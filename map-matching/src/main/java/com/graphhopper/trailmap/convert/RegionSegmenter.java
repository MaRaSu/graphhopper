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
import java.util.Collections;
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
    public static final double DEFAULT_DRIFT_TRIM_TIGHT_FLOOR_M = 5.0;

    /** Active drift-trim tight-snap floor. Defaults to {@link #DEFAULT_DRIFT_TRIM_TIGHT_FLOOR_M};
     *  the converter overrides it with a σ-derived value when auto-sigma estimated the track's
     *  noise (see {@link #setDriftTrimFloorM}). */
    private double driftTrimFloorM = DEFAULT_DRIFT_TRIM_TIGHT_FLOOR_M;

    /** Override the drift-trim tight-snap floor [m]. Used by the converter to scale the floor
     *  with the auto-estimated sigma (an accurate track wants a tighter floor than a noisy one,
     *  because "tight on the road within GPS noise" is a statement about sigma). */
    public void setDriftTrimFloorM(double driftTrimFloorM) {
        this.driftTrimFloorM = driftTrimFloorM;
    }

    // ------------------------------------------------------------------------
    // Optional, independently-toggleable segmenter refinements (2026-06-01).
    // Each is gated by a flag and isolated in its own method, so it can be turned
    // off (set the flag false, or comment out the single call site in segment())
    // if it misbehaves. Both target OVER-aggression on hard-simplified routes
    // (see docs/gh_map_matcher_implementation.md §7 b2).
    // ------------------------------------------------------------------------

    /** Corner-cut tolerance: promote a short BAD run (a transient snap spike from a
     *  simplifier chording a bend) back to routed. Toggle off by setting false. */
    public boolean cornerCutToleranceEnabled = true;
    /** Max length (observations) of a BAD run eligible for corner-cut promotion. */
    public static final int CORNER_CUT_MAX_RUN = 3;
    /** Absolute snap cap [m]: a spike beyond this is too far to be a mere corner cut and
     *  stays off-grid regardless of transience. */
    public static final double CORNER_CUT_MAX_SNAP_M = 25.0;

    /** Phantom-detour guard: suppress a Stage-A detour flag that is actually a directed
     *  routing artifact at a corner — recognised by both endpoints being tight-snapped AND a
     *  SHORT real on-network {@code /route} existing between them (so the long matched-path is
     *  not a detour the user took). A real obstacle detour has a long {@code /route} regardless
     *  of how small its straight-line is, so it is preserved. Needs {@link #directRouteFn}
     *  injected; inert (never suppresses) without it. Toggle off by setting false. */
    public boolean phantomDetourGuardEnabled = true;
    /** Both endpoints must snap within this [m] for the phantom guard to even consider a detour
     *  an on-network routing artifact (so the snapped points represent real positions and the
     *  {@code /route} between them is meaningful). */
    public static final double PHANTOM_DETOUR_TIGHT_SNAP_M = 5.0;

    /** Supplies the on-network shortest-route distance [m] between two points (the routing
     *  engine), or {@code null} if no route. Injected by the converter so the segmenter stays
     *  decoupled from GraphHopper. Used ONLY by the phantom-detour guard. */
    public interface DirectRouteDistance {
        Double distanceMeters(GHPoint a, GHPoint b);
    }

    private DirectRouteDistance directRouteFn;

    /** Inject the route-distance function used by the phantom-detour guard. */
    public void setDirectRouteFn(DirectRouteDistance fn) {
        this.directRouteFn = fn;
    }

    // ------------------------------------------------------------------------
    // Segmentation v2 — joint emission+transition coords decision (2026-06-01).
    //
    // A fully PARALLEL, flag-gated alternative to the Stage A–D classification
    // above (see docs/gh_map_matcher_implementation.md §8). When disabled
    // (default), segment() runs the original path verbatim — v2 is purely
    // additive and reverts cleanly by leaving the flag off. When enabled,
    // segment() delegates to segmentV2(), which produces the same Status array
    // by a different rule and then reuses the identical Stage E–G machinery
    // (buildRawRegions / dropSingletonMatched / mergeConsecutiveCoords /
    // demoteShortMatched / buildMatched), so only the classification changes.
    //
    // Core idea: the matcher's own per-observation signals are already on the
    // Tracepoint — the chosen candidate's snap distance is the EMISSION signal,
    // and distanceFromPrevious (matched-path length) vs the straight-line is the
    // TRANSITION signal. v2 triggers coords on BOTH jointly (not snap alone):
    //   - off-road excursion  → emission high AND a detour transition → coords
    //   - noisy on-road / corner cut / on-network reroute → no off-grid span
    //     under the detour → stays routed (no /route call needed; the emission
    //     check replaces the phantom-detour guard by construction)
    // and then EXPANDS the coords region outward over neighbours whose snap
    // stays above the local baseline (souped-up B.5), to swallow parallel-offset
    // approach/exit tails ("better a bit too much coords than too little").
    // ------------------------------------------------------------------------

    /** Master toggle for the v2 joint-signal segmentation. Off → original behaviour. */
    public boolean segmentationV2Enabled = false;

    public void setSegmentationV2Enabled(boolean enabled) { this.segmentationV2Enabled = enabled; }

    /** When true (default), the optimizer's waypoint candidates are restricted to KEPT (Viterbi)
     *  observations — those guaranteed to lie on the matched path — dropping interior FILTERED obs
     *  (2σ-dropped points whose independent nearest-edge snap may be off the path). Region first/last
     *  candidates are always kept as boundary anchors. Set false to restore the old behaviour of
     *  offering every observation (kept + filtered) as a candidate. Applies to both v1 and v2
     *  segmentation (buildMatched is shared). */
    public boolean optimizerKeptObsOnly = true;

    public void setOptimizerKeptObsOnly(boolean enabled) { this.optimizerKeptObsOnly = enabled; }

    /** Half-width [observations] of the moving window used to estimate the local emission
     *  baseline (the per-obs GPS-noise floor). */
    static final int V2_EMISSION_WINDOW = 15;
    /** Percentile of the windowed snap distances taken as the local baseline. A LOW percentile
     *  reflects the on-road noise floor even when part of the window is off-grid (the off-grid
     *  snaps are the elevated ones we compare against, so they must not set the baseline). */
    static final double V2_EMISSION_BASELINE_PCTL = 0.3;
    /** Floor [m] for the local baseline (a perfectly clean track still has a non-zero noise). */
    static final double V2_EMISSION_BASELINE_MIN_M = 2.0;
    /** Cap [m] for the local baseline (don't let a noisy stretch raise its own trigger bar without
     *  limit). */
    static final double V2_EMISSION_BASELINE_MAX_M = 15.0;
    /** Emission is "elevated" (off-grid candidate) when snap exceeds this multiple of the local
     *  baseline AND the absolute floor below. */
    static final double V2_EMISSION_TRIGGER_MULT = 2.5;
    /** Absolute snap floor [m] below which emission never counts as elevated (so on a clean track
     *  a tiny baseline can't flag ordinary GPS noise). */
    static final double V2_EMISSION_TRIGGER_FLOOR_M = 8.0;
    /** Expansion uses a LOWER bar than the trigger (aggressive outward growth): keep absorbing
     *  neighbours whose snap exceeds this multiple of the local baseline and the floor below. */
    static final double V2_EXPAND_MULT = 1.4;
    /** Absolute snap floor [m] for expansion. */
    static final double V2_EXPAND_FLOOR_M = 5.0;
    /** Minimum length [observations] of a beyond-off-network-threshold run for the absolute
     *  off-network seed to fire on its own (without a detour transition). A transient corner
     *  spike (1–2 points just over the threshold) must NOT self-seed — that is a corner cut, not
     *  an off-grid excursion (idea 4: corners excluded by construction). A genuine off-grid
     *  stretch is many consecutive points over the threshold. Single off-network points adjacent
     *  to a real seed are still absorbed by the outward expansion. */
    static final int V2_OFFGRID_SEED_MIN_RUN = 3;
    /** Aggressive-expansion at region granularity: a routed region sandwiched between coords
     *  regions on BOTH sides and shorter than this matched-path length [m] is absorbed into
     *  coords. These are short on-grid connectors the user briefly rejoined between off-grid
     *  excursions — leaving them as tiny routed islands fragments the route; folding them in
     *  matches "better a bit too much coords than too little". Fires ONLY when flanked by coords
     *  on both sides, so it never touches mostly-on-road tracks (which have no such islands). */
    static final double V2_BRIDGE_ROUTED_MAX_M = 150.0;
    /** Corner-cut tolerance (idea 4): a short off-grid SPIKE that is really a simplifier chording a
     *  bend — the rider stayed on a continuous way, only the snap spiked transiently. Such a run
     *  bracketed by routed on both sides, on a contiguous matched chain, with bounded snaps and no
     *  detour seed, is promoted back to routed.
     *
     *  <p>This is the max length [observations] of the OFF-GRID CORE — the longest run of
     *  consecutive snaps over the off-network threshold WITHIN the coords run — NOT the full run
     *  length. v2's aggressive expansion widens a corner over its on-grid shoulders, so the full
     *  run can be much longer than the transient spike; judging by the core distinguishes a real
     *  corner (short off-grid core) from a sustained off-grid drift (long core). */
    static final int V2_CORNER_CUT_MAX_RUN = 5;
    /** Corner-cut tolerance: a spike beyond this snap [m] is too far to be a mere corner cut and
     *  stays off-grid. */
    static final double V2_CORNER_CUT_MAX_SNAP_M = 25.0;

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
        if (segmentationV2Enabled) {
            return segmentV2(matchResult, observations, snapThresholdM,
                    minDetourM, maxDetourRatio, minRoutedSegmentM);
        }
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
        boolean[] direct = computeDirectFlags(tps, n, minDetourM, maxDetourRatio, snapThresholdM);

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

        // Stage B.4 — corner-cut tolerance (optional; toggle via cornerCutToleranceEnabled).
        // A hard simplifier chords 90° bends, putting a few observations transiently off the
        // way while the user stayed on a continuous way. A global snap threshold flags these
        // as off-grid (especially on long routes where auto-sigma estimates a tiny σ). This
        // promotes such short, continuous, non-detour BAD spikes back to routed. Runs BEFORE
        // drift-trim so promoted corners are not treated as boundaries.
        if (cornerCutToleranceEnabled) {
            applyCornerCutTolerance(status, tps, direct, obsToEdgeMatch, n);
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
                if (snap == null || snap <= driftTrimFloorM) continue;
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
    // Segmentation v2 — joint emission+transition classification (flag-gated)
    // ------------------------------------------------------------------------

    /**
     * v2 coords classification. Produces a {@link Status} array from the matcher's own
     * per-observation signals (snap distance = emission, matched-path vs straight-line =
     * transition), then reuses the identical Stage E–G machinery as {@link #segment}.
     *
     * <p>Algorithm:
     * <ol>
     *   <li><b>Local emission baseline</b> — for each obs, a windowed low-percentile of snap
     *       distances ({@link #computeLocalBaseline}); the per-obs GPS-noise floor (idea 1).</li>
     *   <li><b>emHigh / expandHigh</b> — emission elevated vs the local baseline (trigger bar
     *       and a lower expansion bar), or beyond the absolute off-network threshold.</li>
     *   <li><b>Seeds</b> — (a) any obs beyond the absolute off-network threshold; (b) a detour
     *       transition (matched-path ≫ straight) whose span actually went off-grid (emHigh in
     *       range). A phantom corner / on-network reroute has tight snaps → no emHigh → not
     *       seeded → stays routed (the emission check replaces the phantom-detour guard).</li>
     *   <li><b>Aggressive outward expansion</b> — grow each seed over neighbours that are
     *       expandHigh, until emission returns to baseline (idea 2; swallows parallel-offset
     *       tails — sorel's residual case).</li>
     *   <li>De-singleton, then Stage E–G (shared with v1).</li>
     * </ol>
     */
    private List<TrackRegion> segmentV2(MatchResult matchResult,
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
        int[] obsToEdgeMatch = mapObsToEdgeMatch(tps, observations, edgeMatches);

        // Per-obs snap distance = the matcher's chosen-candidate emission signal.
        // NaN ONLY where the obs is genuinely unmatched (no snap). Filtered observations
        // (dropped by the matcher's 2σ pre-filter for being too CLOSE to a neighbour, not for
        // being off-road) DO carry a valid, separately-computed snap distance — they must use
        // it, exactly as v1's Stage B does. Excluding them here wrongly marks dense on-road
        // stretches as off-grid and trips Seed (a) → spurious coords.
        double[] snap = new double[n];
        for (int i = 0; i < n; i++) {
            Tracepoint tp = tps.get(i);
            Double d = tp.isMatched() ? tp.getDistance() : null;
            snap[i] = (d == null) ? Double.NaN : d;
        }

        // Local emission baseline (windowed low-percentile of snaps).
        double[] baseline = computeLocalBaseline(snap, n);

        // Emission elevation flags (trigger bar) and expansion flags (lower bar).
        boolean[] emHigh = new boolean[n];
        boolean[] expandHigh = new boolean[n];
        for (int i = 0; i < n; i++) {
            double s = snap[i];
            if (Double.isNaN(s)) { emHigh[i] = true; expandHigh[i] = true; continue; }
            emHigh[i] = s > snapThresholdM
                    || (s > V2_EMISSION_TRIGGER_FLOOR_M && s > V2_EMISSION_TRIGGER_MULT * baseline[i]);
            expandHigh[i] = s > V2_EXPAND_FLOOR_M && s > V2_EXPAND_MULT * baseline[i];
        }

        boolean[] coords = new boolean[n];
        // Marks obs seeded by a detour (Seed b) so the corner-cut promotion below never undoes a
        // genuine off-road detour (only Seed-a snap spikes are eligible for promotion).
        boolean[] detourSeeded = new boolean[n];

        // Seed (a): absolute off-network — a SUSTAINED run (≥ V2_OFFGRID_SEED_MIN_RUN) of snaps
        // beyond the threshold is off-grid by definition. Requiring a run (not a single point)
        // means a transient corner spike just over the threshold does NOT self-seed (idea 4);
        // isolated off-network points adjacent to a real seed are still swept up by expansion.
        boolean[] offGrid = new boolean[n];
        for (int i = 0; i < n; i++) {
            offGrid[i] = Double.isNaN(snap[i]) || snap[i] > snapThresholdM;
        }
        int run = 0;
        for (int i = 0; i <= n; i++) {
            if (i < n && offGrid[i]) {
                run++;
            } else {
                if (run >= V2_OFFGRID_SEED_MIN_RUN) {
                    for (int j = i - run; j < i; j++) coords[j] = true;
                }
                run = 0;
            }
        }

        // Seed (b): joint detour trigger. A detour transition (matched-path ≫ straight) is a coords
        // seed when EITHER:
        //   (i) its span went off-grid (emHigh somewhere in [prevIdx..i]) — an off-road excursion; OR
        //  (ii) it is on-grid at both ends but NO SHORT on-network /route exists between the snaps —
        //       a genuine no-connector detour (e.g. crossing a street with no through-way: only the
        //       long way around exists). The /route check (v1 phantom-detour guard) distinguishes this
        //       from a directed-routing artifact / on-network reroute, where a SHORT /route does exist
        //       and the long matched-path is spurious → kept routed.
        // Without an injected route fn, fall back to the emHigh gate alone (conservative: keep routed).
        Tracepoint prevNonFiltered = null;
        int prevIdx = -1;
        for (int i = 0; i < n; i++) {
            Tracepoint tp = tps.get(i);
            if (!tp.isMatched() || tp.isFiltered()) continue;
            Double dfp = tp.getDistanceFromPrevious();
            if (dfp != null && prevNonFiltered != null) {
                double straight = DIST.calcDist(
                        tp.getOriginalPoint().lat, tp.getOriginalPoint().lon,
                        prevNonFiltered.getOriginalPoint().lat, prevNonFiltered.getOriginalPoint().lon);
                if (straight > 0 && dfp > maxDetourRatio * straight && dfp > minDetourM) {
                    boolean emInRange = false;
                    for (int j = prevIdx; j <= i; j++) {
                        if (emHigh[j]) { emInRange = true; break; }
                    }
                    boolean realDetour = emInRange;
                    String reason = emInRange ? "off-grid span" : null;
                    if (!emInRange && directRouteFn != null
                            && prevNonFiltered.getSnappedPoint() != null && tp.getSnappedPoint() != null) {
                        Double routeM = directRouteFn.distanceMeters(
                                prevNonFiltered.getSnappedPoint(), tp.getSnappedPoint());
                        if (routeM == null || routeM > maxDetourRatio * straight) {
                            realDetour = true;
                            reason = "no short on-network connector (/route "
                                    + (routeM == null ? "none" : fmt(routeM) + "m")
                                    + " > " + fmt(maxDetourRatio * straight) + "m)";
                        }
                    }
                    if (realDetour) {
                        for (int j = prevIdx; j <= i; j++) { coords[j] = true; detourSeeded[j] = true; }
                        lastDetours.add(new DetourReport(prevIdx, i, dfp, straight, dfp / straight));
                        LOGGER.info("RegionSegmenter v2: detour SEED obs[{}]→obs[{}] ({}): "
                                        + "matched {}m, straight {}m, ratio {}",
                                prevIdx, i, reason, fmt(dfp), fmt(straight), fmt(dfp / straight));
                    } else {
                        LOGGER.info("RegionSegmenter v2: detour obs[{}]→obs[{}] IGNORED (on-grid span, "
                                        + "short /route exists, ratio {}): on-network reroute/artifact, kept routed",
                                prevIdx, i, fmt(dfp / straight));
                    }
                }
            }
            prevNonFiltered = tp;
            prevIdx = i;
        }

        // Aggressive outward expansion: grow coords over expandHigh neighbours until emission
        // falls back to baseline on both sides.
        boolean changed = true;
        while (changed) {
            changed = false;
            for (int i = 0; i < n; i++) {
                if (!coords[i]) continue;
                if (i > 0 && !coords[i - 1] && expandHigh[i - 1]) { coords[i - 1] = true; changed = true; }
                if (i < n - 1 && !coords[i + 1] && expandHigh[i + 1]) { coords[i + 1] = true; changed = true; }
            }
        }

        // Corner-cut promotion (idea 4): a short off-grid run bracketed by routed on both sides,
        // on a CONTIGUOUS matched chain (the rider stayed on a continuous way), with bounded snaps
        // and NO detour seed, is a simplifier corner cut — promote it back to routed. This undoes
        // the Seed (a) false-positive on long routes where the global threshold shrinks (sbc_keha_g
        // obs 2535–2538: a 4-obs snap spike to 11–18 m on contiguous edges 4085403→4085405→4085408).
        int ci = 0;
        while (ci < n) {
            if (!coords[ci]) { ci++; continue; }
            int rs = ci;
            while (ci < n && coords[ci]) ci++;
            int re = ci - 1;
            boolean bracketed = rs > 0 && re < n - 1 && !coords[rs - 1] && !coords[re + 1];
            if (!bracketed) continue;
            // Off-grid CORE length: longest run of consecutive over-threshold (or unmatched) snaps
            // inside the run. v2 expansion widens a corner over its on-grid shoulders, so the full
            // run can far exceed the transient spike — judge the corner by the core, not the width.
            int maxOffGridRun = 0, curOffGridRun = 0;
            for (int j = rs; j <= re; j++) {
                boolean og = Double.isNaN(snap[j]) || snap[j] > snapThresholdM;
                curOffGridRun = og ? curOffGridRun + 1 : 0;
                if (curOffGridRun > maxOffGridRun) maxOffGridRun = curOffGridRun;
            }
            if (maxOffGridRun > V2_CORNER_CUT_MAX_RUN) continue;
            boolean detourInside = false;
            for (int j = rs; j <= re; j++) {
                if (detourSeeded[j]) { detourInside = true; break; }
            }
            if (detourInside) continue;
            boolean belowCap = true;
            double maxSnap = 0;
            for (int j = rs; j <= re; j++) {
                double s = snap[j];
                if (Double.isNaN(s) || s > V2_CORNER_CUT_MAX_SNAP_M) { belowCap = false; break; }
                maxSnap = Math.max(maxSnap, s);
            }
            if (!belowCap) continue;
            if (!matchedEdgesContiguous(obsToEdgeMatch, rs - 1, re + 1)) continue;
            for (int j = rs; j <= re; j++) coords[j] = false;
            LOGGER.info("RegionSegmenter v2: corner-cut promoted obs[{}..{}] (run {}, off-grid core {}, "
                            + "maxSnap {}m) → routed",
                    rs, re, re - rs + 1, maxOffGridRun, fmt(maxSnap));
        }

        // Status array + de-singleton (mirrors Stage C).
        Status[] status = new Status[n];
        for (int i = 0; i < n; i++) status[i] = coords[i] ? Status.BAD : Status.GOOD;
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

        // Stage E–G (shared with v1).
        List<RawRegion> raw = buildRawRegions(status, n);
        raw = dropSingletonMatched(raw);
        raw = mergeConsecutiveCoords(raw);
        raw = demoteShortMatched(raw, tps, minRoutedSegmentM);
        raw = mergeConsecutiveCoords(raw);
        // v2 aggressive-expansion: absorb short routed islands stranded between coords regions.
        raw = bridgeShortRoutedBetweenCoords(raw, tps, V2_BRIDGE_ROUTED_MAX_M);
        raw = mergeConsecutiveCoords(raw);

        List<TrackRegion> regions = new ArrayList<>();
        for (RawRegion r : raw) {
            if (r.isMatched()) {
                regions.add(buildMatched(tps, edgeMatches, obsToEdgeMatch, r.firstObs(), r.lastObs()));
            } else {
                regions.add(new TrackRegion.Unmatched(r.firstObs(), r.lastObs()));
            }
        }

        LOGGER.info("RegionSegmenter v2: {} observations → {} regions ({} matched, {} unmatched, {} detour-seeds)",
                n, regions.size(),
                regions.stream().filter(rr -> rr instanceof TrackRegion.Matched).count(),
                regions.stream().filter(rr -> rr instanceof TrackRegion.Unmatched).count(),
                lastDetours.size());
        return regions;
    }

    /** Per-obs local emission baseline: a windowed {@link #V2_EMISSION_BASELINE_PCTL} percentile
     *  of snap distances, clamped to [{@link #V2_EMISSION_BASELINE_MIN_M},
     *  {@link #V2_EMISSION_BASELINE_MAX_M}]. NaN (unmatched) snaps are skipped. */
    private static double[] computeLocalBaseline(double[] snap, int n) {
        double[] baseline = new double[n];
        List<Double> win = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            win.clear();
            int lo = Math.max(0, i - V2_EMISSION_WINDOW);
            int hi = Math.min(n - 1, i + V2_EMISSION_WINDOW);
            for (int j = lo; j <= hi; j++) {
                if (!Double.isNaN(snap[j])) win.add(snap[j]);
            }
            double b;
            if (win.isEmpty()) {
                b = V2_EMISSION_BASELINE_MIN_M;
            } else {
                Collections.sort(win);
                b = pctl(win, V2_EMISSION_BASELINE_PCTL);
            }
            baseline[i] = Math.max(V2_EMISSION_BASELINE_MIN_M, Math.min(V2_EMISSION_BASELINE_MAX_M, b));
        }
        return baseline;
    }

    /** Linear-interpolated percentile of a pre-sorted list. p clamped to [0,1]. */
    private static double pctl(List<Double> sorted, double p) {
        if (sorted.isEmpty()) return 0.0;
        if (sorted.size() == 1) return sorted.get(0);
        double pp = Math.max(0.0, Math.min(1.0, p));
        double rank = pp * (sorted.size() - 1);
        int lo = (int) Math.floor(rank);
        int hi = (int) Math.ceil(rank);
        return sorted.get(lo) + (rank - lo) * (sorted.get(hi) - sorted.get(lo));
    }

    private static String fmt(double v) { return String.format("%.1f", v); }

    /** v2 aggressive-expansion: demote to coords any matched region that is bracketed by coords
     *  regions on BOTH sides and whose matched-path length is below {@code maxM}. Such regions are
     *  short on-grid connectors stranded between off-grid excursions; folding them in avoids tiny
     *  routed islands that fragment the route. Caller merges consecutive coords afterwards. */
    private static List<RawRegion> bridgeShortRoutedBetweenCoords(List<RawRegion> in,
                                                                  List<Tracepoint> tps,
                                                                  double maxM) {
        List<RawRegion> out = new ArrayList<>(in.size());
        for (int i = 0; i < in.size(); i++) {
            RawRegion r = in.get(i);
            if (r.isMatched() && i > 0 && i < in.size() - 1
                    && !in.get(i - 1).isMatched() && !in.get(i + 1).isMatched()) {
                double len = computeMatchedPathLength(tps, r.firstObs(), r.lastObs());
                if (len < maxM) {
                    LOGGER.info("RegionSegmenter v2: bridging short routed island obs[{}..{}] "
                                    + "({}m < {}m, coords on both sides) → coords",
                            r.firstObs(), r.lastObs(), fmt(len), fmt(maxM));
                    out.add(new RawRegion(false, r.firstObs(), r.lastObs()));
                    continue;
                }
            }
            out.add(r);
        }
        return out;
    }

    // ------------------------------------------------------------------------
    // Stage A — detour flags
    // ------------------------------------------------------------------------

    private boolean[] computeDirectFlags(List<Tracepoint> tps, int n,
                                         double minDetourM, double maxDetourRatio,
                                         double snapThresholdM) {
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
                    // OFF-GRID-MIDDLE GATE: if the user went off the network BETWEEN the two
                    // endpoints — any skipped/filtered observation between them snaps beyond the
                    // off-network threshold — the long matched-path is a REAL off-road excursion
                    // (e.g. a snow-shoe loop), NOT a routing artifact. In that case NEITHER the
                    // same-edge U-turn suppression NOR the phantom guard may fire; the detour
                    // stands → coordinates. A genuine on-edge U-turn keeps its in-between
                    // observations on the edge (small snaps), so this gate leaves it suppressed.
                    boolean offGridMiddle = false;
                    for (int j = prevNonFilteredIdx + 1; j < i; j++) {
                        Double sj = tps.get(j).getDistance();
                        if (sj != null && sj > snapThresholdM) { offGridMiddle = true; break; }
                    }

                    // U-turn suppression: when two consecutive Viterbi participants snap to
                    // the SAME undirected edge, a long matched-path length is a routing-graph
                    // directional artifact (the two states sit on different directed virtual
                    // edges of the same physical road; routing must go around to satisfy
                    // direction constraints) — not a real obstacle detour. The client's
                    // /route call between the same-edge snap points will produce a sensible
                    // partial-edge geometry, not the round-trip the matcher reports.
                    Integer edgeA = prevNonFiltered.getEdgeId();
                    Integer edgeB = tp.getEdgeId();
                    boolean sameEdge = !offGridMiddle && edgeA != null && edgeA.equals(edgeB);
                    // Phantom-detour guard (optional; phantomDetourGuardEnabled): a long
                    // matched-path is a directed-routing artifact at a corner — NOT a real
                    // off-grid hop — when BOTH endpoints snap tight (clearly on-network) AND a
                    // SHORT real on-network /route exists between the two snaps. The straight-
                    // line is NOT the signal (a real railway detour can have a tiny straight);
                    // the signal is whether the actual road network offers a short path. A
                    // genuine obstacle detour has a long /route (≈ the matcher's path) and is
                    // preserved as coords regardless of how small its straight-line is.
                    boolean phantom = false;
                    Double phantomRouteM = null;
                    if (!offGridMiddle && !sameEdge && phantomDetourGuardEnabled && directRouteFn != null
                            && prevNonFiltered.getDistance() != null
                            && prevNonFiltered.getDistance() <= PHANTOM_DETOUR_TIGHT_SNAP_M
                            && tp.getDistance() != null
                            && tp.getDistance() <= PHANTOM_DETOUR_TIGHT_SNAP_M
                            && prevNonFiltered.getSnappedPoint() != null
                            && tp.getSnappedPoint() != null) {
                        phantomRouteM = directRouteFn.distanceMeters(
                                prevNonFiltered.getSnappedPoint(), tp.getSnappedPoint());
                        // Short real /route exists (within the same detour-ratio gate of the
                        // straight-line) ⇒ the long matched-path is an artifact ⇒ suppress.
                        if (phantomRouteM != null && phantomRouteM <= maxDetourRatio * straight) {
                            phantom = true;
                        }
                    }
                    if (sameEdge) {
                        LOGGER.info("RegionSegmenter: detour obs[{}]→obs[{}] SUPPRESSED "
                                        + "(same edge {}, U-turn artifact): matched {}m, straight {}m, ratio {}",
                                prevNonFilteredIdx, i, edgeA,
                                String.format("%.1f", dfp), String.format("%.1f", straight),
                                String.format("%.2f", dfp / straight));
                    } else if (phantom) {
                        LOGGER.info("RegionSegmenter: detour obs[{}]→obs[{}] SUPPRESSED "
                                        + "(phantom corner artifact: both snaps tight {}m/{}m, "
                                        + "/route {}m ≈ straight {}m, but matcher matched-path {}m): a real "
                                        + "short road path exists, so the long matched-path is a directed artifact",
                                prevNonFilteredIdx, i,
                                String.format("%.1f", prevNonFiltered.getDistance()),
                                String.format("%.1f", tp.getDistance()),
                                String.format("%.1f", phantomRouteM),
                                String.format("%.1f", straight),
                                String.format("%.1f", dfp));
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
    // Stage B.4 — corner-cut tolerance (optional)
    // ------------------------------------------------------------------------

    /**
     * Promotes short, continuous, non-detour BAD runs back to GOOD — a transient snap spike
     * from a simplifier chording a bend, where the user stayed on a continuous way. This is
     * σ-independent: it relies on the SHAPE of the deviation (short transient on a continuous
     * matched chain), not on the absolute snap vs a global threshold — so it works identically
     * on a short snippet and on a long route where auto-sigma estimates a tiny σ.
     *
     * <p>A sandwiched BAD run [s..e] is promoted only when ALL hold:
     * <ul>
     *   <li>bracketed by GOOD on both sides (not at track start/end);</li>
     *   <li>length ≤ {@link #CORNER_CUT_MAX_RUN} (transient, not a sustained departure);</li>
     *   <li>no Stage-A detour flag on the run or its closing transition (a real detour is
     *       genuinely off-grid → leave as coords);</li>
     *   <li>every snap in the run ≤ {@link #CORNER_CUT_MAX_SNAP_M} (a far spike is not a mere
     *       corner cut);</li>
     *   <li>the matcher's edges across the run are contiguous with the brackets (the user
     *       stayed on a continuous chain, not a jump to a parallel/unrelated way).</li>
     * </ul>
     * Sustained BAD runs, detours, far spikes, and discontinuous jumps are left untouched →
     * they still become coordinates regions downstream.
     */
    private void applyCornerCutTolerance(Status[] status, List<Tracepoint> tps,
                                         boolean[] direct, int[] obsToEdgeMatch, int n) {
        int i = 0;
        while (i < n) {
            if (status[i] != Status.BAD) { i++; continue; }
            int runStart = i;
            while (i < n && status[i] == Status.BAD) i++;
            int runEnd = i - 1; // inclusive; i now points past the run

            boolean bracketed = runStart > 0 && runEnd < n - 1
                    && status[runStart - 1] == Status.GOOD && status[runEnd + 1] == Status.GOOD;
            if (!bracketed) continue;
            if (runEnd - runStart + 1 > CORNER_CUT_MAX_RUN) continue;

            // No detour flag across the run or into the GOOD obs closing it.
            boolean detourInvolved = false;
            for (int j = runStart; j <= runEnd + 1; j++) {
                if (direct[j]) { detourInvolved = true; break; }
            }
            if (detourInvolved) continue;

            // Absolute snap cap.
            boolean belowCap = true;
            double maxSnap = 0;
            for (int j = runStart; j <= runEnd; j++) {
                Double d = tps.get(j).getDistance();
                if (d == null || d > CORNER_CUT_MAX_SNAP_M) { belowCap = false; break; }
                maxSnap = Math.max(maxSnap, d);
            }
            if (!belowCap) continue;

            // Matcher edges contiguous (forward-progressing) from bracket-before to after.
            if (!matchedEdgesContiguous(obsToEdgeMatch, runStart - 1, runEnd + 1)) continue;

            for (int j = runStart; j <= runEnd; j++) status[j] = Status.GOOD;
            LOGGER.info("RegionSegmenter: corner-cut tolerance promoted obs[{}..{}] "
                            + "(len {}, maxSnap {}m) back to routed",
                    runStart, runEnd, runEnd - runStart + 1, String.format("%.1f", maxSnap));
        }
    }

    /** True if the EdgeMatch indices from {@code a} to {@code b} are forward-progressing
     *  (non-decreasing), i.e. the matcher walked a continuous chain — no backward jump to an
     *  unrelated/parallel way. Filtered observations (index &lt; 0) are skipped. */
    private static boolean matchedEdgesContiguous(int[] obsToEdgeMatch, int a, int b) {
        int prev = -1;
        for (int k = a; k <= b; k++) {
            int cur = obsToEdgeMatch[k];
            if (cur < 0) continue; // filtered / unmapped — skip
            if (prev >= 0 && cur < prev) return false; // backward jump → discontinuous
            prev = cur;
        }
        return prev >= 0; // at least one mapped edge
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

        // Collect candidate observations (mapped to a slice edge, with a snap) along with
        // whether each is a FILTERED (2σ-dropped) obs. Filtered obs carry an independent
        // nearest-edge snap that is NOT guaranteed to lie on the matched path (it can be a
        // different edge at a junction), so they make poor optimizer waypoint candidates —
        // off-path probes and, when coincident, degenerate zero-length legs.
        List<Integer> allObs = new ArrayList<>();
        List<GHPoint> allSnaps = new ArrayList<>();
        List<Integer> allEdgeIdx = new ArrayList<>();
        List<Boolean> allFiltered = new ArrayList<>();
        for (int i = startObs; i <= endObs; i++) {
            int emIdx = obsToEdgeMatch[i];
            if (emIdx < 0) continue;
            Tracepoint tp = tracepoints.get(i);
            GHPoint snap = tp.getSnappedPoint();
            if (snap == null) continue;
            allObs.add(i);
            allSnaps.add(snap);
            allEdgeIdx.add(rawToDedupIdx[emIdx - firstEm]);
            allFiltered.add(tp.isFiltered());
        }

        // Optimizer waypoint candidates. When optimizerKeptObsOnly is on, drop INTERIOR
        // filtered obs (keep only KEPT/Viterbi obs, which are guaranteed on the matched path),
        // but always keep the region's FIRST and LAST candidate as boundary anchors so the
        // region endpoints and the splice to adjacent coords segments are unchanged. The
        // matcher's edge slice is unchanged, so the matched-path reference is identical; only
        // the set of points the optimizer may route BETWEEN is restricted to on-path ones.
        List<Integer> matchedObsIndices = new ArrayList<>();
        List<GHPoint> obsSnapPoints = new ArrayList<>();
        List<Integer> obsEdgeIdxInSliceL = new ArrayList<>();
        int total = allObs.size();
        for (int k = 0; k < total; k++) {
            boolean boundary = (k == 0 || k == total - 1);
            if (optimizerKeptObsOnly && !boundary && allFiltered.get(k)) {
                continue; // drop interior filtered (off-path) candidate
            }
            matchedObsIndices.add(allObs.get(k));
            obsSnapPoints.add(allSnaps.get(k));
            obsEdgeIdxInSliceL.add(allEdgeIdx.get(k));
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
