/*
 *  Licensed to GraphHopper GmbH under one or more contributor
 *  license agreements. See the NOTICE file distributed with this work for
 *  additional information regarding copyright ownership.
 *
 *  GraphHopper GmbH licenses this file to you under the Apache License,
 *  Version 2.0 (the "License"); you may not use this file except in
 *  compliance with the License. You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
package com.graphhopper.trailmap.matching;

import com.carrotsearch.hppc.IntHashSet;
import com.graphhopper.GraphHopper;
import com.graphhopper.matching.EdgeMatch;
import com.graphhopper.matching.MapMatching;
import com.graphhopper.matching.MatchResult;
import com.graphhopper.matching.Observation;
import com.graphhopper.matching.ObservationWithCandidateStates;
import com.graphhopper.matching.SequenceState;
import com.graphhopper.matching.State;
import com.graphhopper.matching.Tracepoint;
import com.graphhopper.matching.Transition;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.querygraph.QueryGraph;
import com.graphhopper.routing.querygraph.VirtualEdgeIteratorState;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.util.*;
import com.graphhopper.util.shapes.BBox;
import com.graphhopper.util.shapes.GHPoint;
import org.locationtech.jts.geom.Envelope;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import static com.graphhopper.util.DistancePlaneProjection.DIST_PLANE;

/**
 * Trailmap fork of {@link com.graphhopper.matching.MapMatching}.
 *
 * <p>This is a narrow copy-fork of GraphHopper's HMM/Viterbi matcher, created so that the
 * Trailmap-specific matching experiments (see {@code docs/gh_map_matcher_learnings.md}) live
 * entirely in the {@code trailmap} package and never modify GH core — keeping upstream
 * merges easy. It reuses all of GH's heavy machinery unchanged ({@link State},
 * {@link Snap}, {@link QueryGraph}, the {@link MapMatching.Router}, {@link EdgeMatch},
 * {@link MatchResult}, {@link Tracepoint}, etc.); only the HMM orchestration and the
 * candidate-search policy are forked.
 *
 * <p><b>Behaviour by phase</b> (all driven by {@link MatcherConfig}):
 * <ul>
 *   <li><b>Phase 0</b> — with a default-constructed config the matcher is byte-for-byte
 *       equivalent to stock GraphHopper. This is the baseline the experiments deviate from.</li>
 *   <li><b>Phase 1 (M1)</b> — {@link MatcherConfig#candidateRadiusSigmaMult} decouples the
 *       candidate search radius from sigma (see {@link #findCandidateSnaps}).</li>
 *   <li><b>Phase 6 (P3)</b> — {@link MatcherConfig#autoSigma} runs a two-pass sigma estimate
 *       (see {@link #match}).</li>
 * </ul>
 *
 * <p>Phase 5 (P1, densification) is applied by the caller to the observation list before it
 * reaches this matcher — see {@link ObservationDensifier}.
 *
 * <p>The emission/transition profile-awareness work (M2a/b/c) is not implemented here yet;
 * when it is, it will be parameterised through {@link MatcherConfig} and
 * {@link TrailmapHmmProbabilities} so this loop stays stable.
 */
public class TrailmapMapMatching {
    private final BaseGraph graph;
    private final MapMatching.Router router;
    private final LocationIndexTree locationIndex;
    private final MatcherConfig config;
    private double measurementErrorSigma;
    private double transitionProbabilityBeta;
    private final DistanceCalc distanceCalc = new DistancePlaneProjection();
    private QueryGraph queryGraph;
    /** True only during the auto-sigma probe pass. The probe only needs snap distances to
     *  estimate sigma, so it uses the canonical (~sigma) candidate radius even when M1
     *  radius-decoupling is configured — avoiding an expensive wide search at the seed sigma
     *  (the dominant cost in the auto-sigma + M1 combination; see implementation doc §7a). */
    private boolean inProbePass = false;

    private Map<String, Object> statistics = new HashMap<>();

    /**
     * Helper class to hold filtered observations along with the mapping from original indices.
     */
    static class FilterResult {
        final List<Observation> filteredObservations;
        final Map<Integer, Integer> originalToFilteredIndex; // original index -> filtered index
        final Set<Integer> filteredOutIndices; // original indices that were filtered out

        FilterResult(List<Observation> filteredObservations,
                     Map<Integer, Integer> originalToFilteredIndex,
                     Set<Integer> filteredOutIndices) {
            this.filteredObservations = filteredObservations;
            this.originalToFilteredIndex = originalToFilteredIndex;
            this.filteredOutIndices = filteredOutIndices;
        }
    }

    /**
     * Convenience factory mirroring {@link MapMatching#fromGraphHopper}. Reuses GH's own
     * router construction ({@link MapMatching#routerFromGraphHopper}) so no routing setup is
     * duplicated here.
     */
    public static TrailmapMapMatching fromGraphHopper(GraphHopper graphHopper, PMap hints, MatcherConfig config) {
        MapMatching.Router router = MapMatching.routerFromGraphHopper(graphHopper, hints);
        return new TrailmapMapMatching(graphHopper.getBaseGraph(),
                (LocationIndexTree) graphHopper.getLocationIndex(), router, config);
    }

    public TrailmapMapMatching(BaseGraph graph, LocationIndexTree locationIndex,
                               MapMatching.Router router, MatcherConfig config) {
        this.graph = graph;
        this.locationIndex = locationIndex;
        this.router = router;
        this.config = config;
        this.measurementErrorSigma = config.measurementErrorSigma;
        this.transitionProbabilityBeta = config.transitionProbabilityBeta;
    }

    /**
     * Beta parameter of the exponential distribution for modeling transition probabilities.
     * Overrides the value from {@link MatcherConfig}.
     */
    public void setTransitionProbabilityBeta(double transitionProbabilityBeta) {
        this.transitionProbabilityBeta = transitionProbabilityBeta;
    }

    /**
     * Standard deviation of the normal distribution [m] used for modeling the GPS error.
     * Overrides the value from {@link MatcherConfig}.
     */
    public void setMeasurementErrorSigma(double measurementErrorSigma) {
        this.measurementErrorSigma = measurementErrorSigma;
    }

    /**
     * Map-matches the observations.
     *
     * <p>Phase 6 (P3): when {@link MatcherConfig#autoSigma} is on, a probe pass is run at the
     * seed sigma, the per-observation snap distances are used to estimate a track-appropriate
     * sigma (off-grid outliers excluded), and the final match is run at the estimated sigma.
     * Otherwise a single pass is run at the configured sigma.
     */
    public MatchResult match(List<Observation> observations) {
        // V2: adaptive per-observation sigma path (snap-only pre-pass + per-obs σ). Default off.
        if (config.adaptiveSigma) {
            return matchAdaptive(observations);
        }

        if (!config.autoSigma) {
            long t0 = System.nanoTime();
            MatchResult result = matchInternal(observations, null);
            statistics.put("matchFinalPassNs", System.nanoTime() - t0);
            statistics.put("matchProbePassNs", 0L);
            return result;
        }

        // --- Phase 6 (P3): two-pass sigma estimation ---
        // Probe pass: canonical narrow radius (inProbePass=true) — only snap distances are
        // needed to estimate sigma, so we skip the expensive wide M1 search here.
        this.measurementErrorSigma = config.autoSigmaSeedM;
        inProbePass = true;
        long tProbeStart = System.nanoTime();
        MatchResult probe;
        try {
            probe = matchInternal(observations, null);
        } finally {
            inProbePass = false;
        }
        long tProbeEnd = System.nanoTime();
        double estimated = estimateSigma(probe);
        // Final pass: at the (small) estimated sigma, with M1 radius decoupling if configured.
        this.measurementErrorSigma = estimated;
        long tFinalStart = System.nanoTime();
        MatchResult result = matchInternal(observations, null);
        long tFinalEnd = System.nanoTime();
        statistics.put("matchProbePassNs", tProbeEnd - tProbeStart);
        statistics.put("matchFinalPassNs", tFinalEnd - tFinalStart);
        statistics.put("autoSigmaSeedM", config.autoSigmaSeedM);
        statistics.put("autoSigmaEstimatedM", estimated);
        return result;
    }

    /**
     * V2 adaptive per-observation σ path. Runs a cheap snap-only pre-pass (nearest snap per obs,
     * no Viterbi — also replaces the auto-sigma probe-pass Viterbi), derives a per-observation σ
     * from a smoothed/clamped windowed estimate, and runs the final match with that per-obs σ for
     * the candidate radius and the emission term. The FILTER and SEGMENTER use a single
     * representative capped σ (the absolute on/off-network threshold must not follow off-road
     * inflation; reported as {@code autoSigmaEstimatedM}). See MatcherConfig adaptiveSigma*.
     */
    private MatchResult matchAdaptive(List<Observation> observations) {
        int n = observations.size();
        // 1. Snap-only pre-pass — nearest-snap distance per observation. inProbePass forces the
        // canonical auto-expanding radius so the nearest snap is always found cheaply (no Viterbi).
        long tProbeStart = System.nanoTime();
        double[] dAll = new double[n];
        boolean prevProbe = inProbePass;
        inProbePass = true;
        try {
            for (int i = 0; i < n; i++) {
                GHPoint p = observations.get(i).getPoint();
                List<Snap> snaps = findCandidateSnaps(p.lat, p.lon, config.autoSigmaSeedM);
                dAll[i] = snaps.isEmpty() ? Double.NaN : snaps.get(0).getQueryDistance();
            }
        } finally {
            inProbePass = prevProbe;
        }
        long tProbeEnd = System.nanoTime();
        // 2. Representative capped σ (on-network noise) then per-obs windowed σ. The per-obs window
        // drops off-grid outliers (same filter as representativeSigma), so σ tracks on-network GPS
        // noise rather than off-road excursion distance; windows with no on-network snap fall back
        // to the representative σ. This stops σ inflating through off-road stretches — inflation
        // widened the candidate radius enough to admit the off-road detour edge, and transition
        // continuity then glued boundary obs to it, shifting the coords-segment boundary (sorel).
        double reprSigma = representativeSigma(dAll);
        double[] sigmaPerObs = computeAdaptiveSigma(dAll, n, reprSigma);
        this.measurementErrorSigma = reprSigma; // filter uses this; segmenter reads it via stats
        // 3. Final match: per-obs σ for radius + emission; filter on reprSigma.
        long tFinalStart = System.nanoTime();
        MatchResult result = matchInternal(observations, sigmaPerObs);
        long tFinalEnd = System.nanoTime();
        statistics.put("adaptiveSigma", true);
        statistics.put("autoSigmaEstimatedM", reprSigma);
        statistics.put("matchProbePassNs", tProbeEnd - tProbeStart);
        statistics.put("matchFinalPassNs", tFinalEnd - tFinalStart);
        // Diagnostics (additive; no behaviour change): per-obs σ and the snap-only pre-pass nearest
        // snap, so the test harness can dump σ vs snap across a coords boundary.
        statistics.put("adaptiveSigmaPerObs", sigmaPerObs);
        statistics.put("adaptiveSnapPrePass", dAll);
        return result;
    }

    /** Windowed-percentile per-observation σ from nearest-snap distances, clamped
     *  [adaptiveSigmaMinM, adaptiveSigmaMaxM]. NaN (unsnapped) entries skipped within the window.
     *  When {@link MatcherConfig#adaptiveSigmaOutlierFilter} is on, off-grid snaps
     *  (&gt; {@link MatcherConfig#autoSigmaOutlierM}) are dropped from the window so σ reflects
     *  on-network GPS noise, not off-road distance; a window with no on-network snap falls back to
     *  {@code reprSigma} (the representative on-network estimate) instead of the floor. */
    private double[] computeAdaptiveSigma(double[] dAll, int n, double reprSigma) {
        double[] sigma = new double[n];
        int w = config.adaptiveSigmaWindow;
        boolean filter = config.adaptiveSigmaOutlierFilter;
        List<Double> win = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            win.clear();
            int lo = Math.max(0, i - w), hi = Math.min(n - 1, i + w);
            for (int j = lo; j <= hi; j++) {
                double v = dAll[j];
                if (Double.isNaN(v)) continue;
                if (filter && v > config.autoSigmaOutlierM) continue; // drop off-grid outliers
                win.add(v);
            }
            double s;
            if (win.isEmpty()) {
                // No on-network snap in the window (deep off-road): use the representative
                // on-network σ so radius/emission match the on-network baseline rather than
                // collapsing to the floor.
                s = filter ? reprSigma : config.adaptiveSigmaMinM;
            } else {
                Collections.sort(win);
                s = percentile(win, config.adaptiveSigmaPercentile);
            }
            sigma[i] = clamp(s, config.adaptiveSigmaMinM, config.adaptiveSigmaMaxM);
        }
        return sigma;
    }

    /** Representative global σ for the filter + segmenter on/off-network threshold: robust-upper
     *  estimate of the on-grid (≤ outlier) nearest snaps, clamped to a TIGHT cap so off-road
     *  stretches cannot inflate the absolute threshold. */
    private double representativeSigma(double[] dAll) {
        List<Double> onGrid = new ArrayList<>();
        for (double v : dAll) {
            if (!Double.isNaN(v) && v <= config.autoSigmaOutlierM) onGrid.add(v);
        }
        double base;
        if (onGrid.isEmpty()) {
            base = config.autoSigmaSeedM;
        } else {
            Collections.sort(onGrid);
            base = percentile(onGrid, config.autoSigmaPercentile);
        }
        return clamp(base, config.autoSigmaMinM, config.adaptiveSigmaReprMaxM);
    }

    /**
     * Phase 6 (P3) helper: estimate a track-appropriate sigma from a probe match.
     *
     * <p>Drops snap distances above {@link MatcherConfig#autoSigmaOutlierM} (off-grid /
     * off-network sections whose snaps are arbitrary and would inflate the estimate), then
     * takes a ROBUST UPPER estimate — the {@link MatcherConfig#autoSigmaPercentile} percentile
     * of the remaining per-observation snap distances — scales by
     * {@link MatcherConfig#autoSigmaScale}, and clamps to
     * [{@link MatcherConfig#autoSigmaMinM}, {@link MatcherConfig#autoSigmaMaxM}].
     *
     * <p>The percentile (rather than the mean) is deliberate: on a hard-simplified route the
     * mean is dragged to the floor by the many tiny straight-section snaps, making the
     * σ-derived snap threshold too tight for legitimate corner-cut deviations (sbc_74). A high
     * percentile reflects the track's real deviation scale while still excluding off-grid
     * outliers (dropped above {@link MatcherConfig#autoSigmaOutlierM}).
     */
    private double estimateSigma(MatchResult probe) {
        List<Double> kept = new ArrayList<>();
        List<Tracepoint> tps = probe.getTracepoints();
        if (tps != null) {
            // Detour-aware: exclude observations on an off-road excursion (incoming transition
            // is a detour) so σ reflects GPS error on the network, not off-road distance.
            Tracepoint prevNonFiltered = null;
            for (Tracepoint tp : tps) {
                if (!tp.isMatched() || tp.isFiltered() || tp.getDistance() == null) {
                    continue;
                }
                boolean offGridSnap = tp.getDistance() > config.autoSigmaOutlierM;
                boolean offRoadTransition = false;
                if (prevNonFiltered != null && tp.getDistanceFromPrevious() != null) {
                    double straight = distanceCalc.calcDist(
                            prevNonFiltered.getOriginalPoint().lat, prevNonFiltered.getOriginalPoint().lon,
                            tp.getOriginalPoint().lat, tp.getOriginalPoint().lon);
                    double matched = tp.getDistanceFromPrevious();
                    if (straight > 0 && matched > config.autoSigmaDetourRatio * straight
                            && matched > config.autoSigmaDetourMinM) {
                        offRoadTransition = true;
                    }
                }
                if (!offGridSnap && !offRoadTransition) {
                    kept.add(tp.getDistance());
                }
                prevNonFiltered = tp;
            }
        }
        double base;
        if (kept.isEmpty()) {
            base = config.autoSigmaSeedM; // nothing on-grid to learn from; keep seed
        } else {
            Collections.sort(kept);
            base = percentile(kept, config.autoSigmaPercentile);
        }
        double est = config.autoSigmaScale * base;
        return clamp(est, config.autoSigmaMinM, config.autoSigmaMaxM);
    }

    /** Linear-interpolated percentile of a pre-sorted list. p clamped to [0,1]. */
    private static double percentile(List<Double> sorted, double p) {
        if (sorted.isEmpty()) return 0.0;
        if (sorted.size() == 1) return sorted.get(0);
        double pp = Math.max(0.0, Math.min(1.0, p));
        double rank = pp * (sorted.size() - 1);
        int lo = (int) Math.floor(rank);
        int hi = (int) Math.ceil(rank);
        double frac = rank - lo;
        return sorted.get(lo) + frac * (sorted.get(hi) - sorted.get(lo));
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /**
     * @param sigmaPerObs optional per-ORIGINAL-observation σ (aligned to {@code observations}).
     *                    {@code null} → global behaviour (uses {@link #measurementErrorSigma} for
     *                    radius + emission), byte-identical to the pre-adaptive path. When non-null,
     *                    each kept observation's σ drives its candidate radius and emission term
     *                    (the adaptive-σ path).
     */
    private MatchResult matchInternal(List<Observation> observations, double[] sigmaPerObs) {
        // Filter observations and track which original indices were kept
        FilterResult filterResult = filterObservationsWithTracking(observations);
        List<Observation> filteredObservations = filterResult.filteredObservations;
        statistics.put("filteredObservations", filteredObservations.size());

        // Map each filtered (kept) obs back to its original index, so per-observation σ (indexed by
        // original obs) can be applied to the candidate radius + emission. Null when σ is global.
        final int[] f2o;
        if (sigmaPerObs != null) {
            f2o = new int[filteredObservations.size()];
            for (Map.Entry<Integer, Integer> e : filterResult.originalToFilteredIndex.entrySet()) {
                f2o[e.getValue()] = e.getKey();
            }
        } else {
            f2o = null;
        }

        // Snap observations to links. Generates multiple candidate snaps per observation.
        long tCandStart = System.nanoTime();
        List<List<Snap>> snapsPerObservation = IntStream.range(0, filteredObservations.size())
                .mapToObj(k -> {
                    Observation o = filteredObservations.get(k);
                    return sigmaPerObs != null
                            ? findCandidateSnaps(o.getPoint().lat, o.getPoint().lon, sigmaPerObs[f2o[k]])
                            : findCandidateSnaps(o.getPoint().lat, o.getPoint().lon);
                })
                .collect(Collectors.toList());
        long tCandEnd = System.nanoTime();
        // Accumulate (probe + final pass both call matchInternal; we accumulate for final only
        // by overwriting; caller stores per-pass totals via matchProbePassNs/matchFinalPassNs).
        statistics.put("candidateSearchNs", tCandEnd - tCandStart);
        statistics.put("snapsPerObservation", snapsPerObservation.stream().mapToInt(Collection::size).toArray());

        // Create the query graph, containing split edges so that all the places where an observation might have happened
        // are a node. This modifies the Snap objects and puts the new node numbers into them.
        queryGraph = QueryGraph.create(graph, snapsPerObservation.stream().flatMap(Collection::stream).collect(Collectors.toList()));

        // Creates candidates from the Snaps of all observations (a candidate is basically a
        // Snap + direction).
        List<ObservationWithCandidateStates> timeSteps = createTimeSteps(filteredObservations, snapsPerObservation);

        // Per-time-step σ for the emission term (aligned to filtered-obs / timeStep order). Null
        // when σ is global → emission uses the instance σ exactly as before.
        double[] sigmaPerTimeStep = null;
        if (sigmaPerObs != null) {
            sigmaPerTimeStep = new double[filteredObservations.size()];
            for (int k = 0; k < sigmaPerTimeStep.length; k++) sigmaPerTimeStep[k] = sigmaPerObs[f2o[k]];
        }

        // Compute the most likely sequence of map matching candidates. Gap-tolerant: the result is
        // one or more matched runs (§ gh_convert_track_matcher_gap_splitting.md); unmatchable
        // stretches are gaps rather than a whole-track failure.
        long tViterbiStart = System.nanoTime();
        ViterbiResult vr = computeViterbiSequence(timeSteps, sigmaPerTimeStep);
        long tViterbiEnd = System.nanoTime();

        // Flat placed sequence in track order, with its timestep alignment. Iterating
        // stateByTimeStep yields exactly the runs concatenated in order (runs are non-overlapping
        // and increasing), so `seq` matches the pre-gap `seq` when the track is one run.
        List<SequenceState<State, Observation, Path>> seq = new ArrayList<>();
        int[] seqTimeStepTmp = new int[timeSteps.size()];
        int placed = 0;
        for (int t = 0; t < vr.stateByTimeStep.size(); t++) {
            SequenceState<State, Observation, Path> ss = vr.stateByTimeStep.get(t);
            if (ss != null) {
                seq.add(ss);
                seqTimeStepTmp[placed++] = t;
            }
        }
        final int[] seqTimeStep = Arrays.copyOf(seqTimeStepTmp, placed);

        statistics.put("viterbiNs", tViterbiEnd - tViterbiStart);
        statistics.put("transitionDistances", seq.stream().filter(s -> s.transitionDescriptor != null).mapToLong(s -> Math.round(s.transitionDescriptor.getDistance())).toArray());
        statistics.put("visitedNodes", router.getVisitedNodes());
        statistics.put("snapDistanceRanks", IntStream.range(0, seq.size()).map(i -> snapsPerObservation.get(seqTimeStep[i]).indexOf(seq.get(i).state.getSnap())).toArray());
        statistics.put("snapDistances", seq.stream().mapToDouble(s -> s.state.getSnap().getQueryDistance()).toArray());
        statistics.put("maxSnapDistances", IntStream.range(0, seq.size()).mapToDouble(i -> snapsPerObservation.get(seqTimeStep[i]).stream().mapToDouble(Snap::getQueryDistance).max().orElse(-1.0)).toArray());

        List<EdgeIteratorState> path = seq.stream().filter(s1 -> s1.transitionDescriptor != null).flatMap(s1 -> s1.transitionDescriptor.calcEdges().stream()).collect(Collectors.toList());

        // EdgeMatches: build per run and concatenate, so no edge/state is attributed across a gap
        // (prepareEdgeMatches would otherwise glue the last edge of one run to the first of the
        // next). With a single run this equals prepareEdgeMatches(seq) exactly.
        List<EdgeMatch> edgeMatches = new ArrayList<>();
        for (List<SequenceState<State, Observation, Path>> run : vr.runs) {
            edgeMatches.addAll(prepareEdgeMatches(run));
        }

        MatchResult result = new MatchResult(edgeMatches);
        Weighting queryGraphWeighting = queryGraph.wrapWeighting(router.getWeighting());
        result.setMergedPath(new MapMatchedPath(queryGraph, queryGraphWeighting, path));
        result.setMatchMillis(seq.stream().filter(s -> s.transitionDescriptor != null).mapToLong(s -> s.transitionDescriptor.getTime()).sum());
        result.setMatchLength(seq.stream().filter(s -> s.transitionDescriptor != null).mapToDouble(s -> s.transitionDescriptor.getDistance()).sum());
        result.setGPXEntriesLength(gpxLength(observations));
        result.setGraph(queryGraph);
        result.setWeighting(queryGraphWeighting);

        // Build tracepoints with 1:1 correspondence to input observations. Gap timesteps
        // (null in stateByTimeStep) become unmatched tracepoints → coordinates downstream.
        List<Tracepoint> tracepoints = buildTracepoints(observations, filterResult, vr.stateByTimeStep);
        result.setTracepoints(tracepoints);

        return result;
    }

    /**
     * Builds tracepoints array with 1:1 correspondence to original input observations.
     * Similar to OSRM's tracepoints output.
     *
     * For points that went through Viterbi: uses the snap from the matching result.
     * For filtered points (too close to previous): computes snap separately.
     */
    private List<Tracepoint> buildTracepoints(List<Observation> originalObservations,
                                               FilterResult filterResult,
                                               List<SequenceState<State, Observation, Path>> stateByTimeStep) {
        List<Tracepoint> tracepoints = new ArrayList<>(originalObservations.size());

        for (int i = 0; i < originalObservations.size(); i++) {
            Observation obs = originalObservations.get(i);
            GHPoint originalPoint = obs.getPoint();

            if (filterResult.filteredOutIndices.contains(i)) {
                // This observation was filtered out (too close to previous)
                // Compute snap separately - this doesn't affect the route.
                // Filtered observations do not participate in Viterbi, so they have no
                // matcher transition path from a previous tracepoint; distanceFromPrevious = null.
                List<Snap> snaps = findCandidateSnaps(originalPoint.getLat(), originalPoint.getLon());
                if (!snaps.isEmpty()) {
                    Snap snap = snaps.get(0); // closest snap
                    GHPoint snappedPoint = new GHPoint(
                            snap.getSnappedPoint().getLat(),
                            snap.getSnappedPoint().getLon()
                    );
                    double distance = snap.getQueryDistance();
                    int edgeId = snap.getClosestEdge().getEdge();
                    tracepoints.add(new Tracepoint(i, originalPoint, true, snappedPoint, distance, edgeId, null));
                } else {
                    // No snap candidates found
                    tracepoints.add(new Tracepoint(i, originalPoint, true));
                }
            } else {
                // This observation was a Viterbi timestep. It is either PLACED (in a matched run)
                // or a GAP (no candidates / unreachable) — the latter is an unmatched tracepoint,
                // which the segmenter renders as part of a coordinates section.
                Integer filteredIndex = filterResult.originalToFilteredIndex.get(i);
                SequenceState<State, Observation, Path> seqState =
                        (filteredIndex != null && filteredIndex < stateByTimeStep.size())
                                ? stateByTimeStep.get(filteredIndex) : null;
                if (seqState != null) {
                    Snap snap = seqState.state.getSnap();
                    GHPoint snappedPoint = new GHPoint(
                            snap.getSnappedPoint().getLat(),
                            snap.getSnappedPoint().getLon()
                    );
                    double distance = snap.getQueryDistance();
                    int edgeId = snap.getClosestEdge().getEdge();
                    // Pull the matcher's HMM transition distance from the previous Viterbi
                    // state. transitionDescriptor is null for the first state of each run
                    // (no incoming transition — the previous timestep is a gap or track start);
                    // for subsequent states it is the routing Path the HMM chose.
                    Double distanceFromPrevious = (seqState.transitionDescriptor != null)
                            ? seqState.transitionDescriptor.getDistance() : null;
                    tracepoints.add(new Tracepoint(i, originalPoint, false, snappedPoint, distance, edgeId, distanceFromPrevious));
                } else {
                    // GAP timestep: unmatched (no acceptable placement on the network).
                    tracepoints.add(new Tracepoint(i, originalPoint, false));
                }
            }
        }

        return tracepoints;
    }

    /**
     * Filters observations to only those which will be used for map matching (i.e. those which
     * are separated by at least 2 * measurementErrorSigman
     */
    public List<Observation> filterObservations(List<Observation> observations) {
        List<Observation> filtered = new ArrayList<>();
        Observation prevEntry = null;
        double acc = 0.0;
        int last = observations.size() - 1;
        for (int i = 0; i <= last; i++) {
            Observation observation = observations.get(i);
            if (i == 0 || i == last || distanceCalc.calcDist(
                    prevEntry.getPoint().getLat(), prevEntry.getPoint().getLon(),
                    observation.getPoint().getLat(), observation.getPoint().getLon()) > 2 * measurementErrorSigma) {
                if (i > 0) {
                    Observation prevObservation = observations.get(i - 1);
                    acc += distanceCalc.calcDist(
                            prevObservation.getPoint().getLat(), prevObservation.getPoint().getLon(),
                            observation.getPoint().getLat(), observation.getPoint().getLon());
                    acc -= distanceCalc.calcDist(
                            prevEntry.getPoint().getLat(), prevEntry.getPoint().getLon(),
                            observation.getPoint().getLat(), observation.getPoint().getLon());
                }
                // Here we store the meters of distance that we are missing because of the filtering,
                // so that when we add these terms to the distances between the filtered points,
                // the original total distance between the unfiltered points is conserved.
                // (See test for kind of a specification.)
                observation.setAccumulatedLinearDistanceToPrevious(acc);
                filtered.add(observation);
                prevEntry = observation;
                acc = 0.0;
            } else {
                Observation prevObservation = observations.get(i - 1);
                acc += distanceCalc.calcDist(
                        prevObservation.getPoint().getLat(), prevObservation.getPoint().getLon(),
                        observation.getPoint().getLat(), observation.getPoint().getLon());
            }
        }
        return filtered;
    }

    /**
     * Filters observations and tracks which original indices were kept vs filtered out.
     * This is needed to build tracepoints with 1:1 correspondence to input.
     */
    FilterResult filterObservationsWithTracking(List<Observation> observations) {
        List<Observation> filtered = new ArrayList<>();
        Map<Integer, Integer> originalToFilteredIndex = new HashMap<>();
        Set<Integer> filteredOutIndices = new HashSet<>();

        Observation prevEntry = null;
        double acc = 0.0;
        int last = observations.size() - 1;
        int filteredIndex = 0;

        for (int i = 0; i <= last; i++) {
            Observation observation = observations.get(i);
            if (i == 0 || i == last || distanceCalc.calcDist(
                    prevEntry.getPoint().getLat(), prevEntry.getPoint().getLon(),
                    observation.getPoint().getLat(), observation.getPoint().getLon()) > 2 * measurementErrorSigma) {
                if (i > 0) {
                    Observation prevObservation = observations.get(i - 1);
                    acc += distanceCalc.calcDist(
                            prevObservation.getPoint().getLat(), prevObservation.getPoint().getLon(),
                            observation.getPoint().getLat(), observation.getPoint().getLon());
                    acc -= distanceCalc.calcDist(
                            prevEntry.getPoint().getLat(), prevEntry.getPoint().getLon(),
                            observation.getPoint().getLat(), observation.getPoint().getLon());
                }
                observation.setAccumulatedLinearDistanceToPrevious(acc);
                filtered.add(observation);
                originalToFilteredIndex.put(i, filteredIndex);
                filteredIndex++;
                prevEntry = observation;
                acc = 0.0;
            } else {
                filteredOutIndices.add(i);
                Observation prevObservation = observations.get(i - 1);
                acc += distanceCalc.calcDist(
                        prevObservation.getPoint().getLat(), prevObservation.getPoint().getLon(),
                        observation.getPoint().getLat(), observation.getPoint().getLon());
            }
        }
        return new FilterResult(filtered, originalToFilteredIndex, filteredOutIndices);
    }

    /**
     * Finds candidate snaps for one observation.
     *
     * <p><b>Canonical (Phase 0):</b> when {@link MatcherConfig#candidateRadiusSigmaMult} is
     * null, this reproduces stock GraphHopper exactly — an envelope that expands by ~1 sigma
     * each iteration and returns the first non-empty ring (effective radius ≈ sigma).
     *
     * <p><b>Decoupled (Phase 1 / M1):</b> when the multiplier is set, the search radius is
     * {@code clamp(mult * sigma, minM, maxM)} and is used as the per-iteration expansion
     * step. The first non-empty ring is therefore already {@code radius} wide, so it
     * collects <b>all</b> candidates within {@code radius} (coverage), while the emission
     * model keeps using {@code sigma} for sharpness (discrimination). If nothing is found
     * within {@code radius}, the envelope keeps expanding by {@code radius} (up to 50
     * iterations) so off-grid points still get a snap rather than breaking the sequence.
     */
    public List<Snap> findCandidateSnaps(final double queryLat, final double queryLon) {
        return findCandidateSnaps(queryLat, queryLon, measurementErrorSigma);
    }

    /** As {@link #findCandidateSnaps(double, double)} but with an EXPLICIT sigma for the radius
     *  (adaptive-σ path). Called with {@code measurementErrorSigma} from the no-arg overload, so the
     *  global path is unchanged. */
    public List<Snap> findCandidateSnaps(final double queryLat, final double queryLon, double sigma) {
        double searchRadius = (config.isRadiusDecoupled() && !inProbePass)
                ? clamp(config.candidateRadiusSigmaMult * sigma,
                        config.candidateRadiusMinM, config.candidateRadiusMaxM)
                : sigma;
        double rLon = (searchRadius * 360.0 / DistanceCalcEarth.DIST_EARTH.calcCircumference(queryLat));
        double rLat = searchRadius / DistanceCalcEarth.METERS_PER_DEGREE;
        Envelope envelope = new Envelope(queryLon, queryLon, queryLat, queryLat);
        for (int i = 0; i < 50; i++) {
            envelope.expandBy(rLon, rLat);
            List<Snap> snaps = findCandidateSnapsInBBox(queryLat, queryLon, BBox.fromEnvelope(envelope));
            if (!snaps.isEmpty()) {
                return snaps;
            }
        }
        return Collections.emptyList();
    }

    private List<Snap> findCandidateSnapsInBBox(double queryLat, double queryLon, BBox queryShape) {
        EdgeFilter edgeFilter = router.getSnapFilter();
        List<Snap> snaps = new ArrayList<>();
        IntHashSet seenEdges = new IntHashSet();
        IntHashSet seenNodes = new IntHashSet();
        locationIndex.query(queryShape, edgeId -> {
            EdgeIteratorState edge = graph.getEdgeIteratorStateForKey(edgeId * 2);
            if (seenEdges.add(edgeId) && edgeFilter.accept(edge)) {
                Snap snap = new Snap(queryLat, queryLon);
                locationIndex.traverseEdge(queryLat, queryLon, edge, (node, normedDist, wayIndex, pos) -> {
                    if (normedDist < snap.getQueryDistance()) {
                        snap.setQueryDistance(normedDist);
                        snap.setClosestNode(node);
                        snap.setWayIndex(wayIndex);
                        snap.setSnappedPosition(pos);
                    }
                });
                double dist = DIST_PLANE.calcDenormalizedDist(snap.getQueryDistance());
                snap.setClosestEdge(edge);
                snap.setQueryDistance(dist);
                if (snap.isValid() && (snap.getSnappedPosition() != Snap.Position.TOWER || seenNodes.add(snap.getClosestNode()))) {
                    snap.calcSnappedPoint(DistanceCalcEarth.DIST_EARTH);
                    if (queryShape.contains(snap.getSnappedPoint().lat, snap.getSnappedPoint().lon)) {
                        snaps.add(snap);
                    }
                }
            }
        });
        snaps.sort(Comparator.comparingDouble(Snap::getQueryDistance));
        return snaps;
    }

    /**
     * Creates TimeSteps with candidates for the GPX entries but does not create emission or
     * transition probabilities. Creates directed candidates for virtual nodes and undirected
     * candidates for real nodes.
     */
    private List<ObservationWithCandidateStates> createTimeSteps(List<Observation> filteredObservations, List<List<Snap>> splitsPerObservation) {
        if (splitsPerObservation.size() != filteredObservations.size()) {
            throw new IllegalArgumentException(
                    "filteredGPXEntries and queriesPerEntry must have same size.");
        }

        final List<ObservationWithCandidateStates> timeSteps = new ArrayList<>();
        for (int i = 0; i < filteredObservations.size(); i++) {
            Observation observation = filteredObservations.get(i);
            Collection<Snap> splits = splitsPerObservation.get(i);
            List<State> candidates = new ArrayList<>();
            for (Snap split : splits) {
                if (queryGraph.isVirtualNode(split.getClosestNode())) {
                    List<VirtualEdgeIteratorState> virtualEdges = new ArrayList<>();
                    EdgeIterator iter = queryGraph.createEdgeExplorer().setBaseNode(split.getClosestNode());
                    while (iter.next()) {
                        if (!queryGraph.isVirtualEdge(iter.getEdge())) {
                            throw new RuntimeException("Virtual nodes must only have virtual edges "
                                    + "to adjacent nodes.");
                        }
                        virtualEdges.add((VirtualEdgeIteratorState) queryGraph.getEdgeIteratorState(iter.getEdge(), iter.getAdjNode()));
                    }
                    if (virtualEdges.size() != 2) {
                        throw new RuntimeException("Each virtual node must have exactly 2 "
                                + "virtual edges (reverse virtual edges are not returned by the "
                                + "EdgeIterator");
                    }

                    // Create a directed candidate for each of the two possible directions through
                    // the virtual node. We need to add candidates for both directions because
                    // we don't know yet which is the correct one. This will be figured
                    // out by the Viterbi algorithm.
                    candidates.add(new State(observation, split, virtualEdges.get(0), virtualEdges.get(1)));
                    candidates.add(new State(observation, split, virtualEdges.get(1), virtualEdges.get(0)));
                } else {
                    // Create an undirected candidate for the real node.
                    candidates.add(new State(observation, split));
                }
            }

            timeSteps.add(new ObservationWithCandidateStates(observation, candidates));
        }
        return timeSteps;
    }

    static class Label {
        int timeStep;
        State state;
        Label back;
        boolean isDeleted;
        double minusLogProbability;
    }

    /**
     * Per-candidate cost record, populated only when {@link MatcherConfig#debugCandidateCosts}
     * is on. Captures the matcher's own internal cost breakdown for one candidate way at one
     * observation — the authoritative numbers for sizing the M2a desirability penalty.
     *
     * <ul>
     *   <li>{@code emissionCost} — how much this candidate is penalised for its snap distance
     *       (lower = closer). This is what an M2a desirability term would be added to.</li>
     *   <li>{@code transitionCost} — penalty for the best incoming connection (how natural the
     *       forward trip into this candidate is). {@code null} for the first observation.</li>
     *   <li>{@code transitionTripLengthM} — length of that connecting trip. {@code null} for the
     *       first observation.</li>
     *   <li>{@code accumulatedCost} — total cost of the best whole-trip-so-far ending at this
     *       candidate. This is the number the matcher actually compares between candidates.</li>
     * </ul>
     */
    public static class CandidateCost {
        public final double obsLat;
        public final double obsLon;
        public final int edgeId;
        public final double snapDistanceM;
        public final double emissionCost;
        public final Double transitionCost;
        public final Double transitionTripLengthM;
        public final double accumulatedCost;

        CandidateCost(double obsLat, double obsLon, int edgeId, double snapDistanceM,
                      double emissionCost, Double transitionCost, Double transitionTripLengthM,
                      double accumulatedCost) {
            this.obsLat = obsLat;
            this.obsLon = obsLon;
            this.edgeId = edgeId;
            this.snapDistanceM = snapDistanceM;
            this.emissionCost = emissionCost;
            this.transitionCost = transitionCost;
            this.transitionTripLengthM = transitionTripLengthM;
            this.accumulatedCost = accumulatedCost;
        }
    }

    /** Emission log-prob for a candidate at {@code timeStep}: per-obs σ when
     *  {@code sigmaPerTimeStep != null}, else the global instance σ (unchanged behaviour). */
    private static double emissionLog(TrailmapHmmProbabilities p, double[] sigmaPerTimeStep,
                                      int timeStep, double distance) {
        return sigmaPerTimeStep == null
                ? p.emissionLogProbability(distance)
                : p.emissionLogProbability(distance, sigmaPerTimeStep[timeStep]);
    }

    /**
     * Result of the gap-tolerant Viterbi. See {@code docs/gh_convert_track_matcher_gap_splitting.md}.
     *
     * <ul>
     *   <li>{@code runs} — one matched sub-sequence per maximal placeable+bridgeable run, in track
     *       order. Each run's first {@link SequenceState} has a {@code null} transitionDescriptor
     *       (fresh start). Consumed for the EdgeMatch list (built per run, then concatenated, so no
     *       edges/states are attributed across a gap).</li>
     *   <li>{@code stateByTimeStep} — the chosen state for each timestep, or {@code null} for a GAP
     *       timestep (no candidates, or unreachable from the frontier). Length ==
     *       {@code timeSteps.size()}. Drives the 1:1 tracepoints: a {@code null} entry becomes an
     *       unmatched tracepoint, which the segmenter renders as a coordinates section.</li>
     * </ul>
     */
    private static final class ViterbiResult {
        final List<List<SequenceState<State, Observation, Path>>> runs;
        final List<SequenceState<State, Observation, Path>> stateByTimeStep;

        ViterbiResult(List<List<SequenceState<State, Observation, Path>>> runs,
                      List<SequenceState<State, Observation, Path>> stateByTimeStep) {
            this.runs = runs;
            this.stateByTimeStep = stateByTimeStep;
        }
    }

    /** First timestep index at or after {@code from} that has at least one candidate, or -1. A
     *  timestep with no candidates can neither seed a run nor be reached, so it is always a gap. */
    private static int firstSeedableTimeStep(List<ObservationWithCandidateStates> timeSteps, int from) {
        for (int i = from; i < timeSteps.size(); i++) {
            if (!timeSteps.get(i).candidates.isEmpty()) return i;
        }
        return -1;
    }

    /** Deepest reachable label in {@code labels} (max timeStep; ties broken by lower cost). Used to
     *  end a run when the frontier stalled before the last timestep. */
    private static Label deepestTerminal(Map<State, Label> labels) {
        Label best = null;
        for (Label l : labels.values()) {
            if (best == null || l.timeStep > best.timeStep
                    || (l.timeStep == best.timeStep && l.minusLogProbability < best.minusLogProbability)) {
                best = l;
            }
        }
        return best;
    }

    /**
     * Gap-tolerant HMM/Viterbi that splits the track into matched runs instead of failing the whole
     * track when a stretch cannot be matched (restores the OSRM predecessor's section behaviour; see
     * {@code docs/gh_convert_track_matcher_gap_splitting.md}). Seed at the first placeable timestep,
     * expand as far as transitions allow, record that run, then reseed after the stall. The
     * timesteps skipped between runs (no candidates, or unreachable) are gaps — represented as
     * {@code null} entries in {@link ViterbiResult#stateByTimeStep} and rendered downstream as a
     * coordinates section. With a track that matches end-to-end this produces exactly one run and no
     * gaps — byte-identical to the previous single-sequence behaviour.
     */
    private ViterbiResult computeViterbiSequence(
            List<ObservationWithCandidateStates> timeSteps, double[] sigmaPerTimeStep) {
        final int T = timeSteps.size();
        List<List<SequenceState<State, Observation, Path>>> runs = new ArrayList<>();
        List<SequenceState<State, Observation, Path>> stateByTimeStep =
                new ArrayList<>(Collections.nCopies(T, null));
        if (T == 0) {
            return new ViterbiResult(runs, stateByTimeStep);
        }

        final TrailmapHmmProbabilities probabilities = new TrailmapHmmProbabilities(measurementErrorSigma, transitionProbabilityBeta);
        final Map<Transition<State>, Path> roadPaths = new HashMap<>();
        // Diagnostics only (MatcherConfig.debugCandidateCosts); no effect on the match.
        final Map<State, CandidateCost> debugCosts = config.debugCandidateCosts ? new HashMap<>() : null;
        // Phase 2 (M2a): per-candidate desirability penalty (cost units). null when off.
        final Map<State, Double> emPenalty = computeDesirabilityPenalties(timeSteps);

        int start = firstSeedableTimeStep(timeSteps, 0);
        while (start >= 0) {
            final Map<State, Label> labels = new HashMap<>();
            PriorityQueue<Label> q = new PriorityQueue<>(Comparator.comparing(qe0 -> qe0.minusLogProbability));
            for (State candidate : timeSteps.get(start).candidates) {
                // distance from observation to road in meters
                final double distance = candidate.getSnap().getQueryDistance();
                Label label = new Label();
                label.state = candidate;
                label.timeStep = start;
                label.minusLogProbability = emissionLog(probabilities, sigmaPerTimeStep, start, distance) * -1.0
                        + penaltyOf(emPenalty, candidate);
                q.add(label);
                labels.put(candidate, label);
                if (debugCosts != null) {
                    GHPoint p = candidate.getEntry().getPoint();
                    debugCosts.put(candidate, new CandidateCost(p.lat, p.lon,
                            candidate.getSnap().getClosestEdge().getEdge(), distance,
                            -emissionLog(probabilities, sigmaPerTimeStep, start, distance), null, null,
                            label.minusLogProbability));
                }
            }
            Label qe = null;
            boolean reachedEnd = false;
            while (!q.isEmpty()) {
                qe = q.poll();
                if (qe.isDeleted)
                    continue;
                if (qe.timeStep == T - 1) {
                    reachedEnd = true;
                    break;
                }
                State from = qe.state;
                ObservationWithCandidateStates timeStep = timeSteps.get(qe.timeStep);
                ObservationWithCandidateStates nextTimeStep = timeSteps.get(qe.timeStep + 1);
                final double linearDistance = distanceCalc.calcDist(timeStep.observation.getPoint().lat, timeStep.observation.getPoint().lon,
                        nextTimeStep.observation.getPoint().lat, nextTimeStep.observation.getPoint().lon)
                        + nextTimeStep.observation.getAccumulatedLinearDistanceToPrevious();
                int fromNode = from.getSnap().getClosestNode();
                int fromOutEdge = from.isOnDirectedEdge() ? from.getOutgoingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE;
                int[] toNodes = nextTimeStep.candidates.stream().mapToInt(c -> c.getSnap().getClosestNode()).toArray();
                int[] toInEdges = nextTimeStep.candidates.stream().mapToInt(to -> to.isOnDirectedEdge() ? to.getIncomingVirtualEdge().getEdge() : EdgeIterator.ANY_EDGE).toArray();
                List<Path> paths = router.calcPaths(queryGraph, fromNode, fromOutEdge, toNodes, toInEdges);
                for (int i = 0; i < nextTimeStep.candidates.size(); i++) {
                    State to = nextTimeStep.candidates.get(i);
                    Path path = paths.get(i);
                    if (path.isFound()) {
                        double transitionLogProbability = probabilities.transitionLogProbability(path.getDistance(), linearDistance);
                        Transition<State> transition = new Transition<>(from, to);
                        roadPaths.put(transition, path);
                        double minusLogProbability = qe.minusLogProbability - emissionLog(probabilities, sigmaPerTimeStep, qe.timeStep + 1, to.getSnap().getQueryDistance()) + penaltyOf(emPenalty, to) - transitionLogProbability;
                        Label label1 = labels.get(to);
                        if (label1 == null || minusLogProbability < label1.minusLogProbability) {
                            q.stream().filter(oldQe -> !oldQe.isDeleted && oldQe.state == to).findFirst().ifPresent(oldQe -> oldQe.isDeleted = true);
                            Label label = new Label();
                            label.state = to;
                            label.timeStep = qe.timeStep + 1;
                            label.back = qe;
                            label.minusLogProbability = minusLogProbability;
                            q.add(label);
                            labels.put(to, label);
                            if (debugCosts != null) {
                                GHPoint p = to.getEntry().getPoint();
                                debugCosts.put(to, new CandidateCost(p.lat, p.lon,
                                        to.getSnap().getClosestEdge().getEdge(),
                                        to.getSnap().getQueryDistance(),
                                        -emissionLog(probabilities, sigmaPerTimeStep, qe.timeStep + 1, to.getSnap().getQueryDistance()),
                                        -transitionLogProbability, path.getDistance(),
                                        minusLogProbability));
                            }
                        }
                    }
                }
            }

            // End of this run: the frontier reached the last timestep, or stalled at its deepest
            // reachable timestep. Back-trace it into a run and record its per-timestep states.
            Label terminal = reachedEnd ? qe : deepestTerminal(labels);
            ArrayList<SequenceState<State, Observation, Path>> run = new ArrayList<>();
            for (Label t = terminal; t != null; t = t.back) {
                final SequenceState<State, Observation, Path> ss = new SequenceState<>(t.state, t.state.getEntry(),
                        t.back == null ? null : roadPaths.get(new Transition<>(t.back.state, t.state)));
                run.add(ss);
                stateByTimeStep.set(t.timeStep, ss);
            }
            Collections.reverse(run);
            runs.add(run);

            int reached = terminal.timeStep;
            if (reached >= T - 1) break;
            // Reseed after the gap. firstSeedableTimeStep skips no-candidate timesteps (mode A);
            // when the very next timestep has candidates but was unreachable (disconnection),
            // the next run starts there (adjacent runs, zero interior gap).
            start = firstSeedableTimeStep(timeSteps, reached + 1);
        }

        if (debugCosts != null) {
            statistics.put("candidateCosts", new ArrayList<>(debugCosts.values()));
        }
        return new ViterbiResult(runs, stateByTimeStep);
    }

    /**
     * Phase 2 (M2a): per-candidate desirability penalty in HMM cost units, keyed by State.
     * Returns {@code null} when M2a is off (canonical, profile-blind emission).
     *
     * <p>For each observation's candidates, find the lowest routing weight-per-metre
     * ({@code wpm_min}, the most rideable nearby way) and penalise the others by
     * {@code lambda * max(0, ln(wpm / wpm_min) - ln(deadband))}. The most-desirable way pays
     * nothing; ways within {@code deadband}× of it pay nothing; clearly worse ways pay in
     * proportion to how much worse they are per the routing profile. The penalty is added to
     * the candidate's emission cost in {@link #computeViterbiSequence}.
     */
    private Map<State, Double> computeDesirabilityPenalties(List<ObservationWithCandidateStates> timeSteps) {
        if (!config.isEmissionDesirabilityOn()) {
            return null;
        }
        Weighting weighting = router.getWeighting();
        double lambda = config.emissionDesirabilityLambda;
        double lnDeadband = Math.log(Math.max(1.0, config.emissionDesirabilityDeadband));
        Map<State, Double> penalties = new HashMap<>();
        for (ObservationWithCandidateStates ts : timeSteps) {
            // weight/m per candidate, and the minimum across this observation's candidates.
            Map<State, Double> wpm = new HashMap<>();
            double wpmMin = Double.POSITIVE_INFINITY;
            for (State c : ts.candidates) {
                double w = weightPerMeter(weighting, c);
                wpm.put(c, w);
                if (!Double.isNaN(w) && !Double.isInfinite(w) && w < wpmMin) {
                    wpmMin = w;
                }
            }
            for (State c : ts.candidates) {
                double w = wpm.get(c);
                double pen = 0.0;
                if (!Double.isNaN(w) && !Double.isInfinite(w) && wpmMin > 0 && !Double.isInfinite(wpmMin)) {
                    pen = lambda * Math.max(0.0, Math.log(w / wpmMin) - lnDeadband);
                }
                penalties.put(c, pen);
            }
        }
        return penalties;
    }

    /** Routing weight per metre of a candidate's snap edge (the real graph edge), forward
     *  direction. NaN/Infinity if the edge is zero-length or inaccessible. */
    private static double weightPerMeter(Weighting weighting, State candidate) {
        EdgeIteratorState edge = candidate.getSnap().getClosestEdge();
        double dist = edge.getDistance();
        if (dist <= 0) {
            return Double.NaN;
        }
        return weighting.calcEdgeWeight(edge, false) / dist;
    }

    private static double penaltyOf(Map<State, Double> penalties, State s) {
        if (penalties == null) {
            return 0.0;
        }
        Double p = penalties.get(s);
        return p == null ? 0.0 : p;
    }

    private List<EdgeMatch> prepareEdgeMatches(List<SequenceState<State, Observation, Path>> seq) {
        // This creates a list of directed edges (EdgeIteratorState instances turned the right way),
        // each associated with 0 or more of the observations.
        // These directed edges are edges of the real street graph, where nodes are intersections.
        // So in _this_ representation, the path that you get when you just look at the edges goes from
        // an intersection to an intersection.

        // Implementation note: We have to look at both states _and_ transitions, since we can have e.g. just one state,
        // or two states with a transition that is an empty path (observations snapped to the same node in the query graph),
        // but these states still happen on an edge, and for this representation, we want to have that edge.
        // (Whereas in the ResponsePath representation, we would just see an empty path.)

        // Note that the result can be empty, even when the input is not. Observations can be on nodes as well as on
        // edges, and when all observations are on the same node, we get no edge at all.
        // But apart from that corner case, all observations that go in here are also in the result.

        // (Consider totally forbidding candidate states to be snapped to a point, and make them all be on directed
        // edges, then that corner case goes away.)
        List<EdgeMatch> edgeMatches = new ArrayList<>();
        List<State> states = new ArrayList<>();
        EdgeIteratorState currentDirectedRealEdge = null;
        for (SequenceState<State, Observation, Path> transitionAndState : seq) {
            // transition (except before the first state)
            if (transitionAndState.transitionDescriptor != null) {
                for (EdgeIteratorState edge : transitionAndState.transitionDescriptor.calcEdges()) {
                    EdgeIteratorState newDirectedRealEdge = resolveToRealEdge(edge);
                    if (currentDirectedRealEdge != null) {
                        if (!equalEdges(currentDirectedRealEdge, newDirectedRealEdge)) {
                            EdgeMatch edgeMatch = new EdgeMatch(currentDirectedRealEdge, states);
                            edgeMatches.add(edgeMatch);
                            states = new ArrayList<>();
                        }
                    }
                    currentDirectedRealEdge = newDirectedRealEdge;
                }
            }
            // state
            if (transitionAndState.state.isOnDirectedEdge()) { // as opposed to on a node
                EdgeIteratorState newDirectedRealEdge = resolveToRealEdge(transitionAndState.state.getOutgoingVirtualEdge());
                if (currentDirectedRealEdge != null) {
                    if (!equalEdges(currentDirectedRealEdge, newDirectedRealEdge)) {
                        EdgeMatch edgeMatch = new EdgeMatch(currentDirectedRealEdge, states);
                        edgeMatches.add(edgeMatch);
                        states = new ArrayList<>();
                    }
                }
                currentDirectedRealEdge = newDirectedRealEdge;
            }
            states.add(transitionAndState.state);
        }
        if (currentDirectedRealEdge != null) {
            EdgeMatch edgeMatch = new EdgeMatch(currentDirectedRealEdge, states);
            edgeMatches.add(edgeMatch);
        }
        return edgeMatches;
    }

    private double gpxLength(List<Observation> gpxList) {
        if (gpxList.isEmpty()) {
            return 0;
        } else {
            double gpxLength = 0;
            Observation prevEntry = gpxList.get(0);
            for (int i = 1; i < gpxList.size(); i++) {
                Observation entry = gpxList.get(i);
                gpxLength += distanceCalc.calcDist(prevEntry.getPoint().lat, prevEntry.getPoint().lon, entry.getPoint().lat, entry.getPoint().lon);
                prevEntry = entry;
            }
            return gpxLength;
        }
    }

    private boolean equalEdges(EdgeIteratorState edge1, EdgeIteratorState edge2) {
        return edge1.getEdge() == edge2.getEdge()
                && edge1.getBaseNode() == edge2.getBaseNode()
                && edge1.getAdjNode() == edge2.getAdjNode();
    }

    private EdgeIteratorState resolveToRealEdge(EdgeIteratorState edgeIteratorState) {
        if (queryGraph.isVirtualNode(edgeIteratorState.getBaseNode()) || queryGraph.isVirtualNode(edgeIteratorState.getAdjNode())) {
            return graph.getEdgeIteratorStateForKey(((VirtualEdgeIteratorState) edgeIteratorState).getOriginalEdgeKey());
        } else {
            return edgeIteratorState;
        }
    }

    public Map<String, Object> getStatistics() {
        return statistics;
    }

    private static class MapMatchedPath extends Path {
        MapMatchedPath(Graph graph, Weighting weighting, List<EdgeIteratorState> edges) {
            super(graph);
            int prevEdge = EdgeIterator.NO_EDGE;
            for (EdgeIteratorState edge : edges) {
                addDistance(edge.getDistance());
                addTime(GHUtility.calcMillisWithTurnMillis(weighting, edge, false, prevEdge));
                addEdge(edge.getEdge());
                prevEdge = edge.getEdge();
            }
            if (edges.isEmpty()) {
                setFound(false);
            } else {
                setFromNode(edges.get(0).getBaseNode());
                setFound(true);
            }
        }
    }
}
