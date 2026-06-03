package com.graphhopper.trailmap.matching;

/**
 * Tunables for the Trailmap map matcher ({@link TrailmapMapMatching}).
 *
 * <p>This is the single home for every experimental lever of the Trailmap matcher
 * project (see {@code docs/gh_map_matcher_learnings.md}). It is deliberately a plain
 * mutable POJO with a small fluent API so that:
 * <ul>
 *   <li>experiments are <b>config swaps, not code rewrites</b> — the matcher's hot loop
 *       stays stable while we vary parameters;</li>
 *   <li>the <b>default-constructed config reproduces stock GraphHopper exactly</b>
 *       (Phase 0 baseline). Every field below documents what its default means and what
 *       turning it on changes.</li>
 * </ul>
 *
 * <p><b>Phase coverage in this revision:</b> Phase 0 (faithful fork), Phase 1 (M1 —
 * candidate-radius decoupling), Phase 5 (P1 — densification), Phase 6 (P3 — sigma
 * auto-estimation). The emission/transition profile-awareness levers (M2a/b/c) are NOT
 * here yet — they will be added when those phases are implemented.
 */
public class MatcherConfig {

    // ---------------------------------------------------------------------
    // Core HMM parameters (Phase 0). Defaults == stock GraphHopper.
    // ---------------------------------------------------------------------

    /** Standard deviation [m] of the Gaussian emission model (GPS error). Stock GH default
     *  is 10.0. In stock GH this number ALSO sets the candidate search radius; Phase 1
     *  (see {@link #candidateRadiusSigmaMult}) lets the radius be decoupled so that this
     *  value governs emission sharpness only. */
    public double measurementErrorSigma = 10.0;

    /** Beta of the exponential transition model. Stock GH default is 2.0. */
    public double transitionProbabilityBeta = 2.0;

    // ---------------------------------------------------------------------
    // Phase 1 (M1): decouple candidate search radius from sigma.
    // ---------------------------------------------------------------------

    /**
     * Multiplier applied to {@link #measurementErrorSigma} to form the candidate search
     * radius. {@code null} (default) selects the canonical GH behaviour: an
     * expand-by-sigma ring loop that returns the FIRST non-empty ring (effective radius
     * ≈ sigma, candidate set under-populated near boundaries).
     *
     * <p>When set (e.g. 2.0 or 3.0), the matcher instead searches a single decoupled
     * radius {@code clamp(mult * sigma, candidateRadiusMinM, candidateRadiusMaxM)} and
     * collects <b>all</b> candidates within it (not just the first non-empty ring),
     * guaranteeing coverage independently of emission sharpness. See learnings doc §6.1.
     */
    public Double candidateRadiusSigmaMult = null;

    /** Lower floor [m] for the decoupled candidate radius (intercept term, OSRM-shape).
     *  Only used when {@link #candidateRadiusSigmaMult} is set. */
    public double candidateRadiusMinM = 0.0;

    /** Upper cap [m] for the decoupled candidate radius. Only used when
     *  {@link #candidateRadiusSigmaMult} is set. Default mirrors the paper/OSRM 200 m cap. */
    public double candidateRadiusMaxM = 200.0;

    // ---------------------------------------------------------------------
    // Phase 6 (P3): sigma auto-estimation (two-pass).
    // ---------------------------------------------------------------------

    /**
     * When true, the matcher first runs a probe pass at {@link #autoSigmaSeedM}, drops
     * off-grid snap outliers (snap distance &gt; {@link #autoSigmaOutlierM}), computes a
     * robust average of the remaining snap distances, clamps it to
     * [{@link #autoSigmaMinM}, {@link #autoSigmaMaxM}], scales it by
     * {@link #autoSigmaScale}, and re-runs the match at the estimated sigma.
     *
     * <p>This replaces the client/user having to declare "accurate" vs "inaccurate" GPX:
     * the track's own snap-distance distribution decides sigma. v1 heuristic — tune on the
     * corpus. Default off.
     */
    public boolean autoSigma = false;

    /** Sigma [m] used for the probe pass when {@link #autoSigma} is on. */
    public double autoSigmaSeedM = 20.0;

    /** Probe-pass snap distances above this [m] are treated as off-grid (off-network)
     *  sections and excluded from the sigma estimate. */
    public double autoSigmaOutlierM = 40.0;

    /** Lower clamp [m] for the estimated sigma. */
    public double autoSigmaMinM = 4.0;

    /** Upper clamp [m] for the estimated sigma. */
    public double autoSigmaMaxM = 30.0;

    /** estimatedSigma = autoSigmaScale * upperEstimate(filtered snap distances). */
    public double autoSigmaScale = 1.0;

    /**
     * Percentile (0..1) of the filtered snap distances used as the sigma estimate — a
     * ROBUST UPPER estimate rather than the mean. The mean is dominated by the many tiny
     * snaps on straight sections of a hard-simplified route and lands at the floor, making
     * the σ-derived snap threshold too tight for legitimate corner-cut deviations (sbc_74).
     * A high percentile (e.g. 0.9) reflects the track's real deviation scale — corner cuts
     * pass, while genuinely off-grid sections (snaps &gt; {@link #autoSigmaOutlierM}, excluded)
     * are still caught. Tune on the corpus.
     */
    public double autoSigmaPercentile = 0.9;

    /**
     * Detour-aware σ estimation: exclude observations whose incoming HMM transition (in the
     * probe pass) is a detour — i.e. the matcher's matched-path is more than
     * {@link #autoSigmaDetourRatio}× the straight-line AND longer than
     * {@link #autoSigmaDetourMinM}. Such observations are OFF the network (the user genuinely
     * left it — snow-shoe excursions, off-trail walking); their large snap distances reflect
     * off-road distance, NOT GPS error, and would otherwise inflate σ toward the cap and loosen
     * the segmenter's threshold. Estimating σ from only the on-road observations distinguishes
     * an off-road track (low σ → off-road becomes coords) from a noisy on-road GPX (transitions
     * stay ≈ straight-line → nothing excluded → σ reflects the real noise). No extra matcher
     * pass — the probe pass already provides the per-transition distances.
     */
    public double autoSigmaDetourRatio = 2.0;

    /** Minimum matched-path length [m] for the detour-aware exclusion to apply (guards against
     *  flagging tiny wiggles). Mirrors the segmenter's min-detour gate. */
    public double autoSigmaDetourMinM = 50.0;

    // ---------------------------------------------------------------------
    // Phase 5 (P1): densification. NOTE: densification is applied by the
    // *caller* (TrailmapConvertResource) to the observation list BEFORE it is
    // handed to both the matcher and the converter, so observation indices stay
    // 1:1 across the pipeline. The parameter lives here for cohesion; the matcher
    // itself does not read it.
    // ---------------------------------------------------------------------

    /** Maximum allowed gap [m] between consecutive observations. {@code null} (default)
     *  disables densification. When set, the caller inserts linearly-interpolated
     *  observations so no consecutive pair exceeds this distance — giving the matcher and
     *  the downstream {@code /route} fit more anchoring on long simplify.js-collapsed
     *  straight runs. See {@link ObservationDensifier}. */
    public Double densifyMaxGapM = null;

    // ---------------------------------------------------------------------
    // Phase 2 (M2a): profile-aware emission (desirability prior).
    //
    // Adds a penalty to a candidate's emission cost when it snaps onto a way that
    // the routing profile considers much less desirable than the best nearby way:
    //
    //   penalty = lambda * max(0, ln(wpm / wpm_min) - ln(deadband))
    //
    // where wpm = the candidate edge's routing weight per metre and wpm_min = the
    // lowest weight/m among that observation's candidates (the most rideable nearby
    // way pays nothing). Ways within `deadband`× of the best pay nothing; clearly
    // worse ways (e.g. a gravel_scale=10 path or a platform, 4-19× worse) are
    // penalised in proportion to ln(ratio). Profile-general: under a foot profile
    // weights are nearly flat so the penalty self-limits, which is why this is a
    // legitimate prior for bike but inert for foot (see learnings doc §5.6).
    // ---------------------------------------------------------------------

    /** Strength of the desirability penalty (in HMM cost / log-prob units). {@code null}
     *  or {@code <= 0} disables M2a (canonical, profile-blind emission). */
    public Double emissionDesirabilityLambda = null;

    /** Deadband ratio: candidates whose weight/m is within this multiple of the best
     *  nearby weight/m incur no penalty. Keeps near-equivalent good ways unpenalised
     *  (tiered scoring, not a cliff). Must be {@code >= 1.0}. */
    public double emissionDesirabilityDeadband = 1.5;

    // ---------------------------------------------------------------------
    // Adaptive per-observation sigma (V2 — validation of the moving-average σ idea).
    //
    // When on, the matcher runs a cheap SNAP-ONLY pre-pass (nearest snap per obs, no Viterbi —
    // also replaces the expensive auto-sigma probe-pass Viterbi), derives a per-observation σ
    // from a smoothed/clamped windowed estimate of the nearest-snap distances, and uses that
    // per-obs σ for the candidate RADIUS and the EMISSION term in the final Viterbi. The filter
    // and the segmenter keep a single REPRESENTATIVE capped σ (reported as autoSigmaEstimatedM),
    // so this lever isolates the matcher's emission/radius discrimination change.
    //
    // Default off → the global auto-sigma / single-σ paths are unchanged.
    // ---------------------------------------------------------------------

    /** Master switch for the adaptive per-observation σ path. Default off. */
    public boolean adaptiveSigma = false;

    /** Half-width [observations] of the moving window used to estimate the local σ. */
    public int adaptiveSigmaWindow = 15;

    /** Percentile (0..1) of the windowed nearest-snap distances used as the per-obs σ. Above the
     *  noise floor (median) so legitimate GPS wander isn't over-sharpened; below the spikes. */
    public double adaptiveSigmaPercentile = 0.6;

    /** Lower clamp [m] for the per-obs σ (avoid over-sharp emission on near-perfect points). */
    public double adaptiveSigmaMinM = 3.0;

    /** Upper clamp [m] for the per-obs σ (controlled growth — off-road stretches must not inflate
     *  σ without bound; see V1 analysis / sorel). */
    public double adaptiveSigmaMaxM = 20.0;

    /** Upper clamp [m] for the REPRESENTATIVE σ reported to the filter + segmenter (the absolute
     *  on/off-network threshold must NOT follow off-road inflation). */
    public double adaptiveSigmaReprMaxM = 10.0;

    /** When true (default), the per-observation σ window drops off-grid snap outliers
     *  (snap &gt; {@link #autoSigmaOutlierM}) before taking the percentile, so σ reflects
     *  on-network GPS noise rather than off-road excursion distance; windows with no on-network
     *  snap fall back to the representative σ. Without this, σ inflates toward {@link #adaptiveSigmaMaxM}
     *  through off-road stretches, widening the candidate radius enough to admit the off-road
     *  detour edge — transition continuity then glues boundary observations to it, shifting the
     *  coords-segment boundary (sorel off-road-tail regression). */
    public boolean adaptiveSigmaOutlierFilter = true;

    /** True when the adaptive per-observation σ path is active. */
    public boolean isAdaptiveSigma() {
        return adaptiveSigma;
    }

    // ---------------------------------------------------------------------
    // Diagnostics (no effect on matching behaviour or output).
    // ---------------------------------------------------------------------

    /** When true, the matcher records, for every candidate at every observation, its
     *  emission cost, the transition cost of its best incoming connection, the connecting
     *  trip length, and its accumulated cost — into the statistics map under
     *  {@code "candidateCosts"} (a list of {@link TrailmapMapMatching.CandidateCost}).
     *  Used to size the M2a desirability penalty against the matcher's real internal costs.
     *  Default off; intended for the test harness, not the request API. */
    public boolean debugCandidateCosts = false;

    public MatcherConfig() {
    }

    // -- Fluent setters (return this) for terse construction in the resource/tests --

    public MatcherConfig sigma(double s) { this.measurementErrorSigma = s; return this; }
    public MatcherConfig beta(double b) { this.transitionProbabilityBeta = b; return this; }

    public MatcherConfig candidateRadius(Double sigmaMult, double minM, double maxM) {
        this.candidateRadiusSigmaMult = sigmaMult;
        this.candidateRadiusMinM = minM;
        this.candidateRadiusMaxM = maxM;
        return this;
    }

    public MatcherConfig densify(Double maxGapM) { this.densifyMaxGapM = maxGapM; return this; }

    /** True when the decoupled candidate-radius search (Phase 1) is active. */
    public boolean isRadiusDecoupled() {
        return candidateRadiusSigmaMult != null;
    }

    /** True when the profile-aware emission penalty (Phase 2 / M2a) is active. */
    public boolean isEmissionDesirabilityOn() {
        return emissionDesirabilityLambda != null && emissionDesirabilityLambda > 0;
    }

    @Override
    public String toString() {
        return "MatcherConfig{sigma=" + measurementErrorSigma + ", beta=" + transitionProbabilityBeta
                + ", candidateRadiusSigmaMult=" + candidateRadiusSigmaMult
                + ", candidateRadiusMinM=" + candidateRadiusMinM
                + ", candidateRadiusMaxM=" + candidateRadiusMaxM
                + ", autoSigma=" + autoSigma
                + (autoSigma ? ", autoSigmaSeedM=" + autoSigmaSeedM + ", autoSigmaOutlierM=" + autoSigmaOutlierM
                        + ", autoSigmaMinM=" + autoSigmaMinM + ", autoSigmaMaxM=" + autoSigmaMaxM
                        + ", autoSigmaScale=" + autoSigmaScale : "")
                + ", densifyMaxGapM=" + densifyMaxGapM
                + ", emissionDesirabilityLambda=" + emissionDesirabilityLambda
                + (isEmissionDesirabilityOn() ? ", emissionDesirabilityDeadband=" + emissionDesirabilityDeadband : "")
                + '}';
    }
}
