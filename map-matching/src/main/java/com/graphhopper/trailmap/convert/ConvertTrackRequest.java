package com.graphhopper.trailmap.convert;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.graphhopper.util.CustomModel;

import java.util.List;

/**
 * Request DTO for the Trailmap track-to-route conversion endpoint.
 *
 * <p>Input is a GPS track (raw coordinate pairs) plus a routing profile.
 * Output is an editable route: a list of Waypoints and a list of Segments
 * referencing them by id.
 *
 * <p>See {@code docs/gh_convert_track_design.md} for the full design.
 */
public class ConvertTrackRequest {

    /** Raw track as an ordered list of [lat, lng] pairs. */
    @JsonProperty("track")
    private List<double[]> track;

    /** Routing profile name (must exist in server config). Same as {@code /route}.
     *  Used for: the optimizer's GH /route probes and the rendered output the client
     *  will draw. Sections whose matched edges aren't routable under this profile become
     *  coordinates segments. */
    @JsonProperty("profile")
    private String profile;

    /** Optional separate profile used ONLY by the map matcher. When set, MapMatching uses
     *  this profile's weighting for snap-candidate filtering and HMM transition costs,
     *  while routing/optimization continues to use {@link #profile}. Default (null) means
     *  the routing profile is used for matching too — the original behavior.
     *
     *  <p>Useful when the routing profile is restrictive (e.g. {@code gravel} avoiding
     *  small paths) but the user's GPX legitimately follows OSM ways that profile would
     *  penalize. Setting {@code matching_profile = "trailmap_foot"} lets the matcher
     *  follow the user's path while routing still respects the user's preferred profile.
     *  Sections whose matched edges the routing profile blocks entirely still fall back
     *  to coordinates segments. */
    @JsonProperty("matching_profile")
    private String matchingProfile;

    /** Optional. Same shape as {@code /route} customModel. Merged with profile's default. */
    @JsonProperty("custom_model")
    private CustomModel customModel;

    /** Measurement error sigma for {@link com.graphhopper.matching.MapMatching}. Defaults to 20m. */
    @JsonProperty("gps_accuracy_m")
    private Double gpsAccuracyM;

    /** Snap distance above which an observation is considered unmatched. Defaults to 35m. */
    @JsonProperty("snap_threshold_m")
    private Double snapThresholdM;

    /** Routed regions shorter than this are demoted to coordinates segments. Defaults to 40m. */
    @JsonProperty("min_routed_segment_m")
    private Double minRoutedSegmentM;

    /** Detour gate: minimum length (m) for the detour signal to fire. Defaults to 75m. */
    @JsonProperty("min_detour_m")
    private Double minDetourM;

    /** Detour gate: route/straight-line ratio above which a transition is a detour. Defaults to 2.0. */
    @JsonProperty("max_detour_ratio")
    private Double maxDetourRatio;

    /** RDP epsilon (degrees) for simplifying coordinate-segment geometry. Defaults to 0.00003. */
    @JsonProperty("coordinates_simplify_eps")
    private Double coordinatesSimplifyEps;

    /** When true, response includes a {@code debug} object exposing the matcher polyline,
     *  per-observation tracepoints, and detour decisions. Off by default. See API doc §4. */
    @JsonProperty("debug")
    private Boolean debug;

    // ---------------------------------------------------------------------
    // Trailmap custom matcher (experimental). All fields below are inert unless
    // custom_matcher == true, in which case the request is matched by
    // com.graphhopper.trailmap.matching.TrailmapMapMatching instead of stock GH.
    // See docs/gh_map_matcher_learnings.md and MatcherConfig.
    // ---------------------------------------------------------------------

    /** Master switch. When false (default), the existing (lightly-modified GH) matcher is
     *  used. When true, the forked Trailmap matcher is used with the cm_* parameters below
     *  (each defaulting to canonical/off, so {@code custom_matcher=true} alone reproduces
     *  stock GH behaviour — Phase 0). */
    @JsonProperty("custom_matcher")
    private Boolean customMatcher;

    /** Phase 1 (M1): candidate search radius = mult × sigma. Null = canonical expand-by-sigma. */
    @JsonProperty("cm_candidate_radius_sigma_mult")
    private Double cmCandidateRadiusSigmaMult;

    /** Phase 1 (M1): lower floor [m] for the decoupled candidate radius. */
    @JsonProperty("cm_candidate_radius_min_m")
    private Double cmCandidateRadiusMinM;

    /** Phase 1 (M1): upper cap [m] for the decoupled candidate radius. */
    @JsonProperty("cm_candidate_radius_max_m")
    private Double cmCandidateRadiusMaxM;

    /** Phase 6 (P3): enable two-pass sigma auto-estimation. */
    @JsonProperty("cm_auto_sigma")
    private Boolean cmAutoSigma;

    /** Phase 6 (P3): sigma [m] for the probe pass. */
    @JsonProperty("cm_auto_sigma_seed_m")
    private Double cmAutoSigmaSeedM;

    /** Phase 6 (P3): probe snaps above this [m] are off-grid and excluded from the estimate. */
    @JsonProperty("cm_auto_sigma_outlier_m")
    private Double cmAutoSigmaOutlierM;

    /** Phase 6 (P3): lower clamp [m] for the estimated sigma. */
    @JsonProperty("cm_auto_sigma_min_m")
    private Double cmAutoSigmaMinM;

    /** Phase 6 (P3): upper clamp [m] for the estimated sigma. */
    @JsonProperty("cm_auto_sigma_max_m")
    private Double cmAutoSigmaMaxM;

    /** Phase 6 (P3): estimatedSigma = scale × robustUpperEstimate(snaps). */
    @JsonProperty("cm_auto_sigma_scale")
    private Double cmAutoSigmaScale;

    /** Phase 6 (P3): percentile (0..1) of snap distances used as the robust upper sigma
     *  estimate. Higher = looser threshold (passes corner cuts on hard-simplified routes but
     *  risks letting shortcuts back in). Default 0.9. */
    @JsonProperty("cm_auto_sigma_percentile")
    private Double cmAutoSigmaPercentile;

    /** Phase 5 (P1): max gap [m] between consecutive observations; densify above it. Null = off. */
    @JsonProperty("cm_densify_max_gap_m")
    private Double cmDensifyMaxGapM;

    /** Optional transition beta override for the custom matcher. Null = config default (2.0). */
    @JsonProperty("cm_beta")
    private Double cmBeta;

    /** Phase 2 (M2a): profile-aware emission penalty strength. Null/&le;0 = off (profile-blind). */
    @JsonProperty("cm_emission_desirability_lambda")
    private Double cmEmissionDesirabilityLambda;

    /** Phase 2 (M2a): deadband ratio — ways within this multiple of the best nearby weight/m
     *  pay no penalty. Default 1.5. */
    @JsonProperty("cm_emission_desirability_deadband")
    private Double cmEmissionDesirabilityDeadband;

    /** Segmentation v2 (§8): joint emission+transition coords classification with aggressive
     *  outward expansion. When true, {@link RegionSegmenter} uses the parallel v2 classifier
     *  instead of the original Stage A–D path. Independent of {@code custom_matcher} — it only
     *  affects the segmentation stage (works on either matcher's result). Off by default. */
    @JsonProperty("cm_segmentation_v2")
    private Boolean cmSegmentationV2;

    /** Optimizer waypoint candidates restricted to kept (Viterbi) observations (those on the
     *  matched path), dropping interior filtered obs. Default ON (null = on). Send false to
     *  restore the old behaviour (all observations as candidates). Affects the optimizer stage. */
    @JsonProperty("cm_optimizer_kept_obs_only")
    private Boolean cmOptimizerKeptObsOnly;

    /** Adaptive per-observation σ (V2). When true, the matcher derives a per-obs σ from a cheap
     *  snap-only pre-pass and uses it for candidate radius + emission (filter/segmenter use a
     *  representative capped σ). Replaces the auto-sigma probe-pass Viterbi. Default off. */
    @JsonProperty("cm_adaptive_sigma")
    private Boolean cmAdaptiveSigma;

    /** Outlier-condition the per-obs adaptive σ window (drop off-grid snaps so σ tracks on-network
     *  noise, not off-road distance). Only relevant when {@code cm_adaptive_sigma} is on.
     *  Default ON (null = on). Send false to restore the un-filtered windowed σ. */
    @JsonProperty("cm_adaptive_sigma_outlier_filter")
    private Boolean cmAdaptiveSigmaOutlierFilter;

    /** Optimizer twin-edge tolerance: rescue a leg from coords demotion when the only
     *  matcher-vs-/route difference is a coincident parallel edge over the same node pair
     *  (e.g. cycleway + footway over the same stripe). Default ON (null = on). Send false to A/B. */
    @JsonProperty("cm_optimizer_twin_edge_tolerance")
    private Boolean cmOptimizerTwinEdgeTolerance;

    public List<double[]> getTrack() { return track; }
    public void setTrack(List<double[]> track) { this.track = track; }

    public String getProfile() { return profile; }
    public void setProfile(String profile) { this.profile = profile; }

    public String getMatchingProfile() { return matchingProfile; }
    public void setMatchingProfile(String matchingProfile) { this.matchingProfile = matchingProfile; }

    public CustomModel getCustomModel() { return customModel; }
    public void setCustomModel(CustomModel customModel) { this.customModel = customModel; }

    public Double getGpsAccuracyM() { return gpsAccuracyM; }
    public void setGpsAccuracyM(Double gpsAccuracyM) { this.gpsAccuracyM = gpsAccuracyM; }

    public Double getSnapThresholdM() { return snapThresholdM; }
    public void setSnapThresholdM(Double snapThresholdM) { this.snapThresholdM = snapThresholdM; }

    public Double getMinRoutedSegmentM() { return minRoutedSegmentM; }
    public void setMinRoutedSegmentM(Double minRoutedSegmentM) { this.minRoutedSegmentM = minRoutedSegmentM; }

    public Double getMinDetourM() { return minDetourM; }
    public void setMinDetourM(Double minDetourM) { this.minDetourM = minDetourM; }

    public Double getMaxDetourRatio() { return maxDetourRatio; }
    public void setMaxDetourRatio(Double maxDetourRatio) { this.maxDetourRatio = maxDetourRatio; }

    public Double getCoordinatesSimplifyEps() { return coordinatesSimplifyEps; }
    public void setCoordinatesSimplifyEps(Double coordinatesSimplifyEps) { this.coordinatesSimplifyEps = coordinatesSimplifyEps; }

    public Boolean getDebug() { return debug; }
    public void setDebug(Boolean debug) { this.debug = debug; }

    public Boolean getCustomMatcher() { return customMatcher; }
    public void setCustomMatcher(Boolean customMatcher) { this.customMatcher = customMatcher; }

    public Double getCmCandidateRadiusSigmaMult() { return cmCandidateRadiusSigmaMult; }
    public void setCmCandidateRadiusSigmaMult(Double v) { this.cmCandidateRadiusSigmaMult = v; }

    public Double getCmCandidateRadiusMinM() { return cmCandidateRadiusMinM; }
    public void setCmCandidateRadiusMinM(Double v) { this.cmCandidateRadiusMinM = v; }

    public Double getCmCandidateRadiusMaxM() { return cmCandidateRadiusMaxM; }
    public void setCmCandidateRadiusMaxM(Double v) { this.cmCandidateRadiusMaxM = v; }

    public Boolean getCmAutoSigma() { return cmAutoSigma; }
    public void setCmAutoSigma(Boolean v) { this.cmAutoSigma = v; }

    public Double getCmAutoSigmaSeedM() { return cmAutoSigmaSeedM; }
    public void setCmAutoSigmaSeedM(Double v) { this.cmAutoSigmaSeedM = v; }

    public Double getCmAutoSigmaOutlierM() { return cmAutoSigmaOutlierM; }
    public void setCmAutoSigmaOutlierM(Double v) { this.cmAutoSigmaOutlierM = v; }

    public Double getCmAutoSigmaMinM() { return cmAutoSigmaMinM; }
    public void setCmAutoSigmaMinM(Double v) { this.cmAutoSigmaMinM = v; }

    public Double getCmAutoSigmaMaxM() { return cmAutoSigmaMaxM; }
    public void setCmAutoSigmaMaxM(Double v) { this.cmAutoSigmaMaxM = v; }

    public Double getCmAutoSigmaScale() { return cmAutoSigmaScale; }
    public void setCmAutoSigmaScale(Double v) { this.cmAutoSigmaScale = v; }

    public Double getCmAutoSigmaPercentile() { return cmAutoSigmaPercentile; }
    public void setCmAutoSigmaPercentile(Double v) { this.cmAutoSigmaPercentile = v; }

    public Double getCmDensifyMaxGapM() { return cmDensifyMaxGapM; }
    public void setCmDensifyMaxGapM(Double v) { this.cmDensifyMaxGapM = v; }

    public Double getCmBeta() { return cmBeta; }
    public void setCmBeta(Double v) { this.cmBeta = v; }

    public Double getCmEmissionDesirabilityLambda() { return cmEmissionDesirabilityLambda; }
    public void setCmEmissionDesirabilityLambda(Double v) { this.cmEmissionDesirabilityLambda = v; }

    public Double getCmEmissionDesirabilityDeadband() { return cmEmissionDesirabilityDeadband; }
    public void setCmEmissionDesirabilityDeadband(Double v) { this.cmEmissionDesirabilityDeadband = v; }

    public Boolean getCmSegmentationV2() { return cmSegmentationV2; }
    public void setCmSegmentationV2(Boolean v) { this.cmSegmentationV2 = v; }

    public Boolean getCmOptimizerKeptObsOnly() { return cmOptimizerKeptObsOnly; }
    public void setCmOptimizerKeptObsOnly(Boolean v) { this.cmOptimizerKeptObsOnly = v; }

    public Boolean getCmAdaptiveSigma() { return cmAdaptiveSigma; }
    public void setCmAdaptiveSigma(Boolean v) { this.cmAdaptiveSigma = v; }

    public Boolean getCmAdaptiveSigmaOutlierFilter() { return cmAdaptiveSigmaOutlierFilter; }
    public void setCmAdaptiveSigmaOutlierFilter(Boolean v) { this.cmAdaptiveSigmaOutlierFilter = v; }

    public Boolean getCmOptimizerTwinEdgeTolerance() { return cmOptimizerTwinEdgeTolerance; }
    public void setCmOptimizerTwinEdgeTolerance(Boolean v) { this.cmOptimizerTwinEdgeTolerance = v; }
}
