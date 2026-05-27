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
}
