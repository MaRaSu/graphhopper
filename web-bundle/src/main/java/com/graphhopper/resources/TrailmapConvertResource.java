package com.graphhopper.resources;

import com.graphhopper.GraphHopper;
import com.graphhopper.http.ProfileResolver;
import com.graphhopper.matching.MapMatching;
import com.graphhopper.matching.MatchResult;
import com.graphhopper.matching.Observation;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.trailmap.convert.ConvertTrackRequest;
import com.graphhopper.trailmap.convert.ConvertTrackResponse;
import com.graphhopper.trailmap.convert.TrackToRouteConverter;
import com.graphhopper.trailmap.matching.MatcherConfig;
import com.graphhopper.trailmap.matching.ObservationDensifier;
import com.graphhopper.trailmap.matching.TrailmapMapMatching;
import com.graphhopper.util.PMap;
import com.graphhopper.util.Parameters;
import com.graphhopper.util.shapes.GHPoint;
import jakarta.inject.Inject;
import jakarta.ws.rs.Consumes;
import jakarta.ws.rs.POST;
import jakarta.ws.rs.Path;
import jakarta.ws.rs.Produces;
import jakarta.ws.rs.core.MediaType;
import jakarta.ws.rs.core.Response;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * JAX-RS endpoint for converting a raw GPS track into an editable Trailmap route.
 *
 * <p>{@code POST /trailmap/convert_track}
 *
 * <p>Pipeline:
 * <ol>
 *   <li>Map-match the input track with {@link MapMatching}</li>
 *   <li>{@link com.graphhopper.trailmap.convert.RegionSegmenter Segment} into matched/unmatched regions</li>
 *   <li>{@link com.graphhopper.trailmap.convert.RoutedRegionOptimizer Optimize} matched regions to a minimal waypoint set</li>
 *   <li>Build coordinates segments for unmatched regions</li>
 *   <li>Emit waypoints + segments referencing them by id</li>
 * </ol>
 */
@Path("trailmap/convert_track")
public class TrailmapConvertResource {

    private static final Logger LOGGER = LoggerFactory.getLogger(TrailmapConvertResource.class);
    /**
     * Default {@code measurementErrorSigma} for the underlying {@link MapMatching} when
     * the request does not specify {@code gps_accuracy_m}. Set to 10m as a reasonable
     * middle ground — clean GPX exports work fine at this value, while still tolerating
     * activity recordings with moderate noise. Clients with very noisy inputs (vehicle
     * GPX in urban canyons) should set a higher value via the request.
     *
     * <p>Note: MapMatching uses sigma for three things — filter threshold ({@code 2 * sigma}),
     * snap search radius, and Gaussian emission probability. Higher sigma drops fewer
     * observations from the HMM and tolerates looser snaps in the emission model.
     */
    private static final double DEFAULT_GPS_ACCURACY_M = 10.0;
    private static final int MAX_TRACK_POINTS = 30_000;
    private static final int MAX_DENSIFIED_POINTS = 35_000;

    private final GraphHopper graphHopper;
    private final ProfileResolver profileResolver;
    private final MapMatchingResource.MapMatchingRouterFactory mapMatchingRouterFactory;

    @Inject
    public TrailmapConvertResource(GraphHopper graphHopper,
                                   ProfileResolver profileResolver,
                                   MapMatchingResource.MapMatchingRouterFactory mapMatchingRouterFactory) {
        this.graphHopper = graphHopper;
        this.profileResolver = profileResolver;
        this.mapMatchingRouterFactory = mapMatchingRouterFactory;
    }

    @POST
    @Consumes(MediaType.APPLICATION_JSON)
    @Produces(MediaType.APPLICATION_JSON)
    public Response convert(ConvertTrackRequest request) {
        long t0 = System.currentTimeMillis();
        try {
            validate(request);

            // Resolve routing profile (used by optimizer's GH /route probes and the
            // rendered output the client will draw). CH is disabled for the resolver since
            // trailmap-config has profiles_ch: [].
            PMap profileResolverHints = new PMap();
            profileResolverHints.putObject("profile", request.getProfile());
            profileResolverHints.putObject(Parameters.CH.DISABLE, true);
            String profile = profileResolver.resolveProfile(profileResolverHints);

            // Resolve MATCHING profile separately. When request.matching_profile is unset,
            // fall back to the routing profile (original behavior). When set, MapMatching
            // uses this profile's weighting for snap filtering and HMM transition costs —
            // useful when the routing profile is restrictive on edges the user's GPX
            // actually traverses (e.g. gravel deprioritizing OSM paths).
            String matchingProfileName = request.getMatchingProfile() != null
                    ? request.getMatchingProfile() : request.getProfile();
            PMap matchingResolverHints = new PMap();
            matchingResolverHints.putObject("profile", matchingProfileName);
            matchingResolverHints.putObject(Parameters.CH.DISABLE, true);
            String matchingProfile = profileResolver.resolveProfile(matchingResolverHints);

            PMap matchingHints = new PMap();
            matchingHints.putObject("profile", matchingProfile);

            double gpsAccuracy = nz(request.getGpsAccuracyM(), DEFAULT_GPS_ACCURACY_M);
            List<Observation> observations = buildObservations(request.getTrack());

            // Choose matcher: the existing (lightly-modified GH) matcher by default, or the
            // forked Trailmap matcher when custom_matcher=true. Both produce a MatchResult
            // with the same contract, so the conversion pipeline below is identical.
            boolean useCustomMatcher = Boolean.TRUE.equals(request.getCustomMatcher());
            MatchResult matchResult;
            long tStart, tMatched;
            // Set when the custom matcher auto-estimated sigma (P3); drives the segmenter's
            // snap threshold + drift floor below.
            Double estimatedSigmaM = null;
            if (useCustomMatcher) {
                MatcherConfig cfg = buildMatcherConfig(request, gpsAccuracy);
                // Phase 5 (P1): densify BEFORE both match and convert so observation indices
                // stay 1:1 across the whole pipeline.
                if (cfg.densifyMaxGapM != null) {
                    observations = ObservationDensifier.densify(observations, cfg.densifyMaxGapM);
                    if (observations.size() > MAX_DENSIFIED_POINTS) {
                        throw new IllegalArgumentException("densified track too large: " + observations.size()
                                + " points, max " + MAX_DENSIFIED_POINTS);
                    }
                }
                TrailmapMapMatching matching = new TrailmapMapMatching(
                        graphHopper.getBaseGraph(),
                        (LocationIndexTree) graphHopper.getLocationIndex(),
                        mapMatchingRouterFactory.createMapMatchingRouter(matchingHints),
                        cfg);
                tStart = System.currentTimeMillis();
                matchResult = matching.match(observations);
                tMatched = System.currentTimeMillis();
                LOGGER.info("convert_track using custom matcher: {}", cfg);
                if (cfg.autoSigma || cfg.adaptiveSigma) {
                    Object est = matching.getStatistics().get("autoSigmaEstimatedM");
                    if (est instanceof Number) estimatedSigmaM = ((Number) est).doubleValue();
                    LOGGER.info("convert_track custom matcher {}: representative σ={}",
                            cfg.adaptiveSigma ? "adaptiveSigma" : "autoSigma", est);
                }
            } else {
                MapMatching matching = new MapMatching(
                        graphHopper.getBaseGraph(),
                        (LocationIndexTree) graphHopper.getLocationIndex(),
                        mapMatchingRouterFactory.createMapMatchingRouter(matchingHints));
                matching.setMeasurementErrorSigma(gpsAccuracy);
                tStart = System.currentTimeMillis();
                matchResult = matching.match(observations);
                tMatched = System.currentTimeMillis();
            }

            // Segmenter thresholds. In auto-sigma mode, derive the snap threshold and
            // drift-trim floor from the estimated sigma (the noise model the matcher just
            // learned) — an accurate track gets a tighter on/off-network gate than a noisy
            // one. An explicit snap_threshold_m in the request still wins. Outside auto-sigma
            // mode, behaviour is unchanged (request value or fixed defaults).
            double snapThreshold;
            double driftFloor;
            if (estimatedSigmaM != null) {
                snapThreshold = request.getSnapThresholdM() != null
                        ? request.getSnapThresholdM()
                        : TrackToRouteConverter.AUTO_SIGMA_SNAP_THRESHOLD_MULT * estimatedSigmaM;
                driftFloor = TrackToRouteConverter.AUTO_SIGMA_DRIFT_FLOOR_MULT * estimatedSigmaM;
                LOGGER.info("convert_track auto-sigma segmenter thresholds: estSigma={} snapThreshold={} driftFloor={}",
                        estimatedSigmaM, snapThreshold, driftFloor);
            } else {
                snapThreshold = nz(request.getSnapThresholdM(), TrackToRouteConverter.DEFAULT_SNAP_THRESHOLD_M);
                driftFloor = TrackToRouteConverter.DEFAULT_DRIFT_FLOOR_M;
            }

            TrackToRouteConverter converter = new TrackToRouteConverter(graphHopper);
            // Segmentation v2 (§8): joint emission+transition coords classification. Independent
            // of custom_matcher — only affects the segmentation stage. Off by default.
            boolean segmentationV2 = Boolean.TRUE.equals(request.getCmSegmentationV2());
            converter.setSegmentationV2Enabled(segmentationV2);
            if (segmentationV2) {
                LOGGER.info("convert_track using segmentation v2 (joint emission+transition)");
            }
            // Optimizer waypoint candidates: kept (Viterbi) obs only. Default ON (param absent).
            boolean keptObsOnly = request.getCmOptimizerKeptObsOnly() == null
                    || Boolean.TRUE.equals(request.getCmOptimizerKeptObsOnly());
            converter.setOptimizerKeptObsOnly(keptObsOnly);
            if (!keptObsOnly) {
                LOGGER.info("convert_track: optimizer kept-obs-only DISABLED (all obs as candidates)");
            }
            // Optimizer twin-edge tolerance (coincident cycleway/footway parallels). Default ON.
            converter.setOptimizerTwinEdgeTolerance(
                    !Boolean.FALSE.equals(request.getCmOptimizerTwinEdgeTolerance()));
            boolean debug = Boolean.TRUE.equals(request.getDebug());
            ConvertTrackResponse response = converter.convert(
                    matchResult,
                    observations,
                    profile,
                    request.getCustomModel(),
                    snapThreshold,
                    nz(request.getMinRoutedSegmentM(), TrackToRouteConverter.DEFAULT_MIN_ROUTED_SEGMENT_M),
                    nz(request.getCoordinatesSimplifyEps(), TrackToRouteConverter.DEFAULT_COORDINATES_SIMPLIFY_EPS_M),
                    nz(request.getMinDetourM(), TrackToRouteConverter.DEFAULT_MIN_DETOUR_M),
                    nz(request.getMaxDetourRatio(), TrackToRouteConverter.DEFAULT_MAX_DETOUR_RATIO),
                    driftFloor,
                    debug);

            long t1 = System.currentTimeMillis();
            // Overwrite the converter's matching_ms with the real match duration
            response.getStats().matchingMs = tMatched - tStart;
            LOGGER.info("convert_track: profile={}, input_points={}, total_ms={}, match_ms={}",
                    profile, observations.size(), t1 - t0, tMatched - tStart);
            return Response.ok(response).build();

        } catch (IllegalArgumentException e) {
            LOGGER.warn("Bad request: {}", e.getMessage());
            return Response.status(Response.Status.BAD_REQUEST)
                    .entity(Map.of("error", e.getMessage())).build();
        } catch (Exception e) {
            LOGGER.error("convert_track failed", e);
            return Response.status(Response.Status.INTERNAL_SERVER_ERROR)
                    .entity(Map.of("error", e.getClass().getSimpleName() + ": " + e.getMessage())).build();
        }
    }

    private void validate(ConvertTrackRequest req) {
        if (req == null) throw new IllegalArgumentException("Empty body");
        if (req.getTrack() == null || req.getTrack().size() < 2) {
            throw new IllegalArgumentException("track must contain at least 2 [lat,lng] entries");
        }
        if (req.getTrack().size() > MAX_TRACK_POINTS) {
            throw new IllegalArgumentException("track too large: " + req.getTrack().size()
                    + " points, max " + MAX_TRACK_POINTS);
        }
        if (req.getProfile() == null || req.getProfile().isEmpty()) {
            throw new IllegalArgumentException("profile is required");
        }
        for (double[] pt : req.getTrack()) {
            if (pt == null || pt.length < 2) {
                throw new IllegalArgumentException("each track entry must be [lat, lng]");
            }
            double lat = pt[0], lng = pt[1];
            if (!Double.isFinite(lat) || !Double.isFinite(lng)) {
                throw new IllegalArgumentException("track coordinates must be finite: [" + lat + ", " + lng + "]");
            }
            if (lat < -90 || lat > 90 || lng < -180 || lng > 180) {
                throw new IllegalArgumentException("track coordinates out of range: [" + lat + ", " + lng + "]");
            }
        }

        checkRange("gps_accuracy_m", req.getGpsAccuracyM(), 1.0, 100.0);
        checkRange("snap_threshold_m", req.getSnapThresholdM(), 1.0, 300.0);
        checkRange("min_routed_segment_m", req.getMinRoutedSegmentM(), 0.0, 1_000.0);
        checkRange("coordinates_simplify_eps", req.getCoordinatesSimplifyEps(), 0.0, 50.0);
        checkRange("min_detour_m", req.getMinDetourM(), 0.0, 1_000.0);
        checkRange("max_detour_ratio", req.getMaxDetourRatio(), 1.0, 20.0);

        checkRange("cm_beta", req.getCmBeta(), 0.1, 50.0);
        checkRange("cm_candidate_radius_sigma_mult", req.getCmCandidateRadiusSigmaMult(), 0.1, 10.0);
        checkRange("cm_candidate_radius_min_m", req.getCmCandidateRadiusMinM(), 0.0, 300.0);
        checkRange("cm_candidate_radius_max_m", req.getCmCandidateRadiusMaxM(), 1.0, 500.0);
        checkOrdered("cm_candidate_radius_min_m", req.getCmCandidateRadiusMinM(),
                "cm_candidate_radius_max_m", req.getCmCandidateRadiusMaxM());

        checkRange("cm_auto_sigma_seed_m", req.getCmAutoSigmaSeedM(), 1.0, 100.0);
        checkRange("cm_auto_sigma_outlier_m", req.getCmAutoSigmaOutlierM(), 1.0, 500.0);
        checkRange("cm_auto_sigma_min_m", req.getCmAutoSigmaMinM(), 1.0, 100.0);
        checkRange("cm_auto_sigma_max_m", req.getCmAutoSigmaMaxM(), 1.0, 100.0);
        checkOrdered("cm_auto_sigma_min_m", req.getCmAutoSigmaMinM(),
                "cm_auto_sigma_max_m", req.getCmAutoSigmaMaxM());
        checkRange("cm_auto_sigma_scale", req.getCmAutoSigmaScale(), 0.1, 10.0);
        checkRange("cm_auto_sigma_percentile", req.getCmAutoSigmaPercentile(), 0.0, 1.0);

        checkRange("cm_densify_max_gap_m", req.getCmDensifyMaxGapM(), 5.0, 1_000.0);
        checkRange("cm_emission_desirability_lambda", req.getCmEmissionDesirabilityLambda(), 0.0, 20.0);
        checkRange("cm_emission_desirability_deadband", req.getCmEmissionDesirabilityDeadband(), 1.0, 20.0);
    }

    private static void checkRange(String name, Double value, double min, double max) {
        if (value == null) return;
        if (!Double.isFinite(value) || value < min || value > max) {
            throw new IllegalArgumentException(name + " must be finite and in range [" + min + ", " + max
                    + "], got " + value);
        }
    }

    private static void checkOrdered(String minName, Double minValue, String maxName, Double maxValue) {
        if (minValue == null || maxValue == null) return;
        if (minValue > maxValue) {
            throw new IllegalArgumentException(minName + " must be <= " + maxName
                    + ", got " + minValue + " > " + maxValue);
        }
    }

    private static List<Observation> buildObservations(List<double[]> track) {
        List<Observation> out = new ArrayList<>(track.size());
        for (double[] pt : track) {
            out.add(new Observation(new GHPoint(pt[0], pt[1])));
        }
        return out;
    }

    private static double nz(Double v, double fallback) {
        return v != null ? v : fallback;
    }

    /**
     * Builds a {@link MatcherConfig} for the forked Trailmap matcher from the request.
     *
     * <p>{@code gpsAccuracy} (the request's {@code gps_accuracy_m}) seeds the emission sigma.
     * Every cm_* field is optional; when all are unset the resulting config is canonical and
     * reproduces stock GH (Phase 0). cm_* fields enable Phase 1 (radius decoupling), Phase 5
     * (densification — read by the caller, not the matcher) and Phase 6 (sigma estimation).
     */
    private static MatcherConfig buildMatcherConfig(ConvertTrackRequest req, double gpsAccuracy) {
        MatcherConfig cfg = new MatcherConfig();
        cfg.measurementErrorSigma = gpsAccuracy;
        if (req.getCmBeta() != null) {
            cfg.transitionProbabilityBeta = req.getCmBeta();
        }

        // Phase 1 (M1): candidate-radius decoupling.
        cfg.candidateRadiusSigmaMult = req.getCmCandidateRadiusSigmaMult(); // null => canonical
        cfg.candidateRadiusMinM = nz(req.getCmCandidateRadiusMinM(), cfg.candidateRadiusMinM);
        cfg.candidateRadiusMaxM = nz(req.getCmCandidateRadiusMaxM(), cfg.candidateRadiusMaxM);

        // Phase 6 (P3): sigma auto-estimation.
        cfg.autoSigma = Boolean.TRUE.equals(req.getCmAutoSigma());
        cfg.autoSigmaSeedM = nz(req.getCmAutoSigmaSeedM(), cfg.autoSigmaSeedM);
        cfg.autoSigmaOutlierM = nz(req.getCmAutoSigmaOutlierM(), cfg.autoSigmaOutlierM);
        cfg.autoSigmaMinM = nz(req.getCmAutoSigmaMinM(), cfg.autoSigmaMinM);
        cfg.autoSigmaMaxM = nz(req.getCmAutoSigmaMaxM(), cfg.autoSigmaMaxM);
        cfg.autoSigmaScale = nz(req.getCmAutoSigmaScale(), cfg.autoSigmaScale);
        cfg.autoSigmaPercentile = nz(req.getCmAutoSigmaPercentile(), cfg.autoSigmaPercentile);

        // Phase 5 (P1): densification (applied by the caller).
        cfg.densifyMaxGapM = req.getCmDensifyMaxGapM(); // null => off

        // Phase 2 (M2a): profile-aware emission penalty.
        cfg.emissionDesirabilityLambda = req.getCmEmissionDesirabilityLambda(); // null/<=0 => off
        cfg.emissionDesirabilityDeadband = nz(req.getCmEmissionDesirabilityDeadband(), cfg.emissionDesirabilityDeadband);

        // V2: adaptive per-observation sigma (snap-only pre-pass + per-obs σ).
        cfg.adaptiveSigma = Boolean.TRUE.equals(req.getCmAdaptiveSigma());
        // Outlier-condition the per-obs σ window (default on; send false to A/B the un-filtered σ).
        cfg.adaptiveSigmaOutlierFilter = !Boolean.FALSE.equals(req.getCmAdaptiveSigmaOutlierFilter());

        return cfg;
    }
}
