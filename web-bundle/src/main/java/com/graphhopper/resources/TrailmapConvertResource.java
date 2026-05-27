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
            MapMatching matching = new MapMatching(
                    graphHopper.getBaseGraph(),
                    (LocationIndexTree) graphHopper.getLocationIndex(),
                    mapMatchingRouterFactory.createMapMatchingRouter(matchingHints));
            matching.setMeasurementErrorSigma(gpsAccuracy);

            List<Observation> observations = buildObservations(request.getTrack());
            long tStart = System.currentTimeMillis();
            MatchResult matchResult = matching.match(observations);
            long tMatched = System.currentTimeMillis();

            TrackToRouteConverter converter = new TrackToRouteConverter(graphHopper);
            boolean debug = Boolean.TRUE.equals(request.getDebug());
            ConvertTrackResponse response = converter.convert(
                    matchResult,
                    observations,
                    profile,
                    request.getCustomModel(),
                    nz(request.getSnapThresholdM(), TrackToRouteConverter.DEFAULT_SNAP_THRESHOLD_M),
                    nz(request.getMinRoutedSegmentM(), TrackToRouteConverter.DEFAULT_MIN_ROUTED_SEGMENT_M),
                    nz(request.getCoordinatesSimplifyEps(), TrackToRouteConverter.DEFAULT_COORDINATES_SIMPLIFY_EPS_M),
                    nz(request.getMinDetourM(), TrackToRouteConverter.DEFAULT_MIN_DETOUR_M),
                    nz(request.getMaxDetourRatio(), TrackToRouteConverter.DEFAULT_MAX_DETOUR_RATIO),
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
        if (req.getProfile() == null || req.getProfile().isEmpty()) {
            throw new IllegalArgumentException("profile is required");
        }
        for (double[] pt : req.getTrack()) {
            if (pt == null || pt.length < 2) {
                throw new IllegalArgumentException("each track entry must be [lat, lng]");
            }
            double lat = pt[0], lng = pt[1];
            if (lat < -90 || lat > 90 || lng < -180 || lng > 180) {
                throw new IllegalArgumentException("track coordinates out of range: [" + lat + ", " + lng + "]");
            }
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
}
