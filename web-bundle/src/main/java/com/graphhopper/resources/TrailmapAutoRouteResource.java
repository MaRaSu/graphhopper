package com.graphhopper.resources;

import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.http.GHRequestTransformer;
import com.graphhopper.http.ProfileResolver;
import com.graphhopper.jackson.ResponsePathDeserializerHelper;
import com.graphhopper.jackson.ResponsePathSerializer;
import com.graphhopper.trailmap.roundtrip.loop.LoopAutoRoute;
import com.graphhopper.trailmap.roundtrip.loop.LoopHandover;
import com.graphhopper.util.PMap;
import com.graphhopper.util.PointList;
import com.graphhopper.util.StopWatch;
import com.graphhopper.util.shapes.GHPoint;
import jakarta.inject.Inject;
import jakarta.validation.constraints.NotNull;
import jakarta.ws.rs.*;
import jakarta.ws.rs.core.MediaType;
import jakarta.ws.rs.core.Response;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;

/**
 * Trailmap AutoRoute v2 ({@code POST /autoroute}): one press returns up to 3 round-trip loops from one start, made by
 * {@link LoopAutoRoute}. Separate from {@code /route}, whose round_trip (today's AutoRoute engine) it does not change.
 * <p>
 * Request: a {@code /route} body with one point ({@code points: [[lon, lat]]}), {@code profile}, {@code custom_model},
 * {@code details}, {@code elevation}, {@code points_encoded(_multiplier)}, plus
 * <ul>
 *   <li>{@code autoroute.distance}: the target length in metres (required);</li>
 *   <li>{@code autoroute.heading}: 0-360, absent for no direction;</li>
 *   <li>{@code autoroute.seed}: one per press;</li>
 *   <li>{@code autoroute.shown}: the {@code points} strings of the routes this session showed earlier, oldest first,
 *       encoded as this request asks ({@code elevation}, {@code points_encoded_multiplier}).</li>
 * </ul>
 * Response: {@code /route} JSON with up to 3 paths in slot order. Each path's {@code snapped_waypoints} are the loop's
 * waypoints (snapped on {@code <profile>_explore}, as the exploration round_trip returns them), which go to
 * {@code /convert} as today; its geometry, distance and details are the route that /convert will give for them.
 * {@code hints.autoroute.roles} names each slot (guided / surprise), {@code added_waypoints} counts the waypoints added
 * so that /convert follows the loop ({@link LoopHandover}); {@code autoroute.debug_loops=true} also returns the engine's
 * own loops ({@code loops}: distance, [lat, lon] coords), for the harness.
 * <p>
 * Config ({@code graphhopper:} section): {@code autoroute.max_concurrent} (2), presses beyond it wait up to
 * {@code autoroute.queue_seconds} (60); {@code autoroute.max_distance} (120000 m), longer requests get a 400;
 * {@code autoroute.max_inserts} (4), the most waypoints added per loop (a request may override it; 0 = none).
 */
@Path("autoroute")
public class TrailmapAutoRouteResource {

    private static final Logger LOGGER = LoggerFactory.getLogger(TrailmapAutoRouteResource.class);
    static final String DISTANCE = "autoroute.distance", HEADING = "autoroute.heading", SEED = "autoroute.seed",
            SHOWN = "autoroute.shown", DEBUG_LOOPS = "autoroute.debug_loops", MAX_INSERTS = "autoroute.max_inserts";
    static final int MAX_SHOWN = 60;

    /** One engine (its start cache) and one press limit per GraphHopper instance; resources are created per request. */
    private static volatile LoopAutoRoute engine;
    private static volatile Semaphore permits;
    private static volatile GraphHopper engineOwner;

    private final GraphHopperConfig config;
    private final GraphHopper graphHopper;
    private final ProfileResolver profileResolver;
    private final GHRequestTransformer ghRequestTransformer;
    private final String osmDate;
    private final List<String> snapPreventionsDefault;

    @Inject
    public TrailmapAutoRouteResource(GraphHopperConfig config, GraphHopper graphHopper, ProfileResolver profileResolver,
                                     GHRequestTransformer ghRequestTransformer) {
        this.config = config;
        this.graphHopper = graphHopper;
        this.profileResolver = profileResolver;
        this.ghRequestTransformer = ghRequestTransformer;
        this.osmDate = graphHopper.getProperties().getAll().get("datareader.data.date");
        this.snapPreventionsDefault = Arrays.stream(config.getString("routing.snap_preventions_default", "")
                .split(",")).map(String::trim).filter(s -> !s.isEmpty()).toList();
    }

    private void init() {
        if (engine != null && engineOwner == graphHopper) return;
        synchronized (TrailmapAutoRouteResource.class) {
            if (engine == null || engineOwner != graphHopper) {
                engine = new LoopAutoRoute(graphHopper);
                permits = new Semaphore(Math.max(1, config.asPMap().getInt("autoroute.max_concurrent", 2)), true);
                engineOwner = graphHopper;
            }
        }
    }

    @POST
    @Consumes(MediaType.APPLICATION_JSON)
    @Produces(MediaType.APPLICATION_JSON)
    public Response doPost(@NotNull GHRequest request) throws InterruptedException {
        init();
        StopWatch sw = new StopWatch().start();
        if (!request.hasSnapPreventions())
            request.setSnapPreventions(snapPreventionsDefault);
        request = ghRequestTransformer.transformRequest(request);
        if (request.getPoints().size() != 1)
            throw new IllegalArgumentException("autoroute needs exactly one point (the start), got " + request.getPoints().size());
        PMap hints = request.getHints();
        double targetM = hints.getDouble(DISTANCE, -1);
        double maxM = config.asPMap().getDouble("autoroute.max_distance", 120_000);
        if (!(targetM >= 1000 && targetM <= maxM))
            throw new IllegalArgumentException(DISTANCE + " must be between 1000 and " + Math.round(maxM) + " metres");
        double heading = hints.has(HEADING) ? hints.getDouble(HEADING, Double.NaN) : Double.NaN;
        long seed = hints.getLong(SEED, new Random().nextInt(10_000));
        boolean enableElevation = hints.getBool("elevation", false);
        double multiplier = hints.getDouble("points_encoded_multiplier", 1e5);
        List<List<double[]>> shown = decodeShown(hints.getObject(SHOWN, null), enableElevation, multiplier);

        PMap profileResolverHints = new PMap(hints);
        profileResolverHints.putObject("profile", request.getProfile());
        profileResolverHints.putObject("has_curbsides", false);
        String profile = profileResolver.resolveProfile(profileResolverHints);
        String exploreProfile = profile + "_explore";
        if (graphHopper.getProfile(exploreProfile) == null)
            throw new IllegalArgumentException("autoroute needs the profile '" + exploreProfile + "'");

        GHPoint start = request.getPoints().get(0);
        List<LoopAutoRoute.Route> routes;
        long waitMs, engineMs;
        if (!permits.tryAcquire(config.asPMap().getInt("autoroute.queue_seconds", 60), TimeUnit.SECONDS))
            throw new WebApplicationException("autoroute is busy, try again", Response.Status.SERVICE_UNAVAILABLE);
        try {
            waitMs = Math.round(sw.getCurrentSeconds() * 1000);
            routes = engine.generate(profile, request.getCustomModel(), start.getLat(), start.getLon(), targetM, heading,
                    seed, shown);
            engineMs = Math.round(sw.getCurrentSeconds() * 1000) - waitMs;
        } finally {
            permits.release();
        }

        // each loop as the client will rebuild it (explore via-route, then /convert), with waypoints added where /convert
        // would leave the loop
        GHResponse ghResponse = new GHResponse();
        List<String> roles = new ArrayList<>(), labels = new ArrayList<>();
        List<Integer> added = new ArrayList<>();
        List<Map<String, Object>> loops = new ArrayList<>();
        // a request may lower the configured cap (the harness's 0 = no fix), never raise it: this work runs outside permits
        int maxInserts = config.asPMap().getInt(MAX_INSERTS, LoopHandover.MAX_INSERTS);
        LoopHandover handover = new LoopHandover(graphHopper, Math.max(0, Math.min(maxInserts, hints.getInt(MAX_INSERTS, maxInserts))));
        for (LoopAutoRoute.Route r : routes) {
            LoopHandover.Result h = handover.handOver(profile, start, r, request);
            if (h == null) {
                LOGGER.warn("autoroute: dropping a {} loop, routing its waypoints failed", r.source);
                continue;
            }
            ghResponse.add(h.path);
            roles.add(r.role);
            labels.add(r.source + " " + r.label);
            added.add(h.inserted);
            loops.add(Map.of("distance", r.distance, "coords", r.coords));
        }
        double took = sw.stop().getMillisDouble();
        if (ghResponse.getAll().isEmpty())
            throw new IllegalArgumentException("autoroute found no loop of " + Math.round(targetM / 100) / 10.0
                    + " km from this start");

        Map<String, Object> info = new LinkedHashMap<>();
        info.put("engine", "loop-v2-area16");
        info.put("preview", "convert");
        info.put("roles", roles);
        info.put("labels", labels);
        info.put("added_waypoints", added);
        if (hints.getBool(DEBUG_LOOPS, false))
            info.put("loops", loops);
        info.put("seed", seed);
        info.put("shown", shown.size());
        info.put("engine_ms", engineMs);
        info.put("wait_ms", waitMs);
        ghResponse.getHints().putObject("autoroute", info);
        LOGGER.info("autoroute: profile={} distance={} heading={} seed={} shown={} routes={} engine={} ms wait={} ms took={} ms",
                profile, Math.round(targetM), Double.isNaN(heading) ? "-" : Math.round(heading), seed, shown.size(),
                ghResponse.getAll().size(), engineMs, waitMs, Math.round(took));

        boolean pointsEncoded = hints.getBool("points_encoded", true);
        return Response.ok(ResponsePathSerializer.jsonObject(ghResponse,
                        new ResponsePathSerializer.Info(config.getCopyrights(), Math.round(took), osmDate),
                        false, true, enableElevation, pointsEncoded, multiplier)).
                header("X-GH-Took", "" + Math.round(took)).
                type(MediaType.APPLICATION_JSON).
                build();
    }

    /** The session's shown routes: encoded polylines (as this endpoint returned them) to [lat, lon] lists. */
    static List<List<double[]>> decodeShown(Object value, boolean is3D, double multiplier) {
        List<List<double[]>> out = new ArrayList<>();
        if (value == null) return out;
        if (!(value instanceof List<?> list))
            throw new IllegalArgumentException(SHOWN + " must be an array of encoded polylines");
        // the newest routes matter most; a very long session keeps its last MAX_SHOWN
        int from = Math.max(0, list.size() - MAX_SHOWN);
        for (Object o : list.subList(from, list.size())) {
            if (!(o instanceof String s))
                throw new IllegalArgumentException(SHOWN + " must be an array of encoded polylines");
            PointList pl = ResponsePathDeserializerHelper.decodePolyline(s, 1000, is3D, multiplier);
            List<double[]> coords = new ArrayList<>(pl.size());
            for (int i = 0; i < pl.size(); i++) coords.add(new double[]{pl.getLat(i), pl.getLon(i)});
            out.add(coords);
        }
        return out;
    }
}
