package com.graphhopper.resources;

import com.fasterxml.jackson.databind.node.ObjectNode;
import com.carrotsearch.hppc.IntArrayList;
import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.ResponsePath;
import com.graphhopper.http.GHRequestTransformer;
import com.graphhopper.http.ProfileResolver;
import com.graphhopper.jackson.MultiException;
import com.graphhopper.jackson.ResponsePathSerializer;
import com.graphhopper.trailmap.personal.AltRouteVias;
import com.graphhopper.trailmap.personal.DirCellSource;
import com.graphhopper.trailmap.personal.FirebaseTokenVerifier;
import com.graphhopper.trailmap.personal.PersonalCellGrid;
import com.graphhopper.trailmap.personal.PersonalCellSource;
import com.graphhopper.trailmap.personal.PersonalCells;
import com.graphhopper.trailmap.personal.PersonalFamiliarity;
import com.graphhopper.trailmap.personal.PersonalParams;
import com.graphhopper.trailmap.personal.PersonalRouter;
import com.graphhopper.trailmap.personal.PersonalStore;
import com.graphhopper.trailmap.personal.RecreateVias;
import com.graphhopper.trailmap.personal.RemoteCellSource;
import com.graphhopper.trailmap.personal.variations.BitOfNew;
import com.graphhopper.util.PMap;
import com.graphhopper.util.StopWatch;
import jakarta.inject.Inject;
import jakarta.servlet.http.HttpServletResponse;
import jakarta.validation.constraints.NotNull;
import jakarta.ws.rs.*;
import jakarta.ws.rs.core.Context;
import jakarta.ws.rs.core.MediaType;
import jakarta.ws.rs.core.Response;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.nio.file.Paths;
import java.time.Duration;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import static com.graphhopper.util.Parameters.Routing.CALC_POINTS;
import static com.graphhopper.util.Parameters.Routing.INSTRUCTIONS;

/**
 * Trailmap: personalised A→B routing ({@code POST /personal_route}), separate from
 * {@code /route}, which it does not change. Same request and response as {@code POST /route}, plus
 * {@code personal_*} fields (see {@link PersonalParams}).
 *
 * <p>Whose history: the user of the {@code Authorization: Bearer <Firebase ID token>} header (its {@code sub}); the
 * request's {@code personal_user} is ignored unless the previewer mode is on. A request without a token routes as
 * /route when it asks for nothing personal (mode neutral, no bit_of_new history) and gets 401 otherwise; a token that
 * is present but not valid always gets 401.
 *
 * <p>{@code algorithm=bit_of_new} adds "bit of new" variations of the route as further paths (see
 * {@link TrailmapBitOfNew}); without it nothing changes.
 *
 * <p>{@code GET /personal_route/cells} (a user's cells in a bounding box) and {@code GET /personal_route/datasets}
 * serve the local previewer and exist only in the previewer mode.
 *
 * <p>Config ({@code graphhopper:} section). Exactly one cells source enables the endpoints; with none, every endpoint
 * here answers 404, which is the off switch:
 * <ul>
 * <li>{@code personal_routing.cells_source}: the activity service's internal cells URL (production), with the shared
 * secret in the environment variable named by {@code personal_routing.cells_secret_env} ({@code GH_CELLS_SECRET});
 * the disk copy in {@code personal_routing.cells_disk_dir} (none if unset), kept {@code personal_routing.cells_disk_days}
 * (30) days after its last use; one fetch (headers and body) takes at most {@code personal_routing.cells_timeout_ms}
 * (4000), after which the disk copy or plain routing answers; an empty secret answers 503 and logs an error;</li>
 * <li>{@code personal_routing.cells_dir}: local files {@code <cells_dir>/<user>/<set>.csv} (previewer, tests).</li>
 * </ul>
 * {@code personal_routing.previewer} (default: true with cells_dir, false with cells_source),
 * {@code personal_routing.firebase_project} ({@code trailmapv2}), {@code personal_routing.cache_minutes} (5: a user's
 * cells are dropped after 5 idle minutes, even without further requests), {@code personal_routing.max_users} (8).
 *
 * <p>Re-create via points: every path the user's history or bit_of_new generated carries {@code via_points} such that
 * plain {@code /route} [A, ...via_points, B] rides it again, as the app rebuilds a saved segment
 * ({@link RecreateVias}); {@code hints.personal.recreate} says per path whether that holds. Levers:
 * {@code personal_routing.recreate_max_vias} (30 per path, 0 = off) and {@code personal_routing.recreate_budget_ms}
 * (2000 per request; paths not started by then are left as they are). Paths longer than {@link AltRouteVias#MAX_KM}
 * are not pinned (one check route alone costs seconds there); their hint says {@code stopped: "too long"}.
 */
@Path("personal_route")
public class TrailmapPersonalRouteResource {

    private static final Logger LOGGER = LoggerFactory.getLogger(TrailmapPersonalRouteResource.class);
    private static final int MAX_CELLS_RESPONSE = 80_000;
    /**
     * Web origins that may send the Authorization header (CORS preflight): exactly the hosts the web app is served at
     * (lead_web, 2026-10-30); trailmap.fi, www.trailmap.fi, trailmap.guide and global.trailmap.fi only redirect.
     */
    private static final Set<String> WEB_ORIGINS = Set.of("https://web.trailmap.fi", "https://beta.trailmap.fi",
            "https://web.trailmap.guide", "https://ng-test.trailmap.fi", "http://localhost:3000");
    private static final String ALLOW_HEADERS = "Origin,Accept,X-Requested-With,Content-Type,"
            + "Access-Control-Request-Method,Access-Control-Request-Headers,Range,GH-Client";

    /** One store per GraphHopper instance; resources are created per request. */
    private static volatile PersonalStore store;
    private static volatile GraphHopper storeOwner;
    /** The cells source settings the store was made for. */
    private static volatile String storeKey;
    /** One token check per Firebase project; tests may replace it. */
    private static volatile FirebaseTokenVerifier verifier;
    private static volatile String verifierProject;

    private final GraphHopperConfig config;
    private final GraphHopper graphHopper;
    private final ProfileResolver profileResolver;
    private final GHRequestTransformer ghRequestTransformer;
    private final String osmDate;
    private final List<String> snapPreventionsDefault;

    @Inject
    public TrailmapPersonalRouteResource(GraphHopperConfig config, GraphHopper graphHopper, ProfileResolver profileResolver,
                                         GHRequestTransformer ghRequestTransformer) {
        this.config = config;
        this.graphHopper = graphHopper;
        this.profileResolver = profileResolver;
        this.ghRequestTransformer = ghRequestTransformer;
        this.osmDate = graphHopper.getProperties().getAll().get("datareader.data.date");
        // same default as RouteResource, so neutral requests snap exactly as /route does
        this.snapPreventionsDefault = Arrays.stream(config.getString("routing.snap_preventions_default", "")
                .split(",")).map(String::trim).filter(s -> !s.isEmpty()).toList();
    }

    private String cellsDir() {
        return config.getString("personal_routing.cells_dir", "").trim();
    }

    private String cellsSource() {
        return config.getString("personal_routing.cells_source", "").trim();
    }

    /**
     * Off switch: without a cells source in the config every endpoint of this resource answers 404, so neither
     * neutral routing nor bit_of_new is reachable.
     */
    private void requireEnabled() {
        if (cellsDir().isEmpty() && cellsSource().isEmpty())
            throw new NotFoundException("personal_route is not enabled on this server");
    }

    /** The local previewer mode: the GET endpoints and a request-chosen personal_user. Never on in production. */
    private boolean previewer() {
        return config.getBool("personal_routing.previewer", !cellsDir().isEmpty());
    }

    /** The previewer's GET endpoints: 404 unless enabled and in the previewer mode (they serve any user's cells). */
    private void requirePreviewer() {
        requireEnabled();
        if (!previewer())
            throw new NotFoundException("not available on this server");
    }

    private PersonalStore store() {
        PersonalStore s = store;
        String key = String.join("|", cellsDir(), cellsSource(), config.getString("personal_routing.cells_disk_dir", ""),
                config.getString("personal_routing.cells_secret_env", ""), config.getString("personal_routing.cells_timeout_ms", ""));
        if (s != null && storeOwner == graphHopper && key.equals(storeKey)) return s;
        synchronized (TrailmapPersonalRouteResource.class) {
            if (store == null || storeOwner != graphHopper || !key.equals(storeKey)) {
                long ttl = config.asPMap().getLong("personal_routing.cache_minutes", 5) * 60_000L;
                int maxUsers = config.asPMap().getInt("personal_routing.max_users", 8);
                PersonalCellSource source = cellSource();
                if (store != null) store.close();
                store = new PersonalStore(source, graphHopper.getBaseGraph(), ttl, maxUsers);
                storeOwner = graphHopper;
                storeKey = key;
            }
            return store;
        }
    }

    private PersonalCellSource cellSource() {
        String dir = cellsDir(), url = cellsSource();
        if (!dir.isEmpty() && !url.isEmpty())
            throw new IllegalStateException("set only one of personal_routing.cells_dir and personal_routing.cells_source");
        if (!dir.isEmpty())
            return new DirCellSource(Paths.get(dir));
        String secretEnv = config.getString("personal_routing.cells_secret_env", "GH_CELLS_SECRET");
        String secret = System.getenv(secretEnv);
        if (secret == null || secret.isBlank()) {
            // misconfigured server, not the client's fault: loud in the log, 503 to the client, retried per request
            LOGGER.error("personal: personal_routing.cells_source is set but env {} is empty: personal routing is unavailable", secretEnv);
            throw new WebApplicationException(Response.status(Response.Status.SERVICE_UNAVAILABLE)
                    .entity(Map.of("message", "personal routing is not configured on this server"))
                    .type(MediaType.APPLICATION_JSON).build());
        }
        String disk = config.getString("personal_routing.cells_disk_dir", "").trim();
        long diskDays = config.asPMap().getLong("personal_routing.cells_disk_days", 30);
        // well inside the web client's own 15 s route timeout, so a stalled service still lets the route answer
        long timeoutMs = config.asPMap().getLong("personal_routing.cells_timeout_ms", 4_000);
        LOGGER.info("personal: cells from {} (disk copy {}, {} days, timeout {} ms)", url, disk.isEmpty() ? "none" : disk, diskDays, timeoutMs);
        return new RemoteCellSource(url, secret, disk.isEmpty() ? null : Paths.get(disk),
                diskDays * 86_400_000L, Duration.ofMillis(Math.min(3_000, timeoutMs)), Duration.ofMillis(timeoutMs));
    }

    private FirebaseTokenVerifier verifier() {
        String project = config.getString("personal_routing.firebase_project", "trailmapv2");
        FirebaseTokenVerifier v = verifier;
        if (v != null && project.equals(verifierProject)) return v;
        synchronized (TrailmapPersonalRouteResource.class) {
            if (verifier == null || !project.equals(verifierProject)) {
                verifier = FirebaseTokenVerifier.forProject(project);
                verifierProject = project;
            }
            return verifier;
        }
    }

    /** Tests only: a token check with their own keys, for this project. */
    static void setVerifierForTests(FirebaseTokenVerifier v, String project) {
        synchronized (TrailmapPersonalRouteResource.class) {
            verifier = v;
            verifierProject = project;
        }
    }

    private static WebApplicationException unauthorized(String message) {
        return new WebApplicationException(Response.status(Response.Status.UNAUTHORIZED)
                .header("WWW-Authenticate", "Bearer")
                .entity(Map.of("message", message))
                .type(MediaType.APPLICATION_JSON).build());
    }

    /**
     * Whose history this request routes on: the token's user, else (previewer only) the request's personal_user, else
     * nobody. Writes the answer into the hints' personal_user, which everything below reads.
     */
    private void bindUser(String authorization, PMap hints, boolean asksPersonal) {
        String user = "";
        if (authorization != null && !authorization.isBlank()) {
            String a = authorization.trim();
            if (!a.regionMatches(true, 0, "Bearer ", 0, 7))
                throw unauthorized("Authorization must be 'Bearer <Firebase ID token>'");
            try {
                user = verifier().verify(a.substring(7).trim());
            } catch (FirebaseTokenVerifier.KeysUnavailableException e) {
                // Google's keys not reachable yet: transient, not the client's token (clients treat 401 as "turn off")
                LOGGER.warn("personal_route: token not checked: {}", e.getMessage());
                throw new WebApplicationException(Response.status(Response.Status.SERVICE_UNAVAILABLE)
                        .header("Retry-After", "60")
                        .entity(Map.of("message", "personal routing temporarily unavailable: " + e.getMessage()))
                        .type(MediaType.APPLICATION_JSON).build());
            } catch (FirebaseTokenVerifier.InvalidTokenException e) {
                LOGGER.info("personal_route: token refused: {}", e.getMessage());
                throw unauthorized("invalid token: " + e.getMessage());
            }
        } else if (previewer()) {
            user = hints.getString(PersonalParams.USER, "");
        } else if (asksPersonal) {
            throw unauthorized("personal routing needs 'Authorization: Bearer <Firebase ID token>'");
        }
        hints.putObject(PersonalParams.USER, user);
    }

    /** CORS preflight: the web origins may send the Authorization header to this endpoint. */
    @OPTIONS
    public Response preflight(@HeaderParam("Origin") String origin, @Context HttpServletResponse rsp) {
        if (origin != null && WEB_ORIGINS.contains(origin)) {
            rsp.setHeader("Access-Control-Allow-Origin", origin);
            rsp.setHeader("Access-Control-Allow-Headers", ALLOW_HEADERS + ",Authorization");
            rsp.setHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
            rsp.setHeader("Access-Control-Max-Age", "600");
            rsp.addHeader("Vary", "Origin");
        }
        return Response.noContent().build();
    }

    @POST
    @Consumes(MediaType.APPLICATION_JSON)
    @Produces(MediaType.APPLICATION_JSON)
    public Response doPost(@NotNull GHRequest request, @HeaderParam("Authorization") String authorization) throws IOException {
        requireEnabled();
        if (!request.hasSnapPreventions())
            request.setSnapPreventions(snapPreventionsDefault);
        StopWatch sw = new StopWatch().start();
        request = ghRequestTransformer.transformRequest(request);

        PMap profileResolverHints = new PMap(request.getHints());
        profileResolverHints.putObject("profile", request.getProfile());
        profileResolverHints.putObject("has_curbsides", !request.getCurbsides().isEmpty());
        request.setProfile(profileResolver.resolveProfile(profileResolverHints));
        RouteResource.removeLegacyParameters(request.getHints());
        // the re-create via points need every path's edges; asking for them here would change the points (GraphHopper
        // keeps a point at every path-detail boundary when it simplifies), so without them the paths are routed again
        boolean askedEdgeId = request.getPathDetails().contains(RecreateVias.EDGE_ID);
        // algorithm=bit_of_new: the main route is this same request routed as usual; variations follow it
        TrailmapBitOfNew bitOfNew = TrailmapBitOfNew.take(request);
        String mode = request.getHints().getString(PersonalParams.MODE, "neutral").trim();
        bindUser(authorization, request.getHints(),
                !"neutral".equalsIgnoreCase(mode) || (bitOfNew != null && bitOfNew.history != BitOfNew.History.NONE));
        PersonalParams params = PersonalParams.from(request.getHints());
        GHRequest plainRequest = RecreateVias.plainRequest(request);

        PersonalFamiliarity familiarity = null;
        PersonalStore.Lookup lookup = null;
        if (params.isActive()) {
            // a user without this data set routes exactly as /route
            lookup = store().get(params.user, params.cells);
            if (lookup != null) familiarity = lookup.familiarity;
        }
        long computedBefore = familiarity == null ? 0 : familiarity.getComputedEdges();

        PersonalRouter router = new PersonalRouter(graphHopper, familiarity, params);
        GHResponse ghResponse = router.route(request);
        double took = sw.stop().getMillisDouble();
        if (ghResponse.hasErrors())
            throw new MultiException(ghResponse.getErrors());
        int routedPaths = ghResponse.getAll().size();
        // W26 (flag): a main path that crosses a barrier comes alone (BarrierRouter routes it without alternatives);
        // bit_of_new variations are not added to it either: a generated route never crosses a gate on its own initiative
        boolean crossing = hasBarrierCrossing(ghResponse);
        boolean skippedForCrossing = crossing && (bitOfNew != null || "alternative_route".equalsIgnoreCase(request.getAlgorithm()));
        if (crossing && bitOfNew != null) {
            bitOfNew.skipForBarrierCrossing(ghResponse);
        } else if (bitOfNew != null) {
            sw.start();
            PersonalFamiliarity mainFamiliarity = familiarity;
            bitOfNew.apply(graphHopper, request, ghResponse, r -> new PersonalRouter(graphHopper, mainFamiliarity, params).route(r), cells -> {
                PersonalStore.Lookup l = params.user.isEmpty() ? null : store().get(params.user, cells);
                return l == null ? null : l.familiarity;
            });
            // W26 (flag only): every variation gets its hints.barriers.paths entry (its opening-hours notices)
            router.addNoticesForAddedPaths(request, ghResponse);
            took = sw.stop().getMillisDouble();
        }

        boolean pointsEncoded = request.getHints().getBool("points_encoded", true);
        double pointsEncodedMultiplier = request.getHints().getDouble("points_encoded_multiplier", 1e5);
        sw.start();
        // the paths to pin, with their edges: the response's own when it has them; alternatives as the router computed
        // them; else (one route) the same request routed again, unless it is too long to be pinned anyway
        List<ResponsePath> withEdges = new ArrayList<>(ghResponse.getAll());
        List<ResponsePath> alternatives = router.getAlternativesWithEdges();
        if (familiarity != null && !askedEdgeId && alternatives != null && alternatives.size() == routedPaths) {
            for (int i = 0; i < routedPaths; i++) withEdges.set(i, alternatives.get(i));
        } else if (familiarity != null && !askedEdgeId && plainRequest != null && recreateMaxVias() > 0
                && ghResponse.getBest().getDistance() <= AltRouteVias.MAX_KM * 1000) {
            List<String> asked = request.getPathDetails();
            List<String> pd = new ArrayList<>(asked);
            pd.add(RecreateVias.EDGE_ID);
            request.setPathDetails(pd);
            try {
                GHResponse again = new PersonalRouter(graphHopper, familiarity, params).route(request);
                for (int i = 0; i < routedPaths; i++)
                    withEdges.set(i, !again.hasErrors() && again.getAll().size() == routedPaths ? again.getAll().get(i) : null);
            } finally {
                request.setPathDetails(asked);
            }
        }
        List<Map<String, Object>> recreate = recreateVias(plainRequest, ghResponse, withEdges, routedPaths,
                familiarity != null, pointsEncoded ? pointsEncodedMultiplier : 0);
        took = sw.stop().getMillisDouble();
        if (!askedEdgeId)
            for (ResponsePath p : ghResponse.getAll()) p.getPathDetails().remove(RecreateVias.EDGE_ID);

        Map<String, Object> info = new LinkedHashMap<>(params.toMap());
        info.put("active", familiarity != null);
        if (params.mode != PersonalParams.Mode.NEUTRAL && familiarity == null)
            info.put("history", "none");
        info.put("turn_costs", graphHopper.getProfile(request.getProfile()).hasTurnCosts());
        if (familiarity != null) {
            info.put("cells", familiarity.getCells().size());
            info.put("cached", lookup.cached);
            info.put("load_ms", lookup.loadMillis);
            info.put("edges_measured_now", familiarity.getComputedEdges() - computedBefore);
            info.put("edges_measured_total", familiarity.getComputedEdges());
        }
        if (!recreate.isEmpty()) info.put("recreate", recreate);
        if (skippedForCrossing) info.put("alternatives_skipped", "barrier_crossing");
        ghResponse.getHints().putObject("personal", info);
        LOGGER.info("personal_route: mode={} strength={} switch={} profile={} took={} ms measured_now={} distance={}",
                info.get("mode"), params.strength, params.switchPenalty, request.getProfile(), Math.round(took),
                info.getOrDefault("edges_measured_now", 0), Math.round(ghResponse.getBest().getDistance()));

        boolean instructions = request.getHints().getBool(INSTRUCTIONS, true);
        boolean enableElevation = request.getHints().getBool("elevation", false);
        boolean calcPoints = request.getHints().getBool(CALC_POINTS, true);
        ObjectNode json = ResponsePathSerializer.jsonObject(ghResponse,
                new ResponsePathSerializer.Info(config.getCopyrights(), Math.round(took), osmDate),
                instructions, calcPoints, enableElevation, pointsEncoded, pointsEncodedMultiplier);
        if (bitOfNew != null) bitOfNew.decorate(json, ghResponse.getBest().getDistance());
        return Response.ok(json).
                header("X-GH-Took", "" + Math.round(took)).
                type(MediaType.APPLICATION_JSON).
                build();
    }

    /** W26: the response's main path crosses a barrier (hints.barriers.paths[0].crossings not empty; flag only). */
    @SuppressWarnings("unchecked")
    static boolean hasBarrierCrossing(GHResponse rsp) {
        Object b = rsp.getHints().getObject(com.graphhopper.trailmap.barrier.BarrierRouter.HINT, null);
        if (!(b instanceof Map)) return false;
        Object paths = ((Map<String, Object>) b).get("paths");
        if (!(paths instanceof List) || ((List<?>) paths).isEmpty()) return false;
        Object c = ((Map<String, Object>) ((List<?>) paths).get(0)).get("crossings");
        return c instanceof List && !((List<?>) c).isEmpty();
    }

    /**
     * Re-create via points ({@link RecreateVias}): sets each generated path's via_points so that plain /route
     * [A, ...via_points, B] rides it again, as the app rebuilds a saved segment. Generated = the routed paths when the
     * user's history weighted them (else they are /route's own answer and stay untouched) and every bit_of_new
     * variation. withEdges: each path as routed with the edge_id detail (null when not known). One entry per pinned
     * path for hints.personal.recreate.
     */
    /** Vias per path at most; 0 turns the re-create via points off (and the work for them). */
    private int recreateMaxVias() {
        return config.asPMap().getInt("personal_routing.recreate_max_vias", 30);
    }

    private List<Map<String, Object>> recreateVias(GHRequest plain, GHResponse rsp, List<ResponsePath> withEdges,
                                                   int routedPaths, boolean personal, double multiplier) {
        List<Map<String, Object>> out = new ArrayList<>();
        int maxVias = recreateMaxVias();
        long budgetMs = config.asPMap().getLong("personal_routing.recreate_budget_ms", 2_000);
        List<ResponsePath> paths = rsp.getAll();
        if (maxVias <= 0 || (!personal && paths.size() == routedPaths)) return out;
        long deadline = System.nanoTime() + budgetMs * 1_000_000L;
        for (int i = personal ? 0 : routedPaths; i < paths.size(); i++) {
            ResponsePath p = paths.get(i);
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("path", i);
            ResponsePath routed = withEdges.get(i);
            if (p.getDistance() > AltRouteVias.MAX_KM * 1000) {
                // one plain route alone costs seconds there and can't be stopped once started (the app's 15 s)
                m.put("identical", false);
                m.put("stopped", "too long");
            } else if (System.nanoTime() > deadline) {
                m.put("identical", false);
                m.put("stopped", "time budget");
            } else if (plain == null || routed == null || RecreateVias.edgeIds(routed) == null) {
                m.put("identical", false);
                m.put("stopped", plain == null ? "not two points, or curbsides or point hints" : "no edges known");
            } else {
                RecreateVias.Result r = new RecreateVias(graphHopper, plain, maxVias, multiplier, deadline).pin(routed);
                p.setAltRouteViaPoints(r.vias);
                m.putAll(r.toMap());
                if (!r.identical)
                    LOGGER.info("personal_route: path {} not re-created by plain /route: {}", i, r.toMap());
            }
            out.add(m);
        }
        return out;
    }

    /** The data sets the user has, for the previewer's data-set choice. */
    @GET
    @Path("datasets")
    @Produces(MediaType.APPLICATION_JSON)
    public Map<String, Object> dataSets(@QueryParam("user") String user) throws IOException {
        requirePreviewer();
        PMap hints = new PMap();
        hints.putObject(PersonalParams.USER, user == null ? "" : user);
        PersonalParams params = PersonalParams.from(hints); // validates the name
        if (params.user.isEmpty()) throw new IllegalArgumentException("user is required");
        return Map.of("user", params.user, "datasets", store().dataSets(params.user));
    }

    /** A user's cells inside {@code bbox=minLon,minLat,maxLon,maxLat}, as {@code [x, y, rides]} rows. */
    @GET
    @Path("cells")
    @Produces(MediaType.APPLICATION_JSON)
    public Map<String, Object> cells(@QueryParam("user") String user, @QueryParam("cells") String cells,
                                     @QueryParam("bbox") String bbox) throws IOException {
        requirePreviewer();
        PMap hints = new PMap();
        hints.putObject(PersonalParams.MODE, "explore");
        hints.putObject(PersonalParams.USER, user == null ? "" : user);
        hints.putObject(PersonalParams.CELLS, cells == null ? "" : cells);
        PersonalParams params = PersonalParams.from(hints); // validates the names
        String[] b = bbox == null ? new String[0] : bbox.split(",");
        if (b.length != 4) throw new IllegalArgumentException("bbox must be minLon,minLat,maxLon,maxLat");
        double minLon = Double.parseDouble(b[0]), minLat = Double.parseDouble(b[1]);
        double maxLon = Double.parseDouble(b[2]), maxLat = Double.parseDouble(b[3]);

        PersonalCells c = store().cells(params.user, params.cells);
        PersonalCellGrid g = c.getGrid();
        List<long[]> rows = new ArrayList<>();
        boolean[] truncated = {false};
        c.forEachIn(g.cellX(minLon), g.cellX(maxLon), g.cellY(maxLat), g.cellY(minLat), (id, rides) -> {
            if (rows.size() >= MAX_CELLS_RESPONSE) {
                truncated[0] = true;
                return;
            }
            rows.add(new long[]{g.xOf(id), g.yOf(id), rides});
        });
        Map<String, Object> out = new LinkedHashMap<>();
        out.put("zoom", g.getZoom());
        out.put("total_cells", c.size());
        out.put("max_rides", c.getMaxRides());
        out.put("truncated", truncated[0]);
        out.put("cells", rows);
        return out;
    }
}
