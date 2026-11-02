package com.graphhopper.resources;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.graphhopper.GraphHopper;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.jackson.Jackson;
import com.graphhopper.trailmap.tilerouting.TileRouteJson;
import com.graphhopper.trailmap.tilerouting.TileRouteRequest;
import com.graphhopper.trailmap.tilerouting.TileRouter;
import com.graphhopper.util.PMap;
import jakarta.inject.Inject;
import jakarta.validation.constraints.NotNull;
import jakarta.ws.rs.*;
import jakarta.ws.rs.core.MediaType;
import jakarta.ws.rs.core.Response;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;

/**
 * Trailmap: tile tours ({@code POST /tile_route}, interim v1, owner 2026-10-30), a loop from {@code start} that enters
 * every requested map tile. Separate from {@code /route}, which it does not change.
 *
 * <p>Request (JSON): {@code tiles} ["x-y", ...] (required), {@code zoom} 14 | 17 (14), {@code start} [lat, lon]
 * (required), {@code profile} (gravel), {@code safety} standard | strict | no_trails (standard), {@code algorithm}
 * strokes | repair | tour_repair (strokes), {@code variety} off | mild | strong (mild; repair, tour_repair),
 * {@code groups} [["x-y", ...], ...] (ordered groups: group 1 first), {@code walk_in_max_m} 0..300 (300, also
 * under no_trails), {@code seed} (a string; seeds the loop repair's random moves, default the tile list), {@code polish} (false), {@code polish_strength} (5), {@code polish_trail_factor} (5).
 *
 * <p>Response: the tour as waypoints with roles and one segment type per leg (followRoads | direct; direct = a walk-in
 * leg), the walks, per-tile status, stats, and the route geometry as a GeoJSON LineString ([lon, lat]).
 *
 * <p>Config ({@code graphhopper:} section): {@code tile_routing.max_tiles_z14} (50), {@code tile_routing.max_tiles_z17}
 * (150), {@code tile_routing.max_concurrent} (2), {@code tile_routing.queue_seconds} (60),
 * {@code tile_routing.portal_budget_ms} (60000), {@code tile_routing.repair_budget_ms} (10000).
 */
@Path("tile_route")
public class TrailmapTileRouteResource {

    private static final Logger LOGGER = LoggerFactory.getLogger(TrailmapTileRouteResource.class);
    private static final ObjectMapper JSON = Jackson.newObjectMapper();

    /** One concurrency limit per GraphHopper instance; resources are created per request. */
    private static volatile Semaphore permits;
    private static volatile GraphHopper permitsOwner;

    private final GraphHopperConfig config;
    private final GraphHopper graphHopper;

    @Inject
    public TrailmapTileRouteResource(GraphHopperConfig config, GraphHopper graphHopper) {
        this.config = config;
        this.graphHopper = graphHopper;
    }

    private Semaphore permits() {
        if (permits == null || permitsOwner != graphHopper) {
            synchronized (TrailmapTileRouteResource.class) {
                if (permits == null || permitsOwner != graphHopper) {
                    permits = new Semaphore(Math.max(1, config.asPMap().getInt("tile_routing.max_concurrent", 2)), true);
                    permitsOwner = graphHopper;
                }
            }
        }
        return permits;
    }

    TileRouter.Config routerConfig() {
        PMap c = config.asPMap();
        TileRouter.Config rc = new TileRouter.Config();
        rc.maxTilesZ14 = c.getInt("tile_routing.max_tiles_z14", rc.maxTilesZ14);
        rc.maxTilesZ17 = c.getInt("tile_routing.max_tiles_z17", rc.maxTilesZ17);
        rc.portalBudgetMs = c.getLong("tile_routing.portal_budget_ms", rc.portalBudgetMs);
        rc.repairBudgetMs = c.getLong("tile_routing.repair_budget_ms", rc.repairBudgetMs);
        return rc;
    }

    static TileRouteRequest parse(JsonNode b) {
        if (b == null || !b.isObject()) throw new IllegalArgumentException("the body must be a JSON object");
        TileRouteRequest rq = new TileRouteRequest();
        JsonNode tiles = b.get("tiles");
        if (tiles == null || !tiles.isArray()) throw new IllegalArgumentException("tiles must be an array of \"x-y\" keys");
        for (JsonNode k : tiles) {
            if (!k.isTextual()) throw new IllegalArgumentException("tiles must be an array of \"x-y\" keys");
            rq.tiles.add(k.asText());
        }
        JsonNode start = b.get("start");
        if (start == null || !start.isArray() || start.size() != 2 || !start.get(0).isNumber() || !start.get(1).isNumber())
            throw new IllegalArgumentException("start must be [lat, lon]");
        rq.startLat = start.get(0).asDouble();
        rq.startLon = start.get(1).asDouble();
        if (b.has("zoom")) {
            if (!b.get("zoom").isInt()) throw new IllegalArgumentException("zoom must be 14 or 17");
            rq.zoom = b.get("zoom").asInt();
        }
        rq.profile = text(b, "profile", rq.profile);
        rq.safety = text(b, "safety", rq.safety);
        rq.algorithm = text(b, "algorithm", rq.algorithm);
        rq.variety = text(b, "variety", rq.variety);
        JsonNode groups = b.get("groups");
        if (groups != null && !groups.isNull()) {
            if (!groups.isArray()) throw new IllegalArgumentException("groups must be an array of arrays of tile keys");
            for (JsonNode g : groups) {
                if (!g.isArray()) throw new IllegalArgumentException("groups must be an array of arrays of tile keys");
                List<String> l = new ArrayList<>();
                for (JsonNode k : g) l.add(k.asText());
                if (!l.isEmpty()) rq.groups.add(l);
            }
        }
        rq.seedKey = text(b, "seed", null);
        if (b.has("walk_in_max_m")) rq.walkInMaxM = number(b, "walk_in_max_m");
        rq.barriers = text(b, "barriers", null); // W26: "v1" = the app's barrier flag
        if (b.has("polish")) {
            if (!b.get("polish").isBoolean()) throw new IllegalArgumentException("polish must be true or false");
            rq.polish = b.get("polish").asBoolean();
        }
        if (b.has("polish_strength")) rq.polishStrength = number(b, "polish_strength");
        if (b.has("polish_trail_factor")) rq.polishTrailFactor = number(b, "polish_trail_factor");
        return rq;
    }

    private static String text(JsonNode b, String key, String dflt) {
        JsonNode v = b.get(key);
        if (v == null || v.isNull()) return dflt;
        if (!v.isTextual()) throw new IllegalArgumentException(key + " must be a string");
        return v.asText();
    }

    private static double number(JsonNode b, String key) {
        JsonNode v = b.get(key);
        if (!v.isNumber()) throw new IllegalArgumentException(key + " must be a number");
        return v.asDouble();
    }

    @POST
    @Consumes(MediaType.APPLICATION_JSON)
    @Produces(MediaType.APPLICATION_JSON)
    public Response doPost(@NotNull JsonNode body) throws InterruptedException {
        long t0 = System.nanoTime();
        TileRouteRequest rq = parse(body);
        Semaphore p = permits();
        if (!p.tryAcquire(config.asPMap().getInt("tile_routing.queue_seconds", 60), TimeUnit.SECONDS))
            throw new WebApplicationException("tile_route is busy, try again", Response.Status.SERVICE_UNAVAILABLE);
        TileRouter.Result r;
        long waitMs = (System.nanoTime() - t0) / 1_000_000;
        try {
            r = new TileRouter(graphHopper, routerConfig()).route(rq);
        } finally {
            p.release();
        }
        ObjectNode out = TileRouteJson.write(JSON, rq, r);
        ((ObjectNode) out.get("stats").get("timings")).put("waitMs", waitMs);
        ObjectNode geom = out.putObject("points");
        geom.put("type", "LineString");
        ArrayNode cs = geom.putArray("coordinates");
        if (r.route != null)
            for (double[] c : r.route.coords) cs.addArray().add(Math.round(c[1] * 1e6) / 1e6).add(Math.round(c[0] * 1e6) / 1e6);
        LOGGER.info("tile_route tiles={} zoom={} algorithm={} safety={} polish={} fetched={} km={} ms={}", rq.tiles.size(), rq.zoom,
                rq.algorithm, rq.safety, rq.polish, out.get("stats").get("fetched"), r.route == null ? -1 : Math.round(r.route.distanceM / 100) / 10.0,
                r.totalMs);
        return Response.ok(out).build();
    }
}
