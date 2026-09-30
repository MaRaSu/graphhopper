package com.graphhopper.resources;

import com.graphhopper.GraphHopper;
import com.graphhopper.http.ProfileResolver;
import com.graphhopper.trailmap.fixroute.FixRouteRequest;
import com.graphhopper.trailmap.fixroute.FixRouteResponse;
import com.graphhopper.trailmap.fixroute.FixRouteEngine;
import com.graphhopper.util.PMap;
import com.graphhopper.util.Parameters;
import jakarta.inject.Inject;
import jakarta.ws.rs.Consumes;
import jakarta.ws.rs.POST;
import jakarta.ws.rs.Path;
import jakarta.ws.rs.Produces;
import jakarta.ws.rs.core.MediaType;
import jakarta.ws.rs.core.Response;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * {@code POST /trailmap/fix_route} — server-side repair of a saved route against its saved track.
 * See {@code docs/gh_route_fix_spec.md} (requirements) and {@code docs/gh_route_fix_design.md}
 * (design, wire format §4.1). The engine is {@link RouteFixer}; this resource only validates,
 * resolves profile names the way {@code /route} does, and maps errors.
 */
@Path("trailmap/fix_route")
public class TrailmapFixRouteResource {

    private static final Logger LOGGER = LoggerFactory.getLogger(TrailmapFixRouteResource.class);

    /** Legs of one request are processed in parallel on this shared pool. */
    private static final ExecutorService POOL = Executors.newFixedThreadPool(
            Math.max(2, Runtime.getRuntime().availableProcessors()), r -> {
                Thread t = new Thread(r, "fix-route");
                t.setDaemon(true);
                return t;
            });

    private final GraphHopper graphHopper;
    private final ProfileResolver profileResolver;

    @Inject
    public TrailmapFixRouteResource(GraphHopper graphHopper, ProfileResolver profileResolver) {
        this.graphHopper = graphHopper;
        this.profileResolver = profileResolver;
    }

    @POST
    @Consumes(MediaType.APPLICATION_JSON)
    @Produces(MediaType.APPLICATION_JSON)
    public Response fix(FixRouteRequest request) {
        long t0 = System.currentTimeMillis();
        try {
            if (request == null) throw new IllegalArgumentException("Empty body");
            // Resolve profile names exactly as /route does for the same request, so probes route
            // with the profile the client's /route call will use.
            if (request.segments != null) {
                for (FixRouteRequest.Segment s : request.segments) {
                    if (!s.isFollowRoads() || s.profile == null || s.profile.isEmpty()) continue;
                    PMap hints = new PMap();
                    hints.putObject("profile", s.profile);
                    hints.putObject(Parameters.CH.DISABLE, true);
                    s.profile = profileResolver.resolveProfile(hints);
                }
            }
            FixRouteResponse rsp = FixRouteEngine.fix(graphHopper, POOL, request);
            LOGGER.info("fix_route: legs={} status={} added={} probes={} total_ms={}",
                    rsp.legs.size(), rsp.stats.byStatus, rsp.stats.addedWaypoints, rsp.stats.probes,
                    System.currentTimeMillis() - t0);
            return Response.ok(rsp).build();
        } catch (IllegalArgumentException e) {
            LOGGER.warn("fix_route bad request: {}", e.getMessage());
            return Response.status(Response.Status.BAD_REQUEST).entity(Map.of("error", e.getMessage())).build();
        } catch (Exception e) {
            LOGGER.error("fix_route failed", e);
            return Response.status(Response.Status.INTERNAL_SERVER_ERROR)
                    .entity(Map.of("error", e.getClass().getSimpleName() + ": " + e.getMessage())).build();
        }
    }
}
