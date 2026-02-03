/*
 * Trailmap - Route Conversion API
 *
 * JAX-RS resource for converting exploration waypoints to normalized waypoints.
 */
package com.graphhopper.resources;

import com.graphhopper.ConvertRequest;
import com.graphhopper.ConvertResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.jackson.MultiException;
import com.graphhopper.util.PointList;
import com.graphhopper.util.StopWatch;
import com.graphhopper.jackson.ResponsePathSerializer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import jakarta.inject.Inject;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.validation.constraints.NotNull;
import jakarta.ws.rs.*;
import jakarta.ws.rs.core.Context;
import jakarta.ws.rs.core.MediaType;
import jakarta.ws.rs.core.Response;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

/**
 * Resource for converting exploration waypoints to normalized waypoints.
 *
 * <p>This endpoint converts exploration waypoints from a non-finalized
 * exploration round-trip into normalized waypoints suitable for client editing.
 *
 * <p>Usage:
 * <ol>
 *   <li>Call /route with round_trip.mode=exploration&round_trip.finalize=false</li>
 *   <li>Display candidate routes to user</li>
 *   <li>When user selects a route for editing, call /convert with the exploration waypoints</li>
 *   <li>Use the returned normalized waypoints for editing</li>
 * </ol>
 */
@Path("convert")
public class ConvertResource {

    private static final Logger logger = LoggerFactory.getLogger(ConvertResource.class);

    private final GraphHopper graphHopper;

    @Inject
    public ConvertResource(GraphHopper graphHopper) {
        this.graphHopper = graphHopper;
    }

    @POST
    @Consumes(MediaType.APPLICATION_JSON)
    @Produces(MediaType.APPLICATION_JSON)
    public Response doPost(@NotNull ConvertRequest request, @Context HttpServletRequest httpReq) {
        StopWatch sw = new StopWatch().start();

        try {
            request.validate();
        } catch (IllegalArgumentException e) {
            return Response.status(Response.Status.BAD_REQUEST)
                .entity(errorJson(e.getMessage()))
                .type(MediaType.APPLICATION_JSON)
                .build();
        }

        String logPrefix = httpReq.getRemoteAddr() + " " + httpReq.getLocale();

        logger.info("{} Convert request: profile={}, waypoints={}",
            logPrefix, request.getProfile(), request.getWaypoints().size());

        ConvertResponse response = graphHopper.convert(request);

        double took = sw.stop().getMillisDouble();

        if (response.hasErrors()) {
            logger.warn("{} Convert failed: {}", logPrefix, response.getErrors());
            return Response.status(Response.Status.BAD_REQUEST)
                .entity(new MultiException(response.getErrors()))
                .type(MediaType.APPLICATION_JSON)
                .build();
        }

        logger.info("{} Convert success: {} -> {} waypoints, {:.1f}% match, took {:.1f}ms",
            logPrefix, response.getOriginalWaypointCount(), response.getNormalizedWaypointCount(),
            response.getMatchPercentage(), took);

        // Build JSON response
        Map<String, Object> json = buildResponseJson(response, request.isPointsEncoded(),
            request.getPointsEncodedMultiplier());
        json.put("took", Math.round(took));

        return Response.ok(json)
            .header("X-GH-Took", "" + Math.round(took))
            .type(MediaType.APPLICATION_JSON)
            .build();
    }

    private Map<String, Object> buildResponseJson(ConvertResponse response, boolean pointsEncoded,
                                                    double pointsMultiplier) {
        Map<String, Object> json = new LinkedHashMap<>();

        // Normalized waypoints
        List<List<Double>> waypointCoords = response.getNormalizedWaypoints().stream()
            .map(p -> List.of(p.getLon(), p.getLat()))
            .collect(Collectors.toList());
        json.put("snapped_waypoints", waypointCoords);

        // Route geometry
        PointList points = response.getPoints();
        if (pointsEncoded) {
            json.put("points", ResponsePathSerializer.encodePolyline(points, false, pointsMultiplier));
            json.put("points_encoded", true);
            json.put("points_encoded_multiplier", pointsMultiplier);
        } else {
            json.put("points", points.toLineString(false));
            json.put("points_encoded", false);
        }

        // Metrics
        json.put("distance", response.getDistance());
        json.put("time", response.getTime());
        json.put("match_percentage", response.getMatchPercentage());
        json.put("original_waypoint_count", response.getOriginalWaypointCount());
        json.put("normalized_waypoint_count", response.getNormalizedWaypointCount());

        return json;
    }

    private Map<String, Object> errorJson(String message) {
        Map<String, Object> json = new LinkedHashMap<>();
        json.put("error", message);
        return json;
    }
}
