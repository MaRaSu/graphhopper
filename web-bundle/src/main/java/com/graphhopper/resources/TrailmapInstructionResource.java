package com.graphhopper.resources;

import com.graphhopper.GraphHopper;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.trailmap.tbt.InstructionPostProcessor;
import com.graphhopper.trailmap.tbt.RouteInstructionGenerator;
import com.graphhopper.trailmap.tbt.TrailmapInstructionRequest;
import com.graphhopper.trailmap.tbt.TrailmapInstructionResponse;
import com.graphhopper.jackson.ResponsePathSerializer;
import com.graphhopper.util.*;
import jakarta.inject.Inject;
import jakarta.ws.rs.*;
import jakarta.ws.rs.core.MediaType;
import jakarta.ws.rs.core.Response;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

/**
 * JAX-RS endpoint for Trailmap TbT instruction generation.
 * <p>
 * Accepts a full route definition (waypoints + segments with per-segment routing profiles),
 * re-routes to obtain edge sequences, concatenates into a single continuous path, and
 * generates one coherent instruction set. Waypoint boundaries are invisible in the output.
 * <p>
 * Non-routable segments (direct/coordinates) produce no instructions but their geometry
 * is included in the response polyline. The instruction preceding a non-routable segment
 * carries an extraInfo flag {@code next_segment_type} with value "direct" or "coordinates".
 * <p>
 * POST /trailmap/instructions
 */
@Path("trailmap/instructions")
public class TrailmapInstructionResource {

    private static final Logger LOGGER = LoggerFactory.getLogger(TrailmapInstructionResource.class);
    private static final double POLYLINE_PRECISION = 1e6;
    private static final Set<String> RESERVED_INSTRUCTION_KEYS = Set.of(
            "text", "street_name", "time", "distance", "sign", "interval");

    private final RouteInstructionGenerator generator;
    private final InstructionPostProcessor postProcessor;

    @Inject
    public TrailmapInstructionResource(GraphHopper graphHopper, BaseGraph baseGraph,
                                       EncodingManager encodingManager,
                                       TranslationMap translationMap) {
        this.generator = new RouteInstructionGenerator(graphHopper, baseGraph, encodingManager, translationMap);
        this.postProcessor = new InstructionPostProcessor();
    }

    @POST
    @Consumes(MediaType.APPLICATION_JSON)
    @Produces(MediaType.APPLICATION_JSON)
    public Response getInstructions(TrailmapInstructionRequest request) {
        long start = System.currentTimeMillis();

        try {
            validateRequest(request);

            RouteInstructionGenerator.Result result = generator.generate(request);
            postProcessor.process(result.instructions, request.getInstructionProfile());

            TrailmapInstructionResponse response = new TrailmapInstructionResponse();
            response.setInstructions(serializeInstructions(result.instructions));
            response.setPoints(ResponsePathSerializer.encodePolyline(result.polyline, false, POLYLINE_PRECISION));
            response.setPointsEncoded(true);
            response.setPointsEncodedMultiplier(POLYLINE_PRECISION);

            long elapsed = System.currentTimeMillis() - start;
            LOGGER.info("Generated {} instructions, {} polyline points for route with {} segments in {}ms",
                    result.instructions.size(), result.polyline.size(),
                    request.getSegments().size(), elapsed);

            return Response.ok(response).build();

        } catch (IllegalArgumentException e) {
            LOGGER.warn("Bad request: {}", e.getMessage());
            return Response.status(Response.Status.BAD_REQUEST)
                    .entity(Map.of("error", e.getMessage()))
                    .build();
        } catch (IllegalStateException e) {
            LOGGER.error("Instruction generation failed", e);
            return Response.status(Response.Status.INTERNAL_SERVER_ERROR)
                    .entity(Map.of("error", e.getMessage()))
                    .build();
        } catch (Exception e) {
            LOGGER.error("Unexpected error during instruction generation", e);
            return Response.status(Response.Status.INTERNAL_SERVER_ERROR)
                    .entity(Map.of("error", "Internal server error"))
                    .build();
        }
    }

    private void validateRequest(TrailmapInstructionRequest request) {
        if (request.getWaypoints() == null || request.getWaypoints().size() < 2) {
            throw new IllegalArgumentException("At least 2 waypoints required");
        }
        if (request.getSegments() == null || request.getSegments().isEmpty()) {
            throw new IllegalArgumentException("At least 1 segment required");
        }
        if (request.getWaypoints().size() > 1000) {
            throw new IllegalArgumentException("Maximum 1000 waypoints allowed");
        }
        if (request.getSegments().size() > 1000) {
            throw new IllegalArgumentException("Maximum 1000 segments allowed");
        }
        if (request.getInstructionProfile() == null || request.getInstructionProfile().isEmpty()) {
            throw new IllegalArgumentException("instruction_profile is required");
        }

        Set<String> waypointIds = new HashSet<>();
        for (TrailmapInstructionRequest.Waypoint wp : request.getWaypoints()) {
            if (wp.getId() == null || wp.getCoordinates() == null) {
                throw new IllegalArgumentException("Waypoint must have id and coordinates");
            }
            if (!waypointIds.add(wp.getId())) {
                throw new IllegalArgumentException("Duplicate waypoint id: " + wp.getId());
            }
            double lat = wp.getCoordinates().getLat();
            double lng = wp.getCoordinates().getLng();
            if (lat < -90 || lat > 90 || lng < -180 || lng > 180) {
                throw new IllegalArgumentException("Waypoint " + wp.getId() + " has coordinates out of range");
            }
            if (lat == 0.0 && lng == 0.0) {
                throw new IllegalArgumentException("Waypoint " + wp.getId() + " has zero coordinates (likely missing from request)");
            }
        }
        for (TrailmapInstructionRequest.Segment seg : request.getSegments()) {
            if (!waypointIds.contains(seg.getStart())) {
                throw new IllegalArgumentException("Segment references unknown start waypoint: " + seg.getStart());
            }
            if (!waypointIds.contains(seg.getEnd())) {
                throw new IllegalArgumentException("Segment references unknown end waypoint: " + seg.getEnd());
            }
            if (seg.isRoutable() && (seg.getProfile() == null || seg.getProfile().isEmpty())) {
                throw new IllegalArgumentException("Routable segment must have a profile");
            }
        }
    }

    /**
     * Serialize InstructionList to the same JSON format as GH's InstructionListSerializer.
     */
    private List<Map<String, Object>> serializeInstructions(InstructionList instructions) {
        List<Map<String, Object>> result = new ArrayList<>(instructions.size());
        int pointsIndex = 0;

        for (Instruction instruction : instructions) {
            Map<String, Object> instrJson = new LinkedHashMap<>();

            instrJson.put("text", Helper.firstBig(instruction.getTurnDescription(instructions.getTr())));
            instrJson.put("street_name", instruction.getName());
            instrJson.put("time", instruction.getTime());
            instrJson.put("distance", Helper.round(instruction.getDistance(), 3));
            instrJson.put("sign", instruction.getSign());
            Map<String, Object> extraInfo = instruction.getExtraInfoJSON();
            for (String key : extraInfo.keySet()) {
                if (RESERVED_INSTRUCTION_KEYS.contains(key)) {
                    throw new IllegalStateException("extraInfo key '" + key
                            + "' collides with reserved instruction field (sign=" + instruction.getSign()
                            + ", text=" + instruction.getTurnDescription(instructions.getTr()) + ")");
                }
            }
            instrJson.putAll(extraInfo);

            int tmpIndex = pointsIndex + instruction.getLength();
            instrJson.put("interval", Arrays.asList(pointsIndex, tmpIndex));
            pointsIndex = tmpIndex;

            result.add(instrJson);
        }

        return result;
    }
}
