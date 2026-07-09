package com.graphhopper.trailmap.tbt;

import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.ResponsePath;
import com.graphhopper.config.Profile;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.ev.EncodedValueLookup;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.util.*;
import com.graphhopper.util.details.PathDetail;
import com.graphhopper.util.shapes.GHPoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.stream.Collectors;

/**
 * Core logic for Trailmap TbT instruction generation.
 * <p>
 * Handles three segment types:
 * <ul>
 *   <li>{@code followRoads} — routed via GH, produces edge sequences and instructions</li>
 *   <li>{@code direct} — straight line, no instructions, geometry included in polyline</li>
 *   <li>{@code coordinates} — imported track, no instructions, geometry included in polyline</li>
 * </ul>
 * Non-routable segments (direct/coordinates) produce a synthetic CONTINUE_ON_STREET
 * instruction with {@code tbt_available: false} so the client can alert the user.
 * The preceding instruction also carries {@code next_segment_type}, and the first
 * instruction after the gap carries {@code tbt_resumed: true}.
 * <p>
 * Each routable segment is routed individually (no merging of same-profile segments).
 * Edge chains are stitched at segment boundaries with deduplication and micro-routing
 * to bridge any gaps between disconnected edges.
 */
public class RouteInstructionGenerator {

    private static final Logger LOGGER = LoggerFactory.getLogger(RouteInstructionGenerator.class);

    private final GraphHopper graphHopper;
    private final BaseGraph baseGraph;
    private final EncodedValueLookup encodedValueLookup;
    private final TranslationMap translationMap;

    public RouteInstructionGenerator(GraphHopper graphHopper, BaseGraph baseGraph,
                                     EncodedValueLookup encodedValueLookup,
                                     TranslationMap translationMap) {
        this.graphHopper = graphHopper;
        this.baseGraph = baseGraph;
        this.encodedValueLookup = encodedValueLookup;
        this.translationMap = translationMap;
    }

    /**
     * Result of instruction generation: instructions + full route polyline.
     */
    public static class Result {
        public final InstructionList instructions;
        public final PointList polyline;

        public Result(InstructionList instructions, PointList polyline) {
            this.instructions = instructions;
            this.polyline = polyline;
        }
    }

    /**
     * Generate a continuous instruction list and polyline for the entire route.
     */
    public Result generate(TrailmapInstructionRequest request) {
        Map<String, TrailmapInstructionRequest.Coordinates> waypointMap = new HashMap<>();
        for (TrailmapInstructionRequest.Waypoint wp : request.getWaypoints()) {
            waypointMap.put(wp.getId(), wp.getCoordinates());
        }

        // Each routable segment becomes its own chunk (no merging).
        List<Chunk> chunks = buildChunks(request.getSegments(), waypointMap);
        LOGGER.info("Route has {} segments split into {} chunks", request.getSegments().size(), chunks.size());

        // Resolve instruction profile
        Profile instrProfile = graphHopper.getProfile(request.getInstructionProfile());
        if (instrProfile == null) {
            throw new IllegalArgumentException("Unknown instruction profile: " + request.getInstructionProfile());
        }
        Weighting weighting = graphHopper.createWeighting(instrProfile, new PMap());
        Translation tr = translationMap.getWithFallBack(Locale.forLanguageTag(request.getLocale()));

        // Snap preventions from the top-level request
        List<String> snapPreventions = request.getSnapPreventions();

        // Segment-boundary U-turn suppression via the additive defer-commit path. Controlled by
        // the internal flag SUPPRESS_BOUNDARY_UTURN (no API surface). When off, the production
        // path below runs byte-for-byte unchanged. See docs/gh_tbt_pipeline_phasing_change.md.
        if (SUPPRESS_BOUNDARY_UTURN) {
            return generateWithBoundaryUturnSuppression(chunks, weighting, tr, snapPreventions, waypointMap);
        }

        // Process each chunk: route sections produce instructions; non-routable chunks
        // produce geometry only. The full polyline is built by concatenation.
        InstructionList allInstructions = new InstructionList(tr);
        PointList fullPolyline = new PointList(128, true);

        TrailmapInstructionRequest.Coordinates nextStartOverride = null;
        // Track the last routable chunk's edge chain for stitching
        List<Integer> prevEdgeIds = null;
        PointList prevRoutePolyline = null;
        String prevProfile = null;
        // Track non-routable gap so the next routed section's first instruction gets tbt_resumed
        String pendingResumeType = null;

        for (int ci = 0; ci < chunks.size(); ci++) {
            Chunk chunk = chunks.get(ci);

            if (chunk.routableSection != null) {
                // --- Routable chunk: route, extract edges, generate instructions ---
                Section section = chunk.routableSection;

                // Each segment uses ONLY its own initial_heading (null → free start). The server
                // respects the per-segment route data from the API exactly; it does not replicate
                // the client UI nor chain a heading from the previous segment.
                Double heading = section.initialHeading;
                if (nextStartOverride != null) {
                    section.points.set(0, nextStartOverride);
                }

                GHResponse response = routeSection(section, heading, snapPreventions);
                if (response.hasErrors()) {
                    throw new IllegalStateException("Routing failed for chunk " + ci + ": " +
                            response.getErrors().stream().map(Throwable::getMessage).collect(Collectors.joining(", ")));
                }

                ResponsePath responsePath = response.getBest();
                List<Integer> edgeIds = extractEdgeIds(responsePath);
                PointList routePolyline = responsePath.getPoints();

                // Stitch edge chains at boundary between consecutive routable chunks
                boolean uturnAtBoundary = false;
                double stripOverlapM = 0;
                if (prevEdgeIds != null && !prevEdgeIds.isEmpty() && !edgeIds.isEmpty()) {
                    uturnAtBoundary = stitchEdgeChains(prevEdgeIds, edgeIds, prevRoutePolyline, routePolyline,
                            fullPolyline, section.profile, snapPreventions);
                    stripOverlapM = lastStitchOverlapM;
                }

                if (!edgeIds.isEmpty()) {
                    // Build synthetic path and generate instructions
                    TrailmapInstructionRequest.Coordinates startCoord = section.points.get(0);
                    Path syntheticPath = buildSyntheticPath(edgeIds, startCoord);
                    InstructionList sectionInstructions = TrailmapInstructionsFromEdges.calcInstructions(
                            syntheticPath, baseGraph, weighting, encodedValueLookup, tr);

                    // InstructionsFromEdges cannot generate a U-turn as the first instruction
                    // of a standalone synthetic path (no previous edge to compute angle against).
                    // If stitchEdgeChains detected a U-turn at the boundary, patch the first
                    // CONTINUE_ON_STREET to U_TURN_UNKNOWN so appendInstructions() won't strip it.
                    // Also stamp _polyline_start_hint so remapInstructionGeometry anchors this
                    // instruction at the boundary seam: the synthetic-path fromNode is the FAR
                    // endpoint of the shared edge (the end opposite to the snap point) and is
                    // therefore not in the actual route polyline, so coordinate matching has no
                    // valid target and the fallback global-closest match lands at an arbitrary
                    // point — stacking subsequent instructions on top of it.
                    if (uturnAtBoundary && !sectionInstructions.isEmpty()
                            && sectionInstructions.get(0).getSign() == Instruction.CONTINUE_ON_STREET) {
                        sectionInstructions.get(0).setSign(Instruction.U_TURN_UNKNOWN);
                        // Anchor the U-turn at the snap point — the last polyline point of the
                        // previous segment, which is the physical location where the rider
                        // reverses direction. Index is fullPolyline.size() - 1 because the next
                        // segment's polyline first point will dedup against this same point in
                        // appendRoutePolyline; the gap-resumption hint uses fullPolyline.size()
                        // because there is no such overlap in that case.
                        if (fullPolyline.size() > 0) {
                            sectionInstructions.get(0).setExtraInfo("_polyline_start_hint",
                                    fullPolyline.size() - 1);
                        }
                    }

                    boolean isLastRoutableChunk = isLastRoutableChunk(chunks, ci);
                    int instrCountBefore = allInstructions.size();
                    boolean resumingAfterGap = pendingResumeType != null;
                    appendInstructions(allInstructions, sectionInstructions, fullPolyline.size(),
                            isLastRoutableChunk, resumingAfterGap, stripOverlapM);

                    // Append the route response polyline. sectionStartIdx is where the section
                    // physically starts in the full polyline (the seam vertex when the boundary
                    // point was deduplicated — NOT the pre-append size, which would then point
                    // one vertex into the section).
                    int tracePolyBefore = fullPolyline.size();
                    int sectionStartIdx = appendRoutePolyline(fullPolyline, routePolyline);
                    if (PLACEMENT_TRACE) {
                        PLACEMENT_TRACE_LOG.add(new PlacementTraceRecord(PlacementTraceRecord.ROUTED,
                                edgeIds, tracePolyBefore, fullPolyline.size(), routePolyline));
                    }

                    // Mark the first new instruction as resuming TbT after a direct/coordinates gap.
                    // Store the section start index as a hint for remapInstructionGeometry():
                    // the synthetic path's first graph node may not be in the route response
                    // polyline (it's before the snapped start), so coordinate matching would fail.
                    if (resumingAfterGap && allInstructions.size() > instrCountBefore) {
                        Instruction firstNew = allInstructions.get(instrCountBefore);
                        firstNew.setExtraInfo("tbt_resumed", true);
                        firstNew.setExtraInfo("prev_segment_type", pendingResumeType);
                        firstNew.setExtraInfo("_polyline_start_hint", sectionStartIdx);
                        pendingResumeType = null;
                    }
                }

                // Track for next stitching
                prevEdgeIds = edgeIds;
                prevRoutePolyline = routePolyline;
                prevProfile = section.profile;

                // Start the next segment exactly where this one ended (polyline continuity).
                // No heading chaining — heading comes only from the next segment's own data.
                PointList pts = routePolyline;
                nextStartOverride = (pts.size() >= 1)
                        ? coordOf(pts.getLat(pts.size() - 1), pts.getLon(pts.size() - 1))
                        : null;

            } else {
                // --- Non-routable chunk (direct or coordinates) ---
                // Flag the preceding instruction so client knows TbT is about to end
                if (!allInstructions.isEmpty()) {
                    Instruction lastInstr = allInstructions.get(allInstructions.size() - 1);
                    lastInstr.setExtraInfo("next_segment_type", chunk.segmentType);
                }

                // Build gap geometry and create synthetic "entering direct" instruction
                PointList gapGeometry = buildGapGeometry(chunk, waypointMap);
                Instruction directInstr = new Instruction(Instruction.CONTINUE_ON_STREET, "", gapGeometry);
                directInstr.setDistance(calcGapDistance(gapGeometry));
                directInstr.setExtraInfo("segment_type", chunk.segmentType);
                directInstr.setExtraInfo("tbt_available", false);
                directInstr.setExtraInfo("confirm_reason", "entering_direct_segment");
                allInstructions.add(directInstr);

                // Append geometry to polyline
                appendGapPolyline(fullPolyline, gapGeometry);

                // Track that the next routable chunk should be marked as resuming TbT
                pendingResumeType = chunk.segmentType;

                // Reset edge tracking across non-routable gaps
                prevEdgeIds = null;
                prevRoutePolyline = null;
                prevProfile = null;

                // Across a non-routable gap the next segment starts fresh from its own waypoint.
                nextStartOverride = null;
            }
        }

        // Ensure the instruction list ends with FINISH. When the route ends with a
        // non-routable segment, no routable chunk produces a FINISH instruction.
        if (!allInstructions.isEmpty()
                && allInstructions.get(allInstructions.size() - 1).getSign() != Instruction.FINISH) {
            PointList finishPt = new PointList(1, true);
            if (fullPolyline.size() > 0) {
                int last = fullPolyline.size() - 1;
                finishPt.add(fullPolyline.getLat(last), fullPolyline.getLon(last),
                        fullPolyline.is3D() ? fullPolyline.getEle(last) : Double.NaN);
            }
            allInstructions.add(new Instruction(Instruction.FINISH, "", finishPt));
        }

        // Remap instruction geometry to use the full route polyline instead of the
        // synthetic path geometry. The synthetic path uses full graph edges (node-to-node),
        // so its geometry extends beyond the actual snapped route endpoints.
        remapInstructionGeometry(allInstructions, fullPolyline);

        LOGGER.info("Generated {} instructions, polyline has {} points", allInstructions.size(), fullPolyline.size());
        return new Result(allInstructions, fullPolyline);
    }

    // ============================================================================
    // Experimental: segment-boundary U-turn suppression (Option C, defer-commit).
    // Additive path, gated by request.suppressBoundaryUturn. The production generate()
    // body above is left unchanged. See docs/gh_tbt_pipeline_phasing_change.md.
    // ============================================================================

    /**
     * Internal feature flag (no API surface) for segment-boundary U-turn suppression.
     * On by default. Package-private and non-final only so tests can A/B it; production
     * never flips it. Set false to fall back to the legacy per-segment path.
     */
    static boolean SUPPRESS_BOUNDARY_UTURN = true;

    /** One-way traversed overshoot (m) at/below which a boundary U-turn is treated as a snap artifact. Tunable. */
    private static final double BOUNDARY_UTURN_MAX_TRAVERSED_M = 15.0;
    /** Tolerance (m) for locating the junction node within a segment polyline. */
    private static final double JUNCTION_MATCH_TOLERANCE_M = 0.5;

    /** A routable section held (not yet committed) so it can merge with the next one. */
    private static class PendingSection {
        List<Integer> edgeIds;
        PointList routePolyline;
        TrailmapInstructionRequest.Coordinates startCoord;
        InstructionList sectionInstructions;
        boolean resumingAfterGap;
        String resumeType;
        boolean needsUturnHint;   // long boundary U-turn: stamp anchor hint at commit
        double stitchOverlapM;    // same-direction shared boundary edge length (strip-merge exclusion)
    }

    /** A detected short same-edge boundary U-turn (snap artifact eligible for erasure). */
    private static class BoundarySeam {
        int junctionNode;
        int prevPolyJunctionIdx;   // last occurrence of junction in the previous section's polyline
        int currPolyJunctionIdx;   // first occurrence of junction in the current section's polyline
        double overshootM;
    }

    /**
     * Defer-commit variant of generate(): a routable section is held until the next one
     * is seen. If their seam is a short snap-artifact U-turn, they are merged (overshoot
     * erased from edges + geometry) and instructions regenerated over the merged chain —
     * so the correct continuing turn is produced by normal turn logic and no rollback of
     * already-committed state is needed. All other seams commit and stitch exactly as the
     * production path does.
     */
    private Result generateWithBoundaryUturnSuppression(
            List<Chunk> chunks, Weighting weighting, Translation tr,
            List<String> snapPreventions,
            Map<String, TrailmapInstructionRequest.Coordinates> waypointMap) {

        InstructionList allInstructions = new InstructionList(tr);
        PointList fullPolyline = new PointList(128, true);

        TrailmapInstructionRequest.Coordinates nextStartOverride = null;
        String pendingResumeType = null;
        PendingSection pending = null;

        for (Chunk chunk : chunks) {
            if (chunk.routableSection != null) {
                Section section = chunk.routableSection;

                // Heading from this segment's own data only — never chained (see generate()).
                Double heading = section.initialHeading;
                if (nextStartOverride != null) {
                    section.points.set(0, nextStartOverride);
                }

                GHResponse response = routeSection(section, heading, snapPreventions);
                if (response.hasErrors()) {
                    throw new IllegalStateException("Routing failed: " +
                            response.getErrors().stream().map(Throwable::getMessage).collect(Collectors.joining(", ")));
                }
                ResponsePath responsePath = response.getBest();
                List<Integer> edgeIds = extractEdgeIds(responsePath);
                PointList routePolyline = responsePath.getPoints();

                // Try to merge this section into the held one at a short boundary U-turn.
                boolean didMerge = false;
                if (pending != null && pending.edgeIds != null && !pending.edgeIds.isEmpty() && !edgeIds.isEmpty()) {
                    BoundarySeam seam = classifyShortBoundaryUturn(
                            pending.edgeIds, edgeIds, pending.routePolyline, routePolyline);
                    if (seam != null) {
                        LOGGER.debug("Boundary U-turn snap artifact at node {} (overshoot {}m) — merging segments",
                                seam.junctionNode, String.format("%.1f", seam.overshootM));
                        pending = mergeSections(pending, edgeIds, routePolyline, seam, weighting, tr);
                        didMerge = true;
                    }
                }

                if (!didMerge) {
                    // Commit the previously held section, then process this one as the new held section.
                    List<Integer> prevEdgeIds = null;
                    PointList prevRoutePolyline = null;
                    if (pending != null) {
                        commitPending(allInstructions, fullPolyline, pending, false);
                        prevEdgeIds = pending.edgeIds;
                        prevRoutePolyline = pending.routePolyline;
                    }

                    boolean uturnAtBoundary = false;
                    double stripOverlapM = 0;
                    if (prevEdgeIds != null && !prevEdgeIds.isEmpty() && !edgeIds.isEmpty()) {
                        uturnAtBoundary = stitchEdgeChains(prevEdgeIds, edgeIds, prevRoutePolyline, routePolyline,
                                fullPolyline, section.profile, snapPreventions);
                        stripOverlapM = lastStitchOverlapM;
                    }

                    PendingSection ps = new PendingSection();
                    ps.edgeIds = edgeIds;
                    ps.routePolyline = routePolyline;
                    ps.startCoord = section.points.get(0);
                    ps.resumingAfterGap = pendingResumeType != null;
                    ps.resumeType = pendingResumeType;
                    ps.stitchOverlapM = stripOverlapM;
                    if (!edgeIds.isEmpty()) {
                        Path syntheticPath = buildSyntheticPath(edgeIds, section.points.get(0));
                        ps.sectionInstructions = TrailmapInstructionsFromEdges.calcInstructions(
                                syntheticPath, baseGraph, weighting, encodedValueLookup, tr);
                        if (uturnAtBoundary && !ps.sectionInstructions.isEmpty()
                                && ps.sectionInstructions.get(0).getSign() == Instruction.CONTINUE_ON_STREET) {
                            ps.sectionInstructions.get(0).setSign(Instruction.U_TURN_UNKNOWN);
                            ps.needsUturnHint = true;
                        }
                    } else {
                        ps.sectionInstructions = new InstructionList(tr);
                    }
                    pending = ps;
                    pendingResumeType = null;
                }

                // Start the next segment where this one ended (polyline continuity); no heading chaining.
                nextStartOverride = (routePolyline.size() >= 1)
                        ? coordOf(routePolyline.getLat(routePolyline.size() - 1),
                                  routePolyline.getLon(routePolyline.size() - 1))
                        : null;

            } else {
                // Non-routable chunk: flush the held section first, then emit the gap.
                if (pending != null) {
                    commitPending(allInstructions, fullPolyline, pending, false);
                    pending = null;
                }
                if (!allInstructions.isEmpty()) {
                    allInstructions.get(allInstructions.size() - 1)
                            .setExtraInfo("next_segment_type", chunk.segmentType);
                }
                PointList gapGeometry = buildGapGeometry(chunk, waypointMap);
                Instruction directInstr = new Instruction(Instruction.CONTINUE_ON_STREET, "", gapGeometry);
                directInstr.setDistance(calcGapDistance(gapGeometry));
                directInstr.setExtraInfo("segment_type", chunk.segmentType);
                directInstr.setExtraInfo("tbt_available", false);
                directInstr.setExtraInfo("confirm_reason", "entering_direct_segment");
                allInstructions.add(directInstr);
                appendGapPolyline(fullPolyline, gapGeometry);

                pendingResumeType = chunk.segmentType;
                nextStartOverride = null;
            }
        }

        // Commit the final held section (it is the last chunk → keep its FINISH).
        if (pending != null) {
            commitPending(allInstructions, fullPolyline, pending, true);
        }

        if (!allInstructions.isEmpty()
                && allInstructions.get(allInstructions.size() - 1).getSign() != Instruction.FINISH) {
            PointList finishPt = new PointList(1, true);
            if (fullPolyline.size() > 0) {
                int last = fullPolyline.size() - 1;
                finishPt.add(fullPolyline.getLat(last), fullPolyline.getLon(last),
                        fullPolyline.is3D() ? fullPolyline.getEle(last) : Double.NaN);
            }
            allInstructions.add(new Instruction(Instruction.FINISH, "", finishPt));
        }

        remapInstructionGeometry(allInstructions, fullPolyline);
        LOGGER.info("Generated {} instructions (boundary-uturn-suppression path), polyline has {} points",
                allInstructions.size(), fullPolyline.size());
        return new Result(allInstructions, fullPolyline);
    }

    /**
     * Commit a held section: append its instructions (stripping leading CONTINUE / FINISH per the
     * same rules as the production path) and its route polyline. Stamps resume / U-turn anchor hints
     * using the commit-time polyline size so geometry remapping anchors correctly.
     */
    private void commitPending(InstructionList allInstructions, PointList fullPolyline,
                               PendingSection pending, boolean isLastRoutableChunk) {
        if (pending.edgeIds == null || pending.edgeIds.isEmpty()
                || pending.sectionInstructions == null || pending.sectionInstructions.isEmpty()) {
            return; // degenerate / empty section — nothing to commit (matches production skip)
        }
        if (pending.needsUturnHint && fullPolyline.size() > 0
                && pending.sectionInstructions.get(0).getSign() == Instruction.U_TURN_UNKNOWN) {
            pending.sectionInstructions.get(0).setExtraInfo("_polyline_start_hint", fullPolyline.size() - 1);
        }
        int instrCountBefore = allInstructions.size();
        appendInstructions(allInstructions, pending.sectionInstructions, fullPolyline.size(),
                isLastRoutableChunk, pending.resumingAfterGap, pending.stitchOverlapM);
        int tracePolyBefore = fullPolyline.size();
        int sectionStartIdx = appendRoutePolyline(fullPolyline, pending.routePolyline);
        if (PLACEMENT_TRACE) {
            PLACEMENT_TRACE_LOG.add(new PlacementTraceRecord(PlacementTraceRecord.ROUTED,
                    pending.edgeIds, tracePolyBefore, fullPolyline.size(), pending.routePolyline));
        }
        // Anchor the resume marker at the section's PHYSICAL start (the seam vertex when the
        // boundary point was deduplicated) — the pre-append polyline size would point one vertex
        // into the section, shifting the "TbT resumes" anchor by the first polyline leg.
        if (pending.resumingAfterGap && allInstructions.size() > instrCountBefore) {
            Instruction firstNew = allInstructions.get(instrCountBefore);
            firstNew.setExtraInfo("tbt_resumed", true);
            firstNew.setExtraInfo("prev_segment_type", pending.resumeType);
            firstNew.setExtraInfo("_polyline_start_hint", sectionStartIdx);
        }
    }

    /**
     * Classify the seam between a held section and the current one. Returns a non-null
     * BoundarySeam only for a SHORT same-edge opposite-direction overlap (snap artifact):
     * both sections ≥2 edges, share the boundary edge traversed in opposite directions,
     * traversed overshoot ≤ {@link #BOUNDARY_UTURN_MAX_TRAVERSED_M}, and the junction node
     * is locatable in both polylines. Returns null otherwise (preserve current behavior:
     * long out-and-back, same-direction, disconnected, single-edge legs, ambiguous geometry).
     */
    private BoundarySeam classifyShortBoundaryUturn(List<Integer> prevEdgeIds, List<Integer> currEdgeIds,
                                                    PointList prevPoly, PointList currPoly) {
        if (prevEdgeIds.size() < 2 || currEdgeIds.size() < 2) return null;
        int lastPrev = prevEdgeIds.get(prevEdgeIds.size() - 1);
        int firstCurr = currEdgeIds.get(0);
        if (lastPrev != firstCurr) return null; // same-edge boundary only (bounds to single-piece overshoot)
        if (prevPoly == null || prevPoly.isEmpty() || currPoly == null || currPoly.isEmpty()) return null;

        EdgeIteratorState shared = baseGraph.getEdgeIteratorState(lastPrev, Integer.MIN_VALUE);
        int sBase = shared.getBaseNode(), sAdj = shared.getAdjNode();

        EdgeIteratorState prevPenult = baseGraph.getEdgeIteratorState(
                prevEdgeIds.get(prevEdgeIds.size() - 2), Integer.MIN_VALUE);
        EdgeIteratorState currSecond = baseGraph.getEdgeIteratorState(currEdgeIds.get(1), Integer.MIN_VALUE);
        int prevEntry = (prevPenult.getBaseNode() == sBase || prevPenult.getAdjNode() == sBase) ? sBase
                : ((prevPenult.getBaseNode() == sAdj || prevPenult.getAdjNode() == sAdj) ? sAdj : -1);
        int currExit = (currSecond.getBaseNode() == sBase || currSecond.getAdjNode() == sBase) ? sBase
                : ((currSecond.getBaseNode() == sAdj || currSecond.getAdjNode() == sAdj) ? sAdj : -1);
        if (prevEntry == -1 || prevEntry != currExit) return null; // not a U-turn (both pivot on same node)

        int junction = prevEntry;
        NodeAccess na = baseGraph.getNodeAccess();
        double jLat = na.getLat(junction), jLon = na.getLon(junction);
        double snapLat = prevPoly.getLat(prevPoly.size() - 1), snapLon = prevPoly.getLon(prevPoly.size() - 1);
        double overshoot = DistanceCalcEarth.DIST_EARTH.calcDist(jLat, jLon, snapLat, snapLon);
        if (overshoot > BOUNDARY_UTURN_MAX_TRAVERSED_M) return null; // genuine out-and-back → preserve

        int j1 = -1;
        for (int i = prevPoly.size() - 1; i >= 0; i--) {
            if (DistanceCalcEarth.DIST_EARTH.calcDist(prevPoly.getLat(i), prevPoly.getLon(i), jLat, jLon)
                    < JUNCTION_MATCH_TOLERANCE_M) { j1 = i; break; }
        }
        int j2 = -1;
        for (int i = 0; i < currPoly.size(); i++) {
            if (DistanceCalcEarth.DIST_EARTH.calcDist(currPoly.getLat(i), currPoly.getLon(i), jLat, jLon)
                    < JUNCTION_MATCH_TOLERANCE_M) { j2 = i; break; }
        }
        if (j1 < 0 || j2 < 0) return null; // junction not on a polyline vertex within tolerance → decline

        BoundarySeam seam = new BoundarySeam();
        seam.junctionNode = junction;
        seam.prevPolyJunctionIdx = j1;
        seam.currPolyJunctionIdx = j2;
        seam.overshootM = overshoot;
        return seam;
    }

    /**
     * Merge a held section with the current one across a short boundary U-turn: drop both
     * copies of the overshoot edge, splice a spur-free polyline at the junction, and regenerate
     * instructions over the merged chain (yielding the correct continuing turn, no U-turn).
     * The merged section inherits the held section's start coord and resume status, and becomes
     * the new held section (so a following seam classifies against the merged edge chain).
     */
    private PendingSection mergeSections(PendingSection pending, List<Integer> currEdgeIds,
                                         PointList currPoly, BoundarySeam seam,
                                         Weighting weighting, Translation tr) {
        List<Integer> mergedEdges = new ArrayList<>(pending.edgeIds.subList(0, pending.edgeIds.size() - 1));
        mergedEdges.addAll(currEdgeIds.subList(1, currEdgeIds.size()));

        PointList prevPoly = pending.routePolyline;
        boolean is3D = prevPoly.is3D();
        PointList mergedPoly = new PointList(prevPoly.size() + currPoly.size(), is3D);
        for (int i = 0; i <= seam.prevPolyJunctionIdx; i++) {
            mergedPoly.add(prevPoly.getLat(i), prevPoly.getLon(i), is3D ? prevPoly.getEle(i) : Double.NaN);
        }
        for (int i = seam.currPolyJunctionIdx + 1; i < currPoly.size(); i++) {
            mergedPoly.add(currPoly.getLat(i), currPoly.getLon(i),
                    currPoly.is3D() ? currPoly.getEle(i) : Double.NaN);
        }

        InstructionList mergedInstr;
        if (!mergedEdges.isEmpty()) {
            Path mergedPath = buildSyntheticPath(mergedEdges, pending.startCoord);
            mergedInstr = TrailmapInstructionsFromEdges.calcInstructions(
                    mergedPath, baseGraph, weighting, encodedValueLookup, tr);
        } else {
            mergedInstr = new InstructionList(tr);
        }

        PendingSection ps = new PendingSection();
        ps.edgeIds = mergedEdges;
        ps.routePolyline = mergedPoly;
        ps.startCoord = pending.startCoord;
        ps.sectionInstructions = mergedInstr;
        ps.resumingAfterGap = pending.resumingAfterGap;
        ps.resumeType = pending.resumeType;
        ps.needsUturnHint = false;
        // The merged chain still starts with the held section's edges, including any stitched
        // shared context edge — the strip-merge exclusion carries over.
        ps.stitchOverlapM = pending.stitchOverlapM;
        return ps;
    }

    // ---- Chunk building ----

    /**
     * A chunk is either a single routable segment (one FollowRoads segment)
     * or a single non-routable segment (Direct / Coordinates).
     * No merging of same-profile segments — each routable segment is its own chunk.
     */
    static class Chunk {
        Section routableSection;    // non-null for routable chunks
        String segmentType;         // "direct" or "coordinates" for non-routable chunks
        TrailmapInstructionRequest.Segment nonRoutableSegment; // the original segment for non-routable
    }

    List<Chunk> buildChunks(List<TrailmapInstructionRequest.Segment> segments,
                            Map<String, TrailmapInstructionRequest.Coordinates> waypointMap) {
        List<Chunk> chunks = new ArrayList<>();

        for (TrailmapInstructionRequest.Segment seg : segments) {
            if (!seg.isRoutable()) {
                // Add non-routable chunk
                Chunk c = new Chunk();
                c.segmentType = seg.getType();
                c.nonRoutableSegment = seg;
                chunks.add(c);
            } else {
                // Each routable segment becomes its own section — no merging
                Section section = new Section();
                section.profile = seg.getProfile();
                section.customModel = seg.getCustomModel();
                section.initialHeading = seg.getInitialHeading();
                section.headingPenalty = seg.getHeadingPenalty();
                section.points = new ArrayList<>();
                TrailmapInstructionRequest.Coordinates start = waypointMap.get(seg.getStart());
                TrailmapInstructionRequest.Coordinates end = waypointMap.get(seg.getEnd());
                if (start == null || end == null) {
                    throw new IllegalArgumentException("FollowRoads segment references unknown waypoint id(s): start='"
                            + seg.getStart() + "'" + (start == null ? " (unresolved)" : "")
                            + ", end='" + seg.getEnd() + "'" + (end == null ? " (unresolved)" : ""));
                }
                section.points.add(start);
                if (seg.getViaPoints() != null) {
                    section.points.addAll(seg.getViaPoints());
                }
                section.points.add(end);

                Chunk c = new Chunk();
                c.routableSection = section;
                chunks.add(c);
            }
        }
        return chunks;
    }

    // ---- Routing ----

    private GHResponse routeSection(Section section, Double heading, List<String> snapPreventions) {
        GHRequest request = new GHRequest();
        for (TrailmapInstructionRequest.Coordinates coord : section.points) {
            request.addPoint(new GHPoint(coord.getLat(), coord.getLng()));
        }
        request.setProfile(section.profile);
        request.setPathDetails(List.of("edge_id"));
        request.putHint("instructions", false);
        request.putHint("calc_points", true);

        if (section.customModel != null) {
            request.setCustomModel(section.customModel);
        }

        if (snapPreventions != null && !snapPreventions.isEmpty()) {
            request.setSnapPreventions(snapPreventions);
        }

        if (section.headingPenalty != null) {
            request.putHint("heading_penalty", section.headingPenalty);
        }

        if (heading != null && !heading.isNaN()) {
            List<Double> headings = new ArrayList<>();
            headings.add(heading);
            for (int i = 1; i < section.points.size(); i++) {
                headings.add(Double.NaN);
            }
            request.setHeadings(headings);
        }

        return graphHopper.route(request);
    }

    private static final double VIA_POINT_UTURN_MAX_EDGE_DISTANCE = 40.0; // meters

    private List<Integer> extractEdgeIds(ResponsePath path) {
        List<PathDetail> edgeDetails = path.getPathDetails().get("edge_id");
        if (edgeDetails == null || edgeDetails.isEmpty()) {
            return new ArrayList<>();
        }
        // First pass: collect raw edge IDs (no dedup yet)
        List<Integer> rawEdgeIds = new ArrayList<>(edgeDetails.size());
        for (PathDetail detail : edgeDetails) {
            rawEdgeIds.add((Integer) detail.getValue());
        }

        // Second pass: resolve consecutive duplicates.
        // Multi-waypoint GH routes can produce the same edge ID twice at leg
        // boundaries. There are two distinct cases:
        //
        // 1. Same-direction snap: via-point snaps to the middle of an edge.
        //    The edge appears as the last edge of leg N and the first edge of
        //    leg N+1, both traversed in the same direction. Remove one copy.
        //
        // 2. U-turn at via-point: the route reaches the via-point at one end
        //    of an edge and comes back. The edge is traversed in opposite
        //    directions. Detected by checking whether the edges before and
        //    after the duplicate connect to the SAME node of the duplicate
        //    (both enter/exit from the same side → U-turn).
        //    - Short edge (≤ threshold): snap artifact, remove both copies.
        //    - Long edge: intentional out-and-back, keep both copies so
        //      buildSyntheticPath walks the U-turn and InstructionsFromEdges
        //      generates a U-turn instruction.
        List<Integer> edgeIds = new ArrayList<>(rawEdgeIds.size());
        for (int i = 0; i < rawEdgeIds.size(); i++) {
            int edgeId = rawEdgeIds.get(i);
            if (i + 1 < rawEdgeIds.size() && rawEdgeIds.get(i + 1) == edgeId) {
                // Consecutive duplicate found — classify it
                boolean isUturn = false;
                if (i > 0 && i + 2 < rawEdgeIds.size()) {
                    isUturn = isViaPointUturn(rawEdgeIds.get(i - 1), edgeId, rawEdgeIds.get(i + 2));
                }

                if (isUturn) {
                    double edgeDist = baseGraph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE).getDistance();
                    if (edgeDist <= VIA_POINT_UTURN_MAX_EDGE_DISTANCE) {
                        // Short U-turn: snap artifact, remove both copies
                        LOGGER.debug("Removing short U-turn duplicate edge {} ({}m) at via-point boundary",
                                edgeId, String.format("%.1f", edgeDist));
                        i++; // skip the second copy too
                    } else {
                        // Long U-turn: intentional, keep both copies
                        LOGGER.debug("Keeping U-turn duplicate edge {} ({}m) at via-point boundary",
                                edgeId, String.format("%.1f", edgeDist));
                        edgeIds.add(edgeId);
                        edgeIds.add(edgeId);
                        i++; // skip the second copy (already added)
                    }
                } else {
                    // Same-direction snap: remove one copy
                    LOGGER.debug("Removing same-direction duplicate edge {} at via-point boundary", edgeId);
                    i++; // skip the second copy
                    edgeIds.add(edgeId);
                }
            } else {
                edgeIds.add(edgeId);
            }
        }
        return edgeIds;
    }

    /**
     * Determine whether a consecutive duplicate edge at a via-point boundary is a U-turn.
     * Checks if the edges before and after the duplicate connect to the SAME node of the
     * duplicate edge (both enter/exit from the same side).
     */
    private boolean isViaPointUturn(int prevEdgeId, int dupEdgeId, int nextEdgeId) {
        EdgeIteratorState dupEdge = baseGraph.getEdgeIteratorState(dupEdgeId, Integer.MIN_VALUE);
        int nodeA = dupEdge.getBaseNode();
        int nodeB = dupEdge.getAdjNode();

        EdgeIteratorState prevEdge = baseGraph.getEdgeIteratorState(prevEdgeId, Integer.MIN_VALUE);
        Set<Integer> prevNodes = Set.of(prevEdge.getBaseNode(), prevEdge.getAdjNode());
        int entryNode = -1;
        if (prevNodes.contains(nodeA)) entryNode = nodeA;
        else if (prevNodes.contains(nodeB)) entryNode = nodeB;

        EdgeIteratorState nextEdge = baseGraph.getEdgeIteratorState(nextEdgeId, Integer.MIN_VALUE);
        Set<Integer> nextNodes = Set.of(nextEdge.getBaseNode(), nextEdge.getAdjNode());
        int exitNode = -1;
        if (nextNodes.contains(nodeA)) exitNode = nodeA;
        else if (nextNodes.contains(nodeB)) exitNode = nodeB;

        // U-turn: both prev and next connect to the same node of the duplicate edge
        return entryNode != -1 && entryNode == exitNode;
    }

    // ---- Edge chain stitching ----

    /**
     * Length (m) of the same-direction shared boundary edge kept in the CURRENT section's chain
     * by the last {@link #stitchEdgeChains} call (0 when the seam was not a case-1 share). The
     * previous instruction's synthetic distance already covers that edge, so when the current
     * section's initial CONTINUE is stripped, its merged-in distance must exclude this overlap.
     * Reset at the start of every stitchEdgeChains call; consumed by the caller right after.
     */
    private double lastStitchOverlapM = 0;

    /**
     * Stitch edge chains at segment boundaries between two consecutive routable chunks.
     * Three cases:
     * 1. Same edge at boundary → deduplicate (remove last edge of prev or first of current)
     * 2. Edges share a node → direct concatenation (works naturally)
     * 3. Edges don't connect → micro-route between the snapped end of segment N and
     *    snapped start of segment N+1 to bridge the gap.
     * Note: in case 3, the caller's {@code currentEdgeIds} list is mutated in place to prepend the bridging edges.
     */
    private boolean stitchEdgeChains(List<Integer> prevEdgeIds, List<Integer> currentEdgeIds,
                                   PointList prevPolyline, PointList currentPolyline,
                                   PointList fullPolyline, String profile,
                                   List<String> snapPreventions) {
        lastStitchOverlapM = 0;
        if (prevEdgeIds.isEmpty() || currentEdgeIds.isEmpty()) return false;

        int lastPrevEdge = prevEdgeIds.get(prevEdgeIds.size() - 1);
        int firstCurrentEdge = currentEdgeIds.get(0);

        if (lastPrevEdge == firstCurrentEdge) {
            // Same edge at boundary — could be a true duplicate (same direction, snap mid-edge)
            // or a U-turn (opposite direction). Distinguish by checking where the second edge
            // of the current segment connects vs. the second-to-last edge of the previous segment.
            boolean isUturn = false;
            if (currentEdgeIds.size() >= 2 && prevEdgeIds.size() >= 2) {
                int secondToLastPrev = prevEdgeIds.get(prevEdgeIds.size() - 2);
                int secondCurrent = currentEdgeIds.get(1);
                EdgeIteratorState sharedEdge = baseGraph.getEdgeIteratorState(lastPrevEdge, Integer.MIN_VALUE);
                Set<Integer> sharedNodes = Set.of(sharedEdge.getBaseNode(), sharedEdge.getAdjNode());

                // Find which node of the shared edge connects to the previous segment's second-to-last edge
                EdgeIteratorState prevPenult = baseGraph.getEdgeIteratorState(secondToLastPrev, Integer.MIN_VALUE);
                Set<Integer> prevPenultNodes = Set.of(prevPenult.getBaseNode(), prevPenult.getAdjNode());
                int prevEntryNode = -1;
                for (int n : sharedNodes) {
                    if (prevPenultNodes.contains(n)) { prevEntryNode = n; break; }
                }

                // Find which node of the shared edge connects to the current segment's second edge
                EdgeIteratorState currSecond = baseGraph.getEdgeIteratorState(secondCurrent, Integer.MIN_VALUE);
                Set<Integer> currSecondNodes = Set.of(currSecond.getBaseNode(), currSecond.getAdjNode());
                int currExitNode = -1;
                for (int n : sharedNodes) {
                    if (currSecondNodes.contains(n)) { currExitNode = n; break; }
                }

                // If both segments connect to the SAME node of the shared edge, it's a U-turn:
                // prev enters from node X, current exits toward node X (going back the way it came)
                isUturn = prevEntryNode != -1 && prevEntryNode == currExitNode;
            }

            if (isUturn) {
                // U-turn: keep the shared edge in both segments (traversed in opposite directions)
                LOGGER.debug("U-turn detected on boundary edge {} between segments — keeping both", lastPrevEdge);
                return true;
            } else {
                // Same direction: keep the shared edge in seg2's edge list so that
                // InstructionsFromEdges has the context to generate a turn instruction
                // at the transition from the shared edge to seg2's next edge.
                // The resulting first CONTINUE_ON_STREET is stripped by appendInstructions(),
                // and remapInstructionGeometry() corrects any distance overlap.
                // Record the overlap: the previous instruction's synthetic distance already
                // covers this edge, so the strip-merge must exclude it — otherwise every such
                // boundary inflates _cum_route_m by one shared-edge length (in Lapland, up to
                // kilometres), corrupting the matcher's occurrence disambiguation.
                lastStitchOverlapM = baseGraph.getEdgeIteratorState(lastPrevEdge, Integer.MIN_VALUE).getDistance();
                LOGGER.debug("Shared boundary edge {} between segments — keeping for instruction context", lastPrevEdge);
            }
            return false;
        }

        // Check if edges share a node (case 2)
        EdgeIteratorState prevEdge = baseGraph.getEdgeIteratorState(lastPrevEdge, Integer.MIN_VALUE);
        EdgeIteratorState currEdge = baseGraph.getEdgeIteratorState(firstCurrentEdge, Integer.MIN_VALUE);
        Set<Integer> prevNodes = Set.of(prevEdge.getBaseNode(), prevEdge.getAdjNode());
        boolean connected = prevNodes.contains(currEdge.getBaseNode()) || prevNodes.contains(currEdge.getAdjNode());

        if (connected) {
            // Case 2: Edges share a node — natural concatenation, nothing to do
            LOGGER.debug("Boundary edges {} and {} share a node, no bridging needed", lastPrevEdge, firstCurrentEdge);
            return false;
        }

        // Case 3: Edges don't connect — micro-route to bridge the gap
        LOGGER.info("Boundary edges {} and {} are disconnected, micro-routing to bridge", lastPrevEdge, firstCurrentEdge);

        double fromLat = prevPolyline.getLat(prevPolyline.size() - 1);
        double fromLon = prevPolyline.getLon(prevPolyline.size() - 1);
        double toLat = currentPolyline.getLat(0);
        double toLon = currentPolyline.getLon(0);

        GHRequest bridgeReq = new GHRequest(fromLat, fromLon, toLat, toLon);
        bridgeReq.setProfile(profile);
        bridgeReq.setPathDetails(List.of("edge_id"));
        bridgeReq.putHint("instructions", false);
        bridgeReq.putHint("calc_points", true);
        if (snapPreventions != null && !snapPreventions.isEmpty()) {
            bridgeReq.setSnapPreventions(snapPreventions);
        }

        GHResponse bridgeRsp = graphHopper.route(bridgeReq);
        if (bridgeRsp.hasErrors()) {
            LOGGER.warn("Micro-route bridging failed: {}", bridgeRsp.getErrors());
            return false;
        }

        ResponsePath bridgePath = bridgeRsp.getBest();
        List<Integer> bridgeEdgeIds = extractEdgeIds(bridgePath);
        PointList bridgePolyline = bridgePath.getPoints();

        if (!bridgeEdgeIds.isEmpty()) {
            // Deduplicate at the prev/bridge boundary
            if (bridgeEdgeIds.get(0) == lastPrevEdge) {
                bridgeEdgeIds.remove(0);
            }
            // Deduplicate at the bridge/current boundary
            if (!bridgeEdgeIds.isEmpty() && bridgeEdgeIds.get(bridgeEdgeIds.size() - 1) == firstCurrentEdge) {
                bridgeEdgeIds.remove(bridgeEdgeIds.size() - 1);
            }

            // Insert bridging edges before the current edges
            // (We modify currentEdgeIds in place — caller's list)
            currentEdgeIds.addAll(0, bridgeEdgeIds);

            // Insert bridging polyline into the full polyline
            int tracePolyBefore = fullPolyline.size();
            appendRoutePolyline(fullPolyline, bridgePolyline);
            if (PLACEMENT_TRACE) {
                PLACEMENT_TRACE_LOG.add(new PlacementTraceRecord(PlacementTraceRecord.BRIDGE,
                        bridgeEdgeIds, tracePolyBefore, fullPolyline.size(), bridgePolyline));
            }

            LOGGER.debug("Inserted {} bridging edges between segments", bridgeEdgeIds.size());
        }
        return false;
    }

    // ---- Synthetic path ----

    private Path buildSyntheticPath(List<Integer> edgeIds, TrailmapInstructionRequest.Coordinates startCoord) {
        int fromNode = resolveFromNode(edgeIds, startCoord);

        Path path = new Path(baseGraph);
        for (int edgeId : edgeIds) {
            path.addEdge(edgeId);
        }
        path.setFromNode(fromNode);
        path.setFound(true);

        int currentNode = fromNode;
        for (int edgeId : edgeIds) {
            currentNode = walkEdge(edgeId, currentNode);
        }
        path.setEndNode(currentNode);

        return path;
    }

    /**
     * Determine the starting node of the edge chain.
     * <p>
     * When there are 2+ edges, we pick the endpoint of the first edge from which the WHOLE chain
     * walks node-to-node without a break. This deterministically disambiguates the start direction
     * even when the first two edges are parallel (share BOTH endpoints) — e.g. an out-and-back over
     * a parallel-edge pair at the route start — a case the old single-step check (does edge[1] touch
     * nodeA/nodeB?) could not resolve, falling back to proximity and sometimes picking the wrong end,
     * which produced a disconnected synthetic path (IllegalStateException in {@link #walkEdge}).
     * Crucially this is self-contained: it depends only on the final edge chain's internal
     * connectivity, so it behaves identically regardless of how the chain was assembled (plain
     * extraction, bridge-prepended, or boundary-merged).
     * <p>
     * It can only change behavior in cases that previously threw (the disconnected-walk bug) or were
     * already ambiguous; a chain with a unique connected start is resolved the same as before. We fall
     * through to proximity (same fallback as for a single edge) only when BOTH ends connect or NEITHER
     * does. "Both ends connect" is not exclusively a closed loop: a chain that returns to its start
     * with no disambiguating continuation — e.g. a bare two-edge parallel out-and-back [E, E'] over the
     * same node pair — also walks cleanly from either endpoint, so connectivity alone cannot orient it
     * and proximity decides. That residual class does NOT cause the walkEdge 500 (it connects either
     * way); the only risk is a possibly-reversed short out-and-back. Fully removing that orientation
     * ambiguity needs travel direction (oriented edge_key threaded through the build); see
     * docs/gh_tbt_resolve_from_node_parallel_out_and_back.md (Option B).
     */
    private int resolveFromNode(List<Integer> edgeIds, TrailmapInstructionRequest.Coordinates startCoord) {
        EdgeIteratorState firstEdge = baseGraph.getEdgeIteratorState(edgeIds.get(0), Integer.MIN_VALUE);
        int nodeA = firstEdge.getBaseNode();
        int nodeB = firstEdge.getAdjNode();

        // Defensive guard: same-edge U-turn pair would otherwise fall through to unreliable proximity.
        if (edgeIds.size() >= 2 && edgeIds.get(0).intValue() == edgeIds.get(1).intValue()) {
            NodeAccess nodeAccess = baseGraph.getNodeAccess();
            DistanceCalcEarth distCalc = DistanceCalcEarth.DIST_EARTH;
            double distA = distCalc.calcDist(startCoord.getLat(), startCoord.getLng(),
                    nodeAccess.getLat(nodeA), nodeAccess.getLon(nodeA));
            double distB = distCalc.calcDist(startCoord.getLat(), startCoord.getLng(),
                    nodeAccess.getLat(nodeB), nodeAccess.getLon(nodeB));
            // First traversal moves AWAY from the closer endpoint, so fromNode is the OTHER endpoint
            return distA <= distB ? nodeB : nodeA;
        }

        if (edgeIds.size() >= 2) {
            boolean aConnects = chainConnectsFrom(edgeIds, nodeA);
            boolean bConnects = chainConnectsFrom(edgeIds, nodeB);
            if (aConnects && !bConnects) {
                return nodeA;
            } else if (bConnects && !aConnects) {
                return nodeB;
            }
            // Both connect (closed loop OR a symmetric out-and-back that returns to start, e.g. a bare
            // two-edge parallel pair) or neither (broken chain) — connectivity can't orient it; use proximity.
            LOGGER.warn("Ambiguous edge connectivity for chain [{}, {}, ...] ({} edges), falling back to proximity",
                    edgeIds.get(0), edgeIds.get(1), edgeIds.size());
        }

        // Single edge or ambiguous: use proximity to start coordinate
        NodeAccess nodeAccess = baseGraph.getNodeAccess();
        DistanceCalcEarth distCalc = DistanceCalcEarth.DIST_EARTH;
        double distA = distCalc.calcDist(startCoord.getLat(), startCoord.getLng(),
                nodeAccess.getLat(nodeA), nodeAccess.getLon(nodeA));
        double distB = distCalc.calcDist(startCoord.getLat(), startCoord.getLng(),
                nodeAccess.getLat(nodeB), nodeAccess.getLon(nodeB));
        return distA <= distB ? nodeA : nodeB;
    }

    /**
     * Whether the edge chain walks node-to-node without a break when started from {@code fromNode}.
     * Same traversal as {@link #walkEdge} but returns a boolean instead of throwing.
     */
    private boolean chainConnectsFrom(List<Integer> edgeIds, int fromNode) {
        int cur = fromNode;
        for (int edgeId : edgeIds) {
            EdgeIteratorState edge = baseGraph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
            if (edge.getBaseNode() == cur) {
                cur = edge.getAdjNode();
            } else if (edge.getAdjNode() == cur) {
                cur = edge.getBaseNode();
            } else {
                return false;
            }
        }
        return true;
    }

    /**
     * Walk one edge in the chain: given the current node, return the other endpoint.
     */
    private int walkEdge(int edgeId, int currentNode) {
        EdgeIteratorState edge = baseGraph.getEdgeIteratorState(edgeId, Integer.MIN_VALUE);
        if (edge.getBaseNode() == currentNode) {
            return edge.getAdjNode();
        } else if (edge.getAdjNode() == currentNode) {
            return edge.getBaseNode();
        }
        throw new IllegalStateException("Edge " + edgeId + " is not connected to node " + currentNode +
                " (endpoints: " + edge.getBaseNode() + ", " + edge.getAdjNode() + ")");
    }

    // ---- Instruction stitching ----

    /**
     * Append section instructions to the accumulated list.
     * Strips FINISH from non-final routable chunks and adjusts interval offsets.
     * For non-first sections, strips the initial CONTINUE_ON_STREET — unless resuming
     * after a direct/coordinates gap, where that CONTINUE is the "TbT resumes" marker.
     * <p>
     * Note: instruction PointLists still contain synthetic path geometry at this point.
     * They will be remapped to the full route polyline later by remapInstructionGeometry().
     */
    private void appendInstructions(InstructionList target, InstructionList sectionInstructions,
                                    int polylineOffset, boolean isLastRoutableChunk,
                                    boolean resumingAfterGap, double stripOverlapM) {
        boolean isFirstSection = target.isEmpty();

        for (int i = 0; i < sectionInstructions.size(); i++) {
            Instruction instr = sectionInstructions.get(i);

            // Strip FINISH from non-final routable chunks; only the last routable chunk
            // should contribute a FINISH instruction.
            if (instr.getSign() == Instruction.FINISH && !isLastRoutableChunk) {
                continue;
            }

            // Strip initial CONTINUE from non-first sections to avoid duplicate at junction.
            // But keep it when resuming after a direct/coordinates gap — it marks where TbT resumes.
            if (!isFirstSection && !resumingAfterGap
                    && i == 0 && instr.getSign() == Instruction.CONTINUE_ON_STREET) {
                // Merge this instruction's distance/time into the previous instruction —
                // MINUS the stitched shared-edge overlap, which the previous instruction's
                // distance already covers. Without the subtraction, every same-direction
                // boundary double-counts the shared edge into the pre-remap distance sums,
                // corrupting _cum_route_m (the matcher's occurrence-disambiguation key) by
                // one shared-edge length per boundary (Saariselkä: 14.2 km over 26 boundaries,
                // flipping the 20.4 km KEEP_RIGHT onto the wrong pass of an out-and-back).
                if (!target.isEmpty()) {
                    Instruction prev = target.get(target.size() - 1);
                    double mergeDist = Math.max(0, instr.getDistance() - stripOverlapM);
                    double mergeRatio = instr.getDistance() > 0 ? mergeDist / instr.getDistance() : 0;
                    prev.setDistance(prev.getDistance() + mergeDist);
                    prev.setTime(prev.getTime() + Math.round(instr.getTime() * mergeRatio));
                }
                continue;
            }

            target.add(instr);
        }
    }

    /**
     * Check if the chunk at index ci is the last chunk in the list.
     * FINISH is only preserved for the truly final chunk — not just the last routable
     * chunk, since non-routable chunks after it will produce their own instructions.
     */
    private static boolean isLastRoutableChunk(List<Chunk> chunks, int ci) {
        return ci == chunks.size() - 1;
    }

    // ---- Geometry remapping ----

    /**
     * Remap instruction geometry from the synthetic path (node-to-node edges) to the
     * actual full route polyline (snapped start/end points, concatenated across sections).
     * <p>
     * The synthetic path uses full graph edges, so its geometry extends beyond the actual
     * route endpoints. This method replaces each instruction's PointList with the
     * corresponding slice of the full route polyline.
     * <p>
     * Approach: each instruction boundary in the synthetic path occurs at a graph node.
     * We find that node's location in the full polyline by coordinate matching,
     * then slice the polyline at those boundaries.
     */
    /**
     * Diagnostic hook for the marker-drift investigation (off by default; no behavior change).
     * When {@link #REMAP_DIAG} is true, {@link #remapInstructionGeometry} appends one record per
     * instruction to {@link #REMAP_DIAG_LOG} describing how its polyline anchor was chosen. The
     * decisive field is the GLOBAL best match (over the whole polyline): if the true node matches
     * near-exactly at an index BEHIND searchFrom, the monotonic forward search could not reach it.
     * Record layout (double[]):
     *   [0]=instrIndex [1]=branch [2]=sign [3]=searchFrom [4]=chosenIdx [5]=chosenDist
     *   [6]=globalBestIdx [7]=globalBestDist [8]=targetLat [9]=targetLon
     * branch: 0=first(i==0) 1=FINISH 2=hint 3=emptyPts 4=coordMatch
     */
    static boolean REMAP_DIAG = false;
    static final List<double[]> REMAP_DIAG_LOG = new ArrayList<>();

    /**
     * Test-support placement trace (same gated-hook pattern as {@link #REMAP_DIAG}; off by default,
     * no behavior change). When enabled, records — in consumption order — one record per committed
     * routed section, per stitching bridge, and per non-routable gap, each with the exact index range
     * it contributed to the full response polyline and (for routed/bridge records) the final edge
     * chain actually walked. These are append-time facts, captured before and independently of
     * {@link #remapInstructionGeometry} / {@link #matchInstructionStarts}, so the placement-validation
     * oracle (test tree) may rebuild ground-truth route geometry from them non-circularly.
     * Not thread-safe; test-only.
     */
    static boolean PLACEMENT_TRACE = false;
    static final List<PlacementTraceRecord> PLACEMENT_TRACE_LOG = new ArrayList<>();

    static final class PlacementTraceRecord {
        static final int ROUTED = 0, BRIDGE = 1, GAP = 2;
        final int type;
        /** ROUTED: final edge chain passed to buildSyntheticPath (incl. bridge-prepended edges).
         *  BRIDGE: bridge edges after boundary dedup (the ones prepended to the next ROUTED chain). */
        final List<Integer> edgeIds;
        /** fullPolyline size before/after this record's geometry was appended. */
        final int polyBefore, polyAfter;
        /** ROUTED only: the section's response-polyline endpoints (snap points). */
        final double snapStartLat, snapStartLon, snapEndLat, snapEndLon;

        PlacementTraceRecord(int type, List<Integer> edgeIds, int polyBefore, int polyAfter,
                             PointList sectionPolyline) {
            this.type = type;
            this.edgeIds = edgeIds == null ? List.of() : List.copyOf(edgeIds);
            this.polyBefore = polyBefore;
            this.polyAfter = polyAfter;
            if (sectionPolyline != null && sectionPolyline.size() > 0) {
                this.snapStartLat = sectionPolyline.getLat(0);
                this.snapStartLon = sectionPolyline.getLon(0);
                this.snapEndLat = sectionPolyline.getLat(sectionPolyline.size() - 1);
                this.snapEndLon = sectionPolyline.getLon(sectionPolyline.size() - 1);
            } else {
                this.snapStartLat = this.snapStartLon = this.snapEndLat = this.snapEndLon = Double.NaN;
            }
        }
    }

    private void remapInstructionGeometry(InstructionList instructions, PointList fullPolyline) {
        if (instructions.isEmpty() || fullPolyline.isEmpty()) return;

        // Stamp an independent, monotonic position key on each instruction BEFORE the per-instruction
        // distances are overwritten below: the cumulative synthetic route distance to each instruction.
        // matchInstructionStarts() uses it to disambiguate repeated coordinates (self-crossings /
        // out-and-backs) that bare coordinate matching cannot tell apart. It rides through
        // post-processing on the surviving instructions and is dropped at serialization (internal "_" key).
        double cumRouteM = 0;
        for (Instruction instr : instructions) {
            instr.setExtraInfo("_cum_route_m", cumRouteM);
            cumRouteM += instr.getDistance();   // pre-remap leg length (synthetic; independent of matching)
        }

        // For each instruction, determine its start index in the full polyline.
        // The first instruction starts at index 0, FINISH starts at the last point.
        // Other instructions start at graph nodes, which we find by matching coordinates.
        List<Integer> instrPolyStarts = new ArrayList<>();
        for (int i = 0; i < instructions.size(); i++) {
            Instruction instr = instructions.get(i);

            if (instr.getSign() == Instruction.FINISH) {
                instrPolyStarts.add(fullPolyline.size() - 1);
                if (REMAP_DIAG) REMAP_DIAG_LOG.add(new double[]{i, 1, instr.getSign(),
                        -1, fullPolyline.size() - 1, 0, -1, 0, Double.NaN, Double.NaN});
                continue;
            }

            if (i == 0) {
                instrPolyStarts.add(0);
                if (REMAP_DIAG) REMAP_DIAG_LOG.add(new double[]{i, 0, instr.getSign(),
                        0, 0, 0, -1, 0, Double.NaN, Double.NaN});
                continue;
            }

            // Instructions resuming after a direct/coordinates gap carry a polyline start
            // hint because the synthetic path's first graph node may not be in the route
            // response polyline (it precedes the snapped start point on the first edge).
            Object hintObj = instr.getExtraInfoJSON().get("_polyline_start_hint");
            if (hintObj instanceof Number) {
                int hint = ((Number) hintObj).intValue();
                hint = Math.max(hint, instrPolyStarts.get(instrPolyStarts.size() - 1));
                hint = Math.min(hint, fullPolyline.size() - 1);
                instrPolyStarts.add(hint);
                if (REMAP_DIAG) REMAP_DIAG_LOG.add(new double[]{i, 2, instr.getSign(),
                        instrPolyStarts.get(instrPolyStarts.size() - 2), hint, 0, -1, 0, Double.NaN, Double.NaN});
                continue;
            }

            PointList instrPts = instr.getPoints();
            if (instrPts.size() == 0) {
                instrPolyStarts.add(instrPolyStarts.get(instrPolyStarts.size() - 1));
                if (REMAP_DIAG) REMAP_DIAG_LOG.add(new double[]{i, 3, instr.getSign(),
                        instrPolyStarts.get(instrPolyStarts.size() - 1),
                        instrPolyStarts.get(instrPolyStarts.size() - 1), 0, -1, 0, Double.NaN, Double.NaN});
                continue;
            }

            // The instruction's first point is at a graph node. Find it in the full polyline.
            double targetLat = instrPts.getLat(0);
            double targetLon = instrPts.getLon(0);

            // Search forward from the previous instruction's matched index.
            // Monotonicity is guaranteed: searchFrom >= all previous matches,
            // so outbound occurrences of repeated nodes are naturally skipped.
            int searchFrom = instrPolyStarts.get(instrPolyStarts.size() - 1);
            int bestIdx = searchFrom;
            double bestDist = Double.MAX_VALUE;
            for (int pi = searchFrom; pi < fullPolyline.size(); pi++) {
                double dist = Math.abs(fullPolyline.getLat(pi) - targetLat)
                        + Math.abs(fullPolyline.getLon(pi) - targetLon);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestIdx = pi;
                }
                // Accept the first near-exact match (< ~0.55m). On out-and-back routes,
                // the same node coordinate appears multiple times in the polyline;
                // since searchFrom is past earlier occurrences, the first near-exact match
                // after searchFrom is the correct one. Continuing to scan could pick
                // up a false near-match on a parallel trail segment further along.
                // Threshold loosened from 1e-6 to tolerate floating-point drift between code paths.
                if (bestDist < 5e-6) {
                    break;
                }
            }
            if (REMAP_DIAG) {
                // Unconstrained global best over the ENTIRE polyline (diagnostic only).
                int gBestIdx = 0; double gBestDist = Double.MAX_VALUE;
                for (int pi = 0; pi < fullPolyline.size(); pi++) {
                    double dist = Math.abs(fullPolyline.getLat(pi) - targetLat)
                            + Math.abs(fullPolyline.getLon(pi) - targetLon);
                    if (dist < gBestDist) { gBestDist = dist; gBestIdx = pi; }
                }
                REMAP_DIAG_LOG.add(new double[]{i, 4, instr.getSign(),
                        searchFrom, bestIdx, bestDist, gBestIdx, gBestDist, targetLat, targetLon});
            }
            instrPolyStarts.add(bestIdx);
        }

        // Now assign polyline slices to each instruction
        boolean is3D = fullPolyline.is3D();
        for (int i = 0; i < instructions.size(); i++) {
            Instruction instr = instructions.get(i);

            if (instr.getSign() == Instruction.FINISH) {
                PointList finishPts = new PointList(1, is3D);
                int lastIdx = fullPolyline.size() - 1;
                finishPts.add(fullPolyline.getLat(lastIdx), fullPolyline.getLon(lastIdx),
                        is3D ? fullPolyline.getEle(lastIdx) : Double.NaN);
                instr.setPoints(finishPts);
                continue;
            }

            int polyStart = instrPolyStarts.get(i);
            int polyEnd = (i + 1 < instrPolyStarts.size()) ? instrPolyStarts.get(i + 1) : fullPolyline.size() - 1;

            PointList newPts = new PointList(Math.max(1, polyEnd - polyStart), is3D);
            for (int pi = polyStart; pi < polyEnd; pi++) {
                newPts.add(fullPolyline.getLat(pi), fullPolyline.getLon(pi),
                        is3D ? fullPolyline.getEle(pi) : Double.NaN);
            }
            // Ensure at least 1 point per non-FINISH instruction
            if (newPts.size() == 0 && polyStart < fullPolyline.size()) {
                newPts.add(fullPolyline.getLat(polyStart), fullPolyline.getLon(polyStart),
                        is3D ? fullPolyline.getEle(polyStart) : Double.NaN);
            }
            instr.setPoints(newPts);

            // Recalculate distance from the full polyline slice [polyStart, polyEnd] inclusive.
            // The instruction's distance covers the route from its start point to the next
            // instruction's start point, which includes the segment from polyEnd-1 to polyEnd
            // that is not part of this instruction's PointList (which is [polyStart, polyEnd)).
            double oldDist = instr.getDistance();
            if (oldDist > 1.0 && polyEnd > polyStart) {
                double newDist = 0;
                for (int pi = polyStart; pi < polyEnd; pi++) {
                    newDist += DistanceCalcEarth.DIST_EARTH.calcDist(
                            fullPolyline.getLat(pi), fullPolyline.getLon(pi),
                            fullPolyline.getLat(pi + 1), fullPolyline.getLon(pi + 1));
                }
                long oldTime = instr.getTime();
                instr.setDistance(newDist);
                instr.setTime(Math.round(newDist / oldDist * oldTime));
            }
        }

        // Strip internal polyline hints — not for the client
        for (Instruction instr : instructions) {
            instr.getExtraInfoJSON().remove("_polyline_start_hint");
        }
    }

    /**
     * Coordinate-match each instruction's first geometry point to the route polyline and return the
     * per-instruction polyline start index. This is the same monotonic forward match
     * {@link #remapInstructionGeometry} uses, but exposed so the {@code interval} indices sent to the
     * client are derived from each instruction's ACTUAL geometry position rather than recomputed by
     * cumulative {@code getLength()}.
     * <p>
     * Why this exists: {@code getLength()} (point count) only equals an instruction's polyline span
     * when every instruction's geometry tiles the polyline exactly. It does not when an instruction's
     * point count differs from its span — e.g. an empty slice forced to 1 point (a coordinates-gap
     * resume that coincides with the next turn). A single such instruction shifts every later
     * cumulative index by one, which on a simplified polyline can be hundreds of metres. Matching by
     * coordinate makes each interval start land exactly on the instruction's turn vertex, independent
     * of point counts. See docs/gh_tbt_instruction_pipeline.md.
     * <p>
     * Must be called on the FINAL instruction list (after {@link #remapInstructionGeometry} has set
     * each instruction's geometry to polyline slices, and after any post-processing), so the first
     * point of every instruction is a real polyline coordinate.
     *
     * @return a list of start indices, one per instruction, monotonically non-decreasing. The interval
     *         for instruction {@code i} is {@code [starts.get(i), i+1<n ? starts.get(i+1) : polyline.size()-1]}.
     */
    public static List<Integer> matchInstructionStarts(InstructionList instructions, PointList polyline) {
        List<Integer> starts = new ArrayList<>(instructions.size());
        if (instructions.isEmpty()) return starts;
        // Degenerate (no geometry): still return one entry per instruction so callers can index 1:1.
        if (polyline.isEmpty()) {
            for (int i = 0; i < instructions.size(); i++) starts.add(0);
            return starts;
        }
        int n = polyline.size();

        // Cumulative polyline distance per vertex — the scale against which each instruction's
        // independent route-distance key (_cum_route_m) is compared to disambiguate repeated coordinates.
        double[] polyCum = new double[n];
        for (int k = 1; k < n; k++)
            polyCum[k] = polyCum[k - 1] + DistanceCalcEarth.DIST_EARTH.calcDist(
                    polyline.getLat(k - 1), polyline.getLon(k - 1), polyline.getLat(k), polyline.getLon(k));

        // Anchor = the last instruction placed with high confidence. A flagged/recovered instruction
        // does NOT advance the anchor, so a single bad match cannot cascade down the rest of the route.
        int anchorIdx = 0;
        double anchorRouteM = readCumRouteM(instructions.get(0));
        // The anchor is CALIBRATED when its polyline position corresponds to its synthetic route
        // distance — true for a normal turn (matched vertex IS its junction node). It is NOT true
        // for seam-anchored instructions (route start, gap markers, gap resumes): their polyline
        // anchor is the seam while their synthetic distance sits at the section chain start, so
        // the next leg's expected distance carries the pre-snap span (unbounded — 862 m on the
        // Saariselkä 2-segment case, where the start snapped deep into a very long edge). The
        // sanity rejection below is applied only from a calibrated anchor; from an uncalibrated
        // one a coordinate match is accepted as-is (the pre-sanity-band rule, correct there).
        boolean anchorCalibrated = false;

        for (int i = 0; i < instructions.size(); i++) {
            Instruction instr = instructions.get(i);
            if (instr.getSign() == Instruction.FINISH) {
                starts.add(n - 1);
                continue;
            }
            if (i == 0) {
                starts.add(0);
                anchorIdx = 0;
                anchorRouteM = readCumRouteM(instr);
                continue;
            }
            PointList pts = instr.getPoints();
            if (pts.size() == 0) {
                // Defensive (remap guarantees >=1 point per non-FINISH instruction). Keep the output
                // monotonic: a non-confident predecessor may have been placed ahead of the anchor.
                starts.add(Math.max(anchorIdx, starts.get(starts.size() - 1)));
                continue;
            }

            double targetLat = pts.getLat(0), targetLon = pts.getLon(0);
            double cumThis = readCumRouteM(instr);
            boolean haveKey = !Double.isNaN(cumThis) && !Double.isNaN(anchorRouteM);
            // Expected polyline distance = the last confident position plus this leg's synthetic length.
            // Chooses among multiple occurrences, and sanity-checks even a UNIQUE match: a coordinate
            // can be unique-but-wrong when simplification dropped the true occurrence's vertex on an
            // out-and-back — trusting it unvalidated pins the instruction to the other pass.
            double expectedCum = haveKey ? polyCum[anchorIdx] + (cumThis - anchorRouteM) : Double.NaN;

            // Scan from the anchor: count near-exact occurrences of this turn, remember the first and
            // the one nearest the expected route distance, and the global-closest coordinate as a last resort.
            int exactCount = 0, firstExact = -1, distBest = -1;
            double distBestErr = Double.MAX_VALUE;
            int closestIdx = anchorIdx; double closestCoord = Double.MAX_VALUE;
            for (int pi = anchorIdx; pi < n; pi++) {
                double cd = Math.abs(polyline.getLat(pi) - targetLat) + Math.abs(polyline.getLon(pi) - targetLon);
                if (cd < closestCoord) { closestCoord = cd; closestIdx = pi; }
                if (cd < 5e-6) {                                  // a near-exact occurrence of this turn
                    exactCount++;
                    if (firstExact < 0) firstExact = pi;
                    double cumErr = haveKey ? Math.abs(polyCum[pi] - expectedCum) : 0.0;
                    if (cumErr < distBestErr) { distBestErr = cumErr; distBest = pi; }
                    if (!haveKey) break;                          // no key → first match wins (legacy behavior)
                }
            }

            int idx;
            boolean confident;
            if (exactCount >= 1 && (!haveKey || !anchorCalibrated || distBestErr <= MATCH_SANITY_M)) {
                // Take the occurrence nearest the expected route distance (the only occurrence,
                // when unique) — but only while it agrees with the distance key within the sanity
                // band. Legs adjacent to seams carry inherent synthetic-vs-polyline error (snap
                // trims, gap jumps), so the band is generous; a real wrong-occurrence signal is
                // 2 x spur and the observed bugs were 191 m and 4.5 km.
                idx = haveKey ? distBest : firstExact;
                confident = true;
            } else if (exactCount >= 1) {
                // Near-exact coordinate found, but every occurrence is far from where the route
                // distance says this instruction lives — the true occurrence's vertex was most
                // likely simplified away. Place by expected distance (contained, non-anchoring).
                idx = indexNearestCum(polyCum, expectedCum, anchorIdx);
                confident = false;
                LOGGER.warn("TbT interval: instruction #{} ('{}') coordinate matches only {} m from its "
                        + "expected route distance (suspected wrong occurrence); best-effort placing at "
                        + "idx {} (single-instruction recovery)",
                        i, instr.getName(), Math.round(distBestErr), idx);
            } else {
                // No near-exact match anywhere ahead (e.g. the point was simplified away, or a poisoned
                // anchor put the true occurrence behind us). Place by expected distance so the error is
                // CONTAINED to this one instruction, and don't trust it as the next anchor.
                idx = haveKey ? indexNearestCum(polyCum, expectedCum, anchorIdx) : closestIdx;
                confident = false;
                LOGGER.warn("TbT interval: instruction #{} ('{}') has no near-exact polyline match "
                        + "(closest ~{} m); best-effort placing at idx {} (single-instruction recovery)",
                        i, instr.getName(), Math.round(closestCoord * DIAG_DEG_TO_M), idx);
            }

            // Guarantee monotonic output so intervals tile without reversal.
            int prev = starts.get(starts.size() - 1);
            if (idx < prev) idx = prev;
            starts.add(idx);

            if (confident) {
                anchorIdx = idx;
                anchorRouteM = cumThis;
                // Seam-anchored instructions (gap marker / gap resume) match at the seam vertex
                // while their synthetic distance sits elsewhere — they must not calibrate the
                // expectation for the next leg. Everything else anchors at its own junction node.
                Map<String, Object> extra = instr.getExtraInfoJSON();
                anchorCalibrated = !Boolean.TRUE.equals(extra.get("tbt_resumed"))
                        && !Boolean.FALSE.equals(extra.get("tbt_available"));
            }
        }
        return starts;
    }

    /** ~manhattan-degrees→metres near lat 61, for human-readable log messages only. */
    private static final double DIAG_DEG_TO_M = 90000.0;

    /**
     * Sanity band (m) for accepting a coordinate match against the expected route distance in
     * {@link #matchInstructionStarts}. Applied only from a CALIBRATED anchor (one whose polyline
     * position corresponds to its synthetic route distance) — from seam-anchored positions
     * (route start, gap markers/resumes) the expectation carries an unbounded pre-snap offset and
     * must not veto a coordinate match. From a calibrated anchor the leg error is snap-trim scale
     * (metres), far below the wrong-occurrence signal (2 x out-and-back spur; observed bugs:
     * 191 m and 4462 m). A rejected match degrades to the contained, non-anchoring
     * distance-based recovery placement.
     */
    private static final double MATCH_SANITY_M = 300.0;

    private static double readCumRouteM(Instruction instr) {
        Object v = instr.getExtraInfoJSON().get("_cum_route_m");
        return (v instanceof Number) ? ((Number) v).doubleValue() : Double.NaN;
    }

    /** Lowest polyline index >= floor whose cumulative distance is nearest {@code target}. Monotonic,
     *  so it yields a best-effort placement that cannot reverse the interval tiling. */
    private static int indexNearestCum(double[] polyCum, double target, int floor) {
        int n = polyCum.length;
        if (Double.isNaN(target)) return Math.min(Math.max(floor, 0), n - 1);
        int best = Math.min(Math.max(floor, 0), n - 1); double bestErr = Double.MAX_VALUE;
        for (int pi = Math.max(0, floor); pi < n; pi++) {
            double err = Math.abs(polyCum[pi] - target);
            if (err < bestErr) { bestErr = err; best = pi; }
            if (polyCum[pi] >= target) break;   // monotonic: past target, only gets farther
        }
        return best;
    }

    // ---- Polyline building ----

    /**
     * Append route response polyline to the full polyline, skipping the first point
     * if it duplicates the last point of the existing polyline (at segment boundaries).
     *
     * @return the index in {@code fullPolyline} where the appended section physically starts —
     *         i.e. where {@code routePolyline}'s first point lives: the pre-existing seam vertex
     *         when the duplicate was skipped, the first appended index otherwise. Callers that
     *         anchor a "section starts here" hint must use this rather than the pre-append size,
     *         which points one vertex INTO the section whenever the dedup fires.
     */
    private int appendRoutePolyline(PointList fullPolyline, PointList routePolyline) {
        int preSize = fullPolyline.size();
        int startIdx = 0;
        if (fullPolyline.size() > 0 && routePolyline.size() > 0) {
            double lastLat = fullPolyline.getLat(fullPolyline.size() - 1);
            double lastLon = fullPolyline.getLon(fullPolyline.size() - 1);
            double firstLat = routePolyline.getLat(0);
            double firstLon = routePolyline.getLon(0);
            // Use a small tolerance for floating point comparison
            if (DistanceCalcEarth.DIST_EARTH.calcDist(lastLat, lastLon, firstLat, firstLon) < 1.0) {
                startIdx = 1; // skip duplicate point at segment boundary
            }
        }
        for (int i = startIdx; i < routePolyline.size(); i++) {
            fullPolyline.add(routePolyline.getLat(i), routePolyline.getLon(i),
                    routePolyline.is3D() ? routePolyline.getEle(i) : Double.NaN);
        }
        return startIdx == 1 ? preSize - 1 : preSize;
    }

    /**
     * Build geometry for a non-routable gap segment.
     */
    private PointList buildGapGeometry(Chunk chunk, Map<String, TrailmapInstructionRequest.Coordinates> waypointMap) {
        TrailmapInstructionRequest.Segment seg = chunk.nonRoutableSegment;

        if (TrailmapInstructionRequest.TYPE_COORDINATES.equals(chunk.segmentType)
                && seg.getTrackCoordinates() != null && !seg.getTrackCoordinates().isEmpty()) {
            // Use the track coordinates
            PointList points = new PointList(seg.getTrackCoordinates().size(), true);
            for (TrailmapInstructionRequest.Coordinates c : seg.getTrackCoordinates()) {
                points.add(c.getLat(), c.getLng(), Double.NaN);
            }
            return points;
        } else {
            // Direct segment: straight line from start to end
            PointList points = new PointList(2, true);
            TrailmapInstructionRequest.Coordinates start = waypointMap.get(seg.getStart());
            TrailmapInstructionRequest.Coordinates end = waypointMap.get(seg.getEnd());
            if (start == null || end == null) {
                throw new IllegalArgumentException("Direct segment references unknown waypoint id(s): start='"
                        + seg.getStart() + "'" + (start == null ? " (unresolved)" : "")
                        + ", end='" + seg.getEnd() + "'" + (end == null ? " (unresolved)" : ""));
            }
            points.add(start.getLat(), start.getLng(), Double.NaN);
            points.add(end.getLat(), end.getLng(), Double.NaN);
            return points;
        }
    }

    /**
     * Append gap geometry to the full polyline, skipping the first point
     * if it duplicates the polyline's last point.
     */
    private void appendGapPolyline(PointList fullPolyline, PointList gapPoints) {
        int tracePolyBefore = fullPolyline.size();
        for (int i = 0; i < gapPoints.size(); i++) {
            // Skip first point if it matches last point of polyline (avoid duplicate at junction)
            if (i == 0 && fullPolyline.size() > 0) {
                double lastLat = fullPolyline.getLat(fullPolyline.size() - 1);
                double lastLon = fullPolyline.getLon(fullPolyline.size() - 1);
                if (Math.abs(lastLat - gapPoints.getLat(i)) < 1e-7
                        && Math.abs(lastLon - gapPoints.getLon(i)) < 1e-7) {
                    continue;
                }
            }
            fullPolyline.add(gapPoints.getLat(i), gapPoints.getLon(i),
                    gapPoints.is3D() ? gapPoints.getEle(i) : Double.NaN);
        }
        if (PLACEMENT_TRACE) {
            PLACEMENT_TRACE_LOG.add(new PlacementTraceRecord(PlacementTraceRecord.GAP,
                    null, tracePolyBefore, fullPolyline.size(), gapPoints));
        }
    }

    // ---- Helpers ----

    /**
     * Compute straight-line distance of a gap geometry PointList.
     */
    private static double calcGapDistance(PointList points) {
        double dist = 0;
        for (int i = 0; i < points.size() - 1; i++) {
            dist += DistanceCalcEarth.DIST_EARTH.calcDist(
                    points.getLat(i), points.getLon(i),
                    points.getLat(i + 1), points.getLon(i + 1));
        }
        return dist;
    }

    private static TrailmapInstructionRequest.Coordinates coordOf(double lat, double lng) {
        TrailmapInstructionRequest.Coordinates c = new TrailmapInstructionRequest.Coordinates();
        c.setLat(lat);
        c.setLng(lng);
        return c;
    }

    static class Section {
        String profile;
        com.graphhopper.util.CustomModel customModel;
        Double initialHeading;
        Double headingPenalty;
        List<TrailmapInstructionRequest.Coordinates> points;
    }
}
