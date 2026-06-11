package com.graphhopper.trailmap.tbt;

import com.graphhopper.routing.InstructionsHelper;
import com.graphhopper.routing.InstructionsOutgoingEdges;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.ev.*;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.trailmap.shared.PredictedHighway;
import com.graphhopper.trailmap.shared.PredictedSurface;
import com.graphhopper.util.*;
import com.graphhopper.util.shapes.GHPoint;

import java.util.ArrayList;
import java.util.List;

import static com.graphhopper.util.Parameters.Details.*;

/**
 * Trailmap fork of {@link com.graphhopper.routing.InstructionsFromEdges}.
 * <p>
 * Stage 1 of the Trailmap TbT instruction pipeline. Generates instructions from
 * a path edge sequence with trail/bike-aware turn decisions and rich extraInfo
 * metadata for downstream post-processing (Stage 2) and client consumption (Stage 3).
 * <p>
 * This is a standalone implementation of {@link Path.EdgeVisitor}, not a subclass of
 * InstructionsFromEdges (whose fields and methods are all private). The edge visitor
 * pattern, roundabout handling, u-turn detection, and angle math are preserved from
 * the GH original. The {@link #getTurn} method contains custom trail-aware logic.
 */
public class TrailmapInstructionsFromEdges implements Path.EdgeVisitor {

    // --- GH core encoded values ---
    private final Weighting weighting;
    private final NodeAccess nodeAccess;
    private final InstructionList ways;
    private final EdgeExplorer outEdgeExplorer;
    private final EdgeExplorer allExplorer;
    private final BooleanEncodedValue roundaboutEnc;
    private final BooleanEncodedValue roadClassLinkEnc;
    private final EnumEncodedValue<RoadClass> roadClassEnc;
    private final EnumEncodedValue<RoadEnvironment> roadEnvEnc;
    private final IntEncodedValue lanesEnc;
    private final DecimalEncodedValue maxSpeedEnc;

    // --- Trailmap additional encoded values ---
    private final EnumEncodedValue<PredictedHighway> predictedHighwayEnc;
    private final EnumEncodedValue<PredictedSurface> predictedSurfaceEnc;
    private final EnumEncodedValue<Surface> surfaceEnc;
    private final EnumEncodedValue<RouteNetwork> bikeNetworkEnc;

    // --- Three-point orientation state (see GH original for explanation) ---
    private EdgeIteratorState prevEdge;
    private double prevLat;
    private double prevLon;
    private double doublePrevLat, doublePrevLon;
    private int prevNode;
    private double prevOrientation;
    private double prevInstructionPrevOrientation = Double.NaN;
    private Instruction prevInstruction;
    private boolean prevInRoundabout;
    private String prevDestinationAndRef;
    private String prevName;
    private RoadEnvironment prevRoadEnv;
    private String prevInstructionName;

    // --- Trailmap state: previous edge properties for cross-edge comparisons ---
    private PredictedHighway prevPredictedHighway;
    private PredictedSurface prevPredictedSurface;
    private RouteNetwork prevBikeNetwork;

    // --- Junction context from last getTurn() call, consumed by enrichExtraInfo ---
    private InstructionsOutgoingEdges lastOutgoingEdges;

    // --- Visual guidance side-channel ---
    // Set by suppression rules in getTurn() when a junction is visual-worthy but not
    // instruction-worthy. Read by next() after getTurn() returns IGNORE.
    // Carries the angle-based sign the instruction would have if it were a real turn.
    private int visualCandidateSign = Instruction.IGNORE;

    // --- Sign reframer state ---
    // Holds the shape label of the last reframer firing (e.g. "shape_4_sandwich").
    // Reset at the start of each reframeSign() call. Read by next() to tag extraInfo.
    private String lastReframerShape = null;

    private static final int MAX_U_TURN_DISTANCE = 35;

    public TrailmapInstructionsFromEdges(Graph graph, Weighting weighting, EncodedValueLookup evLookup,
                                          InstructionList ways) {
        this.weighting = weighting;
        this.ways = ways;
        this.nodeAccess = graph.getNodeAccess();

        // GH core encoded values
        this.roundaboutEnc = evLookup.getBooleanEncodedValue(Roundabout.KEY);
        this.roadEnvEnc = evLookup.getEnumEncodedValue(RoadEnvironment.KEY, RoadEnvironment.class);
        this.roadClassEnc = evLookup.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        this.roadClassLinkEnc = evLookup.getBooleanEncodedValue(RoadClassLink.KEY);
        this.maxSpeedEnc = evLookup.getDecimalEncodedValue(MaxSpeed.KEY);
        this.lanesEnc = evLookup.hasEncodedValue(Lanes.KEY) ? evLookup.getIntEncodedValue(Lanes.KEY) : null;

        // Trailmap encoded values (optional — may not exist in all graph builds)
        this.predictedHighwayEnc = evLookup.hasEncodedValue(PredictedHighway.KEY)
                ? evLookup.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class) : null;
        this.predictedSurfaceEnc = evLookup.hasEncodedValue(PredictedSurface.KEY)
                ? evLookup.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class) : null;
        this.surfaceEnc = evLookup.hasEncodedValue(Surface.KEY)
                ? evLookup.getEnumEncodedValue(Surface.KEY, Surface.class) : null;
        this.bikeNetworkEnc = evLookup.hasEncodedValue(BikeNetwork.KEY)
                ? evLookup.getEnumEncodedValue(BikeNetwork.KEY, RouteNetwork.class) : null;

        prevNode = -1;
        prevInRoundabout = false;
        prevName = null;
        prevRoadEnv = null;
        prevPredictedHighway = null;
        prevPredictedSurface = null;
        prevBikeNetwork = null;

        // Access filter for roundabout exit counting (GH original uses car).
        // Fall back to bike if car profile absent (trail/bike-only graph builds).
        BooleanEncodedValue accessEnc;
        if (evLookup.hasEncodedValue(VehicleAccess.key("car"))) {
            accessEnc = evLookup.getBooleanEncodedValue(VehicleAccess.key("car"));
        } else if (evLookup.hasEncodedValue(VehicleAccess.key("bike"))) {
            accessEnc = evLookup.getBooleanEncodedValue(VehicleAccess.key("bike"));
        } else {
            accessEnc = null;
        }
        outEdgeExplorer = accessEnc != null
                ? graph.createEdgeExplorer(edge -> edge.get(accessEnc))
                : graph.createEdgeExplorer();
        allExplorer = graph.createEdgeExplorer();
    }

    /**
     * Generate instructions for the given path. Drop-in replacement for
     * {@link com.graphhopper.routing.InstructionsFromEdges#calcInstructions}.
     */
    public static InstructionList calcInstructions(Path path, Graph graph, Weighting weighting,
                                                    EncodedValueLookup evLookup, Translation tr) {
        final InstructionList ways = new InstructionList(tr);
        if (path.isFound()) {
            if (path.getEdgeCount() == 0) {
                ways.add(new FinishInstruction(graph.getNodeAccess(), path.getEndNode()));
            } else {
                path.forEveryEdge(new TrailmapInstructionsFromEdges(graph, weighting, evLookup, ways));
            }
        }
        return ways;
    }

    @Override
    public void next(EdgeIteratorState edge, int index, int prevEdgeId) {
        int adjNode = edge.getAdjNode();
        int baseNode = edge.getBaseNode();

        if (prevNode == -1) {
            prevLat = this.nodeAccess.getLat(baseNode);
            prevLon = this.nodeAccess.getLon(baseNode);
        }

        double adjLat = nodeAccess.getLat(adjNode);
        double adjLon = nodeAccess.getLon(adjNode);
        double latitude, longitude;

        PointList wayGeo = edge.fetchWayGeometry(FetchMode.ALL);
        boolean isRoundabout = edge.get(roundaboutEnc);

        if (wayGeo.size() <= 2) {
            latitude = adjLat;
            longitude = adjLon;
        } else {
            latitude = wayGeo.getLat(1);
            longitude = wayGeo.getLon(1);
        }

        final String name = (String) edge.getValue(STREET_NAME);
        final String ref = (String) edge.getValue(STREET_REF);
        final String destination = (String) edge.getValue(STREET_DESTINATION);
        final String destinationRef = (String) edge.getValue(STREET_DESTINATION_REF);
        final String motorwayJunction = (String) edge.getValue(MOTORWAY_JUNCTION);
        final RoadEnvironment roadEnv = edge.get(roadEnvEnc);

        // Read Trailmap properties for this edge
        PredictedHighway currentPH = predictedHighwayEnc != null ? edge.get(predictedHighwayEnc) : null;
        PredictedSurface currentPS = predictedSurfaceEnc != null ? edge.get(predictedSurfaceEnc) : null;
        RouteNetwork currentBN = bikeNetworkEnc != null ? edge.get(bikeNetworkEnc) : null;

        if ((prevInstruction == null) && (!isRoundabout)) // very first instruction
        {
            int sign = Instruction.CONTINUE_ON_STREET;
            prevInstruction = new Instruction(sign, name, new PointList(10, nodeAccess.is3D()));
            prevInstruction.setExtraInfo(STREET_REF, ref);
            prevInstruction.setExtraInfo(STREET_DESTINATION, destination);
            prevInstruction.setExtraInfo(STREET_DESTINATION_REF, destinationRef);
            prevInstruction.setExtraInfo(MOTORWAY_JUNCTION, motorwayJunction);
            prevInstruction.setExtraInfo("ferry", InstructionsHelper.createFerryInfo(roadEnv, prevRoadEnv));

            double startLat = nodeAccess.getLat(baseNode);
            double startLon = nodeAccess.getLon(baseNode);
            double heading = AngleCalc.ANGLE_CALC.calcAzimuth(startLat, startLon, latitude, longitude);
            prevInstruction.setExtraInfo("heading", Helper.round(heading, 2));

            lastOutgoingEdges = null;
            enrichExtraInfo(prevInstruction, edge, null);

            ways.add(prevInstruction);
            prevName = name;
            prevRoadEnv = roadEnv;
            prevDestinationAndRef = destination + destinationRef;
            prevPredictedHighway = currentPH;
            prevPredictedSurface = currentPS;
            prevBikeNetwork = currentBN;

        } else if (isRoundabout) {
            // remark: names and annotations within roundabout are ignored
            if (!prevInRoundabout) //just entered roundabout
            {
                int sign = Instruction.USE_ROUNDABOUT;
                TrailmapRoundaboutInstruction roundaboutInstruction = new TrailmapRoundaboutInstruction(sign, name,
                        new PointList(10, nodeAccess.is3D()));
                prevInstructionPrevOrientation = prevOrientation;
                if (prevInstruction != null) {
                    EdgeIterator edgeIter = outEdgeExplorer.setBaseNode(baseNode);
                    while (edgeIter.next()) {
                        if ((edgeIter.getAdjNode() != prevNode) && !edgeIter.get(roundaboutEnc)) {
                            roundaboutInstruction.increaseExitNumber();
                            break;
                        }
                    }
                    prevOrientation = AngleCalc.ANGLE_CALC.calcOrientation(doublePrevLat, doublePrevLon, prevLat, prevLon);
                    double orientation = AngleCalc.ANGLE_CALC.calcOrientation(prevLat, prevLon, latitude, longitude);
                    orientation = AngleCalc.ANGLE_CALC.alignOrientation(prevOrientation, orientation);
                    double delta = (orientation - prevOrientation);
                    roundaboutInstruction.setDirOfRotation(delta);
                } else {
                    prevOrientation = AngleCalc.ANGLE_CALC.calcOrientation(prevLat, prevLon, latitude, longitude);
                    prevName = name;
                    prevRoadEnv = roadEnv;
                    prevDestinationAndRef = destination + destinationRef;
                    prevPredictedHighway = currentPH;
                    prevPredictedSurface = currentPS;
                    prevBikeNetwork = currentBN;
                }
                prevInstruction = roundaboutInstruction;
                ways.add(prevInstruction);
            }

            EdgeIterator edgeIter = outEdgeExplorer.setBaseNode(edge.getAdjNode());
            while (edgeIter.next()) {
                if (!edgeIter.get(roundaboutEnc)) {
                    ((RoundaboutInstruction) prevInstruction).increaseExitNumber();
                    break;
                }
            }

        } else if (prevInRoundabout) //previously in roundabout but not anymore
        {
            prevInstruction.setName(name);
            prevInstruction.setExtraInfo(STREET_REF, ref);
            prevInstruction.setExtraInfo(STREET_DESTINATION, destination);
            prevInstruction.setExtraInfo(STREET_DESTINATION_REF, destinationRef);
            prevInstruction.setExtraInfo(MOTORWAY_JUNCTION, motorwayJunction);
            prevInstruction.setExtraInfo("ferry", InstructionsHelper.createFerryInfo(roadEnv, prevRoadEnv));

            double orientation = AngleCalc.ANGLE_CALC.calcOrientation(prevLat, prevLon, latitude, longitude);
            orientation = AngleCalc.ANGLE_CALC.alignOrientation(prevOrientation, orientation);
            double deltaInOut = (orientation - prevOrientation);
            double recentOrientation = AngleCalc.ANGLE_CALC.calcOrientation(doublePrevLat, doublePrevLon, prevLat, prevLon);
            orientation = AngleCalc.ANGLE_CALC.alignOrientation(recentOrientation, orientation);
            double deltaOut = (orientation - recentOrientation);

            prevInstruction = ((RoundaboutInstruction) prevInstruction)
                    .setRadian(deltaInOut)
                    .setDirOfRotation(deltaOut)
                    .setExited();

            lastOutgoingEdges = null;
            enrichExtraInfo(prevInstruction, edge, prevEdge);

            prevInstructionName = prevName;
            prevName = name;
            prevRoadEnv = roadEnv;
            prevDestinationAndRef = destination + destinationRef;
            prevPredictedHighway = currentPH;
            prevPredictedSurface = currentPS;
            prevBikeNetwork = currentBN;

        } else {
            int sign = getTurn(edge, baseNode, prevNode, adjNode, name, destination + destinationRef);

            // Sign reframer (Stage 1 junction-relative reframing layer).
            // Adjusts the rule-chain sign against the visible-alt distribution at the
            // junction. Acts on slight-zone signs (CONTINUE/SLIGHT/KEEP) and on
            // TURN_LEFT/RIGHT (sign ±2) via Shape 6. May demote slight→CONTINUE,
            // upgrade CONTINUE→KEEP, or demote TURN→SLIGHT depending on junction shape.
            int originalSign = sign;
            if (isReframerCandidate(sign)) {
                int reframed = reframeSign(sign, edge, baseNode);
                if (reframed != sign) {
                    sign = reframed;
                }
            }

            // Visual side-channel: if getTurn() returned IGNORE but a rule flagged this
            // junction as visual-worthy, upgrade to a visual instruction.
            boolean isVisual = false;
            if (sign == Instruction.IGNORE && visualCandidateSign != Instruction.IGNORE) {
                sign = visualCandidateSign;
                isVisual = true;
            }

            if (sign != Instruction.IGNORE) {
                // U-turn detection (same as GH original) — skip for visual instructions
                boolean isUTurn = false;
                int uTurnType = Instruction.U_TURN_UNKNOWN;
                if (!isVisual
                        && !Double.isNaN(prevInstructionPrevOrientation)
                        && prevInstruction.getDistance() < MAX_U_TURN_DISTANCE
                        && (sign < 0) == (prevInstruction.getSign() < 0)
                        && (Math.abs(sign) == Instruction.TURN_SLIGHT_RIGHT || Math.abs(sign) == Instruction.TURN_RIGHT || Math.abs(sign) == Instruction.TURN_SHARP_RIGHT)
                        && (Math.abs(prevInstruction.getSign()) == Instruction.TURN_SLIGHT_RIGHT || Math.abs(prevInstruction.getSign()) == Instruction.TURN_RIGHT || Math.abs(prevInstruction.getSign()) == Instruction.TURN_SHARP_RIGHT)
                        && Double.isFinite(weighting.calcEdgeWeight(edge, false)) != Double.isFinite(weighting.calcEdgeWeight(edge, true))
                        && InstructionsHelper.isSameName(prevInstructionName, name)) {
                    GHPoint point = InstructionsHelper.getPointForOrientationCalculation(edge, nodeAccess);
                    double lat = point.getLat();
                    double lon = point.getLon();
                    double currentOrientation = AngleCalc.ANGLE_CALC.calcOrientation(prevLat, prevLon, lat, lon, false);
                    double diff = Math.abs(prevInstructionPrevOrientation - currentOrientation);
                    if (diff > (Math.PI * .9) && diff < (Math.PI * 1.1)) {
                        isUTurn = true;
                        uTurnType = sign < 0 ? Instruction.U_TURN_LEFT : Instruction.U_TURN_RIGHT;
                    }
                }

                if (isUTurn) {
                    prevInstruction.setSign(uTurnType);
                    prevInstruction.setName(name);
                } else {
                    prevInstruction = new Instruction(sign, name, new PointList(10, nodeAccess.is3D()));
                    prevInstructionPrevOrientation = prevOrientation;
                    prevInstructionName = prevName;
                    ways.add(prevInstruction);
                }
                prevInstruction.setExtraInfo(STREET_REF, ref);
                prevInstruction.setExtraInfo(STREET_DESTINATION, destination);
                prevInstruction.setExtraInfo(STREET_DESTINATION_REF, destinationRef);
                prevInstruction.setExtraInfo(MOTORWAY_JUNCTION, motorwayJunction);
                prevInstruction.setExtraInfo("ferry", InstructionsHelper.createFerryInfo(roadEnv, prevRoadEnv));

                enrichExtraInfo(prevInstruction, edge, prevEdge);

                if (isVisual) {
                    prevInstruction.setExtraInfo("visual_guidance", true);
                }
                if (sign != originalSign && lastReframerShape != null) {
                    prevInstruction.setExtraInfo("reframed_from", originalSign);
                    prevInstruction.setExtraInfo("reframer_shape", lastReframerShape);
                    // When the reframer relabels because the rider sees a real junction
                    // (side-turn off the route, sandwich demote to CONTINUE, or sandwich
                    // past-slight magnitude demote to TURN_SLIGHT), stamp trail_fork so
                    // the client fires its fork cue. The existing hasConfusableAlternative
                    // criteria are tighter than the reframer's visual-alt set, so the tag
                    // would otherwise be missing.
                    boolean stampTrailFork =
                            (sign == Instruction.CONTINUE_ON_STREET
                                && ("shape_1_side_turn".equals(lastReframerShape)
                                    || "shape_4_sandwich".equals(lastReframerShape)))
                            || ((sign == Instruction.TURN_SLIGHT_LEFT
                                    || sign == Instruction.TURN_SLIGHT_RIGHT)
                                && ("shape_4_sandwich_past_slight".equals(lastReframerShape)
                                    || "shape_6_anchored".equals(lastReframerShape)));
                    if (stampTrailFork) {
                        prevInstruction.setExtraInfo("trail_fork", true);
                    }
                }
            }
            prevName = name;
            prevRoadEnv = roadEnv;
            prevDestinationAndRef = destination + destinationRef;
            prevPredictedHighway = currentPH;
            prevPredictedSurface = currentPS;
            prevBikeNetwork = currentBN;
        }

        updatePointsAndInstruction(edge, wayGeo);

        if (wayGeo.size() <= 2) {
            doublePrevLat = prevLat;
            doublePrevLon = prevLon;
        } else {
            int beforeLast = wayGeo.size() - 2;
            doublePrevLat = wayGeo.getLat(beforeLast);
            doublePrevLon = wayGeo.getLon(beforeLast);
        }

        prevInRoundabout = isRoundabout;
        prevNode = baseNode;
        prevLat = adjLat;
        prevLon = adjLon;
        prevEdge = edge;
    }

    @Override
    public void finish() {
        if (prevInRoundabout) {
            double orientation = AngleCalc.ANGLE_CALC.calcOrientation(doublePrevLat, doublePrevLon, prevLat, prevLon);
            orientation = AngleCalc.ANGLE_CALC.alignOrientation(prevOrientation, orientation);
            double delta = (orientation - prevOrientation);
            ((RoundaboutInstruction) prevInstruction).setRadian(delta);
        }
        Instruction finishInstruction = new FinishInstruction(nodeAccess, prevEdge.getAdjNode());
        finishInstruction.setExtraInfo("last_heading", AngleCalc.ANGLE_CALC.calcAzimuth(doublePrevLat, doublePrevLon, prevLat, prevLon));
        ways.add(finishInstruction);
    }

    // ========================================================================
    // Turn decision logic — Trailmap custom rules
    // ========================================================================

    /**
     * Determine the turn instruction sign for the current edge transition.
     * <p>
     * Custom trail/bike-aware logic replacing GH's car-centric {@code getTurn()}.
     * Uses PredictedHighway-based continuity signals instead of speed-based heuristics.
     * <p>
     * Rule priority: trivial cases → ferry → emit signals (E1–E4) →
     * suppress signals (S1–S5) → GH fallback.
     * <p>
     * Side effect: stores {@link #lastOutgoingEdges} for use by {@link #enrichExtraInfo}.
     */
    private int getTurn(EdgeIteratorState edge, int baseNode, int prevNode, int adjNode,
                        String name, String destinationAndRef) {
        // Reset visual side-channel for this edge transition
        visualCandidateSign = Instruction.IGNORE;

        // --- Trivial: plain u-turn ---
        if (edge.getEdge() == prevEdge.getEdge())
            return Instruction.U_TURN_UNKNOWN;

        // --- Calculate angle and junction topology ---
        GHPoint point = InstructionsHelper.getPointForOrientationCalculation(edge, nodeAccess);
        double lat = point.getLat();
        double lon = point.getLon();
        prevOrientation = AngleCalc.ANGLE_CALC.calcOrientation(doublePrevLat, doublePrevLon, prevLat, prevLon);
        int sign = InstructionsHelper.calculateSign(prevLat, prevLon, lat, lon, prevOrientation);

        InstructionsOutgoingEdges outgoingEdges = new InstructionsOutgoingEdges(prevEdge, edge, weighting, maxSpeedEnc,
                roadClassEnc, roadClassLinkEnc, lanesEnc, allExplorer, nodeAccess, prevNode, baseNode, adjNode);
        int nrOfPossibleTurns = outgoingEdges.getAllowedTurns();
        RoadEnvironment roadEnv = edge.get(roadEnvEnc);

        // Store for enrichExtraInfo
        lastOutgoingEdges = outgoingEdges;

        // Read Trailmap edge properties
        RoadClass currentRC = edge.get(roadClassEnc);
        RoadClass prevRC = prevEdge.get(roadClassEnc);
        PredictedHighway currentPH = predictedHighwayEnc != null ? edge.get(predictedHighwayEnc) : null;
        // prevPredictedHighway is the state field, updated at end of next()
        PredictedHighway prevPH = prevPredictedHighway;
        PredictedSurface currentSurface = predictedSurfaceEnc != null ? edge.get(predictedSurfaceEnc) : null;
        PredictedSurface prevSurface = prevPredictedSurface;
        RouteNetwork currentBN = bikeNetworkEnc != null ? edge.get(bikeNetworkEnc) : null;
        RouteNetwork prevBN = prevBikeNetwork;

        // --- No alternatives: forced path ---
        if (nrOfPossibleTurns <= 1) {
            if (InstructionsHelper.isToFerry(roadEnv, prevRoadEnv)) return Instruction.FERRY;
            if (Math.abs(sign) > 1 && outgoingEdges.getVisibleTurns() > 1 && !outgoingEdges.mergedOrSplitWay()
                    || InstructionsHelper.isFromFerry(roadEnv, prevRoadEnv)) {
                return sign;
            }
            // E5: Bike on CYCLEWAY makes a real turn while a visually-similar FOOTWAY runs
            // (near-)straight ahead. The footway is access-blocked under the bike weighting
            // and so absent from getAllowedTurns()/getVisibleTurns(); on the ground the
            // rider still sees it as an identical-looking path. Emit the angle-based sign.
            // Visual similarity is checked against BOTH the prev edge surface and the route's
            // next edge surface — the rider sees both at the junction, so an alt matching
            // either is a real go-straight trap.
            if (Math.abs(sign) > 1
                    && currentPH == PredictedHighway.CYCLEWAY
                    && !outgoingEdges.mergedOrSplitWay()
                    && hasConfusableFootwayAlternative(edge, baseNode, prevSurface, currentSurface,
                            prevLat, prevLon, lat, lon, prevOrientation)) {
                return sign;
            }
            // E6: Road → non-road transition at a forced-path junction. The road ends
            // and a cycleway/footway/path picks up with no alternatives. The graph says
            // "no choice", but on the ground the rider faces a road end (vehicle turnaround
            // visual cue) and needs to know the cycleway/path ahead is the continuation.
            // Emit the angle-based sign so the rider gets a normal "continue" / turn
            // instruction at the transition. One-directional only — the reverse
            // (non-road → road) is intentionally not covered, since emerging onto a road
            // typically has clear visual cues.
            if (isRoadInfrastructure(prevPH) && isNonRoadTarget(currentPH)) {
                return sign;
            }
            return Instruction.IGNORE;
        }

        // --- Ferry transitions ---
        if (InstructionsHelper.isToFerry(roadEnv, prevRoadEnv)) return Instruction.FERRY;
        if (InstructionsHelper.isFromFerry(roadEnv, prevRoadEnv)) return Instruction.CONTINUE_ON_STREET;

        // --- Merge/split artifact: always ignore ---
        if (outgoingEdges.mergedOrSplitWay()) return Instruction.IGNORE;

        // =================================================================
        // EMIT signals — force instruction even if angle is small
        // =================================================================

        // E1: Removed. PH changes alone are not navigation decisions — the rider sees the
        // type change themselves. Real decision points (forks, T-junctions, turns) are
        // handled by fork/suppression logic below. PH change data is always in extraInfo
        // for the client to render contextually.

        // E4: bike_network appears or disappears — disabled, same reasoning as E1.
        // Bike network data is available in extraInfo for client-side rendering.
        // boolean bnAppeared = currentBN != null && currentBN != RouteNetwork.MISSING
        //         && (prevBN == null || prevBN == RouteNetwork.MISSING);
        // boolean bnDisappeared = (currentBN == null || currentBN == RouteNetwork.MISSING)
        //         && prevBN != null && prevBN != RouteNetwork.MISSING;
        // if (bnAppeared || bnDisappeared) {
        //     return sign != Instruction.IGNORE ? sign : Instruction.CONTINUE_ON_STREET;
        // }

        // E3: Named road → unnamed trail (or vice versa) + road class change — disabled,
        // same reasoning as E1. Name/class data is in extraInfo for the client.
        // boolean nameAppeared = (name != null && !name.isEmpty()) && (prevName == null || prevName.isEmpty());
        // boolean nameDisappeared = (name == null || name.isEmpty()) && (prevName != null && !prevName.isEmpty());
        // if ((nameAppeared || nameDisappeared) && currentRC != prevRC) {
        //     return sign != Instruction.IGNORE ? sign : Instruction.CONTINUE_ON_STREET;
        // }

        // =================================================================
        // Clear turn: |sign| > 1 (actual angle change)
        // =================================================================
        if (Math.abs(sign) > 1) {
            // S1/S2 same-name suppression precondition: at least one side of the junction
            // must be road infrastructure (motorway/major/minor/service road). Same-name
            // suppression relies on names being physically sign-posted so the rider can see
            // they're still on the same way. That holds on roads but not on cycleways /
            // footways / paths / tracks (informal route names that aren't observable on the
            // ground). Pure non-road junctions are handled by other rules (S3 prominence,
            // S5/S6 trail logic, E2 fork). Transitions in/out of road keep S1/S2 active.
            boolean s1s2ScopeOk = isRoadInfrastructure(currentPH) || isRoadInfrastructure(prevPH);

            // S1: Same PH + same name + all alternatives visually distinct → road bending.
            // "Visually distinct" requires BOTH a different PH bucket (per isConfusableFrom)
            // AND a clearly different surface (asphalt vs known non-asphalt; ASPHALT_OR_UNPAVED
            // is ambiguous and never counts as distinguishing).
            if (s1s2ScopeOk
                    && InstructionsHelper.isSameName(name, prevName)
                    && currentPH != null && currentPH == prevPH
                    && allAlternativesVisuallyDistinct(outgoingEdges, edge, currentPH)) {
                return Instruction.IGNORE;
            }

            // S2: Same name + same RoadClass + all alternatives visually distinct → road curves.
            // Uses the same visual-distinctness helper as S1 — strict RC equality of alts is
            // not enough (cycleway/service_road are visually similar when both unpaved).
            if (s1s2ScopeOk
                    && InstructionsHelper.isSameName(name, prevName)
                    && currentRC == prevRC
                    && allAlternativesVisuallyDistinct(outgoingEdges, edge, currentPH)) {
                return Instruction.IGNORE;
            }

            // GH fallback: same name + alternatives slower by 2x
            if (InstructionsHelper.isSameName(name, prevName)
                    && outgoingEdges.outgoingEdgesAreSlowerByFactor(2)) {
                return Instruction.IGNORE;
            }

            return sign;
        }

        // =================================================================
        // Ambiguous: |sign| <= 1 (going more or less straight)
        // =================================================================
        if (prevEdge == null) {
            return sign;
        }

        boolean outgoingEdgesAreSlower = outgoingEdges.outgoingEdgesAreSlowerByFactor(1);
        EdgeIteratorState otherContinue = outgoingEdges.getOtherContinue(prevLat, prevLon, prevOrientation);
        double delta = InstructionsHelper.calculateOrientationDelta(prevLat, prevLon, lat, lon, prevOrientation);

        // S4: Name changes but same PH + near zero angle + no fork → municipal boundary
        // The "no fork" guard is otherContinue==null (no near-straight alt) AND no E2-confusable
        // alt at wider angle — getOtherContinue alone misses same-PH/same-surface forks at e.g. +50°.
        if (!InstructionsHelper.isSameName(name, prevName)
                && currentPH != null && currentPH == prevPH
                && Math.abs(delta) < 0.15
                && otherContinue == null
                && !hasConfusableAlternative(outgoingEdges, edge, currentPH)) {
            return Instruction.IGNORE;
        }

        // S5: Slight turn on same road (same name + same class)
        // Only suppresses on named ways or road-like infrastructure — unnamed trails need fork guidance.
        // CYCLEWAY excluded from both: unpaved Finnish cycleways have informal names (route
        // names like "Pyynikin rantapolku") that don't help distinguish Y-junction branches.
        // Competing-alt guard (road-infrastructure only): S5's "stay on the named road"
        // premise breaks when the junction holds a same-RC alt the rider perceives as a
        // competing branch — either an opposite-side Y-fork co-branch or a same-side
        // inertia-trap alt that goes straighter than the route. Each helper uses a
        // tiered forward cone: asymmetric surface attractor (alt continues prev surface
        // while route diverges) opens the wider 75° cone; symmetric same-surface
        // junctions stay at the 40° slight bucket. When a competing alt is detected
        // S5 falls through and the downstream emit path (fork handler / F1 / leaving-
        // current-street fallback) handles the instruction.
        if (Math.abs(sign) == 1
                && InstructionsHelper.isSameName(name, prevName)
                && currentRC == prevRC) {
            boolean hasName = name != null && !name.isEmpty();
            boolean isCycleway = currentPH == PredictedHighway.CYCLEWAY;
            boolean isRoadLike = currentPH != null && isRoadInfrastructure(currentPH);
            if ((hasName && !isCycleway) || isRoadLike) {
                boolean hasCompetingAlt = isRoadLike
                        && (hasCompetingOppositeSideRoadAlt(outgoingEdges, currentRC, prevSurface, currentSurface, delta)
                            || hasStraighterRoadAlt(outgoingEdges, currentRC, prevSurface, currentSurface, delta));
                if (!hasCompetingAlt) {
                    // Visual check: route bends while a visually similar alternative goes straighter.
                    if (hasVisualCandidateAlternative(outgoingEdges, edge, currentPH, currentSurface, delta)) {
                        visualCandidateSign = sign;
                    }
                    return Instruction.IGNORE;
                }
                // Competing alt detected — fall through to the standard emit path.
            }
        }

        // S3: All alternatives are lower prominence → obviously staying on main way.
        // Guard 1: rider must be staying on (or descending from) something at least as prominent
        // as the route ahead. When prevPH < currentPH the rider is *joining* a more prominent
        // way — that transition is itself the navigation event and needs an instruction.
        // Guard 2 (road branch only): on road-infrastructure junctions the route's name must
        // continue from prev to current. A name change across a road junction is a perceptual
        // event the rider sees from street signs and intersection geometry (e.g.
        // Kalliojärventie → Houkkalammintie, with the same-name continuation existing as
        // a lower-prominence alt). Prominence alone can't distinguish "route stays on the
        // named way while a side path joins" (silence-worthy) from "route leaves the named
        // way while the same-named continuation goes to a side path" (emit-worthy) — both
        // satisfy the prominence guards. Mirrors the same-name precondition that S1/S2/S5
        // already enforce. Scoped to road infrastructure because cycleway / footway / path /
        // track names are informal route designations not observable on the ground (W3).
        // Guard 3 (non-road only): when the rider is on non-road infrastructure (trails,
        // cycleways, paths) prominence alone is not a reliable visual cue — an unpaved
        // cycleway and a same-surface footway can be visually identical despite different
        // PH. On non-road, S3 escapes (emits the angle-based sign) for two distinct
        // perceptual shapes:
        //   Shape A — Inertia-trap (hasForwardConfusableAlt): a visually-similar alt
        //     goes noticeably straighter than the route, so the rider's go-forward
        //     default would land on the wrong branch unless warned.
        //   Shape B — Y-fork (hasYForkConfusableAlt): a visually-similar alt sits forward
        //     at the junction with no clear "obvious continuation" claim; the rider
        //     faces a real left-or-right choice between two similar-looking ways.
        //     Forward-cone bounded, spread vs route gated by route deviation — see the
        //     helper docstring for the perceptual rationale and thresholds.
        // The two shapes can overlap mathematically (the escape fires either way); they
        // are kept named separately to describe the two distinct rider experiences.
        if (currentPH != null && prevPH != null
                && phProminence(prevPH) >= phProminence(currentPH)
                && allAlternativesLowerProminence(outgoingEdges, currentPH)) {
            boolean bothNonRoad = !isRoadInfrastructure(currentPH) && !isRoadInfrastructure(prevPH);
            // Road-or-mixed branch: require name continuity (W8). A name change at a
            // road-infrastructure junction is the perceptual signal of leaving the named
            // way; defer to downstream emit path (fork handler / leaving-current-street).
            if (!bothNonRoad && !InstructionsHelper.isSameName(name, prevName)) {
                // Fall through — do not silence.
            } else if (bothNonRoad
                    && (hasForwardConfusableAlt(outgoingEdges, edge, currentPH, delta,
                                prevLat, prevLon, prevOrientation)
                        || hasYForkConfusableAlt(outgoingEdges, edge, currentPH, delta,
                                prevLat, prevLon, prevOrientation))) {
                return sign;
            } else {
                // Road-or-mixed silence: opt into the visual side-channel if a visually
                // similar alt goes straighter than the route (W10 — parallel to S5's opt-in).
                // Non-road silence stays without visual: name signals are unreliable on
                // non-road infrastructure (W3), and the Shape A/B escapes are the
                // perceptual gate there.
                if (!bothNonRoad
                        && hasVisualCandidateAlternative(outgoingEdges, edge, currentPH, currentSurface, delta)) {
                    visualCandidateSign = sign;
                }
                return Instruction.IGNORE;
            }
        }

        // S6: Forced trail bend — both prev and current are non-road-infrastructure AND
        // no alternative is a viable forward continuation (no alt within ±90° of incoming).
        // On trails/cycleways large geometric angles are often just path curvature, and
        // back-leg / U-turn-shaped alts are not real navigation choices. This suppresses
        // the leaving-current-street fallback's |Δ|>0.6 and name-based triggers (the
        // latter being unreliable on non-road-infra where names are informal route
        // designations not visible on the ground).
        if (currentPH != null && prevPH != null
                && !isRoadInfrastructure(currentPH)
                && !isRoadInfrastructure(prevPH)
                && !hasViableForwardAlternative(outgoingEdges, prevLat, prevLon, prevOrientation)) {
            return Instruction.IGNORE;
        }

        // --- Fork handling (GH logic + extended for all road classes) ---
        if (otherContinue != null) {
            if (!InstructionsHelper.isSameName(name, prevName)
                    || !InstructionsHelper.isSameName(destinationAndRef, prevDestinationAndRef)
                    || InstructionsHelper.isSameName(otherContinue.getName(), prevName)
                    || !outgoingEdgesAreSlower) {

                final RoadClass otherRoadClass = otherContinue.get(roadClassEnc);
                final boolean link = edge.get(roadClassLinkEnc);
                final boolean prevLink = prevEdge.get(roadClassLinkEnc);
                final boolean otherLink = otherContinue.get(roadClassLinkEnc);

                // Staying on same prominent road class with route's name continuing
                // (GH original, extended to all classes, gated by name continuity per W9).
                // Name guard mirrors the S3 fix: the rule's premise — "route stays on the
                // main way, alt diverges to a lower class" — assumes the rider IS staying
                // on the named way. Without the guard, a name change with a same-RC route
                // and a different-RC alt (e.g. Kalliojärventie → Houkkalammintie at a
                // junction where the same-name continuation is a SERVICE_ROAD alt)
                // silences a real "leaving the named way" event.
                if (InstructionsHelper.isSameName(name, prevName)
                        && (currentRC == prevRC && link == prevLink)
                        && (otherRoadClass != prevRC || otherLink != prevLink)) {
                    // On trails, only suppress if the alternative is visibly different type
                    if (currentPH != null) {
                        PredictedHighway otherPH = predictedHighwayEnc != null
                                ? otherContinue.get(predictedHighwayEnc) : null;
                        if (otherPH != null && otherPH != currentPH) {
                            return Instruction.IGNORE;
                        }
                    } else {
                        // No PH data: use GH's original major-road-only check
                        if (currentRC == RoadClass.MOTORWAY || currentRC == RoadClass.TRUNK
                                || currentRC == RoadClass.PRIMARY || currentRC == RoadClass.SECONDARY
                                || currentRC == RoadClass.TERTIARY) {
                            return Instruction.IGNORE;
                        }
                    }
                }

                GHPoint tmpPoint = InstructionsHelper.getPointForOrientationCalculation(otherContinue, nodeAccess);
                double otherDelta = InstructionsHelper.calculateOrientationDelta(prevLat, prevLon,
                        tmpPoint.getLat(), tmpPoint.getLon(), prevOrientation);

                // Clearly continuing same road (GH original)
                if (Math.abs(delta) < .1 && Math.abs(otherDelta) > .15
                        && InstructionsHelper.isSameName(name, prevName)) {
                    return Instruction.CONTINUE_ON_STREET;
                }

                // F2: Going dead-straight and the fork is visually distinguishable
                // (different PH or different surface major type) → no instruction needed.
                // Only when staying on the same road type — if the route switches type
                // (e.g., cycleway → path), the rider needs guidance even if geometry is straight.
                if (Math.abs(delta) < .1 && Math.abs(otherDelta) > .15
                        && currentPH != null && currentPH == prevPH) {
                    PredictedHighway otherPH = predictedHighwayEnc != null
                            ? otherContinue.get(predictedHighwayEnc) : null;
                    boolean phDistinguishable = currentPH != null && otherPH != null && otherPH != currentPH;
                    boolean surfaceDistinguishable = false;
                    if (predictedSurfaceEnc != null) {
                        PredictedSurface otherSurface = otherContinue.get(predictedSurfaceEnc);
                        surfaceDistinguishable = isAsphalt(currentSurface) != isAsphalt(otherSurface);
                    }
                    if (phDistinguishable || surfaceDistinguishable) {
                        return Instruction.IGNORE;
                    }
                }

                // F1: Road type transition — route leaves one road type for another.
                // Use angle-based sign instead of fork-relative KEEP_LEFT/RIGHT.
                if (currentPH != null && prevPH != null) {
                    boolean phChanges = currentPH != prevPH;
                    boolean surfaceTypeChanges = currentSurface != null && prevSurface != null
                            && isAsphalt(currentSurface) != isAsphalt(prevSurface);
                    if (phChanges || surfaceTypeChanges) {
                        // At a gentle fork where both ways go nearly straight, angle-based
                        // sign=0 ("straight") is misleading — the rider can't tell which of
                        // the two near-straight options to take. Force TURN_SLIGHT in the
                        // fork-relative direction, unless a third way on the same side
                        // could cause confusion.
                        if (sign == 0 && Math.abs(delta) < 0.5 && Math.abs(otherDelta) < 0.5) {
                            boolean routeIsLeft = otherDelta < delta;
                            if (!hasAlternativeOnSameSide(outgoingEdges, edge, otherContinue,
                                    routeIsLeft, prevLat, prevLon, prevOrientation)) {
                                return routeIsLeft ? Instruction.TURN_SLIGHT_LEFT : Instruction.TURN_SLIGHT_RIGHT;
                            }
                        }
                        return sign;
                    }
                }

                if (otherDelta < delta) {
                    return Instruction.KEEP_LEFT;
                } else {
                    return Instruction.KEEP_RIGHT;
                }
            }
        }

        // --- Leaving current street (GH fallback) ---
        if (!outgoingEdgesAreSlower
                && !outgoingEdges.mergedOrSplitWay()
                && (Math.abs(delta) > .6 || outgoingEdges.isLeavingCurrentStreet(prevName, name))) {
            return sign;
        }

        // E2: Trail/cycleway fork — going straight past a confusable alternative.
        // Applies to unnamed trails AND named cycleways (cycleway names are informal route
        // names like "Pyynikin rantapolku" that don't help distinguish Y-junction branches).
        // Three gates: (a) asymmetric type confusability, (b) surface (asphalt vs not),
        // (c) angle from incoming < 60° (same type) or < 45° (confusable but different type).
        if (currentPH != null && !isRoadInfrastructure(currentPH)
                && (name == null || name.isEmpty() || currentPH == PredictedHighway.CYCLEWAY)
                && hasConfusableAlternative(outgoingEdges, edge, currentPH)) {
            // trail_fork tagging happens in enrichExtraInfo from junction state,
            // not from a getTurn-side flag — so any rule that emits at this kind
            // of junction picks up the tag.
            return Instruction.CONTINUE_ON_STREET;
        }

        return Instruction.IGNORE;
    }

    // ========================================================================
    // getTurn helper methods
    // ========================================================================

    /**
     * Check if any accessible alternative (other than the route edge and the other continuing
     * edge) exists on the same side as the route at this fork, within 90° of straight ahead.
     * Used by F1 to guard against ambiguous "slight left/right" when a third way on the
     * same side could confuse the rider.
     */
    private boolean hasAlternativeOnSameSide(InstructionsOutgoingEdges outgoing,
                                              EdgeIteratorState routeEdge, EdgeIteratorState otherContinue,
                                              boolean routeIsLeft, double prevLat, double prevLon,
                                              double prevOrientation) {
        for (EdgeIteratorState alt : outgoing.getAllowedAlternativeTurns()) {
            if (alt.getEdge() == routeEdge.getEdge()) continue;
            if (otherContinue != null && alt.getEdge() == otherContinue.getEdge()) continue;
            GHPoint altPoint = InstructionsHelper.getPointForOrientationCalculation(alt, nodeAccess);
            double altDelta = InstructionsHelper.calculateOrientationDelta(
                    prevLat, prevLon, altPoint.getLat(), altPoint.getLon(), prevOrientation);
            if (Math.abs(altDelta) > Math.PI / 2) continue; // behind us, not confusing
            boolean altIsLeft = altDelta > 0;
            if (altIsLeft == routeIsLeft) return true;
        }
        return false;
    }

    /**
     * Check if all accessible alternative edges are visually distinct from the route edge.
     * <p>
     * An alternative is visually distinct only when BOTH:
     * <ul>
     *   <li>its PredictedHighway is in a different visual bucket (per {@link #isConfusableFrom}), AND</li>
     *   <li>its surface clearly differs from the route's (per {@link #surfacesClearlyDiffer} —
     *       ASPHALT_OR_UNPAVED is ambiguous and never counts as distinguishing).</li>
     * </ul>
     * If either signal is ambiguous the alt is treated as potentially confusing and the
     * helper returns false. Returns false also if PH data is unavailable.
     */
    private boolean allAlternativesVisuallyDistinct(InstructionsOutgoingEdges outgoing,
                                                     EdgeIteratorState routeEdge,
                                                     PredictedHighway currentPH) {
        if (predictedHighwayEnc == null || currentPH == null) return false;
        PredictedSurface currentSurface = predictedSurfaceEnc != null ? routeEdge.get(predictedSurfaceEnc) : null;
        for (EdgeIteratorState alt : outgoing.getAllowedAlternativeTurns()) {
            PredictedHighway altPH = alt.get(predictedHighwayEnc);
            if (isConfusableFrom(currentPH, altPH)) return false;
            if (predictedSurfaceEnc != null) {
                PredictedSurface altSurface = alt.get(predictedSurfaceEnc);
                if (!surfacesClearlyDiffer(currentSurface, altSurface)) return false;
            }
        }
        return true;
    }

    /**
     * Whether two predicted surfaces clearly differ on the asphalt/non-asphalt visual axis.
     * ASPHALT_OR_UNPAVED is treated as ambiguous: it could be either, so it never counts
     * as distinguishing.
     */
    private static boolean surfacesClearlyDiffer(PredictedSurface a, PredictedSurface b) {
        if (a == PredictedSurface.ASPHALT_OR_UNPAVED || b == PredictedSurface.ASPHALT_OR_UNPAVED) return false;
        return isAsphalt(a) != isAsphalt(b);
    }

    /**
     * Visual guidance check: does any alternative go significantly straighter than the route,
     * while being visually similar (non-road type, similar surface)?
     * <p>
     * Three conditions (all must be true for at least one alternative):
     * <ol>
     *   <li>Angular gap: the route deviates at least ~20° more from straight than the alternative</li>
     *   <li>Type proximity: both route and alternative are non-road-infrastructure (trail/path/cycleway)</li>
     *   <li>Surface similarity: not obviously different (both asphalt or both non-asphalt)</li>
     * </ol>
     */
    private boolean hasVisualCandidateAlternative(InstructionsOutgoingEdges outgoing,
                                                   EdgeIteratorState routeEdge,
                                                   PredictedHighway currentPH,
                                                   PredictedSurface currentSurface,
                                                   double routeDelta) {
        if (predictedHighwayEnc == null) return false;
        // Only for non-road types — road infrastructure has signs, markings, etc.
        if (currentPH == null || isRoadInfrastructure(currentPH)) return false;

        double routeDeviation = Math.abs(routeDelta);

        for (EdgeIteratorState alt : outgoing.getAllowedAlternativeTurns()) {
            PredictedHighway altPH = alt.get(predictedHighwayEnc);

            // Type proximity: alternative must also be non-road-infrastructure
            if (altPH == null || isRoadInfrastructure(altPH)) continue;

            // Surface similarity: asphalt vs non-asphalt is visually distinct
            if (predictedSurfaceEnc != null) {
                PredictedSurface altSurface = alt.get(predictedSurfaceEnc);
                if (isAsphalt(currentSurface) != isAsphalt(altSurface)) continue;
            }

            // Angular gap: alternative goes meaningfully straighter than route
            GHPoint altPoint = InstructionsHelper.getPointForOrientationCalculation(alt, nodeAccess);
            double altDelta = InstructionsHelper.calculateOrientationDelta(
                    prevLat, prevLon, altPoint.getLat(), altPoint.getLon(), prevOrientation);
            double altDeviation = Math.abs(altDelta);
            double angularGap = routeDeviation - altDeviation;

            // Route must deviate at least 0.35 rad (~20°) more than the alternative
            if (angularGap > 0.35) return true;
        }
        return false;
    }

    /**
     * E5 helper. Inside the forced-path block ({@code nrOfPossibleTurns <= 1}) the
     * weighting-based alternative counts hide a footway that is bike-blocked but
     * visually indistinguishable from the cycleway the bike is on. Walk
     * {@link #allExplorer} (unfiltered) to find such an alternative regardless of
     * access tags.
     * <p>
     * Returns true iff at least one graph-incident edge at {@code baseNode} (other
     * than the route edge and the previous edge) satisfies:
     * <ol>
     *   <li>{@code predicted_highway == FOOTWAY},</li>
     *   <li>same major surface as <em>either</em> the rider's prev edge or the route's
     *       next edge ({@code isAsphalt(alt)} matches {@code isAsphalt(prev)} OR
     *       {@code isAsphalt(current)}). The rider sees both surfaces at the junction,
     *       so an alt visually matching either is a real confusable.</li>
     *   <li>{@code |altDelta| ≤ |routeDelta| + π/6} — the alt is no more than ~30°
     *       further off the rider's forward line than the route.</li>
     * </ol>
     * <p>
     * Condition 3 captures both confusion patterns:
     * <ul>
     *   <li>alt much straighter than route (rider's go-straight inertia trap), and</li>
     *   <li>alt and route at similar-magnitude turns (symmetric T-junction with no
     *       obvious "default" choice).</li>
     * </ul>
     * It rejects alts that are clearly further off-line than the route — the rider
     * can distinguish those by eye.
     */
    private boolean hasConfusableFootwayAlternative(EdgeIteratorState routeEdge,
                                                     int baseNode,
                                                     PredictedSurface prevSurface,
                                                     PredictedSurface currentSurface,
                                                     double prevLat, double prevLon,
                                                     double routeLat, double routeLon,
                                                     double prevOrientation) {
        if (predictedHighwayEnc == null) return false;

        double routeDelta = InstructionsHelper.calculateOrientationDelta(
                prevLat, prevLon, routeLat, routeLon, prevOrientation);
        final double CONFUSABILITY_TOLERANCE = Math.PI / 6; // ~30°

        EdgeIterator iter = allExplorer.setBaseNode(baseNode);
        while (iter.next()) {
            if (iter.getEdge() == routeEdge.getEdge()) continue;
            if (prevEdge != null && iter.getEdge() == prevEdge.getEdge()) continue;

            if (iter.get(predictedHighwayEnc) != PredictedHighway.FOOTWAY) continue;

            if (predictedSurfaceEnc != null) {
                PredictedSurface altSurface = iter.get(predictedSurfaceEnc);
                boolean altAsphalt = isAsphalt(altSurface);
                boolean matchesPrev = altAsphalt == isAsphalt(prevSurface);
                boolean matchesCurrent = altAsphalt == isAsphalt(currentSurface);
                if (!matchesPrev && !matchesCurrent) continue;
            }

            GHPoint altPt = InstructionsHelper.getPointForOrientationCalculation(iter, nodeAccess);
            double altDelta = InstructionsHelper.calculateOrientationDelta(
                    prevLat, prevLon, altPt.getLat(), altPt.getLon(), prevOrientation);

            if (Math.abs(altDelta) > Math.abs(routeDelta) + CONFUSABILITY_TOLERANCE) continue;
            return true;
        }
        return false;
    }

    /**
     * Check if all accessible alternatives have strictly lower prominence than current.
     * "Prominence" is a Trailmap-defined ordering of PredictedHighway values representing
     * how visually prominent and navigation-relevant a way type is.
     */
    private boolean allAlternativesLowerProminence(InstructionsOutgoingEdges outgoing, PredictedHighway currentPH) {
        if (predictedHighwayEnc == null) return false;
        int currentProm = phProminence(currentPH);
        if (currentProm <= 1) return false; // can't be lower than the bottom
        for (EdgeIteratorState alt : outgoing.getAllowedAlternativeTurns()) {
            if (phProminence(alt.get(predictedHighwayEnc)) >= currentProm) return false;
        }
        return true;
    }

    /**
     * E2 gate: check if any alternative is confusable with the current way.
     * Three-part test per alternative:
     * 1. Asymmetric type confusability (from current PH, could rider mistake the alt?)
     * 2. Surface gate: asphalt vs non-asphalt → never confusable
     * 3. Angle gate: alt must diverge < 60° (same type) or < 45° (different but confusable)
     */
    private boolean hasConfusableAlternative(InstructionsOutgoingEdges outgoing,
                                             EdgeIteratorState routeEdge,
                                             PredictedHighway currentPH) {
        if (predictedHighwayEnc == null) return false;

        PredictedSurface routeSurface = predictedSurfaceEnc != null
                ? routeEdge.get(predictedSurfaceEnc) : null;

        for (EdgeIteratorState alt : outgoing.getAllowedAlternativeTurns()) {
            PredictedHighway altPH = alt.get(predictedHighwayEnc);

            // Gate 1: asymmetric type confusability
            if (!isConfusableFrom(currentPH, altPH)) continue;

            // Gate 2: surface — asphalt vs non-asphalt is always distinguishable
            if (predictedSurfaceEnc != null) {
                PredictedSurface altSurface = alt.get(predictedSurfaceEnc);
                if (isAsphalt(routeSurface) != isAsphalt(altSurface)) continue;
            }

            // Gate 3: angle — compute alt bearing relative to incoming direction
            GHPoint altPoint = InstructionsHelper.getPointForOrientationCalculation(alt, nodeAccess);
            double altDelta = InstructionsHelper.calculateOrientationDelta(
                    prevLat, prevLon, altPoint.getLat(), altPoint.getLon(), prevOrientation);
            double altAngleDeg = Math.toDegrees(Math.abs(altDelta));

            // Tighter threshold when alt is different type (subtle visual cues may help)
            double threshold = (altPH == currentPH) ? 60.0 : 45.0;
            if (altAngleDeg < threshold) return true;
        }
        return false;
    }

    /**
     * S3 escape gate: returns true iff at least one alt is a real "go-straight inertia
     * trap" relative to the route. Three AND-mandatory gates per alt:
     * <ol>
     *   <li><b>PH bucket</b> — {@code isConfusableFrom(currentPH, altPH)} must be true.
     *       This is the layer-2 visual-bucket signal: the alt must be the kind of way the
     *       rider could plausibly mistake for their continuation.</li>
     *   <li><b>Surface</b> — same major surface (asphalt vs non-asphalt) per
     *       {@link #surfacesClearlyDiffer}. ASPHALT_OR_UNPAVED is ambiguous and counts as
     *       a match either way.</li>
     *   <li><b>Angle</b> — alt is meaningfully straighter than the route:
     *       {@code |routeDelta| - |altDelta| > ~0.35 rad (≈20°)}. This is the rider's
     *       inertia trap pattern: going straight would land on the alt while the route
     *       bends off. A side branch — alt at 50–90° off forward while route is gentle —
     *       fails this gate and is not a real trap.</li>
     * </ol>
     * Same shape as {@link #hasConfusableAlternative} (used by S4/E2/F1) for the PH and
     * surface gates, and same straighter-than-route gate as
     * {@link #hasVisualCandidateAlternative} (S5's visual side-channel).
     */
    private boolean hasForwardConfusableAlt(InstructionsOutgoingEdges outgoing,
                                             EdgeIteratorState routeEdge,
                                             PredictedHighway currentPH,
                                             double routeDelta,
                                             double prevLat, double prevLon,
                                             double prevOrientation) {
        if (predictedHighwayEnc == null || currentPH == null) return false;
        PredictedSurface routeSurface = predictedSurfaceEnc != null
                ? routeEdge.get(predictedSurfaceEnc) : null;
        double routeDeviation = Math.abs(routeDelta);
        final double STRAIGHTER_THRESHOLD = 0.35; // ~20°
        for (EdgeIteratorState alt : outgoing.getAllowedAlternativeTurns()) {
            // (a) PH bucket — alt must be confusable per layer-2 table
            PredictedHighway altPH = alt.get(predictedHighwayEnc);
            if (!isConfusableFrom(currentPH, altPH)) continue;

            // (b) surface — must not clearly differ
            if (predictedSurfaceEnc != null) {
                PredictedSurface altSurface = alt.get(predictedSurfaceEnc);
                if (surfacesClearlyDiffer(routeSurface, altSurface)) continue;
            }

            // (c) angle — alt is meaningfully straighter than the route
            GHPoint altPoint = InstructionsHelper.getPointForOrientationCalculation(alt, nodeAccess);
            double altDelta = InstructionsHelper.calculateOrientationDelta(
                    prevLat, prevLon, altPoint.getLat(), altPoint.getLon(), prevOrientation);
            if (routeDeviation - Math.abs(altDelta) > STRAIGHTER_THRESHOLD) return true;
        }
        return false;
    }

    /**
     * S3 escape gate for the Y-fork shape: returns true iff at least one alt is a
     * visually-similar forward co-branch where the rider perceives a real left-or-right
     * choice between two similar-looking ways. Complementary to
     * {@link #hasForwardConfusableAlt} (the inertia-trap shape, where the alt is
     * dominantly straighter than the route).
     * <p>
     * Three AND-mandatory gates per alt:
     * <ol>
     *   <li><b>PH bucket</b> — {@code isConfusableFrom(currentPH, altPH)} must be true.
     *       The alt must be the kind of way the rider could mistake for their continuation.</li>
     *   <li><b>Surface</b> — same major surface (asphalt vs non-asphalt) per
     *       {@link #surfacesClearlyDiffer}. ASPHALT_OR_UNPAVED is ambiguous and counts
     *       as a match either way.</li>
     *   <li><b>Spread vs. route, gated by route deviation</b> — the rider's perception
     *       of "fork" depends on whether the route gives a clear "straight" reference:
     *       <ul>
     *         <li><b>Route near-straight</b> ({@code |routeDelta| ≤ ~11°}): alt's
     *             absolute angle from straight must be ≤ ~30°. The rider has an
     *             unambiguous straight reference; an alt further than ~30° from
     *             straight reads as a side-branch, not a Y-limb.</li>
     *         <li><b>Route in slight-turn zone</b> ({@code |routeDelta| > ~11°}):
     *             spread {@code |routeDelta − altDelta| ≤ ~55°} AND alt's absolute
     *             angle ≤ ~60° (forward cone). Neither branch is "the straight one";
     *             both naturally sit in the slight area and the rider perceives them
     *             as a Y even with the wider spread.</li>
     *       </ul></li>
     * </ol>
     * No lower bound on spread: the few-meter angle sample we read at the junction
     * node is not a trustworthy basis for filtering small-spread junctions, because
     * real Y-forks can start with a sliver of an angle at the node and diverge
     * downstream within 10–20 meters. The rider sees the diverging geometry; the
     * node-local angle does not.
     */
    private boolean hasYForkConfusableAlt(InstructionsOutgoingEdges outgoing,
                                           EdgeIteratorState routeEdge,
                                           PredictedHighway currentPH,
                                           double routeDelta,
                                           double prevLat, double prevLon,
                                           double prevOrientation) {
        if (predictedHighwayEnc == null || currentPH == null) return false;
        PredictedSurface routeSurface = predictedSurfaceEnc != null
                ? routeEdge.get(predictedSurfaceEnc) : null;

        // Route-deviation tier boundary. Matches the calculateSign() straight/slight
        // boundary used everywhere else in the rule chain (|delta| < 0.2 rad ≈ 11°).
        final double STRAIGHT_TIER = 0.2;
        // Tier A — route near-straight: alt's absolute angle from straight must be small.
        final double NEAR_STRAIGHT_ALT_MAX = 0.52;  // ~30°
        // Tier B — route in slight-turn zone: spread between route and alt, plus
        // forward-cone bound on alt's absolute angle.
        final double SLIGHT_SPREAD_MAX    = 0.96;   // ~55°
        final double FORWARD_CONE_MAX     = 1.05;   // ~60°

        double routeDeviation = Math.abs(routeDelta);
        boolean routeNearStraight = routeDeviation <= STRAIGHT_TIER;

        for (EdgeIteratorState alt : outgoing.getAllowedAlternativeTurns()) {
            // (a) PH bucket
            PredictedHighway altPH = alt.get(predictedHighwayEnc);
            if (!isConfusableFrom(currentPH, altPH)) continue;

            // (b) surface
            if (predictedSurfaceEnc != null) {
                PredictedSurface altSurface = alt.get(predictedSurfaceEnc);
                if (surfacesClearlyDiffer(routeSurface, altSurface)) continue;
            }

            // (c) angle: tier-dependent spread bound
            GHPoint altPoint = InstructionsHelper.getPointForOrientationCalculation(alt, nodeAccess);
            double altDelta = InstructionsHelper.calculateOrientationDelta(
                    prevLat, prevLon, altPoint.getLat(), altPoint.getLon(), prevOrientation);
            double altAbs = Math.abs(altDelta);

            if (routeNearStraight) {
                // Route gives a clear straight reference; alt's distance from straight
                // is what determines whether it reads as a co-branch or a side-branch.
                if (altAbs <= NEAR_STRAIGHT_ALT_MAX) return true;
            } else {
                // No clean straight reference; spread between route and alt is the
                // perceptual signal. Forward cone still bounds the alt's absolute
                // angle to keep clear side-turns out.
                if (altAbs > FORWARD_CONE_MAX) continue;
                double spread = Math.abs(routeDelta - altDelta);
                if (spread <= SLIGHT_SPREAD_MAX) return true;
            }
        }
        return false;
    }

    /**
     * S5 competing-alt detector — opposite-side Y-fork shape on a road-infrastructure
     * junction. Returns true iff the junction has an accessible alternative the rider
     * perceives as a competing same-class road on the opposite side from the route,
     * making S5's "slight bend on same named road = no decision" silence wrong.
     * <p>
     * Three AND gates per alt:
     * <ol>
     *   <li><b>Same RC</b> as the route. Mirrors S5's own {@code currentRC==prevRC}
     *       discriminator — we ask whether a competing road of the same class exists,
     *       not whether a side service driveway is incident.</li>
     *   <li><b>Opposite side</b> from route: {@code sign(altΔ) != sign(routeΔ)}.</li>
     *   <li><b>Tiered forward cone</b> by surface-attraction strength:
     *       <ul>
     *         <li><b>Tier A — asymmetric attractor</b>: alt continues the rider's prev
     *             surface AND the route diverges to a different surface
     *             ({@code alt matches prev} AND {@code surfacesClearlyDiffer(prev, route)}).
     *             The alt visibly continues "the road I was on" while the route changes
     *             character; strongest surface attractor. Cone: {@code |altΔ| ≤ 75°} —
     *             surface continuity carries the rider's eye past the slight bucket.</li>
     *         <li><b>Tier B — symmetric / no surface discontinuity</b>: alt matches at
     *             least one surface anchor (prev or route) but Tier A's asymmetric
     *             attractor doesn't apply. Without the surface signal only the angular
     *             Y-shape is the cue; a wider opposite-side alt reads as a clear side
     *             turn, not a co-branch. Cone: {@code |altΔ| ≤ 40°} (slight bucket).</li>
     *         <li>Alt matching neither anchor: not competing (rider distinguishes by
     *             eye — e.g. unpaved branch off an all-asphalt road).</li>
     *       </ul>
     *       {@code ASPHALT_OR_UNPAVED} is ambiguous and matches either anchor (per
     *       {@link #surfacesClearlyDiffer}).</li>
     * </ol>
     * Dual-anchor surface logic mirrors {@link #hasConfusableFootwayAlternative} (E5):
     * the rider perceives both surfaces at the junction, so an alt visually matching
     * either is a real perceptual competitor. The tiered cone preserves the perceptual
     * grading of surface attractors as a scoring signal (not a flat yes/no gate).
     */
    private boolean hasCompetingOppositeSideRoadAlt(InstructionsOutgoingEdges outgoing,
                                                     RoadClass routeRC,
                                                     PredictedSurface prevSurface,
                                                     PredictedSurface routeSurface,
                                                     double routeDelta) {
        if (predictedSurfaceEnc == null || routeRC == null) return false;
        final double CONE_TIER_A = Math.toRadians(75);
        final double CONE_TIER_B = Math.toRadians(40);
        boolean routeChangesSurface = surfacesClearlyDiffer(prevSurface, routeSurface);
        for (EdgeIteratorState alt : outgoing.getAllowedAlternativeTurns()) {
            if (alt.get(roadClassEnc) != routeRC) continue;

            GHPoint altPoint = InstructionsHelper.getPointForOrientationCalculation(alt, nodeAccess);
            double altDelta = InstructionsHelper.calculateOrientationDelta(
                    prevLat, prevLon, altPoint.getLat(), altPoint.getLon(), prevOrientation);
            if (Math.signum(altDelta) == Math.signum(routeDelta)) continue;

            PredictedSurface altSurface = alt.get(predictedSurfaceEnc);
            boolean matchesPrev = !surfacesClearlyDiffer(prevSurface, altSurface);
            boolean matchesRoute = !surfacesClearlyDiffer(routeSurface, altSurface);
            if (!matchesPrev && !matchesRoute) continue;

            boolean tierA = routeChangesSurface && matchesPrev;
            double coneMax = tierA ? CONE_TIER_A : CONE_TIER_B;
            if (Math.abs(altDelta) <= coneMax) return true;
        }
        return false;
    }

    /**
     * S5 competing-alt detector — inertia-trap shape on a road-infrastructure
     * junction. Returns true iff there is a same-RC alt that goes meaningfully
     * straighter than the route, such that the rider's go-straight default would
     * land on the alt instead of following the route's slight bend.
     * <p>
     * Three AND gates per alt:
     * <ol>
     *   <li><b>Same RC</b> as the route.</li>
     *   <li><b>Straighter than route by &gt;20°</b>:
     *       {@code |routeΔ| − |altΔ| > 0.35 rad}. Same threshold as
     *       {@link #hasForwardConfusableAlt} (S3 Shape A).</li>
     *   <li><b>Tiered forward cone</b> on alt's absolute angle, by surface-attraction
     *       strength — same tier definitions as
     *       {@link #hasCompetingOppositeSideRoadAlt}.</li>
     * </ol>
     * Inside S5's {@code |sign|==1} scope the route is always in the slight bucket
     * so "straighter than route" naturally bounds alt below route; the cone is
     * largely redundant in practice but kept for symmetry with the Y-fork shape.
     */
    private boolean hasStraighterRoadAlt(InstructionsOutgoingEdges outgoing,
                                          RoadClass routeRC,
                                          PredictedSurface prevSurface,
                                          PredictedSurface routeSurface,
                                          double routeDelta) {
        if (predictedSurfaceEnc == null || routeRC == null) return false;
        final double CONE_TIER_A = Math.toRadians(75);
        final double CONE_TIER_B = Math.toRadians(40);
        final double STRAIGHTER_THRESHOLD = 0.35; // ~20°
        boolean routeChangesSurface = surfacesClearlyDiffer(prevSurface, routeSurface);
        double routeDeviation = Math.abs(routeDelta);
        for (EdgeIteratorState alt : outgoing.getAllowedAlternativeTurns()) {
            if (alt.get(roadClassEnc) != routeRC) continue;

            GHPoint altPoint = InstructionsHelper.getPointForOrientationCalculation(alt, nodeAccess);
            double altDelta = InstructionsHelper.calculateOrientationDelta(
                    prevLat, prevLon, altPoint.getLat(), altPoint.getLon(), prevOrientation);
            double altDeviation = Math.abs(altDelta);
            if (routeDeviation - altDeviation <= STRAIGHTER_THRESHOLD) continue;

            PredictedSurface altSurface = alt.get(predictedSurfaceEnc);
            boolean matchesPrev = !surfacesClearlyDiffer(prevSurface, altSurface);
            boolean matchesRoute = !surfacesClearlyDiffer(routeSurface, altSurface);
            if (!matchesPrev && !matchesRoute) continue;

            boolean tierA = routeChangesSurface && matchesPrev;
            double coneMax = tierA ? CONE_TIER_A : CONE_TIER_B;
            if (altDeviation <= coneMax) return true;
        }
        return false;
    }

    /**
     * Whether any non-route alternative at the junction is a "viable forward continuation"
     * — its outgoing direction is within ±90° of the incoming direction. Back-legs and
     * U-turn-shaped alts (>90° divergence) are not viable forward and don't constitute a
     * real navigation choice for the rider. Used by S6 (forced trail bend).
     */
    private boolean hasViableForwardAlternative(InstructionsOutgoingEdges outgoing,
                                                double prevLat, double prevLon,
                                                double prevOrientation) {
        for (EdgeIteratorState alt : outgoing.getAllowedAlternativeTurns()) {
            GHPoint altPoint = InstructionsHelper.getPointForOrientationCalculation(alt, nodeAccess);
            double altDelta = InstructionsHelper.calculateOrientationDelta(
                    prevLat, prevLon, altPoint.getLat(), altPoint.getLon(), prevOrientation);
            if (Math.abs(altDelta) < Math.PI / 2) return true;
        }
        return false;
    }

    private static boolean isAsphalt(PredictedSurface surface) {
        return surface == PredictedSurface.ASPHALT;
    }

    /**
     * Asymmetric type confusability: from the rider's perspective on {@code current},
     * could they mistake an alternative of type {@code alt} for their continuation?
     * <p>
     * Key asymmetry: from a narrow path, a wider track is confusable (could be the path
     * getting wider). From a wide cycleway, a narrow path is obviously different.
     */
    private static boolean isConfusableFrom(PredictedHighway current, PredictedHighway alt) {
        switch (current) {
            case PATH:
            case OUTDOOR_PATH:
                // From narrow paths: tracks and other paths look alike
                return alt == PredictedHighway.PATH || alt == PredictedHighway.OUTDOOR_PATH
                        || alt == PredictedHighway.ROUGH_TRACK || alt == PredictedHighway.GOOD_TRACK
                        || alt == PredictedHighway.FOOTWAY || alt == PredictedHighway.CITY_PATH;
            case FOOTWAY:
            case CITY_PATH:
                // Pedestrian/urban paths: similar to forest paths and rough tracks
                return alt == PredictedHighway.PATH || alt == PredictedHighway.OUTDOOR_PATH
                        || alt == PredictedHighway.FOOTWAY || alt == PredictedHighway.CITY_PATH
                        || alt == PredictedHighway.ROUGH_TRACK;
            case ROUGH_TRACK:
                // Rough tracks bridge narrow paths and wider maintained ways
                return alt == PredictedHighway.PATH || alt == PredictedHighway.OUTDOOR_PATH
                        || alt == PredictedHighway.ROUGH_TRACK || alt == PredictedHighway.GOOD_TRACK
                        || alt == PredictedHighway.CYCLEWAY || alt == PredictedHighway.OUTDOOR_WAY
                        || alt == PredictedHighway.SERVICE_ROAD;
            case GOOD_TRACK:
                // Wide tracks: confusable with other wide ways, NOT with narrow paths
                return alt == PredictedHighway.ROUGH_TRACK || alt == PredictedHighway.GOOD_TRACK
                        || alt == PredictedHighway.CYCLEWAY || alt == PredictedHighway.OUTDOOR_WAY
                        || alt == PredictedHighway.SERVICE_ROAD;
            case CYCLEWAY:
                // Wide maintained ways: confusable with each other and tracks. Also FOOTWAY:
                // an unpaved Finnish cycleway is often a 1 m gravel strip visually identical
                // to a same-surface footway running alongside or branching off. The layer-3
                // surface gate disambiguates the paved-cycleway case where width and
                // maintenance markings make the distinction visible.
                return alt == PredictedHighway.ROUGH_TRACK || alt == PredictedHighway.GOOD_TRACK
                        || alt == PredictedHighway.CYCLEWAY || alt == PredictedHighway.OUTDOOR_WAY
                        || alt == PredictedHighway.FOOTWAY;
            case OUTDOOR_WAY:
                // Wide maintained ways: confusable with each other and tracks, NOT paths
                return alt == PredictedHighway.ROUGH_TRACK || alt == PredictedHighway.GOOD_TRACK
                        || alt == PredictedHighway.CYCLEWAY || alt == PredictedHighway.OUTDOOR_WAY;
            case SERVICE_ROAD:
                // Vehicle-width unpaved: confusable with tracks
                return alt == PredictedHighway.ROUGH_TRACK || alt == PredictedHighway.GOOD_TRACK
                        || alt == PredictedHighway.SERVICE_ROAD;
            default:
                // Road types: only confusable with exact same type
                return alt == current;
        }
    }

    /**
     * Whether a PredictedHighway type is built road infrastructure where slight bends
     * are obviously the same road. Used by S5 to limit suppression to road-like types.
     * Excludes CYCLEWAY — unpaved Finnish cycleways are visually similar to outdoor paths/tracks.
     */
    private static boolean isRoadInfrastructure(PredictedHighway ph) {
        return ph == PredictedHighway.MOTORWAY
                || ph == PredictedHighway.MAJOR_ROAD
                || ph == PredictedHighway.MINOR_ROAD
                || ph == PredictedHighway.SERVICE_ROAD;
    }

    /**
     * Road infrastructure as the <em>reframer demote gate</em> sees it — same as
     * {@link #isRoadInfrastructure} but with {@code SERVICE_ROAD} excluded.
     * <p>
     * SERVICE_ROAD is physically in-between road and trail: often a narrow paved or
     * gravel lane visually indistinguishable from an OUTDOOR_WAY / track, with no
     * sign-posted name or class cue the rider can read on the ground. The reframer's
     * demote-to-CONTINUE / soft-real-turn rationale (geometric ambiguity is the
     * dominant signal, the rider scans the junction shape rather than absolute angle)
     * therefore applies to service roads exactly as it does to trails — e.g. a 47°
     * bend onto a service road reads as "slight right" when no confusable straight
     * alternative exists, just like the same bend on a track.
     * <p>
     * REFRAMER-LOCAL ON PURPOSE. This must NOT replace {@link #isRoadInfrastructure}
     * in the suppression chain (S1/S2/S3/S5, E6, enrichExtraInfo): there SERVICE_ROAD
     * keeps full road semantics (name continuity, prominence ordering, the road-infra
     * competing-alt guard). Only {@link #bothNonRoadAtJunction} uses this variant.
     */
    private static boolean isReframerHardRoad(PredictedHighway ph) {
        return ph == PredictedHighway.MOTORWAY
                || ph == PredictedHighway.MAJOR_ROAD
                || ph == PredictedHighway.MINOR_ROAD;
    }

    /**
     * Non-road continuations that the E6 forced-path anti-suppression pairs with the
     * road-side {@link #isRoadInfrastructure} types. Tight allowlist; extend by enum
     * addition if real-world data shows other non-road types need the same hint.
     */
    private static boolean isNonRoadTarget(PredictedHighway ph) {
        return ph == PredictedHighway.CYCLEWAY
                || ph == PredictedHighway.FOOTWAY
                || ph == PredictedHighway.PATH;
    }

    /**
     * Prominence ordering for PredictedHighway — higher value = more prominent way.
     * Used by S3 to suppress instructions when alternatives are visibly less important.
     */
    private static int phProminence(PredictedHighway ph) {
        if (ph == null) return 0;
        switch (ph) {
            case MOTORWAY:     return 10;
            case MAJOR_ROAD:   return 9;
            case MINOR_ROAD:   return 8;
            case CYCLEWAY:     return 7;
            case SERVICE_ROAD: return 6;
            case OUTDOOR_WAY:  return 5;
            case GOOD_TRACK:   return 5;
            case CITY_PATH:    return 4;
            case FOOTWAY:      return 4;
            case OUTDOOR_PATH: return 3;
            case ROUGH_TRACK:  return 3;
            case PATH:         return 2;
            case FERRY:        return 6;
            case UNKNOWN:      return 1;
            default:           return 0;
        }
    }

    // ========================================================================
    // Sign reframer (Stage 1 junction-relative reframing layer)
    // ========================================================================
    //
    // After getTurn() decides a sign, the reframer compares the route's angle
    // against the junction's visible-alt distribution and may adjust the sign
    // to better match rider perception. Six "shapes" of junction are
    // recognised; four rewrite the sign (Shape 1 = no competition, Shape 3 =
    // Y-fork with route on outside, Shape 4 = sandwich, Shape 6 = soft real
    // turn) and two pass through (Shape 2 = anchored straight reference,
    // Shape 5 = real turn beyond Shape 6's scope).
    //
    // Shape 4 (sandwich) has two demote branches sharing the same
    // alts-on-both-sides + non-road detection:
    //   (a) near-straight route → CONTINUE_ON_STREET
    //   (b) past-slight route + same-side more-extreme reference alt → demote
    //       magnitude one bucket to TURN_SLIGHT_<route's direction>. The
    //       reference alt is the rider's anchor for "real left/right" at the
    //       junction, so the route's smaller deviation reads as "slight".
    //
    // Shape 6 (soft real turn) demotes TURN_LEFT/RIGHT → TURN_SLIGHT_LEFT/RIGHT
    // at non-road junctions when the visible-alt distribution makes the turn
    // read as soft despite the geometric angle being past 40°. Two branches
    // share an asymmetric safety gate. The hard forbidden zone strictly
    // disqualifies the demote when any alt sits between the route's angle and
    // 20° on the opposite side. A soft zone on the opposite side covers alts
    // in [20°, 35°] — they don't strictly disqualify but modulate the route-
    // angle cap (40° at 20° up to 55° at 35°), capturing the Y-fork
    // perception of an opposite-side slight-bucket alt:
    //   (a) unanchored — base cap 55° (subject to the opposite-side soft
    //       modifier), no same-side anchor present.
    //   (b) anchored   — base cap dynamic (70° → 55° linearly as the closest
    //       same-side anchor grows from 100° to 130°, subject to the opposite-
    //       side soft modifier), with at least one same-side alt more extreme
    //       than the route by ≥30°. The anchor is the rider's "real left/right"
    //       reference; relabelling the route as "slight" prevents confusion.
    //
    // Visual-alt collection walks the unfiltered explorer to catch
    // access-blocked-but-visible alts (e.g. footways with bike=no when riding
    // on a cycleway), mirroring the pattern used by hasConfusableFootwayAlternative.

    // Reframer thresholds (radians where noted). Picked conservatively; the
    // first-cut intent is "don't bite when the answer is uncertain."
    private static final double REFRAMER_FORWARD_CONE = Math.toRadians(75);
    // Extended cone admitting visually-similar alts (same PH bucket per
    // isConfusableFrom AND same surface major) into the back-leg zone. OSM
    // sometimes mis-draws perceptually-forward turns (~85–90°) as back-leg-zone
    // angles; when the alt is the same type/surface as the route, the rider
    // perceives it as a real forward branch despite the geometric angle.
    // Visually-distinct alts past REFRAMER_FORWARD_CONE remain excluded.
    //
    // Note: this 115° limit and Shape 6's SHAPE6_ANCHOR_NO_CAP_THRESHOLD (130°)
    // are about different perceptual questions and intentionally do not match:
    //   - This cone defines "what enters altDeltas at all" — i.e. which alts
    //     EVERY shape sees as potential forward branches. Past 115° an alt is
    //     a clear back-leg from a generic-forward-perception standpoint, and
    //     Shape 1 no_competition / Shape 4 sandwich / Shape 3 / etc. should
    //     NOT treat such alts as competitors.
    //   - SHAPE6_ANCHOR_NO_CAP_THRESHOLD defines the upper end of the Shape 6
    //     anchor-cap function (where label-separation benefit fully decays).
    //     Shape 6 anchored fires only when a same-side more-extreme alt exists
    //     in altDeltas; the cap function uses the alt's magnitude as input.
    // Aligning these two for "consistency" would change other shapes' behavior
    // (e.g. Shape 1 no_competition would stop demoting at junctions with a
    // visually-similar 115°–130° alt, because altDeltas would no longer be
    // empty). Different shapes ask different perceptual questions; "matching
    // parameters across shapes for code symmetry" is a code-aesthetics goal,
    // not a business-logic one, and should not drive changes here. The
    // architectural cost is that Shape 6's anchor-cap region [115°, 130°] is
    // not reachable through altDeltas — accepted in exchange for keeping each
    // shape's cone matched to its own perceptual question. Today's field-
    // reported case at (61.46739, 23.72984) reaches the demote anyway via the
    // unanchored branch (with the PH-change guard now removed), so the
    // rider-facing outcome is correct.
    private static final double REFRAMER_VISUAL_SIMILAR_CONE = Math.toRadians(115);
    private static final double REFRAMER_REAL_TURN_CUTOFF = Math.toRadians(40);
    private static final double REFRAMER_ANCHORED_THRESHOLD = Math.toRadians(10);
    private static final double REFRAMER_ROUTE_NOISE_MARGIN = Math.toRadians(8);
    // Shape 4 near-straight clamp — the maximum |routeDelta| Shape 4 treats as
    // "straight enough" to demote a sandwich to CONTINUE. Default is the floor
    // (12°). When BOTH sandwich arms are clearly off-axis the clamp linearly
    // relaxes up to a ceiling: the rider's perceived "central zone" widens with
    // arm spread, but only modestly (Appendix E "preserve rule chain when in
    // doubt" bias). See shape4RouteClampFor() — narrowest_arm 60°→12°, 90°→18°,
    // linear in between, flat outside.
    private static final double REFRAMER_SHAPE4_ROUTE_CLAMP_FLOOR = Math.toRadians(12);
    private static final double REFRAMER_SHAPE4_ROUTE_CLAMP_CEILING = Math.toRadians(18);
    private static final double REFRAMER_SHAPE4_RELAX_GATE = Math.toRadians(60);
    private static final double REFRAMER_SHAPE4_RELAX_FULL = Math.toRadians(90);
    // Shape 4 past-slight branch — minimum angular gap between the route and a
    // same-side alt for that alt to count as the rider's "real left/right"
    // anchor. Without a clear gap the alt feels like the same forward
    // continuation as the route, not a more-extreme reference branch.
    private static final double REFRAMER_SHAPE4_PAST_SLIGHT_REFERENCE_GAP = Math.toRadians(30);
    private static final double REFRAMER_SHAPE3A_ROUTE_CLAMP = Math.toRadians(30);
    private static final double REFRAMER_SHAPE3A_ALT_MAX_ABS = Math.toRadians(45);
    private static final double REFRAMER_SHAPE3B_ROUTE_CLAMP = Math.toRadians(25);
    private static final double REFRAMER_SAMESIDE_SPREAD = Math.toRadians(33);
    // Shape 1 side-turn demote — hard thresholds. Both must hold to demote a slight/keep
    // sign to CONTINUE. The route must be genuinely near-straight in absolute terms, AND
    // every visible alt must be clearly past the slight bucket (no plausible fork branch).
    // Avoids relying on fine angular differences that OSM data can't reliably support.
    private static final double REFRAMER_SHAPE1_ROUTE_NEAR_STRAIGHT = Math.toRadians(20);
    private static final double REFRAMER_SHAPE1_ALT_CLEARLY_OFF = Math.toRadians(60);

    // Shape 6 — Soft real-turn. Demotes TURN_LEFT/RIGHT → TURN_SLIGHT_LEFT/RIGHT
    // at non-road junctions when the visible-alt distribution makes the turn
    // read as soft despite the geometric angle being past 40°.
    //
    // SHAPE6_OPPOSITE_TOLERANCE — opposite-side alts strictly inside this absolute
    //   angle of straight (along with same-side alts less extreme than the route)
    //   form the hard forbidden zone: their presence disqualifies the demote.
    //   They act as a "straight reference" or "competing same-side branch" the
    //   rider would scan for.
    // SHAPE6_OPPOSITE_SOFT_MAX — opposite-side alts in [SHAPE6_OPPOSITE_TOLERANCE,
    //   SHAPE6_OPPOSITE_SOFT_MAX] sit in the rider's slight-bucket on the other
    //   side and signal a Y-fork shape. They don't strictly disqualify but
    //   modulate the route-angle cap linearly: at 20° the cap collapses to 40°
    //   (no fire — route must be > 40° for Shape 6), at 35° the cap is the
    //   full unanchored 55°. Beyond 35° the opposite-side alt is far enough to
    //   register as a clear side-branch and stops influencing the cap.
    // SHAPE6_ROUTE_CAP_UNANCHORED — route-angle cap for Branch 6a (no same-side
    //   anchor). Up to ~55° feels slight to the rider when no alt argues otherwise.
    // SHAPE6_ROUTE_CAP_ANCHORED_MAX — route-angle cap for Branch 6b (same-side
    //   anchor close enough to be perceptually confusable with the route).
    // SHAPE6_ANCHOR_FULL_CAP_THRESHOLD / SHAPE6_ANCHOR_NO_CAP_THRESHOLD — anchor
    //   magnitudes bounding the linear taper from ANCHORED_MAX down to
    //   UNANCHORED. Anchors past NO_CAP_THRESHOLD are perceptually "tight" and
    //   provide no label-separation benefit (rider would not confuse them with
    //   a route at 55–70°), so the cap collapses to the unanchored value.
    private static final double SHAPE6_OPPOSITE_TOLERANCE = Math.toRadians(20);
    private static final double SHAPE6_OPPOSITE_SOFT_MAX = Math.toRadians(35);
    private static final double SHAPE6_ROUTE_CAP_UNANCHORED = Math.toRadians(55);
    private static final double SHAPE6_ROUTE_CAP_ANCHORED_MAX = Math.toRadians(70);
    private static final double SHAPE6_ANCHOR_FULL_CAP_THRESHOLD = Math.toRadians(100);
    private static final double SHAPE6_ANCHOR_NO_CAP_THRESHOLD = Math.toRadians(130);

    /**
     * Whether the reframer should act on this sign. Acts on CONTINUE_ON_STREET,
     * TURN_SLIGHT_LEFT/RIGHT, KEEP_LEFT/RIGHT, and TURN_LEFT/RIGHT (Shape 6
     * demote candidates). Passes through all other signs (sharp turns,
     * ferries, u-turns, IGNORE, etc.).
     */
    private static boolean isReframerCandidate(int sign) {
        return sign == Instruction.CONTINUE_ON_STREET
                || sign == Instruction.TURN_SLIGHT_LEFT
                || sign == Instruction.TURN_SLIGHT_RIGHT
                || sign == Instruction.KEEP_LEFT
                || sign == Instruction.KEEP_RIGHT
                || sign == Instruction.TURN_LEFT
                || sign == Instruction.TURN_RIGHT;
    }

    /**
     * Reframe a rule-chain sign against the junction's visible-alt distribution.
     * Returns the new sign (possibly unchanged). Side-effect: sets
     * {@link #lastReframerShape} to a debug label when a shape rewrites the sign.
     */
    private int reframeSign(int decidedSign, EdgeIteratorState routeEdge, int baseNode) {
        lastReframerShape = null;

        // Route's signed delta from incoming direction.
        GHPoint routePoint = InstructionsHelper.getPointForOrientationCalculation(routeEdge, nodeAccess);
        double routeDelta = InstructionsHelper.calculateOrientationDelta(
                prevLat, prevLon, routePoint.getLat(), routePoint.getLon(), prevOrientation);

        // Type-change detection. F1 emits angular signs intentionally when the route
        // crosses a PredictedHighway or major-surface (asphalt/non-asphalt) boundary;
        // those intentional cues should not be upgraded to KEEP by Shape 3.
        boolean typeChange = isTypeChangeAtJunction(routeEdge);

        // Collect visible alts at the junction (unfiltered explorer + visibility filters).
        // Done before Shape 5 because the past-slight Shape 4 branch needs the alt set
        // to detect the sandwich + same-side reference shape on past-slight routes.
        PredictedSurface routeSurface = predictedSurfaceEnc != null
                ? routeEdge.get(predictedSurfaceEnc) : null;
        List<Double> altDeltas = collectVisualAltDeltas(baseNode, routeEdge, routeSurface);

        // Classify alts relative to the route's angular position. Three flags:
        //   leftOfRoute / rightOfRoute   — any alt on that side (sandwich detection)
        //   sameSideMoreExtreme          — an alt on the route's side AND further than
        //                                  the route by at least the reference gap
        //                                  (rider's "real left/right" anchor for the
        //                                  past-slight Shape 4 branch).
        boolean leftOfRoute = false;
        boolean rightOfRoute = false;
        boolean sameSideMoreExtreme = false;
        for (double altDelta : altDeltas) {
            if (altDelta > routeDelta + REFRAMER_ROUTE_NOISE_MARGIN) leftOfRoute = true;
            else if (altDelta < routeDelta - REFRAMER_ROUTE_NOISE_MARGIN) rightOfRoute = true;
            if (routeDelta > 0
                    && altDelta > routeDelta + REFRAMER_SHAPE4_PAST_SLIGHT_REFERENCE_GAP) {
                sameSideMoreExtreme = true;
            } else if (routeDelta < 0
                    && altDelta < routeDelta - REFRAMER_SHAPE4_PAST_SLIGHT_REFERENCE_GAP) {
                sameSideMoreExtreme = true;
            }
        }

        // Shape 4 (sandwich) past-slight branch — the route is past the slight bucket
        // but sits in the middle of a fan with at least one same-side alt that is more
        // extreme. The rule chain emitted KEEP_LEFT/RIGHT (fork-handler interpretation
        // of a 2-arm Y) but the rider sees a 3-arm fan with the route as the middle;
        // "stay left/right" then points the rider at the wrong (more-extreme) branch.
        // Demote magnitude one bucket to TURN_SLIGHT_<route's direction>. Must run
        // before Shape 5's past-slight short-circuit. Limited to non-road junctions
        // (same scope as the existing near-straight branch).
        if (Math.abs(routeDelta) > REFRAMER_REAL_TURN_CUTOFF
                && (decidedSign == Instruction.KEEP_LEFT
                    || decidedSign == Instruction.KEEP_RIGHT)
                && leftOfRoute && rightOfRoute && sameSideMoreExtreme
                && bothNonRoadAtJunction(routeEdge)) {
            lastReframerShape = "shape_4_sandwich_past_slight";
            return routeDelta > 0
                    ? Instruction.TURN_SLIGHT_LEFT
                    : Instruction.TURN_SLIGHT_RIGHT;
        }

        // Shape 6 — Soft real turn. Demote TURN_LEFT/RIGHT → TURN_SLIGHT_LEFT/RIGHT
        // when the junction's visible-alt distribution makes the turn read as soft
        // despite the geometric angle being past 40°. Two branches share an
        // asymmetric safety gate:
        //   (i)  Hard forbidden zone — no alt strictly between the route angle and
        //        20° on the opposite side (same-side less-extreme alts would compete
        //        with the route, opposite-side near-straight alts would anchor
        //        "straight" against which the route's deviation reads as a real turn).
        //   (ii) Soft opposite-side zone — opposite-side alts in [20°, 35°] sit in
        //        the rider's perceptual slight-bucket on the other side and signal
        //        a Y-fork shape. They modulate the route-angle cap linearly: at 20°
        //        the cap collapses to 40° (no fire); at 35° the cap is the full
        //        55° (or anchored max). Beyond 35° the alt is a clear side-branch
        //        and doesn't influence the cap.
        //   6a (unanchored) — route up to 55° (subject to the soft zone modifier),
        //      no same-side anchor.
        //   6b (anchored)   — route up to a dynamic cap (70°→55° as the closest
        //      same-side anchor's magnitude grows from 100° to 130°, subject to
        //      the soft zone modifier), at least one same-side alt more extreme
        //      than the route by ≥30°. The anchor is the rider's "real left/right"
        //      reference; relabelling avoids confusion with the more-extreme branch.
        // Scope: non-road junctions only. PH change at the junction is NOT a
        // guard: anchored fires because the disambiguation work is independent
        // of the type change; unanchored fires because the alt set can lose a
        // real same-side anchor when its OSM angle exceeds the visual-similar
        // cone, and the demote then provides the disambiguation the rider
        // actually needs. Surface change is also NOT a guard — perceived
        // softness is independent of surface continuity.
        if ((decidedSign == Instruction.TURN_LEFT || decidedSign == Instruction.TURN_RIGHT)
                && Math.abs(routeDelta) > REFRAMER_REAL_TURN_CUTOFF
                && bothNonRoadAtJunction(routeEdge)) {

            boolean forbiddenAltExists = false;
            double closestAnchorAbs = Double.POSITIVE_INFINITY;
            // Smallest |altDelta| of any opposite-side alt in the soft zone
            // [SHAPE6_OPPOSITE_TOLERANCE, SHAPE6_OPPOSITE_SOFT_MAX]. The smallest
            // (closest to the forbidden boundary) wins because it shrinks the cap
            // the most — that alt is the rider's nearest "slight branch on the
            // other side" anchoring the Y-fork perception.
            double closestOppositeSoftAbs = Double.POSITIVE_INFINITY;
            for (double altDelta : altDeltas) {
                if (routeDelta > 0) {
                    // Route on left (positive delta). Forbidden zone is (-20°, routeDelta):
                    // any alt to the right of route by less than the full route magnitude
                    // OR within 20° to the right of straight competes/anchors.
                    if (altDelta > -SHAPE6_OPPOSITE_TOLERANCE && altDelta < routeDelta) {
                        forbiddenAltExists = true;
                    }
                    // Same-side anchor: further left than route by ≥ 30°.
                    if (altDelta >= routeDelta + REFRAMER_SHAPE4_PAST_SLIGHT_REFERENCE_GAP) {
                        double absAlt = Math.abs(altDelta);
                        if (absAlt < closestAnchorAbs) closestAnchorAbs = absAlt;
                    }
                    // Opposite-side soft zone: altDelta in [-35°, -20°].
                    if (altDelta >= -SHAPE6_OPPOSITE_SOFT_MAX
                            && altDelta <= -SHAPE6_OPPOSITE_TOLERANCE) {
                        double absAlt = Math.abs(altDelta);
                        if (absAlt < closestOppositeSoftAbs) closestOppositeSoftAbs = absAlt;
                    }
                } else {
                    // Route on right (negative delta). Forbidden zone is (routeDelta, +20°).
                    if (altDelta < SHAPE6_OPPOSITE_TOLERANCE && altDelta > routeDelta) {
                        forbiddenAltExists = true;
                    }
                    // Same-side anchor: further right than route by ≥ 30°.
                    if (altDelta <= routeDelta - REFRAMER_SHAPE4_PAST_SLIGHT_REFERENCE_GAP) {
                        double absAlt = Math.abs(altDelta);
                        if (absAlt < closestAnchorAbs) closestAnchorAbs = absAlt;
                    }
                    // Opposite-side soft zone: altDelta in [+20°, +35°].
                    if (altDelta >= SHAPE6_OPPOSITE_TOLERANCE
                            && altDelta <= SHAPE6_OPPOSITE_SOFT_MAX) {
                        double absAlt = Math.abs(altDelta);
                        if (absAlt < closestOppositeSoftAbs) closestOppositeSoftAbs = absAlt;
                    }
                }
            }

            if (!forbiddenAltExists) {
                double routeCap;
                String shapeLabel;
                if (closestAnchorAbs == Double.POSITIVE_INFINITY) {
                    routeCap = SHAPE6_ROUTE_CAP_UNANCHORED;
                    shapeLabel = "shape_6_unanchored";
                } else {
                    routeCap = shape6AnchoredCap(closestAnchorAbs);
                    shapeLabel = "shape_6_anchored";
                }
                if (closestOppositeSoftAbs != Double.POSITIVE_INFINITY) {
                    routeCap = Math.min(routeCap, shape6OppositeSoftCap(closestOppositeSoftAbs));
                }
                if (Math.abs(routeDelta) <= routeCap) {
                    lastReframerShape = shapeLabel;
                    return routeDelta > 0
                            ? Instruction.TURN_SLIGHT_LEFT
                            : Instruction.TURN_SLIGHT_RIGHT;
                }
            }
        }

        // Shape 5 — route is past the slight bucket. The angular event is real;
        // never reframe.
        if (Math.abs(routeDelta) > REFRAMER_REAL_TURN_CUTOFF) {
            return decidedSign;
        }

        // Shape 2 — a clear straight reference exists; the rule chain's sign is right.
        for (double altDelta : altDeltas) {
            if (Math.abs(altDelta) <= REFRAMER_ANCHORED_THRESHOLD) {
                return decidedSign;
            }
        }

        // Shape 1 — no forward competitor visible. Demote slight to CONTINUE.
        if (altDeltas.isEmpty()) {
            if (decidedSign == Instruction.TURN_SLIGHT_LEFT
                    || decidedSign == Instruction.TURN_SLIGHT_RIGHT
                    || decidedSign == Instruction.KEEP_LEFT
                    || decidedSign == Instruction.KEEP_RIGHT) {
                lastReframerShape = "shape_1_no_competition";
                return Instruction.CONTINUE_ON_STREET;
            }
            return decidedSign;
        }

        // Shape 4 (sandwich) near-straight branch — alts on both sides of the route.
        // No KEEP direction works. Demote near-straight routes to CONTINUE; pass
        // through if the route is off-axis enough that the rider physically feels
        // the bend. The "near-straight" clamp scales with the narrowest sandwich
        // arm (see shape4RouteClampFor): a wider fan widens the rider's perceived
        // central zone, so the clamp linearly relaxes from 12° (default) up to 18°
        // when both arms are clearly off. Limited to non-road junctions (see
        // bothNonRoadAtJunction).
        if (leftOfRoute && rightOfRoute) {
            double routeClamp = shape4RouteClampFor(routeDelta, altDeltas);
            if (Math.abs(routeDelta) <= routeClamp
                    && bothNonRoadAtJunction(routeEdge)
                    && (decidedSign == Instruction.TURN_SLIGHT_LEFT
                        || decidedSign == Instruction.TURN_SLIGHT_RIGHT
                        || decidedSign == Instruction.KEEP_LEFT
                        || decidedSign == Instruction.KEEP_RIGHT)) {
                lastReframerShape = "shape_4_sandwich";
                return Instruction.CONTINUE_ON_STREET;
            }
            return decidedSign;
        }

        // Detect a "straddling fork branch" — an alt on the opposite side of incoming
        // from the route that the rider would perceive as a Y-fork partner.
        // Single gate: alt must be in the forward fan (|alt| ≤ 45°). Past that the alt
        // is clearly a turn, not a forward branch.
        // Route at 0° is treated as straddling against any non-zero alt.
        // (No magnitude-gap check — that would be a fine-grained angular comparison
        // that OSM data isn't reliable enough to support. Junction-shape signals only.)
        boolean hasStraddlingForkBranch = false;
        for (double altDelta : altDeltas) {
            boolean isStraddling = (routeDelta == 0.0) || (altDelta * routeDelta < 0);
            if (isStraddling && Math.abs(altDelta) <= REFRAMER_SHAPE3A_ALT_MAX_ABS) {
                hasStraddlingForkBranch = true;
                break;
            }
        }

        // Shape 3a — Y-fork, straddle of incoming with a real fork-branch alt.
        // KEEP toward the route's side (the empty side, away from where the alts are).
        //
        // Upgrades CONTINUE_ON_STREET to KEEP unconditionally. For TURN_SLIGHT_X,
        // upgrades only when there is no PH/surface change at this junction —
        // at type transitions, F1's angular cue conveys "you're changing road
        // type" and must not be replaced by a fork-relative KEEP that drops that
        // signal. KEEP inputs pass through (rule chain's KEEP direction is trusted).
        if (hasStraddlingForkBranch) {
            if (Math.abs(routeDelta) <= REFRAMER_SHAPE3A_ROUTE_CLAMP
                    && shape3InputCanUpgrade(decidedSign, typeChange)) {
                lastReframerShape = "shape_3a_straddle";
                return leftOfRoute ? Instruction.KEEP_RIGHT : Instruction.KEEP_LEFT;
            }
            return decidedSign;
        }

        // Same-side as incoming (or opposite-side alts that failed the fork-branch
        // bound — in either case we use spread from route as the fork-vs-side-turn metric).
        double maxSpread = 0;
        for (double altDelta : altDeltas) {
            maxSpread = Math.max(maxSpread, Math.abs(altDelta - routeDelta));
        }

        // Shape 3b — same-side Y-fork (small spread, route in slight zone).
        // Same upgrade restriction as Shape 3a.
        if (maxSpread <= REFRAMER_SAMESIDE_SPREAD
                && Math.abs(routeDelta) <= REFRAMER_SHAPE3B_ROUTE_CLAMP
                && shape3InputCanUpgrade(decidedSign, typeChange)) {
            lastReframerShape = "shape_3b_sameside_fork";
            return leftOfRoute ? Instruction.KEEP_RIGHT : Instruction.KEEP_LEFT;
        }

        // Shape 1 (side-turn variant) — demote slight/keep to CONTINUE only when the
        // junction shape unambiguously says "route is the straight one, alts are turns."
        // Two hard conditions:
        //   (a) Route is genuinely near-straight in absolute terms (|route| ≤ 20°).
        //   (b) ALL visible alts are clearly past the slight bucket (|alt| ≥ 60° each).
        // Both must hold. This avoids relying on fine angular differences that OSM
        // data can't support — only fires when the asymmetry is overwhelming (Case A:
        // route -18°, alt -66°). Anything less clear preserves the rule chain's sign.
        // Limited to non-road junctions: at road junctions the rule chain's angular
        // signs are intentional (F1 type-transition cues, leaving-current-street at
        // RC changes) and must not be demoted.
        boolean routeIsNearStraight = Math.abs(routeDelta) <= REFRAMER_SHAPE1_ROUTE_NEAR_STRAIGHT;
        boolean allAltsClearlyOff = true;
        for (double altDelta : altDeltas) {
            if (Math.abs(altDelta) < REFRAMER_SHAPE1_ALT_CLEARLY_OFF) {
                allAltsClearlyOff = false;
                break;
            }
        }
        if (routeIsNearStraight && allAltsClearlyOff
                && bothNonRoadAtJunction(routeEdge)
                && (decidedSign == Instruction.TURN_SLIGHT_LEFT
                    || decidedSign == Instruction.TURN_SLIGHT_RIGHT
                    || decidedSign == Instruction.KEEP_LEFT
                    || decidedSign == Instruction.KEEP_RIGHT)) {
            lastReframerShape = "shape_1_side_turn";
            return Instruction.CONTINUE_ON_STREET;
        }
        return decidedSign;
    }

    /**
     * Whether a Shape-3 upgrade (→ KEEP) is allowed for this input sign.
     * CONTINUE upgrades unconditionally. TURN_SLIGHT_X upgrades only when there
     * is no type change at the junction (otherwise we preserve F1's angular cue).
     * KEEP inputs never upgrade (rule chain's KEEP direction is trusted).
     */
    private static boolean shape3InputCanUpgrade(int sign, boolean typeChange) {
        if (sign == Instruction.CONTINUE_ON_STREET) return true;
        if (sign == Instruction.TURN_SLIGHT_LEFT || sign == Instruction.TURN_SLIGHT_RIGHT) {
            return !typeChange;
        }
        return false;
    }

    /**
     * Compute Shape 4's near-straight route clamp for this junction. Linearly
     * relaxes from {@link #REFRAMER_SHAPE4_ROUTE_CLAMP_FLOOR} (12°) up to
     * {@link #REFRAMER_SHAPE4_ROUTE_CLAMP_CEILING} (18°) as the narrowest sandwich
     * arm grows from {@link #REFRAMER_SHAPE4_RELAX_GATE} (60°) to
     * {@link #REFRAMER_SHAPE4_RELAX_FULL} (90°). Below the gate the floor applies
     * (no relaxation); above the full point the ceiling applies (no further
     * relaxation). The narrowest arm is the smaller of the closest left-of-route
     * alt's |altDelta| and the closest right-of-route alt's |altDelta|; if either
     * side has no alt the floor applies.
     */
    private static double shape4RouteClampFor(double routeDelta, List<Double> altDeltas) {
        double closestLeftAbs = Double.POSITIVE_INFINITY;
        double closestRightAbs = Double.POSITIVE_INFINITY;
        for (double altDelta : altDeltas) {
            double abs = Math.abs(altDelta);
            if (altDelta > routeDelta + REFRAMER_ROUTE_NOISE_MARGIN) {
                if (abs < closestLeftAbs) closestLeftAbs = abs;
            } else if (altDelta < routeDelta - REFRAMER_ROUTE_NOISE_MARGIN) {
                if (abs < closestRightAbs) closestRightAbs = abs;
            }
        }
        if (closestLeftAbs == Double.POSITIVE_INFINITY
                || closestRightAbs == Double.POSITIVE_INFINITY) {
            return REFRAMER_SHAPE4_ROUTE_CLAMP_FLOOR;
        }
        double narrowestArm = Math.min(closestLeftAbs, closestRightAbs);
        if (narrowestArm <= REFRAMER_SHAPE4_RELAX_GATE) return REFRAMER_SHAPE4_ROUTE_CLAMP_FLOOR;
        if (narrowestArm >= REFRAMER_SHAPE4_RELAX_FULL) return REFRAMER_SHAPE4_ROUTE_CLAMP_CEILING;
        double t = (narrowestArm - REFRAMER_SHAPE4_RELAX_GATE)
                / (REFRAMER_SHAPE4_RELAX_FULL - REFRAMER_SHAPE4_RELAX_GATE);
        return REFRAMER_SHAPE4_ROUTE_CLAMP_FLOOR
                + t * (REFRAMER_SHAPE4_ROUTE_CLAMP_CEILING - REFRAMER_SHAPE4_ROUTE_CLAMP_FLOOR);
    }

    /**
     * Compute Shape 6's route-angle cap given the closest same-side anchor's
     * absolute magnitude. Linearly tapers from {@link #SHAPE6_ROUTE_CAP_ANCHORED_MAX}
     * (70°) down to {@link #SHAPE6_ROUTE_CAP_UNANCHORED} (55°) as the anchor grows
     * from {@link #SHAPE6_ANCHOR_FULL_CAP_THRESHOLD} (100°) to
     * {@link #SHAPE6_ANCHOR_NO_CAP_THRESHOLD} (130°). Below the full-cap threshold
     * the maximum applies; above the no-cap threshold the cap collapses to the
     * unanchored value (anchor is perceptually "tight" and no label-separation
     * benefit remains).
     */
    private static double shape6AnchoredCap(double anchorAbs) {
        if (anchorAbs <= SHAPE6_ANCHOR_FULL_CAP_THRESHOLD) {
            return SHAPE6_ROUTE_CAP_ANCHORED_MAX;
        }
        if (anchorAbs >= SHAPE6_ANCHOR_NO_CAP_THRESHOLD) {
            return SHAPE6_ROUTE_CAP_UNANCHORED;
        }
        double t = (anchorAbs - SHAPE6_ANCHOR_FULL_CAP_THRESHOLD)
                / (SHAPE6_ANCHOR_NO_CAP_THRESHOLD - SHAPE6_ANCHOR_FULL_CAP_THRESHOLD);
        return SHAPE6_ROUTE_CAP_ANCHORED_MAX
                + t * (SHAPE6_ROUTE_CAP_UNANCHORED - SHAPE6_ROUTE_CAP_ANCHORED_MAX);
    }

    /**
     * Compute Shape 6's opposite-side soft-zone cap given the closest opposite-side
     * alt magnitude in [{@link #SHAPE6_OPPOSITE_TOLERANCE} (20°),
     * {@link #SHAPE6_OPPOSITE_SOFT_MAX} (35°)]. Linearly ramps from
     * {@link #REFRAMER_REAL_TURN_CUTOFF} (40°, the lower bound of sign ±2 — so the
     * cap effectively forbids firing) at 20° up to {@link #SHAPE6_ROUTE_CAP_UNANCHORED}
     * (55°) at 35°. The closer the opposite-side alt is to the 20° boundary, the
     * stronger the Y-fork perception and the tighter the cap.
     */
    private static double shape6OppositeSoftCap(double oppositeAbs) {
        if (oppositeAbs <= SHAPE6_OPPOSITE_TOLERANCE) {
            return REFRAMER_REAL_TURN_CUTOFF;
        }
        if (oppositeAbs >= SHAPE6_OPPOSITE_SOFT_MAX) {
            return SHAPE6_ROUTE_CAP_UNANCHORED;
        }
        double t = (oppositeAbs - SHAPE6_OPPOSITE_TOLERANCE)
                / (SHAPE6_OPPOSITE_SOFT_MAX - SHAPE6_OPPOSITE_TOLERANCE);
        return REFRAMER_REAL_TURN_CUTOFF
                + t * (SHAPE6_ROUTE_CAP_UNANCHORED - REFRAMER_REAL_TURN_CUTOFF);
    }

    /**
     * Whether neither side of the junction is "hard" road infrastructure (motorway /
     * major road / minor road). Used to gate every reframer demote shape (Shape 1
     * side-turn, Shape 4 sandwich + past-slight, Shape 6 soft real turn) — the
     * reframer's demote logic targets junctions where geometric ambiguity is the
     * dominant signal and the rider scans junction shape, not absolute angle.
     * <p>
     * SERVICE_ROAD is treated as non-road here (see {@link #isReframerHardRoad}):
     * it is physically in-between and visually indistinguishable from a track on the
     * ground, so the reframer applies. At any junction involving a hard road
     * (motorway/major/minor) the rule chain's angular signs are intentional (F1
     * type-transition cues, leaving-current-street at road-class changes) and are
     * preserved. Type/class/name remain in extraInfo regardless, so a demote never
     * loses the rider-facing road information — only the ambiguous direction label.
     */
    private boolean bothNonRoadAtJunction(EdgeIteratorState routeEdge) {
        if (predictedHighwayEnc == null || prevEdge == null) return true;
        PredictedHighway prevPH = prevEdge.get(predictedHighwayEnc);
        PredictedHighway currentPH = routeEdge.get(predictedHighwayEnc);
        if (prevPH == null || currentPH == null) return true;
        return !isReframerHardRoad(prevPH) && !isReframerHardRoad(currentPH);
    }

    /**
     * Whether this junction crosses a PredictedHighway or major-surface
     * (asphalt vs non-asphalt) boundary between {@code prevEdge} and {@code routeEdge}.
     * Used by Shape 3 to preserve F1's intentional angular cue at type transitions.
     */
    private boolean isTypeChangeAtJunction(EdgeIteratorState routeEdge) {
        if (prevEdge == null) return false;
        if (predictedHighwayEnc != null) {
            PredictedHighway prevPH = prevEdge.get(predictedHighwayEnc);
            PredictedHighway currentPH = routeEdge.get(predictedHighwayEnc);
            if (prevPH != null && currentPH != null && prevPH != currentPH) return true;
        }
        if (predictedSurfaceEnc != null) {
            PredictedSurface prevS = prevEdge.get(predictedSurfaceEnc);
            PredictedSurface currentS = routeEdge.get(predictedSurfaceEnc);
            if (prevS != null && currentS != null
                    && prevS != PredictedSurface.ASPHALT_OR_UNPAVED
                    && currentS != PredictedSurface.ASPHALT_OR_UNPAVED
                    && isAsphalt(prevS) != isAsphalt(currentS)) return true;
        }
        return false;
    }

    /**
     * Collect visible alts at {@code baseNode} (the junction node). Walks the
     * unfiltered explorer to catch access-blocked-but-visible alts, then applies
     * surface visibility and forward-cone gates. Visually-similar alts (same PH
     * bucket per {@link #isConfusableFrom} AND same surface major) get an extended
     * cone admission into the back-leg zone — the rider perceives them as real
     * forward branches even when OSM mis-draws the angle as sharper than reality.
     */
    private List<Double> collectVisualAltDeltas(int baseNode, EdgeIteratorState routeEdge,
                                                 PredictedSurface routeSurface) {
        List<Double> deltas = new ArrayList<>();
        PredictedHighway routePH = predictedHighwayEnc != null
                ? routeEdge.get(predictedHighwayEnc) : null;
        PredictedSurface prevSurface = (prevEdge != null && predictedSurfaceEnc != null)
                ? prevEdge.get(predictedSurfaceEnc) : null;
        EdgeIterator iter = allExplorer.setBaseNode(baseNode);
        while (iter.next()) {
            if (iter.getEdge() == routeEdge.getEdge()) continue;
            if (prevEdge != null && iter.getEdge() == prevEdge.getEdge()) continue;

            // Surface visibility: an alt counts as visually similar if it matches
            // either the rider's prev edge surface OR the route's surface — the
            // rider's frame of reference at the junction spans both. Drop only when
            // surface evidence from BOTH sides is available and BOTH sides clearly
            // reject the alt. ASPHALT_OR_UNPAVED at any of the three positions is
            // ambiguous (no signal); surfacesClearlyDiffer already returns false in
            // that case, so a side with unknown surface cannot contribute a
            // contradiction.
            if (predictedSurfaceEnc != null) {
                PredictedSurface altSurface = iter.get(predictedSurfaceEnc);
                if (surfacesClearlyDiffer(altSurface, prevSurface)
                        && surfacesClearlyDiffer(altSurface, routeSurface)) {
                    continue;
                }
            }

            GHPoint altPoint = InstructionsHelper.getPointForOrientationCalculation(iter, nodeAccess);
            double altDelta = InstructionsHelper.calculateOrientationDelta(
                    prevLat, prevLon, altPoint.getLat(), altPoint.getLon(), prevOrientation);
            double absAlt = Math.abs(altDelta);

            // Forward cone: standard 75° admission. Visually-similar alts (same PH
            // bucket + same surface major — surface already gated above) get an
            // extended cone up to 115°: OSM imprecision can mis-draw genuinely
            // forward-perceptible turns (~85–90°) as back-leg-zone angles, and
            // when the alt is the same type/surface as the route the rider
            // perceives it as a real forward branch despite the geometric angle.
            if (absAlt > REFRAMER_FORWARD_CONE) {
                if (absAlt > REFRAMER_VISUAL_SIMILAR_CONE) continue;
                if (predictedHighwayEnc == null || routePH == null) continue;
                PredictedHighway altPH = iter.get(predictedHighwayEnc);
                if (altPH == null || !isConfusableFrom(routePH, altPH)) continue;
            }

            deltas.add(altDelta);
        }
        return deltas;
    }

    // ========================================================================
    // Extra info enrichment
    // ========================================================================

    /**
     * Populate Trailmap-specific extraInfo on an instruction at creation time.
     * Provides rich metadata for Stage 2 (post-processing) and Stage 3 (client).
     */
    private void enrichExtraInfo(Instruction instruction, EdgeIteratorState edge, EdgeIteratorState prevEdge) {
        // Road classification
        RoadClass roadClass = edge.get(roadClassEnc);
        instruction.setExtraInfo("road_class", roadClass.name());
        if (prevEdge != null) {
            RoadClass prevRoadClass = prevEdge.get(roadClassEnc);
            instruction.setExtraInfo("prev_road_class", prevRoadClass.name());
            instruction.setExtraInfo("road_class_changed", roadClass != prevRoadClass);
        }

        // PredictedHighway
        if (predictedHighwayEnc != null) {
            PredictedHighway ph = edge.get(predictedHighwayEnc);
            instruction.setExtraInfo("predicted_highway", ph.name());
            if (prevEdge != null) {
                PredictedHighway prevPh = prevEdge.get(predictedHighwayEnc);
                instruction.setExtraInfo("prev_predicted_highway", prevPh.name());
            }
        }

        // Surface
        if (surfaceEnc != null) {
            Surface surface = edge.get(surfaceEnc);
            if (surface != Surface.MISSING) {
                instruction.setExtraInfo("surface", surface.name());
            }
        }
        if (predictedSurfaceEnc != null) {
            PredictedSurface ps = edge.get(predictedSurfaceEnc);
            instruction.setExtraInfo("predicted_surface", ps.name());
        }

        // Name presence
        String instrName = instruction.getName();
        instruction.setExtraInfo("has_name", instrName != null && !instrName.isEmpty());

        // Bike network
        if (bikeNetworkEnc != null) {
            RouteNetwork bn = edge.get(bikeNetworkEnc);
            if (bn != RouteNetwork.MISSING) {
                instruction.setExtraInfo("bike_network", bn.name());
            }
        }

        // Turn angle in degrees
        if (prevEdge != null) {
            GHPoint point = InstructionsHelper.getPointForOrientationCalculation(edge, nodeAccess);
            double orientation = AngleCalc.ANGLE_CALC.calcOrientation(prevLat, prevLon, point.getLat(), point.getLon());
            double prevOr = AngleCalc.ANGLE_CALC.calcOrientation(doublePrevLat, doublePrevLon, prevLat, prevLon);
            orientation = AngleCalc.ANGLE_CALC.alignOrientation(prevOr, orientation);
            double deltaRad = orientation - prevOr;
            instruction.setExtraInfo("turn_angle_deg", Helper.round(Math.toDegrees(deltaRad), 1));
        }

        // Junction context (from last getTurn call)
        if (lastOutgoingEdges != null) {
            List<EdgeIteratorState> alts = lastOutgoingEdges.getAllowedAlternativeTurns();
            instruction.setExtraInfo("junction_alternatives", alts.size());

            if (!alts.isEmpty() && predictedHighwayEnc != null) {
                PredictedHighway currentPH = edge.get(predictedHighwayEnc);
                StringBuilder altPHs = new StringBuilder();
                boolean hasHigherRoad = false;
                for (int i = 0; i < alts.size(); i++) {
                    PredictedHighway altPH = alts.get(i).get(predictedHighwayEnc);
                    if (i > 0) altPHs.append(",");
                    altPHs.append(altPH.name());
                    if (phProminence(altPH) > phProminence(currentPH)) {
                        hasHigherRoad = true;
                    }
                }
                instruction.setExtraInfo("junction_alt_predicted_highways", altPHs.toString());
                instruction.setExtraInfo("junction_has_higher_road", hasHigherRoad);
            }

            // trail_fork: junction-level marker for a confusable trail/cycleway fork.
            // Stamped on any emitted instruction whose junction matches the criteria,
            // independent of which rule in getTurn() returned the sign. Used by Stage 2
            // Pass 2 C1 (preserve from continuity suppression) and the client app
            // (treat as meaningful fork, not a generic CONTINUE that may be suppressed).
            if (predictedHighwayEnc != null) {
                PredictedHighway phForFork = edge.get(predictedHighwayEnc);
                String nameForFork = instruction.getName();
                if (phForFork != null
                        && !isRoadInfrastructure(phForFork)
                        && (nameForFork == null || nameForFork.isEmpty() || phForFork == PredictedHighway.CYCLEWAY)
                        && hasConfusableAlternative(lastOutgoingEdges, edge, phForFork)) {
                    instruction.setExtraInfo("trail_fork", true);
                }
            }

            // T-junction detection: does the source road continue through this junction?
            // True when an alternative has the same PredictedHighway as the previous edge
            // AND goes roughly straight (|sign| <= 1). Used by Stage 2 M1 join-side-path.
            if (prevEdge != null && predictedHighwayEnc != null) {
                PredictedHighway prevPH = prevEdge.get(predictedHighwayEnc);
                boolean sourceRoadContinues = false;
                for (EdgeIteratorState alt : alts) {
                    if (alt.get(predictedHighwayEnc) == prevPH) {
                        GHPoint altPoint = InstructionsHelper.getPointForOrientationCalculation(alt, nodeAccess);
                        int altSign = InstructionsHelper.calculateSign(prevLat, prevLon,
                                altPoint.getLat(), altPoint.getLon(), prevOrientation);
                        if (Math.abs(altSign) <= 1) {
                            sourceRoadContinues = true;
                            break;
                        }
                    }
                }
                instruction.setExtraInfo("source_road_continues", sourceRoadContinues);
            }

            // junction_has_straight_alt: does any alternative continue roughly straight?
            // Pure geometric check, independent of PH/RC. Used by Stage 2 M1 as a
            // continuation guard — when the source way ends at a real T-junction
            // (no alt within ±30° of straight), the rider is at a forced navigation
            // turn rather than a brief sidepath detour, so M1's "join via sidepath"
            // semantics don't apply.
            if (prevEdge != null) {
                boolean hasStraightAlt = false;
                for (EdgeIteratorState alt : alts) {
                    GHPoint altPt = InstructionsHelper.getPointForOrientationCalculation(alt, nodeAccess);
                    double altDelta = InstructionsHelper.calculateOrientationDelta(
                            prevLat, prevLon, altPt.getLat(), altPt.getLon(), prevOrientation);
                    if (Math.abs(altDelta) <= Math.PI / 6) {  // ~30°
                        hasStraightAlt = true;
                        break;
                    }
                }
                instruction.setExtraInfo("junction_has_straight_alt", hasStraightAlt);
            }
        }
    }

    // ========================================================================
    // Point and instruction accumulation (unchanged from GH)
    // ========================================================================

    private void updatePointsAndInstruction(EdgeIteratorState edge, PointList pl) {
        int len = pl.size() - 1;
        for (int i = 0; i < len; i++) {
            prevInstruction.getPoints().add(pl, i);
        }
        double newDist = edge.getDistance();
        prevInstruction.setDistance(newDist + prevInstruction.getDistance());
        if (prevEdge != null)
            prevInstruction.setTime(GHUtility.calcMillisWithTurnMillis(weighting, edge, false, prevEdge.getEdge()) + prevInstruction.getTime());
        else
            prevInstruction.setTime(weighting.calcEdgeMillis(edge, false) + prevInstruction.getTime());
    }
}
