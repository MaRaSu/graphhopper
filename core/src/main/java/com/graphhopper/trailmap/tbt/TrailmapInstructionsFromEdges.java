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
    private boolean lastTrailFork;

    // --- Visual guidance side-channel ---
    // Set by suppression rules in getTurn() when a junction is visual-worthy but not
    // instruction-worthy. Read by next() after getTurn() returns IGNORE.
    // Carries the angle-based sign the instruction would have if it were a real turn.
    private int visualCandidateSign = Instruction.IGNORE;

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
            // S1: Same PH + same name + all alternatives are different PH → road bending
            if (InstructionsHelper.isSameName(name, prevName)
                    && currentPH != null && currentPH == prevPH
                    && allAlternativesDifferentPH(outgoingEdges, currentPH)) {
                return Instruction.IGNORE;
            }

            // S2: Same name + same RoadClass + no alternative of same RC → road curves
            if (InstructionsHelper.isSameName(name, prevName)
                    && currentRC == prevRC
                    && noAlternativeOfSameRC(outgoingEdges, currentRC)) {
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
        if (!InstructionsHelper.isSameName(name, prevName)
                && currentPH != null && currentPH == prevPH
                && Math.abs(delta) < 0.15
                && otherContinue == null) {
            return Instruction.IGNORE;
        }

        // S5: Slight turn on same road (same name + same class)
        // Only suppresses on named ways or road-like infrastructure — unnamed trails need fork guidance.
        // CYCLEWAY excluded from both: unpaved Finnish cycleways have informal names (route
        // names like "Pyynikin rantapolku") that don't help distinguish Y-junction branches.
        if (Math.abs(sign) == 1
                && InstructionsHelper.isSameName(name, prevName)
                && currentRC == prevRC) {
            boolean hasName = name != null && !name.isEmpty();
            boolean isCycleway = currentPH == PredictedHighway.CYCLEWAY;
            boolean isRoadLike = currentPH != null && isRoadInfrastructure(currentPH);
            if ((hasName && !isCycleway) || isRoadLike) {
                // Visual check: route bends while a visually similar alternative goes straighter.
                if (hasVisualCandidateAlternative(outgoingEdges, edge, currentPH, currentSurface, delta)) {
                    visualCandidateSign = sign;
                }
                return Instruction.IGNORE;
            }
        }

        // S3: All alternatives are lower prominence → obviously staying on main way
        if (currentPH != null && allAlternativesLowerProminence(outgoingEdges, currentPH)) {
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

                // Staying on same prominent road class (GH original, extended to all classes)
                if ((currentRC == prevRC && link == prevLink)
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
            lastTrailFork = true;
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
     * Check if all accessible alternative edges have a different PredictedHighway than current.
     * Returns false if PH data is unavailable.
     */
    private boolean allAlternativesDifferentPH(InstructionsOutgoingEdges outgoing, PredictedHighway currentPH) {
        if (predictedHighwayEnc == null || currentPH == null) return false;
        for (EdgeIteratorState alt : outgoing.getAllowedAlternativeTurns()) {
            if (alt.get(predictedHighwayEnc) == currentPH) return false;
        }
        return true;
    }

    /**
     * Check if no accessible alternative edge has the same RoadClass as current.
     */
    private boolean noAlternativeOfSameRC(InstructionsOutgoingEdges outgoing, RoadClass currentRC) {
        for (EdgeIteratorState alt : outgoing.getAllowedAlternativeTurns()) {
            if (alt.get(roadClassEnc) == currentRC) return false;
        }
        return true;
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

            // E2 trail fork flag — tells post-processing C1 to preserve this instruction
            if (lastTrailFork) {
                instruction.setExtraInfo("trail_fork", true);
                lastTrailFork = false;
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
