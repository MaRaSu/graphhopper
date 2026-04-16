package com.graphhopper.trailmap.tbt;

import com.graphhopper.util.Instruction;
import com.graphhopper.util.InstructionList;
import com.graphhopper.util.PointList;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

/**
 * Stage 2 of the Trailmap TbT instruction pipeline.
 * <p>
 * Post-processes the instruction list produced by Stage 1 (TrailmapInstructionsFromEdges).
 * Operates on the complete list — can detect multi-instruction patterns impossible
 * to see during edge-by-edge generation.
 * <p>
 * Four passes in order:
 * <ol>
 *   <li>Pattern merging — join-side-path, quick-sequence, MTB consecutive forks</li>
 *   <li>Continuity suppression — remove noise instructions</li>
 *   <li>Confirmation insertion — add "continue straight" at important points</li>
 *   <li>Annotation — assign tbt_priority, then_turn links</li>
 * </ol>
 */
public class InstructionPostProcessor {

    private static final Logger LOGGER = LoggerFactory.getLogger(InstructionPostProcessor.class);

    // Pattern merging thresholds
    private static final double SNAP_STUB_MAX_ROUND_TRIP = 40.0;  // meters (covers up to ~20m snap offset)
    private static final double JOIN_SIDE_PATH_MAX_DISTANCE = 15.0;  // meters
    private static final double QUICK_SEQUENCE_MAX_DISTANCE = 50.0;  // meters
    private static final double MTB_CONSECUTIVE_FORK_MAX_DISTANCE = 200.0;  // meters

    // Suppression thresholds
    private static final double MICRO_ARTIFACT_MAX_DISTANCE = 5.0;  // meters

    // Confirmation thresholds
    private static final double REASSURANCE_GAP = 2000.0;  // meters

    /**
     * Apply all post-processing passes to the instruction list.
     * Modifies the list in-place.
     */
    public void process(InstructionList instructions) {
        if (instructions.size() < 2) return;

        int originalSize = instructions.size();

        pass1_patternMerging(instructions);
        pass2_continuitySuppression(instructions);
        pass3_confirmationInsertion(instructions);
        pass4_annotation(instructions);

        LOGGER.debug("Post-processing complete: {} instructions (was {})", instructions.size(), originalSize);
    }

    // ========================================================================
    // Pass 1: Pattern Merging
    // ========================================================================

    /**
     * Detect and merge multi-instruction patterns:
     * M1: Join side path — TURN_X → short segment → TURN_Y (opposite) with PH change
     * M2: Quick sequence — two turns within 50m (not M1) → link with then_turn
     * M3: MTB consecutive forks — 2+ same-direction KEEP within 200m
     */
    private void pass1_patternMerging(InstructionList instructions) {
        // M0: Waypoint snap stub suppression — must run before M1/M2/M3
        suppressSnapStubs(instructions);

        // M1 and M2: sliding window of 2 instructions
        for (int i = 0; i < instructions.size() - 2; i++) {
            Instruction first = instructions.get(i);
            Instruction second = instructions.get(i + 1);

            // Skip FINISH, roundabout, and non-turn instructions
            if (isTerminal(first) || isTerminal(second)) continue;
            if (first.getSign() == Instruction.USE_ROUNDABOUT) continue;

            double gapDistance = first.getDistance();
            if (gapDistance > QUICK_SEQUENCE_MAX_DISTANCE) continue;

            // M1: Join side path
            if (gapDistance <= JOIN_SIDE_PATH_MAX_DISTANCE && isJoinSidePathPattern(first, second)) {
                mergeJoinSidePath(instructions, i, first, second);
                // Re-check: the merged instruction may form an M2 quick-sequence with its
                // new neighbor (e.g., "join cycleway on left, then right 30m").
                // M1→M1 is prevented by isJoinSidePathPattern rejecting already-merged
                // instructions (join_direction check).
                i--;
                continue;
            }

            // M2: Quick sequence — merge into single instruction with then_turn metadata.
            // Look-ahead: if the second instruction would be the first half of an M1
            // join-side-path pair with its successor, skip M2 and let M1 fire on the
            // next iteration. This preserves the M1 pattern (e.g., service road connector
            // to cycleway) which is more informative than an M2 quick-sequence.
            if (gapDistance <= QUICK_SEQUENCE_MAX_DISTANCE && isTurnInstruction(first) && isTurnInstruction(second)) {
                if (i + 2 < instructions.size()) {
                    Instruction third = instructions.get(i + 2);
                    if (!isTerminal(third)
                            && second.getDistance() <= JOIN_SIDE_PATH_MAX_DISTANCE
                            && isJoinSidePathPattern(second, third)) {
                        // Skip M2 — let the loop advance so M1 fires on [second]+[third]
                        continue;
                    }
                }
                mergeQuickSequence(instructions, i, first, second);
                // No re-check (no i--): the then_turn data model is a single object, not a
                // chain. Re-checking would overwrite the existing then_turn, losing the middle
                // turn. 3+ close turns beyond the first M2 pair remain as standalone instructions
                // for the client to handle contextually.
                continue;
            }
        }

        // M3: MTB consecutive forks
        mergeConsecutiveForks(instructions);
    }

    /**
     * M0: Suppress waypoint snap stub artifacts.
     * <p>
     * When a waypoint snaps to a crossing road at a junction (5-15m offset), the edge
     * sequence contains a short detour: turn onto crossing road → U-turn → turn back.
     * The route polyline barely deviates, but instruction generation produces confusing
     * turn-uturn-turn sequences.
     * <p>
     * Detection: sliding window [A, B, C] where:
     * - B is a U-turn (required anchor)
     * - A and B are on the same road (the stub road)
     * - A.distance + B.distance ≤ 30m (short round-trip)
     * - The road before A matches the road after the stub (C), confirming resumption
     * <p>
     * Action: remove A, B, and C; merge their distances into the preceding instruction.
     */
    private void suppressSnapStubs(InstructionList instructions) {
        for (int i = 0; i < instructions.size() - 2; i++) {
            Instruction a = instructions.get(i);
            Instruction b = instructions.get(i + 1);
            Instruction c = instructions.get(i + 2);

            if (isTerminal(a) || isTerminal(b) || isTerminal(c)) continue;

            // B must be a U-turn
            int bSign = b.getSign();
            if (bSign != Instruction.U_TURN_UNKNOWN
                    && bSign != Instruction.U_TURN_LEFT
                    && bSign != Instruction.U_TURN_RIGHT) continue;

            // Round-trip must be short
            double roundTrip = a.getDistance() + b.getDistance();
            if (roundTrip > SNAP_STUB_MAX_ROUND_TRIP) continue;

            // A and B should be on the same road (the stub road)
            String aName = a.getName();
            String bName = b.getName();
            if (aName == null || !aName.equals(bName)) continue;

            // The road before A must match the road after the stub (C)
            // i.e., the route resumes on the same road it was on before the detour
            String roadBeforeStub = extraStr(a, "prev_predicted_highway");
            String roadAfterStub = extraStr(c, "predicted_highway");
            if (roadBeforeStub == null || !roadBeforeStub.equals(roadAfterStub)) continue;

            // All conditions met — suppress the stub
            LOGGER.debug("M0: Suppressing snap stub at index {}: {} → U-turn → {} (round-trip {}m)",
                    i, aName, c.getName(), Math.round(roundTrip));

            // Remove C, B, A (reverse order to keep indices valid), merge into preceding
            if (i > 0) {
                double totalDist = a.getDistance() + b.getDistance() + c.getDistance();
                long totalTime = a.getTime() + b.getTime() + c.getTime();
                Instruction prev = instructions.get(i - 1);
                prev.setDistance(prev.getDistance() + totalDist);
                prev.setTime(prev.getTime() + totalTime);

                // Merge geometry: skip A and B's points (the stub), keep C's points
                PointList cPts = c.getPoints();
                PointList prevPts = prev.getPoints();
                for (int p = 0; p < cPts.size(); p++) {
                    prevPts.add(cPts.getLat(p), cPts.getLon(p),
                            cPts.is3D() ? cPts.getEle(p) : Double.NaN);
                }

                prev.setExtraInfo("snap_stub_suppressed", true);

                instructions.remove(i + 2);  // C
                instructions.remove(i + 1);  // B
                instructions.remove(i);      // A
                i--;  // re-check from previous position
            }
        }
    }

    /**
     * M1: Detect join-side-path pattern.
     * Two opposite turns with a short connector between them, where the user detours
     * briefly via a side path to transition between two different road types.
     * Example: riding on MINOR_ROAD → turn right onto 5m SERVICE_ROAD → turn left onto CYCLEWAY.
     * <p>
     * Critical gate: the source road must CONTINUE through the first junction.
     * At a T-junction the source road ends, forcing the user to turn — that's navigation,
     * not a "join via side path" pattern. The connector type is irrelevant.
     */
    private boolean isJoinSidePathPattern(Instruction first, Instruction second) {
        // Never merge onto an instruction that is already an M1 result — cascading
        // M1 merges collapse genuine multi-turn sequences into a single instruction.
        if (second.getExtraInfoJSON().containsKey("join_direction")) return false;

        int s1 = first.getSign();
        int s2 = second.getSign();

        // Only match actual turn instructions (|sign| 1–3), not KEEP_LEFT/RIGHT (±7)
        // or roundabout signs.
        if (!isTurnInstruction(first) || !isTurnInstruction(second)) return false;

        // Must be opposite direction turns (left then right, or right then left)
        boolean oppositeDirection = (s1 < 0 && s2 > 0) || (s1 > 0 && s2 < 0);
        if (!oppositeDirection) return false;

        // Source and destination must be different road types
        String sourcePH = extraStr(first, "prev_predicted_highway");
        String destPH = extraStr(second, "predicted_highway");
        if (sourcePH == null || destPH == null) return false;
        if (sourcePH.equals(destPH)) return false;

        // Geometry gate: both turns must look like a perpendicular sidepath crossing.
        // Each turn should be in the 60–120° range, and the net direction change should
        // be small (rider ends up going roughly the same heading as before).
        boolean geometryMatches = false;
        Object angle1Obj = first.getExtraInfoJSON().get("turn_angle_deg");
        Object angle2Obj = second.getExtraInfoJSON().get("turn_angle_deg");
        if (angle1Obj instanceof Number && angle2Obj instanceof Number) {
            double angle1 = ((Number) angle1Obj).doubleValue();
            double angle2 = ((Number) angle2Obj).doubleValue();
            double abs1 = Math.abs(angle1);
            double abs2 = Math.abs(angle2);
            double netAngle = Math.abs(angle1 + angle2);
            geometryMatches = abs1 >= 60 && abs1 <= 120 && abs2 >= 60 && abs2 <= 120
                    && netAngle <= 45;
        }

        // T-junction guard: at a true T-junction the source road ends, forcing the
        // rider to turn — that's navigation, not a sidepath crossing. However, if the
        // geometry is a clean perpendicular crossing (both angles 60-120°, net ≤ 45°),
        // the pattern is valid even when source_road_continues is false — the source
        // road may simply change PH at the junction (e.g., cycleway becomes footway).
        Boolean srcContinues = (Boolean) first.getExtraInfoJSON().get("source_road_continues");
        if (!Boolean.TRUE.equals(srcContinues) && !geometryMatches) return false;

        // Reject if geometry is wrong (extreme angles or large net direction change),
        // regardless of source_road_continues.
        if (!geometryMatches && angle1Obj instanceof Number) return false;

        return true;
    }

    /**
     * M1: Merge join-side-path into a single instruction.
     * The first instruction gets the second instruction's name and metadata,
     * with join_direction and join_target_type in extraInfo.
     */
    private void mergeJoinSidePath(InstructionList instructions, int index,
                                    Instruction first, Instruction second) {
        // Determine join direction from the first turn
        String joinDirection = first.getSign() < 0 ? "left" : "right";

        // The merged instruction uses the first turn's sign (the approach direction)
        // but the second instruction's name and road info (the destination)
        first.setName(second.getName());
        first.setDistance(first.getDistance() + second.getDistance());
        first.setTime(first.getTime() + second.getTime());

        // Merge geometry
        PointList mergedPts = first.getPoints();
        PointList secondPts = second.getPoints();
        for (int p = 0; p < secondPts.size(); p++) {
            mergedPts.add(secondPts.getLat(p), secondPts.getLon(p),
                    secondPts.is3D() ? secondPts.getEle(p) : Double.NaN);
        }

        // Set join metadata
        first.setExtraInfo("join_direction", joinDirection);
        first.setExtraInfo("join_target_type", extraStr(second, "predicted_highway"));

        // Copy destination road info from second instruction
        copyExtraIfPresent(second, first, "road_class");
        copyExtraIfPresent(second, first, "predicted_highway");
        copyExtraIfPresent(second, first, "predicted_surface");
        copyExtraIfPresent(second, first, "surface");
        copyExtraIfPresent(second, first, "bike_network");

        // Remove the second instruction
        instructions.remove(index + 1);

        LOGGER.debug("M1: Merged join-side-path at index {}: {} onto {}",
                index, joinDirection, second.getName());
    }

    /**
     * M2: Merge two close turns into a single instruction.
     * The first instruction absorbs the second. The second turn's details are
     * stored in then_turn so the client can render compound phrasing
     * (e.g., "left, then right 50m").
     */
    private void mergeQuickSequence(InstructionList instructions, int index,
                                     Instruction first, Instruction second) {
        Map<String, Object> thenTurn = new HashMap<>();
        thenTurn.put("sign", second.getSign());
        thenTurn.put("distance_m", Math.round(first.getDistance()));
        thenTurn.put("road_class", extraStr(second, "road_class"));
        thenTurn.put("predicted_highway", extraStr(second, "predicted_highway"));
        thenTurn.put("name", second.getName());
        first.setExtraInfo("then_turn", thenTurn);

        // Absorb second instruction's distance, time, and geometry
        first.setDistance(first.getDistance() + second.getDistance());
        first.setTime(first.getTime() + second.getTime());

        PointList secondPts = second.getPoints();
        PointList firstPts = first.getPoints();
        for (int p = 0; p < secondPts.size(); p++) {
            firstPts.add(secondPts.getLat(p), secondPts.getLon(p),
                    secondPts.is3D() ? secondPts.getEle(p) : Double.NaN);
        }

        instructions.remove(index + 1);

        LOGGER.debug("M2: Merged quick sequence at index {}: sign={} then sign={} in {}m",
                index, first.getSign(), thenTurn.get("sign"), thenTurn.get("distance_m"));
    }

    /**
     * M3: Merge consecutive same-direction KEEP_LEFT/KEEP_RIGHT forks on trails.
     */
    private void mergeConsecutiveForks(InstructionList instructions) {
        for (int i = 0; i < instructions.size() - 1; i++) {
            Instruction first = instructions.get(i);
            if (first.getSign() != Instruction.KEEP_LEFT && first.getSign() != Instruction.KEEP_RIGHT) continue;
            if (!isTrailPH(extraStr(first, "predicted_highway"))) continue;

            int targetSign = first.getSign();
            int count = 1;
            double totalDist = first.getDistance();
            int lastMerged = i;

            // Look ahead for consecutive same-direction forks
            for (int j = i + 1; j < instructions.size(); j++) {
                Instruction next = instructions.get(j);
                if (next.getSign() != targetSign) break;
                if (totalDist > MTB_CONSECUTIVE_FORK_MAX_DISTANCE) break;
                if (!isTrailPH(extraStr(next, "predicted_highway"))) break;

                count++;
                totalDist += next.getDistance();
                lastMerged = j;
            }

            if (count >= 2) {
                // Merge into first instruction
                for (int j = lastMerged; j > i; j--) {
                    Instruction removed = instructions.get(j);
                    first.setDistance(first.getDistance() + removed.getDistance());
                    first.setTime(first.getTime() + removed.getTime());

                    PointList removedPts = removed.getPoints();
                    PointList firstPts = first.getPoints();
                    for (int p = 0; p < removedPts.size(); p++) {
                        firstPts.add(removedPts.getLat(p), removedPts.getLon(p),
                                removedPts.is3D() ? removedPts.getEle(p) : Double.NaN);
                    }
                    instructions.remove(j);
                }

                String direction = targetSign == Instruction.KEEP_LEFT ? "left" : "right";
                first.setExtraInfo("sequence_tag", "stay_" + direction + "_" + count);

                LOGGER.debug("M3: Merged {} consecutive {} forks at index {}", count, direction, i);
            }
        }
    }

    // ========================================================================
    // Pass 2: Continuity Suppression
    // ========================================================================

    /**
     * Remove instructions that add no navigation value:
     * C1: CONTINUE_ON_STREET where same road class, unnamed, no class change
     * C2: Micro-artifacts (distance < 5m)
     */
    private void pass2_continuitySuppression(InstructionList instructions) {
        for (int i = instructions.size() - 2; i >= 1; i--) {
            Instruction instr = instructions.get(i);
            if (isTerminal(instr)) continue;

            // Skip direct-segment markers — must not be suppressed
            if (Boolean.FALSE.equals(instr.getExtraInfoJSON().get("tbt_available"))) continue;

            // Skip visual guidance instructions — intentionally created, not artifacts
            if (Boolean.TRUE.equals(instr.getExtraInfoJSON().get("visual_guidance"))) continue;

            // C2: Micro-artifact — very short instruction likely from snapping.
            // Spare instructions with a significant turn: a real junction with a short
            // road after it is navigation-critical, not a snapping artifact.
            if (instr.getDistance() < MICRO_ARTIFACT_MAX_DISTANCE
                    && instr.getSign() != Instruction.USE_ROUNDABOUT
                    && !isSignificantTurn(instr)) {
                mergeIntoPrevious(instructions, i);
                LOGGER.debug("C2: Removed micro-artifact at index {} ({}m)", i, instr.getDistance());
                continue;
            }

            // C1: Meaningless CONTINUE_ON_STREET (but preserve E2 trail fork instructions)
            if (instr.getSign() == Instruction.CONTINUE_ON_STREET) {
                boolean trailFork = Boolean.TRUE.equals(instr.getExtraInfoJSON().get("trail_fork"));
                if (trailFork) continue;

                String rc = extraStr(instr, "road_class");
                String prevRC = extraStr(instr, "prev_road_class");
                boolean hasName = Boolean.TRUE.equals(instr.getExtraInfoJSON().get("has_name"));
                boolean rcChanged = Boolean.TRUE.equals(instr.getExtraInfoJSON().get("road_class_changed"));

                if (!hasName && !rcChanged && Objects.equals(rc, prevRC)) {
                    mergeIntoPrevious(instructions, i);
                    LOGGER.debug("C1: Removed meaningless CONTINUE at index {}", i);
                }
            }
        }
    }

    /**
     * Remove instruction at index, merging its distance/time/geometry into previous.
     */
    private void mergeIntoPrevious(InstructionList instructions, int index) {
        if (index <= 0) return;
        Instruction removed = instructions.get(index);
        Instruction prev = instructions.get(index - 1);

        prev.setDistance(prev.getDistance() + removed.getDistance());
        prev.setTime(prev.getTime() + removed.getTime());

        PointList removedPts = removed.getPoints();
        PointList prevPts = prev.getPoints();
        for (int p = 0; p < removedPts.size(); p++) {
            prevPts.add(removedPts.getLat(p), removedPts.getLon(p),
                    removedPts.is3D() ? removedPts.getEle(p) : Double.NaN);
        }

        instructions.remove(index);
    }

    // ========================================================================
    // Pass 3: Confirmation Insertion
    // ========================================================================

    /**
     * Insert "continue straight" instructions where helpful:
     * F1: PredictedHighway changes while going straight (no existing instruction)
     * F2: Long gap (> 2km) between instructions — reassurance
     */
    private void pass3_confirmationInsertion(InstructionList instructions) {
        // F2: Reassurance for long gaps
        for (int i = 0; i < instructions.size() - 1; i++) {
            Instruction instr = instructions.get(i);
            if (isTerminal(instr)) continue;

            if (instr.getDistance() > REASSURANCE_GAP) {
                // Insert a confirmation roughly at the midpoint
                // For now, just annotate the existing instruction — actual splitting
                // requires knowing the geometry midpoint, which is a more complex operation.
                // The client can use this flag to generate a reassurance alert mid-segment.
                instr.setExtraInfo("needs_reassurance", true);
                LOGGER.debug("F2: Marked reassurance needed at index {} ({}m gap)", i, Math.round(instr.getDistance()));
            }
        }

        // F1 is handled by Stage 1's E1 emit rule — PH changes already generate instructions.
        // If Stage 1 missed any (e.g., within IGNORE paths), we would detect them here by
        // scanning the junctions_crossed data. Deferred until junction accumulation is implemented.
    }

    // ========================================================================
    // Pass 4: Annotation
    // ========================================================================

    /**
     * Assign tbt_priority to each instruction based on its characteristics.
     * Priority determines client-side rendering: voice TTS, beep, or silent.
     */
    private void pass4_annotation(InstructionList instructions) {
        for (int i = 0; i < instructions.size(); i++) {
            Instruction instr = instructions.get(i);
            if (isTerminal(instr)) continue;

            String priority = determinePriority(instr);
            instr.setExtraInfo("tbt_priority", priority);
        }
    }

    /**
     * Determine tbt_priority for a single instruction.
     * Rules checked in order — first match wins.
     */
    private String determinePriority(Instruction instr) {
        int sign = instr.getSign();
        String ph = extraStr(instr, "predicted_highway");
        boolean hasName = Boolean.TRUE.equals(instr.getExtraInfoJSON().get("has_name"));
        boolean rcChanged = Boolean.TRUE.equals(instr.getExtraInfoJSON().get("road_class_changed"));

        // Visual guidance → visual (checked first — these instructions exist only
        // because the visual side-channel flagged them; other rules would assign
        // voice/beep based on the instruction's road properties, defeating the purpose)
        if (Boolean.TRUE.equals(instr.getExtraInfoJSON().get("visual_guidance"))) {
            return "visual";
        }

        // A0: Direct segment boundary → voice
        if (Boolean.FALSE.equals(instr.getExtraInfoJSON().get("tbt_available"))) {
            return "voice";
        }

        // A6: Join-side-path (merged M1) → always voice
        if (instr.getExtraInfoJSON().containsKey("join_direction")) {
            return "voice";
        }

        // A2: Road class transition → voice
        if (rcChanged) {
            return "voice";
        }

        // A3: Sharp turn (|sign| >= 2) on any road type → voice
        if (Math.abs(sign) >= 2) {
            return "voice";
        }

        // A1: On prominent road types → voice
        if (isProminentPH(ph)) {
            return "voice";
        }

        // A1 variant: Named way → voice
        if (hasName) {
            return "voice";
        }

        // A4: KEEP_LEFT/KEEP_RIGHT fork on trails → beep
        if (sign == Instruction.KEEP_LEFT || sign == Instruction.KEEP_RIGHT) {
            return "beep";
        }

        // A5: CONTINUE_ON_STREET confirmation → beep
        if (sign == Instruction.CONTINUE_ON_STREET) {
            return "beep";
        }

        // A7: Roundabout → voice
        if (sign == Instruction.USE_ROUNDABOUT) {
            return "voice";
        }

        // Default → beep
        return "beep";
    }

    // ========================================================================
    // Helpers
    // ========================================================================

    private static boolean isTerminal(Instruction instr) {
        return instr.getSign() == Instruction.FINISH
                || instr.getSign() == Instruction.REACHED_VIA;
    }

    private static boolean isTurnInstruction(Instruction instr) {
        int s = Math.abs(instr.getSign());
        return s >= 1 && s <= 3;
    }

    /**
     * A significant turn is a real navigation decision at a junction, not a snapping artifact.
     * True when the turn angle is >= 30° or the sign indicates at least a normal turn (|sign| >= 2).
     */
    private static boolean isSignificantTurn(Instruction instr) {
        if (Math.abs(instr.getSign()) >= 2) return true;
        Object angleObj = instr.getExtraInfoJSON().get("turn_angle_deg");
        if (angleObj instanceof Number) {
            return Math.abs(((Number) angleObj).doubleValue()) >= 30;
        }
        return false;
    }

    private static boolean isTrailPH(String ph) {
        if (ph == null) return false;
        switch (ph) {
            case "OUTDOOR_WAY":
            case "OUTDOOR_PATH":
            case "GOOD_TRACK":
            case "ROUGH_TRACK":
            case "PATH":
            case "CITY_PATH":
                return true;
            default:
                return false;
        }
    }

    private static boolean isProminentPH(String ph) {
        if (ph == null) return false;
        switch (ph) {
            case "MAJOR_ROAD":
            case "MINOR_ROAD":
            case "CYCLEWAY":
                return true;
            default:
                return false;
        }
    }

    private static String extraStr(Instruction instr, String key) {
        Object val = instr.getExtraInfoJSON().get(key);
        return val != null ? val.toString() : null;
    }

    private static void copyExtraIfPresent(Instruction from, Instruction to, String key) {
        Object val = from.getExtraInfoJSON().get(key);
        if (val != null) {
            to.setExtraInfo(key, val);
        }
    }
}
