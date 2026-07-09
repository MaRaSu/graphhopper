package com.graphhopper.trailmap.tbt;

import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.FetchMode;
import com.graphhopper.util.Instruction;
import com.graphhopper.util.InstructionList;
import com.graphhopper.util.PointList;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Independent, distance-anchored placement oracle for TbT instructions.
 * See docs/gh_tbt_instruction_placement_validation_design.md.
 *
 * <p>Ground truth is rebuilt from {@link RouteInstructionGenerator#PLACEMENT_TRACE_LOG}
 * (append-time assembly facts: final edge chains, bridge chains, gap geometry, and each
 * record's index range in the shipped polyline) plus the directed graph's full pillar
 * geometry. Each instruction's expected position is its creation-time anchor — the
 * {@code _gen_node_id} / {@code _gen_cum_synth_m} stamps set by
 * {@link TrailmapInstructionsFromEdges} at instruction creation — resolved to a cumulative
 * route distance along the un-simplified chain geometry.
 *
 * <p>HARD INVARIANT (non-circularity): the oracle never reads {@code _cum_route_m},
 * {@code _polyline_start_hint}, instruction PointLists, or the reported intervals to decide
 * the truth. Those are the subjects under test. The reported interval enters only on the
 * "reported" side of the comparison.
 */
class InstructionPlacementOracle {

    // ---- Tunables (see design doc §6: bug signal is 30–450 m, oracle noise < 5 m) ----
    static double EPS_DIST_M = 5.0;      // CHECK 1: |s_report − s_oracle| tolerance
    static double EPS_PERP_M = 2.0;      // CHECK 3: perpendicular offset tolerance
    static final double SNAP_LOCATE_PERP_TOL_M = 5.0;  // snap point must sit on its edge within this
    static final double STAMP_MATCH_TOL_M = 0.5;       // (_gen_node_id, _gen_cum_synth_m) → chain position
    static final double SLICE_LEN_TOL_M = 10.0;        // oracle self-check: trimmed chain vs slice length
    static final double DIST_TRIANG_TOL_M = 3.0;       // CHECK 5 tolerance (plus 1% relative)

    private static final DistanceCalcEarth DIST = DistanceCalcEarth.DIST_EARTH;

    // ================================================================== model

    /** A graph node's position along one record's chain. */
    static final class NodePos {
        final int node;
        final double geomDistM;   // cumulative 2D distance along full pillar geometry
        final double edgeDistM;   // cumulative sum of edge.getDistance() — matches _gen_cum_synth_m exactly
        final double lat, lon;

        NodePos(int node, double geomDistM, double edgeDistM, double lat, double lon) {
            this.node = node;
            this.geomDistM = geomDistM;
            this.edgeDistM = edgeDistM;
            this.lat = lat;
            this.lon = lon;
        }
    }

    /** Ground-truth model of one trace record. */
    static final class RecordModel {
        final RouteInstructionGenerator.PlacementTraceRecord rec;
        double globalStartS;      // route distance at this record's effective slice start
        double sliceLenM;         // shipped-polyline length of this record's slice
        // ROUTED/BRIDGE only:
        List<NodePos> nodes;      // chain node positions (size = edges + 1)
        double tStart;            // chain geom distance of the snapped slice start
        double tEnd;              // chain geom distance of the snapped slice end
        RecordModel bridge;       // the BRIDGE record whose edges are this chain's prefix (or null)
        double bridgeLeadInM;     // bridge-slice distance from bridge slice start to chain start
        final List<String> warnings = new ArrayList<>();

        RecordModel(RouteInstructionGenerator.PlacementTraceRecord rec) {
            this.rec = rec;
        }
    }

    /** One walked chain candidate (used for orientation scoring, then kept). */
    private static final class ChainWalk {
        List<NodePos> nodes;
        PointList geom;           // concatenated full pillar geometry, travel direction
        double[] geomCum;         // cumulative 2D distance per geometry point
        int[] nodeGeomIdx;        // geometry index of each chain node
        double totalEdgeDistM;
    }

    // ================================================================== report

    static final class Row {
        int idx;
        int sign;
        String signName;
        Integer node;             // creation anchor node (null if none)
        Double sOracle;           // expected route distance (null → exempt from CHECK 1)
        double sReport;           // route distance of the reported interval start
        Double perpM;             // CHECK 3 offset (null if not applicable)
        boolean pass = true;      // CHECK 1 verdict (true when exempt)
        String note = "";
    }

    static final class Report {
        final List<Row> rows = new ArrayList<>();
        final List<String> buildErrors = new ArrayList<>();
        final List<String> warnings = new ArrayList<>();
        final List<String> structuralFailures = new ArrayList<>();
        double totalRouteM;
        int checkedCount;

        boolean oracleBuildOk() { return buildErrors.isEmpty(); }

        List<Row> check1Failures() {
            List<Row> f = new ArrayList<>();
            for (Row r : rows) if (!r.pass) f.add(r);
            return f;
        }

        boolean allPass() {
            return oracleBuildOk() && check1Failures().isEmpty() && structuralFailures.isEmpty();
        }

        /** Verbatim-output-first verdict table (mirrors the why_way.sh convention). */
        String table() {
            StringBuilder sb = new StringBuilder();
            sb.append(String.format("route: %.1f km, %d instructions, %d placement-checked%n",
                    totalRouteM / 1000.0, rows.size(), checkedCount));
            for (String e : buildErrors) sb.append("ORACLE BUILD ERROR: ").append(e).append('\n');
            for (String w : warnings) sb.append("warning: ").append(w).append('\n');
            sb.append(String.format("%-4s %-22s %-10s %11s %11s %9s %7s  %s%n",
                    "#", "sign", "node", "s_oracle", "s_report", "ds", "perp", "verdict"));
            for (Row r : rows) {
                sb.append(String.format(Locale.ROOT, "%-4d %-22s %-10s %11s %11.1f %9s %7s  %s %s%n",
                        r.idx, r.signName,
                        r.node == null ? "-" : String.valueOf(r.node),
                        r.sOracle == null ? "-" : String.format(Locale.ROOT, "%.1f", r.sOracle),
                        r.sReport,
                        r.sOracle == null ? "-" : String.format(Locale.ROOT, "%.1f", r.sReport - r.sOracle),
                        r.perpM == null ? "-" : String.format(Locale.ROOT, "%.1f", r.perpM),
                        r.sOracle == null ? "exempt" : (r.pass ? "PASS" : "FAIL"),
                        r.note));
            }
            for (String s : structuralFailures) sb.append("STRUCTURAL FAIL: ").append(s).append('\n');
            sb.append(allPass() ? "VERDICT: ALL PASS" : "VERDICT: FAIL").append('\n');
            return sb.toString();
        }
    }

    // ================================================================== entry point

    /**
     * Run the full endpoint-mirror pipeline (generate → post-process → matchInstructionStarts)
     * with the placement trace enabled, then validate placement against the oracle.
     */
    static Report runAndValidate(BaseGraph graph, RouteInstructionGenerator generator,
                                 InstructionPostProcessor postProcessor,
                                 TrailmapInstructionRequest request) {
        RouteInstructionGenerator.PLACEMENT_TRACE_LOG.clear();
        RouteInstructionGenerator.PLACEMENT_TRACE = true;
        RouteInstructionGenerator.Result result;
        List<RouteInstructionGenerator.PlacementTraceRecord> records;
        try {
            result = generator.generate(request);
            records = new ArrayList<>(RouteInstructionGenerator.PLACEMENT_TRACE_LOG);
        } finally {
            RouteInstructionGenerator.PLACEMENT_TRACE = false;
            RouteInstructionGenerator.PLACEMENT_TRACE_LOG.clear();
        }
        postProcessor.process(result.instructions, request.getInstructionProfile());
        List<Integer> intervalStarts =
                RouteInstructionGenerator.matchInstructionStarts(result.instructions, result.polyline);
        return validate(graph, records, result.instructions, result.polyline, intervalStarts);
    }

    /** Build the oracle from trace records and evaluate all checks against the final list. */
    static Report validate(BaseGraph graph,
                           List<RouteInstructionGenerator.PlacementTraceRecord> records,
                           InstructionList instructions, PointList polyline,
                           List<Integer> intervalStarts) {
        Report report = new Report();

        // Route distance along the SHIPPED polyline (2D) — the scale both sides are compared on.
        double[] polyCum = cumulativeDistances(polyline);
        report.totalRouteM = polyCum.length == 0 ? 0 : polyCum[polyCum.length - 1];

        List<RecordModel> models = buildModels(graph, records, polyline, polyCum, report);

        evaluate(models, instructions, polyline, polyCum, intervalStarts, report);
        return report;
    }

    // ================================================================== oracle build

    private static List<RecordModel> buildModels(BaseGraph graph,
                                                 List<RouteInstructionGenerator.PlacementTraceRecord> records,
                                                 PointList polyline, double[] polyCum, Report report) {
        List<RecordModel> models = new ArrayList<>();
        RecordModel pendingBridge = null;

        for (RouteInstructionGenerator.PlacementTraceRecord rec : records) {
            RecordModel m = new RecordModel(rec);
            int effStartIdx = effectiveSliceStart(rec, polyline);
            m.globalStartS = polyCum[effStartIdx];
            int lastIdx = Math.max(effStartIdx, Math.min(rec.polyAfter - 1, polyCum.length - 1));
            m.sliceLenM = polyCum[lastIdx] - polyCum[effStartIdx];

            if (rec.type == RouteInstructionGenerator.PlacementTraceRecord.GAP) {
                models.add(m);
                continue;
            }

            if (rec.edgeIds.isEmpty()) {
                if (rec.type == RouteInstructionGenerator.PlacementTraceRecord.BRIDGE) {
                    pendingBridge = m;   // geometry-only bridge (all edges deduped away)
                }
                models.add(m);
                continue;
            }

            int bridgePrefix = 0;
            if (rec.type == RouteInstructionGenerator.PlacementTraceRecord.ROUTED && pendingBridge != null) {
                bridgePrefix = pendingBridge.rec.edgeIds.size();
                if (bridgePrefix > rec.edgeIds.size()
                        || !rec.edgeIds.subList(0, bridgePrefix).equals(pendingBridge.rec.edgeIds)) {
                    m.warnings.add("bridge edges are not a prefix of the routed chain — ignoring bridge pairing");
                    bridgePrefix = 0;
                } else {
                    m.bridge = pendingBridge;
                }
            }

            buildChainModel(graph, m, bridgePrefix, polyline, report);
            report.warnings.addAll(m.warnings);

            pendingBridge = rec.type == RouteInstructionGenerator.PlacementTraceRecord.BRIDGE ? m : null;
            models.add(m);
        }
        return models;
    }

    /**
     * Walk the record's edge chain into full oriented pillar geometry, orient it (connectivity
     * first, snap-endpoint score as tie-breaker), and trim it to the record's snapped slice.
     * A chain that cannot be walked from either end is the HTTP-500 disconnection class —
     * reported as an oracle build error naming the offending record.
     */
    private static void buildChainModel(BaseGraph graph, RecordModel m, int bridgePrefix,
                                        PointList polyline, Report report) {
        List<Integer> edgeIds = m.rec.edgeIds;
        EdgeIteratorState first = graph.getEdgeIteratorState(edgeIds.get(0), Integer.MIN_VALUE);
        int[] candidates = first.getBaseNode() == first.getAdjNode()
                ? new int[]{first.getBaseNode()}
                : new int[]{first.getBaseNode(), first.getAdjNode()};

        ChainWalk best = null;
        double bestScore = Double.MAX_VALUE;
        String bestDesc = null;
        for (int fromNode : candidates) {
            ChainWalk walk = walkChain(graph, edgeIds, fromNode);
            if (walk == null) continue;
            double[] trim = locateSlice(walk, m.rec, bridgePrefix);
            double trimmedLen = trim[1] - trim[0];
            double score = Math.abs(trimmedLen - m.sliceLenM) + trim[2] + trim[3];
            if (score < bestScore) {
                bestScore = score;
                best = walk;
                m.tStart = trim[0];
                m.tEnd = trim[1];
                bestDesc = String.format(Locale.ROOT,
                        "fromNode=%d trimmedLen=%.1f sliceLen=%.1f perpStart=%.2f perpEnd=%.2f",
                        fromNode, trimmedLen, m.sliceLenM, trim[2], trim[3]);
            }
        }

        if (best == null) {
            report.buildErrors.add("chain does not walk node-to-node from either end of edge "
                    + edgeIds.get(0) + " (record edges=" + abbreviate(edgeIds) + ") — disconnected assembly");
            return;
        }
        m.nodes = best.nodes;

        double trimmedLen = m.tEnd - m.tStart;
        if (Math.abs(trimmedLen - m.sliceLenM) > SLICE_LEN_TOL_M) {
            m.warnings.add(String.format(Locale.ROOT,
                    "record self-check: trimmed chain length %.1f m vs slice length %.1f m (Δ %.1f m; %s)",
                    trimmedLen, m.sliceLenM, trimmedLen - m.sliceLenM, bestDesc));
        }

        if (m.bridge != null && m.bridge.nodes != null && !m.bridge.nodes.isEmpty()) {
            // Lead-in from the bridge's slice start to this chain's start node, measured on the
            // bridge slice: locate the chain start coordinate on the bridge slice geometry.
            NodePos chainStart = m.nodes.get(0);
            double[] loc = locateOnPolylineRange(polyline, m.bridge.rec.polyBefore,
                    m.bridge.rec.polyAfter, chainStart.lat, chainStart.lon);
            m.bridgeLeadInM = loc[0];
        }
    }

    /** Walk the chain from fromNode; null if it breaks (candidate direction invalid). */
    private static ChainWalk walkChain(BaseGraph graph, List<Integer> edgeIds, int fromNode) {
        NodeAccess na = graph.getNodeAccess();
        ChainWalk w = new ChainWalk();
        w.nodes = new ArrayList<>(edgeIds.size() + 1);
        w.geom = new PointList(edgeIds.size() * 4, false);
        w.nodeGeomIdx = new int[edgeIds.size() + 1];

        int cur = fromNode;
        double edgeDist = 0;
        w.geom.add(na.getLat(cur), na.getLon(cur));
        w.nodes.add(new NodePos(cur, 0, 0, na.getLat(cur), na.getLon(cur)));
        w.nodeGeomIdx[0] = 0;

        for (int i = 0; i < edgeIds.size(); i++) {
            EdgeIteratorState es = graph.getEdgeIteratorState(edgeIds.get(i), Integer.MIN_VALUE);
            int next;
            if (es.getBaseNode() == cur) next = es.getAdjNode();
            else if (es.getAdjNode() == cur) next = es.getBaseNode();
            else return null;

            // Oriented state: geometry runs base(cur) → adj(next) in travel direction.
            EdgeIteratorState oriented = graph.getEdgeIteratorState(edgeIds.get(i), next);
            if (oriented == null) return null;
            PointList wayGeo = oriented.fetchWayGeometry(FetchMode.ALL);
            for (int p = 1; p < wayGeo.size(); p++) {   // skip shared first point
                w.geom.add(wayGeo.getLat(p), wayGeo.getLon(p));
            }
            edgeDist += es.getDistance();
            cur = next;
            w.nodeGeomIdx[i + 1] = w.geom.size() - 1;
            w.nodes.add(new NodePos(cur, Double.NaN /* filled below */, edgeDist,
                    na.getLat(cur), na.getLon(cur)));
        }

        w.geomCum = cumulativeDistances(w.geom);
        w.totalEdgeDistM = edgeDist;
        List<NodePos> fixed = new ArrayList<>(w.nodes.size());
        for (int i = 0; i < w.nodes.size(); i++) {
            NodePos n = w.nodes.get(i);
            fixed.add(new NodePos(n.node, w.geomCum[w.nodeGeomIdx[i]], n.edgeDistM, n.lat, n.lon));
        }
        w.nodes = fixed;
        return w;
    }

    /**
     * Locate the record's snapped slice on the walked chain.
     * Returns {tStart, tEnd, perpStart, perpEnd} (chain geometry distances / meters).
     * snapStart lives on the first non-bridge edge; snapEnd on the last edge — searched there
     * first, widened to the whole chain (with the perp cost reported) only if not found.
     */
    private static double[] locateSlice(ChainWalk w, RouteInstructionGenerator.PlacementTraceRecord rec,
                                        int bridgePrefix) {
        int startEdge = Math.min(bridgePrefix, w.nodeGeomIdx.length - 2);
        double[] s = locateOnGeomRange(w.geom, w.geomCum,
                w.nodeGeomIdx[startEdge], w.nodeGeomIdx[startEdge + 1], rec.snapStartLat, rec.snapStartLon);
        if (s[1] > SNAP_LOCATE_PERP_TOL_M) {
            s = locateOnGeomRange(w.geom, w.geomCum, 0, w.geom.size() - 1, rec.snapStartLat, rec.snapStartLon);
        }
        int lastEdge = w.nodeGeomIdx.length - 2;
        double[] e = locateOnGeomRange(w.geom, w.geomCum,
                w.nodeGeomIdx[lastEdge], w.nodeGeomIdx[lastEdge + 1], rec.snapEndLat, rec.snapEndLon);
        if (e[1] > SNAP_LOCATE_PERP_TOL_M) {
            e = locateOnGeomRange(w.geom, w.geomCum, 0, w.geom.size() - 1, rec.snapEndLat, rec.snapEndLon);
        }
        return new double[]{s[0], e[0], s[1], e[1]};
    }

    // ================================================================== evaluation

    private static void evaluate(List<RecordModel> models,
                                 InstructionList instructions, PointList polyline, double[] polyCum,
                                 List<Integer> intervalStarts, Report report) {
        // Cursors enforcing route-order (monotone) matching.
        int recCursor = 0, nodeCursor = 0, gapCursor = -1;
        double prevOracleS = -1;

        for (int i = 0; i < instructions.size(); i++) {
            Instruction instr = instructions.get(i);
            Row row = new Row();
            row.idx = i;
            row.sign = instr.getSign();
            row.signName = signName(instr);
            int startIdx = intervalStarts.get(i);
            row.sReport = polyCum[Math.max(0, Math.min(startIdx, polyCum.length - 1))];

            Object nodeObj = instr.getExtraInfoJSON().get("_gen_node_id");
            Object synthObj = instr.getExtraInfoJSON().get("_gen_cum_synth_m");
            Object tbtAvailable = instr.getExtraInfoJSON().get("tbt_available");
            NodePos matchedPos = null;

            if (nodeObj instanceof Number && synthObj instanceof Number) {
                int node = ((Number) nodeObj).intValue();
                double synthM = ((Number) synthObj).doubleValue();
                row.node = node;
                int[] match = findChainPosition(models, recCursor, nodeCursor, node, synthM);
                if (match == null) {
                    match = findChainPosition(models, 0, 0, node, synthM);
                    if (match != null) {
                        row.note = "matched BEHIND the monotone cursor (ordering anomaly)";
                        report.warnings.add("instr " + i + ": " + row.note);
                    }
                }
                if (match == null) {
                    report.warnings.add("instr " + i + " (" + row.signName + "): creation stamp (node="
                            + node + ", synth=" + String.format(Locale.ROOT, "%.1f", synthM)
                            + ") not found on any traced chain — exempted");
                    row.note = "stamp not on any chain";
                } else {
                    recCursor = match[0];
                    nodeCursor = match[1];
                    RecordModel m = models.get(match[0]);
                    matchedPos = m.nodes.get(match[1]);
                    row.sOracle = expectedS(m, matchedPos, row);
                }
            } else if (Boolean.FALSE.equals(tbtAvailable)) {
                // Synthetic gap marker: anchored at its gap record's start.
                int g = nextGapRecord(models, gapCursor);
                if (g >= 0) {
                    gapCursor = g;
                    row.sOracle = models.get(g).globalStartS;
                    row.note = "gap marker";
                } else {
                    report.warnings.add("instr " + i + ": gap marker without a matching GAP trace record");
                }
            } else if (instr.getSign() == Instruction.FINISH) {
                // Generator-appended synthetic FINISH (route ends with a gap): route end.
                row.sOracle = report.totalRouteM;
                row.note = "synthetic finish";
            } else {
                row.note = "no creation anchor (post-processor insert)";
            }

            if (row.sOracle != null) {
                report.checkedCount++;
                double ds = row.sReport - row.sOracle;
                row.pass = Math.abs(ds) <= EPS_DIST_M;
                if (!row.pass) {
                    String diag = String.format(Locale.ROOT, "off by %.1f m", ds);
                    if (matchedPos != null) {
                        // Wrong-occurrence confirmation: does the anchor coordinate ALSO lie on the
                        // polyline at the REPORTED position? If yes, the same coordinate was matched
                        // on a different pass — the out-and-back signature (Δs ≈ 2 × spur).
                        double perpAtReport = perpNear(polyline, polyCum, row.sReport,
                                matchedPos.lat, matchedPos.lon, 5.0);
                        diag += perpAtReport <= EPS_PERP_M
                                ? String.format(Locale.ROOT,
                                        " — WRONG OCCURRENCE confirmed: same coord at both passes (spur ~%.1f m one-way)",
                                        Math.abs(ds) / 2)
                                : String.format(Locale.ROOT, " (anchor coord %.1f m off polyline at reported pos)",
                                        perpAtReport);
                    }
                    row.note = appendNote(row.note, diag);
                }
                // CHECK 2 (oracle sanity): expected positions must be non-decreasing.
                if (row.sOracle < prevOracleS - 1.0) {
                    report.structuralFailures.add("oracle s not monotone at instr " + i
                            + String.format(Locale.ROOT, " (%.1f after %.1f)", row.sOracle, prevOracleS));
                }
                prevOracleS = Math.max(prevOracleS, row.sOracle);
                // CHECK 3 (secondary): anchor coordinate must lie on the polyline near s_oracle.
                if (matchedPos != null) {
                    row.perpM = perpNear(polyline, polyCum, row.sOracle, matchedPos.lat, matchedPos.lon, 30.0);
                }
            }
            report.rows.add(row);
        }

        structuralChecks(instructions, polyline, polyCum, intervalStarts, report);
    }

    private static void structuralChecks(InstructionList instructions, PointList polyline,
                                         double[] polyCum, List<Integer> intervalStarts, Report report) {
        // CHECK 2: reported interval starts non-decreasing.
        for (int i = 1; i < intervalStarts.size(); i++) {
            if (intervalStarts.get(i) < intervalStarts.get(i - 1)) {
                report.structuralFailures.add("interval starts not monotone at instr " + i
                        + " (" + intervalStarts.get(i) + " after " + intervalStarts.get(i - 1) + ")");
            }
        }
        // CHECK 4: tiling — first at 0, FINISH at the last polyline index, single FINISH.
        if (!intervalStarts.isEmpty() && intervalStarts.get(0) != 0) {
            report.structuralFailures.add("first interval does not start at 0 (was " + intervalStarts.get(0) + ")");
        }
        int finishCount = 0;
        for (Instruction instr : instructions) if (instr.getSign() == Instruction.FINISH) finishCount++;
        if (finishCount != 1) {
            report.structuralFailures.add("expected exactly one FINISH, found " + finishCount);
        }
        if (!instructions.isEmpty()
                && instructions.get(instructions.size() - 1).getSign() == Instruction.FINISH
                && intervalStarts.get(intervalStarts.size() - 1) != polyline.size() - 1) {
            report.structuralFailures.add("FINISH interval start " + intervalStarts.get(intervalStarts.size() - 1)
                    + " != last polyline index " + (polyline.size() - 1));
        }
        // CHECK 5 (secondary): reported distance ≈ interval polyline span.
        for (int i = 0; i + 1 < instructions.size(); i++) {
            double span = polyCum[clampIdx(intervalStarts.get(i + 1), polyCum.length)]
                    - polyCum[clampIdx(intervalStarts.get(i), polyCum.length)];
            double d = instructions.get(i).getDistance();
            double tol = DIST_TRIANG_TOL_M + 0.01 * Math.max(span, d);
            if (Math.abs(span - d) > tol) {
                report.warnings.add(String.format(Locale.ROOT,
                        "instr %d: reported distance %.1f m vs interval span %.1f m (Δ %.1f m)",
                        i, d, span, d - span));
            }
        }
    }

    /** Expected route distance for a chain position, applying the seam clamp rules. */
    private static double expectedS(RecordModel m, NodePos pos, Row row) {
        double d = pos.geomDistM;
        if (m.nodes == null) return m.globalStartS;
        if (d < m.tStart - 0.01) {
            // Anchor precedes the snapped slice start: a pre-snap context node (boundary U-turn
            // far endpoint, gap-resume graph node) — production anchors these at the seam — or a
            // junction on a bridge prefix, whose geometry lives in the preceding bridge slice.
            if (m.bridge != null) {
                double s = m.bridge.globalStartS + m.bridgeLeadInM + d;
                double bridgeEnd = m.bridge.globalStartS + m.bridge.sliceLenM;
                if (s <= bridgeEnd + 1.0) {
                    row.note = appendNote(row.note, "on bridge prefix");
                    return Math.min(s, bridgeEnd);
                }
            }
            row.note = appendNote(row.note, "pre-snap anchor clamped to seam");
            return m.globalStartS;
        }
        if (d > m.tEnd + 0.01) {
            row.note = appendNote(row.note, "post-snap anchor clamped to slice end");
            return m.globalStartS + (m.tEnd - m.tStart);
        }
        return m.globalStartS + (d - m.tStart);
    }

    /**
     * Find the chain position matching a creation stamp, searching forward from the cursor.
     * Match = same node id AND cumulative edge distance within {@link #STAMP_MATCH_TOL_M}
     * (exact by construction: both sides sum the same edge.getDistance() values), which makes
     * the out-and-back occurrence choice unambiguous.
     * Returns {recordIdx, nodeIdx} or null.
     */
    private static int[] findChainPosition(List<RecordModel> models, int recFrom, int nodeFrom,
                                           int node, double synthM) {
        for (int r = recFrom; r < models.size(); r++) {
            RecordModel m = models.get(r);
            if (m.nodes == null
                    || m.rec.type != RouteInstructionGenerator.PlacementTraceRecord.ROUTED) continue;
            int start = (r == recFrom) ? nodeFrom : 0;
            for (int n = start; n < m.nodes.size(); n++) {
                NodePos p = m.nodes.get(n);
                if (p.node == node && Math.abs(p.edgeDistM - synthM) <= STAMP_MATCH_TOL_M) {
                    return new int[]{r, n};
                }
            }
        }
        return null;
    }

    private static int nextGapRecord(List<RecordModel> models, int after) {
        for (int r = after + 1; r < models.size(); r++) {
            if (models.get(r).rec.type == RouteInstructionGenerator.PlacementTraceRecord.GAP) return r;
        }
        return -1;
    }

    // ================================================================== geometry helpers

    private static double[] cumulativeDistances(PointList pl) {
        double[] cum = new double[Math.max(1, pl.size())];
        cum[0] = 0;
        for (int i = 1; i < pl.size(); i++) {
            cum[i] = cum[i - 1] + DIST.calcDist(pl.getLat(i - 1), pl.getLon(i - 1), pl.getLat(i), pl.getLon(i));
        }
        return cum;
    }

    /**
     * The polyline index where a record's geometry effectively starts. appendRoutePolyline /
     * appendGapPolyline drop the record's first point when it duplicates the previous record's
     * last point — the record then physically starts one index earlier.
     */
    private static int effectiveSliceStart(RouteInstructionGenerator.PlacementTraceRecord rec, PointList polyline) {
        int idx = Math.min(rec.polyBefore, polyline.size() - 1);
        if (Double.isNaN(rec.snapStartLat) || idx >= polyline.size()) return Math.max(0, idx);
        double d = DIST.calcDist(rec.snapStartLat, rec.snapStartLon, polyline.getLat(idx), polyline.getLon(idx));
        if (d > 0.01 && rec.polyBefore > 0) return rec.polyBefore - 1;  // first point was deduped
        return idx;
    }

    /**
     * Closest-approach of a point onto a geometry index range [fromIdx, toIdx].
     * Returns {cumulative distance at the projection, perpendicular distance}.
     * Local planar approximation — exact at snap/junction scales.
     */
    private static double[] locateOnGeomRange(PointList geom, double[] geomCum, int fromIdx, int toIdx,
                                              double lat, double lon) {
        double bestPerp = Double.MAX_VALUE, bestDist = geomCum[Math.min(fromIdx, geomCum.length - 1)];
        double mLat = 111320.0;
        double mLon = 111320.0 * Math.cos(Math.toRadians(lat));
        for (int i = fromIdx; i < Math.min(toIdx, geom.size() - 1); i++) {
            double ax = (geom.getLon(i) - lon) * mLon, ay = (geom.getLat(i) - lat) * mLat;
            double bx = (geom.getLon(i + 1) - lon) * mLon, by = (geom.getLat(i + 1) - lat) * mLat;
            double dx = bx - ax, dy = by - ay;
            double len2 = dx * dx + dy * dy;
            double t = len2 < 1e-9 ? 0 : Math.max(0, Math.min(1, -(ax * dx + ay * dy) / len2));
            double px = ax + t * dx, py = ay + t * dy;
            double perp = Math.sqrt(px * px + py * py);
            if (perp < bestPerp) {
                bestPerp = perp;
                bestDist = geomCum[i] + t * (geomCum[i + 1] - geomCum[i]);
            }
        }
        if (fromIdx >= geom.size() - 1 && fromIdx < geom.size()) {  // degenerate single-point range
            bestPerp = DIST.calcDist(lat, lon, geom.getLat(fromIdx), geom.getLon(fromIdx));
            bestDist = geomCum[fromIdx];
        }
        return new double[]{bestDist, bestPerp};
    }

    /** Closest-approach onto a polyline index range; returns {distance from range start, perp}. */
    private static double[] locateOnPolylineRange(PointList polyline, int fromIdx, int toIdx,
                                                  double lat, double lon) {
        int from = Math.max(0, fromIdx - 1);  // include the possible dedup-shared point
        int to = Math.min(toIdx, polyline.size());
        PointList slice = new PointList(to - from, false);
        for (int i = from; i < to; i++) slice.add(polyline.getLat(i), polyline.getLon(i));
        double[] cum = cumulativeDistances(slice);
        return locateOnGeomRange(slice, cum, 0, slice.size() - 1, lat, lon);
    }

    /** Min perpendicular distance from a coordinate to the polyline within ±window of route distance s. */
    private static double perpNear(PointList polyline, double[] polyCum, double s,
                                   double lat, double lon, double windowM) {
        int lo = lowerBound(polyCum, s - windowM);
        int hi = lowerBound(polyCum, s + windowM);
        double[] r = locateOnGeomRange(polyline, polyCum, Math.max(0, lo - 1),
                Math.min(polyline.size() - 1, hi + 1), lat, lon);
        return r[1];
    }

    private static int lowerBound(double[] cum, double v) {
        int lo = 0, hi = cum.length - 1;
        while (lo < hi) {
            int mid = (lo + hi) >>> 1;
            if (cum[mid] < v) lo = mid + 1;
            else hi = mid;
        }
        return lo;
    }

    private static int clampIdx(int idx, int len) {
        return Math.max(0, Math.min(idx, len - 1));
    }

    private static String appendNote(String note, String add) {
        return note.isEmpty() ? add : note + "; " + add;
    }

    private static String abbreviate(List<Integer> ids) {
        if (ids.size() <= 6) return ids.toString();
        return "[" + ids.get(0) + ", " + ids.get(1) + ", ... " + ids.get(ids.size() - 1)
                + "] (" + ids.size() + " edges)";
    }

    static String signName(Instruction instr) {
        switch (instr.getSign()) {
            case Instruction.U_TURN_UNKNOWN: return "U_TURN_UNKNOWN";
            case Instruction.U_TURN_LEFT: return "U_TURN_LEFT";
            case Instruction.KEEP_LEFT: return "KEEP_LEFT";
            case Instruction.TURN_SHARP_LEFT: return "TURN_SHARP_LEFT";
            case Instruction.TURN_LEFT: return "TURN_LEFT";
            case Instruction.TURN_SLIGHT_LEFT: return "TURN_SLIGHT_LEFT";
            case Instruction.CONTINUE_ON_STREET: return "CONTINUE";
            case Instruction.TURN_SLIGHT_RIGHT: return "TURN_SLIGHT_RIGHT";
            case Instruction.TURN_RIGHT: return "TURN_RIGHT";
            case Instruction.TURN_SHARP_RIGHT: return "TURN_SHARP_RIGHT";
            case Instruction.FINISH: return "FINISH";
            case Instruction.USE_ROUNDABOUT: return "USE_ROUNDABOUT";
            case Instruction.KEEP_RIGHT: return "KEEP_RIGHT";
            case Instruction.U_TURN_RIGHT: return "U_TURN_RIGHT";
            default: return "SIGN_" + instr.getSign();
        }
    }
}
