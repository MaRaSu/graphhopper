package com.graphhopper.trailmap.convert;

import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.ResponsePath;
import com.graphhopper.matching.EdgeMatch;
import com.graphhopper.matching.Observation;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.AngleCalc;
import com.graphhopper.util.CustomModel;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.PointList;
import com.graphhopper.util.details.PathDetail;
import com.graphhopper.util.shapes.GHPoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Stage 3 of the conversion pipeline. Given one {@link TrackRegion.Matched}, choose the
 * minimum waypoint subset such that calling {@code /route} between consecutive chosen
 * waypoints reproduces the user's path for the region.
 *
 * <h2>Probe verdict</h2>
 *
 * <p>For each probe from {@code candidate[i]} to {@code candidate[j]}:
 * <ol>
 *   <li>Ask the per-region {@link RegionReference} whether the range contains any
 *       <i>geographically bad</i> leg (a leg where {@code /route}'s chosen path is far
 *       from the user's actual GPS line). If yes, the probe FAILS — forcing escalation
 *       to a coordinates segment for that leg.</li>
 *   <li>Otherwise, build the expected edge_key sequence {@code E} from the reference,
 *       call {@code /route(snap[i], snap[j])} for the actual sequence {@code A}, and
 *       apply:
 *       <b>PASS iff {@code A == E[x..len(E)-y]} for some {@code x, y ∈ {0,1}}.</b>
 *       The boundary tolerance handles the snap-at-junction case.</li>
 * </ol>
 *
 * <h2>Reference choice — {@link ReferenceMode}</h2>
 *
 * <ul>
 *   <li>{@link ReferenceMode#ROUTE_GROUND_TRUTH_WITH_GEOMETRY_FALLBACK} (default).
 *       Uses {@code /route(all_snaps_in_region)} as the structural reference AND
 *       checks per-leg geographic agreement between {@code /route}'s polyline and the
 *       original GPS observations. Catches the case where matcher and {@code /route}
 *       agree on "some path exists" but disagree on WHICH path — typically when the
 *       matcher (matching_profile) snaps to an OSM way the rendering profile cannot
 *       traverse, so {@code /route} picks a geographically distant parallel.</li>
 *   <li>{@link ReferenceMode#ROUTE_GROUND_TRUTH}. Same structural reference, no
 *       geographic fallback. Use when GPS noise is high or geometry-deviation
 *       thresholding is unwanted.</li>
 *   <li>{@link ReferenceMode#MATCHER_EDGES}. Original behavior: matcher's edges as
 *       reference. Fails on profile-driven micro-edge differences at structurally
 *       complex graph nodes (crossings, lane splits) even when the rendered path is
 *       visibly identical.</li>
 * </ul>
 *
 * <p>All three strategies share the same search loop, escalation behavior, and probe
 * rule. Only the source of the "expected" sequence and the geographic-validity check
 * differ.
 *
 * <h2>Search</h2>
 *
 * <p>Exponential extension + binary refinement per region. When the smallest possible
 * probe ({@code cursor → cursor+1}) fails, the leg is emitted as a coordinates segment
 * (see {@link Result#legIsCoords()}).
 */
public class RoutedRegionOptimizer {

    private static final Logger LOGGER = LoggerFactory.getLogger(RoutedRegionOptimizer.class);

    /** Per-cursor probe guard — the real anti-thrash limit. A single cursor's exponential+binary
     *  search needs only O(log span) probes (~28 even for a ~9.5k-candidate region). This caps a
     *  pathological cursor; on hit the search commits best-so-far (or escalates one coords step) and
     *  CONTINUES, so the region always runs to its end. The OUTER loop is progress-bounded (cursor
     *  strictly increases), so this cannot run away — replacing the old per-region total cap that
     *  aborted legitimately-large matched regions mid-way (tail-drop → 44 km chord). */
    private static final int MAX_PROBES_PER_CURSOR = 64;

    /** Generous whole-region backstop (paranoia only — the progress-bounded outer loop cannot run
     *  away). On hit, the remaining tail is emitted as a faithful coordinates leg, NEVER dropped, so
     *  a budget hit degrades to "raw track drawn", never a straight chord across the map. */
    private static final int MAX_PROBES_PER_REGION = 50_000;

    /**
     * For a U-turn apex pair to be treated as a real user turn (and force a waypoint),
     * the apex edge must appear EXACTLY this many times in the matcher's slice — once
     * forward, once reverse. A higher count indicates HMM direction-flip oscillation
     * (Viterbi alternating between virtual edge directions over a short, possibly
     * noisy GPS span) and is suppressed.
     *
     * <p>Empirically: out-n-back's real U-turn has edge appearing exactly 2 times
     * (clean fwd→rev pair) → forced; suora Region 3 has the apex edge appearing 3
     * times (rev→fwd→rev oscillation) → suppressed.
     */
    private static final int UTURN_CLEAN_PAIR_COUNT = 2;

    /**
     * Default per-leg maximum allowed distance (m) from any GPS observation in a leg's
     * range to the {@code /route} polyline for that leg. Beyond this, the leg is
     * "geographically bad" → escalate to coordinates rather than render a routed
     * segment far from where the user actually walked.
     *
     * <p>20 m sits well above typical GPS noise (~5–10 m) and snap-to-edge distance,
     * while small enough to catch real disagreements (e.g., footway user vs parallel
     * road 30–50 m away).
     */
    public static final double DEFAULT_MAX_LEG_DEVIATION_M = 20.0;

    /**
     * Soft start-heading penalty (seconds) applied to each probe {@code /route} call that
     * carries an inherited heading. Mirrors the client's heading-chain rendering: each
     * rendered leg's exit bearing becomes the next leg's start heading, so a probe must
     * validate the heading-constrained path the client will actually draw — not a plain
     * point-to-point path that may differ at the start junction.
     *
     * <p>The value MUST match whatever the client passes as {@code heading_penalty} when
     * it renders {@code /route} legs (cross-team dependency). Confirmed from the client's
     * actual request (2026-06-01): the client uses {@code heading_penalty=60} with
     * {@code headings=[exitBearing, null]}. A mismatch reintroduces the very divergence this
     * feature fixes (optimizer validates one path, client renders another).
     */
    public static final int CLIENT_HEADING_PENALTY_S = 60;

    private static final AngleCalc ANGLE_CALC = AngleCalc.ANGLE_CALC;

    /**
     * Which reference the probe rule compares {@code /route}'s output against. See
     * class-level javadoc.
     *
     * <p>{@link #MATCHER_EDGES} is the production default — paired with the node-pair
     * tolerance, it preserves the matcher's globally-optimized Viterbi sequence as the
     * reference, restoring the matcher's value on noisy GPS while still handling
     * profile-mismatch micro-edge differences at junctions structurally.
     *
     * <p>{@link #ROUTE_GROUND_TRUTH} and {@link #ROUTE_GROUND_TRUTH_WITH_GEOMETRY_FALLBACK}
     * are <b>legacy/diagnostic modes</b> from an earlier iteration that used
     * {@code /route(all_snaps)} as the structural reference instead of the matcher.
     * That approach abandoned the Viterbi sequence on noisy tracks (visibly worse on
     * dense trail networks with Garmin-class GPS) and is no longer the default. The
     * geometry fallback in particular was the safety net for the
     * {@code /route}-as-reference blind spot (where both sides of the comparison agreed
     * on a path far from the user's GPS line). With {@code MATCHER_EDGES} the matcher's
     * Viterbi already constrains the reference to stay close to GPS, so the geometry
     * fallback is redundant. Kept as available {@code ReferenceMode} values for
     * comparison/research scenarios but not used in production.
     */
    public enum ReferenceMode {
        MATCHER_EDGES,
        ROUTE_GROUND_TRUTH,
        ROUTE_GROUND_TRUTH_WITH_GEOMETRY_FALLBACK
    }

    public static final ReferenceMode DEFAULT_REFERENCE_MODE =
            ReferenceMode.MATCHER_EDGES;

    private static final DistanceCalcEarth DIST = DistanceCalcEarth.DIST_EARTH;

    private final GraphHopper graphHopper;
    private final ReferenceMode referenceMode;
    private final double maxLegDeviationM;

    /** Twin-edge tolerance (fallback only): when a leg would otherwise be demoted to coords,
     *  treat coincident parallel edges over the same node pair (e.g. a cycleway + footway mapped
     *  as separate ways over the same stripe) as equivalent before deciding. Default on. */
    private boolean twinEdgeTolerance = true;
    /** Two edges sharing a node pair are twins only if the longer is ≤ {@link #TWIN_RATIO_MAX}×
     *  the shorter — guards against a genuine alternate path (a longer "route" between the same
     *  two junctions) being mistaken for a coincident twin. */
    static final double TWIN_RATIO_MAX = 1.5;
    /** Below this length the ratio test is skipped (very short edges can't deviate enough to
     *  matter, and the ratio is noisy on them) — they count as twins on node-pair alone. */
    static final double TWIN_MIN_LEN_M = 20.0;
    /** Cache: graph edge id → canonical (min) edge id of its geometry-guarded twin group. */
    private final Map<Integer, Integer> twinCanonCache = new HashMap<>();

    /** Toggle the twin-edge tolerance fallback (default on). */
    public void setTwinEdgeTolerance(boolean enabled) { this.twinEdgeTolerance = enabled; }

    public RoutedRegionOptimizer(GraphHopper graphHopper) {
        this(graphHopper, DEFAULT_REFERENCE_MODE, DEFAULT_MAX_LEG_DEVIATION_M);
    }

    public RoutedRegionOptimizer(GraphHopper graphHopper, ReferenceMode referenceMode) {
        this(graphHopper, referenceMode, DEFAULT_MAX_LEG_DEVIATION_M);
    }

    public RoutedRegionOptimizer(GraphHopper graphHopper,
                                 ReferenceMode referenceMode,
                                 double maxLegDeviationM) {
        this.graphHopper = graphHopper;
        this.referenceMode = referenceMode;
        this.maxLegDeviationM = maxLegDeviationM;
    }

    public ReferenceMode getReferenceMode() { return referenceMode; }

    /**
     * Result of optimization. Each leg is either a routed leg (rendered by the client
     * via {@code /route} between {@code waypoints[i]} and {@code waypoints[i+1]}) or a
     * coordinates leg (raw GPX from {@code waypointObsIndices[i]} to
     * {@code waypointObsIndices[i+1]}).
     */
    public record Result(
            List<GHPoint> waypoints,
            List<Integer> waypointObsIndices,
            List<Double> legDistancesM,
            List<Boolean> legIsCoords,
            /** Per-leg start heading the optimizer VALIDATED the leg with (the /route exit
             *  bearing inherited from the previous accepted leg), or null for the first leg
             *  of the region and the first leg after a coords escalation. This is exactly the
             *  heading the client must pass so its /route reproduces the validated path. */
            List<Double> legInitialHeadings
    ) {
        public double totalDistanceM() {
            return legDistancesM.stream().mapToDouble(Double::doubleValue).sum();
        }
    }

    public Result optimize(TrackRegion.Matched region,
                           List<Observation> observations,
                           String profile, CustomModel customModel) {
        List<GHPoint> candidates = region.obsSnapPoints();
        List<Integer> candObs = region.matchedObsIndices();

        if (candidates.size() < 2 || region.edgeMatches().isEmpty()) {
            return new Result(
                    List.of(region.startSnap(), region.endSnap()),
                    List.of(region.firstObservation(), region.lastObservation()),
                    List.of(region.matchedLengthM()),
                    List.of(false),
                    Arrays.asList((Double) null)); // single leg, no inherited heading
        }

        RegionReference ref = buildReference(region, observations, profile, customModel);

        // Node-pair tolerances (only for MATCHER_EDGES mode). At region start, compare
        // matcher's full edge_key sequence against /route(all_snaps)'s sequence. Where
        // they disagree but the disagreeing middles share the same entry+exit graph
        // nodes, they're micro-alternatives at a junction (e.g., parallel footway/
        // cycleway edges with identical geometry, different OSM tags). Record those as
        // substitutions to apply during probe-time comparison.
        List<TolerancePair> tolerances = (referenceMode == ReferenceMode.MATCHER_EDGES)
                ? computeNodePairTolerances(region, profile, customModel)
                : List.of();

        // Forced waypoint candidates (sorted ascending). These are observations the
        // search loop MUST commit as waypoints — currently only U-turn apex points,
        // identified from back-traversal pairs in the matcher's slice. The exponential
        // extension caps at the next forced candidate so the search can't probe past
        // it without committing it as a waypoint first.
        int[] forcedCands = findUTurnForcedCandidates(region);
        if (forcedCands.length > 0 && LOGGER.isDebugEnabled()) {
            StringBuilder sb = new StringBuilder();
            for (int k : forcedCands) {
                if (sb.length() > 0) sb.append(',');
                sb.append("cand[").append(k).append("]=obs[").append(candObs.get(k)).append("]");
            }
            LOGGER.debug("optimize: forced U-turn waypoints in region [{}..{}]: {}",
                    region.firstObservation(), region.lastObservation(), sb);
        }

        List<GHPoint> chosenSnaps = new ArrayList<>();
        List<Integer> chosenObs = new ArrayList<>();
        List<Double> legDists = new ArrayList<>();
        List<Boolean> legCoords = new ArrayList<>();
        List<Double> legHeadings = new ArrayList<>();
        chosenSnaps.add(candidates.get(0));
        chosenObs.add(candObs.get(0));

        int cursor = 0;
        final int lastCand = candidates.size() - 1;
        int probes = 0;

        // Heading chain. At region start there is no inherited heading (matches today's
        // headingless first probe exactly). After each accepted routed leg, currentHeading
        // becomes that leg's /route exit bearing — the same bearing the client carries
        // into the next leg's /route call. Reset to null across any coords escalation:
        // a fresh routed segment after a raw-coords stretch has no inherited heading
        // (mirrors RouteInstructionGenerator resetting heading at non-routable gaps).
        Double currentHeading = null;

        while (cursor < lastCand) {
            int step = 1;
            int lastOk = -1;
            double lastOkDist = 0;
            double lastOkExitHeading = Double.NaN;
            int lastTried = cursor;
            int probesThisCursor = 0; // per-cursor thrash guard (resets each advance)

            // Cap target at the next forced waypoint (if any) so the search can't
            // probe past it.
            int forcedCap = nextForcedAfter(forcedCands, cursor);
            int searchLimit = (forcedCap >= 0) ? Math.min(lastCand, forcedCap) : lastCand;

            while (true) {
                int probeIdx = Math.min(cursor + step, searchLimit);
                if (probeIdx == lastTried && probeIdx != cursor + step) break;
                lastTried = probeIdx;
                ProbeOutcome po = probe(candidates, ref, tolerances, cursor, probeIdx,
                        profile, customModel, currentHeading);
                probes++;
                probesThisCursor++;
                if (po.pass) {
                    lastOk = probeIdx;
                    lastOkDist = po.distanceM;
                    lastOkExitHeading = po.exitHeading;
                    if (probeIdx == searchLimit) break;
                    if (cursor + step >= searchLimit) break;
                    step *= 2;
                } else {
                    break;
                }
                if (probesThisCursor >= MAX_PROBES_PER_CURSOR) break;
            }

            if (lastOk < 0) {
                int next = cursor + 1;
                chosenSnaps.add(candidates.get(next));
                chosenObs.add(candObs.get(next));
                legDists.add(0.0);
                legCoords.add(true);
                legHeadings.add(null); // coords leg carries no heading
                LOGGER.debug("optimize: escalate to COORDS at cand[{}]→cand[{}] (obs[{}]→obs[{}])",
                        cursor, next, candObs.get(cursor), candObs.get(next));
                cursor = next;
                // Coords gap breaks the heading chain — the next routed leg starts fresh.
                currentHeading = null;
                continue;
            }

            int best = lastOk;
            double bestDist = lastOkDist;
            double bestExitHeading = lastOkExitHeading;
            if (lastTried > lastOk) {
                int lo = lastOk + 1;
                int hi = lastTried;
                while (lo <= hi) {
                    int mid = (lo + hi) / 2;
                    ProbeOutcome po = probe(candidates, ref, tolerances, cursor, mid,
                            profile, customModel, currentHeading);
                    probes++;
                    probesThisCursor++;
                    if (po.pass) {
                        best = mid;
                        bestDist = po.distanceM;
                        bestExitHeading = po.exitHeading;
                        lo = mid + 1;
                    } else {
                        hi = mid - 1;
                    }
                    if (probesThisCursor >= MAX_PROBES_PER_CURSOR) break;
                }
            }

            chosenSnaps.add(candidates.get(best));
            chosenObs.add(candObs.get(best));
            legDists.add(bestDist);
            legCoords.add(false);
            // The heading this leg was VALIDATED with is the current inherited heading
            // (null for the region's first leg / first after a coords gap) — record it
            // BEFORE updating currentHeading to this leg's exit bearing.
            legHeadings.add(currentHeading);
            cursor = best;
            // Chain the accepted leg's exit bearing into the next probe's start heading,
            // exactly as the client does between rendered legs.
            currentHeading = Double.isNaN(bestExitHeading) ? null : bestExitHeading;

            if (probes >= MAX_PROBES_PER_REGION) {
                // Paranoia backstop. Emit the UNoptimized remainder as a faithful coordinates leg
                // (raw GPX to the region end) rather than dropping it — degrade to "raw track
                // drawn", never a straight chord. Should not trigger given the per-cursor guard.
                LOGGER.warn("optimize: hit region probe backstop {} for region [{}..{}]; emitting "
                        + "remaining tail obs[{}..{}] as coordinates (faithful, no chord)",
                        MAX_PROBES_PER_REGION, region.firstObservation(), region.lastObservation(),
                        candObs.get(cursor), candObs.get(lastCand));
                chosenSnaps.add(candidates.get(lastCand));
                chosenObs.add(candObs.get(lastCand));
                legDists.add(0.0);
                legCoords.add(true);
                legHeadings.add(null);
                cursor = lastCand;
                break;
            }
        }

        LOGGER.debug("optimize[{}]: region [{}..{}] {} candidates → {} waypoints ({} routed + {} coords) in {} probes",
                referenceMode, region.firstObservation(), region.lastObservation(),
                candidates.size(), chosenSnaps.size(),
                (int) legCoords.stream().filter(b -> !b).count(),
                (int) legCoords.stream().filter(b -> b).count(),
                probes);
        return new Result(chosenSnaps, chosenObs, legDists, legCoords, legHeadings);
    }

    private RegionReference buildReference(TrackRegion.Matched region,
                                           List<Observation> observations,
                                           String profile, CustomModel customModel) {
        if (referenceMode == ReferenceMode.MATCHER_EDGES) {
            return new MatcherEdgesReference(region);
        }
        // ROUTE_GROUND_TRUTH or ROUTE_GROUND_TRUTH_WITH_GEOMETRY_FALLBACK.
        // The fallback variant is the same class with the geo check enabled.
        double geoThreshold = (referenceMode == ReferenceMode.ROUTE_GROUND_TRUTH_WITH_GEOMETRY_FALLBACK)
                ? maxLegDeviationM : 0.0;
        try {
            return new RouteGroundTruthReference(graphHopper, region, observations,
                    profile, customModel, geoThreshold);
        } catch (RuntimeException e) {
            LOGGER.warn("RouteGroundTruth reference build failed for region [{}..{}]; "
                            + "falling back to matcher reference: {}",
                    region.firstObservation(), region.lastObservation(), e.getMessage());
            return new MatcherEdgesReference(region);
        }
    }

    private record ProbeOutcome(boolean pass, double distanceM, double exitHeading) {}

    /**
     * Probe a {@code /route} from {@code candidates[start]} to {@code candidates[end]}.
     *
     * @param inHeading inherited start heading (exit bearing of the already-accepted route
     *                  up to {@code start}), or {@code null} for the first leg of a region /
     *                  the first leg after a coords gap. When non-null, it is applied as a
     *                  soft start heading + {@link #CLIENT_HEADING_PENALTY_S} penalty so the
     *                  verdict reflects the heading-constrained path the client will render.
     */
    private ProbeOutcome probe(List<GHPoint> candidates,
                               RegionReference ref,
                               List<TolerancePair> tolerances,
                               int start, int end,
                               String profile, CustomModel customModel,
                               Double inHeading) {
        // Geographic-disagreement check FIRST: if any leg in the probe's range is
        // marked bad, force fail so the optimizer narrows down and escalates that leg.
        if (ref.isBadLeg(start, end)) {
            return new ProbeOutcome(false, 0, Double.NaN);
        }

        GHPoint pStart = candidates.get(start);
        GHPoint pEnd = candidates.get(end);

        int[] E = ref.expected(start, end);

        GHRequest req = new GHRequest(pStart, pEnd);
        req.setProfile(profile);
        if (customModel != null) req.setCustomModel(customModel);
        req.setPathDetails(List.of("edge_key"));
        req.putHint("instructions", false);
        req.putHint("calc_points", true);

        // Soft start-heading lever (same as the client's per-leg rendering and the TbT
        // RouteInstructionGenerator): heading on the start point, NaN on the end point,
        // plus the heading penalty. Omitted entirely when there is no inherited heading,
        // so the first-leg path is byte-for-byte identical to today's behaviour.
        if (inHeading != null && !inHeading.isNaN()) {
            req.setHeadings(Arrays.asList(inHeading, Double.NaN));
            req.putHint("heading_penalty", CLIENT_HEADING_PENALTY_S);
        }

        GHResponse rsp;
        try {
            rsp = graphHopper.route(req);
        } catch (Exception e) {
            return new ProbeOutcome(false, 0, Double.NaN);
        }
        if (rsp.hasErrors()) return new ProbeOutcome(false, 0, Double.NaN);

        ResponsePath path = rsp.getBest();
        List<PathDetail> ekDetails = path.getPathDetails().get("edge_key");
        if (ekDetails == null) return new ProbeOutcome(false, 0, Double.NaN);

        int[] A = dedupConsecutive(pdValuesInt(ekDetails));
        double exitHeading = exitHeadingOf(path.getPoints());

        // Try the basic rule first.
        if (applyEdgeKeyRule(E, A)) {
            return new ProbeOutcome(true, path.getDistance(), exitHeading);
        }

        // If basic rule fails, try with node-pair tolerance substitutions applied to E.
        // The tolerances were precomputed at region start by comparing matcher's full
        // sequence vs /route(all_snaps)'s full sequence; if matcher's micro-edges at a
        // junction are substituted for /route's equivalents (same node pair), the
        // resulting sequence should match A under the basic rule.
        if (!tolerances.isEmpty()) {
            int[] E_substituted = applyTolerances(E, tolerances);
            if (!Arrays.equals(E, E_substituted) && applyEdgeKeyRule(E_substituted, A)) {
                return new ProbeOutcome(true, path.getDistance(), exitHeading);
            }
        }

        // Twin-edge tolerance — LAST-RESORT fallback, only reached when the basic rule and the
        // node-pair tolerance have already failed (i.e. this leg is otherwise about to be demoted
        // to coords). Coincident parallel edges over the same node pair (cycleway + footway over
        // the same stripe) make /route pick the sibling twin of a matcher edge, so the edge_key
        // sequences differ even though the physical path is identical. Canonicalize BOTH sequences
        // to a per-node-pair (unordered, geometry-guarded) twin representative and re-test. This
        // can only flip a would-be-coords leg to routed; it never changes a passing leg or the
        // chosen edges.
        if (twinEdgeTolerance) {
            int[] Ec = twinCanonicalize(E);
            int[] Ac = twinCanonicalize(A);
            if (applyEdgeKeyRule(Ec, Ac)) {
                if (LOGGER.isDebugEnabled()) {
                    LOGGER.debug("optimize: TWIN-RESCUE leg cand[{}..{}] E={} A={} → Ec={} Ac={}",
                            start, end, Arrays.toString(E), Arrays.toString(A),
                            Arrays.toString(Ec), Arrays.toString(Ac));
                }
                return new ProbeOutcome(true, path.getDistance(), exitHeading);
            }
        }

        return new ProbeOutcome(false, 0, Double.NaN);
    }

    /**
     * Map an edge_key sequence to DIRECTION-PRESERVING twin-group symbols, then collapse consecutive
     * duplicates. Two edges belong to the same group when they share a node pair AND pass the length
     * guard ({@link #TWIN_MIN_LEN_M} / {@link #TWIN_RATIO_MAX}); the symbol is the group's min edge id
     * times two plus a direction bit. So a cycleway and its footway twin traversed the SAME way map to
     * one symbol (the parallel-twin fix), while a same-edge reversal (a genuine U-turn / out-and-back
     * in the matcher path) keeps two distinct symbols and is NOT flattened.
     */
    private int[] twinCanonicalize(int[] edgeKeys) {
        int[] out = new int[edgeKeys.length];
        for (int i = 0; i < edgeKeys.length; i++) {
            out[i] = twinCanonical(edgeKeys[i]);
        }
        return dedupConsecutive(out);
    }

    /** Direction-preserving twin symbol for {@code edgeKey}: {@code groupRep*2 + directionBit}, where
     *  {@code groupRep} is the geometry-guarded twin group's min edge id and the bit records traversal
     *  toward the higher- vs lower-numbered node. Same-direction twins collapse; reversals do not. */
    private int twinCanonical(int edgeKey) {
        Graph graph = graphHopper.getBaseGraph();
        EdgeIteratorState st;
        try {
            st = graph.getEdgeIteratorStateForKey(edgeKey);
        } catch (Exception e) {
            return edgeKey; // unresolvable — keep distinct
        }
        int rep = twinGroupRep(st.getEdge(), st.getBaseNode(), st.getAdjNode(), st.getDistance());
        int dirBit = st.getBaseNode() < st.getAdjNode() ? 0 : 1;
        return rep * 2 + dirBit;
    }

    /** Min edge id of the geometry-guarded twin group (edges between the same node pair as the given
     *  edge that pass {@link #isTwin}). Direction-independent → cached by edge id. */
    private int twinGroupRep(int edgeId, int base, int adj, double len) {
        Integer cached = twinCanonCache.get(edgeId);
        if (cached != null) return cached;
        int rep = edgeId;
        EdgeExplorer explorer = graphHopper.getBaseGraph().createEdgeExplorer();
        EdgeIterator it = explorer.setBaseNode(base);
        while (it.next()) {
            if (it.getAdjNode() != adj) continue;          // only parallels between base↔adj
            if (it.getEdge() == edgeId) continue;          // skip self
            if (!isTwin(len, it.getDistance())) continue;  // geometry guard
            if (it.getEdge() < rep) rep = it.getEdge();
        }
        twinCanonCache.put(edgeId, rep);
        return rep;
    }

    /** Length guard for twin equivalence: very short edges (≤ {@link #TWIN_MIN_LEN_M}) are twins on
     *  node-pair alone; otherwise the longer must be ≤ {@link #TWIN_RATIO_MAX}× the shorter. */
    static boolean isTwin(double lenA, double lenB) {
        double hi = Math.max(lenA, lenB), lo = Math.min(lenA, lenB);
        if (hi <= TWIN_MIN_LEN_M) return true;
        return lo > 0 && hi <= TWIN_RATIO_MAX * lo;
    }

    /**
     * Exit bearing of a route polyline — the azimuth of its final segment, mirroring how
     * the client (and {@code RouteInstructionGenerator}) derive the next leg's start
     * heading. Returns {@code NaN} when the polyline has fewer than two points.
     */
    private static double exitHeadingOf(PointList pts) {
        int n = pts.size();
        if (n < 2) return Double.NaN;
        return ANGLE_CALC.calcAzimuth(
                pts.getLat(n - 2), pts.getLon(n - 2),
                pts.getLat(n - 1), pts.getLon(n - 1));
    }

    static boolean applyEdgeKeyRule(int[] E, int[] A) {
        if (Arrays.equals(E, A)) return true;
        for (int x = 0; x <= 1; x++) {
            for (int y = 0; y <= 1; y++) {
                if (x == 0 && y == 0) continue;
                int remaining = E.length - x - y;
                if (remaining < 0) continue;
                if (remaining == 0) {
                    if (A.length == 0) return true;
                    continue;
                }
                int[] sub = Arrays.copyOfRange(E, x, E.length - y);
                if (Arrays.equals(sub, A)) return true;
            }
        }
        return false;
    }

    // ------------------------------------------------------------------------
    // Node-pair tolerance: micro-alternative at junction detection
    // ------------------------------------------------------------------------

    /**
     * Records a tolerated divergence between matcher's edges and /route's edges at a
     * junction. Both sequences are guaranteed to span the same pair of graph nodes
     * (same entry, same exit). At probe time, if {@code matcherKeys} appears as a
     * contiguous subsequence in the probe's expected E, it is substituted with
     * {@code routeKeys} before comparing to /route's actual A.
     */
    private record TolerancePair(int[] matcherKeys, int[] routeKeys) {}

    /**
     * At region start, compare matcher's full edge_key sequence against /route(all
     * region snaps)'s sequence. Find divergent middles after boundary stripping, and
     * for each one, check whether matcher's middle and /route's middle span the same
     * pair of graph nodes (entry node + exit node). If yes, record a tolerance pair.
     */
    private List<TolerancePair> computeNodePairTolerances(TrackRegion.Matched region,
                                                          String profile,
                                                          CustomModel customModel) {
        List<EdgeMatch> slice = region.edgeMatches();
        if (slice.isEmpty()) return List.of();

        // Matcher's full E from the slice (already dedup-by-edge_key in the segmenter).
        int[] E_full = new int[slice.size()];
        for (int i = 0; i < slice.size(); i++) {
            E_full[i] = slice.get(i).getEdgeState().getEdgeKey();
        }
        int[] E_dedup = dedupConsecutive(E_full); // typically no-op given segmenter dedup

        // /route(all_snaps_in_region).
        List<GHPoint> snaps = region.obsSnapPoints();
        if (snaps.size() < 2) return List.of();
        GHRequest req = new GHRequest(snaps);
        req.setProfile(profile);
        if (customModel != null) req.setCustomModel(customModel);
        req.setPathDetails(List.of("edge_key"));
        req.putHint("instructions", false);
        req.putHint("calc_points", true);
        GHResponse rsp;
        try {
            rsp = graphHopper.route(req);
        } catch (Exception e) {
            return List.of();
        }
        if (rsp.hasErrors()) return List.of();
        ResponsePath path = rsp.getBest();
        List<PathDetail> ek = path.getPathDetails().get("edge_key");
        if (ek == null) return List.of();
        int[] A_dedup = dedupConsecutive(pdValuesInt(ek));

        // Try boundary stripping (x, y ∈ {0,1}) before finding the divergent middle.
        // Snap-at-region-boundary cases (matcher's leading or trailing edge absent from
        // /route's output because the snap is at a junction) are absorbed here, leaving
        // a clean structural comparison for what remains.
        Graph graph = graphHopper.getBaseGraph();
        for (int x = 0; x <= 1; x++) {
            for (int y = 0; y <= 1; y++) {
                if (E_dedup.length - x - y < 1) continue;
                int[] E_trunc = Arrays.copyOfRange(E_dedup, x, E_dedup.length - y);
                int prefixLen = 0;
                while (prefixLen < E_trunc.length && prefixLen < A_dedup.length
                        && E_trunc[prefixLen] == A_dedup[prefixLen]) {
                    prefixLen++;
                }
                int suffixLen = 0;
                while (suffixLen < E_trunc.length - prefixLen
                        && suffixLen < A_dedup.length - prefixLen
                        && E_trunc[E_trunc.length - 1 - suffixLen]
                                == A_dedup[A_dedup.length - 1 - suffixLen]) {
                    suffixLen++;
                }
                int eMidLen = E_trunc.length - prefixLen - suffixLen;
                int aMidLen = A_dedup.length - prefixLen - suffixLen;
                if (eMidLen == 0 || aMidLen == 0) continue;
                int[] eMid = Arrays.copyOfRange(E_trunc, prefixLen, E_trunc.length - suffixLen);
                int[] aMid = Arrays.copyOfRange(A_dedup, prefixLen, A_dedup.length - suffixLen);

                try {
                    EdgeIteratorState eFirst = graph.getEdgeIteratorStateForKey(eMid[0]);
                    EdgeIteratorState eLast = graph.getEdgeIteratorStateForKey(eMid[eMid.length - 1]);
                    EdgeIteratorState aFirst = graph.getEdgeIteratorStateForKey(aMid[0]);
                    EdgeIteratorState aLast = graph.getEdgeIteratorStateForKey(aMid[aMid.length - 1]);
                    if (eFirst.getBaseNode() == aFirst.getBaseNode()
                            && eLast.getAdjNode() == aLast.getAdjNode()) {
                        LOGGER.debug("optimize: node-pair tolerance for region [{}..{}]: "
                                        + "matcher={} → /route={} (x={},y={})",
                                region.firstObservation(), region.lastObservation(),
                                Arrays.toString(eMid), Arrays.toString(aMid), x, y);
                        return List.of(new TolerancePair(eMid, aMid));
                    }
                } catch (Exception ignored) {
                    // Skip lookups that fail for any reason.
                }
            }
        }
        return List.of();
    }

    /**
     * Substitute occurrences of any {@link TolerancePair#matcherKeys()} subsequence in
     * {@code E} with the corresponding {@link TolerancePair#routeKeys()}. Returns the
     * (possibly transformed) sequence dedup'd of consecutive duplicates.
     */
    private static int[] applyTolerances(int[] E, List<TolerancePair> tolerances) {
        if (tolerances.isEmpty()) return E;
        List<Integer> result = new ArrayList<>(E.length);
        int i = 0;
        while (i < E.length) {
            boolean substituted = false;
            for (TolerancePair tp : tolerances) {
                if (matchesAt(E, i, tp.matcherKeys())) {
                    for (int k : tp.routeKeys()) result.add(k);
                    i += tp.matcherKeys().length;
                    substituted = true;
                    break;
                }
            }
            if (!substituted) {
                result.add(E[i]);
                i++;
            }
        }
        int[] out = new int[result.size()];
        for (int j = 0; j < out.length; j++) out[j] = result.get(j);
        return dedupConsecutive(out);
    }

    private static boolean matchesAt(int[] arr, int start, int[] pattern) {
        if (start + pattern.length > arr.length) return false;
        for (int k = 0; k < pattern.length; k++) {
            if (arr[start + k] != pattern[k]) return false;
        }
        return true;
    }

    // ------------------------------------------------------------------------
    // Forced-waypoint detection: U-turn apex preservation
    // ------------------------------------------------------------------------

    /**
     * Scan the matcher's slice for U-turn apex patterns and return the sorted set of
     * candidate indices to force as waypoints.
     *
     * <p>A U-turn pair is two adjacent slice entries with the same undirected edge id
     * but flipped direction (their edge_keys differ by exactly 1). When the user
     * actually turned around on a single physical edge, the matcher's pre-dedup
     * sequence has [..., K+, K-, ...]; with the segmenter's edge_key dedup preserved,
     * both entries survive in the slice.
     *
     * <p>To avoid forcing waypoints on Viterbi direction-flip noise (matcher uncertain
     * about direction across a short, possibly noisy GPS span — pattern: edge appears
     * 3+ times in slice with alternating directions), only force when the apex edge
     * appears in the slice EXACTLY {@link #UTURN_CLEAN_PAIR_COUNT} times (clean
     * forward-then-reverse traversal). Additionally require at least one candidate on
     * each side of the pair so the U-turn is observation-supported. The forced
     * candidate is the FIRST candidate on the post-turn side (semantically "the
     * observation where the user has turned and is now heading the new direction").
     */
    private static int[] findUTurnForcedCandidates(TrackRegion.Matched region) {
        List<EdgeMatch> slice = region.edgeMatches();
        int[] candEdgeIdx = region.obsEdgeIdxInSlice();
        if (slice.size() < 2 || candEdgeIdx.length == 0) return new int[0];

        List<Integer> forced = new ArrayList<>();
        for (int i = 0; i + 1 < slice.size(); i++) {
            int kA = slice.get(i).getEdgeState().getEdgeKey();
            int kB = slice.get(i + 1).getEdgeState().getEdgeKey();
            if ((kA ^ 1) != kB) continue; // not same edge, flipped direction

            // Structural cleanness check: the apex edge must appear in the slice
            // EXACTLY UTURN_CLEAN_PAIR_COUNT times (= 2). More occurrences mean the
            // matcher's Viterbi oscillated between virtual edge directions across a
            // short span — HMM artifact, not a real user U-turn.
            int edgeId = slice.get(i).getEdgeState().getEdge();
            int sliceEdgeCount = 0;
            for (EdgeMatch em : slice) {
                if (em.getEdgeState().getEdge() == edgeId) sliceEdgeCount++;
            }
            if (sliceEdgeCount != UTURN_CLEAN_PAIR_COUNT) continue;

            // Observation support: at least one candidate must be attributed to each
            // side of the pair.
            int firstHalfCount = 0;
            int secondHalfCount = 0;
            int firstAtSecondHalf = -1;
            int lastAtFirstHalf = -1;
            for (int k = 0; k < candEdgeIdx.length; k++) {
                if (candEdgeIdx[k] == i) {
                    firstHalfCount++;
                    lastAtFirstHalf = k;
                } else if (candEdgeIdx[k] == i + 1) {
                    secondHalfCount++;
                    if (firstAtSecondHalf < 0) firstAtSecondHalf = k;
                }
            }
            if (firstHalfCount == 0 || secondHalfCount == 0) {
                continue; // no observation supports one side — no waypoint forced
            }

            int chosen = (firstAtSecondHalf >= 0) ? firstAtSecondHalf : lastAtFirstHalf;
            if (chosen >= 0 && !forced.contains(chosen)) {
                forced.add(chosen);
            }
        }
        // Already in ascending order since we iterate slice positions ascending and
        // first-at-second-half is monotonically non-decreasing across pairs.
        int[] out = new int[forced.size()];
        for (int i = 0; i < out.length; i++) out[i] = forced.get(i);
        return out;
    }

    /**
     * Smallest forced candidate index strictly greater than {@code cursor}, or -1 if
     * none.
     */
    private static int nextForcedAfter(int[] forcedCands, int cursor) {
        for (int f : forcedCands) {
            if (f > cursor) return f;
        }
        return -1;
    }

    // ------------------------------------------------------------------------
    // Reference strategies
    // ------------------------------------------------------------------------

    private interface RegionReference {
        /** Expected edge_key sequence (dedup'd) for a probe from {@code candStart} to {@code candEnd}. */
        int[] expected(int candStart, int candEnd);

        /**
         * True if any ground-truth leg in [{@code candStart}..{@code candEnd-1}] is
         * geographically bad — its {@code /route} polyline is too far from the user's
         * GPS observations to be a faithful render of the actual path.
         */
        default boolean isBadLeg(int candStart, int candEnd) { return false; }
    }

    private static final class MatcherEdgesReference implements RegionReference {
        private final List<EdgeMatch> slice;
        private final int[] candEdgeIdx;

        MatcherEdgesReference(TrackRegion.Matched region) {
            this.slice = region.edgeMatches();
            this.candEdgeIdx = region.obsEdgeIdxInSlice();
        }

        @Override
        public int[] expected(int candStart, int candEnd) {
            int eFrom = Math.min(candEdgeIdx[candStart], candEdgeIdx[candEnd]);
            int eTo = Math.max(candEdgeIdx[candStart], candEdgeIdx[candEnd]);
            int[] keys = new int[eTo - eFrom + 1];
            for (int i = eFrom; i <= eTo; i++) {
                keys[i - eFrom] = slice.get(i).getEdgeState().getEdgeKey();
            }
            return dedupConsecutive(keys);
        }
    }

    /**
     * OSRM-style ground truth: one {@code /route} call with ALL region snap points as
     * via-points using the rendering profile. The per-leg edge_keys are extracted from
     * the response.
     *
     * <p>When {@code geometryThresholdM > 0}, also performs a per-leg geographic check:
     * each leg's {@code /route} polyline must be within {@code geometryThresholdM} of
     * every GPS observation in the leg's obs-index range; otherwise the leg is marked
     * "bad" and {@link #isBadLeg(int, int)} returns true for any probe covering it.
     */
    private static final class RouteGroundTruthReference implements RegionReference {
        private final int[][] legKeys;
        private final boolean[] legIsBad;

        RouteGroundTruthReference(GraphHopper gh,
                                  TrackRegion.Matched region,
                                  List<Observation> observations,
                                  String profile, CustomModel customModel,
                                  double geometryThresholdM) {
            List<GHPoint> snaps = region.obsSnapPoints();
            GHRequest req = new GHRequest(snaps);
            req.setProfile(profile);
            if (customModel != null) req.setCustomModel(customModel);
            req.setPathDetails(List.of("edge_key"));
            req.putHint("instructions", false);
            req.putHint("calc_points", true);

            GHResponse rsp = gh.route(req);
            if (rsp.hasErrors()) {
                throw new RuntimeException("ground-truth /route hasErrors: " + rsp.getErrors());
            }
            ResponsePath path = rsp.getBest();
            List<Integer> wpIndices = path.getWaypointIndices();
            List<PathDetail> ek = path.getPathDetails().get("edge_key");
            if (wpIndices == null || wpIndices.size() < 2 || ek == null) {
                throw new RuntimeException("ground-truth /route missing waypointIndices or edge_key");
            }
            PointList polyline = path.getPoints();

            // Per-leg edge_keys.
            int legCount = wpIndices.size() - 1;
            this.legKeys = new int[legCount][];
            for (int leg = 0; leg < legCount; leg++) {
                int legStart = wpIndices.get(leg);
                int legEnd = wpIndices.get(leg + 1);
                List<Integer> keys = new ArrayList<>();
                for (PathDetail d : ek) {
                    int dFirst = d.getFirst();
                    int dLast = d.getLast();
                    if (dFirst < legEnd && dLast > legStart) {
                        keys.add(((Number) d.getValue()).intValue());
                    }
                }
                int[] arr = new int[keys.size()];
                for (int k = 0; k < keys.size(); k++) arr[k] = keys.get(k);
                legKeys[leg] = dedupConsecutive(arr);
            }

            // Per-leg geographic check (only when threshold > 0).
            this.legIsBad = new boolean[legCount];
            if (geometryThresholdM > 0 && observations != null) {
                List<Integer> matchedObs = region.matchedObsIndices();
                int candCount = matchedObs.size();
                int boundLegs = Math.min(legCount, candCount - 1);
                for (int leg = 0; leg < boundLegs; leg++) {
                    int legStart = wpIndices.get(leg);
                    int legEnd = wpIndices.get(leg + 1);
                    int obsStart = matchedObs.get(leg);
                    int obsEnd = matchedObs.get(leg + 1);
                    double maxDev = maxObsToLegPolylineDistance(observations, obsStart, obsEnd,
                            polyline, legStart, legEnd);
                    if (maxDev > geometryThresholdM) {
                        legIsBad[leg] = true;
                        LOGGER.debug("RouteGroundTruth: leg {} (obs[{}..{}]) BAD — max obs-to-polyline {}m > {}m",
                                leg, obsStart, obsEnd,
                                String.format("%.1f", maxDev),
                                String.format("%.1f", geometryThresholdM));
                    }
                }
            }
        }

        @Override
        public int[] expected(int candStart, int candEnd) {
            List<Integer> all = new ArrayList<>();
            for (int leg = candStart; leg < candEnd && leg < legKeys.length; leg++) {
                for (int k : legKeys[leg]) all.add(k);
            }
            int[] arr = new int[all.size()];
            for (int i = 0; i < arr.length; i++) arr[i] = all.get(i);
            return dedupConsecutive(arr);
        }

        @Override
        public boolean isBadLeg(int candStart, int candEnd) {
            for (int leg = candStart; leg < candEnd && leg < legIsBad.length; leg++) {
                if (legIsBad[leg]) return true;
            }
            return false;
        }
    }

    // ------------------------------------------------------------------------
    // Geometry helpers
    // ------------------------------------------------------------------------

    /**
     * Max distance from any observation in {@code obs[obsStart..obsEnd]} (inclusive)
     * to the polyline sub-range {@code polyline[polyStart..polyEnd]} (inclusive).
     */
    private static double maxObsToLegPolylineDistance(List<Observation> observations,
                                                      int obsStart, int obsEnd,
                                                      PointList polyline,
                                                      int polyStart, int polyEnd) {
        if (polyEnd <= polyStart) return 0;
        double maxDev = 0;
        for (int i = obsStart; i <= obsEnd; i++) {
            GHPoint p = observations.get(i).getPoint();
            double d = pointToPolylineDistance(p.lat, p.lon, polyline, polyStart, polyEnd);
            if (d > maxDev) maxDev = d;
        }
        return maxDev;
    }

    /** Min distance from a point to a polyline sub-range. */
    private static double pointToPolylineDistance(double pLat, double pLon,
                                                  PointList line, int from, int to) {
        double min = Double.POSITIVE_INFINITY;
        for (int i = from; i + 1 <= to; i++) {
            double d = pointToSegmentDistance(pLat, pLon,
                    line.getLat(i), line.getLon(i),
                    line.getLat(i + 1), line.getLon(i + 1));
            if (d < min) min = d;
        }
        return min;
    }

    /** Perpendicular point-to-segment distance in meters, clamped to segment endpoints. */
    private static double pointToSegmentDistance(double pLat, double pLon,
                                                 double aLat, double aLon,
                                                 double bLat, double bLon) {
        double meanLatRad = Math.toRadians((aLat + bLat) * 0.5);
        double mPerDegLat = 111_320.0;
        double mPerDegLon = 111_320.0 * Math.cos(meanLatRad);

        double ax = aLon * mPerDegLon, ay = aLat * mPerDegLat;
        double bx = bLon * mPerDegLon, by = bLat * mPerDegLat;
        double px = pLon * mPerDegLon, py = pLat * mPerDegLat;

        double dx = bx - ax, dy = by - ay;
        double segLenSq = dx * dx + dy * dy;
        if (segLenSq == 0) {
            return DIST.calcDist(pLat, pLon, aLat, aLon);
        }
        double t = ((px - ax) * dx + (py - ay) * dy) / segLenSq;
        if (t < 0) t = 0;
        else if (t > 1) t = 1;
        double cx = ax + t * dx;
        double cy = ay + t * dy;
        double ex = px - cx, ey = py - cy;
        return Math.sqrt(ex * ex + ey * ey);
    }

    private static int[] dedupConsecutive(int[] xs) {
        if (xs.length == 0) return xs;
        int[] out = new int[xs.length];
        int n = 0;
        int prev = Integer.MIN_VALUE;
        for (int x : xs) {
            if (x != prev) {
                out[n++] = x;
                prev = x;
            }
        }
        return Arrays.copyOf(out, n);
    }

    private static int[] pdValuesInt(List<PathDetail> details) {
        int[] out = new int[details.size()];
        for (int i = 0; i < details.size(); i++) {
            out[i] = ((Number) details.get(i).getValue()).intValue();
        }
        return out;
    }
}
