/*
 * Trailmap - Gravel Segment Analysis
 *
 * All tunable levers for the gravel-network extraction in one place.
 * See docs/gravel_segments_design.md §10. No tuning threshold is hard-coded:
 * every lever is settable, and gravelSizeThresholdM has no default (it is the
 * primary knob and must be supplied explicitly).
 */
package com.graphhopper.trailmap.analysis.gravel;

import com.graphhopper.routing.ev.RoadClass;
import com.graphhopper.trailmap.shared.GravelScale;
import com.graphhopper.trailmap.shared.MtbScale;
import com.graphhopper.trailmap.shared.PredictedSurface;

import java.util.EnumMap;
import java.util.EnumSet;
import java.util.Map;
import java.util.Set;

/**
 * Tunable configuration for {@code GravelSegmentTool}. Defaults match the design's
 * §3.3 / §10 tables. {@link #gravelSizeThresholdM} is intentionally unset (NaN) and
 * must be provided by the caller, or {@link #validate()} throws.
 */
public class GravelAnalysisConfig {

    // --- Phase B: role classification levers ---

    /**
     * {@code gravel_scale} values that count as qualifying gravel (TARGET). Per the client's
     * requirement {@code ZERO, ZERO_PLUS, ONE} qualify. {@code ZERO} (well-maintained unpaved,
     * e.g. unpaved cycleways) IS included; {@code TWO} is excluded (rougher gravel — useful only
     * as a connector, not as real gravel); {@code ZERO_MINUS} (paved) and THREE/FOUR/UNKNOWN/FERRY
     * are excluded.
     */
    public Set<GravelScale> targetGravelScales =
            EnumSet.of(GravelScale.ZERO, GravelScale.ZERO_PLUS, GravelScale.ONE);

    /**
     * {@code predicted_surface} values that DISQUALIFY a way from TARGET — i.e. "non-asphalt"
     * surfaces only are gravel. {@code gravel_scale} already decides gravel quality (rough/sand/mud
     * land in non-target scales), so the surface gate just rejects paved and asphalt-ambiguous
     * surfaces. A previous whitelist {COMPACTED, FINE_GRAVEL, MEDIUM_GRAVEL} wrongly dropped
     * gravel-quality {@code GROUND} tracks (OSM {@code surface=dirt} → GROUND) — a category-
     * definition error the client corrected.
     */
    public Set<PredictedSurface> excludedSurfaces =
            EnumSet.of(PredictedSurface.ASPHALT, PredictedSurface.ASPHALT_OR_UNPAVED);

    /**
     * Road classes that are IGNORED (excluded from the graph entirely — neither output nor
     * connectivity) when not qualifying gravel: the non-rideable pedestrian ways. A gravel "loop"
     * that closes only through these is still a dead-end, so they must not connect. Every OTHER
     * non-gravel road (drivable roads, drivable tracks, cycleways) is ANCHOR and DOES connect.
     *
     * <p>Note: a {@code highway=path} that is itself qualifying gravel is still TARGET (the gravel
     * metrics decide TARGET before this); this set only governs NON-gravel pedestrian ways.
     * Admitting a short path/rough way as a deliberate <i>connector</i> (bounded length/grade) is
     * the deferred Phase-2 "connector" feature — not this blanket rule.</p>
     */
    public Set<RoadClass> ignoredRoadClasses = EnumSet.of(
            RoadClass.PATH, RoadClass.FOOTWAY, RoadClass.STEPS, RoadClass.BRIDLEWAY,
            RoadClass.PEDESTRIAN);

    /**
     * {@code gravel_scale} values that demote a {@code highway=track} from ANCHOR to IGNORED —
     * a selective expansion of the ignored class. A track is normally a rideable connector (ANCHOR),
     * but a track this rough is hike-a-bike: it is no more a real connector than a footway, so a
     * gravel "loop" closing only through it is still a dead-end. Default {@code {FOUR}} (4 = not
     * rideable, walking required). Applies ONLY to {@code road_class == TRACK}; all other road
     * classes keep their existing ANCHOR/IGNORED treatment regardless of gravel_scale.
     */
    public Set<GravelScale> ignoredTrackGravelScales = EnumSet.of(GravelScale.FOUR);

    /**
     * {@code mtb_scale} values that force IGNORED. <b>Empty by default</b>: {@code gravel_scale}
     * already encodes MTB difficulty (the parser pushes genuine technical MTB toward
     * THREE/FOUR/UNKNOWN, which are non-qualifying), so the filter stays purely
     * {@code gravel_scale}-driven. Lever retained to re-impose an explicit MTB exclusion if needed.
     */
    public Set<MtbScale> mtbTrailScales = EnumSet.noneOf(MtbScale.class);

    /**
     * Road classes that form the <b>real-road backbone</b> for dead-end pruning (Phase C). Per spec
     * §1, a gravel cluster is a dead-end unless it reaches a "real road" that anchors it; the size
     * threshold then decides whether a single-attachment cluster is a substantial network (kept) or
     * a thin dead-end (pruned). Only ANCHOR edges of these classes anchor a cluster — rough/ground
     * ANCHOR tracks still provide connectivity but do NOT terminate a dead-end, so a gravel loop
     * reachable only through them is judged a closed cluster by its own qualifying length.
     *
     * <p>Default = the drivable road grid plus paved cycleways (real bike infrastructure that you
     * can legitimately reach a gravel network through). {@code TRACK} is deliberately excluded — a
     * rough/ground forest track is not the backbone, so gravel reachable only through such tracks is
     * a dead-end cluster judged by size. Pedestrian classes are already IGNORED. {@code SERVICE} is
     * included (urban/yard service roads are real); a forestry service spur that is itself gravel is
     * TARGET, not ANCHOR, so it does not falsely anchor.</p>
     */
    public Set<RoadClass> backboneRoadClasses = EnumSet.of(
            RoadClass.MOTORWAY, RoadClass.TRUNK, RoadClass.PRIMARY, RoadClass.SECONDARY,
            RoadClass.TERTIARY, RoadClass.RESIDENTIAL, RoadClass.UNCLASSIFIED, RoadClass.SERVICE,
            RoadClass.LIVING_STREET, RoadClass.ROAD, RoadClass.CYCLEWAY);

    // --- Pipeline stage toggles (each stage can be excluded independently for testing) ---

    /**
     * STEP 2 master switch (default ON). When ON, standalone gravel clusters whose looped core
     * totals ≥ {@link #gravelSizeThresholdM} are rescued (goal §6 case B). When OFF, only case A
     * survives — pure Step 1.
     */
    public boolean enableStandaloneRescue = true;

    /**
     * STEP 3 master switch (default ON). When ON, bounded connector chains (edges that are neither
     * road R nor gravel T — i.e. tracks and paths) that bridge otherwise-separate gravel are
     * admitted into the connectivity used by Steps 1–2, so gravel that loops/links only through such
     * a short crossing is recognised. When OFF, the base graph is exactly R ∪ T.
     */
    public boolean enableConnectors = true;

    /**
     * Connector candidacy + cost is <b>scored by quality</b>, not a flat length limit. A way is a
     * connector candidate iff its {@code gravel_scale} is a key here (and it passes the same surface
     * + bike-access gates as Target); the value is its <b>cost-per-metre weight</b>. A corridor of
     * connectors is admitted iff the cheapest weighted path bridging two distinct attachment points
     * costs ≤ {@link #connectorCostBudget}. So a good-but-not-Target band (e.g. {@code TWO}) is light
     * and can stretch far, while a hike-a-bike band ({@code FOUR}) is heavy and only a few metres fit
     * the budget — yet both may appear in the same corridor, each spending from the shared budget in
     * proportion to how bad it is. The Target bands ({@link #targetGravelScales}) must NOT appear here
     * (they are output, not glue). Tune per run to get "easy gravel" (lighter weights / bigger budget)
     * vs "extreme gravel" (stricter) outputs.
     */
    public Map<GravelScale, Double> connectorWeights = defaultConnectorWeights();

    private static Map<GravelScale, Double> defaultConnectorWeights() {
        Map<GravelScale, Double> w = new EnumMap<>(GravelScale.class);
        w.put(GravelScale.TWO, 1.0);    // good-but-not-Target: reach = budget
        w.put(GravelScale.THREE, 2.5);  // rougher
        w.put(GravelScale.FOUR, 10.0);  // hike-a-bike: only a few metres fit the budget
        return w;
    }

    /**
     * Per-{@code mtb_scale} overrides of the connector weight, keyed by gravel band then mtb band —
     * the connector analogue of how the gravel-routing profile lowers speed/priority for technical
     * tracks (e.g. {@code gravel_scale==TWO && mtb_scale==ONE}). When a connector edge matches an entry
     * here, this weight is used instead of {@link #connectorWeights}; otherwise the gravel-band weight
     * applies. Default: a "hard gravel" {@code TWO} that is also MTB-technical ({@code mtb_scale==ONE})
     * costs like a {@code THREE} (2.5) — it is harder to ride than a plain hard-gravel track. Candidacy
     * is unaffected (still by {@link #connectorWeights} band); this only changes the cost.
     */
    public Map<GravelScale, Map<MtbScale, Double>> connectorMtbWeightOverrides =
            defaultConnectorMtbWeightOverrides();

    private static Map<GravelScale, Map<MtbScale, Double>> defaultConnectorMtbWeightOverrides() {
        Map<GravelScale, Map<MtbScale, Double>> m = new EnumMap<>(GravelScale.class);
        Map<MtbScale, Double> two = new EnumMap<>(MtbScale.class);
        two.put(MtbScale.ONE, 2.5);     // gs2 + mtb1 → cost like gs3
        m.put(GravelScale.TWO, two);
        return m;
    }

    /**
     * The connector cost-per-metre weight for an edge of the given gravel/mtb bands: a
     * {@link #connectorMtbWeightOverrides} entry if present, else the {@link #connectorWeights} band
     * weight, else a large fallback (1000) for a band with no configured weight.
     */
    public double connectorWeight(GravelScale gravel, MtbScale mtb) {
        Map<MtbScale, Double> ov = connectorMtbWeightOverrides.get(gravel);
        if (ov != null) {
            Double w = ov.get(mtb);
            if (w != null) return w;
        }
        Double base = connectorWeights.get(gravel);
        return base != null ? base : 1000.0;
    }

    /**
     * The single weighted-length budget (in weighted metres) for an admitted connector corridor — the
     * cheapest weighted path bridging two distinct attachment points must cost ≤ this. With the default
     * weights a pure {@code TWO} corridor reaches this many real metres; {@code THREE} ~40%, {@code
     * FOUR} ~10%. With the default weights a pure scale-2 ("hard gravel") corridor reaches this many
     * real metres, scale-3 ~40%, scale-4 ("real push / no ride") ~10%.
     */
    public double connectorCostBudget = 450.0;

    /**
     * When true (default), admitted connectors are split in the OUTPUT into "needed" vs "redundant":
     * a connector is <b>needed</b> if it is a bridge in the kept network (removing it would disconnect
     * some kept TARGET gravel — i.e. it actually rescues gravel), and <b>redundant</b> otherwise (a
     * loop beside gravel that is already connected). Redundant connectors are emitted with
     * {@code connectivity = "connector_redundant"} so they can be styled faintly or hidden.
     *
     * <p><b>This is purely an output label.</b> It NEVER removes a connector from the graph analysis —
     * all within-budget connectors always provide connectivity to the filter, so the marking can never
     * sever a needed link (the bug the old connectivity gate had). Connectivity is a MUST; the pruned
     * output is a nicety. When false, every admitted connector is just {@code "connector"}.</p>
     */
    public boolean connectorMarkRedundant = true;

    // --- Phase C/D: network size lever ---

    /**
     * The single size lever (metres): minimum total <b>qualifying (TARGET) gravel length</b> for a
     * dead-end gravel cluster to be kept (see {@link
     * com.graphhopper.trailmap.analysis.gravel.prune.GravelNetworkFilter}). A "dead-end cluster" is
     * a pendant appendage that reaches the real-road backbone at a <i>single</i> articulation point
     * (or an island reaching it at none) — its whole appendage (the access stick <i>and</i> any
     * terminal loop) is kept or dropped <b>as one unit</b> by this length; never partially. Gravel
     * that is 2-edge-connected to the backbone (reachable two independent ways) is a through-network
     * and is always kept. No safe default — must be set explicitly (the client uses 4&nbsp;km).
     */
    public double gravelSizeThresholdM = Double.NaN;

    /** Optional separate threshold for islands. Defaults to {@link #gravelSizeThresholdM}. */
    public double islandSizeThresholdM = Double.NaN;

    /**
     * Minimum cyclic-core length (metres) a ROAD connected-component must have to count as part of the
     * real road grid (goal §5). The road grid is the 2-edge-connected (cyclic) core of the road
     * network; but a road component is only the real grid if its own cyclic core reaches this floor.
     * This drops <b>isolated tiny road loops</b> — parking aisles, turning circles, yards (often a few
     * tens of metres of service road) — that are 2-edge-connected in isolation yet are not part of the
     * through-road grid, so they cannot hand a dead-end gravel road a false "second junction".
     *
     * <p>The main road network is one huge component and always clears this floor, so a normal grid is
     * unaffected; only stray isolated road cycles are excluded. 0 disables the floor. Set high enough
     * that an isolated service/forestry loop (which can easily run a few hundred metres) is NOT
     * mistaken for grid; isolated small road networks below it (e.g. tiny-island lanes) lose their grid
     * status, which is acceptable — that gravel is out of interest. The parking-lot loop that motivated
     * this (way 1202926746 et al.) is ~63&nbsp;m of cyclic core.</p>
     */
    public double gridMinComponentCoreLenM = 3000.0;

    // --- Phase E: grouping levers ---

    /** Require equal {@code ref} (not just {@code name}) to merge two edges into one logical road. */
    public boolean requireSameRef = false;

    /** Emit unnamed gravel runs under a synthetic "(unnamed)" bucket (else drop them from logical roads). */
    public boolean emitUnnamed = true;

    /**
     * Resolve the effective island threshold (Phase D), falling back to the shared gravel threshold.
     */
    public double effectiveIslandThresholdM() {
        return Double.isNaN(islandSizeThresholdM) ? gravelSizeThresholdM : islandSizeThresholdM;
    }

    /**
     * Validate that the required levers are set. Call before running the analysis.
     *
     * @throws IllegalStateException if {@link #gravelSizeThresholdM} is unset or non-positive.
     */
    public void validate() {
        if (Double.isNaN(gravelSizeThresholdM) || gravelSizeThresholdM <= 0)
            throw new IllegalStateException("gravelSizeThresholdM must be set to a positive value "
                    + "(the primary size lever has no safe default; supply gravel.size_threshold_m).");
        if (!Double.isNaN(islandSizeThresholdM) && islandSizeThresholdM <= 0)
            throw new IllegalStateException("islandSizeThresholdM, if set, must be positive.");
        if (gridMinComponentCoreLenM < 0)
            throw new IllegalStateException("gridMinComponentCoreLenM must be >= 0 (0 disables it).");
        if (targetGravelScales.isEmpty())
            throw new IllegalStateException("targetGravelScales must not be empty.");
        if (enableConnectors) {
            if (Double.isNaN(connectorCostBudget) || connectorCostBudget <= 0)
                throw new IllegalStateException("connectorCostBudget must be positive when "
                        + "enableConnectors is true.");
            if (connectorWeights.isEmpty())
                throw new IllegalStateException("connectorWeights must be non-empty when "
                        + "enableConnectors is true (it defines the connector candidate bands).");
            for (GravelScale gs : connectorWeights.keySet())
                if (targetGravelScales.contains(gs))
                    throw new IllegalStateException("connectorWeights must not include a Target band: "
                            + gs + " is in targetGravelScales (connectors are glue, not output).");
            // Weights are used directly as Dijkstra edge costs; they must be positive and finite
            // (a zero weight makes arbitrarily long connector chains free; negative/NaN/∞ break the
            // cost model).
            for (Map.Entry<GravelScale, Double> w : connectorWeights.entrySet())
                requirePositiveFinite("connectorWeights[" + w.getKey() + "]", w.getValue());
            for (Map.Entry<GravelScale, Map<MtbScale, Double>> g : connectorMtbWeightOverrides.entrySet())
                for (Map.Entry<MtbScale, Double> w : g.getValue().entrySet())
                    requirePositiveFinite("connectorMtbWeightOverrides[" + g.getKey() + "][" + w.getKey() + "]",
                            w.getValue());
        }
    }

    private static void requirePositiveFinite(String name, Double v) {
        if (v == null || Double.isNaN(v) || Double.isInfinite(v) || v <= 0)
            throw new IllegalStateException(name + " must be a positive finite weight, got " + v);
    }
}
