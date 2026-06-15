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

import java.util.EnumSet;
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
     * {@code predicted_surface} values a TARGET way must also have. A way qualifies as gravel only
     * if its {@code gravel_scale} is in {@link #targetGravelScales} <b>and</b> its predicted surface
     * is one of these — keeping genuine gravel/compacted surfaces and excluding paved/rough/ground.
     * Defaults: {@code COMPACTED} (well-maintained), {@code FINE_GRAVEL} (outdoor paths),
     * {@code MEDIUM_GRAVEL} (rideable but slower).
     */
    public Set<PredictedSurface> targetSurfaces =
            EnumSet.of(PredictedSurface.COMPACTED, PredictedSurface.FINE_GRAVEL,
                    PredictedSurface.MEDIUM_GRAVEL);

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

    // --- Phase 2: bounded connector chains (toggleable; default OFF) ---

    /**
     * Master switch for Phase 2 connectors. When {@code false} (default) the pipeline is pure
     * Phase 1: paths and gravel_scale-4 tracks stay IGNORED and the run is exactly the Phase-1
     * result — so Phase 1 and Phase 2 can be validated independently. When {@code true}, normally-
     * IGNORED ways that match the connector criteria below are admitted as bounded connector chains
     * (promoted to ANCHOR connectivity) so that two otherwise-separate gravel pieces count as one.
     */
    public boolean enableConnectors = false;

    /**
     * Maximum total length (metres) of a connector chain that may be admitted. A chain longer than
     * this is not a "short crossing" and is dropped. Per spec §3 a connector is a <i>chain</i>, not a
     * single edge — the bound is on cumulative chain length, which is why this needs a traversal,
     * not a stateless filter. Tunable; the starting value is a short crossing, not a route leg.
     */
    public double connectorMaxChainLenM = 200.0;

    /**
     * {@code road_class == TRACK} gravel scales eligible to act as a connector when {@link
     * #enableConnectors}. Default {@code {FOUR}} — the hike-a-bike tracks that are IGNORED in
     * Phase 1 but may legitimately bridge two gravel networks over a short distance.
     */
    public Set<GravelScale> connectorTrackGravelScales = EnumSet.of(GravelScale.FOUR);

    /**
     * Pedestrian road classes eligible to act as a connector when {@link #enableConnectors}.
     * Default {@code {PATH}} — a short path may bridge gravel; footway/steps stay excluded.
     */
    public Set<RoadClass> connectorPathClasses = EnumSet.of(RoadClass.PATH);

    /**
     * Minimum distinct attachment points (TARGET/ANCHOR nodes) a connector chain must touch to be
     * admitted. Default 2: a connector must <i>bridge</i> two things; a chain that dead-ends bridges
     * nothing and is dropped (so a rough track that merely props up a single spur is not admitted).
     */
    public int connectorMinAttachmentPoints = 2;

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
        if (targetGravelScales.isEmpty())
            throw new IllegalStateException("targetGravelScales must not be empty.");
        if (targetSurfaces.isEmpty())
            throw new IllegalStateException("targetSurfaces must not be empty.");
        if (enableConnectors && (Double.isNaN(connectorMaxChainLenM) || connectorMaxChainLenM <= 0))
            throw new IllegalStateException("connectorMaxChainLenM must be positive when "
                    + "enableConnectors is true.");
    }
}
