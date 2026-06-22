/*
 * Trailmap - Gravel Segment Analysis
 *
 * Phase B role classifier: the ordered cascade from docs/gravel_segments_design.md §3.3.
 * Pure function of encoded values read from an edge; first match wins.
 */
package com.graphhopper.trailmap.analysis.gravel.role;

import com.graphhopper.routing.ev.BooleanEncodedValue;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.ev.RoadClass;
import com.graphhopper.routing.ev.VehicleAccess;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.trailmap.analysis.gravel.GravelAnalysisConfig;
import com.graphhopper.trailmap.shared.GravelScale;
import com.graphhopper.trailmap.shared.MtbScale;
import com.graphhopper.trailmap.shared.PredictedSurface;
import com.graphhopper.util.EdgeIteratorState;

/**
 * Classifies an edge into a {@link EdgeRole} by evaluating an ordered cascade:
 *
 * <pre>
 * TARGET  if gravel_scale ∈ TARGET_GRAVEL_SCALES AND predicted_surface ∈ TARGET_SURFACES
 *            AND mtb_scale ∉ MTB_TRAIL_SCALES              (qualifying gravel — the only output)
 * IGNORED else if road_class ∈ IGNORED_ROAD_CLASSES        (non-rideable pedestrian: path/footway/…)
 * IGNORED else if road_class == TRACK
 *            AND gravel_scale ∈ IGNORED_TRACK_GRAVEL_SCALES (track too rough to ride: hike-a-bike)
 * ANCHOR  otherwise                                        (rideable non-gravel: roads/tracks/cycleways)
 * </pre>
 *
 * The three roles are the load-bearing model. ANCHOR is never output but counts for
 * <i>connectivity</i> — a gravel road woven into the network by a service road, drivable track,
 * cycleway, etc. must not look like a dead-end. IGNORED is excluded entirely: a {@code highway=path}
 * (or footway/steps/…) is not a rideable connector, so a gravel "loop" closing only through one is
 * still a dead-end. The downstream {@code GravelNetworkFilter} decides dead-ends from cycles in the
 * TARGET ∪ ANCHOR graph.
 */
public class EdgeRoleClassifier {

    private final GravelAnalysisConfig config;
    private final EnumEncodedValue<GravelScale> gravelScaleEnc;
    private final EnumEncodedValue<RoadClass> roadClassEnc;
    private final EnumEncodedValue<MtbScale> mtbScaleEnc;
    private final EnumEncodedValue<PredictedSurface> predictedSurfaceEnc;
    /** Trailmap bike access (BikeAccessParser). Null if the build does not carry it — gate disabled. */
    private final BooleanEncodedValue bikeAccessEnc;

    public EdgeRoleClassifier(GravelAnalysisConfig config,
                              EnumEncodedValue<GravelScale> gravelScaleEnc,
                              EnumEncodedValue<RoadClass> roadClassEnc,
                              EnumEncodedValue<MtbScale> mtbScaleEnc,
                              EnumEncodedValue<PredictedSurface> predictedSurfaceEnc,
                              BooleanEncodedValue bikeAccessEnc) {
        this.config = config;
        this.gravelScaleEnc = gravelScaleEnc;
        this.roadClassEnc = roadClassEnc;
        this.mtbScaleEnc = mtbScaleEnc;
        this.predictedSurfaceEnc = predictedSurfaceEnc;
        this.bikeAccessEnc = bikeAccessEnc;
    }

    /** Resolve EV handles from the encoding manager and build a classifier. */
    public static EdgeRoleClassifier create(GravelAnalysisConfig config, EncodingManager em) {
        String bikeAccessKey = VehicleAccess.key("bike");
        return new EdgeRoleClassifier(config,
                em.getEnumEncodedValue(GravelScale.KEY, GravelScale.class),
                em.getEnumEncodedValue(RoadClass.KEY, RoadClass.class),
                em.getEnumEncodedValue(MtbScale.KEY, MtbScale.class),
                em.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class),
                em.hasEncodedValue(bikeAccessKey) ? em.getBooleanEncodedValue(bikeAccessKey) : null);
    }

    public EdgeRole classify(EdgeIteratorState edge) {
        // Access pre-gate: a way a bicycle cannot legally traverse in EITHER direction is excluded
        // entirely (IGNORED) — it is neither gravel output nor a usable connector, exactly like a
        // non-rideable pedestrian way (a gravel "loop" closing only through it is still a dead-end).
        // bike_access is produced by BikeAccessParser, the authoritative Trailmap bike-access policy
        // (blocks no/restricted/military/emergency/private), so the qualifying access values are
        // decided there, never re-decided here.
        if (!isBikeTraversable(edge))
            return EdgeRole.IGNORED;
        GravelScale gravel = edge.get(gravelScaleEnc);
        RoadClass roadClass = edge.get(roadClassEnc);
        MtbScale mtb = edge.get(mtbScaleEnc);
        PredictedSurface surface = edge.get(predictedSurfaceEnc);
        return classify(gravel, roadClass, mtb, surface);
    }

    /** True iff a bicycle may traverse the edge in at least one direction (or access is unavailable). */
    private boolean isBikeTraversable(EdgeIteratorState edge) {
        return bikeAccessEnc == null || edge.get(bikeAccessEnc) || edge.getReverse(bikeAccessEnc);
    }

    /**
     * Whether an edge is eligible to act as a <b>Phase 2 connector</b>: a near-Target gravel way that
     * just misses the Target bands. Candidacy is by {@code gravel_scale} band (a key of {@code
     * connectorWeights}), gated by the same surface + bike-access rules as Target. Quality (the
     * cost-per-metre weight) is read from {@code connectorWeights} by the caller. Pure-quality test —
     * road class does not gate it (a near-Target track and a near-Target path are both candidates,
     * differing only by their weight).
     */
    public boolean isConnectorCandidate(EdgeIteratorState edge) {
        if (!isBikeTraversable(edge))
            return false;
        if (config.excludedSurfaces.contains(edge.get(predictedSurfaceEnc)))
            return false;
        return config.connectorWeights.containsKey(edge.get(gravelScaleEnc));
    }

    /** Classify from decoded values; testable without an edge. {@code roadClass} is unused
     *  (kept for signature stability) — connectivity now comes from ALL edges being ANCHOR. */
    public EdgeRole classify(GravelScale gravel, RoadClass roadClass, MtbScale mtb,
                             PredictedSurface surface) {
        // TARGET — qualifying gravel: a rideable gravel_scale, a non-asphalt surface, and not a
        // genuine MTB trail. gravel_scale carries the quality judgement; the surface gate only
        // rejects paved / asphalt-ambiguous surfaces. Road type does not gate it.
        if (config.targetGravelScales.contains(gravel)
                && !config.excludedSurfaces.contains(surface)
                && !config.mtbTrailScales.contains(mtb))
            return EdgeRole.TARGET;
        // IGNORED — a non-rideable pedestrian way: excluded entirely, does NOT connect.
        if (config.ignoredRoadClasses.contains(roadClass))
            return EdgeRole.IGNORED;
        // IGNORED — a track too rough to be a real connector (hike-a-bike, gravel_scale FOUR by
        // default). Selective expansion of the ignored class: applies only to road_class==TRACK.
        if (roadClass == RoadClass.TRACK && config.ignoredTrackGravelScales.contains(gravel))
            return EdgeRole.IGNORED;
        // ANCHOR — a rideable non-gravel way (road / drivable track / cycleway): connects, not output.
        return EdgeRole.ANCHOR;
    }
}
