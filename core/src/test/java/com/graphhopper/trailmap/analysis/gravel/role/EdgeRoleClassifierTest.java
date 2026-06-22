package com.graphhopper.trailmap.analysis.gravel.role;

import com.graphhopper.routing.ev.RoadClass;
import com.graphhopper.trailmap.analysis.gravel.GravelAnalysisConfig;
import com.graphhopper.trailmap.shared.GravelScale;
import com.graphhopper.trailmap.shared.MtbScale;
import com.graphhopper.trailmap.shared.PredictedSurface;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Table-driven tests for the Phase B role cascade (design §3.3 / §15), including the
 * cascade-order cases that the ordering is specifically designed to resolve.
 */
class EdgeRoleClassifierTest {

    private final EdgeRoleClassifier classifier =
            new EdgeRoleClassifier(new GravelAnalysisConfig(), null, null, null, null, null);

    /** Default surface is a qualifying gravel surface so gravel_scale assertions read cleanly. */
    private EdgeRole role(GravelScale g, RoadClass rc, MtbScale m) {
        return classifier.classify(g, rc, m, PredictedSurface.COMPACTED);
    }

    private EdgeRole role(GravelScale g, RoadClass rc, MtbScale m, PredictedSurface s) {
        return classifier.classify(g, rc, m, s);
    }

    @Test
    void unpavedRealRoadsAreTargetNotAnchor() {
        // The crux: an unpaved tertiary/unclassified/residential road is clamped to ZERO_PLUS by
        // GravelScaleParser and is qualifying gravel (TARGET) — road_class does NOT anchor it.
        // (Real Tampere examples: Okslammintie/Elovaarantie = unclassified ZERO_PLUS gravel.)
        assertEquals(EdgeRole.TARGET, role(GravelScale.ZERO_PLUS, RoadClass.TERTIARY, MtbScale.ZERO_MINUS));
        assertEquals(EdgeRole.TARGET, role(GravelScale.ZERO_PLUS, RoadClass.UNCLASSIFIED, MtbScale.ZERO_MINUS));
        assertEquals(EdgeRole.TARGET, role(GravelScale.ONE, RoadClass.RESIDENTIAL, MtbScale.UNKNOWN));
    }

    @Test
    void pavedIsAnchorRegardlessOfRoadClass() {
        // gravel_scale == ZERO_MINUS (paved) is the anchoring backbone. A paved tertiary
        // (e.g. the paved stretch of Pinsiöntie) is ANCHOR, not gravel.
        assertEquals(EdgeRole.ANCHOR, role(GravelScale.ZERO_MINUS, RoadClass.TERTIARY, MtbScale.ZERO_MINUS));
        assertEquals(EdgeRole.ANCHOR, role(GravelScale.ZERO_MINUS, RoadClass.MOTORWAY, MtbScale.UNKNOWN));
        assertEquals(EdgeRole.ANCHOR, role(GravelScale.ZERO_MINUS, RoadClass.CYCLEWAY, MtbScale.UNKNOWN));
    }

    @Test
    void gravelServiceRoadIsTarget() {
        // A gravel service road stays TARGET (it is exactly the dead-end Phase C may prune).
        assertEquals(EdgeRole.TARGET, role(GravelScale.ONE, RoadClass.SERVICE, MtbScale.UNKNOWN));
        assertEquals(EdgeRole.TARGET, role(GravelScale.ZERO_PLUS, RoadClass.SERVICE, MtbScale.UNKNOWN));
    }

    @Test
    void rideableGravelIsTarget() {
        // TARGET set = {ZERO, ZERO_PLUS, ONE}: ZERO (unpaved cycleway) is included.
        assertEquals(EdgeRole.TARGET, role(GravelScale.ZERO, RoadClass.CYCLEWAY, MtbScale.UNKNOWN));
        assertEquals(EdgeRole.TARGET, role(GravelScale.ZERO_PLUS, RoadClass.TRACK, MtbScale.ZERO));
        assertEquals(EdgeRole.TARGET, role(GravelScale.ONE, RoadClass.TRACK, MtbScale.ONE));
        // classification is purely gravel_scale-driven: mtb_scale no longer forces IGNORED by default
        assertEquals(EdgeRole.TARGET, role(GravelScale.ONE, RoadClass.TRACK, MtbScale.TWO));
    }

    @Test
    void surfaceGateRejectsOnlyAsphalt() {
        // gravel_scale carries the quality judgement; the surface gate only rejects asphalt /
        // asphalt-ambiguous surfaces. Every other (non-asphalt) surface with a qualifying scale is
        // TARGET — including GROUND (OSM surface=dirt) and ROUGH_GRAVEL.
        assertEquals(EdgeRole.TARGET, role(GravelScale.ZERO_PLUS, RoadClass.TRACK, MtbScale.UNKNOWN, PredictedSurface.COMPACTED));
        assertEquals(EdgeRole.TARGET, role(GravelScale.ONE, RoadClass.TRACK, MtbScale.UNKNOWN, PredictedSurface.FINE_GRAVEL));
        assertEquals(EdgeRole.TARGET, role(GravelScale.ONE, RoadClass.TRACK, MtbScale.UNKNOWN, PredictedSurface.MEDIUM_GRAVEL));
        assertEquals(EdgeRole.TARGET, role(GravelScale.ZERO_PLUS, RoadClass.TRACK, MtbScale.UNKNOWN, PredictedSurface.GROUND));
        assertEquals(EdgeRole.TARGET, role(GravelScale.ONE, RoadClass.TRACK, MtbScale.UNKNOWN, PredictedSurface.ROUGH_GRAVEL));
        // asphalt and the asphalt-ambiguous surface -> ANCHOR even with a qualifying gravel_scale.
        assertEquals(EdgeRole.ANCHOR, role(GravelScale.ZERO_PLUS, RoadClass.TRACK, MtbScale.UNKNOWN, PredictedSurface.ASPHALT));
        assertEquals(EdgeRole.ANCHOR, role(GravelScale.ONE, RoadClass.TRACK, MtbScale.UNKNOWN, PredictedSurface.ASPHALT_OR_UNPAVED));
    }

    @Test
    void nonQualifyingNonPedestrianIsAnchor() {
        // Non-qualifying gravel on a rideable/drivable way is ANCHOR (connectivity only, never output).
        assertEquals(EdgeRole.ANCHOR, role(GravelScale.TWO, RoadClass.TRACK, MtbScale.UNKNOWN));
        assertEquals(EdgeRole.ANCHOR, role(GravelScale.THREE, RoadClass.TRACK, MtbScale.UNKNOWN));
        assertEquals(EdgeRole.ANCHOR, role(GravelScale.FOUR, RoadClass.SERVICE, MtbScale.UNKNOWN));
        assertEquals(EdgeRole.ANCHOR, role(GravelScale.UNKNOWN, RoadClass.TRACK, MtbScale.UNKNOWN));
        assertEquals(EdgeRole.ANCHOR, role(GravelScale.FERRY, RoadClass.OTHER, MtbScale.UNKNOWN));
        assertEquals(EdgeRole.ANCHOR, role(GravelScale.ZERO_MINUS, RoadClass.CYCLEWAY, MtbScale.UNKNOWN));
    }

    @Test
    void roughTrackIsIgnoredNotAnchor() {
        // A highway=track with gravel_scale FOUR (hike-a-bike) is too rough to be a real connector
        // -> IGNORED (selective expansion of the ignored class), not ANCHOR.
        assertEquals(EdgeRole.IGNORED, role(GravelScale.FOUR, RoadClass.TRACK, MtbScale.UNKNOWN));
        // ...but only for TRACK: a rough non-gravel value on other rideable classes stays ANCHOR.
        assertEquals(EdgeRole.ANCHOR, role(GravelScale.FOUR, RoadClass.SERVICE, MtbScale.UNKNOWN));
        assertEquals(EdgeRole.ANCHOR, role(GravelScale.FOUR, RoadClass.UNCLASSIFIED, MtbScale.UNKNOWN));
        // ...and a track that is still rideable enough (THREE or better) remains a connector.
        assertEquals(EdgeRole.ANCHOR, role(GravelScale.THREE, RoadClass.TRACK, MtbScale.UNKNOWN));
        assertEquals(EdgeRole.ANCHOR, role(GravelScale.TWO, RoadClass.TRACK, MtbScale.UNKNOWN));
    }

    @Test
    void pedestrianWaysIgnored() {
        // Non-rideable pedestrian ways do not connect (and are never output) -> IGNORED.
        assertEquals(EdgeRole.IGNORED, role(GravelScale.FOUR, RoadClass.PATH, MtbScale.UNKNOWN));
        assertEquals(EdgeRole.IGNORED, role(GravelScale.UNKNOWN, RoadClass.PATH, MtbScale.UNKNOWN));
        assertEquals(EdgeRole.IGNORED, role(GravelScale.THREE, RoadClass.FOOTWAY, MtbScale.UNKNOWN));
        assertEquals(EdgeRole.IGNORED, role(GravelScale.FOUR, RoadClass.STEPS, MtbScale.UNKNOWN));
        // ...but a path/footway that is itself qualifying gravel is still TARGET (gravel decides first).
        assertEquals(EdgeRole.TARGET, role(GravelScale.ZERO_PLUS, RoadClass.PATH, MtbScale.UNKNOWN));
    }
}
