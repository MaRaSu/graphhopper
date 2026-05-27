package com.graphhopper.trailmap.shared;

import com.graphhopper.reader.ReaderWay;
import com.graphhopper.routing.ev.ArrayEdgeIntAccess;
import com.graphhopper.routing.ev.EdgeIntAccess;
import com.graphhopper.routing.ev.EncodedValue;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.storage.IntsRef;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static com.graphhopper.trailmap.shared.PredictedSurface.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;

/**
 * Tests for PredictedSurfaceParser — verifies OSM tag combinations map to correct
 * PredictedSurface values.
 *
 * Core focus: ROAD NETWORK business logic.
 *
 * Road network = {motorway, trunk, primary, secondary, tertiary (+_link variants),
 *                 unclassified, residential}. For these highways the predicted_surface
 *                 must be one of only four values:
 *     ASPHALT, ASPHALT_OR_UNPAVED (ambiguous), COMPACTED, FINE_GRAVEL.
 * Rougher categories (MEDIUM/ROUGH_GRAVEL, GROUND, SAND, MUD) are physically
 * incompatible with a road-network designation and must not appear.
 *
 * Defaults when no `surface` tag (and no smoothness / tracktype / mtb:scale signal):
 *     motorway, trunk, primary (+_link) → ASPHALT
 *     secondary (+_link)                → ASPHALT_OR_UNPAVED
 *     tertiary (+_link), unclassified,
 *     residential                       → ASPHALT_OR_UNPAVED
 *
 * Per `surface` tag (overrides default):
 *     asphalt | paved | concrete | paving_stones → ASPHALT
 *     compacted                                  → COMPACTED
 *     fine_gravel                                → FINE_GRAVEL
 *     gravel | unpaved                           → COMPACTED
 *         (mappers overload `gravel`; on a road network it means compacted)
 */
class PredictedSurfaceParserTest {

    private PredictedSurfaceParser parser;

    @BeforeEach
    void setUp() {
        parser = new PredictedSurfaceParser(PredictedSurface.create());
    }

    /**
     * Build a ReaderWay from key-value tag pairs, compute predicted surface, assert result.
     */
    private void assertSurface(PredictedSurface expected, String description, String... tagPairs) {
        ReaderWay way = new ReaderWay(1);
        for (int i = 0; i < tagPairs.length; i += 2) {
            way.setTag(tagPairs[i], tagPairs[i + 1]);
        }
        assertEquals(expected, parser.computePredictedSurface(way), description);
    }

    /**
     * Collect mismatches instead of failing immediately, then report all at once.
     */
    private void assertSurfaceCollect(List<String> failures, PredictedSurface expected,
                                      String description, String... tagPairs) {
        ReaderWay way = new ReaderWay(1);
        for (int i = 0; i < tagPairs.length; i += 2) {
            way.setTag(tagPairs[i], tagPairs[i + 1]);
        }
        PredictedSurface actual = parser.computePredictedSurface(way);
        if (actual != expected) {
            failures.add(description + " ==> expected: " + expected + " but was: " + actual);
        }
    }

    // =================================================================
    // ROAD NETWORK — defaults (no surface tag)
    // =================================================================

    @Test
    void testRoadNetworkDefaultsNoSurface() {
        List<String> failures = new ArrayList<>();

        // Higher tier: always ASPHALT
        assertSurfaceCollect(failures, ASPHALT, "motorway / no surface",
                "highway", "motorway");
        assertSurfaceCollect(failures, ASPHALT, "motorway_link / no surface",
                "highway", "motorway_link");
        assertSurfaceCollect(failures, ASPHALT, "trunk / no surface",
                "highway", "trunk");
        assertSurfaceCollect(failures, ASPHALT, "trunk_link / no surface",
                "highway", "trunk_link");
        assertSurfaceCollect(failures, ASPHALT, "primary / no surface",
                "highway", "primary");
        assertSurfaceCollect(failures, ASPHALT, "primary_link / no surface",
                "highway", "primary_link");

        // Lower tier: AMBIGUOUS (ASPHALT_OR_UNPAVED)
        assertSurfaceCollect(failures, ASPHALT_OR_UNPAVED, "secondary / no surface",
                "highway", "secondary");
        assertSurfaceCollect(failures, ASPHALT_OR_UNPAVED, "secondary_link / no surface",
                "highway", "secondary_link");
        assertSurfaceCollect(failures, ASPHALT_OR_UNPAVED, "tertiary / no surface",
                "highway", "tertiary");
        assertSurfaceCollect(failures, ASPHALT_OR_UNPAVED, "tertiary_link / no surface",
                "highway", "tertiary_link");
        assertSurfaceCollect(failures, ASPHALT_OR_UNPAVED, "unclassified / no surface",
                "highway", "unclassified");
        assertSurfaceCollect(failures, ASPHALT_OR_UNPAVED, "residential / no surface",
                "highway", "residential");

        if (!failures.isEmpty()) {
            fail(failures.size() + " failures:\n  " + String.join("\n  ", failures));
        }
    }

    // =================================================================
    // ROAD NETWORK — explicit `surface=asphalt|paved|concrete|paving_stones`
    // =================================================================

    @Test
    void testRoadNetworkSurfaceAsphalt() {
        List<String> failures = new ArrayList<>();

        for (String hw : new String[]{
                "motorway", "trunk", "primary", "secondary", "tertiary",
                "unclassified", "residential"}) {
            assertSurfaceCollect(failures, ASPHALT, hw + " + asphalt",
                    "highway", hw, "surface", "asphalt");
            assertSurfaceCollect(failures, ASPHALT, hw + " + paved",
                    "highway", hw, "surface", "paved");
            assertSurfaceCollect(failures, ASPHALT, hw + " + concrete",
                    "highway", hw, "surface", "concrete");
            assertSurfaceCollect(failures, ASPHALT, hw + " + paving_stones",
                    "highway", hw, "surface", "paving_stones");
        }

        if (!failures.isEmpty()) {
            fail(failures.size() + " failures:\n  " + String.join("\n  ", failures));
        }
    }

    // =================================================================
    // ROAD NETWORK — explicit `surface=compacted`
    // =================================================================

    @Test
    void testRoadNetworkSurfaceCompacted() {
        List<String> failures = new ArrayList<>();

        for (String hw : new String[]{
                "secondary", "secondary_link", "tertiary", "tertiary_link",
                "unclassified", "residential"}) {
            assertSurfaceCollect(failures, COMPACTED, hw + " + compacted",
                    "highway", hw, "surface", "compacted");
        }

        if (!failures.isEmpty()) {
            fail(failures.size() + " failures:\n  " + String.join("\n  ", failures));
        }
    }

    // =================================================================
    // ROAD NETWORK — explicit `surface=fine_gravel`
    // =================================================================

    @Test
    void testRoadNetworkSurfaceFineGravel() {
        List<String> failures = new ArrayList<>();

        for (String hw : new String[]{
                "secondary", "secondary_link", "tertiary", "tertiary_link",
                "unclassified", "residential"}) {
            assertSurfaceCollect(failures, FINE_GRAVEL, hw + " + fine_gravel",
                    "highway", hw, "surface", "fine_gravel");
        }

        if (!failures.isEmpty()) {
            fail(failures.size() + " failures:\n  " + String.join("\n  ", failures));
        }
    }

    // =================================================================
    // ROAD NETWORK — explicit `surface=gravel` or `surface=unpaved`
    // (mappers overload these; on road network they mean compacted)
    // =================================================================

    @Test
    void testRoadNetworkSurfaceGravelOrUnpaved() {
        List<String> failures = new ArrayList<>();

        // The original case that triggered this work
        assertSurfaceCollect(failures, COMPACTED, "secondary + gravel (original case)",
                "highway", "secondary", "surface", "gravel");

        for (String hw : new String[]{
                "secondary", "secondary_link", "tertiary", "tertiary_link",
                "unclassified", "residential"}) {
            assertSurfaceCollect(failures, COMPACTED, hw + " + gravel",
                    "highway", hw, "surface", "gravel");
            assertSurfaceCollect(failures, COMPACTED, hw + " + unpaved",
                    "highway", hw, "surface", "unpaved");
        }

        if (!failures.isEmpty()) {
            fail(failures.size() + " failures:\n  " + String.join("\n  ", failures));
        }
    }

    // =================================================================
    // FERRY (rule 1)
    // =================================================================

    @Test
    void testFerry() {
        assertSurface(FERRY, "route=ferry",
                "route", "ferry");
        assertSurface(FERRY, "route=ferry + highway=service (ferry wins)",
                "route", "ferry", "highway", "service");
    }

    // =================================================================
    // Encoded value pipeline integration
    // =================================================================

    @Test
    void testEncodedValuePipeline() {
        EnumEncodedValue<PredictedSurface> enc = PredictedSurface.create();
        enc.init(new EncodedValue.InitializerConfig());
        PredictedSurfaceParser fullParser = new PredictedSurfaceParser(enc);
        EdgeIntAccess access = new ArrayEdgeIntAccess(1);
        IntsRef relFlags = new IntsRef(2);

        // Original triggering case end-to-end through the pipeline
        ReaderWay way = new ReaderWay(1);
        way.setTag("highway", "secondary");
        way.setTag("surface", "gravel");
        fullParser.handleWayTags(0, access, way, relFlags);
        assertEquals(COMPACTED, enc.getEnum(false, 0, access),
                "pipeline: secondary + gravel → COMPACTED");

        // Secondary with no surface → AMBIGUOUS
        access = new ArrayEdgeIntAccess(1);
        way = new ReaderWay(2);
        way.setTag("highway", "secondary");
        fullParser.handleWayTags(0, access, way, relFlags);
        assertEquals(ASPHALT_OR_UNPAVED, enc.getEnum(false, 0, access),
                "pipeline: secondary / no surface → AMBIGUOUS");
    }
}
