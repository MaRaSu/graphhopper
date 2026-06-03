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
    // ROAD NETWORK — dirt / ground / sand / mud clamp
    // Per agreed business logic, road-network highways must only produce
    // ASPHALT / AMBIGUOUS / COMPACTED / FINE_GRAVEL. The GROUND / SAND / MUD
    // rules don't filter on highway class, so without the fix a road tagged
    // with these surfaces leaks into a trail-grade category. The clamp:
    // road-network + {dirt, ground, sand, mud} → COMPACTED.
    // =================================================================

    @Test
    void testRoadNetworkDifficultSurfaceClamp() {
        List<String> failures = new ArrayList<>();

        for (String hw : new String[]{
                "secondary", "secondary_link", "tertiary", "tertiary_link",
                "unclassified", "residential"}) {
            assertSurfaceCollect(failures, COMPACTED, hw + " + dirt",
                    "highway", hw, "surface", "dirt");
            assertSurfaceCollect(failures, COMPACTED, hw + " + ground",
                    "highway", hw, "surface", "ground");
            assertSurfaceCollect(failures, COMPACTED, hw + " + sand",
                    "highway", hw, "surface", "sand");
            assertSurfaceCollect(failures, COMPACTED, hw + " + mud",
                    "highway", hw, "surface", "mud");
        }

        if (!failures.isEmpty()) {
            fail(failures.size() + " failures:\n  " + String.join("\n  ", failures));
        }
    }

    // =================================================================
    // REGRESSION GUARD — non-road-network classes with same surfaces
    // The clamp only applies to LIKELY_COMPACT_HIGHWAYS (lower-tier road
    // network). Paths, tracks, service, cycleway, footway must still reach
    // the GROUND / SAND / MUD rules unchanged.
    // =================================================================

    @Test
    void testNonRoadNetworkDifficultSurfaceUnchanged() {
        List<String> failures = new ArrayList<>();

        // Paths and tracks — go to GROUND (rule 4) for dirt/ground
        assertSurfaceCollect(failures, GROUND, "path + dirt",
                "highway", "path", "surface", "dirt");
        assertSurfaceCollect(failures, GROUND, "path + ground",
                "highway", "path", "surface", "ground");
        assertSurfaceCollect(failures, GROUND, "track + dirt",
                "highway", "track", "surface", "dirt");
        assertSurfaceCollect(failures, GROUND, "track + ground",
                "highway", "track", "surface", "ground");

        // SAND / MUD rules require highway in {path, track}
        assertSurfaceCollect(failures, SAND, "path + sand",
                "highway", "path", "surface", "sand");
        assertSurfaceCollect(failures, SAND, "track + sand",
                "highway", "track", "surface", "sand");
        assertSurfaceCollect(failures, MUD, "path + mud",
                "highway", "path", "surface", "mud");
        assertSurfaceCollect(failures, MUD, "track + mud",
                "highway", "track", "surface", "mud");

        // Service / cycleway / footway — NOT in LIKELY_COMPACT_HIGHWAYS,
        // dirt/ground still go to GROUND, sand/mud unmatched → UNKNOWN
        assertSurfaceCollect(failures, GROUND, "service + dirt",
                "highway", "service", "surface", "dirt");
        assertSurfaceCollect(failures, GROUND, "cycleway + dirt",
                "highway", "cycleway", "surface", "dirt");
        assertSurfaceCollect(failures, GROUND, "footway + ground",
                "highway", "footway", "surface", "ground");

        if (!failures.isEmpty()) {
            fail(failures.size() + " failures:\n  " + String.join("\n  ", failures));
        }
    }

    // =================================================================
    // ITEM #2 — primary + surface=compacted trusts the mapper's positive
    // declaration. All other surfaces on primary still default to ASPHALT.
    // =================================================================

    @Test
    void testPrimaryCompactedException() {
        List<String> failures = new ArrayList<>();

        // The exception: primary + compacted → COMPACTED (was ASPHALT)
        assertSurfaceCollect(failures, COMPACTED, "primary + compacted",
                "highway", "primary", "surface", "compacted");
        assertSurfaceCollect(failures, COMPACTED, "primary_link + compacted",
                "highway", "primary_link", "surface", "compacted");

        // Generic unpaved tags on primary: still ASPHALT (don't trust generic)
        assertSurfaceCollect(failures, ASPHALT, "primary + gravel (don't trust generic)",
                "highway", "primary", "surface", "gravel");
        assertSurfaceCollect(failures, ASPHALT, "primary + unpaved (don't trust generic)",
                "highway", "primary", "surface", "unpaved");
        assertSurfaceCollect(failures, ASPHALT, "primary + dirt (don't trust generic)",
                "highway", "primary", "surface", "dirt");

        // Trunk and motorway: still always ASPHALT, even with compacted (out of scope)
        assertSurfaceCollect(failures, ASPHALT, "trunk + compacted (still ASPHALT, out of scope)",
                "highway", "trunk", "surface", "compacted");
        assertSurfaceCollect(failures, ASPHALT, "motorway + compacted (still ASPHALT)",
                "highway", "motorway", "surface", "compacted");

        // Asphalt-family surfaces on primary: ASPHALT (unchanged)
        assertSurfaceCollect(failures, ASPHALT, "primary + asphalt",
                "highway", "primary", "surface", "asphalt");
        assertSurfaceCollect(failures, ASPHALT, "primary + paved",
                "highway", "primary", "surface", "paved");

        // Primary with no surface: still ASPHALT (default unchanged)
        assertSurfaceCollect(failures, ASPHALT, "primary / no surface",
                "highway", "primary");

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
