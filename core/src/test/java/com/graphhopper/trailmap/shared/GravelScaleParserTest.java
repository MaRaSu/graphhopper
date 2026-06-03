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

import static com.graphhopper.trailmap.shared.GravelScale.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;

/**
 * Tests for GravelScaleParser — verifies OSM tag combinations map to correct GravelScale values.
 *
 * Test cases derived from track_and_path_gravelscale.md specification.
 * Each assertScale() call documents: expected value, human-readable description, and OSM tags.
 */
class GravelScaleParserTest {

    private GravelScaleParser parser;

    @BeforeEach
    void setUp() {
        parser = new GravelScaleParser(GravelScale.create());
    }

    /**
     * Helper: create a ReaderWay from key-value tag pairs, compute gravel scale, assert result.
     */
    private void assertScale(GravelScale expected, String description, String... tagPairs) {
        ReaderWay way = new ReaderWay(1);
        for (int i = 0; i < tagPairs.length; i += 2) {
            way.setTag(tagPairs[i], tagPairs[i + 1]);
        }
        assertEquals(expected, parser.computeGravelScale(way), description);
    }

    // =================================================================
    // PATH scenarios
    // =================================================================

    @Test
    void testPathBasic() {
        assertScale(FOUR, "bare path",
                "highway", "path");
        assertScale(ZERO_MINUS, "path + asphalt",
                "highway", "path", "surface", "asphalt");
        assertScale(ZERO_MINUS, "path + asphalt + narrow width",
                "highway", "path", "surface", "asphalt", "width", "0.8");
        assertScale(ONE, "path + fine_gravel",
                "highway", "path", "surface", "fine_gravel");
        assertScale(ONE, "path + compacted",
                "highway", "path", "surface", "compacted");
        assertScale(ONE, "path + mtb:scale=0-",
                "highway", "path", "mtb:scale", "0-");
        assertScale(TWO, "path + mtb:scale=0",
                "highway", "path", "mtb:scale", "0");
        assertScale(TWO, "path + mtb:scale=1",
                "highway", "path", "mtb:scale", "1");
        assertScale(FOUR, "path + mtb:scale=2",
                "highway", "path", "mtb:scale", "2");
    }

    @Test
    void testPathWithMtbScaleAndSurface() {
        assertScale(ONE, "path + 0- + fine_gravel",
                "highway", "path", "mtb:scale", "0-", "surface", "fine_gravel");
        assertScale(ONE, "path + 0- + compacted",
                "highway", "path", "mtb:scale", "0-", "surface", "compacted");
        assertScale(ONE, "path + 0 + fine_gravel",
                "highway", "path", "mtb:scale", "0", "surface", "fine_gravel");
        assertScale(ONE, "path + 0 + compacted",
                "highway", "path", "mtb:scale", "0", "surface", "compacted");
        assertScale(TWO, "path + 1 + fine_gravel",
                "highway", "path", "mtb:scale", "1", "surface", "fine_gravel");
        assertScale(TWO, "path + 1 + compacted",
                "highway", "path", "mtb:scale", "1", "surface", "compacted");
        assertScale(FOUR, "path + 2 + fine_gravel",
                "highway", "path", "mtb:scale", "2", "surface", "fine_gravel");
        assertScale(FOUR, "path + 2 + compacted",
                "highway", "path", "mtb:scale", "2", "surface", "compacted");
    }

    @Test
    void testPathWithMtbScaleAndTracktype() {
        assertScale(ONE, "path + 0- + grade1",
                "highway", "path", "mtb:scale", "0-", "tracktype", "grade1");
        assertScale(ONE, "path + 0 + grade1 (was ZERO bug)",
                "highway", "path", "mtb:scale", "0", "tracktype", "grade1");
        assertScale(ONE, "path + 0 + grade2",
                "highway", "path", "mtb:scale", "0", "tracktype", "grade2");
        assertScale(ONE, "path + 0 + grade3",
                "highway", "path", "mtb:scale", "0", "tracktype", "grade3");
        assertScale(TWO, "path + 1 + grade1",
                "highway", "path", "mtb:scale", "1", "tracktype", "grade1");
        assertScale(TWO, "path + 1 + grade2",
                "highway", "path", "mtb:scale", "1", "tracktype", "grade2");
        assertScale(FOUR, "path + 2 + grade1 (was ZERO bug)",
                "highway", "path", "mtb:scale", "2", "tracktype", "grade1");
        assertScale(FOUR, "path + 2 + grade2",
                "highway", "path", "mtb:scale", "2", "tracktype", "grade2");
    }

    @Test
    void testPathWithMtbScaleAndSmoothness() {
        assertScale(ONE, "path + 0- + intermediate",
                "highway", "path", "mtb:scale", "0-", "smoothness", "intermediate");
        assertScale(ONE, "path + 0- + bad",
                "highway", "path", "mtb:scale", "0-", "smoothness", "bad");
        assertScale(ONE, "path + 0 + intermediate",
                "highway", "path", "mtb:scale", "0", "smoothness", "intermediate");
        assertScale(ONE, "path + 0 + bad",
                "highway", "path", "mtb:scale", "0", "smoothness", "bad");
        assertScale(TWO, "path + 1 + intermediate",
                "highway", "path", "mtb:scale", "1", "smoothness", "intermediate");
        assertScale(TWO, "path + 1 + bad",
                "highway", "path", "mtb:scale", "1", "smoothness", "bad");
    }

    @Test
    void testPathWithWidth() {
        // Wide path + good surface → ZERO_PLUS (C2)
        assertScale(ZERO_PLUS, "path + fine_gravel + wide",
                "highway", "path", "surface", "fine_gravel", "width", "3");
        assertScale(ZERO_PLUS, "path + compacted + wide",
                "highway", "path", "surface", "compacted", "width", "3");
        assertScale(ZERO_PLUS, "path + 0 + compacted + wide",
                "highway", "path", "mtb:scale", "0", "surface", "compacted", "width", "3");

        // Narrow path (<1m) capped at TWO (C3)
        assertScale(TWO, "path + fine_gravel + narrow",
                "highway", "path", "surface", "fine_gravel", "width", "0.8");
        assertScale(TWO, "path + compacted + narrow",
                "highway", "path", "surface", "compacted", "width", "0.8");
        assertScale(TWO, "path + 0 + narrow",
                "highway", "path", "mtb:scale", "0", "width", "0.8");
        assertScale(TWO, "path + 0 + compacted + narrow",
                "highway", "path", "mtb:scale", "0", "surface", "compacted", "width", "0.8");
    }

    @Test
    void testPathTracktypeOnly() {
        // C4: path + tracktype as only quality indicator → TWO (unreliable)
        assertScale(TWO, "path + grade1 only",
                "highway", "path", "tracktype", "grade1");
        assertScale(TWO, "path + grade2 only",
                "highway", "path", "tracktype", "grade2");
        assertScale(TWO, "path + grade3 only",
                "highway", "path", "tracktype", "grade3");
        assertScale(TWO, "path + grade4 only",
                "highway", "path", "tracktype", "grade4");
        assertScale(FOUR, "path + grade5 only",
                "highway", "path", "tracktype", "grade5");
    }

    // =================================================================
    // TRACK scenarios
    // =================================================================

    @Test
    void testTrackBasic() {
        assertScale(THREE, "bare track",
                "highway", "track");
        assertScale(ZERO_PLUS, "track + fine_gravel",
                "highway", "track", "surface", "fine_gravel");
        assertScale(ZERO_PLUS, "track + compacted",
                "highway", "track", "surface", "compacted");
        assertScale(THREE, "track + gravel",
                "highway", "track", "surface", "gravel");
        assertScale(ONE, "track + unpaved",
                "highway", "track", "surface", "unpaved");
    }

    @Test
    void testTrackWithTracktype() {
        assertScale(ZERO, "track + grade1",
                "highway", "track", "tracktype", "grade1");
        assertScale(ZERO_PLUS, "track + grade2",
                "highway", "track", "tracktype", "grade2");
        assertScale(ONE, "track + grade3",
                "highway", "track", "tracktype", "grade3");
        assertScale(TWO, "track + grade4",
                "highway", "track", "tracktype", "grade4");
        assertScale(THREE, "track + grade5",
                "highway", "track", "tracktype", "grade5");
    }

    @Test
    void testTrackWithMtbScale() {
        assertScale(ZERO_PLUS, "track + 0-",
                "highway", "track", "mtb:scale", "0-");
        assertScale(ONE, "track + 0",
                "highway", "track", "mtb:scale", "0");
        assertScale(TWO, "track + 1",
                "highway", "track", "mtb:scale", "1");
        assertScale(FOUR, "track + 2",
                "highway", "track", "mtb:scale", "2");
    }

    @Test
    void testTrackWithMtbScaleAndSurface() {
        assertScale(ZERO_PLUS, "track + 0- + fine_gravel",
                "highway", "track", "mtb:scale", "0-", "surface", "fine_gravel");
        assertScale(ZERO_PLUS, "track + 0- + compacted",
                "highway", "track", "mtb:scale", "0-", "surface", "compacted");
        assertScale(ZERO_PLUS, "track + 0 + fine_gravel",
                "highway", "track", "mtb:scale", "0", "surface", "fine_gravel");
        assertScale(ZERO_PLUS, "track + 0 + compacted",
                "highway", "track", "mtb:scale", "0", "surface", "compacted");
        assertScale(TWO, "track + 1 + fine_gravel",
                "highway", "track", "mtb:scale", "1", "surface", "fine_gravel");
        assertScale(TWO, "track + 1 + compacted",
                "highway", "track", "mtb:scale", "1", "surface", "compacted");
        assertScale(FOUR, "track + 2 + fine_gravel",
                "highway", "track", "mtb:scale", "2", "surface", "fine_gravel");
        assertScale(FOUR, "track + 2 + compacted",
                "highway", "track", "mtb:scale", "2", "surface", "compacted");
    }

    @Test
    void testTrackWithMtbScaleAndTracktype() {
        // mtb:scale=0- and 0 don't block ZERO rule
        assertScale(ZERO, "track + 0- + grade1",
                "highway", "track", "mtb:scale", "0-", "tracktype", "grade1");
        assertScale(ZERO, "track + 0 + grade1",
                "highway", "track", "mtb:scale", "0", "tracktype", "grade1");
        // mtb:scale=1+ blocks ZERO rule (B1 bugfix)
        assertScale(TWO, "track + 1 + grade1 (was ZERO bug)",
                "highway", "track", "mtb:scale", "1", "tracktype", "grade1");
        assertScale(FOUR, "track + 2 + grade1 (was ZERO bug)",
                "highway", "track", "mtb:scale", "2", "tracktype", "grade1");
        // grade2 combinations
        assertScale(ZERO_PLUS, "track + 0- + grade2",
                "highway", "track", "mtb:scale", "0-", "tracktype", "grade2");
        assertScale(ZERO_PLUS, "track + 0 + grade2",
                "highway", "track", "mtb:scale", "0", "tracktype", "grade2");
        assertScale(TWO, "track + 1 + grade2",
                "highway", "track", "mtb:scale", "1", "tracktype", "grade2");
        assertScale(FOUR, "track + 2 + grade2",
                "highway", "track", "mtb:scale", "2", "tracktype", "grade2");
    }

    @Test
    void testTrackWithMtbScaleAndSmoothness() {
        assertScale(ZERO_PLUS, "track + 0- + intermediate",
                "highway", "track", "mtb:scale", "0-", "smoothness", "intermediate");
        assertScale(ZERO_PLUS, "track + 0- + bad",
                "highway", "track", "mtb:scale", "0-", "smoothness", "bad");
        assertScale(ZERO_PLUS, "track + 0 + intermediate",
                "highway", "track", "mtb:scale", "0", "smoothness", "intermediate");
        assertScale(ONE, "track + 0 + bad",
                "highway", "track", "mtb:scale", "0", "smoothness", "bad");
        assertScale(TWO, "track + 1 + intermediate",
                "highway", "track", "mtb:scale", "1", "smoothness", "intermediate");
        assertScale(TWO, "track + 1 + bad",
                "highway", "track", "mtb:scale", "1", "smoothness", "bad");
    }

    @Test
    void testTrackSmoothnessRules() {
        // Smoothness alone
        assertScale(ZERO_PLUS, "track + intermediate",
                "highway", "track", "smoothness", "intermediate");
        assertScale(ONE, "track + bad",
                "highway", "track", "smoothness", "bad");
        assertScale(THREE, "track + horrible",
                "highway", "track", "smoothness", "horrible");
        assertScale(FOUR, "track + very_horrible",
                "highway", "track", "smoothness", "very_horrible");

        // C1: smoothness=bad rescued by strong supporting tag
        assertScale(ZERO_PLUS, "track + bad + compacted (C1 rescue)",
                "highway", "track", "smoothness", "bad", "surface", "compacted");
        assertScale(ZERO_PLUS, "track + bad + fine_gravel (C1 rescue)",
                "highway", "track", "smoothness", "bad", "surface", "fine_gravel");
        assertScale(ZERO_PLUS, "track + bad + 0- (C1 rescue)",
                "highway", "track", "smoothness", "bad", "mtb:scale", "0-");
        // grade2 is NOT a strong tag — no rescue
        assertScale(ONE, "track + bad + grade2 (no rescue)",
                "highway", "track", "smoothness", "bad", "tracktype", "grade2");
    }

    // =================================================================
    // TRACK surface/tracktype combinations
    // =================================================================

    /**
     * Helper: collect mismatches instead of failing immediately, then report all at once.
     */
    private void assertScaleCollect(List<String> failures, GravelScale expected, String description, String... tagPairs) {
        ReaderWay way = new ReaderWay(1);
        for (int i = 0; i < tagPairs.length; i += 2) {
            way.setTag(tagPairs[i], tagPairs[i + 1]);
        }
        GravelScale actual = parser.computeGravelScale(way);
        if (actual != expected) {
            failures.add(description + " ==> expected: " + expected + " but was: " + actual);
        }
    }

    @Test
    void testTrackSurfaceTracktypeCombinations() {
        List<String> failures = new ArrayList<>();

        // #1: bare track → THREE
        assertScaleCollect(failures, THREE, "#1 bare track",
                "highway", "track");

        // #2: track + dirt → THREE
        assertScaleCollect(failures, THREE, "#2 track + dirt",
                "highway", "track", "surface", "dirt");

        // #3: track + dirt + grade5 → THREE
        assertScaleCollect(failures, THREE, "#3 track + dirt + grade5",
                "highway", "track", "surface", "dirt", "tracktype", "grade5");

        // #4: track + grade5 (no surface) → THREE
        assertScaleCollect(failures, THREE, "#4 track + grade5",
                "highway", "track", "tracktype", "grade5");

        // #5: track + ground → THREE
        assertScaleCollect(failures, THREE, "#5 track + ground",
                "highway", "track", "surface", "ground");

        // #6: track + ground + grade4 → TWO
        assertScaleCollect(failures, TWO, "#6 track + ground + grade4",
                "highway", "track", "surface", "ground", "tracktype", "grade4");

        // #7: track + ground + grade3 → ONE
        assertScaleCollect(failures, ONE, "#7 track + ground + grade3",
                "highway", "track", "surface", "ground", "tracktype", "grade3");

        // #8: track + gravel → THREE
        assertScaleCollect(failures, THREE, "#8 track + gravel",
                "highway", "track", "surface", "gravel");

        // #9: track + gravel + grade5 → THREE
        assertScaleCollect(failures, THREE, "#9 track + gravel + grade5",
                "highway", "track", "surface", "gravel", "tracktype", "grade5");

        // #10: track + unpaved → ONE
        assertScaleCollect(failures, ONE, "#10 track + unpaved",
                "highway", "track", "surface", "unpaved");

        // #11: track + unpaved + grade5 → TWO
        assertScaleCollect(failures, TWO, "#11 track + unpaved + grade5",
                "highway", "track", "surface", "unpaved", "tracktype", "grade5");

        // #11a: track + unpaved + grade1 → ZERO (grade1 dominates)
        assertScaleCollect(failures, ZERO, "#11a track + unpaved + grade1",
                "highway", "track", "surface", "unpaved", "tracktype", "grade1");

        // #11b: track + unpaved + grade2 → ZERO_PLUS (grade2 dominates)
        assertScaleCollect(failures, ZERO_PLUS, "#11b track + unpaved + grade2",
                "highway", "track", "surface", "unpaved", "tracktype", "grade2");

        // #12: track + horrible smoothness → THREE
        assertScaleCollect(failures, THREE, "#12 track + horrible",
                "highway", "track", "smoothness", "horrible");

        // #13: track + very_horrible smoothness → FOUR
        assertScaleCollect(failures, FOUR, "#13 track + very_horrible",
                "highway", "track", "smoothness", "very_horrible");

        // #14: track + dirt + horrible smoothness → THREE
        assertScaleCollect(failures, THREE, "#14 track + dirt + horrible",
                "highway", "track", "surface", "dirt", "smoothness", "horrible");

        // #15: track + grade5 + horrible smoothness → THREE
        assertScaleCollect(failures, THREE, "#15 track + grade5 + horrible",
                "highway", "track", "tracktype", "grade5", "smoothness", "horrible");

        // #16: track + mud → FOUR
        assertScaleCollect(failures, FOUR, "#16 track + mud",
                "highway", "track", "surface", "mud");

        // #17: track + mud + grade5 → FOUR
        assertScaleCollect(failures, FOUR, "#17 track + mud + grade5",
                "highway", "track", "surface", "mud", "tracktype", "grade5");

        if (!failures.isEmpty()) {
            fail(failures.size() + " failures:\n  " + String.join("\n  ", failures));
        }
    }

    // =================================================================
    // ROAD NETWORK — secondary (boundary class: can be paved or unpaved)
    //
    // Secondary defaults to ZERO_PLUS (not ZERO_MINUS) when no surface is
    // tagged — globally ~95% paved but in Nordic / rural regions genuinely
    // mixed, so defaulting to paved is unsafe for routing. Explicit asphalt
    // surface still produces ZERO_MINUS. Explicit unpaved-ish surfaces map
    // to ZERO_PLUS (mappers overload `gravel`; on the road network it means
    // compacted-quality).
    // =================================================================

    @Test
    void testSecondaryRoadNetwork() {
        List<String> failures = new ArrayList<>();

        // Original triggering case
        assertScaleCollect(failures, ZERO_PLUS, "secondary + gravel (original case)",
                "highway", "secondary", "surface", "gravel");

        // The "massive error" case
        assertScaleCollect(failures, ZERO_PLUS, "secondary / no surface (boundary default)",
                "highway", "secondary");
        assertScaleCollect(failures, ZERO_PLUS, "secondary_link / no surface",
                "highway", "secondary_link");

        // Explicit paved surfaces — still ZERO_MINUS
        assertScaleCollect(failures, ZERO_MINUS, "secondary + asphalt",
                "highway", "secondary", "surface", "asphalt");
        assertScaleCollect(failures, ZERO_MINUS, "secondary + paved",
                "highway", "secondary", "surface", "paved");
        assertScaleCollect(failures, ZERO_MINUS, "secondary + concrete",
                "highway", "secondary", "surface", "concrete");
        assertScaleCollect(failures, ZERO_MINUS, "secondary + paving_stones",
                "highway", "secondary", "surface", "paving_stones");

        // Explicit unpaved-ish — ZERO_PLUS
        assertScaleCollect(failures, ZERO_PLUS, "secondary + compacted",
                "highway", "secondary", "surface", "compacted");
        assertScaleCollect(failures, ZERO_PLUS, "secondary + fine_gravel",
                "highway", "secondary", "surface", "fine_gravel");
        assertScaleCollect(failures, ZERO_PLUS, "secondary + unpaved",
                "highway", "secondary", "surface", "unpaved");
        assertScaleCollect(failures, ZERO_PLUS, "secondary_link + gravel",
                "highway", "secondary_link", "surface", "gravel");

        if (!failures.isEmpty()) {
            fail(failures.size() + " failures:\n  " + String.join("\n  ", failures));
        }
    }

    // =================================================================
    // ROAD NETWORK — regression guard for other classes
    // (tertiary_link added to LIKELY_COMPACT_HIGHWAYS for parity; verify
    //  the existing tertiary/unclassified/residential behaviors are intact)
    // =================================================================

    @Test
    void testRoadNetworkOtherClassesUnchanged() {
        List<String> failures = new ArrayList<>();

        // Higher tier — still always ZERO_MINUS
        assertScaleCollect(failures, ZERO_MINUS, "motorway",
                "highway", "motorway");
        assertScaleCollect(failures, ZERO_MINUS, "trunk",
                "highway", "trunk");
        assertScaleCollect(failures, ZERO_MINUS, "primary",
                "highway", "primary");
        assertScaleCollect(failures, ZERO_MINUS, "primary_link",
                "highway", "primary_link");

        // Tertiary + gravel — long-standing behavior, must still be ZERO_PLUS
        assertScaleCollect(failures, ZERO_PLUS, "tertiary + gravel",
                "highway", "tertiary", "surface", "gravel");
        assertScaleCollect(failures, ZERO_PLUS, "unclassified + gravel",
                "highway", "unclassified", "surface", "gravel");
        assertScaleCollect(failures, ZERO_PLUS, "residential + gravel",
                "highway", "residential", "surface", "gravel");

        // tertiary_link — newly included, should behave like tertiary
        assertScaleCollect(failures, ZERO_PLUS, "tertiary_link + gravel",
                "highway", "tertiary_link", "surface", "gravel");

        if (!failures.isEmpty()) {
            fail(failures.size() + " failures:\n  " + String.join("\n  ", failures));
        }
    }

    // =================================================================
    // ROAD NETWORK — dirt / ground / sand / mud clamp (rule 1b)
    // Per agreed business logic, lower-tier road-network highways with
    // these surfaces must clamp to ZERO_PLUS instead of falling through
    // to rule 11 FOUR. Implemented as an early clamp rule (rule 1b)
    // before the FOUR rule's catch-all and before exclusion logic in
    // rules 4-9 can prevent classification.
    // =================================================================

    @Test
    void testRoadNetworkDifficultSurfaceClamp() {
        List<String> failures = new ArrayList<>();

        for (String hw : new String[]{
                "secondary", "secondary_link", "tertiary", "tertiary_link",
                "unclassified", "residential"}) {
            assertScaleCollect(failures, ZERO_PLUS, hw + " + dirt",
                    "highway", hw, "surface", "dirt");
            assertScaleCollect(failures, ZERO_PLUS, hw + " + ground",
                    "highway", hw, "surface", "ground");
            assertScaleCollect(failures, ZERO_PLUS, hw + " + sand",
                    "highway", hw, "surface", "sand");
            assertScaleCollect(failures, ZERO_PLUS, hw + " + mud",
                    "highway", hw, "surface", "mud");
        }

        if (!failures.isEmpty()) {
            fail(failures.size() + " failures:\n  " + String.join("\n  ", failures));
        }
    }

    // =================================================================
    // REGRESSION GUARD — non-road-network classes must keep prior behavior
    // The new early clamp only fires for LIKELY_COMPACT_HIGHWAYS. All other
    // highway classes must reach the same rule they did before.
    // =================================================================

    @Test
    void testNonRoadNetworkDifficultSurfaceUnchanged() {
        List<String> failures = new ArrayList<>();

        // Paths — rule 11 FOUR catches via "path" branch
        assertScaleCollect(failures, FOUR, "path + dirt",
                "highway", "path", "surface", "dirt");
        assertScaleCollect(failures, FOUR, "path + ground",
                "highway", "path", "surface", "ground");
        assertScaleCollect(failures, FOUR, "path + sand",
                "highway", "path", "surface", "sand");
        assertScaleCollect(failures, FOUR, "path + mud",
                "highway", "path", "surface", "mud");

        // Tracks — rule 10 (THREE) catches dirt/ground/gravel; rule 11 catches mud/sand
        assertScaleCollect(failures, THREE, "track + dirt",
                "highway", "track", "surface", "dirt");
        assertScaleCollect(failures, THREE, "track + ground",
                "highway", "track", "surface", "ground");
        assertScaleCollect(failures, FOUR, "track + mud",
                "highway", "track", "surface", "mud");
        assertScaleCollect(failures, FOUR, "track + sand",
                "highway", "track", "surface", "sand");

        // Service — currently rule 7 "service" branch fires for dirt/ground (ONE),
        // rule 11 catches mud/sand (FOUR). Service is NOT in LIKELY_COMPACT_HIGHWAYS.
        assertScaleCollect(failures, ONE, "service + dirt",
                "highway", "service", "surface", "dirt");
        assertScaleCollect(failures, ONE, "service + ground",
                "highway", "service", "surface", "ground");

        if (!failures.isEmpty()) {
            fail(failures.size() + " failures:\n  " + String.join("\n  ", failures));
        }
    }

    // =================================================================
    // ITEM #2 — primary + compacted (no code change needed in this parser)
    //
    // gravel_scale's rule 2 already has an outer guard `&& !matchesUnpavedSurface`
    // where UNPAVED_SURFACES = {unpaved, compacted, gravel, fine_gravel}. So any
    // motorway/trunk/primary tagged with any of those four surfaces already skips
    // rule 2 and falls through to later rules. compacted → ZERO_PLUS via rule 4's
    // dedicated check. gravel/unpaved → UNKNOWN (conservative fall-through).
    //
    // These tests pin the behavior so future edits don't regress it.
    // =================================================================

    @Test
    void testPrimaryCompactedException() {
        List<String> failures = new ArrayList<>();

        // primary/primary_link + compacted → ZERO_PLUS (already correct via existing guard)
        assertScaleCollect(failures, ZERO_PLUS, "primary + compacted",
                "highway", "primary", "surface", "compacted");
        assertScaleCollect(failures, ZERO_PLUS, "primary_link + compacted",
                "highway", "primary_link", "surface", "compacted");

        // trunk/motorway + compacted also fall through to ZERO_PLUS (same guard)
        assertScaleCollect(failures, ZERO_PLUS, "trunk + compacted",
                "highway", "trunk", "surface", "compacted");
        assertScaleCollect(failures, ZERO_PLUS, "motorway + compacted",
                "highway", "motorway", "surface", "compacted");

        // Generic unpaved tags on primary → UNKNOWN (conservative — pre-existing
        // gravel_scale behavior; asymmetric with predicted_surface where these
        // remain ASPHALT. Not addressed in this scope.)
        assertScaleCollect(failures, UNKNOWN, "primary + gravel (asymmetry: PS returns ASPHALT)",
                "highway", "primary", "surface", "gravel");
        assertScaleCollect(failures, UNKNOWN, "primary + unpaved (same asymmetry)",
                "highway", "primary", "surface", "unpaved");

        // Asphalt-family surfaces on primary: ZERO_MINUS (unchanged)
        assertScaleCollect(failures, ZERO_MINUS, "primary + asphalt",
                "highway", "primary", "surface", "asphalt");
        assertScaleCollect(failures, ZERO_MINUS, "primary + paved",
                "highway", "primary", "surface", "paved");

        // Primary with no surface: ZERO_MINUS (default unchanged)
        assertScaleCollect(failures, ZERO_MINUS, "primary / no surface",
                "highway", "primary");

        if (!failures.isEmpty()) {
            fail(failures.size() + " failures:\n  " + String.join("\n  ", failures));
        }
    }

    // =================================================================
    // Encoded value pipeline integration
    // =================================================================

    @Test
    void testEncodedValuePipeline() {
        EnumEncodedValue<GravelScale> enc = GravelScale.create();
        enc.init(new EncodedValue.InitializerConfig());
        GravelScaleParser fullParser = new GravelScaleParser(enc);
        EdgeIntAccess access = new ArrayEdgeIntAccess(1);
        IntsRef relFlags = new IntsRef(2);

        ReaderWay way = new ReaderWay(1);
        way.setTag("highway", "path");
        fullParser.handleWayTags(0, access, way, relFlags);
        assertEquals(FOUR, enc.getEnum(false, 0, access), "pipeline: bare path");

        // Reset for second test
        access = new ArrayEdgeIntAccess(1);
        way = new ReaderWay(2);
        way.setTag("highway", "track");
        way.setTag("tracktype", "grade1");
        fullParser.handleWayTags(0, access, way, relFlags);
        assertEquals(ZERO, enc.getEnum(false, 0, access), "pipeline: track + grade1");
    }
}
