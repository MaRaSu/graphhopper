package com.graphhopper.trailmap.shared;

import com.graphhopper.reader.ReaderWay;
import com.graphhopper.routing.ev.ArrayEdgeIntAccess;
import com.graphhopper.routing.ev.EdgeIntAccess;
import com.graphhopper.routing.ev.EncodedValue;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.storage.IntsRef;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static com.graphhopper.trailmap.shared.GravelScale.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

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
