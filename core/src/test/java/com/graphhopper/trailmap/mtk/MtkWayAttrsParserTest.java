package com.graphhopper.trailmap.mtk;

import com.graphhopper.reader.ReaderWay;
import com.graphhopper.routing.ev.*;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.util.parsers.TagParser;
import com.graphhopper.storage.IntsRef;
import com.graphhopper.trailmap.shared.*;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.PMap;
import com.graphhopper.util.PointList;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;
import java.util.zip.GZIPOutputStream;

import static org.junit.jupiter.api.Assertions.*;

/**
 * MTK overlay (alternative A) on fixture ways, against today's Trailmap parsers. Each way is "read" as
 * GraphHopper's OSMReader hands it to the parsers: one call per edge, in way order, with the artificial tags
 * edge_distance and point_list set. Fixture: mtk_way_attrs_fixture.csv next to this class (24 rows: every
 * class and paving value, multi-piece ways, a piece gap, a stale hash, hazard-tagged and OSM-tagged ways).
 *
 * The owner's mapping (way-network cases, "Enrichment rule (decided)", 2026-11-02) is written out here as
 * literal expected values, never computed with the unit's own code.
 */
class MtkWayAttrsParserTest {

    /** The fixture ways' node ids and their SHA-1 prefix, computed with `printf '101,...,107' | shasum -a 1`. */
    static final long[] NODES = {101, 102, 103, 104, 105, 106, 107};
    static final String NODES_HASH = "8e21a6077f286951";

    @TempDir
    static Path tmp;
    static Path fixtureGz;

    @BeforeAll
    static void writeFixture() throws IOException {
        fixtureGz = tmp.resolve("mtk_way_attrs_finland_latest.csv.gz");
        try (InputStream in = MtkWayAttrsParserTest.class.getResourceAsStream("mtk_way_attrs_fixture.csv");
             OutputStream out = new GZIPOutputStream(Files.newOutputStream(fixtureGz))) {
            assertNotNull(in, "fixture resource");
            in.transferTo(out);
        }
    }

    // ------------------------------------------------------------------------------------------------
    // harness: an encoding manager with today's units, their parsers in GraphHopper's order, + the overlay
    // ------------------------------------------------------------------------------------------------

    static final class Run {
        final EncodingManager em;
        final List<TagParser> parsers = new ArrayList<>();
        final MtkWayAttrsParser mtk;
        final EdgeIntAccess eia;
        int nextEdge = 0;
        double lat = 60.0;

        Run(String sidecarFile, PMap props) {
            TrailmapImportRegistry reg = new TrailmapImportRegistry();
            Map<String, ImportUnit> units = new LinkedHashMap<>();
            ArrayDeque<String> q = new ArrayDeque<>(Arrays.asList(MtkWayAttrsParser.REQUIRED));
            while (!q.isEmpty()) {
                String n = q.removeFirst();
                ImportUnit u = reg.createImportUnit(n);
                if (units.put(n, u) == null) q.addAll(u.getRequiredImportUnits());
            }
            EncodingManager.Builder b = new EncodingManager.Builder();
            units.values().forEach(u -> {
                if (u.getCreateEncodedValue() != null) b.add(u.getCreateEncodedValue().apply(new PMap()));
            });
            em = b.build();
            for (String n : new ImportUnitSorter(units).sort())
                if (units.get(n).getCreateTagParser() != null)
                    parsers.add(units.get(n).getCreateTagParser().apply(em, new PMap()));
            if (sidecarFile != null) {
                props.putObject("file", sidecarFile);
                mtk = MtkWayAttrsParser.create(em, props);
                parsers.add(mtk); // REQUIRED makes GraphHopper sort the overlay after these
            } else {
                mtk = null;
            }
            eia = ArrayEdgeIntAccess.createFromBytes(em.getBytesForFlags());
        }

        Run(String sidecarFile) {
            this(sidecarFile, new PMap());
        }

        /** Reads one way: one edge per length (m), consecutive points along a meridian. Returns the edge ids. */
        int[] read(ReaderWay way, double... edgeLens) {
            int[] ids = new int[edgeLens.length];
            for (int i = 0; i < edgeLens.length; i++)
                ids[i] = edge(way, edgeLens[i], 0);
            return ids;
        }

        /** One edge of {@code way}, starting {@code gapM} metres after the previous edge's end. */
        int edge(ReaderWay way, double lenM, double gapM) {
            lat += gapM / 111_195.0;
            PointList pl = new PointList(2, false);
            pl.add(lat, 24.0);
            lat += lenM / 111_195.0;
            pl.add(lat, 24.0);
            double d = Math.max(0.001, DistanceCalcEarth.DIST_EARTH.calcDistance(pl));
            way.setTag("point_list", pl);
            way.setTag("edge_distance", d);
            int id = nextEdge++;
            for (TagParser p : parsers)
                p.handleWayTags(id, eia, way, new IntsRef(2));
            return id;
        }

        /** Ends the way being read, as GraphHopper's next way (or postImportOSM) does: the unit writes its edges then. */
        void endWay() {
            if (mtk != null) mtk.finishWay();
        }

        GravelScale gravel(int e) {
            endWay();
            return em.getEnumEncodedValue(GravelScale.KEY, GravelScale.class).getEnum(false, e, eia);
        }

        PredictedSurface surface(int e) {
            endWay();
            return em.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class).getEnum(false, e, eia);
        }

        MtbScale mtb(int e) {
            endWay();
            return em.getEnumEncodedValue(MtbScale.KEY, MtbScale.class).getEnum(false, e, eia);
        }

        boolean unknownTrack(int e) {
            endWay();
            return em.getBooleanEncodedValue(RouteIssues.KEY_UNKNOWN_TRACK).getBool(false, e, eia);
        }

        /** Every encoded value of edge e, by name. */
        Map<String, Object> all(int e) {
            endWay();
            Map<String, Object> m = new TreeMap<>();
            for (EncodedValue ev : em.getEncodedValues()) {
                Object v;
                if (ev instanceof EnumEncodedValue) v = ((EnumEncodedValue<?>) ev).getEnum(false, e, eia);
                else if (ev instanceof BooleanEncodedValue) v = ((BooleanEncodedValue) ev).getBool(false, e, eia);
                else if (ev instanceof DecimalEncodedValue) v = ((DecimalEncodedValue) ev).getDecimal(false, e, eia);
                else if (ev instanceof IntEncodedValue) v = ((IntEncodedValue) ev).getInt(false, e, eia);
                else v = "?";
                m.put(ev.getName(), v);
            }
            return m;
        }
    }

    static ReaderWay way(long id, String... tags) {
        ReaderWay w = new ReaderWay(id);
        for (long n : NODES) w.getNodes().add(n);
        for (int i = 0; i < tags.length; i += 2) w.setTag(tags[i], tags[i + 1]);
        return w;
    }

    /** Reads the way with and without the overlay; returns [withMtk, today] value maps per edge. */
    static List<Map<String, Object>[]> compare(PMap props, long id, String... tags) {
        Run on = new Run(fixtureGz.toString(), props);
        Run off = new Run(null);
        int[] a = on.read(way(id, tags), 100, 100, 100);
        int[] b = off.read(way(id, tags), 100, 100, 100);
        List<Map<String, Object>[]> r = new ArrayList<>();
        for (int i = 0; i < a.length; i++) {
            @SuppressWarnings("unchecked")
            Map<String, Object>[] pair = new Map[]{on.all(a[i]), off.all(b[i])};
            r.add(pair);
        }
        return r;
    }

    static List<Map<String, Object>[]> compare(long id, String... tags) {
        return compare(new PMap(), id, tags);
    }

    static void assertUnchanged(long id, String... tags) {
        for (Map<String, Object>[] p : compare(id, tags))
            assertEquals(p[1], p[0], "way " + id + " must keep today's values");
    }

    /** Only the named keys may differ, and they must equal the expected values. */
    static void assertOnly(Map<String, Object>[] p, Map<String, Object> expected) {
        for (String k : p[1].keySet()) {
            Object want = expected.containsKey(k) ? expected.get(k) : p[1].get(k);
            assertEquals(want, p[0].get(k), k);
        }
    }

    /**
     * The values the mapping writes on a track edge, taken from today's parsers on reference ways that get the
     * same scale from OSM tags (so the derived numeric/priority values are what an OSM-tagged edge stores):
     * gravel 1 = an unnamed service road (rule 7), gravel 2 = an untagged "-tie" track (rule 9),
     * mtb 0 = an untagged track.
     */
    static Map<String, Object> track(GravelScale g) {
        Run ref = new Run(null);
        ReaderWay gravelRef = g == GravelScale.ONE ? way(9, "highway", "service")
                : way(9, "highway", "track", "name", "Kuusitie");
        Map<String, Object> gv = ref.all(ref.read(gravelRef, 100)[0]);
        assertEquals(g, gv.get(GravelScale.KEY), "reference way");
        Map<String, Object> mv = ref.all(ref.read(way(10, "highway", "track"), 100)[0]);
        assertEquals(MtbScale.ZERO, mv.get(MtbScale.KEY), "reference way");
        Map<String, Object> m = new HashMap<>();
        m.put(GravelScale.KEY, g);
        m.put(GravelScaleNum.KEY, gv.get(GravelScaleNum.KEY));
        m.put(GravelBasePriority.KEY, gv.get(GravelBasePriority.KEY));
        m.put(MtbScale.KEY, MtbScale.ZERO);
        m.put(MtbScaleNum.KEY, mv.get(MtbScaleNum.KEY));
        m.put(MtbBasePriority.KEY, mv.get(MtbBasePriority.KEY));
        m.put(RouteIssues.KEY_UNKNOWN_TRACK, false);
        return m;
    }

    // ------------------------------------------------------------------------------------------------
    // T: untagged tracks
    // ------------------------------------------------------------------------------------------------

    @Test
    void t1_ajotie_isGravelOne_mtbZero_flagCleared() {
        List<Map<String, Object>[]> r = compare(1001, "highway", "track");
        assertEquals(GravelScale.THREE, r.get(0)[1].get(GravelScale.KEY), "today");
        assertEquals(true, r.get(0)[1].get(RouteIssues.KEY_UNKNOWN_TRACK), "today");
        for (Map<String, Object>[] p : r)
            assertOnly(p, track(GravelScale.ONE));
    }

    @Test
    void t1_ajopolku_isGravelTwo_flagCleared() {
        for (Map<String, Object>[] p : compare(1002, "highway", "track"))
            assertOnly(p, track(GravelScale.TWO));
    }

    @Test
    void t1_talvitie_polku_noClass_unchanged() {
        assertUnchanged(1003, "highway", "track");
        assertUnchanged(1004, "highway", "track");
        assertUnchanged(1005, "highway", "track");
    }

    @Test
    void t2_tieName_followsClass_onAjotie() {
        List<Map<String, Object>[]> r = compare(1006, "highway", "track", "name", "Metsätie");
        assertEquals(GravelScale.TWO, r.get(0)[1].get(GravelScale.KEY), "today: rule 9");
        for (Map<String, Object>[] p : r)
            assertOnly(p, track(GravelScale.ONE));
    }

    @Test
    void t2_tieName_onTalvitie_keepsRule9() {
        List<Map<String, Object>[]> r = compare(1007, "highway", "track", "name", "Kangastie");
        assertEquals(GravelScale.TWO, r.get(0)[0].get(GravelScale.KEY));
        assertUnchanged(1007, "highway", "track", "name", "Kangastie");
    }

    @Test
    void t3_obstacle_and_t4_trailVisibility_hazardSkip() {
        assertUnchanged(1008, "highway", "track", "obstacle", "vegetation");
        assertUnchanged(1009, "highway", "track", "trail_visibility", "good");
        assertUnchanged(1008, "highway", "track", "obstacle", "fallen_tree");
    }

    @Test
    void hazardSkip_canBeSwitchedOff() {
        PMap props = new PMap().putObject("skip_hazard", false);
        for (Map<String, Object>[] p : compare(props, 1009, "highway", "track", "trail_visibility", "good"))
            assertEquals(GravelScale.ONE, p[0].get(GravelScale.KEY));
    }

    @Test
    void osmWins_onTaggedTracksAndAreas() {
        assertUnchanged(1010, "highway", "track", "tracktype", "grade3");
        assertUnchanged(1010, "highway", "track", "surface", "gravel");
        assertUnchanged(1010, "highway", "track", "smoothness", "bad");
        assertUnchanged(1010, "highway", "track", "mtb:scale", "1");
        assertUnchanged(1001, "highway", "track", "area", "yes");
    }

    // ------------------------------------------------------------------------------------------------
    // R: minor roads without surface — "as if surface=asphalt / unpaved were tagged"
    // ------------------------------------------------------------------------------------------------

    /** The overlay's road edges must equal today's parsers on the same way with the surface really tagged. */
    static void assertAsIfTagged(long id, String surface, String... tags) {
        List<String> tagged = new ArrayList<>(Arrays.asList(tags));
        tagged.add("surface");
        tagged.add(surface);
        Run on = new Run(fixtureGz.toString());
        Run off = new Run(null);
        Run real = new Run(null);
        int[] a = on.read(way(id, tags), 100, 100, 100);
        int[] b = off.read(way(id, tags), 100, 100, 100);
        int[] c = real.read(way(id, tagged.toArray(new String[0])), 100, 100, 100);
        Set<String> changes = Set.of(GravelScale.KEY, GravelScaleNum.KEY, GravelBasePriority.KEY, PredictedSurface.KEY);
        for (int i = 0; i < a.length; i++) {
            Map<String, Object> mOn = on.all(a[i]), mOff = off.all(b[i]), mReal = real.all(c[i]);
            for (String k : mOff.keySet())
                assertEquals(changes.contains(k) ? mReal.get(k) : mOff.get(k), mOn.get(k), "way " + id + " " + k);
        }
    }

    @Test
    void r_paved_isAsphalt() {
        Run on = new Run(fixtureGz.toString());
        int[] e = on.read(way(2001, "highway", "unclassified"), 100, 100, 100);
        assertEquals(GravelScale.ZERO_MINUS, on.gravel(e[0]));
        assertEquals(PredictedSurface.ASPHALT, on.surface(e[0]));
        assertAsIfTagged(2001, "asphalt", "highway", "unclassified");
    }

    @Test
    void r_unpaved_isCompacted_zeroPlus() {
        Run on = new Run(fixtureGz.toString());
        int[] e = on.read(way(2002, "highway", "residential"), 100, 100, 100);
        assertEquals(GravelScale.ZERO_PLUS, on.gravel(e[0]));
        assertEquals(PredictedSurface.COMPACTED, on.surface(e[0]));
        assertAsIfTagged(2002, "unpaved", "highway", "residential");
    }

    @Test
    void r_unpavedService_isOne() {
        Run on = new Run(fixtureGz.toString());
        int[] e = on.read(way(2003, "highway", "service"), 100, 100, 100);
        assertEquals(GravelScale.ONE, on.gravel(e[0]));
        assertEquals(PredictedSurface.COMPACTED, on.surface(e[0]));
        assertAsIfTagged(2003, "unpaved", "highway", "service");
    }

    @Test
    void r_noPaving_osmSurface_hazard_otherClass_unchanged() {
        assertUnchanged(2004, "highway", "tertiary");
        assertUnchanged(2005, "highway", "unclassified", "surface", "gravel");
        assertUnchanged(2006, "highway", "residential", "obstacle", "vegetation");
        assertUnchanged(2006, "highway", "residential", "trail_visibility", "excellent");
        assertUnchanged(3001, "highway", "primary");
        assertUnchanged(3001, "highway", "path");
        // a road row on a track: the track case reads the class, not the paving
        for (Map<String, Object>[] p : compare(2001, "highway", "track"))
            assertOnly(p, track(GravelScale.ONE));
    }

    @Test
    void r_allSelectorClasses() {
        for (String hw : MtkWayAttrsParser.ROAD_CLASSES)
            assertAsIfTagged(2001, "asphalt", "highway", hw);
        assertEquals(Set.of("secondary", "secondary_link", "tertiary", "tertiary_link", "unclassified",
                "residential", "service"), MtkWayAttrsParser.ROAD_CLASSES);
    }

    // ------------------------------------------------------------------------------------------------
    // pieces, staleness, edge order
    // ------------------------------------------------------------------------------------------------

    @Test
    void multiPiece_midpointPicksThePiece() {
        // pieces 0–0.5 ajotie, 0.5–1 polku; edges 100, 110, 90 m: midpoints 0.167, 0.517, 0.85
        Run on = new Run(fixtureGz.toString());
        int[] e = on.read(way(1011, "highway", "track"), 100, 110, 90);
        assertEquals(GravelScale.ONE, on.gravel(e[0]));
        assertEquals(GravelScale.THREE, on.gravel(e[1]), "midpoint 0.517 is in the second piece");
        assertEquals(GravelScale.THREE, on.gravel(e[2]));
        assertFalse(on.unknownTrack(e[0]));
        assertTrue(on.unknownTrack(e[1]));

        // an edge across the boundary: 0–220 m has its midpoint at 0.367 → first piece
        on = new Run(fixtureGz.toString());
        e = on.read(way(1011, "highway", "track"), 220, 80);
        assertEquals(GravelScale.ONE, on.gravel(e[0]));
        assertEquals(GravelScale.THREE, on.gravel(e[1]));
        // 0–80, 80–300: midpoints 0.133 and 0.633
        on = new Run(fixtureGz.toString());
        e = on.read(way(1011, "highway", "track"), 80, 220);
        assertEquals(GravelScale.ONE, on.gravel(e[0]));
        assertEquals(GravelScale.THREE, on.gravel(e[1]));
    }

    @Test
    void multiPiece_unmatchedStretchUnchanged() {
        // pieces 0–0.3 ajotie, 0.7–1 ajopolku; the middle edge (0.5) is in no piece
        Run on = new Run(fixtureGz.toString());
        int[] e = on.read(way(1012, "highway", "track"), 100, 100, 100);
        assertEquals(GravelScale.ONE, on.gravel(e[0]));
        assertEquals(GravelScale.THREE, on.gravel(e[1]));
        assertTrue(on.unknownTrack(e[1]));
        assertEquals(GravelScale.TWO, on.gravel(e[2]));
        assertEquals(1, on.mtk.stats.edgesNoPiece);
    }

    @Test
    void multiPiece_roadPavedThenUnpaved() {
        // 0–0.4 paved, 0.4–1 unpaved
        Run on = new Run(fixtureGz.toString());
        int[] e = on.read(way(2007, "highway", "unclassified"), 100, 100, 100);
        assertEquals(PredictedSurface.ASPHALT, on.surface(e[0]));
        assertEquals(PredictedSurface.COMPACTED, on.surface(e[1]));
        assertEquals(PredictedSurface.COMPACTED, on.surface(e[2]));
    }

    @Test
    void barrierEdge_zeroLength_keepsOffsets() {
        // 0–140, a 0 m barrier edge at 140, 140–300: midpoints 0.233, 0.467, 0.733 → ajotie, ajotie, polku
        Run on = new Run(fixtureGz.toString());
        ReaderWay w = way(1011, "highway", "track");
        int a = on.edge(w, 140, 0);
        int barrier = on.edge(w, 0, 0);
        int c = on.edge(w, 160, 0);
        assertEquals(GravelScale.ONE, on.gravel(a));
        assertEquals(GravelScale.ONE, on.gravel(barrier));
        assertEquals(GravelScale.THREE, on.gravel(c));
    }

    @Test
    void missingNodesGap_isAddedToTheOffset() {
        // 0–100 m, then 120 m of the way's nodes missing (an extract boundary), then an 80 m edge: its midpoint is
        // at 260 m (0.867, polku); without counting the gap it would be at 140 m (0.467, ajotie), and the way
        // would sum to 180 m of its 300 m and be dropped as stale_length.
        Run on = new Run(fixtureGz.toString());
        ReaderWay w = way(1011, "highway", "track");
        int a = on.edge(w, 100, 0);
        int b = on.edge(w, 80, 120);
        assertEquals(GravelScale.ONE, on.gravel(a));
        assertEquals(GravelScale.THREE, on.gravel(b));
        assertEquals(0, on.mtk.stats.waysStaleLength);
    }

    @Test
    void elevation_edgesArePositionedIn2D_notByThe3DEdgeDistance() {
        // As GraphHopper with elevation: point_list is 3D and edge_distance is its 3D length. Pieces 0–0.5 ajotie,
        // 0.5–1 polku (way_len_m 300, 2D). Edge 1 climbs 96 m over 140 m (3D length 169.8 m), edge 2 is 10 m flat,
        // edge 3 150 m flat. In 2D edge 2's midpoint is at 145 m (0.483, ajotie); by the summed 3D edge_distance
        // it would be at 174.8 m (0.583, polku).
        Run on = new Run(fixtureGz.toString());
        ReaderWay w = way(1011, "highway", "track");
        int a = edge3D(on, w, 140, 0, 96);
        int b = edge3D(on, w, 10, 96, 96);
        int c = edge3D(on, w, 150, 96, 96);
        assertEquals(GravelScale.ONE, on.gravel(a));
        assertEquals(GravelScale.ONE, on.gravel(b), "2D midpoint 0.483 is in the first piece");
        assertEquals(GravelScale.THREE, on.gravel(c));
        on.mtk.logSummary();
        assertEquals(1.0, on.mtk.stats.lenRatioMin, 1e-3, "summed 2D length / way_len_m");
        assertEquals(1.0, on.mtk.stats.lenRatioMax, 1e-3, "summed 2D length / way_len_m");
    }

    /** One edge of {@code way} along a meridian with elevation, edge_distance set to its 3D length as GraphHopper does. */
    static int edge3D(Run run, ReaderWay way, double lenM, double ele0, double ele1) {
        PointList pl = new PointList(2, true);
        pl.add(run.lat, 24.0, ele0);
        run.lat += lenM / 111_195.0;
        pl.add(run.lat, 24.0, ele1);
        double d3 = DistanceCalcEarth.DIST_EARTH.calcDistance(pl);
        assertTrue(Math.abs(ele1 - ele0) < 1 || d3 > lenM + 1, "the point list is 3D");
        way.setTag("point_list", pl);
        way.setTag("edge_distance", d3);
        int id = run.nextEdge++;
        for (TagParser p : run.parsers)
            p.handleWayTags(id, run.eia, way, new IntsRef(2));
        return id;
    }

    @Test
    void staleHash_unchanged() {
        assertUnchanged(1013, "highway", "track");
        Run on = new Run(fixtureGz.toString());
        on.read(way(1013, "highway", "track"), 100, 100, 100);
        assertEquals(1, on.mtk.stats.waysStaleHash);
        // the same way with the node list the sidecar was made from: applied
        Run on2 = new Run(fixtureGz.toString());
        ReaderWay w = new ReaderWay(1013);
        for (long n : new long[]{201, 202, 203, 204}) w.getNodes().add(n);
        w.setTag("highway", "track");
        int[] e = on2.read(w, 100, 100, 100);
        assertEquals(GravelScale.ONE, on2.gravel(e[0]));
    }

    @Test
    void movedNodes_sameIds_lengthBeyondTolerance_unchanged_withinApplied() {
        // way 1001: hash matches, way_len_m 300; tolerance max(5 m, 1 % = 3 m) = 5 m.
        // nodes moved so the way is now 310 m (10 m off): every edge keeps today's values, counted as stale_length
        Run on = new Run(fixtureGz.toString());
        Run off = new Run(null);
        int[] a = on.read(way(1001, "highway", "track"), 100, 100, 110);
        int[] b = off.read(way(1001, "highway", "track"), 100, 100, 110);
        for (int i = 0; i < a.length; i++)
            assertEquals(off.all(b[i]), on.all(a[i]), "edge " + i + " of a length-stale way keeps today's values");
        assertEquals(1, on.mtk.stats.waysStaleLength);
        assertEquals(0, on.mtk.stats.waysStaleHash);
        assertEquals(0, on.mtk.stats.waysTrack);
        // 296 m (4 m short, within 5 m): applied
        Run on2 = new Run(fixtureGz.toString());
        int[] e = on2.read(way(1001, "highway", "track"), 100, 100, 96);
        for (int id : e)
            assertEquals(GravelScale.ONE, on2.gravel(id));
        assertEquals(0, on2.mtk.stats.waysStaleLength);
        assertEquals(1, on2.mtk.stats.waysTrack);
        // the same holds for the road case (re-run parsers): 2001 paved, 12 m long → unchanged
        Run on3 = new Run(fixtureGz.toString());
        Run off3 = new Run(null);
        int[] c = on3.read(way(2001, "highway", "unclassified"), 100, 100, 112);
        int[] d = off3.read(way(2001, "highway", "unclassified"), 100, 100, 112);
        for (int i = 0; i < c.length; i++)
            assertEquals(off3.all(d[i]), on3.all(c[i]));
        assertEquals(1, on3.mtk.stats.waysStaleLength);
    }

    @Test
    void lengthTolerance_boundary_exactly5m_and_exactly1pct_areApplied() {
        // short way: 1 % of 100 m = 1 m < 5 m, so 5 m is the tolerance
        assertFalse(MtkWayAttrsParser.isLengthStale(105, 100, 5, 0.01), "exactly 5 m: applied");
        assertFalse(MtkWayAttrsParser.isLengthStale(95, 100, 5, 0.01), "exactly -5 m: applied");
        assertTrue(MtkWayAttrsParser.isLengthStale(105.001, 100, 5, 0.01), "just over 5 m: stale");
        assertTrue(MtkWayAttrsParser.isLengthStale(94.999, 100, 5, 0.01));
        // long way: 1 % of 1000 m = 10 m > 5 m, so 10 m is the tolerance
        assertFalse(MtkWayAttrsParser.isLengthStale(1010, 1000, 5, 0.01), "exactly 1 %: applied");
        assertFalse(MtkWayAttrsParser.isLengthStale(990, 1000, 5, 0.01), "exactly -1 %: applied");
        assertTrue(MtkWayAttrsParser.isLengthStale(1010.001, 1000, 5, 0.01), "just over 1 %: stale");
        assertTrue(MtkWayAttrsParser.isLengthStale(989.999, 1000, 5, 0.01));
        // the crossover: at 500 m both are 5 m
        assertFalse(MtkWayAttrsParser.isLengthStale(505, 500, 5, 0.01));
        assertTrue(MtkWayAttrsParser.isLengthStale(505.001, 500, 5, 0.01));
        // the defaults are the spec's
        assertEquals(5.0, MtkWayAttrsParser.LEN_TOL_M);
        assertEquals(0.01, MtkWayAttrsParser.LEN_TOL_FRAC);
    }

    @Test
    void lengthTolerance_boundary_throughTheParser() {
        // way_len_m 300 → tolerance 5 m. The meridian edges in this harness measure lenM to ~1e-6 m, so 305 m is
        // at the boundary only within float noise; 304.99 applies and 305.01 is stale.
        Run on = new Run(fixtureGz.toString());
        int[] e = on.read(way(1001, "highway", "track"), 100, 100, 104.99);
        assertEquals(GravelScale.ONE, on.gravel(e[2]));
        Run on2 = new Run(fixtureGz.toString());
        int[] f = on2.read(way(1001, "highway", "track"), 100, 100, 105.01);
        assertEquals(GravelScale.THREE, on2.gravel(f[2]));
        assertEquals(1, on2.mtk.stats.waysStaleLength);
    }

    @Test
    void wayRevisitedLater_secondRunUnchanged() {
        Run on = new Run(fixtureGz.toString());
        int[] first = on.read(way(1001, "highway", "track"), 300);
        on.read(way(4242, "highway", "track"), 100);
        int[] second = on.read(way(1001, "highway", "track"), 100, 100);
        assertEquals(GravelScale.ONE, on.gravel(first[0]));
        assertEquals(GravelScale.THREE, on.gravel(second[0]));
        assertEquals(1, on.mtk.stats.waysRevisited);
    }

    @Test
    void nodeIdsHash_matchesShasum() {
        com.carrotsearch.hppc.LongArrayList l = new com.carrotsearch.hppc.LongArrayList();
        l.add(1, 2, 3);
        assertEquals(Long.parseUnsignedLong("b85e2d4914e22b5a", 16), MtkWayAttrsParser.nodeIdsHash(l));
        l.clear();
        for (long n : NODES) l.add(n);
        assertEquals(Long.parseUnsignedLong(NODES_HASH, 16), MtkWayAttrsParser.nodeIdsHash(l));
    }

    @Test
    void minClassShare_lever() {
        PMap props = new PMap().putObject("min_class_share", 0.94);
        // 1002 ajopolku has class_share 0.90 < 0.94
        for (Map<String, Object>[] p : compare(props, 1002, "highway", "track"))
            assertEquals(p[1], p[0]);
        for (Map<String, Object>[] p : compare(props, 1001, "highway", "track"))
            assertEquals(GravelScale.ONE, p[0].get(GravelScale.KEY));
    }

    // ------------------------------------------------------------------------------------------------
    // the file: missing, unreadable, malformed → no MTK values, never an exception
    // ------------------------------------------------------------------------------------------------

    static void assertNoChangeAnywhere(String file) {
        Run on = new Run(file);
        Run off = new Run(null);
        assertTrue(on.mtk.stats.sidecarWaysSeen == 0);
        for (long id : new long[]{1001, 1002, 1006, 2001, 2002, 2003}) {
            String hw = id >= 2000 ? "unclassified" : "track";
            int[] a = on.read(way(id, "highway", hw), 100, 100, 100);
            int[] b = off.read(way(id, "highway", hw), 100, 100, 100);
            for (int i = 0; i < a.length; i++)
                assertEquals(off.all(b[i]), on.all(a[i]), file + " way " + id);
        }
    }

    @Test
    void missingFile_noMtkValues() {
        assertNoChangeAnywhere(tmp.resolve("does-not-exist.csv.gz").toString());
        assertTrue(MtkSidecar.load(tmp.resolve("does-not-exist.csv.gz").toString()).isEmpty());
        assertTrue(MtkSidecar.load("").isEmpty());
        assertTrue(MtkSidecar.load(null).isEmpty());
        assertTrue(MtkSidecar.load(tmp.toString()).isEmpty(), "a directory");
    }

    @Test
    void unreadableFiles_noMtkValues() throws IOException {
        // truncated gzip
        byte[] gz = Files.readAllBytes(fixtureGz);
        Path truncated = tmp.resolve("truncated.csv.gz");
        Files.write(truncated, Arrays.copyOf(gz, gz.length / 2));
        assertNoChangeAnywhere(truncated.toString());
        // not gzip at all
        Path notGz = tmp.resolve("plain-but-named.csv.gz");
        Files.writeString(notGz, "way_id,node_ids_hash\n1,2\n");
        assertNoChangeAnywhere(notGz.toString());
        // wrong header
        Path badHeader = tmp.resolve("bad-header.csv");
        Files.writeString(badHeader, "way,hash,len,from,to,class,cs,paved,ps,ms,snap,ver\n"
                + "1001,8e21a6077f286951,300.0,0,1,ajotie,1,,0,1,2026-10,x\n");
        assertNoChangeAnywhere(badHeader.toString());
        // empty
        Path empty = tmp.resolve("empty.csv");
        Files.writeString(empty, "");
        assertNoChangeAnywhere(empty.toString());
    }

    @Test
    void malformedRows_skipped_inconsistentWays_dropped() throws IOException {
        String h = String.join(",", MtkSidecar.HEADER) + "\n";
        String ok = "1001," + NODES_HASH + ",300.0,0.0000,1.0000,ajotie,0.95,,0,0.97,2026-10,x\n";
        Path f = tmp.resolve("rows.csv");
        Files.writeString(f, h + ok
                + "1002,zz,300.0,0,1,ajotie,1,,0,1,2026-10,x\n"                       // bad hash
                + "1003," + NODES_HASH + ",300.0,0.8,0.2,ajotie,1,,0,1,2026-10,x\n"   // from > to
                + "1004," + NODES_HASH + ",300.0,0,1,motorway,1,,0,1,2026-10,x\n"     // unknown class
                + "1005," + NODES_HASH + ",300.0,0,1\n"                                // short row
                + "1006," + NODES_HASH + ",300.0,0.0,0.6,ajotie,1,,0,1,2026-10,x\n"   // overlapping pieces
                + "1006," + NODES_HASH + ",300.0,0.5,1.0,polku,1,,0,1,2026-10,x\n"
                + "1007," + NODES_HASH + ",300.0,0.0,0.5,ajotie,1,,0,1,2026-10,x\n"   // pieces disagree on length
                + "1007," + NODES_HASH + ",250.0,0.5,1.0,ajotie,1,,0,1,2026-10,x\n"
                + "\n");
        MtkSidecar s = MtkSidecar.load(f.toString());
        assertEquals(1, s.wayCount(), s.getStatus());
        assertTrue(s.findWay(1001) >= 0);
        assertEquals(-1, s.findWay(1006));
        assertEquals(-1, s.findWay(1007));
        assertTrue(s.getStatus().contains("4 malformed rows"), s.getStatus());
        assertTrue(s.getStatus().contains("2 inconsistent ways"), s.getStatus());
    }

    @Test
    void fixtureLoads_unsortedRowsAreSorted() throws IOException {
        MtkSidecar s = MtkSidecar.load(fixtureGz.toString());
        assertEquals(21, s.wayCount(), s.getStatus());
        assertEquals(24, s.pieceCount());
        // rows in reverse order give the same table
        List<String> lines = new ArrayList<>(Arrays.asList(new String(
                MtkWayAttrsParserTest.class.getResourceAsStream("mtk_way_attrs_fixture.csv").readAllBytes(),
                StandardCharsets.UTF_8).split("\n")));
        String header = lines.remove(0);
        Collections.reverse(lines);
        Path rev = tmp.resolve("reversed.csv");
        Files.writeString(rev, header + "\n" + String.join("\n", lines) + "\n");
        MtkSidecar r = MtkSidecar.load(rev.toString());
        assertEquals(s.wayCount(), r.wayCount());
        int w = r.findWay(1011);
        assertEquals(MtkSidecar.CLASS_AJOTIE, r.getMtkClass(r.findPiece(w, 0.1)));
        assertEquals(MtkSidecar.CLASS_POLKU, r.getMtkClass(r.findPiece(w, 0.9)));
        assertEquals(MtkSidecar.CLASS_POLKU, r.getMtkClass(r.findPiece(w, 1.0)));
    }

    // ------------------------------------------------------------------------------------------------
    // wiring
    // ------------------------------------------------------------------------------------------------

    @Test
    void registry_providesTheUnit_withoutAnEncodedValue() {
        ImportUnit u = new TrailmapImportRegistry().createImportUnit(MtkWayAttrsParser.KEY);
        assertNotNull(u, "TrailmapImportRegistry must register " + MtkWayAttrsParser.KEY);
        assertNull(u.getCreateEncodedValue(), "no encoded value: graph format unchanged");
        assertEquals(Arrays.asList(MtkWayAttrsParser.REQUIRED), u.getRequiredImportUnits());
    }
}
