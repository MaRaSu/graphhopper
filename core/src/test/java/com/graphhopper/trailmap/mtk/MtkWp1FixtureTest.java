package com.graphhopper.trailmap.mtk;

import com.graphhopper.reader.ReaderWay;
import com.graphhopper.trailmap.shared.GravelScale;
import com.graphhopper.trailmap.shared.PredictedSurface;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;

import static org.junit.jupiter.api.Assertions.*;

/**
 * The overlay on WP1's fixture: 20 sidecar rows from a real full-Finland mtk-enrich run (OSM 2026-09-29) and the
 * 18 OSM ways they belong to, with their real tags and node ids (way_network, 2026-11-01; files
 * wp1_mtk_way_attrs_fixture.csv.gz and wp1_fixture_ways.csv next to this class, copied unchanged from
 * way-network pipeline/mtk-enrich/fixture/).
 * <p>
 * The sidecar rows contain data from the National Land Survey of Finland Topographic Database 10/2026
 * (CC BY 4.0), modified (matched to OSM ways).
 * <p>
 * The fixture has no coordinates, so each way is read as equal-length edges between consecutive nodes; the
 * expected piece per edge is computed here from that layout.
 */
class MtkWp1FixtureTest {

    @TempDir
    static Path tmp;
    static Path sidecar;
    static final Map<Long, String[]> WAYS = new LinkedHashMap<>(); // id -> [case, row_state, tags, node_ids]

    @BeforeAll
    static void load() throws IOException {
        sidecar = tmp.resolve("wp1.csv.gz");
        try (InputStream in = MtkWp1FixtureTest.class.getResourceAsStream("wp1_mtk_way_attrs_fixture.csv.gz")) {
            Files.write(sidecar, Objects.requireNonNull(in).readAllBytes());
        }
        String csv;
        try (InputStream in = MtkWp1FixtureTest.class.getResourceAsStream("wp1_fixture_ways.csv")) {
            csv = new String(Objects.requireNonNull(in).readAllBytes(), StandardCharsets.UTF_8);
        }
        List<String> lines = new ArrayList<>(Arrays.asList(csv.split("\n")));
        assertEquals("case,way_id,row_state,tags,node_ids", lines.remove(0).trim());
        for (String l : lines) {
            if (l.isBlank()) continue;
            List<String> f = splitCsv(l.trim());
            assertEquals(5, f.size(), l);
            WAYS.put(Long.parseLong(f.get(1)), new String[]{f.get(0), f.get(2), f.get(3), f.get(4)});
        }
        assertEquals(18, WAYS.size());
    }

    static List<String> splitCsv(String l) {
        List<String> out = new ArrayList<>();
        StringBuilder cur = new StringBuilder();
        boolean q = false;
        for (int i = 0; i < l.length(); i++) {
            char c = l.charAt(i);
            if (c == '"') q = !q;
            else if (c == ',' && !q) {
                out.add(cur.toString());
                cur.setLength(0);
            } else cur.append(c);
        }
        out.add(cur.toString());
        return out;
    }

    static ReaderWay way(long id) {
        String[] w = WAYS.get(id);
        ReaderWay r = new ReaderWay(id);
        for (String kv : w[2].split(";")) {
            int e = kv.indexOf('=');
            r.setTag(kv.substring(0, e), kv.substring(e + 1));
        }
        for (String n : w[3].trim().split(" ")) r.getNodes().add(Long.parseLong(n));
        return r;
    }

    /** Reads the way as (nodes - 1) equal edges summing to the sidecar's way_len_m (or 100 m if absent). */
    static int[] read(MtkWayAttrsParserTest.Run run, long id) {
        ReaderWay w = way(id);
        MtkSidecar s = MtkSidecar.load(sidecar.toString());
        int idx = s.findWay(id);
        double len = idx >= 0 ? s.getWayLength(idx) : 100;
        int n = w.getNodes().size() - 1;
        double[] lens = new double[n];
        Arrays.fill(lens, len / n);
        return run.read(w, lens);
    }

    static void assertUnchanged(long id) {
        MtkWayAttrsParserTest.Run on = new MtkWayAttrsParserTest.Run(sidecar.toString());
        MtkWayAttrsParserTest.Run off = new MtkWayAttrsParserTest.Run(null);
        int[] a = read(on, id), b = read(off, id);
        for (int i = 0; i < a.length; i++)
            assertEquals(off.all(b[i]), on.all(a[i]), WAYS.get(id)[0]);
    }

    @Test
    void nodeIdsHash_freshRowsMatch_staleRowDoesNot() {
        MtkSidecar s = MtkSidecar.load(sidecar.toString());
        assertEquals(17, s.wayCount(), s.getStatus());
        int fresh = 0;
        for (Map.Entry<Long, String[]> e : WAYS.entrySet()) {
            int idx = s.findWay(e.getKey());
            String state = e.getValue()[1];
            if (state.equals("absent")) {
                assertEquals(-1, idx);
                continue;
            }
            long h = MtkWayAttrsParser.nodeIdsHash(way(e.getKey()).getNodes());
            if (state.equals("fresh")) {
                assertEquals(s.getHash(idx), h, e.getValue()[0]);
                fresh++;
            } else {
                assertEquals("stale", state);
                assertNotEquals(s.getHash(idx), h, e.getValue()[0]);
            }
        }
        assertEquals(16, fresh);
    }

    @Test
    void tracks() {
        MtkWayAttrsParserTest.Run on = new MtkWayAttrsParserTest.Run(sidecar.toString());
        for (int e : read(on, 4339586)) { // T1 ajotie
            assertEquals(GravelScale.ONE, on.gravel(e));
            assertFalse(on.unknownTrack(e));
        }
        for (int e : read(on, 4229443)) { // T1 ajopolku
            assertEquals(GravelScale.TWO, on.gravel(e));
            assertFalse(on.unknownTrack(e));
        }
        for (int e : read(on, 5110510)) // T2 "-tie" on ajotie
            assertEquals(GravelScale.ONE, on.gravel(e));
        assertUnchanged(22738916); // T1 talvitie
        assertUnchanged(4232325);  // T1 polku
        assertUnchanged(4226160);  // T1 empty class
        assertUnchanged(27753807); // T2 "-tie" on polku: rule 9 stays
        assertUnchanged(26751211); // T3 obstacle
        assertUnchanged(4358278);  // T4 trail_visibility
        assertUnchanged(4342183);  // tracktype etc.: OSM wins
    }

    @Test
    void trackMultiPiece_withGaps() {
        // pieces 0–0.5085 ajotie, 0.5470–0.5684 ajotie, 0.6923–0.7308 ajopolku; the rest unmatched
        MtkWayAttrsParserTest.Run on = new MtkWayAttrsParserTest.Run(sidecar.toString());
        int[] e = read(on, 22575912);
        int n = e.length, ones = 0, twos = 0, threes = 0;
        for (int i = 0; i < n; i++) {
            double mid = (i + 0.5) / n;
            GravelScale want = mid < 0.5085 || (mid >= 0.5470 && mid < 0.5684) ? GravelScale.ONE
                    : mid >= 0.6923 && mid < 0.7308 ? GravelScale.TWO : GravelScale.THREE;
            assertEquals(want, on.gravel(e[i]), "edge " + i + " of " + n + " at " + mid);
            if (want == GravelScale.ONE) ones++;
            else if (want == GravelScale.TWO) twos++;
            else threes++;
        }
        assertTrue(ones > 0 && threes > 0, ones + "/" + twos + "/" + threes);
    }

    @Test
    void roads() {
        MtkWayAttrsParserTest.Run on = new MtkWayAttrsParserTest.Run(sidecar.toString());
        for (int e : read(on, 4066353)) { // unclassified unpaved
            assertEquals(PredictedSurface.COMPACTED, on.surface(e));
            assertEquals(GravelScale.ZERO_PLUS, on.gravel(e));
        }
        for (int e : read(on, 4238893)) { // service driveway unpaved
            assertEquals(PredictedSurface.COMPACTED, on.surface(e));
            assertEquals(GravelScale.ONE, on.gravel(e));
        }
        for (int e : read(on, 4303522)) { // tertiary paved
            assertEquals(PredictedSurface.ASPHALT, on.surface(e));
            assertEquals(GravelScale.ZERO_MINUS, on.gravel(e));
        }
        assertUnchanged(4066300); // stale hash
        assertUnchanged(2448849); // surface tagged: OSM wins
        assertUnchanged(4048348); // no sidecar row
    }

    @Test
    void roadMultiPiece_unpavedThenPaved() {
        MtkWayAttrsParserTest.Run on = new MtkWayAttrsParserTest.Run(sidecar.toString());
        int[] e = read(on, 4066357);
        for (int i = 0; i < e.length; i++) {
            double mid = (i + 0.5) / e.length;
            assertEquals(mid < 0.9077 ? PredictedSurface.COMPACTED : PredictedSurface.ASPHALT, on.surface(e[i]), "at " + mid);
        }
    }
}
