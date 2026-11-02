package com.graphhopper.trailmap.mtk;

import com.graphhopper.GraphHopperConfig;
import com.graphhopper.config.Profile;
import com.graphhopper.json.Statement;
import com.graphhopper.routing.ev.*;
import com.graphhopper.routing.util.AllEdgesIterator;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.trailmap.TrailmapGraphHopper;
import com.graphhopper.trailmap.shared.*;
import com.graphhopper.util.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;

import static org.junit.jupiter.api.Assertions.*;

/**
 * The overlay inside a real GraphHopper import (OSMReader → WaySegmentParser → parsers) of a small OSM file:
 * that a way's edges reach the unit one at a time and in way order, through junction splits and a barrier
 * edge, so the summed 2D edge length midpoint picks the right piece; and that the configured unit with a
 * missing sidecar gives exactly today's edge values.
 *
 * Track 1011: nodes 101..107 northwards, 0.001° (~111 m) apart. Road 5000 crosses it at 103 and 106, so the
 * track is split there; node 104 is a gate (barrier edge). Sidecar pieces: 0–0.45 ajotie, 0.45–1 polku. Edge
 * midpoints along the track (fractions): 101–103 0.167, 103–104 0.417, barrier at 0.5, 104–106 0.667,
 * 106–107 0.917 → gravel 1, 1, 3, 3, 3.
 */
class MtkOverlayImportTest {

    @TempDir
    Path tmp;

    static final double LAT0 = 60.0, LON = 24.0, STEP = 0.001;

    static String osm() {
        StringBuilder s = new StringBuilder("<?xml version='1.0' encoding='UTF-8'?>\n<osm version='0.6'>\n");
        for (int i = 0; i < 7; i++) {
            s.append(String.format(Locale.ROOT, "<node id='%d' lat='%.7f' lon='%.7f'%s\n", 101 + i, LAT0 + i * STEP, LON,
                    i == 3 ? "><tag k='barrier' v='gate'/></node>" : "/>"));
        }
        // the road's own end nodes, east and west of the track
        s.append(String.format(Locale.ROOT, "<node id='201' lat='%.7f' lon='%.7f'/>\n", LAT0 + 2 * STEP, LON - 0.002));
        s.append(String.format(Locale.ROOT, "<node id='202' lat='%.7f' lon='%.7f'/>\n", LAT0 + 2 * STEP, LON + 0.002));
        s.append(String.format(Locale.ROOT, "<node id='203' lat='%.7f' lon='%.7f'/>\n", LAT0 + 5 * STEP, LON + 0.002));
        s.append("<way id='1011'>");
        for (int i = 0; i < 7; i++) s.append("<nd ref='").append(101 + i).append("'/>");
        s.append("<tag k='highway' v='track'/></way>\n");
        s.append("<way id='5000'><nd ref='201'/><nd ref='103'/><nd ref='202'/><nd ref='203'/><nd ref='106'/>"
                + "<tag k='highway' v='residential'/></way>\n");
        s.append("</osm>\n");
        return s.toString();
    }

    static double len(double[][] pts) {
        double d = 0;
        for (int i = 1; i < pts.length; i++)
            d += DistanceCalcEarth.DIST_EARTH.calcDist(pts[i - 1][0], pts[i - 1][1], pts[i][0], pts[i][1]);
        return d;
    }

    static String hash(long... nodes) {
        com.carrotsearch.hppc.LongArrayList l = new com.carrotsearch.hppc.LongArrayList();
        l.add(nodes);
        return String.format("%016x", MtkWayAttrsParser.nodeIdsHash(l));
    }

    Path sidecar() throws IOException {
        double[][] track = new double[7][];
        for (int i = 0; i < 7; i++) track[i] = new double[]{LAT0 + i * STEP, LON};
        double[][] road = {{LAT0 + 2 * STEP, LON - 0.002}, {LAT0 + 2 * STEP, LON}, {LAT0 + 2 * STEP, LON + 0.002},
                {LAT0 + 5 * STEP, LON + 0.002}, {LAT0 + 5 * STEP, LON}};
        String h = String.join(",", MtkSidecar.HEADER) + "\n";
        String rows = String.format(Locale.ROOT, "1011,%s,%.1f,0.0000,0.4500,ajotie,0.9,,0,0.9,2026-10,t\n"
                        + "1011,%s,%.1f,0.4500,1.0000,polku,0.9,,0,0.9,2026-10,t\n"
                        + "5000,%s,%.1f,0.0000,1.0000,ajotie,0.9,unpaved,0.9,0.9,2026-10,t\n",
                hash(101, 102, 103, 104, 105, 106, 107), len(track), hash(101, 102, 103, 104, 105, 106, 107), len(track),
                hash(201, 103, 202, 203, 106), len(road));
        Path f = tmp.resolve("sidecar.csv");
        Files.writeString(f, h + rows);
        return f;
    }

    static final String EVS = "road_class, road_environment, roundabout, osm_way_id, gravel_scale, gravel_scale_num, "
            + "gravel_base_priority, mtb_scale, mtb_scale_num, mtb_base_priority, predicted_surface, "
            + "issue_biking_blocked, issue_biking_blocked_risk, issue_foot_blocked, issue_narrow, "
            + "issue_poor_visibility, issue_vegetation, issue_mud, issue_unknown_path, issue_unknown_track, issue_ferry, "
            + "issue_driveway";

    TrailmapGraphHopper importWith(String name, String mtkEntry) throws IOException {
        Path osm = tmp.resolve("fixture.osm");
        if (!Files.exists(osm)) Files.writeString(osm, osm());
        GraphHopperConfig config = new GraphHopperConfig();
        config.putObject("datareader.file", osm.toString());
        config.putObject("graph.location", tmp.resolve(name).toString());
        config.putObject("graph.encoded_values", mtkEntry == null ? EVS : EVS + ", " + mtkEntry);
        config.putObject("import.osm.ignored_highways", "");
        config.putObject("prepare.min_network_size", 0);
        CustomModelHolder.profile(config);
        TrailmapGraphHopper hopper = new TrailmapGraphHopper();
        hopper.setImportRegistry(new TrailmapImportRegistry());
        hopper.init(config);
        hopper.importOrLoad();
        return hopper;
    }

    static final class CustomModelHolder {
        static void profile(GraphHopperConfig config) {
            com.graphhopper.util.CustomModel cm = new com.graphhopper.util.CustomModel();
            cm.addToSpeed(Statement.If("true", Statement.Op.LIMIT, "60"));
            Profile p = new Profile("mtk_test");
            p.setCustomModel(cm);
            config.setProfiles(Collections.singletonList(p));
        }
    }

    /** Edges of a way as (south lat of the edge's span, edge id), sorted south → north / west → east. */
    static List<int[]> edgesOf(TrailmapGraphHopper h, int wayId) {
        BaseGraph g = h.getBaseGraph();
        IntEncodedValue wid = h.getEncodingManager().getIntEncodedValue(OSMWayID.KEY);
        List<int[]> r = new ArrayList<>();
        AllEdgesIterator it = g.getAllEdges();
        while (it.next())
            if (it.get(wid) == wayId) r.add(new int[]{it.getEdge()});
        return r;
    }

    static double midLat(TrailmapGraphHopper h, int edge) {
        PointList pl = h.getBaseGraph().getEdgeIteratorState(edge, Integer.MIN_VALUE).fetchWayGeometry(FetchMode.ALL);
        return (pl.getLat(0) + pl.getLat(pl.size() - 1)) / 2;
    }

    @Test
    void wayEdgesInOrder_midpointPicksPiece_throughJunctionsAndBarrier() throws IOException {
        TrailmapGraphHopper h = importWith("on", "mtk_way_attrs|file=" + sidecar());
        EnumEncodedValue<GravelScale> gs = h.getEncodingManager().getEnumEncodedValue(GravelScale.KEY, GravelScale.class);
        EnumEncodedValue<PredictedSurface> ps = h.getEncodingManager().getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class);
        List<int[]> track = edgesOf(h, 1011);
        assertEquals(5, track.size(), "4 edges + 1 barrier edge");
        int ones = 0, threes = 0;
        for (int[] e : track) {
            EdgeIteratorState s = h.getBaseGraph().getEdgeIteratorState(e[0], Integer.MIN_VALUE);
            double frac = (midLat(h, e[0]) - LAT0) / (6 * STEP);
            GravelScale want = frac < 0.45 ? GravelScale.ONE : GravelScale.THREE;
            assertEquals(want, s.get(gs), "track edge at fraction " + frac);
            if (want == GravelScale.ONE) ones++;
            else threes++;
        }
        assertEquals(2, ones);
        assertEquals(3, threes);
        for (int[] e : edgesOf(h, 5000))
            assertEquals(PredictedSurface.COMPACTED, h.getBaseGraph().getEdgeIteratorState(e[0], Integer.MIN_VALUE).get(ps));
        h.close();
    }

    @Test
    void configuredUnit_missingFile_sameEdgesAsWithoutTheUnit() throws IOException {
        TrailmapGraphHopper without = importWith("without", null);
        TrailmapGraphHopper missing = importWith("missing", "mtk_way_attrs|file=" + tmp.resolve("nope.csv.gz"));
        BaseGraph a = without.getBaseGraph(), b = missing.getBaseGraph();
        assertEquals(a.getEdges(), b.getEdges());
        assertEquals(a.getNodes(), b.getNodes());
        assertEquals(without.getEncodingManager().toEncodedValuesAsString(), missing.getEncodingManager().toEncodedValuesAsString());
        assertEquals(without.getEncodingManager().getBytesForFlags(), missing.getEncodingManager().getBytesForFlags());
        for (int e = 0; e < a.getEdges(); e++) {
            EdgeIteratorState x = a.getEdgeIteratorState(e, Integer.MIN_VALUE), y = b.getEdgeIteratorState(e, Integer.MIN_VALUE);
            assertEquals(x.fetchWayGeometry(FetchMode.ALL).toString(), y.fetchWayGeometry(FetchMode.ALL).toString());
            assertEquals(x.getFlags(), y.getFlags(), "edge " + e + " flags");
        }
        without.close();
        missing.close();
    }
}
