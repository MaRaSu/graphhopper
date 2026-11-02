package com.graphhopper.trailmap.tilerouting;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.config.Profile;
import com.graphhopper.jackson.Jackson;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.util.AllEdgesIterator;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.trailmap.TrailmapGraphHopper;
import com.graphhopper.trailmap.barrier.BarrierClass;
import com.graphhopper.trailmap.barrier.BarrierProfiles;
import com.graphhopper.trailmap.tilerouting.foundation.TileMath;
import com.graphhopper.util.*;
import com.graphhopper.util.shapes.GHPoint;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.*;

/**
 * W26 S4 /tile_route walk-in check (test_lead_routing, CATALOG §8): synthetic one-tile jobs at changed-set barriers
 * (gravel): the z17 tile that holds a point 30 m behind the barrier, home 1.5 km outside. Each job is routed without
 * and with the flag; both responses are written to {@code <tw.out>/barrier-walks.json} for the independent check
 * tile-routing/verify/barrier-walks.mjs (which reads the graph's barrier table, not this code).
 * <pre>-Dtw.graph=DIR -Dtw.config=FILE -Dtw.out=DIR [-Dtw.n=N]</pre>
 */
public class BarrierTileWalkTest {

    static TrailmapGraphHopper hopper;

    static boolean open(Weighting w, EdgeIteratorState e) {
        return Double.isFinite(w.calcEdgeWeight(e, false)) || Double.isFinite(w.calcEdgeWeight(e, true));
    }

    static int find(int[] p, int x) {
        while (p[x] != x) {
            p[x] = p[p[x]];
            x = p[x];
        }
        return x;
    }

    static GHPoint along(Weighting w, int start, int skip, double d) {
        BaseGraph g = hopper.getBaseGraph();
        EdgeExplorer ex = g.createEdgeExplorer();
        Map<Integer, Double> dist = new HashMap<>();
        PriorityQueue<double[]> q = new PriorityQueue<>(Comparator.comparingDouble(x -> x[0]));
        dist.put(start, 0.0);
        q.add(new double[]{0, start});
        while (!q.isEmpty()) {
            double[] cur = q.poll();
            int n = (int) cur[1];
            if (cur[0] > dist.getOrDefault(n, Double.MAX_VALUE)) continue;
            EdgeIterator it = ex.setBaseNode(n);
            while (it.next()) {
                if (it.getEdge() == skip || !open(w, it)) continue;
                double nd = cur[0] + it.getDistance();
                if (cur[0] <= d && nd >= d && it.getDistance() > 0.5) {
                    PointList pl = it.fetchWayGeometry(FetchMode.ALL);
                    double rest = d - cur[0];
                    for (int i = 0; i + 1 < pl.size(); i++) {
                        double seg = DistanceCalcEarth.DIST_EARTH.calcDist(pl.getLat(i), pl.getLon(i), pl.getLat(i + 1), pl.getLon(i + 1));
                        if (seg >= rest || i + 2 == pl.size()) {
                            double f = seg <= 0 ? 0 : Math.min(1, rest / seg);
                            return new GHPoint(pl.getLat(i) + f * (pl.getLat(i + 1) - pl.getLat(i)), pl.getLon(i) + f * (pl.getLon(i + 1) - pl.getLon(i)));
                        }
                        rest -= seg;
                    }
                }
                if (nd < dist.getOrDefault(it.getAdjNode(), Double.MAX_VALUE)) {
                    dist.put(it.getAdjNode(), nd);
                    q.add(new double[]{nd, it.getAdjNode()});
                }
            }
        }
        return null;
    }

    @Test
    void walks() throws Exception {
        Assumptions.assumeTrue(System.getProperty("tw.graph") != null, "tw.graph not set");
        ObjectMapper yaml = Jackson.initObjectMapper(new ObjectMapper(new YAMLFactory()));
        GraphHopperConfig config = yaml.treeToValue(yaml.readTree(new File(System.getProperty("tw.config"))).get("graphhopper"), GraphHopperConfig.class);
        config.putObject("graph.location", System.getProperty("tw.graph"));
        hopper = new TrailmapGraphHopper();
        hopper.init(config);
        hopper.setAllowWrites(false);
        hopper.load();
        ObjectMapper json = Jackson.newObjectMapper();
        TileRouter.Config rc = new TileRouter.Config();
        rc.clock = () -> TileRouter.virtualClock(1000);
        TileRouter router = new TileRouter(hopper, rc);
        String prof = "gravel";
        Profile p = hopper.getProfile(prof);
        Weighting w = hopper.createWeighting(p, new PMap());
        EnumEncodedValue<BarrierClass> enc = BarrierProfiles.barrierEnc(hopper.getEncodingManager(), p);
        BaseGraph g = hopper.getBaseGraph();
        NodeAccess na = g.getNodeAccess();
        int[] par = new int[g.getNodes()];
        for (int i = 0; i < par.length; i++) par[i] = i;
        AllEdgesIterator it = g.getAllEdges();
        while (it.next()) if (open(w, it)) {
            int a = find(par, it.getBaseNode()), b = find(par, it.getAdjNode());
            if (a != b) par[a] = b;
        }
        Map<Integer, Integer> size = new HashMap<>();
        it = g.getAllEdges();
        while (it.next()) if (open(w, it)) size.merge(find(par, it.getBaseNode()), 1, Integer::sum);
        int main = size.entrySet().stream().max(Map.Entry.comparingByValue()).get().getKey();
        int max = Integer.getInteger("tw.n", 60), n = 0;
        ArrayNode out = json.createArrayNode();
        Set<String> tiles = new HashSet<>();
        it = g.getAllEdges();
        while (it.next() && n < max) {
            if (it.getDistance() >= 0.01 || !it.get(enc).closes() || open(w, it)) continue;
            if (na.getLat(it.getBaseNode()) != na.getLat(it.getAdjNode()) || na.getLon(it.getBaseNode()) != na.getLon(it.getAdjNode())) continue;
            int ca = find(par, it.getBaseNode()), cb = find(par, it.getAdjNode());
            if (ca == cb || (ca != main && cb != main)) continue;
            int outside = ca == main ? it.getBaseNode() : it.getAdjNode();
            int inside = outside == it.getBaseNode() ? it.getAdjNode() : it.getBaseNode();
            GHPoint in = along(w, inside, it.getEdge(), Integer.getInteger("tw.inM", 30)), home = along(w, outside, it.getEdge(), 1500);
            if (in == null || home == null) continue;
            TileMath.Tile t = TileMath.Tile.containing(in.lat, in.lon, 17, 20);
            String key = t.key;
            if (!tiles.add(key)) continue;
            n++;
            ObjectNode o = out.addObject();
            o.put("gate_lat", na.getLat(it.getBaseNode())).put("gate_lon", na.getLon(it.getBaseNode())).put("class", it.get(enc).name());
            for (String flag : new String[]{null, "v1"}) {
                TileRouteRequest rq = new TileRouteRequest();
                rq.tiles.add(key);
                rq.zoom = 17;
                rq.startLat = home.lat;
                rq.startLon = home.lon;
                rq.profile = prof;
                rq.seedKey = key;
                rq.handover = false;
                rq.barriers = flag;
                try {
                    o.set(flag == null ? "noflag" : "flag", TileRouteJson.write(json, rq, router.route(rq)));
                } catch (Exception e) {
                    o.put(flag == null ? "noflag" : "flag", "exception: " + e);
                }
            }
        }
        File dir = new File(System.getProperty("tw.out"));
        dir.mkdirs();
        json.writerWithDefaultPrettyPrinter().writeValue(new File(dir, "barrier-walks.json"), out);
        System.out.println("TILEWALK jobs " + n);
        hopper.close();
    }
}
