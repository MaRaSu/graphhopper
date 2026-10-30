package com.graphhopper.trailmap.roundtrip.loop;

import com.graphhopper.GraphHopper;
import com.graphhopper.util.CustomModel;

import java.util.*;

/**
 * Trailmap AutoRoute v2 (round 4, "area16"): one press = 3 round-trip loops from one start. The loop sources are a2
 * (shaped holes, {@link MultiHoleEngine}) and c2 (the client's templates as a band, {@link BandLoopEngine}); the
 * {@link MergeEngine} picks 3 from both pools: slots 1-2 follow the heading, slot 3 is the "surprise". The routes the
 * session has already shown ({@code shown}) make their ~300 m areas cost more and steer the pick to new ways.
 * <p>
 * Stateless per request: the session memory is rebuilt from {@code shown} on every call, so concurrent sessions never
 * share it. Only the start's local graphs are cached (read-only after build), bounded to {@link #CACHE_SIZE} starts.
 * The prototype is algo/autoroute round 4 (test package prototype/autoroute/m); this copy is the same algorithm with
 * its settings fixed.
 */
public class LoopAutoRoute {

    static final int CACHE_SIZE = 16;

    /** One route of a press: the engine's loop and its waypoints (both [lat, lon], waypoints without start / end). */
    public static final class Route {
        public final String source, label, role;
        public final double distance;
        public final List<double[]> coords, waypoints;

        Route(MergeEngine.Route r) {
            this.source = r.source;
            this.label = r.label;
            this.role = r.role;
            this.distance = r.distance;
            this.coords = r.coords;
            this.waypoints = r.waypoints;
        }
    }

    private static final class Local {
        final MultiHoleEngine.LocalGraph lg;
        final Topology topo;

        Local(MultiHoleEngine.LocalGraph lg, Topology topo) {
            this.lg = lg;
            this.topo = topo;
        }
    }

    private final GraphHopper hopper;
    private final Map<String, Local> cache;

    public LoopAutoRoute(GraphHopper hopper) {
        this(hopper, CACHE_SIZE);
    }

    /** cacheSize: how many starts keep their local graphs (each start's presses reuse them). */
    public LoopAutoRoute(GraphHopper hopper, int cacheSize) {
        this.hopper = hopper;
        this.cache = new LinkedHashMap<>(cacheSize, 0.75f, true) {
            @Override
            protected boolean removeEldestEntry(Map.Entry<String, Local> eldest) {
                return size() > cacheSize;
            }
        };
    }

    /**
     * One press.
     *
     * @param profile     the standard profile (e.g. gravel); {@code <profile>_explore} must exist
     * @param requestModel the request's custom model (e.g. the ferry fragment), or null
     * @param headingDeg  NaN for no direction
     * @param shown       the routes shown earlier in this session, oldest first, as [lat, lon] lists (may be empty)
     */
    public List<Route> generate(String profile, CustomModel requestModel, double lat, double lon, double targetM,
                                double headingDeg, long seed, List<List<double[]>> shown) {
        MergeEngine engine = new MergeEngine(hopper, profile, requestModel);
        Local local = local(engine, profile, requestModel, lat, lon, targetM);
        Memory memory = new Memory(lat, lon, engine.params.memFreeM, engine.params.memZ);
        for (List<double[]> r : shown) memory.add(r);
        List<Route> out = new ArrayList<>();
        for (MergeEngine.Route r : engine.generate(local.lg, local.topo, lat, lon, targetM, headingDeg, seed, memory))
            out.add(new Route(r));
        return out;
    }

    /** The start's local graph and topology, as MergeRunTest builds them (lg / topo null when the start can't snap). */
    private Local local(MergeEngine engine, String profile, CustomModel requestModel, double lat, double lon, double targetM) {
        String key = profile + "|" + (requestModel == null ? "" : requestModel.toString()) + "|" + lat + "," + lon + "," + targetM;
        synchronized (cache) {
            Local l = cache.get(key);
            if (l != null) return l;
        }
        MultiHoleEngine.LocalGraph lg = engine.holes.build(lat, lon, 0.5 * targetM + 3000);
        Topology topo = Topology.build(hopper, engine.band.exploreWeighting(), profile + "_explore", lat, lon,
                engine.band.params.radiusFactor * targetM + 5000);
        Local l = new Local(lg, topo);
        synchronized (cache) {
            cache.put(key, l);
        }
        return l;
    }
}
