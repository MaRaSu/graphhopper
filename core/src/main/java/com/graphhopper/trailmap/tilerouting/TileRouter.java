package com.graphhopper.trailmap.tilerouting;

import com.graphhopper.GraphHopper;
import com.graphhopper.json.Statement;
import com.graphhopper.trailmap.tilerouting.foundation.Candidate;
import com.graphhopper.trailmap.tilerouting.foundation.Foundation;
import com.graphhopper.trailmap.tilerouting.foundation.Via;
import com.graphhopper.trailmap.tilerouting.looprepair.LoopRepairCandidate;
import com.graphhopper.trailmap.tilerouting.strokes.StrokeCoverCandidate;
import com.graphhopper.trailmap.tilerouting.tourrepair.FreeOrderRepairCandidate;
import com.graphhopper.util.CustomModel;

import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Tile tours (interim v1, owner 2026-10-30): a loop from the start that enters every requested map tile. Runs one of
 * the round-1…4 algorithms ({@link TileRouteRequest#algorithm}) on the shared tile foundation, expands walk-ins into
 * direct legs, and optionally runs the re-use polish. Ported from the test-package prototypes
 * ({@code com.graphhopper.trailmap.prototype.tilerouting}, runner TileRoutingRunTest) with their defaults.
 */
public class TileRouter {

    /** Config levers (graphhopper: tile_routing.*). */
    public static final class Config {
        public int maxTilesZ14 = 50, maxTilesZ17 = 150;
        /** Portal tour search budget when it seeds tour_repair (owner: the 60 s limit is in; it stops on its own sooner). */
        public long portalBudgetMs = 60_000;
        /** Loop repair search budget (prototype tr.c2.budgetMs). */
        public long repairBudgetMs = 10_000;
        /** A new clock per request for the search budgets (tests: a virtual clock, so a run is reproducible). */
        public java.util.function.Supplier<java.util.function.LongSupplier> clock = () -> System::nanoTime;
        /** How tour_repair gets its measured A* bound (a full edge scan, ≈ 25 s on the Europe graph). */
        public Heuristic heuristic = Heuristic.BACKGROUND;
    }

    /**
     * tour_repair's A* bound. Only tour_repair reads it (portal tour legs); strokes and repair never measure it.
     * <ul>
     * <li>BACKGROUND (server): the first request per profile measures the bound for its three safety levels on one
     * background thread, never on the request; until a bound is ready, tour_repair uses the weighting's own bound
     * (still a lower bound, so legs stay shortest; the search settles more states).</li>
     * <li>INLINE (tests): measured on the request, as the stored references were made.</li>
     * <li>BUILT_IN (tests): never measured; the weighting's own bound, as BACKGROUND before its bound is ready.</li>
     * </ul>
     */
    public enum Heuristic {BACKGROUND, INLINE, BUILT_IN}

    /** A clock that advances stepNs per call: budgets and the annealing schedule become iteration counts. */
    public static java.util.function.LongSupplier virtualClock(long stepNs) {
        long[] t = {0};
        return () -> t[0] += stepNs;
    }

    /** Safety levels: the job sets' custom models (tile-routing/make-jobs.mjs SAFETY), byte for byte. */
    public static CustomModel safetyModel(String safety) {
        switch (safety) {
            case "standard":
                return null;
            case "strict":
                return ban("gravel_scale == THREE || gravel_scale == FOUR || gravel_scale == UNKNOWN");
            case "no_trails":
                return ban("gravel_scale == TWO || gravel_scale == THREE || gravel_scale == FOUR || gravel_scale == UNKNOWN");
            default:
                throw new IllegalArgumentException("safety must be standard|strict|no_trails, got " + safety);
        }
    }

    private static CustomModel ban(String condition) {
        CustomModel cm = new CustomModel();
        cm.addToPriority(Statement.If(condition, Statement.Op.MULTIPLY, "0"));
        return cm;
    }

    /** The tour as the app takes it: waypoints with one segment type per leg, plus what happened to each tile. */
    public static final class Result {
        public List<Via> vias = new ArrayList<>();
        public List<String> segments = new ArrayList<>();
        public List<Foundation.Walk> walks = new ArrayList<>();
        public List<Foundation.TileInfo> tiles = new ArrayList<>();
        public Foundation.Route route;
        public String algorithm, note = "";
        public long foundationMs, solveMs, polishMs, handoverMs, totalMs;
        /** The hand-over's pins and flags (null when off, or for standard safety: nothing to pin). */
        public TileHandover.Stats handover;
        /** Edge keys held at their peak by the request's search structures (home floods + searchers), memory check. */
        public long keysHeld;
        public ReusePolish.Stats polish;
    }

    /** One measured A* bound per (graph, profile, safety): a full edge scan, the same every time. */
    private static final Map<String, Double> HEURISTIC = new ConcurrentHashMap<>();
    /** Keys whose background measurement is queued or done. */
    private static final Set<String> WARMING = ConcurrentHashMap.newKeySet();
    /** One low-priority daemon thread measures the bounds in the background, one key after another. */
    private static final java.util.concurrent.ExecutorService WARM_UP = java.util.concurrent.Executors.newSingleThreadExecutor(r -> {
        Thread t = new Thread(r, "tile-route-heuristic");
        t.setDaemon(true);
        t.setPriority(Thread.MIN_PRIORITY);
        return t;
    });

    private static String heuristicKey(GraphHopper hopper, String profile, String safety) {
        return System.identityHashCode(hopper) + "|" + profile + "|" + safety;
    }

    /** Queues the background measurement of the profile's bounds, the request's safety level first. Never blocks. */
    private void warmUp(String profile, String firstSafety) {
        List<String> order = new ArrayList<>(List.of(firstSafety));
        for (String s : new String[]{"standard", "strict", "no_trails"}) if (!order.contains(s)) order.add(s);
        for (String safety : order) {
            String key = heuristicKey(hopper, profile, safety);
            if (!WARMING.add(key)) continue;
            WARM_UP.execute(() -> {
                try {
                    Foundation f = new Foundation(hopper, profile, safetyModel(safety), new Foundation.Params());
                    double[] r = f.tightenHeuristic();
                    HEURISTIC.put(key, r[1]);
                    LOGGER.info("tile_route A* bound {} {}: {} per m (weighting's own {}), {} ms", profile, safety, r[1], r[0], Math.round(r[2]));
                } catch (RuntimeException e) {
                    WARMING.remove(key); // the next request queues it again
                    LOGGER.warn("tile_route A* bound {} {} failed", profile, safety, e);
                }
            });
        }
    }

    /** The measured bound for (profile, safety) is ready (tests). */
    static boolean boundReady(GraphHopper hopper, String profile, String safety) {
        return HEURISTIC.containsKey(heuristicKey(hopper, profile, safety));
    }

    /** tour_repair's bound per {@link Config#heuristic}; NaN = the weighting's own bound. */
    private double heuristicFor(TileRouteRequest rq, Foundation f) {
        String key = heuristicKey(hopper, rq.profile, rq.safety);
        switch (config.heuristic) {
            case INLINE:
                return HEURISTIC.computeIfAbsent(key, k -> f.tightenHeuristic()[1]);
            case BUILT_IN:
                return Double.NaN;
            default:
                warmUp(rq.profile, rq.safety);
                return HEURISTIC.getOrDefault(key, Double.NaN);
        }
    }

    private static final org.slf4j.Logger LOGGER = org.slf4j.LoggerFactory.getLogger(TileRouter.class);

    private final GraphHopper hopper;
    private final Config config;

    public TileRouter(GraphHopper hopper, Config config) {
        this.hopper = hopper;
        this.config = config;
    }

    public Result route(TileRouteRequest rq) {
        long t0 = System.nanoTime();
        validate(rq);
        CustomModel cm = safetyModel(rq.safety);
        Candidate cand = candidate(rq);
        Foundation.Params params = new Foundation.Params();
        params.marginM = rq.zoom == 17 ? 20 : 40;
        // owner (A20, 2026-10-31): "No trails should still allow short walk to the tile"
        params.walkMaxM = !Double.isNaN(rq.walkInMaxM) ? rq.walkInMaxM : 300;
        params.walkAvoidsBarriers = com.graphhopper.trailmap.barrier.BarrierRouter.VERSION.equals(rq.barriers);
        cand.configure(params);
        Foundation f = new Foundation(hopper, rq.profile, cm, params);
        // only tour_repair's portal legs (LegOracle) search with the bound; strokes and repair never read it, but any
        // request queues the background measurement so a later tour_repair finds it ready
        if ("tour_repair".equals(rq.algorithm))
            f.dijkstra.minWeightPerM = heuristicFor(rq, f);
        else if (config.heuristic == Heuristic.BACKGROUND)
            warmUp(rq.profile, rq.safety);

        Foundation.Home home;
        try {
            home = f.home(rq.startLat, rq.startLon);
        } catch (IllegalStateException e) {
            throw new IllegalArgumentException("the start is not on a rideable road");
        }
        List<Foundation.TileInfo> infos = new ArrayList<>();
        for (String k : rq.tiles) infos.add(f.analyse(k, rq.zoom, home));
        Result res = new Result();
        res.algorithm = rq.algorithm;
        res.tiles = infos;
        res.foundationMs = (System.nanoTime() - t0) / 1_000_000;

        Candidate.Problem pb = new Candidate.Problem();
        pb.jobId = rq.seedKey != null ? rq.seedKey : String.join(",", rq.tiles);
        pb.foundation = f;
        pb.home = home;
        pb.ordered = true;
        pb.loop = true;
        if (!rq.groups.isEmpty()) {
            pb.groupRank = new HashMap<>();
            pb.groupCount = rq.groups.size();
            for (int g = 0; g < rq.groups.size(); g++)
                for (String k : rq.groups.get(g)) pb.groupRank.putIfAbsent(k, g);
        }
        Map<String, Foundation.TileInfo> walkTiles = new LinkedHashMap<>();
        for (Foundation.TileInfo ti : infos) {
            if ("reachable".equals(ti.status) || ti.walk) pb.tiles.add(ti);
            if (ti.walk) walkTiles.put(ti.tile.key, ti);
        }
        long s0 = System.nanoTime();
        Candidate.Solution sol = cand.solve(pb);
        Foundation.Plan plan = walkTiles.isEmpty() ? Foundation.routedPlan(sol.vias) : f.expandWalks(sol.vias, walkTiles);
        res.solveMs = (System.nanoTime() - s0) / 1_000_000;
        res.note = sol.note;
        res.keysHeld = home.fwd.size() + home.back.size() + f.dijkstra.peakKeys() + sol.keysHeld;

        if (rq.polish && pb.groupRank == null) {
            long p0 = System.nanoTime();
            ReusePolish rp = new ReusePolish(f, 1 / rq.polishStrength, rq.polishTrailFactor == 1 ? null : 1 / rq.polishTrailFactor);
            plan = rp.polish(plan, pb.tiles);
            res.polish = rp.stats;
            res.polishMs = (System.nanoTime() - p0) / 1_000_000;
        }
        if (rq.handover && cm != null) {
            long h0 = System.nanoTime();
            TileHandover h = new TileHandover(f);
            plan = h.pin(plan);
            res.handover = h.stats;
            res.handoverMs = (System.nanoTime() - h0) / 1_000_000;
        }
        res.vias = plan.vias;
        res.segments = plan.segments;
        res.walks = plan.walks;
        res.route = f.assemble(plan, sol.passThrough, rq.handover);
        res.totalMs = (System.nanoTime() - t0) / 1_000_000;
        return res;
    }

    /**
     * The re-use polish alone on a given plan (e.g. a stored prototype run), with the request's foundation: for the
     * comparison with tile-routing/polish/polish.mjs on the same input.
     */
    public Result polishPlan(TileRouteRequest rq, Foundation.Plan plan) {
        long t0 = System.nanoTime();
        validate(rq);
        Foundation.Params params = new Foundation.Params();
        params.marginM = rq.zoom == 17 ? 20 : 40;
        params.walkMaxM = !Double.isNaN(rq.walkInMaxM) ? rq.walkInMaxM : 300;
        params.walkAvoidsBarriers = com.graphhopper.trailmap.barrier.BarrierRouter.VERSION.equals(rq.barriers);
        candidate(rq).configure(params);
        // no A* bound: the polish never searches with the foundation's EdgeDijkstra
        Foundation f = new Foundation(hopper, rq.profile, safetyModel(rq.safety), params);
        Foundation.Home home = f.home(rq.startLat, rq.startLon);
        Result res = new Result();
        res.algorithm = rq.algorithm;
        List<Foundation.TileInfo> reach = new ArrayList<>();
        for (String k : rq.tiles) {
            Foundation.TileInfo ti = f.analyse(k, rq.zoom, home);
            res.tiles.add(ti);
            if ("reachable".equals(ti.status) || ti.walk) reach.add(ti);
        }
        ReusePolish rp = new ReusePolish(f, 1 / rq.polishStrength, rq.polishTrailFactor == 1 ? null : 1 / rq.polishTrailFactor);
        Foundation.Plan out = rp.polish(plan, reach);
        res.polish = rp.stats;
        res.vias = out.vias;
        res.segments = out.segments;
        res.walks = out.walks;
        res.route = f.assemble(out, false);
        res.totalMs = (System.nanoTime() - t0) / 1_000_000;
        return res;
    }

    /** The request's algorithm (package-private for TileRouteMemoryTest, which holds one alive to weigh it). */
    Candidate candidate(TileRouteRequest rq) {
        switch (rq.algorithm) {
            case "strokes":
                return new StrokeCoverCandidate();
            case "repair": {
                LoopRepairCandidate c = new LoopRepairCandidate(rq.variety);
                c.budgetMs = config.repairBudgetMs;
                c.clock = config.clock.get();
                return c;
            }
            case "tour_repair": {
                FreeOrderRepairCandidate c = new FreeOrderRepairCandidate(rq.variety, config.portalBudgetMs);
                c.budgetMs = config.repairBudgetMs;
                c.clock = config.clock.get();
                return c;
            }
            default:
                throw new IllegalArgumentException("algorithm must be strokes|repair|tour_repair, got " + rq.algorithm);
        }
    }

    private void validate(TileRouteRequest rq) {
        if (rq.zoom != 14 && rq.zoom != 17) throw new IllegalArgumentException("zoom must be 14 or 17, got " + rq.zoom);
        if (rq.tiles == null || rq.tiles.isEmpty()) throw new IllegalArgumentException("tiles must not be empty");
        int max = rq.zoom == 14 ? config.maxTilesZ14 : config.maxTilesZ17;
        if (rq.tiles.size() > max)
            throw new IllegalArgumentException("at most " + max + " tiles at zoom " + rq.zoom + ", got " + rq.tiles.size());
        Set<String> seen = new HashSet<>();
        int n = 1 << rq.zoom;
        for (String k : rq.tiles) {
            if (k == null || !k.matches("\\d+-\\d+")) throw new IllegalArgumentException("tile key must be x-y, got " + k);
            String[] xy = k.split("-");
            long x = Long.parseLong(xy[0]), y = Long.parseLong(xy[1]);
            if (x >= n || y >= n) throw new IllegalArgumentException("tile " + k + " is outside zoom " + rq.zoom);
            if (!seen.add(k)) throw new IllegalArgumentException("tile " + k + " is listed twice");
        }
        for (List<String> g : rq.groups)
            for (String k : g)
                if (!seen.contains(k)) throw new IllegalArgumentException("group tile " + k + " is not in tiles");
        if (Double.isNaN(rq.startLat) || Double.isNaN(rq.startLon) || Math.abs(rq.startLat) > 90 || Math.abs(rq.startLon) > 180)
            throw new IllegalArgumentException("start must be a valid lat, lon");
        if (hopper.getProfile(rq.profile) == null) throw new IllegalArgumentException("unknown profile " + rq.profile);
        safetyModel(rq.safety);
        if (!Set.of("off", "mild", "strong").contains(rq.variety))
            throw new IllegalArgumentException("variety must be off|mild|strong, got " + rq.variety);
        if (!Double.isNaN(rq.walkInMaxM) && (rq.walkInMaxM < 0 || rq.walkInMaxM > 300))
            throw new IllegalArgumentException("walk_in_max_m must be 0..300");
        if (!(rq.polishStrength >= 1 && rq.polishStrength <= 20)) throw new IllegalArgumentException("polish_strength must be 1..20");
        if (!(rq.polishTrailFactor >= 1 && rq.polishTrailFactor <= 20)) throw new IllegalArgumentException("polish_trail_factor must be 1..20");
    }
}
