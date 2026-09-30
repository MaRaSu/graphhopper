package com.graphhopper.trailmap.fixroute;

import com.graphhopper.GraphHopper;

import java.util.concurrent.ExecutorService;

/** Selects the {@code /fix_route} engine from {@code options.engine}. */
public final class FixRouteEngine {

    public static final String GEOMETRY = "geometry";
    public static final String MATCHING = "matching";

    private FixRouteEngine() {
    }

    public static FixRouteResponse fix(GraphHopper hopper, ExecutorService pool, FixRouteRequest req) {
        String engine = req != null && req.options != null && req.options.engine != null ? req.options.engine : MATCHING;
        if (MATCHING.equals(engine)) return new MatchPipelineFixer(hopper, pool).fix(req);
        if (GEOMETRY.equals(engine)) return new RouteFixer(hopper, pool).fix(req);
        throw new IllegalArgumentException("options.engine must be geometry | matching");
    }
}
