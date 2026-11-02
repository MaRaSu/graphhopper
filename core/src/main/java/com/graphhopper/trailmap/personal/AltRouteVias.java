/*
 * Trailmap - re-create via points for /route alternatives
 */
package com.graphhopper.trailmap.personal;

import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.ResponsePath;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.ev.EncodedValueLookup;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.util.details.PathDetail;
import com.graphhopper.util.details.PathDetailsBuilderFactory;
import com.graphhopper.util.details.PathDetailsFromEdges;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;
import java.util.Map;
import java.util.function.Function;

/**
 * Via points for plain {@code /route algorithm=alternative_route} that make the app's rebuild ride each alternative
 * again. The app stores a chosen alternative's {@code via_points} with the segment and rebuilds it with plain
 * {@code /route} [A, ...via_points, B] (no pass_through, the segment's first-point heading). AlternativeRoute's own
 * three vias sit on junctions, where a rebuild's turn is free, so on turn-cost profiles the rebuild can take another
 * way.
 *
 * <p>Per path, {@link RecreateVias#pin} replays the path's own vias the way the app does; when they reproduce it they
 * are left exactly as they are. Otherwise mid-edge vias are pinned, and kept only when their rebuild is identical or
 * strictly closer than the old vias' (never worse than before). Once the request's {@link #BUDGET_MS} is spent, the
 * remaining paths keep their own vias, as do paths longer than {@link #MAX_KM}.
 */
public final class AltRouteVias {

    private static final Logger LOGGER = LoggerFactory.getLogger(AltRouteVias.class);
    /**
     * Time for all of a request's alternatives (owner, 2026-10-31: 500 ms on /route). Once spent, the remaining paths
     * are not even checked; a check already running finishes (one plain route of the path's length).
     * {@code -Dtrailmap.alt_vias.budget_ms} overrides it (measurements only).
     */
    public static final long BUDGET_MS = Long.getLong("trailmap.alt_vias.budget_ms", 500);
    /** Vias per alternative at most (owner, 2026-10-31: cap 30). */
    public static final int MAX_VIAS = 30;
    /**
     * Longer alternatives keep their own vias unchecked: on the flexible-mode server one check route alone costs
     * seconds there (129 km: ~2.5 s), which the budget cannot stop once started. {@code -Dtrailmap.alt_vias.max_km}
     * overrides it (measurements only).
     */
    public static final double MAX_KM = Double.parseDouble(System.getProperty("trailmap.alt_vias.max_km", "40"));

    private AltRouteVias() {
    }

    /**
     * @param paths         the alternatives as computed (on the query graph), in response order
     * @param responsePaths their response paths, whose via points this sets
     * @param router        plain /route as the server answers it
     */
    public static void apply(BaseGraph graph, LocationIndex locationIndex, EncodedValueLookup lookup,
                             PathDetailsBuilderFactory detailsFactory, Function<GHRequest, GHResponse> router,
                             GHRequest request, List<Path> paths, List<ResponsePath> responsePaths, Weighting weighting,
                             Graph queryGraph) {
        GHRequest plain = RecreateVias.plainRequest(request);
        if (plain == null || paths.size() != responsePaths.size()) return;
        double multiplier = request.getHints().getBool("points_encoded", true)
                ? request.getHints().getDouble("points_encoded_multiplier", 1e5) : 0;
        long deadline = System.nanoTime() + BUDGET_MS * 1_000_000L;
        // the alternatives first, the best path (path 0, rebuilt as A to B, which mostly needs nothing) with what is left
        for (int n = 0; n < paths.size(); n++) {
            int i = (n + 1) % paths.size();
            if (System.nanoTime() > deadline) break; // the rest keep their own vias, unchecked
            Path path = paths.get(i);
            ResponsePath shown = responsePaths.get(i);
            if (!path.isFound() || path.getDistance() > MAX_KM * 1000) continue;
            ResponsePath probe = withEdges(path, shown, lookup, detailsFactory, weighting, queryGraph);
            RecreateVias.Result r = new RecreateVias(graph, locationIndex, router, plain, MAX_VIAS, multiplier, deadline)
                    .pin(probe);
            // the path's own vias stay exactly as they were (no rounding, no copy) when they are kept
            if (!r.keptGiven) shown.setAltRouteViaPoints(r.vias.size() > 0 ? r.vias : null);
            if (!r.identical)
                LOGGER.info("alternative_route: path {} not re-created by plain /route: {}", i, r.toMap());
        }
    }

    /**
     * A computed path as {@link RecreateVias#pin} takes it: its full points, its edge_id detail and the shown path's
     * via points. The response path has no edge_id detail unless the request asked for it, and adding it there
     * would change the simplified points.
     */
    public static ResponsePath withEdges(Path path, ResponsePath shown, EncodedValueLookup lookup,
                                         PathDetailsBuilderFactory detailsFactory, Weighting weighting, Graph queryGraph) {
        ResponsePath probe = new ResponsePath();
        probe.setPoints(path.calcPoints());
        Map<String, List<PathDetail>> details = PathDetailsFromEdges.calcDetails(
                path, lookup, weighting, List.of(RecreateVias.EDGE_ID), detailsFactory, 0, queryGraph);
        probe.addPathDetails(details);
        probe.setAltRouteViaPoints(shown.getAltRouteViaPoints());
        return probe;
    }
}
