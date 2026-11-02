package com.graphhopper.resources;

import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.ResponsePath;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.trailmap.personal.PersonalFamiliarity;
import com.graphhopper.trailmap.personal.variations.BitOfNew;
import com.graphhopper.trailmap.personal.variations.MainPath;
import com.graphhopper.util.CustomModel;
import com.graphhopper.util.Helper;
import com.graphhopper.util.InstructionList;
import com.graphhopper.util.PMap;
import com.graphhopper.util.PointList;

import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

/**
 * {@code algorithm=bit_of_new} on {@code POST /personal_route}: the main route exactly as /personal_route routes the
 * same request, then "bit of new" variations of it ({@link BitOfNew}): round 4's alternatives, then round-3 A's
 * that are not the same as an earlier path. Each path in the response carries {@code variation_source}
 * ({@code main | round4 | a3}) and, for the variations, a {@code variation} object (its deviations: from/to % of the
 * main route, length, extra length, max distance from main; its whole length and extra length).
 *
 * <p>Request fields: {@code bit_of_new_history} {@code none} (default) | {@code avoid} | {@code explore} = the detour
 * flavour of both sources; avoid uses the user's {@code bit_of_new_lifetime_cells} data set (default
 * {@code cycling_lifetime}), explore the {@code bit_of_new_recent_cells} one (default {@code cycling_last12m}), both of
 * {@code personal_user}. {@code bit_of_new_max_km} (default 25, at most 60): a longer main route is returned without
 * variations and {@code hints.bit_of_new.skipped} says why; so is the route of a user without the history data set
 * that avoid / explore need. Only two points (A to B).
 */
final class TrailmapBitOfNew {

    static final String ALGORITHM = "bit_of_new";
    static final String HISTORY = "bit_of_new_history";
    static final String LIFETIME_CELLS = "bit_of_new_lifetime_cells";
    static final String RECENT_CELLS = "bit_of_new_recent_cells";

    /** Loads one of the request user's data sets (null when the user has no such data set). */
    interface Histories {
        PersonalFamiliarity get(String cells) throws IOException;
    }

    final BitOfNew.History history;
    /** Longest main route that gets variations, km (request field; default and hard upper bound). */
    static final String MAX_KM = "bit_of_new_max_km";
    static final double DEFAULT_MAX_KM = 25, HARD_MAX_KM = 60;

    final String lifetimeCells, recentCells;
    final double maxKm;
    final boolean addedEdgeId;
    /** null when the route was too long for variations (then only the main route is returned). */
    BitOfNew.Result result;

    private TrailmapBitOfNew(BitOfNew.History history, String lifetimeCells, String recentCells, double maxKm, boolean addedEdgeId) {
        this.history = history;
        this.lifetimeCells = lifetimeCells;
        this.recentCells = recentCells;
        this.maxKm = maxKm;
        this.addedEdgeId = addedEdgeId;
    }

    /**
     * null unless the request asks for {@code algorithm=bit_of_new}; then the request is turned into the plain
     * request of its main route (default algorithm, the bit_of_new fields removed) plus the {@code edge_id} path
     * detail the generators start from.
     */
    static TrailmapBitOfNew take(GHRequest request) {
        if (!ALGORITHM.equalsIgnoreCase(Helper.isEmpty(request.getAlgorithm()) ? "" : request.getAlgorithm().trim()))
            return null; // any other request is left exactly as it is
        if (request.getPoints().size() != 2)
            throw new IllegalArgumentException("algorithm=" + ALGORITHM + " needs exactly two points (A to B)");
        PMap hints = request.getHints();
        BitOfNew.History h = BitOfNew.History.of(hints.getString(HISTORY, "none"));
        String life = hints.getString(LIFETIME_CELLS, "cycling_lifetime"), recent = hints.getString(RECENT_CELLS, "cycling_last12m");
        double maxKm = hints.getDouble(MAX_KM, DEFAULT_MAX_KM);
        if (!(maxKm > 0 && maxKm <= HARD_MAX_KM))
            throw new IllegalArgumentException(MAX_KM + " must be > 0 and <= " + HARD_MAX_KM);
        hints.remove(HISTORY);
        hints.remove(LIFETIME_CELLS);
        hints.remove(RECENT_CELLS);
        hints.remove(MAX_KM);
        request.setAlgorithm("");
        return new TrailmapBitOfNew(h, life, recent, maxKm, !request.getPathDetails().contains(MainPath.EDGE_ID));
    }

    /**
     * Computes the variations of the routed main route and adds them to the response as further paths. The
     * generators start from the main route with the {@code edge_id} path detail; when the request did not ask for
     * it, the same request is routed once more with it (the response keeps the plain request's main route, so its
     * points stay exactly as /personal_route gives them).
     */
    void apply(GraphHopper hopper, GHRequest request, GHResponse rsp, Function<GHRequest, GHResponse> router,
               Histories histories) throws IOException {
        ResponsePath main = rsp.getBest(), base = main;
        // the generators were built for 5-10 km legs; a 90-135 km route took 10-64 s on 8999, so longer routes get none
        if (main.getDistance() > maxKm * 1000) {
            skip(rsp, String.format(java.util.Locale.ROOT, "route %.1f km is longer than %s=%.0f km", main.getDistance() / 1000, MAX_KM, maxKm));
            return;
        }
        // a user without the history data set gets the main route only, as /personal_route routes such a user as /route
        PersonalFamiliarity lifetime = null, last12 = null;
        if (history == BitOfNew.History.AVOID) {
            lifetime = histories.get(lifetimeCells);
            if (lifetime == null) {
                skip(rsp, HISTORY + "=avoid needs personal_user with a '" + lifetimeCells + "' data set");
                return;
            }
        } else if (history == BitOfNew.History.EXPLORE) {
            last12 = histories.get(recentCells);
            if (last12 == null) {
                skip(rsp, HISTORY + "=explore needs personal_user with a '" + recentCells + "' data set");
                return;
            }
        }
        if (addedEdgeId) {
            List<String> asked = request.getPathDetails();
            List<String> pd = new ArrayList<>(asked);
            pd.add(MainPath.EDGE_ID);
            request.setPathDetails(pd);
            try {
                GHResponse withIds = router.apply(request);
                if (withIds.hasErrors())
                    throw new IllegalStateException("bit_of_new: routing the main route with edge ids failed: " + withIds.getErrors());
                base = withIds.getBest();
            } finally {
                request.setPathDetails(asked);
            }
        }
        // the variations follow the request's own custom_model (e.g. the app's ferry exclusion), as the main route does
        PMap weightingHints = new PMap();
        if (request.getCustomModel() != null)
            weightingHints.putObject(CustomModel.KEY, request.getCustomModel());
        Weighting profileW = hopper.createWeighting(hopper.getProfile(request.getProfile()), weightingHints);
        result = BitOfNew.compute(hopper.getBaseGraph(), hopper.getEncodingManager(), hopper.getLocationIndex(), profileW,
                base, history, lifetime, last12);
        boolean ele = main.getPoints().is3D();
        for (BitOfNew.Variation v : result.offered) {
            ResponsePath p = new ResponsePath();
            PointList pl = new PointList(v.coords.size(), ele);
            for (double[] c : v.coords) {
                if (ele) pl.add(c[1], c[0], c.length > 2 ? c[2] : 0);
                else pl.add(c[1], c[0]);
            }
            p.setPoints(pl);
            p.setDistance(v.lengthM);
            // travel time estimated from the main route's average speed (the generators work on geometry and cost)
            p.setTime(main.getDistance() > 0 ? Math.round(main.getTime() * v.lengthM / main.getDistance()) : 0);
            p.setWaypoints(main.getWaypoints());
            // its edges, which the re-create via points pin plain /route onto
            if (v.edgeIds != null) p.addPathDetails(Map.of(MainPath.EDGE_ID, v.edgeIds));
            p.setInstructions(new InstructionList(0, hopper.getTranslationMap().getWithFallBack(request.getLocale())));
            if (ele) {
                double up = 0, down = 0;
                for (int i = 1; i < pl.size(); i++) {
                    double d = pl.getEle(i) - pl.getEle(i - 1);
                    if (d > 0) up += d;
                    else down -= d;
                }
                p.setAscend(up);
                p.setDescend(down);
            }
            rsp.add(p);
        }
        Map<String, Object> info = new LinkedHashMap<>();
        info.put("history", history.key());
        info.put("round4", result.round4.size());
        info.put("a3", result.a3.size());
        info.put("a3_dropped_as_duplicate", result.a3Duplicates);
        info.put("ms_round4", Math.round(result.msRound4));
        info.put("ms_a3", Math.round(result.msA3));
        info.put("ms_total", Math.round(result.msTotal));
        rsp.getHints().putObject("bit_of_new", info);
    }

    /** No variations: the main route only, with the reason in {@code hints.bit_of_new.skipped}. */
    /** W26 (flag): no variations for a main route that crosses a barrier; the main route alone, with the reason */
    void skipForBarrierCrossing(GHResponse rsp) {
        skip(rsp, "barrier_crossing: the main route crosses a closed barrier");
    }

    private void skip(GHResponse rsp, String why) {
        Map<String, Object> info = new LinkedHashMap<>();
        info.put("history", history.key());
        info.put("skipped", why);
        rsp.getHints().putObject("bit_of_new", info);
    }

    /** Marks each serialised path with its source and, for the variations, their deviations. */
    void decorate(ObjectNode json, double mainDistance) {
        ArrayNode paths = (ArrayNode) json.get("paths");
        if (paths == null || paths.isEmpty()) return;
        ((ObjectNode) paths.get(0)).put("variation_source", "main");
        if (result == null) return; // route too long: no variations
        for (int k = 0; k < result.offered.size() && k + 1 < paths.size(); k++) {
            BitOfNew.Variation v = result.offered.get(k);
            ObjectNode p = (ObjectNode) paths.get(k + 1);
            p.put("variation_source", v.source);
            ObjectNode var = p.putObject("variation");
            var.put("length_m", Math.round(v.lengthM));
            var.put("extra_m", Math.round(v.lengthM - mainDistance));
            ArrayNode devs = var.putArray("deviations");
            for (BitOfNew.Deviation d : v.deviations) {
                ObjectNode o = devs.addObject();
                o.put("from_pct", Math.round(d.fromPct));
                o.put("to_pct", Math.round(d.toPct));
                o.put("length_m", Math.round(d.lengthM));
                o.put("extra_m", Math.round(d.extraM));
                o.put("max_dist_m", Math.round(d.maxDistM));
            }
        }
    }
}
