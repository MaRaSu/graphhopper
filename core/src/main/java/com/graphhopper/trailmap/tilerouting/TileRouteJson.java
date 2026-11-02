package com.graphhopper.trailmap.tilerouting;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.graphhopper.trailmap.tilerouting.foundation.Foundation;
import com.graphhopper.trailmap.tilerouting.foundation.Via;

/**
 * The tour as JSON, in the prototype runner's waypoints.json format (format 1: waypoints, headings, roles,
 * passThrough, segments + walks when the tour walks in), which the app and tile-routing/verify/verify.mjs read, plus
 * per-tile status and stats.
 */
public final class TileRouteJson {

    private TileRouteJson() {
    }

    public static ObjectNode write(ObjectMapper json, TileRouteRequest rq, TileRouter.Result r) {
        ObjectNode o = json.createObjectNode();
        o.put("format", 1).put("algorithm", rq.algorithm).put("profile", rq.profile).put("safety", rq.safety);
        if (!"strokes".equals(rq.algorithm)) o.put("variety", rq.variety);
        o.putArray("home").add(round6(rq.startLat)).add(round6(rq.startLon));
        ArrayNode wp = o.putArray("waypoints"), hd = o.putArray("headings"), roles = o.putArray("roles");
        for (Via v : r.vias) {
            wp.addArray().add(round6(v.lat)).add(round6(v.lon));
            if (Double.isNaN(v.heading)) hd.addNull();
            else hd.add(Math.round(v.heading * 10) / 10.0);
            roles.add(v.tileKey == null ? v.role : v.role + ":" + v.tileKey);
        }
        o.put("passThrough", false);
        ArrayNode seg = o.putArray("segments");
        for (String s : r.segments) seg.add(s);
        if (!r.walks.isEmpty()) {
            ArrayNode wa = o.putArray("walks");
            for (Foundation.Walk w : r.walks) {
                ObjectNode wo = wa.addObject().put("tile", w.tile).put("kind", w.kind);
                wo.set("a", json.createArrayNode().add(round6(w.aLat)).add(round6(w.aLon)));
                wo.set("t", json.createArrayNode().add(round6(w.tLat)).add(round6(w.tLon)));
                wo.set("b", json.createArrayNode().add(round6(w.bLat)).add(round6(w.bLon)));
                wo.put("inM", Math.round(w.inM * 10) / 10.0).put("outM", Math.round(w.outM * 10) / 10.0);
            }
        }
        ArrayNode tiles = o.putArray("tiles");
        int fetched = 0, unreachable = 0, walked = 0;
        for (Foundation.TileInfo ti : r.tiles) {
            ObjectNode t = tiles.addObject().put("tile", ti.tile.key);
            boolean ridden = r.route != null && r.route.error == null && Foundation.fetched(ti.tile, r.route.coords);
            String status = ti.walk ? "walk" : "reachable".equals(ti.status) ? (ridden ? "fetched" : "missed") : ti.status;
            t.put("status", status);
            if (ti.reason != null) t.put("reason", ti.reason);
            if ("fetched".equals(status)) fetched++;
            else if ("walk".equals(status)) walked++;
            else if ("unreachable".equals(status)) unreachable++;
        }
        ObjectNode st = o.putObject("stats");
        st.put("requested", r.tiles.size()).put("fetched", fetched).put("walked", walked).put("unreachable", unreachable);
        if (r.route != null) {
            st.put("distanceM", Math.round(r.route.distanceM)).put("walkM", Math.round(r.route.walkM));
            if (r.route.error != null) st.put("routeError", r.route.error);
        }
        st.put("waypoints", r.vias.size());
        if (r.polish != null) {
            ObjectNode p = st.putObject("polish");
            p.put("legsTried", r.polish.tried).put("legsChanged", r.polish.changed)
                    .put("kmBefore", Math.round(r.polish.kmBefore * 1000) / 1000.0).put("kmAfter", Math.round(r.polish.kmAfter * 1000) / 1000.0);
            p.putPOJO("legs", r.polish.legs);
        }
        if (r.handover != null) {
            // W21: pins added so the app's plain /route rebuilds the tour; flagged = segments it still can't (with metres)
            ObjectNode h = st.putObject("handover");
            h.put("segments", r.handover.segments).put("pinnedSegments", r.handover.pinnedSegments).put("pins", r.handover.pins);
            ArrayNode fl = h.putArray("flagged");
            for (TileHandover.Flag f : r.handover.flags)
                fl.addObject().put("segment", f.segment).put("offM", Double.isFinite(f.offM) ? Math.round(f.offM) : -1).put("reason", f.reason);
        }
        st.putObject("timings").put("foundationMs", r.foundationMs).put("solveMs", r.solveMs).put("polishMs", r.polishMs)
                .put("handoverMs", r.handoverMs).put("totalMs", r.totalMs);
        o.put("note", r.note);
        return o;
    }

    static double round6(double v) {
        return Math.round(v * 1e6) / 1e6;
    }
}
