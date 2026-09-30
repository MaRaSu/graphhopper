package com.graphhopper.trailmap.fixroute;

import com.graphhopper.GraphHopper;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.ev.RoadClass;
import com.graphhopper.routing.ev.RoadEnvironment;
import com.graphhopper.routing.ev.Subnetwork;
import com.graphhopper.routing.util.DefaultSnapFilter;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.util.SnapPreventionEdgeFilter;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.trailmap.fixroute.FixRouteRequest.LatLng;
import com.graphhopper.trailmap.fixroute.FixRouteResponse.Endpoint;
import com.graphhopper.util.CustomModel;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.PMap;
import com.graphhopper.util.shapes.GHPoint;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;

/**
 * {@code /fix_route} engine (design doc §3). Given a saved route — fixed waypoints, per-leg
 * routing settings and the saved reference track — returns, per leg, the fewest extra waypoints
 * that make the client's own routing reproduce the reference, and names the stretches that today's
 * road network can no longer reproduce.
 *
 * <p>One verdict everywhere: a (sub-)leg is accepted when the route the client would draw
 * ({@link ClientRoute}) follows the corresponding slice of the reference in order, within the
 * materiality tolerance ({@link PathSimilarity}). The map matcher is used only to translate the
 * reference onto today's graph so waypoints can be placed on real roads.
 */
public class RouteFixer {

    private static final DistanceCalcEarth DIST = DistanceCalcEarth.DIST_EARTH;
    /** The client's heading penalty [s] when a segment carries no explicit one. */
    public static final double CLIENT_HEADING_PENALTY = 60;
    /** Saved leg lengths as an alignment tie-breaker (design doc §6). */
    private static final double ALIGN_LENGTH_WEIGHT = 0.5;
    /** Distance before a waypoint over which the reference's arrival direction is measured. */
    private static final double ARRIVAL_BEARING_M = 15.0;
    /** Candidates closer than this along the reference collapse. */
    private static final double CANDIDATE_MIN_GAP_M = 1.0;
    /** Spacing of the global candidate grid along the reference [m]. */
    private static final double CANDIDATE_GRID_M = 20.0;
    /** Max distance between a candidate's grid arc and where its snap projects back [m]. */
    private static final double CANDIDATE_PROJECTION_DRIFT_M = 3.0;
    /** Unroutable-cause sampling step along the stretch [m]. */
    private static final double CAUSE_SAMPLE_M = 20.0;
    /** A reference vertex further than this from every edge is off-road (legacy spike tips). */
    private static final double OFF_ROAD_M = 3.0;
    /** Hysteresis between accepting an existing leg and creating a new one [m]; see follows(). */
    static final double NEW_LEG_MARGIN_M = 2.0;
    /** Max distance between a waypoint and its heading-aware start snap for a heading to be used. */
    private static final double HEADING_SNAP_STABLE_M = 1.0;
    /** Absolute length slack for legs the server creates [m] (see follows()). */
    private static final double NEW_LEG_LEN_SLACK_M = 5.0;
    /** A reroute running back along the reference for more than this is a spur [m]. */
    private static final double SPUR_MIN_M = 20.0;
    /** How far behind / ahead of an anchor a spur is looked for [m]. */
    private static final double SPUR_SEARCH_M = 5_000.0;
    private static final int SPUR_MAX_MOVES = 3;
    /** Forced-waypoint rule: a step this short [m] counts as pinned (about two grid steps). */
    private static final double FORCED_STEP_M = 45.0;
    /** Forced-waypoint rule: this many pinned steps in a row make a forced run. */
    private static final int FORCED_RUN_MIN = 3;
    /** A way counts as running along the reference within this angle [deg]. */
    private static final double ALIGN_BEARING_DEG = 35.0;
    /** Reference window around a waypoint for the snap-mismatch test [m]. */
    private static final double MISMATCH_SEARCH_M = 50.0;

    private final GraphHopper hopper;
    private final ExecutorService pool;

    /** @param pool legs run on it in parallel; null = sequential */
    public RouteFixer(GraphHopper hopper, ExecutorService pool) {
        this.hopper = hopper;
        this.pool = pool;
    }

    // ------------------------------------------------------------------------------------------

    public FixRouteResponse fix(FixRouteRequest req) {
        long t0 = System.currentTimeMillis();
        validate(req);
        FixRouteRequest.Options opt = req.options != null ? req.options : new FixRouteRequest.Options();
        List<double[]> wps = new ArrayList<>();
        for (FixRouteRequest.Waypoint w : req.waypoints) wps.add(new double[]{w.coordinates.lat, w.coordinates.lng});
        double[] saved = new double[req.segments.size()];
        for (int i = 0; i < saved.length; i++) {
            Double s = req.segments.get(i).savedLengthM;
            saved[i] = s == null ? Double.NaN : s;
        }
        WaypointAligner.Result al = new WaypointAligner(opt.waypointAlignMaxM, ALIGN_LENGTH_WEIGHT)
                .align(req.reference, wps, saved);
        ReferenceTrack ref = new ReferenceTrack(req.reference);
        long deadline = t0 + opt.timeBudgetMs;

        List<FixRouteResponse.Leg> legs = new ArrayList<>();
        if (pool == null) {
            for (int i = 0; i < req.segments.size(); i++) legs.add(fixLeg(req, opt, ref, al, i, deadline));
        } else {
            List<Future<FixRouteResponse.Leg>> fs = new ArrayList<>();
            for (int i = 0; i < req.segments.size(); i++) {
                final int k = i;
                fs.add(pool.submit(() -> fixLeg(req, opt, ref, al, k, deadline)));
            }
            try {
                for (Future<FixRouteResponse.Leg> f : fs) legs.add(f.get());
            } catch (Exception e) {
                throw new IllegalStateException("leg processing failed: " + e.getMessage(), e);
            }
        }

        FixRouteResponse rsp = new FixRouteResponse();
        rsp.legs = legs;
        for (FixRouteResponse.Leg l : legs) {
            rsp.stats.probes += l.metrics.probes;
            rsp.stats.addedWaypoints += l.metrics.addedWaypoints;
            rsp.stats.byStatus.merge(l.status, 1, Integer::sum);
        }
        rsp.stats.totalMs = System.currentTimeMillis() - t0;
        if (req.debug) {
            Map<String, Object> dbg = new LinkedHashMap<>();
            List<Map<String, Object>> align = new ArrayList<>();
            for (WaypointAligner.Placement p : al.placements()) {
                Map<String, Object> m = new LinkedHashMap<>();
                m.put("arc_m", p.aligned() ? round1(p.arcM()) : null);
                m.put("offset_m", p.aligned() ? round1(p.offsetM()) : null);
                m.put("candidates", p.candidateCount());
                align.add(m);
            }
            dbg.put("waypoint_alignment", align);
            dbg.put("reference_length_m", round1(ref.lengthM()));
            rsp.debug = dbg;
        }
        return rsp;
    }

    static void validate(FixRouteRequest req) {
        if (req == null) throw new IllegalArgumentException("Empty body");
        if (req.reference == null || req.reference.size() < 2)
            throw new IllegalArgumentException("reference must contain at least 2 [lat,lng] entries");
        for (double[] p : req.reference) {
            if (p == null || p.length < 2 || !Double.isFinite(p[0]) || !Double.isFinite(p[1])
                    || Math.abs(p[0]) > 90 || Math.abs(p[1]) > 180)
                throw new IllegalArgumentException("reference entries must be valid [lat,lng]");
        }
        if (req.waypoints == null || req.waypoints.size() < 2)
            throw new IllegalArgumentException("need at least 2 waypoints");
        if (req.segments == null || req.segments.size() != req.waypoints.size() - 1)
            throw new IllegalArgumentException("segments.length must equal waypoints.length - 1");
        for (int i = 0; i < req.segments.size(); i++) {
            FixRouteRequest.Segment s = req.segments.get(i);
            if (s.start == null || !s.start.equals(req.waypoints.get(i).id)
                    || s.end == null || !s.end.equals(req.waypoints.get(i + 1).id))
                throw new IllegalArgumentException("segment " + i + " must run from waypoint " + i + " to " + (i + 1));
            if (s.isFollowRoads() && (s.profile == null || s.profile.isEmpty()))
                throw new IllegalArgumentException("segment " + i + ": profile is required for followRoads");
        }
        String p = req.unroutablePolicy == null ? FixRouteRequest.POLICY_REROUTE : req.unroutablePolicy;
        if (!p.equals(FixRouteRequest.POLICY_REROUTE) && !p.equals(FixRouteRequest.POLICY_COORDINATES)
                && !p.equals(FixRouteRequest.POLICY_BOTH))
            throw new IllegalArgumentException("unroutable_policy must be reroute | coordinates | both");
    }

    // ------------------------------------------------------------------------------------------
    // Per leg
    // ------------------------------------------------------------------------------------------

    /**
     * Diagnostic (tests only): for leg {@code i}, every candidate arc at which a SINGLE added
     * waypoint makes both halves pass the created-leg verdict. Shows whether the greedy search's
     * waypoint count is minimal.
     */
    List<Double> singleWaypointSolutions(FixRouteRequest req, int i) {
        FixRouteRequest.Options opt = req.options != null ? req.options : new FixRouteRequest.Options();
        List<double[]> wps = new ArrayList<>();
        for (FixRouteRequest.Waypoint w : req.waypoints) wps.add(new double[]{w.coordinates.lat, w.coordinates.lng});
        WaypointAligner.Result al = new WaypointAligner(opt.waypointAlignMaxM, ALIGN_LENGTH_WEIGHT).align(req.reference, wps, null);
        ReferenceTrack ref = new ReferenceTrack(req.reference);
        FixRouteRequest.Segment seg = req.segments.get(i);
        FixRouteRequest.Waypoint wa = req.waypoints.get(i), wb = req.waypoints.get(i + 1);
        ClientRoute.Settings settings = new ClientRoute.Settings(seg.profile, seg.customModel, req.snapPreventions);
        LegState st = new LegState();
        st.usable = usableFilter(settings);
        WaypointAligner.Placement pa = al.placements().get(i), pb = al.placements().get(i + 1);
        st.arcA = pa.arcM();
        st.arcB = pb.arcM();
        st.offA = pa.offsetM();
        st.offB = pb.offsetM();
        List<Node> nodes = new ArrayList<>();
        nodes.add(new Node(new GHPoint(wa.coordinates.lat, wa.coordinates.lng), pa.arcM(), wa.id, Double.NaN));
        nodes.addAll(candidates(ref, pa.arcM(), pb.arcM(), st, opt));
        nodes.add(new Node(new GHPoint(wb.coordinates.lat, wb.coordinates.lng), pb.arcM(), wb.id, Double.NaN));
        double penalty = seg.headingPenalty != null ? seg.headingPenalty : CLIENT_HEADING_PENALTY;
        List<Double> ok = new ArrayList<>();
        int last = nodes.size() - 1;
        for (int k = 1; k < last; k++) {
            Step s1 = probe(nodes, 0, k, seg.initialHeading, ref, settings, penalty, opt, st);
            if (!s1.ok()) continue;
            Double h = s1.route() == null || Double.isNaN(s1.route().exitHeading()) ? null : s1.route().exitHeading();
            if (probe(nodes, k, last, h, ref, settings, penalty, opt, st).ok()) ok.add(nodes.get(k).arcM());
        }
        return ok;
    }

    /** One point the search may route between: the leg's two fixed waypoints, or a new candidate. */
    private record Node(GHPoint pos, double arcM, String fixedId, double anchorWindowM) {
        boolean isFixed() {
            return fixedId != null;
        }
    }

    /** An accepted or failed step between two nodes. */
    private record Step(int from, int to, boolean ok, Double heading, ClientRoute.Leg route) {
    }

    /** An assembled piece of a leg: an accepted sub-leg, or a non-reproducible stretch. */
    private record Item(int from, int to, boolean ok, Double heading, ClientRoute.Leg route,
                        double coreFromArc, double coreToArc) {
        Item(int from, int to, boolean ok, Double heading, ClientRoute.Leg route) {
            this(from, to, ok, heading, route, Double.NaN, Double.NaN);
        }

        /** Same stretch, new anchors (the non-reproducible core stays what it was). */
        Item withAnchors(int newFrom, int newTo, ClientRoute.Leg newRoute) {
            return new Item(newFrom, newTo, false, null, newRoute, coreFromArc, coreToArc);
        }
    }

    /**
     * Spur rule (design doc §3.7): the route around a non-reproducible stretch must not run into
     * the dead end and turn back. If the reroute from the stretch's first anchor starts by running
     * BACKWARDS along the reference (or, symmetrically, arrives at the last anchor from beyond it),
     * the anchor is moved outward — accepted sub-legs on that side are absorbed into the reroute —
     * to where the reroute actually leaves (re-joins) the reference. Fixed waypoints are never
     * crossed. Returns the (possibly shifted) index of the stretch item.
     */
    private int moveAnchorsOffSpurs(List<Item> items, int b, List<Node> nodes, ReferenceTrack ref,
                                    ClientRoute.Settings settings, double penalty, FixRouteRequest.Options opt,
                                    LegState st) {
        for (int iter = 0; iter < SPUR_MAX_MOVES; iter++) {
            Item it = items.get(b);
            Node na = nodes.get(it.from()), nb = nodes.get(it.to());
            ClientRoute.Leg rr = ClientRoute.route(hopper, settings, na.pos(), List.of(), nb.pos(), null, penalty);
            st.probes++;
            items.set(b, it.withAnchors(it.from(), it.to(), rr));
            if (!rr.ok()) return b;
            boolean moved = false;
            double back = runAlongReference(rr.points(), true, ref, na.arcM(), opt);
            if (back < na.arcM() - SPUR_MIN_M && b > 0 && items.get(b - 1).ok()) {
                Item prev = items.get(b - 1);
                int m = nodeAtOrBefore(nodes, prev.from(), prev.to(), back);
                Step cut = m > prev.from() ? probe(nodes, prev.from(), m, prev.heading(), ref, settings, penalty, opt, st) : null;
                if (cut != null && cut.ok()) {
                    // Split the preceding sub-leg at the divergence point: it keeps its validated start.
                    items.set(b - 1, new Item(prev.from(), m, true, cut.heading(), cut.route()));
                    items.set(b, items.get(b).withAnchors(m, items.get(b).to(), null));
                } else {
                    items.remove(b - 1);
                    b--;
                    items.set(b, items.get(b).withAnchors(prev.from(), items.get(b).to(), null));
                }
                moved = true;
            }
            it = items.get(b);
            if (moved) {
                rr = ClientRoute.route(hopper, settings, nodes.get(it.from()).pos(), List.of(), nb.pos(), null, penalty);
                st.probes++;
                items.set(b, it.withAnchors(it.from(), it.to(), rr));
                if (!rr.ok()) return b;
            }
            double fwd = runAlongReference(rr.points(), false, ref, nb.arcM(), opt);
            if (fwd > nb.arcM() + SPUR_MIN_M && b + 1 < items.size() && items.get(b + 1).ok()) {
                Item next = items.get(b + 1);
                int m = nodeAtOrAfter(nodes, next.from(), next.to(), fwd);
                Step cut = m < next.to() ? probe(nodes, m, next.to(), null, ref, settings, penalty, opt, st) : null;
                if (cut != null && cut.ok()) {
                    items.set(b + 1, new Item(m, next.to(), true, cut.heading(), cut.route()));
                    items.set(b, items.get(b).withAnchors(items.get(b).from(), m, null));
                } else {
                    items.remove(b + 1);
                    items.set(b, items.get(b).withAnchors(items.get(b).from(), next.to(), null));
                }
                moved = true;
            }
            if (!moved) return b;
        }
        return b;
    }

    /**
     * Forced-waypoint rule (owner review 2026-09-24, routes 42833 / 45779): when the reference can
     * only be followed by pinning a waypoint every grid step — a run of at least
     * {@link #FORCED_RUN_MIN} accepted steps each no longer than {@link #FORCED_STEP_M}, possibly
     * interleaved with failed steps — the profile will not take that path (e.g. against a one-way, or
     * through an impossible roundabout cut). Such a run is not a fix: it becomes one non-reproducible
     * stretch, reported and routed around like any other.
     */
    private static List<Step> mergeForcedRuns(List<Step> steps, List<Node> nodes) {
        List<Step> out = new ArrayList<>();
        int k = 0;
        while (k < steps.size()) {
            int j = k, forced = 0;
            while (j < steps.size() && isShortOrBad(steps.get(j), nodes)) {
                if (steps.get(j).ok()) forced++;
                j++;
            }
            if (forced >= FORCED_RUN_MIN) {
                out.add(new Step(steps.get(k).from(), steps.get(j - 1).to(), false, null, null));
                k = j;
            } else {
                out.add(steps.get(k));
                k++;
            }
        }
        return out;
    }

    private static boolean isShortOrBad(Step s, List<Node> nodes) {
        return !s.ok() || nodes.get(s.to()).arcM() - nodes.get(s.from()).arcM() <= FORCED_STEP_M;
    }

    /** Candidate node in (lo, hi) with the largest arc at or before {@code arc}; {@code lo} if none. */
    private static int nodeAtOrBefore(List<Node> nodes, int lo, int hi, double arc) {
        int best = lo;
        for (int k = lo + 1; k < hi; k++) if (nodes.get(k).arcM() <= arc) best = k;
        return best;
    }

    /** Candidate node in (lo, hi) with the smallest arc at or after {@code arc}; {@code hi} if none. */
    private static int nodeAtOrAfter(List<Node> nodes, int lo, int hi, double arc) {
        for (int k = lo + 1; k < hi; k++) if (nodes.get(k).arcM() >= arc) return k;
        return hi;
    }

    /**
     * Follows the route from its start (or backwards from its end) while it stays on the reference,
     * and returns the lowest (highest) arc reached: where the route actually leaves (re-joins) the
     * reference. Only the part of the reference behind (ahead of) the anchor is considered.
     */
    private static double runAlongReference(List<double[]> pts, boolean fromStart, ReferenceTrack ref,
                                            double anchorArc, FixRouteRequest.Options opt) {
        List<double[]> line = PathSimilarity.resample(pts);
        double extreme = anchorArc;
        double lo = fromStart ? anchorArc - SPUR_SEARCH_M : anchorArc - opt.materialityMaxM;
        double hi = fromStart ? anchorArc + opt.materialityMaxM : anchorArc + SPUR_SEARCH_M;
        int n = line.size();
        for (int k = 0; k < n; k++) {
            double[] p = line.get(fromStart ? k : n - 1 - k);
            double[] pr = ref.projectBetween(p, lo, hi);
            if (pr[1] > opt.materialityMaxM) break;
            extreme = fromStart ? Math.min(extreme, pr[0]) : Math.max(extreme, pr[0]);
        }
        return extreme;
    }

    private static final class LegState {
        int probes;
        boolean timedOut;
        /** Edges the leg's profile may use (+ snap preventions); computed once per leg. */
        EdgeFilter usable;
        /** Max stub length [m] tolerated at a fixed end whose own snap is off the reference; 0 = none. */
        double startStubM, endStubM;
        double arcA, arcB;
        /** Distance of each fixed end waypoint from its aligned point on the reference [m]. */
        double offA, offB;
    }

    FixRouteResponse.Leg fixLeg(FixRouteRequest req, FixRouteRequest.Options opt, ReferenceTrack ref,
                                WaypointAligner.Result al, int i, long deadline) {
        long t0 = System.currentTimeMillis();
        FixRouteRequest.Segment seg = req.segments.get(i);
        FixRouteRequest.Waypoint wa = req.waypoints.get(i), wb = req.waypoints.get(i + 1);
        FixRouteResponse.Leg out = new FixRouteResponse.Leg();
        out.index = i;
        out.segmentId = seg.id;
        WaypointAligner.Placement pa = al.placements().get(i), pb = al.placements().get(i + 1);
        if (pa.aligned() && pb.aligned()) {
            out.metrics.refFromM = round1(pa.arcM());
            out.metrics.refToM = round1(pb.arcM());
        }
        if (!seg.isFollowRoads()) {
            out.status = FixRouteResponse.SKIPPED;
            out.segments.add(unchanged(seg));
            return finish(out, t0);
        }
        if (!pa.aligned() || !pb.aligned()) {
            out.status = FixRouteResponse.UNALIGNED;
            out.segments.add(unchanged(seg));
            return finish(out, t0);
        }
        if (System.currentTimeMillis() > deadline) {
            out.status = FixRouteResponse.NOT_PROCESSED;
            out.segments.add(unchanged(seg));
            return finish(out, t0);
        }

        ClientRoute.Settings settings = new ClientRoute.Settings(seg.profile, seg.customModel, req.snapPreventions);
        double penalty = seg.headingPenalty != null ? seg.headingPenalty : CLIENT_HEADING_PENALTY;
        GHPoint a = new GHPoint(wa.coordinates.lat, wa.coordinates.lng);
        GHPoint b = new GHPoint(wb.coordinates.lat, wb.coordinates.lng);
        List<GHPoint> via = new ArrayList<>();
        if (seg.viaPoints != null) for (LatLng v : seg.viaPoints) via.add(new GHPoint(v.lat, v.lng));
        LegState st = new LegState();
        st.usable = usableFilter(settings);

        // 3a quick check: the leg exactly as the client routes it.
        ClientRoute.Leg quick = ClientRoute.route(hopper, settings, a, via, b, seg.initialHeading, penalty);
        st.probes++;
        st.arcA = pa.arcM();
        st.arcB = pb.arcM();
        st.offA = pa.offsetM();
        st.offB = pb.offsetM();
        if (quick.ok()) {
            // §3.8 waypoint snap mismatch: the waypoint's own snap is off the reference (typically
            // on a crossing road at a junction), which forces a stub no added waypoint can remove.
            // Reported (the client decides whether to move the waypoint) and tolerated in the verdict.
            FixRouteResponse.SnapMismatch sm = snapMismatch(wa.id, quick.points().get(0), ref, pa.arcM(), st.usable, opt);
            if (sm != null) {
                out.waypointSnapMismatch = sm;
                st.startStubM = 2 * sm.offsetM + 2 * opt.materialityMaxM;
            }
            FixRouteResponse.SnapMismatch se = snapMismatch(wb.id, quick.points().get(quick.points().size() - 1),
                    ref, pb.arcM(), st.usable, opt);
            if (se != null) {
                st.endStubM = 2 * se.offsetM + 2 * opt.materialityMaxM;
                if (sm == null && i == req.segments.size() - 1) out.waypointSnapMismatch = se;
            }
            quick = trimStubs(quick, true, true, ref, st, opt);
        }
        if (req.debug && quick.ok()) {
            PathSimilarity.Result qs = similarity(quick, ref, pa.arcM(), pb.arcM(),
                    endWindow(a, quick, true, opt, st.offA), endWindow(b, quick, false, opt, st.offB), opt.materialityMaxM, st.usable);
            out.debug = new LinkedHashMap<>();
            out.debug.put("quick_frechet_within", qs.frechetWithin());
            out.debug.put("quick_route_to_ref_max_m", round1(qs.routeToRefMaxM()));
            out.debug.put("quick_ref_to_route_max_m", round1(qs.refToRouteMaxM()));
            out.debug.put("quick_route_m", round1(qs.routeLengthM()));
            out.debug.put("quick_slice_m", round1(qs.referenceLengthM()));
        }
        if (quick.ok() && follows(quick, ref, pa.arcM(), pb.arcM(), endWindow(a, quick, true, opt, st.offA),
                endWindow(b, quick, false, opt, st.offB), opt, 0, st.usable)) {
            out.status = FixRouteResponse.OK;
            out.metrics.acceptedBy = "geometry";
            out.segments.add(unchanged(seg));
            out.metrics.probes = st.probes;
            return finish(out, t0);
        }

        // 3b heading repair: one extra route with the reference's direction of travel at the start.
        boolean prevRouted = i > 0 && req.segments.get(i - 1).isFollowRoads();
        if (seg.initialHeading == null && prevRouted && pa.arcM() >= ARRIVAL_BEARING_M) {
            double h = bearing(ref.pointAt(pa.arcM() - ARRIVAL_BEARING_M), ref.pointAt(pa.arcM()));
            ClientRoute.Leg hr = ClientRoute.route(hopper, settings, a, via, b, h, penalty);
            st.probes++;
            if (hr.ok()) hr = trimStubs(hr, true, true, ref, st, opt);
            if (hr.ok() && startSnapStable(a, hr) && follows(hr, ref, pa.arcM(), pb.arcM(), endWindow(a, hr, true, opt, st.offA),
                    endWindow(b, hr, false, opt, st.offB), opt, NEW_LEG_MARGIN_M, st.usable)) {
                out.status = FixRouteResponse.FIXED;
                out.metrics.acceptedBy = "heading";
                FixRouteResponse.Segment s = unchanged(seg);
                s.initialHeading = round1(h);
                s.distanceM = round1(hr.distanceM());
                out.segments.add(s);
                out.metrics.probes = st.probes;
                return finish(out, t0);
            }
        }

        // 3c full fix.
        return fullFix(req, opt, ref, seg, wa, wb, pa.arcM(), pb.arcM(), settings, penalty, st, deadline, out, t0);
    }

    private FixRouteResponse.Leg fullFix(FixRouteRequest req, FixRouteRequest.Options opt, ReferenceTrack ref,
                                         FixRouteRequest.Segment seg, FixRouteRequest.Waypoint wa,
                                         FixRouteRequest.Waypoint wb, double arcA, double arcB,
                                         ClientRoute.Settings settings, double penalty, LegState st,
                                         long deadline, FixRouteResponse.Leg out, long t0) {
        GHPoint a = new GHPoint(wa.coordinates.lat, wa.coordinates.lng);
        GHPoint b = new GHPoint(wb.coordinates.lat, wb.coordinates.lng);
        List<Node> nodes = new ArrayList<>();
        nodes.add(new Node(a, arcA, wa.id, Double.NaN));
        nodes.addAll(candidates(ref, arcA, arcB, st, opt));
        nodes.add(new Node(b, arcB, wb.id, Double.NaN));
        int last = nodes.size() - 1;

        // Search (exponential + binary), farthest-first from each cursor. A step that fails even
        // to the next node becomes part of a non-reproducible stretch.
        List<Step> steps = new ArrayList<>();
        int cursor = 0;
        Double heading = seg.initialHeading; // first sub-leg keeps the leg's stored heading
        while (cursor < last) {
            if (System.currentTimeMillis() > deadline || st.probes >= opt.maxProbes) {
                st.timedOut = true;
                break;
            }
            // Farthest first: most legs deviate only locally, so the rest often passes in one go.
            Step best = probe(nodes, cursor, last, heading, ref, settings, penalty, opt, st);
            if (!best.ok()) {
                // Exponential extension over the intermediate nodes, NOT stopping at the first
                // failure: reachability is not monotone — one bad candidate nearby (e.g. snapped onto
                // a side road) fails the short step while a longer step past it passes. Stopping
                // there declared long reproducible stretches unroutable. Then binary refinement
                // between the farthest passing node and the next probed node beyond it.
                Step lastOk = null;
                int firstFail = last;
                for (int step = 1; cursor + step < last; step *= 2) {
                    Step s = probe(nodes, cursor, cursor + step, heading, ref, settings, penalty, opt, st);
                    if (s.ok()) {
                        lastOk = s;
                        firstFail = last;
                    } else if (lastOk != null && firstFail == last) {
                        firstFail = cursor + step;
                    }
                }
                if (lastOk != null) {
                    int lo = lastOk.to() + 1, hi = firstFail - 1;
                    while (lo <= hi) {
                        int mid = (lo + hi) >>> 1;
                        Step s = probe(nodes, cursor, mid, heading, ref, settings, penalty, opt, st);
                        if (s.ok()) {
                            lastOk = s;
                            lo = mid + 1;
                        } else {
                            hi = mid - 1;
                        }
                    }
                }
                best = lastOk; // null: not even the next node is reachable faithfully
            }
            if (best == null) {
                steps.add(new Step(cursor, cursor + 1, false, null, null));
                cursor = cursor + 1;
                heading = null; // a non-reproducible stretch breaks the heading chain
            } else {
                steps.add(best);
                cursor = best.to();
                heading = best.route() == null || Double.isNaN(best.route().exitHeading())
                        ? null : best.route().exitHeading();
            }
        }
        if (req.debug) {
            if (out.debug == null) out.debug = new LinkedHashMap<>();
            List<String> trace = new ArrayList<>();
            for (Step sp : steps) {
                Node x = nodes.get(sp.from()), y = nodes.get(sp.to());
                trace.add(String.format(java.util.Locale.ROOT, "%s %.0f->%.0f%s", sp.ok() ? "ok" : "BAD",
                        x.arcM(), y.arcM(), sp.heading() == null ? "" : " h=" + Math.round(sp.heading())));
            }
            out.debug.put("steps", trace);
            out.debug.put("candidates", nodes.size() - 2);
        }
        out.metrics.probes = st.probes;
        if (st.timedOut) {
            out.status = FixRouteResponse.NOT_PROCESSED;
            out.segments.clear();
            out.segments.add(unchanged(seg));
            return finish(out, t0);
        }

        // Assemble: accepted steps → followRoads sub-legs; runs of failed steps → stretches.
        String policy = req.unroutablePolicy == null ? FixRouteRequest.POLICY_REROUTE : req.unroutablePolicy;
        steps = mergeForcedRuns(steps, nodes);
        List<Item> items = new ArrayList<>();
        for (int k = 0; k < steps.size(); k++) {
            Step s = steps.get(k);
            if (s.ok()) {
                items.add(new Item(s.from(), s.to(), true, s.heading(), s.route()));
                continue;
            }
            int from = s.from(), to = s.to();
            while (k + 1 < steps.size() && !steps.get(k + 1).ok()) to = steps.get(++k).to();
            items.add(new Item(from, to, false, null, null, nodes.get(from).arcM(), nodes.get(to).arcM()));
        }
        for (int bi = 0; bi < items.size(); bi++) {
            if (!items.get(bi).ok()) bi = moveAnchorsOffSpurs(items, bi, nodes, ref, settings, penalty, opt, st);
        }

        List<FixRouteResponse.Segment> reroute = new ArrayList<>(), coords = new ArrayList<>();
        List<FixRouteResponse.Unroutable> stretches = new ArrayList<>();
        int added = 0;
        for (Item it : items) {
            Node na = nodes.get(it.from()), nb = nodes.get(it.to());
            if (it.ok()) {
                FixRouteResponse.Segment fs = routed(na, nb, it.heading(), it.route().distanceM());
                reroute.add(fs);
                coords.add(fs);
                continue;
            }
            ClientRoute.Leg rr = it.route() != null ? it.route()
                    : ClientRoute.route(hopper, settings, na.pos(), List.of(), nb.pos(), null, penalty);
            if (it.route() == null) st.probes++;
            // Report the non-reproducible CORE; the spur rule may have moved the anchors outward.
            double coreFrom = Double.isNaN(it.coreFromArc()) ? na.arcM() : it.coreFromArc();
            double coreTo = Double.isNaN(it.coreToArc()) ? nb.arcM() : it.coreToArc();
            double refLen = coreTo - coreFrom;
            FixRouteResponse.Segment rseg = routed(na, nb, null, rr.ok() ? rr.distanceM() : 0);
            if (refLen < opt.minUnroutableM) {
                // Too short to report (e.g. a junction corner cut): route it, whatever the policy.
                reroute.add(rseg);
                coords.add(rseg);
                continue;
            }
            FixRouteResponse.Unroutable u = new FixRouteResponse.Unroutable();
            u.refFromM = round1(coreFrom);
            u.refToM = round1(coreTo);
            u.referenceLengthM = round1(refLen);
            u.rerouteLengthM = rr.ok() ? round1(rr.distanceM()) : null;
            u.cause = cause(ref, coreFrom, coreTo, settings, rr.ok(), opt);
            stretches.add(u);
            reroute.add(rseg);
            coords.add(coordinatesSegment(ref, na, nb));
        }
        for (FixRouteResponse.Segment s : reroute) if (s.end.newPoint != null) added++;

        out.metrics.probes = st.probes;
        out.metrics.addedWaypoints = added;
        out.metrics.acceptedBy = stretches.isEmpty() ? "geometry" : "none";
        if (stretches.isEmpty() && added == 0 && reroute.size() == 1 && reroute.get(0).initialHeading == null
                && seg.initialHeading == null) {
            // Nothing could be improved and what deviates is below the reporting minimum: the leg
            // stays exactly as it is — report it as such rather than as "fixed".
            out.status = FixRouteResponse.OK;
            out.metrics.acceptedBy = "tolerated";
            out.segments = new ArrayList<>(List.of(unchanged(seg)));
        } else if (stretches.isEmpty()) {
            out.status = FixRouteResponse.FIXED;
            out.segments = reroute;
        } else {
            out.status = FixRouteResponse.UNROUTABLE;
            out.unroutable = stretches;
            out.segments = policy.equals(FixRouteRequest.POLICY_COORDINATES) ? coords : reroute;
            if (policy.equals(FixRouteRequest.POLICY_BOTH)) {
                out.alternatives = new LinkedHashMap<>();
                FixRouteResponse.Alternative ar = new FixRouteResponse.Alternative();
                ar.segments = reroute;
                FixRouteResponse.Alternative ac = new FixRouteResponse.Alternative();
                ac.segments = coords;
                out.alternatives.put("reroute", ar);
                out.alternatives.put("coordinates", ac);
            }
        }
        return finish(out, t0);
    }

    /**
     * One step from {@code nodes[from]} to {@code nodes[to]}, routed exactly as the client will
     * route that sub-leg. The client applies whichever heading the server returns, so any option is
     * parity-safe; the order and the stability rule make the result survive the client's NEXT open:
     * <ul>
     *   <li>At a new waypoint, no heading is tried first. A heading changes where GH snaps the start
     *       (it picks a road matching the heading, not the nearest one), the client then stores that
     *       snapped point, and a short leg can route differently from there on the next open.</li>
     *   <li>A route with a heading is only accepted when its start snap stays at the waypoint
     *       ({@link #HEADING_SNAP_STABLE_M}).</li>
     *   <li>At the leg's fixed start the stored heading is tried first (it is the client's data),
     *       then none.</li>
     * </ul>
     */
    private Step probe(List<Node> nodes, int from, int to, Double heading, ReferenceTrack ref,
                       ClientRoute.Settings settings, double penalty, FixRouteRequest.Options opt, LegState st) {
        Node na = nodes.get(from), nb = nodes.get(to);
        List<Double> tries = new ArrayList<>(3);
        if (from == 0) {
            tries.add(heading);
            if (heading != null) tries.add(null);
        } else {
            tries.add(null);
            if (heading != null) tries.add(heading);
        }
        // Last: the reference's own direction of travel into this node (heading repair at every
        // sub-leg start). A short step that otherwise U-turns or loops at a junction usually passes
        // with it — and a later run, finding that step as an existing leg, would add exactly this.
        if (na.arcM() >= ARRIVAL_BEARING_M) {
            tries.add(bearing(ref.pointAt(na.arcM() - ARRIVAL_BEARING_M), ref.pointAt(na.arcM())));
        }
        for (Double h : tries) {
            ClientRoute.Leg leg = ClientRoute.route(hopper, settings, na.pos(), List.of(), nb.pos(), h, penalty);
            st.probes++;
            if (!leg.ok()) continue;
            if (h != null && !startSnapStable(na.pos(), leg)) continue;
            leg = trimStubs(leg, na.isFixed(), nb.isFixed(), ref, st, opt);
            double wa = na.isFixed() ? endWindow(na.pos(), leg, true, opt, st.offA) : na.anchorWindowM();
            double wb = nb.isFixed() ? endWindow(nb.pos(), leg, false, opt, st.offB) : nb.anchorWindowM();
            if (follows(leg, ref, na.arcM(), nb.arcM(), wa, wb, opt, NEW_LEG_MARGIN_M, st.usable)) return new Step(from, to, true, h, leg);
        }
        return new Step(from, to, false, null, null);
    }

    /** Snap mismatch at a fixed waypoint, or null when its route end lies on the reference. */
    private FixRouteResponse.SnapMismatch snapMismatch(String id, double[] routeEnd, ReferenceTrack ref, double arc,
                                                       EdgeFilter usable, FixRouteRequest.Options opt) {
        double[] pr = ref.projectNear(routeEnd, arc, MISMATCH_SEARCH_M);
        if (pr[1] <= opt.materialityMaxM) return null;
        double[] onRef = ref.pointAt(arc);
        Snap s = hopper.getLocationIndex().findClosest(onRef[0], onRef[1], usable);
        if (!s.isValid()) return null;
        FixRouteResponse.SnapMismatch m = new FixRouteResponse.SnapMismatch();
        m.waypointId = id;
        m.suggested = new LatLng(round6(s.getSnappedPoint().lat), round6(s.getSnappedPoint().lon));
        m.offsetM = round1(DIST.calcDist(routeEnd[0], routeEnd[1], s.getSnappedPoint().lat, s.getSnappedPoint().lon));
        return m.offsetM > opt.materialityMaxM / 2 ? m : null;
    }

    /**
     * Drops the stub a mismatched fixed waypoint forces (route points off the reference at that
     * end), up to the stub length allowed for that waypoint; beyond it nothing is trimmed, so a real
     * deviation is never hidden.
     */
    private static ClientRoute.Leg trimStubs(ClientRoute.Leg leg, boolean atStart, boolean atEnd, ReferenceTrack ref,
                                             LegState st, FixRouteRequest.Options opt) {
        List<double[]> pts = leg.points();
        int from = 0, to = pts.size() - 1;
        if (atStart && st.startStubM > 0) from = stubEnd(pts, true, ref, st.arcA, st.startStubM, opt);
        if (atEnd && st.endStubM > 0) to = stubEnd(pts, false, ref, st.arcB, st.endStubM, opt);
        if (from == 0 && to == pts.size() - 1) return leg;
        if (to - from < 1) return leg;
        List<double[]> kept = new ArrayList<>(pts.subList(from, to + 1));
        double cut = PathSimilarity.length(pts) - PathSimilarity.length(kept);
        return new ClientRoute.Leg(true, null, kept, leg.edgeKeys(), leg.distanceM() - cut, leg.exitHeading());
    }

    /** Index of the first (last) route point back on the reference, or the end index if too far. */
    private static int stubEnd(List<double[]> pts, boolean fromStart, ReferenceTrack ref, double arc, double maxM,
                               FixRouteRequest.Options opt) {
        int n = pts.size();
        double walked = 0;
        for (int k = 0; k < n; k++) {
            int idx = fromStart ? k : n - 1 - k;
            if (k > 0) {
                double[] a = pts.get(fromStart ? idx - 1 : idx + 1), b = pts.get(idx);
                walked += DIST.calcDist(a[0], a[1], b[0], b[1]);
            }
            if (walked > maxM) return fromStart ? 0 : n - 1;
            if (ref.projectNear(pts.get(idx), arc, maxM + MISMATCH_SEARCH_M)[1] <= opt.materialityMaxM / 2) return idx;
        }
        return fromStart ? 0 : n - 1;
    }

    /** The route starts where the waypoint is: the client will store (nearly) the same point. */
    private static boolean startSnapStable(GHPoint wp, ClientRoute.Leg leg) {
        double[] s = leg.points().get(0);
        return DIST.calcDist(wp.lat, wp.lon, s[0], s[1]) <= HEADING_SNAP_STABLE_M;
    }

    /**
     * Waypoint candidates between the leg's ends: points of the matched reference on today's graph,
     * excluding points the client's own snapping would move (off-network, or on an edge class the
     * client's snap preventions exclude, e.g. a ferry).
     */
    private List<Node> candidates(ReferenceTrack ref, double arcA, double arcB, LegState st,
                                  FixRouteRequest.Options opt) {
        // Candidates sit on a GLOBAL arc grid of the reference, so a later run over a shorter leg
        // (after this run's waypoints were applied) is offered the same positions. Each is the
        // nearest road the leg's profile may use, with the client's snap preventions — i.e. exactly
        // where the client's /route would snap that point — found WITHOUT the HMM matcher: the
        // matcher's choice depends on how much of the track it sees, which made candidates differ
        // between runs. A candidate on the wrong one of two parallel roads is harmless: the
        // geometric verdict rejects any sub-leg through it.
        EdgeFilter usable = st.usable;
        // A sub-leg shorter than the tolerance "follows" anything, so candidates keep at least one
        // tolerance from each other and from the leg's own ends: otherwise a later run could step a
        // few metres into a stretch this run found non-reproducible (idempotence).
        double gap = Math.max(CANDIDATE_MIN_GAP_M, opt.materialityMaxM);
        List<Node> out = new ArrayList<>();
        double lastArc = arcA;
        for (ReferenceTrack.ArcPoint ap : ref.slicePointsWithGrid(arcA, arcB, CANDIDATE_GRID_M)) {
            if (!ap.grid()) continue;
            double arc = ap.arcM();
            if (arc <= arcA + gap || arc >= arcB - gap || arc - lastArc < gap) continue;
            double[] p = ap.point();
            Snap snap = hopper.getLocationIndex().findClosest(p[0], p[1], usable);
            if (!snap.isValid() || snap.getQueryDistance() > opt.offnetSnapM) continue;
            GHPoint sp = snap.getSnappedPoint();
            GHPoint pos = new GHPoint(round6(sp.lat), round6(sp.lon));
            // The node's position along the reference is where its SNAP projects — exactly what a
            // later run computes when it aligns this waypoint. If that drifts from the grid point
            // (a snap onto a road meeting the reference at an angle), the node is not used: a
            // shifted boundary would make the next run see a different leg.
            double[] pr = ref.projectNear(new double[]{pos.lat, pos.lon}, arc, CANDIDATE_GRID_M);
            if (Math.abs(pr[0] - arc) > CANDIDATE_PROJECTION_DRIFT_M) continue;
            double nodeArc = pr[0];
            if (nodeArc <= arcA + gap || nodeArc >= arcB - gap || nodeArc - lastArc < gap) continue;
            out.add(new Node(pos, nodeArc, null, opt.materialityMaxM + snap.getQueryDistance() + 5));
            lastArc = nodeArc;
        }
        return out;
    }

    /** Edges the leg's profile (+ custom model) may use, outside isolated islands, minus snap preventions. */
    private EdgeFilter usableFilter(ClientRoute.Settings s) {
        PMap hints = new PMap();
        if (s.customModel() != null) hints.putObject(CustomModel.KEY, s.customModel());
        Weighting w = hopper.createWeighting(hopper.getProfile(s.profile()), hints);
        EdgeFilter usable = new DefaultSnapFilter(w,
                hopper.getEncodingManager().getBooleanEncodedValue(Subnetwork.key(s.profile())));
        EdgeFilter prevent = snapPreventionFilter(s.snapPreventions());
        return prevent == null ? usable : e -> usable.accept(e) && prevent.accept(e);
    }

    /** Accepts every edge except those the snap preventions exclude; null when there are none. */
    private EdgeFilter snapPreventionFilter(List<String> preventions) {
        if (preventions == null || preventions.isEmpty()) return null;
        EnumEncodedValue<RoadClass> rc = hopper.getEncodingManager().getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<RoadEnvironment> re = hopper.getEncodingManager().getEnumEncodedValue(RoadEnvironment.KEY, RoadEnvironment.class);
        return new SnapPreventionEdgeFilter(EdgeFilter.ALL_EDGES, rc, re, preventions);
    }

    /**
     * Why a stretch cannot be reproduced, from what exists next to the reference:
     * no road of any kind → {@code removed}; roads exist but the leg's profile / custom model
     * cannot use them → {@code restricted}; usable roads on both sides but no connection that
     * follows the reference → {@code disconnected}.
     */
    private String cause(ReferenceTrack ref, double from, double to, ClientRoute.Settings s, boolean rerouted,
                         FixRouteRequest.Options opt) {
        return new UnroutableCause(hopper).cause(ref, from, to, s, rerouted, opt.offnetSnapM);
    }

    // ------------------------------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------------------------------

    /**
     * @param marginM 0 when judging the client's EXISTING leg; {@link #NEW_LEG_MARGIN_M} when
     *                accepting something the server creates (a new sub-leg or heading). The margin is
     *                hysteresis: once applied, the client moves waypoints onto GH's snapped route ends
     *                and resamples nothing the same way, so a newly created leg must still pass at the
     *                full tolerance next time (idempotence) rather than sit exactly on the threshold.
     */
    private boolean follows(ClientRoute.Leg leg, ReferenceTrack ref, double arcA, double arcB,
                            double windowA, double windowB, FixRouteRequest.Options opt, double marginM,
                            EdgeFilter usable) {
        List<double[]> pts = leg.points();
        List<double[]> slice = LegacySpikes.remove(
                ref.anchoredSlice(arcA, arcB, pts.get(0), pts.get(pts.size() - 1), windowA, windowB), p -> offRoad(p, usable));
        double tol = Math.max(1.0, opt.materialityMaxM - marginM);
        // Created legs get a tight absolute length slack: with the generous one, a candidate that
        // snapped onto a side road produces an out-and-back spur the length test would absorb.
        double lenSlack = marginM > 0 ? Math.min(opt.materialityLenSlackM, NEW_LEG_LEN_SLACK_M) : opt.materialityLenSlackM;
        return PathSimilarity.compare(pts, slice, tol).follows(opt.materialityLenRatio, lenSlack);
    }

    private PathSimilarity.Result similarity(ClientRoute.Leg leg, ReferenceTrack ref, double arcA, double arcB,
                                             double windowA, double windowB, double tol, EdgeFilter usable) {
        List<double[]> pts = leg.points();
        List<double[]> slice = LegacySpikes.remove(
                ref.anchoredSlice(arcA, arcB, pts.get(0), pts.get(pts.size() - 1), windowA, windowB), p -> offRoad(p, usable));
        return PathSimilarity.compare(pts, slice, tol);
    }

    /**
     * True when no edge the leg's profile may use lies within {@link #OFF_ROAD_M} of the point. A
     * legacy spike tip can sit on a way the profile cannot use (a footway for a bike profile); the
     * client's GH route can never go there either, so for spike detection that is "off-road".
     */
    boolean offRoad(double[] p, EdgeFilter usable) {
        Snap near = hopper.getLocationIndex().findClosest(p[0], p[1], usable);
        return !near.isValid() || near.getQueryDistance() > OFF_ROAD_M;
    }

    /**
     * Anchoring window at a fixed waypoint: how far the slice end may be moved inward to where the
     * route actually starts / ends. Two things put the saved track's slice end away from the route's
     * end, and the window covers the larger:
     * <ul>
     *   <li>a legacy spike (road → off-road waypoint → road): its arm is about as long as the
     *       waypoint's distance to the route's own snapped end point;</li>
     *   <li>the waypoint sitting off its aligned point on the track, e.g. the first waypoint (pinned
     *       to the track start) after the client moved it onto the road, leaving the saved track's
     *       off-road start arm inside the first leg (route 22708: 31 m).</li>
     * </ul>
     * Plus the tolerance; nothing larger, or a short leg could trim its slice away and pass.
     */
    private static double endWindow(GHPoint wp, ClientRoute.Leg leg, boolean start, FixRouteRequest.Options opt,
                                    double alignOffsetM) {
        double[] e = start ? leg.points().get(0) : leg.points().get(leg.points().size() - 1);
        double toRoute = DIST.calcDist(wp.lat, wp.lon, e[0], e[1]);
        return Math.max(toRoute, Double.isNaN(alignOffsetM) ? 0 : alignOffsetM) + opt.materialityMaxM;
    }

    private static FixRouteResponse.Segment unchanged(FixRouteRequest.Segment seg) {
        FixRouteResponse.Segment s = new FixRouteResponse.Segment();
        s.type = seg.type;
        s.start = Endpoint.fixed(seg.start);
        s.end = Endpoint.fixed(seg.end);
        s.initialHeading = seg.initialHeading;
        s.viaPoints = seg.viaPoints == null || seg.viaPoints.isEmpty() ? null : seg.viaPoints;
        s.trackCoordinates = seg.trackCoordinates;
        s.distanceM = seg.savedLengthM == null ? 0 : seg.savedLengthM;
        return s;
    }

    private static FixRouteResponse.Segment routed(Node a, Node b, Double heading, double distanceM) {
        FixRouteResponse.Segment s = new FixRouteResponse.Segment();
        s.type = FixRouteRequest.TYPE_FOLLOW_ROADS;
        s.start = endpoint(a);
        s.end = endpoint(b);
        s.initialHeading = heading == null ? null : round1(heading);
        s.distanceM = round1(distanceM);
        return s;
    }

    private static FixRouteResponse.Segment coordinatesSegment(ReferenceTrack ref, Node a, Node b) {
        FixRouteResponse.Segment s = new FixRouteResponse.Segment();
        s.type = FixRouteRequest.TYPE_COORDINATES;
        s.start = endpoint(a);
        s.end = endpoint(b);
        List<LatLng> track = new ArrayList<>();
        track.add(new LatLng(a.pos().lat, a.pos().lon));
        List<double[]> slice = ref.slice(a.arcM(), b.arcM());
        for (int i = 1; i < slice.size() - 1; i++) track.add(new LatLng(slice.get(i)[0], slice.get(i)[1]));
        track.add(new LatLng(b.pos().lat, b.pos().lon));
        s.trackCoordinates = track;
        double d = 0;
        for (int i = 1; i < track.size(); i++)
            d += DIST.calcDist(track.get(i - 1).lat, track.get(i - 1).lng, track.get(i).lat, track.get(i).lng);
        s.distanceM = round1(d);
        return s;
    }

    private static Endpoint endpoint(Node n) {
        return n.isFixed() ? Endpoint.fixed(n.fixedId()) : Endpoint.added(n.pos().lat, n.pos().lon);
    }

    private static FixRouteResponse.Leg finish(FixRouteResponse.Leg out, long t0) {
        out.metrics.ms = System.currentTimeMillis() - t0;
        return out;
    }

    private static double bearing(double[] from, double[] to) {
        return com.graphhopper.util.AngleCalc.ANGLE_CALC.calcAzimuth(from[0], from[1], to[0], to[1]);
    }

    static double round6(double v) {
        return Math.round(v * 1e6) / 1e6;
    }

    private static double round1(double v) {
        return Math.round(v * 10) / 10.0;
    }
}
