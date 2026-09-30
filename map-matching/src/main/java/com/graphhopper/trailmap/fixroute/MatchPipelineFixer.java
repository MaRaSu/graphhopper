package com.graphhopper.trailmap.fixroute;

import com.graphhopper.GraphHopper;
import com.graphhopper.matching.MapMatching;
import com.graphhopper.matching.MatchResult;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.ev.RoadClass;
import com.graphhopper.routing.ev.RoadEnvironment;
import com.graphhopper.routing.ev.Subnetwork;
import com.graphhopper.routing.util.DefaultSnapFilter;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.util.SnapPreventionEdgeFilter;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.trailmap.convert.RegionSegmenter;
import com.graphhopper.trailmap.convert.TrackRegion;
import com.graphhopper.trailmap.fixroute.FixRouteRequest.LatLng;
import com.graphhopper.trailmap.fixroute.FixRouteResponse.Endpoint;
import com.graphhopper.trailmap.matching.MatcherConfig;
import com.graphhopper.trailmap.matching.TrailmapMapMatching;
import com.graphhopper.util.CustomModel;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.PMap;
import com.graphhopper.util.shapes.GHPoint;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;

/**
 * {@code /fix_route} engine built on the proven {@code /convert_track} chain (design doc §3.6,
 * owner decision 2026-09-24): the saved reference is map-matched onto today's graph — the matched
 * road sequence is what the saved route took — the {@code /convert_track} segmenter separates what
 * today's network can reproduce from what it cannot, and the minimal-waypoint search accepts a
 * sub-leg only when the client's own route takes exactly the matched roads.
 *
 * <p>Runs next to the geometry engine ({@link RouteFixer}), which stays as the reference to compare
 * against; selected by {@code options.engine = "matching"}.
 *
 * <p>Kept from the geometry engine (validated on the corpus): waypoint alignment, the client-identical
 * request builder, legacy spike removal, the wire format, unroutable policies and cause detection.
 */
public class MatchPipelineFixer {

    private static final DistanceCalcEarth DIST = DistanceCalcEarth.DIST_EARTH;
    private static final double ALIGN_LENGTH_WEIGHT = 0.5;
    public static final double CLIENT_HEADING_PENALTY = 60;

    // Matcher + segmenter: EXACTLY the production /convert_track configuration the client sends
    // (shared/models/gh-server-route-convert.ts, confirmed 2026-09-24): gps_accuracy_m 20,
    // custom_matcher with candidate radius 3×σ, auto + adaptive σ (max 10), emission desirability
    // λ 1, β 4, segmentation v2; snap threshold and drift floor derived from the estimated σ as
    // TrailmapConvertResource does; min routed segment 40 m; detour gates at their defaults.
    static final double GPS_ACCURACY_M = 20.0;
    static final double CANDIDATE_RADIUS_SIGMA_MULT = 3.0;
    static final double AUTO_SIGMA_MAX_M = 10.0;
    static final double EMISSION_DESIRABILITY_LAMBDA = 1.0;
    static final double BETA = 4.0;
    static final double MIN_DETOUR_M = 75.0;
    static final double MAX_DETOUR_RATIO = 2.0;
    static final double MIN_ROUTED_SEGMENT_M = 40.0;
    /** Experiment switch: fixed segmenter thresholds instead of σ-derived ones (0 = σ-derived). */
    static double FIXED_SNAP_THRESHOLD_M = Double.parseDouble(System.getProperty("fix.snap", "0"));
    static double FIXED_DRIFT_FLOOR_M = Double.parseDouble(System.getProperty("fix.drift", "5"));
    /** Segmenter variant (experiment switch; production /convert_track uses v2). */
    static boolean SEGMENTATION_V2 = !"false".equals(System.getProperty("fix.segv2"));
    /** Upper bound of the matcher's 2σ thinning distance: 2 × the representative σ cap (10) + 1. */
    static final double THINNING_CLEAR_M = 2 * 10.0 + 1;

    static MatcherConfig productionMatcherConfig() {
        MatcherConfig cfg = new MatcherConfig();
        cfg.measurementErrorSigma = GPS_ACCURACY_M;
        cfg.transitionProbabilityBeta = BETA;
        cfg.candidateRadiusSigmaMult = CANDIDATE_RADIUS_SIGMA_MULT;
        cfg.autoSigma = true;
        cfg.adaptiveSigma = true;
        cfg.autoSigmaMaxM = AUTO_SIGMA_MAX_M;
        cfg.emissionDesirabilityLambda = EMISSION_DESIRABILITY_LAMBDA;
        return cfg;
    }

    /** How far along the reference a waypoint's matcher observation may move from its aligned arc. */
    static final double WAYPOINT_PROJECT_M = 300.0;
    /** Runs longer than this [m of reference] are split at a waypoint (parallel matching). */
    static final double MAX_RUN_M = 10_000.0;
    /** Matching context [m of reference] added beyond an artificial run cut. */
    static final double RUN_CONTEXT_M = 500.0;
    /** A turn at a waypoint sharper than this [deg] is a reversal (U-turn). */
    static final double REVERSAL_DEG = 150.0;
    /** Distance before / after a waypoint over which reversal and departure are measured [m]. */
    static final double REVERSAL_PROBE_M = 15.0;
    /** Forced-waypoint rule: a routed piece this short [m] counts as pinned (about two grid steps). */
    static final double FORCED_STEP_M = 45.0;
    /** Forced-waypoint rule: this many pinned pieces in a row make a forced run. */
    static final int FORCED_RUN_MIN = 3;
    /** A routed piece shorter than this [m] beside a stretch is folded into it. */
    static final double TINY_STEP_M = 12.0;
    /** A short stretch is silently routed only if its detour is at most max(2×, +this) [m]. */
    static final double SHORT_DETOUR_SLACK_M = 50.0;
    /** Pin test next to a gap: "gain" (default); "savedLength" / "shorter" = earlier variants, comparisons only. */
    static String PIN_MODE = System.getProperty("fix.pin", "gain");

    /** A waypoint next to a gap goes if the route without it is this much [m] shorter. */
    static final double PIN_SLACK_M = 20.0;
    /** A snap mismatch's suggested position is at most this far [m] from where the waypoint snaps. */
    static final double MISMATCH_MAX_M = 20.0;
    /** A matched observation this close [m of saved route] to a fixed waypoint's is "at" the waypoint. */
    static final double FIXED_END_REACH_M = 25.0;
    /** Two routes within this [m] of each other, in order, are the same route (added waypoints moot). */
    static final double SAME_ROUTE_M = 3.0;
    /** A created piece must stay this far [m] inside the deviation threshold (stable on re-runs). */
    static final double FIX_MARGIN_M = 5.0;
    /** Slack [m] on top of twice the waypoint's offset for the stub it forces. */
    static final double STUB_SLACK_M = 20.0;
    private final GraphHopper hopper;
    private final ExecutorService pool;
    private final java.util.concurrent.atomic.AtomicLong matchNs = new java.util.concurrent.atomic.AtomicLong();
    private final java.util.concurrent.atomic.AtomicLong tolNs = new java.util.concurrent.atomic.AtomicLong();
    private final java.util.concurrent.atomic.AtomicLong segNs = new java.util.concurrent.atomic.AtomicLong();

    public MatchPipelineFixer(GraphHopper hopper, ExecutorService pool) {
        this.hopper = hopper;
        this.pool = pool;
    }

    public FixRouteResponse fix(FixRouteRequest req) {
        long t0 = System.currentTimeMillis();
        RouteFixer.validate(req);
        FixRouteRequest.Options opt = req.options != null ? req.options : new FixRouteRequest.Options();
        int n = req.segments.size();
        List<double[]> wps = new ArrayList<>();
        for (FixRouteRequest.Waypoint w : req.waypoints) wps.add(new double[]{w.coordinates.lat, w.coordinates.lng});
        double[] saved = new double[n];
        for (int i = 0; i < n; i++) {
            Double s = req.segments.get(i).savedLengthM;
            saved[i] = s == null ? Double.NaN : s;
        }
        WaypointAligner.Result al = new WaypointAligner(opt.waypointAlignMaxM, ALIGN_LENGTH_WEIGHT).align(req.reference, wps, saved);
        ReferenceTrack ref = new ReferenceTrack(req.reference);
        long deadline = t0 + opt.timeBudgetMs;

        FixRouteResponse.Leg[] legs = new FixRouteResponse.Leg[n];
        List<int[]> runs = new ArrayList<>();
        List<Integer> longLegs = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            FixRouteRequest.Segment seg = req.segments.get(i);
            boolean aligned = al.placements().get(i).aligned() && al.placements().get(i + 1).aligned();
            if (!seg.isFollowRoads() || !aligned) {
                legs[i] = simple(req, al, i, seg.isFollowRoads() ? FixRouteResponse.UNALIGNED : FixRouteResponse.SKIPPED);
                continue;
            }
            // A long leg is fixed on its own (LongLegFixer): no matching, so it is in no run.
            if (al.placements().get(i + 1).arcM() - al.placements().get(i).arcM() >= opt.fixLongLegM) {
                longLegs.add(i);
                continue;
            }
            int[] last = runs.isEmpty() ? null : runs.get(runs.size() - 1);
            // Long runs are cut at a waypoint every ~MAX_RUN_M of reference, so their matches run in
            // parallel; a fixed waypoint is a forced observation anyway, so little context is lost.
            boolean tooLong = last != null
                    && al.placements().get(i).arcM() - al.placements().get(last[0]).arcM() > MAX_RUN_M;
            // A waypoint where the saved route reverses (U-turn at the waypoint) ends a run: the
            // matcher cannot U-turn at an observation and would loop around a block to get there.
            boolean reverses = last != null && reversesAt(ref, al.placements().get(i).arcM());
            boolean continues = last != null && last[1] == i - 1 && sameSettings(req.segments.get(i - 1), seg) && !reverses;
            if (continues && !tooLong) last[1] = i;
            else {
                if (continues) last[3] = 1; // artificial cut: both sides get matching context
                runs.add(new int[]{i, i, continues ? 1 : 0, 0});
            }
        }
        List<Future<?>> fs = new ArrayList<>();
        for (int[] run : runs) {
            Runnable job = () -> processRun(req, opt, ref, al, run[0], run[1], run[2] == 1, run[3] == 1, deadline, legs);
            if (pool == null) job.run();
            else fs.add(pool.submit(job));
        }
        for (int i : longLegs) {
            Runnable job = () -> legs[i] = fixLongLeg(req, opt, ref, al, i, deadline);
            if (pool == null) job.run();
            else fs.add(pool.submit(job));
        }
        try {
            for (Future<?> f : fs) f.get();
        } catch (Exception e) {
            throw new IllegalStateException("run processing failed: " + e.getMessage(), e);
        }

        FixRouteResponse rsp = new FixRouteResponse();
        for (FixRouteResponse.Leg l : legs) {
            rsp.legs.add(l);
            rsp.stats.probes += l.metrics.probes;
            rsp.stats.addedWaypoints += l.metrics.addedWaypoints;
            rsp.stats.byStatus.merge(l.status, 1, Integer::sum);
        }
        rsp.stats.totalMs = System.currentTimeMillis() - t0;
        rsp.stats.matchingMs = matchNs.get() / 1_000_000;
        rsp.stats.fixingMs = rsp.stats.totalMs - rsp.stats.matchingMs;
        if (req.debug) {
            rsp.debug = new LinkedHashMap<>();
            rsp.debug.put("match_ms", matchNs.get() / 1_000_000);
            rsp.debug.put("segment_ms", segNs.get() / 1_000_000);
            rsp.debug.put("tolerance_ms", tolNs.get() / 1_000_000);
        }
        return rsp;
    }

    /**
     * A long leg ({@link FixRouteRequest.Options#fixLongLegM}): today's best route stands unless it
     * leaves the saved route's corridor; then a few hidden via points pull it back, in the same
     * segment (see {@link LongLegFixer}). A leg that already has via points (a chosen alternative
     * route, or a previous fix) is left as it is.
     */
    private FixRouteResponse.Leg fixLongLeg(FixRouteRequest req, FixRouteRequest.Options opt, ReferenceTrack ref,
                                            WaypointAligner.Result al, int i, long deadline) {
        long t0 = System.currentTimeMillis();
        if (t0 > deadline) return simple(req, al, i, FixRouteResponse.NOT_PROCESSED);
        FixRouteRequest.Segment seg = req.segments.get(i);
        FixRouteResponse.Leg out = simple(req, al, i, FixRouteResponse.OK);
        out.metrics.acceptedBy = "long_leg";
        if (seg.viaPoints != null && !seg.viaPoints.isEmpty()) {
            out.metrics.acceptedBy = "long_leg_has_vias";
            return out;
        }
        ClientRoute.Settings settings = new ClientRoute.Settings(seg.profile, seg.customModel, req.snapPreventions);
        FixRouteRequest.Waypoint wa = req.waypoints.get(i), wb = req.waypoints.get(i + 1);
        GHPoint pa = new GHPoint(wa.coordinates.lat, wa.coordinates.lng), pb = new GHPoint(wb.coordinates.lat, wb.coordinates.lng);
        double penalty = seg.headingPenalty != null ? seg.headingPenalty : CLIENT_HEADING_PENALTY;
        List<double[]> saved = ref.slice(al.placements().get(i).arcM(), al.placements().get(i + 1).arcM());
        String p = seg.profile == null ? "" : seg.profile;
        boolean walk = p.contains("foot") || p.contains("hike") || p.contains("walk") || p.contains("run");
        LongLegFixer.Result r = new LongLegFixer(hopper).fix(settings, usableFilter(settings), weighting(settings),
                pa, pb, seg.initialHeading, penalty, saved, walk, opt);
        out.metrics.probes = r.probes();
        out.metrics.ms = System.currentTimeMillis() - t0;
        if (req.debug) {
            out.debug = new LinkedHashMap<>();
            out.debug.put("long_leg", String.format(java.util.Locale.ROOT, "%s: corridor %.0f m, outside today %.0f m, with vias %.0f m, vias %d",
                    r.note(), LongLegFixer.corridorWidth(opt, PathSimilarity.length(saved)), r.offTodayM(), r.offFixedM(), r.vias().size()));
        }
        if ("no_route".equals(r.note())) {
            out.status = FixRouteResponse.NO_ROUTE;
            out.metrics.acceptedBy = "none";
            return out;
        }
        if (r.fixed() == null) {
            out.metrics.acceptedBy = "long_leg_" + r.note();
            return out;
        }
        out.status = FixRouteResponse.FIXED;
        out.metrics.acceptedBy = "long_leg_vias";
        FixRouteResponse.Segment s = out.segments.get(0);
        s.viaPoints = new ArrayList<>();
        for (GHPoint v : r.vias()) s.viaPoints.add(new LatLng(RouteFixer.round6(v.lat), RouteFixer.round6(v.lon)));
        s.distanceM = RouteFixerMath.round1(r.fixed().distanceM());
        return out;
    }

    private Weighting weighting(ClientRoute.Settings s) {
        PMap hints = new PMap();
        if (s.customModel() != null) hints.putObject(CustomModel.KEY, s.customModel());
        return hopper.createWeighting(hopper.getProfile(s.profile()), hints);
    }

    /** The reference turns back (by more than {@link #REVERSAL_DEG}) at arc {@code s}. */
    static boolean reversesAt(ReferenceTrack ref, double s) {
        if (s < REVERSAL_PROBE_M || s > ref.lengthM() - REVERSAL_PROBE_M) return false;
        double in = RouteFixerMath.bearing(ref.pointAt(s - REVERSAL_PROBE_M), ref.pointAt(s));
        double out = RouteFixerMath.bearing(ref.pointAt(s), ref.pointAt(s + REVERSAL_PROBE_M));
        double diff = Math.abs(((out - in) % 360 + 540) % 360 - 180); // 0 = same direction
        return diff >= REVERSAL_DEG;
    }

    private static boolean sameSettings(FixRouteRequest.Segment a, FixRouteRequest.Segment b) {
        return Objects.equals(a.profile, b.profile)
                && Objects.equals(a.customModel == null ? null : a.customModel.toString(),
                b.customModel == null ? null : b.customModel.toString());
    }

    private static FixRouteResponse.Leg simple(FixRouteRequest req, WaypointAligner.Result al, int i, String status) {
        FixRouteResponse.Leg l = new FixRouteResponse.Leg();
        l.index = i;
        l.segmentId = req.segments.get(i).id;
        l.status = status;
        l.segments.add(unchanged(req.segments.get(i)));
        setArcs(l, al, i);
        return l;
    }

    private static void setArcs(FixRouteResponse.Leg l, WaypointAligner.Result al, int i) {
        WaypointAligner.Placement a = al.placements().get(i), b = al.placements().get(i + 1);
        if (a.aligned() && b.aligned()) {
            l.metrics.refFromM = RouteFixerMath.round1(a.arcM());
            l.metrics.refToM = RouteFixerMath.round1(b.arcM());
        }
    }

    // ------------------------------------------------------------------------------------------
    // One run: match once, segment once, then each leg.
    // ------------------------------------------------------------------------------------------

    private void processRun(FixRouteRequest req, FixRouteRequest.Options opt, ReferenceTrack ref,
                            WaypointAligner.Result al, int first, int last, boolean ctxBefore, boolean ctxAfter,
                            long deadline, FixRouteResponse.Leg[] legs) {
        FixRouteRequest.Segment seg0 = req.segments.get(first);
        ClientRoute.Settings settings = new ClientRoute.Settings(seg0.profile, seg0.customModel, req.snapPreventions);
        EdgeFilter usable = usableFilter(settings);

        // Waypoint observations lie ON the reference, at the point nearest to where the waypoint is
        // routed from (its road snap), between the neighbouring waypoints. Not at the snap itself:
        // a snap on a way beside the reference (a footway along the road) would make the matcher
        // detour out to it and back, planting a false U-turn in the matched path at the waypoint.
        // Not at the aligned arc either: for a legacy off-road waypoint that is the spike tip, off
        // any road; the snap projects to the spike's base on the road instead.
        int m = last - first + 2;
        double[] arcs = new double[m];
        double[][] pos = new double[m][];
        for (int k = 0; k < m; k++) {
            double arc = al.placements().get(first + k).arcM();
            double lo = k == 0 ? arc : al.placements().get(first + k - 1).arcM();
            double hi = k == m - 1 ? arc : al.placements().get(first + k + 1).arcM();
            FixRouteRequest.Waypoint w = req.waypoints.get(first + k);
            Snap s = hopper.getLocationIndex().findClosest(w.coordinates.lat, w.coordinates.lng, usable);
            if (s.isValid()) {
                double[] pr = ref.projectBetween(new double[]{s.getSnappedPoint().lat, s.getSnappedPoint().lon},
                        Math.max(lo, arc - WAYPOINT_PROJECT_M), Math.min(hi, arc + WAYPOINT_PROJECT_M));
                if (Double.isFinite(pr[1])) arc = pr[0];
            }
            if (k > 0 && arc < arcs[k - 1]) arc = arcs[k - 1];
            arcs[k] = arc;
            pos[k] = ref.pointAt(arc);
        }
        RunObservations obs = new RunObservations(ref, arcs, pos, THINNING_CLEAR_M / 2, p -> offRoad(p, usable),
                ctxBefore ? RUN_CONTEXT_M : 0, ctxAfter ? RUN_CONTEXT_M : 0);

        MatchResult match;
        List<TrackRegion> regions;
        List<RegionSegmenter.DetourReport> lastDetours;
        long tm = System.currentTimeMillis();
        try {
            PMap hints = new PMap();
            hints.putObject("profile", settings.profile());
            // As TrailmapConvertResource: the matcher gets the profile only. The client's custom
            // model shapes the routes (probes), not the matching.
            MapMatching.Router router = MapMatching.routerFromGraphHopper(hopper, hints);
            TrailmapMapMatching mm = new TrailmapMapMatching(hopper.getBaseGraph(),
                    (LocationIndexTree) hopper.getLocationIndex(), router, productionMatcherConfig());
            long tMatch = System.nanoTime();
            match = mm.match(obs.observations);
            matchNs.addAndGet(System.nanoTime() - tMatch);
            long tSeg = System.nanoTime();
            // Thresholds from the σ the matcher estimated, as TrailmapConvertResource does.
            Object est = mm.getStatistics().get("autoSigmaEstimatedM");
            double sigmaEst = est instanceof Number ? ((Number) est).doubleValue() : GPS_ACCURACY_M;
            double snapThreshold = FIXED_SNAP_THRESHOLD_M > 0 ? FIXED_SNAP_THRESHOLD_M
                    : com.graphhopper.trailmap.convert.TrackToRouteConverter.AUTO_SIGMA_SNAP_THRESHOLD_MULT * sigmaEst;
            double driftFloor = FIXED_SNAP_THRESHOLD_M > 0 ? FIXED_DRIFT_FLOOR_M
                    : com.graphhopper.trailmap.convert.TrackToRouteConverter.AUTO_SIGMA_DRIFT_FLOOR_MULT * sigmaEst;
            RegionSegmenter segmenter = new RegionSegmenter();
            segmenter.setSegmentationV2Enabled(SEGMENTATION_V2);
            segmenter.setDriftTrimFloorM(driftFloor);
            segmenter.setDirectRouteFn((a, b) -> {
                ClientRoute.Leg l = ClientRoute.route(hopper, settings, a, List.of(), b, null, CLIENT_HEADING_PENALTY);
                return l.ok() ? l.distanceM() : null;
            });
            regions = segmenter.segment(match, obs.observations, snapThreshold, MIN_DETOUR_M, MAX_DETOUR_RATIO,
                    MIN_ROUTED_SEGMENT_M);
            lastDetours = new ArrayList<>(segmenter.getLastDetours());
            segNs.addAndGet(System.nanoTime() - tSeg);
        } catch (Exception e) {
            for (int i = first; i <= last; i++) legs[i] = simple(req, al, i, FixRouteResponse.NOT_PROCESSED);
            return;
        }
        long matchMs = System.currentTimeMillis() - tm;

        for (int i = first; i <= last; i++) {
            if (System.currentTimeMillis() > deadline) {
                legs[i] = simple(req, al, i, FixRouteResponse.NOT_PROCESSED);
                continue;
            }
            long t0 = System.currentTimeMillis();
            FixRouteResponse.Leg out = fixLeg(req, opt, ref, al, i, obs.waypointObs[i - first],
                    obs.waypointObs[i - first + 1], obs, regions, settings);
            if (req.debug) {
                int wa = obs.waypointObs[i - first], wb = obs.waypointObs[i - first + 1];
                List<String> um = new ArrayList<>();
                for (TrackRegion r : regions) {
                    if (!(r instanceof TrackRegion.Unmatched) || r.lastObservation() < wa || r.firstObservation() > wb) continue;
                    StringBuilder sb = new StringBuilder(String.format(java.util.Locale.ROOT, "obs %d..%d arc %.0f..%.0f snaps:",
                            r.firstObservation(), r.lastObservation(), obs.arcs.get(r.firstObservation()), obs.arcs.get(r.lastObservation())));
                    for (int o = Math.max(0, r.firstObservation() - 2); o <= Math.min(match.getTracepoints().size() - 1, r.lastObservation() + 2); o++) {
                        com.graphhopper.matching.Tracepoint tp = match.getTracepoints().get(o);
                        sb.append(' ').append(o).append('=').append(tp.getDistance() == null ? "-" : String.format(java.util.Locale.ROOT, "%.0f", tp.getDistance()))
                                .append(tp.isFiltered() ? "f" : "").append(tp.isMatched() ? "" : "U");
                    }
                    um.add(sb.toString());
                }
                if (!um.isEmpty()) {
                    if (out.debug == null) out.debug = new LinkedHashMap<>();
                    out.debug.put("unmatched_regions", um);
                    List<String> det = new ArrayList<>();
                    for (RegionSegmenter.DetourReport d : lastDetours) {
                        if (d.toObs() >= wa - 2 && d.fromObs() <= wb + 2) det.add(d.fromObs() + "->" + d.toObs() + " matched " + Math.round(d.matchedLengthM()) + " straight " + Math.round(d.straightM()));
                    }
                    out.debug.put("detours", det);
                }
            }
            out.metrics.ms = System.currentTimeMillis() - t0 + (i == first ? matchMs : 0);
            legs[i] = out;
        }
    }

    /** One node of a leg's assembled piece chain. */
    private record Node(GHPoint pos, double arc, String fixedId) {
    }

    /** A routed sub-leg, or a stretch (routed == false) between two nodes. */
    private record Chunk(Node from, Node to, boolean routed, Double heading, double distanceM,
                         double coreFrom, double coreTo) {
    }

    private FixRouteResponse.Leg fixLeg(FixRouteRequest req, FixRouteRequest.Options opt, ReferenceTrack ref,
                                        WaypointAligner.Result al, int i, int woA, int woB, RunObservations obs,
                                        List<TrackRegion> regions, ClientRoute.Settings settings) {
        FixRouteRequest.Segment seg = req.segments.get(i);
        FixRouteRequest.Waypoint wa = req.waypoints.get(i), wb = req.waypoints.get(i + 1);
        GHPoint pa = new GHPoint(wa.coordinates.lat, wa.coordinates.lng), pb = new GHPoint(wb.coordinates.lat, wb.coordinates.lng);
        double penalty = seg.headingPenalty != null ? seg.headingPenalty : CLIENT_HEADING_PENALTY;
        FixRouteResponse.Leg out = new FixRouteResponse.Leg();
        out.index = i;
        out.segmentId = seg.id;
        setArcs(out, al, i);
        int probes = 0;

        // Only a DEVIATION is fixed (owner decision 2026-09-25): the leg exactly as the client routes
        // it, against the saved route. Nowhere as far off as the threshold → leave it as the router
        // draws it (a cycleway beside the old road, OSM redrawn, profile drift, drawing noise).
        Deviation.Rule rule = rule(opt, seg);
        List<GHPoint> via = new ArrayList<>();
        if (seg.viaPoints != null) for (LatLng v : seg.viaPoints) via.add(new GHPoint(v.lat, v.lng));
        ClientRoute.Leg q = ClientRoute.route(hopper, settings, pa, via, pb, seg.initialHeading, penalty);
        probes++;
        if (!q.ok()) {
            // Not even the client's own route exists between the leg's waypoints (owner, 2026-09-25:
            // 116370's legs outside the routing map were reported as 531 km "removed"). Nothing to
            // fix or to report as changed map data; the leg stays as the client has it.
            out.status = FixRouteResponse.NO_ROUTE;
            out.metrics.acceptedBy = "none";
            out.metrics.probes = probes;
            out.segments.add(unchanged(seg));
            return out;
        }
        List<double[]> savedLeg = savedSlice(ref, obs, obs.arcs.get(woA), obs.arcs.get(woB));
        // Snap mismatch (spec §5.6): a fixed waypoint whose own snap is on another way than the
        // matched road there (a footway beside the road, a wrong-way carriageway, a crossing
        // street). The route is forced from that way onto the matched road — no added waypoint
        // removes it — so that connector is not judged, and it is reported with the matched position.
        EndMatch ea = endMatch(regions, woA, true, obs), eb = endMatch(regions, woB, false, obs);
        double legConA = ea == null ? 0 : connector(pa, ea.mr(), ea.cand(), true, settings, penalty);
        double legConB = eb == null ? 0 : connector(pb, eb.mr(), eb.cand(), false, settings, penalty);
        if (legConA > 0) out.waypointSnapMismatch = mismatch(wa.id, pa, ea.mr(), ea.cand());
        if (req.debug && ea != null) {
            if (out.debug == null) out.debug = new LinkedHashMap<>();
            GHPoint ms = ea.mr().obsSnapPoints().get(ea.cand());
            double[] op = new double[]{obs.observations.get(woA).getPoint().lat, obs.observations.get(woA).getPoint().lon};
            out.debug.put("start_match", String.format(java.util.Locale.ROOT,
                    "wo %d arc %.0f obs(%.6f,%.6f) cand %d obs %d arc %.0f snap(%.6f,%.6f) con %.0f",
                    woA, obs.arcs.get(woA), op[0], op[1], ea.cand(), ea.mr().matchedObsIndices().get(ea.cand()),
                    obs.arcs.get(ea.mr().matchedObsIndices().get(ea.cand())), ms.lat, ms.lon, legConA));
        }
        if (legConB > 0 && i == req.segments.size() - 1 && out.waypointSnapMismatch == null)
            out.waypointSnapMismatch = mismatch(wb.id, pb, eb.mr(), eb.cand());
        double exclA = legConA > 0 ? legConA + rule.zoneM() : 0, exclB = legConB > 0 ? legConB + rule.zoneM() : 0;
        Deviation.Ends legEnds = Deviation.Ends.of(rule, true, true).withConnectors(exclA, exclB);
        if (req.debug && q.ok()) {
            if (out.debug == null) out.debug = new LinkedHashMap<>();
            List<String> dev = new ArrayList<>();
            for (Deviation.Stretch d : Deviation.stretches(q.points(), savedLeg, rule, legEnds))
                dev.add(String.format(java.util.Locale.ROOT, "%s %.0f..%.0f peak %.0f extra %.0f", d.onRoute() ? "route" : "saved", d.fromM(), d.toM(), d.peakM(), d.extraM()));
            out.debug.put("deviations", dev);
            out.debug.put("saved_leg", savedLeg);
            out.debug.put("saved_leg_arcs", obs.arcs.get(woA) + ".." + obs.arcs.get(woB));
        }
        if (q.ok() && Deviation.within(q.points(), savedLeg, rule, legEnds)) {
            out.status = FixRouteResponse.OK;
            out.metrics.acceptedBy = "within_threshold";
            out.metrics.probes = probes;
            out.segments.add(unchanged(seg));
            return out;
        }

        // Pieces of the leg in order: the parts of matched regions inside [woA, woB], and gaps.
        List<Chunk> chunks = new ArrayList<>();
        Node start = new Node(pa, obs.arcs.get(woA), wa.id), end = new Node(pb, obs.arcs.get(woB), wb.id);
        GapContext gc = new GapContext(settings, penalty, ref, obs, rule, start, end, exclA, exclB,
                usableFilter(settings), opt.offnetSnapM);
        int[] probeCount = {0};
        Node cursor = start;
        Double heading = seg.initialHeading;
        boolean quickDone = false;
        for (TrackRegion r : regions) {
            if (r.lastObservation() < woA || r.firstObservation() > woB) continue;
            if (!(r instanceof TrackRegion.Matched mr) || mr.edgeMatches().isEmpty()) continue;
            List<Integer> cands = mr.matchedObsIndices();
            int cA = -1, cB = -1;
            for (int c = 0; c < cands.size(); c++) {
                int o = cands.get(c);
                if (o >= woA && cA < 0) cA = c;
                if (o <= woB) cB = c;
            }
            if (cA < 0 || cB <= cA) continue;
            // The region starts (ends) AT the fixed waypoint only when its first (last) matched
            // observation is right there — not past an unmatched gap (a closed bridge next to it).
            boolean fixedStart = cursor == start && cands.get(cA) - woA <= 2
                    && obs.arcs.get(cands.get(cA)) - obs.arcs.get(woA) <= FIXED_END_REACH_M;
            boolean fixedEnd = woB - cands.get(cB) <= 2
                    && obs.arcs.get(woB) - obs.arcs.get(cands.get(cB)) <= FIXED_END_REACH_M;
            final int ca = cA, cb = cB;
            MatchedLegOptimizer[] self = new MatchedLegOptimizer[1];
            MatchedLegOptimizer lo = new MatchedLegOptimizer(hopper, mr, settings, penalty, () -> {
                long tt = System.nanoTime();
                List<MatchedLegOptimizer.TolerancePair> t = self[0].nodePairTolerances(ca, cb);
                tolNs.addAndGet(System.nanoTime() - tt);
                return t;
            });
            self[0] = lo;
            // Stub allowance at each fixed end: a waypoint whose own road snap is off the saved path
            // forces edges from that snap to the path — about twice its offset, plus slack.
            double offA = snapOffset(pa, ref, obs.arcs.get(woA), settings), offB = snapOffset(pb, ref, obs.arcs.get(woB), settings);
            // Snap mismatch (spec §5.6): the waypoint's own snap is on another way than the matched
            // road at the waypoint (a footway beside the road, a crossing street). The route is
            // forced from that way onto the matched road — no added waypoint removes it — so that
            // connector is tolerated in the verdict and reported with the matched position.
            double conA = fixedStart ? legConA : 0, conB = fixedEnd ? legConB : 0;
            lo.setStubs(fixedStart ? Math.max(stubAllowance(offA), conA + STUB_SLACK_M) : 0,
                    fixedEnd ? Math.max(stubAllowance(offB), conB + STUB_SLACK_M) : 0);

            // Quick check: the whole leg inside this region → the client's route against the edges.
            if (fixedStart && fixedEnd && !quickDone) {
                quickDone = true;
                if (req.debug && q.ok()) {
                    if (out.debug == null) out.debug = new LinkedHashMap<>();
                    out.debug.put("quick_E", java.util.Arrays.toString(lo.expected(cA, cB)));
                    out.debug.put("quick_A", java.util.Arrays.toString(com.graphhopper.trailmap.shared.EdgeKeyMatching.dedupConsecutive(q.edgeKeys())));
                    out.debug.put("cands", cA + ".." + cB + " of " + cands.size() + " obs " + cands.get(cA) + ".." + cands.get(cB) + " wo " + woA + ".." + woB);
                }
                // No edge-verdict shortcut to OK here: the deviation gate above already found a
                // deviation, and the matcher agreeing with the route's roads does not undo it (it
                // can smooth over a real deviation — owner review, 49302:275).
            }
            // Gap before this region part → stretch from the cursor to the region's first node.
            Node regionStart = fixedStart ? start : candNode(mr, cA, obs);
            if (!fixedStart) {
                chunks.addAll(fillGap(new Gap(cursor, regionStart, cursor == start ? heading : null), gc, probeCount));
                heading = null;
            }
            if (req.debug) {
                if (out.debug == null) out.debug = new LinkedHashMap<>();
                out.debug.merge("regions", "[" + r.firstObservation() + ".." + r.lastObservation() + " fixedStart=" + fixedStart + " fixedEnd=" + fixedEnd + "]", (x, y) -> x + " " + y);
            }
            if (req.debug) lo.failTrace = new LinkedHashMap<>();
            // A sub-leg is accepted when it takes the matched roads OR has no deviation: waypoints
            // are only added where the route would otherwise deviate.
            // Created pieces must stay a margin inside the threshold, so the re-routed result is
            // not a borderline case that the next run judges the other way.
            lo.geometryCheck = (a, b, leg) -> within(leg, ref, obs,
                    a == ca && fixedStart ? obs.arcs.get(woA) : obs.arcs.get(mr.matchedObsIndices().get(a)),
                    b == cb && fixedEnd ? obs.arcs.get(woB) : obs.arcs.get(mr.matchedObsIndices().get(b)),
                    rule.tighter(FIX_MARGIN_M),
                    Deviation.Ends.of(rule, a == ca && fixedStart, b == cb && fixedEnd)
                            .withConnectors(a == ca && fixedStart ? exclA : 0, b == cb && fixedEnd ? exclB : 0));
            double arcStart = obs.arcs.get(woA);
            Double departure = fixedStart && arcStart + REVERSAL_PROBE_M < ref.lengthM()
                    ? RouteFixerMath.bearing(ref.pointAt(arcStart), ref.pointAt(arcStart + REVERSAL_PROBE_M)) : null;
            List<MatchedLegOptimizer.Piece> pieces = lo.optimize(cA, cB, fixedStart ? pa : null, fixedEnd ? pb : null,
                    fixedStart ? heading : null, departure);
            probes += lo.probes;
            if (lo.failTrace != null && !lo.failTrace.isEmpty()) out.debug.put("failed_steps", new ArrayList<>(lo.failTrace.values()));
            if (req.debug) {
                // The matched road path over this leg's candidates, [lat, lng] (review exports).
                int[] idx = mr.obsEdgeIdxInSlice();
                List<double[]> path = new ArrayList<>();
                for (int e = Math.min(idx[cA], idx[cB]); e <= Math.max(idx[cA], idx[cB]); e++) {
                    com.graphhopper.util.PointList g = mr.edgeMatches().get(e).getEdgeState().fetchWayGeometry(com.graphhopper.util.FetchMode.ALL);
                    for (int p = path.isEmpty() ? 0 : 1; p < g.size(); p++) path.add(new double[]{g.getLat(p), g.getLon(p)});
                }
                @SuppressWarnings("unchecked")
                List<List<double[]>> all = (List<List<double[]>>) out.debug.computeIfAbsent("matched_paths", x -> new ArrayList<List<double[]>>());
                all.add(path);
            }
            for (MatchedLegOptimizer.Piece p : pieces) {
                Node f = p.fromCand() == cA ? regionStart : candNode(mr, p.fromCand(), obs);
                Node t = (p.toCand() == cB && fixedEnd) ? end : candNode(mr, p.toCand(), obs);
                chunks.add(new Chunk(f, t, p.routed(), p.heading(), p.routed() ? p.route().distanceM() : 0, f.arc(), t.arc()));
            }
            cursor = fixedEnd ? end : candNode(mr, cB, obs);
            if (fixedEnd) break;
        }
        if (cursor != end) chunks.addAll(fillGap(new Gap(cursor, end, cursor == start ? heading : null), gc, probeCount));
        probes += probeCount[0];

        chunks = absorbTinySteps(mergeForcedRuns(chunks));
        if (req.debug) {
            if (out.debug == null) out.debug = new LinkedHashMap<>();
            List<String> tr = new ArrayList<>();
            for (Chunk c : chunks) tr.add(String.format(java.util.Locale.ROOT, "%s %.0f->%.0f%s", c.routed() ? "ok" : "BAD",
                    c.from().arc(), c.to().arc(), c.heading() == null ? "" : " h=" + Math.round(c.heading())));
            out.debug.put("steps", tr);
        }
        // Merge consecutive stretches, report / route them per policy.
        String policy = req.unroutablePolicy == null ? FixRouteRequest.POLICY_REROUTE : req.unroutablePolicy;
        List<FixRouteResponse.Segment> reroute = new ArrayList<>(), coords = new ArrayList<>();
        List<FixRouteResponse.Unroutable> stretches = new ArrayList<>();
        UnroutableCause causes = new UnroutableCause(hopper);
        int[] stretchProbes = {0};
        List<Node[]> rNodes = new ArrayList<>();       // per reroute segment: its two nodes
        List<Boolean> rStretch = new ArrayList<>();    // per reroute segment: routes round a stretch
        for (int k = 0; k < chunks.size(); k++) {
            Chunk c = chunks.get(k);
            if (c.routed()) {
                FixRouteResponse.Segment s = routed(c.from(), c.to(), c.heading(), c.distanceM());
                reroute.add(s);
                coords.add(s);
                rNodes.add(new Node[]{c.from(), c.to()});
                rStretch.add(false);
                continue;
            }
            Node a = c.from(), b = c.to();
            while (k + 1 < chunks.size() && !chunks.get(k + 1).routed()) b = chunks.get(++k).to();
            ClientRoute.Leg rr = ClientRoute.route(hopper, settings, a.pos(), List.of(), b.pos(), null, penalty);
            probes++;
            FixRouteResponse.Segment rs = routed(a, b, null, rr.ok() ? rr.distanceM() : 0);
            double len = b.arc() - a.arc();
            // Too short to report — but only when its detour is short too. A 20 m closed bridge
            // with a 5 km way around is exactly what must be reported (owner review, 49302).
            boolean shortDetour = rr.ok() && rr.distanceM() <= Math.max(2 * len, len + SHORT_DETOUR_SLACK_M);
            // Routing across it has no deviation from the saved route: the matcher failed here, the
            // roads did not (a winding trail the saved line cuts straight, a wrong-way carriageway).
            Deviation.Ends stretchEnds = Deviation.Ends.of(rule, a.fixedId() != null, b.fixedId() != null)
                    .withConnectors(a == start ? exclA : 0, b == end ? exclB : 0);
            // Unroutable means no route can be found along the saved route here — the section that
            // would have to be bridged with coordinates (owner, 2026-09-25). A section a route CAN
            // be forced along (waypoints on it) is only not preferred by the profile — stairs for a
            // gravel profile (92304:2): not reported, and not worth forcing; routed as the client does.
            if ((len < opt.minUnroutableM && shortDetour)
                    || within(rr, ref, obs, a.arc(), b.arc(), rule.tighter(FIX_MARGIN_M), stretchEnds)) {
                reroute.add(rs);
                coords.add(rs);
                rNodes.add(new Node[]{a, b});
                rStretch.add(false);
                continue;
            }
            boolean forcible = forcedAlong(a, b, ref, obs, settings, penalty, rule, stretchEnds, opt, stretchProbes);
            probes += stretchProbes[0];
            stretchProbes[0] = 0;
            if (forcible) {
                // Left to the router like a gap (pins beside it are cleaned up the same way), but
                // not reported: it is routable.
                reroute.add(rs);
                coords.add(rs);
                rNodes.add(new Node[]{a, b});
                rStretch.add(true);
                continue;
            }
            FixRouteResponse.Unroutable u = new FixRouteResponse.Unroutable();
            u.refFromM = RouteFixerMath.round1(a.arc());
            u.refToM = RouteFixerMath.round1(b.arc());
            u.referenceLengthM = RouteFixerMath.round1(len);
            u.rerouteLengthM = rr.ok() ? RouteFixerMath.round1(rr.distanceM()) : null;
            u.cause = causes.cause(ref, a.arc(), b.arc(), settings, rr.ok(), opt.offnetSnapM);
            stretches.add(u);
            reroute.add(rs);
            coords.add(coordinatesSegment(ref, a, b));
            rNodes.add(new Node[]{a, b});
            rStretch.add(true);
        }
        List<String> pruneLog = req.debug ? new ArrayList<>() : null;
        PruneContext pc = new PruneContext(settings, penalty, ref, obs, rule, start, end, exclA, exclB);
        // Clean up, then move the waypoints the route still turns back at (a U-turn often appears
        // only once its neighbours are gone), then clean up again around the moved ones.
        // Moving a U-turn waypoint first makes it cheaper, so the clean-up does not drop a waypoint
        // that is useful once well placed (17232:23: a sightseeing detour).
        if (RELOCATE_UTURNS && RELOCATE_FIRST) probes += relocateUturns(reroute, rNodes, pc, pruneLog);
        if (PRUNE_WAYPOINTS) probes += pruneWaypoints(reroute, rNodes, rStretch, pc, pruneLog);
        if (RELOCATE_UTURNS) {
            probes += relocateUturns(reroute, rNodes, pc, pruneLog);
            if (PRUNE_WAYPOINTS) probes += pruneWaypoints(reroute, rNodes, rStretch, pc, pruneLog);
        }
        if (pruneLog != null) {
            if (out.debug == null) out.debug = new LinkedHashMap<>();
            out.debug.put("prune", pruneLog);
        }
        int added = 0;
        for (FixRouteResponse.Segment s : reroute) if (s.end.newPoint != null) added++;
        // Waypoints that do not change the route are never kept (owner rule): the client would draw
        // the same line without them — e.g. next to a stretch that stays unroutable, or where the
        // deviation comes from the fixed waypoint itself (it forces a loop).
        if (added > 0 && q.ok()) {
            List<double[]> assembled = assemble(reroute, req, settings, penalty);
            probes += reroute.size();
            // (Pins into a dead-end before a stretch are removed one by one in pruneWaypoints; the
            // earlier whole-leg length test also threw away a leg's useful waypoints — 16153:17.)
            boolean same = assembled != null && PathSimilarity.compare(assembled, q.points(), SAME_ROUTE_M).frechetWithin();
            if (LEGACY_WHOLE_LEG_RULE && !same && !stretches.isEmpty() && assembled != null) {
                double savedLen = PathSimilarity.length(savedLeg);
                same = Math.abs(PathSimilarity.length(assembled) - savedLen) > Math.abs(q.distanceM() - savedLen) + 20.0;
            }
            if (same) {
                added = 0;
                reroute = new ArrayList<>(List.of(unchanged(seg)));
                if (stretches.isEmpty()) {
                    out.status = FixRouteResponse.OK;
                    out.metrics.acceptedBy = "no_effect";
                    out.metrics.probes = probes;
                    out.segments = reroute;
                    return out;
                }
            }
        }
        out.metrics.probes = probes;
        out.metrics.addedWaypoints = added;
        // The clean-up can leave one piece that still deviates where the search had used waypoints
        // instead of a heading (10108:9: a waypoint pinning a start loop, moved and then removed).
        // Then the start heading along the saved route is tried, as a re-run would.
        if (stretches.isEmpty() && added == 0 && reroute.size() == 1 && FixRouteRequest.TYPE_FOLLOW_ROADS.equals(reroute.get(0).type)) {
            ClientRoute.Leg fin = ClientRoute.route(hopper, settings, pa, via, pb, reroute.get(0).initialHeading, penalty);
            probes++;
            double arcStart = obs.arcs.get(woA);
            if (fin.ok() && !Deviation.within(fin.points(), savedLeg, rule, legEnds) && arcStart + REVERSAL_PROBE_M < ref.lengthM()) {
                double dep = RouteFixerMath.bearing(ref.pointAt(arcStart), ref.pointAt(arcStart + REVERSAL_PROBE_M));
                ClientRoute.Leg hr = ClientRoute.route(hopper, settings, pa, via, pb, dep, penalty);
                probes++;
                if (hr.ok() && DIST.calcDist(pa.lat, pa.lon, hr.points().get(0)[0], hr.points().get(0)[1])
                        <= DIST.calcDist(pa.lat, pa.lon, fin.points().get(0)[0], fin.points().get(0)[1]) + 1.0
                        && Deviation.within(hr.points(), savedLeg, rule.tighter(FIX_MARGIN_M), legEnds)) {
                    FixRouteResponse.Segment hs = unchanged(seg);
                    hs.initialHeading = RouteFixerMath.round1(dep);
                    hs.distanceM = RouteFixerMath.round1(hr.distanceM());
                    reroute = new ArrayList<>(List.of(hs));
                }
            }
        }
        // A heading repair is judged on the final result (after the clean-up), and a heading within
        // HEADING_SAME_DEG of the stored one is the same heading — else a re-run keeps "repairing" a
        // rounding difference (10108: 151.7° stored, 152° computed).
        boolean headingRepair = stretches.isEmpty() && reroute.size() == 1
                && FixRouteRequest.TYPE_FOLLOW_ROADS.equals(reroute.get(0).type)
                && headingDiffers(reroute.get(0).initialHeading, seg.initialHeading);
        if (stretches.isEmpty() && added == 0 && reroute.size() == 1 && !headingRepair) {
            // Nothing added, nothing to report (e.g. a zero-length leg, or a deviation below the
            // reporting minimum that no waypoint improves): the leg stays exactly as the client
            // has it, including its stored heading.
            out.status = FixRouteResponse.OK;
            out.metrics.acceptedBy = "tolerated";
            out.segments.add(unchanged(seg));
        } else if (stretches.isEmpty()) {
            out.status = FixRouteResponse.FIXED;
            out.metrics.acceptedBy = "edges";
            out.segments = reroute;
        } else {
            out.status = FixRouteResponse.UNROUTABLE;
            out.metrics.acceptedBy = "none";
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
        return out;
    }

    /**
     * Forced-waypoint rule (owner review 2026-09-24, routes 42833 / 45779): when the matched path can
     * only be reproduced by pinning a waypoint every grid step — at least {@link #FORCED_RUN_MIN}
     * routed pieces in a row each no longer than {@link #FORCED_STEP_M} (non-reproducible steps may
     * sit in between) — the profile will not take that path; forcing it is not a fix. The run becomes
     * one non-reproducible stretch, reported and routed around.
     */
    private static List<Chunk> mergeForcedRuns(List<Chunk> chunks) {
        List<Chunk> out = new ArrayList<>();
        int k = 0;
        while (k < chunks.size()) {
            int j = k, forced = 0;
            while (j < chunks.size() && (!chunks.get(j).routed()
                    || chunks.get(j).to().arc() - chunks.get(j).from().arc() <= FORCED_STEP_M)) {
                if (chunks.get(j).routed()) forced++;
                j++;
            }
            if (forced >= FORCED_RUN_MIN) {
                Chunk a = chunks.get(k), b = chunks.get(j - 1);
                out.add(new Chunk(a.from(), b.to(), false, null, 0, a.from().arc(), b.to().arc()));
                k = j;
            } else {
                out.add(chunks.get(k));
                k++;
            }
        }
        return out;
    }

    /**
     * A routed piece shorter than {@link #TINY_STEP_M} next to a non-reproducible stretch is folded
     * into the stretch: its only effect would be a new waypoint a few metres from an existing one
     * (the stretch's anchor), and a later run starting at that anchor would add it again.
     */
    private static List<Chunk> absorbTinySteps(List<Chunk> chunks) {
        List<Chunk> out = new ArrayList<>(chunks);
        for (int k = 0; k < out.size(); k++) {
            Chunk c = out.get(k);
            if (!c.routed() || c.to().arc() - c.from().arc() >= TINY_STEP_M) continue;
            if (k + 1 < out.size() && !out.get(k + 1).routed()) {
                Chunk n = out.get(k + 1);
                out.set(k + 1, new Chunk(c.from(), n.to(), false, null, 0, c.from().arc(), n.coreTo()));
                out.remove(k);
                k--;
            } else if (k > 0 && !out.get(k - 1).routed()) {
                Chunk p = out.get(k - 1);
                out.set(k - 1, new Chunk(p.from(), c.to(), false, null, 0, p.coreFrom(), c.to().arc()));
                out.remove(k);
                k--;
            }
        }
        return out;
    }

    private static Node candNode(TrackRegion.Matched r, int cand, RunObservations obs) {
        GHPoint s = r.obsSnapPoints().get(cand);
        return new Node(new GHPoint(RouteFixerMath.round6(s.lat), RouteFixerMath.round6(s.lon)),
                obs.arcs.get(r.matchedObsIndices().get(cand)), null);
    }

    // ------------------------------------------------------------------------------------------

    /** Distance [m] from the waypoint's own road snap to the saved path near its aligned arc. */
    private double snapOffset(GHPoint wp, ReferenceTrack ref, double arc, ClientRoute.Settings s) {
        Snap sn = hopper.getLocationIndex().findClosest(wp.lat, wp.lon, usableFilter(s));
        if (!sn.isValid()) return 0;
        double[] pr = ref.projectNear(new double[]{sn.getSnappedPoint().lat, sn.getSnappedPoint().lon}, arc, 100);
        return Double.isFinite(pr[1]) ? pr[1] : 0;
    }

    // ------------------------------------------------------------------------------------------
    // Waypoint clean-up (owner review 2026-09-25: 20630:15 got 7 waypoints where 3 do; 10122:50 kept
    // 4 anchors round an unroutable stretch that change nothing). The search places a waypoint at
    // every join of a matched part and a gap, and anchors each stretch; here each added waypoint is
    // tried away, one at a time: it goes when the client's route without it is the same route, or
    // (away from stretches) still has no deviation, or (next to a stretch) it only pins the route
    // into a dead-end before the gap. It only ever removes waypoints.
    // ------------------------------------------------------------------------------------------

    private record PruneContext(ClientRoute.Settings settings, double penalty, ReferenceTrack ref, RunObservations obs,
                                Deviation.Rule rule, Node start, Node end, double exclA, double exclB) {
    }

    /** Removes added waypoints that change nothing; returns the probes it used. */
    private int pruneWaypoints(List<FixRouteResponse.Segment> segs, List<Node[]> nodes, List<Boolean> stretch,
                               PruneContext c, List<String> log) {
        int probes = 0;
        // Where the gaps are, as found — merged pieces must not spread "next to a gap" along them.
        List<double[]> gaps = new ArrayList<>();
        for (int k = 0; k < nodes.size(); k++) if (stretch.get(k)) gaps.add(new double[]{nodes.get(k)[0].arc(), nodes.get(k)[1].arc()});
        int i = 1;
        while (i < segs.size()) {
            Node a = nodes.get(i - 1)[0], x = nodes.get(i - 1)[1], b = nodes.get(i)[1];
            if (x.fixedId() != null) {
                i++;
                continue;
            }
            Double h0 = segs.get(i - 1).initialHeading, h1 = segs.get(i).initialHeading;
            ClientRoute.Leg merged = ClientRoute.route(hopper, c.settings(), a.pos(), List.of(), b.pos(), h0, c.penalty());
            ClientRoute.Leg p0 = ClientRoute.route(hopper, c.settings(), a.pos(), List.of(), x.pos(), h0, c.penalty());
            ClientRoute.Leg p1 = ClientRoute.route(hopper, c.settings(), x.pos(), List.of(), b.pos(), h1, c.penalty());
            probes += 3;
            boolean drop = false;
            if (merged.ok() && p0.ok() && p1.ok()) {
                List<double[]> two = new ArrayList<>(p0.points());
                two.addAll(p1.points());
                drop = PathSimilarity.compare(merged.points(), two, SAME_ROUTE_M).frechetWithin();
                // Next to an unroutable stretch the route goes round anyway; a waypoint there stays
                // only if it brings the route closer to the saved route's length — a pin into a
                // dead-end before the gap makes it longer (owner review: 16153:17, 30291:6, the
                // closed bridge 49302:176), a waypoint fixing a deviation beside the gap does not.
                if (!drop && nearGap(x.arc(), gaps)) {
                    if (PIN_MODE.equals("gain")) {
                        // What the waypoint buys vs what it costs, in the terms quality is judged by:
                        // the saved route it makes the route follow (less saved route farther than the
                        // deviation threshold from the route) against the extra riding it adds. A pin
                        // into a dead-end before a gap follows L and costs ~2L; a waypoint fixing a
                        // deviation beside the gap follows far more than it costs (42235: 4.7 km for 212 m).
                        List<double[]> saved = savedSlice(c.ref(), c.obs(), a.arc(), b.arc());
                        double gain = Deviation.farLength(saved, merged.points(), c.rule().peakM())
                                - Deviation.farLength(saved, two, c.rule().peakM());
                        double cost = p0.distanceM() + p1.distanceM() - merged.distanceM();
                        drop = gain < cost;
                        if (log != null) log.add(String.format(java.util.Locale.ROOT, "  gain=%.0f cost=%.0f", gain, cost));
                    } else { // earlier variants, for before/after comparisons only
                        double savedM = b.arc() - a.arc();
                        drop = turnsBack(p0, p1) || (PIN_MODE.equals("savedLength")
                                ? Math.abs(merged.distanceM() - savedM) + PIN_SLACK_M < Math.abs(p0.distanceM() + p1.distanceM() - savedM)
                                : merged.distanceM() + PIN_SLACK_M < p0.distanceM() + p1.distanceM());
                    }
                }
                if (!drop && !stretch.get(i - 1) && !stretch.get(i)) {
                    drop = within(merged, c.ref(), c.obs(), a.arc(), b.arc(), c.rule().tighter(FIX_MARGIN_M),
                            Deviation.Ends.of(c.rule(), a.fixedId() != null, b.fixedId() != null)
                                    .withConnectors(a == c.start() ? c.exclA() : 0, b == c.end() ? c.exclB() : 0));
                }
            }
            if (log != null) log.add(String.format(java.util.Locale.ROOT, "wp@%.0f %s nextToStretch=%s turnsBack=%s merged=%.0f two=%.0f",
                    x.arc(), drop ? "DROP" : "keep", stretch.get(i - 1) || stretch.get(i),
                    merged.ok() && p0.ok() && p1.ok() && turnsBack(p0, p1), merged.ok() ? merged.distanceM() : -1,
                    p0.ok() && p1.ok() ? p0.distanceM() + p1.distanceM() : -1));
            if (drop) {
                segs.set(i - 1, routed(a, b, h0, merged.distanceM()));
                nodes.set(i - 1, new Node[]{a, b});
                stretch.set(i - 1, stretch.get(i - 1) || stretch.get(i));
                segs.remove(i);
                nodes.remove(i);
                stretch.remove(i);
            } else {
                i++;
            }
        }
        return probes;
    }

    // ------------------------------------------------------------------------------------------
    // Waypoint placement (owner review 2026-09-29: 32997 — 1.1 km into a dead-end before a private
    // road and back; 44932:14 — 30 m past a junction, U-turn, 30 m back; 42235:0). A waypoint the
    // route turns back at is almost always misplaced: it pins the route beyond the point where it
    // should turn. It is moved to where the retracing starts — just onto the road the route
    // continues on — unless the saved route itself goes out and back there.
    // ------------------------------------------------------------------------------------------

    /** How far [m] onto the continuing road a relocated waypoint is placed (clear of the junction). */
    static final double RELOCATE_PAST_M = 5.0;
    /** Comparison switch (before/after reviews): {@code -Dfix.relocate=false} keeps waypoints where found. */
    static boolean RELOCATE_UTURNS = !"false".equals(System.getProperty("fix.relocate"));    /** Comparison switch: also relocate before the first clean-up (default on). */
    static boolean RELOCATE_FIRST = !"false".equals(System.getProperty("fix.relocateFirst"));

    /** Moves added waypoints the route turns back at; returns the probes it used. */
    private int relocateUturns(List<FixRouteResponse.Segment> segs, List<Node[]> nodes, PruneContext c, List<String> log) {
        int probes = 0;
        for (int i = 1; i < segs.size(); i++) {
            Node a = nodes.get(i - 1)[0], x = nodes.get(i - 1)[1], b = nodes.get(i)[1];
            if (x.fixedId() != null) continue;
            Double h0 = segs.get(i - 1).initialHeading, h1 = segs.get(i).initialHeading;
            ClientRoute.Leg p0 = ClientRoute.route(hopper, c.settings(), a.pos(), List.of(), x.pos(), h0, c.penalty());
            ClientRoute.Leg p1 = ClientRoute.route(hopper, c.settings(), x.pos(), List.of(), b.pos(), h1, c.penalty());
            probes += 2;
            double r = retrace(p0, p1);
            if (r <= 0) continue;
            List<double[]> saved = savedSlice(c.ref(), c.obs(), a.arc(), b.arc());
            // The saved route itself goes out and back here (a viewpoint at a dead-end): keep it.
            List<double[]> tail = tailOf(p0.points(), r);
            double savedNear = PathSimilarity.length(saved) - Deviation.farLength(saved, tail, SAVED_RETRACE_NEAR_M);
            if (savedNear >= 1.5 * r) {
                if (log != null) log.add(String.format(java.util.Locale.ROOT, "uturn@%.0f %.0fm kept: saved route turns back too", x.arc(), r));
                continue;
            }
            double[] np = pointAlong(p1.points(), r + RELOCATE_PAST_M);
            if (np == null) continue;
            GHPoint pos = new GHPoint(RouteFixerMath.round6(np[0]), RouteFixerMath.round6(np[1]));
            double[] pr = c.ref().projectBetween(np, a.arc(), b.arc());
            Node x2 = new Node(pos, Double.isFinite(pr[1]) ? pr[0] : x.arc(), null);
            ClientRoute.Leg q0 = ClientRoute.route(hopper, c.settings(), a.pos(), List.of(), x2.pos(), h0, c.penalty());
            ClientRoute.Leg q1 = ClientRoute.route(hopper, c.settings(), x2.pos(), List.of(), b.pos(), h1, c.penalty());
            probes += 2;
            boolean ok = q0.ok() && q1.ok() && retrace(q0, q1) <= 0;
            String why = !q0.ok() || !q1.ok() ? "no route" : ok ? "" : "still turns back " + Math.round(retrace(q0, q1)) + "m";
            if (ok) {
                // It may give up at most the out-and-back it removes, nothing more of the saved route.
                List<double[]> before = new ArrayList<>(p0.points()), after = new ArrayList<>(q0.points());
                before.addAll(p1.points());
                after.addAll(q1.points());
                double peak = c.rule().peakM();
                double fb = Deviation.farLength(saved, before, peak), fa = Deviation.farLength(saved, after, peak);
                // The out-and-back it removes, plus the tolerance zone where it met the junction.
                ok = fa <= fb + r + peak;
                if (!ok) why = String.format(java.util.Locale.ROOT, "leaves more saved route: %.0f -> %.0f m", fb, fa);
            }
            if (log != null) log.add(String.format(java.util.Locale.ROOT, "uturn@%.0f %.0fm %s", x.arc(), r, ok ? "MOVED to " + pos : "kept (" + why + ")"));
            if (!ok) continue;
            segs.set(i - 1, routed(a, x2, h0, q0.distanceM()));
            segs.set(i, routed(x2, b, h1, q1.distanceM()));
            nodes.set(i - 1, new Node[]{a, x2});
            nodes.set(i, new Node[]{x2, b});
        }
        return probes;
    }

    /** A saved route within this [m] of a retraced piece covers it; covering it twice = it turns back too. */
    static final double SAVED_RETRACE_NEAR_M = 15.0;

    /** Length [m] the route rides back at the waypoint between in and out (same edges, reversed); 0 if none. */
    static double retrace(ClientRoute.Leg in, ClientRoute.Leg out) {
        if (!in.ok() || !out.ok() || in.edgeKeys() == null || out.edgeKeys() == null) return 0;
        int[] ka = com.graphhopper.trailmap.shared.EdgeKeyMatching.dedupConsecutive(in.edgeKeys());
        int[] kb = com.graphhopper.trailmap.shared.EdgeKeyMatching.dedupConsecutive(out.edgeKeys());
        double[] la = in.keyLengthsM(), lb = out.keyLengthsM();
        double m = 0;
        int i = ka.length - 1, j = 0;
        while (i >= 0 && j < kb.length && (ka[i] >> 1) == (kb[j] >> 1) && ka[i] != kb[j]) {
            m += Math.min(la != null && i < la.length ? la[i] : 0, lb != null && j < lb.length ? lb[j] : 0);
            i--;
            j++;
        }
        return m;
    }

    /** The last {@code m} metres of a line. */
    private static List<double[]> tailOf(List<double[]> line, double m) {
        List<double[]> out = new ArrayList<>();
        double d = 0;
        for (int k = line.size() - 1; k >= 0; k--) {
            out.add(0, line.get(k));
            if (k > 0) d += DIST.calcDist(line.get(k)[0], line.get(k)[1], line.get(k - 1)[0], line.get(k - 1)[1]);
            if (d >= m) {
                if (k > 0) out.add(0, line.get(k - 1));
                break;
            }
        }
        return out;
    }

    /** The point {@code m} metres along a line, or null beyond its end. */
    private static double[] pointAlong(List<double[]> line, double m) {
        double d = 0;
        for (int k = 1; k < line.size(); k++) {
            double[] p = line.get(k - 1), q = line.get(k);
            double s = DIST.calcDist(p[0], p[1], q[0], q[1]);
            if (d + s >= m && s > 0) {
                double t = (m - d) / s;
                return new double[]{p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])};
            }
            d += s;
        }
        return null;
    }

    /** Two start headings closer than this [deg] are the same heading. */
    static final double HEADING_SAME_DEG = 10.0;

    private static boolean headingDiffers(Double a, Double b) {
        if (a == null || b == null) return a != b;
        double d = Math.abs(((a - b) % 360 + 540) % 360 - 180);
        return d > HEADING_SAME_DEG;
    }

    /** Spacing [m] of the waypoints used to test whether a route can be forced along a section. */
    static final double FORCE_STEP_M = 10.0;

    /**
     * Can a route be forced along the saved route between a and b — with waypoints on it every
     * {@link #FORCE_STEP_M}, snapped to the nearest usable way — so that it follows the saved route
     * (the deviation rule)? Then the section is routable, only not preferred.
     */
    private boolean forcedAlong(Node a, Node b, ReferenceTrack ref, RunObservations obs, ClientRoute.Settings settings,
                                double penalty, Deviation.Rule rule, Deviation.Ends ends, FixRouteRequest.Options opt,
                                int[] probes) {
        if (!FORCED_CHECK || b.arc() - a.arc() < 1) return false;
        EdgeFilter usable = usableFilter(settings);
        List<GHPoint> via = new ArrayList<>();
        for (double arc = a.arc() + FORCE_STEP_M; arc < b.arc() - FORCE_STEP_M / 2; arc += FORCE_STEP_M) {
            double[] p = ref.pointAt(arc);
            Snap sn = hopper.getLocationIndex().findClosest(p[0], p[1], usable);
            if (!sn.isValid() || sn.getQueryDistance() > opt.offnetSnapM) return false; // off the usable network
            via.add(new GHPoint(sn.getSnappedPoint().lat, sn.getSnappedPoint().lon));
        }
        ClientRoute.Leg forced = ClientRoute.route(hopper, settings, a.pos(), via, b.pos(), null, penalty);
        probes[0]++;
        return within(forced, ref, obs, a.arc(), b.arc(), rule.tighter(FIX_MARGIN_M), ends);
    }

    /** Comparison switch (before/after reviews): {@code -Dfix.forced=false} reports forcible sections as before. */
    static boolean FORCED_CHECK = !"false".equals(System.getProperty("fix.forced"));

    /** A pin sits next to a gap: within this far [m] of an unroutable (or left-to-the-router) section. */
    static final double PIN_REACH_M = 150.0;

    /** The waypoint at {@code arc} is within {@link #PIN_REACH_M} of a section routed round (not followed). */
    private static boolean nearGap(double arc, List<double[]> gaps) {
        for (double[] g : gaps) {
            double d = arc < g[0] ? g[0] - arc : arc > g[1] ? arc - g[1] : 0;
            if (d <= PIN_REACH_M) return true;
        }
        return false;
    }

    /** The route arriving at a waypoint (p0) and leaving it (p1) turn back on the same road there. */
    private static boolean turnsBack(ClientRoute.Leg p0, ClientRoute.Leg p1) {
        int[] a = com.graphhopper.trailmap.shared.EdgeKeyMatching.dedupConsecutive(p0.edgeKeys());
        int[] b = com.graphhopper.trailmap.shared.EdgeKeyMatching.dedupConsecutive(p1.edgeKeys());
        if (a.length == 0 || b.length == 0) return false;
        return (a[a.length - 1] >> 1) == (b[0] >> 1) && a[a.length - 1] != b[0];
    }

    /** The route the client draws for these (followRoads) segments, concatenated; null on failure. */
    private List<double[]> assemble(List<FixRouteResponse.Segment> segs, FixRouteRequest req,
                                    ClientRoute.Settings settings, double penalty) {
        java.util.Map<String, GHPoint> byId = new java.util.HashMap<>();
        for (FixRouteRequest.Waypoint w : req.waypoints) byId.put(w.id, new GHPoint(w.coordinates.lat, w.coordinates.lng));
        List<double[]> out = new ArrayList<>();
        for (FixRouteResponse.Segment s : segs) {
            GHPoint a = s.start.newPoint != null ? new GHPoint(s.start.newPoint.lat, s.start.newPoint.lng) : byId.get(s.start.waypointId);
            GHPoint b = s.end.newPoint != null ? new GHPoint(s.end.newPoint.lat, s.end.newPoint.lng) : byId.get(s.end.waypointId);
            if (a == null || b == null) return null;
            ClientRoute.Leg l = ClientRoute.route(hopper, settings, a, List.of(), b, s.initialHeading, penalty);
            if (!l.ok()) return null;
            out.addAll(l.points());
        }
        return out;
    }

    // ------------------------------------------------------------------------------------------
    // Gaps: where the matcher gave up (an unmatched region), the saved line has no matched road to
    // take waypoint candidates from — e.g. a winding trail the saved line cuts straight across,
    // whose bends the segmenter reads as a detour (owner review, 45039:55). Candidates are then
    // the saved line's own points snapped to the nearest usable way, accepted by the same
    // deviation rule; what no candidate reaches stays a stretch.
    // ------------------------------------------------------------------------------------------

    /** Spacing [m] of gap candidates along the saved line (at most {@link #GAP_MAX_CANDIDATES}). */
    static final double GAP_STEP_M = 20.0;
    static final int GAP_MAX_CANDIDATES = 50;

    private record Gap(Node from, Node to, Double heading) {
    }

    private record GapContext(ClientRoute.Settings settings, double penalty, ReferenceTrack ref, RunObservations obs,
                              Deviation.Rule rule, Node start, Node end, double exclA, double exclB,
                              EdgeFilter usable, double maxSnapM) {
    }

    /** Comparison switch (before/after reviews only): the deployed build's whole-leg length rule. Default off. */
    static boolean LEGACY_WHOLE_LEG_RULE = Boolean.getBoolean("fix.legacyWholeLeg");

    /** Comparison switch (before/after reviews), on by default: waypoint clean-up. */
    static boolean PRUNE_WAYPOINTS = !"false".equals(System.getProperty("fix.prune"));

    /** Experiment switch (regression comparisons): {@code -Dfix.gaps=false} reports gaps as before. */
    static boolean GAP_CANDIDATES = !"false".equals(System.getProperty("fix.gaps"));

    private List<Chunk> fillGap(Gap g, GapContext c, int[] probes) {
        if (!GAP_CANDIDATES) return List.of(new Chunk(g.from(), g.to(), false, null, 0, g.from().arc(), g.to().arc()));
        double len = g.to().arc() - g.from().arc();
        List<Node> cand = new ArrayList<>();
        cand.add(g.from());
        if (len > GAP_STEP_M) {
            double step = Math.max(GAP_STEP_M, len / GAP_MAX_CANDIDATES);
            for (double arc = g.from().arc() + step; arc < g.to().arc() - step / 2; arc += step) {
                double[] p = c.ref().pointAt(arc);
                Snap sn = hopper.getLocationIndex().findClosest(p[0], p[1], c.usable());
                if (!sn.isValid() || sn.getQueryDistance() > c.maxSnapM()) continue;
                cand.add(new Node(new GHPoint(RouteFixerMath.round6(sn.getSnappedPoint().lat), RouteFixerMath.round6(sn.getSnappedPoint().lon)), arc, null));
            }
        }
        cand.add(g.to());
        List<Chunk> out = new ArrayList<>();
        int i = 0;
        Double heading = g.heading();
        while (i < cand.size() - 1) {
            Node a = cand.get(i);
            int best = -1;
            ClientRoute.Leg bestLeg = null;
            // Farthest candidate reachable without a deviation (few candidates: scan from the far end).
            for (int j = cand.size() - 1; j > i; j--) {
                Node b = cand.get(j);
                ClientRoute.Leg leg = ClientRoute.route(hopper, c.settings(), a.pos(), List.of(), b.pos(), heading, c.penalty());
                probes[0]++;
                boolean ok = within(leg, c.ref(), c.obs(), a.arc(), b.arc(), c.rule().tighter(FIX_MARGIN_M),
                        Deviation.Ends.of(c.rule(), a.fixedId() != null, b.fixedId() != null)
                                .withConnectors(a == c.start() ? c.exclA() : 0, b == c.end() ? c.exclB() : 0));
                if (ok) {
                    best = j;
                    bestLeg = leg;
                    break;
                }
            }
            if (best < 0) {
                out.add(new Chunk(a, cand.get(i + 1), false, null, 0, a.arc(), cand.get(i + 1).arc()));
                i++;
            } else {
                out.add(new Chunk(a, cand.get(best), true, heading, bestLeg.distanceM(), a.arc(), cand.get(best).arc()));
                i = best;
            }
            heading = null;
        }
        return out;
    }

    /** {@link Deviation#within} for a routed piece against the saved route between two arcs. */
    static boolean within(ClientRoute.Leg leg, ReferenceTrack ref, RunObservations obs, double fromArc, double toArc,
                          Deviation.Rule rule, Deviation.Ends ends) {
        if (!leg.ok() || leg.points() == null || leg.points().size() < 2 || toArc - fromArc < 1) return false;
        return Deviation.within(leg.points(), savedSlice(ref, obs, fromArc, toArc), rule, ends);
    }

    /** The leg's deviation rule: the request's options, each overridable per leg. */
    static Deviation.Rule rule(FixRouteRequest.Options o, FixRouteRequest.Segment s) {
        return new Deviation.Rule(
                s.fixMinDeviationM != null ? s.fixMinDeviationM : o.fixMinDeviationM,
                s.fixMinDeviationLengthM != null ? s.fixMinDeviationLengthM : o.fixMinDeviationLengthM,
                s.fixDetourMinM != null ? s.fixDetourMinM : o.fixDetourMinM,
                s.fixDetourMinRatio != null ? s.fixDetourMinRatio : o.fixDetourMinRatio,
                s.fixWaypointZoneM != null ? s.fixWaypointZoneM : o.fixWaypointZoneM,
                s.fixWaypointArmM != null ? s.fixWaypointArmM : o.fixWaypointArmM);
    }

    /**
     * The saved route between two arcs as the deviation rule sees it: without legacy spikes (the
     * old router's straight line out to an off-road waypoint and back) — no road path has them, and
     * the matcher does not see them either.
     */
    static List<double[]> savedSlice(ReferenceTrack ref, RunObservations obs, double fromArc, double toArc) {
        if (obs.spikeArcs.isEmpty() || obs.spikeArcs.subSet(fromArc, true, toArc, true).isEmpty()) return ref.slice(fromArc, toArc);
        List<double[]> out = new ArrayList<>();
        for (ReferenceTrack.ArcPoint p : ref.slicePointsWithGrid(fromArc, toArc, Double.MAX_VALUE)) {
            Double near = obs.spikeArcs.ceiling(p.arcM() - 1e-6);
            if (near != null && near <= p.arcM() + 1e-6) continue;
            out.add(p.point());
        }
        return out.size() >= 2 ? out : ref.slice(fromArc, toArc);
    }

    /** A fixed waypoint's matched position: region and candidate at (within 2 obs of) its observation. */
    private record EndMatch(TrackRegion.Matched mr, int cand) {
    }

    private static EndMatch endMatch(List<TrackRegion> regions, int wo, boolean start, RunObservations obs) {
        for (TrackRegion r : regions) {
            if (!(r instanceof TrackRegion.Matched mr) || mr.edgeMatches().isEmpty()) continue;
            List<Integer> cands = mr.matchedObsIndices();
            if (start) {
                for (int c = 0; c < cands.size(); c++) {
                    if (cands.get(c) < wo) continue;
                    if (cands.get(c) - wo <= 2 && c + 1 < cands.size()
                            && obs.arcs.get(cands.get(c)) - obs.arcs.get(wo) <= FIXED_END_REACH_M) return new EndMatch(mr, c);
                    break;
                }
            } else {
                for (int c = cands.size() - 1; c >= 0; c--) {
                    if (cands.get(c) > wo) continue;
                    if (wo - cands.get(c) <= 2 && c > 0
                            && obs.arcs.get(wo) - obs.arcs.get(cands.get(c)) <= FIXED_END_REACH_M) return new EndMatch(mr, c);
                    break;
                }
            }
        }
        return null;
    }

    /** Stub a fixed waypoint may force: out to its road snap and back, twice its offset, plus slack. */
    private static double stubAllowance(double offsetM) {
        return 2 * offsetM + STUB_SLACK_M;
    }

    /**
     * Length [m] of the route the client is forced to take between a fixed waypoint's own snap and
     * the matched road — 0 when the waypoint snaps onto the matched road itself (its edge at the
     * waypoint, or one of the neighbouring matched edges, or the same spot at a junction).
     */
    private double connector(GHPoint wp, TrackRegion.Matched mr, int cand, boolean start,
                             ClientRoute.Settings s, double penalty) {
        Snap sn = hopper.getLocationIndex().findClosest(wp.lat, wp.lon, usableFilter(s));
        if (!sn.isValid()) return 0;
        GHPoint m = mr.obsSnapPoints().get(cand);
        double apart = DIST.calcDist(sn.getSnappedPoint().lat, sn.getSnappedPoint().lon, m.lat, m.lon);
        // Right beside it (a footway along the road, the other carriageway) — or it is not a
        // mismatch of this waypoint at all but a placement problem elsewhere.
        if (apart < 1.0 || apart > MISMATCH_MAX_M) return 0;
        int edge = sn.getClosestEdge().getEdge();
        int[] idx = mr.obsEdgeIdxInSlice();
        for (int e = Math.max(0, idx[cand] - 2); e <= Math.min(mr.edgeMatches().size() - 1, idx[cand] + 2); e++) {
            if ((ReferenceMatcher.originalEdgeKey(mr.edgeMatches().get(e).getEdgeState()) >> 1) == edge) return 0;
        }
        GHPoint mp = new GHPoint(RouteFixerMath.round6(m.lat), RouteFixerMath.round6(m.lon));
        ClientRoute.Leg c = start ? ClientRoute.route(hopper, s, wp, List.of(), mp, null, penalty)
                : ClientRoute.route(hopper, s, mp, List.of(), wp, null, penalty);
        return c.ok() ? Math.max(c.distanceM(), 1.0) : 0;
    }

    /** The mismatch report: the matched road position at the waypoint as the suggestion. */
    private FixRouteResponse.SnapMismatch mismatch(String id, GHPoint wp, TrackRegion.Matched mr, int cand) {
        GHPoint m = mr.obsSnapPoints().get(cand);
        FixRouteResponse.SnapMismatch r = new FixRouteResponse.SnapMismatch();
        r.waypointId = id;
        r.suggested = new LatLng(RouteFixerMath.round6(m.lat), RouteFixerMath.round6(m.lon));
        r.offsetM = RouteFixerMath.round1(DIST.calcDist(wp.lat, wp.lon, m.lat, m.lon));
        return r;
    }

    private EdgeFilter usableFilter(ClientRoute.Settings s) {
        PMap hints = new PMap();
        if (s.customModel() != null) hints.putObject(CustomModel.KEY, s.customModel());
        Weighting w = hopper.createWeighting(hopper.getProfile(s.profile()), hints);
        EdgeFilter usable = new DefaultSnapFilter(w,
                hopper.getEncodingManager().getBooleanEncodedValue(Subnetwork.key(s.profile())));
        if (s.snapPreventions() == null || s.snapPreventions().isEmpty()) return usable;
        EnumEncodedValue<RoadClass> rc = hopper.getEncodingManager().getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        EnumEncodedValue<RoadEnvironment> re = hopper.getEncodingManager().getEnumEncodedValue(RoadEnvironment.KEY, RoadEnvironment.class);
        EdgeFilter prevent = new SnapPreventionEdgeFilter(EdgeFilter.ALL_EDGES, rc, re, s.snapPreventions());
        return e -> usable.accept(e) && prevent.accept(e);
    }

    private boolean offRoad(double[] p, EdgeFilter usable) {
        Snap near = hopper.getLocationIndex().findClosest(p[0], p[1], usable);
        return !near.isValid() || near.getQueryDistance() > 3.0;
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
        s.start = a.fixedId() != null ? Endpoint.fixed(a.fixedId()) : Endpoint.added(a.pos().lat, a.pos().lon);
        s.end = b.fixedId() != null ? Endpoint.fixed(b.fixedId()) : Endpoint.added(b.pos().lat, b.pos().lon);
        s.initialHeading = heading == null ? null : RouteFixerMath.round1(heading);
        s.distanceM = RouteFixerMath.round1(distanceM);
        return s;
    }

    private static FixRouteResponse.Segment coordinatesSegment(ReferenceTrack ref, Node a, Node b) {
        FixRouteResponse.Segment s = new FixRouteResponse.Segment();
        s.type = FixRouteRequest.TYPE_COORDINATES;
        s.start = a.fixedId() != null ? Endpoint.fixed(a.fixedId()) : Endpoint.added(a.pos().lat, a.pos().lon);
        s.end = b.fixedId() != null ? Endpoint.fixed(b.fixedId()) : Endpoint.added(b.pos().lat, b.pos().lon);
        List<LatLng> track = new ArrayList<>();
        track.add(new LatLng(a.pos().lat, a.pos().lon));
        List<double[]> slice = ref.slice(a.arc(), b.arc());
        for (int i = 1; i < slice.size() - 1; i++) track.add(new LatLng(slice.get(i)[0], slice.get(i)[1]));
        track.add(new LatLng(b.pos().lat, b.pos().lon));
        s.trackCoordinates = track;
        double d = 0;
        for (int i = 1; i < track.size(); i++)
            d += DIST.calcDist(track.get(i - 1).lat, track.get(i - 1).lng, track.get(i).lat, track.get(i).lng);
        s.distanceM = RouteFixerMath.round1(d);
        return s;
    }
}
