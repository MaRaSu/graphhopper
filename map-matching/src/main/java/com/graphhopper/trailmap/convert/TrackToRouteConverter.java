package com.graphhopper.trailmap.convert;

import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.jackson.ResponsePathSerializer;
import com.graphhopper.matching.EdgeMatch;
import com.graphhopper.matching.MatchResult;
import com.graphhopper.matching.Observation;
import com.graphhopper.matching.Tracepoint;
import com.graphhopper.util.CustomModel;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.FetchMode;
import com.graphhopper.util.PointList;
import com.graphhopper.util.shapes.GHPoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;

/**
 * Orchestrator for the track-to-route conversion pipeline.
 *
 * <p>Given a pre-built {@link MatchResult} (from {@link com.graphhopper.matching.MapMatching})
 * and the original observation list, runs stages 2–5 of the design:
 * <ol>
 *   <li>Region segmentation</li>
 *   <li>Routed-region waypoint minimization (per matched region)</li>
 *   <li>Coordinates-region simplification (per unmatched region)</li>
 *   <li>Combine + finalize: demote short routed, merge consecutive coordinates,
 *       generate UUIDs, build response</li>
 * </ol>
 */
public class TrackToRouteConverter {

    private static final Logger LOGGER = LoggerFactory.getLogger(TrackToRouteConverter.class);

    // Defaults — mirror the OSRM client reference (trailmap/shared/models/route-convert.ts).
    public static final double DEFAULT_SNAP_THRESHOLD_M = 35.0;
    /** Minimum matched-path length for a routed region; shorter regions get demoted to
     *  coordinates by the segmenter. OSRM client uses 40 m (MIN_ROUTED_SEGMENT_LENGTH). */
    public static final double DEFAULT_MIN_ROUTED_SEGMENT_M = 40.0;
    /** Per-Viterbi-transition minimum matched-leg length to be flagged as a detour.
     *  Transitions shorter than this never flag even at high ratios — guards against
     *  noise on tiny spans. OSRM client uses 75 m (MIN_DETOUR_DISTANCE).
     *  Per-request tunable via {@code min_detour_m}. */
    public static final double DEFAULT_MIN_DETOUR_M = 75.0;
    /** Per-Viterbi-transition matched/straight ratio threshold. The matcher's HMM
     *  transition into a tracepoint is flagged as a detour if its length is more than
     *  this multiple of the straight-line distance between observations (and also
     *  exceeds {@link #DEFAULT_MIN_DETOUR_M}). OSRM client uses 2.0 (MAX_DETOUR_FACTOR).
     *  Per-request tunable via {@code max_detour_ratio}. */
    public static final double DEFAULT_MAX_DETOUR_RATIO = 2.0;
    /** RDP epsilon in METERS. Client constant 0.00003 was degrees ≈ 3.3m at the equator;
     *  we convert to a meter value with similar geographic effect. */
    public static final double DEFAULT_COORDINATES_SIMPLIFY_EPS_M = 3.0;

    /** Default drift-trim tight-snap floor [m] when not derived from sigma. Mirrors the
     *  segmenter's own default. */
    public static final double DEFAULT_DRIFT_FLOOR_M = RegionSegmenter.DEFAULT_DRIFT_TRIM_TIGHT_FLOOR_M;

    /** Auto-sigma mode: snap-distance threshold = this × estimated sigma. ≈2.5 reproduces the
     *  stricter ("tarkka") client preset (25 m / 10 m sigma) and honours the conservative
     *  "better too-long coords than wrong-routed" bias. Used only when the request did not set
     *  an explicit snap_threshold_m. */
    public static final double AUTO_SIGMA_SNAP_THRESHOLD_MULT = 2.5;

    /** Auto-sigma mode: drift-trim floor = this × estimated sigma. ≈1.25 keeps the floor at the
     *  same fraction of sigma the legacy 5 m floor implied at the historical sigma=4 estimate. */
    public static final double AUTO_SIGMA_DRIFT_FLOOR_MULT = 1.25;

    private final RegionSegmenter segmenter = new RegionSegmenter();
    private final CoordinatesRegionBuilder coordsBuilder = new CoordinatesRegionBuilder();
    private final RoutedRegionOptimizer optimizer;
    private final GraphHopper graphHopper;

    public TrackToRouteConverter(GraphHopper graphHopper) {
        this.graphHopper = graphHopper;
        this.optimizer = new RoutedRegionOptimizer(graphHopper);
    }

    /** Enable the segmenter's v2 joint emission+transition classification (§8). Off by default;
     *  flips the {@link RegionSegmenter} to its parallel v2 path for this request only. */
    public void setSegmentationV2Enabled(boolean enabled) {
        segmenter.setSegmentationV2Enabled(enabled);
    }

    /** Restrict optimizer waypoint candidates to kept (Viterbi) observations (default on). */
    public void setOptimizerKeptObsOnly(boolean enabled) {
        segmenter.setOptimizerKeptObsOnly(enabled);
    }

    /** Toggle the optimizer's twin-edge tolerance (coincident cycleway/footway parallels;
     *  default on) — a last-resort fallback that rescues a leg from coords demotion when the
     *  only matcher-vs-/route difference is a coincident parallel edge over the same node pair. */
    public void setOptimizerTwinEdgeTolerance(boolean enabled) {
        optimizer.setTwinEdgeTolerance(enabled);
    }

    /**
     * Backwards-compatible overload using default detour thresholds. New callers should
     * prefer the full-arity {@code convert} so detour tunables can be set per request.
     */
    public ConvertTrackResponse convert(MatchResult matchResult,
                                        List<Observation> observations,
                                        String profile,
                                        CustomModel customModel,
                                        double snapThresholdM,
                                        double minRoutedSegmentM,
                                        double simplifyEpsM) {
        return convert(matchResult, observations, profile, customModel,
                snapThresholdM, minRoutedSegmentM, simplifyEpsM,
                DEFAULT_MIN_DETOUR_M, DEFAULT_MAX_DETOUR_RATIO);
    }

    /**
     * Backward-compatible overload that runs the conversion without populating debug data.
     */
    public ConvertTrackResponse convert(MatchResult matchResult,
                                        List<Observation> observations,
                                        String profile,
                                        CustomModel customModel,
                                        double snapThresholdM,
                                        double minRoutedSegmentM,
                                        double simplifyEpsM,
                                        double minDetourM,
                                        double maxDetourRatio) {
        return convert(matchResult, observations, profile, customModel,
                snapThresholdM, minRoutedSegmentM, simplifyEpsM,
                minDetourM, maxDetourRatio, DEFAULT_DRIFT_FLOOR_M, false);
    }

    /**
     * Run the conversion. Caller is responsible for invoking
     * {@link com.graphhopper.matching.MapMatching} and providing the result + observations.
     *
     * <p>When {@code includeDebug} is true, a {@link ConvertTrackResponse.Debug} block is
     * attached to the response with the matcher's polyline, per-observation tracepoints,
     * and detour decisions. See API doc §4.
     */
    public ConvertTrackResponse convert(MatchResult matchResult,
                                        List<Observation> observations,
                                        String profile,
                                        CustomModel customModel,
                                        double snapThresholdM,
                                        double minRoutedSegmentM,
                                        double simplifyEpsM,
                                        double minDetourM,
                                        double maxDetourRatio,
                                        double driftFloorM,
                                        boolean includeDebug) {
        long t0 = System.currentTimeMillis();

        // Stage 2: region segmentation. The segmenter now applies all three rendering-mode
        // criteria (snap quality, per-Viterbi-transition detour, minimum routed length)
        // so each region returned here is ready to be processed downstream as-is.
        // observations is passed through so the segmenter's obs→EdgeMatch mapping can
        // use the matcher's direction-aware State.getEntry() attribution.
        segmenter.setDriftTrimFloorM(driftFloorM);
        // Inject the on-network route-distance function used by the segmenter's phantom-detour
        // guard (distinguishes a directed-routing artifact at a corner — short real /route —
        // from a genuine obstacle detour — long /route). Bound to the rendering profile +
        // customModel so it asks the same question the client's /route will answer.
        segmenter.setDirectRouteFn((a, b) -> directRouteDistanceM(a, b, profile, customModel));
        List<TrackRegion> regions = segmenter.segment(matchResult, observations,
                snapThresholdM, minDetourM, maxDetourRatio, minRoutedSegmentM);

        long tSegmented = System.currentTimeMillis();

        // Stages 3 + 4: build per-region segments
        List<ProtoSegment> protos = new ArrayList<>();
        for (TrackRegion region : regions) {
            if (region instanceof TrackRegion.Matched matched) {
                RoutedRegionOptimizer.Result opt = optimizer.optimize(matched, observations, profile, customModel);
                expandOptimizerResult(opt, observations, simplifyEpsM, protos);
            } else if (region instanceof TrackRegion.Unmatched unmatched) {
                List<ConvertTrackResponse.Coordinates> coords = coordsBuilder.build(
                        observations, unmatched, simplifyEpsM);
                protos.add(ProtoSegment.coords(coords, geoLength(coords)));
            }
        }

        // Splice snap-point boundaries onto coords segments adjacent to routed segments,
        // so the route remains visually connected. Coords segments otherwise start/end at
        // raw GPS points which can be tens of meters off the snapped road position.
        spliceBoundariesOntoCoords(protos);

        // Stage 5: Merge consecutive coords segments
        List<ProtoSegment> merged = mergeConsecutiveCoords(protos);

        long tOptimized = System.currentTimeMillis();

        // Count matched vs unmatched observations using the same threshold the segmenter used
        int matchedCount = 0;
        if (matchResult.getTracepoints() != null) {
            for (var tp : matchResult.getTracepoints()) {
                if (tp.isMatched() && tp.getDistance() != null && tp.getDistance() < snapThresholdM) {
                    matchedCount++;
                }
            }
        }

        // Assign UUIDs to waypoints. A waypoint shared between two segments (end of N == start of N+1)
        // must produce a single Waypoint entry referenced by both.
        ConvertTrackResponse response = buildResponse(merged, observations.size(), matchedCount,
                tSegmented - t0, tOptimized - tSegmented);

        if (includeDebug) {
            response.setDebug(buildDebug(matchResult, observations,
                    segmenter.getLastDetours(), regions, snapThresholdM));
        }

        return response;
    }

    /** Builds the response's {@code debug} block. Aggregates the matcher's polyline,
     *  per-observation tracepoints, each detour the segmenter accepted, and the
     *  segmentation regions for client-side visual verification. */
    private ConvertTrackResponse.Debug buildDebug(
            MatchResult matchResult,
            List<Observation> observations,
            List<RegionSegmenter.DetourReport> detourReports,
            List<TrackRegion> regions,
            double snapThresholdM) {
        ConvertTrackResponse.Debug debug = new ConvertTrackResponse.Debug();

        // --- matcher.polyline + tracepoints ---
        ConvertTrackResponse.MatcherDebug md = new ConvertTrackResponse.MatcherDebug();
        List<EdgeMatch> edges = matchResult.getEdgeMatches();
        PointList combined = new PointList();
        for (EdgeMatch em : edges) {
            PointList g = em.getEdgeState().fetchWayGeometry(FetchMode.ALL);
            // Skip the first point of each subsequent edge to avoid duplicating shared
            // vertices between consecutive edges.
            int startIdx = combined.isEmpty() ? 0 : 1;
            for (int i = startIdx; i < g.size(); i++) {
                combined.add(g.getLat(i), g.getLon(i));
            }
        }
        md.polylineEncoded = ResponsePathSerializer.encodePolyline(combined, false, 1e5);
        md.polylineLengthM = matchResult.getMatchLength();
        md.edgeCount = edges.size();

        List<Tracepoint> tracepoints = matchResult.getTracepoints();
        if (tracepoints != null) {
            md.tracepoints = new ArrayList<>(tracepoints.size());
            for (int i = 0; i < tracepoints.size(); i++) {
                Tracepoint tp = tracepoints.get(i);
                ConvertTrackResponse.TracepointDebug t = new ConvertTrackResponse.TracepointDebug();
                t.index = i;
                if (tp.getOriginalPoint() != null) {
                    t.original = new ConvertTrackResponse.Coordinates(
                            tp.getOriginalPoint().lat, tp.getOriginalPoint().lon);
                }
                if (tp.getSnappedPoint() != null) {
                    t.snap = new ConvertTrackResponse.Coordinates(
                            tp.getSnappedPoint().lat, tp.getSnappedPoint().lon);
                }
                t.snapDistanceM = tp.getDistance();
                t.edgeId = tp.getEdgeId();
                t.matched = tp.isMatched();
                t.filtered = tp.isFiltered();
                md.tracepoints.add(t);
            }
        }
        debug.matcher = md;

        // --- detours ---
        if (detourReports != null && !detourReports.isEmpty()) {
            debug.detours = new ArrayList<>(detourReports.size());
            for (RegionSegmenter.DetourReport r : detourReports) {
                ConvertTrackResponse.DetourDebug d = new ConvertTrackResponse.DetourDebug();
                d.fromObs = r.fromObs();
                d.toObs = r.toObs();
                d.matchedLengthM = r.matchedLengthM();
                d.straightM = r.straightM();
                d.ratio = r.ratio();
                debug.detours.add(d);
            }
        }

        // --- segmentation regions ---
        if (regions != null && !regions.isEmpty()) {
            ConvertTrackResponse.SegmentationDebug seg = new ConvertTrackResponse.SegmentationDebug();
            seg.regions = new ArrayList<>(regions.size());
            for (TrackRegion region : regions) {
                ConvertTrackResponse.RegionDebug rd = new ConvertTrackResponse.RegionDebug();
                rd.firstObs = region.firstObservation();
                rd.lastObs = region.lastObservation();
                rd.type = (region instanceof TrackRegion.Matched) ? "matched" : "coordinates";
                seg.regions.add(rd);
            }
            debug.segmentation = seg;
        }

        return debug;
    }

    /**
     * Compose the final response: build waypoint list (deduplicating shared boundaries),
     * build segment list referencing waypoints by id.
     */
    private ConvertTrackResponse buildResponse(List<ProtoSegment> protos,
                                               int totalInputPoints,
                                               int matchedCount,
                                               long matchingMs,
                                               long optimizationMs) {
        List<ConvertTrackResponse.Waypoint> waypoints = new ArrayList<>();
        List<ConvertTrackResponse.Segment> segments = new ArrayList<>();

        // For routed segments we use their waypoint list directly.
        // For coordinates segments we synthesize start/end waypoints from the first/last point.
        // Shared waypoint at segment boundary (e.g. end of routed N == start of coords N+1)
        // is deduplicated by spatial proximity (< ~0.5m).
        String lastEndpointId = null;
        GHPoint lastEndpoint = null;

        for (int si = 0; si < protos.size(); si++) {
            ProtoSegment p = protos.get(si);

            if (p.type == ProtoType.ROUTED) {
                // Build waypoints in order; reuse lastEndpointId for the first if it matches
                List<String> ids = new ArrayList<>();
                for (int wi = 0; wi < p.waypoints.size(); wi++) {
                    GHPoint pt = p.waypoints.get(wi);
                    if (wi == 0 && lastEndpoint != null && samePoint(pt, lastEndpoint)) {
                        ids.add(lastEndpointId);
                    } else {
                        String id = UUID.randomUUID().toString();
                        waypoints.add(new ConvertTrackResponse.Waypoint(id, pt.lat, pt.lon));
                        ids.add(id);
                    }
                }
                // Emit one segment per consecutive waypoint pair with its leg distance and
                // (when the optimizer validated one) its start heading. A null heading is
                // omitted from the JSON — the client's "no heading constraint" signal.
                for (int wi = 0; wi < ids.size() - 1; wi++) {
                    double legDist = (p.legDistancesM != null && wi < p.legDistancesM.size())
                            ? p.legDistancesM.get(wi) : 0;
                    Double legHeading = (p.legHeadingsDeg != null && wi < p.legHeadingsDeg.size())
                            ? p.legHeadingsDeg.get(wi) : null;
                    segments.add(ConvertTrackResponse.Segment.routed(
                            ids.get(wi), ids.get(wi + 1), legDist, legHeading));
                }
                lastEndpoint = p.waypoints.get(p.waypoints.size() - 1);
                lastEndpointId = ids.get(ids.size() - 1);
            } else {
                // Coordinates segment: synthesize start/end from first/last coord
                if (p.coords.isEmpty()) continue;
                ConvertTrackResponse.Coordinates first = p.coords.get(0);
                ConvertTrackResponse.Coordinates last = p.coords.get(p.coords.size() - 1);
                String startId;
                GHPoint firstP = new GHPoint(first.getLat(), first.getLng());
                if (lastEndpoint != null && samePoint(firstP, lastEndpoint)) {
                    startId = lastEndpointId;
                } else {
                    startId = UUID.randomUUID().toString();
                    waypoints.add(new ConvertTrackResponse.Waypoint(startId, first.getLat(), first.getLng()));
                }
                String endId = UUID.randomUUID().toString();
                waypoints.add(new ConvertTrackResponse.Waypoint(endId, last.getLat(), last.getLng()));

                segments.add(ConvertTrackResponse.Segment.coordinates(startId, endId, p.coords, p.distanceM));
                lastEndpoint = new GHPoint(last.getLat(), last.getLng());
                lastEndpointId = endId;
            }
        }

        ConvertTrackResponse resp = new ConvertTrackResponse();
        resp.setWaypoints(waypoints);
        resp.setSegments(segments);

        ConvertTrackResponse.Stats stats = new ConvertTrackResponse.Stats();
        stats.inputPoints = totalInputPoints;
        stats.matchedPoints = matchedCount;
        stats.unmatchedPoints = totalInputPoints - matchedCount;
        stats.routedSegments = (int) segments.stream().filter(s -> ConvertTrackResponse.Segment.TYPE_FOLLOW_ROADS.equals(s.getType())).count();
        stats.coordinatesSegments = (int) segments.stream().filter(s -> ConvertTrackResponse.Segment.TYPE_COORDINATES.equals(s.getType())).count();
        stats.totalDistanceM = segments.stream().mapToDouble(ConvertTrackResponse.Segment::getDistanceM).sum();
        stats.matchingMs = matchingMs;
        stats.optimizationMs = optimizationMs;
        resp.setStats(stats);

        LOGGER.info("convert: {} waypoints, {} segments ({} routed, {} coords), total {}m",
                waypoints.size(), segments.size(), stats.routedSegments,
                stats.coordinatesSegments, String.format("%.1f", stats.totalDistanceM));
        return resp;
    }

    /**
     * Expand the optimizer's per-region {@link RoutedRegionOptimizer.Result} into one or
     * more {@link ProtoSegment}s. A run of consecutive routed legs becomes one routed
     * ProtoSegment; each coords-escalation leg becomes its own coords ProtoSegment built
     * from the raw GPX observations between the two waypoint obs indices.
     */
    private void expandOptimizerResult(RoutedRegionOptimizer.Result opt,
                                       List<Observation> observations,
                                       double simplifyEpsM,
                                       List<ProtoSegment> protos) {
        List<GHPoint> wps = opt.waypoints();
        List<Integer> wpObs = opt.waypointObsIndices();
        List<Double> legDists = opt.legDistancesM();
        List<Boolean> legCoords = opt.legIsCoords();
        List<Double> legHeadings = opt.legInitialHeadings();

        if (wps.size() < 2) {
            // Degenerate: optimizer returned a single waypoint. Nothing to emit.
            return;
        }

        int runStart = 0;
        List<Double> runLegs = new ArrayList<>();
        List<Double> runHeadings = new ArrayList<>();

        for (int i = 0; i < wps.size() - 1; i++) {
            if (legCoords.get(i)) {
                // Close the routed run accumulated so far (if any).
                if (i > runStart) {
                    List<GHPoint> sub = new ArrayList<>(wps.subList(runStart, i + 1));
                    protos.add(ProtoSegment.routed(sub, new ArrayList<>(runLegs), new ArrayList<>(runHeadings)));
                    runLegs.clear();
                    runHeadings.clear();
                }
                // Emit the coords leg from raw GPX observations.
                int fromObs = wpObs.get(i);
                int toObs = wpObs.get(i + 1);
                List<ConvertTrackResponse.Coordinates> coords = coordsBuilder.buildRange(
                        observations, fromObs, toObs, simplifyEpsM);
                protos.add(ProtoSegment.coords(coords, geoLength(coords)));
                runStart = i + 1;
            } else {
                runLegs.add(legDists.get(i));
                runHeadings.add(legHeadings != null && i < legHeadings.size() ? legHeadings.get(i) : null);
            }
        }
        // Trailing routed run.
        if (runStart < wps.size() - 1) {
            List<GHPoint> sub = new ArrayList<>(wps.subList(runStart, wps.size()));
            protos.add(ProtoSegment.routed(sub, new ArrayList<>(runLegs), new ArrayList<>(runHeadings)));
        }
    }

    /**
     * For each coords proto, prepend the previous routed proto's last waypoint (if any) and
     * append the next routed proto's first waypoint (if any), to ensure the route is
     * geometrically continuous across matched-vs-unmatched boundaries.
     *
     * <p>If the coords proto's existing endpoint is already very close to the snap point,
     * we replace it rather than adding a duplicate.
     */
    private void spliceBoundariesOntoCoords(List<ProtoSegment> protos) {
        for (int i = 0; i < protos.size(); i++) {
            ProtoSegment p = protos.get(i);
            if (p.type != ProtoType.COORDS) continue;

            // Prepend prev routed's last waypoint
            if (i > 0 && protos.get(i - 1).type == ProtoType.ROUTED) {
                List<GHPoint> prevWps = protos.get(i - 1).waypoints;
                if (!prevWps.isEmpty()) {
                    GHPoint snap = prevWps.get(prevWps.size() - 1);
                    ConvertTrackResponse.Coordinates first = p.coords.isEmpty()
                            ? null : p.coords.get(0);
                    ConvertTrackResponse.Coordinates snapCoord =
                            new ConvertTrackResponse.Coordinates(snap.lat, snap.lon);
                    if (first != null && sameCoord(first, snapCoord)) {
                        // Already aligned — replace to ensure exact match (avoids ID dedup miss)
                        p.coords.set(0, snapCoord);
                    } else {
                        p.coords.add(0, snapCoord);
                    }
                }
            }

            // Append next routed's first waypoint
            if (i + 1 < protos.size() && protos.get(i + 1).type == ProtoType.ROUTED) {
                List<GHPoint> nextWps = protos.get(i + 1).waypoints;
                if (!nextWps.isEmpty()) {
                    GHPoint snap = nextWps.get(0);
                    ConvertTrackResponse.Coordinates last = p.coords.isEmpty()
                            ? null : p.coords.get(p.coords.size() - 1);
                    ConvertTrackResponse.Coordinates snapCoord =
                            new ConvertTrackResponse.Coordinates(snap.lat, snap.lon);
                    if (last != null && sameCoord(last, snapCoord)) {
                        p.coords.set(p.coords.size() - 1, snapCoord);
                    } else {
                        p.coords.add(snapCoord);
                    }
                }
            }

            p.distanceM = geoLength(p.coords);
        }
    }

    private List<ProtoSegment> mergeConsecutiveCoords(List<ProtoSegment> in) {
        List<ProtoSegment> out = new ArrayList<>();
        for (ProtoSegment p : in) {
            if (!out.isEmpty() && out.get(out.size() - 1).type == ProtoType.COORDS && p.type == ProtoType.COORDS) {
                ProtoSegment last = out.get(out.size() - 1);
                // Concatenate, dedup boundary
                if (!last.coords.isEmpty() && !p.coords.isEmpty()
                        && sameCoord(last.coords.get(last.coords.size() - 1), p.coords.get(0))) {
                    last.coords.addAll(p.coords.subList(1, p.coords.size()));
                } else {
                    last.coords.addAll(p.coords);
                }
                last.distanceM = geoLength(last.coords);
            } else {
                out.add(p);
            }
        }
        return out;
    }

    /** On-network shortest-route distance [m] between two points under the given profile,
     *  or null if no route. Used by the segmenter's phantom-detour guard. Plain point-to-point
     *  (no headings) — it answers "does a short real road path exist between these snaps?". */
    private Double directRouteDistanceM(GHPoint a, GHPoint b, String profile, CustomModel customModel) {
        try {
            GHRequest req = new GHRequest(a, b);
            req.setProfile(profile);
            if (customModel != null) req.setCustomModel(customModel);
            req.putHint("instructions", false);
            req.putHint("calc_points", false);
            GHResponse rsp = graphHopper.route(req);
            if (rsp.hasErrors() || rsp.getBest() == null) return null;
            return rsp.getBest().getDistance();
        } catch (Exception e) {
            return null; // no route / engine error → guard stays conservative (keeps the detour)
        }
    }

    private static double geoLength(List<ConvertTrackResponse.Coordinates> coords) {
        if (coords.size() < 2) return 0;
        double total = 0;
        DistanceCalcEarth d = DistanceCalcEarth.DIST_EARTH;
        for (int i = 1; i < coords.size(); i++) {
            total += d.calcDist(
                    coords.get(i - 1).getLat(), coords.get(i - 1).getLng(),
                    coords.get(i).getLat(), coords.get(i).getLng());
        }
        return total;
    }

    private static boolean samePoint(GHPoint a, GHPoint b) {
        return Math.abs(a.lat - b.lat) < 5e-6 && Math.abs(a.lon - b.lon) < 5e-6;
    }

    private static boolean sameCoord(ConvertTrackResponse.Coordinates a, ConvertTrackResponse.Coordinates b) {
        return Math.abs(a.getLat() - b.getLat()) < 5e-6 && Math.abs(a.getLng() - b.getLng()) < 5e-6;
    }

    // -- Internal scratch representation between stages --

    private enum ProtoType { ROUTED, COORDS }

    private static class ProtoSegment {
        ProtoType type;
        List<GHPoint> waypoints;
        List<Double> legDistancesM;
        /** Per-leg start heading (deg) validated by the optimizer; null where none. Parallel
         *  to legDistancesM. Surfaced as Segment.initialHeading. */
        List<Double> legHeadingsDeg;
        List<ConvertTrackResponse.Coordinates> coords;
        double distanceM;

        static ProtoSegment routed(List<GHPoint> wps, List<Double> legs, List<Double> headings) {
            ProtoSegment p = new ProtoSegment();
            p.type = ProtoType.ROUTED;
            p.waypoints = new ArrayList<>(wps);
            p.legDistancesM = new ArrayList<>(legs);
            p.legHeadingsDeg = new ArrayList<>(headings);
            p.distanceM = legs.stream().mapToDouble(Double::doubleValue).sum();
            return p;
        }
        static ProtoSegment coords(List<ConvertTrackResponse.Coordinates> cs, double dist) {
            ProtoSegment p = new ProtoSegment();
            p.type = ProtoType.COORDS;
            p.coords = new ArrayList<>(cs);
            p.distanceM = dist;
            return p;
        }
    }
}
