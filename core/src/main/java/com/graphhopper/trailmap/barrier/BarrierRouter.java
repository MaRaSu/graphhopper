package com.graphhopper.trailmap.barrier;

import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.ResponsePath;
import com.graphhopper.config.Profile;
import com.graphhopper.routing.*;
import com.graphhopper.routing.ev.BooleanEncodedValue;
import com.graphhopper.routing.ev.EncodedValueLookup;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.lm.LandmarkStorage;
import com.graphhopper.routing.querygraph.QueryGraph;
import com.graphhopper.routing.util.DefaultSnapFilter;
import com.graphhopper.routing.util.DirectedEdgeFilter;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.storage.RoutingCHGraph;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.util.*;
import com.graphhopper.util.details.PathDetailsBuilderFactory;
import com.graphhopper.util.exceptions.ConnectionNotFoundException;
import com.graphhopper.util.shapes.GHPoint;

import java.util.*;

import static com.graphhopper.util.DistanceCalcEarth.DIST_EARTH;
import static com.graphhopper.util.Parameters.Algorithms.ALT_ROUTE;
import static com.graphhopper.util.Parameters.Algorithms.ROUND_TRIP;

/**
 * /route with {@code "barriers": "v1"} (docs/gh_barrier_gate_implementation.md §1.2–1.8). Without that hint every call
 * goes to the stock {@link Router} unchanged. With it:
 * <ol>
 * <li>route points snap on {@code <profile>_subnetwork_b}, so a point behind a barrier stays where it is;</li>
 * <li>a heading snap that would land on the other side of a closed barrier than the plain snap is dropped;</li>
 * <li>when no connection exists, the request is searched again with ASK barriers open at
 * {@link BarrierPassWeighting#CROSSING_WEIGHT}; the path is returned with its crossings (W1 / W2, 12.5 m either side)
 * in {@code hints.barriers}. If that fails too and a route exists through a REFUSE barrier, the answer is a
 * {@link BarrierRefusedException} naming the barrier;</li>
 * <li>every path through a NOTICE barrier (opening hours) lists it in {@code hints.barriers.paths[i].notices}.</li>
 * </ol>
 * An ASK barrier with an allowed way round is never crossed: the first search succeeds and goes round.
 */
public class BarrierRouter extends Router {

    public static final String HINT = "barriers";
    public static final String VERSION = "v1";
    /** internal: which barrier classes the solver opens (none | ask | ask_refuse) */
    static final String PASS_HINT = "barriers.internal_pass";
    static final double HALF_CROSSING_M = 12.5, GROUP_M = 25, JUNCTION_GAP_M = 2, NOTICE_M = 1.0;
    static final double HEADING_GUARD_M = 50;

    private final BarrierTable table;
    private final Map<Long, List<BarrierTable.Row>> noticeCells;

    public BarrierRouter(BaseGraph graph, EncodingManager encodingManager, LocationIndex locationIndex,
                         Map<String, Profile> profilesByName, PathDetailsBuilderFactory pathDetailsBuilderFactory,
                         TranslationMap translationMap, RouterConfig routerConfig, WeightingFactory weightingFactory,
                         Map<String, RoutingCHGraph> chGraphs, Map<String, LandmarkStorage> landmarks, BarrierTable table) {
        super(graph, encodingManager, locationIndex, profilesByName, pathDetailsBuilderFactory, translationMap,
                routerConfig, weightingFactory, chGraphs, landmarks);
        this.table = table;
        this.noticeCells = table.noticeCells(); // built once per table, shared by every router
    }

    public static boolean isOn(GHRequest request) {
        return VERSION.equals(request.getHints().getString(HINT, ""));
    }

    // ---------------------------------------------------------------------------------------------- entry point

    @Override
    public GHResponse route(GHRequest request) {
        if (!isOn(request))
            return super.route(request);
        request.getHints().putObject(PASS_HINT, "none");
        GHResponse first;
        try {
            first = super.route(request);
        } finally {
            request.getHints().remove(PASS_HINT);
        }
        if (!isConnectionNotFound(first) || ROUND_TRIP.equalsIgnoreCase(request.getAlgorithm())) {
            addNotices(request, first);
            return first;
        }
        GHResponse crossed = routeWithPass(request, "ask");
        if (!crossed.hasErrors()) {
            addNotices(request, crossed);
            return crossed;
        }
        GHResponse refused = routeWithPass(request, "ask_refuse");
        if (!refused.hasErrors()) {
            @SuppressWarnings("unchecked")
            Map<String, Object> refusal = (Map<String, Object>) refused.getHints().getObject("barriers.refusal", null);
            if (refusal != null) {
                GHResponse rsp = new GHResponse();
                rsp.addError(new BarrierRefusedException((String) refusal.remove("message"), refusal));
                return rsp;
            }
        }
        addNotices(request, first);
        return first;
    }

    private GHResponse routeWithPass(GHRequest request, String pass) {
        String algorithm = request.getAlgorithm();
        request.getHints().putObject(PASS_HINT, pass);
        // a crossing goes on the main path only: alternatives are not searched through barriers
        if (ALT_ROUTE.equalsIgnoreCase(algorithm))
            request.setAlgorithm("");
        try {
            return super.route(request);
        } finally {
            request.getHints().remove(PASS_HINT);
            request.setAlgorithm(algorithm);
        }
    }

    private static boolean isConnectionNotFound(GHResponse rsp) {
        if (!rsp.hasErrors())
            return false;
        for (Throwable t : rsp.getErrors())
            if (!(t instanceof ConnectionNotFoundException))
                return false;
        return true;
    }

    // ---------------------------------------------------------------------------------------------- solver

    @Override
    protected Solver createFlexSolver(GHRequest request, Map<String, Profile> profilesByName, RouterConfig routerConfig,
                                      EncodingManager encodingManager, WeightingFactory weightingFactory, BaseGraph baseGraph,
                                      LocationIndex locationIndex) {
        if (!isOn(request))
            return super.createFlexSolver(request, profilesByName, routerConfig, encodingManager, weightingFactory, baseGraph, locationIndex);
        return new BarrierFlexSolver(request, profilesByName, routerConfig, encodingManager, weightingFactory, baseGraph,
                locationIndex, request.getHints().getString(PASS_HINT, "none"));
    }

    /** Which barrier classes this request's solver opens (none | ask | ask_refuse); for subclasses' own solvers. */
    protected static String passOf(GHRequest request) {
        return request.getHints().getString(PASS_HINT, "none");
    }

    /** The flagged solver. Public so a router subclass can extend it (its routeVia needs this type for the crossings). */
    public static class BarrierFlexSolver extends FlexSolver {
        private final String pass;
        Weighting base;
        EnumEncodedValue<BarrierClass> barrierEnc;

        protected BarrierFlexSolver(GHRequest request, Map<String, Profile> profilesByName, RouterConfig routerConfig,
                          EncodedValueLookup lookup, WeightingFactory weightingFactory, BaseGraph graph,
                          LocationIndex locationIndex, String pass) {
            super(request, profilesByName, routerConfig, lookup, weightingFactory, graph, locationIndex);
            this.pass = pass;
        }

        @Override
        protected Weighting createWeighting() {
            base = super.createWeighting();
            barrierEnc = BarrierProfiles.barrierEnc(lookup, profile);
            if (barrierEnc == null || "none".equals(pass))
                return base;
            EnumSet<BarrierClass> open = "ask".equals(pass) ? EnumSet.of(BarrierClass.ASK)
                    : EnumSet.of(BarrierClass.ASK, BarrierClass.REFUSE);
            return new BarrierPassWeighting(base, barrierEnc, open, BarrierPassWeighting.CROSSING_WEIGHT);
        }

        @Override
        protected EdgeFilter createSnapFilter() {
            return new DefaultSnapFilter(base, BarrierProfiles.subnetworkEnc(lookup, profile.getName()));
        }

        @Override
        protected DirectedEdgeFilter createDirectedEdgeFilter() {
            BooleanEncodedValue sub = BarrierProfiles.subnetworkEnc(lookup, profile.getName());
            return (edge, reverse) -> !edge.get(sub) && Double.isFinite(base.calcEdgeWeight(edge, reverse));
        }

        EdgeFilter snapFilter() {
            return createSnapFilter();
        }

        DirectedEdgeFilter directedFilter() {
            return createDirectedEdgeFilter();
        }

        PathCalculator pathCalculator(QueryGraph queryGraph) {
            return createPathCalculator(queryGraph);
        }

        Weighting weighting() {
            return weighting;
        }

        String modeName() {
            return barrierEnc != null && BarrierClass.KEY_FOOT.equals(barrierEnc.getName()) ? "foot" : "bike";
        }

        Set<BarrierClass> crossable() {
            if ("ask".equals(pass))
                return EnumSet.of(BarrierClass.ASK);
            if ("ask_refuse".equals(pass))
                return EnumSet.of(BarrierClass.ASK, BarrierClass.REFUSE);
            return EnumSet.noneOf(BarrierClass.class);
        }
    }

    // ---------------------------------------------------------------------------------------------- via routing

    @Override
    protected GHResponse routeAlt(GHRequest request, Solver solver) {
        // flag only: the same heading-snap guard as routeVia (§1.5), then alternatives as today with the guarded headings
        if (!(solver instanceof BarrierFlexSolver) || request.getHeadings().isEmpty())
            return super.routeAlt(request, solver);
        BarrierFlexSolver bs = (BarrierFlexSolver) solver;
        List<Double> original = request.getHeadings();
        List<Double> headings = new ArrayList<>(original);
        EdgeFilter snapFilter = bs.snapFilter();
        DirectedEdgeFilter directedEdgeFilter = bs.directedFilter();
        List<Snap> snaps = ViaRouting.lookup(encodingManager, request.getPoints(), snapFilter, locationIndex,
                request.getSnapPreventions(), request.getPointHints(), directedEdgeFilter, headings);
        guardHeadingSnaps(request, snapFilter, directedEdgeFilter, snaps, headings, bs.base, bs.barrierEnc);
        if (headings.equals(original))
            return super.routeAlt(request, solver);
        request.setHeadings(headings);
        try {
            return super.routeAlt(request, solver);
        } finally {
            request.setHeadings(original);
        }
    }

    @Override
    protected GHResponse routeVia(GHRequest request, Solver solver) {
        if (!(solver instanceof BarrierFlexSolver))
            return super.routeVia(request, solver);
        BarrierFlexSolver bs = (BarrierFlexSolver) solver;
        GHResponse ghRsp = new GHResponse();
        StopWatch sw = new StopWatch().start();
        DirectedEdgeFilter directedEdgeFilter = bs.directedFilter();
        EdgeFilter snapFilter = bs.snapFilter();
        List<Double> headings = new ArrayList<>(request.getHeadings());
        List<Snap> snaps = ViaRouting.lookup(encodingManager, request.getPoints(), snapFilter, locationIndex,
                request.getSnapPreventions(), request.getPointHints(), directedEdgeFilter, headings);
        guardHeadingSnaps(request, snapFilter, directedEdgeFilter, snaps, headings, bs.base, bs.barrierEnc);
        ghRsp.addDebugInfo("idLookup:" + sw.stop().getSeconds() + "s");
        QueryGraph queryGraph = QueryGraph.create(graph, snaps);
        PathCalculator pathCalculator = bs.pathCalculator(queryGraph);
        boolean passThrough = request.getHints().getBool(Parameters.Routing.PASS_THROUGH, false);
        String curbsideStrictness = request.getHints().has(Parameters.Routing.CURBSIDE_STRICTNESS)
                ? request.getHints().getString(Parameters.Routing.CURBSIDE_STRICTNESS, "strict")
                : (request.getHints().getBool("force_curbside", true) ? "strict" : "soft");
        ViaRouting.Result result = ViaRouting.calcPaths(request.getPoints(), queryGraph, snaps, directedEdgeFilter,
                pathCalculator, request.getCurbsides(), curbsideStrictness, headings, passThrough);
        if (request.getPoints().size() != result.paths.size() + 1)
            throw new RuntimeException("There should be exactly one more point than paths. points:" + request.getPoints().size() + ", paths:" + result.paths.size());

        PointList waypoints = new PointList(snaps.size(), graph.getNodeAccess().is3D());
        for (Snap snap : snaps)
            waypoints.add(snap.getSnappedPoint());
        ResponsePath responsePath = createPathMerger(request, bs.weighting(), queryGraph, headings)
                .doWork(waypoints, result.paths, encodingManager, translationMap.getWithFallBack(request.getLocale()));
        responsePath.addDebugInfo(result.debug);
        ghRsp.add(responsePath);
        ghRsp.getHints().putObject("visited_nodes.sum", result.visitedNodes);
        ghRsp.getHints().putObject("visited_nodes.average", (float) result.visitedNodes / (snaps.size() - 1));

        Set<BarrierClass> crossable = bs.crossable();
        if (!crossable.isEmpty() && bs.barrierEnc != null && !responsePath.hasErrors()) {
            String mode = bs.modeName();
            List<Map<String, Object>> crossings = new ArrayList<>();
            Map<String, Object> refusal = null;
            for (int leg = 0; leg < result.paths.size(); leg++) {
                for (Map<String, Object> c : crossingsOfLeg(result.paths.get(leg), queryGraph, bs.barrierEnc, crossable, mode, leg)) {
                    if ("refuse".equals(c.get("class"))) {
                        if (refusal == null)
                            refusal = refusalOf(c, leg);
                    } else {
                        crossings.add(c);
                    }
                }
            }
            addPointIndexes(crossings, result.paths, responsePath.getPoints());
            if (refusal != null) {
                ghRsp.getHints().putObject("barriers.refusal", refusal);
            } else {
                ghRsp.getHints().putObject("barriers.crossings", crossings);
            }
        }
        return ghRsp;
    }

    private static final String W1_M = "_w1_m", W2_M = "_w2_m";

    /**
     * {@code w1_index} / {@code w2_index}: the index of the last point of the response's {@code points} at or before
     * W1 (W2), so a client can cut the path at W1 and W2 without searching. Measured along the response polyline;
     * the legs' metres are offset by the lengths of the legs before.
     */
    private static void addPointIndexes(List<Map<String, Object>> crossings, List<Path> legs, PointList pts) {
        double[] cum = new double[pts.size()];
        for (int i = 1; i < pts.size(); i++)
            cum[i] = cum[i - 1] + DIST_EARTH.calcDist(pts.getLat(i - 1), pts.getLon(i - 1), pts.getLat(i), pts.getLon(i));
        for (Map<String, Object> c : crossings) {
            int leg = (Integer) c.get("leg");
            double offset = 0;
            for (int k = 0; k < leg; k++)
                offset += legs.get(k).getDistance();
            c.put("w1_index", indexAtOrBefore(cum, offset + (Double) c.remove(W1_M)));
            c.put("w2_index", indexAtOrBefore(cum, offset + (Double) c.remove(W2_M)));
        }
    }

    private static int indexAtOrBefore(double[] cum, double s) {
        int i = 0;
        while (i + 1 < cum.length && cum[i + 1] <= s + 1e-6)
            i++;
        return i;
    }

    private PathMerger createPathMerger(GHRequest request, Weighting weighting, QueryGraph queryGraph, List<Double> headings) {
        boolean enableInstructions = request.getHints().getBool(Parameters.Routing.INSTRUCTIONS, routerConfig.isInstructionsEnabled());
        boolean calcPoints = request.getHints().getBool(Parameters.Routing.CALC_POINTS, routerConfig.isCalcPoints());
        double wayPointMaxDistance = request.getHints().getDouble(Parameters.Routing.WAY_POINT_MAX_DISTANCE, 0.5);
        double elevationWayPointMaxDistance = request.getHints().getDouble(Parameters.Routing.ELEVATION_WAY_POINT_MAX_DISTANCE, routerConfig.getElevationWayPointMaxDistance());
        RamerDouglasPeucker peucker = new RamerDouglasPeucker().
                setMaxDistance(wayPointMaxDistance).
                setElevationMaxDistance(elevationWayPointMaxDistance);
        PathMerger pathMerger = new PathMerger(queryGraph, weighting).
                setCalcPoints(calcPoints).
                setRamerDouglasPeucker(peucker).
                setEnableInstructions(enableInstructions).
                setPathDetailsBuilders(pathDetailsBuilderFactory, request.getPathDetails()).
                setSimplifyResponse(routerConfig.isSimplifyResponse() && wayPointMaxDistance > 0);
        if (!headings.isEmpty())
            pathMerger.setFavoredHeading(headings.get(0));
        return pathMerger;
    }

    /**
     * GraphHopper's heading snap (edges within 20 m whose bearing matches) can put a point on the far side of a closed
     * barrier, e.g. W2 12.5 m behind a gate onto the road in front. For each point with a heading: if the heading
     * snap and the plain snap differ, the plain snap's edge can't be reached from the heading snap's edge within
     * {@value #HEADING_GUARD_M} m over open edges and that search met a closed barrier edge, the plain snap is used
     * and that point's heading is dropped.
     */
    private void guardHeadingSnaps(GHRequest request, EdgeFilter snapFilter, DirectedEdgeFilter directedEdgeFilter,
                                   List<Snap> snaps, List<Double> headings, Weighting base, EnumEncodedValue<BarrierClass> enc) {
        if (headings.isEmpty())
            return;
        boolean any = false;
        for (Double h : headings)
            any |= h != null && !Double.isNaN(h);
        if (!any)
            return;
        List<Snap> plain = ViaRouting.lookup(encodingManager, request.getPoints(), snapFilter, locationIndex,
                request.getSnapPreventions(), request.getPointHints(), directedEdgeFilter, Collections.emptyList());
        boolean single = headings.size() == 1;
        for (int i = 0; i < snaps.size(); i++) {
            Double h = single ? (i == 0 ? headings.get(0) : null) : headings.get(i);
            if (h == null || Double.isNaN(h))
                continue;
            Snap a = snaps.get(i), b = plain.get(i);
            if (a.getClosestEdge().getEdge() == b.getClosestEdge().getEdge())
                continue;
            if (enc != null && separatedByBarrier(a.getClosestEdge(), b.getClosestEdge(), base, enc, HEADING_GUARD_M)) {
                snaps.set(i, b);
                if (single)
                    headings.set(0, Double.NaN);
                else
                    headings.set(i, Double.NaN);
            }
        }
    }

    /**
     * True when the two edges are not connected within maxM over open edges AND that search met a closed ASK/REFUSE
     * barrier edge, i.e. the heading snap is on the other side of a barrier. Two edges that are merely far apart (a
     * bridge over a road, a parallel road with a distant junction) are not separated by a barrier.
     */
    private boolean separatedByBarrier(EdgeIteratorState from, EdgeIteratorState to, Weighting base,
                                       EnumEncodedValue<BarrierClass> enc, double maxM) {
        Set<Integer> target = new HashSet<>(Arrays.asList(to.getBaseNode(), to.getAdjNode()));
        Map<Integer, Double> dist = new HashMap<>();
        PriorityQueue<double[]> q = new PriorityQueue<>(Comparator.comparingDouble(x -> x[0]));
        for (int n : new int[]{from.getBaseNode(), from.getAdjNode()}) {
            dist.put(n, 0.0);
            q.add(new double[]{0, n});
        }
        boolean metBarrier = false;
        EdgeExplorer explorer = graph.createEdgeExplorer();
        while (!q.isEmpty()) {
            double[] cur = q.poll();
            int n = (int) cur[1];
            if (target.contains(n))
                return false;
            if (cur[0] > maxM || cur[0] > dist.getOrDefault(n, Double.MAX_VALUE))
                continue;
            EdgeIterator it = explorer.setBaseNode(n);
            while (it.next()) {
                if (!Double.isFinite(base.calcEdgeWeight(it, false)) && !Double.isFinite(base.calcEdgeWeight(it, true))) {
                    metBarrier |= it.get(enc).closes();
                    continue;
                }
                double nd = cur[0] + it.getDistance();
                if (nd <= maxM && nd < dist.getOrDefault(it.getAdjNode(), Double.MAX_VALUE)) {
                    dist.put(it.getAdjNode(), nd);
                    q.add(new double[]{nd, it.getAdjNode()});
                }
            }
        }
        return metBarrier;
    }

    // ---------------------------------------------------------------------------------------------- crossings

    /** One leg's geometry with cumulative metres, the barrier edges on it and the junction positions. */
    private static final class Leg {
        final List<double[]> pts = new ArrayList<>(); // lat, lon, cumulative m
        final List<Double> junctions = new ArrayList<>();
        final List<double[]> barriers = new ArrayList<>(); // position m, edge index in edges
        final List<EdgeIteratorState> edges;
        double length;

        Leg(List<EdgeIteratorState> edges) {
            this.edges = edges;
        }

        GHPoint at(double s) {
            s = Math.max(0, Math.min(length, s));
            for (int i = 1; i < pts.size(); i++) {
                double[] a = pts.get(i - 1), b = pts.get(i);
                if (b[2] >= s) {
                    double seg = b[2] - a[2];
                    double f = seg <= 0 ? 0 : (s - a[2]) / seg;
                    return new GHPoint(a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]));
                }
            }
            double[] last = pts.get(pts.size() - 1);
            return new GHPoint(last[0], last[1]);
        }
    }

    private Leg legOf(Path path, QueryGraph queryGraph, EnumEncodedValue<BarrierClass> enc, Set<BarrierClass> crossable) {
        Leg leg = new Leg(path.calcEdges());
        int baseNodes = graph.getNodes();
        EdgeExplorer explorer = graph.createEdgeExplorer();
        double s = 0;
        for (int e = 0; e < leg.edges.size(); e++) {
            EdgeIteratorState edge = leg.edges.get(e);
            PointList pl = edge.fetchWayGeometry(FetchMode.ALL);
            if (leg.pts.isEmpty())
                leg.pts.add(new double[]{pl.getLat(0), pl.getLon(0), 0});
            if (crossable.contains(edge.get(enc)))
                leg.barriers.add(new double[]{s, e});
            for (int i = 1; i < pl.size(); i++) {
                s += DIST_EARTH.calcDist(pl.getLat(i - 1), pl.getLon(i - 1), pl.getLat(i), pl.getLon(i));
                leg.pts.add(new double[]{pl.getLat(i), pl.getLon(i), s});
            }
            int adj = edge.getAdjNode();
            if (e < leg.edges.size() - 1 && adj < baseNodes && degree(explorer, adj) >= 3)
                leg.junctions.add(s);
        }
        leg.length = s;
        return leg;
    }

    private static int degree(EdgeExplorer explorer, int node) {
        int d = 0;
        EdgeIterator it = explorer.setBaseNode(node);
        while (it.next())
            d++;
        return d;
    }

    private List<Map<String, Object>> crossingsOfLeg(Path path, QueryGraph queryGraph, EnumEncodedValue<BarrierClass> enc,
                                                     Set<BarrierClass> crossable, String mode, int legIndex) {
        List<Map<String, Object>> out = new ArrayList<>();
        Leg leg = legOf(path, queryGraph, enc, crossable);
        if (leg.barriers.isEmpty())
            return out;
        NodeAccess na = graph.getNodeAccess();
        int i = 0;
        while (i < leg.barriers.size()) {
            int j = i;
            // a chain of barriers each within 25 m of the previous one is one crossing (no overlapping crossings)
            while (j + 1 < leg.barriers.size() && leg.barriers.get(j + 1)[0] - leg.barriers.get(j)[0] <= GROUP_M)
                j++;
            double first = leg.barriers.get(i)[0], last = leg.barriers.get(j)[0];
            double back = 0, fwd = leg.length;
            boolean backIsJunction = false, fwdIsJunction = false;
            for (double jn : leg.junctions) {
                if (jn <= first && jn >= back) {
                    back = jn;
                    backIsJunction = true;
                }
                if (jn >= last && jn <= fwd) {
                    fwd = jn;
                    fwdIsJunction = true;
                }
            }
            double w1 = place(first, back, backIsJunction, -1);
            double w2 = place(last, fwd, fwdIsJunction, +1);
            List<Map<String, Object>> barriers = new ArrayList<>();
            BarrierClass cls = BarrierClass.ASK;
            List<String> reasons = new ArrayList<>();
            for (int k = i; k <= j; k++) {
                EdgeIteratorState edge = leg.edges.get((int) leg.barriers.get(k)[1]);
                BarrierClass c = edge.get(enc);
                if (c == BarrierClass.REFUSE)
                    cls = BarrierClass.REFUSE;
                BarrierTable.Row row = table.lookup(edge, na);
                Map<String, Object> b = new LinkedHashMap<>();
                b.put("osm_node", row == null ? -1 : row.osm_node);
                b.put("lat", round(na.getLat(edge.getBaseNode())));
                b.put("lon", round(na.getLon(edge.getBaseNode())));
                b.put("class", c.name().toLowerCase(Locale.ROOT));
                String reason = row == null ? null : ("foot".equals(mode) ? row.foot_reason : row.bike_reason);
                b.put("reason_code", reason);
                b.put("tags", row == null ? Collections.emptyMap() : row.tags);
                barriers.add(b);
                if (reason != null && !reasons.contains(reason))
                    reasons.add(reason);
            }
            GHPoint p1 = leg.at(w1), p2 = leg.at(w2);
            Map<String, Object> c = new LinkedHashMap<>();
            c.put("class", cls.name().toLowerCase(Locale.ROOT));
            c.put("mode", mode);
            c.put("leg", legIndex);
            c.put("reason_codes", reasons);
            c.put("barriers", barriers);
            c.put("w1", Arrays.asList(round(p1.lon), round(p1.lat)));
            c.put("w2", Arrays.asList(round(p2.lon), round(p2.lat)));
            c.put("w1_is_route_point", w1 <= 0);
            c.put("w2_is_route_point", w2 >= leg.length);
            c.put("length_m", Math.round((w2 - w1) * 10) / 10.0);
            c.put(W1_M, w1); // leg metres, turned into point indexes by addPointIndexes, then removed
            c.put(W2_M, w2);
            out.add(c);
            i = j + 1;
        }
        return out;
    }

    /**
     * W1 (dir -1) or W2 (dir +1): 12.5 m from the barrier along the leg, but never past the nearest junction (2 m
     * before it) and never past the leg's end (then the route point itself). If a junction is closer than 2 m, half
     * way to it.
     */
    static double place(double barrier, double limit, boolean limitIsJunction, int dir) {
        double want = barrier + dir * HALF_CROSSING_M;
        if (!limitIsJunction)
            return dir < 0 ? Math.max(want, limit) : Math.min(want, limit);
        double stop = limit - dir * JUNCTION_GAP_M;
        if (dir < 0 ? stop >= barrier : stop <= barrier)
            return (barrier + limit) / 2;
        return dir < 0 ? Math.max(want, stop) : Math.min(want, stop);
    }

    private Map<String, Object> refusalOf(Map<String, Object> crossing, int leg) {
        @SuppressWarnings("unchecked")
        List<Map<String, Object>> barriers = (List<Map<String, Object>>) crossing.get("barriers");
        Map<String, Object> barrier = barriers.get(0);
        for (Map<String, Object> b : barriers)
            if ("refuse".equals(b.get("class"))) {
                barrier = b;
                break;
            }
        Map<String, Object> r = new LinkedHashMap<>();
        r.put("message", "Point " + (leg + 1) + " can only be reached through a closed barrier ("
                + barrier.get("reason_code") + ") at " + barrier.get("lat") + "," + barrier.get("lon"));
        r.put("details", "BarrierRefused"); // replaces the class name in GraphHopper's error hint
        r.put("point_index", leg + 1);
        r.put("leg_index", leg);
        r.put("mode", crossing.get("mode"));
        r.put("class", "refuse");
        r.put("reason_code", barrier.get("reason_code"));
        r.put("barrier", barrier);
        r.put("nearest_approach", crossing.get("w1"));
        // the refused crossing in the ask-crossing shape (owner 2026-11-02: "Draw a direct line" = the same ~25 m
        // W1 / direct / W2 as an ask crossing); placed along the path through it (the ask_refuse search), §1.5 rules
        Map<String, Object> c = new LinkedHashMap<>();
        for (String k : new String[]{"class", "mode", "reason_codes", "barriers", "w1", "w2", "w1_is_route_point",
                "w2_is_route_point", "length_m"})
            c.put(k, crossing.get(k));
        c.put("class", "refuse");
        r.put("crossing", c);
        return r;
    }

    // ---------------------------------------------------------------------------------------------- notices

    private static long cell(double lat, double lon) {
        return BarrierTable.cell(lat, lon);
    }

    /**
     * For a router subclass that adds paths after {@link #route} (e.g. /personal_route's bit_of_new variations): gives
     * each added path its {@code hints.barriers.paths[i]} entry, no crossings and its opening-hours notices, so there is
     * one entry per path. Flag only; does nothing without the flag or without the hint.
     */
    @SuppressWarnings("unchecked")
    public void addNoticesForAddedPaths(GHRequest request, GHResponse rsp) {
        if (!isOn(request) || rsp.hasErrors())
            return;
        Object b = rsp.getHints().getObject(HINT, null);
        if (!(b instanceof Map))
            return;
        Object listed = ((Map<String, Object>) b).get("paths");
        if (!(listed instanceof List))
            return;
        List<Map<String, Object>> paths = (List<Map<String, Object>>) listed;
        Profile profile = profilesByName.get(request.getProfile());
        BarrierClass.Mode mode = profile == null ? null : BarrierProfiles.modeOf(profile);
        for (int p = paths.size(); p < rsp.getAll().size(); p++) {
            Map<String, Object> pm = new LinkedHashMap<>();
            pm.put("crossings", Collections.emptyList());
            pm.put("notices", mode == null ? Collections.emptyList() : noticesOf(rsp.getAll().get(p).getPoints(), mode));
            paths.add(pm);
        }
    }

    /** Puts {@code hints.barriers} on the response: version, and per path its crossings and notices. */
    @SuppressWarnings("unchecked")
    private void addNotices(GHRequest request, GHResponse rsp) {
        if (rsp.hasErrors())
            return;
        Profile profile = profilesByName.get(request.getProfile());
        BarrierClass.Mode mode = profile == null ? null : BarrierProfiles.modeOf(profile);
        Object stored = rsp.getHints().getObject("barriers.crossings", null);
        List<Map<String, Object>> crossings = stored == null ? Collections.emptyList() : (List<Map<String, Object>>) stored;
        rsp.getHints().remove("barriers.crossings");
        List<Map<String, Object>> paths = new ArrayList<>();
        for (int p = 0; p < rsp.getAll().size(); p++) {
            Map<String, Object> pm = new LinkedHashMap<>();
            pm.put("crossings", p == 0 ? crossings : Collections.emptyList());
            pm.put("notices", mode == null ? Collections.emptyList() : noticesOf(rsp.getAll().get(p).getPoints(), mode));
            paths.add(pm);
        }
        Map<String, Object> b = new LinkedHashMap<>();
        b.put("version", VERSION);
        b.put("paths", paths);
        rsp.getHints().putObject(HINT, b);
    }

    private List<Map<String, Object>> noticesOf(PointList pts, BarrierClass.Mode mode) {
        List<Map<String, Object>> out = new ArrayList<>();
        if (noticeCells.isEmpty() || pts == null || pts.size() < 2)
            return out;
        Set<BarrierTable.Row> seen = new LinkedHashSet<>();
        for (int i = 1; i < pts.size(); i++) {
            double aLat = pts.getLat(i - 1), aLon = pts.getLon(i - 1), bLat = pts.getLat(i), bLon = pts.getLon(i);
            long c0 = cell(Math.min(aLat, bLat), Math.min(aLon, bLon)), c1 = cell(Math.max(aLat, bLat), Math.max(aLon, bLon));
            long latLo = Math.floorDiv(c0, 1_000_000L), latHi = Math.floorDiv(c1, 1_000_000L);
            long lonLo = Math.floorMod(c0, 1_000_000L), lonHi = Math.floorMod(c1, 1_000_000L);
            for (long la = latLo; la <= latHi; la++)
                for (long lo = lonLo; lo <= lonHi; lo++) {
                    List<BarrierTable.Row> rows = noticeCells.get(la * 1_000_000L + lo);
                    if (rows == null)
                        continue;
                    for (BarrierTable.Row r : rows) {
                        if (!BarrierClass.NOTICE.name().equals(mode == BarrierClass.Mode.FOOT ? r.foot : r.bike))
                            continue;
                        // a path that starts or ends at the barrier does not pass through it
                        if (DIST_EARTH.calcDist(r.lat, r.lon, pts.getLat(0), pts.getLon(0)) <= NOTICE_M
                                || DIST_EARTH.calcDist(r.lat, r.lon, pts.getLat(pts.size() - 1), pts.getLon(pts.size() - 1)) <= NOTICE_M)
                            continue;
                        if (distToSegment(r.lat, r.lon, aLat, aLon, bLat, bLon) <= NOTICE_M)
                            seen.add(r);
                    }
                }
        }
        for (BarrierTable.Row r : seen) {
            Map<String, Object> n = new LinkedHashMap<>();
            n.put("type", "opening_hours");
            n.put("osm_node", r.osm_node);
            n.put("lat", round(r.lat));
            n.put("lon", round(r.lon));
            n.put("opening_hours", r.tags.get("opening_hours"));
            n.put("barrier", r.tags.get("barrier"));
            out.add(n);
        }
        return out;
    }

    private static double distToSegment(double lat, double lon, double aLat, double aLon, double bLat, double bLon) {
        if (DIST_EARTH.validEdgeDistance(lat, lon, aLat, aLon, bLat, bLon))
            return DIST_EARTH.calcDenormalizedDist(DIST_EARTH.calcNormalizedEdgeDistance(lat, lon, aLat, aLon, bLat, bLon));
        return Math.min(DIST_EARTH.calcDist(lat, lon, aLat, aLon), DIST_EARTH.calcDist(lat, lon, bLat, bLon));
    }

    private static double round(double v) {
        return Math.round(v * 1e6) / 1e6;
    }
}
