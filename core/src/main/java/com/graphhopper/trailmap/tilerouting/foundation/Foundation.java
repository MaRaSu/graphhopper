package com.graphhopper.trailmap.tilerouting.foundation;

import com.carrotsearch.hppc.IntArrayList;
import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.ResponsePath;
import com.graphhopper.routing.ev.Subnetwork;
import com.graphhopper.routing.util.DefaultSnapFilter;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.util.AllEdgesIterator;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.util.*;
import com.graphhopper.util.shapes.BBox;
import com.graphhopper.util.shapes.GHPoint;

import java.util.*;

/**
 * The shared tile_routing foundation every candidate builds on: inset tiles, directed portals, directed reachability
 * from and back to home, bounded one-to-many costs ({@link EdgeDijkstra}), route assembly through GraphHopper's
 * normal via routing, and a light coverage check for development. Official scoring is the independent Node verifier.
 */
public final class Foundation {

    /** Levers (owner's: margin; the rest are prototype defaults, printed with every run). */
    public static final class Params {
        public double marginM = 40;
        /** How far past the inset border the portal point sits (at most half the inside chord). */
        public double placeM = 15;
        /** Keep the portal point at least this far from the edge's end nodes when the inside part allows. */
        public double nodeClearM = 5;
        /** An edge that leaves the inset again within this many metres is a shallow clip. */
        public double shallowChordM = 30;
        public int minPortals = 2, maxPortals = 6;
        /** Reachability flood radius around home (straight line). Beyond it = unresolved, never unreachable. */
        public double reachRadiusM = 40_000;
        public int maxSettled = 3_000_000;
        public double deadEndWalkM = 5_000;
        /**
         * Walk-in (round 3): a tile with no rideable, home-connected way entering it gets walk portals at rideable,
         * home-connected road points A within this straight-line distance of the inset (one way, metres). 0 = off:
         * identical to round 2. Lever -Dtr.walk.maxM (default 300; very_strict job sets default 0).
         */
        public double walkMaxM = 0;
        /** Price of one walked metre in the profile's weight units (both ways count), so a ride always beats a walk. */
        public double walkWeightPerM = 2.0;
        /** Diagnostic only: the nearest road point is measured out to this radius for every unreachable tile. */
        public double walkDiagM = 1_000;
        /** T sits this far inside the inset border, so the straight line clearly enters the inset. */
        public double walkInsideM = 1;
        /**
         * W26 (flag only): a walk never passes a closed barrier. A walk line A→T that comes within {@link #walkBarrierM}
         * of a barrier closed for the profile (class ask or refuse; the barrier is a point, its 0 m barrier edge) is not
         * offered; a tile that loses every walk that way stays unreachable and its reason names the barrier.
         */
        public boolean walkAvoidsBarriers = false;
        public double walkBarrierM = 5;

        @Override
        public String toString() {
            return String.format(Locale.ROOT, "marginM=%.0f placeM=%.0f nodeClearM=%.0f shallowChordM=%.0f portals=%d..%d reachRadiusM=%.0f maxSettled=%d",
                    marginM, placeM, nodeClearM, shallowChordM, minPortals, maxPortals, reachRadiusM, maxSettled)
                    + (walkMaxM > 0 ? String.format(Locale.ROOT, " walkMaxM=%.0f walkWeightPerM=%.2f", walkMaxM, walkWeightPerM) : "");
        }
    }

    /** Per requested tile: its portals and status. */
    public static final class TileInfo {
        public TileMath.Tile tile;
        public List<Portal> portals = new ArrayList<>();
        /** Entering crossings before reachability and selection. */
        public int crossings;
        /** reachable | unreachable | unresolved */
        public String status;
        /**
         * Shallow tile: no rideable way enters the normal inset, so {@link #tile} carries a smaller margin,
         * min(margin, 0.8 × this depth). NaN for ordinary tiles.
         */
        public double shallowDepthM = Double.NaN;
        public String reason;
        /**
         * Walk tile (status "walk"): no rideable, home-connected way enters it, and {@link #portals} are road points
         * A within {@link Params#walkMaxM} of the inset, each with its walk target ({@link Portal#walkM} &gt; 0).
         */
        public boolean walk;
        /** Diagnostic for an unreachable or walk tile: straight-line metres from the nearest road point A to the inset (NaN = none within walkDiagM). */
        public double walkNearestM = Double.NaN;
    }

    /** Home: its snapped edge as two directed states, and the directed reachability flood. */
    public static final class Home {
        public double lat, lon;
        public List<Portal> states = new ArrayList<>();
        public SparseKeys fwd, back;
        public boolean fwdBounded, backBounded;
        public long ms;
    }

    public final GraphHopper hopper;
    public final String profile;
    public final Weighting weighting;
    public final BaseGraph graph;
    public final NodeAccess na;
    public final Params params;
    public final EdgeDijkstra dijkstra;
    private final EdgeExplorer explorer;

    /**
     * Per-request custom model merged onto the profile's own (the safety level "strict": priority 0 on hard
     * gravel_scale classes), exactly as GraphHopper's router merges a request's custom_model; null = the profile as is.
     */
    public final CustomModel customModel;

    public Foundation(GraphHopper hopper, String profile, Params params) {
        this(hopper, profile, null, params);
    }

    public Foundation(GraphHopper hopper, String profile, CustomModel customModel, Params params) {
        this.hopper = hopper;
        this.profile = profile;
        this.params = params;
        this.customModel = customModel;
        PMap hints = new PMap();
        if (customModel != null) hints.putObject(CustomModel.KEY, customModel);
        this.weighting = hopper.createWeighting(hopper.getProfile(profile), hints);
        this.graph = hopper.getBaseGraph();
        this.na = graph.getNodeAccess();
        this.explorer = graph.createEdgeExplorer();
        this.dijkstra = new EdgeDijkstra(graph, weighting);
        // GraphHopper's own snap filter: rideable and not on a small island cut off by prepare.min_network_size
        this.snapFilter = new DefaultSnapFilter(weighting, hopper.getEncodingManager().getBooleanEncodedValue(Subnetwork.key(profile)));
    }

    /**
     * Measures the smallest weight per metre over every rideable directed edge of the graph (edges under 1 m are
     * skipped; their slack is below a metre's worth of weight) and gives it to the A* heuristic. Returns
     * {weighting's own bound, measured bound, ms}.
     */
    public double[] tightenHeuristic() {
        long t0 = System.nanoTime();
        double min = Double.POSITIVE_INFINITY;
        AllEdgesIterator it = graph.getAllEdges();
        while (it.next()) {
            double d = it.getDistance();
            if (d < 1) continue;
            double w = Math.min(weighting.calcEdgeWeight(it, false), weighting.calcEdgeWeight(it, true));
            if (Double.isFinite(w)) min = Math.min(min, w / d);
        }
        dijkstra.minWeightPerM = min;
        return new double[]{weighting.calcMinWeightPerDistance(), min, (System.nanoTime() - t0) / 1e6};
    }

    /** Snap filter for home and plain via points, as GraphHopper's router uses. */
    public final EdgeFilter snapFilter;

    /** The same point ridden the other way (null if that direction is not rideable). */
    public Portal twin(Portal p) {
        EdgeIteratorState st = graph.getEdgeIteratorStateForKey(p.key ^ 1);
        double w = weighting.calcEdgeWeight(st, false);
        if (!Double.isFinite(w)) return null;
        Portal q = new Portal();
        q.tileKey = p.tileKey;
        q.edge = p.edge;
        q.key = p.key ^ 1;
        q.fromNode = p.toNode;
        q.toNode = p.fromNode;
        q.lat = p.lat;
        q.lon = p.lon;
        q.heading = (p.heading + 180) % 360;
        q.frac = 1 - p.frac;
        q.edgeWeight = w;
        q.edgeLenM = p.edgeLenM;
        q.side = p.side;
        q.chordM = p.chordM;
        q.exitsInEdge = p.exitsInEdge;
        q.name = p.name;
        q.walkM = p.walkM;
        q.walkLat = p.walkLat;
        q.walkLon = p.walkLon;
        return q;
    }

    public boolean rideable(EdgeIteratorState e) {
        return Double.isFinite(weighting.calcEdgeWeight(e, false)) || Double.isFinite(weighting.calcEdgeWeight(e, true));
    }

    // ------------------------------------------------------------------ home and reachability

    public Home home(double lat, double lon) {
        long t0 = System.nanoTime();
        Home h = new Home();
        h.lat = lat;
        h.lon = lon;
        Snap snap = hopper.getLocationIndex().findClosest(lat, lon, snapFilter);
        if (!snap.isValid()) throw new IllegalStateException("home does not snap");
        EdgeIteratorState se = snap.getClosestEdge();
        for (int key : new int[]{se.getEdgeKey(), se.getReverseEdgeKey()}) {
            EdgeIteratorState st = graph.getEdgeIteratorStateForKey(key);
            double w = weighting.calcEdgeWeight(st, false);
            if (!Double.isFinite(w)) continue;
            PointList pl = st.fetchWayGeometry(FetchMode.ALL);
            Portal p = basePortal("home", st, w, pl);
            double[] proj = project(pl, snap.getSnappedPoint().lat, snap.getSnappedPoint().lon);
            p.frac = p.edgeLenM > 0 ? proj[0] / p.edgeLenM : 0;
            p.lat = snap.getSnappedPoint().lat;
            p.lon = snap.getSnappedPoint().lon;
            p.heading = proj[1];
            h.states.add(p);
        }
        h.fwd = new SparseKeys();
        h.back = new SparseKeys();
        h.fwdBounded = flood(h, true);
        h.backBounded = flood(h, false);
        h.ms = (System.nanoTime() - t0) / 1_000_000;
        return h;
    }

    /**
     * Directed, turn-legal flood over edge keys within the radius. Forward: keys you can arrive on from home.
     * Backward: keys from which home can be reached. Returns true if the radius cut anything off.
     */
    private boolean flood(Home h, boolean forward) {
        return flood(h.states, forward ? h.fwd : h.back, forward, h.lat, h.lon, Integer.MAX_VALUE);
    }

    /**
     * The same flood from any seeds, centred on (lat0, lon0) with the reach radius, stopping after maxKeys keys.
     * Returns true if the radius or the key cap cut anything off (the result is then not a proof of closure).
     */
    private boolean flood(List<Portal> seeds, SparseKeys seen, boolean forward, double lat0, double lon0, int maxKeys) {
        IntArrayList qKey = new IntArrayList(), qNode = new IntArrayList();
        for (Portal p : seeds) {
            seen.set(p.key);
            qKey.add(p.key);
            qNode.add(forward ? p.toNode : p.fromNode);
        }
        boolean bounded = false;
        double r = params.reachRadiusM;
        for (int head = 0; head < qKey.size(); head++) {
            if (qKey.size() >= maxKeys) return true;
            int key = qKey.get(head), node = qNode.get(head), edge = key >> 1;
            EdgeIterator it = explorer.setBaseNode(node);
            while (it.next()) {
                int next = forward ? it.getEdgeKey() : it.getReverseEdgeKey();
                if (seen.get(next)) continue;
                int other = it.getAdjNode();
                if (dist(lat0, lon0, na.getLat(other), na.getLon(other)) > r) {
                    bounded = true;
                    continue;
                }
                if (!Double.isFinite(weighting.calcEdgeWeight(it, !forward))) continue;
                double t = forward ? weighting.calcTurnWeight(edge, node, it.getEdge()) : weighting.calcTurnWeight(it.getEdge(), node, edge);
                if (!Double.isFinite(t)) continue;
                seen.set(next);
                qKey.add(next);
                qNode.add(other);
            }
        }
        return bounded;
    }

    // ------------------------------------------------------------------ tiles and portals

    public TileInfo analyse(String tileKey, int zoom, Home home) {
        TileInfo ti = new TileInfo();
        ti.tile = TileMath.Tile.parse(tileKey, zoom, params.marginM);
        TileMath.Tile t = ti.tile;
        Set<Integer> edges = new TreeSet<>();
        hopper.getLocationIndex().query(new BBox(t.minLon, t.maxLon, t.minLat, t.maxLat), edges::add);
        List<Portal> all = entriesOf(t, edges);
        if (all.isEmpty()) {
            // Graded margin (tile_verify / owner, 2026-10-27, Turvesuonkatu z17 74147-36927): the inset is the safety
            // margin for counting a tile, not a test of rideability. If no rideable way enters the inset but a
            // rideable, home-connected one enters the tile, the tile is "shallow": its inset shrinks to
            // min(margin, 0.8 × the deepest point available) and it is routed like any other tile.
            double deepest = deepestRideable(t, edges, home);
            if (deepest <= 0) {
                ti.status = "unreachable";
                ti.reason = "no rideable, home-connected way enters the tile";
                return walkIn(ti, home);
            }
            ti.shallowDepthM = deepest;
            ti.tile = new TileMath.Tile(t.z, t.x, t.y, Math.min(params.marginM, 0.8 * deepest));
            t = ti.tile;
            all = entriesOf(t, edges);
            if (all.isEmpty()) {
                ti.status = "unresolved";
                ti.reason = String.format(Locale.ROOT, "shallow tile (deepest %.1f m) but no entry into its %.1f m inset", deepest, t.marginM);
                return ti;
            }
        }
        ti.crossings = all.size();
        List<Portal> ok = new ArrayList<>();
        int noFwd = 0, noBack = 0;
        for (Portal p : all) {
            boolean f = home.fwd.get(p.key), b = home.back.get(p.key);
            if (!f) noFwd++;
            if (!b) noBack++;
            if (f && b) ok.add(p);
        }
        if (ok.isEmpty()) {
            boolean bounded = home.fwdBounded || home.backBounded;
            String island = bounded ? islandProof(all, home) : null;
            ti.status = !bounded || island != null ? "unreachable" : "unresolved";
            ti.reason = String.format(Locale.ROOT, "%d entries, %d not reachable from home, %d cannot return home%s", all.size(),
                    noFwd, noBack, island != null ? "; " + island
                            : bounded ? " (flood bounded at " + Math.round(params.reachRadiusM / 1000) + " km)" : "");
            return "unreachable".equals(ti.status) ? walkIn(ti, home) : ti;
        }
        for (Portal p : ok) markDeadEnd(p);
        ti.portals = select(ok);
        ti.status = "reachable";
        return ti;
    }

    // ------------------------------------------------------------------ walk-in (round 3)

    /**
     * An unreachable tile (no rideable, home-connected way enters it): measure the nearest rideable, home-connected
     * road point A (diagnostic, out to walkDiagM) and, when walking is on and A lies within walkMaxM of the inset,
     * turn the tile into a walk tile whose portals are the best road point A per approach side. Each portal carries
     * its walk target T (the closest point of the inset to A, {@link Params#walkInsideM} inside the border) and the
     * one-way walk metres A→T. To the candidates a walk portal is an ordinary portal at A.
     */
    private TileInfo walkIn(TileInfo ti, Home home) {
        List<Portal> cands = walkCandidates(ti.tile, home, Math.max(params.walkMaxM, params.walkDiagM));
        for (Portal p : cands) ti.walkNearestM = Double.isNaN(ti.walkNearestM) ? p.walkM : Math.min(ti.walkNearestM, p.walkM);
        if (params.walkMaxM <= 0) return ti;
        List<Portal> ok = new ArrayList<>();
        for (Portal p : cands) if (p.walkM <= params.walkMaxM) ok.add(p);
        if (ok.isEmpty()) return ti;
        ok.sort(Comparator.comparingDouble((Portal p) -> p.walkM).thenComparingInt(p -> p.key));
        if (ok.size() > params.maxPortals) ok = new ArrayList<>(ok.subList(0, params.maxPortals));
        // W26 (flag only): a walk that passes a closed barrier is not offered. Dropped after the portal cap, so no
        // other walk takes its place (losing a walk is the only change); the diagnostic above is unchanged.
        List<String> blocked = new ArrayList<>();
        if (params.walkAvoidsBarriers) ok = withoutBarrierWalks(ok, blocked);
        if (ok.isEmpty()) {
            ti.reason = ti.reason + "; can only be reached through a closed barrier " + blocked.get(0);
            return ti;
        }
        ti.walk = true;
        ti.status = "walk";
        ti.portals = ok;
        ti.reason = String.format(Locale.ROOT, "%s; walk-in %.0f m (%d road points within %.0f m)", ti.reason, ok.get(0).walkM, ok.size(), params.walkMaxM);
        return ti;
    }

    /**
     * The walks whose straight line A→T stays farther than {@link Params#walkBarrierM} from every barrier closed for the
     * profile (a 0 m barrier edge of class ask or refuse; tested against the line, not only its road end A). For each
     * dropped walk, "(REASON) at lat,lon" of the barrier it passes goes into {@code blocked}, in BarrierRouter's
     * refusal wording.
     */
    private List<Portal> withoutBarrierWalks(List<Portal> walks, List<String> blocked) {
        List<Portal> out = new ArrayList<>();
        for (Portal p : walks) {
            String hit = closedBarrierOn(p.lat, p.lon, p.walkLat, p.walkLon);
            if (hit == null) out.add(p);
            else blocked.add(hit);
        }
        return out;
    }

    /**
     * "(REASON) at lat,lon" of a barrier closed for the profile (a 0 m barrier edge of class ask or refuse) within
     * {@link Params#walkBarrierM} of the straight line from (aLat, aLon) to (tLat, tLon); null when there is none.
     */
    private String closedBarrierOn(double aLat, double aLon, double tLat, double tLon) {
        com.graphhopper.routing.ev.EnumEncodedValue<com.graphhopper.trailmap.barrier.BarrierClass> enc =
                com.graphhopper.trailmap.barrier.BarrierProfiles.barrierEnc(hopper.getEncodingManager(), hopper.getProfile(profile));
        if (enc == null) return null;
        boolean foot = com.graphhopper.trailmap.barrier.BarrierClass.KEY_FOOT.equals(enc.getName());
        com.graphhopper.trailmap.barrier.BarrierTable table = com.graphhopper.trailmap.barrier.BarrierTable.of(hopper);
        double minLat = Math.min(aLat, tLat), maxLat = Math.max(aLat, tLat);
        double minLon = Math.min(aLon, tLon), maxLon = Math.max(aLon, tLon);
        double padLat = params.walkBarrierM / TileMath.M_PER_DEG_LAT, padLon = params.walkBarrierM / (TileMath.M_PER_DEG_LON_EQ * Math.cos(Math.toRadians(aLat)));
        Set<Integer> edges = new TreeSet<>();
        hopper.getLocationIndex().query(new BBox(minLon - padLon, maxLon + padLon, minLat - padLat, maxLat + padLat), edges::add);
        for (int e : edges) {
            EdgeIteratorState st = graph.getEdgeIteratorState(e, Integer.MIN_VALUE);
            if (st.getDistance() >= 0.01 || !st.get(enc).closes()) continue;
            double bLat = na.getLat(st.getBaseNode()), bLon = na.getLon(st.getBaseNode());
            if (bLat != na.getLat(st.getAdjNode()) || bLon != na.getLon(st.getAdjNode())) continue;
            double[] t = xy(aLat, aLon, tLat, tLon), b = xy(aLat, aLon, bLat, bLon);
            double len2 = t[0] * t[0] + t[1] * t[1];
            double f = len2 <= 0 ? 0 : Math.max(0, Math.min(1, (b[0] * t[0] + b[1] * t[1]) / len2));
            if (Math.hypot(b[0] - f * t[0], b[1] - f * t[1]) > params.walkBarrierM) continue;
            com.graphhopper.trailmap.barrier.BarrierTable.Row row = table.lookup(st, na);
            String reason = row == null ? null : foot ? row.foot_reason : row.bike_reason;
            return String.format(Locale.ROOT, "(%s) at %.7f,%.7f", reason, bLat, bLon);
        }
        return null;
    }

    /** Local metres east/north of (lat0, lon0). */
    private static double[] xy(double lat0, double lon0, double lat, double lon) {
        return new double[]{(lon - lon0) * TileMath.M_PER_DEG_LON_EQ * Math.cos(Math.toRadians(lat0)), (lat - lat0) * TileMath.M_PER_DEG_LAT};
    }

    /**
     * The best road point per approach side (the side of the inset its closest inset point lies on: N, S, E, W or a
     * corner such as NE) among rideable edges GraphHopper can snap to and that are in home's reach both ways, within
     * radius metres of the inset. Sampled along each edge's geometry every metre. Each result is a portal at A with
     * walkM = metres A→T, T = the inset point closest to A pushed walkInsideM inside.
     */
    public List<Portal> walkCandidates(TileMath.Tile t, Home home, double radius) {
        double cLat = t.centreLat();
        double dLat = radius / TileMath.M_PER_DEG_LAT, dLon = radius / (TileMath.M_PER_DEG_LON_EQ * Math.cos(Math.toRadians(cLat)));
        Set<Integer> edges = new TreeSet<>();
        hopper.getLocationIndex().query(new BBox(t.inMinLon - dLon, t.inMaxLon + dLon, t.inMinLat - dLat, t.inMaxLat + dLat), edges::add);
        // the inset in local metres around its south-west corner
        double[] ne = xy(t.inMinLat, t.inMinLon, t.inMaxLat, t.inMaxLon);
        double in = params.walkInsideM;
        Map<String, Portal> best = new TreeMap<>();
        for (int e : edges) {
            EdgeIteratorState st0 = graph.getEdgeIteratorState(e, Integer.MIN_VALUE);
            if (!snapFilter.accept(st0)) continue;
            int key = -1;
            double w = Double.NaN;
            for (int k : new int[]{st0.getEdgeKey(), st0.getReverseEdgeKey()}) {
                EdgeIteratorState st = graph.getEdgeIteratorStateForKey(k);
                double wk = weighting.calcEdgeWeight(st, false);
                if (Double.isFinite(wk) && home.fwd.get(k) && home.back.get(k)) {
                    key = k;
                    w = wk;
                    break;
                }
            }
            if (key < 0) continue;
            EdgeIteratorState st = graph.getEdgeIteratorStateForKey(key);
            PointList pl = st.fetchWayGeometry(FetchMode.ALL);
            double[] cum = cumulative(pl);
            double total = cum[pl.size() - 1];
            for (int i = 0; i + 1 < pl.size(); i++) {
                double segLen = cum[i + 1] - cum[i];
                int steps = Math.max(1, (int) Math.ceil(segLen));
                // per segment: the best sample per side
                for (int s = 0; s <= steps; s++) {
                    double f = (double) s / steps;
                    double la = pl.getLat(i) + f * (pl.getLat(i + 1) - pl.getLat(i)), lo = pl.getLon(i) + f * (pl.getLon(i + 1) - pl.getLon(i));
                    double[] p = xy(t.inMinLat, t.inMinLon, la, lo);
                    double tx = Math.max(in, Math.min(ne[0] - in, p[0])), ty = Math.max(in, Math.min(ne[1] - in, p[1]));
                    double d = Math.hypot(p[0] - tx, p[1] - ty);
                    if (d > radius || d < 1e-6) continue;
                    String side = (p[1] > ne[1] - in ? "N" : p[1] < in ? "S" : "") + (p[0] > ne[0] - in ? "E" : p[0] < in ? "W" : "");
                    if (side.isEmpty()) continue; // inside the inset: cannot happen for an unreachable tile
                    Portal b = best.get(side);
                    if (b != null && b.walkM <= d) continue;
                    Portal q = basePortal(t.key, st, w, pl);
                    double along = cum[i] + f * segLen;
                    q.frac = total > 0 ? along / total : 0;
                    q.lat = la;
                    q.lon = lo;
                    q.heading = bearing(pl.getLat(i), pl.getLon(i), pl.getLat(i + 1), pl.getLon(i + 1));
                    q.side = side;
                    q.walkM = d;
                    q.walkLat = t.inMinLat + ty / TileMath.M_PER_DEG_LAT;
                    q.walkLon = t.inMinLon + tx / (TileMath.M_PER_DEG_LON_EQ * Math.cos(Math.toRadians(t.inMinLat)));
                    best.put(side, q);
                }
            }
        }
        return new ArrayList<>(best.values());
    }

    /** Every rideable directed entry into the tile's inset square among the given edges. */
    private List<Portal> entriesOf(TileMath.Tile t, Set<Integer> edges) {
        List<Portal> all = new ArrayList<>();
        for (int e : edges) {
            for (int key : new int[]{GHUtility.createEdgeKey(e, false), GHUtility.createEdgeKey(e, true)}) {
                EdgeIteratorState st = graph.getEdgeIteratorStateForKey(key);
                double w = weighting.calcEdgeWeight(st, false);
                if (!Double.isFinite(w)) continue;
                all.addAll(entries(t, st, w));
            }
        }
        return all;
    }

    /**
     * How far (metres from the nearest tile border) a rideable edge that GraphHopper can snap to and that is in
     * home's reach both ways gets into the tile; 0 if none enters it. Sampled along the geometry every metre.
     */
    private double deepestRideable(TileMath.Tile t, Set<Integer> edges, Home home) {
        double deepest = 0;
        for (int e : edges) {
            EdgeIteratorState st = graph.getEdgeIteratorState(e, Integer.MIN_VALUE);
            if (!snapFilter.accept(st)) continue;
            if (!(home.fwd.get(st.getEdgeKey()) || home.fwd.get(st.getReverseEdgeKey()))
                    || !(home.back.get(st.getEdgeKey()) || home.back.get(st.getReverseEdgeKey()))) continue;
            PointList pl = st.fetchWayGeometry(FetchMode.ALL);
            for (int i = 0; i + 1 < pl.size(); i++) {
                double la1 = pl.getLat(i), lo1 = pl.getLon(i), la2 = pl.getLat(i + 1), lo2 = pl.getLon(i + 1);
                int steps = Math.max(1, (int) Math.ceil(dist(la1, lo1, la2, lo2)));
                for (int s = 0; s <= steps; s++) {
                    double f = (double) s / steps, la = la1 + f * (la2 - la1), lo = lo1 + f * (lo2 - lo1);
                    if (la <= t.minLat || la >= t.maxLat || lo <= t.minLon || lo >= t.maxLon) continue;
                    double mPerLon = TileMath.M_PER_DEG_LON_EQ * Math.cos(Math.toRadians(la));
                    double d = Math.min(Math.min((la - t.minLat) * TileMath.M_PER_DEG_LAT, (t.maxLat - la) * TileMath.M_PER_DEG_LAT),
                            Math.min((lo - t.minLon) * mPerLon, (t.maxLon - lo) * mPerLon));
                    deepest = Math.max(deepest, d);
                }
            }
        }
        return deepest;
    }

    /**
     * Home's flood was cut at the radius, so "not in it" is no proof. Flood from each entry instead: if, for every
     * entry, the keys reachable from it (or the keys that reach it) close within the radius without containing home,
     * the entries are an island and the tile is unreachable. Returns the reason, or null if any entry stays open.
     */
    private String islandProof(List<Portal> entries, Home home) {
        Set<Integer> homeKeys = new HashSet<>();
        for (Portal s : home.states) homeKeys.add(s.key);
        int maxKeys = 200_000, largest = 0;
        boolean anyOut = false, anyIn = false;
        for (Portal p : entries) {
            boolean proven = false;
            for (boolean forward : new boolean[]{true, false}) {
                if (forward ? home.back.get(p.key) : home.fwd.get(p.key)) continue; // this direction is fine
                SparseKeys seen = new SparseKeys();
                boolean cut = flood(List.of(p), seen, forward, home.lat, home.lon, maxKeys);
                boolean hasHome = false;
                for (int k : homeKeys) hasHome |= seen.get(k);
                if (!cut && !hasHome) {
                    proven = true;
                    largest = Math.max(largest, seen.cardinality());
                    if (forward) anyOut = true;
                    else anyIn = true;
                    break;
                }
            }
            if (!proven) return null;
        }
        return String.format(Locale.ROOT, "island: the entries' component closes within %d keys without home (%s)", largest,
                anyOut && anyIn ? "cannot return home / home cannot reach it" : anyOut ? "cannot return home" : "home cannot reach it");
    }

    /** Every place where the directed edge enters the inset square. */
    private List<Portal> entries(TileMath.Tile t, EdgeIteratorState st, double w) {
        List<Portal> out = new ArrayList<>();
        PointList pl = st.fetchWayGeometry(FetchMode.ALL);
        double[] cum = cumulative(pl);
        double total = cum[pl.size() - 1];
        for (int i = 0; i + 1 < pl.size(); i++) {
            double la1 = pl.getLat(i), lo1 = pl.getLon(i), la2 = pl.getLat(i + 1), lo2 = pl.getLon(i + 1);
            if (t.insetContains(la1, lo1)) continue;
            double[] c = t.clip(la1, lo1, la2, lo2);
            if (c == null) continue;
            double segLen = cum[i + 1] - cum[i];
            double sEntry = cum[i] + c[0] * segLen;
            // walk on to where the edge leaves the inset (or ends)
            double sExit = cum[i] + c[1] * segLen;
            boolean exits = c[1] < 1;
            for (int j = i + 1; !exits && j + 1 < pl.size(); j++) {
                double[] cj = t.clip(pl.getLat(j), pl.getLon(j), pl.getLat(j + 1), pl.getLon(j + 1));
                double lj = cum[j + 1] - cum[j];
                if (cj == null) {
                    exits = true;
                    sExit = cum[j];
                } else {
                    sExit = cum[j] + cj[1] * lj;
                    exits = cj[1] < 1;
                }
            }
            if (!exits) sExit = total;
            Portal p = basePortal(t.key, st, w, pl);
            p.chordM = sExit - sEntry;
            p.exitsInEdge = exits;
            double s = sEntry + Math.min(params.placeM, p.chordM / 2);
            // plain via points: keep the point mid-edge, clear of junction nodes, where the inside part allows
            double lo = Math.max(sEntry, params.nodeClearM), hi = Math.min(sExit, total - params.nodeClearM);
            if (lo < hi) s = Math.max(lo, Math.min(hi, s));
            p.frac = total > 0 ? s / total : 0;
            double[] at = pointAt(pl, cum, s);
            p.lat = at[0];
            p.lon = at[1];
            p.heading = at[2];
            p.side = t.side(la1 + c[0] * (la2 - la1), lo1 + c[0] * (lo2 - lo1));
            out.add(p);
        }
        return out;
    }

    private Portal basePortal(String tileKey, EdgeIteratorState st, double w, PointList pl) {
        Portal p = new Portal();
        p.tileKey = tileKey;
        p.edge = st.getEdge();
        p.key = st.getEdgeKey();
        p.fromNode = st.getBaseNode();
        p.toNode = st.getAdjNode();
        p.edgeWeight = w;
        p.edgeLenM = cumulative(pl)[pl.size() - 1];
        String n = st.getName();
        p.name = n == null ? "" : n;
        return p;
    }

    /** Ride on from the portal along degree-2 chains: ending at a degree-1 node = dead end. */
    private void markDeadEnd(Portal p) {
        double walked = (1 - p.frac) * p.edgeLenM;
        int inEdge = p.edge, node = p.toNode;
        while (walked < params.deadEndWalkM) {
            EdgeIterator it = explorer.setBaseNode(node);
            int count = 0, nextEdge = -1, nextNode = -1;
            double nextLen = 0;
            while (it.next()) {
                if (it.getEdge() == inEdge || it.getAdjNode() == node || !rideable(it)) continue;
                count++;
                nextEdge = it.getEdge();
                nextNode = it.getAdjNode();
                nextLen = it.getDistance();
            }
            if (count == 0) {
                p.deadEnd = true;
                p.stubM = walked;
                return;
            }
            if (count > 1) return;
            walked += nextLen;
            inEdge = nextEdge;
            node = nextNode;
        }
    }

    /**
     * 2–6 portals, spread over approach sides and distinct roads: round-robin over sides, inside a side preferring
     * real entries over shallow clips and dead ends, then the profile's preferred roads (low weight per metre).
     */
    private List<Portal> select(List<Portal> ok) {
        Map<String, List<Portal>> bySide = new TreeMap<>();
        for (Portal p : ok) bySide.computeIfAbsent(p.side, k -> new ArrayList<>()).add(p);
        Comparator<Portal> pref = Comparator.<Portal>comparingInt(p -> shallow(p) ? 1 : 0)
                .thenComparingInt(p -> p.deadEnd ? 1 : 0)
                .thenComparingDouble(Portal::weightPerM)
                .thenComparingInt(p -> p.key);
        for (List<Portal> l : bySide.values()) l.sort(pref);
        List<Portal> out = new ArrayList<>();
        Set<String> usedRoad = new HashSet<>();
        Set<Integer> usedEdge = new HashSet<>();
        // pass 1: distinct roads per side; pass 2: fill up with anything left
        for (int pass = 0; pass < 2 && out.size() < params.maxPortals; pass++) {
            boolean added = true;
            while (added && out.size() < params.maxPortals) {
                added = false;
                for (Map.Entry<String, List<Portal>> en : bySide.entrySet()) {
                    if (out.size() >= params.maxPortals) break;
                    for (Iterator<Portal> it = en.getValue().iterator(); it.hasNext(); ) {
                        Portal p = it.next();
                        String road = en.getKey() + "|" + (p.name.isEmpty() ? "e" + p.edge : p.name);
                        boolean fresh = !usedRoad.contains(road) && !usedEdge.contains(p.key);
                        if (pass == 0 && !fresh) continue;
                        if (usedEdge.contains(p.key)) {
                            it.remove();
                            continue;
                        }
                        it.remove();
                        out.add(p);
                        usedRoad.add(road);
                        usedEdge.add(p.key);
                        added = true;
                        break;
                    }
                }
            }
        }
        return out;
    }

    public boolean shallow(Portal p) {
        return p.exitsInEdge && p.chordM < params.shallowChordM;
    }

    // ------------------------------------------------------------------ assembly and the light coverage check

    /** One routed geometry through the via points. */
    public static final class Route {
        public List<double[]> coords = new ArrayList<>();
        public double distanceM;
        /** GraphHopper's route weight (sum over legs), to compare with a candidate's own table cost. */
        public double weight;
        /** Straight-line walk metres (direct legs) inside distanceM; 0 for a fully routed plan. */
        public double walkM;
        public String error;
    }

    public Route assemble(List<Via> vias, boolean passThrough) {
        return assemble(vias, passThrough, false);
    }

    /**
     * asApp: route the way the app rebuilds a saved route (W21): no custom model (the app stores only the profile),
     * snap_preventions ferry as its /route requests send. Otherwise with the request's safety model, as the tour was
     * built.
     */
    public Route assemble(List<Via> vias, boolean passThrough, boolean asApp) {
        Route r = new Route();
        List<GHPoint> pts = new ArrayList<>();
        List<Double> headings = new ArrayList<>();
        boolean anyHeading = false;
        for (Via v : vias) {
            pts.add(new GHPoint(v.lat, v.lon));
            headings.add(v.heading);
            anyHeading |= !Double.isNaN(v.heading);
        }
        GHRequest req = new GHRequest(pts).setProfile(profile);
        if (asApp) req.setSnapPreventions(List.of("ferry"));
        else if (customModel != null) req.setCustomModel(customModel);
        if (anyHeading) req.setHeadings(headings);
        if (passThrough) req.putHint(Parameters.Routing.PASS_THROUGH, true);
        GHResponse res = hopper.route(req);
        if (res.hasErrors()) {
            r.error = res.getErrors().toString();
            return r;
        }
        ResponsePath best = res.getBest();
        PointList pl = best.getPoints();
        for (int i = 0; i < pl.size(); i++) r.coords.add(new double[]{pl.getLat(i), pl.getLon(i)});
        r.distanceM = best.getDistance();
        r.weight = best.getRouteWeight();
        return r;
    }

    // ------------------------------------------------------------------ walk export (round 3)

    /** App segment types (TrailmapInstructionRequest TYPE_FOLLOW_ROADS / TYPE_DIRECT). */
    public static final String FOLLOW_ROADS = "followRoads", DIRECT = "direct";

    /** One walk: road point A → T inside the tile → back to A (stub) or on to another road point B (through). */
    public static final class Walk {
        public String tile, kind;
        public double aLat, aLon, tLat, tLon, bLat, bLon, inM, outM;
        /** Priced cost of the stub and of the best through walk considered (NaN = none). */
        public double stubCost, throughCost = Double.NaN;
    }

    /** The exported plan: waypoints and one segment type per leg (vias.size() − 1). */
    public static final class Plan {
        public List<Via> vias = new ArrayList<>();
        public List<String> segments = new ArrayList<>();
        public List<Walk> walks = new ArrayList<>();

        public boolean hasDirect() {
            return segments.contains(DIRECT);
        }
    }

    /** All legs routed: the candidate's vias as they are (a job with no walk tiles). */
    public static Plan routedPlan(List<Via> vias) {
        Plan p = new Plan();
        p.vias.addAll(vias);
        for (int i = 0; i + 1 < vias.size(); i++) p.segments.add(FOLLOW_ROADS);
        return p;
    }

    /**
     * Expands each via that sits on a walk portal of a walk tile into the walk: A, T (direct), back to A (direct),
     * or A, T, B (direct, direct) when walking out to another of the tile's road points B is cheaper in total
     * (walked metres × walkWeightPerM + GraphHopper's road weight on to the next via). Each walk leg ≤ walkMaxM.
     */
    public Plan expandWalks(List<Via> vias, Map<String, TileInfo> walkTiles) {
        Plan plan = new Plan();
        double W = params.walkWeightPerM;
        for (int i = 0; i < vias.size(); i++) {
            Via v = vias.get(i);
            if (i > 0) plan.segments.add(FOLLOW_ROADS);
            plan.vias.add(v);
            TileInfo ti = v.tileKey == null ? null : walkTiles.get(v.tileKey);
            if (ti == null || i == 0 || i + 1 == vias.size()) continue;
            Portal a = null;
            for (Portal p : ti.portals) if (dist(p.lat, p.lon, v.lat, v.lon) < 1 && (a == null || p.walkM < a.walkM)) a = p;
            if (a == null) continue;
            Via next = vias.get(i + 1);
            Walk w = new Walk();
            w.tile = ti.tile.key;
            w.aLat = a.lat;
            w.aLon = a.lon;
            w.tLat = a.walkLat;
            w.tLon = a.walkLon;
            w.inM = dist(a.lat, a.lon, a.walkLat, a.walkLon);
            w.outM = w.inM;
            w.bLat = a.lat;
            w.bLon = a.lon;
            w.kind = "stub";
            Route onA = assemble(List.of(v, next), false);
            w.stubCost = onA.error == null ? 2 * w.inM * W + onA.weight : Double.POSITIVE_INFINITY;
            double bestCost = w.stubCost;
            for (Portal b : ti.portals) {
                if (b == a || dist(a.lat, a.lon, b.lat, b.lon) < 5) continue;
                double[] t = throughPoint(ti.tile, a, b);
                if (t == null) continue;
                double in = dist(a.lat, a.lon, t[0], t[1]), out = dist(t[0], t[1], b.lat, b.lon);
                if (in > params.walkMaxM || out > params.walkMaxM) continue;
                // W26 (flag only): a through walk never passes a closed barrier either (in leg A→T or out leg T→B)
                if (params.walkAvoidsBarriers && (closedBarrierOn(a.lat, a.lon, t[0], t[1]) != null || closedBarrierOn(t[0], t[1], b.lat, b.lon) != null))
                    continue;
                Route onB = assemble(List.of(new Via(b.lat, b.lon, Double.NaN, "walkout", b.tileKey), next), false);
                if (onB.error != null) continue;
                double c = (in + out) * W + onB.weight;
                if (Double.isNaN(w.throughCost) || c < w.throughCost) w.throughCost = c;
                if (c < bestCost) {
                    bestCost = c;
                    w.kind = "through";
                    w.tLat = t[0];
                    w.tLon = t[1];
                    w.inM = in;
                    w.outM = out;
                    w.bLat = b.lat;
                    w.bLon = b.lon;
                }
            }
            plan.segments.add(DIRECT);
            plan.vias.add(new Via(w.tLat, w.tLon, Double.NaN, "walk", w.tile));
            plan.segments.add(DIRECT);
            plan.vias.add(new Via(w.bLat, w.bLon, Double.NaN, "walkout", w.tile));
            plan.walks.add(w);
        }
        return plan;
    }

    /**
     * The point T of the tile's inset (walkInsideM inside its border) that minimises |A T| + |T B|: on the straight
     * line A–B when that line crosses the inset, else on the inset border (sampled every metre).
     */
    private double[] throughPoint(TileMath.Tile t, Portal a, Portal b) {
        double in = params.walkInsideM;
        double[] ne = xy(t.inMinLat, t.inMinLon, t.inMaxLat, t.inMaxLon);
        double[] pa = xy(t.inMinLat, t.inMinLon, a.lat, a.lon), pb = xy(t.inMinLat, t.inMinLon, b.lat, b.lon);
        double x0 = in, y0 = in, x1 = ne[0] - in, y1 = ne[1] - in;
        double bestS = Double.POSITIVE_INFINITY, bx = 0, by = 0;
        // the line A–B through the shrunk inset: its midpoint inside
        int n = (int) Math.ceil(Math.hypot(pb[0] - pa[0], pb[1] - pa[1]));
        double inside0 = -1, inside1 = -1;
        for (int k = 0; k <= n; k++) {
            double f = n == 0 ? 0 : (double) k / n, x = pa[0] + f * (pb[0] - pa[0]), y = pa[1] + f * (pb[1] - pa[1]);
            if (x >= x0 && x <= x1 && y >= y0 && y <= y1) {
                if (inside0 < 0) inside0 = f;
                inside1 = f;
            }
        }
        if (inside0 >= 0) {
            double f = (inside0 + inside1) / 2;
            bx = pa[0] + f * (pb[0] - pa[0]);
            by = pa[1] + f * (pb[1] - pa[1]);
        } else {
            double[][] corners = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
            for (int c = 0; c < 4; c++) {
                double[] p = corners[c], q = corners[(c + 1) % 4];
                int m = Math.max(1, (int) Math.ceil(Math.hypot(q[0] - p[0], q[1] - p[1])));
                for (int k = 0; k <= m; k++) {
                    double f = (double) k / m, x = p[0] + f * (q[0] - p[0]), y = p[1] + f * (q[1] - p[1]);
                    double s = Math.hypot(x - pa[0], y - pa[1]) + Math.hypot(x - pb[0], y - pb[1]);
                    if (s < bestS) {
                        bestS = s;
                        bx = x;
                        by = y;
                    }
                }
            }
        }
        return new double[]{t.inMinLat + by / TileMath.M_PER_DEG_LAT, t.inMinLon + bx / (TileMath.M_PER_DEG_LON_EQ * Math.cos(Math.toRadians(t.inMinLat)))};
    }

    /**
     * The plan's geometry: each run of followRoads legs routed as one plain-via GraphHopper request (exactly
     * {@link #assemble} for a plan with no direct legs), direct legs as straight lines. distanceM includes the walks;
     * walkM is the direct part.
     */
    public Route assemble(Plan plan, boolean passThrough) {
        return assemble(plan, passThrough, false);
    }

    /** The plan's geometry; asApp as in {@link #assemble(List, boolean, boolean)}. */
    public Route assemble(Plan plan, boolean passThrough, boolean asApp) {
        if (!plan.hasDirect()) return assemble(plan.vias, passThrough, asApp);
        Route r = new Route();
        int i = 0;
        while (i + 1 < plan.vias.size()) {
            if (DIRECT.equals(plan.segments.get(i))) {
                Via a = plan.vias.get(i), b = plan.vias.get(i + 1);
                if (r.coords.isEmpty()) r.coords.add(new double[]{a.lat, a.lon});
                double[] last = r.coords.get(r.coords.size() - 1);
                // the routed run ends at A's snap; the walk starts at A itself
                if (dist(last[0], last[1], a.lat, a.lon) > 0.01) r.coords.add(new double[]{a.lat, a.lon});
                r.coords.add(new double[]{b.lat, b.lon});
                double d = dist(a.lat, a.lon, b.lat, b.lon);
                r.distanceM += d;
                r.walkM += d;
                i++;
                continue;
            }
            int j = i;
            while (j + 1 < plan.vias.size() && FOLLOW_ROADS.equals(plan.segments.get(j))) j++;
            Route part = assemble(plan.vias.subList(i, j + 1), passThrough, asApp);
            if (part.error != null) {
                r.error = part.error;
                return r;
            }
            for (int k = 0; k < part.coords.size(); k++) {
                double[] c = part.coords.get(k);
                if (k == 0 && !r.coords.isEmpty()) {
                    double[] last = r.coords.get(r.coords.size() - 1);
                    if (dist(last[0], last[1], c[0], c[1]) < 0.01) continue;
                }
                r.coords.add(c);
            }
            r.distanceM += part.distanceM;
            r.weight += part.weight;
            i = j;
        }
        return r;
    }

    /** Polyline (lat, lon) of whole directed edges, e.g. a search path from {@link EdgeDijkstra#pathKeys}. */
    public List<double[]> geometry(List<Integer> keys) {
        List<double[]> out = new ArrayList<>();
        for (int k : keys) {
            PointList pl = graph.getEdgeIteratorStateForKey(k).fetchWayGeometry(FetchMode.ALL);
            for (int i = out.isEmpty() ? 0 : 1; i < pl.size(); i++) out.add(new double[]{pl.getLat(i), pl.getLon(i)});
        }
        return out;
    }

    /** Development check only: does the geometry enter each tile's inset square? */
    public static boolean fetched(TileMath.Tile t, List<double[]> coords) {
        for (int i = 0; i + 1 < coords.size(); i++) {
            double[] a = coords.get(i), b = coords.get(i + 1);
            if (t.segmentEntersInset(a[0], a[1], b[0], b[1])) return true;
        }
        return coords.size() == 1 && t.insetContains(coords.get(0)[0], coords.get(0)[1]);
    }

    /** Development estimate: heading reversals ≥ 150° between consecutive ≥ 3 m segments. */
    public static int uTurns(List<double[]> coords) {
        int n = 0;
        double prev = Double.NaN;
        double[] last = null;
        for (double[] c : coords) {
            if (last != null && dist(last[0], last[1], c[0], c[1]) >= 3) {
                double b = bearing(last[0], last[1], c[0], c[1]);
                if (!Double.isNaN(prev)) {
                    double d = Math.abs(b - prev) % 360;
                    if (d > 180) d = 360 - d;
                    if (d >= 150) n++;
                }
                prev = b;
                last = c;
            } else if (last == null) last = c;
        }
        return n;
    }

    // ------------------------------------------------------------------ geometry helpers

    static double[] cumulative(PointList pl) {
        double[] cum = new double[pl.size()];
        for (int i = 1; i < pl.size(); i++)
            cum[i] = cum[i - 1] + dist(pl.getLat(i - 1), pl.getLon(i - 1), pl.getLat(i), pl.getLon(i));
        return cum;
    }

    /** lat, lon, heading at distance s along the polyline. */
    static double[] pointAt(PointList pl, double[] cum, double s) {
        for (int i = 0; i + 1 < pl.size(); i++) {
            if (s <= cum[i + 1] || i + 2 == pl.size()) {
                double len = cum[i + 1] - cum[i];
                double f = len > 0 ? Math.max(0, Math.min(1, (s - cum[i]) / len)) : 0;
                double la = pl.getLat(i) + f * (pl.getLat(i + 1) - pl.getLat(i));
                double lo = pl.getLon(i) + f * (pl.getLon(i + 1) - pl.getLon(i));
                return new double[]{la, lo, bearing(pl.getLat(i), pl.getLon(i), pl.getLat(i + 1), pl.getLon(i + 1))};
            }
        }
        return new double[]{pl.getLat(0), pl.getLon(0), 0};
    }

    /** Along-distance and heading of the closest point of the polyline to (lat, lon). */
    static double[] project(PointList pl, double lat, double lon) {
        double[] cum = cumulative(pl);
        double best = Double.MAX_VALUE, bestS = 0, bestH = 0;
        double mPerLon = TileMath.M_PER_DEG_LON_EQ * Math.cos(Math.toRadians(lat));
        for (int i = 0; i + 1 < pl.size(); i++) {
            double ax = (pl.getLon(i) - lon) * mPerLon, ay = (pl.getLat(i) - lat) * TileMath.M_PER_DEG_LAT;
            double bx = (pl.getLon(i + 1) - lon) * mPerLon, by = (pl.getLat(i + 1) - lat) * TileMath.M_PER_DEG_LAT;
            double dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
            double f = l2 > 0 ? Math.max(0, Math.min(1, -(ax * dx + ay * dy) / l2)) : 0;
            double px = ax + f * dx, py = ay + f * dy, d = px * px + py * py;
            if (d < best) {
                best = d;
                bestS = cum[i] + f * (cum[i + 1] - cum[i]);
                bestH = bearing(pl.getLat(i), pl.getLon(i), pl.getLat(i + 1), pl.getLon(i + 1));
            }
        }
        return new double[]{bestS, bestH};
    }

    public static double dist(double lat1, double lon1, double lat2, double lon2) {
        return DistanceCalcEarth.DIST_EARTH.calcDist(lat1, lon1, lat2, lon2);
    }

    public static double bearing(double lat1, double lon1, double lat2, double lon2) {
        double f1 = Math.toRadians(lat1), f2 = Math.toRadians(lat2), dl = Math.toRadians(lon2 - lon1);
        double y = Math.sin(dl) * Math.cos(f2);
        double x = Math.cos(f1) * Math.sin(f2) - Math.sin(f1) * Math.cos(f2) * Math.cos(dl);
        double b = Math.toDegrees(Math.atan2(y, x));
        b = (b + 360) % 360;
        return b >= 360 ? 0 : b;
    }
}
