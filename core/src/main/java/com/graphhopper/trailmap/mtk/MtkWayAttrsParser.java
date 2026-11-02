/*
 * Trailmap - MTK overlay import unit (alternative A, docs/mtk-import-alternatives.md).
 *
 * Runs after the Trailmap OSM parsers on the same edge and overwrites their values only where the
 * owner's per-case selector says OSM is ambiguous ("OSM wins"; way-network cases/track-untagged.md and
 * cases/minor-road-surface-unknown.md, "Enrichment rule (decided)", 2026-11-02):
 *
 * Tracks (highway=track, not area=yes, none of surface/smoothness/tracktype/mtb:scale):
 *   ajotie   -> gravel_scale 1, mtb_scale 0, issue_unknown_track false
 *   ajopolku -> gravel_scale 2, mtb_scale 0, issue_unknown_track false
 *   talvitie, polku, no class -> unchanged (a "-tie" name follows the MTK class, so it needs no rule here)
 * Roads (secondary/tertiary (+links), unclassified, residential, service; same exclusions):
 *   MTK paved/unpaved counts as if surface=asphalt / surface=unpaved were tagged: the existing gravel_scale
 *   and predicted_surface parsers are re-run on a copy of the way with that tag.
 * Not applied to ways with an obstacle or trail_visibility tag (OSM hazard evidence wins), nor to a way
 * whose node-id hash differs from the sidecar's (the way was edited after the match), nor to a way whose summed
 * 2D edge length differs from way_len_m by more than max(5 m, 1 %) (nodes moved after the match: stale_length).
 * A way's edges are therefore held until the way ends and written in finishWay.
 *
 * Known limits (accepted by lead_routing, 2026-11-02; both fall back to "no MTK value", like a stale way):
 * - The length is GraphHopper's summed edge length, measured on simplified point lists. A way that doubles back
 *   on itself can lose a collinear spike node and come out short: on fi-2026-09-27, 1 way (1441800198, 6.6 m)
 *   is skipped as stale_length although its OSM nodes match way_len_m.
 * - The import's last way is written in postImportOSM, after OSMReader has copied the edges of turn-restriction
 *   via ways, so if that way is such a via way, the copied edge pair keeps today's values.
 *
 * Pieces to edges: GraphHopper hands the parsers one way's edges in way order. The unit sums each edge's 2D
 * length (its point_list, lat/lon only, the metric of way_len_m; never the 3D edge_distance) along the way and
 * takes the piece covering the edge's midpoint fraction (midpoint / way_len_m).
 *
 * The unit has no encoded value of its own, so the graph format is unchanged; without a sidecar it writes
 * nothing.
 */
package com.graphhopper.trailmap.mtk;

import com.carrotsearch.hppc.IntArrayList;
import com.carrotsearch.hppc.LongArrayList;
import com.carrotsearch.hppc.cursors.LongCursor;
import com.graphhopper.reader.ReaderWay;
import com.graphhopper.routing.ev.*;
import com.graphhopper.routing.util.parsers.TagParser;
import com.graphhopper.storage.IntsRef;
import com.graphhopper.trailmap.shared.*;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.PMap;
import com.graphhopper.util.PointList;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;

public class MtkWayAttrsParser implements TagParser {
    private static final Logger LOGGER = LoggerFactory.getLogger(MtkWayAttrsParser.class);

    public static final String KEY = "mtk_way_attrs";

    /** Encoded values this unit overwrites; declared as required import units so their parsers run first. */
    public static final String[] REQUIRED = {GravelScale.KEY, GravelScaleNum.KEY, GravelBasePriority.KEY,
            PredictedSurface.KEY, MtbScale.KEY, MtbScaleNum.KEY, MtbBasePriority.KEY, RouteIssues.KEY_BIKING_BLOCKED};

    static final Set<String> ROAD_CLASSES = Set.of("secondary", "secondary_link", "tertiary", "tertiary_link",
            "unclassified", "residential", "service");
    static final String[] AMBIGUITY_EXCLUDERS = {"surface", "smoothness", "tracktype", "mtb:scale"};
    static final String[] HAZARD_TAGS = {"obstacle", "trail_visibility"};

    /** A gap between two consecutive edges of a way (nodes missing in an extract) larger than this is added. */
    private static final double GAP_M = 0.5;

    enum Case {NONE, TRACK, ROAD}

    /** Length tolerance of the stale-length check: max(5 m, 1 % of way_len_m) (architect_hq, 2026-11-02). */
    static final double LEN_TOL_M = 5.0, LEN_TOL_FRAC = 0.01;

    private final MtkSidecar sidecar;
    private final double minClassShare, minPavedShare;
    private final boolean skipHazard;
    private final double lenTolM, lenTolFrac;

    private final EnumEncodedValue<GravelScale> gravelScaleEnc;
    private final DecimalEncodedValue gravelScaleNumEnc, gravelBasePriorityEnc;
    private final EnumEncodedValue<MtbScale> mtbScaleEnc;
    private final DecimalEncodedValue mtbScaleNumEnc, mtbBasePriorityEnc;
    private final BooleanEncodedValue unknownTrackEnc;
    // today's parsers, re-run on a copy of the way for the road case ("as if surface were tagged")
    private final TagParser[] roadParsers;

    // per-way state; GraphHopper parses one way's edges consecutively and in way order
    private long curWayId = Long.MIN_VALUE;
    private int curWay = -1;        // sidecar way index; -1 = nothing to apply on this way
    private Case curCase = Case.NONE;
    private double cum;              // summed 2D edge length before the current edge
    private double prevLat = Double.NaN, prevLon = Double.NaN;
    private final BitSet visited = new BitSet();
    // the current way's edges, held until its summed length is known (applied or dropped in finishWay)
    private final IntArrayList pendEdge = new IntArrayList(), pendPiece = new IntArrayList();
    private final List<ReaderWay> pendTags = new ArrayList<>();
    private final List<IntsRef> pendRel = new ArrayList<>();
    private EdgeIntAccess pendEia;

    final Stats stats = new Stats();

    public MtkWayAttrsParser(MtkSidecar sidecar, EncodedValueLookup lookup, PMap props) {
        this.sidecar = sidecar;
        this.minClassShare = props.getDouble("min_class_share", 0);
        this.minPavedShare = props.getDouble("min_paved_share", 0);
        this.skipHazard = props.getBool("skip_hazard", true);
        this.lenTolM = props.getDouble("len_tol_m", LEN_TOL_M);
        this.lenTolFrac = props.getDouble("len_tol_frac", LEN_TOL_FRAC);

        gravelScaleEnc = lookup.getEnumEncodedValue(GravelScale.KEY, GravelScale.class);
        gravelScaleNumEnc = lookup.getDecimalEncodedValue(GravelScaleNum.KEY);
        gravelBasePriorityEnc = lookup.getDecimalEncodedValue(GravelBasePriority.KEY);
        mtbScaleEnc = lookup.getEnumEncodedValue(MtbScale.KEY, MtbScale.class);
        mtbScaleNumEnc = lookup.getDecimalEncodedValue(MtbScaleNum.KEY);
        mtbBasePriorityEnc = lookup.getDecimalEncodedValue(MtbBasePriority.KEY);
        unknownTrackEnc = lookup.getBooleanEncodedValue(RouteIssues.KEY_UNKNOWN_TRACK);
        roadParsers = new TagParser[]{
                new GravelScaleParser(gravelScaleEnc),
                new GravelScaleNumParser(gravelScaleNumEnc),
                new GravelBasePriorityParser(gravelBasePriorityEnc),
                new PredictedSurfaceParser(lookup.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class))
        };
    }

    /** The import unit's parser factory: loads the sidecar named by the {@code file} prop. */
    public static MtkWayAttrsParser create(EncodedValueLookup lookup, PMap props) {
        MtkSidecar sidecar = MtkSidecar.load(props.getString("file", ""));
        MtkWayAttrsParser p = new MtkWayAttrsParser(sidecar, lookup, props);
        LOGGER.info("MTK overlay: {} (min_class_share={}, min_paved_share={}, skip_hazard={})",
                sidecar.isEmpty() ? "inactive, no MTK values" : "active", p.minClassShare, p.minPavedShare, p.skipHazard);
        return p;
    }

    @Override
    public void handleWayTags(int edgeId, EdgeIntAccess edgeIntAccess, ReaderWay way, IntsRef relationFlags) {
        if (sidecar.isEmpty())
            return;
        if (way.getId() != curWayId)
            startWay(way);

        // Position the edge in 2D (lat/lon only), the metric of the sidecar's way_len_m and fractions; GraphHopper's
        // edge_distance is 3D when elevation is on. edge_distance is only the fallback without a point list.
        PointList pl = way.getTag("point_list", null);
        double distance = pl != null && pl.size() > 1
                ? DistanceCalcEarth.calcDistance(pl, false) : way.getTag("edge_distance", 0.0);
        if (pl != null && !pl.isEmpty()) {
            if (!Double.isNaN(prevLat)) {
                double gap = DistanceCalcEarth.DIST_EARTH.calcDist(prevLat, prevLon, pl.getLat(0), pl.getLon(0));
                if (gap > GAP_M)
                    cum += gap;
            }
            prevLat = pl.getLat(pl.size() - 1);
            prevLon = pl.getLon(pl.size() - 1);
        }
        double mid = cum + distance / 2;
        cum += distance;
        if (curWay < 0)
            return;

        double frac = Math.min(1.0, Math.max(0.0, mid / sidecar.getWayLength(curWay)));
        int piece = sidecar.findPiece(curWay, frac);
        // held until the way ends: whether its summed length matches way_len_m is known only then
        pendEdge.add(edgeId);
        pendPiece.add(piece);
        pendEia = edgeIntAccess;
        if (curCase == Case.ROAD) {
            // the tags as they are for this edge (GraphHopper changes artificial tags per edge)
            ReaderWay copy = new ReaderWay(way.getId());
            copy.setTags(way.getTags());
            pendTags.add(copy);
            pendRel.add(relationFlags);
        }
    }

    /** True when the summed 2D edge length differs from way_len_m by more than max(tolM, tolFrac × way_len_m). */
    static boolean isLengthStale(double summedM, double wayLenM, double tolM, double tolFrac) {
        return Math.abs(summedM - wayLenM) > Math.max(tolM, tolFrac * wayLenM);
    }

    private void startWay(ReaderWay way) {
        finishWay();
        curWayId = way.getId();
        curWay = -1;
        curCase = Case.NONE;
        cum = 0;
        prevLat = prevLon = Double.NaN;

        int idx = sidecar.findWay(curWayId);
        if (idx < 0)
            return;
        if (visited.get(idx)) {
            // the way's edges came in two runs; the offsets of the second run are unknown
            stats.waysRevisited++;
            return;
        }
        visited.set(idx);
        stats.sidecarWaysSeen++;
        Case c = selectCase(way);
        if (c == Case.NONE) {
            stats.waysOsmWins++;
            return;
        }
        if (skipHazard && hasHazard(way)) {
            stats.waysHazard++;
            return;
        }
        if (nodeIdsHash(way.getNodes()) != sidecar.getHash(idx)) {
            stats.waysStaleHash++;
            return;
        }
        curWay = idx;
        curCase = c;
    }

    /** Ends the current way: applies its held edges, or drops them all when its length is stale. */
    void finishWay() {
        if (curWay >= 0) {
            double wayLen = sidecar.getWayLength(curWay);
            if (isLengthStale(cum, wayLen, lenTolM, lenTolFrac)) {
                stats.waysStaleLength++;
            } else {
                if (curCase == Case.TRACK) stats.waysTrack++;
                else stats.waysRoad++;
                double ratio = cum / wayLen;
                stats.lenRatioMin = Math.min(stats.lenRatioMin, ratio);
                stats.lenRatioMax = Math.max(stats.lenRatioMax, ratio);
                if (Math.abs(ratio - 1) > 0.01)
                    stats.lenRatioOff1pct++;
                stats.lenRatioCount++;
                int road = 0;
                for (int i = 0; i < pendEdge.size(); i++) {
                    int edgeId = pendEdge.get(i), piece = pendPiece.get(i);
                    if (piece < 0)
                        stats.edgesNoPiece++;
                    else if (curCase == Case.TRACK)
                        applyTrack(edgeId, pendEia, piece);
                    else
                        applyRoad(edgeId, pendEia, pendTags.get(road), pendRel.get(road), piece);
                    if (curCase == Case.ROAD)
                        road++;
                }
            }
        }
        curWay = -1;
        pendEdge.clear();
        pendPiece.clear();
        pendTags.clear();
        pendRel.clear();
        pendEia = null;
    }

    private void applyTrack(int edgeId, EdgeIntAccess eia, int piece) {
        byte cls = sidecar.getMtkClass(piece);
        GravelScale gravel;
        if (cls == MtkSidecar.CLASS_AJOTIE)
            gravel = GravelScale.ONE;
        else if (cls == MtkSidecar.CLASS_AJOPOLKU)
            gravel = GravelScale.TWO;
        else {
            stats.edgesTrackUnchanged++;
            return;
        }
        if (sidecar.getClassShare(piece) < minClassShare) {
            stats.edgesBelowShare++;
            return;
        }
        gravelScaleEnc.setEnum(false, edgeId, eia, gravel);
        gravelScaleNumEnc.setDecimal(false, edgeId, eia, GravelScaleNum.getNumericValue(gravel));
        gravelBasePriorityEnc.setDecimal(false, edgeId, eia, GravelBasePriority.getPriority(gravel));
        mtbScaleEnc.setEnum(false, edgeId, eia, MtbScale.ZERO);
        mtbScaleNumEnc.setDecimal(false, edgeId, eia, MtbScaleNum.getNumericValue(MtbScale.ZERO));
        mtbBasePriorityEnc.setDecimal(false, edgeId, eia, MtbBasePriority.getPriority(MtbScale.ZERO));
        unknownTrackEnc.setBool(false, edgeId, eia, false);
        if (gravel == GravelScale.ONE) stats.edgesAjotie++;
        else stats.edgesAjopolku++;
    }

    private void applyRoad(int edgeId, EdgeIntAccess eia, ReaderWay way, IntsRef relationFlags, int piece) {
        byte paved = sidecar.getMtkPaved(piece);
        String surface;
        if (paved == MtkSidecar.PAVED_PAVED)
            surface = "asphalt";
        else if (paved == MtkSidecar.PAVED_UNPAVED)
            surface = "unpaved";
        else {
            stats.edgesRoadUnchanged++;
            return;
        }
        if (sidecar.getPavedShare(piece) < minPavedShare) {
            stats.edgesBelowShare++;
            return;
        }
        ReaderWay asIfTagged = new ReaderWay(way.getId());
        asIfTagged.setTags(way.getTags());
        asIfTagged.setTag("surface", surface);
        for (TagParser p : roadParsers)
            p.handleWayTags(edgeId, eia, asIfTagged, relationFlags);
        if (paved == MtkSidecar.PAVED_PAVED) stats.edgesPaved++;
        else stats.edgesUnpaved++;
    }

    static Case selectCase(ReaderWay way) {
        if ("yes".equals(way.getTag("area")))
            return Case.NONE;
        for (String t : AMBIGUITY_EXCLUDERS)
            if (way.hasTag(t))
                return Case.NONE;
        String highway = way.getTag("highway");
        if ("track".equals(highway))
            return Case.TRACK;
        if (highway != null && ROAD_CLASSES.contains(highway))
            return Case.ROAD;
        return Case.NONE;
    }

    static boolean hasHazard(ReaderWay way) {
        for (String t : HAZARD_TAGS)
            if (way.hasTag(t))
                return true;
        return false;
    }

    /** The first 16 hex chars of the SHA-1 of the node ids joined with ',' in way order, as an unsigned long. */
    public static long nodeIdsHash(LongArrayList nodes) {
        StringBuilder sb = new StringBuilder(nodes.size() * 11);
        for (LongCursor c : nodes) {
            if (sb.length() > 0)
                sb.append(',');
            sb.append(c.value);
        }
        byte[] d;
        try {
            d = MessageDigest.getInstance("SHA-1").digest(sb.toString().getBytes(StandardCharsets.US_ASCII));
        } catch (NoSuchAlgorithmException e) {
            throw new IllegalStateException(e);
        }
        long h = 0;
        for (int i = 0; i < 8; i++)
            h = (h << 8) | (d[i] & 0xFF);
        return h;
    }

    /** Logs what the overlay did; called once after the import. */
    public void logSummary() {
        finishWay();
        curWay = -1;
        if (sidecar.isEmpty()) {
            LOGGER.info("MTK overlay: inactive ({})", sidecar.getStatus());
            return;
        }
        LOGGER.info("MTK overlay summary: {}", stats);
    }

    static final class Stats {
        long sidecarWaysSeen, waysTrack, waysRoad, waysOsmWins, waysHazard, waysStaleHash, waysStaleLength, waysRevisited;
        long edgesAjotie, edgesAjopolku, edgesTrackUnchanged, edgesPaved, edgesUnpaved, edgesRoadUnchanged,
                edgesNoPiece, edgesBelowShare;
        long lenRatioCount, lenRatioOff1pct;
        double lenRatioMin = Double.POSITIVE_INFINITY, lenRatioMax = Double.NEGATIVE_INFINITY;

        @Override
        public String toString() {
            double pct = sidecarWaysSeen == 0 ? 0 : 100.0 / sidecarWaysSeen;
            return String.format(Locale.ROOT, "sidecar ways seen %d: applied track %d, road %d; skipped osm-wins %d, hazard %d,"
                            + " stale_hash %d (%.3f%%), stale_length %d (%.3f%%), revisited %d"
                            + " | edges set: track ajotie %d, ajopolku %d, road paved %d, unpaved %d; left: track class %d, road no paving %d,"
                            + " no piece %d, below share %d | summed 2D edge length / way_len_m over %d applied ways: min %.4f, max %.4f, %d off by > 1%%",
                    sidecarWaysSeen, waysTrack, waysRoad, waysOsmWins, waysHazard, waysStaleHash, waysStaleHash * pct,
                    waysStaleLength, waysStaleLength * pct, waysRevisited,
                    edgesAjotie, edgesAjopolku, edgesPaved, edgesUnpaved, edgesTrackUnchanged, edgesRoadUnchanged,
                    edgesNoPiece, edgesBelowShare, lenRatioCount,
                    lenRatioCount == 0 ? Double.NaN : lenRatioMin, lenRatioCount == 0 ? Double.NaN : lenRatioMax, lenRatioOff1pct);
        }
    }
}
