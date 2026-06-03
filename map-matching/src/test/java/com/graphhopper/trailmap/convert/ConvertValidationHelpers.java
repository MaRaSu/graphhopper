package com.graphhopper.trailmap.convert;

import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.GraphHopper;
import com.graphhopper.ResponsePath;
import com.graphhopper.matching.Observation;
import com.graphhopper.util.AngleCalc;
import com.graphhopper.util.CustomModel;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.PointList;
import com.graphhopper.util.shapes.GHPoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Helpers shared between {@code TrackConvertDiagnosticTest} and
 * {@code TrackConvertValidationTest}.
 *
 * <p>Three jobs:
 * <ol>
 *   <li><b>Realize</b> the converted route's polyline by re-routing each {@code followRoads}
 *       segment locally (same call the client will make) and concatenating with the
 *       {@code coordinates} segments' raw geometry. The result is the polyline a client
 *       would render.</li>
 *   <li><b>Measure</b> the realized polyline's faithfulness to the input GPX: for each
 *       GPX point, distance to nearest point on the realized polyline. Reports max + mean.</li>
 *   <li><b>Summarize</b> the response in compact form for inclusion in assertion failure
 *       messages — the validation test is silent on success but self-diagnosing on failure.</li>
 * </ol>
 */
public final class ConvertValidationHelpers {

    private static final DistanceCalcEarth DIST = DistanceCalcEarth.DIST_EARTH;

    private ConvertValidationHelpers() {}

    // ----------------------------------------------------------------------
    // Realize polyline
    // ----------------------------------------------------------------------

    /**
     * Materialize the full realized polyline of a converted route. For each
     * {@code followRoads} segment, calls {@code gh.route(start, end)} to obtain the
     * geometry the client would render. For each {@code coordinates} segment, uses
     * the inline {@code track_coordinates} list directly.
     *
     * <p>Consecutive segments share a boundary waypoint by id (per API contract), so we
     * de-duplicate the boundary point when concatenating to avoid an immediate-repeat
     * vertex in the resulting polyline.
     */
    public static PointList realizePolyline(ConvertTrackResponse rsp,
                                            GraphHopper gh,
                                            String profile,
                                            CustomModel customModel) {
        // Lookup: id → coordinates
        java.util.Map<String, ConvertTrackResponse.Coordinates> wpMap = new java.util.HashMap<>();
        for (ConvertTrackResponse.Waypoint w : rsp.getWaypoints()) {
            wpMap.put(w.getId(), w.getCoordinates());
        }

        PointList out = new PointList(256, false);

        // Heading chain — mirror the client receiving the server's initialHeading: each
        // followRoads leg after a previous followRoads leg is rendered with a start heading =
        // the previous leg's exit bearing + heading_penalty=60; reset to null across a
        // coordinates segment (and at the very start). This makes the faithfulness check
        // measure the path the client actually draws when initialHeading is honoured.
        Double currentHeading = null;
        for (ConvertTrackResponse.Segment seg : rsp.getSegments()) {
            PointList segPts;
            if (ConvertTrackResponse.Segment.TYPE_FOLLOW_ROADS.equals(seg.getType())) {
                ConvertTrackResponse.Coordinates s = wpMap.get(seg.getStart());
                ConvertTrackResponse.Coordinates e = wpMap.get(seg.getEnd());
                GHRequest req = new GHRequest(s.getLat(), s.getLng(), e.getLat(), e.getLng());
                req.setProfile(profile);
                if (customModel != null) req.setCustomModel(customModel);
                req.putHint("instructions", false);
                req.putHint("calc_points", true);
                if (currentHeading != null && !currentHeading.isNaN()) {
                    req.setHeadings(Arrays.asList(currentHeading, Double.NaN));
                    req.putHint("heading_penalty", 60);
                }
                GHResponse rr = gh.route(req);
                if (rr.hasErrors()) {
                    throw new IllegalStateException("Re-routing followRoads segment failed: "
                            + rr.getErrors());
                }
                ResponsePath path = rr.getBest();
                segPts = path.getPoints();
                // Chain this leg's exit bearing into the next followRoads leg.
                int np = segPts.size();
                currentHeading = (np >= 2)
                        ? AngleCalc.ANGLE_CALC.calcAzimuth(
                                segPts.getLat(np - 2), segPts.getLon(np - 2),
                                segPts.getLat(np - 1), segPts.getLon(np - 1))
                        : null;
            } else {
                // coordinates segment
                List<ConvertTrackResponse.Coordinates> coords = seg.getTrackCoordinates();
                segPts = new PointList(coords.size(), false);
                for (ConvertTrackResponse.Coordinates c : coords) {
                    segPts.add(c.getLat(), c.getLng());
                }
                currentHeading = null; // chain breaks across a coordinates segment
            }
            appendDedupingBoundary(out, segPts);
        }
        return out;
    }

    private static void appendDedupingBoundary(PointList target, PointList src) {
        int startIdx = 0;
        if (target.size() > 0 && src.size() > 0) {
            double lastLat = target.getLat(target.size() - 1);
            double lastLon = target.getLon(target.size() - 1);
            double firstLat = src.getLat(0);
            double firstLon = src.getLon(0);
            if (DIST.calcDist(lastLat, lastLon, firstLat, firstLon) < 1.0) {
                startIdx = 1;
            }
        }
        for (int i = startIdx; i < src.size(); i++) {
            target.add(src.getLat(i), src.getLon(i));
        }
    }

    // ----------------------------------------------------------------------
    // Deviation metric
    // ----------------------------------------------------------------------

    /**
     * For each input GPX point, computes the minimum distance to the realized polyline
     * (treating the polyline as a sequence of line segments — distance to nearest segment,
     * not nearest vertex). Returns an array parallel to {@code gpx}.
     */
    public static double[] perPointDeviations(List<Observation> gpx, PointList realized) {
        double[] out = new double[gpx.size()];
        for (int i = 0; i < gpx.size(); i++) {
            GHPoint p = gpx.get(i).getPoint();
            out[i] = pointToPolylineDistance(p.lat, p.lon, realized);
        }
        return out;
    }

    /**
     * REVERSE deviation: for each realized-route point, the minimum distance to the GPX
     * polyline (consecutive GPX points joined as segments). Catches the case the forward
     * metric is blind to — the realized route ADDING an excursion/loop the GPX never took
     * (a straight GPX routed onto a looping OSM way). The forward {@link #perPointDeviations}
     * stays small there because every GPX point still has a nearby realized point; this
     * reverse metric spikes at the far side of the added loop. Returns an array parallel to
     * {@code realized}.
     */
    public static double[] realizedToGpxDeviations(PointList realized, List<Observation> gpx) {
        PointList gpxLine = new PointList(gpx.size(), false);
        for (Observation o : gpx) gpxLine.add(o.getPoint().lat, o.getPoint().lon);
        double[] out = new double[realized.size()];
        for (int i = 0; i < realized.size(); i++) {
            out[i] = pointToPolylineDistance(realized.getLat(i), realized.getLon(i), gpxLine);
        }
        return out;
    }

    public static double maxDeviation(double[] devs) {
        double m = 0;
        for (double d : devs) if (d > m) m = d;
        return m;
    }

    public static double meanDeviation(double[] devs) {
        if (devs.length == 0) return 0;
        double s = 0;
        for (double d : devs) s += d;
        return s / devs.length;
    }

    /** Index of worst-deviating GPX point (for failure messages). */
    public static int worstDeviationIndex(double[] devs) {
        int best = 0;
        double m = -1;
        for (int i = 0; i < devs.length; i++) {
            if (devs[i] > m) { m = devs[i]; best = i; }
        }
        return best;
    }

    private static double pointToPolylineDistance(double pLat, double pLon, PointList line) {
        if (line.size() == 0) return Double.POSITIVE_INFINITY;
        if (line.size() == 1) {
            return DIST.calcDist(pLat, pLon, line.getLat(0), line.getLon(0));
        }
        double min = Double.POSITIVE_INFINITY;
        for (int i = 0; i + 1 < line.size(); i++) {
            double d = pointToSegmentDistance(
                    pLat, pLon,
                    line.getLat(i), line.getLon(i),
                    line.getLat(i + 1), line.getLon(i + 1));
            if (d < min) min = d;
        }
        return min;
    }

    /**
     * Perpendicular point-to-segment distance in meters (clamped to segment endpoints).
     * Uses a simple equirectangular projection — accurate enough at the scales we test
     * (sub-100km tracks, sub-meter precision irrelevant).
     */
    private static double pointToSegmentDistance(double pLat, double pLon,
                                                 double aLat, double aLon,
                                                 double bLat, double bLon) {
        // Project to local meters using mean latitude
        double meanLatRad = Math.toRadians((aLat + bLat) * 0.5);
        double mPerDegLat = 111_320.0;
        double mPerDegLon = 111_320.0 * Math.cos(meanLatRad);

        double ax = aLon * mPerDegLon, ay = aLat * mPerDegLat;
        double bx = bLon * mPerDegLon, by = bLat * mPerDegLat;
        double px = pLon * mPerDegLon, py = pLat * mPerDegLat;

        double dx = bx - ax, dy = by - ay;
        double segLenSq = dx * dx + dy * dy;
        if (segLenSq == 0) {
            // Degenerate segment → fall back to haversine to endpoint
            return DIST.calcDist(pLat, pLon, aLat, aLon);
        }
        double t = ((px - ax) * dx + (py - ay) * dy) / segLenSq;
        if (t < 0) t = 0;
        else if (t > 1) t = 1;
        double cx = ax + t * dx;
        double cy = ay + t * dy;
        double ex = px - cx, ey = py - cy;
        return Math.sqrt(ex * ex + ey * ey);
    }

    // ----------------------------------------------------------------------
    // Summaries / diffs (for failure messages)
    // ----------------------------------------------------------------------

    /** Compact one-line-per-segment dump. */
    public static String summarizeSegments(ConvertTrackResponse rsp) {
        StringBuilder sb = new StringBuilder();
        sb.append(String.format("Total: %d waypoints, %d segments, %.1fm%n",
                rsp.getWaypoints().size(), rsp.getSegments().size(),
                rsp.getStats() != null ? rsp.getStats().totalDistanceM : 0));
        for (int i = 0; i < rsp.getSegments().size(); i++) {
            ConvertTrackResponse.Segment s = rsp.getSegments().get(i);
            int coords = s.getTrackCoordinates() == null ? 0 : s.getTrackCoordinates().size();
            sb.append(String.format("  [%2d] %-12s %7.1fm  start=%s end=%s%s%n",
                    i, s.getType(), s.getDistanceM(),
                    abbrev(s.getStart()), abbrev(s.getEnd()),
                    coords > 0 ? "  (track_coordinates=" + coords + ")" : ""));
        }
        return sb.toString();
    }

    private static String abbrev(String id) {
        if (id == null || id.length() <= 8) return id;
        return id.substring(0, 8);
    }

    /**
     * Produce a paste-ready fixture entry from a converted response, for capturing
     * baselines. The caller fills in {@code name}, {@code gpx}, {@code profile}, and
     * deviation thresholds; this method emits the {@code expected_segments} and
     * {@code total_distance_m} blocks with reasonable default tolerances.
     */
    public static String emitFixtureSegmentsBlock(ConvertTrackResponse rsp,
                                                  double perSegmentTolM,
                                                  double totalTolM) {
        StringBuilder sb = new StringBuilder();
        sb.append("  \"expected_segments\": [\n");
        List<ConvertTrackResponse.Segment> segs = rsp.getSegments();
        for (int i = 0; i < segs.size(); i++) {
            ConvertTrackResponse.Segment s = segs.get(i);
            sb.append(String.format(
                    "    { \"type\": \"%s\", \"distance_m\": %.2f, \"tolerance_m\": %.1f }%s%n",
                    s.getType(), s.getDistanceM(), perSegmentTolM,
                    i + 1 < segs.size() ? "," : ""));
        }
        sb.append("  ],\n");
        double total = rsp.getStats() != null ? rsp.getStats().totalDistanceM : 0;
        sb.append(String.format(
                "  \"total_distance_m\": { \"value\": %.2f, \"tolerance_m\": %.1f }%n",
                total, totalTolM));
        return sb.toString();
    }

    /**
     * Build a readable diff of expected vs actual segment sequences.
     * Used in validation test failure messages.
     */
    public static String segmentSequenceDiff(List<ExpectedSegment> expected,
                                             List<ConvertTrackResponse.Segment> actual) {
        StringBuilder sb = new StringBuilder();
        sb.append("Expected segments (").append(expected.size())
                .append(") vs actual (").append(actual.size()).append("):\n");
        int n = Math.max(expected.size(), actual.size());
        for (int i = 0; i < n; i++) {
            ExpectedSegment e = i < expected.size() ? expected.get(i) : null;
            ConvertTrackResponse.Segment a = i < actual.size() ? actual.get(i) : null;
            String marker;
            if (e == null) marker = "  [extra ]";
            else if (a == null) marker = "  [missing]";
            else if (!e.type.equals(a.getType())) marker = "  [TYPE   ]";
            else if (Math.abs(e.distanceM - a.getDistanceM()) > e.toleranceM) marker = "  [DIST   ]";
            else marker = "  [   ok  ]";
            String expStr = e == null ? "                                  "
                    : String.format("%-12s %7.1fm ±%.1f", e.type, e.distanceM, e.toleranceM);
            String actStr = a == null ? "                          "
                    : String.format("%-12s %7.1fm", a.getType(), a.getDistanceM());
            sb.append(String.format("%s [%2d] expected: %s   actual: %s%n",
                    marker, i, expStr, actStr));
        }
        return sb.toString();
    }

    // ----------------------------------------------------------------------
    // Fixture types
    // ----------------------------------------------------------------------

    /** One expected segment in a fixture entry. */
    public static final class ExpectedSegment {
        public String type;
        public double distanceM;
        public double toleranceM;
        public ExpectedSegment() {}
        public ExpectedSegment(String type, double distanceM, double toleranceM) {
            this.type = type;
            this.distanceM = distanceM;
            this.toleranceM = toleranceM;
        }
    }

    /** One numeric value with a tolerance. */
    public static final class ValueWithTol {
        public double value;
        public double toleranceM;
    }

    /** Max-allowed cap on a numeric metric. */
    public static final class MaxCap {
        public double max;
    }

    /** A single test case in the fixture file. */
    public static final class TestCase {
        public String name;
        public String gpx;
        public String profile;
        /** Optional separate profile for map matching. When null, defaults to {@link #profile}. */
        public String matchingProfile;
        public java.util.Map<String, Object> requestOverrides;
        public List<ExpectedSegment> expectedSegments;
        public ValueWithTol totalDistanceM;
        public MaxCap maxDeviationFromInputGpxM;
        public MaxCap meanDeviationFromInputGpxM;
        public String notes;
    }

    /** Top-level fixture document. */
    public static final class TestCases {
        public List<TestCase> cases = new ArrayList<>();
    }
}
