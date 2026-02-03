/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Profile-aware waypoint snapper with multi-candidate support.
 * Finds edges in the search area, scores them based on snapping preference,
 * and uses weighted random selection for route variety.
 */
package com.graphhopper.trailmap.roundtrip.snapping;

import com.graphhopper.routing.ev.EncodedValueLookup;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.trailmap.roundtrip.config.RoundTripProfile;
import com.graphhopper.trailmap.shared.GravelScale;
import com.graphhopper.trailmap.shared.MtbScale;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.FetchMode;
import com.graphhopper.util.PointList;
import com.graphhopper.util.shapes.BBox;
import com.graphhopper.util.shapes.GHPoint;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Profile-aware waypoint snapper with multi-candidate support.
 *
 * This snapper:
 * 1. Finds all candidate edges within the search radius using LocationIndex
 * 2. Scores each candidate based on snapping preference (gravel_scale, mtb_scale)
 * 3. Selects from top candidates using weighted random selection (seed-based)
 *
 * This provides both quality (preference-based scoring) and variety
 * (weighted random selection among good candidates).
 */
public class ProfileAwareSnapper implements WaypointSnapStrategy {

    private static final Logger logger = LoggerFactory.getLogger(ProfileAwareSnapper.class);

    // Distance calculation
    private static final DistanceCalcEarth DIST_CALC = DistanceCalcEarth.DIST_EARTH;

    // Configuration
    private static final double DISTANCE_PENALTY_PER_100M = -1.0;
    private static final int MAX_CANDIDATES = 30;
    private static final int TOP_N_FOR_SELECTION = 5;

    // Golden ratio for hash mixing
    private static final long GOLDEN_RATIO = 0x9E3779B97F4A7C15L;

    // Graph reference for edge iteration
    private final Graph graph;

    /**
     * Create snapper without graph reference (uses findClosest only).
     */
    public ProfileAwareSnapper() {
        this(null);
    }

    /**
     * Create snapper with graph reference for multi-candidate support.
     *
     * @param graph Graph for edge iteration (null to use findClosest fallback)
     */
    public ProfileAwareSnapper(Graph graph) {
        this.graph = graph;
    }

    @Override
    public String getName() {
        return "profile-aware";
    }

    @Override
    public boolean isProfileAware() {
        return true;
    }

    @Override
    public Snap findBestSnap(GHPoint targetPoint,
                             RoundTripProfile profile,
                             LocationIndex locationIndex,
                             EdgeFilter edgeFilter,
                             EncodedValueLookup encodedValueLookup) {
        // Legacy method - use GRAVEL preference as default
        return findBestSnap(targetPoint, profile, locationIndex, edgeFilter,
                            encodedValueLookup, SnappingPreference.GRAVEL, 0, 0);
    }

    @Override
    public Snap findBestSnap(GHPoint targetPoint,
                             RoundTripProfile profile,
                             LocationIndex locationIndex,
                             EdgeFilter edgeFilter,
                             EncodedValueLookup encodedValueLookup,
                             SnappingPreference preference,
                             long seed,
                             int waypointIndex) {

        double searchRadius = profile.getSnapping().getSearchRadiusMeters();

        // Find all candidates within search radius
        List<Snap> candidates;
        if (graph != null) {
            candidates = findAllCandidatesInRadius(
                targetPoint, searchRadius, locationIndex, edgeFilter);
        } else {
            // Fallback: just use findClosest
            candidates = new ArrayList<>();
            Snap closest = locationIndex.findClosest(
                targetPoint.getLat(), targetPoint.getLon(), edgeFilter);
            if (closest.isValid() && closest.getQueryDistance() <= searchRadius) {
                candidates.add(closest);
            }
        }

        if (candidates.isEmpty()) {
            logger.debug("No snap candidates found within {}m of {}", searchRadius, targetPoint);
            return null;
        }

        logger.debug("Found {} snap candidates within {}m", candidates.size(), searchRadius);

        // Get encoded values for scoring
        EnumEncodedValue<GravelScale> gravelEnc = getGravelScaleEnc(encodedValueLookup);
        EnumEncodedValue<MtbScale> mtbEnc = getMtbScaleEnc(encodedValueLookup);

        // Score all candidates
        List<ScoredSnap> scored = new ArrayList<>();
        for (Snap snap : candidates) {
            double score = scoreSnapCandidate(snap, preference, gravelEnc, mtbEnc);
            scored.add(new ScoredSnap(snap, score));
        }

        // Sort by score descending
        scored.sort(Comparator.comparingDouble(s -> -s.score));

        // If only one candidate or no variation requested, return best
        if (scored.size() == 1 || seed == 0) {
            Snap best = scored.get(0).snap;
            logger.debug("Returning best snap (score: {})", scored.get(0).score);
            return best;
        }

        // Take top N for weighted random selection
        int topN = Math.min(TOP_N_FOR_SELECTION, scored.size());
        List<ScoredSnap> topCandidates = scored.subList(0, topN);

        // Weighted random selection based on seed
        Snap selected = selectWeightedRandom(topCandidates, seed, waypointIndex);

        if (logger.isDebugEnabled()) {
            int selectedIndex = -1;
            for (int i = 0; i < topCandidates.size(); i++) {
                if (topCandidates.get(i).snap == selected) {
                    selectedIndex = i;
                    break;
                }
            }
            logger.debug("Selected candidate {} of {} (scores: {})",
                selectedIndex + 1, topN,
                topCandidates.stream().map(s -> String.format("%.1f", s.score)).toList());
        }

        return selected;
    }

    /**
     * Find all edges within radius and create snap candidates.
     */
    private List<Snap> findAllCandidatesInRadius(GHPoint target, double radiusMeters,
                                                  LocationIndex locationIndex,
                                                  EdgeFilter edgeFilter) {
        List<Snap> candidates = new ArrayList<>();

        // Create search bounding box
        double deltaLat = radiusMeters / 111000.0;  // ~111km per degree lat
        double deltaLon = radiusMeters / (111000.0 * Math.cos(Math.toRadians(target.getLat())));
        BBox searchBox = new BBox(
            target.getLon() - deltaLon, target.getLon() + deltaLon,
            target.getLat() - deltaLat, target.getLat() + deltaLat
        );

        // Collect edges from spatial index
        Set<Integer> seenEdges = new HashSet<>();
        locationIndex.query(searchBox, edgeId -> {
            if (seenEdges.size() < MAX_CANDIDATES && seenEdges.add(edgeId)) {
                EdgeIteratorState edge = graph.getEdgeIteratorStateForKey(edgeId * 2);
                if (edgeFilter.accept(edge)) {
                    Snap snap = createSnapForEdge(target, edge, radiusMeters);
                    if (snap != null && snap.isValid()) {
                        candidates.add(snap);
                    }
                }
            }
        });

        // Also add the closest snap if not already included
        // (ensures we always have at least one good candidate)
        Snap closest = locationIndex.findClosest(target.getLat(), target.getLon(), edgeFilter);
        if (closest.isValid() && closest.getQueryDistance() <= radiusMeters) {
            boolean alreadyHave = candidates.stream()
                .anyMatch(s -> s.getClosestEdge().getEdge() == closest.getClosestEdge().getEdge());
            if (!alreadyHave) {
                candidates.add(closest);
            }
        }

        return candidates;
    }

    /**
     * Create a Snap for a specific edge.
     */
    private Snap createSnapForEdge(GHPoint target, EdgeIteratorState edge, double maxDist) {
        PointList points = edge.fetchWayGeometry(FetchMode.ALL);
        if (points.size() < 2) {
            return null;
        }

        double minDist = Double.MAX_VALUE;
        int bestWayIndex = 0;

        // Check each segment of the edge
        for (int i = 0; i < points.size() - 1; i++) {
            double lat1 = points.getLat(i);
            double lon1 = points.getLon(i);
            double lat2 = points.getLat(i + 1);
            double lon2 = points.getLon(i + 1);

            // Calculate distance to this segment
            double dist = calcDistanceToSegment(
                target.getLat(), target.getLon(),
                lat1, lon1, lat2, lon2);

            if (dist < minDist) {
                minDist = dist;
                bestWayIndex = i;
            }
        }

        if (minDist > maxDist) {
            return null;
        }

        Snap snap = new Snap(target.getLat(), target.getLon());
        snap.setQueryDistance(minDist);
        snap.setClosestEdge(edge.detach(false));
        snap.setWayIndex(bestWayIndex);
        snap.setSnappedPosition(Snap.Position.EDGE);

        try {
            snap.calcSnappedPoint(DIST_CALC);
        } catch (Exception e) {
            // If snapped point calculation fails, skip this candidate
            logger.debug("Failed to calculate snapped point: {}", e.getMessage());
            return null;
        }

        return snap;
    }

    /**
     * Calculate distance from a point to a line segment.
     */
    private double calcDistanceToSegment(double lat, double lon,
                                          double lat1, double lon1,
                                          double lat2, double lon2) {
        // Project point onto line
        double dx = lat2 - lat1;
        double dy = lon2 - lon1;

        if (dx == 0 && dy == 0) {
            // Segment is a point
            return DIST_CALC.calcDist(lat, lon, lat1, lon1);
        }

        // Parameter t of the projection onto the line
        double t = ((lat - lat1) * dx + (lon - lon1) * dy) / (dx * dx + dy * dy);

        if (t < 0) {
            // Closest to start point
            return DIST_CALC.calcDist(lat, lon, lat1, lon1);
        } else if (t > 1) {
            // Closest to end point
            return DIST_CALC.calcDist(lat, lon, lat2, lon2);
        } else {
            // Closest to a point on the segment
            double projLat = lat1 + t * dx;
            double projLon = lon1 + t * dy;
            return DIST_CALC.calcDist(lat, lon, projLat, projLon);
        }
    }

    /**
     * Score a snap candidate based on preference pattern.
     */
    private double scoreSnapCandidate(Snap snap, SnappingPreference preference,
                                       EnumEncodedValue<GravelScale> gravelEnc,
                                       EnumEncodedValue<MtbScale> mtbEnc) {
        double score = 0;

        EdgeIteratorState edge = snap.getClosestEdge();
        if (edge != null) {
            GravelScale gravelScale = null;
            MtbScale mtbScale = null;

            try {
                if (gravelEnc != null) {
                    gravelScale = edge.get(gravelEnc);
                }
            } catch (Exception e) {
                // Encoded value not available for this edge
            }

            try {
                if (mtbEnc != null) {
                    mtbScale = edge.get(mtbEnc);
                }
            } catch (Exception e) {
                // Encoded value not available for this edge
            }

            score += preference.scoreEdge(gravelScale, mtbScale);
        }

        // Distance penalty - prefer closer snaps
        score += (snap.getQueryDistance() / 100.0) * DISTANCE_PENALTY_PER_100M;

        return score;
    }

    /**
     * Weighted random selection from top candidates.
     */
    private Snap selectWeightedRandom(List<ScoredSnap> candidates, long seed, int waypointIndex) {
        if (candidates.isEmpty()) return null;
        if (candidates.size() == 1) return candidates.get(0).snap;

        // Normalize scores to weights (shift so all positive)
        double minScore = candidates.stream().mapToDouble(s -> s.score).min().orElse(0);
        double[] weights = new double[candidates.size()];
        double totalWeight = 0;

        for (int i = 0; i < candidates.size(); i++) {
            // +1 to avoid zero weight, squared to favor higher scores more
            double normalizedScore = candidates.get(i).score - minScore + 1.0;
            weights[i] = normalizedScore * normalizedScore;
            totalWeight += weights[i];
        }

        // Deterministic random based on seed
        double rand = deterministicRandom(seed, waypointIndex);

        // Select based on cumulative weights
        double target = rand * totalWeight;
        double cumulative = 0;

        for (int i = 0; i < candidates.size(); i++) {
            cumulative += weights[i];
            if (cumulative >= target) {
                return candidates.get(i).snap;
            }
        }

        // Fallback (shouldn't happen)
        return candidates.get(candidates.size() - 1).snap;
    }

    /**
     * Deterministic random value in [0, 1) based on seed and index.
     */
    private double deterministicRandom(long seed, int index) {
        long combined = seed ^ (index * 127L) ^ ("snap_select".hashCode() * 31L);
        combined *= GOLDEN_RATIO;
        combined ^= combined >>> 33;
        combined *= GOLDEN_RATIO;
        return (Math.abs(combined) % 1000000) / 1000000.0;
    }

    /**
     * Get the gravel_scale encoded value if available.
     */
    private EnumEncodedValue<GravelScale> getGravelScaleEnc(EncodedValueLookup lookup) {
        if (lookup == null) return null;
        try {
            if (lookup.hasEncodedValue(GravelScale.KEY)) {
                return lookup.getEnumEncodedValue(GravelScale.KEY, GravelScale.class);
            }
        } catch (Exception e) {
            logger.debug("GravelScale encoded value not available: {}", e.getMessage());
        }
        return null;
    }

    /**
     * Get the mtb_scale encoded value if available.
     */
    private EnumEncodedValue<MtbScale> getMtbScaleEnc(EncodedValueLookup lookup) {
        if (lookup == null) return null;
        try {
            if (lookup.hasEncodedValue(MtbScale.KEY)) {
                return lookup.getEnumEncodedValue(MtbScale.KEY, MtbScale.class);
            }
        } catch (Exception e) {
            logger.debug("MtbScale encoded value not available: {}", e.getMessage());
        }
        return null;
    }

    /**
     * Internal class to hold snap with its score.
     */
    private static class ScoredSnap {
        final Snap snap;
        final double score;

        ScoredSnap(Snap snap, double score) {
            this.snap = snap;
            this.score = score;
        }
    }
}
