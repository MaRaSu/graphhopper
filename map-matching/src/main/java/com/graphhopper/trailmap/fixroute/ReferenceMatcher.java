package com.graphhopper.trailmap.fixroute;

import com.graphhopper.GraphHopper;
import com.graphhopper.matching.EdgeMatch;
import com.graphhopper.matching.MapMatching;
import com.graphhopper.matching.MatchResult;
import com.graphhopper.matching.Observation;
import com.graphhopper.matching.Tracepoint;
import com.graphhopper.routing.querygraph.VirtualEdgeIteratorState;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.trailmap.matching.MatcherConfig;
import com.graphhopper.trailmap.matching.ObservationDensifier;
import com.graphhopper.trailmap.matching.TrailmapMapMatching;
import com.graphhopper.trailmap.shared.EdgeKeyMatching;
import com.graphhopper.util.CustomModel;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.PMap;
import com.graphhopper.util.Parameters;
import com.graphhopper.util.shapes.GHPoint;

import java.util.ArrayList;
import java.util.List;

/**
 * Map-matches a slice of the clean reference onto today's graph with the leg's OWN routing settings
 * (profile + custom model), using the Trailmap matcher fork unchanged (design doc §3.4 step 1).
 *
 * <p>Clean-input settings: the reference is a routed line (vertices on old road geometry, RDP 3 m),
 * not GPS, so a small σ and chord densification. The matcher's built-in gap splitting turns
 * stretches with no acceptable road into gaps instead of failing the match.
 */
public class ReferenceMatcher {

    /** Starting values — owner tunes after corpus runs (design doc §6). */
    public static final double DEFAULT_SIGMA_M = 5.0;
    public static final double DEFAULT_DENSIFY_M = 20.0;

    private final GraphHopper hopper;
    private final double sigmaM;
    private final double densifyM;

    public ReferenceMatcher(GraphHopper hopper) {
        this(hopper, DEFAULT_SIGMA_M, DEFAULT_DENSIFY_M);
    }

    public ReferenceMatcher(GraphHopper hopper, double sigmaM, double densifyM) {
        this.hopper = hopper;
        this.sigmaM = sigmaM;
        this.densifyM = densifyM;
    }

    public record Match(MatchResult result, List<Observation> observations, int[] edgeKeys,
                        int gapObservations, double maxSnapM) {
        public boolean hasGap() {
            return gapObservations > 0;
        }
    }

    public Match match(List<double[]> slice, String profile, CustomModel customModel) {
        List<Observation> obs = new ArrayList<>(slice.size());
        for (double[] p : slice) obs.add(new Observation(new GHPoint(p[0], p[1])));
        return matchObservations(ObservationDensifier.densify(obs, densifyM), profile, customModel);
    }

    /** Match exactly these observations (no densification): callers that need stable positions. */
    public Match matchPoints(List<double[]> points, String profile, CustomModel customModel) {
        List<Observation> obs = new ArrayList<>(points.size());
        for (double[] p : points) obs.add(new Observation(new GHPoint(p[0], p[1])));
        return matchObservations(obs, profile, customModel);
    }

    private Match matchObservations(List<Observation> obs, String profile, CustomModel customModel) {
        PMap hints = new PMap();
        hints.putObject("profile", profile);
        hints.putObject(Parameters.CH.DISABLE, true);
        if (customModel != null) hints.putObject(CustomModel.KEY, customModel);
        MapMatching.Router router = MapMatching.routerFromGraphHopper(hopper, hints);

        MatcherConfig cfg = new MatcherConfig().sigma(sigmaM);
        TrailmapMapMatching mm = new TrailmapMapMatching(hopper.getBaseGraph(),
                (LocationIndexTree) hopper.getLocationIndex(), router, cfg);
        MatchResult res = mm.match(obs);

        List<EdgeMatch> ems = res.getEdgeMatches();
        int[] keys = new int[ems.size()];
        for (int i = 0; i < ems.size(); i++) keys[i] = originalEdgeKey(ems.get(i).getEdgeState());
        int gaps = 0;
        double maxSnap = 0;
        for (Tracepoint tp : res.getTracepoints()) {
            if (!tp.isMatched()) {
                gaps++;
            } else if (tp.getDistance() != null) {
                maxSnap = Math.max(maxSnap, tp.getDistance());
            }
        }
        return new Match(res, obs, EdgeKeyMatching.dedupConsecutive(keys), gaps, maxSnap);
    }

    /** Virtual (query-graph) edges report the key of the real edge they split. */
    static int originalEdgeKey(EdgeIteratorState e) {
        return e instanceof VirtualEdgeIteratorState v ? v.getOriginalEdgeKey() : e.getEdgeKey();
    }
}
