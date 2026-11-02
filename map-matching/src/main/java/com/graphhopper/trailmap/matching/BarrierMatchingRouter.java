package com.graphhopper.trailmap.matching;

import com.graphhopper.GraphHopper;
import com.graphhopper.config.Profile;
import com.graphhopper.matching.MapMatching;
import com.graphhopper.routing.DijkstraBidirectionRef;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.querygraph.QueryGraph;
import com.graphhopper.routing.util.DefaultSnapFilter;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.trailmap.barrier.BarrierClass;
import com.graphhopper.trailmap.barrier.BarrierPassWeighting;
import com.graphhopper.trailmap.barrier.BarrierProfiles;
import com.graphhopper.util.PMap;
import com.graphhopper.util.Parameters;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;

/**
 * The map-matching router for a recorded track with {@code "barriers": "v1"} (W26 S4, /convert_track): the matching
 * profile's weighting with ask and refuse barrier edges passable at {@link #MATCH_CROSSING_WEIGHT} (a recording is
 * evidence that the user passed), and snapping on {@code <profile>_subnetwork_b}, so observations behind a barrier
 * match where they are. Everything else as GraphHopper's {@link MapMatching#routerFromGraphHopper} (Dijkstra; the
 * Trailmap configs have no LM profiles).
 */
public final class BarrierMatchingRouter {

    /** Weight of passing a closed barrier edge while matching (weight units ≈ s): small, the passage was recorded. */
    public static final double MATCH_CROSSING_WEIGHT = 1;

    private BarrierMatchingRouter() {
    }

    public static MapMatching.Router create(GraphHopper hopper, PMap hints) {
        return create(hopper, hints, true);
    }

    /** @param passBarriers false: barriers stay closed (only the snapping is barrier-aware), for the track's pieces */
    public static MapMatching.Router create(GraphHopper hopper, PMap hints, boolean passBarriers) {
        String profileStr = hints.getString("profile", "");
        Profile profile = hopper.getProfile(profileStr);
        if (profile == null)
            throw new IllegalArgumentException("Could not find profile '" + profileStr + "'");
        Weighting base = hopper.createWeighting(profile, hints);
        EnumEncodedValue<BarrierClass> enc = BarrierProfiles.barrierEnc(hopper.getEncodingManager(), profile);
        Weighting weighting = enc == null || !passBarriers ? base
                : new BarrierPassWeighting(base, enc, EnumSet.of(BarrierClass.ASK, BarrierClass.REFUSE), MATCH_CROSSING_WEIGHT);
        DefaultSnapFilter snapFilter = new DefaultSnapFilter(base, BarrierProfiles.subnetworkEnc(hopper.getEncodingManager(), profileStr));
        int maxVisitedNodes = hints.getInt(Parameters.Routing.MAX_VISITED_NODES, Integer.MAX_VALUE);
        return new MapMatching.Router() {
            @Override
            public EdgeFilter getSnapFilter() {
                return snapFilter;
            }

            @Override
            public List<Path> calcPaths(QueryGraph queryGraph, int fromNode, int fromOutEdge, int[] toNodes, int[] toInEdges) {
                List<Path> result = new ArrayList<>();
                for (int i = 0; i < toNodes.length; i++) {
                    DijkstraBidirectionRef algo = new DijkstraBidirectionRef(queryGraph, queryGraph.wrapWeighting(weighting), TraversalMode.EDGE_BASED) {
                        @Override
                        protected void initCollections(int size) {
                            super.initCollections(50);
                        }
                    };
                    algo.setMaxVisitedNodes(maxVisitedNodes);
                    result.add(algo.calcPath(fromNode, toNodes[i], fromOutEdge, toInEdges[i]));
                }
                return result;
            }

            @Override
            public Weighting getWeighting() {
                return weighting;
            }
        };
    }
}
