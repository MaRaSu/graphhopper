/*
 * Trailmap - Personal routing
 */
package com.graphhopper.trailmap.personal;

import com.graphhopper.GHRequest;
import com.graphhopper.GraphHopper;
import com.graphhopper.ResponsePath;
import com.graphhopper.config.Profile;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.Router;
import com.graphhopper.routing.RouterConfig;
import com.graphhopper.routing.WeightingFactory;
import com.graphhopper.routing.ev.EncodedValueLookup;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.trailmap.barrier.BarrierRouter;
import com.graphhopper.trailmap.barrier.BarrierTable;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * A {@link Router} for one personal request: the same routing as {@code /route}, always in flexible
 * mode, with the profile's weighting wrapped in {@link PersonalWeighting}. Created per request, as
 * GraphHopper creates its own router per request. With {@code familiarity == null} or an inactive
 * mode it routes exactly as the plain router does.
 * <p>
 * W26: a {@link BarrierRouter}, so a request with {@code "barriers": "v1"} gets /route's barrier answer (a crossing of an
 * ask barrier with its W1 / W2, or the refuse answer) with the personal weighting on top. Without the flag BarrierRouter
 * routes exactly as Router.
 */
public class PersonalRouter extends BarrierRouter {

    private final PersonalFamiliarity familiarity;
    private final PersonalParams params;
    private List<ResponsePath> alternativesWithEdges;


    public PersonalRouter(GraphHopper hopper, PersonalFamiliarity familiarity, PersonalParams params) {
        super(hopper.getBaseGraph(), hopper.getEncodingManager(), hopper.getLocationIndex(), profilesByName(hopper),
                hopper.getPathDetailsBuilderFactory(), hopper.getTranslationMap(), hopper.getRouterConfig(),
                hopper::createWeighting, hopper.getCHGraphs(), hopper.getLandmarks(),
                BarrierTable.of(hopper));
        this.familiarity = familiarity;
        this.params = params;
    }

    private static Map<String, Profile> profilesByName(GraphHopper hopper) {
        Map<String, Profile> m = new LinkedHashMap<>();
        for (Profile p : hopper.getProfiles()) m.put(p.getName(), p);
        return m;
    }

    /**
     * Personal alternatives are not plain /route: the /personal_route resource pins them itself, from the paths with
     * their edges recorded here ({@link #getAlternativesWithEdges}), so it needs no second search for them. Without
     * an active history they are /route's own alternatives, pinned as /route pins them.
     */
    @Override
    protected void recreateAltRouteVias(GHRequest request, List<Path> paths, List<ResponsePath> responsePaths,
                                        Weighting weighting, Graph queryGraph) {
        if (familiarity == null || params == null || !params.isActive()) {
            super.recreateAltRouteVias(request, paths, responsePaths, weighting, queryGraph);
            return;
        }
        if (paths.size() != responsePaths.size()) return;
        // an unfound search (e.g. a barrier router's first pass) records nothing: its retry goes through routeVia
        if (paths.stream().anyMatch(p -> !p.isFound())) return;
        alternativesWithEdges = new ArrayList<>();
        for (int i = 0; i < paths.size(); i++)
            alternativesWithEdges.add(AltRouteVias.withEdges(paths.get(i), responsePaths.get(i), encodingManager,
                    pathDetailsBuilderFactory, weighting, queryGraph));
    }

    /** The last alternative_route answer's paths with their edges, in response order; null for any other routing. */
    public List<ResponsePath> getAlternativesWithEdges() {
        return alternativesWithEdges;
    }

    @Override
    protected Solver createSolver(GHRequest request) {
        // Personal weights change per request, so prepared CH/LM data can't be used.
        return createFlexSolver(request, profilesByName, routerConfig, encodingManager, weightingFactory, graph, locationIndex);
    }

    @Override
    protected Solver createFlexSolver(GHRequest request, Map<String, Profile> profilesByName, RouterConfig routerConfig,
                                      EncodingManager encodingManager, WeightingFactory weightingFactory, BaseGraph baseGraph,
                                      LocationIndex locationIndex) {
        if (BarrierRouter.isOn(request))
            return new PersonalBarrierFlexSolver(request, profilesByName, routerConfig, encodingManager, weightingFactory,
                    baseGraph, locationIndex, passOf(request), familiarity, params);
        return new PersonalFlexSolver(request, profilesByName, routerConfig, encodingManager, weightingFactory, baseGraph,
                locationIndex, familiarity, params);
    }

    /**
     * W26 flag: BarrierRouter's solver (barrier-aware snapping, the pass weighting, its routeVia's crossings) with the
     * personal weighting on top. A subclass of BarrierFlexSolver, else BarrierRouter.routeVia would not list crossings.
     */
    static class PersonalBarrierFlexSolver extends BarrierFlexSolver {
        private final PersonalFamiliarity familiarity;
        private final PersonalParams params;

        PersonalBarrierFlexSolver(GHRequest request, Map<String, Profile> profilesByName, RouterConfig routerConfig,
                                  EncodedValueLookup lookup, WeightingFactory weightingFactory, BaseGraph graph,
                                  LocationIndex locationIndex, String pass, PersonalFamiliarity familiarity, PersonalParams params) {
            super(request, profilesByName, routerConfig, lookup, weightingFactory, graph, locationIndex, pass);
            this.familiarity = familiarity;
            this.params = params;
        }

        @Override
        protected Weighting createWeighting() {
            Weighting barrierWeighting = super.createWeighting();
            if (familiarity == null || params == null || !params.isActive()) return barrierWeighting;
            return new PersonalWeighting(barrierWeighting, familiarity, params);
        }
    }

    static class PersonalFlexSolver extends FlexSolver {
        private final PersonalFamiliarity familiarity;
        private final PersonalParams params;

        PersonalFlexSolver(GHRequest request, Map<String, Profile> profilesByName, RouterConfig routerConfig,
                           EncodedValueLookup lookup, WeightingFactory weightingFactory, BaseGraph graph,
                           LocationIndex locationIndex, PersonalFamiliarity familiarity, PersonalParams params) {
            super(request, profilesByName, routerConfig, lookup, weightingFactory, graph, locationIndex);
            this.familiarity = familiarity;
            this.params = params;
        }

        @Override
        protected Weighting createWeighting() {
            Weighting profileWeighting = super.createWeighting();
            if (familiarity == null || params == null || !params.isActive()) return profileWeighting;
            return new PersonalWeighting(profileWeighting, familiarity, params);
        }
    }
}
