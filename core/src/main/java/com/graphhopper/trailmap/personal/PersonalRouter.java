/*
 * Trailmap - Personal routing
 */
package com.graphhopper.trailmap.personal;

import com.graphhopper.GHRequest;
import com.graphhopper.GraphHopper;
import com.graphhopper.config.Profile;
import com.graphhopper.routing.Router;
import com.graphhopper.routing.RouterConfig;
import com.graphhopper.routing.WeightingFactory;
import com.graphhopper.routing.ev.EncodedValueLookup;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.index.LocationIndex;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * A {@link Router} for one personal request: the same routing as {@code /route}, always in flexible
 * mode, with the profile's weighting wrapped in {@link PersonalWeighting}. Created per request, as
 * GraphHopper creates its own router per request. With {@code familiarity == null} or an inactive
 * mode it routes exactly as the plain router does.
 */
public class PersonalRouter extends Router {

    private final PersonalFamiliarity familiarity;
    private final PersonalParams params;

    public PersonalRouter(GraphHopper hopper, PersonalFamiliarity familiarity, PersonalParams params) {
        super(hopper.getBaseGraph(), hopper.getEncodingManager(), hopper.getLocationIndex(), profilesByName(hopper),
                hopper.getPathDetailsBuilderFactory(), hopper.getTranslationMap(), hopper.getRouterConfig(),
                hopper::createWeighting, hopper.getCHGraphs(), hopper.getLandmarks());
        this.familiarity = familiarity;
        this.params = params;
    }

    private static Map<String, Profile> profilesByName(GraphHopper hopper) {
        Map<String, Profile> m = new LinkedHashMap<>();
        for (Profile p : hopper.getProfiles()) m.put(p.getName(), p);
        return m;
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
        return new PersonalFlexSolver(request, profilesByName, routerConfig, encodingManager, weightingFactory, baseGraph,
                locationIndex, familiarity, params);
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
