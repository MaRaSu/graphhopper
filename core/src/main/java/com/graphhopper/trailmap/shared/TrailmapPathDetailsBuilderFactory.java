/*
 * Trailmap - TrailmapPathDetailsBuilderFactory
 *
 * Path details factory that swaps in Trailmap-specific builders where the stock
 * builder would expose an internal-only value.
 *
 * Currently that is predicted_highway only: the default factory hands out a generic
 * EnumDetails that reads the stored enum, which would leak internal-only refinements
 * (e.g. SERVICE_DRIVEWAY) to API clients.
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.routing.Path;
import com.graphhopper.routing.ev.EncodedValueLookup;
import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.details.PathDetailsBuilder;
import com.graphhopper.util.details.PathDetailsBuilderFactory;

import java.util.List;

/**
 * Extends the default factory and replaces the predicted_highway builder with one that
 * projects to the client-facing value. Everything else is left to the parent so new
 * upstream path details keep working unchanged.
 */
public class TrailmapPathDetailsBuilderFactory extends PathDetailsBuilderFactory {

    @Override
    public List<PathDetailsBuilder> createPathDetailsBuilders(List<String> requestedPathDetails, Path path,
                                                              EncodedValueLookup evl, Weighting weighting, Graph graph) {
        List<PathDetailsBuilder> builders =
                super.createPathDetailsBuilders(requestedPathDetails, path, evl, weighting, graph);

        for (int i = 0; i < builders.size(); i++) {
            if (PredictedHighway.KEY.equals(builders.get(i).getName())) {
                EnumEncodedValue<PredictedHighway> predictedHighwayEnc =
                        evl.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class);
                builders.set(i, new PredictedHighwayDetails(predictedHighwayEnc));
                break;
            }
        }
        return builders;
    }
}
