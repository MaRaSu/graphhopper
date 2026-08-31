/*
 * Trailmap - PredictedHighwayDetails
 *
 * Path details builder for predicted_highway that emits the EXTERNAL (coarse) value.
 *
 * The stock EnumDetails reads the stored enum directly, which would put internal-only
 * refinements (e.g. SERVICE_DRIVEWAY) on the wire. This builder applies
 * PredictedHighway.toExternal() so the path details channel carries the same coarse
 * value as the TbT instruction channel.
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.details.AbstractPathDetailsBuilder;

/**
 * predicted_highway path details, projected to the client-facing value.
 * <p>
 * Interval boundaries are decided on the projected value too: consecutive edges that
 * differ only in an internal-only refinement (SERVICE_ROAD followed by SERVICE_DRIVEWAY)
 * stay in a single interval, exactly as they did before the refinement existed.
 */
public class PredictedHighwayDetails extends AbstractPathDetailsBuilder {

    private final EnumEncodedValue<PredictedHighway> predictedHighwayEnc;
    private PredictedHighway externalValue;

    public PredictedHighwayDetails(EnumEncodedValue<PredictedHighway> predictedHighwayEnc) {
        super(PredictedHighway.KEY);
        this.predictedHighwayEnc = predictedHighwayEnc;
    }

    @Override
    protected Object getCurrentValue() {
        return externalValue.toString();
    }

    @Override
    public boolean isEdgeDifferentToLastEdge(EdgeIteratorState edge) {
        PredictedHighway value = edge.get(predictedHighwayEnc).toExternal();
        // reference equality is fine for enum values
        if (value != externalValue) {
            this.externalValue = value;
            return true;
        }
        return false;
    }
}
