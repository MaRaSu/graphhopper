/*
 * Trailmap - Personal routing
 */
package com.graphhopper.trailmap.personal;

import com.graphhopper.routing.weighting.AbstractAdjustedWeighting;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.util.EdgeIteratorState;

/**
 * Wraps a profile's weighting with the user's history.
 *
 * <ul>
 *   <li>Edge cost: {@code profile cost × (1 + strength × q)}, q in 0..1 by mode. favor: q = 1 − habit
 *       (rarely ridden edges cost more); avoid: q = habit; explore: q = share (anything ridden costs
 *       more). habit = share × min(1, rides / saturation). The factor is never below 1, so the
 *       profile's minimum weight per metre, and with it the A* estimate, stays valid, and a route
 *       costs at most (1 + strength) times the profile's best.</li>
 *   <li>Switch penalty: at a junction where the route goes from a ridden to an unridden edge or
 *       back, {@code switchPenalty} is added to the turn cost. A short dart into an unridden side
 *       street and back pays it twice. Only profiles with turn costs evaluate turns.</li>
 * </ul>
 * Travel time ({@code calcEdgeMillis}, {@code calcTurnMillis}) is the profile's, unchanged.
 */
public class PersonalWeighting extends AbstractAdjustedWeighting {

    private final PersonalFamiliarity familiarity;
    private final PersonalParams params;

    public PersonalWeighting(Weighting profileWeighting, PersonalFamiliarity familiarity, PersonalParams params) {
        super(profileWeighting);
        this.familiarity = familiarity;
        this.params = params;
    }

    @Override
    public double calcEdgeWeight(EdgeIteratorState edgeState, boolean reverse) {
        double w = superWeighting.calcEdgeWeight(edgeState, reverse);
        if (params.strength <= 0 || Double.isInfinite(w)) return w;
        return w * (1 + params.strength * q(edgeState));
    }

    double q(EdgeIteratorState edge) {
        double share = familiarity.share(edge);
        switch (params.mode) {
            case EXPLORE:
                return share;
            case FAVOR:
                return 1 - habit(share, edge);
            case AVOID:
                return habit(share, edge);
            default:
                return 0;
        }
    }

    private double habit(double share, EdgeIteratorState edge) {
        if (share <= 0) return 0;
        return share * Math.min(1, familiarity.rides(edge) / params.ridesSaturation);
    }

    @Override
    public double calcTurnWeight(int inEdge, int viaNode, int outEdge) {
        double t = superWeighting.calcTurnWeight(inEdge, viaNode, outEdge);
        if (params.switchPenalty <= 0 || Double.isInfinite(t)
                || !EdgeIterator.Edge.isValid(inEdge) || !EdgeIterator.Edge.isValid(outEdge))
            return t;
        boolean inKnown = familiarity.share(inEdge) >= params.knownShare;
        boolean outKnown = familiarity.share(outEdge) >= params.knownShare;
        return inKnown == outKnown ? t : t + params.switchPenalty;
    }

    @Override
    public String getName() {
        return "personal";
    }
}
