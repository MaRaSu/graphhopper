package com.graphhopper.trailmap.barrier;

import com.graphhopper.routing.ev.EnumEncodedValue;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.util.EdgeIteratorState;

import java.util.EnumSet;

/**
 * A profile's weighting in which barrier edges of the given classes are passable at a fixed weight. Everything else,
 * turn costs included, is the wrapped weighting. Used for the second island marking (ask and refuse open, so islands
 * behind barriers are not marked) and for the crossing search (ask open at {@link #CROSSING_WEIGHT}).
 */
public class BarrierPassWeighting implements Weighting {

    /** A crossing costs this much (weight units ≈ seconds), so a route uses as few crossings as possible. */
    public static final double CROSSING_WEIGHT = 1800;

    private final Weighting base;
    private final EnumEncodedValue<BarrierClass> enc;
    private final EnumSet<BarrierClass> open;
    private final double weight;

    public BarrierPassWeighting(Weighting base, EnumEncodedValue<BarrierClass> enc, EnumSet<BarrierClass> open, double weight) {
        this.base = base;
        this.enc = enc;
        this.open = open;
        this.weight = weight;
    }

    public Weighting getBase() {
        return base;
    }

    boolean isOpenBarrier(EdgeIteratorState edge) {
        return open.contains(edge.get(enc));
    }

    @Override
    public double calcMinWeightPerDistance() {
        return base.calcMinWeightPerDistance();
    }

    @Override
    public double calcEdgeWeight(EdgeIteratorState edgeState, boolean reverse) {
        if (isOpenBarrier(edgeState))
            return weight;
        return base.calcEdgeWeight(edgeState, reverse);
    }

    @Override
    public long calcEdgeMillis(EdgeIteratorState edgeState, boolean reverse) {
        if (isOpenBarrier(edgeState))
            return 0;
        return base.calcEdgeMillis(edgeState, reverse);
    }

    @Override
    public double calcTurnWeight(int inEdge, int viaNode, int outEdge) {
        return base.calcTurnWeight(inEdge, viaNode, outEdge);
    }

    @Override
    public long calcTurnMillis(int inEdge, int viaNode, int outEdge) {
        return base.calcTurnMillis(inEdge, viaNode, outEdge);
    }

    @Override
    public boolean hasTurnCosts() {
        return base.hasTurnCosts();
    }

    @Override
    public String getName() {
        return base.getName();
    }
}
