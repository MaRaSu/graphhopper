package com.graphhopper.trailmap.tilerouting.tourrepair;

import com.graphhopper.trailmap.tilerouting.foundation.Foundation;
import com.graphhopper.trailmap.tilerouting.foundation.Via;
import com.graphhopper.trailmap.tilerouting.looprepair.LoopRepairCandidate;
import com.graphhopper.trailmap.tilerouting.portaltour.PortalViterbiCandidate;

import java.util.List;

/**
 * Free order + loop repair ({@code algorithm=tour_repair}). Seeds the loop repair's ruin-and-recreate search with the
 * portal tour's free-order solution, then repairs it with the loop repair's profile-weight objective at the given
 * variety (default mild). The given-order seeds are not used. Foundation lever: nodeClearM = 0 (the repair's
 * midChord placement). Ported from the round-2 prototype (c4) with its defaults: seed c1f, nodeClearM 0.
 */
public class FreeOrderRepairCandidate extends LoopRepairCandidate {

    private final long seedBudgetMs;

    public FreeOrderRepairCandidate(String variety, long seedBudgetMs) {
        super(variety);
        this.seedBudgetMs = seedBudgetMs;
    }

    @Override
    public String name() {
        return "c4";
    }

    @Override
    public void configure(Foundation.Params params) {
        params.nodeClearM = 0;
    }

    @Override
    protected boolean useGivenOrderSeeds() {
        return false;
    }

    @Override
    protected List<Via> seedVias(Problem pb) {
        PortalViterbiCandidate seed = new PortalViterbiCandidate(true);
        seed.budgetMs = seedBudgetMs;
        seed.clock = clock;
        return seed.solve(pb).vias;
    }
}
