package com.graphhopper.trailmap.tilerouting.foundation;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/** A tile_routing candidate: given home and the analysed tiles, choose the ordered via points. */
public interface Candidate {

    /** The job as the candidate sees it. Tiles are the reachable ones, in the user's order when {@link #ordered}. */
    final class Problem {
        public String jobId;
        public Foundation foundation;
        public Foundation.Home home;
        public List<Foundation.TileInfo> tiles = new ArrayList<>();
        public boolean ordered;
        public boolean loop = true;
        /**
         * Ordered groups (job field {@code groups}): tile key → group rank 0..G-1; tiles in no group rank G (after
         * the last group). Vias must visit tiles in non-decreasing rank; order is free inside a rank. null = free order.
         */
        public Map<String, Integer> groupRank;
        public int groupCount;

        /** Rank of a tile (0 when the job has no groups). */
        public int rank(String tileKey) {
            if (groupRank == null || tileKey == null) return 0;
            return groupRank.getOrDefault(tileKey, groupCount);
        }

        public boolean grouped() {
            return groupRank != null;
        }
    }

    final class Solution {
        /** Home first and last for a loop. */
        public List<Via> vias = new ArrayList<>();
        /** Route with pass_through (portal headings are arrival-and-continue directions). */
        public boolean passThrough;
        public long tableMs, solveMs;
        public long searches, settled;
        public int unresolvedLegs;
        /** Edge keys the candidate's search structures held at their peak, summed over its searchers (memory check). */
        public long keysHeld;
        /** Optional: the candidate's own cost per leg (vias i → i+1), for the parity check against GraphHopper. */
        public double[] legCost;
        public String note = "";
    }

    String name();

    /** Optional: adjust the foundation's levers before tiles are analysed (default: leave them). */
    default void configure(Foundation.Params params) {
    }

    Solution solve(Problem problem);
}
