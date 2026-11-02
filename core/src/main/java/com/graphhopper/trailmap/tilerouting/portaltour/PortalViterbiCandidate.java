package com.graphhopper.trailmap.tilerouting.portaltour;

import com.graphhopper.trailmap.tilerouting.foundation.*;

import java.util.*;

/**
 * Candidate 1, portal tour search, for plain via points (no headings, no pass_through: what the app rebuilds from
 * waypoints).
 * <ul>
 * <li>Fixed order: exact second-order Viterbi ({@link PlainViterbi}) over home, the tiles in the given order, home.</li>
 * <li>Free order: portal GTSP. Farthest insertion by centre distance from home builds the order; 2-opt and or-opt
 * (segments of 1–3) improve it, each move scored by the Viterbi (portal re-pick for the whole order), only moves
 * whose new neighbours are among each tile's k nearest (or home); then absorption: a tile whose inset another leg
 * already crosses is dropped from the via list when every dropped tile stays crossed and the score does not rise.
 * Everything runs under a time budget.</li>
 * </ul>
 * Levers (the prototype's system properties and defaults): budgetMs (tr.budgetMs 5000; the caller sets it), k (tr.k 6),
 * uTurnPenalty (tr.uTurnPenalty 60), shallowPenalty (tr.shallowPenalty 60).
 */
public class PortalViterbiCandidate implements Candidate {

    /** Wall-clock budget for the moves after the seed. */
    public long budgetMs = 5000;
    /** The budget's clock: the wall clock in production, a virtual per-call clock in reproducible tests. */
    public java.util.function.LongSupplier clock = System::nanoTime;
    public int k = 6;
    public double uTurnPenalty = 60;
    public double shallowPenalty = 60;
    private final boolean forceFree;

    public PortalViterbiCandidate() {
        this(false);
    }

    public PortalViterbiCandidate(boolean forceFree) {
        this.forceFree = forceFree;
    }

    @Override
    public String name() {
        return forceFree ? "c1f" : "c1";
    }

    @Override
    public Solution solve(Problem pb) {
        long t0 = System.nanoTime();
        Foundation f = pb.foundation;
        LegOracle o = new LegOracle(f, pb.home, pb.tiles, k);
        PlainViterbi vit = new PlainViterbi();
        vit.uTurnPenalty = uTurnPenalty;
        vit.shallowPenalty = shallowPenalty;
        boolean free = forceFree || !pb.ordered;
        Solution sol = new Solution();
        sol.passThrough = false;
        sol.note = String.format(Locale.ROOT, "%s plain-via viterbi uTurnPenalty=%.0f shallowPenalty=%.0f", free ? "free" : "fixed",
                uTurnPenalty, shallowPenalty);

        List<Integer> order = new ArrayList<>();
        for (int i = 1; i <= pb.tiles.size(); i++) order.add(i);
        PlainViterbi.Result best;
        List<Integer> absorbed = new ArrayList<>();
        if (!free) {
            best = vit.solve(o, withHome(order));
        } else {
            // ordered groups: rank per oracle node (1..N = pb.tiles); null = free order
            int[] rank = null;
            if (pb.grouped()) {
                rank = new int[pb.tiles.size() + 1];
                for (int i = 1; i <= pb.tiles.size(); i++) rank[i] = pb.rank(pb.tiles.get(i - 1).tile.key);
                sol.note += " groups=" + pb.groupCount;
            }
            if (rank == null) order = farthestInsertion(o, order, null);
            else {
                // seeds for ordered groups: rank-constrained insertion, and the free tour (both directions) with its
                // tiles stably sorted by rank; the cheapest by Viterbi score starts the search
                List<Integer> freeSeed = farthestInsertion(o, order, null), rev = new ArrayList<>(freeSeed);
                Collections.reverse(rev);
                final int[] rk = rank;
                List<List<Integer>> seeds = new ArrayList<>(List.of(farthestInsertion(o, order, rank), freeSeed, rev));
                for (int i = 1; i < 3; i++) seeds.get(i).sort(Comparator.comparingInt(t -> rk[t]));
                double bestSeed = Double.POSITIVE_INFINITY;
                int pick = 0;
                for (int i = 0; i < seeds.size(); i++) {
                    double sc = vit.solve(o, withHome(seeds.get(i))).score;
                    if (sc < bestSeed) {
                        bestSeed = sc;
                        pick = i;
                    }
                }
                order = seeds.get(pick);
                sol.note += " seed=" + new String[]{"insertion", "freeSorted", "freeReversedSorted"}[pick];
            }
            Search s = new Search(o, vit, budgetMs * 1_000_000);
            s.rank = rank;
            s.clock = clock;
            long ts = clock.getAsLong();
            best = s.improve(order);
            order = s.order;
            long deadline = s.deadline;
            s.seedMs = (s.deadline - budgetMs * 1_000_000 - ts) / 1_000_000;
            // absorption: drop tiles other legs already cross
            boolean dropped = true;
            while (dropped && clock.getAsLong() < deadline) {
                dropped = false;
                for (int pos = 0; pos < order.size(); pos++) {
                    int tile = order.get(pos);
                    // a grouped tile keeps its own via: "in passing" on a later leg would visit it out of order
                    if (rank != null && rank[tile] < pb.groupCount) continue;
                    if (!crossedByOtherLeg(o, best, tile, pos + 1)) continue;
                    List<Integer> trial = new ArrayList<>(order);
                    trial.remove(pos);
                    PlainViterbi.Result r = vit.solve(o, withHome(trial));
                    List<Integer> gone = new ArrayList<>(absorbed);
                    gone.add(tile);
                    if (r.score <= best.score && allCrossed(o, r, gone)) {
                        best = r;
                        order = trial;
                        absorbed = gone;
                        dropped = true;
                        break;
                    }
                }
            }
            sol.note += String.format(Locale.ROOT, " k=%d budgetMs=%d(after seed %dms) moves=%d evals=%d%s absorbed=%s", k, budgetMs, s.seedMs, s.moves, s.evals,
                    s.timedOut ? " BUDGET_HIT" : "", keys(o, absorbed));
        }

        sol.vias.add(new Via(pb.home.lat, pb.home.lon, Double.NaN, "home", null));
        for (int i = 1; i + 1 < best.chosen.size(); i++) {
            Portal p = best.chosen.get(i).primary();
            sol.vias.add(new Via(p.lat, p.lon, Double.NaN, "portal", p.tileKey));
        }
        sol.vias.add(new Via(pb.home.lat, pb.home.lon, Double.NaN, "home", null));
        sol.legCost = best.legs.stream().mapToDouble(l -> l.cost).toArray();
        if (!Double.isFinite(best.score)) sol.note += " NO_FINITE_TOUR";
        sol.note += String.format(Locale.ROOT, " road=%.0f score=%.0f viaUTurns=%d", best.roadCost, best.score, best.uTurnsAtVias);
        sol.searches = o.searches;
        sol.settled = o.settled;
        sol.unresolvedLegs = o.budgetHits;
        sol.tableMs = o.tableNs / 1_000_000;
        sol.solveMs = (System.nanoTime() - t0) / 1_000_000 - sol.tableMs;
        return sol;
    }

    static List<Integer> withHome(List<Integer> order) {
        List<Integer> l = new ArrayList<>(order.size() + 2);
        l.add(0);
        l.addAll(order);
        l.add(0);
        return l;
    }

    /** Farthest insertion by straight-line centre distance, from the loop home→home. */
    static List<Integer> farthestInsertion(LegOracle o, List<Integer> tiles) {
        return farthestInsertion(o, tiles, null);
    }

    /**
     * With ranks (ordered groups), a tile is only inserted where the tour stays in non-decreasing rank (home first
     * ranks lowest, home last highest); a monotone tour always has such a position.
     */
    static List<Integer> farthestInsertion(LegOracle o, List<Integer> tiles, int[] rank) {
        List<Integer> rest = new ArrayList<>(tiles);
        rest.sort(Comparator.comparingDouble(t -> -o.centreDist(0, t)));
        List<Integer> tour = new ArrayList<>(List.of(0, 0));
        for (int t : rest) {
            int bestPos = 1;
            double bestAdd = Double.MAX_VALUE;
            for (int i = 0; i + 1 < tour.size(); i++) {
                int a = tour.get(i), b = tour.get(i + 1);
                if (rank != null && ((i > 0 && rank[a] > rank[t]) || (i + 2 < tour.size() && rank[t] > rank[b]))) continue;
                double add = o.centreDist(a, t) + o.centreDist(t, b) - o.centreDist(a, b);
                if (add < bestAdd) {
                    bestAdd = add;
                    bestPos = i + 1;
                }
            }
            tour.add(bestPos, t);
        }
        return new ArrayList<>(tour.subList(1, tour.size() - 1));
    }

    /** 2-opt and or-opt, first improvement, Viterbi-scored, neighbour-restricted. */
    static final class Search {
        final LegOracle o;
        final PlainViterbi vit;
        final long budgetNs;
        long deadline;
        java.util.function.LongSupplier clock = System::nanoTime;
        List<Integer> order;
        int moves, evals;
        boolean timedOut;
        long seedMs;

        /** Ordered groups: rank per node; moves that break non-decreasing rank are not generated. null = free. */
        int[] rank;

        boolean monotone(List<Integer> ord) {
            if (rank == null) return true;
            for (int i = 1; i < ord.size(); i++) if (rank[ord.get(i - 1)] > rank[ord.get(i)]) return false;
            return true;
        }

        Search(LegOracle o, PlainViterbi vit, long budgetNs) {
            this.o = o;
            this.vit = vit;
            this.budgetNs = budgetNs;
        }

        PlainViterbi.Result improve(List<Integer> start) {
            order = start;
            PlainViterbi.Result best = vit.solve(o, withHome(order));
            evals++;
            // the seed fills the neighbour bundles (most of the table); the budget is for the moves after it
            deadline = clock.getAsLong() + budgetNs;
            boolean improved = true;
            while (improved) {
                improved = false;
                for (List<Integer> cand : neighbourhood(order)) {
                    if (clock.getAsLong() > deadline) {
                        timedOut = true;
                        return best;
                    }
                    PlainViterbi.Result r = vit.solve(o, withHome(cand));
                    evals++;
                    if (r.score < best.score - 1e-6) {
                        best = r;
                        order = cand;
                        moves++;
                        improved = true;
                        break;
                    }
                }
            }
            return best;
        }

        /** Candidate orders whose new adjacencies are all near (k nearest or home). */
        List<List<Integer>> neighbourhood(List<Integer> ord) {
            List<List<Integer>> out = new ArrayList<>();
            List<Integer> full = withHome(ord);
            int n = ord.size();
            // 2-opt: reverse ord[i..j]
            for (int i = 0; i < n; i++)
                for (int j = i + 1; j < n; j++) {
                    int a = full.get(i), b = full.get(i + 1), c = full.get(j + 1), d = full.get(j + 2);
                    if (!o.near(a, c) || !o.near(b, d)) continue;
                    List<Integer> l = new ArrayList<>(ord);
                    Collections.reverse(l.subList(i, j + 1));
                    if (monotone(l)) out.add(l);
                }
            // or-opt: move a segment of 1..3 elsewhere, forward or reversed
            for (int len = 1; len <= 3; len++)
                for (int i = 0; i + len <= n; i++) {
                    List<Integer> seg = new ArrayList<>(ord.subList(i, i + len));
                    List<Integer> rest = new ArrayList<>(ord.subList(0, i));
                    rest.addAll(ord.subList(i + len, n));
                    List<Integer> restFull = withHome(rest);
                    for (int p = 0; p <= rest.size(); p++) {
                        if (p == i) continue;
                        for (int rev = 0; rev < 2; rev++) {
                            List<Integer> s = new ArrayList<>(seg);
                            if (rev == 1) {
                                if (len == 1) continue;
                                Collections.reverse(s);
                            }
                            int before = restFull.get(p), after = restFull.get(p + 1);
                            if (!o.near(before, s.get(0)) || !o.near(s.get(s.size() - 1), after)) continue;
                            List<Integer> l = new ArrayList<>(rest);
                            l.addAll(p, s);
                            if (monotone(l)) out.add(l);
                        }
                    }
                }
            return out;
        }
    }

    /** Is the tile at position pos (in the order with home) crossed by a leg that does not start or end at it? */
    static boolean crossedByOtherLeg(LegOracle o, PlainViterbi.Result r, int tile, int pos) {
        TileMath.Tile t = o.tiles.get(tile);
        for (int li = 0; li < r.legs.size(); li++) {
            if (li == pos - 1 || li == pos) continue; // leg li goes from position li to li+1
            if (Foundation.fetched(t, o.f.geometry(r.legs.get(li).path))) return true;
        }
        return false;
    }

    static boolean allCrossed(LegOracle o, PlainViterbi.Result r, List<Integer> tiles) {
        if (!Double.isFinite(r.score)) return false;
        List<List<double[]>> geoms = new ArrayList<>();
        for (LegOracle.Leg l : r.legs) geoms.add(o.f.geometry(l.path));
        for (int tile : tiles) {
            boolean hit = false;
            for (List<double[]> g : geoms) if (Foundation.fetched(o.tiles.get(tile), g)) {
                hit = true;
                break;
            }
            if (!hit) return false;
        }
        return true;
    }

    static List<String> keys(LegOracle o, List<Integer> tiles) {
        List<String> l = new ArrayList<>();
        for (int t : tiles) l.add(o.tiles.get(t).key);
        return l;
    }
}
