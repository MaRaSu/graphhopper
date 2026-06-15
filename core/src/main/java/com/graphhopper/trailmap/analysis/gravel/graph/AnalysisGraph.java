/*
 * Trailmap - Gravel Segment Analysis
 *
 * The working graph: a compact in-memory, undirected projection of the BaseGraph
 * restricted to TARGET ∪ ANCHOR edges. See docs/gravel_segments_design.md §5.
 */
package com.graphhopper.trailmap.analysis.gravel.graph;

import com.graphhopper.trailmap.analysis.gravel.role.EdgeRole;

import java.util.ArrayList;
import java.util.List;

/**
 * Lightweight undirected graph used by Phases C–E. Nodes reuse GraphHopper node IDs
 * ({@code 0 .. nodeCount-1}); each kept edge ("analysis edge") carries the attributes the
 * later phases need. Adjacency is symmetric (every analysis edge is listed under both
 * endpoints) and oneway is never consulted — connectivity, not round-trippability.
 *
 * <p>Edges support soft removal ({@link #remove(int)} / {@link #isRemoved(int)}) so
 * Phases C and D can prune in place and later phases see only survivors. Build via
 * {@link Builder}.</p>
 */
public class AnalysisGraph {

    private final int nodeCount;
    private final int[] ghEdgeId;
    private final int[] nodeA;
    private final int[] nodeB;
    private final EdgeRole[] role;
    private final double[] lengthM;
    private final int[] osmWayId;
    private final String[] name;
    private final String[] ref;
    private final int[] nameHash;
    private final boolean[] backbone;
    private final boolean[] removed;

    // CSR adjacency: for node n, analysis-edge ids are adjEdges[adjStart[n] .. adjStart[n+1]).
    private final int[] adjStart;
    private final int[] adjEdges;

    private AnalysisGraph(int nodeCount, int[] ghEdgeId, int[] nodeA, int[] nodeB, EdgeRole[] role,
                          double[] lengthM, int[] osmWayId, String[] name, String[] ref, int[] nameHash,
                          boolean[] backbone, int[] adjStart, int[] adjEdges) {
        this.nodeCount = nodeCount;
        this.ghEdgeId = ghEdgeId;
        this.nodeA = nodeA;
        this.nodeB = nodeB;
        this.role = role;
        this.lengthM = lengthM;
        this.osmWayId = osmWayId;
        this.name = name;
        this.ref = ref;
        this.nameHash = nameHash;
        this.backbone = backbone;
        this.removed = new boolean[ghEdgeId.length];
        this.adjStart = adjStart;
        this.adjEdges = adjEdges;
    }

    public int nodeCount() {
        return nodeCount;
    }

    public int edgeCount() {
        return ghEdgeId.length;
    }

    // --- CSR adjacency access (hot path; avoid allocating) ---

    /** Start index into the adjacency array for {@code node}. */
    public int adjBegin(int node) {
        return adjStart[node];
    }

    /** End index (exclusive) into the adjacency array for {@code node}. */
    public int adjEnd(int node) {
        return adjStart[node + 1];
    }

    /** Analysis-edge id stored at adjacency slot {@code i} ({@code adjBegin <= i < adjEnd}). */
    public int adjEdge(int i) {
        return adjEdges[i];
    }

    /** The endpoint of analysis edge {@code e} that is not {@code node}. */
    public int other(int e, int node) {
        return nodeA[e] == node ? nodeB[e] : nodeA[e];
    }

    // --- per-edge accessors ---

    public int ghEdgeId(int e) {
        return ghEdgeId[e];
    }

    public int nodeA(int e) {
        return nodeA[e];
    }

    public int nodeB(int e) {
        return nodeB[e];
    }

    public EdgeRole role(int e) {
        return role[e];
    }

    /** Reassign an edge's role. Used by the Phase 2 connector pre-pass (CONNECTOR → ANCHOR). */
    public void setRole(int e, EdgeRole r) {
        role[e] = r;
    }

    public double lengthM(int e) {
        return lengthM[e];
    }

    public int osmWayId(int e) {
        return osmWayId[e];
    }

    public String name(int e) {
        return name[e];
    }

    public String ref(int e) {
        return ref[e];
    }

    public int nameHash(int e) {
        return nameHash[e];
    }

    /**
     * Whether this edge is part of the <b>real-road backbone</b> — an ANCHOR edge whose road class
     * is a drivable real road (per {@code GravelAnalysisConfig.backboneRoadClasses}). The pendant
     * pruner treats backbone nodes as the network the gravel must reach: a gravel cluster reachable
     * from a backbone node is anchored; one reaching only rough tracks / other gravel is a dead-end
     * cluster judged by its own size. Always {@code false} for TARGET edges.
     */
    public boolean isBackbone(int e) {
        return backbone[e];
    }

    public boolean isRemoved(int e) {
        return removed[e];
    }

    public void remove(int e) {
        removed[e] = true;
    }

    /** Builder accumulating analysis edges, then materializing CSR adjacency. */
    public static class Builder {
        private final List<int[]> endpoints = new ArrayList<>();   // {ghEdgeId, nodeA, nodeB, osmWayId, nameHash, backbone}
        private final List<EdgeRole> roles = new ArrayList<>();
        private final List<Double> lengths = new ArrayList<>();
        private final List<String> names = new ArrayList<>();
        private final List<String> refs = new ArrayList<>();
        private int maxNode = -1;

        /**
         * Add one kept (TARGET or ANCHOR) analysis edge, not part of the real-road backbone.
         *
         * @param name the raw edge name (may be null/empty for unnamed edges)
         * @param ref  the raw edge ref (may be null/empty); used only when requireSameRef is set
         */
        public Builder addEdge(int ghEdgeId, int nodeA, int nodeB, EdgeRole role,
                               double lengthM, int osmWayId, String name, String ref, int nameHash) {
            return addEdge(ghEdgeId, nodeA, nodeB, role, lengthM, osmWayId, name, ref, nameHash, false);
        }

        /**
         * Add one kept (TARGET or ANCHOR) analysis edge.
         *
         * @param backbone whether this is a real-road backbone edge (see {@link #isBackbone(int)})
         */
        public Builder addEdge(int ghEdgeId, int nodeA, int nodeB, EdgeRole role,
                               double lengthM, int osmWayId, String name, String ref, int nameHash,
                               boolean backbone) {
            if (role == EdgeRole.IGNORED)
                throw new IllegalArgumentException("IGNORED edges must not be added to the working graph");
            endpoints.add(new int[]{ghEdgeId, nodeA, nodeB, osmWayId, nameHash, backbone ? 1 : 0});
            roles.add(role);
            lengths.add(lengthM);
            names.add(name);
            refs.add(ref);
            maxNode = Math.max(maxNode, Math.max(nodeA, nodeB));
            return this;
        }

        /**
         * @param nodeCount total node count of the source graph (so isolated high-id nodes are
         *                  represented); if {@code <= maxNode} it is raised to {@code maxNode + 1}.
         */
        public AnalysisGraph build(int nodeCount) {
            int n = Math.max(nodeCount, maxNode + 1);
            int m = endpoints.size();
            int[] ghEdgeId = new int[m];
            int[] nodeA = new int[m];
            int[] nodeB = new int[m];
            int[] osmWayId = new int[m];
            int[] nameHash = new int[m];
            boolean[] backbone = new boolean[m];
            EdgeRole[] role = new EdgeRole[m];
            double[] lengthM = new double[m];
            String[] name = new String[m];
            String[] ref = new String[m];

            int[] degree = new int[n + 1];
            for (int e = 0; e < m; e++) {
                int[] ep = endpoints.get(e);
                ghEdgeId[e] = ep[0];
                nodeA[e] = ep[1];
                nodeB[e] = ep[2];
                osmWayId[e] = ep[3];
                nameHash[e] = ep[4];
                backbone[e] = ep[5] != 0;
                role[e] = roles.get(e);
                lengthM[e] = lengths.get(e);
                name[e] = names.get(e);
                ref[e] = refs.get(e);
                degree[ep[1]]++;
                degree[ep[2]]++;
            }

            // CSR offsets via prefix sum.
            int[] adjStart = new int[n + 1];
            int acc = 0;
            for (int v = 0; v < n; v++) {
                adjStart[v] = acc;
                acc += degree[v];
            }
            adjStart[n] = acc;

            int[] adjEdges = new int[acc];
            int[] cursor = adjStart.clone();
            for (int e = 0; e < m; e++) {
                adjEdges[cursor[nodeA[e]]++] = e;
                adjEdges[cursor[nodeB[e]]++] = e;
            }

            return new AnalysisGraph(n, ghEdgeId, nodeA, nodeB, role, lengthM, osmWayId, name, ref,
                    nameHash, backbone, adjStart, adjEdges);
        }

        /** Build sizing the node count from the largest endpoint seen. */
        public AnalysisGraph build() {
            return build(maxNode + 1);
        }
    }
}
