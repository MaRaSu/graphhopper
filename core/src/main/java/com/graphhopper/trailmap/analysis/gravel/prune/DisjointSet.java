/*
 * Trailmap - Gravel Segment Analysis
 *
 * Minimal union-find (disjoint-set) with path-halving + union-by-size. Shared by the network
 * filter (2-edge-connected components / appendage grouping) and the connector pre-pass.
 */
package com.graphhopper.trailmap.analysis.gravel.prune;

/** Compact union-find over {@code [0, n)}. */
final class DisjointSet {

    private final int[] parent;
    private final int[] size;

    DisjointSet(int n) {
        parent = new int[n];
        size = new int[n];
        for (int i = 0; i < n; i++) {
            parent[i] = i;
            size[i] = 1;
        }
    }

    int find(int x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    }

    void union(int a, int b) {
        int ra = find(a), rb = find(b);
        if (ra == rb) return;
        if (size[ra] < size[rb]) {
            int t = ra; ra = rb; rb = t;
        }
        parent[rb] = ra;
        size[ra] += size[rb];
    }
}
