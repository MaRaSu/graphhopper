/*
 * Trailmap - Gravel Segment Analysis
 *
 * Phase B EdgeFilter factories surfacing the role predicate as the standard
 * com.graphhopper.routing.util.EdgeFilter, so role-based selection plugs into any
 * GraphHopper traversal primitive unchanged. See docs/gravel_segments_design.md §6.
 */
package com.graphhopper.trailmap.analysis.gravel.role;

import com.graphhopper.routing.util.EdgeFilter;

/**
 * EdgeFilter factories over an {@link EdgeRoleClassifier}: {@code target()},
 * {@code anchor()}, and {@code working()} (= TARGET ∪ ANCHOR). These exist so the role
 * predicate is consumable by GraphHopper primitives that take an {@link EdgeFilter}; in
 * the main pipeline role is computed once during the {@code AnalysisGraph} build and
 * cached per analysis edge, so later phases read a field rather than re-evaluating.
 */
public final class RoleEdgeFilters {

    private final EdgeRoleClassifier classifier;

    public RoleEdgeFilters(EdgeRoleClassifier classifier) {
        this.classifier = classifier;
    }

    public EdgeFilter target() {
        return edge -> classifier.classify(edge) == EdgeRole.TARGET;
    }

    public EdgeFilter anchor() {
        return edge -> classifier.classify(edge) == EdgeRole.ANCHOR;
    }

    public EdgeFilter working() {
        return edge -> classifier.classify(edge) != EdgeRole.IGNORED;
    }
}
