/*
 * Trailmap - Gravel Segment Analysis
 *
 * Edge role in the gravel-network extraction (Phase B of the design).
 * See docs/gravel_segments_design.md §3.
 */
package com.graphhopper.trailmap.analysis.gravel.role;

/**
 * The role an edge plays in the well-connected gravel-network extraction.
 *
 * <ul>
 *   <li>{@link #TARGET} — qualifying gravel; the only role ever emitted in output.</li>
 *   <li>{@link #ANCHOR} — a rideable non-gravel way that provides connectivity (and, for the
 *       real-road subset, anchors a non-dead-end). Never output and never pruned.</li>
 *   <li>{@link #CONNECTOR} — a <b>Phase 2</b> transient role: a normally-IGNORED way (a path or a
 *       gravel_scale-4 track) that may be admitted as a bounded connector chain linking otherwise
 *       separate gravel. Exists only between classification and the connector pre-pass, which
 *       resolves each chain to {@link #ANCHOR} (admitted) or removes it (too long / dead-ending).
 *       Only present when connectors are enabled; never output.</li>
 *   <li>{@link #IGNORED} — everything else (MTB trails, paths, rough/unknown). Excluded from the
 *       working graph entirely.</li>
 * </ul>
 *
 * The working graph for Phases C–E is {@code TARGET ∪ ANCHOR}, treated undirected.
 */
public enum EdgeRole {
    TARGET,
    ANCHOR,
    CONNECTOR,
    IGNORED
}
