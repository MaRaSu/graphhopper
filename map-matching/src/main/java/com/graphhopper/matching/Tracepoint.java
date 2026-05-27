/*
 *  Licensed to GraphHopper GmbH under one or more contributor
 *  license agreements. See the NOTICE file distributed with this work for
 *  additional information regarding copyright ownership.
 *
 *  GraphHopper GmbH licenses this file to you under the Apache License,
 *  Version 2.0 (the "License"); you may not use this file except in
 *  compliance with the License. You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
package com.graphhopper.matching;

import com.graphhopper.util.shapes.GHPoint;

/**
 * Represents a single tracepoint in the map matching result.
 * Provides 1:1 correspondence with input GPS observations, similar to OSRM's tracepoints.
 *
 * Each tracepoint has:
 * - matched: true if snap data was successfully computed, false if no candidates found
 * - filtered: true if the point was skipped during Viterbi (too close to previous point),
 *             false if the point participated in the Viterbi algorithm
 *
 * Both filtered and non-filtered points can have snap data (matched=true).
 */
public class Tracepoint {
    private final int originalIndex;
    private final GHPoint originalPoint;
    private final boolean matched;
    private final boolean filtered;
    private final GHPoint snappedPoint;
    private final Double distance;
    private final Integer edgeId;
    private final Double distanceFromPrevious;

    /**
     * Creates a tracepoint with snap data.
     *
     * @param originalIndex index in original input array
     * @param originalPoint original GPS coordinates
     * @param filtered true if this point was filtered out before Viterbi
     * @param snappedPoint snapped coordinates on road
     * @param distance distance from original to snapped point in meters
     * @param edgeId edge ID of matched road segment
     * @param distanceFromPrevious matched-path length (meters) of the matcher's transition
     *                             from the previous non-filtered tracepoint to this one;
     *                             null for the first non-filtered tracepoint and for filtered
     *                             tracepoints (which did not participate in Viterbi).
     *                             Mirrors OSRM's tracepoint distance_to_previous.
     */
    public Tracepoint(int originalIndex, GHPoint originalPoint, boolean filtered,
                      GHPoint snappedPoint, double distance, int edgeId,
                      Double distanceFromPrevious) {
        this.originalIndex = originalIndex;
        this.originalPoint = originalPoint;
        this.matched = true;
        this.filtered = filtered;
        this.snappedPoint = snappedPoint;
        this.distance = distance;
        this.edgeId = edgeId;
        this.distanceFromPrevious = distanceFromPrevious;
    }

    /**
     * Creates a tracepoint without snap data (no candidates found).
     *
     * @param originalIndex index in original input array
     * @param originalPoint original GPS coordinates
     * @param filtered true if this point was filtered out before Viterbi
     */
    public Tracepoint(int originalIndex, GHPoint originalPoint, boolean filtered) {
        this.originalIndex = originalIndex;
        this.originalPoint = originalPoint;
        this.matched = false;
        this.filtered = filtered;
        this.snappedPoint = null;
        this.distance = null;
        this.edgeId = null;
        this.distanceFromPrevious = null;
    }

    /**
     * @return index in the original input observation array
     */
    public int getOriginalIndex() {
        return originalIndex;
    }

    /**
     * @return original GPS coordinates from input
     */
    public GHPoint getOriginalPoint() {
        return originalPoint;
    }

    /**
     * @return true if snap data is available, false if no snap candidates were found
     */
    public boolean isMatched() {
        return matched;
    }

    /**
     * @return true if this point was filtered out before Viterbi (too close to previous point),
     *         false if it participated in the Viterbi algorithm
     */
    public boolean isFiltered() {
        return filtered;
    }

    /**
     * @return snapped coordinates on the matched road, or null if not matched
     */
    public GHPoint getSnappedPoint() {
        return snappedPoint;
    }

    /**
     * @return distance in meters from original point to snapped point, or null if not matched
     */
    public Double getDistance() {
        return distance;
    }

    /**
     * @return edge ID of the matched road segment, or null if not matched
     */
    public Integer getEdgeId() {
        return edgeId;
    }

    /**
     * @return matched-path length (meters) of the matcher's HMM transition from the
     *         previous non-filtered tracepoint to this one; null for the first
     *         non-filtered tracepoint and for filtered tracepoints. Mirrors OSRM's
     *         tracepoint distance_to_previous and is the authoritative per-leg
     *         matched-path distance computed by the Viterbi algorithm.
     */
    public Double getDistanceFromPrevious() {
        return distanceFromPrevious;
    }

    @Override
    public String toString() {
        if (matched) {
            return "Tracepoint{index=" + originalIndex +
                   ", matched=true, filtered=" + filtered +
                   ", distance=" + distance +
                   ", distanceFromPrevious=" + distanceFromPrevious +
                   ", snapped=" + snappedPoint + "}";
        } else {
            return "Tracepoint{index=" + originalIndex +
                   ", matched=false, filtered=" + filtered + "}";
        }
    }
}
