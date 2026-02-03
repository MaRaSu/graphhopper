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
package com.graphhopper.routing.util;

import com.graphhopper.reader.ReaderWay;

/**
 * Filter interface for determining if an area (closed polygon) way
 * should be included in the routing graph.
 * <p>
 * This enables routing through area polygons like parking lots, pedestrian plazas,
 * and parks by including their boundary edges in the routing graph.
 * <p>
 * Implementations should check:
 * 1. Whether the way is a closed polygon (first node == last node)
 * 2. Whether the area type is passable based on OSM tags
 * 3. Whether access restrictions allow routing through the area
 */
public interface AreaWayFilter {
    /**
     * Determines if this way represents a routable area polygon.
     *
     * @param way the OSM way to check
     * @return true if this way is an area that should be included in routing
     */
    boolean acceptAreaWay(ReaderWay way);

    /**
     * No-op implementation that rejects all areas.
     * Used as default when area routing is disabled.
     */
    AreaWayFilter NONE = way -> false;
}
