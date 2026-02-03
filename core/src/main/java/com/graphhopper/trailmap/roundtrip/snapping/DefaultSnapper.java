/*
 * Trailmap - Enhanced Round-Trip Routing
 *
 * Default waypoint snapper - just finds the closest road.
 */
package com.graphhopper.trailmap.roundtrip.snapping;

import com.graphhopper.routing.ev.EncodedValueLookup;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.trailmap.roundtrip.config.RoundTripProfile;
import com.graphhopper.util.shapes.GHPoint;

/**
 * Default snapper that simply finds the closest road.
 * This is equivalent to the original GH behavior.
 */
public class DefaultSnapper implements WaypointSnapStrategy {

    @Override
    public String getName() {
        return "default";
    }

    @Override
    public boolean isProfileAware() {
        return false;
    }

    @Override
    public Snap findBestSnap(GHPoint targetPoint,
                             RoundTripProfile profile,
                             LocationIndex locationIndex,
                             EdgeFilter edgeFilter,
                             EncodedValueLookup encodedValueLookup) {
        Snap snap = locationIndex.findClosest(
            targetPoint.getLat(), targetPoint.getLon(), edgeFilter);

        if (snap.isValid()) {
            return snap;
        }
        return null;
    }
}
