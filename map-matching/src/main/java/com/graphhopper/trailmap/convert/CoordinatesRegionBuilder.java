package com.graphhopper.trailmap.convert;

import com.graphhopper.matching.Observation;
import com.graphhopper.util.PointList;
import com.graphhopper.util.RamerDouglasPeucker;

import java.util.ArrayList;
import java.util.List;

/**
 * Stage 4: build a simplified coordinates list for an {@link TrackRegion.Unmatched} region.
 *
 * <p>Takes the raw observations in the region's index range and applies Ramer–Douglas–Peucker
 * simplification with a configurable epsilon (in meters — converted from the legacy
 * degree-units epsilon to keep parity with the client default).
 */
public class CoordinatesRegionBuilder {

    public List<ConvertTrackResponse.Coordinates> build(List<Observation> observations,
                                                        TrackRegion.Unmatched region,
                                                        double simplifyEpsM) {
        return buildRange(observations, region.firstObservation(), region.lastObservation(),
                simplifyEpsM);
    }

    /**
     * Build coordinates for an observation range [{@code fromObs}..{@code toObs}] inclusive,
     * applying RDP simplification with {@code simplifyEpsM} (meters). Used by the
     * optimizer's coords-escalation path, where a coordinates leg appears inside a
     * matched region between two chosen waypoint observations.
     */
    public List<ConvertTrackResponse.Coordinates> buildRange(List<Observation> observations,
                                                             int fromObs, int toObs,
                                                             double simplifyEpsM) {
        int n = toObs - fromObs + 1;
        if (n <= 0) return List.of();

        PointList pl = new PointList(n, false);
        for (int i = fromObs; i <= toObs; i++) {
            pl.add(observations.get(i).getPoint().lat, observations.get(i).getPoint().lon);
        }
        if (n > 2 && simplifyEpsM > 0) {
            new RamerDouglasPeucker().setMaxDistance(simplifyEpsM).simplify(pl);
        }

        List<ConvertTrackResponse.Coordinates> out = new ArrayList<>(pl.size());
        for (int i = 0; i < pl.size(); i++) {
            if (Double.isNaN(pl.getLat(i))) continue;
            out.add(new ConvertTrackResponse.Coordinates(pl.getLat(i), pl.getLon(i)));
        }
        return out;
    }
}
