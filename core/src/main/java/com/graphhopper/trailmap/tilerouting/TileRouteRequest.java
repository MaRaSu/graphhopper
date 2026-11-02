package com.graphhopper.trailmap.tilerouting;

import java.util.ArrayList;
import java.util.List;

/**
 * One tile-tour request: visit every requested map tile on a loop from the start. Defaults are the interim-v1
 * defaults the owner approved (2026-10-30).
 */
public class TileRouteRequest {

    /** Tile keys "x-y" at {@link #zoom}, in the user's order (the loop repair seeds from it; the order itself is free). */
    public List<String> tiles = new ArrayList<>();
    /** 14 (VeloViewer / squadrats) or 17 (squadratinhos). */
    public int zoom = 14;
    public double startLat = Double.NaN, startLon = Double.NaN;
    public String profile = "gravel";
    /** standard | strict (no hard / walk-only ways) | no_trails (no trails either). */
    public String safety = "standard";
    /** strokes | repair | tour_repair. */
    public String algorithm = "strokes";
    /** Loop repair's different-way-back lever: off | mild | strong (repair, tour_repair). */
    public String variety = "mild";
    /** Ordered groups of tile keys: visit group 1 first, then group 2, ...; tiles in no group last. Empty = free order. */
    public List<List<String>> groups = new ArrayList<>();
    /** Walk-in to a tile without a rideable way, one way, metres; NaN = the default 300 (also under no_trails: owner, A20 2026-10-31). */
    public double walkInMaxM = Double.NaN;
    /** W26: "v1" = the app's barrier flag (a walk-in never passes a closed barrier); null = as before. */
    public String barriers;

    /** Seeds the loop repair's random moves; null = the tile list (the same tiles give the same tour). */
    public String seedKey;

    /** Re-use polish: re-route legs so roads already ridden are avoided where a good option exists. */
    public boolean polish = false;
    /** How much dearer a road ridden by another leg is during the polish (5 = 5×). */
    public double polishStrength = 5;
    /** How much dearer trail and hard sections are during the polish (5 = 5×; 1 = not at all). */
    public double polishTrailFactor = 5;
    /**
     * Hand the tour over the way the app rebuilds it (W21, {@link TileHandover}): pins where plain /route would leave a
     * leg, and the geometry routed as the app routes it. Always on for the endpoint; tests that compare the solver's
     * own waypoints with stored prototype runs turn it off.
     */
    public boolean handover = true;
}
