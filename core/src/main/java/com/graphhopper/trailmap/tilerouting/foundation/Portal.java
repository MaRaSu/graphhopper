package com.graphhopper.trailmap.tilerouting.foundation;

/**
 * A directed entry into a tile's inset square: a point on a graph edge just inside the inset border, travelled in
 * {@link #key}'s direction (from {@link #fromNode} to {@link #toNode}). Riding the edge through this point fetches
 * the tile. Home is modelled with the same class (one portal per direction of its snapped edge).
 */
public final class Portal {
    public String tileKey;
    public int edge;
    /** GraphHopper edge key: edge·2 (+1 against storage direction). */
    public int key;
    public int fromNode, toNode;
    public double lat, lon;
    /** Travel direction at the point, degrees from north. */
    public double heading;
    /** Position of the point along the directed edge, 0 = fromNode, 1 = toNode (by geometry length). */
    public double frac;
    /** Weight of the whole directed edge (profile weighting). */
    public double edgeWeight;
    public double edgeLenM;
    /** Inset border the edge enters through: N, E, S, W. */
    public String side;
    /** Length of the edge's part inside the inset from the entry (to its exit or the edge end). */
    public double chordM;
    /** The edge leaves the inset again within {@link #chordM} (a clip, not a way into the tile). */
    public boolean exitsInEdge;
    /** Riding on from this point ends in a dead-end stub (the route must turn around). */
    public boolean deadEnd;
    /** Metres from the point to the end of the stub (dead-end portals). */
    public double stubM;
    public String name = "";
    /**
     * Walk portal (round 3): this point A is on a road outside a tile no rideable way enters; the tile is fetched by
     * walking straight to (walkLat, walkLon) inside its inset and back. walkM = one-way metres A→T; 0 = an ordinary
     * portal.
     */
    public double walkM;
    public double walkLat, walkLon;

    public double weightPerM() {
        return edgeLenM > 0 ? edgeWeight / edgeLenM : Double.POSITIVE_INFINITY;
    }

    @Override
    public String toString() {
        return String.format(java.util.Locale.ROOT, "%s e%d%s %s %.5f,%.5f h=%.0f chord=%.0f%s%s '%s'", tileKey, edge,
                (key & 1) == 1 ? "r" : "f", side, lat, lon, heading, chordM, exitsInEdge ? " clip" : "",
                deadEnd ? String.format(java.util.Locale.ROOT, " dead-end(%.0fm)", stubM) : "", name)
                + (walkM > 0 ? String.format(java.util.Locale.ROOT, " WALK %.0fm to %.6f,%.6f", walkM, walkLat, walkLon) : "");
    }
}
