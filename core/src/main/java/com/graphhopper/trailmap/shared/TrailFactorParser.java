/*
 * Trailmap - TrailFactorParser
 *
 * Computes trail_factor from OSM tags: width, trail_visibility, obstacle, smoothness.
 * The factor is the minimum of four sub-factors, each defaulting to 1.0 when the
 * relevant tag is absent.
 *
 * Width and visibility apply only to paths/tracks. Smoothness applies to all ways.
 * Smoothness penalties are moderated when mtb:scale is present to avoid double-penalty.
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.reader.ReaderWay;
import com.graphhopper.routing.ev.DecimalEncodedValue;
import com.graphhopper.routing.ev.EdgeIntAccess;
import com.graphhopper.routing.util.parsers.TagParser;
import com.graphhopper.storage.IntsRef;

public class TrailFactorParser implements TagParser {

    private final DecimalEncodedValue trailFactorEnc;

    public TrailFactorParser(DecimalEncodedValue trailFactorEnc) {
        this.trailFactorEnc = trailFactorEnc;
    }

    @Override
    public void handleWayTags(int edgeId, EdgeIntAccess edgeIntAccess, ReaderWay way, IntsRef relationFlags) {
        double factor = computeTrailFactor(way);
        trailFactorEnc.setDecimal(false, edgeId, edgeIntAccess, factor);
    }

    public double computeTrailFactor(ReaderWay way) {
        String highway = way.getTag("highway");
        boolean isPathOrTrack = "path".equals(highway) || "track".equals(highway);
        boolean isPath = "path".equals(highway);

        double widthFactor = isPathOrTrack ? computeWidthFactor(way) : 1.0;
        double visibilityFactor = isPath ? computeVisibilityFactor(way) : 1.0;
        double vegetationFactor = isPathOrTrack ? computeVegetationFactor(way) : 1.0;
        double smoothnessFactor = computeSmoothnessFactor(way);

        double combined = Math.min(Math.min(widthFactor, visibilityFactor),
                Math.min(vegetationFactor, smoothnessFactor));

        // Clamp to [0, 1] and round to 0.05 steps
        combined = Math.max(0.05, Math.round(combined * 20.0) / 20.0);
        return Math.min(1.0, combined);
    }

    /**
     * Width factor for paths and tracks.
     * Finnish OSM mapping convention: 0.6m is the thin/normal path threshold.
     * Below 0.6m = thin path, below 0.3m = essentially pushing.
     */
    private double computeWidthFactor(ReaderWay way) {
        String widthStr = way.getTag("width");
        if (widthStr == null) {
            return 1.0;
        }

        double width = parseWidth(widthStr);
        if (Double.isNaN(width) || width <= 0) {
            return 1.0;
        }

        if (width >= 0.6) {
            return 1.0;
        } else if (width >= 0.5) {
            return 0.80;
        } else if (width >= 0.4) {
            return 0.60;
        } else if (width >= 0.3) {
            return 0.40;
        } else {
            return 0.20;
        }
    }

    /**
     * Parse width value from OSM tag string.
     * Handles formats like "0.5", "1.2 m", "0.5m".
     */
    private static double parseWidth(String widthStr) {
        try {
            // Strip common suffixes
            String cleaned = widthStr.replaceAll("[^0-9.]", "");
            if (cleaned.isEmpty())
                return Double.NaN;
            return Double.parseDouble(cleaned);
        } catch (NumberFormatException e) {
            return Double.NaN;
        }
    }

    /**
     * Visibility factor for paths only.
     * Whitelisted values (excellent, good, intermediate) = no penalty.
     * Anything else = near-pushing penalty.
     */
    private double computeVisibilityFactor(ReaderWay way) {
        String visibility = way.getTag("trail_visibility");
        if (visibility == null) {
            return 1.0;
        }
        switch (visibility) {
            case "excellent":
            case "good":
                return 1.0;
            case "intermediate":
                return 0.80;
            case "bad":
                return 0.60;
            default:
                // horrible, no, or any other value
                return 0.35;
        }
    }

    /**
     * Vegetation factor for paths and tracks.
     */
    private double computeVegetationFactor(ReaderWay way) {
        String obstacle = way.getTag("obstacle");
        if ("vegetation".equals(obstacle)) {
            return 0.40;
        }
        return 1.0;
    }

    /**
     * Smoothness factor for all ways.
     * Uses moderated penalties when mtb:scale is present (to avoid double-penalty
     * since mtb:scale already captures trail difficulty).
     */
    private double computeSmoothnessFactor(ReaderWay way) {
        String smoothness = way.getTag("smoothness");
        if (smoothness == null) {
            return 1.0;
        }

        boolean hasMtbScale = way.getTag("mtb:scale") != null;

        if (hasMtbScale) {
            // Moderated: mtb:scale carries the primary difficulty signal
            switch (smoothness) {
                case "horrible":
                    return 0.95;
                case "very_horrible":
                    return 0.85;
                case "impassable":
                    return 0.10;
                default:
                    return 1.0;
            }
        } else {
            // Full: smoothness is the primary difficulty signal
            switch (smoothness) {
                case "very_bad":
                    return 0.90;
                case "horrible":
                    return 0.85;
                case "very_horrible":
                    return 0.60;
                case "impassable":
                    return 0.10;
                default:
                    return 1.0;
            }
        }
    }

}
