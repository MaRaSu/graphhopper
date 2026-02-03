/*
 * Trailmap - Custom ImportRegistry for Trailmap encoded values
 *
 * Registers Trailmap-specific encoded values and parsers:
 * - gravel_scale: GravelScale enum computed from multiple OSM tags
 * - gravel_scale_num: Numeric version of gravel_scale for formulas
 * - gravel_base_priority: Pre-computed priority for gravel profile
 * - mtb_scale: MtbScale enum for MTB technical difficulty
 * - mtb_scale_num: Numeric version of mtb_scale for formulas
 * - mtb_base_priority: Pre-computed priority for MTB profile
 * - predicted_surface: PredictedSurface for UI display (profile-independent)
 * - predicted_highway: PredictedHighway for UI display (profile-independent)
 * - issue_*: Boolean flags for route issues (returned as path details)
 *
 * To use this registry, configure GraphHopper with:
 *   graph.encoded_values: gravel_scale, gravel_scale_num, gravel_base_priority, mtb_scale, mtb_scale_num, mtb_base_priority, ...
 */
package com.graphhopper.trailmap.shared;

import com.graphhopper.routing.ev.BooleanEncodedValue;
import com.graphhopper.routing.ev.DecimalEncodedValue;
import com.graphhopper.routing.ev.DefaultImportRegistry;
import com.graphhopper.routing.ev.ImportUnit;

/**
 * Import registry that extends DefaultImportRegistry with Trailmap-specific encoders.
 */
public class TrailmapImportRegistry extends DefaultImportRegistry {

    @Override
    public ImportUnit createImportUnit(String name) {
        // Check for Trailmap-specific encoded values first

        // === Gravel Scale (enum) ===
        if (GravelScale.KEY.equals(name)) {
            return ImportUnit.create(name,
                props -> GravelScale.create(),
                (lookup, props) -> new GravelScaleParser(
                    lookup.getEnumEncodedValue(GravelScale.KEY, GravelScale.class))
            );
        }

        // === Gravel Scale Numeric ===
        if (GravelScaleNum.KEY.equals(name)) {
            return ImportUnit.create(name,
                props -> GravelScaleNum.create(),
                (lookup, props) -> new GravelScaleNumParser(
                    lookup.getDecimalEncodedValue(GravelScaleNum.KEY))
            );
        }

        // === Gravel Base Priority ===
        if (GravelBasePriority.KEY.equals(name)) {
            return ImportUnit.create(name,
                props -> GravelBasePriority.create(),
                (lookup, props) -> new GravelBasePriorityParser(
                    lookup.getDecimalEncodedValue(GravelBasePriority.KEY))
            );
        }

        // === MTB Scale (enum) ===
        if (MtbScale.KEY.equals(name)) {
            return ImportUnit.create(name,
                props -> MtbScale.create(),
                (lookup, props) -> new MtbScaleParser(
                    lookup.getEnumEncodedValue(MtbScale.KEY, MtbScale.class))
            );
        }

        // === MTB Scale Numeric ===
        if (MtbScaleNum.KEY.equals(name)) {
            return ImportUnit.create(name,
                props -> MtbScaleNum.create(),
                (lookup, props) -> new MtbScaleNumParser(
                    lookup.getDecimalEncodedValue(MtbScaleNum.KEY))
            );
        }

        // === MTB Base Priority ===
        if (MtbBasePriority.KEY.equals(name)) {
            return ImportUnit.create(name,
                props -> MtbBasePriority.create(),
                (lookup, props) -> new MtbBasePriorityParser(
                    lookup.getDecimalEncodedValue(MtbBasePriority.KEY))
            );
        }

        // === Predicted Surface ===
        if (PredictedSurface.KEY.equals(name)) {
            return ImportUnit.create(name,
                props -> PredictedSurface.create(),
                (lookup, props) -> new PredictedSurfaceParser(
                    lookup.getEnumEncodedValue(PredictedSurface.KEY, PredictedSurface.class))
            );
        }

        // === Predicted Highway ===
        if (PredictedHighway.KEY.equals(name)) {
            return ImportUnit.create(name,
                props -> PredictedHighway.create(),
                (lookup, props) -> new PredictedHighwayParser(
                    lookup.getEnumEncodedValue(PredictedHighway.KEY, PredictedHighway.class))
            );
        }

        // === Piste Type ===
        if (PisteType.KEY.equals(name)) {
            return ImportUnit.create(name,
                props -> PisteType.create(),
                (lookup, props) -> new PisteTypeParser(
                    lookup.getEnumEncodedValue(PisteType.KEY, PisteType.class))
            );
        }

        // === MTB Winter ===
        if (MtbWinter.KEY.equals(name)) {
            return ImportUnit.create(name,
                props -> MtbWinter.create(),
                (lookup, props) -> new MtbWinterParser(
                    lookup.getEnumEncodedValue(MtbWinter.KEY, MtbWinter.class))
            );
        }

        // === Route Issues ===
        // All issue flags are set by a single RouteIssuesParser.
        // The parser is registered with issue_biking_blocked and declares dependencies
        // on all other issue encoded values to ensure they exist before parser creation.

        if (RouteIssues.KEY_BIKING_BLOCKED.equals(name)) {
            // This is the primary key - it creates the parser that handles ALL issues
            return ImportUnit.create(name,
                props -> RouteIssues.createBikingBlocked(),
                (lookup, props) -> new RouteIssuesParser(
                    lookup.getBooleanEncodedValue(RouteIssues.KEY_BIKING_BLOCKED),
                    lookup.getBooleanEncodedValue(RouteIssues.KEY_FOOT_BLOCKED),
                    lookup.getBooleanEncodedValue(RouteIssues.KEY_NARROW),
                    lookup.getBooleanEncodedValue(RouteIssues.KEY_POOR_VISIBILITY),
                    lookup.getBooleanEncodedValue(RouteIssues.KEY_VEGETATION),
                    lookup.getBooleanEncodedValue(RouteIssues.KEY_MUD),
                    lookup.getBooleanEncodedValue(RouteIssues.KEY_UNKNOWN_PATH),
                    lookup.getBooleanEncodedValue(RouteIssues.KEY_UNKNOWN_TRACK),
                    lookup.getBooleanEncodedValue(RouteIssues.KEY_FERRY)),
                // Declare dependencies so ImportUnitSorter processes these first
                RouteIssues.KEY_FOOT_BLOCKED,
                RouteIssues.KEY_NARROW,
                RouteIssues.KEY_POOR_VISIBILITY,
                RouteIssues.KEY_VEGETATION,
                RouteIssues.KEY_MUD,
                RouteIssues.KEY_UNKNOWN_PATH,
                RouteIssues.KEY_UNKNOWN_TRACK,
                RouteIssues.KEY_FERRY
            );
        }

        if (RouteIssues.KEY_FOOT_BLOCKED.equals(name)) {
            return ImportUnit.create(name,
                props -> RouteIssues.createFootBlocked(),
                null  // No parser - handled by RouteIssuesParser via KEY_BIKING_BLOCKED
            );
        }

        if (RouteIssues.KEY_NARROW.equals(name)) {
            return ImportUnit.create(name,
                props -> RouteIssues.createNarrow(),
                null
            );
        }

        if (RouteIssues.KEY_POOR_VISIBILITY.equals(name)) {
            return ImportUnit.create(name,
                props -> RouteIssues.createPoorVisibility(),
                null
            );
        }

        if (RouteIssues.KEY_VEGETATION.equals(name)) {
            return ImportUnit.create(name,
                props -> RouteIssues.createVegetation(),
                null
            );
        }

        if (RouteIssues.KEY_MUD.equals(name)) {
            return ImportUnit.create(name,
                props -> RouteIssues.createMud(),
                null
            );
        }

        if (RouteIssues.KEY_UNKNOWN_PATH.equals(name)) {
            return ImportUnit.create(name,
                props -> RouteIssues.createUnknownPath(),
                null
            );
        }

        if (RouteIssues.KEY_UNKNOWN_TRACK.equals(name)) {
            return ImportUnit.create(name,
                props -> RouteIssues.createUnknownTrack(),
                null
            );
        }

        if (RouteIssues.KEY_FERRY.equals(name)) {
            return ImportUnit.create(name,
                props -> RouteIssues.createFerry(),
                null
            );
        }

        // Fall back to default registry for standard encoded values
        return super.createImportUnit(name);
    }
}
