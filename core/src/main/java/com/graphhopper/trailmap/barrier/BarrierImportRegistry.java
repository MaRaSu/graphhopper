package com.graphhopper.trailmap.barrier;

import com.graphhopper.routing.ev.ImportRegistry;
import com.graphhopper.routing.ev.ImportUnit;

import java.util.ArrayList;
import java.util.List;

/**
 * Adds the encoded values {@code bike_barrier} and {@code foot_barrier} to whatever import registry the server uses
 * (TrailmapImportRegistry), so that registry needs no change. Their parsers fill the shared {@link BarrierTable}.
 * They come with {@code bike_access} / {@code foot_access} (as a required import unit), so no config needs to list
 * them and the config files stay unchanged.
 */
public class BarrierImportRegistry implements ImportRegistry {

    private final ImportRegistry delegate;
    private final BarrierTable table;

    public BarrierImportRegistry(ImportRegistry delegate, BarrierTable table) {
        this.delegate = delegate;
        this.table = table;
    }

    @Override
    public ImportUnit createImportUnit(String name) {
        if (BarrierClass.KEY_BIKE.equals(name))
            return unit(name, BarrierClass.Mode.BIKE);
        if (BarrierClass.KEY_FOOT.equals(name))
            return unit(name, BarrierClass.Mode.FOOT);
        ImportUnit unit = delegate.createImportUnit(name);
        // bike_access / foot_access bring their barrier class with them, so no config lists it
        if (unit != null && ("bike_access".equals(name) || "foot_access".equals(name))) {
            List<String> required = new ArrayList<>(unit.getRequiredImportUnits());
            required.add("bike_access".equals(name) ? BarrierClass.KEY_BIKE : BarrierClass.KEY_FOOT);
            return ImportUnit.create(name, unit.getCreateEncodedValue(), unit.getCreateTagParser(), required.toArray(new String[0]));
        }
        return unit;
    }

    private ImportUnit unit(String name, BarrierClass.Mode mode) {
        return ImportUnit.create(name,
                props -> BarrierClass.create(name),
                (lookup, props) -> new BarrierClassParser(lookup.getEnumEncodedValue(name, BarrierClass.class), mode, table));
    }

    public ImportRegistry getDelegate() {
        return delegate;
    }
}
