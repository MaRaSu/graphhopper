package com.graphhopper.trailmap.analysis.gravel;

import com.graphhopper.trailmap.shared.GravelScale;
import com.graphhopper.trailmap.shared.MtbScale;
import org.junit.jupiter.api.Test;

import java.util.EnumMap;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class GravelAnalysisConfigTest {

    private static GravelAnalysisConfig validConfig() {
        GravelAnalysisConfig cfg = new GravelAnalysisConfig();
        cfg.gravelSizeThresholdM = 2000;
        cfg.enableConnectors = true;
        return cfg;
    }

    @Test
    void rejectsNonPositiveOrNonFiniteConnectorWeights() {
        validConfig().validate();   // defaults are valid

        for (double bad : new double[]{0.0, -1.0, Double.NaN, Double.POSITIVE_INFINITY}) {
            GravelAnalysisConfig cfg = validConfig();
            cfg.connectorWeights.put(GravelScale.TWO, bad);
            assertThrows(IllegalStateException.class, cfg::validate, "weight " + bad + " must be rejected");
        }
        // ...and the same for an mtb override value.
        GravelAnalysisConfig cfg = validConfig();
        Map<MtbScale, Double> ov = new EnumMap<>(MtbScale.class);
        ov.put(MtbScale.ONE, -2.0);
        cfg.connectorMtbWeightOverrides.put(GravelScale.THREE, ov);
        assertThrows(IllegalStateException.class, cfg::validate, "negative mtb-override weight must be rejected");
    }

    @Test
    void connectorWeightAppliesMtbOverride() {
        GravelAnalysisConfig cfg = new GravelAnalysisConfig();
        // Plain hard-gravel (gs2): base weight.
        assertEquals(1.0, cfg.connectorWeight(GravelScale.TWO, MtbScale.ZERO_MINUS), 1e-9);
        assertEquals(1.0, cfg.connectorWeight(GravelScale.TWO, MtbScale.UNKNOWN), 1e-9);
        // gs2 that is MTB-technical (mtb1) costs like gs3.
        assertEquals(2.5, cfg.connectorWeight(GravelScale.TWO, MtbScale.ONE), 1e-9);
        assertEquals(cfg.connectorWeight(GravelScale.THREE, MtbScale.ZERO_MINUS),
                cfg.connectorWeight(GravelScale.TWO, MtbScale.ONE), 1e-9);
        // Other bands unaffected by mtb (no override) -> base weight.
        assertEquals(2.5, cfg.connectorWeight(GravelScale.THREE, MtbScale.ONE), 1e-9);
        assertEquals(10.0, cfg.connectorWeight(GravelScale.FOUR, MtbScale.ONE), 1e-9);
    }
}
