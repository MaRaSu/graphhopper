package com.graphhopper.trailmap.tbt;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * One-command placement diagnostic — the JVM half of tbt-tools/why_instruction.sh.
 *
 * Usage (from graphhopper/):
 *   mvn -o -q -pl core test -Dtest=InstructionPlacementDiagTest -DfailIfNoTests=false \
 *       -Dsurefire.useFile=false -Dtbt.payload=/path/to/payload.json
 *
 * Prints the per-instruction verdict table (oracle vs reported route distance) verbatim,
 * then fails iff any instruction is misplaced — so the wrapper's exit code is the verdict.
 * Skipped entirely when -Dtbt.payload is not set (normal test runs).
 */
public class InstructionPlacementDiagTest {

    @BeforeAll
    static void setup() throws Exception {
        Assumptions.assumeTrue(System.getProperty("tbt.payload") != null,
                "diagnostic runner: -Dtbt.payload not set — skipping");
        InstructionPlacementValidationTest.setup();
    }

    @Test
    void whyInstruction() throws Exception {
        String payloadPath = System.getProperty("tbt.payload");
        File f = new File(payloadPath);
        assertTrue(f.isFile(), "payload file not found: " + f.getAbsolutePath());

        TrailmapInstructionRequest request =
                InstructionPlacementValidationTest.parsePayload(Files.readString(Path.of(payloadPath)));
        InstructionPlacementOracle.Report report = InstructionPlacementValidationTest.run(request);

        System.out.println();
        System.out.println("=== instruction placement verdict: " + f.getName() + " ===");
        System.out.print(report.table());
        System.out.println();

        assertTrue(report.allPass(), "placement FAIL — see verdict table above");
    }
}
