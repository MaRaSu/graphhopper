#!/bin/bash
# Test runner for WaypointNormalizer integration tests
# Uses existing graph-cache and trailmap-config.yml

TEST_METHOD="${1:-testFinlandTwoPointRoute}"

cd "$(dirname "$0")"

# Need heap for loading the large Finland graph
mvn test -pl core \
    -Dtest="WaypointNormalizerIntegrationTest#${TEST_METHOD}" \
    -DargLine="-Xms2g -Xmx8g" \
    2>&1
