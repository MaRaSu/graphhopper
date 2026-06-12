/*
 * Trailmap - Area Routing Rules POJO
 *
 * Defines which OSM area polygons are passable for routing.
 * Loaded from JSON configuration file.
 */
package com.graphhopper.trailmap.areas;

import com.fasterxml.jackson.annotation.JsonProperty;

import java.util.*;

/**
 * Configuration POJO for area routing rules.
 * Loaded from JSON file specified by import.osm.area_routing.rules_file config.
 */
public class AreaRoutingRules {
    private String description = "";
    private List<Rule> rules = new ArrayList<>();

    @JsonProperty("global_forbidden_tags")
    private List<TagMatcher> globalForbiddenTags = new ArrayList<>();

    public String getDescription() {
        return description;
    }

    public void setDescription(String description) {
        this.description = description;
    }

    public List<Rule> getRules() {
        return rules;
    }

    public void setRules(List<Rule> rules) {
        this.rules = rules;
    }

    public List<TagMatcher> getGlobalForbiddenTags() {
        return globalForbiddenTags;
    }

    public void setGlobalForbiddenTags(List<TagMatcher> globalForbiddenTags) {
        this.globalForbiddenTags = globalForbiddenTags;
    }

    /**
     * A tag key-value matcher. Value of "*" matches any value.
     */
    public static class TagMatcher {
        private String key = "";
        private String value = "";

        public TagMatcher() {}

        public TagMatcher(String key, String value) {
            this.key = key;
            this.value = value;
        }

        public String getKey() {
            return key;
        }

        public void setKey(String key) {
            this.key = key;
        }

        public String getValue() {
            return value;
        }

        public void setValue(String value) {
            this.value = value;
        }

        /**
         * Checks if this matcher matches the given tag value.
         * @param actualValue the actual tag value (may be null if tag not present)
         * @return true if the tag is present and matches
         */
        public boolean matches(String actualValue) {
            if (actualValue == null) {
                return false;
            }
            return "*".equals(value) || actualValue.equals(value);
        }

        @Override
        public String toString() {
            return key + "=" + value;
        }
    }

    /**
     * A rule that defines when an area is considered passable.
     */
    public static class Rule {
        private String name = "";
        private String description = "";

        @JsonProperty("required_tags")
        private Map<String, String> requiredTags = new HashMap<>();

        @JsonProperty("forbidden_tags")
        private List<TagMatcher> forbiddenTags = new ArrayList<>();

        @JsonProperty("area_check")
        private String areaCheck = "explicit_or_closed";

        private Map<String, Boolean> access = new HashMap<>();

        public String getName() {
            return name;
        }

        public void setName(String name) {
            this.name = name;
        }

        public String getDescription() {
            return description;
        }

        public void setDescription(String description) {
            this.description = description;
        }

        public Map<String, String> getRequiredTags() {
            return requiredTags;
        }

        public void setRequiredTags(Map<String, String> requiredTags) {
            this.requiredTags = requiredTags;
        }

        public List<TagMatcher> getForbiddenTags() {
            return forbiddenTags;
        }

        public void setForbiddenTags(List<TagMatcher> forbiddenTags) {
            this.forbiddenTags = forbiddenTags;
        }

        /**
         * How to check if the way is an area:
         * - "explicit": only if area=yes tag is present
         * - "explicit_or_closed": if area=yes OR first node == last node (closed polygon)
         * - "closed_only": only closed polygons without area=yes
         */
        public String getAreaCheck() {
            return areaCheck;
        }

        public void setAreaCheck(String areaCheck) {
            this.areaCheck = areaCheck;
        }

        /**
         * Default access permissions for this area type.
         * Keys: "foot", "bike", "car". Values: true/false.
         */
        public Map<String, Boolean> getAccess() {
            return access;
        }

        public void setAccess(Map<String, Boolean> access) {
            this.access = access;
        }

        @Override
        public String toString() {
            return "Rule{name='" + name + "', requiredTags=" + requiredTags + "}";
        }
    }

    /**
     * Creates default rules for common passable areas.
     * These defaults match the bundled area_routing_rules.json file.
     */
    public static AreaRoutingRules createDefault() {
        AreaRoutingRules rules = new AreaRoutingRules();
        rules.setDescription("Default area routing rules for Trailmap");

        // Global forbidden tags - never route through these
        List<TagMatcher> forbidden = new ArrayList<>();
        forbidden.add(new TagMatcher("access", "private"));
        forbidden.add(new TagMatcher("access", "no"));
        forbidden.add(new TagMatcher("building", "*"));
        forbidden.add(new TagMatcher("natural", "water"));
        forbidden.add(new TagMatcher("natural", "wetland"));
        forbidden.add(new TagMatcher("natural", "cliff"));
        forbidden.add(new TagMatcher("natural", "glacier"));
        forbidden.add(new TagMatcher("landuse", "reservoir"));
        forbidden.add(new TagMatcher("landuse", "industrial"));
        forbidden.add(new TagMatcher("landuse", "quarry"));
        forbidden.add(new TagMatcher("waterway", "*"));
        forbidden.add(new TagMatcher("military", "*"));
        forbidden.add(new TagMatcher("aeroway", "*"));
        rules.setGlobalForbiddenTags(forbidden);

        List<Rule> ruleList = new ArrayList<>();

        // Parking lots
        Rule parking = new Rule();
        parking.setName("parking_lots");
        parking.setDescription("Parking lots at trailheads and other locations");
        parking.setRequiredTags(Map.of("amenity", "parking"));
        parking.setForbiddenTags(List.of());
        parking.setAreaCheck("explicit_or_closed");
        parking.setAccess(Map.of("foot", true, "bike", true, "car", false));
        ruleList.add(parking);

        // Pedestrian areas
        Rule pedestrian = new Rule();
        pedestrian.setName("pedestrian_areas");
        pedestrian.setDescription("Pedestrian plazas and squares with explicit area=yes");
        pedestrian.setRequiredTags(Map.of("highway", "pedestrian", "area", "yes"));
        pedestrian.setForbiddenTags(List.of());
        pedestrian.setAreaCheck("explicit");
        pedestrian.setAccess(Map.of("foot", true, "bike", true, "car", false));
        ruleList.add(pedestrian);

        // Squares
        Rule squares = new Rule();
        squares.setName("squares");
        squares.setDescription("Town squares and plazas");
        squares.setRequiredTags(Map.of("place", "square"));
        squares.setForbiddenTags(List.of());
        squares.setAreaCheck("explicit_or_closed");
        squares.setAccess(Map.of("foot", true, "bike", true, "car", false));
        ruleList.add(squares);

        // Rest areas
        Rule restAreas = new Rule();
        restAreas.setName("rest_areas");
        restAreas.setDescription("Highway rest areas");
        restAreas.setRequiredTags(Map.of("highway", "rest_area"));
        restAreas.setForbiddenTags(List.of());
        restAreas.setAreaCheck("explicit_or_closed");
        restAreas.setAccess(Map.of("foot", true, "bike", true, "car", false));
        ruleList.add(restAreas);

        // Picnic sites
        Rule picnic = new Rule();
        picnic.setName("picnic_sites");
        picnic.setDescription("Picnic areas");
        picnic.setRequiredTags(Map.of("tourism", "picnic_site"));
        picnic.setForbiddenTags(List.of());
        picnic.setAreaCheck("explicit_or_closed");
        picnic.setAccess(Map.of("foot", true, "bike", true, "car", false));
        ruleList.add(picnic);

        // Public transport platforms
        Rule platform = new Rule();
        platform.setName("platform");
        platform.setDescription("Public transport platforms");
        platform.setRequiredTags(Map.of("public_transport", "platform"));
        platform.setForbiddenTags(List.of());
        platform.setAreaCheck("explicit_or_closed");
        platform.setAccess(Map.of("foot", true, "bike", false, "car", false));
        ruleList.add(platform);

        rules.setRules(ruleList);
        return rules;
    }
}
