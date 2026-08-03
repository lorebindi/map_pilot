#ifndef HIGHWAYS_HPP
#define HIGHWAYS_HPP

#include <string_view>

/*
 * This header defines different classes of highways according to their functional and physical
 * characteristics.
 *
 * Key distinctions:
 * - Motorway and Trunk roads (including their Link variants) are physically divided highways.
 *   This means the two directions of traffic are separated by a physical barrier,
 *   and each direction has its own independent set of nodes and edges.
 *   U-turns are generally not allowed on these roads.
 *
 * - Primary, Secondary, Tertiary, Residential, and Unclassified roads are usually two-way
 *   undivided streets with no physical separation. Both directions share the same nodes with
 *   two different edges, one per direction, with the same length.
 *   U-turns and two-way traffic are possible on these roads.
 */
enum class HighwayType {
    Unknown = 0,
    Motorway,       // like A1 Milan-Naples (the two directions don't share nodes, independent nodes)
    MotorwayLink,   // highway interchanges, e.g. take the A30 to reach Caserta leaving the A1
    Trunk,          // may represent "superstrade" -- major extra-urban roads with limited access
    TrunkLink,      // on/off ramps
    Primary,        // major national/regional roads, often "SS" (strade statali)
    PrimaryLink,    // connector segments (ramps) linking a primary road to another primary or lower-class road
    Secondary,      // SP (strade provinciali)
    SecondaryLink,  // connector segments linking a secondary road to another secondary or lower-class road
    Tertiary,
    TertiaryLink,
    Residential,    // slow speed
    Unclassified,
};

// Maps the OSM-style string label (e.g. "motorway", "primary_link") to a HighwayType.
// Returns HighwayType::Unknown if the string doesn't match any known class.
HighwayType parse_highway_class(std::string_view s);

// Returns true if the street is a two-way undivided street (Primary, Secondary, Tertiary,
// Residential, Unclassified -- i.e. NOT Motorway/MotorwayLink/Trunk/TrunkLink).
bool is_two_way_undivided_street(HighwayType t);

#endif //HIGHWAYS_HPP