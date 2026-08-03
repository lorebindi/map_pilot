#include "highways.hpp"
#include <unordered_map>

HighwayType parse_highway_class(std::string_view s) {
    static const std::unordered_map<std::string_view, HighwayType> lookup = {
        {"motorway",       HighwayType::Motorway},
        {"motorway_link",  HighwayType::MotorwayLink},
        {"trunk",          HighwayType::Trunk},
        {"trunk_link",     HighwayType::TrunkLink},
        {"primary",        HighwayType::Primary},
        {"primary_link",   HighwayType::PrimaryLink},
        {"secondary",      HighwayType::Secondary},
        {"secondary_link", HighwayType::SecondaryLink},
        {"tertiary",       HighwayType::Tertiary},
        {"tertiary_link",  HighwayType::TertiaryLink},
        {"unclassified",   HighwayType::Unclassified},
        {"residential",    HighwayType::Residential},
    };

    auto it = lookup.find(s);
    return it != lookup.end() ? it->second : HighwayType::Unknown;
}

/* Returns true if the street is NOT physically divided (i.e. U-turns are possible):
 * exactly Primary, Secondary, Tertiary, Residential.
 * Everything else (Motorway*, Trunk*, Link variants, Unclassified, Unknown) returns false. */
bool is_two_way_undivided_street(HighwayType t) {
    return t == HighwayType::Primary   ||
           t == HighwayType::Secondary ||
           t == HighwayType::Tertiary  ||
           t == HighwayType::Residential;
}