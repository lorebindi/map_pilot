#include "heuristics.hpp"
#include "graph.hpp"
#include <cmath>

constexpr float heuristic_zero(const Graph&, uint16_t, uint16_t) noexcept {
    return 0.0f;
}

float heuristic_haversine(const Graph& g, uint16_t curr, uint16_t dest) {
    constexpr float EARTH_RADIUS_M = 6'371'000.0f; // meters
    //constexpr float DEG_TO_RAD = static_cast<float>(M_PI) / 180.0f;

    const Node& a_node = g.node(curr);
    const Node& b_node = g.node(dest);

    // double precision internally: avoids catastrophic cancellation in d_lat/d_lon
    // for nodes that are close together (common case: adjacent OSM intersections)
    double lat1_rad = deg_to_rad(static_cast<double>(a_node.lat));
    double lon1_rad = deg_to_rad(static_cast<double>(a_node.lon));
    double lat2_rad = deg_to_rad(static_cast<double>(b_node.lat));
    double lon2_rad = deg_to_rad(static_cast<double>(b_node.lon));

    double d_lat = lat2_rad - lat1_rad;
    double d_lon = lon2_rad - lon1_rad;

    double sin_dlat = std::sin(d_lat / 2.0);
    double sin_dlon = std::sin(d_lon / 2.0);

    double h = sin_dlat * sin_dlat +
               std::cos(lat1_rad) * std::cos(lat2_rad) * sin_dlon * sin_dlon;

    // guard against rounding pushing h slightly outside [0,1], which would
    // make sqrt(1.0 - h) NaN and silently corrupt every A* comparison downstream
    h = std::clamp(h, 0.0, 1.0);

    double c = 2.0 * std::atan2(std::sqrt(h), std::sqrt(1.0 - h));
    return static_cast<float>(EARTH_RADIUS_KM * c);
}