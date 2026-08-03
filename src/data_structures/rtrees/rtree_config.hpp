#ifndef RTREE_CONFIG_HPP
#define RTREE_CONFIG_HPP

#pragma once

#include <cstdint>

namespace rtree_config {
    constexpr uint8_t MAX_ENTRIES = 8;
    constexpr uint8_t MIN_ENTRIES = MAX_ENTRIES / 2;
    constexpr uint8_t K_NEAREST = 5; // Number of edges to return in nearest edges queries.
    constexpr float MAX_EDGE_DIST = 50.0f; // Maximum distance (in meters) while looking for a nearest edge to a custom position.
    constexpr int MAX_DEPTH = 20;
    constexpr int REINSERT_PERCENT = 30;
}

#endif