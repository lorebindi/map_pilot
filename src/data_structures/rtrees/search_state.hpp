#ifndef SEARCH_STATE_HPP
#define SEARCH_STATE_HPP

#pragma once

#include <array>
#include "rtree_config.hpp"

struct Node;
struct Edge;

/*
 * This structure allows to store the k-Nearest edges to a custom position
 * during a search in the rtree.
 *
 * It contains:
 *  - the nearest edges found
 *  - their distances to the query point
 *  - the projected coordinates of the query on each edge
 *
 * n_entries tells how many slots are currently filled (<= K_NEAREST).
 *
 */

/*          edge
 * A |-----------------| B
 *            |
 *            | projection
 *            |
 *            p (lat,lon) searched position
 */

struct SearchResult {
    const Node* src; // src node of the k-nearest edge
    const Edge* edge; // k-nearest edge
    float dist; // distances between custom position p(lat,lon) and the previous edge
    float proj_lat; // latitude of the projection of the custom position on the previous edge
    float proj_lon; // longitude of the projection of the custom position on the previous edge
};

using SearchState = std::array<SearchResult, rtree_config::K_NEAREST>;

#endif //SEARCH_STATE_HPP