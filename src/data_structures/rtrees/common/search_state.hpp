#ifndef SEARCH_STATE_HPP
#define SEARCH_STATE_HPP

#pragma once

#include <array>
#include "../rtree_config.hpp"

struct Node;
struct Edge;

/* This struct represent a single k-nearest search result */

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
    double dist; // distances between custom position p(lat,lon) and the previous edge
    double proj_lat; // latitude of the projection of the custom position on the previous edge
    double proj_lon; // longitude of the projection of the custom position on the previous edge
};

/*
 * This structure allows to store the result of k-Nearest edges search from a custom position
 *
 * It contains:
 *  - the nearest edges found
 *  - their distances to the query point
 *  - the projected coordinates of the query on each edge
 *
 * n_entries tells how many slots are currently filled (<= K_NEAREST).
 *
 */
struct SearchState {
    std::array<SearchResult, rtree_config::K_NEAREST> items{};
    size_t n_entries = 0;

    // Distance ordered insertion (it keeps the K nearest edges)
    void insert(const Node* src, const Edge* edge, double dist, double proj_lat, double proj_lon) {
        // searching the correct position
        size_t p = 0;
        while (p < n_entries && dist > items[p].dist) p++;
        // 'items' is full e dist is bigger than the ones inside
        if (n_entries == rtree_config::K_NEAREST && p == rtree_config::K_NEAREST) {
            return;
        }
        // if the array isn't full increase n_entries
        if (n_entries < rtree_config::K_NEAREST) {
            n_entries++;
        }
        // right shift to create a free slot
        for (size_t i = n_entries - 1; i > p; --i) {
            items[i] = items[i - 1];
        }
        // insert the new 'search result' in position p
        items[p] = {src, edge, dist, proj_lat, proj_lon};
    }

};

#endif //SEARCH_STATE_HPP