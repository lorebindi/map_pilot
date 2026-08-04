#ifndef RTREE_NODE_HPP
#define RTREE_NODE_HPP

#pragma once

#include <array>
#include <cassert>
#include <memory>
#include <variant>
#include "../rtree_config.hpp"
#include "../bounding_box.hpp"
#include "../edge_ptr.hpp"

/*
 * A single node in the R-tree.
 *
 * - Each node stores up to MAX_ENTRIES MBRs.
 * - If the node is an internal node, it stores child pointers.
 * - If the node is a leaf, it stores edge_ptr_t references.
 *
 * All entries have their own bounding box (bounding_rects[i]).
 */

class RTreeNode {
public:
    explicit RTreeNode(bool is_leaf)
        : is_leaf_(is_leaf) {}

    bool is_leaf() const noexcept {
        return is_leaf_;
    }

    std::array<BoundingBox, rtree_config::MAX_ENTRIES> bounding_rects{};

    // This is an array of pointers to edge_ptr in case of leaf node. (MAX_ENTRIES)
    using LeafEntries = std::array<unique_ptr<EdgePtr>, rtree_config::MAX_ENTRIES>;

    // This is an array of pointers to children in case of internal node. (MAX_ENTRIES)
    using InternalEntries = std::array<unique_ptr<RTreeNode>, rtree_config::MAX_ENTRIES>;

    variant<LeafEntries, InternalEntries> entries;

    uint8_t n_entries = 0; /* only 8 bit because the number of entries are small. */

    // It returns the bounding box of the i-th element
    BoundingBox bounding_box_at(uint8_t i) const { return bounding_rects[i]; }

    /*
    * Shifts entries left starting from 'start_index', compacting the array
    * after an entry has been removed. Works for both leaf and internal nodes.
    */
    void shift_entries_left(uint8_t start_index);

    /*
    * Updates the bounding rectangle of the child node after insertion into one of its
    * children (i.e. the one identified by 'child index').
    *
    * Parameters:
    * - 'child_index': the index of the child in 'node->children'.
    *
    * Use case:
    *  This function is called when the minimum bounding rectangle of the just added/modified
    *  child need to be (re)calculated.
    */
    void update_child_bounding_rect (uint8_t child_index);

private:
    bool is_leaf_;
};

#endif