#ifndef RTREE_NODE_HPP
#define RTREE_NODE_HPP

#pragma once

#include <array>
#include <cassert>
#include <memory>
#include <variant>
#include "../rtree_config.hpp"
#include "../common/bounding_box.hpp"
#include "../common/edge_ptr.hpp"

class RTreeNode;

// Represents the new entry to insert during the split operation: either a EdgePtr or a RTreeNode.
using SplitEntry = std::variant<
    std::unique_ptr<EdgePtr>,
    std::unique_ptr<RTreeNode>
>;

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

    /*
    * Inserts a new entry into this node.
    *
    * This function assumes that the node has available space. It does not
    * perform overflow handling or splitting; the caller is responsible for
    * checking n_entries before calling it.
    *
    * For leaf nodes, the inserted entry must be an EdgePtr.
    * For internal nodes, the inserted entry must be an RTreeNode.
    */
    void insert_entry(BoundingBox rect, std::unique_ptr<EdgePtr> edge);
    void insert_entry(BoundingBox rect, std::unique_ptr<RTreeNode> child);
    void insert_entry(const BoundingBox& bb, SplitEntry entry);

    // It returns the bounding box of the i-th element
    BoundingBox bounding_box_at(uint8_t i) const { return bounding_rects[i]; }

    /*
    * Shifts all entries one position to the left starting from 'start_index',
    * compacting the node after an entry has been removed.
    *
    * Precondition:
    * - The node entries are already stored contiguously in the range [0, n_entries).
    * - Exactly one "hole" exists at position 'start_index', typically because
    *   the entry at that position has just been moved out.
    *
    * This function fills that hole by shifting all subsequent entries one
    * position to the left. It is not intended for compacting nodes containing
    * multiple holes.
    *
    * Works for both leaf and internal nodes. Complexity: O(n-start_index).
    */
    void shift_entries_left(uint8_t start_index);

    /*
    * Compacts the node by removing any holes left by previously removed entries.
    *
    * This function scans the occupied portion of the node and moves all valid
    * entries towards the beginning of the arrays, preserving their relative
    * order. The corresponding bounding rectangles are moved together with their
    * entries.
    *
    * A "hole" is represented by a nullptr in the entries array. After compaction,
    * all valid entries occupy the contiguous range [0, n_entries), and n_entries
    * is updated to reflect the new number of entries.
    *
    * Works for both leaf and internal nodes.
    */
    void compact_entries();

    /*
    * They remove an entry from the RTreeNode and return it.
    *
    * Note: They don't shit entries left, it's a caller's responsibility
    */
    std::unique_ptr<EdgePtr> extract_leaf_entry(uint8_t index);
    std::unique_ptr<RTreeNode> extract_internal_entry(uint8_t index);

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

    void adopt_split_children(uint8_t index, std::unique_ptr<RTreeNode> child1, std::unique_ptr<RTreeNode> child2);

private:
    bool is_leaf_;
};

#endif