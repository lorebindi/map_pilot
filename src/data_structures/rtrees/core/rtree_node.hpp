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
    std::unique_ptr<RTreeNode>,
    std::unique_ptr<EdgePtr>
>;

/*
 * A single entry collected during tree condensation/rebalancing: either an
 * orphaned internal node (RTreeNode, shared by both RTree and RStarTree) or
 * an orphaned leaf entry (EdgePtr), tagged with its former bounding box and
 * tree level (needed to reinsert at the correct level).
 *
 * The tree owns the EdgePtr *wrapper* (hence unique_ptr<EdgePtr> here), even
 * though EdgePtr's own src/dst/edge fields are non-owning pointers into
 * Graph's storage -- two independent ownership facts, not in tension.
 */
struct TreeEntry {
    BoundingBox b_box;
    uint16_t tree_node_level;
    variant<unique_ptr<RTreeNode>, unique_ptr<EdgePtr>> data;

    TreeEntry() = default;

    TreeEntry(BoundingBox b_box, unique_ptr<EdgePtr> data, uint16_t tree_node_level=std::numeric_limits<uint16_t>::max()) {
        this->b_box = b_box;
        this->tree_node_level = tree_node_level;
        this->data = std::move(data);
    }

    TreeEntry(BoundingBox b_box, unique_ptr<RTreeNode> data, uint16_t tree_node_level=std::numeric_limits<uint16_t>::max()) {
        this->b_box = b_box;
        this->tree_node_level = tree_node_level;
        this->data = std::move(data);
    }

    bool is_data_entry() const noexcept {
        return holds_alternative<unique_ptr<EdgePtr>>(data);
    }
};

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
    void insert_entry(TreeEntry &&entry);

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


// This function return the minimum bounding rectangle that includes each bounding box belonging to the 'bb_storage''s [start,end] range
BoundingBox range_array_mbr(const std::array<TreeEntry*, rtree_config::MAX_ENTRIES + 1> &bb_storage, uint8_t start, uint8_t end) {
    BoundingBox result = bb_storage[start]->b_box;
    for (; start < end; start++) {
        result = result.union_bounding_box(bb_storage[start]->b_box);
    }
    return result;
}

/*
* Returns the perimeter (margin) of the minimum bounding rectangle that encloses all the bounding
* boxes in 'bbs' starting from 'start' position.
*/
inline float calculate_margin(const std::array<TreeEntry*, rtree_config::MAX_ENTRIES + 1> &entries, uint8_t first, uint8_t last) {
    assert(first < last);
    assert(last <= entries.size());

    float xmin = std::numeric_limits<float>::max();
    float ymin = std::numeric_limits<float>::max();
    float xmax = std::numeric_limits<float>::lowest();
    float ymax = std::numeric_limits<float>::lowest();

    for (uint8_t i = first; i < last; ++i) {
        xmin = std::min(xmin, entries[i]->b_box.x_min);
        ymin = std::min(ymin, entries[i]->b_box.y_min);
        xmax = std::max(xmax, entries[i]->b_box.x_max);
        ymax = std::max(ymax, entries[i]->b_box.y_max);
    }

    return (xmax - xmin) + (ymax - ymin);
}

#endif