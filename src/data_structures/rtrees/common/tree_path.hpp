#ifndef TREE_PATH_HPP
#define TREE_PATH_HPP

/*
 * This module provide data structures and functions to store and manipulate
 * a path of a rtree, starting from a leaf up to the root.
 *
 * The use of this module it's important in the elimination of an edge in the rtree
 * structure.
 */

#pragma once

#include <cassert>
#include <cstdint>
#include <vector>
#include <limits>

#include "../rtree_config.hpp"

class RTreeNode; // fwd decl, avoids circular dependency with rtree_node.hpp

/*
 * A single step in a leaf-to-root path through the R-tree: a non-owning
 * pointer to the node visited, its index within its parent (filled in later,
 * as the path is built bottom-up), and the tree level it occupies.
 *
 * 'node' is non-owning: TreePath only records a traversal through the
 * live tree, it never takes ownership of anything.
 */
struct PathStep {
    RTreeNode* node;
    uint8_t index_in_parent = std::numeric_limits<uint8_t>::max(); // replaces the implicit UINT8_MAX init
    uint16_t tree_node_level;
};

namespace tree_path_config {
    constexpr uint8_t STARTING_DIM = 8;
}

/*
 * Records the path from a found leaf up to the root of the R-tree, used
 * during edge deletion (condense_tree) to walk back up and rebalance.
 *
 * Replaces tree_path_init/_add_step/_update_index_parent/_free: constructor
 * replaces init, destructor is trivial/default (non-owning pointers, nothing
 * to free), add_step/update_index_parent keep their names as methods.
 * std::vector replaces the manual capacity/realloc growth (was STARTING_DIM).
 */
class TreePath {
public:
    TreePath() { steps_.reserve(tree_path_config::STARTING_DIM); }

    // Adds a path step at the end of the tree_path
    void add_step(RTreeNode* node, uint8_t pos, uint16_t tree_node_level) {
        assert(pos == n_steps());
        steps_.push_back(PathStep{node, std::numeric_limits<uint8_t>::max(), tree_node_level});
    }

    // Update the index in parent for the rtree node stored in the pos-th path_step
    void update_index_parent(uint8_t pos, uint8_t index_in_parent) {
        assert(pos < n_steps());
        assert(index_in_parent < rtree_config::MAX_ENTRIES);
        assert(steps_[pos].index_in_parent == std::numeric_limits<uint8_t>::max());
        steps_[pos].index_in_parent = index_in_parent;
    }

    uint8_t n_steps() const noexcept { return static_cast<uint8_t>(steps_.size()); }

    PathStep& operator[](uint8_t i) { return steps_[i]; }
    const PathStep& operator[](uint8_t i) const { return steps_[i]; }

    // Necessary for iterators
    auto begin() { return steps_.begin(); }
    auto end() { return steps_.end(); }
    auto begin() const { return steps_.begin(); }
    auto end() const { return steps_.end(); }

private:
    std::vector<PathStep> steps_;
};


#endif //TREE_PATH_HPP