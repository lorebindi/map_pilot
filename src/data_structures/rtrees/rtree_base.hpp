#ifndef RTREE_BASE_HPP
#define RTREE_BASE_HPP

#include <optional>

#include "search_state.hpp"
#include "bounding_box.hpp"
#include "rtree_node.hpp"
#include "tree_path.hpp"

struct Edge;

/*
 * Abstract base shared by RTree and RStarTree. Owns the root node and
 * implements every operation the two variants share (search, remove, MBR
 * queries). Only insert_edge differs between them -- that's the sole pure
 * virtual method subclasses must supply.
 *
 * Replaces the original tree_policy_t vtable-in-a-struct: Graph now holds a
 * std::unique_ptr<RTreeBase> and calls virtual methods directly, instead of
 * a raw tree_policy_t* plus a hand-rolled function-pointer table.
 */
class RTreeBase {
public:
    virtual ~RTreeBase() = default;

    RTreeBase() = default;
    RTreeBase(const RTreeBase&) = delete;
    RTreeBase& operator=(const RTreeBase&) = delete;
    RTreeBase(RTreeBase&&) noexcept = default;
    RTreeBase& operator=(RTreeBase&&) noexcept = default;

    // The one behavior that differs between R-tree and R*-tree.
    virtual void insert_edge(const Node& src, const Node& dst, std::unique_ptr<EdgePtr> edge) = 0;

    // removing and search method are shared between rtree and r*tree
    void remove_edge(const Node& src, const Node& dst, const EdgePtr& edge);
    SearchState search_k_nearest_edges(BoundingBox query) const;
    // This function return the minimum bounding rectangle that includes each entries of the rtree root
    BoundingBox root_mbr() const;

protected:
    std::unique_ptr<RTreeNode> root_;

    // Shared helper used by both insert_edge overrides after they've done
    // their variant-specific choose-leaf/split work at the top level.
    // (fill in once rtree_core's shared recursion helpers are ported)
};

#endif //RTREE_BASE_HPP