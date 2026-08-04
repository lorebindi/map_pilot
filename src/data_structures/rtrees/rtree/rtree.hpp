#ifndef RTREE_HPP
#define RTREE_HPP

#include "../core/rtree_base.hpp"

class RTree final : public RTreeBase {

public:
    RTree() = default;
    ~RTree() override = default;

    RTree(const RTree&) = delete;
    RTree& operator=(const RTree&) = delete;
    RTree(RTree&&) noexcept = default;
    RTree& operator=(RTree&&) noexcept = default;

    void insert_edge(const Node& src, const Node& dst, std::unique_ptr<EdgePtr> edge) override;

protected:

    void insert_edge_internal(RTreeNode* node, BoundingBox rect, EdgePtr* edge, bool is_root) override;

    std::unique_ptr<RTreeNode> insert_internal_node(RTreeNode* node, const RTreeNode* best_parent, const TreeEntry& reinsert, uint16_t curr_level) override;
};

#endif RTREE_HPP