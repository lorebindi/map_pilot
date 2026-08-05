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

    /*
    * This function check the parameters and call insert_edge_internal.
    *
    * Note: due to the splitting process, root can change.
    *
    * Parameters:
    *  - 'src': reference to the source node of the edge to insert.
    *  - 'dst': reference to the destination node of the edge to insert.
    *  - 'edge': reference to the edge to insert.
    */
    void insert_edge(const Node& src, const Node& dst, const Edge& edge) override;

protected:

    InsertResult insert_edge_internal(RTreeNode* node, BoundingBox rect, std::unique_ptr<EdgePtr> e) override;

    InsertResult insert_internal_node(RTreeNode* node, const RTreeNode* best_parent, TreeEntry& reinsert, uint16_t curr_level) override;
};

#endif RTREE_HPP