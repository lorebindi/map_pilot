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

    /*
    * Inserts an edge into the appropriate leaf node of the R-tree. If the node overflows,
    * a quadratic split is performed. Splits may propagate up the tree, possibly requiring a new root.
    *
    * Parameters:
    *  - 'node': pointer to the current node (can be internal or leaf) where insertion is attempted.
    *  - 'rect': bounding box of the edge to be inserted.
    *  - 'e': pointer to the EdgePtr structure that wraps the graph edge and its endpoints.
    *  - 'is_root': boolean flag indicating whether 'node' is the root (important for split handling)
    */
    InsertResult insert_edge_internal(RTreeNode* node, BoundingBox rect, std::unique_ptr<EdgePtr> e) override;

    /*
    * Reinserts an orphaned internal node into the tree at its designated parent location.
    *
    * Traverses the R-tree recursively down to the specified `best_parent` node at the target level. Once reached,
    * it attempts to insert the orphaned internal node stored in `reinsert`. If `best_parent` has reached its
    * maximum capacity, the node is split and the resulting structural changes are propagated upwards.
    *
    * Parameters:
    *   - 'node': pointer to the current subtree root being inspected during traversal.
    *   - 'best_parent': target node where the orphaned internal entry should be inserted.
    *   - 'reinsert': wrapper holding the orphaned internal node and its bounding box metadata.
    *   - 'curr_level': current level depth of `node` during recursion (0 = root level).
    */
    InsertResult insert_internal_node(RTreeNode* node, const RTreeNode* best_parent, TreeEntry& reinsert, uint16_t curr_level) override;
};

#endif RTREE_HPP