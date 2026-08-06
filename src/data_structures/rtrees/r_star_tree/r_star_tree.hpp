#ifndef R_STAR_TREE_HPP
#define R_STAR_TREE_HPP

#include "../core/rtree_base.hpp"

class RStarTree final : public RTreeBase {

public:
    RStarTree() = default;
    ~RStarTree() override = default;

    RStarTree(const RStarTree&) = delete;
    RStarTree& operator=(const RStarTree&) = delete;
    RStarTree(RStarTree&&) noexcept = default;
    RStarTree& operator=(RStarTree&&) noexcept = default;

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

    /*
    * Reinsert all the entry (data entry or internal node entry) inside 'to_reinsert' in the
    * rtree starting from 'root'.
    *
    * This function ensures that all orphaned entries are correctly placed back into
    * the R-tree/R*tree while maintaining its structural and bounding-box properties.
    *
    *
    * Note: due to the splitting process, can change the root
    *
    * Parameters:
    *  - 'to_reinsert': pointer to the array of the R-tree node have to be reinsert.
    */
    void reinsert_nodes(ReinsertEntries& to_reinsert) override;

private:

    /*
     * R*-specific insert bookkeeping, alive only for the
     * duration of one insert_edge() call.
    */
    struct InsertSession {
        std::array<bool, rtree_config::MAX_DEPTH> overflowed_level{};
        ReinsertEntries to_reinsert;
    };
    InsertSession session_;

    void session_reset() noexcept {
        session_.overflowed_level.fill(false);
        session_.to_reinsert.clear();
    }

    /*
    * Handles overflow of an R*-tree node during insertion, following R*-tree
    * policy: the first time a given non-root level overflows during one
    * insert_edge() operation, force a reinsertion of a fraction of that node's
    * entries instead of splitting immediately. Only split once that level has
    * already been through forced reinsertion (or the overflowing node is the
    * root, which cannot be reinserted).
    *
    * Parameters:
    *  - 'node': pointer to the overflowing node (not owned by this function).
    *  - 'bb': bounding box of the new entry being inserted.
    *  - 'new_entry': the new entry (data edge or child node) causing overflow.
    *  - 'curr_level': level of 'node' in the tree (0 = root).
    *
    * Returns:
    *  - InsertResult with split == false if a forced reinsertion was performed
    *    (the overflowing entries needing reinsertion are pushed onto
    *    session_.to_reinsert as a side effect; no replacement node is needed).
    *  - InsertResult with split == true, carrying the two resulting groups,
    *    if a split occurred. The caller is responsible for installing the
    *    result (attaching to the parent, or promoting to a new root via
    *    create_new_root() if curr_level == 0).
    */
    InsertResult overflow_treatment(RTreeNode* node, TreeEntry new_entry, uint16_t curr_level);

    /*
    * Removes the p entries farthest from node's MBR center, among node's
    * existing entries plus the incoming 'entry'/'bb', and pushes the removed
    * ones onto session_.to_reinsert.
    *
    * Parameters:
    *  - 'node': the overflowing node (leaf or internal); entries are removed
    *    from it in place.
    *  - 'entry': the incoming entry that triggered overflow. Only
    *    consulted, never physically present in 'node'.
    *  - 'p': number of entries to remove (farthest first).
    *  - 'curr_level': depth of 'node' (0 = root).
    *
    * Returns true if 'entry' itself was among the p removed (i.e. it was never
    * physically inserted into 'node' and the caller must not insert it either).
    */
    bool remove_farthest_entries(RTreeNode* node, TreeEntry entry, uint16_t p, uint16_t curr_level);

    /*
    * Performs the forced reinsertion strategy used by the R*-tree overflow
    * treatment.
    *
    * When a node overflows, this function removes the entries that are farthest
    * from the node center and stores them for later reinsertion. If the current
    * entry is not selected for reinsertion, it is inserted into the node after
    * the removal process.
    *
    * After modifying the node contents, the updated bounding rectangle is
    * propagated toward the root to preserve the correctness of the tree MBRs.
    *
    * The removed entries are not reinserted immediately; they are collected in
    * the reinsertion container and will be handled later by reinsert_nodes().
    *
    * Parameters:
    *  - 'node': overflowing node on which the forced reinsertion is performed.
    *  - 'entry': new entry that triggered the overflow.
    *  - 'curr_level': current level of the node in the tree (root level = 0).
    */
    void forced_reinsert(RTreeNode* node, TreeEntry entry, uint16_t curr_level);
};

#endif //R_STAR_TREE_HPP