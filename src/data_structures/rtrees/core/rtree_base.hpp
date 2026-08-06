#ifndef RTREE_BASE_HPP
#define RTREE_BASE_HPP

#include <optional>

#include "../common/search_state.hpp"
#include "../common/bounding_box.hpp"
#include "../common/parent_candidates.hpp"
#include "rtree_node.hpp"
#include "../common/tree_path.hpp"
#include "../common/reinsert_entries.hpp"

struct Edge;

/*
 * Result returned by the recursive insertion procedure.
 *
 * During an insertion, a subtree can either absorb the new entry without
 * changing its structure, or it can overflow and require a split.
 *
 * - split == false:
 *   The insertion was completed inside the current subtree. No replacement
 *   node needs to be propagated to the parent.
 *
 * - split == true:
 *   The current subtree was split and a new node (typically a new parent
 *   containing the resulting groups) must replace the previous subtree in
 *   the parent. Ownership of that new node is transferred through the
 *   unique_ptr.
 */
struct InsertResult {
    bool split;
    std::unique_ptr<RTreeNode> group1;
    std::unique_ptr<RTreeNode> group2;
};

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

    // This function return the minimum bounding rectangle that includes each entries of the rtree root
    BoundingBox root_mbr() const;

    // The one behavior that differs between R-tree and R*-tree.
    virtual void insert_edge(const Node& src, const Node& dst, const Edge& edge) = 0;

    /*
    * This method checks parameters and call remove_edge_internal
    *
    * Parameters:
    *  - 'src': reference to the source node of the edge to remove.
    *  - 'dst': reference to the destination node of the edge to remove.
    *  - 'edge': referece to the edge to remove.
    */
    void remove_edge(const Node& src, const Node& dst, const EdgePtr& edge);


    /*
    * This function check the parameters and call search_rtree.
    *
    * Parameters:
    *  - 'query': bounding box representing the custom position.
    */
    SearchState search_k_nearest_edges(const BoundingBox query);

protected:
    std::unique_ptr<RTreeNode> root_;

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
    virtual InsertResult insert_edge_internal(RTreeNode* node, BoundingBox rect, std::unique_ptr<EdgePtr> e) = 0;

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
    virtual InsertResult insert_internal_node(RTreeNode* node, const RTreeNode* best_parent, TreeEntry& reinsert, uint16_t curr_level) = 0;

    /*
    * Reinserts all entries contained in 'to_reinsert' into the tree.
    *
    * The entries can represent either data entries (leaf level) or internal node
    * entries (subtrees). This function restores the tree structure after
    * restructuring operations by placing the entries back into appropriate
    * locations while preserving R-tree invariants and bounding-box correctness.
    *
    * The concrete insertion policy is delegated to derived classes through the
    * virtual insertion routines, allowing both R-tree and R*-tree variants to
    * provide their specific insertion behavior.
    *
    * Note:
    * - The root may change during reinsertion due to node splits.
    *
    * Parameters:
    *  - 'to_reinsert': collection of entries that must be reinserted into the tree.
    */
    void reinsert_nodes(ReinsertEntries& to_reinsert);

    /*
    * Visit recursively the rtree and store all possible overlapping parent of 'reinsert'
    * in parameter 'result'.
    *
    * Parameters:
    *  - 'node': pointer to the current node of the R-tree being visited.
    *  - 'reinsert': pointer to the tree entry that needs to be reinserted.
    *  - 'curr_level': current depth level in the tree (root = 0).
    *  - 'result': container (tree_entries_arr_t) where overlapping parent candidates are collected.
    */
    void get_overlapping_parent(const RTreeNode* node, const TreeEntry& reinsert, uint16_t curr_level, ParentCandidates& result);

    void create_new_root(InsertResult&& result);

private:

    /*
    * CondenseTree algorithm for R-tree deletion.
    *
    * Iterates the tree_path from the leaf up to the root, performing the following:
    *  - Removes from the rtree not full enough nodes and collects their entries for reinsertion
    *      (they can be leafs, i.e. edge_ptr, or internal nodes, i.e. r_tree_node).
    *  - Update the bounding rectangles of remaining nodes to tightly fit their entries.
    *  - Reinsert orphaned entries at the end of this function.
    *
    * Returns the root of the rtree.
    *
    * Note: due to the splitting process, can return a different pointer to the root.
    *
    * Parameters:
    *  - 'root': pointer to the root of the R-tree.
    *  - 'p': contains the path from the found leaf to the root of the R-tree.
    */
    void condense_tree(TreePath &tree_path);

    /*
    * This function removes an edge from the rtree.
    *
    * After the elimination, if the leaf entries are lesser than 'MIN_ENTRIES', it
    * performs merging operation going back up the tree.
    *
    * Note: can return a different pointer to the root.
    *
    * Parameters:
    *  - 'rect': rect of the entry to the remove.
    *  - 'e': edge to remove.
    */
    void remove_edge_internal(BoundingBox rect, const EdgePtr &edge);

    void search_internal(const RTreeNode* node, const BoundingBox& query, SearchState& state);


};

#endif //RTREE_BASE_HPP