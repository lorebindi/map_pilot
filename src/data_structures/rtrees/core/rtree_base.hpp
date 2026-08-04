#ifndef RTREE_BASE_HPP
#define RTREE_BASE_HPP

#include <optional>

#include "../search_state.hpp"
#include "../bounding_box.hpp"
#include "../parent_candidates.hpp"
#include "../rtree_node.hpp"
#include "../tree_path.hpp"
#include "../reinsert_entries.hpp"

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

    // This function return the minimum bounding rectangle that includes each entries of the rtree root
    BoundingBox root_mbr() const;

    // The one behavior that differs between R-tree and R*-tree.
    virtual void insert_edge(const Node& src, const Node& dst, std::unique_ptr<EdgePtr> edge) = 0;

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

    virtual void insert_edge_internal(RTreeNode* node, BoundingBox rect, EdgePtr *e, bool is_root) = 0;

    virtual std::unique_ptr<RTreeNode> insert_internal_node(RTreeNode* node, const RTreeNode* best_parent,
                                           const TreeEntry& reinsert, uint16_t curr_level) = 0;

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
    * Reinsert all the entry (data entry or internal node entry) inside 'to_reinsert' in the
    * rtree starting from 'root'.
    *
    * This function ensures that all orphaned entries are correctly placed back into
    * the R-tree while maintaining its structural and bounding-box properties.
    *
    * Returns the root.
    *
    * Note: due to the splitting process, can return a different pointer to the root
    *
    * Parameters:
    *  - 'root': pointer to the root of the R-tree.
    *  - 'to_reinsert': pointer to the array of the R-tree node have to be reinsert.
    */
    void reinsert_nodes(ReinsertEntries& to_reinsert);

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