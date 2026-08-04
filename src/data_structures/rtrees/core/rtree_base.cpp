#include "rtree_base.hpp"

#include "../common/parent_candidates.hpp"
#include "../common/reinsert_entries.hpp"
#include "graph/graph.hpp"
#include "utils/utm_converter.hpp"

BoundingBox RTreeBase::root_mbr() const {
    get_node_mbr(*root_, rtree_config::MAX_ENTRIES);
}


/*
 * This recursive function search the leaf that contains 'edge_query'. Returns true if found, otherwise false.
 *
 * During the recursion process, back from the leaf to the root, it records the relative path in 'tree_path',
 * storing each node, its index within its parent and the level of the tree in which it is located .
 *
 * Parameters:
 *  - 'root': pointer to the root of the subtree being visited (at the begininning the root of the R-tree).
 *  - 'edge_query': pointer to the edge we are looking for.
 *  - 'rect_query': bounding box we are looking for.
 *  - 'pos': is used to return the position of 'e' in the leaf.
 *  - 'tree_path': is used to store the path starting from the found leaf to the root.
 *  - 'level': the current level of the node being visited.
 */
bool find_leaf(RTreeNode *root, const Edge *edge_query, const BoundingBox &rect_query, uint8_t *pos, TreePath &tree_path, uint16_t level=0) {
    if (!root) {
        return false;
    }

    for (uint8_t i = 0; i < root->n_entries; i++) {
        if (root->bounding_rects[i].overlaps(rect_query)) {
            if (root->is_leaf()) {
                const RTreeNode::LeafEntries& leaves = std::get<RTreeNode::LeafEntries>(root->entries);
                for (uint8_t j = 0; j < root->n_entries; j++) {
                    if (leaves[j]->edge == edge_query) {
                        *pos = j;
                        tree_path.add_step(root, tree_path.n_steps(), level);
                        return true; // found it
                    }
                }
            }
            else {
                const RTreeNode::InternalEntries& internal = std::get<RTreeNode::InternalEntries>(root->entries);
                bool found = find_leaf(internal[i].get(), edge_query, rect_query, pos, tree_path, level+1);
                if (found) {
                    // update path with the child
                    tree_path.update_index_parent(tree_path.n_steps()-1, i);
                    // add the parent to the path
                    tree_path.add_step(root, tree_path.n_steps(), level);
                    return found;
                }
            }
        }
    }

    return false;
}

/*
 * Iterates over a given tree path from a leaf up to the root and performs two tasks:
 *  1. Collects nodes that have fewer entries than MIN_ENTRIES into 'nodes_to_free' for later freeing.
 *  2. Extracts the entries (leaf edges or internal child nodes) from underfull nodes and
 *     stores them in 'reinsert' for reinsertion into the tree.
 *
 * During this process, it also:
 *  - Sets to NULL the corresponding child pointer in the parent for nodes that will be removed.
 *  - Shifts left remaining entries in the parent to compact the array.
 *  - Updates the parent's bounding rectangles for nodes that are not underfull.
 *
 * Parameters:
 *  - 'p': tree path from leaf to root.
 *  - 'nodes_to_free': preallocated vector to store pointers to nodes that will be freed (dimension = p->n_steps).
 *  - 'free_count': pointer to a counter tracking the number of nodes added to 'nodes_to_free'.
 *  - 'reinsert': array to store entries (edges or internal nodes) that need to be reinserted into the tree.
 */
static void collect_nodes_to_free_and_to_reinsert(TreePath &tree_path, std::vector<std::unique_ptr<RTreeNode>> &nodes_to_free, ReinsertEntries &reinsert) {
    for (uint16_t i = 0; i < tree_path.n_steps(); i++) {
        RTreeNode *curr = tree_path[i].node; // non-owning, tree_path never owned
        uint8_t curr_index_in_parent = tree_path[i].index_in_parent;
        uint16_t curr_tree_node_level = tree_path[i].tree_node_level;
        RTreeNode* parent = (i + 1 < tree_path.n_steps()) ? tree_path[i + 1].node : nullptr;

        /* extract the entries of the node if it isn't the root and it hasn't enough entries. */
        if (curr->n_entries < rtree_config::MIN_ENTRIES && curr_tree_node_level != 0) {
            if (curr->is_leaf()) {
                auto& data = std::get<RTreeNode::LeafEntries>(curr->entries);
                for (uint8_t j = 0; j < curr->n_entries; j++) {
                    reinsert.add(curr->bounding_rects[j], curr_tree_node_level, std::move(data[j]));
                }
            } else {
                auto& children = std::get<RTreeNode::InternalEntries>(curr->entries);
                for (uint8_t j = 0; j < curr->n_entries; j++) {
                    reinsert.add(curr->bounding_rects[j], curr_tree_node_level, std::move(children[j]));
                }
            }

            if (parent != nullptr) {
                auto& parent_children = std::get<RTreeNode::InternalEntries>(parent->entries);
                // Transfer ownership of curr out of the tree and into nodes_to_free.
                // This also leaves parent_children[curr_index_in_parent] == nullptr.
                nodes_to_free.push_back(std::move(parent_children[curr_index_in_parent]));
                parent->shift_entries_left(curr_index_in_parent);
            }
        } else if (parent != nullptr) {
            parent->update_child_bounding_rect(curr_index_in_parent);
        }
    }
}

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
void RTreeBase::get_overlapping_parent(const RTreeNode* node, const TreeEntry& reinsert, uint16_t curr_level, ParentCandidates& result) {
    assert(node != nullptr);
    if (node->is_leaf()) return; //early exit

    // Target level is grandparent level so candidates[i] gives nodes at parent level
    const uint16_t target_level = (reinsert.tree_node_level >= 2) ? reinsert.tree_node_level - 2 : 0;

    if (curr_level == target_level) {

        // the recursion is in the level of the possible parents: collect all the possible parents
        const auto& candidates = std::get<RTreeNode::InternalEntries>(node->entries);
        for (uint8_t i = 0; i < node->n_entries; i++) {
            if (node->bounding_rects[i].overlaps(reinsert.b_box)) {
                result.add(node->bounding_rects[i], candidates[i].get());
            }
        }
        return;
    }

    if (!node->is_leaf()) {
        const auto& children = std::get<RTreeNode::InternalEntries>(node->entries);
        for (uint8_t i = 0; i < node->n_entries; i++) {
            if (node->bounding_rects[i].overlaps(reinsert.b_box)) {
                get_overlapping_parent(children[i].get(), reinsert, curr_level + 1, result);
            }
        }
    }

}

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
void RTreeBase::reinsert_nodes(ReinsertEntries& to_reinsert) {

    for (uint16_t i = 0; i < to_reinsert.size(); i++) {
        TreeEntry& entry = to_reinsert[i];

        if (entry.is_data_entry()) {
            // orphaned leaf edge: take ownership from to_reinsert, insert starting from root
            auto edge = std::move(std::get<std::unique_ptr<EdgePtr>>(entry.data));
            this->insert_edge(*edge->src, *edge->dst, std::move(edge));

        }else {
            // orphaned internal node: find candidate parents still live in the
            // tree, pick the one needing least enlargement, reinsert there
            ParentCandidates parents;

            get_overlapping_parent(this->root_.get(), entry, 0, parents);
            uint8_t best_parent_index = get_area_min_enlargement_index(parents, parents.size(), to_reinsert[i].b_box);
            RTreeNode* best_parent = parents[best_parent_index];

            this->root_ = insert_internal_node(this->root_.get(), best_parent, entry, 0);
        }
    }
}

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
void RTreeBase::condense_tree(TreePath &tree_path) {
    /* if the root is an empty leaf, nothing to condense */
    if (this->root_->is_leaf() && this->root_->n_entries == 0)
        return;

    std::vector<std::unique_ptr<RTreeNode>> nodes_to_free; /* destroyed at scope exit */
    ReinsertEntries to_reinsert;
    // fill 'nodes_to_free' and 'nodes_to_reinsert'
    collect_nodes_to_free_and_to_reinsert(tree_path, nodes_to_free, to_reinsert);
    // reinsert
    this->reinsert_nodes(to_reinsert);
    // if the root has only one child, it becomes the new root
    if (!this->root_->is_leaf() && this->root_->n_entries == 1) {
        auto& children = std::get<RTreeNode::InternalEntries>(this->root_->entries);
        this->root_ = std::move(children[0]);
    }
}

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
void RTreeBase::remove_edge_internal(BoundingBox rect, const EdgePtr &e) {
    uint8_t position_leaf = 0;
    TreePath leaf_to_root_path;
    // search the leaf and recording leaf-to-root path
    if (!find_leaf(this->root_.get(), e.edge, rect, &position_leaf, leaf_to_root_path)) {
        return;
    }
    RTreeNode *leaf = leaf_to_root_path[0].node;
    // removing the edge 'e' in the leaf and left shifting for the possible remaining edges and bb
    leaf->shift_entries_left(position_leaf);
    // fixing the rtree structure (i.e. call to condense_tree)
    this->condense_tree(leaf_to_root_path);
}

/*
* This method checks parameters and call remove_edge_internal
*
* Parameters:
*  - 'src': reference to the source node of the edge to remove.
*  - 'dst': reference to the destination node of the edge to remove.
*  - 'edge': referece to the edge to remove.
*/
void RTreeBase::remove_edge(const Node &src, const Node &dst, const EdgePtr &e) {
    if (!root_) {
        throw std::runtime_error("RTreeBase::remove_edge: root cannot be null.");
    }

    if (e.dst->id != dst.id) {
        throw std::invalid_argument("RTreeBase::remove_edge: destination nodes do not match.");
    }


    BoundingBox bb = BoundingBox::from_points(src.lat, src.lon, dst.lat, dst.lon);
    remove_edge_internal(bb, e);
}

/*
 * This function update the search state if the edge is the in the k-nearest seen so far
 * and if the dist from custom position is lesser than MAX_EDGE_DIST.
 *
 * Parameters:
 *  - 'query': bounding box that represent the custom position.
 *  - 'e': pointer to the edge_ptr_t to add to 's'.
 *  - 's': search state structures to update.
 */
void update_k_nearest_edges(const BoundingBox& query, const EdgePtr& e, SearchState& s) {
    PointToSegmentResult result = point_to_segment_distance( query.y_max, query.x_max, e.src->lat, e.src->lon, e.dst->lat, e.dst->lon);

    // discard the edges if it's too distant
    if (result.distance_m > rtree_config::MAX_EDGE_DIST) {
        return;
    }
    // insert the result
    s.insert(e.src, e.edge,result.distance_m, result.proj_lat, result.proj_lon);
}


/*
* Performs a range query on the R-tree starting from 'node'.
* The query is defined as a bounding box (in this case usually a point:
*   x_min = x_max, y_min = y_max).
*
* Parameters:
*  - 'node': pointer to the current R-tree node being searched.
*  - 'query': bounding box representing the custom position.
*  - 's': pointer to the search state structure that stores intermediate results.
*  - 'update_k_nearest_edges': callback function invoked when a matching leaf entry is found.
*/
void RTreeBase::search_internal(const RTreeNode* node, const BoundingBox& query, SearchState& state) {
    if (!node) return;

    for (uint8_t i = 0; i < node->n_entries; i++) {
        if (node->bounding_rects[i].overlaps(query)) {
            if (node->is_leaf()) {
                const auto& leaf_entries = std::get<RTreeNode::LeafEntries>(node->entries);
                // found a matching leaf entry, invoke update_k_nearest_edges
                update_k_nearest_edges(query, *leaf_entries[i], state);
            }
            else {
                const auto& children = std::get<RTreeNode::InternalEntries>(node->entries);
                // recursively search child nodes
                search_internal(children[i].get(), query, state);
            }
        }
    }
}

/*
* This function check the parameters and call search_rtree.
*
* Parameters:
*  - 'query': bounding box representing the custom position.
*/
SearchState RTreeBase::search_k_nearest_edges(const BoundingBox query) {
    if (this->root_ == nullptr) {
        throw std::runtime_error("RTreeBase::search_k_nearest_edges: root cannot be null.");
    }

    SearchState state;
    this->search_internal(this->root_.get(), query, state);

    return state;
}
