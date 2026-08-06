#include "r_star_tree.hpp"

#include <numeric>

#include "graph/graph.hpp"

/*
 * Distance from a bounding box's MAX corner (x_max, y_max) to a point
 * (represented as a degenerate bounding box).
 */
float distance_from_center(const BoundingBox& a, const BoundingBox& center) const {
    const float dx = a.x_max - center.x_max;
    const float dy = a.y_max - center.y_max;
    return std::sqrt(dx * dx + dy * dy);
}

/*
 * Orders the MAX_ENTRIES+1 candidates (node's existing entries plus the new
 * entry causing overflow) by distance, descending, from the node's MBR
 * center -- identifying farthest-first candidates for R*-tree reinsertion.
 *
 * NOTE: matches original C semantics exactly -- distance is computed from
 * each entry's bounding box MAX corner (x_max, y_max) to the node's MBR
 * center, not from each entry's own center. This is almost certainly not
 * what the R*-tree paper specifies (true center-to-center distance), but is
 * preserved here for behavioral parity. See distance_from_center below.
 *
 * Index MAX_ENTRIES in the returned order refers to the new entry ('bb');
 * indices [0, MAX_ENTRIES) refer to node->bounding_rects[i].
 */
std::array<uint16_t, rtree_config::MAX_ENTRIES + 1> order_entries_by_center_distance(const RTreeNode* node, const BoundingBox& bb) const {
    const BoundingBox node_mbr_center = get_node_mbr(*node, node->n_entries).center();  // degenerate box: min == max == center

    std::array<float, rtree_config::MAX_ENTRIES + 1> dist{};
    for (uint16_t i = 0; i < rtree_config::MAX_ENTRIES; ++i)
        dist[i] = distance_from_center(node->bounding_rects[i], node_mbr_center);
    dist[rtree_config::MAX_ENTRIES] = distance_from_center(bb, node_mbr_center);

    std::array<uint16_t, rtree_config::MAX_ENTRIES + 1> farthest_sorting = {};
    std::iota(farthest_sorting.begin(), farthest_sorting.end(), 0); // used to populate the 'order' array with numbers
    std::sort(farthest_sorting.begin(), farthest_sorting.end(),[&dist](uint16_t a, uint16_t b) { return dist[a] > dist[b]; });  // farthest first

    return farthest_sorting;
}

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
bool RStarTree::remove_farthest_entries(RTreeNode* node, TreeEntry entry, uint16_t p, uint16_t curr_level) {
    auto order = order_entries_by_center_distance(node, entry.b_box);  // farthest first, [0..MAX_ENTRIES]

    const uint16_t reinsert_level = node->is_leaf() ? curr_level : curr_level + 1;
    bool entry_removed = false;
    std::vector<uint16_t> node_indices_to_remove; // indices into 'node'
    node_indices_to_remove.reserve(p);

    for (uint16_t i = 0; i < p; ++i) {
        if (order[i] == rtree_config::MAX_ENTRIES) {
            // one of the p-farthest entries is 'entry'.
            // it has to be reinserted instead than inserted in 'node'.
            this->session_.to_reinsert.add(std::move(entry));
            entry_removed = true;
        }
        else {
            // take track of the farthest entries to remove from 'node'
            node_indices_to_remove.push_back(order[i]);
        }
    }

    for (uint16_t idx : node_indices_to_remove) {
        BoundingBox removed_bb = node->bounding_box_at(idx);
        if (node->is_leaf()) {
            auto edge_ptr = node->extract_leaf_entry(idx);
            session_.to_reinsert.add(removed_bb, reinsert_level, std::move(edge_ptr));
        } else {
            auto child_ptr = node->extract_internal_entry(idx);
            session_.to_reinsert.add(removed_bb, reinsert_level, std::move(child_ptr));
        }
    }
    // Compacting
    node->compact_entries();
    return entry_removed;
}

/*
* Recursively propagates the updated MBR of 'updated_child' up toward the
* root, starting the search from 'current' (which must be the tree root on
* the initial call).
*
* Parameters:
*  - 'current': node currently being inspected.
*  - 'updated_child': the child node whose MBR has changed (identified by
*    pointer identity, not value).
*
* Returns true if 'current's own bounding rect for the path toward
* 'updated_child' was changed as a result -- signaling the caller (parent)
* that it must update its own stored rect too and continue propagating.
*/
bool propagate_mbr_to_root(RTreeNode* current, const RTreeNode* updated_child) {
    if (current->is_leaf())
        return false;

    auto& entries = std::get<RTreeNode::InternalEntries>(current->entries);

    for (uint16_t i = 0; i < current->n_entries; ++i) {
        BoundingBox updated_mbr = get_node_mbr(*updated_child, updated_child->n_entries);
        if (entries[i].get() == updated_child) {
            BoundingBox old_mbr = current->bounding_rects[i];
            current->bounding_rects[i] = updated_mbr;
            return !old_mbr.equals(current->bounding_rects[i]);
        }
        else if (!entries[i]->is_leaf() && updated_mbr.overlaps(current->bounding_rects[i])) {
            if (propagate_mbr_to_root(entries[i].get(), updated_child)) {
                BoundingBox old_mbr = current->bounding_rects[i];
                current->bounding_rects[i] = get_node_mbr(*entries[i], entries[i]->n_entries);
                return !old_mbr.equals(current->bounding_rects[i]);
            }
        }
    }
    return false;
}

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
void RStarTree::forced_reinsert(RTreeNode* node, TreeEntry entry, uint16_t curr_level) {
    const uint16_t p = static_cast<uint16_t>(((rtree_config::MAX_ENTRIES + 1) * rtree_config::REINSERT_PERCENT + 99) / 100);
    bool entry_removed = this->remove_farthest_entries(node, std::move(entry), p, curr_level);

    /* update the MBR of 'node' and propagate the changes up to the root. */
    propagate_mbr_to_root(this->root_.get(), node);

    // 'entry' was placed into this->session_.to_reinsert
    if (entry_removed) return;

    // 'entry' have to be inserted in 'node'
    if (node->is_leaf()) {
        auto edge_ptr = std::get<std::unique_ptr<EdgePtr>>(std::move(entry.data));
        node->insert_entry(entry.b_box, std::move(edge_ptr));
    } else {
        auto child_ptr = std::get<std::unique_ptr<RTreeNode>>(std::move(entry.data));
        node->insert_entry(entry.b_box, std::move(child_ptr));
    }
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
InsertResult RStarTree::overflow_treatment(RTreeNode* node, TreeEntry new_entry, uint16_t curr_level) {
    // if 'node' is not the root and that level is not yet overflowed, then reinsert
    if (curr_level != 0 && !session_.overflowed_level[curr_level]) {
        this->session_.overflowed_level[curr_level] = true;
        // 'reinsert' keeps the closest entries in 'node' and pushes the
        // farthest ones onto session_.to_reinsert directly.
        this->forced_reinsert( node, std::move(new_entry), curr_level);
        return InsertResult{false, nullptr, nullptr};
    }

    // otherwise, split.
    assert(curr_level == 0 || session_.overflowed_level[curr_level]);

    auto [group1, group2] = split(node, new_entry.b_box, std::move(new_entry));
    return InsertResult{true, std::move(group1), std::move(group2)};
}

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
InsertResult RStarTree::insert_edge_internal(RTreeNode* node, BoundingBox rect, std::unique_ptr<EdgePtr> e) {
    if (node->is_leaf()) {
        if (node->n_entries < rtree_config::MAX_ENTRIES) {
            node->insert_entry(rect, std::move(e));
            return {false, nullptr, nullptr};
        }
        else {
            // overflow treatment
            TreeEntry new_entry;
            new_entry.b_box = rect;
            new_entry.data = std::move(e);
            InsertResult overflow_result = overflow_treatment(node, std::move(new_entry), 0);
        }
    }
}


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
void RStarTree::insert_edge(const Node& src, const Node& dst, const Edge& edge) {
    if (edge.y != dst.id)
        throw std::invalid_argument("RStarTree::insert_edge: destination nodes aren't the same.");

    BoundingBox bb = BoundingBox::from_points(src.lat, src.lon, dst.lat, dst.lon);
    auto e = std::make_unique<EdgePtr>(EdgePtr{&src, &dst, &edge});
    // reset the InsertSession attribute
    this->session_reset();
    // invoking the recursive insertion
    InsertResult result = this->insert_edge_internal(this->root_.get(), bb, std::move(e));
    // update the root
    if (result.split)
        create_new_root(std::move(result));
    // reinsertion of to-reinsert entries
    this->reinsert_nodes(this->session_.to_reinsert);
}