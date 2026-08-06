#include "rtree.hpp"
#include "graph/graph.hpp"

/* This function implements node quadratic splitting, proposed
 * by Guttman (1984). It ensures that:
 *  - The tree does not exceed its defined capacity.
 *  - Queries remain efficient by minimizing bounding box overlap.
 *  - The tree remains balanced, avoiding performance degradation.
 *
 * Parameters:
 *  - node: the node to be split (full node)
 *  - new_rect: bounding box of the new entry to be inserted
 *  - new_entry: pointer to the new entry (i.e. 'rtree_node_t *' in case
 *      of 'node' is an internal node or 'edge_ptr_t *' in case is a leaf)
 *
 * Return value: a pair containing the two new RTreeNodes.
 */
std::pair<std::unique_ptr<RTreeNode>, std::unique_ptr<RTreeNode>> quadratic_split(RTreeNode* node, const BoundingBox& new_rect, SplitEntry new_entry) {
    assert(node != nullptr);
    assert(node->n_entries == rtree_config::MAX_ENTRIES);

    auto group1 = std::make_unique<RTreeNode>(node->is_leaf());
    auto group2 = std::make_unique<RTreeNode>(node->is_leaf());

    // Temporary storage containing all entries: existing entries + the new overflowing entry.
    std::array<BoundingBox, rtree_config::MAX_ENTRIES + 1> temp_bb;
    std::array<SplitEntry, rtree_config::MAX_ENTRIES + 1> temp_entries;

    // Move existing entries from the node into temporary storage
    for (uint8_t i = 0; i < node->n_entries; i++) {
        temp_bb[i] = node->bounding_rects[i];
        if (node->is_leaf()) {
            auto& entries = std::get<RTreeNode::LeafEntries>(node->entries);
            temp_entries[i] = std::move(entries[i]);
        } else {
            auto& children = std::get<RTreeNode::InternalEntries>(node->entries);
            temp_entries[i] = std::move(children[i]);
        }
    }

    // Adding overflowing entry
    temp_bb[node->n_entries] = new_rect;
    temp_entries[node->n_entries] = std::move(new_entry);

    /* Quadratic_split: First, chose two seeds (i.e. bounding_rects) by considering
     * all pairs of entries and determining the most wasteful if put together and separate them. */
    uint8_t seed1 = 0;
    uint8_t seed2 = 1;
    float max_d = -1.0f;
    for (uint8_t i = 0; i < rtree_config::MAX_ENTRIES + 1; i++) {
        for (uint8_t j = i + 1; j < rtree_config::MAX_ENTRIES + 1; j++) {
            float d = temp_bb[i].enlargement_needed(temp_bb[j]);
            if (d > max_d) {
                max_d = d;
                seed1 = i;
                seed2 = j;
            }
        }
    }

    group1->insert_entry(temp_bb[seed1], std::move(temp_entries[seed1]));
    group2->insert_entry(temp_bb[seed2], std::move(temp_entries[seed2]));

    // Quadratic_split: Assign each remaining entry to the group where it causes the smallest increase in MBR area */
    uint8_t remaining = rtree_config::MAX_ENTRIES + 1 - 2;
    for (uint8_t i = 0; i < rtree_config::MAX_ENTRIES + 1; i++) {
        if (i == seed1 || i == seed2) continue;
        // if one group has few entries all the rest must be assigned to it, in order to guarantees 'MIN_ENTRIES'
        if (group1->n_entries + remaining == rtree_config::MIN_ENTRIES) {
            group1->insert_entry(temp_bb[i], std::move(temp_entries[i]));
            remaining--;
            continue;
        }
        if (group2->n_entries + remaining == rtree_config::MIN_ENTRIES) {
            group2->insert_entry(temp_bb[i], std::move(temp_entries[i]));
            remaining--;
            continue;
        }

        // Retrieves the minimun bounding rectangle of the two group.
        BoundingBox mbr1 = get_node_mbr(*group1, group1->n_entries);
        BoundingBox mbr2 = get_node_mbr(*group2, group2->n_entries);

        float enlargement1 = mbr1.enlargement_needed(temp_bb[i]);
        float enlargement2 = mbr2.enlargement_needed(temp_bb[i]);

        if (enlargement1 < enlargement2)
            group1->insert_entry(temp_bb[i], std::move(temp_entries[i]));
        else {
            group2->insert_entry(temp_bb[i], std::move(temp_entries[i]));
        }
        remaining--;
    }

    return {std::move(group1), std::move(group2)};
}

/*
* Splits a full node (leaf or internal) using the quadratic split algorithm.
*
* The split creates two new groups of entries. These groups are inserted
* into a new parent node, which is returned as the replacement subtree.
*
* Parameters:
*  - 'node': pointer to the full node to split.
*  - 'rect': bounding box of the new entry that caused the overflow.
*  - 'entry': the new entry to insert during the split. It can be either
*             an EdgePtr (leaf insertion) or an RTreeNode (internal insertion).
*  - 'is_root': true if the node being split is the current root.
*
* Returns:
*  - InsertResult containing:
*     - split = true because the node was split.
*     - replacement = the new parent node containing the two split groups.
*/
InsertResult split_node_and_propagate(RTreeNode* node, BoundingBox rect, SplitEntry entry) {
    assert(node != nullptr);
    assert(node->n_entries == rtree_config::MAX_ENTRIES);

    auto [group1, group2] = quadratic_split(node, rect, std::move(entry));

    return {true, std::move(group1), std::move(group2)};
}

/*
* Propagates a child split upward when the target internal node is full.
*
* Handles an overflow in a full internal node caused by a split child. It replaces
* the original child at `child_index` with `group1`, updates its MBR, and then
* performs a quadratic split on `node` by incorporating `group2`.
*
* Parameters:
*   - 'node': pointer to the full internal node to be split.
*   - 'group1': first resulting node from the child split (replaces child at child_index)
*   - 'group2': second resulting node from the child split (inserted during quadratic split).
*   - 'child_index': index of the child node that was originally split.
*/
InsertResult forward_split(RTreeNode* node, std::unique_ptr<RTreeNode> group1, std::unique_ptr<RTreeNode> group2, uint8_t child_index) {
    assert(node != nullptr);
    assert(group1 != nullptr);
    assert(group2 != nullptr);
    assert(!node->is_leaf());

    // This node is full: replace the old child with the first split group.
    auto& children = std::get<RTreeNode::InternalEntries>(node->entries);
    children[child_index] = std::move(group1);
    node->update_child_bounding_rect(child_index);

    // Split this node by inserting the second split group.
    auto [new_group1, new_group2] = quadratic_split(node,get_node_mbr(*group2, group2->n_entries), SplitEntry{std::move(group2)} );

    return {true, std::move(new_group1), std::move(new_group2)};
}

/*
* Inserts an edge into the appropriate leaf node of the R-tree.
* If the node overflows, a quadratic split is performed.
* Splits may propagate up the tree, possibly requiring a new root.
*
* Parameters:
*  - 'node': pointer to the current node (can be internal or leaf) where insertion is attempted.
*  - 'rect': bounding box of the edge to be inserted.
*  - 'e': pointer to the EdgePtr structure that wraps the graph edge and its endpoints.
*  - 'is_root': boolean flag indicating whether 'node' is the root (important for split handling)
*/
InsertResult RTree::insert_edge_internal(RTreeNode *node, BoundingBox rect, std::unique_ptr<EdgePtr> e) {
    if (node->is_leaf()) {
        if (node->n_entries < rtree_config::MAX_ENTRIES) {
            node->insert_entry(rect, std::move(e));
            return {false, nullptr, nullptr};
        }
        else {
            // need to split the leaf returning the two new nodes
            return split_node_and_propagate(node, rect, std::move(e));
        }
    }
    // Internal node: find the best child for insertion.
    // The best child is the one requiring the least enlargement of its bounding rectangle.
    uint8_t best_child = get_area_min_enlargement_index(*node, node->n_entries, rect);
    auto& children = std::get<RTreeNode::InternalEntries>(node->entries);
    RTreeNode *original_child = children[best_child].get();
    InsertResult result = insert_edge_internal(original_child, rect, std::move(e));

    // No split occurred: update the bounding rectangle and return.
    if (!result.split) {
        node->update_child_bounding_rect(best_child);
        return {false, nullptr, nullptr};
    }

    // A split occurred in the child: free the original child and handle the two resulting nodes.
    if (node->n_entries < rtree_config::MAX_ENTRIES) {
        node->adopt_split_children(best_child, std::move(result.group1), std::move(result.group2));
        return {false, nullptr, nullptr};
    }
    else {
        return forward_split(node, std::move(result.group1), std::move(result.group2), best_child);
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
void RTree::insert_edge(const Node& src, const Node& dst, const Edge& edge) {
    if (edge.y != dst.id)
        throw std::invalid_argument("RTree::insert_edge: destination nodes aren't the same.");

    BoundingBox bb = BoundingBox::from_points(src.lat, src.lon, dst.lat, dst.lon);
    auto e = std::make_unique<EdgePtr>(EdgePtr{&src, &dst, &edge});

    InsertResult result = this->insert_edge_internal(this->root_.get(), bb, std::move(e));

    if (result.split)
        create_new_root(std::move(result));
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
InsertResult RTree::insert_internal_node(RTreeNode* node, const RTreeNode* best_parent, TreeEntry& reinsert, uint16_t curr_level) {
    assert(node != nullptr);
    assert(!reinsert.is_data_entry());

    // No need to go down under the tree level of the node to reinsert
    if (curr_level > reinsert.tree_node_level - 1)
        return {false, nullptr, nullptr};

    // Reached the chosen parent
    if (node == best_parent) {
        // If 'node' is not full then the reinsertion happen here
        if (node->n_entries < rtree_config::MAX_ENTRIES) {
            node->insert_entry(reinsert.b_box, std::get<std::unique_ptr<RTreeNode>>(std::move(reinsert.data)));
            return {false, nullptr, nullptr};
        }
        // Otherwise we need to split the 'node' and propagate
        return split_node_and_propagate(node, reinsert.b_box, std::move(std::get<std::unique_ptr<RTreeNode>>(reinsert.data)));
    }
    else {
        auto& children = std::get<RTreeNode::InternalEntries>(node->entries);
        for (uint8_t i = 0; i < node->n_entries; ++i) {
            if (!node->bounding_rects[i].overlaps(reinsert.b_box))
                continue;
            else {
                InsertResult result = insert_internal_node(children[i].get(), best_parent, reinsert, curr_level + 1);
                // Child absorbed the insertion.
                if (!result.split) {
                    node->update_child_bounding_rect(i);
                    return {false, nullptr, nullptr};
                }
                else {
                    // Child was split.
                    if (node->n_entries < rtree_config::MAX_ENTRIES) {
                        node->adopt_split_children(i, std::move(result.group1), std::move(result.group2));
                        return {false, nullptr, nullptr};
                    }
                    else {
                        return forward_split(node, std::move(result.group1), std::move(result.group2), i);
                    }
                }


            }
        }
    }
    assert(false && "insert_internal_node: best_parent not found");
    std::abort();
}


