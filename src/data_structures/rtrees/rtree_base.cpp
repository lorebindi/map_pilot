#include "rtree_base.hpp"

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
bool find_leaf(RTreeNode *root, Edge *edge_query, const BoundingBox &rect_query, uint8_t *pos, TreePath &tree_path, uint16_t level) {
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
                RTreeNode::InternalEntries& internal = std::get<RTreeNode::InternalEntries>(root->entries);
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