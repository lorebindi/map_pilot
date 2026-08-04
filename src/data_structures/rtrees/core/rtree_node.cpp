#include <cassert>
#include "../rtree_node.hpp"
/*
 * Shifts entries left starting from 'start_index', compacting the array
 * after an entry has been removed. Works for both leaf and internal nodes.
 */
void RTreeNode::shift_entries_left(uint8_t start_index) {
    assert(start_index < n_entries);

    if (is_leaf()) {
        auto& data = std::get<LeafEntries>(entries);
        for (uint8_t i = start_index; i < n_entries - 1; i++) {
            data[i] = std::move(data[i + 1]);
            bounding_rects[i] = bounding_rects[i + 1];
        }
        // data[n_entries - 1] is already nullptr here
    } else {
        auto& children = std::get<InternalEntries>(entries);
        for (uint8_t i = start_index; i < n_entries - 1; i++) {
            children[i] = std::move(children[i + 1]);
            bounding_rects[i] = bounding_rects[i + 1];
        }
        // children[n_entries - 1] is already nullptr here
    }

    n_entries--;
}

void RTreeNode::update_child_bounding_rect (uint8_t child_index) {
    assert(child_index < this->n_entries);
    assert(is_leaf());

    auto& children = std::get<RTreeNode::InternalEntries>(this->entries);
    this->bounding_rects[child_index] = get_node_mbr(*children[child_index], children[child_index]->n_entries);
}

