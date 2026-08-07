#include <cassert>
#include "rtree_node.hpp"

/*
* Inserts a new entry into this node.
*
* This function assumes that the node has available space. It does not
* perform overflow handling or splitting; the caller is responsible for
* checking n_entries before calling it.
*
* For leaf nodes, the inserted entry must be an EdgePtr.
* For internal nodes, the inserted entry must be an RTreeNode.
*/
void RTreeNode::insert_entry(BoundingBox rect, std::unique_ptr<EdgePtr> edge) {
    assert(is_leaf());
    assert(n_entries < rtree_config::MAX_ENTRIES);

    bounding_rects[n_entries] = rect;

    auto& data = std::get<LeafEntries>(entries);
    data[n_entries] = std::move(edge);

    n_entries++;
}

void RTreeNode::insert_entry(BoundingBox rect, std::unique_ptr<RTreeNode> child) {
    assert(!is_leaf());
    assert(n_entries < rtree_config::MAX_ENTRIES);

    bounding_rects[n_entries] = rect;

    auto& data = std::get<InternalEntries>(entries);
    data[n_entries] = std::move(child);

    n_entries++;
}

void RTreeNode::insert_entry(const BoundingBox& bb, SplitEntry entry) {
    std::visit([this, &bb](auto& ptr) {
        this->insert_entry(bb, std::move(ptr));
    }, entry);
}

void RTreeNode::insert_entry(TreeEntry &&entry) {
    insert_entry(entry.b_box, std::move(entry.data));
}


/*
 * Shifts all entries one position to the left starting from 'start_index',
 * compacting the node after an entry has been removed.
 *
 * Precondition:
 * - The node entries are already stored contiguously in the range
 *   [0, n_entries).
 * - Exactly one "hole" exists at position 'start_index', typically because
 *   the entry at that position has just been moved out.
 *
 * This function fills that hole by shifting all subsequent entries one
 * position to the left. It is not intended for compacting nodes containing
 * multiple holes.
 *
 * Works for both leaf and internal nodes.
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

/*
* Compacts the node by removing any holes left by previously removed entries.
*
* This function scans the occupied portion of the node and moves all valid
* entries towards the beginning of the arrays, preserving their relative
* order. The corresponding bounding rectangles are moved together with their
* entries.
*
* A "hole" is represented by a nullptr in the entries array. After compaction,
* all valid entries occupy the contiguous range [0, n_entries), and n_entries
* is updated to reflect the new number of entries.
*
* Works for both leaf and internal nodes.
*/
void RTreeNode::compact_entries() {
    uint8_t write = 0; // indicates the where to write the next valid entries.

    if (this->is_leaf()) {
        auto& data = std::get<LeafEntries>(this->entries);

        for (uint8_t read = 0; read < this->n_entries; read++) {
            if (data[read] != nullptr) {
                if (write != read) {
                    data[write] = std::move(data[read]);
                    this->bounding_rects[write] = this->bounding_rects[read];
                }
                write++;
            }
        }
        this->n_entries = write;
    }else {
        auto& children = std::get<InternalEntries>(this->entries);

        for (uint8_t read = 0; read < this->n_entries; read++) {
            if (children[read] != nullptr) {
                if (write != read) {
                    children[write] = std::move(children[read]);
                    this->bounding_rects[write] = this->bounding_rects[read];
                }
                write++;
            }
        }
        this->n_entries = write;
    }
}

/*
* They remove an entry from the RTreeNode and return it.
*
* Note: They don't shit entries left, it's a caller's responsibility
*/
std::unique_ptr<EdgePtr> RTreeNode::extract_leaf_entry(uint8_t index) {
    assert(this->is_leaf());
    assert(index < n_entries);

    auto& internal_entries = std::get<LeafEntries>(entries);

    auto removed = std::move(internal_entries[index]);
    --n_entries;

    return removed;
}
std::unique_ptr<RTreeNode> RTreeNode::extract_internal_entry(uint8_t index) {
    assert(!this->is_leaf());

    assert(index < n_entries);

    auto& internal_entries = std::get<InternalEntries>(entries);

    auto removed = std::move(internal_entries[index]);
    --n_entries;

    return removed;
}

/*
* Updates the bounding rectangle of the child node after insertion into one of its
* children (i.e. the one identified by 'child index').
*
* Parameters:
* - 'child_index': the index of the child in 'node->children'.
*
* Use case:
*  This function is called when the minimum bounding rectangle of the just added/modified
*  child need to be (re)calculated.
*/
void RTreeNode::update_child_bounding_rect (uint8_t child_index) {
    assert(child_index < this->n_entries);
    assert(is_leaf());

    auto& children = std::get<RTreeNode::InternalEntries>(this->entries);
    this->bounding_rects[child_index] = get_node_mbr(*children[child_index], children[child_index]->n_entries);
}

/*
 * Adopts two nodes resulting from a child node split. The current node must not be full.
 *
 * Replaces the original split child at 'child_index' with 'child1' and
 * appends 'child2' as a new entry at the end of the node's entries array.
 *
 * Parameters:
 *  - 'index': the index of the child in 'node' that was split.
 *  - 'child1': first resulting node from the split (replaces the original child).
 *  - 'child2': second resulting node from the split (appended to the entries).
 *
 */
void RTreeNode::adopt_split_children(uint8_t index, std::unique_ptr<RTreeNode> child1, std::unique_ptr<RTreeNode> child2) {
    BoundingBox bb1 = get_node_mbr(*child1, child1->n_entries);
    BoundingBox bb2 = get_node_mbr(*child1, child2->n_entries);

    auto& entries = std::get<RTreeNode::InternalEntries>(this->entries);

    this->bounding_rects[index] = bb1;
    entries[index] =  std::move(child1);

    this->bounding_rects[this->n_entries] = bb2;
    entries[this->n_entries] = std::move(child2);
    this->n_entries++;
}

