#ifndef REINSERT_ENTRIES_HPP
#define REINSERT_ENTRIES_HPP

#include <cstdint>
#include <deque>
#include <variant>
#include <memory>
#include "bounding_box.hpp"
#include "edge_ptr.hpp"

class RTreeNode; // fwd decl
struct TreeEntry;

namespace reinsert_entries_config {
    constexpr uint16_t INITIAL_CAPACITY = 8; // was MIN
}

/*
 * Growable array of TreeEntry, used to collect and reinsert nodes/edges
 * during R-tree/R*-tree condensation or rebalancing. Shared by both tree
 * variants since RTreeNode is the common node type between them.
*/
class ReinsertEntries {
public:
    ReinsertEntries() {  }

    // This function add an element to the reinsert entries vector
    void add(BoundingBox bb, uint16_t tree_node_level, unique_ptr<RTreeNode> node) {
        entries_.emplace_back(bb, std::move(node), tree_node_level);
    }
    void add(BoundingBox bb, uint16_t tree_node_level, unique_ptr<EdgePtr> edge) {
        entries_.emplace_back(bb, std::move(edge), tree_node_level);
    }
    void add(TreeEntry entry) {
        entries_.push_back(std::move(entry));
    }

    // Merge the 'src' with current reinsert entries.
    void merge(ReinsertEntries&& src) {
        entries_.insert(entries_.end(),
                         std::make_move_iterator(src.entries_.begin()),
                         std::make_move_iterator(src.entries_.end()));
        src.entries_.clear();
    }

    // It returns the bounding box of the i-th element
    BoundingBox bounding_box_at(uint8_t i) const { return entries_[i].b_box; }

    // It returns the size of the reinsert entries
    uint16_t size() const noexcept { return static_cast<uint16_t>(entries_.size()); }
    // It return true if the reinsert entries is empty
    bool empty() const noexcept { return entries_.empty(); }
    // Removes all the elements keeping the allocated size
    void clear() noexcept { entries_.clear(); }

    TreeEntry& operator[](uint16_t i) { return entries_[i]; }
    const TreeEntry& operator[](uint16_t i) const { return entries_[i]; }

    // Necessary for iteration
    auto begin() { return entries_.begin(); }
    auto end() { return entries_.end(); }
    auto begin() const { return entries_.begin(); }
    auto end() const { return entries_.end(); }

private:

    // std::deque because are needed pointers/references to existing elements across insertion operation
    std::deque<TreeEntry> entries_;
};


#endif