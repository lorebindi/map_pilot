#ifndef PARENT_CANDIDATES_HPP
#define PARENT_CANDIDATES_HPP

/*
* Non-owning candidate list produced by get_overlapping_parent(): nodes that
* are still live in (and owned by) the rtree, being scanned only to pick the
* best reinsertion parent by minimum enlargement.
*
* Deliberately NOT a TreeEntriesArr: TreeEntriesArr owns its entries (used
* for genuinely detached nodes during condensation).
*/

#include <vector>
#include "bounding_box.hpp"

class RTreeNode;

class ParentCandidates {
public:
    void add(BoundingBox bb, RTreeNode* node) { candidates_.push_back({bb, node}); }

    BoundingBox bounding_box_at(uint8_t i) const { return candidates_[i].bb; }
    uint8_t size() const { return static_cast<uint8_t>(candidates_.size()); }
    RTreeNode* operator[](uint8_t i) const { return candidates_[i].node; }

private:
    struct Candidate { BoundingBox bb; RTreeNode* node; }; // raw pointer to the rtree node --> non-owning
    std::vector<Candidate> candidates_;
};


#endif //PARENT_CANDIDATES_HPP