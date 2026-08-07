#include "r_star_tree.hpp"

#include <numeric>
#include <set>

#include "graph/graph.hpp"

/*
 * Distance from a bounding box's MAX corner (x_max, y_max) to a point
 * (represented as a degenerate bounding box).
 */
float distance_from_center(const BoundingBox& a, const BoundingBox& center) {
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
std::array<uint16_t, rtree_config::MAX_ENTRIES + 1> order_entries_by_center_distance(const RTreeNode* node, const BoundingBox& bb) {
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
bool RStarTree::remove_farthest_entries(RTreeNode *node, TreeEntry &entry, uint16_t p, uint16_t curr_level) {
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
void RStarTree::forced_reinsert(RTreeNode* node, TreeEntry &entry, uint16_t curr_level) {
    const uint16_t p = static_cast<uint16_t>(((rtree_config::MAX_ENTRIES + 1) * rtree_config::REINSERT_PERCENT + 99) / 100);
    bool entry_removed = this->remove_farthest_entries(node, entry, p, curr_level);

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
 * Determines the best axis (X or Y) along which to split a full node in the R*-tree
 * when inserting a new bounding box.
 *
 * The R*-tree split heuristic evaluates candidate splits along each axis by:
 * 1. Sorting all bounding boxes (existing + new) by their lower and upper
 *    values along the axis (xmin/xmax or ymin/ymax).
 * 2. Generating all possible distributions for each sorting of the bounding boxes.
 *    With the term 'distribution' is intended a possible division of the bounding
 *    boxes in two groups, respecting the minimum fill requirement (MIN_ENTRIES).
 * 3. For each distribution, computing the margin (perimeter) of the MBR that
 *    contains each of the two group.
 * 4. Summing the margins across all candidate distributions for the axis.
 *
 * The axis with the smallest total margin is chosen as the split axis, since
 * it tends to produce more compact child nodes and consequently reduces overlap.
 *
 * Parameters:
 *  - node: pointer to the R*-tree node being split.
 *  - new_bb: bounding box of the entry to be inserted.
 *  - bb_xmin_ordered: output array of pointers to bounding boxes ordered by xmin.
 *  - bb_xmax_ordered: output array of pointers to bounding boxes ordered by xmax.
 *  - bb_ymin_ordered: output array of pointers to bounding boxes ordered by ymin.
 *  - bb_ymax_ordered: output array of pointers to bounding boxes ordered by ymax.
 *
 *  Returns: the best axis 'X' or 'Y'
 */
char choose_split_axis (
    const std::array<TreeEntry*, rtree_config::MAX_ENTRIES + 1> &xmin_ordered,
    const std::array<TreeEntry*, rtree_config::MAX_ENTRIES + 1> &xmax_ordered,
    const std::array<TreeEntry*, rtree_config::MAX_ENTRIES + 1> &ymin_ordered,
    const std::array<TreeEntry*, rtree_config::MAX_ENTRIES + 1> &ymax_ordered
    ) {
    // margins (perimeters) sum for each axis
    float sum_x = 0.0f;
    float sum_y = 0.0f;

    // computing for X-axis the sum of all margin of all minimum bounding boxes that include each distribution.
    for (int k = rtree_config::MIN_ENTRIES; k <= rtree_config::MAX_ENTRIES - rtree_config::MIN_ENTRIES + 1; k++) {
        sum_x += calculate_margin(xmin_ordered, 0, k) + calculate_margin(xmin_ordered, k, (rtree_config::MAX_ENTRIES+1) - k);
        sum_x += calculate_margin(xmax_ordered, 0, k) + calculate_margin(xmax_ordered, k, (rtree_config::MAX_ENTRIES+1) - k);
    }

    // computing for Y-axis the sum of all margin of all minimum bounding boxes that include each distribution.
    for (int k = rtree_config::MIN_ENTRIES; k <= rtree_config::MAX_ENTRIES - rtree_config::MIN_ENTRIES + 1; k++) {
        sum_y += calculate_margin(ymin_ordered, 0, k) + calculate_margin(ymin_ordered, k, (rtree_config::MAX_ENTRIES+1) - k);
        sum_y += calculate_margin(ymax_ordered, 0, k) + calculate_margin(ymax_ordered, k, (rtree_config::MAX_ENTRIES+1) - k);
    }

    if (sum_x < sum_y) return 'X';
    else return 'Y';
}

/*
 * Evaluates one candidate distribution (order[0..k) vs order[k..MAX_ENTRIES+1))
 * and updates the running best (index, is_max flag) if this distribution is
 * better than the best seen so far -- first by minimizing overlap area
 * between the two groups' MBRs, then (tie-break) by minimizing total area.
 *
 * Parameters:
 *  - 'entries': the MAX_ENTRIES+1 candidate tree entries.
 *  - 'k': candidate split point.
 *  - 'is_max': whether 'order' is a max-ordering.
 *  - 'index' / 'minimum_overlapping_area' / 'minimum_distribution_area' / 'result':
 *    running best state, updated in place.
 */
void check_distribution_goodness(
    const std::array<TreeEntry*, rtree_config::MAX_ENTRIES + 1> &entries,
    uint16_t k, bool is_max, uint16_t &index, float &minimum_overlapping_area,
    float &minimum_distribution_area, bool &result) {

    //  computing the minimun bounding rectangle of the two groups of the distribution.
    BoundingBox mbr1 = range_array_mbr(entries, 0, k);
    BoundingBox mbr2 = range_array_mbr(entries, k, (rtree_config::MAX_ENTRIES + 1) - k);
    // computing the overlapping area of the two groups (i.e. mbr1 and mbr2).
    float curr_overlap_area = mbr1.overlapping_bounding_box(mbr2).area();
    // if the value of overlapping area is the minimum seen so far, store the distribution.
    if (curr_overlap_area < minimum_overlapping_area) {
        index = k;
        minimum_overlapping_area = curr_overlap_area;
        result = is_max;
        return;
    }
    // if the value of overlapping area is equal to the minimum seen so far, store the distribution
    // only if it's area is smaller that the one seen so far.
    if (curr_overlap_area == minimum_overlapping_area) {
        float curr_area = mbr1.area() + mbr2.area();
        if (curr_area < minimum_distribution_area) {
            minimum_distribution_area = curr_area;
            index = k;
            result = is_max;
        }
    }
}

/*
 * Chooses the distribution index along one axis that minimizes overlap
 * (tie-broken by total area) between the two resulting groups, considering
 * both the min-ordering and max-ordering of that axis.
 *
 * Parameters:
 *  - 'bb_storage': the MAX_ENTRIES+1 candidate bounding boxes.
 *  - 'min_ordered' / 'max_ordered': index orderings for this axis, sorted
 *    by min and max coordinate respectively.
 *  - 'index': output, set to the chosen split point.
 *
 * Returns true if the best distribution came from 'max_ordered', false if
 * it came from 'min_ordered' -- the caller uses this to pick which ordering
 * to physically split by.
 */
bool choose_split_index(
    const std::array<TreeEntry*, rtree_config::MAX_ENTRIES + 1> &min_ordered,
    const std::array<TreeEntry*, rtree_config::MAX_ENTRIES + 1> &max_ordered,
    uint16_t& index) {

    float minimum_overlapping_area = std::numeric_limits<float>::max();
    float minimum_distribution_area = std::numeric_limits<float>::max();
    bool result = false;  // false -> min_ordered, true -> max_ordered

    for (uint16_t k = rtree_config::MIN_ENTRIES - 1; k <= rtree_config::MAX_ENTRIES - rtree_config::MIN_ENTRIES; ++k) {
        check_distribution_goodness(min_ordered, k, false, index, minimum_overlapping_area, minimum_distribution_area, result);
        check_distribution_goodness(max_ordered, k, true, index, minimum_overlapping_area, minimum_distribution_area, result);
    }
    return result;
}

/*
 * Builds four pointer orderings over 'bb_storage' (size MAX_ENTRIES+1), one
 * per axis-extremum: sorted by x_min, x_max, y_min, y_max respectively.
 * Used by choose_split_axis / choose_split_index to evaluate candidate
 * distributions along both axes without re-sorting.
 *
 * Parameters:
 *  - 'bb_storage': the MAX_ENTRIES+1 candidate bounding boxes to be split
 *    (node's existing entries plus the new entry), indexed consistently
 *    with the caller (split()).
 *
 * Returns a tuple of four read-only BoundingBox pointers arrays, each a permutation of
 * [0, MAX_ENTRIES], sorted ascending by the corresponding coordinate.
 */
std::tuple<std::array<TreeEntry*, rtree_config::MAX_ENTRIES + 1>, std::array<TreeEntry*, rtree_config::MAX_ENTRIES + 1>,
           std::array<TreeEntry*, rtree_config::MAX_ENTRIES + 1>, std::array<TreeEntry*, rtree_config::MAX_ENTRIES + 1>>
build_split_orderings(std::array<TreeEntry, rtree_config::MAX_ENTRIES + 1> &bb_storage) {
    std::array<TreeEntry*, rtree_config::MAX_ENTRIES + 1> xmin_order, xmax_order, ymin_order, ymax_order;

    for (uint8_t i = 0; i < bb_storage.size(); i++) {
        xmin_order[i] = &bb_storage[i];
        xmax_order[i] = &bb_storage[i];
        ymin_order[i] = &bb_storage[i];
        ymax_order[i] = &bb_storage[i];
    }

    std::sort(xmin_order.begin(), xmin_order.end(), [](const TreeEntry* a, const TreeEntry* b) {return a->b_box.x_min < b->b_box.x_min;});
    std::sort(xmax_order.begin(), xmax_order.end(), [](const TreeEntry* a, const TreeEntry* b) {return a->b_box.x_max < b->b_box.x_max;});
    std::sort(ymin_order.begin(), ymin_order.end(), [](const TreeEntry* a, const TreeEntry* b) {return a->b_box.y_min < b->b_box.y_min;});
    std::sort(ymax_order.begin(), ymax_order.end(), [](const TreeEntry* a, const TreeEntry* b) {return a->b_box.y_max < b->b_box.y_max;});

    return {xmin_order, xmax_order, ymin_order, ymax_order};
}

/*
 * Splits the (full) 'node' into two new nodes 'group1'/'group2', following
 * the R*-tree split algorithm: choose the axis (X or Y) whose min/max
 * sortings minimize total margin, then choose the distribution index k along
 * that axis minimizing overlap (tie-broken by area). Entries [0..k] go to
 * group1, entries (k..MAX_ENTRIES] go to group2.
 *
 * Parameters:
 *  - 'node': the full node being split (its entries are moved out; the node
 *    itself is left empty and is expected to be discarded by the caller).
 *  - 'new_bb' / 'new_entry': the entry that triggered the split, treated as
 *    one of the MAX_ENTRIES+1 candidates alongside node's existing entries.
 *
 * Returns the two resulting nodes as owning pointers.
 */
std::pair<std::unique_ptr<RTreeNode>, std::unique_ptr<RTreeNode>> split(RTreeNode* node, TreeEntry &new_entry) {
    assert(node != nullptr);
    assert(node->n_entries == rtree_config::MAX_ENTRIES);
    // defensive check: new_entry must still own its payload at entry to
    // split() -- it must not have been moved-from by an earlier call in the
    // same operation (e.g. a previous overflow_treatment/reinsert step that
    // consumed it without this being the terminal use).
    assert((node->is_leaf()
        ? std::holds_alternative<std::unique_ptr<EdgePtr>>(new_entry.data) &&
          std::get<std::unique_ptr<EdgePtr>>(new_entry.data) != nullptr
        : std::holds_alternative<std::unique_ptr<RTreeNode>>(new_entry.data) &&
          std::get<std::unique_ptr<RTreeNode>>(new_entry.data) != nullptr)
        && "split: new_entry already moved-from");

    // moving node's entries and 'new_entry' in 'bb_storage', now it's the owner
    std::array<TreeEntry, rtree_config::MAX_ENTRIES + 1> bb_storage;
    for (uint16_t i = 0; i < rtree_config::MAX_ENTRIES; ++i) {
        if (node->is_leaf())
            bb_storage[i] = TreeEntry(node->bounding_box_at(i), node->extract_leaf_entry(i));
        else
            bb_storage[i] = TreeEntry(node->bounding_box_at(i), node->extract_internal_entry(i));
    }
    bb_storage[rtree_config::MAX_ENTRIES] = std::move(new_entry);

    // building arrays of pointers to the entries of 'bb_storage' sorted according to axis
    auto [xmin_order, xmax_order, ymin_order, ymax_order] = build_split_orderings(bb_storage);
    // choose the axis that guarantees to produce more compact child nodes.
    char axis = choose_split_axis(xmin_order, xmax_order, ymin_order, ymax_order);

    uint16_t k;
    bool use_max_ordering;
    if (axis == 'X')
        use_max_ordering = choose_split_index(xmin_order, xmax_order, k);
    else
        use_max_ordering = choose_split_index(ymin_order, ymax_order, k);

    // catch the definitive order
    auto& def_order = (axis == 'X')
        ? (use_max_ordering ? xmax_order : xmin_order)
        : (use_max_ordering ? ymax_order : ymin_order);
    // sanity check: def_order must be a permutation of [0, MAX_ENTRIES].
    assert(std::set<const TreeEntry*>(def_order.begin(), def_order.end()).size() == rtree_config::MAX_ENTRIES + 1);

    // building the two group of definitive distribution.
    auto group1 = std::make_unique<RTreeNode>(node->is_leaf());
    auto group2 = std::make_unique<RTreeNode>(node->is_leaf());

    for (size_t i = 0; i <= k; i++) {
        group1->insert_entry(std::move(*def_order[i]));
    }
    for (size_t i = k; i <= rtree_config::MAX_ENTRIES; i++) {
        group2->insert_entry(std::move(*def_order[i]));
    }

    return {std::move(group1), std::move(group2)};
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
InsertResult RStarTree::overflow_treatment(RTreeNode *node, TreeEntry &new_entry, uint16_t curr_level) {
    // if 'node' is not the root and that level is not yet overflowed, then reinsert
    if (curr_level != 0 && !session_.overflowed_level[curr_level]) {
        this->session_.overflowed_level[curr_level] = true;
        // 'reinsert' keeps the closest entries in 'node' and pushes the
        // farthest ones onto session_.to_reinsert directly.
        this->forced_reinsert( node, new_entry, curr_level);
        return InsertResult{false, nullptr, nullptr};
    }

    // otherwise, split.
    assert(curr_level == 0 || session_.overflowed_level[curr_level]);

    auto [group1, group2] = split(node, new_entry);
    return InsertResult{true, std::move(group1), std::move(group2)};
}

/*
 * Selects the best child leaf of an internal node in which to insert the new
 * bounding box 'bb', following the R*-tree ChooseSubtree policy when the
 * children of 'node' are leaves.
 *
 * Selection criteria (in order of priority):
 *
 * 1. Minimum sum of overlap areas with sibling nodes after enlargement.
 * 2. Minimum area enlargement required to include 'bb'.
 * 3. Minimum original area.
 *
 * Parameters:
 *  - 'node': parent node containing candidate leaf nodes.
 *  - 'bb': bounding box to insert.
 *
 * Returns:
 *  - Index of the best child node.
 */
uint8_t choose_leaf_child(const RTreeNode* node, const BoundingBox& bb) {
    assert(node != nullptr);
    assert(!node->is_leaf());
    assert(node->n_entries > 0);

    const auto& children = std::get<RTreeNode::InternalEntries>(node->entries);
    assert(children[0]->is_leaf());

    uint8_t best_child = std::numeric_limits<uint8_t>::max();

    // minimum sum of overlapping area seen so far.
    float min_sum_overlap_area = std::numeric_limits<float>::infinity();
    // minimum enlargement area required
    float min_enlargement_area = std::numeric_limits<float>::infinity();
    float min_area = std::numeric_limits<float>::infinity();

    for (uint8_t i = 0; i < node->n_entries; ++i) {
        // calculate the minimum bb that includes the i-th bounding box of 'node' and 'bb'.
        BoundingBox temp_enlargement = node->bounding_box_at(i).union_bounding_box(bb);
        // required enlargement area to include the i-th bounding box of 'node' and 'bb'.
        float enlargement_area = node->bounding_box_at(i).enlargement_needed(bb);
        float original_area = node->bounding_box_at(i).area();
        float sum_overlap_area = 0.0f;

        // Compute the sum of overlap areas between the enlarged MBR of the selected child and all its siblings.
        for (uint8_t j = 0; j < node->n_entries; ++j) {
            if (i == j) continue;
            sum_overlap_area += temp_enlargement.overlapping_bounding_box(node->bounding_box_at(j)).area();
        }
        if (sum_overlap_area < min_sum_overlap_area ||
            (sum_overlap_area == min_sum_overlap_area &&
             enlargement_area < min_enlargement_area) ||
            (sum_overlap_area == min_sum_overlap_area &&
             enlargement_area == min_enlargement_area &&
             original_area < min_area)) {

            min_sum_overlap_area = sum_overlap_area;
            min_enlargement_area = enlargement_area;
            min_area = original_area;

            best_child = i;
             }
    }

    assert(best_child != std::numeric_limits<uint8_t>::max());

    return best_child;
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
InsertResult RStarTree::insert_edge_internal(RTreeNode* node, TreeEntry &new_entry) {
    if (node->is_leaf()) {
        if (node->n_entries < rtree_config::MAX_ENTRIES) {
            node->insert_entry(new_entry.b_box, std::move(std::get<std::unique_ptr<EdgePtr>>(std::move(new_entry.data))));
            return {false, nullptr, nullptr};
        }
        else {
            // overflow treatment
            InsertResult overflow_result = overflow_treatment(node, new_entry, 0);
        }
    }
    // 'node' is an internal node
    uint8_t best_child = std::numeric_limits<uint8_t>::max();
    auto& children = std::get<RTreeNode::InternalEntries>(node->entries);
    // If children are leaves, choose the leaf with the minimum overlap/enlargement according to the R*-tree leaf selection heuristic.
    if (children[0]->is_leaf()) {
        best_child = choose_leaf_child(node, new_entry.b_box);
    } else {
        // Children are internal nodes: choose the child requiring the minimum enlargement of its bounding rectangle.
        best_child = get_area_min_enlargement_index(*node, node->n_entries, new_entry.b_box);
    }

    RTreeNode* original_child = children[best_child].get();
    InsertResult result = insert_edge_internal(original_child, new_entry);

    // No split occurred: update the MBR of the modified child.
    if (!result.split) {
        node->update_child_bounding_rect(best_child);
        return {false, nullptr, nullptr};
    }

    // A split occurred in the child: free the original child and handle the two resulting nodes.
    auto child_group_1 = std::move(result.group1);
    auto child_group_2 = std::move(result.group2);


    if (node->n_entries < rtree_config::MAX_ENTRIES) {
        node->adopt_split_children(best_child,std::move(child_group_1), std::move(child_group_2));
        return {false, nullptr, nullptr};
    }
    else {
        // Keep the first group in the old slot and insert the second group as the overflowing entry.
        node->bounding_rects[best_child] = get_node_mbr(*child_group_1, child_group_1->n_entries);
        children[best_child] = std::move(child_group_1);
        TreeEntry overflow_entry(get_node_mbr(*child_group_2, child_group_2->n_entries), std::move(child_group_2));
        return overflow_treatment(node, overflow_entry, false);
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
    TreeEntry new_entry;
    new_entry.b_box = bb;
    new_entry.data = std::move(e);
    InsertResult result = this->insert_edge_internal(this->root_.get(), new_entry);
    // update the root
    if (result.split)
        create_new_root(std::move(result));
    // reinsertion of to-reinsert entries
    this->reinsert_nodes(this->session_.to_reinsert);
}