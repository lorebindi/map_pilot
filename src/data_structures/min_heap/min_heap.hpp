#ifndef MIN_HEAP_HPP
#define MIN_HEAP_HPP

#include <cstdint>
#include <vector>
#include <optional>
#include <limits>

using namespace std;

/*
 * Binary min-min_heap used as a priority queue in shortest-path algorithms
 * such as Dijkstra or A*. Stores graph node IDs and supports efficient
 * retrieval and update (decrease_key) of the node with smallest distance.
 */
class MinHeap {
public:
    MinHeap(uint16_t capacity, uint16_t source);

    // vectors clean up themselves -- no custom destructor, no free_min_heap needed
    ~MinHeap() = default;
    MinHeap(const MinHeap&) = default;
    MinHeap& operator=(const MinHeap&) = default;
    MinHeap(MinHeap&&) noexcept = default;
    MinHeap& operator=(MinHeap&&) noexcept = default;

    // This function extracts minimum node from min_heap.
    optional<uint16_t> extract_min();
    // This function updates the dist value for the node identified by id. Then updates the position for this node.
    void decrease_key(uint16_t id, double key);
    // This function returns true if n satisfy minHeap property
    bool is_in_min_heap(uint16_t id) const noexcept;
    // A utility function to check if the given minHeap is empty or not
    bool is_empty() const noexcept;
    // A utility function to print dist.
    void print_dist() const;

private:
    vector<uint16_t> heap_;      // this vector contains the IDs of the graph's nodes
    vector<uint16_t> position_;  // this vector contains the position in min_heap for each node i (indexed by node id).
    vector<double> dist_;        // this array contains for each node i the value of the distance (indexed by node id).
    uint16_t size_;              // this is the size of min_heap (i.e the number of nodes currently in min_heap).
    uint16_t capacity_;          // this is the size of all'arrays (i.e the total quantity of nodes in the graph)

    // Swap two nodes in minHeap. Needed for minHeapify
    void swap_nodes(uint16_t i, uint16_t j) noexcept;
    // Restores the minHeap property for node n.
    void min_heapify(uint16_t i);
};
#endif //MIN_HEAP_HPP