#include "min_heap.hpp"
#include <iostream>

using namespace std;

MinHeap::MinHeap(uint16_t capacity, uint16_t source)
    : heap_(capacity), position_(capacity), dist_(capacity, numeric_limits<double>::infinity()),
      size_(capacity), capacity_(capacity)
{
    for (uint16_t i = 0; i < capacity; ++i) {
        heap_[i] = i;
        position_[i] = i;
    }
    dist_[source] = 0.0;

    // building the min-min_heap
    for (int i = capacity / 2 - 1; i >= 0; --i) {
        min_heapify(static_cast<uint16_t>(i));
    }
}

// Swap two nodes in minHeap. Needed for minHeapify
void MinHeap::swap_nodes(uint16_t i, uint16_t j) noexcept {
    uint16_t id_i = heap_[i];
    uint16_t id_j = heap_[j];
    heap_[i] = id_j;
    heap_[j] = id_i;
    position_[id_i] = j;
    position_[id_j] = i;
}

// Restores the minHeap property for node n.
void MinHeap::min_heapify(uint16_t i) {
    uint16_t left = 2 * i + 1;
    uint16_t right = 2 * i + 2;
    uint16_t smallest = i;

    if (left < size_ && dist_[heap_[left]] < dist_[heap_[smallest]]) {
        smallest = left;
    }
    if (right < size_ && dist_[heap_[right]] < dist_[heap_[smallest]]) {
        smallest = right;
    }
    if (smallest != i) {
        swap_nodes(smallest, i);
        min_heapify(smallest);
    }
}

// This function extracts minimum node from min_heap.
optional<uint16_t> MinHeap::extract_min() {
    if (size_ == 0) return nullopt;

    uint16_t root = heap_[0];
    uint16_t last_node = heap_[size_ - 1];
    heap_[0] = last_node;
    position_[root] = size_ - 1;
    position_[last_node] = 0;
    --size_;
    min_heapify(0);

    return root;
}

// This function updates the dist value for the node identified by id. Then updates the position for this node.
void MinHeap::decrease_key(uint16_t id, double key) {
    uint16_t i = position_[id];
    dist_[id] = key;
    while (i > 0 && dist_[heap_[i]] < dist_[heap_[(i - 1) / 2]]) {
        swap_nodes(i, (i - 1) / 2);
        i = (i - 1) / 2;
    }
}

// This function returns true if n satisfy minHeap property
bool MinHeap::is_in_min_heap(uint16_t id) const noexcept {
    return position_[id] < size_;
}

// A utility function to check if the given minHeap is empty or not
bool MinHeap::is_empty() const noexcept {
    return size_ == 0;
}

// A utility function to print dist.
void MinHeap::print_dist() const {
    cout << "dist = [";
    for (uint16_t i = 0; i < capacity_ - 1; ++i) {
        cout << dist_[i] << ", ";
    }
    cout << dist_[capacity_ - 1] << "]\n";
}