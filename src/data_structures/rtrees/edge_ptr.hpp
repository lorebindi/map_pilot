#ifndef EDGE_PTR_HPP
#define EDGE_PTR_HPP

class Node; // fwd decl
class Edge; // fwd decl

/*
 * Represents the content of a single leaf entry in the R-tree/R*-tree.
 *
 * Holds only non-owning pointers into the graph's own storage (src node,
 * dst node, and the actual edge) -- it does not manage their memory.
 *
 * SAFETY REQUIREMENT (see design note): these raw pointers are only valid
 * as long as Graph's nodes_/adjacency_ storage guarantees pointer stability
 * across insertions.
 */
struct EdgePtr {
    const Node* src; // pointer to the source node of the edge.
    const Node* dst; // pointer to the destination node of the edge.
    const Edge* edge; // pointer to the actual edge structure in the graph.
};

#endif