#ifndef GRAPH_HPP
#define GRAPH_HPP

/*
 * This header defines the core graph data structures and functions used to represent
 * and manipulate a road network.
 *
 * The graph is modeled as a directed adjacency list:
 *   - Each node represents a geographical location (lat, lon).
 *   - Each edge connects two nodes and stores metadata (weight, type, name).
 *   - An R-tree is integrated for efficient spatial indexing of edges,
 *     supporting queries like "find the nearest road segment to a custom position".
 *
 * ----------------------------------------------------------------------------------
 * Summary of Design Choises:
 *
 *  - The graph supports up to 65,535 nodes.
 *  - Each node can have up to 65,535 exit edges.
 */

#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <optional>
#include <stdexcept>

#include "highways.hpp"
#include "../data_structures/rtrees/common/bounding_box.hpp"


/*
 * Represents a single node of the graph.
 */
struct Node {
    uint16_t id;
    float lat;
    float lon;
    float dist = numeric_limits<float>::infinity();
    int32_t prev = -1;
};

/*
 * Represents a directed edge in the adjacency list.
 * No more manual 'next' pointer — ownership lives in Graph::adjacency_.
 */
struct Edge {
    uint16_t y;          // destination node id
    std::string name;
    float weight;
    HighwayType type;
};

class Graph; // fwd decl
using HeuristicFunc = std::function<float(const Graph&, uint16_t, uint16_t)>;

class Graph {
public:
    // Building a graph starting from nodes' file and edges' file and returns it.
    Graph(const std::string& node_filepath,
          const std::string& edge_filepath,
          std::unique_ptr<ITreeStrategy> strategy);

    // Rule of five
    Graph(const Graph&) = delete;
    Graph& operator=(const Graph&) = delete;
    Graph(Graph&&) noexcept = default;
    Graph& operator=(Graph&&) noexcept = default;
    ~Graph() = default;

	/* Returns the current number of edges in the graph */
    int edge_count() const noexcept;

	/* This method print the best path from custom starting position p to a default node in the graph.
 	* 1) Check if p is in the global bounding box of the graph.
 	* 2) Check the nearest edge to p and if the distance is < threshold. (grid for spatial indexing, R-tree)
 	* 3) Compute di path. */
    void print_path(bool custom_src, float lat_src, float lon_src, uint16_t src_id,
                     bool custom_dest, float lat_dest, float lon_dest, uint16_t dest_id,
                     const HeuristicFunc& h);

	/*
 	* Prepares a custom node in the graph at the given coordinates (lat, lon).
 	* - Creates a bounding box for the coordinates.
 	* - Checks if the position is within the graph's area.
 	* - Finds the nearest edge and splits it to insert a new node.
 	* Returns {new_node_id, dist_to_nearest_edge} on success, std::nullopt if the
 	* position is outside the graph's area or no nearby edge was found.
 	*/
    std::optional<std::pair<uint16_t, float>> prepare_custom_node(float lat, float lon);

private:
	std::vector<std::unique_ptr<Node>> nodes_;
	std::vector<std::vector<std::unique_ptr<Edge>>> adjacency_;  // adjacency_[x] = sorted-by-y out-edges of x
    std::unique_ptr<RTree> r_tree_;
    int32_t last_src_id_ = -1; /* =-1 none shortest path computed, otherwise >=0 */
    bool structure_modified_ = false; /* true if some insertion/deletion of nodes/edges has been computed previously */

	/* This function read nodes from file and added them to the graph.
	Each node must have a different ID starting from 0. */
    void extract_nodes(const std::string& node_filepath);
	/* This function read edges from file and added them to the graph*/
    void extract_edges(const std::string& edge_filepath);

    void add_node(uint16_t id, float lat, float lon);
    void add_edge(uint16_t x, uint16_t y, float weight, const std::string& name, HighwayType h);
    void remove_edge(uint16_t x, const Edge& e);

	/* This function reset distance and prev fields in each node of the graph.*/
    void reset() noexcept;
	/* Generalized shortest-path algorithm: Dijkstra or A* depending on heuristic.
 	* O(ELogV) function */
    void shortest_path(uint16_t src, uint16_t dest, const HeuristicFunc& h);

	/*
 	* This function builds the shortest path from the source node to the destination node
 	* in the graph using the A* algorithm with a heuristic function 'h'.
 	* It returns the path as vector with the sequence of node IDs from source to destination.
 	*
 	* Parameters:
 	*  - src_id: source node ID
 	*  - dest_id: destination node ID
 	*  - h: heuristic function used by A* (can vary)
 	*  - path: pre-allocated array where the path will be stored
 	*  - path_len: initial length of the path (should be 0), updated as nodes are added
 	*
 	* The function reconstructs the path using the 'prev' field of each node after running
 	* the shortest_path algorithm with the specified heuristic, and reverses the array
 	* to go from source to destination.
 	*/
    std::vector<uint16_t> build_path(uint16_t src_id, uint16_t dest_id, const HeuristicFunc& h);

	/*
 	* This function splits one edge in the graph by adding a new node at the given
 	* coordinates ('lat','lon'). It removes the original edge and adds two new edges:
 	* - from the original source node to the new node
 	* - from the new node to the original destination node
 	*
 	* Parameters:
 	*   - src: source node ID for the edge to split
 	*   - dest: destination node ID for the edge to split
 	*   - e: pointer to the edge to split
 	*   - lat, lon: coordinates of the new node to insert
 	*
 	* Returns:
 	*   the ID of the newly added node
	*/
    uint16_t split_edge_with_node(uint16_t src, uint16_t dest, const Edge& e, float lat, float lon);

	/*
 	* This function splits a bidirectional street in the graph by adding a new node at the given
 	* coordinates ('lat','lon'). Both edges (src->dest and dest->src) are removed and replaced with
 	* edges passing through the new node:
 	* - from src to the new node
 	* - from the new node to dest
 	* - from dest to the new node
 	* - from the new node to src
 	*
 	* Parameters:
 	*   src: source node ID of the first edge (src -> dest)
 	*   dest: destination node ID of the first edge (src -> dest)
 	*   e1: pointer to the edge from src to dest
 	*   e2: pointer to the edge from dest to src
 	*   lat, lon: coordinates of the new node to insert
 	*
 	* Returns:
 	*   the ID of the newly added node
 	*/
    uint16_t split_bidirectional_street_with_node(uint16_t src, uint16_t dest,
                                                    const Edge& e1, const Edge& e2,
                                                    float lat, float lon);

	/*
 	* Finds the closest edge to the point in 'bb' and splits it by inserting a new node.
 	* If the edge is two-way undivided (i.e. primary, secondary, ...) and the opposite
 	* edge is nearby, it splits both, if not (i.e. motorway, trunk, ...)  it splits only
 	* the right edge.
 	*
 	* Parameters:
 	*  - bb: bounding box representing the custom position (lat, lon)
 	*  - p_id: pointer to store the ID of the created node
 	*  - dist_from_nearest_edge: pointer to store the distance from the custom position to the nearest edge
 	*
 	* Returns:
 	*  - true if an edge was found and split, false otherwise
 	*
 	* Note: bb.y_max = bb.y_min = latitude, bb.x_max = bb.x_min = longitude
 	*
 	*/
    std::optional<std::pair<uint16_t, float>> split_nearest_edge(BoundingBox bb);
};


#endif