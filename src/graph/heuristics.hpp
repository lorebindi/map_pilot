#ifndef HEURISTICS_HPP
#define HEURISTICS_HPP

#include <cstdint>

class Graph; // forward declaration

/* Heuristic for plain Dijkstra: always returns 0, so A* degenerates to Dijkstra. */
constexpr float heuristic_zero(const Graph& g, uint16_t curr, uint16_t dest) noexcept;

/* Heuristic based on the Haversine formula: the "orthodromic distance",
 * i.e. the great-circle distance between two points on a sphere. */
float heuristic_haversine(const Graph& g, uint16_t curr, uint16_t dest);

#endif //HEURISTICS_HPP