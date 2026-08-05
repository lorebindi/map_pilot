#ifndef BOUNDING_BOX_HPP
#define BOUNDING_BOX_HPP

#include <cstdint>
#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace std;

/*
 * A 2D rectangle in latitude/longitude space, representing the MBR of one
 * or more graph edges (or of R-tree nodes containing them).
 *
 * The bounding box is logically constructed from the latitude/longitude
 * coordinates of two graph nodes (edge endpoints).
 *
 * - Enables efficient spatial indexing of graph edges within the R-tree structure.
 * - Supports fast queries such as intersection, containment, and nearest-neighbor.
 *
 * Note:
 * - The road segment is logically represented by the diagonal of the bounding box,
 *   which may introduce inaccuracies for curved or irregular geometries.
 * - This trade-off is intentional, as it reduces complexity and storage overhead
 *   while preserving acceptable spatial locality for routing tasks.
 */
struct BoundingBox {
    float x_min = 0.0f, y_min = 0.0f, x_max = 0.0f, y_max = 0.0f;

    // This function create the bounding box starting from two position (lat,lon).
    static BoundingBox from_points(float a_lat, float a_lon, float b_lat, float b_lon) noexcept {
        return BoundingBox{
            min(a_lon, b_lon), min(a_lat, b_lat),
            max(a_lon, b_lon), max(a_lat, b_lat)
        };
    }

    // Replaces create_bounding_box(lat, lat, lon, lon) call pattern for a single point
    static BoundingBox from_point(float lat, float lon) noexcept {
        return BoundingBox{lon, lat, lon, lat};
    }

    // This function return true if the two bounding box overlap, false otherwise.
    bool overlaps(const BoundingBox& other) const noexcept {
        return x_min <= other.x_max && x_max >= other.x_min &&
               y_min <= other.y_max && y_max >= other.y_min;
    }

    // This function return true if the passed bounding box is fully contained, false otherwise.
    bool contains(const BoundingBox& inner) const noexcept {
        return x_min <= inner.x_min && x_max >= inner.x_max &&
               y_min <= inner.y_min && y_max >= inner.y_max;
    }

    // This function returns the bounding box of the overlapping area between 'b1' and 'b2'.
    BoundingBox overlapping_bounding_box(BoundingBox& other) const noexcept {
        BoundingBox result{
            std::max(x_min, other.x_min), std::max(y_min, other.y_min),
            std::min(x_max, other.x_max), std::min(y_max, other.y_max)
        };
        if (result.x_min > result.x_max || result.y_min > result.y_max) {
            result = BoundingBox{}; // all zeros, matching the original's degenerate-case reset
        }
        return result;
    }

    // This function make the union of the current bounding box and the 'other'
    BoundingBox union_bounding_box(const BoundingBox& other) const noexcept {
        return BoundingBox{
            std::min(x_min, other.x_min),
            std::min(y_min, other.y_min),
            std::max(x_max, other.x_max),
            std::max(y_max, other.y_max)
        };
    }

    // This function calculates the needed enlargement to merge the current one and 'other'
    float enlargement_needed(const BoundingBox& other) const noexcept {
        BoundingBox unionBox = union_bounding_box(other);

        return (unionBox.x_max - unionBox.x_min) * (unionBox.y_max - unionBox.y_min)
               - (x_max - x_min) * (y_max - y_min);
    }

    // returns the area of the current bb
    float area() const noexcept {
        float width = x_max - x_min;
        float height = y_max - y_min;
        return width * height;
    }

    bool is_empty() const noexcept {
        return x_min == 0.0f && x_max == 0.0f && y_min == 0.0f && y_max == 0.0f;
    }

    BoundingBox center() const noexcept {
        float cx = (x_min + x_max) / 2.0f;
        float cy = (y_min + y_max) / 2.0f;
        return BoundingBox{cx, cy, cx, cy};
    }

    bool bbs_equals(const BoundingBox& other) const noexcept {
        return x_min == other.x_min && x_max == other.x_max &&
               y_min == other.y_min && y_max == other.y_max;
    }

};

/*
 * Returns the index [0, n) of the entry within 'container' that requires the
 * smallest enlargement of its bounding box to accommodate 'rect'. Ties are
 * broken by the entry with the smallest current area.
*/
template <typename Container>
uint8_t get_area_min_enlargement_index(const Container& container, uint8_t n, BoundingBox rect) {
     uint8_t best_index = UINT8_MAX;
     float min_enlargement = INFINITY;
     float min_area = INFINITY;

     for (uint8_t i = 0; i < n; i++) {
         BoundingBox bb = container.bounding_box_at(i);
         float temp_enlargement = bb.enlargement_needed(rect);
         float temp_area = bb.area();

         if (temp_enlargement < min_enlargement ||
            (temp_enlargement == min_enlargement && temp_area < min_area)) {
             min_enlargement = temp_enlargement;
             min_area = temp_area;
             best_index = i;
             min_area = temp_area;
             }
     }
     return best_index;
}

/*
 * Returns the minimum bounding rectangle that includes each of the first
 * 'n_entries' entries in 'container'.
 */
template <typename Container>
BoundingBox get_node_mbr(const Container& container, uint8_t n_entries) {
    if (n_entries == 0) {
        throw std::logic_error("get_node_mbr: node has no entries");
    }
    BoundingBox result = container.bounding_box_at(0);
    for (uint8_t i = 1; i < n_entries; i++) {
        result = result.union_bounding_box(container.bounding_box_at(i));
    }
    return result;
}

inline void bb_reset(BoundingBox& bb) noexcept {
    bb = BoundingBox{};
}

// return the center of 'rect' as a degenerate (point) bounding box
inline BoundingBox get_bb_center(BoundingBox rect) noexcept {
    float x = (rect.x_min + rect.x_max) / 2.0f;
    float y = (rect.y_min + rect.y_max) / 2.0f;
    return BoundingBox{x, y, x, y};
}

#endif //BOUNDING_BOX_HPP