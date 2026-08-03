#ifndef UTM_CONVERTER_HPP
#define UTM_CONVERTER_HPP

enum class Hemisphere { North, South };

struct UtmCoordinate {
    int zone;
    Hemisphere hemisphere;
    double easting;
    double northing;
};

struct PointToSegmentResult {
    double distance_m;   // distance in meters from P to segment AB
    double proj_lat;     // NOTE: see .cpp -- currently UTM easting, not lat (see flag below)
    double proj_lon;     // NOTE: see .cpp -- currently UTM northing, not lon
};

/*
 * Returns the distance in meters between a point P and the segment defined
 * by points A and B, plus the coordinates of P's projection onto that segment.
 * Inputs are geographic coordinates (lat, lon), NOT pre-projected UTM coordinates.
 */
PointToSegmentResult point_to_segment_distance(double lat_p, double lon_p,
                                                 double lat_a, double lon_a,
                                                 double lat_b, double lon_b);

/*
 * Returns the distance in METERS between a node of an existing edge in the
 * graph and the new node that will split the edge.
 * (Despite the original name/comment implying km, the underlying UTM
 * easting/northing values are in meters -- see conversion notes.)
 */
double node_to_node_distance(double lat_a, double lon_a, double lat_n, double lon_n);

#endif // UTM_CONVERTER_HPP