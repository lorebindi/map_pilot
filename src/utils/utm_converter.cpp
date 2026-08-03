#include "utm_converter.hpp"
#include <cmath>
#include <stdexcept>
#include <iostream>

namespace {
    constexpr double WGS84_A = 6378137.0;
    constexpr double WGS84_F = 1.0 / 298.257223563;
    constexpr double K0 = 0.9996;

    constexpr double deg_to_rad(double deg) noexcept {
        return deg * M_PI / 180.0;
    }

    int get_utm_zone(double lon) noexcept {
        return static_cast<int>((lon + 180.0) / 6.0) + 1;
    }

    UtmCoordinate latlon_to_utm(double lat, double lon) {
        if (lat < -80.0 || lat > 84.0) {
            std::cerr << "Latitude out of UTM bounds\n";
        }

        UtmCoordinate utm{};
        utm.zone = get_utm_zone(lon);
        utm.hemisphere = (lat >= 0) ? Hemisphere::North : Hemisphere::South;

        double lat_rad = deg_to_rad(lat);
        double lon_rad = deg_to_rad(lon);
        double central_meridian_rad = deg_to_rad((utm.zone - 1) * 6 - 180 + 3);

        double a = WGS84_A;
        double f = WGS84_F;
        double e = std::sqrt(f * (2 - f));

        double sin_lat = std::sin(lat_rad);
        double cos_lat = std::cos(lat_rad);
        double tan_lat = std::tan(lat_rad);

        double N = a / std::sqrt(1 - e * e * sin_lat * sin_lat);
        double T = tan_lat * tan_lat;
        double e_prime_sq = (e * e) / (1 - e * e);
        double C = e_prime_sq * cos_lat * cos_lat;
        double A = cos_lat * (lon_rad - central_meridian_rad);

        double M = a * ((1 - e*e/4 - 3*std::pow(e,4)/64 - 5*std::pow(e,6)/256) * lat_rad
                       - (3*e*e/8 + 3*std::pow(e,4)/32 + 45*std::pow(e,6)/1024) * std::sin(2*lat_rad)
                       + (15*std::pow(e,4)/256 + 45*std::pow(e,6)/1024) * std::sin(4*lat_rad)
                       - (35*std::pow(e,6)/3072) * std::sin(6*lat_rad));

        double easting = K0 * N * (A + (1 - T + C) * std::pow(A,3) / 6
                        + (5 - 18*T + T*T + 72*C - 58*e_prime_sq) * std::pow(A,5) / 120)
                        + 500000.0;

        double northing = K0 * (M + N * tan_lat * (std::pow(A,2) / 2
                        + (5 - T + 9*C + 4*C*C) * std::pow(A,4) / 24
                        + (61 - 58*T + T*T + 600*C - 330*e_prime_sq) * std::pow(A,6) / 720));

        if (lat < 0) northing += 10000000.0;

        utm.easting = easting;
        utm.northing = northing;
        return utm;
    }

    double distance_point_to_point(const UtmCoordinate& a, const UtmCoordinate& b) noexcept {
        double dx = b.easting - a.easting;
        double dy = b.northing - a.northing;
        return std::sqrt(dx*dx + dy*dy); // meters
    }
}

PointToSegmentResult point_to_segment_distance(double lat_p, double lon_p,
                                                 double lat_a, double lon_a,
                                                 double lat_b, double lon_b) {
    UtmCoordinate p = latlon_to_utm(lat_p, lon_p);
    UtmCoordinate a = latlon_to_utm(lat_a, lon_a);
    UtmCoordinate b = latlon_to_utm(lat_b, lon_b);
    // NOTE: assumes P, A, B share the same UTM zone/hemisphere (inherited from original)

    double dx = b.easting - a.easting;
    double dy = b.northing - a.northing;
    double length_sq = dx*dx + dy*dy;

    // Data-dependent condition (degenerate/duplicate edge endpoints), not a
    // programmer invariant -- must not be compiled away by NDEBUG like assert() would be.
    if (length_sq <= 1e-12) {
        throw std::runtime_error("point_to_segment_distance: degenerate segment (A == B)");
    }

    double t = ((p.easting - a.easting) * dx + (p.northing - a.northing) * dy) / length_sq;
    t = std::clamp(t, 0.0, 1.0);

    double proj_x = a.easting + t * dx;
    double proj_y = a.northing + t * dy;

    double dist_sq = (p.easting - proj_x) * (p.easting - proj_x)
                    + (p.northing - proj_y) * (p.northing - proj_y);

    // NOTE: proj_x/proj_y are UTM easting/northing, not lat/lon -- the original
    // .c returned these directly too, despite the struct/result being named as
    // if they were lat/lon. Flagging rather than silently "fixing": if callers
    // (e.g. graph.cpp's split_edge_with_node) need lat/lon back, you need an
    // inverse UTM->latlon transform, which the original code never had either.
    return PointToSegmentResult{std::sqrt(dist_sq), proj_x, proj_y};
}

double node_to_node_distance(double lat_a, double lon_a, double lat_n, double lon_n) {
    UtmCoordinate a = latlon_to_utm(lat_a, lon_a);
    UtmCoordinate n = latlon_to_utm(lat_n, lon_n);

    double dist = distance_point_to_point(a, n); // meters, not km (see header note)
    return std::round(dist); // round to nearest meter
}