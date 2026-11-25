// ==============================================================================
// COLLISION DANGER ZONE & SEPARATING AXIS THEOREM (SAT)
// Modern C++20 | 2D OBB Intersection Test
// ==============================================================================

#pragma once

#include <cmath>
#include <array>
#include <vector>
#include "common/autosar_types.hpp"
#include "common/config_constants.hpp"

namespace av::safety {

struct Point2D {
    double x{0.0};
    double y{0.0};
};

struct ZoneCheckResult {
    bool is_inside{false};
    double min_dist_from_bumper{99.0};
    double lat_offset{0.0};
};

class CollisionDangerZone {
public:
    CollisionDangerZone(double zone_width = av::config::DANGER_ZONE_WIDTH_M,
                        double zone_length = av::config::DANGER_ZONE_LENGTH_M,
                        double front_overhang = av::config::EGO_FRONT_OVERHANG_M);

    std::array<Point2D, 4> get_danger_zone_corners_world(double ego_x, double ego_y, double ego_yaw_rad) const;

    static std::array<Point2D, 4> get_obstacle_obb_corners_world(const types::ObstacleFrenetOBB& obs);

    bool check_obb_intersection(const std::array<Point2D, 4>& zone_corners,
                                const std::array<Point2D, 4>& obs_corners,
                                double ego_yaw_rad, double obs_yaw_rad) const;

    std::pair<bool, double> check_point_in_zone(double px, double py,
                                                double ego_x, double ego_y,
                                                double ego_yaw_rad, double& out_y_local) const;

    ZoneCheckResult is_obstacle_in_zone(const types::ObstacleFrenetOBB& obs,
                                        double ego_x, double ego_y,
                                        double ego_yaw_rad) const;

private:
    static bool sat_overlap_on_axis(const Point2D& axis,
                                    const std::array<Point2D, 4>& corners_a,
                                    const std::array<Point2D, 4>& corners_b);

    double zone_width_;
    double zone_length_;
    double front_overhang_;
};

} // namespace av::safety
