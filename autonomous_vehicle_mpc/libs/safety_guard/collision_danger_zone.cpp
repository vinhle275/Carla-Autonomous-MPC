#include "collision_danger_zone.hpp"
#include <algorithm>
#include <limits>

namespace av::safety {

CollisionDangerZone::CollisionDangerZone(double zone_width, double zone_length, double front_overhang)
    : zone_width_(zone_width), zone_length_(zone_length), front_overhang_(front_overhang) {}

std::array<Point2D, 4> CollisionDangerZone::get_danger_zone_corners_world(double ego_x, double ego_y, double ego_yaw_rad) const {
    double cos_th = std::cos(ego_yaw_rad);
    double sin_th = std::sin(ego_yaw_rad);

    Point2D t_vec{cos_th, sin_th};
    Point2D n_vec{-sin_th, cos_th};

    double center_x = ego_x + (front_overhang_ + zone_length_ / 2.0) * t_vec.x;
    double center_y = ego_y + (front_overhang_ + zone_length_ / 2.0) * t_vec.y;

    double hl = zone_length_ / 2.0;
    double hw = zone_width_ / 2.0;

    return {
        Point2D{center_x - hl * t_vec.x - hw * n_vec.x, center_y - hl * t_vec.y - hw * n_vec.y}, // rear-right
        Point2D{center_x + hl * t_vec.x - hw * n_vec.x, center_y + hl * t_vec.y - hw * n_vec.y}, // front-right
        Point2D{center_x + hl * t_vec.x + hw * n_vec.x, center_y + hl * t_vec.y + hw * n_vec.y}, // front-left
        Point2D{center_x - hl * t_vec.x + hw * n_vec.x, center_y - hl * t_vec.y + hw * n_vec.y}  // rear-left
    };
}

std::array<Point2D, 4> CollisionDangerZone::get_obstacle_obb_corners_world(const types::ObstacleFrenetOBB& obs) {
    double cx = obs.center_x;
    double cy = obs.center_y;
    double hl = obs.extent_x;
    double hw = obs.extent_y;
    double yaw = obs.box_yaw;

    double cos_y = std::cos(yaw);
    double sin_y = std::sin(yaw);
    Point2D t_obs{cos_y, sin_y};
    Point2D n_obs{-sin_y, cos_y};

    return {
        Point2D{cx - hl * t_obs.x - hw * n_obs.x, cy - hl * t_obs.y - hw * n_obs.y},
        Point2D{cx + hl * t_obs.x - hw * n_obs.x, cy + hl * t_obs.y - hw * n_obs.y},
        Point2D{cx + hl * t_obs.x + hw * n_obs.x, cy + hl * t_obs.y + hw * n_obs.y},
        Point2D{cx - hl * t_obs.x + hw * n_obs.x, cy - hl * t_obs.y + hw * n_obs.y}
    };
}

bool CollisionDangerZone::sat_overlap_on_axis(const Point2D& axis,
                                              const std::array<Point2D, 4>& corners_a,
                                              const std::array<Point2D, 4>& corners_b) {
    double min_a = std::numeric_limits<double>::max();
    double max_a = -std::numeric_limits<double>::max();
    for (const auto& c : corners_a) {
        double proj = c.x * axis.x + c.y * axis.y;
        min_a = std::min(min_a, proj);
        max_a = std::max(max_a, proj);
    }

    double min_b = std::numeric_limits<double>::max();
    double max_b = -std::numeric_limits<double>::max();
    for (const auto& c : corners_b) {
        double proj = c.x * axis.x + c.y * axis.y;
        min_b = std::min(min_b, proj);
        max_b = std::max(max_b, proj);
    }

    return !(max_a < min_b || max_b < min_a);
}

bool CollisionDangerZone::check_obb_intersection(const std::array<Point2D, 4>& zone_corners,
                                                const std::array<Point2D, 4>& obs_corners,
                                                double ego_yaw_rad, double obs_yaw_rad) const {
    Point2D axes[4] = {
        Point2D{std::cos(ego_yaw_rad), std::sin(ego_yaw_rad)},
        Point2D{-std::sin(ego_yaw_rad), std::cos(ego_yaw_rad)},
        Point2D{std::cos(obs_yaw_rad), std::sin(obs_yaw_rad)},
        Point2D{-std::sin(obs_yaw_rad), std::cos(obs_yaw_rad)}
    };

    for (const auto& ax : axes) {
        double norm = std::hypot(ax.x, ax.y);
        if (norm > 1e-6) {
            Point2D unit_ax{ax.x / norm, ax.y / norm};
            if (!sat_overlap_on_axis(unit_ax, zone_corners, obs_corners)) {
                return false; // Trục phân tách tìm thấy -> không va chạm
            }
        }
    }
    return true; // Tất cả các trục đều đè lên nhau -> Giao cắt OBB!
}

std::pair<bool, double> CollisionDangerZone::check_point_in_zone(double px, double py,
                                                                 double ego_x, double ego_y,
                                                                 double ego_yaw_rad,
                                                                 double& out_y_local) const {
    double dx = px - ego_x;
    double dy = py - ego_y;
    double cos_th = std::cos(ego_yaw_rad);
    double sin_th = std::sin(ego_yaw_rad);

    double x_local = dx * cos_th + dy * sin_th;
    double y_local = -dx * sin_th + dy * cos_th;
    out_y_local = y_local;

    double d_front = x_local - front_overhang_;
    double hw = zone_width_ / 2.0;

    bool in_length = (d_front >= 0.0 && d_front <= zone_length_);
    bool in_width = (std::abs(y_local) <= hw);
    return {in_length && in_width, d_front};
}

ZoneCheckResult CollisionDangerZone::is_obstacle_in_zone(const types::ObstacleFrenetOBB& obs,
                                                         double ego_x, double ego_y,
                                                         double ego_yaw_rad) const {
    ZoneCheckResult res{};
    auto zone_corners = get_danger_zone_corners_world(ego_x, ego_y, ego_yaw_rad);
    auto obs_corners = get_obstacle_obb_corners_world(obs);

    // 1. Kiểm tra SAT giao cắt hộp
    bool is_intersect = check_obb_intersection(zone_corners, obs_corners, ego_yaw_rad, obs.box_yaw);

    // 2. Kiểm tra các điểm góc và tâm vật cản
    std::vector<Point2D> test_points = {
        obs_corners[0], obs_corners[1], obs_corners[2], obs_corners[3],
        Point2D{obs.center_x, obs.center_y}
    };

    double min_d_front = std::numeric_limits<double>::max();
    double lat_at_min = 0.0;
    bool any_pt_inside = false;

    for (const auto& pt : test_points) {
        double y_loc = 0.0;
        auto [in_zone, d_fr] = check_point_in_zone(pt.x, pt.y, ego_x, ego_y, ego_yaw_rad, y_loc);
        if (in_zone) {
            any_pt_inside = true;
        }
        if (d_fr < min_d_front) {
            min_d_front = d_fr;
            lat_at_min = y_loc;
        }
    }

    res.is_inside = (is_intersect || any_pt_inside);
    res.min_dist_from_bumper = (min_d_front != std::numeric_limits<double>::max()) ? min_d_front : 0.0;
    res.lat_offset = lat_at_min;
    return res;
}

} // namespace av::safety
