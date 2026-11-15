#include "obstacle_detector.hpp"
#include "common/frenet_utils.hpp"
#include <algorithm>
#include <unordered_set>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace av::perception {

ObstacleDetector::ObstacleDetector(double detection_range, double fov_deg,
                                   double surround_range, double track_drop_behind)
    : detection_range_(detection_range),
      fov_deg_(fov_deg),
      half_fov_rad_(fov_deg * M_PI / 360.0),
      surround_range_(surround_range),
      track_drop_behind_(track_drop_behind) {}

void ObstacleDetector::clear_memory() {
    tracked_obstacles_.clear();
    prev_positions_.clear();
}

std::pair<double, double> ObstacleDetector::get_closest_point_on_obb(double px, double py,
                                                                     double box_cx, double box_cy,
                                                                     double box_yaw_rad,
                                                                     double half_len, double half_wid) {
    double dx = px - box_cx;
    double dy = py - box_cy;
    double cos_b = std::cos(box_yaw_rad);
    double sin_b = std::sin(box_yaw_rad);

    double x_local = dx * cos_b + dy * sin_b;
    double y_local = -dx * sin_b + dy * cos_b;

    double x_cl = std::clamp(x_local, -half_len, half_len);
    double y_cl = std::clamp(y_local, -half_wid, half_wid);

    double closest_x = box_cx + x_cl * cos_b - y_cl * sin_b;
    double closest_y = box_cy + x_cl * sin_b + y_cl * cos_b;
    return {closest_x, closest_y};
}

std::vector<types::ObstacleFrenetOBB> ObstacleDetector::process_scan(
    double ego_x, double ego_y, double ego_yaw_rad,
    const std::vector<RawActorData>& raw_actors,
    const std::vector<double>& x_traj,
    const std::vector<double>& y_traj,
    const std::vector<double>& theta_traj,
    const std::vector<double>& s_traj,
    size_t current_idx,
    double dt) {

    double sensor_x = ego_x + 2.0 * std::cos(ego_yaw_rad);
    double sensor_y = ego_y + 2.0 * std::sin(ego_yaw_rad);

    std::unordered_set<int32_t> current_seen_ids;

    for (const auto& actor : raw_actors) {
        double dist_to_center = std::hypot(actor.x - ego_x, actor.y - ego_y);
        if (dist_to_center > (detection_range_ + actor.extent_x + 8.0)) continue;

        // Điểm gần nhất trên vỏ OBB
        auto [cx, cy] = get_closest_point_on_obb(sensor_x, sensor_y,
                                                 actor.x, actor.y, actor.yaw_rad,
                                                 actor.extent_x, actor.extent_y);

        double dist_surface_front = std::hypot(cx - sensor_x, cy - sensor_y);
        double dist_surface_ego = std::hypot(cx - ego_x, cy - ego_y);
        if (dist_surface_front < 0.1) continue;

        double angle_to_surface = std::atan2(cy - sensor_y, cx - sensor_x);
        double rel_angle = math::normalize_angle(angle_to_surface - ego_yaw_rad);

        // Vùng 1: Quét xa phía trước trong FOV 120 độ
        bool in_front_fov = (dist_surface_front <= detection_range_) && (std::abs(rel_angle) <= half_fov_rad_);
        // Vùng 2: Quét 360 độ cự ly gần
        bool in_surround = (dist_surface_ego <= surround_range_);

        if (in_front_fov || in_surround) {
            current_seen_ids.insert(actor.id);

            // Ước lượng vận tốc thế giới
            double vx = actor.vx;
            double vy = actor.vy;
            if (prev_positions_.count(actor.id) && dt > 1e-4) {
                double ds = std::hypot(actor.x - prev_positions_[actor.id].first,
                                       actor.y - prev_positions_[actor.id].second);
                double sp_est = ds / dt;
                if (sp_est > std::hypot(vx, vy) && sp_est < 40.0) {
                    vx = (actor.x - prev_positions_[actor.id].first) / dt;
                    vy = (actor.y - prev_positions_[actor.id].second) / dt;
                }
            }
            prev_positions_[actor.id] = {actor.x, actor.y};

            // Chiếu sang hệ tọa độ Frenet
            auto proj_front = math::project_to_frenet(cx, cy, x_traj, y_traj, theta_traj, s_traj, current_idx);
            auto proj_center = math::project_to_frenet(actor.x, actor.y, x_traj, y_traj, theta_traj, s_traj, current_idx);

            double hl = actor.extent_x;
            double hw = actor.extent_y;

            // Tính bao Frenet [s_rear, s_front] x [d_min, d_max]
            double s_r = proj_center.s - hl;
            double s_f = proj_center.s + hl;
            double d_min = proj_center.d - hw;
            double d_max = proj_center.d + hw;

            double v_s = vx * proj_center.tx + vy * proj_center.ty;
            double v_d = vx * proj_center.nx + vy * proj_center.ny;

            types::ObstacleFrenetOBB obb;
            obb.id = actor.id;
            obb.s_rear = s_r;
            obb.s_front = s_f;
            obb.d_min = d_min;
            obb.d_max = d_max;
            obb.d_center = proj_center.d;
            obb.center_x = actor.x;
            obb.center_y = actor.y;
            obb.vx = vx;
            obb.vy = vy;
            obb.v_s = v_s;
            obb.v_d = v_d;
            obb.box_yaw = actor.yaw_rad;
            obb.extent_x = hl;
            obb.extent_y = hw;
            obb.is_moving = (std::hypot(vx, vy) > 0.35);
            obb.is_vehicle = actor.is_vehicle;

            // Nhận diện vật cản cắt ngang đường
            if (std::abs(v_d) >= 1.5 && std::abs(v_d) >= 1.5 * std::abs(v_s) && std::abs(v_s) < 3.0) {
                obb.is_lateral_crossing = true;
            }

            tracked_obstacles_[actor.id] = obb;
        }
    }

    // Xóa các track đã qua khỏi xe (> 3m sau lưng)
    std::vector<int32_t> to_delete;
    auto proj_ego = math::project_to_frenet(ego_x, ego_y, x_traj, y_traj, theta_traj, s_traj, current_idx);

    for (const auto& [tid, obs] : tracked_obstacles_) {
        if (!current_seen_ids.count(tid)) {
            if (obs.s_front < proj_ego.s - track_drop_behind_) {
                to_delete.push_back(tid);
            }
        }
    }
    for (int32_t tid : to_delete) {
        tracked_obstacles_.erase(tid);
        prev_positions_.erase(tid);
    }

    std::vector<types::ObstacleFrenetOBB> result;
    result.reserve(tracked_obstacles_.size());
    for (const auto& [_, obs] : tracked_obstacles_) {
        result.push_back(obs);
    }
    std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
        return a.s_rear < b.s_rear;
    });
    return result;
}

} // namespace av::perception
