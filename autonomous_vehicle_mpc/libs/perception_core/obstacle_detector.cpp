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
        if (dist_surface_front < 0.05) continue;

        double angle_to_surface = std::atan2(cy - sensor_y, cx - sensor_x);
        double rel_angle = math::normalize_angle(angle_to_surface - ego_yaw_rad);

        // Vùng 1: Quét xa phía trước trong FOV 120 độ
        bool in_front_fov = (dist_surface_front <= detection_range_) && (std::abs(rel_angle) <= half_fov_rad_);
        // Vùng 2: Quét 360 độ cự ly gần (Blind-spot / Surround)
        bool in_surround = (dist_surface_ego <= surround_range_);

        if (in_front_fov || in_surround) {
            current_seen_ids.insert(actor.id);

            // Ước lượng vận tốc thế giới bằng vi phân vị trí qua các chu kỳ (nếu CARLA set_transform làm get_velocity() = 0)
            double vx = actor.vx;
            double vy = actor.vy;
            if (prev_positions_.count(actor.id) && dt > 1e-4) {
                double dx_p = actor.x - prev_positions_[actor.id].first;
                double dy_p = actor.y - prev_positions_[actor.id].second;
                double ds = std::hypot(dx_p, dy_p);
                double sp_est = ds / dt;
                if (sp_est > std::hypot(vx, vy) && sp_est < 40.0) {
                    vx = dx_p / dt;
                    vy = dy_p / dt;
                }
            }
            prev_positions_[actor.id] = {actor.x, actor.y};

            // Chiếu tâm và 9 điểm đặc trưng OBB xoay theo box_yaw sang hệ tọa độ Frenet
            auto proj_center = math::project_to_frenet(actor.x, actor.y, x_traj, y_traj, theta_traj, s_traj, current_idx);
            double hl = std::max(actor.extent_x, 0.3);
            double hw = std::max(actor.extent_y, 0.3);

            double s_r = 0.0, s_f = 0.0, d_min = 0.0, d_max = 0.0;
            math::compute_obb_frenet_bounds(actor.x, actor.y, actor.yaw_rad, hl, hw,
                                            x_traj, y_traj, theta_traj, s_traj, current_idx,
                                            s_r, s_f, d_min, d_max);

            double v_s = vx * proj_center.tx + vy * proj_center.ty;
            double v_d = vx * proj_center.nx + vy * proj_center.ny;

            types::ObstacleFrenetOBB obb{};
            obb.id = actor.id;
            obb.s_rear = s_r;
            obb.s_front = s_f;
            obb.s_center = 0.5 * (s_r + s_f);
            obb.d_min = d_min;
            obb.d_max = d_max;
            obb.d_center = 0.5 * (d_min + d_max);
            obb.center_x = actor.x;
            obb.center_y = actor.y;
            obb.vx = vx;
            obb.vy = vy;
            obb.v_s = v_s;
            obb.v_d = v_d;
            obb.box_yaw = actor.yaw_rad;
            obb.extent_x = hl;
            obb.extent_y = hw;
            obb.is_moving = (std::hypot(vx, vy) > config::DANGER_MOVING_SPEED_THRESHOLD_MS);
            obb.is_vehicle = (actor.is_vehicle != 0);

            // Nhận diện vật cản cắt ngang đường: |v_d| >= 1.5 m/s, |v_d| >= 1.5 * |v_s| và |v_s| < 3.0 m/s
            if (std::abs(v_d) >= 1.5 && std::abs(v_d) >= 1.5 * std::abs(v_s) && std::abs(v_s) < 3.0) {
                obb.is_lateral_crossing = true;
            }

            tracked_obstacles_[actor.id] = obb;
        }
    }

    // Cập nhật lại tọa độ Frenet cho các track trong bộ nhớ tạm thời khuất FOV & xóa track đã lùi sau lưng hoặc ra ngoài tầm
    std::unordered_map<int32_t, RawActorData> raw_by_id;
    for (const auto& a : raw_actors) {
        raw_by_id[a.id] = a;
    }

    std::vector<int32_t> to_delete;
    auto proj_ego = math::project_to_frenet(ego_x, ego_y, x_traj, y_traj, theta_traj, s_traj, current_idx);

    for (auto& [tid, obs] : tracked_obstacles_) {
        if (!current_seen_ids.count(tid)) {
            if (!raw_by_id.count(tid)) {
                to_delete.push_back(tid);
                continue;
            }
            const auto& act_now = raw_by_id[tid];
            obs.center_x = act_now.x;
            obs.center_y = act_now.y;
            obs.box_yaw = act_now.yaw_rad;

            auto [cx, cy] = get_closest_point_on_obb(sensor_x, sensor_y,
                                                     obs.center_x, obs.center_y, obs.box_yaw,
                                                     obs.extent_x, obs.extent_y);
            double dx = cx - sensor_x;
            double dy = cy - sensor_y;
            double dist_now = std::hypot(dx, dy);
            double angle_now = std::atan2(dy, dx);
            double rel_angle_now = math::normalize_angle(angle_now - ego_yaw_rad);
            double d_long_now = dist_now * std::cos(rel_angle_now);

            math::compute_obb_frenet_bounds(obs.center_x, obs.center_y, obs.box_yaw,
                                            obs.extent_x, obs.extent_y,
                                            x_traj, y_traj, theta_traj, s_traj, current_idx,
                                            obs.s_rear, obs.s_front, obs.d_min, obs.d_max);
            obs.s_center = 0.5 * (obs.s_rear + obs.s_front);
            obs.d_center = 0.5 * (obs.d_min + obs.d_max);

            if (d_long_now < -track_drop_behind_ || dist_now > (detection_range_ + 5.0) ||
                obs.s_front < proj_ego.s - track_drop_behind_) {
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
