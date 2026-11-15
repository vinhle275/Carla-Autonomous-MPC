// ==============================================================================
// TRAJECTORY & SCENARIO CSV LOADER
// Modern C++20 | Fast CSV parsing
// ==============================================================================

#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include "frenet_utils.hpp"
#include "config_constants.hpp"

namespace av::data {

struct TrajectoryPoint {
    double x{0.0};
    double y{0.0};
    double theta_rad{0.0};
    double v_m_s{0.0};
    double s{0.0};
};

struct GlobalTrajectory {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> theta;
    std::vector<double> v;
    std::vector<double> s;
    bool is_closed_loop{false};

    size_t size() const { return x.size(); }
    bool empty() const { return x.empty(); }
};

struct StaticObstacleRecord {
    double x{0.0};
    double y{0.0};
    double size{0.5};
};

struct MovingObstacleWaypoint {
    int32_t obstacle_id{0};
    int32_t wp_index{0};
    double x{0.0};
    double y{0.0};
    double speed_m_s{9.0};
};

class TrajectoryLoader {
public:
    static GlobalTrajectory load_reference_trajectory(const std::string& filepath,
                                                     bool override_v_ref = true,
                                                     double target_speed = av::config::TARGET_SPEED_MS) {
        GlobalTrajectory traj;
        std::ifstream file(filepath);
        if (!file.is_open()) {
            std::cerr << "[TRAJECTORY LOADER] Không thể mở file: " << filepath << std::endl;
            return traj;
        }

        std::string line;
        // Đọc header
        if (!std::getline(file, line)) return traj;

        while (std::getline(file, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string item;
            std::vector<double> vals;
            while (std::getline(ss, item, ',')) {
                try {
                    vals.push_back(std::stod(item));
                } catch (...) {
                    break;
                }
            }
            if (vals.size() >= 4) {
                traj.x.push_back(vals[0]);
                traj.y.push_back(vals[1]);
                traj.theta.push_back(vals[2]);
                traj.v.push_back(vals[3]);
            }
        }

        if (traj.empty()) return traj;

        // Làm mịn góc hướng (np.unwrap)
        av::math::unwrap_angles(traj.theta);

        // Tự động nâng tốc độ tối đa lên target_speed nếu bật override_v_ref
        if (override_v_ref && !traj.v.empty()) {
            double max_v = *std::max_element(traj.v.begin(), traj.v.end());
            if (max_v > 0.0) {
                double scale = target_speed / max_v;
                for (auto& v_val : traj.v) {
                    v_val *= scale;
                }
            }
        }

        // Tính s_traj (độ dài cung tích lũy)
        traj.s.resize(traj.x.size(), 0.0);
        for (size_t i = 1; i < traj.x.size(); ++i) {
            double dx = traj.x[i] - traj.x[i - 1];
            double dy = traj.y[i] - traj.y[i - 1];
            traj.s[i] = traj.s[i - 1] + std::hypot(dx, dy);
        }

        // Kiểm tra quỹ đạo khép kín
        double end_dist = std::hypot(traj.x.back() - traj.x.front(), traj.y.back() - traj.y.front());
        traj.is_closed_loop = (end_dist < 2.0);

        std::cout << "[TRAJECTORY LOADER] Nạp thành công " << traj.size()
                  << " waypoints từ " << filepath << " (Tổng chiều dài: "
                  << traj.s.back() << "m)" << std::endl;
        return traj;
    }

    static std::vector<StaticObstacleRecord> load_static_obstacles(const std::string& filepath) {
        std::vector<StaticObstacleRecord> obstacles;
        std::ifstream file(filepath);
        if (!file.is_open()) return obstacles;

        std::string line;
        if (!std::getline(file, line)) return obstacles; // Skip header

        while (std::getline(file, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string item;
            std::vector<double> vals;
            while (std::getline(ss, item, ',')) {
                try {
                    vals.push_back(std::stod(item));
                } catch (...) {
                    break;
                }
            }
            if (vals.size() >= 2) {
                StaticObstacleRecord rec;
                rec.x = vals[0];
                rec.y = vals[1];
                rec.size = (vals.size() >= 3) ? vals[2] : 0.5;
                obstacles.push_back(rec);
            }
        }
        std::cout << "[SCENARIO LOADER] Nạp " << obstacles.size()
                  << " vật cản tĩnh từ " << filepath << std::endl;
        return obstacles;
    }

    static std::vector<MovingObstacleWaypoint> load_moving_obstacles(const std::string& filepath) {
        std::vector<MovingObstacleWaypoint> wps;
        std::ifstream file(filepath);
        if (!file.is_open()) return wps;

        std::string line;
        if (!std::getline(file, line)) return wps; // Skip header

        while (std::getline(file, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string item;
            std::vector<std::string> tokens;
            while (std::getline(ss, item, ',')) {
                tokens.push_back(item);
            }
            if (tokens.size() >= 5) {
                try {
                    MovingObstacleWaypoint mw;
                    mw.obstacle_id = std::stoi(tokens[0]);
                    mw.wp_index = std::stoi(tokens[1]);
                    mw.x = std::stod(tokens[2]);
                    mw.y = std::stod(tokens[3]);
                    mw.speed_m_s = std::stod(tokens[4]);
                    wps.push_back(mw);
                } catch (...) {}
            }
        }
        std::cout << "[SCENARIO LOADER] Nạp " << wps.size()
                  << " waypoints vật cản di động từ " << filepath << std::endl;
        return wps;
    }
};

} // namespace av::data
