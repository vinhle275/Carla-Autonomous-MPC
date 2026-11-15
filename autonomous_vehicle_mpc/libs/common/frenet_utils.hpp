// ==============================================================================
// FRENET UTILITIES & TRAJECTORY MATH ROUTINES
// Modern C++20 | Header-only fast vector math
// ==============================================================================

#pragma once

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <vector>
#include <algorithm>
#include <stdexcept>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "config_constants.hpp"

namespace av::math {

/// @brief Chuẩn hóa góc về đoạn [-pi, pi]
inline double normalize_angle(double angle) {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

/// @brief Khử nhảy pha góc 2*pi cho mảng liên tục (tương đương np.unwrap)
inline void unwrap_angles(std::vector<double>& angles) {
    if (angles.size() < 2) return;
    for (size_t i = 1; i < angles.size(); ++i) {
        double diff = angles[i] - angles[i - 1];
        while (diff > M_PI) {
            angles[i] -= 2.0 * M_PI;
            diff = angles[i] - angles[i - 1];
        }
        while (diff < -M_PI) {
            angles[i] += 2.0 * M_PI;
            diff = angles[i] - angles[i - 1];
        }
    }
}

/// @brief Tính sai lệch vuông góc (lateral deviation) từ (x, y) tới đoạn thẳng nối 2 điểm tham chiếu
inline double calculate_lateral_deviation(double x, double y,
                                          double x_ref, double y_ref,
                                          double x_next, double y_next) {
    double dx = x_next - x_ref;
    double dy = y_next - y_ref;
    double len = std::hypot(dx, dy);
    if (len < 1e-6) return 0.0;
    // Tích có hướng 2D xác định khoảng cách có dấu
    return ((x - x_ref) * dy - (y - y_ref) * dx) / len;
}

/// @brief Cấu trúc kết quả chiếu Frenet
struct FrenetProjection {
    double s{0.0};          // Tọa độ dọc s (m)
    double d{0.0};          // Tọa độ ngang d (m, d > 0 bên phải, d < 0 bên trái)
    double nx{0.0};         // Vector pháp tuyến ngang hướng sang phải
    double ny{0.0};
    double tx{0.0};         // Vector tiếp tuyến dọc theo hướng đường
    double ty{0.0};
    size_t closest_idx{0};  // Chỉ số waypoint gần nhất
};

/// @brief Chiếu điểm (x, y) lên quỹ đạo tham chiếu trong cửa sổ quanh hint_idx
inline FrenetProjection project_to_frenet(double x, double y,
                                          const std::vector<double>& x_traj,
                                          const std::vector<double>& y_traj,
                                          const std::vector<double>& theta_traj,
                                          const std::vector<double>& s_traj,
                                          size_t hint_idx = 0,
                                          size_t window_back = 150,
                                          size_t window_fwd = 300) {
    FrenetProjection res{};
    size_t n = x_traj.size();
    if (n == 0) return res;

    size_t start_i = (hint_idx >= window_back) ? (hint_idx - window_back) : 0;
    size_t end_i = std::min(n, hint_idx + window_fwd);
    if (end_i <= start_i) {
        start_i = 0;
        end_i = n;
    }

    double min_d_sq = std::numeric_limits<double>::max();
    size_t best_i = start_i;

    for (size_t i = start_i; i < end_i; ++i) {
        double dx = x_traj[i] - x;
        double dy = y_traj[i] - y;
        double d_sq = dx * dx + dy * dy;
        if (d_sq < min_d_sq) {
            min_d_sq = d_sq;
            best_i = i;
        }
    }

    double xr = x_traj[best_i];
    double yr = y_traj[best_i];
    double th = theta_traj[best_i];

    double tx = std::cos(th);
    double ty = std::sin(th);
    // CARLA Frenet convention: nx = -sin(th), ny = cos(th) -> d > 0 là BÊN PHẢI, d < 0 là BÊN TRÁI
    double nx = -ty;
    double ny = tx;

    double s_base = (best_i < s_traj.size()) ? s_traj[best_i] : 0.0;
    res.s = s_base + tx * (x - xr) + ty * (y - yr);
    res.d = nx * (x - xr) + ny * (y - yr);
    res.nx = nx;
    res.ny = ny;
    res.tx = tx;
    res.ty = ty;
    res.closest_idx = best_i;

    return res;
}

/// @brief Nội suy 1 chiều (tương đương np.interp)
inline double interp1d(double x_query, const std::vector<double>& x_vals, const std::vector<double>& y_vals) {
    if (x_vals.empty() || y_vals.empty()) return 0.0;
    if (x_query <= x_vals.front()) return y_vals.front();
    if (x_query >= x_vals.back()) return y_vals.back();

    auto it = std::lower_bound(x_vals.begin(), x_vals.end(), x_query);
    size_t idx = std::distance(x_vals.begin(), it);
    if (idx == 0) return y_vals[0];

    double x0 = x_vals[idx - 1];
    double x1 = x_vals[idx];
    double y0 = y_vals[idx - 1];
    double y1 = y_vals[idx];

    double t = (x1 > x0) ? ((x_query - x0) / (x1 - x0)) : 0.0;
    return y0 + t * (y1 - y0);
}

} // namespace av::math
