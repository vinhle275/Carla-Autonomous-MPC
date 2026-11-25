#include "yellow_line_detector.hpp"
#include "common/frenet_utils.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace av::perception {

namespace {
constexpr double BEV_RES = 0.10;       // 0.10 m/px -> 160x180 BEV grid (siêu nhanh < 0.3ms)
constexpr double BEV_LAT = 8.0;        // [-8.0m, +8.0m]
constexpr double BEV_X_MIN = 2.0;      // 2.0m trước camera
constexpr double BEV_X_MAX = 20.0;     // 20.0m trước camera
constexpr int BEV_W = 160;             // 2 * 8.0 / 0.10
constexpr int BEV_H = 180;             // (20.0 - 2.0) / 0.10
constexpr double LANE_WIDTH_DEFAULT = 3.5;
constexpr double EMA_ALPHA = 0.15;
} // namespace

YellowLineDetector::YellowLineDetector(int img_w, int img_h)
    : img_w_(img_w), img_h_(img_h) {}

void YellowLineDetector::rgb_to_hsv_opencv(uint8_t r, uint8_t g, uint8_t b, int& out_h, int& out_s, int& out_v) {
    uint8_t cmax = std::max({r, g, b});
    uint8_t cmin = std::min({r, g, b});
    uint8_t diff = cmax - cmin;

    out_v = cmax;
    out_s = (cmax == 0) ? 0 : static_cast<int>((255.0 * diff) / cmax);

    if (diff == 0) {
        out_h = 0;
        return;
    }

    double h_deg = 0.0;
    if (cmax == r) {
        h_deg = 60.0 * (static_cast<double>(g) - static_cast<double>(b)) / diff;
    } else if (cmax == g) {
        h_deg = 60.0 * (2.0 + (static_cast<double>(b) - static_cast<double>(r)) / diff);
    } else {
        h_deg = 60.0 * (4.0 + (static_cast<double>(r) - static_cast<double>(g)) / diff);
    }
    if (h_deg < 0.0) h_deg += 360.0;
    out_h = static_cast<int>(h_deg * 0.5); // Chuẩn OpenCV H in [0, 180]
}

void YellowLineDetector::ground_to_pixel(double X, double Y, int width, int height, int& out_u, int& out_v) {
    double f = (0.5 * width) / std::tan(0.5 * config::FRONT_CAM_FOV_DEG * M_PI / 180.0);
    double cx = 0.5 * width;
    double cy = 0.5 * height;
    double th = -config::FRONT_CAM_PITCH_DEG * M_PI / 180.0; // Góc chúc xuống (+5 deg)
    double h = config::FRONT_CAM_Z_M;                        // 1.2m
    double depth = X * std::cos(th) + h * std::sin(th);
    double up = X * std::sin(th) - h * std::cos(th);
    if (depth <= 0.1) {
        out_u = -1;
        out_v = -1;
        return;
    }
    out_u = static_cast<int>(std::lround(cx + f * Y / depth));
    out_v = static_cast<int>(std::lround(cy - f * up / depth));
}

bool YellowLineDetector::pixel_to_ground_local(int u, int v, int width, int height,
                                               double& out_x_local, double& out_y_local) const {
    double fov_rad = config::FRONT_CAM_FOV_DEG * M_PI / 180.0;
    double focal = (0.5 * width) / std::tan(0.5 * fov_rad);
    double cu = 0.5 * width;
    double cv = 0.5 * height;

    double pitch_rad = config::FRONT_CAM_PITCH_DEG * M_PI / 180.0;
    double cos_p = std::cos(pitch_rad);
    double sin_p = std::sin(pitch_rad);

    double xc = 1.0;
    double yc = (u - cu) / focal;
    double zc = (v - cv) / focal;

    double ray_x =  xc * cos_p + zc * sin_p;
    double ray_y =  yc;
    double ray_z = -xc * sin_p + zc * cos_p;

    if (ray_z <= 1e-3) return false;

    double scale = config::FRONT_CAM_Z_M / ray_z;
    out_x_local = config::FRONT_CAM_X_M + scale * ray_x;
    out_y_local = scale * ray_y;
    return (out_x_local >= 2.5 && out_x_local <= 22.0);
}

LaneDetectionResult YellowLineDetector::detect_from_rgb_image(
    const uint8_t* rgb_data, int width, int height,
    double ego_x, double ego_y, double ego_yaw_rad,
    double ref_x, double ref_y, double ref_theta,
    bool map_has_yellow_solid, bool is_roadblock, bool is_junction, double dt,
    double map_road_min, double map_road_max) {

    auto t_start = std::chrono::steady_clock::now();

    if (!rgb_data || width <= 0 || height <= 0) {
        auto fallback = detect_from_features(ego_x, ego_y, ego_yaw_rad, ref_x, ref_y, ref_theta,
                                             map_has_yellow_solid, is_roadblock, is_junction, dt,
                                             map_road_min, map_road_max);
        fallback.detect_time_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_start).count();
        return fallback;
    }

    // Độ lệch ngang của xe so với đường tham chiếu (quy đổi từ hệ thân xe -> hệ Frenet)
    double d_ego_ref = -std::sin(ref_theta) * (ego_x - ref_x) + std::cos(ref_theta) * (ego_y - ref_y);

    // 1. Ước lượng độ sáng trung vị mặt đường nhựa vùng dưới tâm ảnh để lọc vạch trắng thích ứng
    std::vector<int> road_v_samples;
    road_v_samples.reserve(256);
    int r_y0 = static_cast<int>(height * 0.72);
    int r_x0 = static_cast<int>(width * 0.35);
    int r_x1 = static_cast<int>(width * 0.65);
    for (int v = r_y0; v < height; v += 2) {
        for (int u = r_x0; u < r_x1; u += 2) {
            int idx = (v * width + u) * 3;
            int h_val, s_val, v_val;
            rgb_to_hsv_opencv(rgb_data[idx], rgb_data[idx + 1], rgb_data[idx + 2], h_val, s_val, v_val);
            road_v_samples.push_back(v_val);
        }
    }
    int v_asphalt = 150;
    if (!road_v_samples.empty()) {
        std::nth_element(road_v_samples.begin(), road_v_samples.begin() + road_v_samples.size() / 2, road_v_samples.end());
        v_asphalt = road_v_samples[road_v_samples.size() / 2];
    }
    int white_v_thresh = std::max(config::WHITE_V_LOW, std::min(245, v_asphalt + 42));

    // 2. Analytic IPM trực tiếp lên lưới BEV (BEV_H x BEV_W)
    std::array<uint8_t, BEV_H * BEV_W> bev_y{};
    std::array<uint8_t, BEV_H * BEV_W> bev_w{};

    int u_horizon = 0, v_horizon = 0;
    ground_to_pixel(BEV_X_MAX, 0.0, width, height, u_horizon, v_horizon);
    int min_valid_v = std::max(0, v_horizon - 2);

    for (int r = 0; r < BEV_H; ++r) {
        double X = BEV_X_MAX - r * BEV_RES;
        for (int c = 0; c < BEV_W; ++c) {
            double Y = c * BEV_RES - BEV_LAT;
            int u = -1, v = -1;
            ground_to_pixel(X, Y, width, height, u, v);
            if (u < 0 || u >= width || v < min_valid_v || v >= height) continue;

            int idx = (v * width + u) * 3;
            int h_val, s_val, v_val;
            rgb_to_hsv_opencv(rgb_data[idx], rgb_data[idx + 1], rgb_data[idx + 2], h_val, s_val, v_val);

            bool is_yel = (h_val >= config::YELLOW_H_LOW && h_val <= config::YELLOW_H_HIGH &&
                           s_val >= config::YELLOW_S_LOW && s_val <= config::YELLOW_S_HIGH &&
                           v_val >= config::YELLOW_V_LOW && v_val <= config::YELLOW_V_HIGH);
            if (is_yel) {
                bev_y[r * BEV_W + c] = 1;
            } else if (v_val >= white_v_thresh && s_val <= config::WHITE_S_HIGH) {
                bev_w[r * BEV_W + c] = 1;
            }
        }
    }

    // Hàm loại bỏ vạch dừng ngang rộng (>= 0.8m = 8 cột liên tiếp) và tìm các vạch dọc bằng Sliding Window
    auto find_bev_lines = [&](std::array<uint8_t, BEV_H * BEV_W>& bev, bool is_yellow_mask) -> std::vector<DetectedBeVLine> {
        constexpr int wide_run = static_cast<int>(0.80 / BEV_RES); // 8 cột
        for (int r = 0; r < BEV_H; ++r) {
            int run = 0;
            for (int c = 0; c < BEV_W; ++c) {
                if (bev[r * BEV_W + c]) {
                    run++;
                } else {
                    if (run >= wide_run) {
                        for (int k = std::max(0, c - run - 1); k < std::min(BEV_W, c + 1); ++k) {
                            bev[r * BEV_W + k] = 0;
                        }
                    }
                    run = 0;
                }
            }
            if (run >= wide_run) {
                for (int k = std::max(0, BEV_W - run - 1); k < BEV_W; ++k) {
                    bev[r * BEV_W + k] = 0;
                }
            }
        }

        // Histogram vùng gần (X từ 2.5m đến 10.5m trước camera)
        int r_near0 = std::max(0, static_cast<int>((BEV_X_MAX - 10.5) / BEV_RES));
        int r_near1 = std::min(BEV_H, static_cast<int>((BEV_X_MAX - 2.5) / BEV_RES));
        std::array<double, BEV_W> hist_raw{};
        for (int r = r_near0; r < r_near1; ++r) {
            for (int c = 0; c < BEV_W; ++c) {
                if (bev[r * BEV_W + c]) hist_raw[c] += 1.0;
            }
        }
        std::array<double, BEV_W> hist{};
        for (int c = 0; c < BEV_W; ++c) {
            double s = 0.0;
            int cnt = 0;
            for (int dc = -2; dc <= 2; ++dc) {
                int cc = c + dc;
                if (cc >= 0 && cc < BEV_W) {
                    s += hist_raw[cc];
                    cnt++;
                }
            }
            hist[c] = (cnt > 0) ? (s / cnt) : 0.0;
        }

        std::vector<int> peaks;
        int sep = static_cast<int>(0.80 / BEV_RES);
        auto h2 = hist;
        while (true) {
            auto it = std::max_element(h2.begin(), h2.end());
            double max_v = *it;
            if (max_v < 3.0) break;
            int c_peak = static_cast<int>(std::distance(h2.begin(), it));
            peaks.push_back(c_peak);
            for (int k = std::max(0, c_peak - sep); k <= std::min(BEV_W - 1, c_peak + sep); ++k) {
                h2[k] = 0.0;
            }
        }

        constexpr int n_win = 18;
        int win_h = BEV_H / n_win;
        int margin = static_cast<int>(0.65 / BEV_RES);
        std::vector<DetectedBeVLine> out_lines;

        for (int base : peaks) {
            double cur = static_cast<double>(base);
            double step = 0.0;
            std::vector<double> xs_col;
            std::vector<double> ys_row;
            int valid_wins = 0;

            for (int k = 0; k < n_win; ++k) {
                int r1 = BEV_H - k * win_h;
                int r0 = std::max(0, r1 - win_h);
                int c_min = std::max(0, static_cast<int>( std::floor(cur - margin)));
                int c_max = std::min(BEV_W - 1, static_cast<int>(std::ceil(cur + margin)));

                double sum_c = 0.0;
                int count_pts = 0;
                for (int rr = r0; rr < r1; ++rr) {
                    for (int cc = c_min; cc <= c_max; ++cc) {
                        if (bev[rr * BEV_W + cc]) {
                            sum_c += cc;
                            count_pts++;
                            xs_col.push_back(cc);
                            ys_row.push_back(rr);
                        }
                    }
                }
                if (count_pts >= 2) {
                    double mc = sum_c / count_pts;
                    if (valid_wins > 0) {
                        step = 0.4 * step + 0.6 * std::clamp(mc - (cur - step), -3.0, 3.0);
                    }
                    cur = mc + step;
                    valid_wins++;
                } else {
                    cur += step;
                }
            }

            if (valid_wins < 2 || xs_col.size() < 6) continue;

            auto [min_r_it, max_r_it] = std::minmax_element(ys_row.begin(), ys_row.end());
            double x_min = BEV_X_MAX - (*max_r_it) * BEV_RES;
            double x_max = BEV_X_MAX - (*min_r_it) * BEV_RES;
            if (x_min > 8.5 || (x_max - x_min) < 1.8) continue;

            // Hồi quy tuyến tính col = a * row + b
            double n_d = static_cast<double>(xs_col.size());
            double mean_r = std::accumulate(ys_row.begin(), ys_row.end(), 0.0) / n_d;
            double mean_c = std::accumulate(xs_col.begin(), xs_col.end(), 0.0) / n_d;
            double num = 0.0, den = 0.0;
            for (size_t i = 0; i < xs_col.size(); ++i) {
                double dr = ys_row[i] - mean_r;
                num += dr * (xs_col[i] - mean_c);
                den += dr * dr;
            }
            double slope_rc = (den > 1e-6) ? (num / den) : 0.0;
            double intercept_rc = mean_c - slope_rc * mean_r;

            // Kiểm tra độ lệch chuẩn ngang
            double sq_err = 0.0;
            for (size_t i = 0; i < xs_col.size(); ++i) {
                double e = xs_col[i] - (slope_rc * ys_row[i] + intercept_rc);
                sq_err += e * e;
            }
            if (std::sqrt(sq_err / n_d) > (0.35 / BEV_RES)) continue;

            // Tính tỷ lệ lấp đầy theo hàng (fill_ratio)
            int r_start = static_cast<int>(*min_r_it);
            int r_end = static_cast<int>(*max_r_it);
            int hit_rows = 0;
            for (int rr = r_start; rr <= r_end; ++rr) {
                int c_pred = std::clamp(static_cast<int>(std::lround(slope_rc * rr + intercept_rc)), 3, BEV_W - 4);
                bool row_hit = false;
                for (int cc = c_pred - 3; cc <= c_pred + 3; ++cc) {
                    if (bev[rr * BEV_W + cc]) {
                        row_hit = true;
                        break;
                    }
                }
                if (row_hit) hit_rows++;
            }
            double fill = static_cast<double>(hit_rows) / static_cast<double>(BEV_H);
            if (fill < 0.10) continue;

            // Ngoại suy tuyến tính vùng gần (X < 11.0m) về tâm xe (X = -FRONT_CAM_X_M)
            std::vector<double> x_near, c_near;
            for (size_t i = 0; i < xs_col.size(); ++i) {
                double x_val = BEV_X_MAX - ys_row[i] * BEV_RES;
                if (x_val < 11.0) {
                    x_near.push_back(x_val);
                    c_near.push_back(xs_col[i]);
                }
            }
            const auto& x_fit = (x_near.size() > 6) ? x_near : ys_row;
            const auto& c_fit = (x_near.size() > 6) ? c_near : xs_col;
            double col_at_ego = mean_c;
            if (x_near.size() > 6) {
                double m_x = std::accumulate(x_fit.begin(), x_fit.end(), 0.0) / x_fit.size();
                double m_c = std::accumulate(c_fit.begin(), c_fit.end(), 0.0) / c_fit.size();
                double num_x = 0.0, den_x = 0.0;
                for (size_t i = 0; i < x_fit.size(); ++i) {
                    double dx = x_fit[i] - m_x;
                    num_x += dx * (c_fit[i] - m_c);
                    den_x += dx * dx;
                }
                double a_x = (den_x > 1e-6) ? (num_x / den_x) : 0.0;
                double b_x = m_c - a_x * m_x;
                col_at_ego = a_x * (-config::FRONT_CAM_X_M) + b_x;
            } else {
                double row_at_ego = (BEV_X_MAX - (-config::FRONT_CAM_X_M)) / BEV_RES;
                col_at_ego = slope_rc * row_at_ego + intercept_rc;
            }

            double d_ego_m = col_at_ego * BEV_RES - BEV_LAT;
            types::LineType ltype = (fill >= 0.68) ? types::LineType::SOLID : types::LineType::DASHED;
            out_lines.push_back({d_ego_m, is_yellow_mask, ltype, fill});
        }
        return out_lines;
    };

    auto lines_y = find_bev_lines(bev_y, true);
    auto lines_w = find_bev_lines(bev_w, false);
    std::vector<DetectedBeVLine> all_lines = lines_y;
    all_lines.insert(all_lines.end(), lines_w.begin(), lines_w.end());
    std::sort(all_lines.begin(), all_lines.end(), [](const auto& a, const auto& b) {
        return a.d_ego_m < b.d_ego_m;
    });

    // Gộp vạch trùng vị trí (< 0.55m), ưu tiên vạch vàng rồi fill_ratio cao hơn
    std::vector<DetectedBeVLine> merged;
    for (const auto& ln : all_lines) {
        if (!merged.empty() && std::abs(ln.d_ego_m - merged.back().d_ego_m) < 0.55) {
            const auto& a = merged.back();
            bool keep_a = (a.is_yellow != ln.is_yellow) ? a.is_yellow : (a.fill_ratio >= ln.fill_ratio);
            if (!keep_a) merged.back() = ln;
        } else {
            merged.push_back(ln);
        }
    }

    // Kiểm tra nhiễu lưới vạch vàng ô mắt cáo trong giao lộ hoặc vạch đi bộ Zebra Crosswalk
    int n_yellow = 0;
    bool yellow_on_right = false;
    std::vector<DetectedBeVLine> white_only;
    for (const auto& l : merged) {
        if (l.is_yellow) {
            n_yellow++;
            if (l.d_ego_m > 0.5) yellow_on_right = true;
        } else {
            white_only.push_back(l);
        }
    }
    int close_white_pairs = 0;
    for (size_t i = 1; i < white_only.size(); ++i) {
        if (std::abs(white_only[i].d_ego_m - white_only[i - 1].d_ego_m) < 1.8) {
            close_white_pairs++;
        }
    }

    bool cv_rejected = (n_yellow >= 2 || yellow_on_right || white_only.size() >= 5 || close_white_pairs >= 2);

    LaneDetectionResult res;
    if (!cv_rejected && !merged.empty()) {
        std::vector<DetectedBeVLine> left, right;
        for (const auto& l : merged) {
            if (l.d_ego_m < 0.0) left.push_back(l);
            else if (l.d_ego_m > 0.0) right.push_back(l);
        }
        std::sort(left.begin(), left.end(), [](const auto& a, const auto& b) { return a.d_ego_m > b.d_ego_m; }); // gần -> xa
        std::sort(right.begin(), right.end(), [](const auto& a, const auto& b) { return a.d_ego_m < b.d_ego_m; });

        bool has_yellow_left = false;
        for (const auto& l : left) {
            if (l.is_yellow) {
                has_yellow_left = true;
                res.line_type = (l.line_type != types::LineType::NONE) ? l.line_type : types::LineType::SOLID;
                res.is_yellow_solid = (res.line_type == types::LineType::SOLID);
                res.d_yellow_line_m = std::max(0.0, -l.d_ego_m);
                break;
            }
        }

        auto walk_out = [&](const std::vector<DetectedBeVLine>& side, int sign, bool has_yel) -> std::pair<bool, std::pair<double, std::string>> {
            if (side.empty()) return {false, {0.0, "none"}};
            double prev = 0.0;
            int crossed_dashed = 0;
            for (const auto& ln : side) {
                if (prev != 0.0 && std::abs(ln.d_ego_m - prev) < 2.0) continue;
                if (prev != 0.0 && std::abs(ln.d_ego_m - prev) > LANE_WIDTH_DEFAULT + 1.5) {
                    return {true, {prev + sign * LANE_WIDTH_DEFAULT, "assumed"}};
                }
                if (ln.is_yellow) return {true, {ln.d_ego_m, "yellow"}};
                if (ln.line_type == types::LineType::SOLID) return {true, {ln.d_ego_m, "solid"}};
                prev = ln.d_ego_m;
                crossed_dashed++;
                if (crossed_dashed >= 1) break;
            }
            if (prev == 0.0) return {false, {0.0, "none"}};
            double far = prev + sign * LANE_WIDTH_DEFAULT;
            double max_lat = (sign < 0 && !has_yel) ? 5.25 : (BEV_LAT - 0.5);
            if (std::abs(far) > max_lat) far = sign * max_lat;
            return {true, {far, "assumed"}};
        };

        auto [ok_l, bound_l] = walk_out(left, -1, has_yellow_left);
        auto [ok_r, bound_r] = walk_out(right, +1, false);

        bool has_l = !left.empty() && (-left[0].d_ego_m < 3.8);
        bool has_r = !right.empty() && (right[0].d_ego_m < 3.8);
        res.confidence = (has_l && has_r) ? 1.0 : ((has_l || has_r) ? 0.4 : 0.0);

        if (res.confidence >= 0.5 && ok_l && ok_r) {
            double clr_l = (bound_l.second == "yellow")
                           ? (config::EGO_HALF_WIDTH_M + config::YELLOW_LINE_CLEARANCE_M)
                           : config::R_CLEARANCE_EDGE;
            double raw_min = std::min(bound_l.first + clr_l + d_ego_ref, config::STANDARD_ROAD_MIN_M);
            double raw_max = std::max(bound_r.first - config::R_CLEARANCE_EDGE + d_ego_ref, 0.45);
            res.road_min_m = raw_min;
            res.road_max_m = raw_max;
            res.source = "CV";
            if (res.is_yellow_solid) {
                hold_timer_s_ = hold_max_time_s_;
            }
        }
    }

    // 3. Fallback về OpenDRIVE Map (_map_fallback chuẩn src/yellow_line_detector.py) khi confidence < 0.5
    if (res.confidence < 0.5 || res.source != "CV") {
        res = detect_from_features(ego_x, ego_y, ego_yaw_rad, ref_x, ref_y, ref_theta,
                                   map_has_yellow_solid, is_roadblock, is_junction, dt,
                                   map_road_min, map_road_max);
    }

    // 4. Lọc mượt EMA (alpha = 0.15, bù độ dịch chuyển đường tham chiếu khi chuyển làn chéo để không bị trễ pha)
    res.road_min_m = std::min(res.road_min_m, config::STANDARD_ROAD_MIN_M);
    res.road_max_m = std::max(res.road_max_m, 0.45);
    if (!has_ema_) {
        ema_min_ = res.road_min_m;
        ema_max_ = res.road_max_m;
        prev_map_min_ = map_road_min;
        prev_map_max_ = map_road_max;
        has_ema_ = true;
    } else {
        double d_map_min = map_road_min - prev_map_min_;
        double d_map_max = map_road_max - prev_map_max_;
        prev_map_min_ = map_road_min;
        prev_map_max_ = map_road_max;

        ema_min_ += d_map_min;
        ema_max_ += d_map_max;

        if (std::abs(res.road_min_m - ema_min_) > 2.5) {
            ema_min_ = res.road_min_m;
        } else {
            ema_min_ += EMA_ALPHA * (res.road_min_m - ema_min_);
        }
        if (std::abs(res.road_max_m - ema_max_) > 2.5) {
            ema_max_ = res.road_max_m;
        } else {
            ema_max_ += EMA_ALPHA * (res.road_max_m - ema_max_);
        }
    }
    res.road_min_m = std::min(ema_min_, config::STANDARD_ROAD_MIN_M);
    res.road_max_m = std::max(ema_max_, 0.45);
    last_valid_road_min_ = res.road_min_m;
    last_valid_road_max_ = res.road_max_m;

    res.detect_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_start).count();
    return res;
}

LaneDetectionResult YellowLineDetector::detect_from_features(
    double ego_x, double ego_y, double /*ego_yaw_rad*/,
    double ref_x, double ref_y, double ref_theta,
    bool map_has_yellow_solid, bool /*is_roadblock*/, bool /*is_junction*/, double /*dt*/,
    double map_road_min, double map_road_max) {

    LaneDetectionResult res;

    // Chiếu vị trí xe so với tim làn (đồng bộ 100% với _map_fallback trong src/yellow_line_detector.py)
    double nx = -std::sin(ref_theta);
    double ny = std::cos(ref_theta);
    double d_lat = (ego_x - ref_x) * nx + (ego_y - ref_y) * ny;

    double d_left = map_road_min - config::R_CLEARANCE_EDGE;
    res.road_min_m = std::min(map_road_min, config::STANDARD_ROAD_MIN_M);
    res.road_max_m = std::max(map_road_max, 0.45);
    res.confidence = 0.0; // _map_fallback giữ confidence = 0.0 để phân biệt với CV thực
    res.source = "MAP";

    if (map_has_yellow_solid) {
        res.is_yellow_solid = true;
        res.line_type = types::LineType::SOLID;
        res.d_yellow_line_m = std::max(0.0, d_lat - d_left);
    } else {
        res.is_yellow_solid = false;
        res.line_type = types::LineType::NONE;
        res.d_yellow_line_m = std::max(0.0, d_lat - d_left);
    }

    return res;
}

} // namespace av::perception
