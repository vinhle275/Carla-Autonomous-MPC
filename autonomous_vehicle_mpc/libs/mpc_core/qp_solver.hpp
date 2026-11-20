// ==============================================================================
// OFFICIAL OSQP v1.0.0 C++20 RAII WRAPPER (AUTOSAR ADAPTIVE / MISRA-FRIENDLY)
// Uses official OSQP C library (third_party/osqp) with CSC sparse matrices,
// Ruiz equilibration, adaptive rho, polishing, and in-place data updates.
// Solves: min 0.5 * x' * P * x + q' * x   s.t.   l <= A * x <= u
// ==============================================================================

#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "osqp.h"

namespace av::mpc {

struct QpSettings {
    int max_iter{400};
    double eps_abs{1e-4};
    double eps_rel{1e-4};
    double rho{0.1};
    double sigma{1e-6};
    double alpha{1.6};
    bool warm_starting{true};
    bool polishing{true};
    bool adaptive_rho{true};
};

class QpSolver {
public:
    using Settings = QpSettings;

    QpSolver() = default;

    ~QpSolver() {
        cleanup_solver();
    }

    QpSolver(const QpSolver&) = delete;
    QpSolver& operator=(const QpSolver&) = delete;

    /// @brief Khởi tạo kích thước bài toán QP (n biến, m ràng buộc) và cấp phát sẵn buffer CSC
    void setup(int n, int m, const QpSettings& settings = QpSettings()) {
        settings_ = settings;
        if (n_ != n || m_ != m) {
            cleanup_solver();
            n_ = n;
            m_ = m;
            x_.assign(n_, 0.0);
            y_.assign(m_, 0.0);
            q_vec_.assign(n_, 0.0);
            l_vec_.assign(m_, 0.0);
            u_vec_.assign(m_, 0.0);
            P_p_.reserve(n_ + 1);
            A_p_.reserve(n_ + 1);
        }
    }

    void reset_warm_start() {
        std::fill(x_.begin(), x_.end(), 0.0);
        std::fill(y_.begin(), y_.end(), 0.0);
        if (solver_ != nullptr) {
            osqp_cold_start(solver_);
        }
    }

    int last_status() const { return last_status_; }
    int last_iter() const { return last_iter_; }

    /// @brief Giải bài toán QP bằng thư viện OSQP chính thức (tự động cập nhật tại chỗ nếu cùng cấu trúc thưa)
    bool solve_dense(const std::vector<std::vector<double>>& P,
                     const std::vector<double>& q,
                     const std::vector<std::vector<double>>& A,
                     const std::vector<double>& l,
                     const std::vector<double>& u,
                     std::vector<double>& x_out,
                     double* out_cost = nullptr) {
        if (n_ <= 0 || m_ < 0) return false;
        if (static_cast<int>(P.size()) != n_ || static_cast<int>(q.size()) != n_) return false;
        if (m_ > 0 && (static_cast<int>(A.size()) != m_ ||
                       static_cast<int>(l.size()) != m_ ||
                       static_cast<int>(u.size()) != m_)) {
            return false;
        }

        // 1 & 2. Chuyển P (tam giác trên) và A (m x n) sang CSC theo khung thưa tích lũy
        // Giữ nguyên các slot đã đăng ký (kể cả khi giá trị tạm thời = 0.0) để tránh Sparsity Churn
        bool p_expanded = build_or_reuse_csc(P, n_, n_, true, prev_P_p_, prev_P_i_, P_x_, P_i_, P_p_);
        bool a_expanded = build_or_reuse_csc(A, m_, n_, false, prev_A_p_, prev_A_i_, A_x_, A_i_, A_p_);

        // 3. Sao chép vector q, l, u sang kiểu OSQPFloat
        q_vec_.resize(n_);
        for (int i = 0; i < n_; ++i) q_vec_[i] = static_cast<OSQPFloat>(q[i]);
        l_vec_.resize(m_);
        u_vec_.resize(m_);
        for (int k = 0; k < m_; ++k) {
            l_vec_[k] = static_cast<OSQPFloat>(l[k]);
            u_vec_[k] = static_cast<OSQPFloat>(u[k]);
        }

        // 4. Cập nhật dữ liệu tại chỗ nếu cấu trúc thưa không đổi
        bool same_pattern = (solver_ != nullptr) && !p_expanded && !a_expanded &&
                            (P_p_ == prev_P_p_) && (P_i_ == prev_P_i_) &&
                            (A_p_ == prev_A_p_) && (A_i_ == prev_A_i_);

        if (same_pattern) {
            OSQPInt err_mat = osqp_update_data_mat(
                solver_,
                P_x_.data(), nullptr, static_cast<OSQPInt>(P_x_.size()),
                A_x_.data(), nullptr, static_cast<OSQPInt>(A_x_.size()));
            OSQPInt err_vec = osqp_update_data_vec(
                solver_, q_vec_.data(), l_vec_.data(), u_vec_.data());
            if (err_mat != 0 || err_vec != 0) {
                same_pattern = false;
            }
        }

        if (!same_pattern) {
            cleanup_solver();
            std::fill(y_.begin(), y_.end(), static_cast<OSQPFloat>(0.0));

            OSQPCscMatrix P_csc{};
            OSQPCscMatrix A_csc{};
            OSQPCscMatrix_set_data(&P_csc,
                                   static_cast<OSQPInt>(n_),
                                   static_cast<OSQPInt>(n_),
                                   static_cast<OSQPInt>(P_x_.size()),
                                   P_x_.data(),
                                   P_i_.data(),
                                   P_p_.data());
            OSQPCscMatrix_set_data(&A_csc,
                                   static_cast<OSQPInt>(m_),
                                   static_cast<OSQPInt>(n_),
                                   static_cast<OSQPInt>(A_x_.size()),
                                   A_x_.data(),
                                   A_i_.data(),
                                   A_p_.data());

            OSQPSettings osqp_settings{};
            osqp_set_default_settings(&osqp_settings);
            osqp_settings.verbose = 0;
            osqp_settings.warm_starting = settings_.warm_starting ? 1 : 0;
            osqp_settings.polishing = settings_.polishing ? 1 : 0;
            osqp_settings.max_iter = static_cast<OSQPInt>(settings_.max_iter);
            osqp_settings.eps_abs = static_cast<OSQPFloat>(settings_.eps_abs);
            osqp_settings.eps_rel = static_cast<OSQPFloat>(settings_.eps_rel);
            osqp_settings.rho = static_cast<OSQPFloat>(settings_.rho);
            osqp_settings.sigma = static_cast<OSQPFloat>(settings_.sigma);
            osqp_settings.alpha = static_cast<OSQPFloat>(settings_.alpha);
            osqp_settings.adaptive_rho = settings_.adaptive_rho
                                             ? OSQP_ADAPTIVE_RHO_UPDATE_ITERATIONS
                                             : OSQP_ADAPTIVE_RHO_UPDATE_DISABLED;

            OSQPInt exitflag = osqp_setup(&solver_,
                                          &P_csc,
                                          q_vec_.data(),
                                          &A_csc,
                                          l_vec_.data(),
                                          u_vec_.data(),
                                          static_cast<OSQPInt>(m_),
                                          static_cast<OSQPInt>(n_),
                                          &osqp_settings);
            if (exitflag != 0 || solver_ == nullptr) {
                cleanup_solver();
                return false;
            }

            prev_P_p_ = P_p_;
            prev_P_i_ = P_i_;
            prev_A_p_ = A_p_;
            prev_A_i_ = A_i_;
        }

        // 5. Nạp nghiệm Warm-start từ chu kỳ trước (chỉ nạp dual y_ khi giữ nguyên cấu trúc ràng buộc)
        if (settings_.warm_starting && !x_.empty() && static_cast<int>(x_.size()) == n_) {
            const OSQPFloat* y_warm = (same_pattern && static_cast<int>(y_.size()) == m_) ? y_.data() : nullptr;
            osqp_warm_start(solver_, x_.data(), y_warm);
        }

        // 6. Giải bài toán QP bằng OSQP
        OSQPInt solve_err = osqp_solve(solver_);
        if (solve_err != 0 || solver_->info == nullptr || solver_->solution == nullptr) {
            return false;
        }

        last_status_ = static_cast<int>(solver_->info->status_val);
        last_iter_ = static_cast<int>(solver_->info->iter);

        // Chấp nhận nghiệm khi OSQP hội tụ (OSQP_SOLVED, OSQP_SOLVED_INACCURATE)
        // hoặc đạt max_iter nhưng cả sai số nguyên thủy (prim_res) và đối ngẫu (dual_res) đều nhỏ (< 0.05)
        bool is_solved = (last_status_ == OSQP_SOLVED) ||
                         (last_status_ == OSQP_SOLVED_INACCURATE) ||
                         (last_status_ == OSQP_MAX_ITER_REACHED &&
                          solver_->info->prim_res < 0.05 &&
                          solver_->info->dual_res < 0.05);

        if (!is_solved || solver_->solution->x == nullptr) {
            return false;
        }

        for (int i = 0; i < n_; ++i) {
            if (!std::isfinite(static_cast<double>(solver_->solution->x[i]))) {
                reset_warm_start();
                return false;
            }
        }

        x_.resize(n_);
        x_out.resize(n_);
        for (int i = 0; i < n_; ++i) {
            x_[i] = solver_->solution->x[i];
            x_out[i] = static_cast<double>(solver_->solution->x[i]);
        }
        if (solver_->solution->y != nullptr) {
            y_.resize(m_);
            for (int k = 0; k < m_; ++k) {
                OSQPFloat y_val = solver_->solution->y[k];
                y_[k] = std::isfinite(static_cast<double>(y_val)) ? y_val : static_cast<OSQPFloat>(0.0);
            }
        }

        if (out_cost) {
            *out_cost = static_cast<double>(solver_->info->obj_val);
        }
        return true;
    }

    /// @brief Overload tương thích ngược với P dạng đường chéo
    bool solve(const std::vector<double>& P_diag,
               const std::vector<double>& q,
               const std::vector<std::vector<double>>& A,
               const std::vector<double>& l,
               const std::vector<double>& u,
               std::vector<double>& x_out) {
        std::vector<std::vector<double>> P(n_, std::vector<double>(n_, 0.0));
        for (int i = 0; i < n_ && i < static_cast<int>(P_diag.size()); ++i) {
            P[i][i] = P_diag[i];
        }
        return solve_dense(P, q, A, l, u, x_out);
    }

private:
    void cleanup_solver() {
        if (solver_ != nullptr) {
            osqp_cleanup(solver_);
            solver_ = nullptr;
        }
        prev_P_p_.clear();
        prev_P_i_.clear();
        prev_A_p_.clear();
        prev_A_i_.clear();
    }

    static bool build_or_reuse_csc(const std::vector<std::vector<double>>& M,
                                   int rows, int cols, bool upper_only,
                                   const std::vector<OSQPInt>& prev_p,
                                   const std::vector<OSQPInt>& prev_i,
                                   std::vector<OSQPFloat>& out_x,
                                   std::vector<OSQPInt>& out_i,
                                   std::vector<OSQPInt>& out_p) {
        bool has_prev = (static_cast<int>(prev_p.size()) == cols + 1);
        bool expanded = !has_prev;

        if (has_prev) {
            for (int col = 0; col < cols && !expanded; ++col) {
                OSQPInt idx = prev_p[col];
                OSQPInt idx_end = prev_p[col + 1];
                int max_row = upper_only ? col : (rows - 1);
                for (int row = 0; row <= max_row; ++row) {
                    bool in_prev = (idx < idx_end && prev_i[idx] == row);
                    if (in_prev) {
                        idx++;
                    } else {
                        double val = M[row][col];
                        if ((upper_only && row == col) || std::abs(val) > 1e-12) {
                            expanded = true;
                            break;
                        }
                    }
                }
            }

            if (!expanded) {
                out_p = prev_p;
                out_i = prev_i;
                out_x.resize(prev_i.size());
                for (int col = 0; col < cols; ++col) {
                    for (OSQPInt k = prev_p[col]; k < prev_p[col + 1]; ++k) {
                        out_x[k] = static_cast<OSQPFloat>(M[prev_i[k]][col]);
                    }
                }
                return false;
            }
        }

        out_x.clear();
        out_i.clear();
        out_p.resize(cols + 1);

        OSQPInt nnz = 0;
        for (int col = 0; col < cols; ++col) {
            out_p[col] = nnz;
            OSQPInt idx = has_prev ? prev_p[col] : 0;
            OSQPInt idx_end = has_prev ? prev_p[col + 1] : 0;
            int max_row = upper_only ? col : (rows - 1);
            for (int row = 0; row <= max_row; ++row) {
                bool in_prev = has_prev && (idx < idx_end && prev_i[idx] == row);
                if (in_prev) {
                    idx++;
                }
                double val = M[row][col];
                if (in_prev || (upper_only && row == col) || std::abs(val) > 1e-12) {
                    out_x.push_back(static_cast<OSQPFloat>(val));
                    out_i.push_back(static_cast<OSQPInt>(row));
                    nnz++;
                }
            }
        }
        out_p[cols] = nnz;
        return true;
    }

    int n_{0};
    int m_{0};
    int last_status_{0};
    int last_iter_{0};
    QpSettings settings_{};

    OSQPSolver* solver_{nullptr};

    // Các buffer cấp phát sẵn tái sử dụng qua các chu kỳ (tránh cấp phát heap lặp lại)
    std::vector<OSQPFloat> x_;
    std::vector<OSQPFloat> y_;
    std::vector<OSQPFloat> q_vec_;
    std::vector<OSQPFloat> l_vec_;
    std::vector<OSQPFloat> u_vec_;

    std::vector<OSQPFloat> P_x_;
    std::vector<OSQPInt> P_i_;
    std::vector<OSQPInt> P_p_;

    std::vector<OSQPFloat> A_x_;
    std::vector<OSQPInt> A_i_;
    std::vector<OSQPInt> A_p_;

    std::vector<OSQPInt> prev_P_p_;
    std::vector<OSQPInt> prev_P_i_;
    std::vector<OSQPInt> prev_A_p_;
    std::vector<OSQPInt> prev_A_i_;
};

} // namespace av::mpc
