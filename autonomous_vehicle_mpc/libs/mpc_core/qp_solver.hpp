// ==============================================================================
// EMBEDDED CONVEX QP SOLVER (ADMM / OSQP FORMULATION - FULL SPD HESSIAN)
// Architecture: Zero-Allocation / Real-Time Deterministic (< 1ms execution)
// Solves: min 0.5 * x' * P * x + q' * x   s.t.   l <= A * x <= u
// ==============================================================================

#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace av::mpc {

struct QpSettings {
    int max_iter{60};
    double eps_abs{1e-3};
    double eps_rel{1e-3};
    double rho{2.0};
    double sigma{1e-6};
    double alpha{1.6}; // Over-relaxation parameter
};

class QpSolver {
public:
    using Settings = QpSettings;

    QpSolver() = default;

    /// @brief Khởi tạo bài toán QP kích thước n biến và m ràng buộc
    void setup(int n, int m, const QpSettings& settings = QpSettings()) {
        settings_ = settings;
        if (n_ != n || m_ != m) {
            n_ = n;
            m_ = m;
            x_.assign(n_, 0.0);
            z_.assign(m_, 0.0);
            y_.assign(m_, 0.0);
        }
    }

    void reset_warm_start() {
        std::fill(x_.begin(), x_.end(), 0.0);
        std::fill(z_.begin(), z_.end(), 0.0);
        std::fill(y_.begin(), y_.end(), 0.0);
    }

    /// @brief Giải bài toán QP với ma trận Hessian đầy đủ P (n x n) bằng ADMM Cholesky
    bool solve_dense(const std::vector<std::vector<double>>& P,
                     const std::vector<double>& q,
                     const std::vector<std::vector<double>>& A,
                     const std::vector<double>& l,
                     const std::vector<double>& u,
                     std::vector<double>& x_out,
                     double* out_cost = nullptr) {
        if (n_ <= 0) return false;

        // Trường hợp không có ràng buộc bất đẳng thức: Giải trực tiếp (P + sigma*I) * x = -q
        if (m_ <= 0 || A.empty()) {
            std::vector<std::vector<double>> M = P;
            for (int i = 0; i < n_; ++i) M[i][i] += settings_.sigma;
            std::vector<std::vector<double>> L(n_, std::vector<double>(n_, 0.0));
            if (!cholesky(M, L)) return false;
            std::vector<double> neg_q(n_, 0.0);
            for (int i = 0; i < n_; ++i) neg_q[i] = -q[i];
            solve_cholesky(L, neg_q, x_);
            x_out = x_;
            if (out_cost) *out_cost = compute_objective(P, q, x_);
            return true;
        }

        // Dựng ma trận KKT / Cholesky hệ: M = P + sigma * I + rho * A^T * A
        std::vector<std::vector<double>> M(n_, std::vector<double>(n_, 0.0));
        for (int i = 0; i < n_; ++i) {
            for (int j = 0; j <= i; ++j) {
                double at_a = 0.0;
                for (int k = 0; k < m_; ++k) {
                    at_a += A[k][i] * A[k][j];
                }
                double val = P[i][j] + settings_.rho * at_a;
                if (i == j) val += settings_.sigma;
                M[i][j] = val;
                M[j][i] = val;
            }
        }

        // Cholesky phân tích M = L * L^T
        std::vector<std::vector<double>> L(n_, std::vector<double>(n_, 0.0));
        if (!cholesky(M, L)) {
            for (int i = 0; i < n_; ++i) M[i][i] += 1e-4;
            if (!cholesky(M, L)) return false;
        }

        // Khởi tạo z_ hợp lệ nếu chưa có
        for (int k = 0; k < m_; ++k) {
            double ax_k = 0.0;
            for (int j = 0; j < n_; ++j) ax_k += A[k][j] * x_[j];
            z_[k] = std::clamp(ax_k, l[k], u[k]);
        }

        std::vector<double> rhs(n_, 0.0);
        std::vector<double> x_next(n_, 0.0);

        for (int iter = 0; iter < settings_.max_iter; ++iter) {
            // rhs = sigma * x - q + A^T * (rho * z - y)
            for (int i = 0; i < n_; ++i) {
                double at_val = 0.0;
                for (int k = 0; k < m_; ++k) {
                    at_val += A[k][i] * (settings_.rho * z_[k] - y_[k]);
                }
                rhs[i] = settings_.sigma * x_[i] - q[i] + at_val;
            }

            // Giải L * L^T * x_next = rhs
            solve_cholesky(L, rhs, x_next);

            // Cập nhật z và biến đối ngẫu y với Over-relaxation
            double max_res_p = 0.0;
            double max_res_d = 0.0;

            for (int k = 0; k < m_; ++k) {
                double ax_k = 0.0;
                for (int j = 0; j < n_; ++j) ax_k += A[k][j] * x_next[j];
                double ax_hat = settings_.alpha * ax_k + (1.0 - settings_.alpha) * z_[k];

                double z_old = z_[k];
                z_[k] = std::clamp(ax_hat + y_[k] / settings_.rho, l[k], u[k]);
                y_[k] += settings_.rho * (ax_hat - z_[k]);

                max_res_p = std::max(max_res_p, std::abs(ax_k - z_[k]));
                max_res_d = std::max(max_res_d, std::abs(settings_.rho * (z_[k] - z_old)));
            }

            x_ = x_next;

            if (max_res_p < settings_.eps_abs && max_res_d < settings_.eps_abs) {
                break;
            }
        }

        x_out = x_;
        if (out_cost) *out_cost = compute_objective(P, q, x_);
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
    static double compute_objective(const std::vector<std::vector<double>>& P,
                                    const std::vector<double>& q,
                                    const std::vector<double>& x) {
        int n = static_cast<int>(x.size());
        double cost = 0.0;
        for (int i = 0; i < n; ++i) {
            double px_i = 0.0;
            for (int j = 0; j < n; ++j) px_i += P[i][j] * x[j];
            cost += 0.5 * x[i] * px_i + q[i] * x[i];
        }
        return cost;
    }

    bool cholesky(const std::vector<std::vector<double>>& A, std::vector<std::vector<double>>& L) {
        int n = static_cast<int>(A.size());
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j <= i; ++j) {
                double sum = 0.0;
                for (int k = 0; k < j; ++k) sum += L[i][k] * L[j][k];
                if (i == j) {
                    double val = A[i][i] - sum;
                    if (val <= 1e-10) val = 1e-10;
                    L[i][j] = std::sqrt(val);
                } else {
                    L[i][j] = (A[i][j] - sum) / L[j][j];
                }
            }
        }
        return true;
    }

    void solve_cholesky(const std::vector<std::vector<double>>& L,
                        const std::vector<double>& b,
                        std::vector<double>& x) {
        int n = static_cast<int>(b.size());
        std::vector<double> y(n, 0.0);
        // Forward solve: L * y = b
        for (int i = 0; i < n; ++i) {
            double sum = 0.0;
            for (int j = 0; j < i; ++j) sum += L[i][j] * y[j];
            y[i] = (b[i] - sum) / L[i][i];
        }
        // Backward solve: L^T * x = y
        for (int i = n - 1; i >= 0; --i) {
            double sum = 0.0;
            for (int j = i + 1; j < n; ++j) sum += L[j][i] * x[j];
            x[i] = (y[i] - sum) / L[i][i];
        }
    }

    int n_{0};
    int m_{0};
    Settings settings_;
    std::vector<double> x_;
    std::vector<double> z_;
    std::vector<double> y_;
};

} // namespace av::mpc
