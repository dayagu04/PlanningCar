// nav_core.cpp — 核心算法实现
#include "nav_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <queue>
#include <stdexcept>

namespace navcore {

// ============================================================
//  HeightField — 双线性插值
// ============================================================
HeightField::HeightField(std::vector<double> heights, int resolution,
                         double size, double origin_offset)
    : heights_(std::move(heights)),
      res_(resolution),
      size_(size),
      spacing_(size / static_cast<double>(resolution - 1)),
      origin_(origin_offset) {
    if (static_cast<int>(heights_.size()) != res_ * res_) {
        throw std::invalid_argument("HeightField: heights length != res*res");
    }
}

double HeightField::sample(double x, double y) const {
    if (heights_.empty()) return 0.0;
    const double lx = x - origin_;
    const double ly = y - origin_;
    const double gx = lx / spacing_;
    const double gy = ly / spacing_;
    if (gx < 0.0 || gy < 0.0 ||
        gx > static_cast<double>(res_ - 1) ||
        gy > static_cast<double>(res_ - 1)) {
        return 0.0;
    }
    const int i0 = static_cast<int>(gx);
    const int j0 = static_cast<int>(gy);
    const int i1 = std::min(i0 + 1, res_ - 1);
    const int j1 = std::min(j0 + 1, res_ - 1);
    const double fx = gx - i0;
    const double fy = gy - j0;
    const double h00 = heights_[j0 * res_ + i0];
    const double h10 = heights_[j0 * res_ + i1];
    const double h01 = heights_[j1 * res_ + i0];
    const double h11 = heights_[j1 * res_ + i1];
    const double h0 = h00 * (1.0 - fx) + h10 * fx;
    const double h1 = h01 * (1.0 - fx) + h11 * fx;
    return h0 * (1.0 - fy) + h1 * fy;
}

// ============================================================
//  地形特征
// ============================================================
TerrainFeatures extract_features(const double* heights, int rows, int cols,
                                 double cell_size) {
    TerrainFeatures f{0.0, 0.0, 0.0, 0.0};
    const int n = rows * cols;
    if (n < 2 || heights == nullptr) return f;

    // Mean / min / max / range
    double sum = 0.0, hmin = heights[0], hmax = heights[0];
    for (int k = 0; k < n; ++k) {
        sum += heights[k];
        if (heights[k] < hmin) hmin = heights[k];
        if (heights[k] > hmax) hmax = heights[k];
    }
    const double mean = sum / static_cast<double>(n);

    // Average gradient magnitude (central diff; one-sided at edges)
    double grad_sum = 0.0;
    int grad_n = 0;
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            double dy, dx;
            if (rows == 1) dy = 0.0;
            else if (r == 0) dy = (heights[(r + 1) * cols + c] - heights[r * cols + c]) / cell_size;
            else if (r == rows - 1) dy = (heights[r * cols + c] - heights[(r - 1) * cols + c]) / cell_size;
            else dy = (heights[(r + 1) * cols + c] - heights[(r - 1) * cols + c]) / (2.0 * cell_size);

            if (cols == 1) dx = 0.0;
            else if (c == 0) dx = (heights[r * cols + (c + 1)] - heights[r * cols + c]) / cell_size;
            else if (c == cols - 1) dx = (heights[r * cols + c] - heights[r * cols + (c - 1)]) / cell_size;
            else dx = (heights[r * cols + (c + 1)] - heights[r * cols + (c - 1)]) / (2.0 * cell_size);

            grad_sum += std::sqrt(dx * dx + dy * dy);
            ++grad_n;
        }
    }
    const double mean_grad = grad_n > 0 ? grad_sum / grad_n : 0.0;
    const double slope_deg = std::atan(mean_grad) * 180.0 / 3.14159265358979323846;

    // Roughness = std of residuals after subtracting the best-fit plane
    //   z(r, c) ≈ a*c + b*r + h0     (least-squares fit on the grid)
    // For uniformly-spaced grids the normal equations decouple — Sxx, Syy
    // depend only on grid size — so we get away with three accumulators.
    // Mean centred coordinates avoid the constant term.
    const double crow = (rows - 1) * 0.5;
    const double ccol = (cols - 1) * 0.5;
    double Sxx = 0.0, Syy = 0.0, Sxy = 0.0;
    double Sxz = 0.0, Syz = 0.0;
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const double xc = (c - ccol) * cell_size;
            const double yr = (r - crow) * cell_size;
            const double z  = heights[r * cols + c] - mean;
            Sxx += xc * xc;
            Syy += yr * yr;
            Sxy += xc * yr;
            Sxz += xc * z;
            Syz += yr * z;
        }
    }
    // Solve [Sxx Sxy; Sxy Syy] [a; b] = [Sxz; Syz]
    double a = 0.0, b = 0.0;
    const double det = Sxx * Syy - Sxy * Sxy;
    if (std::fabs(det) > 1e-12) {
        a = (Sxz * Syy - Syz * Sxy) / det;
        b = (Syz * Sxx - Sxz * Sxy) / det;
    }
    double sq_resid = 0.0;
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const double xc = (c - ccol) * cell_size;
            const double yr = (r - crow) * cell_size;
            const double z  = heights[r * cols + c] - mean;
            const double resid = z - (a * xc + b * yr);
            sq_resid += resid * resid;
        }
    }
    const double rough = std::sqrt(sq_resid / static_cast<double>(n));

    f.slope_deg = slope_deg;
    f.roughness = rough;
    f.height_range = hmax - hmin;
    f.mean_height = mean;
    return f;
}

// ============================================================
//  分类器
// ============================================================
const char* terrain_name(TerrainType t) {
    switch (t) {
        case TerrainType::Flat: return "flat";
        case TerrainType::Slope: return "slope";
        case TerrainType::Rough: return "rough";
        case TerrainType::Transition: return "transition";
    }
    return "unknown";
}

TerrainClassifier::TerrainClassifier(ClassifierThresholds t, int history_len)
    : t_(t), history_len_(history_len) {
    history_.reserve(history_len_);
}

void TerrainClassifier::reset() { history_.clear(); }

TerrainType TerrainClassifier::classify_static(double slope_deg, double roughness,
                                               double imu_pitch_deg,
                                               double imu_roll_deg) const {
    const double pitch = std::abs(imu_pitch_deg);
    const double roll  = std::abs(imu_roll_deg);

    if (pitch >= t_.slope_imu_pitch_min && roughness < t_.slope_roughness_max)
        return TerrainType::Slope;
    if (roughness >= t_.rough_roughness_min || roll >= t_.rough_imu_roll_min)
        return TerrainType::Rough;
    if (slope_deg < t_.flat_slope_max && roughness < t_.flat_roughness_max
        && pitch < t_.flat_imu_pitch_max)
        return TerrainType::Flat;
    if (slope_deg >= t_.slope_angle_min)
        return TerrainType::Slope;
    return TerrainType::Transition;
}

TerrainType TerrainClassifier::classify(double slope_deg, double roughness,
                                        double imu_pitch_deg, double imu_roll_deg) {
    TerrainType cur = classify_static(slope_deg, roughness, imu_pitch_deg, imu_roll_deg);
    history_.push_back(cur);
    if (static_cast<int>(history_.size()) > history_len_) {
        history_.erase(history_.begin());
    }
    if (history_.size() >= 3) {
        const auto& h = history_;
        const std::size_t n = h.size();
        // 最近 3 个全部不同 → Transition
        const TerrainType a = h[n - 3], b = h[n - 2], c = h[n - 1];
        if (a != b && b != c && a != c) return TerrainType::Transition;
    }
    return cur;
}

// ============================================================
//  TSP
// ============================================================
namespace {
inline double dist(const Point& a, const Point& b) {
    const double dx = a.first - b.first;
    const double dy = a.second - b.second;
    return std::sqrt(dx * dx + dy * dy);
}
double tour_length(const Point& start, const std::vector<Point>& tour) {
    if (tour.empty()) return 0.0;
    double total = dist(start, tour.front());
    for (std::size_t i = 0; i + 1 < tour.size(); ++i) {
        total += dist(tour[i], tour[i + 1]);
    }
    return total;
}
}  // namespace

std::pair<std::vector<Point>, TSPInfo>
optimize_waypoint_order(Point start, std::vector<Point> waypoints) {
    TSPInfo info{0.0, 0.0, 0.0, 0.0};
    if (waypoints.empty()) return {std::move(waypoints), info};

    info.original_length = tour_length(start, waypoints);

    // Greedy NN
    std::vector<Point> remaining = waypoints;
    std::vector<Point> tour;
    tour.reserve(remaining.size());
    Point cur = start;
    while (!remaining.empty()) {
        std::size_t best = 0;
        double best_d = dist(cur, remaining[0]);
        for (std::size_t i = 1; i < remaining.size(); ++i) {
            double d = dist(cur, remaining[i]);
            if (d < best_d) { best_d = d; best = i; }
        }
        tour.push_back(remaining[best]);
        cur = remaining[best];
        remaining.erase(remaining.begin() + best);
    }
    info.greedy_length = tour_length(start, tour);

    // 2-opt
    if (tour.size() >= 4) {
        bool improved = true;
        int it = 0;
        const int max_iter = 100;
        double best_len = info.greedy_length;
        while (improved && it < max_iter) {
            improved = false;
            ++it;
            for (std::size_t i = 0; i + 1 < tour.size(); ++i) {
                for (std::size_t j = i + 1; j < tour.size(); ++j) {
                    std::reverse(tour.begin() + i, tour.begin() + j + 1);
                    double new_len = tour_length(start, tour);
                    if (new_len < best_len - 1e-6) {
                        best_len = new_len;
                        improved = true;
                    } else {
                        // 还原
                        std::reverse(tour.begin() + i, tour.begin() + j + 1);
                    }
                }
            }
        }
        info.optimized_length = best_len;
    } else {
        info.optimized_length = info.greedy_length;
    }
    info.improvement_pct = info.original_length > 0
        ? 100.0 * (info.original_length - info.optimized_length) / info.original_length
        : 0.0;
    return {std::move(tour), info};
}

}  // namespace navcore
