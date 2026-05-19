// nav_core.hpp — 核心算法 C++ 接口
#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

namespace navcore {

using Point = std::pair<double, double>;
using ElevationFn = std::function<double(double, double)>;

// ================== 双线性高度采样 ==================
class HeightField {
public:
    HeightField() = default;
    HeightField(std::vector<double> heights, int resolution,
                double size, double origin_offset);

    double sample(double x, double y) const;
    int resolution() const { return res_; }
    bool empty() const { return heights_.empty(); }

private:
    std::vector<double> heights_;
    int res_ = 0;
    double size_ = 0.0;
    double spacing_ = 0.0;
    double origin_ = 0.0;
};

// ================== 地形特征 ==================
struct TerrainFeatures {
    double slope_deg;
    double roughness;
    double height_range;
    double mean_height;
};
TerrainFeatures extract_features(const double* heights, int rows, int cols,
                                 double cell_size);

// ================== 地形分类 ==================
enum class TerrainType : int { Flat = 0, Slope = 1, Rough = 2, Transition = 3 };
const char* terrain_name(TerrainType t);

struct ClassifierThresholds {
    double flat_slope_max      = 5.0;
    double flat_roughness_max  = 0.02;
    double flat_imu_pitch_max  = 3.0;
    double slope_angle_min     = 5.0;
    double slope_imu_pitch_min = 3.0;
    double slope_roughness_max = 0.05;
    double rough_roughness_min = 0.05;
    double rough_imu_roll_min  = 2.0;
};

class TerrainClassifier {
public:
    explicit TerrainClassifier(ClassifierThresholds t = {}, int history_len = 5);
    TerrainType classify(double slope_deg, double roughness,
                         double imu_pitch_deg, double imu_roll_deg);
    void reset();

private:
    TerrainType classify_static(double slope_deg, double roughness,
                                double imu_pitch_deg, double imu_roll_deg) const;
    ClassifierThresholds t_;
    int history_len_;
    std::vector<TerrainType> history_;
};

// ================== TSP ==================
struct TSPInfo {
    double original_length;
    double greedy_length;
    double optimized_length;
    double improvement_pct;
};

std::pair<std::vector<Point>, TSPInfo>
optimize_waypoint_order(Point start, std::vector<Point> waypoints);

// ================== 路径规划 ==================
struct PlannerConfig {
    double grid_size = 0.5;
    double world_size = 30.0;
    double heuristic_weight = 1.0;  // 1.0 纯 A*
    bool use_theta_star = true;     // any-angle 平滑
};

struct Obstacle {
    double x, y, radius;
};

class GridPlanner {
public:
    explicit GridPlanner(PlannerConfig cfg);

    // 重建代价图：地形类别基础代价 + 海拔梯度，最后重涂持久障碍。
    // 传 nullptr 跳过海拔项（兼容 lidar-only 场景）
    void update_cost_map(TerrainType terrain_type, const ElevationFn* sampler);

    void add_obstacles(const std::vector<Obstacle>& obs);
    void clear_obstacles();

    // 返回路径（世界坐标），找不到返回空
    std::vector<Point> plan(Point start, Point goal);

    int grid_dim() const { return grid_dim_; }
    double cost_at(int gx, int gy) const;
    PlannerConfig config() const { return cfg_; }

private:
    PlannerConfig cfg_;
    int grid_dim_;
    std::vector<double> cost_;
    std::vector<Obstacle> obstacles_;

    inline int cell_idx(int gx, int gy) const { return gx * grid_dim_ + gy; }
    std::pair<int, int> world_to_grid(double x, double y) const;
    Point grid_to_world(int gx, int gy) const;
    void paint_obstacle(double x, double y, double radius);
    std::optional<std::pair<int, int>> nearest_free_cell(int cx, int cy, int max_ring) const;
    bool line_of_sight_grid(int x0, int y0, int x1, int y1) const;
    bool segment_clear_world(double x0, double y0, double x1, double y1) const;
    std::vector<Point> reconstruct_path(const std::vector<int>& parents,
                                        int start_id, int goal_id) const;
    std::vector<Point> los_simplify(const std::vector<Point>& path) const;
};

// 全局工具：LOS-aware 路径简化（用作 Python 回退）
std::vector<Point> simplify_path_los(const std::vector<Point>& path,
                                     const std::vector<Obstacle>& obstacles,
                                     double tolerance = 0.3);

}  // namespace navcore
