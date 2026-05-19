// nav_planner.cpp — A* / Theta* 实现
#include "nav_core.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

namespace navcore {

namespace {
constexpr double SQRT2 = 1.41421356237309504880;
constexpr double BLOCKED = 999.0;
constexpr double INF = std::numeric_limits<double>::infinity();

inline double octile(int ax, int ay, int bx, int by) {
    int dx = std::abs(ax - bx);
    int dy = std::abs(ay - by);
    return (dx + dy) + (SQRT2 - 2.0) * std::min(dx, dy);
}
inline double euclid(double ax, double ay, double bx, double by) {
    return std::hypot(ax - bx, ay - by);
}
}  // namespace

GridPlanner::GridPlanner(PlannerConfig cfg)
    : cfg_(cfg),
      grid_dim_(static_cast<int>(cfg.world_size / cfg.grid_size)) {
    cost_.assign(static_cast<std::size_t>(grid_dim_) * grid_dim_, 1.0);
}

std::pair<int, int> GridPlanner::world_to_grid(double x, double y) const {
    int gx = static_cast<int>((x + cfg_.world_size / 2.0) / cfg_.grid_size);
    int gy = static_cast<int>((y + cfg_.world_size / 2.0) / cfg_.grid_size);
    gx = std::max(0, std::min(grid_dim_ - 1, gx));
    gy = std::max(0, std::min(grid_dim_ - 1, gy));
    return {gx, gy};
}

Point GridPlanner::grid_to_world(int gx, int gy) const {
    double x = gx * cfg_.grid_size - cfg_.world_size / 2.0 + cfg_.grid_size / 2.0;
    double y = gy * cfg_.grid_size - cfg_.world_size / 2.0 + cfg_.grid_size / 2.0;
    return {x, y};
}

double GridPlanner::cost_at(int gx, int gy) const {
    if (gx < 0 || gy < 0 || gx >= grid_dim_ || gy >= grid_dim_) return BLOCKED;
    return cost_[cell_idx(gx, gy)];
}

void GridPlanner::paint_obstacle(double x, double y, double radius) {
    auto [gx, gy] = world_to_grid(x, y);
    const double cells = radius / cfg_.grid_size;
    const int r_cells = static_cast<int>(std::ceil(cells)) + 1;
    const double rr = cells * cells;
    for (int dx = -r_cells; dx <= r_cells; ++dx) {
        for (int dy = -r_cells; dy <= r_cells; ++dy) {
            if (dx * dx + dy * dy > rr) continue;
            int nx = gx + dx, ny = gy + dy;
            if (nx >= 0 && nx < grid_dim_ && ny >= 0 && ny < grid_dim_) {
                cost_[cell_idx(nx, ny)] = BLOCKED;
            }
        }
    }
}

void GridPlanner::update_cost_map(TerrainType t, const ElevationFn* sampler) {
    double base;
    switch (t) {
        case TerrainType::Flat:       base = 1.0; break;
        case TerrainType::Slope:      base = 2.0; break;
        case TerrainType::Rough:      base = 3.0; break;
        case TerrainType::Transition: base = 2.5; break;
        default:                      base = 1.5; break;
    }
    std::fill(cost_.begin(), cost_.end(), base);

    if (sampler && *sampler) {
        const double gs = cfg_.grid_size;
        const double ws = cfg_.world_size;
        std::vector<double> elev(static_cast<std::size_t>(grid_dim_) * grid_dim_);
        for (int i = 0; i < grid_dim_; ++i) {
            double x = i * gs - ws / 2.0 + gs / 2.0;
            for (int j = 0; j < grid_dim_; ++j) {
                double y = j * gs - ws / 2.0 + gs / 2.0;
                elev[i * grid_dim_ + j] = (*sampler)(x, y);
            }
        }
        for (int i = 0; i < grid_dim_; ++i) {
            for (int j = 0; j < grid_dim_; ++j) {
                double gx, gy;
                if (i == 0)
                    gx = (elev[(i + 1) * grid_dim_ + j] - elev[i * grid_dim_ + j]) / gs;
                else if (i == grid_dim_ - 1)
                    gx = (elev[i * grid_dim_ + j] - elev[(i - 1) * grid_dim_ + j]) / gs;
                else
                    gx = (elev[(i + 1) * grid_dim_ + j] - elev[(i - 1) * grid_dim_ + j]) / (2.0 * gs);

                if (j == 0)
                    gy = (elev[i * grid_dim_ + (j + 1)] - elev[i * grid_dim_ + j]) / gs;
                else if (j == grid_dim_ - 1)
                    gy = (elev[i * grid_dim_ + j] - elev[i * grid_dim_ + (j - 1)]) / gs;
                else
                    gy = (elev[i * grid_dim_ + (j + 1)] - elev[i * grid_dim_ + (j - 1)]) / (2.0 * gs);

                cost_[cell_idx(i, j)] = base + 10.0 * std::sqrt(gx * gx + gy * gy);
            }
        }
    }

    // 重涂持久障碍
    for (const auto& o : obstacles_) paint_obstacle(o.x, o.y, o.radius);
}

void GridPlanner::add_obstacles(const std::vector<Obstacle>& obs) {
    for (const auto& o : obs) {
        obstacles_.push_back(o);
        paint_obstacle(o.x, o.y, o.radius);
    }
}

void GridPlanner::clear_obstacles() { obstacles_.clear(); }

std::optional<std::pair<int, int>>
GridPlanner::nearest_free_cell(int cx, int cy, int max_ring) const {
    if (cx >= 0 && cx < grid_dim_ && cy >= 0 && cy < grid_dim_
        && cost_[cell_idx(cx, cy)] < BLOCKED) {
        return std::make_pair(cx, cy);
    }
    for (int ring = 1; ring <= max_ring; ++ring) {
        // 上下两条边
        for (int dx = -ring; dx <= ring; ++dx) {
            for (int dy : {-ring, ring}) {
                int nx = cx + dx, ny = cy + dy;
                if (nx >= 0 && nx < grid_dim_ && ny >= 0 && ny < grid_dim_
                    && cost_[cell_idx(nx, ny)] < BLOCKED) {
                    return std::make_pair(nx, ny);
                }
            }
        }
        // 左右两条边
        for (int dy = -ring + 1; dy <= ring - 1; ++dy) {
            for (int dx : {-ring, ring}) {
                int nx = cx + dx, ny = cy + dy;
                if (nx >= 0 && nx < grid_dim_ && ny >= 0 && ny < grid_dim_
                    && cost_[cell_idx(nx, ny)] < BLOCKED) {
                    return std::make_pair(nx, ny);
                }
            }
        }
    }
    return std::nullopt;
}

// Bresenham 风格的格栅 LOS：要求穿过的每个格子都可通行，
// 并禁止对角"贴角"穿墙（两个相邻直角格至少一个可通行）。
bool GridPlanner::line_of_sight_grid(int x0, int y0, int x1, int y1) const {
    int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    int x = x0, y = y0;
    while (true) {
        if (x < 0 || x >= grid_dim_ || y < 0 || y >= grid_dim_) return false;
        if (cost_[cell_idx(x, y)] >= BLOCKED) return false;
        if (x == x1 && y == y1) return true;
        int e2 = 2 * err;
        bool stepped_x = false, stepped_y = false;
        if (e2 > -dy) { err -= dy; x += sx; stepped_x = true; }
        if (e2 <  dx) { err += dx; y += sy; stepped_y = true; }
        // 对角步：检查两个直角邻居至少一个可走，否则视为"贴角穿墙"
        if (stepped_x && stepped_y) {
            int ax = x - sx, ay = y;
            int bx = x, by = y - sy;
            if (ax >= 0 && ax < grid_dim_ && ay >= 0 && ay < grid_dim_
                && bx >= 0 && bx < grid_dim_ && by >= 0 && by < grid_dim_) {
                if (cost_[cell_idx(ax, ay)] >= BLOCKED &&
                    cost_[cell_idx(bx, by)] >= BLOCKED) {
                    return false;
                }
            }
        }
    }
}

std::vector<Point>
GridPlanner::reconstruct_path(const std::vector<int>& parents,
                              int start_id, int goal_id) const {
    std::vector<int> ids;
    int node = goal_id;
    while (node != -1) {
        ids.push_back(node);
        if (node == start_id) break;
        node = parents[node];
    }
    std::reverse(ids.begin(), ids.end());
    std::vector<Point> pts;
    pts.reserve(ids.size());
    for (int id : ids) {
        int gx = id / grid_dim_;
        int gy = id % grid_dim_;
        pts.push_back(grid_to_world(gx, gy));
    }
    return pts;
}

// LOS-aware Douglas-Peucker：相比原版多了"段内是否可通行"检查，
// 避免简化把绕障路径折回直线穿障。
std::vector<Point> GridPlanner::los_simplify(const std::vector<Point>& path) const {
    const int n = static_cast<int>(path.size());
    if (n <= 2) return path;

    std::vector<Point> out;
    out.reserve(n);
    out.push_back(path[0]);
    int anchor = 0;
    for (int i = 2; i < n; ++i) {
        auto [ax, ay] = world_to_grid(path[anchor].first, path[anchor].second);
        auto [ix, iy] = world_to_grid(path[i].first, path[i].second);
        if (!line_of_sight_grid(ax, ay, ix, iy)) {
            out.push_back(path[i - 1]);
            anchor = i - 1;
        }
    }
    out.push_back(path.back());
    return out;
}

std::vector<Point> GridPlanner::plan(Point start, Point goal) {
    auto [sgx, sgy] = world_to_grid(start.first, start.second);
    auto [ggx, ggy] = world_to_grid(goal.first, goal.second);

    // 目标格被堵：找最近可通行格代替
    if (cost_[cell_idx(ggx, ggy)] >= BLOCKED) {
        auto repl = nearest_free_cell(ggx, ggy,
                                      static_cast<int>(3.0 / cfg_.grid_size));
        if (!repl) return {};
        ggx = repl->first;
        ggy = repl->second;
    }
    // 起点格被堵（罕见）：也找一格
    if (cost_[cell_idx(sgx, sgy)] >= BLOCKED) {
        auto repl = nearest_free_cell(sgx, sgy, 4);
        if (!repl) return {};
        sgx = repl->first;
        sgy = repl->second;
    }

    if (sgx == ggx && sgy == ggy) {
        return {start, goal};
    }

    const int N = grid_dim_ * grid_dim_;
    std::vector<double> g(N, INF);
    std::vector<int>    parent(N, -1);
    std::vector<uint8_t> closed(N, 0);

    const int start_id = sgx * grid_dim_ + sgy;
    const int goal_id  = ggx * grid_dim_ + ggy;
    g[start_id] = 0.0;
    parent[start_id] = start_id;  // sentinel：自指 = 起点

    using Item = std::tuple<double, int, int>;  // (f, tie, id)
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
    open.emplace(octile(sgx, sgy, ggx, ggy), 0, start_id);
    int counter = 1;

    const int dx8[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
    const int dy8[8] = { 0, 0,-1, 1, -1,  1,-1, 1};
    const double step8[8] = {1, 1, 1, 1, SQRT2, SQRT2, SQRT2, SQRT2};
    const double w = cfg_.heuristic_weight;
    const bool theta = cfg_.use_theta_star;

    while (!open.empty()) {
        auto [f, _tie, current] = open.top();
        open.pop();
        if (closed[current]) continue;
        if (current == goal_id) break;
        closed[current] = 1;
        const int cx = current / grid_dim_;
        const int cy = current % grid_dim_;

        for (int k = 0; k < 8; ++k) {
            int nx = cx + dx8[k];
            int ny = cy + dy8[k];
            if (nx < 0 || nx >= grid_dim_ || ny < 0 || ny >= grid_dim_) continue;
            // 修复：对角穿墙角
            if (std::abs(dx8[k]) + std::abs(dy8[k]) == 2) {
                if (cost_[cell_idx(cx + dx8[k], cy)] >= BLOCKED &&
                    cost_[cell_idx(cx, cy + dy8[k])] >= BLOCKED) continue;
            }
            const int nid = nx * grid_dim_ + ny;
            if (closed[nid]) continue;
            const double ncost = cost_[nid];
            if (ncost >= BLOCKED) continue;

            // Theta*：尝试用 parent 直连邻居
            double tentative_g = INF;
            int new_parent = current;
            if (theta) {
                const int p = parent[current];
                const int px = p / grid_dim_;
                const int py = p % grid_dim_;
                if (line_of_sight_grid(px, py, nx, ny)) {
                    const double seg = euclid(px, py, nx, ny) * cfg_.grid_size;
                    // 用平均 cost 近似线段穿越代价
                    const double avg_cost = 0.5 * (cost_[cell_idx(px, py)] + ncost);
                    const double cost_units = seg / cfg_.grid_size;  // 步数等价
                    tentative_g = g[p] + cost_units * avg_cost;
                    new_parent = p;
                }
            }
            const double a_star_g = g[current] + step8[k] * ncost;
            if (a_star_g < tentative_g) {
                tentative_g = a_star_g;
                new_parent = current;
            }

            if (tentative_g < g[nid]) {
                g[nid] = tentative_g;
                parent[nid] = new_parent;
                const double fnew = tentative_g + w * octile(nx, ny, ggx, ggy);
                open.emplace(fnew, counter++, nid);
            }
        }
    }

    if (parent[goal_id] == -1) return {};

    // 重建路径（自 goal 沿 parent 回溯到 start）
    std::vector<Point> path;
    int node = goal_id;
    while (true) {
        int gx = node / grid_dim_;
        int gy = node % grid_dim_;
        path.push_back(grid_to_world(gx, gy));
        if (node == start_id) break;
        node = parent[node];
    }
    std::reverse(path.begin(), path.end());

    // LOS-aware 简化：去掉冗余转折，但不能直线穿障
    return los_simplify(path);
}

// 独立工具：给定路径 + 障碍列表，做 LOS-aware 简化。Python 回退用。
std::vector<Point> simplify_path_los(const std::vector<Point>& path,
                                     const std::vector<Obstacle>& obstacles,
                                     double tolerance) {
    (void)tolerance;  // 当前实现：用障碍 LOS 检查替代了纯几何 tolerance
    const int n = static_cast<int>(path.size());
    if (n <= 2) return path;

    auto seg_clear = [&](const Point& a, const Point& b) {
        for (const auto& o : obstacles) {
            // 点到线段距离
            const double vx = b.first - a.first;
            const double vy = b.second - a.second;
            const double wx = o.x - a.first;
            const double wy = o.y - a.second;
            const double c1 = vx * wx + vy * wy;
            if (c1 <= 0.0) {
                if (std::hypot(o.x - a.first, o.y - a.second) < o.radius) return false;
                continue;
            }
            const double c2 = vx * vx + vy * vy;
            if (c2 <= c1) {
                if (std::hypot(o.x - b.first, o.y - b.second) < o.radius) return false;
                continue;
            }
            const double t = c1 / c2;
            const double px = a.first + t * vx;
            const double py = a.second + t * vy;
            if (std::hypot(o.x - px, o.y - py) < o.radius) return false;
        }
        return true;
    };

    std::vector<Point> out;
    out.reserve(n);
    out.push_back(path[0]);
    int anchor = 0;
    for (int i = 2; i < n; ++i) {
        if (!seg_clear(path[anchor], path[i])) {
            out.push_back(path[i - 1]);
            anchor = i - 1;
        }
    }
    out.push_back(path.back());
    return out;
}

}  // namespace navcore
