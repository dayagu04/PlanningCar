// bindings.cpp — pybind11 绑定
#include "nav_core.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>

namespace py = pybind11;
using namespace navcore;

namespace {

// 把 numpy 数组转 (rows, cols, data*) 用于 extract_features
TerrainFeatures extract_features_np(py::array_t<double, py::array::c_style | py::array::forcecast> arr,
                                    double cell_size) {
    if (arr.ndim() != 2) {
        throw std::invalid_argument("heights must be 2-D array");
    }
    return extract_features(arr.data(),
                            static_cast<int>(arr.shape(0)),
                            static_cast<int>(arr.shape(1)),
                            cell_size);
}

// 从 list/tuple 转 vector<Obstacle>
std::vector<Obstacle> to_obstacles(const py::iterable& src) {
    std::vector<Obstacle> out;
    for (auto item : src) {
        if (py::isinstance<py::tuple>(item) || py::isinstance<py::list>(item)) {
            auto t = item.cast<py::sequence>();
            if (py::len(t) < 3) throw std::invalid_argument("obstacle needs (x, y, r)");
            out.push_back({t[0].cast<double>(), t[1].cast<double>(), t[2].cast<double>()});
        } else if (py::isinstance<py::dict>(item)) {
            auto d = item.cast<py::dict>();
            out.push_back({d["x"].cast<double>(), d["y"].cast<double>(), d["r"].cast<double>()});
        } else {
            // 假定有 .x .y .r 属性
            out.push_back({item.attr("x").cast<double>(),
                           item.attr("y").cast<double>(),
                           item.attr("r").cast<double>()});
        }
    }
    return out;
}

}  // namespace

PYBIND11_MODULE(nav_core_cpp, m) {
    m.doc() = "Navigation core (C++): A*/Theta*, TSP, terrain features & classifier";

    // ============= HeightField =============
    py::class_<HeightField>(m, "HeightField")
        .def(py::init<>())
        .def(py::init<std::vector<double>, int, double, double>(),
             py::arg("heights"), py::arg("resolution"),
             py::arg("size"), py::arg("origin_offset"))
        .def("sample", &HeightField::sample, py::arg("x"), py::arg("y"))
        .def("empty", &HeightField::empty)
        .def_property_readonly("resolution", &HeightField::resolution);

    // ============= Terrain features =============
    py::class_<TerrainFeatures>(m, "TerrainFeatures")
        .def_readwrite("slope_deg", &TerrainFeatures::slope_deg)
        .def_readwrite("roughness", &TerrainFeatures::roughness)
        .def_readwrite("height_range", &TerrainFeatures::height_range)
        .def_readwrite("mean_height", &TerrainFeatures::mean_height)
        .def("__repr__", [](const TerrainFeatures& f) {
            return "TerrainFeatures(slope_deg=" + std::to_string(f.slope_deg) +
                   ", roughness=" + std::to_string(f.roughness) +
                   ", height_range=" + std::to_string(f.height_range) +
                   ", mean_height=" + std::to_string(f.mean_height) + ")";
        });

    m.def("extract_features", &extract_features_np,
          py::arg("heights"), py::arg("cell_size") = 0.1,
          "Compute slope_deg / roughness / height_range / mean_height from a 2-D height grid");

    // ============= Terrain classifier =============
    py::enum_<TerrainType>(m, "TerrainType")
        .value("FLAT", TerrainType::Flat)
        .value("SLOPE", TerrainType::Slope)
        .value("ROUGH", TerrainType::Rough)
        .value("TRANSITION", TerrainType::Transition)
        .export_values();

    m.def("terrain_name", &terrain_name, py::arg("t"));

    py::class_<ClassifierThresholds>(m, "ClassifierThresholds")
        .def(py::init<>())
        .def_readwrite("flat_slope_max", &ClassifierThresholds::flat_slope_max)
        .def_readwrite("flat_roughness_max", &ClassifierThresholds::flat_roughness_max)
        .def_readwrite("flat_imu_pitch_max", &ClassifierThresholds::flat_imu_pitch_max)
        .def_readwrite("slope_angle_min", &ClassifierThresholds::slope_angle_min)
        .def_readwrite("slope_imu_pitch_min", &ClassifierThresholds::slope_imu_pitch_min)
        .def_readwrite("slope_roughness_max", &ClassifierThresholds::slope_roughness_max)
        .def_readwrite("rough_roughness_min", &ClassifierThresholds::rough_roughness_min)
        .def_readwrite("rough_imu_roll_min", &ClassifierThresholds::rough_imu_roll_min);

    py::class_<TerrainClassifier>(m, "TerrainClassifier")
        .def(py::init<ClassifierThresholds, int>(),
             py::arg("thresholds") = ClassifierThresholds{},
             py::arg("history_len") = 5)
        .def("classify", &TerrainClassifier::classify,
             py::arg("slope_deg"), py::arg("roughness"),
             py::arg("imu_pitch_deg"), py::arg("imu_roll_deg"))
        .def("reset", &TerrainClassifier::reset);

    // ============= TSP =============
    py::class_<TSPInfo>(m, "TSPInfo")
        .def_readwrite("original_length", &TSPInfo::original_length)
        .def_readwrite("greedy_length", &TSPInfo::greedy_length)
        .def_readwrite("optimized_length", &TSPInfo::optimized_length)
        .def_readwrite("improvement_pct", &TSPInfo::improvement_pct);

    m.def("optimize_waypoint_order",
          [](Point start, std::vector<Point> waypoints) {
              auto [tour, info] = optimize_waypoint_order(start, std::move(waypoints));
              py::dict d;
              d["original_length"] = info.original_length;
              d["greedy_length"] = info.greedy_length;
              d["optimized_length"] = info.optimized_length;
              d["improvement_pct"] = info.improvement_pct;
              return py::make_tuple(tour, d);
          },
          py::arg("start"), py::arg("waypoints"),
          "Greedy NN + 2-opt waypoint ordering. Returns (tour, info_dict).");

    // ============= Planner =============
    py::class_<PlannerConfig>(m, "PlannerConfig")
        .def(py::init<>())
        .def_readwrite("grid_size", &PlannerConfig::grid_size)
        .def_readwrite("world_size", &PlannerConfig::world_size)
        .def_readwrite("heuristic_weight", &PlannerConfig::heuristic_weight)
        .def_readwrite("use_theta_star", &PlannerConfig::use_theta_star);

    py::class_<Obstacle>(m, "Obstacle")
        .def(py::init<double, double, double>(),
             py::arg("x"), py::arg("y"), py::arg("radius"))
        .def_readwrite("x", &Obstacle::x)
        .def_readwrite("y", &Obstacle::y)
        .def_readwrite("radius", &Obstacle::radius);

    py::class_<GridPlanner>(m, "GridPlanner")
        .def(py::init<PlannerConfig>(), py::arg("config"))
        .def("update_cost_map",
             [](GridPlanner& self, TerrainType t, py::object sampler) {
                 if (sampler.is_none()) {
                     self.update_cost_map(t, nullptr);
                 } else {
                     ElevationFn fn = sampler.cast<ElevationFn>();
                     self.update_cost_map(t, &fn);
                 }
             },
             py::arg("terrain_type"), py::arg("elevation_sampler") = py::none())
        .def("add_obstacles",
             [](GridPlanner& self, py::iterable obs) {
                 self.add_obstacles(to_obstacles(obs));
             },
             py::arg("obstacles"))
        .def("clear_obstacles", &GridPlanner::clear_obstacles)
        .def("plan", &GridPlanner::plan,
             py::arg("start"), py::arg("goal"),
             "Plan a path from start to goal; returns list of (x, y) or empty list")
        .def_property_readonly("grid_dim", &GridPlanner::grid_dim)
        .def("cost_at", &GridPlanner::cost_at, py::arg("gx"), py::arg("gy"));

    // ============= 工具：LOS 简化（独立，给 Python 回退用） =============
    m.def("simplify_path_los",
          [](std::vector<Point> path, py::iterable obs, double tol) {
              return simplify_path_los(path, to_obstacles(obs), tol);
          },
          py::arg("path"), py::arg("obstacles"), py::arg("tolerance") = 0.3);

    m.attr("__version__") = "1.0.0";
}
