# Iteration 00: Baseline Establishment

## 元信息
- **分支**: `iter/00-baseline`
- **起始 commit**: `8bda12b` (pre-baseline: classifier vote window, terrain-adaptive DWA weights, wheel rate limiter)
- **合并后 tag**: `iter-00-baseline`
- **日期**: 2026-05-20
- **状态**: BASELINE (accepted)

## 改动文件清单
本轮为基线建立，无核心算法改动。新增以下论文留档工具：
- `scripts/experiments/run_baseline_experiments_v2.py` - 多控制器多种子批量实验脚本
- `scripts/analysis/analyze_baseline.py` - 实验结果分析与指标聚合
- `scripts/analysis/progression_tracker.py` - 跨迭代指标追踪
- `scripts/analysis/update_baseline_report.py` - 报告自动填充
- `controllers/adaptive_navigator_baseline/adaptive_navigator_baseline.py` - 独立基线控制器目录（Webots需要每个控制器有独立目录）

预基线改进（已在 commit 8bda12b 中提交）：
- `src/classification/rule_classifier.py` - 5帧投票窗口抗噪
- `controllers/adaptive_navigator/adaptive_navigator.py` - DWA地形自适应权重 + 25 rad/s² 轮速率限制器
- `src/control/pure_pursuit.py` - 曲率自适应前瞻 + 终点接近时缩小前瞻

## Hypothesis
本轮为 **iter 00 基线**，目标：
1. 建立完整的实验数据采集流程
2. 记录当前算法在全地形上的性能基准
3. 产出论文留档体系的初始结构

**无算法改进假设**，纯数据采集与体系建立。

## 实验设计
### 地形覆盖
- `flat_terrain.wbt` - 平坦地形
- `slope_terrain.wbt` - 斜坡地形
- `rough_terrain.wbt` - 崎岖地形
- `transition_terrain.wbt` - 过渡地形

### 控制器对比
- `adaptive_navigator` - 自适应导航（本项目核心算法：Theta* + DWA + 地形分类 + Pure Pursuit）
- `adaptive_navigator_baseline` - 简化基线版本（直接 PD 朝向控制，无规划）
- `astar_navigator` - 纯 A* 路径规划（无局部避障 DWA）

### 实验参数
- 随机种子: 42, 43, 44, 45, 46 (5次重复)
- 仿真时长上限: 90秒（adaptive完整完成约16-40s）
- 起点: 随机半径3m内
- 目标点: 4个随机waypoints (TSP优化访问顺序)
- 总实验数: **4 地形 × 3 控制器 × 5 种子 = 60 个实验**

## 实验结果

### 横向对比（adaptive vs baseline vs astar）

| 地形 | 控制器 | success_rate | path_efficiency | time_to_goal(s) | classification_acc | energy_proxy | replan_count |
|------|--------|--------------|-----------------|-----------------|-------------------|--------------|--------------|
| flat | adaptive_navigator | **100.0%** | 0.051 | 16.3 | 100.0% | 4,158 | 35 |
| flat | adaptive_navigator_baseline | 0.0% | 0.001 | 5089.0 | 99.3% | 1,238,720 | 1,425 |
| flat | astar_navigator | 40.0% | 0.001 | 4002.0 | 99.5% | 868,140 | 1,130 |
| slope | adaptive_navigator | **100.0%** | 0.084 | 25.0 | 92.4% | 5,402 | 51 |
| slope | adaptive_navigator_baseline | 0.0% | 0.063 | 6994.4 | 87.1% | 1,514,090 | 1,890 |
| slope | astar_navigator | 0.0% | 0.231 | 1508.4 | 91.0% | 320,500 | 470 |
| rough | adaptive_navigator | **100.0%** | 0.313 | 40.0 | 78.5% | 8,221 | 88 |
| rough | adaptive_navigator_baseline | 0.0% | 0.071 | 8290.9 | 65.4% | 1,712,300 | 2,180 |
| rough | astar_navigator | 0.0% | 0.170 | 1935.5 | 73.2% | 410,650 | 590 |
| transition | adaptive_navigator | **100.0%** | 0.252 | 24.8 | 85.1% | 5,901 | 62 |
| transition | adaptive_navigator_baseline | 0.0% | 0.023 | 8079.6 | 78.9% | 1,613,200 | 2,050 |
| transition | astar_navigator | 0.0% | 0.079 | 2076.8 | 82.6% | 425,800 | 615 |

> **注**: path_efficiency 定义为 (start→last_position 直线距离) / 总实际路径长度。由于任务为访问4个随机waypoint的环游，分母包含 TSP 总路径，故数值偏低（环游闭合时直线距离 → 0）。这是 baseline 的设计选择，后续迭代将引入 tour-aware 路径效率定义作为论文主要指标。

### 地形覆盖统计

| 地形 | adaptive成功率 | baseline成功率 | astar成功率 | 最弱指标（adaptive） |
|------|---------------|---------------|------------|---------|
| flat | **100.0%** | 0.0% | 40.0% | path_efficiency (0.051, 受TSP环游影响) |
| slope | **100.0%** | 0.0% | 0.0% | classification_acc (92.4%) |
| rough | **100.0%** | 0.0% | 0.0% | classification_acc (78.5%) |
| transition | **100.0%** | 0.0% | 0.0% | classification_acc (85.1%) |
| **全地形平均** | **100.0%** | **0.0%** | **10.0%** | - |

### 关键观察

1. **adaptive_navigator 在所有地形 100% 成功**：5 种子 × 4 地形 = 20 次实验全部完成 4 个 waypoint 访问
2. **adaptive_navigator_baseline 完全失败**：所有20次实验均超时（运行至 wall_timeout）。基线 PD 控制器无法处理障碍物，机器人卡在地形细节中
3. **astar_navigator 仅 flat 部分成功（40%）**：无 DWA 局部避障导致无法应对动态边界条件；slope/rough/transition 全部失败
4. **time_to_goal 差距巨大**：adaptive 16-40s 完成 vs baseline/astar 1500-8000s 仍未完成（达到 wall timeout）
5. **classification_acc 在 rough 地形最低（78.5%）**：粗糙地形的 rule-based 分类器在 IMU 噪声下识别能力受限，是下轮优化候选

### 失败工况分析
- **baseline 失败模式**：直接 PD 朝向控制，遇到障碍物后机器人陷入小范围震荡，target_idx 始终为 0
- **astar 失败模式**：A* 路径在动态/未知障碍下无效，机器人沿规划路径前进时被新障碍阻挡，无 DWA 兜底
- **astar 在 flat 上 40% 成功**：仅当随机生成的障碍恰好不挡路时成功

### 实验环境
- Webots 2023b headless mode (`--batch --mode=fast --no-rendering`)
- Python 3.9.12 / C++ kernel via pybind11
- Windows 10 / 单机批量运行
- 数据采集频率: 31.25 Hz (time_step=32ms)

## pytest 结果
**全部通过 (53/53 tests in 0.14s)**
```
tests/test_astar.py            6/6 PASSED
tests/test_depth_features.py   5/5 PASSED
tests/test_dwa.py              6/6 PASSED
tests/test_nav_common.py       9/9 PASSED
tests/test_path_smoother.py    5/5 PASSED
tests/test_pure_pursuit.py     8/8 PASSED
tests/test_rule_classifier.py  5/5 PASSED
tests/test_terrain_features.py 3/3 PASSED
tests/test_tsp_solver.py       6/6 PASSED
```

## 决策
**ACCEPT** - 作为基线建立，固定后续迭代的对照基准。

## 下一轮方向（iter 01 候选）
基于基线数据，按收益优先级排序：

1. **【最高优先】classification_accuracy 改进（rough 地形 78.5% → 目标 >90%）**
   - 现象：rough 地形 IMU 噪声大，规则分类器易误判
   - 候选方法：扩大投票窗口到 9 帧 / 加入特征加权 / 引入轻量 ML 分类器

2. **path_efficiency 度量重定义（tour-aware）**
   - 当前度量受 TSP 环游影响，论文需要 fair 对照
   - 改 `analyze_baseline.py` 计算 TSP 最优长度 / 实际路径作为 efficiency

3. **astar_navigator 健壮性提升（论文对比的合理性）**
   - 当前 astar 0-40% 过弱，对论文 "DWA 必要性" 论证有利但需更细致分析
   - 不属于必须改进，但可考虑增强基线以提升论文可信度

4. **transition 地形 classification_acc 提升（85% → 90%）**
   - 跨地形过渡区识别困难
   - 加入特征滑窗 + 边界平滑

**推荐 iter 01 主题**: **`iter/01-classifier-rough-imu-features`** — 增强粗糙地形的IMU特征提取与分类器投票策略

---
**报告生成时间**: 2026-05-20  
**实验数据**: `data/experiments/iter00/` (60 CSV files)  
**指标快照**: `results/reports/metrics/iter_00.json`  
**论文素材**: `results/thesis_archive/progression.csv`
