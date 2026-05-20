# Iteration 01: Classifier — Robust Rough Terrain via IMU Vote Window

## 元信息
- **分支**: `iter/01-classifier-rough-imu-features`
- **起始 commit**: `292529e` (merge iter00: baseline establishment)
- **基线**: iter-00-baseline
- **日期**: 2026-05-20
- **状态**: IN_PROGRESS

## Hypothesis
**瓶颈**: rough 地形 classification_accuracy 仅 78.5%（全系统最弱），transition 地形也仅 85.1%。

**原因分析**:
- IMU 在崎岖地形下 roll/pitch 信号噪声大，单帧分类易误判 ROUGH ↔ FLAT
- 当前 5 帧投票窗口不足以平滑 rough 地形上的高频抖动
- transition 地形天然有跨类切换，5 帧窗口对边界处响应过快

**改动假设**: 将 `TerrainClassifier` 默认 `vote_window` 从 5 扩展到 9 帧（约 288ms 平滑窗口），同时改进 tie-break 策略：
- 当 ROUGH 与 FLAT 票数接近时，偏向最近 3 帧的多数票（短期记忆）
- 这样在持续 rough 状态下不会被偶发 FLAT 帧污染

**预期收益**:
- rough classification_acc: 78.5% → **≥ 88%** (+9.5pp)
- transition classification_acc: 85.1% → **≥ 90%** (+4.9pp)
- success_rate / path_efficiency 不回退（首要指标硬约束）

## 计划改动
1. `src/classification/rule_classifier.py` - 默认 vote_window 5 → 9，加入短期偏向 tie-break
2. `tests/test_rule_classifier.py` - 新增噪声鲁棒性测试（注入随机噪声序列，验证最终输出稳定）
3. 不动 C++ 内核（无需重编）
4. 不改控制器（继续用默认参数）

## 实验设计
完全复用 iter 00 的实验脚本：
```
python scripts/experiments/run_baseline_experiments_v2.py --iter-tag iter01
python scripts/analysis/analyze_baseline.py --iter-tag iter01 --iter-num 1
```

## 实验结果

### 纵向对比（vs iter-00-baseline）
*待实验完成后填充*

### 横向对比（adaptive vs baseline vs astar）
*待实验完成后填充*

### 关键观察
*待实验完成后填充*

## pytest 结果
*待测试完成后填充*

## 决策
*待数据齐全后填写*

---
**实验数据**: `data/experiments/iter01/`  
**指标快照**: `results/reports/metrics/iter_01.json`
