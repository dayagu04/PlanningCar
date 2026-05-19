# 不规则地面机器人自适应导航系统

> 本科毕业设计 — 王志伟 (22205050124)
> 基于 Webots 仿真平台的不规则地面自适应导航研究

## 项目概述

针对工地巡检、矿山勘探、灾害救援等户外作业场景中移动机器人在不规则地面环境下导航存在的问题，本课题基于 Webots 仿真平台实现「感知—识别—导航—验证」全流程仿真。

## 技术方案

- **平台**: Windows 10 + Webots (>=2023b) + Python 3.9
- **机器人**: 四轮差速移动机器人 (基于 Webots 内置模型)
- **传感器**: 激光雷达 + IMU + GPS + 罗盘 + 深度相机
- **地形类型**: 平坦 / 斜坡 / 凹凸 / 过渡区
- **核心算法**: 地形特征提取 → 规则分类识别 → 自适应路径规划

## 目录结构

```
.
├── worlds/                   # Webots 世界文件 (.wbt)
├── controllers/              # Webots 控制器
│   ├── adaptive_navigator/   # 自适应导航控制器
│   └── astar_navigator/      # A* 路径规划控制器
├── protos/                   # 自定义 PROTO 节点
├── src/                      # 核心 Python 算法库
│   ├── perception/           # 地形特征提取
│   ├── classification/       # 地形分类
│   ├── planning/             # 路径规划 (A* / TSP)
│   ├── control/              # 自适应控制
│   └── utils/                # 工具函数
├── cpp/                      # C++ 高性能模块 (pybind11)
│   ├── include/              # 头文件
│   └── src/                  # 实现 + Python 绑定
├── scripts/
│   ├── figures/              # 论文图表 & 地形/世界生成
│   ├── experiments/          # 实验运行 & 数据分析
│   └── tools/                # 构建、启动、参数调优
├── tests/                    # 单元测试
├── data/                     # 实验数据 (logs / experiments)
├── results/                  # 图表与报告
├── docs/                     # 文档 (论文大纲、任务书、开题报告)
├── config.yaml               # 全局配置
└── requirements.txt          # Python 依赖
```

## 快速开始

> 仓库未包含 `.venv/`（体积大、平台相关），首次拉取后需在本地创建。

### 1. 创建虚拟环境并安装依赖
```powershell
# 在仓库根目录执行（要求 Python 3.9）
py -3.9 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
pip install -r requirements.txt
```

### 2. 激活虚拟环境（后续使用）
```powershell
.\.venv\Scripts\Activate.ps1
```

### 3. 运行单元测试
```powershell
.\.venv\Scripts\python.exe -m pytest tests/ -v
```

### 4. 安装 Webots
- 官网下载: https://cyberbotics.com/
- 安装后将 `<Webots>\lib\controller\python` 添加到 `PYTHONPATH`

### 5. 启动仿真
```powershell
webots worlds\flat_terrain.wbt
```

## 研究进度

| 阶段 | 内容 | 状态 |
|------|------|------|
| 第1-2周  | 文献综述、环境调研               | 完成 |
| 第3周    | Webots + Python 环境搭建         | 进行中 |
| 第4-6周  | 四种地形场景搭建、特征提取算法   | 待开始 |
| 第7-8周  | 分类识别、自适应导航联动         | 待开始 |
| 第9-10周 | 多算法对比实验                   | 待开始 |
| 第11-13周 | 性能验证、报告撰写              | 待开始 |
| 第14-16周 | 论文撰写与提交                  | 待开始 |

## 评价指标

- 平均运动速度
- 姿态稳定性 (IMU 翻滚/俯仰角波动)
- 路径跟踪误差
- 任务完成率
