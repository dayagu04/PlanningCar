"""Webots 主导航控制器入口（占位 — 待 Webots 安装后实现）。

本文件应在 Webots 中作为 controller 启动。运行时 Webots 会注入 controller 模块。
开发期可通过设置 PYTHONPATH 指向 <Webots>/lib/controller/python 来获得类型提示。

接口约定：
    - 读取激光雷达 / IMU / GPS / 罗盘
    - 调用 src.perception.terrain_features 提取特征
    - 调用 src.classification.rule_classifier 识别地形
    - 调用 src.control.adaptive_params 获取动作参数
    - 控制四轮电机
"""

import sys
import os

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)


def main():
    try:
        from controller import Robot  # type: ignore
    except ImportError:
        print("[adaptive_navigator] Webots controller module not found.")
        print("Run this script from inside Webots, or set PYTHONPATH to "
              "<Webots>/lib/controller/python.")
        return

    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    while robot.step(timestep) != -1:
        pass


if __name__ == "__main__":
    main()
