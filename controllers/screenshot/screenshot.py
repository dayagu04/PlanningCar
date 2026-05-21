"""Minimal Supervisor controller that captures a viewport screenshot and exits.

Usage: set this as the controller in a .wbt, run Webots with --mode=realtime
for a few steps to let the scene render, then this controller exports the
viewport image and calls simulationQuit().

Environment variables:
  SCREENSHOT_OUTPUT — path to save the PNG (default: screenshot.png in CWD)
  SCREENSHOT_STEPS — number of timesteps to wait before capture (default: 30)
"""

import os
import sys
from controller import Supervisor

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

output_path = os.environ.get("SCREENSHOT_OUTPUT", "screenshot.png")
wait_steps = int(os.environ.get("SCREENSHOT_STEPS", "30"))

for _ in range(wait_steps):
    robot.step(timestep)

robot.exportImage(output_path, 100)
print(f"[screenshot] saved: {output_path}")
robot.simulationQuit(0)
