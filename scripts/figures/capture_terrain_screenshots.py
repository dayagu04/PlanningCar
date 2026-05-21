"""Capture Figure 3-2: four terrain screenshots via Webots Supervisor.

For each terrain world, patches the controller to 'screenshot', launches
Webots in realtime mode (rendering on), waits for it to export the image
and quit, then assembles the four PNGs into 图3-2_四种地形仿真场景.png.
"""

import os
import re
import subprocess
import sys
import time

import matplotlib.pyplot as plt
import matplotlib.image as mpimg

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
WORLDS_DIR = os.path.join(PROJECT_ROOT, "worlds")
FIGURES_DIR = os.path.join(PROJECT_ROOT, "results", "figures", "thesis")
WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
SCREENSHOT_STEPS = "60"   # ~2 s at 32 ms/step — enough for scene to settle

os.makedirs(FIGURES_DIR, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 300
plt.rcParams["savefig.dpi"] = 300

TERRAINS = [
    ("flat_terrain.wbt",       "(a) 平坦地形 (Flat Terrain)"),
    ("slope_terrain.wbt",      "(b) 斜坡地形 (Slope Terrain)"),
    ("rough_terrain.wbt",      "(c) 凹凸地形 (Rough Terrain)"),
    ("transition_terrain.wbt", "(d) 过渡区地形 (Transition Terrain)"),
]


def patch_and_run(world_file, out_png):
    src = os.path.join(WORLDS_DIR, world_file)
    with open(src, "r", encoding="utf-8") as f:
        content = f.read()
    patched = re.sub(r'controller "[^"]*"', 'controller "screenshot"', content)
    tmp = src.replace(".wbt", "_screenshot_tmp.wbt")
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(patched)

    env = os.environ.copy()
    env["WEBOTS_HOME"] = r"C:\Program Files\Webots"
    env["SCREENSHOT_OUTPUT"] = out_png
    env["SCREENSHOT_STEPS"] = SCREENSHOT_STEPS

    cmd = [WEBOTS_EXE, "--mode=realtime", "--stdout", "--stderr", tmp]
    print(f"  launching Webots for {world_file} ...")
    proc = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
    try:
        proc.wait(timeout=60)
    except subprocess.TimeoutExpired:
        print(f"  timeout — killing")
        proc.kill()
        proc.wait(timeout=5)
    finally:
        if os.path.exists(tmp):
            os.remove(tmp)

    if os.path.exists(out_png):
        print(f"  [OK] {out_png}")
        return True
    print(f"  [FAIL] no output for {world_file}")
    return False


def assemble(shots):
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    for ax, (img_path, title) in zip(axes.flatten(), shots):
        if img_path and os.path.exists(img_path):
            img = mpimg.imread(img_path)
            ax.imshow(img)
        else:
            ax.text(0.5, 0.5, "(截图失败)", ha="center", va="center",
                    transform=ax.transAxes, fontsize=12)
        ax.set_title(title, fontsize=11, fontweight="bold")
        ax.axis("off")

    plt.tight_layout()
    save_path = os.path.join(FIGURES_DIR, "图3-2b_四种地形仿真场景_Webots截图.png")
    plt.savefig(save_path, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"[OK] {save_path}")


def main():
    if not os.path.exists(WEBOTS_EXE):
        print(f"FATAL: Webots not found at {WEBOTS_EXE}")
        sys.exit(1)

    shots = []
    for world_file, title in TERRAINS:
        out_png = os.path.join(FIGURES_DIR, f"_tmp_{world_file.replace('.wbt', '.png')}")
        ok = patch_and_run(world_file, out_png)
        shots.append((out_png if ok else None, title))

    assemble(shots)

    for _, title in TERRAINS:
        tmp = os.path.join(FIGURES_DIR, f"_tmp_{title}")
    for world_file, _ in TERRAINS:
        tmp = os.path.join(FIGURES_DIR, f"_tmp_{world_file.replace('.wbt', '.png')}")
        if os.path.exists(tmp):
            os.remove(tmp)


if __name__ == "__main__":
    main()
