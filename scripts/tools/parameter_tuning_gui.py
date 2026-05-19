"""Real-time parameter tuning GUI for navigation system.

Provides sliders to adjust classifier thresholds and control parameters in real-time.
Updates config.yaml which can be reloaded by controllers.
"""

import os
import sys
import tkinter as tk
from tkinter import ttk, messagebox
import yaml

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.utils.config import Config, get_config


class ParameterTuningGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("导航系统参数调整 - Navigation Parameter Tuning")
        self.root.geometry("800x700")
        self.root.resizable(True, True)

        self.config = get_config()
        self.param_vars = {}

        self.create_widgets()
        self.load_current_values()

    def create_widgets(self):
        # Title
        title = tk.Label(
            self.root,
            text="实时参数调整面板\nReal-time Parameter Tuning Panel",
            font=("Arial", 14, "bold"),
            pady=10,
        )
        title.pack()

        # Notebook for tabs
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=10, pady=5)

        # Tab 1: Classifier Thresholds
        classifier_frame = ttk.Frame(notebook)
        notebook.add(classifier_frame, text="地形分类器 (Classifier)")
        self.create_classifier_tab(classifier_frame)

        # Tab 2: Control Parameters
        control_frame = ttk.Frame(notebook)
        notebook.add(control_frame, text="控制参数 (Control)")
        self.create_control_tab(control_frame)

        # Tab 3: Navigation Settings
        nav_frame = ttk.Frame(notebook)
        notebook.add(nav_frame, text="导航设置 (Navigation)")
        self.create_navigation_tab(nav_frame)

        # Buttons
        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=10)

        tk.Button(
            button_frame,
            text="保存配置 (Save Config)",
            command=self.save_config,
            bg="#4CAF50",
            fg="white",
            font=("Arial", 10, "bold"),
            padx=20,
            pady=5,
        ).pack(side="left", padx=5)

        tk.Button(
            button_frame,
            text="重置默认 (Reset Defaults)",
            command=self.reset_defaults,
            bg="#FF9800",
            fg="white",
            font=("Arial", 10, "bold"),
            padx=20,
            pady=5,
        ).pack(side="left", padx=5)

        tk.Button(
            button_frame,
            text="退出 (Exit)",
            command=self.root.quit,
            bg="#F44336",
            fg="white",
            font=("Arial", 10, "bold"),
            padx=20,
            pady=5,
        ).pack(side="left", padx=5)

    def create_classifier_tab(self, parent):
        canvas = tk.Canvas(parent)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        params = [
            ("classifier.flat_slope_max", "平坦地形最大坡度 (deg)", 0, 10, 0.5),
            ("classifier.flat_roughness_max", "平坦地形最大粗糙度", 0, 0.1, 0.005),
            ("classifier.flat_imu_pitch_max", "平坦地形最大 Pitch (deg)", 0, 10, 0.5),
            ("classifier.slope_imu_pitch_min", "斜坡最小 Pitch (deg)", 0, 15, 0.5),
            ("classifier.rough_roughness_min", "凹凸地形最小粗糙度", 0, 0.2, 0.005),
        ]

        for i, (key, label, min_val, max_val, resolution) in enumerate(params):
            self.create_slider(scrollable_frame, key, label, min_val, max_val, resolution, row=i)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def create_control_tab(self, parent):
        canvas = tk.Canvas(parent)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        terrains = ["flat", "slope", "rough", "transition"]
        terrain_labels = {
            "flat": "平坦 (Flat)",
            "slope": "斜坡 (Slope)",
            "rough": "凹凸 (Rough)",
            "transition": "过渡 (Transition)",
        }

        row = 0
        for terrain in terrains:
            # Section header
            tk.Label(
                scrollable_frame,
                text=f"\n{terrain_labels[terrain]}",
                font=("Arial", 11, "bold"),
            ).grid(row=row, column=0, columnspan=3, sticky="w", padx=10, pady=5)
            row += 1

            # Max speed
            self.create_slider(
                scrollable_frame,
                f"control.{terrain}.max_speed",
                "最大速度 (rad/s)",
                0.5, 8.0, 0.1,
                row=row
            )
            row += 1

            # Turn gain
            self.create_slider(
                scrollable_frame,
                f"control.{terrain}.turn_gain",
                "转向增益",
                0.5, 6.0, 0.1,
                row=row
            )
            row += 1

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def create_navigation_tab(self, parent):
        frame = ttk.Frame(parent, padding=20)
        frame.pack(fill="both", expand=True)

        tk.Label(
            frame,
            text="导航参数 (Navigation Parameters)",
            font=("Arial", 12, "bold"),
        ).pack(pady=10)

        self.create_slider(
            frame,
            "navigation.distance_tolerance",
            "航点到达容差 (m)",
            0.1, 2.0, 0.1,
            row=0
        )

        tk.Label(
            frame,
            text="\n航点坐标 (Waypoints)\n(需手动编辑 config.yaml)",
            font=("Arial", 10),
            fg="gray",
        ).pack(pady=20)

    def create_slider(self, parent, key, label, min_val, max_val, resolution, row):
        frame = tk.Frame(parent)
        frame.grid(row=row, column=0, columnspan=3, sticky="ew", padx=10, pady=5)

        tk.Label(frame, text=label, width=30, anchor="w").pack(side="left")

        var = tk.DoubleVar()
        self.param_vars[key] = var

        slider = tk.Scale(
            frame,
            from_=min_val,
            to=max_val,
            resolution=resolution,
            orient="horizontal",
            variable=var,
            length=300,
        )
        slider.pack(side="left", padx=10)

        value_label = tk.Label(frame, text="", width=8, anchor="e")
        value_label.pack(side="left")

        def update_label(*args):
            value_label.config(text=f"{var.get():.3f}")

        var.trace("w", update_label)

    def load_current_values(self):
        """Load current config values into sliders."""
        for key, var in self.param_vars.items():
            value = self.config.get(key)
            if value is not None:
                var.set(value)

    def save_config(self):
        """Save current slider values to config.yaml."""
        for key, var in self.param_vars.items():
            keys = key.split(".")
            d = self.config.data
            for k in keys[:-1]:
                if k not in d:
                    d[k] = {}
                d = d[k]
            d[keys[-1]] = var.get()

        try:
            self.config.save()
            messagebox.showinfo(
                "保存成功",
                "配置已保存到 config.yaml\n重启控制器以应用新参数。"
            )
        except Exception as e:
            messagebox.showerror("保存失败", f"错误: {e}")

    def reset_defaults(self):
        """Reset all parameters to default values."""
        if messagebox.askyesno("确认重置", "确定要重置所有参数为默认值吗？"):
            self.config = Config()  # Reload defaults
            self.load_current_values()
            messagebox.showinfo("重置完成", "所有参数已重置为默认值。")


def main():
    root = tk.Tk()
    app = ParameterTuningGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
