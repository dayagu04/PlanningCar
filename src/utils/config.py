"""Configuration loader for navigation system."""

import os
import yaml
from typing import Dict, Any

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEFAULT_CONFIG_PATH = os.path.join(PROJECT_ROOT, "config.yaml")


class Config:
    def __init__(self, config_path: str = None):
        path = config_path or DEFAULT_CONFIG_PATH
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                self.data = yaml.safe_load(f)
        else:
            self.data = self._default_config()

    def _default_config(self) -> Dict[str, Any]:
        return {
            "simulation": {
                "time_step_ms": 32,
                "max_duration_s": 300,
            },
            "robot": {
                "wheel_radius": 0.06,
                "wheelbase": 0.4,
            },
            "sensors": {
                "lidar": {"enabled": True, "max_range": 10.0},
                "rangefinder": {"enabled": False, "width": 64, "height": 64},
            },
            "classifier": {
                "flat_slope_max": 5.0,
                "flat_roughness_max": 0.02,
                "flat_imu_pitch_max": 3.0,
                "slope_imu_pitch_min": 3.0,
                "rough_roughness_min": 0.05,
            },
            "control": {
                "flat": {"max_speed": 5.0, "turn_gain": 4.0},
                "slope": {"max_speed": 2.0, "turn_gain": 2.0},
                "rough": {"max_speed": 1.5, "turn_gain": 1.5},
                "transition": {"max_speed": 2.0, "turn_gain": 2.0},
            },
            "navigation": {
                "waypoints": [
                    [2.0, 0.0],
                    [2.0, 2.0],
                    [-2.0, 2.0],
                    [-2.0, -2.0],
                    [2.0, -2.0],
                    [0.0, 0.0],
                ],
                "distance_tolerance": 0.5,
            },
        }

    def get(self, key_path: str, default=None):
        """Get config value by dot-separated path, e.g., 'control.flat.max_speed'."""
        keys = key_path.split(".")
        value = self.data
        for k in keys:
            if isinstance(value, dict) and k in value:
                value = value[k]
            else:
                return default
        return value

    def save(self, path: str = None):
        """Save current config to YAML file."""
        out_path = path or DEFAULT_CONFIG_PATH
        with open(out_path, "w", encoding="utf-8") as f:
            yaml.dump(self.data, f, default_flow_style=False, allow_unicode=True)


# Global config instance
_config = None


def get_config(reload: bool = False) -> Config:
    global _config
    if _config is None or reload:
        _config = Config()
    return _config
