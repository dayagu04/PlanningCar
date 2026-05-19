"""Generate .wbt world files — larger, more realistic terrain scenes."""

import os
import math
import random

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
WORLDS_DIR = os.path.join(PROJECT_ROOT, "worlds")

TERRAIN_SIZE = 40
RESOLUTION = 80
MOTOR_MAX_VELOCITY = 25.0


def _tree(idx, x, y, z, height=4.0, foliage_color="0.18 0.45 0.18", trunk_color="0.35 0.22 0.12"):
    trunk_h = height * 0.45
    foliage_h = height * 0.6
    return f'''DEF TREE_{idx} Solid {{
  translation {x:.2f} {y:.2f} {z:.2f}
  children [
    Pose {{
      translation 0 0 {trunk_h/2:.3f}
      children [
        Shape {{
          appearance PBRAppearance {{ baseColor {trunk_color} roughness 1 metalness 0 }}
          geometry Cylinder {{ height {trunk_h:.3f} radius 0.18 }}
        }}
      ]
    }}
    Pose {{
      translation 0 0 {trunk_h + foliage_h/2:.3f}
      children [
        Shape {{
          appearance PBRAppearance {{ baseColor {foliage_color} roughness 0.9 metalness 0 }}
          geometry Cone {{ height {foliage_h:.3f} bottomRadius 1.1 }}
        }}
      ]
    }}
  ]
  name "tree_{idx}"
  boundingObject Pose {{
    translation 0 0 {trunk_h/2:.3f}
    children [ Cylinder {{ height {trunk_h:.3f} radius 0.18 }} ]
  }}
}}'''


def _rock(idx, x, y, z, size=0.5, color="0.45 0.42 0.4"):
    sx, sy, sz = size, size * 0.85, size * 0.6
    return f'''DEF ROCK_{idx} Solid {{
  translation {x:.2f} {y:.2f} {z + sz/2:.3f}
  rotation 0 0 1 {random.uniform(0, 6.28):.2f}
  children [
    Shape {{
      appearance PBRAppearance {{ baseColor {color} roughness 1 metalness 0 }}
      geometry Box {{ size {sx:.2f} {sy:.2f} {sz:.2f} }}
    }}
  ]
  name "rock_{idx}"
  boundingObject Box {{ size {sx:.2f} {sy:.2f} {sz:.2f} }}
}}'''


def _scenery(seed: int, sample_height_fn, size=TERRAIN_SIZE,
             tree_count=14, rock_count=20, exclusion_radius=4.0):
    """Place trees and rocks; returns (wbt_string, obstacles_list).

    obstacles_list: [(x, y, radius), ...] for every solid object with a
    bounding cylinder/box — used by the A* planner to mark no-go cells.
    """
    rng = random.Random(seed)
    half = size / 2.0
    items = []
    obstacles = []

    for i in range(tree_count):
        for _ in range(20):
            x = rng.uniform(-half + 1, half - 1)
            y = rng.uniform(-half + 1, half - 1)
            if math.hypot(x, y) >= exclusion_radius:
                break
        z = sample_height_fn(x, y) - 0.05
        height = rng.uniform(3.0, 5.0)
        green_shade = rng.uniform(0.12, 0.25)
        foliage = f"{green_shade*0.5:.2f} {green_shade*1.8:.2f} {green_shade*0.6:.2f}"
        items.append(_tree(i + 1, x, y, z, height=height, foliage_color=foliage))
        obstacles.append((x, y, 0.5))  # trunk radius 0.18 + safety margin

    for i in range(rock_count):
        for _ in range(20):
            x = rng.uniform(-half + 0.5, half - 0.5)
            y = rng.uniform(-half + 0.5, half - 0.5)
            if math.hypot(x, y) >= exclusion_radius * 0.8:
                break
        z = sample_height_fn(x, y)
        rsize = rng.uniform(0.3, 0.9)
        gray = rng.uniform(0.35, 0.55)
        col = f"{gray:.2f} {gray*0.95:.2f} {gray*0.9:.2f}"
        items.append(_rock(i + 1, x, y, z, size=rsize, color=col))
        obstacles.append((x, y, rsize * 0.7))

    return "\n".join(items), obstacles


def robot_node(x=0.0, y=0.0, z=0.2, controller="adaptive_navigator"):
    return f'''Robot {{
  translation {x} {y} {z}
  rotation 0 0 1 0
  children [
    DEF BODY Shape {{
      appearance PBRAppearance {{ baseColor 0.2 0.4 0.8 roughness 0.5 metalness 0.3 }}
      geometry Box {{ size 0.4 0.3 0.1 }}
    }}
    DEF LIDAR Lidar {{
      translation 0 0 0.08
      horizontalResolution 360
      fieldOfView 6.2832
      numberOfLayers 1
      maxRange 15
    }}
    DEF GPS GPS {{ translation 0 0 0.05 }}
    DEF COMPASS Compass {{ translation 0 0 0.05 }}
    DEF IMU InertialUnit {{ translation 0 0 0.05 }}
    DEF WHEEL_FL HingeJoint {{
      jointParameters HingeJointParameters {{ axis 0 1 0 anchor 0.15 0.18 -0.03 }}
      device [ RotationalMotor {{ name "wheel_fl" maxVelocity {MOTOR_MAX_VELOCITY} }} ]
      endPoint Solid {{
        translation 0.15 0.18 -0.03
        rotation 1 0 0 1.5708
        children [ Shape {{ appearance PBRAppearance {{ baseColor 0.1 0.1 0.1 roughness 0.8 }} geometry Cylinder {{ height 0.04 radius 0.06 }} }} ]
        name "wheel_fl"
        boundingObject Cylinder {{ height 0.04 radius 0.06 }}
        physics Physics {{ density 500 }}
      }}
    }}
    DEF WHEEL_FR HingeJoint {{
      jointParameters HingeJointParameters {{ axis 0 1 0 anchor 0.15 -0.18 -0.03 }}
      device [ RotationalMotor {{ name "wheel_fr" maxVelocity {MOTOR_MAX_VELOCITY} }} ]
      endPoint Solid {{
        translation 0.15 -0.18 -0.03
        rotation 1 0 0 1.5708
        children [ Shape {{ appearance PBRAppearance {{ baseColor 0.1 0.1 0.1 roughness 0.8 }} geometry Cylinder {{ height 0.04 radius 0.06 }} }} ]
        name "wheel_fr"
        boundingObject Cylinder {{ height 0.04 radius 0.06 }}
        physics Physics {{ density 500 }}
      }}
    }}
    DEF WHEEL_RL HingeJoint {{
      jointParameters HingeJointParameters {{ axis 0 1 0 anchor -0.15 0.18 -0.03 }}
      device [ RotationalMotor {{ name "wheel_rl" maxVelocity {MOTOR_MAX_VELOCITY} }} ]
      endPoint Solid {{
        translation -0.15 0.18 -0.03
        rotation 1 0 0 1.5708
        children [ Shape {{ appearance PBRAppearance {{ baseColor 0.1 0.1 0.1 roughness 0.8 }} geometry Cylinder {{ height 0.04 radius 0.06 }} }} ]
        name "wheel_rl"
        boundingObject Cylinder {{ height 0.04 radius 0.06 }}
        physics Physics {{ density 500 }}
      }}
    }}
    DEF WHEEL_RR HingeJoint {{
      jointParameters HingeJointParameters {{ axis 0 1 0 anchor -0.15 -0.18 -0.03 }}
      device [ RotationalMotor {{ name "wheel_rr" maxVelocity {MOTOR_MAX_VELOCITY} }} ]
      endPoint Solid {{
        translation -0.15 -0.18 -0.03
        rotation 1 0 0 1.5708
        children [ Shape {{ appearance PBRAppearance {{ baseColor 0.1 0.1 0.1 roughness 0.8 }} geometry Cylinder {{ height 0.04 radius 0.06 }} }} ]
        name "wheel_rr"
        boundingObject Cylinder {{ height 0.04 radius 0.06 }}
        physics Physics {{ density 500 }}
      }}
    }}
  ]
  name "adaptive_robot"
  boundingObject Box {{ size 0.4 0.3 0.1 }}
  physics Physics {{ density 1000 }}
  controller "{controller}"
}}'''


TARGET_MARKERS = '''DEF TARGET_MARKER_1 Solid {
  translation 8 0 0.05
  children [
    Shape {
      appearance PBRAppearance { baseColor 1 0 0 emissiveColor 0.5 0 0 }
      geometry Cylinder { height 0.1 radius 0.3 }
    }
  ]
  name "target_1"
}
DEF TARGET_MARKER_2 Solid {
  translation 8 8 0.05
  children [
    Shape {
      appearance PBRAppearance { baseColor 1 0.5 0 emissiveColor 0.5 0.25 0 }
      geometry Cylinder { height 0.1 radius 0.3 }
    }
  ]
  name "target_2"
}
DEF TARGET_MARKER_3 Solid {
  translation -8 8 0.05
  children [
    Shape {
      appearance PBRAppearance { baseColor 1 1 0 emissiveColor 0.5 0.5 0 }
      geometry Cylinder { height 0.1 radius 0.3 }
    }
  ]
  name "target_3"
}
DEF TARGET_MARKER_4 Solid {
  translation -8 -8 0.05
  children [
    Shape {
      appearance PBRAppearance { baseColor 0 1 0 emissiveColor 0 0.5 0 }
      geometry Cylinder { height 0.1 radius 0.3 }
    }
  ]
  name "target_4"
}
DEF TARGET_MARKER_5 Solid {
  translation 8 -8 0.05
  children [
    Shape {
      appearance PBRAppearance { baseColor 0 0.5 1 emissiveColor 0 0.25 0.5 }
      geometry Cylinder { height 0.1 radius 0.3 }
    }
  ]
  name "target_5"
}'''


def make_world(title, heights_file, size=TERRAIN_SIZE, resolution=RESOLUTION,
               robot_x=0.0, robot_y=0.0, robot_z=0.5, terrain_color="0.55 0.5 0.4",
               texture_url="https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/default/worlds/textures/grass.jpg",
               texture_scale=20, scenery="", fog_color="0.78 0.82 0.88"):
    with open(heights_file, "r") as f:
        heights_str = f.read().strip()

    spacing = size / (resolution - 1)
    robot = robot_node(robot_x, robot_y, robot_z)

    return f'''#VRML_SIM R2025a utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"

WorldInfo {{
  title "{title}"
  basicTimeStep 32
  contactProperties [
    ContactProperties {{
      material1 "terrain"
      coulombFriction [
        0.85
      ]
      softCFM 0.00005
    }}
  ]
}}
Viewpoint {{
  orientation -0.4 -0.4 0.82 1.9
  position 18 -18 14
  follow "adaptive_robot"
  followType "Pan and Tilt Shot"
  followSmoothness 0.3
}}
TexturedBackground {{
  texture "noon_cloudy_countryside"
  luminosity 1.2
}}
TexturedBackgroundLight {{
  texture "noon_cloudy_countryside"
  castShadows TRUE
}}
DirectionalLight {{
  direction -0.4 -0.5 -1
  color 1.0 0.96 0.88
  intensity 2.5
  castShadows TRUE
}}
Fog {{
  color {fog_color}
  visibilityRange 90
}}
DEF TERRAIN Solid {{
  translation -{size/2} -{size/2} 0
  children [
    DEF TERRAIN_SHAPE Shape {{
      appearance PBRAppearance {{
        baseColor {terrain_color}
        baseColorMap ImageTexture {{
          url [
            "{texture_url}"
          ]
        }}
        roughness 0.95
        metalness 0
        textureTransform TextureTransform {{
          scale {texture_scale} {texture_scale}
        }}
      }}
      geometry ElevationGrid {{
        height [{heights_str}]
        xDimension {resolution}
        xSpacing {spacing:.4f}
        yDimension {resolution}
        ySpacing {spacing:.4f}
      }}
    }}
  ]
  name "terrain"
  contactMaterial "terrain"
  boundingObject USE TERRAIN_SHAPE
  locked TRUE
}}
{scenery}
{TARGET_MARKERS}
{robot}
'''


if __name__ == "__main__":
    import sys
    import json
    sys.path.insert(0, PROJECT_ROOT)
    from src.utils.terrain_sampling import sample_terrain_height

    GRASS = "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/default/worlds/textures/grass.jpg"

    terrains = [
        ("斜坡地形导航 - Slope Terrain Navigation", "slope_heights.txt", "slope_terrain.wbt",
         (-5.0, 0.0, 0.8), "0.85 0.9 0.75", GRASS, 25, "0.78 0.82 0.88", 11),
        ("凹凸地形导航 - Rough Terrain Navigation", "rough_heights.txt", "rough_terrain.wbt",
         (0.0, 0.0, 0.5), "0.85 0.78 0.6", GRASS, 30, "0.85 0.83 0.78", 22),
        ("过渡区地形导航 - Transition Terrain Navigation", "transition_heights.txt", "transition_terrain.wbt",
         (-8.0, 0.0, 0.5), "0.8 0.85 0.7", GRASS, 25, "0.8 0.84 0.86", 33),
    ]

    for title, heights_file, wbt_file, robot_xyz, color, tex_url, tex_scale, fog, seed in terrains:
        heights_path = os.path.join(WORLDS_DIR, heights_file)
        wbt_path = os.path.join(WORLDS_DIR, wbt_file)

        def make_sampler(world_basename):
            def sampler(x, y):
                return sample_terrain_height(world_basename, WORLDS_DIR, x, y)
            return sampler

        scenery_str, obstacles = _scenery(seed, make_sampler(wbt_file))
        rx, ry, rz = robot_xyz
        content = make_world(title, heights_path, robot_x=rx, robot_y=ry, robot_z=rz,
                             terrain_color=color, texture_url=tex_url, texture_scale=tex_scale,
                             scenery=scenery_str, fog_color=fog)
        with open(wbt_path, "w", encoding="utf-8") as f:
            f.write(content)
        # Persist obstacles alongside the .wbt so the controller can find them
        obs_path = wbt_path.replace(".wbt", "_obstacles.json")
        with open(obs_path, "w", encoding="utf-8") as f:
            json.dump([{"x": x, "y": y, "r": r} for x, y, r in obstacles], f, indent=2)
        print(f"Created: {wbt_file} (robot at {rx}, {ry}, {rz}, {len(obstacles)} obstacles)")
