"""Generate .wbt world files — larger, more realistic terrain scenes."""

import os

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
WORLDS_DIR = os.path.join(PROJECT_ROOT, "worlds")

TERRAIN_SIZE = 40
RESOLUTION = 80


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
      device [ RotationalMotor {{ name "wheel_fl" maxVelocity 10 }} ]
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
      device [ RotationalMotor {{ name "wheel_fr" maxVelocity 10 }} ]
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
      device [ RotationalMotor {{ name "wheel_rl" maxVelocity 10 }} ]
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
      device [ RotationalMotor {{ name "wheel_rr" maxVelocity 10 }} ]
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
               robot_x=0.0, robot_y=0.0, robot_z=0.5, terrain_color="0.35 0.55 0.25"):
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
        0.6
      ]
    }}
  ]
}}
Viewpoint {{
  orientation -0.4 -0.4 0.82 1.9
  position 15 -15 12
  follow "adaptive_robot"
  followType "Pan and Tilt Shot"
  followSmoothness 0.3
}}
TexturedBackground {{
  texture "noon_cloudy_countryside"
}}
TexturedBackgroundLight {{
  texture "noon_cloudy_countryside"
  castShadows TRUE
}}
DEF TERRAIN Solid {{
  translation -{size/2} -{size/2} 0
  children [
    DEF TERRAIN_SHAPE Shape {{
      appearance PBRAppearance {{
        baseColor {terrain_color}
        roughness 0.9
        metalness 0
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
{TARGET_MARKERS}
{robot}
'''


if __name__ == "__main__":
    terrains = [
        ("斜坡地形导航 - Slope Terrain Navigation", "slope_heights.txt", "slope_terrain.wbt",
         -5.0, 0.0, 0.8, "0.45 0.55 0.3"),
        ("凹凸地形导航 - Rough Terrain Navigation", "rough_heights.txt", "rough_terrain.wbt",
         0.0, 0.0, 0.5, "0.5 0.4 0.25"),
        ("过渡区地形导航 - Transition Terrain Navigation", "transition_heights.txt", "transition_terrain.wbt",
         -8.0, 0.0, 0.5, "0.4 0.5 0.3"),
    ]

    for title, heights_file, wbt_file, rx, ry, rz, color in terrains:
        heights_path = os.path.join(WORLDS_DIR, heights_file)
        wbt_path = os.path.join(WORLDS_DIR, wbt_file)
        content = make_world(title, heights_path, robot_x=rx, robot_y=ry, robot_z=rz, terrain_color=color)
        with open(wbt_path, "w", encoding="utf-8") as f:
            f.write(content)
        print(f"Created: {wbt_file} (robot at {rx}, {ry}, {rz})")
