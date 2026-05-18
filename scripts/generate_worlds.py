"""Generate .wbt world files for all terrain types."""

import os

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
WORLDS_DIR = os.path.join(PROJECT_ROOT, "worlds")


def robot_node(x=0.0, y=0.0, z=0.5, controller="adaptive_navigator"):
    return f'''Robot {{
  translation {x} {y} {z}
  children [
    DEF BODY Shape {{
      appearance PBRAppearance {{ baseColor 0.2 0.4 0.8 roughness 0.5 }}
      geometry Box {{ size 0.4 0.3 0.1 }}
    }}
    DEF LIDAR Lidar {{
      translation 0 0 0.08
      horizontalResolution 360
      fieldOfView 6.2832
      numberOfLayers 1
      maxRange 10.0
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
        children [ Shape {{ appearance PBRAppearance {{ baseColor 0.1 0.1 0.1 }} geometry Cylinder {{ height 0.04 radius 0.06 }} }} ]
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
        children [ Shape {{ appearance PBRAppearance {{ baseColor 0.1 0.1 0.1 }} geometry Cylinder {{ height 0.04 radius 0.06 }} }} ]
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
        children [ Shape {{ appearance PBRAppearance {{ baseColor 0.1 0.1 0.1 }} geometry Cylinder {{ height 0.04 radius 0.06 }} }} ]
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
        children [ Shape {{ appearance PBRAppearance {{ baseColor 0.1 0.1 0.1 }} geometry Cylinder {{ height 0.04 radius 0.06 }} }} ]
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


def make_world(title, heights_file, size=20, resolution=40,
               robot_x=0.0, robot_y=0.0, robot_z=0.5):
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
}}
Viewpoint {{
  orientation -0.3 0.0 0.95 0.5
  position -5 -15 12
}}
TexturedBackground {{
}}
TexturedBackgroundLight {{
}}
DEF TERRAIN Solid {{
  translation -{size/2} -{size/2} 0
  children [
    DEF TERRAIN_SHAPE Shape {{
      appearance PBRAppearance {{ baseColor 0.4 0.6 0.3 roughness 0.9 }}
      geometry ElevationGrid {{
        height [{heights_str}]
        xDimension {resolution}
        xSpacing {spacing:.4f}
        yDimension {resolution}
        ySpacing {spacing:.4f}
      }}
    }}
  ]
  boundingObject USE TERRAIN_SHAPE
  locked TRUE
}}
{robot}
'''


if __name__ == "__main__":
    terrains = [
        ("Slope Terrain Navigation", "slope_heights.txt", "slope_terrain.wbt",
         -3.0, 0.0, 0.8),
        ("Rough Terrain Navigation", "rough_heights.txt", "rough_terrain.wbt",
         0.0, 0.0, 0.5),
        ("Transition Terrain Navigation", "transition_heights.txt", "transition_terrain.wbt",
         -3.0, 0.0, 0.5),
    ]

    for title, heights_file, wbt_file, rx, ry, rz in terrains:
        heights_path = os.path.join(WORLDS_DIR, heights_file)
        wbt_path = os.path.join(WORLDS_DIR, wbt_file)
        content = make_world(title, heights_path, robot_x=rx, robot_y=ry, robot_z=rz)
        with open(wbt_path, "w", encoding="utf-8") as f:
            f.write(content)
        print(f"Created: {wbt_file} (robot at {rx}, {ry}, {rz})")
