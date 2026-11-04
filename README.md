# Rev Mobile Base — Mecanum 01 Rev D in ROS 2

The Rev D build of a four-wheel mecanum robot, simulated in ROS 2 Jazzy and
Gazebo Harmonic. It was split on 2026-10-31 from
[mecanum_wheel_bot](https://github.com/Safwanny/mecanum_wheel_bot), which stays
the original prototype.

Rev D changes the sensor setup and how the sensors are used, so this repository
starts again from a small baseline: the robot model, its sensors, and driving it
by hand in simulation. Everything above that (state estimation, mapping,
navigation, perception) gets rebuilt for Rev D from here.

## What it does today

- **Robot model.** The Rev D v4 CAD with the type_04M_Ardu pan turret
  (`Rev_D_T4M_ardu`): a body mesh, four turning wheels and the turret, with
  collision and inertial geometry from the same measured numbers.
- **Driving.** `mecanum_drive_controller` (ros2_control) drives all four wheels
  from keyboard teleop. Gazebo models the mecanum wheel contact with
  direction-dependent friction, so the robot strafes instead of sliding.
- **Turret.** A position controller pans the camera turret through ±135° on
  `/turret_controller/commands`.
- **Sensors**, all simulated:

| Sensor | Part | Simulated as | Topic |
| --- | --- | --- | --- |
| ToF ring | 4 × VL53L7CX on Pololu #3418 carriers, flush on each 11.25° hull wall | 8 × 8 zones over 60°, 15 Hz, 0.02–2 m with datasheet noise and dropout, pitched 11.25° down | `/tof/<face>/points`, `/tof/<face>/rows` |
| ToF camera | Arducam ToF (B0410) on the pan turret | 240 × 136 depth, 62.8 × 37.9°, 0.2–4 m, 30 Hz | `/tof_camera/depth_image`, `/tof_camera/points` |

Rev D has no LiDAR any more.

- **Visualisation.** RViz shows the ToF rows that are kept,
  faint ToF rays, and the camera's 3D depth cloud. A separate window shows the
  camera depth colour-graded: near is red, far is blue, no return is black.

Everything is simulation only. No hardware driver exists yet.

## Packages

| Package | Responsibility |
| --- | --- |
| `rev_base_description` | Xacro model, Rev D meshes, turret, sensors, ros2_control declaration, RViz configs |
| `rev_base_gazebo` | Gazebo worlds (`my_world`, `navigation_basic`) |
| `rev_base_bringup` | Simulation launch, controller config, generated SDF with wheel contact |
| `rev_base_tools` | ToF sensor model, row selection, ray markers, camera cloud restamp, depth colouring, `stop_simulation` |

## Documentation

| You want to | Read |
| --- | --- |
| Build, run, drive and inspect it | [GUIDE.md](GUIDE.md) |
| Look up a frame, topic, dimension or design decision | [ARCHITECTURE.md](ARCHITECTURE.md) |

## Next

Not yet built for Rev D: state estimation, mapping and localization,
navigation, using the ToF data for perception, and the hardware drivers. Each
one is tracked as a GitHub issue when work starts.
