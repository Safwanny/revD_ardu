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

- **Robot model.** The Rev D CAD, split into a body mesh and four turning wheel
  meshes, with collision and inertial geometry from the same measured numbers.
- **Driving.** `mecanum_drive_controller` (ros2_control) drives all four wheels
  from keyboard teleop. Gazebo models the mecanum wheel contact with
  direction-dependent friction, so the robot strafes instead of sliding.
- **Sensors**, all simulated:

| Sensor | Part | Simulated as | Topic |
| --- | --- | --- | --- |
| LiDAR | 360° planar, D500-style | 720 rays, 10 Hz, 0.10–4.0 m, scan plane 103 mm up | `/scan` |
| ToF ring | 4 × VL53L7CX on Pololu #3418 carriers, one per hull face | 8 × 8 zones over 60°, 15 Hz, 0.02–3.5 m, pitched 11.25° down | `/tof/<face>/points`, `/tof/<face>/rows` |
| ToF camera | Arducam ToF (B0410) on the deck, ahead of the LiDAR | 240 × 136 depth, 62.8 × 37.9°, 0.2–4 m, 30 Hz | `/tof_camera/depth_image`, `/tof_camera/points` |

- **Visualisation.** RViz shows the LiDAR scan, the ToF rows that are kept,
  faint ToF rays, and the camera's 3D depth cloud. A separate window shows the
  camera depth colour-graded: near is red, far is blue, no return is black.

Everything is simulation only. No hardware driver exists yet.

## Packages

| Package | Responsibility |
| --- | --- |
| `rev_base_description` | Xacro model, Rev D meshes, sensors, ros2_control declaration, RViz configs |
| `rev_base_gazebo` | Gazebo worlds (`my_world`, `navigation_basic`) |
| `rev_base_bringup` | Simulation launch, controller config, generated SDF with wheel contact |
| `rev_base_tools` | ToF row selection, ray markers, camera cloud restamp, depth colouring, `stop_simulation` |

## Documentation

| You want to | Read |
| --- | --- |
| Build, run, drive and inspect it | [docs/GUIDE.md](docs/GUIDE.md) |
| Look up a frame, topic, dimension or design decision | [ARCHITECTURE.md](ARCHITECTURE.md) |

## Next

Not yet built for Rev D: state estimation, mapping and localization,
navigation, using the ToF data for perception, and the hardware drivers. Each
one is tracked as a GitHub issue when work starts.
