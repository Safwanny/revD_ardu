# ARCHITECTURE — Rev D baseline

ROS 2 Jazzy, Gazebo Harmonic, one un-namespaced robot. The URDF robot name is
`mobile_base`. The scope is the robot model, its three sensor types and manual
driving; nothing estimates state, maps or plans yet.

## Package map

| Package | Responsibility |
| --- | --- |
| `rev_base_description` | `urdf/` Xacro model, `meshes/rev_d/` (installed from the repo root), `rviz/`, `display.launch.py` for the model alone |
| `rev_base_gazebo` | `worlds/my_world.sdf` (10 × 10 m apartment), `worlds/navigation_basic.sdf` (room with an interior wall) |
| `rev_base_bringup` | `simulation.launch.py`, `camera_view.launch.py`, `config/controllers.yaml`, `scripts/generate_sim_sdf.py` |
| `rev_base_tools` | `tof_rows`, `tof_ray_markers`, `tof_camera_points`, `tof_depth_colour`, `stop_simulation`; `config/tof_rows.yaml` |

## Runtime data flow

```text
teleop_twist_keyboard (stamped)
        |
        v
/mobile_base_controller/reference  (TwistStamped, body frame)
        |
        v
mecanum_drive_controller --> four wheel velocities --> gz_ros2_control --> Gazebo
        +--> /mobile_base_controller/odometry
        +--> /tf  odom -> base_footprint

Gazebo sensors --> ros_gz_bridge
        /scan
        /tof/<face>/points ---> tof_rows ---> /tof/<face>/rows ---> tof_ray_markers ---> /tof/rays
        /tof_camera/points_raw ---> tof_camera_points ---> /tof_camera/points
        /tof_camera/depth_image ---> tof_depth_colour ---> /tof_camera/depth_colour ---> rqt_image_view
        /tof_camera/image, /tof_camera/camera_info, /clock
```

`<face>` is `front`, `rear`, `left` or `right`.

## Frames

```text
odom --(mecanum_drive_controller)--> base_footprint --> base_link
                                                          +-- lidar_link
                                                          +-- tof_front_link, tof_rear_link, tof_left_link, tof_right_link
                                                          +-- tof_camera_link --> tof_camera_lens_link, tof_camera_optical_frame
                                                          +-- four *_wheel_link (from /joint_states)
```

- The controller is the only publisher of `odom -> base_footprint`.
  `robot_state_publisher` publishes everything below `base_footprint`.
- `base_footprint` is on the ground under the robot centre. `base_link` sits on
  the axle line, `wheel_radius` (29.96 mm) up, so the wheel joints are at z = 0.
- REP-103 throughout: +X forward, +Y left, +Z up, SI units.

## Geometry

Every number lives in `rev_base_description/urdf/properties.xacro`, with the
measurement or source next to it. Highlights:

| Item | Value |
| --- | --- |
| Hull bounding box | 295.9 × 168.8 × 49 mm, underside 11.56 mm off the ground |
| Wheels | 60 mm printed mecanum, 31.5 mm wide; FL and RR hand A, FR and RL hand B |
| Wheelbase × track | 210 × 186.5 mm |
| Deck top | 58.06 mm; M3 insert grid at 20 mm pitch |
| Mass | 1.40 kg body + 4 × 0.08 kg wheels: BOM estimates, to replace by weighing |
| Wheel speed limit | 10.47 rad/s (100 rpm assumed until the real gearing is known) |

The meshes come from `rev_base_description/scripts/split_rev_d_mesh.py`, which
splits the whole-robot CAD STL into the body and four wheels and checks them
against the same numbers.

## Sensors

### LiDAR

A D500-style 360° planar LiDAR on 30 mm M3 standoffs over deck holes
(40, ±10) and (20, ±10) mm, centred at x = 30 mm. The scan plane (`lidar_link`,
mid height of the head) is 103 mm up, which clears the ToF camera in front of
it. The standoffs exist for that clearance.

### ToF ring

Four VL53L7CX, one at the centre of each hull face, 32 mm off the ground. Each
gives an 8 × 8 grid over 60 × 60°.

- **Pitch.** Every sensor is pitched 11.25° down (1.5 zones), so zone row 5 is
  exactly level and reaches as far as the part can see. The hull faces lean
  14.86° (front and rear) and 9.27° (sides), so the CAD needs a 3.61° wedge
  (thick at the bottom) front and rear, and a 1.98° wedge (thick at the top) on
  the sides. `tof_pitch` is the calibration knob.
- **Rows.** All 64 zones are simulated, because the real part has no per-zone
  readout. `tof_rows` keeps rows 4, 5 and 6 per `config/tof_rows.yaml`:
  - row 4 is −7.5° and meets the floor at 0.24 m
  - row 5 is level
  - row 6 is +7.5°

  `tof_ray_markers` reads the same file.
- **Grid order.** Gazebo fills the organised cloud bottom row first. Row 0 is
  the bottom of the grid, and `tof_ray_markers` relies on that.
- **Board.** The carrier is modelled to Pololu's drawing. Which way the header
  side faces is a guess (`tof_board_offset_y`).

### ToF camera

The camera is an Arducam ToF (B0410). It has no RGB and stands upright on an
L bracket screwed to deck holes (60, ±10) mm, looking straight ahead.

- **Sensor type.** Gazebo simulates it as an `rgbd_camera`. Its pixels are
  square, so the image is 240 × 136 instead of the real 240 × 180, which keeps
  both the real 62.8° and 37.9° fields.
- **Cloud frame.** Gazebo's cloud is x-forward but stamped with the optical
  frame, so `tof_camera_points` restamps it into `tof_camera_lens_link`.
- **Estimated dimensions.** Board thickness, lens depth and lens position are
  estimates. Arducam's STEP model would replace them.
- **Real driver.** Arducam's own ROS 2 node publishes no `Image` or
  `CameraInfo`, and hard-codes the frame `map`. The real camera will need a
  node of our own.

## Simulation launch

`simulation.launch.py` runs in this order:

1. Resolve the world and write a copy to `/tmp/mobile_base_world_<pid>_<engine>.sdf`
   with the physics step and render engine applied.
2. Run `generate_sim_sdf.py`, which expands the Xacro and adds the mecanum
   wheel contact (below), then writes `/tmp/mobile_base_sim_<pid>.sdf`.
3. Spawn the robot, then `joint_state_broadcaster`, then `mobile_base_controller`
   and RViz.
4. Alongside: the bridge, the ToF nodes, and `camera_view.launch.py`.

Arguments:

| Argument | Default | Meaning |
| --- | --- | --- |
| `world` | `navigation_basic` | world name or absolute SDF path |
| `gui` | `true` | Gazebo client window; `false` runs the server only |
| `rviz` | `true` | main RViz window |
| `camera_windows` | `true` | colour-graded depth window |
| `tof_markers` | `true` | ToF ray markers |
| `render_engine` | `ogre2` | `ogre` only for software-rendered checks; its ranges are wrong |
| `physics_max_step_size` | `0.001` | Gazebo physics step, seconds |
| `start_controller` | `true` | spawn `mobile_base_controller` |
| `x`, `y`, `z`, `yaw` | `0, 0, 0.10, 0` | spawn pose |
| `use_sim_time` | `true` | Gazebo clock for every node |

`_gazebo_environment()` puts the Gazebo vendor libraries first on the library
path. It also removes any workspace overlay of `mecanum_drive_controller`, so
the controller always comes from `/opt/ros/jazzy`.

## Mecanum wheel contact

The URDF wheels are plain cylinders for collision. `generate_sim_sdf.py`
attaches Gazebo friction with a direction to each wheel:

- **Friction values:** `mu = 0.8` along the roller axis and `mu2 = 0.2`
  across it.
- **Direction:** an X pattern across the four wheels, expressed in
  `base_footprint`.
- **Rollers:** passive roller links are rejected.

Plain cylinders that grip equally in every direction make the robot slide when
it should strafe. `test_mecanum_wheel_contact.py` checks the generated SDF.

## Controller

`config/controllers.yaml`:

- **Command timeout:** `reference_timeout: 0.5` s. With no new command, the
  wheels stop after 0.5 s.
- **Turning calibration:** `sum_of_robot_center_projection_on_X_Y_axis` is
  0.18268, not the geometric 0.19825. That value was calibrated against
  Gazebo's true turn angle, so it depends on the simulated contact. Re-measure
  it on hardware.
- **Covariances:** odometry covariances are constant, and sideways velocity
  gets less confidence than forward.

## Where to start

| Task | Start here |
| --- | --- |
| Change robot geometry | `properties.xacro`, then `test_mecanum_wheels.py` and the controller kinematics |
| Change wheel contact | `generate_sim_sdf.py`, then `test_mecanum_wheel_contact.py` |
| Move or retune a sensor | `properties.xacro` (pose, range, rate), the sensor's `*.gazebo.xacro`, then `test_sensor_topics.py` |
| Add a sensor | description Xacro, Gazebo Xacro, a bridge entry in `simulation.launch.py`, a check in `test_sensor_topics.py` |
| Change which ToF rows are kept | `rev_base_tools/config/tof_rows.yaml` |
| Change the RViz view | `rev_base_description/rviz/mobile_base_sim.rviz` |

## Known boundaries

- **Simulation only.** No result here is evidence about hardware.
- **Render engine.** Under `ogre`, Gazebo's ranges are wrong while the clouds
  still look valid. Use `ogre2`, which needs a GPU.
- **Leftover processes.** Gazebo launch tests can leave `gz sim` servers
  running. Give each run its own `ROS_DOMAIN_ID` and `GZ_PARTITION`, and finish
  with `stop_simulation`.
- **Temporary files.** The generated `/tmp/mobile_base_*` files are not removed
  on shutdown. `stop_simulation` clears them.
- **Not visible in `image_view`.** `image_view` freezes on 32FC1 depth, which
  is why the depth is colour-graded and shown in `rqt_image_view`.
