# ARCHITECTURE — Rev D baseline

ROS 2 Jazzy, Gazebo Harmonic, one un-namespaced robot. The URDF robot name is
`mobile_base`. The scope is the robot model (Rev D v4 with the type_04M_Ardu
pan turret, CAD `Rev_D_T4M_ardu`), its ToF sensors and IMU, manual driving,
turret panning and wheel + gyro odometry; nothing maps or plans yet. There is no LiDAR.

## Package map

| Package | Responsibility |
| --- | --- |
| `rev_base_description` | `urdf/` Xacro model, `meshes/rev_d/` (installed from the repo root), `rviz/`, `display.launch.py` for the model alone, `scripts/import_rev_d_meshes.py` |
| `rev_base_gazebo` | `worlds/my_world.sdf` (10 × 10 m apartment), `worlds/navigation_basic.sdf` (room with an interior wall) |
| `rev_base_bringup` | `simulation.launch.py`, `camera_view.launch.py`, `config/controllers.yaml`, `scripts/generate_sim_sdf.py` |
| `rev_base_tools` | `tof_rows`, `tof_ray_markers`, `tof_camera_points`, `tof_depth_colour`, `stop_simulation`, `imu_bias`, `odom_eval`, `odom_eval_plot`; `config/tof_rows.yaml` |

## Runtime data flow

```text
teleop_twist_keyboard (stamped)
        |
        v
/mobile_base_controller/reference  (TwistStamped, body frame)
        |
        v
mecanum_drive_controller --> four wheel velocities --> gz_ros2_control --> Gazebo
        +--> /mobile_base_controller/odometry (vx, vy) --+
      /imu/data --> imu_bias --> /imu/data_unbiased (vyaw) --+--> ekf_filter_node
                                                              +--> /odometry/filtered
                                                              +--> /tf  odom -> base_footprint

/turret_controller/commands  (Float64MultiArray, pan rad)
        |
        v
JointGroupPositionController --> turret_pan_joint --> gz_ros2_control --> Gazebo

joint_state_broadcaster --> /joint_states (4 wheels + turret_pan_joint) --> robot_state_publisher

Gazebo sensors --> ros_gz_bridge
        /tof/<face>/points ---> tof_rows ---> /tof/<face>/rows ---> tof_ray_markers ---> /tof/rays
        /tof_camera/points_raw ---> tof_camera_points ---> /tof_camera/points
        /tof_camera/depth_image ---> tof_depth_colour ---> /tof_camera/depth_colour ---> rqt_image_view
        /tof_camera/image, /tof_camera/camera_info, /clock
        /imu/data  (BNO085 stand-in, 100 Hz, no orientation)
        /ground_truth/odom  (Gazebo true pose; evaluation only, never /tf)
```

`<face>` is `front`, `rear`, `left` or `right`.

## Frames

```text
odom --(ekf_filter_node)--> base_footprint --> base_link
                                                          +-- tof_front_link, tof_rear_link, tof_left_link, tof_right_link
                                                          +-- imu_link
                                                          +-- four *_wheel_link (from /joint_states)
                                                          +-- turret_link (turret_pan_joint, from /joint_states)
                                                                +-- tof_camera_link --> tof_camera_lens_link, tof_camera_optical_frame
```

- `ekf_filter_node` is the only publisher of `odom -> base_footprint`; the
  controller runs with `enable_odom_tf: false`. `robot_state_publisher`
  publishes everything below `base_footprint`.
- `base_footprint` is on the ground under the robot centre. `base_link` sits on
  the axle line, `wheel_radius` (29.96 mm) up, so the wheel joints are at z = 0.
- REP-103 throughout: +X forward, +Y left, +Z up, SI units.

## Geometry

Every number lives in `rev_base_description/urdf/properties.xacro`, with the
measurement or source next to it. Highlights:

| Item | Value |
| --- | --- |
| Hull bounding box | 296.9 × 168.1 × 49 mm, underside 11.56 mm off the ground; every wall leans 11.25° |
| Wheels | 60 mm printed mecanum, 31.5 mm wide; FL and RR hand A, FR and RL hand B |
| Wheelbase × track | 210 × 186.5 mm |
| Deck top | 58.06 mm; M3 insert grid at 20 mm pitch (`Deck_plate_T` loses five inserts to the turret) |
| Turret | pan axis (35.79, 0) mm on the deck top, ±138° hard stops, ±135° URDF limit |
| Mass | 1.40 kg body + 4 × 0.08 kg wheels + 0.12 kg turret: estimates, to replace by weighing |
| Wheel speed limit | 10.47 rad/s (100 rpm assumed until the real gearing is known) |

The meshes come from `rev_base_description/scripts/import_rev_d_meshes.py`. It
reads the per-link STLs in `Rev_D_T4M_ardu/ros2/meshes`, simplifies them about
5× and checks them against the same numbers. The CAD export puts each wheel
frame 3.5 mm inboard of the roller centre (the bounding box of wheel + hub), so
the script re-centres the wheels on the rollers, which keeps the track at
186.5 mm. The CAD's own URDF (`mecanum_rev_d_tof_turret.urdf`, wheels at
±89.75 mm) is not used.

## Sensors

### ToF ring

Four VL53L7CX, one at the centre of each hull face, 32 mm off the ground. Each
gives an 8 × 8 grid over 60 × 60°.

- **Pitch.** Every sensor is pitched 11.25° down (1.5 zones), so zone row 5 is
  exactly level and reaches as far as the part can see. Since Rev D v3 every
  hull wall leans 11.25° (`WALL_SLANT`; the plane fits of `body.stl` agree), so
  the carriers sit flush on the walls and need no wedge. `tof_pitch` is the
  calibration knob.
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

### IMU

A BNO085 stand-in on `imu_link`, axes as `base_link`, at the `base_link`
origin until its mount is in the CAD. Gazebo's `imu` sensor publishes
`/imu/data` at 100 Hz.

- **Noise.** Gyro white noise 1.7e-3 rad/s (BNO055 rms) and a static bias of
  1.5e-4 rad/s, the BNO08x's 0.5°/min heading drift spec. Sources in
  `properties.xacro`. `gz sim --seed 1` draws the same bias every run.
- **No orientation.** gz-sensors fills orientation from the true pose with no
  noise, so `enable_orientation` is off: nothing downstream can fuse truth.

### Ground truth

Gazebo's `OdometryPublisher` publishes the model's true pose on
`/ground_truth/odom` (`ground_truth -> base_footprint`, 50 Hz). Only tests and
evaluation scripts read it; `test_sensor_topics.py` fails if a robot node
subscribes. Its own TF topic is never bridged.

### Turret

The type_04M_Ardu turret pans the camera. An STS3215 servo under the deck turns
a 45T gear platform through a 37T pinion (1.216 : 1).

- **Joint.** `turret_pan_joint` is revolute about +Z at the pan axis on the
  deck top. Its URDF limit is the ±135° software limit, inside the ±138° hard
  stops. Gazebo enforces the URDF limit; it ignores ros2_control command
  bounds, which is why the limit lives there.
- **Control.** `turret_controller` (`position_controllers/JointGroupPositionController`)
  takes one angle in radians on `/turret_controller/commands`. The sim
  reaches it almost at once; the real servo turns at about 3.9 rad/s at the
  platform.
- **Mesh split.** The pod, servo, pinion and post are fixed and in `body.stl`.
  The gear, platform, cradle and camera envelope turn, in `turret.stl`.

### ToF camera

The camera is an Arducam ToF (B0410). It has no RGB and stands upright in the
turret cradle, looking along the turret's +X. Its lens front is (17, 0, 44.5) mm
from the pan joint, about 102 mm off the ground.

- **Sensor type.** Gazebo simulates it as an `rgbd_camera`. Its pixels are
  square, so the image is 240 × 136 instead of the real 240 × 180, which keeps
  both the real 62.8° and 37.9° fields.
- **Cloud frame.** Gazebo's cloud is x-forward but stamped with the optical
  frame, so `tof_camera_points` restamps it into `tof_camera_lens_link`.
- **Estimated dimensions.** The camera envelope, lens depth and lens position
  are estimates, in the CAD as well as here. Measuring the B0410 replaces them.
- **Real driver.** Arducam's own ROS 2 node publishes no `Image` or
  `CameraInfo`, and hard-codes the frame `map`. The real camera will need a
  node of our own.

## Simulation launch

`simulation.launch.py` runs in this order:

1. Resolve the world and write a copy to `/tmp/mobile_base_world_<pid>_<engine>.sdf`
   with the physics step and render engine applied.
2. Run `generate_sim_sdf.py`, which expands the Xacro and adds the mecanum
   wheel contact (below), then writes `/tmp/mobile_base_sim_<pid>.sdf`.
3. Spawn the robot, then `joint_state_broadcaster`, then `mobile_base_controller`,
   `turret_controller` and RViz.
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
| `start_controller` | `true` | spawn `mobile_base_controller` (`turret_controller` always starts) |
| `seed` | `1` | Gazebo RNG seed; sets the IMU bias draw |
| `imu_bias` | `true` | remove the gyro bias learnt at standstill |
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

## State estimation

`robot_localization`'s `ekf_node` (`config/ekf.yaml`), `two_d_mode`, 50 Hz:

- **Wheels:** vx and vy from `/mobile_base_controller/odometry`. Wheel yaw is
  not fused: it reads turns about 4.5 % long.
- **IMU:** vyaw from `/imu/data_unbiased`. Orientation is never fused (in
  sim it is Gazebo truth).
- **Gyro bias:** `robot_localization` has no bias state, so `imu_bias` learns
  gyro z bias whenever the wheel odometry says the robot is stopped (after a
  0.5 s settle, at least 5 s in total, rates above 0.02 rad/s rejected) and
  subtracts it. The same node runs on the real robot, behind the BNO085's
  own on-chip calibration. `imu_bias:=false` passes the gyro through, for
  comparison.
- **Output:** `/odometry/filtered` and `odom -> base_footprint`.
- **Evaluation:** `odom_eval` scores it against `/ground_truth/odom`; it is
  the one node allowed to read the truth.

## Where to start

| Task | Start here |
| --- | --- |
| Change robot geometry | `properties.xacro`, then `test_mecanum_wheels.py` and the controller kinematics |
| New CAD export | `import_rev_d_meshes.py <Rev_D_T4M_ardu> meshes/rev_d`, then check `properties.xacro` against the READMEs |
| Change the turret | `properties.xacro` (`turret_*`), `turret.xacro`, `turret_controller` in `controllers.yaml` |
| Change wheel contact | `generate_sim_sdf.py`, then `test_mecanum_wheel_contact.py` |
| Move or retune a sensor | `properties.xacro` (pose, range, rate), the sensor's `*.gazebo.xacro`, then `test_sensor_topics.py` |
| Add a sensor | description Xacro, Gazebo Xacro, a bridge entry in `simulation.launch.py`, a check in `test_sensor_topics.py`; a new gz system goes in both worlds and `test_worlds.py` |
| Change which ToF rows are kept | `rev_base_tools/config/tof_rows.yaml` |
| Change the odometry fusion | `config/ekf.yaml` or `imu_bias.py`, then `odom_eval` |
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
