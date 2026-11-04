# GUIDE — building, running and inspecting Rev D

Every command lives here. [ARCHITECTURE.md](ARCHITECTURE.md) is the reference
for frames, topics and dimensions.

## Two rules before anything else

**Render with `ogre2`.** The legacy Ogre1 backend gets range sensors wrong while
the clouds and timestamps still look valid. Any range read off an Ogre1 run is
not evidence.

**Stop cleanly when you are done.** Leftover Gazebo servers hold memory until
unrelated things start failing:

```bash
ros2 run rev_base_tools stop_simulation
```

It matches on full command lines, kills launch parents first, and clears the
generated `/tmp/mobile_base_*` files. `--dry-run` lists what it would stop.

## Build

```bash
cd ~/ros2_ws
rosdep install --from-paths src/rev_mobile_base --ignore-src -r -y
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to rev_base_bringup \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Pin `Python3_EXECUTABLE`. Without it CMake can pick up `~/.local/bin/python3.12`,
which has no `catkin_pkg`.

The prototype in `~/ros2_ws/src/mobile_base` publishes the same topic names. Run
only one robot per ROS domain.

## Run the simulation

```bash
ros2 launch rev_base_bringup simulation.launch.py world:=my_world
```

This opens three windows:
- the Gazebo window
- RViz, with `odom` as the fixed frame
- one window with the ToF camera depth colour-graded: near is red, far is blue,
  no return is black

The arguments (`gui`, `rviz`, `camera_windows`, `tof_markers`, spawn pose, and
the rest) are listed in [ARCHITECTURE.md](ARCHITECTURE.md#simulation-launch).

To view the model alone, without Gazebo, with sliders for the four wheels and
the turret (`use_joint_state_gui:=false` holds them at zero):

```bash
ros2 launch rev_base_description display.launch.py
```

To reopen the depth window on a stack that is already running:

```bash
ros2 launch rev_base_bringup camera_view.launch.py
```

## Drive it

Run teleop in its own terminal, since it needs keyboard focus. Commands are
stamped body-frame twists on `/cmd_vel_teleop`, and go through the safety
chain (`twist_mux`, then `safety_governor`) to the controller:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true \
  -p frame_id:=base_link \
  -p speed:=0.15 \
  -p turn:=0.5 \
  -p use_sim_time:=true \
  -r cmd_vel:=/cmd_vel_teleop
```

| Key | Motion |
| --- | --- |
| `i` / `,` | forward / reverse |
| `j` / `l` | rotate in place |
| `J` / `L` (Shift) | strafe left / right |
| `U` / `O` / `M` / `>` (Shift) | diagonals |
| `k` or any unmapped key | stop |

Lowercase `u`, `o`, `m` and `.` mix driving with rotation; they are not the
mecanum diagonals.

Teleop sends one message per keypress and never sends anything when a key is
released. The robot keeps its last command until the controller's 0.5 s
`reference_timeout` runs out, which is about 7.5 cm at 0.15 m/s. Press `k` to
stop on purpose.

The safety chain caps speed at 0.45 m/s and slows only motion that closes on
something the ToF ring sees, so the robot stops 3 cm short of it; driving
away is never limited. A wall within 5 cm of the side caps speed at
0.27 m/s. If a ToF face goes quiet, motion toward it creeps after 0.2 s and
stops after 0.75 s. E-stop and release:

```bash
ros2 topic pub --once /e_stop std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /e_stop std_msgs/msg/Bool "{data: false}"
```

Compare `/cmd_vel_mux` with `/mobile_base_controller/reference` to see what
the governor changed.

## Score the odometry

Drives a 60 s standstill, three laps of a 2 m square (drive, stop, turn on the
spot) and three laps strafed while rotating, then prints the EKF's and the
wheels' end errors against Gazebo truth (ADR 0003 thresholds) and exits 0 on
a pass. Spawn it with room for the square:

```bash
ros2 launch rev_base_bringup simulation.launch.py x:=-4.0 y:=-3.2
```

```bash
ros2 run rev_base_tools odom_eval --ros-args -p use_sim_time:=true -p output_dir:=odom_eval
```

`output_dir` gets `report.txt` and one 10 Hz TUM file per run and source.
Plot them, or score them with evo:

```bash
ros2 run rev_base_tools odom_eval_plot odom_eval
```

```bash
evo_ape tum odom_eval/square_truth.tum odom_eval/square_ekf.tum --align_origin
```

Launch with `seed:=<n>` for another IMU bias draw, and `imu_bias:=false` to
score the EKF on the raw gyro.

## Pan the turret

Angle in radians, positive to the left, limited to ±135° (±2.356 rad):

```bash
ros2 topic pub --once /turret_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.8]}"
```

The camera, its cloud and its depth image turn with it.

## Look at the sensors

| Topic | What it is |
| --- | --- |
| `/tof/<face>/points` | full 8 × 8 grid per ToF sensor, organised, misses and dropped zones NaN |
| `/tof/<face>/rows` | the kept rows only (4, 5, 6), 3 × 8 |
| `/tof/rays` | RViz markers: one faint ray per kept zone, hits brighter than misses |
| `/tof_camera/depth_image` | camera depth, 32FC1 metres |
| `/tof_camera/depth_colour` | the same, colour-graded, bgr8 |
| `/tof_camera/points` | camera cloud in `tof_camera_lens_link` |
| `/mobile_base_controller/odometry` | wheel odometry |
| `/imu/data` | IMU, 100 Hz in `imu_link`; rates and accelerations only |
| `/imu/data_unbiased` | the same with the standstill gyro z bias removed; the EKF's input |
| `/odometry/filtered` | EKF odometry |
| `/ground_truth/odom` | Gazebo true pose, for evaluation only |
| `/joint_states` | four wheels and `turret_pan_joint` |

`<face>` is `front`, `rear`, `left` or `right`.

```bash
ros2 topic hz /tof_camera/depth_image
ros2 topic hz /tof/front/rows
ros2 topic echo --once /tof_camera/camera_info
ros2 run tf2_ros tf2_echo odom base_footprint
```

`image_view` freezes on 32FC1 depth, so look at `/tof_camera/depth_colour` in
`rqt_image_view` instead.

To change which ToF rows are kept, edit `rev_base_tools/config/tof_rows.yaml`
and relaunch.

## Tests

Run them sequentially. In parallel, the Gazebo launch test competes for memory
and fails for reasons that look unrelated:

```bash
cd ~/ros2_ws
colcon test --executor sequential --packages-select \
  rev_base_description rev_base_gazebo rev_base_tools rev_base_bringup
colcon test-result --verbose
```

| Test | Checks |
| --- | --- |
| `test_mecanum_wheels.py` | wheel positions, axes, meshes and hands against Rev D geometry; turret joint and limits |
| `test_worlds.py` | each world parses, needs no downloads, loads the IMU system and renders with `ogre2` |
| `test_mecanum_wheel_contact.py` | the generated SDF carries the directional wheel friction |
| `test_sensor_topics.py` | launches Gazebo headless and checks every sensor topic, the IMU rate and the odometry TF; no robot node reads `/ground_truth/odom`; a turret command moves the pan joint |
| `test_safety_chain.py` | launches Gazebo headless and drives through the safety chain: forward to a wall at the speed cap and stops short, slides along it at the side cap into a corner, gets out both ways, diagonal in and out, spins in the open; the e-stop holds |
| `rev_base_tools/test/` | safety governor (braking speed, swept path, clearance at an angle, turning, silent faces), ToF noise and dropout, grid order, row selection, depth colouring, `stop_simulation` matching, odometry scoring, gyro bias estimation |

`test_sensor_topics.py` and `test_safety_chain.py` each run on their own
`ROS_DOMAIN_ID` and `GZ_PARTITION`, set in `rev_base_bringup/CMakeLists.txt`,
and need a GPU for `ogre2`. CI excludes them.
