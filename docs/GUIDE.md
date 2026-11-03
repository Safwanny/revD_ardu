# GUIDE — building, running and inspecting Rev D

Every command lives here. [ARCHITECTURE.md](../ARCHITECTURE.md) is the reference
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
the rest) are listed in [ARCHITECTURE.md](../ARCHITECTURE.md#simulation-launch).

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

Run teleop in its own terminal, since it needs keyboard focus. Commands go
straight to the controller as stamped body-frame twists:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true \
  -p frame_id:=base_link \
  -p speed:=0.15 \
  -p turn:=0.5 \
  -p use_sim_time:=true \
  -r cmd_vel:=/mobile_base_controller/reference
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
| `/tof/<face>/points` | full 8 × 8 grid per ToF sensor, organised, misses non-finite |
| `/tof/<face>/rows` | the kept rows only (4, 5, 6), 3 × 8 |
| `/tof/rays` | RViz markers: one faint ray per kept zone, hits brighter than misses |
| `/tof_camera/depth_image` | camera depth, 32FC1 metres |
| `/tof_camera/depth_colour` | the same, colour-graded, bgr8 |
| `/tof_camera/points` | camera cloud in `tof_camera_lens_link` |
| `/mobile_base_controller/odometry` | wheel odometry |
| `/imu/data` | IMU, 100 Hz in `imu_link`; rates and accelerations only |
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
| `rev_base_tools/test/` | ToF grid order, row selection, depth colouring, `stop_simulation` matching |

`test_sensor_topics.py` runs on its own `ROS_DOMAIN_ID` and `GZ_PARTITION`, set
in `rev_base_bringup/CMakeLists.txt`, and needs a GPU for `ogre2`. CI excludes
it.
