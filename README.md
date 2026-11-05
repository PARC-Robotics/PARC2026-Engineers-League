# PARC 2026 Engineers League: Simulation

Gazebo simulation of the CAYTU SITO-E robot for the Pan-African Robotics Competition (PARC) Engineers League
2026. The robot drives to four AprilTags in an order given by the referee, avoiding obstacles. At each tag it
stops 0.5 m away, faces it, and reports the tag number with a camera image.

The simulated robot publishes the same topic names as the physical robot, so a solution developed here runs on
the real robot unchanged. Arenas are either fixed or generated from a seed.

- [Requirements](#requirements)
- [Build](#build)
- [Quick start](#quick-start)
- [The task](#the-task)
- [Launch files](#launch-files)
- [Randomised arena](#randomised-arena)
- [Robot interface](#robot-interface)
- [Microphone](#microphone)
- [Referee](#referee)
- [Simulation speed](#simulation-speed)
- [Troubleshooting](#troubleshooting)
- [Repository layout](#repository-layout)
- [References](#references)

## Requirements

- Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic (`ros-jazzy-ros-gz`).
- A GPU for realistic simulation speed (see [Simulation speed](#simulation-speed)).
- `alsa-utils` for microphone capture.

Install the ROS dependencies of the packages with rosdep:

```bash
cd ~/parc_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Build

The repository is cloned into the `src` directory of a colcon workspace. It contains four packages:
`parc_robot_description`, `parc_robot_bringup`, and the referee packages `parc_msgs` and `parc_referee`.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/parc_ws
colcon build --symlink-install
source install/setup.bash
```

## Quick start

```bash
# Use the GPU for rendering (WSL2; see Simulation speed)
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA GALLIUM_DRIVER=d3d12

# Randomised arena, reproducible from the seed
ros2 launch parc_robot_bringup phase2_hard.launch.py seed:=7

# In a second terminal: the referee and its dashboard at http://localhost:8080/
ros2 launch parc_referee referee.launch.py odom_topic:=/botwheel_explorer/odom
```

Drive the robot to check that everything is connected:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
```

## The task

A run is coordinated through five topics under `/parc/`. Message definitions are in `parc_msgs`.

| Topic | Direction | Type | When |
| --- | --- | --- | --- |
| `/parc/tag_order` | referee to robot | `parc_msgs/TagOrder` | The judges arm the run. Latched. |
| `/parc/start` | robot to referee | `parc_msgs/StartSignal` | Once, when the robot starts. Starts the clock. |
| `/parc/tag_visit` | robot to referee | `parc_msgs/TagVisit` | Once per tag, while stopped at the tag. |
| `/parc/finish` | robot to referee | `parc_msgs/Finish` | Only to end the run early. |
| `/parc/run_status` | referee to robot | `parc_msgs/RunStatus` | Twice a second and on every change. Latched. |

1. The robot waits for `/parc/tag_order`. Any movement before the start signal is a false start.
2. The robot publishes one `StartSignal`, set to the voice method when the start word (`go`, `allez` or `vai`)
   was spoken.
3. The robot visits the tags in the given order, avoiding obstacles. At each tag it stops 0.5 m from the tag,
   facing it within 12 degrees, and publishes one `TagVisit` with the tag ID (1, 4, 7 or 9) and a JPEG
   `CompressedImage` in which the tag is clearly visible. It then holds still for at least 3 s.
4. The run ends when the last tag in the order is reported, when `Finish` is published, after 10 minutes, or when
   a judge stops it. The robot stops when `/parc/run_status` shows `FINISHED`, `TIMED_OUT` or `TERMINATED`.

The robot publishes raw images only, so a solution encodes the JPEG itself, for example with
`cv2.imencode('.jpg', image)`. Judges decide whether a visit scores.

## Launch files

All launch files are in `parc_robot_bringup/launch`.

| Launch file | Arena | Start pose |
| --- | --- | --- |
| `phase2_hard.launch.py` | Randomised from `seed` | (0, -7), facing +y |
| `phase2.launch.py` | Fixed phase 2 arena, `worlds/nba_sim.sdf`: four tags at the wall centres, three tables with chairs | `config/phase2_params.yaml` |
| `task.launch.py` | Cafe scene, `worlds/task_world.sdf`, with a goal marker | `config/task_params.yaml` |
| `robot_interface.launch.py` | None. Starts the Gazebo bridge and the converters, scan nodes and microphone node. Included by the three launch files above. | |

RViz opens with the robot model, TF, the lidar scan, a coloured point cloud of the top camera's depth
(`DepthCloud`), and image views of the top camera colour, top camera depth and bottom camera colour.

Arguments:

| Argument | Default | Launch files | Description |
| --- | --- | --- | --- |
| `seed` | `1` | `phase2_hard` | World seed. `-1` picks a random seed and logs it. |
| `world_dir` | `/tmp/parc_worlds` | `phase2_hard` | Output directory for the generated world and ground truth. |
| `gui` | `true` | `phase2_hard` | `false` runs Gazebo without a window. |
| `rviz` | `true` | `phase2_hard` | `false` skips RViz. |
| `use_sim_time` | `true` | all | Use the Gazebo clock. |
| `audio` | `true` | all | Publish the microphone topics. |
| `audio_source` | `mic` | all | `mic`, `file` or `silence`. |
| `audio_device` | `default` | all | Capture device passed to `arecord -D`. |
| `audio_file` | empty | all | Wav file looped when `audio_source` is `file`. |

## Randomised arena

The generator in `parc_robot_bringup/world_generator.py` builds a 7 m by 6 m walled arena (x from -3.5 to 3.5,
y from -8 to -2) from a seed. The same seed always gives the same arena.

- **Tags.** AprilTag 36h11 IDs 1, 4, 7 and 9 are 0.15 m wide with their centre 1.0 m above the floor, on a
  0.25 m white panel on a post. Each tag is either on a wall or on a free-standing post facing a random
  direction. Tags are at least 2.2 m apart and at least 1.8 m from the start.
- **Obstacles.** Twelve static everyday objects are placed between the tags: three cafe tables with chairs,
  some with a seated visitor; wall-hugging furniture (sofa, bookshelf, cabinet, vending machine); and
  free-standing items (cartons, bin, plant, stool, suitcase). Heights range from 0.4 m cartons to the 1.8 m
  vending machine.
- **Reachability.** Every layout is checked. The visit pose 0.5 m in front of each tag and the approach 1.0 m in
  front of it are reachable from the start with 0.40 m of free space around the robot centre. The mean ratio of
  path length to straight-line distance between the start and the visit poses is at least 1.2, so driving in
  straight lines does not work.
- **Output.** `parc_random_seed<N>.sdf` is the world. `parc_random_seed<N>.json` is the ground truth: tag poses,
  visit poses (position and the yaw that faces the tag) and obstacles. The JSON is for judging and testing and
  is not given to solutions.

Generate a world without starting Gazebo:

```bash
python3 -m parc_robot_bringup.world_generator --seed 7 --out /tmp/parc_worlds
```

## Robot interface

The simulated robot publishes these topics. `camera1` is the top RealSense and `camera2` the bottom one.

| Topic | Type | Notes |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | Drive command. |
| `/botwheel_explorer/odom` | `nav_msgs/Odometry` | Frame `odom`, child frame `base_footprint`. |
| `/scan` | `sensor_msgs/LaserScan` | Frame `laser`. Includes returns from the robot's own body. |
| `/scan_filtered` | `sensor_msgs/LaserScan` | `/scan` without returns closer than 0.3 m. |
| `/scan_merged` | `sensor_msgs/LaserScan` | `/scan_filtered` merged with `/camera1/scan` and `/camera2/scan`. |
| `/camera{1,2}/camera{1,2}/color/image_raw` | `sensor_msgs/Image` | `rgb8`, 640 by 480, 30 Hz. |
| `/camera{1,2}/camera{1,2}/color/camera_info` | `sensor_msgs/CameraInfo` | |
| `/camera{1,2}/camera{1,2}/depth/image_rect_raw` | `sensor_msgs/Image` | `16UC1`, millimetres, 0 for no return or a return closer than 0.3 m. Noise standard deviation 0.004 x distance² metres (1.6 cm at 2 m). |
| `/camera{1,2}/camera{1,2}/depth/camera_info` | `sensor_msgs/CameraInfo` | |
| `/camera{1,2}/camera{1,2}/imu` | `sensor_msgs/Imu` | 200 Hz. |
| `/camera{1,2}/scan` | `sensor_msgs/LaserScan` | From the depth image. |
| `/audio`, `/audio/channel0` to `/audio/channel5`, `/speech_audio` | `audio_common_msgs/AudioData` | See [Microphone](#microphone). |
| `/is_speeching` | `std_msgs/Bool` | Latched. |
| `/sound_direction`, `/sound_localization` | `std_msgs/Int32`, `geometry_msgs/PoseStamped` | Latched. |
| `/joint_states`, `/tf`, `/tf_static`, `/robot_description`, `/clock` | | |

Topics under `/sim/` exist only in simulation and must not be used by solutions: `/sim/pose`, the four
`/sim/*_collisions` contact topics, and the depth images in metres (`/sim/camera{1,2}/depth/depth_image`).

The physical robot's obstacle-avoidance layer and its battery, LED, volume, network and WebRTC topics are not
simulated. `/scan_filtered` and `/scan_merged` approximate the robot's filter and merger nodes, whose exact
behaviour is not specified.

## Microphone

`scripts/respeaker_sim.py` runs as `respeaker_node` and publishes the ReSpeaker topics from the host computer's
microphone, so a solution can be tried with a real voice.

- Audio is 16 kHz, signed 16-bit little-endian, mono, in chunks of 1024 samples (15.6 messages per second).
- `/audio` and `/audio/channel0` carry the microphone signal. `channel1` to `channel4` repeat it as the raw
  microphone channels. `channel5` is silent.
- `/is_speeching` follows an energy-based speech detector that adapts to the background level. `/speech_audio`
  publishes the whole utterance once, after 0.5 s of silence.
- A single microphone has no direction, so `/sound_direction` is always the node's `doa_deg` parameter
  (default 0) and `/sound_localization` is the matching unit vector in `base_link`.

Capture uses `arecord`. Device names depend on the system; list them with `arecord -L`. The `pulse` device
needs the ALSA PulseAudio plugin (`libasound2-plugins`). When capture fails, the node logs the `arecord` error
and publishes silence.

```bash
# Real microphone
ros2 launch parc_robot_bringup phase2_hard.launch.py audio_device:=pulse

# Loop a recording (16 kHz, mono, 16-bit wav)
ros2 launch parc_robot_bringup phase2_hard.launch.py audio_source:=file audio_file:=/path/to/speech.wav

# No audio topics
ros2 launch parc_robot_bringup phase2_hard.launch.py audio:=false
```

Speech recognition and text-to-speech are not part of this repository; solutions provide their own.

## Referee

The referee node, `parc_referee`, with its messages in `parc_msgs`, publishes the tag order and run status,
records the run, and serves a dashboard at `http://localhost:8080/`. Judges use `http://localhost:8080/judge?token=<judge_token>`. The
default token, `parc2026`, is for testing only.

```bash
ros2 launch parc_referee referee.launch.py odom_topic:=/botwheel_explorer/odom judge_token:=<secret>
```

The referee reads odometry to detect false starts and movement during the 3 s hold. The robot's odometry
topic is `/botwheel_explorer/odom`, which differs from the referee's default of `/odom`, so the launch argument is
required. Run records and images are written to `~/parc_runs`.

`parc_referee` includes `example_team`, a node that shows the topic interface and can fake a run with
`ros2 run parc_referee example_team --ros-args -p demo:=true`. It subscribes to a `CompressedImage` topic, which
the robot does not publish. A solution subscribes to `/camera1/camera1/color/image_raw` instead and encodes each
visit image to JPEG.

## Simulation speed

Gazebo renders a camera stream only while something subscribes to it, so speed depends on which topics are in
use. The robot has two colour cameras, two depth cameras and a lidar. With the default software renderer under
WSL2 the full sensor set runs at about 0.01 times real time. Selecting the GPU through Mesa's Direct3D 12
driver raises this to about 0.4:

```bash
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA GALLIUM_DRIVER=d3d12
```

The real-time factor is shown in the Gazebo window, or with
`gz topic -e -t /world/<world name>/stats -n 1`. A real-time factor below 1 stretches simulated time: a 10-minute
run takes longer on the wall clock, and nodes must use `use_sim_time`.

## Troubleshooting

- **The simulation looks frozen, or topics publish rarely.** Check the real-time factor. See
  [Simulation speed](#simulation-speed).
- **Gazebo warns `Found additional publishers on /clock`, or two simulations interfere.** Another Gazebo is
  running on the same machine. Stop it, or give each its own partition with `export GZ_PARTITION=<name>` in every
  terminal that uses it.
- **No audio.** The node logs the `arecord` error. Check the device with `arecord -L`, or use
  `audio_source:=file` or `audio:=false`.
- **RViz reports `Expected topic type of 'sensor_msgs/msg/Image', saw topic type 'sensor_msgs/msg/Image'`.**
  This is an RViz bug in adding a `DepthCloud` display from the "By topic" tab. Add it from the "By display type"
  tab and type the depth topic into `Depth Map Topic`.
- **`Could not resolve file [Wood_Floor_Dark.jpg]` or `[Maple.jpg]`.** Two textures referenced by the cafe
  models are not in the repository. The simulation runs regardless.

## Repository layout

| Path | Contents |
| --- | --- |
| `parc_robot_bringup/launch` | Launch files. |
| `parc_robot_bringup/config` | `gz_bridge.yaml` (Gazebo to ROS topic map), start poses and goal for the fixed arenas. |
| `parc_robot_bringup/worlds` | Fixed arenas. |
| `parc_robot_bringup/models` | Gazebo models: tag textures, cafe furniture, visitors, goal marker. |
| `parc_robot_bringup/parc_robot_bringup/world_generator.py` | Seeded arena generator. |
| `parc_robot_bringup/scripts` | `depth_converter.py` (metres to millimetres, noise model), `scan_filter.py`, `scan_merger.py`, `respeaker_sim.py`. |
| `parc_robot_bringup/rviz` | RViz configuration. |
| `parc_robot_description` | Robot URDF (xacro), meshes, robot state publisher launch files. `sitoe_robot.urdf` and `sitoe_robot.sdf` are exported snapshots that the launch files do not use. |
| `parc_msgs` | Referee message definitions: `TagOrder`, `StartSignal`, `TagVisit`, `Finish`, `RunStatus`. |
| `parc_referee` | Referee node, web dashboard, `example_team` and the referee launch file. |
| `resources` | Template for the team submission README. |

## References

- [Cafe world](https://app.gazebosim.org/OpenRobotics/fuel/models/Cafe)
- [Other Gazebo models](https://app.gazebosim.org/models)
- [realsense_gazebo_plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)
- [realsense-ros](https://github.com/realsenseai/realsense-ros)
- [RobotCAD](https://github.com/drfenixion/freecad.robotcad)
