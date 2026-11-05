"""Launch the randomised arena: tag and obstacle layout generated from `seed`."""

import os
import random
import tempfile

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from parc_robot_bringup.world_generator import START, generate, write_world

RANDOM_SEED = -1


def generate_launch_description():
    pkg_path = FindPackageShare(package="parc_robot_bringup").find("parc_robot_bringup")
    pkg_description = FindPackageShare(package="parc_robot_description").find(
        "parc_robot_description"
    )
    pkg_ros_gz_sim = FindPackageShare(package="ros_gz_sim").find("ros_gz_sim")

    rviz_config_file = os.path.join(pkg_path, "rviz/task.rviz")
    set_env_vars_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(pkg_path, "models")
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        ),
        DeclareLaunchArgument(
            "seed",
            default_value="1",
            description=f"World seed; {RANDOM_SEED} picks a random seed and logs it",
        ),
        DeclareLaunchArgument(
            "world_dir",
            default_value=os.path.join(tempfile.gettempdir(), "parc_worlds"),
            description="Directory for the generated world and its ground-truth JSON",
        ),
        DeclareLaunchArgument(
            "gui", default_value="true", description="Show the Gazebo window"
        ),
        DeclareLaunchArgument(
            "rviz", default_value="true", description="Start RViz"
        ),
        DeclareLaunchArgument(
            "audio",
            default_value="true",
            description="Publish the ReSpeaker topics from the host microphone",
        ),
        DeclareLaunchArgument(
            "audio_source",
            default_value="mic",
            description="Audio source: mic, file or silence",
        ),
        DeclareLaunchArgument(
            "audio_device",
            default_value="default",
            description="ALSA/Pulse capture device passed to arecord -D",
        ),
        DeclareLaunchArgument(
            "audio_file",
            default_value="",
            description="16 kHz mono 16-bit wav looped when audio_source is file",
        ),
    ]

    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_description, "launch", "robot_state_publisher.launch.py")]
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    start_robot_interface_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_path, "launch", "robot_interface.launch.py")]
        ),
        launch_arguments={
            "audio": LaunchConfiguration("audio"),
            "audio_source": LaunchConfiguration("audio_source"),
            "audio_device": LaunchConfiguration("audio_device"),
            "audio_file": LaunchConfiguration("audio_file"),
        }.items(),
    )

    def spawn_world_and_robot(context):
        seed = int(LaunchConfiguration("seed").perform(context))
        if seed == RANDOM_SEED:
            seed = random.randrange(1, 1_000_000)
        world = generate(seed)
        world_path, truth_path = write_world(
            world, LaunchConfiguration("world_dir").perform(context)
        )

        headless = LaunchConfiguration("gui").perform(context).lower() == "false"
        gz_args = (
            f"-r -s --headless-rendering -v2 {world_path}"
            if headless
            else f"-r -v4 {world_path}"
        )

        return [
            LogInfo(msg=f"World seed {seed}: {world_path} (ground truth: {truth_path})"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")]
                ),
                launch_arguments={
                    "gz_args": gz_args,
                    "on_exit_shutdown": "true",
                }.items(),
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-name",
                    "sitoe_robot",
                    "-x",
                    str(START[0]),
                    "-y",
                    str(START[1]),
                    "-z",
                    "0.0",
                    "-Y",
                    str(START[2]),
                ],
            ),
        ]

    set_gazebo_camera_view_cmd = ExecuteProcess(
        cmd=[
            "gz",
            "service",
            "-s",
            "/gui/move_to/pose",
            "--reqtype",
            "gz.msgs.GUICamera",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "2000",
            "--req",
            "pose: {position: {x: -0.2599, y: -9.7375, z: 3.8379} orientation: {x: -0.20186, y: 0.2022, z: 0.6772, w: 0.6780}}",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    ld = LaunchDescription(declare_args)
    ld.add_action(set_env_vars_resources)
    ld.add_action(start_rviz_cmd)
    ld.add_action(TimerAction(period=4.0, actions=[set_gazebo_camera_view_cmd]))
    ld.add_action(OpaqueFunction(function=spawn_world_and_robot))
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_robot_interface_cmd)
    return ld
