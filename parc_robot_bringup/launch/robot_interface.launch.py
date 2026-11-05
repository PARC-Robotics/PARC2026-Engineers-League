import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_path = FindPackageShare(package="parc_robot_bringup").find("parc_robot_bringup")
    bridge_params = os.path.join(pkg_path, "config/gz_bridge.yaml")
    use_sim_time = {"use_sim_time": True}

    declare_args = [
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

    gz_ros_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={bridge_params}"],
        output="screen",
    )

    depth_converter = Node(
        package="parc_robot_bringup",
        executable="depth_converter.py",
        output="screen",
        parameters=[use_sim_time],
    )

    def depth_scan_node(camera):
        return Node(
            package="depthimage_to_laserscan",
            executable="depthimage_to_laserscan_node",
            name=f"depthimage_to_laserscan_{camera}",
            output="screen",
            parameters=[
                use_sim_time,
                {
                    "output_frame": f"{camera}_depth_frame",
                    "range_min": 0.3,
                    "range_max": 8.0,
                    "scan_height": 10,
                },
            ],
            remappings=[
                ("depth", f"/{camera}/{camera}/depth/image_rect_raw"),
                ("depth_camera_info", f"/{camera}/{camera}/depth/camera_info"),
                ("scan", f"/{camera}/scan"),
            ],
        )

    scan_filter = Node(
        package="parc_robot_bringup",
        executable="scan_filter.py",
        output="screen",
        parameters=[use_sim_time],
    )

    scan_merger = Node(
        package="parc_robot_bringup",
        executable="scan_merger.py",
        output="screen",
        parameters=[use_sim_time],
    )

    respeaker = Node(
        package="parc_robot_bringup",
        executable="respeaker_sim.py",
        output="screen",
        condition=IfCondition(LaunchConfiguration("audio")),
        parameters=[
            {
                "source": LaunchConfiguration("audio_source"),
                "device": LaunchConfiguration("audio_device"),
                "wav_file": LaunchConfiguration("audio_file"),
            }
        ],
    )

    ld = LaunchDescription(declare_args)
    for action in (
        gz_ros_bridge,
        depth_converter,
        depth_scan_node("camera1"),
        depth_scan_node("camera2"),
        scan_filter,
        scan_merger,
        respeaker,
    ):
        ld.add_action(action)
    return ld
