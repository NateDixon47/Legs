#!/usr/bin/env python3
"""Bring up the bipedal robot in MuJoCo via ros2_control.

  robot_state_publisher  -> /robot_description, /tf
  mujoco ros2_control_node (loads scene.xml + robot_description, steps MuJoCo)
  spawners: joint_state_broadcaster, position_controller

Control the legs:
  ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray \
    "data: [0.0, 0.5, 1.0, 0.0, 0.5, 1.0]"
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg = FindPackageShare("legs_description")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg, "urdf", "legs_feet.urdf"]),
            " headless:=",
            LaunchConfiguration("headless"),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content.perform(context), value_type=str
        )
    }

    controllers = PathJoinSubstitution([pkg, "config", "controllers.yaml"])
    mujoco_plugins = PathJoinSubstitution([pkg, "config", "mujoco_ros2_control_plugins.yaml"])

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
        ),
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            emulate_tty=True,
            output="both",
            parameters=[
                {"use_sim_time": True},
                ParameterFile(controllers),
                ParameterFile(mujoco_plugins),
            ],
            # On Humble the controller_manager reads the URDF from /robot_description.
            remappings=(
                [("~/robot_description", "/robot_description")]
                if os.environ.get("ROS_DISTRO") == "humble"
                else []
            ),
        ),
    ]

    # Spawn both controllers in a single sequential call. Two parallel spawners
    # race on the controller-switch and trip a STRICT-switch abort.
    nodes.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "position_controller",
                "--param-file",
                controllers,
            ],
            output="both",
        )
    )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run MuJoCo without the Simulate window",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
