import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLD_NAME = "zero_to_robot_track"


def parse_teams(text: str, known: dict) -> list[int]:
    teams = [int(t) for t in text.replace(" ", "").split(",") if t]
    bad = [t for t in teams if t not in known]
    if bad or not teams:
        raise RuntimeError(f"teams must be from {sorted(known)}, got '{text}'")
    if len(set(teams)) != len(teams):
        raise RuntimeError(f"a team appears twice in '{text}'")
    return teams


def setup(context: LaunchContext) -> list:
    desc_share = get_package_share_directory("ztr_description")
    sim_share = get_package_share_directory("ztr_gazebo")
    with open(os.path.join(desc_share, "config", "teams.yaml")) as f:
        team_cfg = yaml.safe_load(f)["teams"]

    teams = parse_teams(LaunchConfiguration("teams").perform(context), team_cfg)
    max_speed = float(LaunchConfiguration("max_speed").perform(context))
    max_turn = float(LaunchConfiguration("max_turn").perform(context))
    timeout = float(LaunchConfiguration("timeout").perform(context))
    follow = LaunchConfiguration("follow").perform(context).lower() == "true"
    prefix = LaunchConfiguration("topic_prefix").perform(context)
    car_xacro = os.path.join(desc_share, "urdf", "ztr_car.sdf.xacro")

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
            launch_arguments={
                "gz_args": "-r " + os.path.join(sim_share, "worlds", "race_track.sdf")}.items(),
        )
    ]

    bridge_args = ["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"]
    bridge_remaps = []
    for n in teams:
        name = f"team{n}"
        cfg = team_cfg[n]
        sdf = xacro.process_file(car_xacro, mappings={
            "name": name, "color": cfg["color"], "max_speed": str(max_speed),
            "collide_bitmask": str(1 << n)}).toxml()
        actions.append(Node(
            package="ros_gz_sim", executable="create", output="screen",
            arguments=["-world", WORLD_NAME, "-name", name, "-string", sdf,
                       "-x", str(cfg["x"]), "-y", str(cfg["y"]), "-z", "0.05"],
        ))
        actions.append(Node(
            package="ztr_gazebo", executable="cmd_vel_guard.py", namespace=name,
            name="cmd_vel_guard", output="screen",
            parameters=[{"max_linear": max_speed, "max_angular": max_turn,
                         "timeout_s": timeout}],
        ))
        bridge_args += [
            f"/model/{name}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            f"/model/{name}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        ]
        bridge_remaps += [(f"/model/{name}/cmd_vel", f"/{name}/cmd_vel_safe"),
                          (f"/model/{name}/odometry", f"/{name}/odom")]

    actions.append(Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="ztr_bridge",
        output="screen", arguments=bridge_args, remappings=bridge_remaps,
    ))

    if prefix:
        actions.append(Node(
            package="ztr_gazebo", executable="mqtt_joystick_bridge.py", name="mqtt_joystick_bridge",
            output="screen",
            parameters=[{"topic_prefix": prefix, "teams": teams,
                         "max_speed": max_speed, "max_turn": max_turn}],
        ))

    if follow:
        name = f"team{teams[0]}"
        actions.append(TimerAction(period=8.0, actions=[
            ExecuteProcess(cmd=["ign", "service", "-s", "/gui/follow",
                                "--reqtype", "ignition.msgs.StringMsg",
                                "--reptype", "ignition.msgs.Boolean",
                                "--timeout", "3000", "--req", f'data: "{name}"']),
            ExecuteProcess(cmd=["ign", "service", "-s", "/gui/follow/offset",
                                "--reqtype", "ignition.msgs.Vector3d",
                                "--reptype", "ignition.msgs.Boolean",
                                "--timeout", "3000", "--req", "x: -9.0, y: 0.0, z: 4.0"]),
        ]))
    return actions


def generate_launch_description() -> LaunchDescription:
    desc_models = os.path.join(get_package_share_directory("ztr_description"), "models")
    sim_models = os.path.join(get_package_share_directory("ztr_gazebo"), "models")
    return LaunchDescription([
        DeclareLaunchArgument("teams", default_value="1,2,3,4,5,6",
                              description="Comma-separated team numbers (1-6)"),
        DeclareLaunchArgument("max_speed", default_value="5.0",
                              description="Car speed limit, m/s"),
        DeclareLaunchArgument("max_turn", default_value="1.0",
                              description="Turn-rate limit, rad/s"),
        DeclareLaunchArgument("timeout", default_value="0.7",
                              description="Stop a car after this many seconds without commands"),
        DeclareLaunchArgument("topic_prefix", default_value="",
                              description="MQTT topic prefix of the ESP32 joysticks (empty = no MQTT)"),
        DeclareLaunchArgument("follow", default_value="false",
                              description="Chase camera behind the first team's car"),
        AppendEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", desc_models),
        AppendEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", sim_models),
        OpaqueFunction(function=setup),
    ])
