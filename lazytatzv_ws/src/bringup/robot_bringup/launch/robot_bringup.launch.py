// Copyright 2026 Tatsukiyano
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    # --- 1. SETTINGS & PATHS ---
    pkg_bringup = get_package_share_directory("robot_bringup")

    # Launch Configurations
    use_foxglove = LaunchConfiguration("use_foxglove")
    use_rviz = LaunchConfiguration("use_rviz")

    paths = {
        "phys": os.path.join(pkg_bringup, "config", "physical.yaml"),
        "mux": os.path.join(pkg_bringup, "config", "twist_mux.yaml"),
        "joy": os.path.join(pkg_bringup, "config", "teleop.yaml"),
        "life": os.path.join(pkg_bringup, "config", "lifecycle.yaml"),
        "urdf": os.path.join(pkg_bringup, "urdf", "robot.urdf"),
    }

    # Load actuator type
    with open(paths["phys"], "r") as f:
        phys_params = (
            yaml.safe_load(f).get("/kinematics_engine_node", {}).get("ros__parameters", {})
        )
        actuator_type = phys_params.get("actuator_type", "at")

    # Build managed nodes list
    dynamic_managed_nodes = []
    if actuator_type in ["at", "ddsm"]:
        dynamic_managed_nodes.append("/communication/serial_gateway")
    elif actuator_type == "can":
        dynamic_managed_nodes.append("/communication/usb_can_analyzer")

    for side in ["front_left", "front_right", "rear_left", "rear_right"]:
        dynamic_managed_nodes.append(f"/motors/{side}")

    # Load USER nodes and Merge
    with open(paths["life"], "r") as f:
        life_config = yaml.safe_load(f)
    user_nodes = (
        life_config.get("/lifecycle_manager_robot", {})
        .get("ros__parameters", {})
        .get("node_names", [])
    )
    total_managed_nodes = dynamic_managed_nodes + user_nodes

    print(f"\n[MASTER LAUNCH] Mode: {actuator_type.upper()}")

    # --- Setup Actuator Config ---
    if actuator_type == "at":
        m_pkg, m_plugin = "robstride_driver", "robstride_driver::RobstrideAtNode"
        act_yaml = os.path.join(pkg_bringup, "config", "actuators_robstride.yaml")
    elif actuator_type == "can":
        m_pkg, m_plugin = "robstride_driver", "robstride_driver::RobstrideCanNode"
        act_yaml = os.path.join(pkg_bringup, "config", "actuators_robstride.yaml")
    elif actuator_type == "ddsm":
        m_pkg, m_plugin = "ddsm115_ros2_driver", "ddsm115_ros2_driver::DDSM115DriverNode"
        act_yaml = os.path.join(pkg_bringup, "config", "actuators_ddsm.yaml")
    elif actuator_type == "virtual":
        m_pkg, m_plugin = "virtual_actuator", "virtual_actuator::VirtualActuatorNode"
        act_yaml = os.path.join(pkg_bringup, "config", "actuators_robstride.yaml")

    # --- Composable Nodes (Spinal Cord) ---
    control_nodes = [
        ComposableNode(
            package="mecanum_kinematics",
            plugin="mecanum_kinematics::MecanumKinematicsNode",
            name="mecanum_kinematics_node",
            parameters=[paths["phys"]],
        ),
        ComposableNode(
            package="mecanum_kinematics",
            plugin="mecanum_kinematics::WheelSpeedsDispatcher",
            name="speed_dispatcher",
            namespace="hal",
            parameters=[act_yaml],
        ),
    ]

    if actuator_type in ["at", "ddsm"]:
        control_nodes.append(
            ComposableNode(
                package="serial_gateway",
                plugin="serial_gateway::SerialGateway",
                name="serial_gateway",
                namespace="communication",
                parameters=[act_yaml],
            )
        )
    elif actuator_type == "can":
        control_nodes.append(
            ComposableNode(
                package="seeed_usb_can_analyzer_driver",
                plugin="seeed_usb_can_analyzer_driver::UsbCanAnalyzerNode",
                name="usb_can_analyzer",
                namespace="communication",
                parameters=[act_yaml],
            )
        )

    for side in ["front_left", "front_right", "rear_left", "rear_right"]:
        control_nodes.append(
            ComposableNode(
                package=m_pkg,
                plugin=m_plugin,
                name=side,
                namespace="motors",
                parameters=[act_yaml, {"joint_name": f"{side}_wheel_joint"}],
            )
        )

    control_container = ComposableNodeContainer(
        name="actuator_control_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=control_nodes,
        output="screen",
    )

    # --- Actions ---
    return [
        control_container,
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_robot",
            parameters=[{"autostart": True, "node_names": total_managed_nodes}],
        ),
        Node(package="joy", executable="joy_node", name="joy_node", parameters=[paths["joy"]]),
        Node(
            package="base_teleop",
            executable="base_teleop_node",
            name="teleop",
            parameters=[paths["joy"]],
            remappings=[("cmd_vel", "cmd_vel_joy")],
        ),
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            parameters=[paths["mux"]],
            remappings=[("cmd_vel_out", "cmd_vel")],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": open(paths["urdf"]).read()}],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_aggregator",
            parameters=[
                {
                    "source_list": [
                        f"/motors/{s}/joint_states"
                        for s in ["front_left", "front_right", "rear_left", "rear_right"]
                    ],
                    "rate": 50,
                }
            ],
        ),
        # Optional Visualization
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge_node",
            name="foxglove_bridge",
            condition=IfCondition(use_foxglove),
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2", condition=IfCondition(use_rviz)
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_foxglove", default_value="true", description="Launch Foxglove Bridge"
            ),
            DeclareLaunchArgument("use_rviz", default_value="false", description="Launch RViz2"),
            OpaqueFunction(function=launch_setup),
        ]
    )
