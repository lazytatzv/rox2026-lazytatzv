import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):
    pkg_bringup = get_package_share_directory('robot_bringup')
    
    # Config Paths
    physical_params = os.path.join(pkg_bringup, 'config', 'physical.yaml')
    twist_mux_params = os.path.join(pkg_bringup, 'config', 'twist_mux.yaml')
    teleop_params = os.path.join(pkg_bringup, 'config', 'teleop.yaml')
    urdf_file = os.path.join(pkg_bringup, 'urdf', 'robot.urdf')

    with open(physical_params, 'r') as f:
        config = yaml.safe_load(f)
    actuator_type = config.get('/kinematics_engine_node', {}).get('ros__parameters', {}).get('actuator_type', 'at')

    # Actuator Selection
    if actuator_type == 'at':
        act_yaml = os.path.join(pkg_bringup, 'config', 'actuators_robstride.yaml')
        motor_pkg, motor_plugin = 'robstride_driver', 'robstride_driver::RobstrideMotorNode'
    elif actuator_type == 'ddsm':
        act_yaml = os.path.join(pkg_bringup, 'config', 'actuators_ddsm.yaml')
        motor_pkg, motor_plugin = 'ddsm115_ros2_driver', 'ddsm115_ros2_driver::DDSM115DriverNode'
    else: # can
        act_yaml = os.path.join(pkg_bringup, 'config', 'actuators_robstride.yaml')
        motor_pkg = 'el05_usb_can_driver'

    nodes = []
    
    # --- 1. CORE LAYER (Global) ---
    nodes.append(ComposableNode(package='joy', plugin='joy::JoyNode', name='joy_node', parameters=[teleop_params]))
    nodes.append(ComposableNode(package='base_teleop', plugin='base_teleop::BaseTeleopNode', name='teleop', 
                                parameters=[teleop_params], remappings=[('cmd_vel', 'cmd_vel_joy')]))
    nodes.append(ComposableNode(package='twist_mux', plugin='twist_mux::TwistMux', name='twist_mux', 
                                parameters=[twist_mux_params], remappings=[('cmd_vel_out', 'cmd_vel')]))
    nodes.append(ComposableNode(package='mecanum_kinematics', plugin='mecanum_kinematics::MecanumKinematicsNode', 
                                name='kinematics_engine', parameters=[physical_params]))

    # --- 2. HAL LAYER (/hal) ---
    nodes.append(ComposableNode(package='mecanum_kinematics', plugin='mecanum_kinematics::WheelSpeedsDispatcher', 
                                name='speed_dispatcher', namespace='hal', parameters=[act_yaml]))

    # --- 3. COMMUNICATION LAYER (/communication) ---
    if actuator_type in ['at', 'ddsm']:
        nodes.append(ComposableNode(package='serial_gateway', plugin='serial_gateway::SerialGateway', 
                                    name='serial_gateway', namespace='communication', parameters=[act_yaml]))

    if actuator_type == 'can':
        nodes.append(ComposableNode(package='seeed_usb_can_analyzer_driver', plugin='seeed_usb_can_analyzer_driver::UsbCanAnalyzerNode', 
                                    name='usb_can_analyzer', namespace='communication', parameters=[act_yaml]))

    # --- 4. MOTOR LAYER (/motors) ---
    if actuator_type in ['at', 'ddsm']:
        for side in ['front_left', 'front_right', 'rear_left', 'rear_right']:
            nodes.append(ComposableNode(package=motor_pkg, plugin=motor_plugin, name=side, namespace='motors', parameters=[act_yaml]))

    container = ComposableNodeContainer(
        name='robot_master_container', namespace='', package='rclcpp_components',
        executable='component_container', composable_node_descriptions=nodes, output='screen')

    launch_actions = [container]

    # --- 5. SYSTEM AGGREGATION & VISUALIZATION ---
    with open(urdf_file, 'r') as f: robot_desc = f.read()
    launch_actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher', 
                               name='robot_state_publisher', parameters=[{'robot_description': robot_desc}]))

    source_list = [f'/motors/{s}/joint_states' for s in ['front_left', 'front_right', 'rear_left', 'rear_right']]
    launch_actions.append(Node(package='joint_state_publisher', executable='joint_state_publisher', 
                               name='joint_aggregator', parameters=[{'source_list': source_list, 'rate': 50}]))

    launch_actions.append(Node(package='foxglove_bridge', executable='foxglove_bridge_node', name='foxglove_bridge'))

    if actuator_type == 'can':
        for side in ['front_left', 'front_right', 'rear_left', 'rear_right']:
            launch_actions.append(Node(package='el05_usb_can_driver', executable='el05_motor_node', 
                                       name=side, namespace='motors', parameters=[act_yaml]))

    return launch_actions

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
