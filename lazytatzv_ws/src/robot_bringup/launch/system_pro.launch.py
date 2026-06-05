import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):
    # パス取得
    package_bringup_share_directory = get_package_share_directory('robot_bringup')
    physical_params_path = os.path.join(package_bringup_share_directory, 'config', 'physical.yaml')
    twist_mux_configuration_path = os.path.join(package_bringup_share_directory, 'config', 'twist_mux.yaml')
    teleop_params_path = os.path.join(package_bringup_share_directory, 'config', 'teleop.yaml')

    # physical.yaml から actuator_type を読み込む
    with open(physical_params_path, 'r') as f:
        config = yaml.safe_load(f)
    
    # ノード名 "/kinematics_engine_node" の配下にあるパラメータを取得
    actuator_type = config.get('/kinematics_engine_node', {}).get('ros__parameters', {}).get('actuator_type', 'at')
    
    print(f"[LAUNCH] Actuator Type detected from YAML: {actuator_type}")

    # 動的なYAMLパス設定 (at -> actuators_robstride.yaml, ddsm -> actuators_ddsm.yaml)
    if actuator_type == 'at':
        actuator_params_path = os.path.join(package_bringup_share_directory, 'config', 'actuators_robstride.yaml')
    elif actuator_type == 'ddsm':
        actuator_params_path = os.path.join(package_bringup_share_directory, 'config', 'actuators_ddsm.yaml')
    else: # can
        actuator_params_path = os.path.join(package_bringup_share_directory, 'config', 'actuators_robstride.yaml')

    nodes = []

    # 1. 共通ノード (Joy, Teleop, TwistMux, Kinematics)
    nodes.append(ComposableNode(
        package='joy', plugin='joy::JoyNode', name='joystick_driver_node', parameters=[teleop_params_path]))
    nodes.append(ComposableNode(
        package='base_teleop', plugin='base_teleop::BaseTeleopNode', name='base_teleoperation_node',
        parameters=[teleop_params_path], remappings=[('cmd_vel', 'cmd_vel_joy')]))
    nodes.append(ComposableNode(
        package='twist_mux', plugin='twist_mux::TwistMux', name='velocity_command_multiplexer',
        parameters=[twist_mux_configuration_path], remappings=[('cmd_vel_out', 'cmd_vel')]))
    nodes.append(ComposableNode(
        package='mecanum_kinematics', plugin='mecanum_kinematics::MecanumKinematicsNode',
        name='kinematics_engine_node', parameters=[physical_params_path]))
    nodes.append(ComposableNode(
        package='mecanum_kinematics', plugin='mecanum_kinematics::WheelSpeedsDispatcher',
        name='wheel_speeds_dispatcher', parameters=[actuator_params_path]))

    # 2. ドライバ固有のノード
    if actuator_type == 'at' or actuator_type == 'ddsm':
        # Serial Gateway
        nodes.append(ComposableNode(
            package='serial_gateway', plugin='serial_gateway::SerialGateway',
            name='serial_gateway', parameters=[actuator_params_path]))
        
        motor_pkg = 'robstride_driver' if actuator_type == 'at' else 'ddsm115_ros2_driver'
        motor_plugin = 'robstride_driver::RobstrideMotorNode' if actuator_type == 'at' else 'ddsm115_ros2_driver::Ddsm115DriverNode'
        
        # 4輪モーターノード (AT/DDSM)
        for side in ['front_left', 'front_right', 'rear_left', 'rear_right']:
            nodes.append(ComposableNode(
                package=motor_pkg, plugin=motor_plugin, name=f'{side}_motor',
                parameters=[actuator_params_path]))

    elif actuator_type == 'can':
        # 先輩の CAN ドライバ構成 (Composable非対応の場合に備えて通常のNodeアクションも検討可能だが、ここではComposableを試みる)
        # usb_can_analyzer
        nodes.append(ComposableNode(
            package='seeed_usb_can_analyzer_driver', plugin='seeed_usb_can_analyzer_driver::UsbCanAnalyzerNode',
            name='usb_can_analyzer', parameters=[actuator_params_path]))
        
        # el05_usb_can_driver (Pythonノードなので別枠で追加)
        pass

    container = ComposableNodeContainer(
        name='robot_system_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=nodes,
        output='screen',
    )

    launch_actions = [container]

    # Joint State Aggregator (Unifies 4 motor states into a single /joint_states topic)
    source_list = [
        '/front_left_motor/joint_states',
        '/front_right_motor/joint_states',
        '/rear_left_motor/joint_states',
        '/rear_right_motor/joint_states'
    ]
    launch_actions.append(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_aggregator',
        parameters=[{'source_list': source_list, 'rate': 50}],
    ))

    # Python版ドライバ (CAN版など) の追加
    if actuator_type == 'can':
        for side in ['front_left', 'front_right', 'rear_left', 'rear_right']:
            launch_actions.append(Node(
                package='el05_usb_can_driver',
                executable='el05_motor_node',
                name=f'{side}_motor',
                parameters=[actuator_params_path],
                # 必要に応じてリマップ
            ))

    return launch_actions

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
