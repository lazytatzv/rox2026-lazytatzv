import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """
    真・最強構成 (Vendor-Neutral版): 
    - 1ノード = 1モーターの完全分散設計
    - シリアルゲートウェイによる一元管理
    - モータータイプ（robstride/ddsm）を引数で即座に切り替え可能
    - YAMLによる全ノード集中パラメータ管理
    """
    
    # 引数定義
    motor_type_arg = DeclareLaunchArgument(
        'motor_type',
        default_value='robstride',
        description='Type of motor to use (robstride, ddsm)'
    )
    motor_type = LaunchConfiguration('motor_type')

    # パス取得
    package_bringup_share_directory = get_package_share_directory('robot_bringup')
    twist_mux_configuration_path = os.path.join(package_bringup_share_directory, 'config', 'twist_mux.yaml')
    physical_params_path = os.path.join(package_bringup_share_directory, 'config', 'physical.yaml')
    teleop_params_path = os.path.join(package_bringup_share_directory, 'config', 'teleop.yaml')
    
    # 動的なYAMLパス設定
    actuator_params_path = PythonExpression([
        "'", os.path.join(package_bringup_share_directory, 'config', 'actuators_'), "' + '", motor_type, "' + '.yaml'"
    ])

    # 動的なプラグイン名設定
    motor_plugin = PythonExpression([
        "'motor_driver::RobstrideMotorNode' if '", motor_type, "' == 'robstride' else 'motor_driver::DdsmMotorNode'"
    ])

    container = ComposableNodeContainer(
        name='robot_system_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # 1. Standard Joystick Driver
            ComposableNode(
                package='joy',
                plugin='joy::JoyNode',
                name='joystick_driver_node',
                parameters=[teleop_params_path],
            ),
            # 2. Base Teleoperation Node
            ComposableNode(
                package='base_teleop',
                plugin='base_teleop::BaseTeleopNode',
                name='base_teleoperation_node',
                parameters=[teleop_params_path],
                remappings=[('cmd_vel', 'cmd_vel_joy')],
            ),
            # 3. Twist Multiplexer
            ComposableNode(
                package='twist_mux',
                plugin='twist_mux::TwistMux',
                name='velocity_command_multiplexer',
                parameters=[twist_mux_configuration_path],
                remappings=[('cmd_vel_out', 'cmd_vel')],
            ),
            # 4. Mecanum Kinematics Calculation Node
            ComposableNode(
                package='mecanum_kinematics',
                plugin='mecanum_kinematics::MecanumKinematicsNode',
                name='kinematics_engine_node',
                parameters=[physical_params_path],
            ),
            ComposableNode(
                package='mecanum_kinematics',
                plugin='mecanum_kinematics::WheelSpeedsDispatcher',
                name='wheel_speeds_dispatcher',
                parameters=[actuator_params_path],
            ),
            
            # --- Distributed Actuator Layer ---
            
            # 5. Serial Gateway
            ComposableNode(
                package='motor_driver',
                plugin='motor_driver::SerialGateway',
                name='serial_gateway',
                parameters=[actuator_params_path],
            ),
            
            # 6. Front Left Motor
            ComposableNode(
                package='motor_driver',
                plugin=motor_plugin,
                name='front_left_motor',
                parameters=[actuator_params_path],
                remappings=[('~/target_velocity', 'front_left/target_velocity')],
            ),
            ComposableNode(
                package='motor_driver',
                plugin='motor_driver::MotorController',
                name='front_left_motor_controller',
                parameters=[actuator_params_path],
            ),
            # 7. Front Right Motor
            ComposableNode(
                package='motor_driver',
                plugin=motor_plugin,
                name='front_right_motor',
                parameters=[actuator_params_path],
                remappings=[('~/target_velocity', 'front_right/target_velocity')],
            ),
            ComposableNode(
                package='motor_driver',
                plugin='motor_driver::MotorController',
                name='front_right_motor_controller',
                parameters=[actuator_params_path],
            ),
            # 8. Rear Left Motor
            ComposableNode(
                package='motor_driver',
                plugin=motor_plugin,
                name='rear_left_motor',
                parameters=[actuator_params_path],
                remappings=[('~/target_velocity', 'rear_left/target_velocity')],
            ),
            ComposableNode(
                package='motor_driver',
                plugin='motor_driver::MotorController',
                name='rear_left_motor_controller',
                parameters=[actuator_params_path],
            ),
            # 9. Rear Right Motor
            ComposableNode(
                package='motor_driver',
                plugin=motor_plugin,
                name='rear_right_motor',
                parameters=[actuator_params_path],
                remappings=[('~/target_velocity', 'rear_right/target_velocity')],
            ),
            ComposableNode(
                package='motor_driver',
                plugin='motor_driver::MotorController',
                name='rear_right_motor_controller',
                parameters=[actuator_params_path],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        motor_type_arg,
        container
    ])
