import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """
    最強・究極構成 (Actuator Abstraction版): 
    - 1ノード = 1モーターの完全分散設計
    - 高速ゲートウェイによるシリアル一元管理
    - YAMLによる全ノード集中パラメータ管理
    - Composable Nodes による低レイテンシ実行
    """
    
    # パス取得
    package_bringup_share_directory = get_package_share_directory('robot_bringup')
    twist_mux_configuration_path = os.path.join(package_bringup_share_directory, 'config', 'twist_mux.yaml')
    
    # Split YAML configurations
    physical_params_path = os.path.join(package_bringup_share_directory, 'config', 'physical.yaml')
    teleop_params_path = os.path.join(package_bringup_share_directory, 'config', 'teleop.yaml')
    actuator_params_path = os.path.join(package_bringup_share_directory, 'config', 'actuators.yaml')

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
            
            # 5. AT Bus Gateway (Owns /dev/ttyUSB1)
            ComposableNode(
                package='at_motor_driver',
                plugin='at_motor_driver::AtBusGateway',
                name='at_bus_gateway',
                parameters=[actuator_params_path],
            ),
            
            # 6. Front Left Motor
            ComposableNode(
                package='at_motor_driver',
                plugin='at_motor_driver::AtMotorNode',
                name='front_left_motor',
                parameters=[actuator_params_path],
                remappings=[('~/target_velocity', 'front_left/target_velocity')],
            ),
            ComposableNode(
                package='at_motor_driver',
                plugin='at_motor_driver::MotorController',
                name='front_left_motor_controller',
                parameters=[actuator_params_path],
            ),
            # 7. Front Right Motor
            ComposableNode(
                package='at_motor_driver',
                plugin='at_motor_driver::AtMotorNode',
                name='front_right_motor',
                parameters=[actuator_params_path],
                remappings=[('~/target_velocity', 'front_right/target_velocity')],
            ),
            ComposableNode(
                package='at_motor_driver',
                plugin='at_motor_driver::MotorController',
                name='front_right_motor_controller',
                parameters=[actuator_params_path],
            ),
            # 8. Rear Left Motor
            ComposableNode(
                package='at_motor_driver',
                plugin='at_motor_driver::AtMotorNode',
                name='rear_left_motor',
                parameters=[actuator_params_path],
                remappings=[('~/target_velocity', 'rear_left/target_velocity')],
            ),
            ComposableNode(
                package='at_motor_driver',
                plugin='at_motor_driver::MotorController',
                name='rear_left_motor_controller',
                parameters=[actuator_params_path],
            ),
            # 9. Rear Right Motor
            ComposableNode(
                package='at_motor_driver',
                plugin='at_motor_driver::AtMotorNode',
                name='rear_right_motor',
                parameters=[actuator_params_path],
                remappings=[('~/target_velocity', 'rear_right/target_velocity')],
            ),
            ComposableNode(
                package='at_motor_driver',
                plugin='at_motor_driver::MotorController',
                name='rear_right_motor_controller',
                parameters=[actuator_params_path],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
