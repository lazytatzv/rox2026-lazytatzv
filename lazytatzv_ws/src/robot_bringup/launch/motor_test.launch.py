import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Paths
    package_bringup_share_directory = get_package_share_directory('robot_bringup')
    actuator_params_path = os.path.join(package_bringup_share_directory, 'config', 'actuators.yaml')

    # Arguments
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyUSB1', description='Serial port')
    motor_id_arg = DeclareLaunchArgument('id', default_value='0x0C', description='Motor CAN ID (hex)')
    use_pid_arg = DeclareLaunchArgument('use_pid', default_value='true', description='Enable PID controller')
    
    # Optional: still allow using a name from YAML for other params (like max_speed)
    motor_name_arg = DeclareLaunchArgument('name', default_value='test_motor', description='Node name prefix')

    port = LaunchConfiguration('port')
    motor_id_str = LaunchConfiguration('id')
    motor_name = LaunchConfiguration('name')
    use_pid = LaunchConfiguration('use_pid')

    # 1. Gateway
    gateway_node = Node(
        package='at_motor_driver',
        executable='at_bus_gateway_node',
        name='at_bus_gateway_test',
        parameters=[
            actuator_params_path,
            {'serial_port': port}
        ],
        output='screen'
    )

    # 2. Motor Node
    motor_node = Node(
        package='at_motor_driver',
        executable='at_motor_node', 
        name=[motor_name, '_driver'],
        parameters=[
            actuator_params_path,
            {
                # Note: AtMotorNode expects an int for motor_id. 
                # Launch doesn't easily convert hex string to int in substitution,
                # but AtMotorNode's declare_parameter and get_parameter will handle it if it's a string that looks like hex?
                # Actually ROS2 params are typed. We might need the user to pass decimal or we handle hex in code.
                # For now, we'll try passing it as is.
                'motor_id': 12, # Default 0x0C as int just in case
                'topic_target_velocity': [motor_name, '/target_velocity']
            }
        ],
        output='screen'
    )

    # 3. Motor Controller (Optional)
    controller_node = Node(
        package='at_motor_driver',
        executable='motor_controller_node',
        name=[motor_name, '_controller'],
        condition=IfCondition(use_pid),
        parameters=[
            actuator_params_path,
            {
                'desired_topic': [motor_name, '/desired_velocity'],
                'output_topic': [motor_name, '/target_velocity']
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        port_arg,
        motor_id_arg,
        use_pid_arg,
        motor_name_arg,
        gateway_node,
        motor_node,
        controller_node
    ])
