import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Paths
    package_bringup_share_directory = get_package_share_directory('robot_bringup')

    # Arguments
    motor_type_arg = DeclareLaunchArgument(
        'motor_type',
        default_value='robstride',
        description='Type of motor to test (robstride, ddsm)'
    )
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyUSB1', description='Serial port')
    motor_id_arg = DeclareLaunchArgument('id', default_value='12', description='Motor ID (decimal)')
    use_pid_arg = DeclareLaunchArgument('use_pid', default_value='true', description='Enable PID controller')
    motor_name_arg = DeclareLaunchArgument('name', default_value='test_motor', description='Node name prefix')

    motor_type = LaunchConfiguration('motor_type')
    port = LaunchConfiguration('port')
    motor_id = LaunchConfiguration('id')
    motor_name = LaunchConfiguration('name')
    use_pid = LaunchConfiguration('use_pid')

    # 動的なYAMLパス設定
    actuator_params_path = PythonExpression([
        "'", os.path.join(package_bringup_share_directory, 'config', 'actuators_'), "' + '", motor_type, "' + '.yaml'"
    ])

    # 動的なプラグイン名設定
    motor_plugin = PythonExpression([
        "'motor_driver::RobstrideMotorNode' if '", motor_type, "' == 'robstride' else 'motor_driver::DdsmMotorNode'"
    ])
    
    # ノード実行ファイル名（CMakeLists.txt の EXECUTABLE 指定に合わせる）
    motor_exe = PythonExpression([
        "'robstride_motor_node' if '", motor_type, "' == 'robstride' else 'ddsm_motor_node'"
    ])

    # 1. Gateway
    gateway_node = Node(
        package='motor_driver',
        executable='serial_gateway_node',
        name='serial_gateway_test',
        parameters=[
            actuator_params_path,
            {'serial_port': port}
        ],
        output='screen'
    )

    # 2. Motor Node
    motor_node = Node(
        package='motor_driver',
        executable=motor_exe, 
        name=[motor_name, '_driver'],
        parameters=[
            actuator_params_path,
            {
                'motor_id': motor_id,
                'topic_target_velocity': [motor_name, '/target_velocity']
            }
        ],
        output='screen'
    )

    # 3. Motor Controller (Optional)
    controller_node = Node(
        package='motor_driver',
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
        motor_type_arg,
        port_arg,
        motor_id_arg,
        use_pid_arg,
        motor_name_arg,
        gateway_node,
        motor_node,
        controller_node
    ])
