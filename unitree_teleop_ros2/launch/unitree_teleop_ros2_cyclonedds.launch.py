from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/tracker_cmd',
        description='Command velocity topic name'
    )
    
    # Set CycloneDDS URI and RMW implementation environment variables
    cyclonedds_uri_env = SetEnvironmentVariable(
        'CYCLONEDDS_URI', '/root/ugv_exp_ws/src/unitree_interface/unitree_teleop_ros2/unitree_teleop_ros2_cyclonedds.xml'
    )
    rmw_implementation_env = SetEnvironmentVariable(
        'RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'
    )
    
    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.1,
            'autorepeat_rate': 20.0
        }]
    )
    
    # Unitree teleop node
    unitree_teleop_ros2_node = Node(
        package='unitree_teleop_ros2',
        executable='unitree_teleop_ros2',
        name='unitree_teleop_ros2',
        remappings=[
            ('twist_auto', LaunchConfiguration('cmd_vel_topic'))
        ],
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    return LaunchDescription([
        joy_dev_arg,
        cmd_vel_topic_arg,
        cyclonedds_uri_env,
        rmw_implementation_env,
        joy_node,
        unitree_teleop_ros2_node
    ])
