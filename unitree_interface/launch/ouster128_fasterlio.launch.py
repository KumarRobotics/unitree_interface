from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer, LifecycleNode
from launch_ros.descriptions import ComposableNode
import launch_ros.actions
from launch.actions import SetEnvironmentVariable, RegisterEventHandler, EmitEvent, LogInfo
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
from launch.events import Shutdown

from launch.substitutions import EnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from pathlib import Path
import lifecycle_msgs.msg


def generate_launch_description():

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    stdout_colorized_envvar = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')
    
    rviz = LaunchConfiguration('rviz', default='false')
    rviz_use_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to launch RVIZ'
    )
    
    # Ouster params file (use composite params for os_sensor/os_cloud)
    ouster_ros_pkg_dir = get_package_share_directory('ouster_ros')
    default_params_file = Path(ouster_ros_pkg_dir) / 'config' / 'os_sensor_cloud_image_params.yaml'
    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument('params_file', default_value=str(default_params_file),
                                            description='name or path to the parameters file to use.')
    
    # Faster-LIO configuration
    config_file = PathJoinSubstitution([get_package_share_directory('faster_lio'), 'config', 'ouster128.yaml'])
    rviz_config = PathJoinSubstitution([get_package_share_directory('faster_lio'), 'rviz_cfg', 'loam_livox.rviz'])
    
    # Ouster namespace
    ouster_ns = LaunchConfiguration('ouster_ns')
    ouster_ns_arg = DeclareLaunchArgument(
        'ouster_ns', 
        default_value='ouster',
        description='Namespace for ouster nodes'
    )
    
    # Ouster sensor hostname
    sensor_hostname = LaunchConfiguration('sensor_hostname')
    sensor_hostname_arg = DeclareLaunchArgument(
        'sensor_hostname',
        default_value='192.168.100.12',
        description='Hostname or IP address of the Ouster sensor'
    )

    # auto_start arg (matches sensor.composite.launch.py)
    auto_start = LaunchConfiguration('auto_start')
    auto_start_arg = DeclareLaunchArgument('auto_start', default_value='True')

    # Ouster sensor component (composable)
    os_sensor = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterSensor',
        name='os_sensor',
        namespace=ouster_ns,
        parameters=[params_file, {'auto_start': auto_start, 'sensor_hostname': sensor_hostname}],
    )

    # Ouster cloud component
    os_cloud = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterCloud',
        name='os_cloud',
        namespace=ouster_ns,
        parameters=[params_file],
    )

    # Faster-LIO component
    faster_lio_component = ComposableNode(
        package='faster_lio',
        plugin='faster_lio::LaserMappingComponent',
        name='laserMapping',
        namespace='',
        parameters=[config_file]
    )

    # Component container that will hold components (ouster cloud + faster_lio)
    container = ComposableNodeContainer(
        name='mapping_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            os_sensor,
            os_cloud,
            faster_lio_component,
        ],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config,
                    '--ros-args', '--log-level', 'warn'],
        parameters=[{'use_sim_time' : True}],
        output='screen',
        condition=IfCondition(rviz)
    )

    # Set CycloneDDS URI and RMW implementation environment variables
    cyclonedds_uri_env = SetEnvironmentVariable(
        'CYCLONEDDS_URI', '/root/agx_cyclonedds.xml',
        # 'CYCLONEDDS_URI', '/root/agx_cyclonedds.xml'
    )
    rmw_implementation_env = SetEnvironmentVariable(
        'RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'
    )

    ld = LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        rviz_use_arg,
        ouster_ns_arg,
        params_file_arg,
        auto_start_arg,
        sensor_hostname_arg,
    ])
    
    # Set environment variables
    ld.add_action(cyclonedds_uri_env)
    ld.add_action(rmw_implementation_env)
    
    # Add the component container with components (os_sensor, ouster cloud, faster_lio)
    ld.add_action(container)

    # Debugging: print resolved sensor hostname
    ld.add_action(LogInfo(msg=['Using sensor_hostname: ', str(sensor_hostname)]))
    
    # Add rviz if requested
    ld.add_action(rviz_node)

    # No lifecycle events: components run inside the container and use the params file.

    return ld
