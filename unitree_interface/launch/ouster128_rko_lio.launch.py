# Copyright 2023 Ouster, Inc.
#
# Launch ouster nodes + RKO LIO + rosbag2 Recorder in one container

from pathlib import Path
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ouster_ros_pkg_dir = get_package_share_directory('ouster_ros')
    default_params_file = Path(ouster_ros_pkg_dir) / 'config' / 'os_sensor_cloud_image_params.yaml'

    # Args
    params_file     = LaunchConfiguration('params_file')
    ouster_ns       = LaunchConfiguration('ouster_ns')
    rviz_enable     = LaunchConfiguration('viz')
    auto_start      = LaunchConfiguration('auto_start')
    bag_dir         = LaunchConfiguration('bag_dir')       # where to write MCAP files
    record_all      = LaunchConfiguration('record_all')    # toggle all topics

    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=str(default_params_file),
        description='Parameters file for Ouster components'
    )
    ouster_ns_arg = DeclareLaunchArgument('ouster_ns', default_value='ouster')
    rviz_enable_arg = DeclareLaunchArgument('viz', default_value='True')
    auto_start_arg = DeclareLaunchArgument('auto_start', default_value='True')

    # Bag args (optional)
    bag_dir_arg = DeclareLaunchArgument('bag_dir', default_value='/root/unitree_ws/bags/')
    record_all_arg = DeclareLaunchArgument('record_all', default_value='True')

    # --- Ouster components ---
    os_sensor = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterSensor',
        name='os_sensor',
        namespace=ouster_ns,
        parameters=[params_file, {'auto_start': auto_start}],
    )

    os_cloud = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterCloud',
        name='os_cloud',
        namespace=ouster_ns,
        parameters=[params_file],
    )

    # --- RKO LIO (no namespace so topics below are global like /ouster/imu) ---
    rko = ComposableNode(
        package='rko_lio',
        plugin='rko_lio::ros::OnlineNode',
        name='online_node_component',
        namespace='',
        parameters=[
            params_file,  # keep if your node reads general params from it
            {
                # match producer topics from Ouster under /ouster/*
                'imu_topic':   'ouster/imu',     # resolves to /ouster/imu
                'lidar_topic': 'ouster/points',  # resolves to /ouster/points

                # frames & behavior
                'rviz': False,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'deskew': True,
                'lidar_frame': 'os_lidar',
                'imu_frame': 'os_imu',
                'extrinsic_lidar2base_quat_xyzw_xyz': [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                'extrinsic_imu2base_quat_xyzw_xyz':  [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                'publish_local_map': True,
                'publish_map_after': 1.0,
                'publish_lidar_acceleration': False,
                'invert_odom_tf': False,
                'mode': 'online',
            },
        ],
    )

    # --- rosbag2 Recorder (Jazzy component) ---
    recorder = ComposableNode(
        package='rosbag2_composable_recorder',
        plugin='rosbag2_composable_recorder::ComposableRecorder',
        name="recorder",
        parameters=[{'topics': [
                "/ouster/points",
                "/ouster/imu",
                "/rko_lio/odometry",
                "/rko_lio/local_map",
                ],
                    'storage_id': 'mcap',
                    'record_all': False,
                    'disable_discovery': False,
                    'serialization_format': 'cdr',
                    'start_recording_immediately': False,
                    "bag_prefix": '/bags/ugv_'}],
        remappings=[],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # --- Container ---
    os_container = ComposableNodeContainer(
        name='os_container',
        namespace=ouster_ns,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            os_sensor,
            os_cloud,
            rko,
            recorder,
        ],
    )

    # Optional RViz
    rviz_launch_file_path = Path(ouster_ros_pkg_dir) / 'launch' / 'rviz.launch.py'
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(rviz_launch_file_path)]),
        condition=IfCondition(rviz_enable),
    )

    return launch.LaunchDescription([
        params_file_arg, ouster_ns_arg, rviz_enable_arg, auto_start_arg,
        bag_dir_arg, record_all_arg,
        rviz_launch,
        os_container,
    ])
