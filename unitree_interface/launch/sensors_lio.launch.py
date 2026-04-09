# Copyright 2023 Ouster, Inc.
#
# Launch Ouster nodes + RKO LIO + rosbag2 Recorder + ZED Multi Camera in one container
# ROS 2 Humble version with delayed/staggered ZED startup

import os
from pathlib import Path
import yaml
import launch

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution


def parse_array_param(param):
    """Parse array-like launch arguments."""
    str_val = param.replace('[', '')
    str_val = str_val.replace(']', '')
    str_val = str_val.replace(' ', '')
    if str_val == '':
        return []
    return str_val.split(',')


def launch_setup(context, *args, **kwargs):
    actions = []

    ouster_ros_pkg_dir = get_package_share_directory('ouster_ros')
    zed_wrapper_pkg_dir = get_package_share_directory('zed_wrapper')
    default_params_file = Path(ouster_ros_pkg_dir) / 'config' / 'os_sensor_cloud_image_params.yaml'

    multi_zed_xacro_path = os.path.join(zed_wrapper_pkg_dir, 'urdf', 'zed_multi.urdf.xacro')

    # Launch configurations
    params_file = LaunchConfiguration('params_file')
    ouster_ns = LaunchConfiguration('ouster_ns')
    auto_start = LaunchConfiguration('auto_start')
    sensor_hostname = LaunchConfiguration('sensor_hostname')

    # ZED configurations
    cam_names = LaunchConfiguration('cam_names')
    cam_models = LaunchConfiguration('cam_models')
    cam_serials = LaunchConfiguration('cam_serials')
    cam_ids = LaunchConfiguration('cam_ids')
    disable_tf = LaunchConfiguration('disable_tf')
    zed_start_delay = LaunchConfiguration('zed_start_delay')

    # Context values
    ouster_ns_val = ouster_ns.perform(context)
    names_arr = parse_array_param(cam_names.perform(context))
    models_arr = parse_array_param(cam_models.perform(context))
    serials_arr = parse_array_param(cam_serials.perform(context))
    ids_arr = parse_array_param(cam_ids.perform(context))
    disable_tf_val = disable_tf.perform(context)
    zed_start_delay_val = float(zed_start_delay.perform(context))

    num_cams = len(names_arr)

    # Validate camera arrays
    if num_cams != len(models_arr):
        return [
            LogInfo(
                msg=TextSubstitution(
                    text='The `cam_models` array argument must match the size of the `cam_names` array argument.'
                )
            )
        ]

    if len(serials_arr) != num_cams and len(ids_arr) != num_cams:
        return [
            LogInfo(
                msg=TextSubstitution(
                    text='The `cam_serials` or `cam_ids` array argument must match the size of the `cam_names` array argument.'
                )
            )
        ]

    # --- Ouster components ---
    os_sensor = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterSensor',
        name='os_sensor',
        namespace=ouster_ns,
        parameters=[
            params_file,
            {
                'auto_start': auto_start,
                'sensor_hostname': sensor_hostname,
                'timestamp_mode': 'TIME_FROM_PTP_1588',
                'ptp_utc_tai_offset': 0.0,
            },
        ],
    )

    os_cloud = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterCloud',
        name='os_cloud',
        namespace=ouster_ns,
        parameters=[
            params_file,
            {
                'timestamp_mode': 'TIME_FROM_PTP_1588',
                'ptp_utc_tai_offset': 0.0,
            },
        ],
    )

    # --- RKO LIO ---
    rko = ComposableNode(
        package='rko_lio',
        plugin='rko_lio::ros::OnlineNode',
        name='online_node_component',
        namespace='ugv',
        parameters=[
            params_file,
            {
                'imu_topic': 'ouster/imu',
                'lidar_topic': 'ouster/points',
                'map_topic': '/ugv/rko_lio/local_map',
                'odom_topic': '/ugv/rko_lio/odometry',
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'deskew': True,
                'lidar_frame': 'os_lidar',
                'imu_frame': 'os_imu',
                'extrinsic_lidar2base_quat_xyzw_xyz': [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.25],
                'extrinsic_imu2base_quat_xyzw_xyz': [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.25],
                'publish_local_map': True,
                'publish_map_after': 1.0,
                'publish_lidar_acceleration': False,
                'invert_odom_tf': False,
                'mode': 'online',
            },
        ],
    )

    # rosbag recorder
    recorder = ComposableNode(
        package='rosbag2_composable_recorder',
        plugin='rosbag2_composable_recorder::ComposableRecorder',
        name='recorder',
        parameters=[
            {
                'topics': [
                    "/ugv/ouster/points",
                    "/ugv/ouster/imu",
                    "/ugv/rko_lio/odometry",
                    "/ugv/rko_lio/local_map",
                    "/ugv/zed_front/rgb/color/rect/image",
                    "/ugv/zed_front/rgb/color/rect/camera_info",
                    "/ugv/zed_left/rgb/color/rect/image",
                    "/ugv/zed_left/rgb/color/rect/camera_info",
                    "/ugv/zed_right/rgb/color/rect/image",
                    "/ugv/zed_right/rgb/color/rect/camera_info",
                    "/ugv/zed_back/rgb/color/rect/image",
                    "/ugv/zed_back/rgb/color/rect/camera_info",
                ],
                'storage_id': 'mcap',
                'record_all': False,
                'disable_discovery': False,
                'serialization_format': 'cdr',
                'start_recording_immediately': False,
                'bag_prefix': '/bags/ugv_',
            }
        ],
        remappings=[],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # UBlox
    ublox_cfg = os.path.join(
        get_package_share_directory('ublox_gps'),
        'config',
        'zed_f9p.yaml'
    )
    with open(ublox_cfg, 'r') as f:
        ublox_params = yaml.safe_load(f)['ublox_gps_node']['ros__parameters']

    ublox = ComposableNode(
        package='ublox_gps',
        plugin='ublox_node::UbloxNode',
        name='ublox_gps_node',
        parameters=[ublox_params],
        remappings=[
            ("/aidalm", "/ublox_raw/aidalm"),
            ("/timtm2", "/ublox_raw/timtm2"),
            ("/rtcm", "/ublox_raw/rtcm"),
            ("/nmea", "/ublox_raw/nmea"),
            ("/navclock", "/ublox_raw/navclock"),
            ("/navcov", "/ublox_raw/navcov"),
            ("/navheading", "/ublox_raw/navheading"),
            ("/navrelposned", "/ublox_raw/navrelposned"),
            ("/navstate", "/ublox_raw/navstate"),
            ("/navsvin", "/ublox_raw/navsvin"),
            ("/navstatus", "/ublox_raw/navstatus"),
            ("/aideph", "/ublox_raw/aideph"),
            ("/diagnostics", "/ublox_raw/diagnostics"),
            ("/monhw", "/ublox_raw/monhw"),
            ("/navsin", "/ublox_raw/nmea"),
            ("/rtcm", "/ublox_raw/rtcm"),
            ("/rxmrtcm", "/ublox_raw/rxmrtcm"),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # unified container
    container_name = 'ugv_container'

    ugv_container = ComposableNodeContainer(
        name=container_name,
        namespace='ugv',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            os_sensor,
            os_cloud,
            rko,
            ublox,
            recorder,
        ],
    )
    actions.append(ugv_container)

    # ZED multi-camera, staggered startup with TimerAction
    for cam_idx, name in enumerate(names_arr):
        model = models_arr[cam_idx]
        serial = serials_arr[cam_idx] if len(serials_arr) == num_cams else '0'
        cam_id = ids_arr[cam_idx] if len(ids_arr) == num_cams else '-1'

        info = '* Scheduling ZED ROS2 node for camera ' + name + ' (' + model
        if serial != '0':
            info += ', serial: ' + serial
        elif cam_id != '-1':
            info += ', id: ' + cam_id
        info += ')'
        actions.append(LogInfo(msg=TextSubstitution(text=info)))

        publish_tf = 'false'
        if cam_idx == 0 and disable_tf_val.lower() == 'false':
            publish_tf = 'true'

        zed_wrapper_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(zed_wrapper_pkg_dir, 'launch', 'zed_camera.launch.py')
            ),
            launch_arguments={
                'container_name': container_name,
                'camera_name': name,
                'camera_model': model,
                'serial_number': serial,
                'camera_id': cam_id,
                'publish_tf': publish_tf,
                'publish_map_tf': publish_tf,
                'namespace': 'ugv',
            }.items()
        )

        delay_seconds = cam_idx * zed_start_delay_val

        actions.append(
            TimerAction(
                period=delay_seconds,
                actions=[
                    LogInfo(
                        msg=TextSubstitution(
                            text=f'* Launching {name} after {delay_seconds:.1f}s delay'
                        )
                    ),
                    zed_wrapper_launch,
                ],
            )
        )

    # Robot state publisher for multi-camera setup
    xacro_command = ['xacro ', multi_zed_xacro_path, ' ']
    for idx, name in enumerate(names_arr):
        xacro_command.append('camera_name_' + str(idx) + ':=' + name + ' ')

    rsp_name = 'zed_state_publisher'
    actions.append(
        LogInfo(
            msg=TextSubstitution(
                text='* Starting robot_state_publisher node for ZED multi-camera: ' + rsp_name
            )
        )
    )

    multi_rsp_node = Node(
        package='robot_state_publisher',
        namespace='ugv',
        executable='robot_state_publisher',
        name=rsp_name,
        output='screen',
        parameters=[{
            'robot_description': Command(xacro_command).perform(context)
        }]
    )
    actions.append(multi_rsp_node)

    return actions


def generate_launch_description():
    ouster_ros_pkg_dir = get_package_share_directory('ouster_ros')
    default_params_file = Path(ouster_ros_pkg_dir) / 'config' / 'os_sensor_cloud_image_params.yaml'

    return launch.LaunchDescription([
        # Ouster arguments
        DeclareLaunchArgument(
            'params_file',
            default_value=str(default_params_file),
            description='Parameters file for Ouster components'
        ),
        DeclareLaunchArgument('ouster_ns', default_value='ugv/ouster'),
        DeclareLaunchArgument('viz', default_value='False'),
        DeclareLaunchArgument('auto_start', default_value='True'),
        DeclareLaunchArgument(
            'sensor_hostname',
            default_value='192.168.100.12',
            description='Ouster sensor hostname or IP address'
        ),
        DeclareLaunchArgument('bag_dir', default_value='/root/unitree_ws/bags/'),
        DeclareLaunchArgument('record_all', default_value='True'),

        # ZED Multi Camera arguments
        DeclareLaunchArgument(
            'cam_names',
            default_value='zed_front,zed_back,zed_left,zed_right',
            description='Array of camera names, e.g. [zed_front,zed_back]'
        ),
        DeclareLaunchArgument(
            'cam_models',
            default_value='zedxone4k,zedxone4k,zedxone4k,zedxone4k',
            description='Array of camera models, e.g. [zed2i,zed2]'
        ),
        DeclareLaunchArgument(
            'cam_serials',
            default_value='313815015,317242564,311718578,315786072',
            description='Array of camera serials, e.g. [35199186,23154724]'
        ),
        DeclareLaunchArgument(
            'cam_ids',
            default_value='',
            description='Array of camera IDs, e.g. [0,1]'
        ),
        DeclareLaunchArgument(
            'disable_tf',
            default_value='False',
            description='If True disable TF broadcasting for all the cameras.'
        ),
        DeclareLaunchArgument(
            'zed_start_delay',
            default_value='2.0',
            description='Delay in seconds between starting consecutive ZED cameras.'
        ),

        OpaqueFunction(function=launch_setup)
    ])