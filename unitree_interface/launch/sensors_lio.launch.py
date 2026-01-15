# Copyright 2023 Ouster, Inc.
#
# Launch ouster nodes + RKO LIO + rosbag2 Recorder + ZED Multi Camera in one container

import os
from pathlib import Path
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution


def parse_array_param(param):
    """Parse array-like launch arguments."""
    str_val = param.replace('[', '')
    str_val = str_val.replace(']', '')
    str_val = str_val.replace(' ', '')
    arr = str_val.split(',')
    return arr


def launch_setup(context, *args, **kwargs):
    actions = []

    ouster_ros_pkg_dir = get_package_share_directory('ouster_ros')
    zed_wrapper_pkg_dir = get_package_share_directory('zed_wrapper')
    default_params_file = Path(ouster_ros_pkg_dir) / 'config' / 'os_sensor_cloud_image_params.yaml'

    # URDF/xacro file for multi ZED cameras
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

    # Get values from context
    ouster_ns_val = ouster_ns.perform(context)
    names_arr = parse_array_param(cam_names.perform(context))
    models_arr = parse_array_param(cam_models.perform(context))
    serials_arr = parse_array_param(cam_serials.perform(context))
    ids_arr = parse_array_param(cam_ids.perform(context))
    disable_tf_val = disable_tf.perform(context)

    num_cams = len(names_arr)

    # Validate camera arrays
    if num_cams != len(models_arr):
        return [
            LogInfo(msg=TextSubstitution(
                text='The `cam_models` array argument must match the size of the `cam_names` array argument.'))
        ]

    if num_cams != len(serials_arr) and num_cams != len(ids_arr):
        return [
            LogInfo(msg=TextSubstitution(
                text='The `cam_serials` or `cam_ids` array argument must match the size of the `cam_names` array argument.'))
        ]

    # --- Ouster components ---
    os_sensor = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterSensor',
        name='os_sensor',
        namespace=ouster_ns,
        parameters=[params_file, {'auto_start': auto_start,
                                  'sensor_hostname': sensor_hostname,
                                  'timestamp_mode': 'TIME_FROM_PTP_1588'}],
    )

    os_cloud = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterCloud',
        name='os_cloud',
        namespace=ouster_ns,
        parameters=[params_file, {'timestamp_mode': 'TIME_FROM_PTP_1588'}],
    )
    
    # --- RKO LIO (no namespace so topics below are global like /ouster/imu) ---
    rko = ComposableNode(
        package='rko_lio',
        plugin='rko_lio::ros::OnlineNode',
        name='online_node_component',
        namespace='ugv',
        parameters=[
            params_file,  # keep if your node reads general params from it
            {
                'imu_topic':   'ouster/imu',
                'lidar_topic': 'ouster/points',
                # output topics
                'map_topic': '/ugv/rko_lio/local_map',
                'odom_topic': '/ugv/rko_lio/odometry',
                # frames & behavior
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'deskew': True,
                'lidar_frame': 'os_lidar',
                'imu_frame': 'os_imu',
                'extrinsic_lidar2base_quat_xyzw_xyz': [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.25],
                'extrinsic_imu2base_quat_xyzw_xyz':  [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.25],
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
        name="recorder",
        parameters=[{'topics': [
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
                    "bag_prefix": '/bags/ugv_'}],
        remappings=[],
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
            recorder,
        ],
    )
    actions.append(ugv_container)

    # zed multi-camera
    cam_idx = 0
    for name in names_arr:
        model = models_arr[cam_idx]
        serial = serials_arr[cam_idx] if len(serials_arr) == num_cams else '0'
        cam_id = ids_arr[cam_idx] if len(ids_arr) == num_cams else '-1'

        info = '* Starting a ZED ROS2 node for camera ' + name + ' (' + model
        if serial != '0':
            info += ', serial: ' + serial
        elif cam_id != '-1':
            info += ', id: ' + cam_id
        info += ')'
        actions.append(LogInfo(msg=TextSubstitution(text=info)))
        
        publish_tf = 'false'
        if cam_idx == 0:
            if disable_tf_val.lower() == 'false':
                publish_tf = 'true'
        
        zed_wrapper_launch = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource([
                zed_wrapper_pkg_dir, '/launch/zed_camera.launch.py'
            ]),
            launch_arguments={
                'container_name': container_name,
                'camera_name': name,
                'camera_model': model,
                'serial_number': serial,
                'camera_id': cam_id,
                'publish_tf': publish_tf,
                'publish_map_tf': publish_tf,
                'namespace': 'ugv'
            }.items()
        )
        actions.append(zed_wrapper_launch)

        cam_idx += 1
    
    xacro_command = ['xacro ', multi_zed_xacro_path, ' ']
    for idx, name in enumerate(names_arr):
        xacro_command.append('camera_name_' + str(idx) + ':=' + name + ' ')

    # robot state publisher node for multi-camera setup
    rsp_name = 'zed_state_publisher'
    info = '* Starting robot_state_publisher node for ZED multi-camera: ' + rsp_name
    actions.append(LogInfo(msg=TextSubstitution(text=info)))

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
            'params_file', default_value=str(default_params_file),
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
            description='An array containing the name of the cameras, e.g. [zed_front,zed_back]'),
        DeclareLaunchArgument(
            'cam_models',
            default_value='zedxone4k,zedxone4k,zedxone4k,zedxone4k',
            description='An array containing the model of the cameras, e.g. [zed2i,zed2]'),
        DeclareLaunchArgument(
            'cam_serials',
            default_value='313815015,317242564,311718578,315786072',
            description='An array containing the serial number of the cameras, e.g. [35199186,23154724]'),
        DeclareLaunchArgument(
            'cam_ids',
            default_value='',
            description='An array containing the ID number of the cameras, e.g. [0,1]'),
        DeclareLaunchArgument(
            'disable_tf',
            default_value='False',
            description='If `True` disable TF broadcasting for all the cameras.'),

        OpaqueFunction(function=launch_setup)
    ])
