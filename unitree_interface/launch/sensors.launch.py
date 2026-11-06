# UGV sensors in separate, individually restartable processes.
#
#   /ugv/lidar_container     os_sensor + os_cloud (intra-process packets -> cloud)
#   /ugv/bievr_lio           BIEVR-LIO (default), standalone node; lio:=rko_lio for RKO-LIO
#   /ugv/<cam>_container     one ZED camera each (component_container_isolated)
#   /ugv/recorder_container  empty; /bags/start_recording.sh loads the recorder here
#
# Replaces sensors_lio.launch.py, which put everything (and, via the planner's
# container:= argument, the mapper/planner too) in one process: a ZED SDK abort
# took the lidar, LIO and planner down with it (component_container_mt exit -6 /
# -11 / 1 in ~/.ros/log, preceded by "CAMERA NOT INITIALIZED" / "CAMERA STREAM
# FAILED TO START").
#
# Containers respawn by default and their components are reloaded on every start
# (launch_ros only loads composable_node_descriptions once, so a plain respawn
# would come back empty). gnocchi publishes a GNSS-only pose while RKO-LIO is
# down and rebuilds its graph when the LIO returns in its new frame.

import os
from pathlib import Path

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def parse_array_param(param):
    s = param.replace('[', '').replace(']', '').replace(' ', '')
    return [] if s == '' else s.split(',')


def as_bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')


def container(name, respawn, respawn_delay, executable='component_container_mt', arguments=None):
    return ComposableNodeContainer(
        name=name, namespace='ugv', package='rclcpp_components', executable=executable,
        arguments=arguments or [], output='screen', composable_node_descriptions=[],
        respawn=respawn, respawn_delay=respawn_delay)


def reload_on_start(target, actions_or_fn):
    """Run the given actions every time target (re)starts."""
    handler = actions_or_fn if callable(actions_or_fn) else (lambda event, context: actions_or_fn)
    return RegisterEventHandler(OnProcessStart(target_action=target, on_start=handler))


def launch_setup(context, *args, **kwargs):
    actions = []
    zed_wrapper_pkg_dir = get_package_share_directory('zed_wrapper')
    params_file = LaunchConfiguration('params_file')
    ouster_ns = LaunchConfiguration('ouster_ns')
    respawn = as_bool(context, 'respawn')
    respawn_delay = float(LaunchConfiguration('respawn_delay').perform(context))
    zed_respawn_delay = float(LaunchConfiguration('zed_respawn_delay').perform(context))
    ipc = as_bool(context, 'use_intra_process_comms')
    ipc_args = [{'use_intra_process_comms': ipc}]

    names = parse_array_param(LaunchConfiguration('cam_names').perform(context))
    models = parse_array_param(LaunchConfiguration('cam_models').perform(context))
    serials = parse_array_param(LaunchConfiguration('cam_serials').perform(context))
    ids = parse_array_param(LaunchConfiguration('cam_ids').perform(context))
    disable_tf = as_bool(context, 'disable_tf')
    zed_start_delay = float(LaunchConfiguration('zed_start_delay').perform(context))
    n = len(names)
    if n != len(models):
        return [LogInfo(msg='`cam_models` must match the size of `cam_names`.')]
    if len(serials) != n and len(ids) != n:
        return [LogInfo(msg='`cam_serials` or `cam_ids` must match the size of `cam_names`.')]

    # --- Ouster ---------------------------------------------------------------
    os_params = [params_file, {'timestamp_mode': 'TIME_FROM_PTP_1588', 'ptp_utc_tai_offset': 0.0}]
    lidar_nodes = [
        ComposableNode(package='ouster_ros', plugin='ouster_ros::OusterSensor', name='os_sensor',
                       namespace=ouster_ns, extra_arguments=ipc_args,
                       parameters=os_params + [{'auto_start': LaunchConfiguration('auto_start'),
                                                'sensor_hostname': LaunchConfiguration('sensor_hostname')}]),
        ComposableNode(package='ouster_ros', plugin='ouster_ros::OusterCloud', name='os_cloud',
                       namespace=ouster_ns, extra_arguments=ipc_args, parameters=os_params),
    ]
    lidar = container('lidar_container', respawn, respawn_delay)
    actions += [lidar, reload_on_start(lidar, lambda event, ctx: [
        LogInfo(msg=f'* lidar_container started (pid {event.pid}); loading Ouster components'),
        LoadComposableNodes(target_container=lidar, composable_node_descriptions=lidar_nodes)])]

    # --- LIO: its own process, so a LIO crash leaves the lidar driver up -------
    lio = LaunchConfiguration('lio').perform(context)
    unitree_share = get_package_share_directory('unitree_interface')
    if lio == 'bievr_lio':
        # On the field bags BIEVR-LIO held up where RKO-LIO slid (long_road_bad:
        # 0.37 vs 15.6 m RMS against GNSS) and matched it elsewhere.
        # Odometry is reported for base_link (gnocchi: topics.odom = bievr_lio/odom).
        actions.append(Node(
            package='bievr_lio_ros2', executable='process_topics', name='bievr_lio', namespace='ugv',
            output='screen', respawn=respawn, respawn_delay=respawn_delay,
            arguments=['--sensor_config_file', os.path.join(unitree_share, 'config', 'bievr_ugv.yaml'),
                       '--params_file', os.path.join(unitree_share, 'config', 'bievr_params.yaml')],
            parameters=[{
                'topic_namespace': 'ugv/bievr_lio',   # -> /ugv/bievr_lio/odom
                'base_frame': 'base_link',
                # os_imu in base_link [x y z qx qy qz qw]: the driver's os_sensor->os_imu
                # (0.0063, -0.0118, 0.0076), os_sensor 0.25 - 0.0362 m above base_link.
                'base_to_imu': [0.0063, -0.0118, 0.2214, 0.0, 0.0, 0.0, 1.0],
                'publish_tf': True,                   # odom -> base_link, as RKO-LIO did
                'publish_registered_cloud': False,    # full registered scan, ~10 Hz; not used
            }]))
    elif lio == 'rko_lio':
        actions.append(Node(
            package='rko_lio', executable='online_node', name='rko_lio', namespace='ugv', output='screen',
            respawn=respawn, respawn_delay=respawn_delay,
            parameters=[{
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
            }]))
    elif lio != 'none':
        return [LogInfo(msg=f'Unknown lio:={lio} (bievr_lio, rko_lio or none)')]

    # --- Recorder target -------------------------------------------------------
    # Not respawned: a recorder that died should be noticed, not silently replaced
    # by an empty container.
    actions.append(container('recorder_container', False, 0.0))

    # --- ZED cameras: one isolated container each ------------------------------
    zed_launch = os.path.join(zed_wrapper_pkg_dir, 'launch', 'zed_camera.launch.py')
    zed_override = LaunchConfiguration('zed_params_override').perform(context)
    for i, name in enumerate(names):
        serial = serials[i] if len(serials) == n else '0'
        cam_id = ids[i] if len(ids) == n else '-1'
        publish_tf = 'true' if (i == 0 and not disable_tf) else 'false'
        cam_container = container(f'{name}_container', respawn, zed_respawn_delay,
                                  executable='component_container_isolated',
                                  arguments=['--use_multi_threaded_executor'])
        starts = {'count': 0}

        def load_zed(event, ctx, name=name, model=models[i], serial=serial, cam_id=cam_id,
                     publish_tf=publish_tf, starts=starts):
            # zed_camera.launch.py also starts the camera's robot_state_publisher;
            # that process is not part of the container, so only the first start may.
            starts['count'] += 1
            return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(zed_launch),
                launch_arguments={
                    'container_name': f'{name}_container',
                    'enable_ipc': LaunchConfiguration('use_intra_process_comms'),
                    'camera_name': name,
                    'camera_model': model,
                    'serial_number': serial,
                    'camera_id': cam_id,
                    'publish_tf': publish_tf,
                    'publish_map_tf': publish_tf,
                    'publish_urdf': 'true' if starts['count'] == 1 else 'false',
                    'namespace': 'ugv',
                    'ros_params_override_path': zed_override,
                }.items())]

        # Staggered startup, as before: opening several GMSL cameras at once is
        # where startup failures were seen.
        actions.append(TimerAction(period=i * zed_start_delay, actions=[
            LogInfo(msg=f'* Starting {name} in /ugv/{name}_container'),
            cam_container,
            reload_on_start(cam_container, load_zed),
        ]))

    # Robot state publisher for the multi-camera rig
    xacro = ['xacro ', os.path.join(zed_wrapper_pkg_dir, 'urdf', 'zed_multi.urdf.xacro'), ' ']
    for i, name in enumerate(names):
        xacro.append(f'camera_name_{i}:={name} ')
    actions.append(Node(package='robot_state_publisher', namespace='ugv', executable='robot_state_publisher',
                        name='zed_state_publisher', output='screen',
                        parameters=[{'robot_description': Command(xacro).perform(context)}]))
    return actions


def generate_launch_description():
    default_params_file = Path(get_package_share_directory('ouster_ros')) / 'config' / 'os_sensor_cloud_image_params.yaml'
    return launch.LaunchDescription([
        DeclareLaunchArgument('use_intra_process_comms', default_value='true'),
        DeclareLaunchArgument('respawn', default_value='true',
                              description='Restart a crashed sensor process (and reload its components)'),
        DeclareLaunchArgument('respawn_delay', default_value='3.0'),
        DeclareLaunchArgument('lio', default_value='bievr_lio', description='bievr_lio, rko_lio or none'),
        # After an Argus restart every camera exits at once; reopening only once
        # nvargus-daemon has settled avoids a chain of daemon crashes. Opens are
        # serialized by the ZED wrapper itself (/tmp/zed_open.lock).
        DeclareLaunchArgument('zed_respawn_delay', default_value='10.0'),
        DeclareLaunchArgument('params_file', default_value=str(default_params_file)),
        DeclareLaunchArgument('ouster_ns', default_value='ugv/ouster'),
        DeclareLaunchArgument('auto_start', default_value='True'),
        DeclareLaunchArgument('sensor_hostname', default_value='192.168.100.12'),
        DeclareLaunchArgument('cam_names', default_value='zed_front,zed_left,zed_right'),
        DeclareLaunchArgument('cam_models', default_value='zedxone4k,zedxone4k,zedxone4k'),
        DeclareLaunchArgument('cam_serials', default_value='313815015,311718578,315786072'),
        DeclareLaunchArgument('cam_ids', default_value=''),
        DeclareLaunchArgument('disable_tf', default_value='False'),
        DeclareLaunchArgument('zed_start_delay', default_value='2.0'),
        DeclareLaunchArgument('zed_params_override', default_value=os.path.join(
                                  get_package_share_directory('unitree_interface'), 'config', 'zed_exposure.yaml'),
                              description='ZED parameter override file (exposure); empty for wrapper defaults'),
        OpaqueFunction(function=launch_setup),
    ])
