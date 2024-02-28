import os
import fnmatch
import stretch_body.robot_params

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, OrSubstitution, AndSubstitution, NotSubstitution
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path


def symlinks_to_has_beta_teleop_kit():
    usb_device_seen = {
        'hello-navigation-camera': False,
        'hello-gripper-camera': False,
    }

    listOfFiles = os.listdir('/dev')
    pattern = "hello*"
    for entry in listOfFiles:
        if fnmatch.fnmatch(entry, pattern):
            usb_device_seen[entry] = True

    return all(usb_device_seen.values())


def symlinks_to_has_nav_head_cam():
    usb_device_seen = {
        'hello-nav-head-camera': False,
    }

    listOfFiles = os.listdir('/dev')
    pattern = "hello*"
    for entry in listOfFiles:
        if fnmatch.fnmatch(entry, pattern):
            usb_device_seen[entry] = True

    return all(usb_device_seen.values())


def map_configuration_to_drivers(model, tool, has_beta_teleop_kit, has_nav_head_cam):
    """This method maps configurations to drivers. I.e. it identifies the robot configuration
    based on the variables provided and returns which drivers should be activated. If the
    variables don't constitute a valid configuration, something is wrong with the hardware,
    so the function raises an exception.

    Returns
    -------
    Tuple
        tuple with four elements:
          which_realsense_drivers ('d435i-only' or 'both'),
          add_gripper_driver (True or False),
          add_navigation_driver (True or False),
          add_head_nav_driver (True or False)
    """
    # Stretch RE1
    if   model == "RE1V0" and tool == "tool_stretch_gripper"   and has_beta_teleop_kit == False and has_nav_head_cam == False:
        return 'd435-only', False, False, False
    elif model == "RE1V0" and tool == "tool_stretch_gripper"   and has_beta_teleop_kit == True  and has_nav_head_cam == False:
        return 'd435-only', True,  True,  False
    elif model == "RE1V0" and tool == "tool_stretch_dex_wrist" and has_beta_teleop_kit == False and has_nav_head_cam == False:
        return 'd435-only', False, False, False
    elif model == "RE1V0" and tool == "tool_stretch_dex_wrist" and has_beta_teleop_kit == True  and has_nav_head_cam == False:
        return 'd435-only', True,  True,  False
    # Stretch 2
    elif model == "RE2V0" and tool == "tool_stretch_gripper"   and has_beta_teleop_kit == False and has_nav_head_cam == False:
        return 'd435-only', False, False, False
    elif model == "RE2V0" and tool == "tool_stretch_gripper"   and has_beta_teleop_kit == True  and has_nav_head_cam == False:
        return 'd435-only', True,  True,  False
    elif model == "RE2V0" and tool == "tool_stretch_dex_wrist" and has_beta_teleop_kit == False and has_nav_head_cam == False:
        return 'd435-only', False, False, False
    elif model == "RE2V0" and tool == "tool_stretch_dex_wrist" and has_beta_teleop_kit == True  and has_nav_head_cam == False:
        return 'd435-only', True,  True,  False
    # Stretch 2+ (upgraded Stretch 2)
    elif model == "RE2V0" and tool == "eoa_wrist_dw3_tool_sg3" and has_beta_teleop_kit == False and has_nav_head_cam == True:
        return 'both'     , False, False, True
    # Stretch 3
    elif model == "SE3"   and tool == "eoa_wrist_dw3_tool_sg3" and has_beta_teleop_kit == False and has_nav_head_cam == True:
        return 'both'     , False, False, True

    raise ValueError(f'cannot find valid configuration for model={model}, tool={tool}, has_beta_teleop_kit={has_beta_teleop_kit}, has_nav_head_cam={has_nav_head_cam}')

def generate_launch_description():
    teleop_interface_package = str(get_package_share_path('stretch_web_teleop'))
    core_package = str(get_package_share_path('stretch_core'))
    rosbridge_package = str(get_package_share_path('rosbridge_server'))
    stretch_core_path = str(get_package_share_directory('stretch_core'))
    stretch_navigation_path = str(get_package_share_directory('stretch_nav2'))
    navigation_bringup_path = str(get_package_share_directory('nav2_bringup'))
    
    _, robot_params = stretch_body.robot_params.RobotParams().get_params()
    stretch_serial_no = robot_params['robot']['serial_no']
    stretch_model = robot_params['robot']['model_name']
    stretch_tool = robot_params['robot']['tool']
    stretch_has_beta_teleop_kit = symlinks_to_has_beta_teleop_kit()
    stretch_has_nav_head_cam = symlinks_to_has_nav_head_cam()
    drivers_realsense, driver_gripper_cam, driver_navigation_cam, driver_nav_head_cam =
        map_configuration_to_drivers(stretch_model, stretch_tool, stretch_has_beta_teleop_kit, stretch_has_nav_head_cam)

    # Declare launch arguments
    params_file = DeclareLaunchArgument('params', default_value=[
        PathJoinSubstitution([teleop_interface_package, 'config', 'configure_video_streams_params.yaml'])])
    map_yaml = DeclareLaunchArgument('map_yaml', description='filepath to previously captured map', default_value='')
    d405_arg = DeclareLaunchArgument('d405', default_value='true')
    gripper_camera_arg = DeclareLaunchArgument('gripper_camera', default_value='false')
    wide_angle_cam_arg = DeclareLaunchArgument('wide_angle_cam', default_value='true')
    navigation_camera_arg = DeclareLaunchArgument('navigation_camera', default_value='false')
    certfile_arg = DeclareLaunchArgument('certfile', default_value=stretch_serial_no + '+6.pem')
    keyfile_arg = DeclareLaunchArgument('keyfile', default_value=stretch_serial_no + '+6-key.pem')
    nav2_params_file_param = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(stretch_navigation_path, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    dict_file_path = os.path.join(core_package, 'config', 'stretch_marker_dict.yaml')
    depthimage_to_laserscan_config = os.path.join(core_package, 'config', 'depthimage_to_laser_scan_params.yaml')

    # Launch only D435i if there there is no D405
    d435i_launch = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('d405')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([core_package, 'launch', 'd435i_low_resolution.launch.py']))
            )
        ]
    )

    # Launch both D435i and D405 if there is D405
    multi_camera_launch = GroupAction(
        condition=IfCondition(LaunchConfiguration('d405')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([teleop_interface_package, 'launch', 'multi_camera.launch.py']))
            )
        ]
    )

    # Gripper Camera Node
    # Publish blank image if there is no gripper fisheye camera or D405
    gripper_camera_node = Node(
        package='image_publisher',
        executable='image_publisher_node',
        name='gripper_camera_node',
        output='screen',
        parameters=[{'publish_rate': 15.0}],
        remappings=[('image_raw', '/gripper_camera/color/image_rect_raw')],
        arguments=[PathJoinSubstitution([teleop_interface_package, 'nodes', 'blank_image.png'])],
        condition=UnlessCondition(OrSubstitution(LaunchConfiguration('gripper_camera'), LaunchConfiguration('d405')))
    )

    beta_uvc_navigation_camera_group = GroupAction(
        condition=IfCondition(AndSubstitution(LaunchConfiguration('navigation_camera'), NotSubstitution(LaunchConfiguration('wide_angle_cam')))),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([core_package, 'launch', 'beta_navigation_camera.launch.py']))
            )
        ]
    )

    uvc_navigation_camera_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('wide_angle_cam')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([core_package, 'launch', 'navigation_camera.launch.py']))
            )
        ]
    )

    uvc_gripper_camera_group = GroupAction(
        condition=IfCondition(AndSubstitution(LaunchConfiguration('gripper_camera'), NotSubstitution(LaunchConfiguration('d405')))),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([core_package, 'launch', 'beta_gripper_camera.launch.py']))
            )
        ]
    )

    # Navigation Camera Node
    # Publish blank image if navigation camera does not exist
    navigation_camera_node = Node(
        package='image_publisher',
        executable='image_publisher_node',
        name='navigation_camera_node',
        output='screen',
        parameters=[{'publish_rate': 15.0}],
        remappings=[('image_raw', '/navigation_camera/image_raw')],
        arguments=[PathJoinSubstitution([teleop_interface_package, 'nodes', 'blank_image.png'])],
        condition=UnlessCondition(OrSubstitution(LaunchConfiguration('navigation_camera'), LaunchConfiguration('wide_angle_cam')))
    )
                        
    tf2_web_republisher_node = Node(
        package='tf2_web_republisher_py',
        executable='tf2_web_republisher',
        name='tf2_web_republisher_node'
    )

    # Stretch Driver
    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([core_package, 'launch', 'stretch_driver.launch.py'])),
        launch_arguments={'broadcast_odom_tf': 'True'}.items())
    
    # Rosbridge Websocket
    rosbridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(PathJoinSubstitution([rosbridge_package, 'launch', 'rosbridge_websocket_launch.xml'])),
        launch_arguments={
            'port': '9090',
            'address': 'localhost',
            'ssl': 'true',
            'certfile': PathJoinSubstitution([teleop_interface_package, 'certificates', LaunchConfiguration('certfile')]),
            'keyfile': PathJoinSubstitution([teleop_interface_package, 'certificates', LaunchConfiguration('keyfile')]),
            'authenticate': 'false'
        }.items()
    )

    # Configure Video Streams
    configure_video_streams_node = Node(
        package='stretch_web_teleop',
        executable='configure_video_streams.py',
        # name='configure_video_streams_node',
        output='screen',
        arguments=[LaunchConfiguration('params'), LaunchConfiguration('d405'), LaunchConfiguration('wide_angle_cam')]
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/rplidar.launch.py']))

    navigation_bringup_launch = GroupAction(
        condition=LaunchConfigurationNotEquals('map_yaml', ''),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([stretch_navigation_path, '/launch/bringup_launch.py']),
                launch_arguments={'use_sim_time': 'false', 
                                'autostart': 'true',
                                'map': PathJoinSubstitution([teleop_interface_package, 'maps', LaunchConfiguration('map_yaml')]),
                                'params_file': LaunchConfiguration('nav2_params_file'),
                                'use_rviz': 'false'}.items())
        ]
    )

    ld = LaunchDescription([
        map_yaml,
        nav2_params_file_param,
        params_file,
        gripper_camera_arg,
        d405_arg,
        navigation_camera_arg,
        wide_angle_cam_arg,
        certfile_arg,
        keyfile_arg,
        d435i_launch,
        gripper_camera_node,
        multi_camera_launch,
        uvc_navigation_camera_group,
        beta_uvc_navigation_camera_group,
        uvc_gripper_camera_group,
        navigation_camera_node,
        configure_video_streams_node,
        tf2_web_republisher_node,
        stretch_driver_launch,
        rosbridge_launch,
        rplidar_launch,
        navigation_bringup_launch
    ])

    ld.add_action(
        ExecuteProcess(
            cmd=[
                [
                    FindExecutable(name="ros2"),
                    " service call ",
                    "/reinitialize_global_localization ",
                    "std_srvs/srv/Empty ",
                    "\"{}\"",
                ]
            ],
            shell=True,
        ),
    )

    ld.add_action(
        ExecuteProcess(
            cmd=[
                [
                    FindExecutable(name="ros2"),
                    " param set ",
                    "/rosbridge_websocket ",
                    "std_msgs/msg/Bool ",
                    "true",
                ]
            ],
            shell=True,
        )
    )

    ld.add_action(
        ExecuteProcess(
            cmd=[
                [
                    FindExecutable(name="ros2"),
                    " param set ",
                    "/gripper_camera ",
                    "depth_module.enable_auto_exposure ",
                    "true",
                ]
            ],
            shell=True,
        )
    )
    return ld
