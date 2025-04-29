import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # package share directories
    rosbot_pkg_share = get_package_share_directory('rosbot_navigation')
    aiil_demo_pkg_share = get_package_share_directory('aiil_rosbot_demo')

    # launch arguments
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    session_file_path_arg = DeclareLaunchArgument(
        'session_file_path_arg',
        default_value=PathJoinSubstitution([rosbot_pkg_share, 'find_object_session', 'session.bin']),
        description='Path to the session.bin file'
    )

    repeater_input_image_topic_arg = DeclareLaunchArgument(
        'repeater_input_image_topic', default_value='/camera/color/image_raw')
    repeater_output_image_topic_arg = DeclareLaunchArgument(
        'repeater_output_image_topic', default_value='/camera/color/image_raw/repeat')
    repeater_use_compressed_arg = DeclareLaunchArgument(
        'repeater_use_compressed', default_value='false')

    camera_info_topic_arg = DeclareLaunchArgument(
        'node2_camera_info_topic', default_value='/camera/color/camera_info')
    depth_topic_arg = DeclareLaunchArgument(
        'node2_depth_topic', default_value='/camera/depth/image_raw')
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic', default_value='/scan')
    map_frame_arg = DeclareLaunchArgument(
        'map_frame', default_value='map')
    camera_frame_arg = DeclareLaunchArgument(
        'camera_optical_frame', default_value='camera_color_optical_frame')
    find_object_gui_arg = DeclareLaunchArgument(
        'gui', default_value='false')

    # configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    session_file_path = LaunchConfiguration('session_file_path_arg')
    repeater_input_image_topic = LaunchConfiguration('repeater_input_image_topic')
    repeater_output_image_topic = LaunchConfiguration('repeater_output_image_topic')
    repeater_use_compressed = LaunchConfiguration('repeater_use_compressed')
    node2_camera_info_topic = LaunchConfiguration('node2_camera_info_topic')
    node2_depth_topic = LaunchConfiguration('node2_depth_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    map_frame = LaunchConfiguration('map_frame')
    camera_optical_frame = LaunchConfiguration('camera_optical_frame')
    find_object_gui = LaunchConfiguration('gui')

    # repeater node
    repeater_node = Node(
        package='aiil_rosbot_demo',
        executable='best_effort_repeater',
        name='image_qos_repeater',
        output='screen',
        parameters=[
            {'sub_topic_name': repeater_input_image_topic},
            {'repeat_topic_name': repeater_output_image_topic},
            {'use_compressed': repeater_use_compressed},
        ]
    )

    # find_object_2d node
    find_object_node = Node(
        package='find_object_2d',
        executable='find_object_2d',
        name='find_object_2d',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'subscribe_depth': True,
            'gui': False,
            'session_path': session_file_path
        }],
        remappings=[
            ('image', repeater_output_image_topic),
            ('rgb/image_rect_color', repeater_output_image_topic),
            ('depth_registered/image_raw', node2_depth_topic),
            ('depth_registered/camera_info', node2_camera_info_topic),
        ],
        additional_env={'QT_QPA_PLATFORM': 'offscreen'}
    )

    # hazard marker detector node (node2.py)
    hazard_marker_node = Node(
        package='rosbot_navigation',
        executable='node2',
        name='hazard_marker_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'camera_info_topic': node2_camera_info_topic,
            'depth_image_topic': node2_depth_topic,
            'laser_scan_topic': scan_topic,
            'find_object_topic': '/find_object/objects',
            'hazard_marker_topic': '/hazards',
            'status_topic': '/snc_status',
            'map_frame': map_frame,
            'camera_optical_frame': camera_optical_frame,
        }]
    )

    # search explorer node (node1.py)
    search_explorer_node = Node(
        package='rosbot_navigation',
        executable='node1',
        name='search_explorer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'status_topic': '/snc_status',
            'hazard_marker_topic': '/hazards',
            'map_frame': map_frame,
        }]
    )

    # position tracker node (position_tracker_node.py) with delay
    delayed_position_tracker = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rosbot_navigation',
                executable='position_tracker_node',
                name='position_tracker',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        session_file_path_arg,
        repeater_input_image_topic_arg,
        repeater_output_image_topic_arg,
        repeater_use_compressed_arg,
        camera_info_topic_arg,
        depth_topic_arg,
        scan_topic_arg,
        map_frame_arg,
        camera_frame_arg,
        find_object_gui_arg,

        repeater_node,
        find_object_node,
        hazard_marker_node,
        search_explorer_node,      # <-- added node1.py launch
        delayed_position_tracker, # <-- delayed launch of position_tracker_node.py
    ])
