import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    package_name = 'protobot'

    # Robot description (xacro) - fixed path & arguments
    xacro_file = os.path.join(
        get_package_share_directory(package_name),
        'description',
        'robot.urdf.xacro'
    )
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' use_ros2_control:=true',
        ' sim_mode:=false'
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # Config files
    controller_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'my_controllers.yaml'
    )
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'twist_mux.yaml'
    )

    # 1. Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'use_ros2_control': 'true'
        }.items()
    )

    # 2. Joystick + teleop + pose manager
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'joystick.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # 3. Twist multiplexer
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_params, {'use_sim_time': False}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel')]
    )

    # 4. ros2_control node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controller_params_file],
        output='screen'
    )
    delayed_ros2_control_node = TimerAction(period=3.0, actions=[ros2_control_node])

    # 5. Spawner - DiffDrive controller
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[diff_drive_spawner]
        )
    )

    # 6. Spawner - Joint State Broadcaster
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[joint_broad_spawner]
        )
    )

    # 7. Hokuyo LiDAR driver
    # hokuyo_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(get_package_share_directory('urg_node2'), 'launch', 'urg_node2.launch.py')
    #     ]),
    #     launch_arguments={'use_sim_time': 'false'}.items()
    # )

    node_hokuyo_drive = IncludeLaunchDescription(
       PythonLaunchDescriptionSource([
           os.path.join(
               get_package_share_directory('urg_node2'),
               'launch',
               'urg_node2.launch.py'
           )
       ]))


    # paper_weight_node = Node(
    #     package='protobot',
    #     executable='paper_weight_serial.py',
    #     name='paper_weight_serial',
    #     output='screen',
    #     parameters=[{
    #         'port': '/dev/ttyUSB1',
    #         'baud': 115200,
    #         'publish_rate': 10.0
    #     }]
    # )

    # pose_manager_node = Node(
    #     package='protobot',
    #     executable='pose_manager_node.py',
    #     name='pose_manager_node',
    #     output='screen',
    #     parameters=[{
    #         'paper_threshold': 0.20,
    #         'trigger_count': 5,
    #         'home_pose_name': 'home',
    #         'map_frame': 'map',
    #         'base_frame': 'base_link',
    #         'wait_at_goal_sec': 10.0
    #     }]
    # )

    # Final launch description
    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        delayed_ros2_control_node,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        node_hokuyo_drive,
        # paper_weight_node,
        # pose_manager_node
        # hokuyo_node,
    ])