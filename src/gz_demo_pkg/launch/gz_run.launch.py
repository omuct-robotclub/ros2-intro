import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, NotSubstitution
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer

def generate_launch_description():
    pkg_share = get_package_share_directory('gz_demo_pkg')
    # slam_toolbox_share = get_package_share_directory('slam_toolbox')
    description_path = os.path.join(pkg_share,'description')
    config_path = os.path.join(pkg_share,'config')
    # slam_toolbox_launch_source = os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')

    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_spawn_model_launch_source = os.path.join(ros_gz_sim_share, "launch", "gz_spawn_model.launch.py")

    arges = [
        DeclareLaunchArgument(name='use_sim_time',default_value='true',description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(name='world_name',default_value='my_world',description='gz world name'),
        DeclareLaunchArgument(name='control_mode',default_value='cmd',description='omuni control mode: joy or cmd'),

        DeclareLaunchArgument(name='model', default_value=os.path.join(description_path,'urdf','omuni.urdf.xacro'), description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='world', default_value=os.path.join(description_path,'sdf','world.sdf'), description='Absolute path to robot model file'),

        DeclareLaunchArgument(name='rviz_config', default_value=os.path.join(config_path,'gz_run_rviz.rviz')),
        DeclareLaunchArgument(name='bridge_config', default_value=os.path.join(config_path,'bridge.yaml')),
        DeclareLaunchArgument(name='ekf_config', default_value=os.path.join(config_path,'ekf.yaml')),
        DeclareLaunchArgument(name='slam_config', default_value=os.path.join(config_path,'slam_params.yaml')),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world_name')
    control_mode = LaunchConfiguration('control_mode')
    left_lidar_topic = '/left_scan'
    right_lidar_topic = '/right_scan'
    merged_lidar_topic = '/merged_scan'

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, {'use_sim_time': use_sim_time}]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    gz_server = GzServer(
        world_sdf_file=LaunchConfiguration('world'),
        container_name='ros_gz_container',
        create_own_container='True',
        use_composition='True',
    )

    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
        launch_arguments={
            'world': world_name,
            'topic': '/robot_description',
            'entity_name': 'omuni_bot',
            'x' : '0.0',
            'y' : '0.0',
            'z' : '1.0',
        }.items(),
    )

    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=LaunchConfiguration('bridge_config'),
        container_name='ros_gz_container',
        create_own_container='False',
        use_composition='True',
    )

    moter_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="moter_bridge",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration('use_sim_time'),
                "override_timestamps_with_wall_time": NotSubstitution(LaunchConfiguration('use_sim_time')),
            }
        ],
        arguments=[
            "/whl_lf_v@std_msgs/msg/Float64@gz.msgs.Double",
            "/whl_rf_v@std_msgs/msg/Float64@gz.msgs.Double",
            "/whl_lb_v@std_msgs/msg/Float64@gz.msgs.Double",
            "/whl_rb_v@std_msgs/msg/Float64@gz.msgs.Double",
        ]
    )

    running_container = ComposableNodeContainer(
        name='main_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[],
        output='screen',
    )

    load_nodes = LoadComposableNodes(
        target_container='main_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gz_demo_pkg',
                plugin='omuni_control::OmuniControl',
                name='omuni_whl_node',
                extra_arguments=[{'use_intra_process_comms': True}],
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'mode' : control_mode,
                }],
            ),
        ]
    )

    return LaunchDescription(arges + [
        ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen'),
        gz_server,
        spawn_entity,
        ros_gz_bridge,
        moter_bridge,
        robot_state_publisher_node,
        joint_state_publisher,
        running_container,
        load_nodes,
        rviz,
    ])