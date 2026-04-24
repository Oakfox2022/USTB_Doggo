import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_ustb_doggo_urdf = get_package_share_directory('ustb_doggo_urdf')
    pkg_ustb_doggo_slam = get_package_share_directory('ustb_doggo_slam')

    slam_config = os.path.join(pkg_ustb_doggo_slam, 'config', 'slam_toolbox_config.yaml')

    use_sim_time = True

    # ================= Gazebo =================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(pkg_ustb_doggo_urdf, 'world', 'roomMin.world')
        }.items(),
    )

    # ================= Robot Description =================
    urdf_path = os.path.join(pkg_ustb_doggo_urdf, 'urdf', 'ustb_doggo_urdf.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = {'robot_description': f.read()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'doggo', '-z', '0.5'],
        output='screen'
    )

    # ================= Controllers =================
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    leg_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg_effort_controller'],
        output='screen'
    )

    wheel_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_velocity_controller'],
        output='screen'
    )

    # ================= EKF =================
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            os.path.join(pkg_ustb_doggo_slam, 'config', 'ekf.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    ekf_delayed = TimerAction(
        period=3.0,
        actions=[ekf]
    )

    # ================= SLAM =================
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config,{'use_sim_time': use_sim_time}]
    )

    slam_delayed = TimerAction(
        period=5.0,
        actions=[slam_node]
    )

    # ================= RViz =================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(pkg_ustb_doggo_slam, 'config', 'ustb_doggo_slam.rviz')
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    rviz_delayed = TimerAction(
        period=7.0,
        actions=[rviz]
    )

    # ================= 控制节点 =================
    cartesian_impedance = Node(
        package='ustb_doggo_impedance_control',
        executable='cartesian_impedance',
        name='cartesian_impedance',
        output='screen'
    )

    trot_gait_generator = Node(
        package='ustb_doggo_impedance_control',
        executable='trot_gait_generator',
        name='trot_gait_generator',
        output='screen'
    )

    keyboard_control_trot = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--',
            'bash', '-c',
            'ros2 run ustb_doggo_impedance_control keyboard_control_trot; exec bash'
        ],
        additional_env=os.environ,
        output='screen'
    )

    virtual_model_control = Node(
        package='ustb_doggo_virtual_model_control',
        executable='virtual_model_control',
        name='virtual_model_control',
        output='screen'
    )

    keyboard_control_vmc = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--',
            'bash', '-c',
            'ros2 run ustb_doggo_virtual_model_control keyboard_control_vmc; exec bash'
        ],
        additional_env=os.environ,
        output='screen'
    )

    # ================= 控制器启动链 =================
    delay_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster]
        )
    )

    delay_leg_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[leg_controller]
        )
    )

    delay_wheel_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=leg_controller,
            on_exit=[wheel_controller]
        )
    )

    # ================= Launch =================
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,

        delay_joint_state_broadcaster,
        delay_leg_controller,
        delay_wheel_controller,

        ekf_delayed,
        slam_delayed,
        rviz_delayed,

        # cartesian_impedance,
        # trot_gait_generator,
        # keyboard_control_trot,

        virtual_model_control,
        keyboard_control_vmc
    ])
