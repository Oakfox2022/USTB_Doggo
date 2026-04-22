import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_my_robot = get_package_share_directory('ustb_doggo_urdf')

    urdf_path = os.path.join(
        pkg_my_robot,
        'urdf',
        'ustb_doggo_urdf.urdf'
    )
    
    # ========== 关键：读取URDF内容 ==========
    with open(urdf_path, 'r') as f:
        urdf_content = f.read()
    robot_description = {'robot_description': urdf_content}

    controllers_config = os.path.join(pkg_my_robot, 'config', 'controllers.yaml')

    # ========== 启动 rviz2 ==========  
    rviz_node = TimerAction(
        period = 5.0,
        actions = [
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz_ocs2',
                output='screen',
                parameters=[{'use_sim_time': True}],
                arguments=['-d' + os.path.join(pkg_my_robot, 'config', 'ustb_doggo_gazebo.rviz')]
            )
        ]
    )

    # 1. 启动 Gazebo Classic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
    )

    # 2. robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': True}
            ]  # 使用统一的机器人描述
    )

    # 3. 把 URDF 模型丢进 Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-z', '0.5'  # 重要！让机器人从空中落下，避免嵌入地面
        ],
        output='screen'
    )

    # ========== 4. 加载控制器 ==========
    # 先加载joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    # 加载腿部控制器
    leg_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg_effort_controller'],
        output='screen'
    )

    wheel_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_velocity_controller'],
        output='screen'
    )
    
    # ========== 延迟启动逻辑 ==========
    # 等joint_state_broadcaster启动后再启动其他控制器

    delay_leg_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[leg_controller_spawner],
        )
    )

    delay_wheel_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=leg_controller_spawner,
            on_exit=[wheel_controller_spawner],
        )
    )

    return LaunchDescription([
        rviz_node,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        delay_leg_controller,
        delay_wheel_controller
    ])