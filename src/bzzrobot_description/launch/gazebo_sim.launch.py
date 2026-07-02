from launch_ros.actions import Node  # 用于创建 ROS 节点
from launch import LaunchDescription  # 用于创建 launch 描述符
from launch.actions import ExecuteProcess, DeclareLaunchArgument  # 用于执行进程和声明 launch 参数
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution  # 用于获取 launch 参数的值、执行命令和路径拼接
from launch_ros.substitutions import FindPackageShare  # 用于查找包的 share 目录


def generate_launch_description():
    # 1. 声明参数：xacro 文件路径
    declare_urdf_path = DeclareLaunchArgument(
        "urdf_path",
        default_value=PathJoinSubstitution([
            FindPackageShare('bzzrobot_description'),
            'urdf',
            'bzz2robot',
            'bzz002.urdf.xacro'
        ]),
        description="xacro 文件路径，默认值为 bzz002.urdf.xacro"
    )

    # 2. 声明参数：Gazebo 世界文件路径
    declare_world_path = DeclareLaunchArgument(
        "world_path",
        default_value=PathJoinSubstitution([
            FindPackageShare('bzzrobot_description'),
            'world',
            'room.world'
        ]),
        description="Gazebo 世界文件路径，默认值为 room.world"
    )

    return LaunchDescription([
        declare_urdf_path,
        declare_world_path,

        # 3. 启动 robot_state_publisher，解析 xacro 并发布到 /robot_description 话题
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('urdf_path')]),
                'use_sim_time': True  # 使用 Gazebo 仿真时间
            }]
        ),

        # 4. 启动 Gazebo 仿真环境，加载指定的世界文件
        # -s libgazebo_ros_factory.so：支持从 ROS 话题创建模型（spawn_entity）
        # 注意：不加载 libgazebo_ros_state.so，因为 diff_drive 插件会自动发布 /joint_states
        # 两个插件同时发布同一话题会产生冲突
        ExecuteProcess(
            cmd=['gazebo', '--verbose', 
                 '-s', 'libgazebo_ros_factory.so',
                 LaunchConfiguration('world_path')],
            output='screen',
        ),

        # 5. 将机器人模型生成到 Gazebo 中
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'bzz002',          # 模型名称
                '-topic', 'robot_description', # 从话题读取模型描述
                '-x', '0',                     # 初始 X 坐标
                '-y', '0',                     # 初始 Y 坐标
                '-z', '0.0015',                # 初始 Z 坐标（抬高避免穿地）
            ],
            output='screen',
        ),
    ])
