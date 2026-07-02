from launch_ros.actions import Node
from launch import LaunchDescription # 用于创建 launch 描述符
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution  # 用于获取 launch 参数的值、执行命令和路径拼接
from launch_ros.substitutions import FindPackageShare # 用于查找包的 share 目录


def generate_launch_description():
    # 声明参数
    declare_urdf_path = DeclareLaunchArgument(
        "urdf_path",
        default_value=PathJoinSubstitution([
            FindPackageShare('bzzrobot_description'),
            'urdf',
            'bzz2robot',
            'bzz002.urdf.xacro'
        ]),
        description="xacro文件路径，默认值为bzz002.urdf.xacro"
    )
    
    # 获取 rviz2 配置文件路径
    rviz_config_path = DeclareLaunchArgument(
        "rviz_config_path",
        default_value=PathJoinSubstitution([
            FindPackageShare('bzzrobot_description'),
            'config',
            'display_robot_model.rviz'
        ]),
        description="rviz2配置文件路径，默认值为display_robot_model.rviz"
    )


    return LaunchDescription([
        declare_urdf_path,
        rviz_config_path,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('urdf_path')])}]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
        ),

        ExecuteProcess(
            cmd=['rviz2', '-d', LaunchConfiguration('rviz_config_path')],
            output='screen',
        ),
   ])