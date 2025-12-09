from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_name = 'mecanum_robot_description'
    pkg_share_dir = get_package_share_directory(pkg_name)  # .../share/mecanum_robot_description
    pkg_share_parent = os.path.dirname(pkg_share_dir)      # .../share

    # ---- World dosyasının adı (sadece isim) ----
    default_world = 'empty_robot_world.sdf'

    # Launch argument'ler
    world_file = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Xacro yolu (RViz'de düzgün çalışan xacro)
    xacro_path = os.path.join(pkg_share_dir, 'urdf', 'mecanum_robot.xacro')

    # xacro komutu (URDF üretimi)
    robot_description_cmd = Command([
        'xacro ',
        xacro_path
    ])

    # robot_state_publisher için string içerik
    robot_description = ParameterValue(
        robot_description_cmd,
        value_type=str
    )

    # 🔴 ÖNEMLİ KISIM: Resource path
    # Gazebo şu URI'yi çözmeye çalışıyor:
    #   model://mecanum_robot_description/meshes/...
    # Bu yüzden model kökünü .../share altına koymamız gerekiyor.
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            pkg_share_parent,  # .../share  → içinde mecanum_robot_description klasörü var
            ':',
            os.path.join(pkg_share_dir, 'worlds'),
            ':$GZ_SIM_RESOURCE_PATH'
        ]
    )

    # Gazebo'yu world ismiyle başlat (dosya yolu değil, isim)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        emulate_tty=True
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # ROS–GZ clock köprüsü
    parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
        name='gz_bridge'
    )

    # Robotu spawn eden node
    create_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_description_cmd,
            '-name', 'mecanum_robot',
            '-z', '0.40',
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_file',
            default_value=default_world,
            description='World SDF file name (searched in GZ_SIM_RESOURCE_PATH)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        set_resource_path,
        gz_sim,
        robot_state_publisher,
        parameter_bridge,
        create_node,
    ])
