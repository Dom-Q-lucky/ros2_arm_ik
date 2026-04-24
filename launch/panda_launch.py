import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 自动定位功能包的安装位置，不再需要桌面路径
    pkg_share = get_package_share_directory('panda_description')
    cube_urdf_path = os.path.join(pkg_share, 'urdf', 'cube.urdf')
    target_circle_urdf_path = os.path.join(pkg_share, 'urdf', 'target_circle.urdf')
    # 指向你的模型文件
    urdf_file = os.path.join(pkg_share, 'urdf', 'panda_fixed.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 环境变量：让 Gazebo 能够找到模型文件 (meshes)
    set_gz_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=os.path.join(pkg_share, '..')
    )

    # 1. 启动 Gazebo 物理仿真
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r empty.sdf '}.items(),
    )

    # 2. 启动机器人状态发布者 (把 URDF 发给 RViz 看)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 3. 启动 RViz 2 界面
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': True}]
    )

    # 4. 在 Gazebo 中生成机器人
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'panda', '-file', urdf_file, '-z', '0.1']
    )

    # 5. 时间桥接器：同步仿真时间 (解决紫色警告)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    spawn_cube = Node(
        package = 'ros_gz_sim',
        executable = 'create',
        arguments = [
            '-file', cube_urdf_path,
            '-name', 'my_big_cube',
            '-x', '0.5',
            '-y', '0.0',
            '-z', '0.06',
        ],
        output = 'screen',
    )
    spawn_target_circle = Node(
        package = 'ros_gz_sim',
        executable = 'create',
        arguments = [
            '-file', target_circle_urdf_path,
            '-name', 'target_platform',
            '-x', '-0.6',
            '-y', '-0.4',
            '-z', '0.025',
        ],
        output = 'screen',
    )

    vision_bridge = Node(
        package = 'ros_gz_bridge',
        executable = 'parameter_bridge',
        arguments = [
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'

        ],
        output = 'screen'
    )

    # 6. 加载控制器
    load_jsb = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'])
    load_arm = Node(package='controller_manager', executable='spawner', arguments=['panda_arm_controller'])
    load_hand = Node(package='controller_manager', executable='spawner', arguments=['panda_hand_controller'])
    return LaunchDescription([
        set_gz_path,
        gz_sim,
        bridge,
        rsp,
        rviz,
        spawn,
        load_jsb,
        load_arm,
        load_hand,
        spawn_cube,
        spawn_target_circle,
        vision_bridge
    ])
