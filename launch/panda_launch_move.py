import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command  # 关键：用于解析 xacro

def generate_launch_description():
    # 1. 路径配置
    pkg_share = get_package_share_directory('panda_description')
    
    xacro_file = os.path.join(pkg_share, 'urdf', 'mobile_panda.urdf.xacro')
    
    cube_urdf_path = os.path.join(pkg_share, 'urdf', 'cube.urdf')
    target_circle_urdf_path = os.path.join(pkg_share, 'urdf', 'target_circle.urdf')

    robot_description_content = Command(['xacro ', xacro_file])

    set_gz_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=os.path.join(pkg_share, '..')
    )


    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r empty.sdf '}.items(),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content, 
            'use_sim_time': True
        }]
    )

    spawn_panda = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'mobile_panda',
            '-topic', 'robot_description',
            '-z', '0.1'
        ],
        output='screen'
    )


    spawn_cube = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', cube_urdf_path, '-name', 'my_big_cube', '-x', '0.7', '-y', '0.0', '-z', '0.06'],
        output='screen'
    )
    
    spawn_target_circle = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', target_circle_urdf_path, '-name', 'target_platform', '-x', '-0.6', '-y', '-0.4', '-z', '0.025'],
        output='screen'
    )


    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # 如果有底盘相机，在此处继续添加桥接项
            '/model/mobile_panda/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/mobile_panda/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            
            '/base_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            'base_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        # remappings = [
        #     ('/model/mobile_panda/cmd_vel', 'cmd_vel'),
        #     ('/model/mobile_panda/odometry', 'odom')
        # ],
        output='screen'
    )


    load_jsb = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'])
    load_arm = Node(package='controller_manager', executable='spawner', arguments=['panda_arm_controller'])
    load_hand = Node(package='controller_manager', executable='spawner', arguments=['panda_hand_controller'])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        set_gz_path,
        gz_sim,
        bridge,
        rsp,
        rviz,
        spawn_panda,
        spawn_cube,
        spawn_target_circle,
        load_jsb,
        load_arm,
        load_hand
    ])