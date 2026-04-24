本项目旨在实现机械臂视觉识别物体并抓取，现在已经基本实现功能，但是还存在
夹爪偏移的问题没有解决。

启动gazebo仿真和rviz
ros2 launch panda_description panda_launch.py

单纯的依靠坐标抓取
python3 panda_pick.py

视觉识别抓取
python3 box_follow.py

