cmake_minimum_required(VERSION 3.10.2)
# 1. 项目名称：必须和 package.xml 里的 <name> 标签一模一样
project(panda_description)

find_package(ament_cmake REQUIRED)

# 2. 安装指令：这是解决你“RViz 没图像”和“Launch 找不到”的核心
# 它的意思是：把这四个文件夹完整地复制到系统的“公共安装区”
install(DIRECTORY 
  meshes 
  urdf 
  config   # <--- 必须加，否则控制器报错
  launch   # <--- 必须加，否则 ros2 launch 找不到文件
  DESTINATION share/${PROJECT_NAME}
)

# 3. 结尾：宣告这是一个正式的 ROS 2 功能包
ament_package()
