cmake_minimum_required(VERSION 3.8)
project(dummy_real_v3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 构建依赖
find_package(ament_cmake REQUIRED)

# 安装所有需要的资源目录
install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
