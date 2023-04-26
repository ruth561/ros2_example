cmake_minimum_required(VERSION 3.8)
project(sample0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_interface REQUIRED)

# 実行可能ファイルを指定
add_executable(chat_node src/chat_node.cpp)
ament_target_dependencies(chat_node rclcpp my_interface) 

install(TARGETS
  chat_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
