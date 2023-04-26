cmake_minimum_required(VERSION 3.8)
project(sample1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(my_interface REQUIRED)

# 実行可能ファイルを指定
add_executable(client src/client.cpp)
ament_target_dependencies(client rclcpp rclcpp_action my_interface) 

add_executable(server src/server.cpp)
ament_target_dependencies(server rclcpp rclcpp_action my_interface) 

install(TARGETS
    client
    server
    DESTINATION lib/${PROJECT_NAME})

ament_package()
