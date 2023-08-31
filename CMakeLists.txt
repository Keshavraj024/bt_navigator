cmake_minimum_required(VERSION 3.8)
project(bt_navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)

set(TARGET_DEPENDS
    rclcpp
    behaviortree_cpp
    yaml-cpp
    rclcpp_action
    nav2_msgs
    tf2
    tf2_geometry_msgs
)

install(DIRECTORY
        launch
        maps
        config
        bt_xml
        worlds
        DESTINATION share/${PROJECT_NAME})
        
        
include_directories(include)

add_executable(init_pose_publisher src/init_pose_publisher.cpp)
ament_target_dependencies(init_pose_publisher rclcpp geometry_msgs tf2)

add_executable(bt_node src/bt_wrapper.cpp src/behaviour_tree_action_node.cpp)
ament_target_dependencies(bt_node ${TARGET_DEPENDS})
target_link_libraries(bt_node ${YAML_CPP_LIBRARIES})

install(TARGETS
        init_pose_publisher
        bt_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
