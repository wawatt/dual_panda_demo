cmake_minimum_required(VERSION 3.20)
project(dual_panda_demo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)


find_package(moveit_common REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs  REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometric_shapes REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)


add_executable(my_move_group
    src/my_move_group.cpp)     
ament_target_dependencies(my_move_group
    rclcpp moveit_msgs ament_cmake rclcpp_action geometry_msgs tf2_ros
    moveit_core moveit_ros_planning_interface tf2_geometry_msgs moveit_ros_planning Eigen3
    visualization_msgs moveit_visual_tools)  

# add_executable(dual_planning
#     src/dual_planning.cpp)     
# ament_target_dependencies(dual_planning 
#     rclcpp moveit_msgs ament_cmake rclcpp_action geometry_msgs tf2_ros
#     moveit_core moveit_ros_planning_interface tf2_geometry_msgs moveit_ros_planning Eigen3 kurn3d_interfaces
#     )   


# add_executable(move_group
#     src/move_group.cpp)     
# ament_target_dependencies(move_group 
#     rclcpp moveit_msgs ament_cmake rclcpp_action geometry_msgs tf2_ros
#     moveit_core moveit_ros_planning_interface tf2_geometry_msgs moveit_ros_planning Eigen3 kurn3d_interfaces
#     )   

# add_executable(NowAndWant_new
#     src/NowAndWant_new_mc.cpp)     
# ament_target_dependencies(NowAndWant_new 
#     rclcpp moveit_msgs ament_cmake rclcpp_action geometry_msgs tf2_ros
#     moveit_core moveit_ros_planning_interface tf2_geometry_msgs moveit_ros_planning Eigen3 kurn3d_interfaces
#     moveit_visual_tools
# )

# add_executable(mc_multi_pipelines_NO_UPDOWN
#     src/mc_multi_pipelines.cpp)     
# ament_target_dependencies(mc_multi_pipelines_NO_UPDOWN 
#     rclcpp moveit_msgs ament_cmake rclcpp_action geometry_msgs tf2_ros
#     moveit_core moveit_ros_planning_interface tf2_geometry_msgs moveit_ros_planning Eigen3 kurn3d_interfaces
#     moveit_visual_tools
# )

# add_executable(mc_multi_pipelines_ADD_UPDOWN_OPJS
#     src/mc_multi_pipelines.cpp)     
# target_compile_definitions(mc_multi_pipelines_ADD_UPDOWN_OPJS PRIVATE ADD_UPDOWN_OPJS)
# ament_target_dependencies(mc_multi_pipelines_ADD_UPDOWN_OPJS
#     rclcpp moveit_msgs ament_cmake rclcpp_action geometry_msgs tf2_ros
#     moveit_core moveit_ros_planning_interface tf2_geometry_msgs moveit_ros_planning Eigen3 kurn3d_interfaces
#     moveit_visual_tools
# )

# add_executable(mc_multi_pipelines_NO_OCC_OPJS
#     src/mc_multi_pipelines.cpp)     
# target_compile_definitions(mc_multi_pipelines_NO_OCC_OPJS PRIVATE NO_OCC_OPJS)
# ament_target_dependencies(mc_multi_pipelines_NO_OCC_OPJS
#     rclcpp moveit_msgs ament_cmake rclcpp_action geometry_msgs tf2_ros
#     moveit_core moveit_ros_planning_interface tf2_geometry_msgs moveit_ros_planning Eigen3 kurn3d_interfaces
#     visualization_msgs moveit_visual_tools)  

# add_executable(mc_multi_pipelines_JINGMEN2
#     src/mc_multi_pipelines.cpp)     
# target_compile_definitions(mc_multi_pipelines_JINGMEN2 PRIVATE ADD_UPDOWN_OPJS)
# ament_target_dependencies(mc_multi_pipelines_JINGMEN2
#     rclcpp moveit_msgs ament_cmake rclcpp_action geometry_msgs tf2_ros
#     moveit_core moveit_ros_planning_interface tf2_geometry_msgs moveit_ros_planning Eigen3 kurn3d_interfaces
#     visualization_msgs moveit_visual_tools)  

# add_executable(mc_multi_pipelines_JZJ_ROBOT
#     src/mc_multi_pipelines.cpp)     
# target_compile_definitions(mc_multi_pipelines_JZJ_ROBOT PRIVATE JZJ_ROBOT ADD_UPDOWN_OPJS)
# ament_target_dependencies(mc_multi_pipelines_JZJ_ROBOT
#     rclcpp moveit_msgs ament_cmake rclcpp_action geometry_msgs tf2_ros
#     moveit_core moveit_ros_planning_interface tf2_geometry_msgs moveit_ros_planning Eigen3 kurn3d_interfaces
#     visualization_msgs moveit_visual_tools)  

# add_executable(mc_multi_pipelines_magna_R
#     src/mc_multi_pipelines.cpp)     
# target_compile_definitions(mc_multi_pipelines_magna_R PRIVATE NO_OCC_OPJS)
# ament_target_dependencies(mc_multi_pipelines_magna_R
#     rclcpp moveit_msgs ament_cmake rclcpp_action geometry_msgs tf2_ros
#     moveit_core moveit_ros_planning_interface tf2_geometry_msgs moveit_ros_planning Eigen3 kurn3d_interfaces
#     visualization_msgs moveit_visual_tools)  

# add_executable(mc_multi_pipelines_magna_L
#     src/mc_multi_pipelines.cpp)     
# target_compile_definitions(mc_multi_pipelines_magna_L PRIVATE JZJ_ROBOT NO_OCC_OPJS)
# ament_target_dependencies(mc_multi_pipelines_magna_L
#     rclcpp moveit_msgs ament_cmake rclcpp_action geometry_msgs tf2_ros
#     moveit_core moveit_ros_planning_interface tf2_geometry_msgs moveit_ros_planning Eigen3 kurn3d_interfaces
#     visualization_msgs moveit_visual_tools)  

# add_executable(mc_one_pipelines
#     src/mc_one_pipelines.cpp)     
# target_compile_definitions(mc_one_pipelines PRIVATE JZJ_ROBOT)
# ament_target_dependencies(mc_one_pipelines
#     rclcpp moveit_msgs ament_cmake rclcpp_action geometry_msgs tf2_ros
#     moveit_core moveit_ros_planning_interface tf2_geometry_msgs moveit_ros_planning Eigen3 kurn3d_interfaces
#     visualization_msgs moveit_visual_tools)  
 

# add_executable(test_client
#     src/test_client.cpp)     
# ament_target_dependencies(test_client
#     rclcpp ament_cmake kurn3d_interfaces)


# add_executable(MyPlan
#     src/MyPlan.cpp)     
# target_compile_definitions(MyPlan PRIVATE JZJ_ROBOT)
# ament_target_dependencies(MyPlan
#     rclcpp moveit_msgs ament_cmake rclcpp_action geometry_msgs tf2_ros
#     moveit_core moveit_ros_planning_interface tf2_geometry_msgs moveit_ros_planning Eigen3 kurn3d_interfaces
#     visualization_msgs moveit_visual_tools) 

# add_executable(MyPlan_client
#     src/MyPlan_client.cpp)     
# ament_target_dependencies(MyPlan_client
#     rclcpp ament_cmake kurn3d_interfaces)

install(TARGETS
    my_move_group
    DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)  

ament_package()
