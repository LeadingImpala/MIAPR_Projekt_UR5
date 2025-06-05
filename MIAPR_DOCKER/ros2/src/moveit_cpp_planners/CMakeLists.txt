cmake_minimum_required(VERSION 3.8)
project(moveit_cpp_planners)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Required dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 20)

# === Source files ===
set(SOURCES
  src/hello_moveit.cpp
  src/kinematics.cpp
  src/check_collisions.cpp
  src/planning_scene_ompl.cpp
  src/planning_scene_stomp.cpp
  src/planning_scene_chomp.cpp
  src/planning_scene_service.cpp
)

# === Executables ===
foreach(source_file ${SOURCES})
  get_filename_component(exe_name ${source_file} NAME_WE)
  add_executable(${exe_name} ${source_file})
  ament_target_dependencies(${exe_name}
    rclcpp
    moveit_ros_planning_interface
    moveit_visual_tools
    Eigen3
    tf2_geometry_msgs
  )
  install(TARGETS ${exe_name}
    DESTINATION lib/${PROJECT_NAME})
endforeach()

# === Install launch files ===
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# === Install config files ===
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
