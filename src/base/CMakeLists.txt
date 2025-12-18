cmake_minimum_required(VERSION 3.8)
project(base)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 1. Generate Custom Messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelVelocities.msg"
  DEPENDENCIES std_msgs
)

# 2. Build Core Math Libraries (No ROS dependencies)
add_library(base_math_lib
  src/library/kinematics_calculator.cpp
)
target_include_directories(base_math_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- Hardware-Lib (SSC32, Phidgets) ---
add_library(diff_drive_lib src/library/diff_drive_lib.cpp)

target_include_directories(diff_drive_lib PUBLIC 
    include
    /usr/local/include   # Pfad zur manuell installierten phidget21.h
)

target_link_libraries(diff_drive_lib phidget21)

# 3. Build Nodes

# --- Kinematics Node ---
add_executable(kinematics_node src/nodes/kinematics_node.cpp)

# Link internal math library
target_link_libraries(kinematics_node base_math_lib)

# Link ROS dependencies using the modern 'ament' macro
ament_target_dependencies(kinematics_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
# Link custom messages generated above
rosidl_target_interfaces(kinematics_node ${PROJECT_NAME} "rosidl_typesupport_cpp")


# --- Hardware Node ---
add_executable(hardware_node src/nodes/hardware_node.cpp)
target_link_libraries(hardware_node diff_drive_lib base_math_lib)
# Externe ROS-Pakete (KEIN 'base' hier!)
ament_target_dependencies(hardware_node 
  rclcpp 
  sensor_msgs
)
# Eigene Messages linkt man hiermit:
rosidl_target_interfaces(hardware_node ${PROJECT_NAME} "rosidl_typesupport_cpp")


# 4. Install (Install-Abschnitt aktualisieren)
install(TARGETS base_math_lib kinematics_node hardware_node diff_drive_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/ DESTINATION include)
# Verzeichnisnamen prüfen: Heißt es 'params' oder 'config'?
install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(base_math_lib diff_drive_lib)
ament_package()
