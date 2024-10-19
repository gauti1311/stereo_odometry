cmake_minimum_required(VERSION 3.8)
project(stereo_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(Ceres REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Set C++ version
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(
  include
  # ${CERES_INCLUDE_DIRS}  # For Ceres solver
)

# Set source files
set(SOURCES
  src/stereo_odometry_node.cpp
  src/feature.cpp
  src/Frame.cpp
  src/visualOdometry.cpp
  src/utils.cpp
  src/bucket.cpp
  # src/optimizer.cpp
  # src/evaluate_odometry.cpp
  # src/matrix.cpp
  # Add other source files here if necessary
)

# Add executable
add_executable(stereo_odometry_node ${SOURCES})

# Link necessary libraries
target_link_libraries(stereo_odometry_node
  ${OpenCV_LIBS}
  ${YAML_CPP_LIBRARIES}
  # ${CERES_LIBRARIES}
)

# Add ament dependencies
ament_target_dependencies(stereo_odometry_node
  rclcpp
  sensor_msgs
  nav_msgs
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
  yaml-cpp
)

# Install include files
install(DIRECTORY include/
  DESTINATION include/
)


# Install directories
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install the executable
install(TARGETS stereo_odometry_node
  DESTINATION lib/${PROJECT_NAME})

# Mark the package as an ament package
ament_package()
