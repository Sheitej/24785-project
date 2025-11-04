cmake_minimum_required(VERSION 3.5)
project(lo_dev)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++1z")
set(CMAKE_CXX_FLAGS_RELEASE "-O4 -Wall -g")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(TBB_INCLUDE_DIR "/usr/include/tbb")
add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter)


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)  # added to enable OusterPointXYZIRT format
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(TBB REQUIRED)  # not used but keep
find_package(tsl-robin-map REQUIRED)
# find_package(livox_ros_driver2 REQUIRED)

find_package(OpenMP REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${TBB_INCLUDE_DIR}
  ${Sophus_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${tsl-robin-map_INCLUDE_DIRS}
  # ${livox_ros_driver2_INCLUDE_DIRS}
)

# Link directories
link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
  ${tsl-robin-map_LIBRARY_DIRS}
)

set(GTSAM_LIBRARIES gtsam)
# set(tsl-robin-map_LIBRARIES tsl-robin-map)

# Define Library
add_library(LoDevLib SHARED
  src/Frontend/frontend.cpp
  src/parameter/parameter.cpp
  src/VoxelMap/VoxelHashMap.cpp
  src/VoxelMap/VoxelUtils.cpp
)

# Set up library include directory
target_include_directories(LoDevLib PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/LoDevLib>  
    $<INSTALL_INTERFACE:include/LoDevLib>  # <prefix>/include/mylib
)

ament_target_dependencies(LoDevLib
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs 
  sensor_msgs
  image_transport
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  visualization_msgs
  pcl_conversions
  # livox_ros_driver2
)

target_link_libraries(LoDevLib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  gtsam
  ${CERES_LIBRARIES}
  ${tsl-robin-map_LIBRARIES}
  OpenMP::OpenMP_CXX
)

# Frontend node
add_executable(frontend_node src/frontend_node.cpp)
ament_target_dependencies(frontend_node rclcpp nav_msgs sensor_msgs tf2_ros tf2_eigen pcl_conversions pcl_msgs PCL)
target_link_libraries(frontend_node
  gtsam
  LoDevLib
#  ${tsl-robin-map_LIBRARIES}
#  OenMP::OpenMP_CXX
)

# Install Exported Libraries
ament_export_targets(export_lo_dev HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS LoDevLib
  EXPORT export_lo_dev
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(LoDevLib)

#Install executables
install(TARGETS
  frontend_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
