cmake_minimum_required(VERSION 2.8.3)
project(surgery_perception)
add_definitions(-std=c++17)
set(CMAKE_CXX_STANDARD 17)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  open3d_conversions
  message_filters
)

# find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED) 
find_package(Open3D REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR}) 
include_directories(${Open3D_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
CATKIN_DEPENDS roscpp std_msgs sensor_msgs roslib open3d_conversions
DEPENDS Open3D
)

add_executable(poisson_reconstruction src/poisson_reconstruction.cpp)
target_link_libraries(poisson_reconstruction ${catkin_LIBRARIES} ${Open3D_LIBRARIES})
target_include_directories(poisson_reconstruction PUBLIC ${Open3D_INCLUDE_DIRS})

add_executable(global_registration src/global_registration.cpp)
target_link_libraries(global_registration ${catkin_LIBRARIES} ${Open3D_LIBRARIES})
target_include_directories(global_registration PUBLIC ${Open3D_INCLUDE_DIRS})


# add_executable(perception_node src/perception_node.cpp)
# add_executable(lvr2_reconstruction src/lvr2_reconstruction.cpp)
# target_link_libraries(perception_node ${catkin_LIBRARIES})
# target_link_libraries(lvr2_reconstruction ${catkin_LIBRARIES} ${PCL_LIBRARIES})


# catkin_install_python(PROGRAMS
#     scripts/poisson_reconstruction.py
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

