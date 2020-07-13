cmake_minimum_required(VERSION 2.8.3)
project(storage_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  pcl_conversions
  cv_bridge
  sensor_msgs
)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Describe catkin Project
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(save_pointcloud_PointXYZ src/save_pointcloud_PointXYZ.cpp)
target_link_libraries(save_pointcloud_PointXYZ ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(save_pointcloud_PointXYZRGB src/save_pointcloud_PointXYZRGB.cpp)
target_link_libraries(save_pointcloud_PointXYZRGB ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(save_pointcloud_PointXYZI src/save_pointcloud_PointXYZI.cpp)
target_link_libraries(save_pointcloud_PointXYZI ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(save_rgbd src/save_rgbd.cpp)
target_link_libraries(save_rgbd ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(save_rgb_only src/save_rgb_only.cpp)
target_link_libraries(save_rgb_only ${catkin_LIBRARIES} ${OpenCV_LIBS})
