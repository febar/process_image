cmake_minimum_required(VERSION 2.8.3)
project(stream_video)

find_package(OpenCV REQUIRED)
find_package(PCL 1.2 REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  cv_bridge
  camera_info_manager
  std_srvs
  sensor_msgs
  pcl_conversions
  image_transport
)


include_directories(${catkin_INCLUDE_DIRS} include)
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  
)

add_executable(main_vision src/main_vision.cpp src/Cvision.cpp)
target_link_libraries(main_vision 
  ${PCL_LIBRARIES} 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS}
)



