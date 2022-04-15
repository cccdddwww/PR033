cmake_minimum_required(VERSION 2.8.3)
project(pcl_image)

add_compile_options(-std=c++11)
add_definitions("-Wall -g -o0")

find_package(OpenCV)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
rospy
sensor_msgs
cv_bridge
std_msgs
geometry_msgs
message_generation
image_transport

)

add_message_files(
  FILES
  pointmatrix.msg
  Point.msg
)
generate_messages(
 DEPENDENCIES
 geometry_msgs
 std_msgs
)



catkin_package(
  INCLUDE_DIRS include 
  CATKIN_DEPENDS roscpp rospy sensor_msgs pcl_ros message_runtime geometry_msgs std_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})


add_executable(pcl_test_node src/pcl_test_node.cpp src/pcl_test_core.cpp)

add_executable(pcl_filter_node src/pcl_filter_node.cpp src/pcl_filter_core.cpp)

add_executable(projection_node src/projection.cpp)
add_executable(publisher src/publisher.cpp)
target_link_libraries(pcl_test_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(projection_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(pcl_filter_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(publisher
 ${catkin_LIBRARIES}

)
add_dependencies(publisher ${PROJECT_NAME}_generate_messages_cpp)

