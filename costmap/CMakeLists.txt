cmake_minimum_required(VERSION 2.8.3)
project(costmap)

#add_compile_options(-std=c++11)
add_compile_options(-std=c++11 -lcurses -pthread -O2  -Wpointer-arith -ffast-math -funsafe-math-optimizations)
add_definitions("-Wall -g")

find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
sensor_msgs
cv_bridge
image_transport
geometry_msgs
message_generation
std_msgs
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
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros message_runtime geometry_msgs std_msgs
)

file(GLOB_RECURSE SRC_FILE ${PROJECT_SOURCE_DIR}/detect2/*.cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/detect2/
)
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})


add_executable(costmap_node src/costmap_2d_node.cpp src/costmap_ros.cpp src/costmap.cpp src/costmap_2d_publisher.cpp ${SRC_FILE})


target_link_libraries(costmap_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)


add_dependencies(costmap_node ${PROJECT_NAME}_generate_messages_cpp)
