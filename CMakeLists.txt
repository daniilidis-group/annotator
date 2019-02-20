cmake_minimum_required(VERSION 2.8)
project(annotator)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall")

find_package(catkin REQUIRED COMPONENTS roscpp rospy
		    rosbag cv_bridge
		    image_transport ffmpeg_image_transport_msgs
		    audio_common_msgs
		    ffmpeg_image_transport
		    rosgraph_msgs
		    sensor_msgs flex_sync message_generation)

add_service_files(DIRECTORY srv FILES PlayerCmd.srv)

generate_messages()

catkin_package(CATKIN_DEPENDS
roscpp
rosbag
flex_sync
ffmpeg_image_transport_msgs
sensor_msgs
message_runtime
ffmpeg_image_transport
)

find_package(OpenCV)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

# add the executables

add_executable(bag_player src/bag_player.cpp src/bag_player_node.cpp)
add_dependencies(bag_player ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bag_player ${catkin_LIBRARIES})
