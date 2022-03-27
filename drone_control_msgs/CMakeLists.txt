cmake_minimum_required(VERSION 2.8.3)
project(drone_control_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs)

#include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES
  GetDroneState.msg
  SetDroneState.msg
)

#add_service_files(
#
#)

# add_action_files(
#   DIRECTORY action
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs)

