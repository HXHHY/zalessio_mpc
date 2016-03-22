cmake_minimum_required(VERSION 2.8.3)
project(mpc_framework)

find_package(catkin_simple REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_simple()
cs_add_executable(dynamics_test src/uav_dynamics.cpp )
cs_export()
