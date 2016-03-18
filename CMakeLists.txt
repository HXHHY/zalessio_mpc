cmake_minimum_required(VERSION 2.8.3)
project(rpg_slq_mpc)

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
cs_add_executable(rk4_test src/rk4_test.cpp  src/sequential_linear_quadratic.cpp )
cs_export()
