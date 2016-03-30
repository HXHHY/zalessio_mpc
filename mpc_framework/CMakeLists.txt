cmake_minimum_required(VERSION 2.8.3)
project(mpc_framework)

find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  quad_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(SOURCE_FILES
  src/mpc_calculations.cpp
)

cs_add_executable(dynamics_test src/uav_dynamics.cpp )

cs_add_library(${PROJECT_NAME}_lib ${SOURCE_FILES})

cs_add_executable(rpg_mpc_framework src/mpc_framework.cpp)
target_link_libraries(rpg_mpc_framework ${PROJECT_NAME}_lib)

cs_install()
cs_export()
