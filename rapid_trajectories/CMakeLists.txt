cmake_minimum_required(VERSION 2.8.3)
project(rapid_trajectories)

find_package(catkin_simple REQUIRED)
catkin_simple()
quad_cmake_setup()

cs_add_library(${PROJECT_NAME} src/RapidTrajectoryGenerator.cpp src/SingleAxisTrajectory.cpp)

#cs_add_executable(test_rapid_trajectory src/Demo.cpp src/RapidTrajectoryGenerator.cpp src/SingleAxisTrajectory.cpp)
#cs_add_executable(window_flight_rapid_trajectories src/test_window_flight.cpp src/RapidTrajectoryGenerator.cpp src/SingleAxisTrajectory.cpp)

cs_export()
