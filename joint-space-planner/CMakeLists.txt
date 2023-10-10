project(joint_space_planner)

cmake_minimum_required(VERSION 3.0)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -lrt")

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(joint_space_planner joint_space_planner.cpp joint_space_planner.h)
target_link_libraries (joint_space_planner Eigen3::Eigen MotionPlanning)

