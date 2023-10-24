project(instrument_motion_planner)

cmake_minimum_required(VERSION 3.0)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -lrt")

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(instrument_motion_planner instrument_motion_planner.cpp instrument_motion_planner.h instrument_jog.h pt_to_pt_planner.h sterile_engagement.h)
target_link_libraries (instrument_motion_planner Eigen3::Eigen MotionPlanning)

