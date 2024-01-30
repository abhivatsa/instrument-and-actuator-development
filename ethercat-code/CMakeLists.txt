cmake_minimum_required(VERSION 3.0)
project(ecat_master)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(EtherCAT REQUIRED)

add_executable(ecat_master master.cpp)

target_link_libraries(ecat_master PUBLIC EtherLab::ethercat)


