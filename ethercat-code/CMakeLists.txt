project(ecat_config_publisher_project)

cmake_minimum_required(VERSION 3.0)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(EtherCAT REQUIRED)
find_package(iceoryx_posh CONFIG REQUIRED)
find_package(iceoryx_hoofs CONFIG REQUIRED)
get_target_property(ICEORYX_CXX_STANDARD iceoryx_posh::iceoryx_posh CXX_STANDARD)

include(IceoryxPlatform)
include(IceoryxPackageHelper)

add_executable(ecat_config_publisher master.cpp master.h)

target_link_libraries(ecat_config_publisher PUBLIC iceoryx_posh::iceoryx_posh EtherLab::ethercat )