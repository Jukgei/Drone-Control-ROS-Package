cmake_minimum_required(VERSION 2.8.3)
project(FlightControl)


find_package(catkin REQUIRED COMPONENTS
        std_msgs
        nav_msgs
        roscpp
        geometry_msgs
        dji_sdk
        image_transport # for message_filter
        )

add_message_files(FILES state.msg opticalflow.msg)
find_package(DJIOSDK REQUIRED)

find_package(darknet_ros QUIET)
if(darknet_ros_FOUND)
    message(STATUS "Found darknet_ros package, will use it for object depth perception demo.")
    add_definitions(-DUSE_DARKNET_ROS)
endif()

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


generate_messages(DEPENDENCIES std_msgs)
catkin_package()

add_executable(flightcontrol
        src/main.cpp src/flightcontrol.cpp src/pid.cpp)

target_link_libraries(flightcontrol
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(flightcontrol dji_sdk_generate_messages_cpp)
