cmake_minimum_required(VERSION 2.8.3)
project(imu-ros-rt)

include("${CMAKE_SOURCE_DIR}/cmake/sl_cmake_global/LAB.cmake")

find_package(xenomai_ros QUIET)

if(Xenomai_FOUND)
  message("COMPILING FOR XENOMAI !")
  add_xenomai_flags()
  include_directories( ${Xenomai_INCLUDE_DIR} )
endif()

find_package(X11 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

set(header_directory  ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(src_directory     ${CMAKE_CURRENT_SOURCE_DIR}/src)

include_directories(BEFORE ${header_directory})

set(SRCS_IMU
  ${src_directory}/ImuInterface.cpp
)

add_library(imu-ros-rt ${SRCS_IMU})
target_link_libraries(imu-ros-rt ${LAB_STD_LIBS} ${Boost_LIBRARIES})


catkin_package(
  INCLUDE_DIRS ${header_directory}
  LIBRARIES imu-ros-rt
)

