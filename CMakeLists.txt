cmake_minimum_required(VERSION 2.8.3)
project(imu_core)

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

set(SRCS_IMU_TEST
  ${src_directory}/test.cpp
)

set(SRCS_IMU_TEST_INTERFACE
  ${src_directory}/test.cpp
)


add_library(imu-core ${SRCS_IMU})
target_link_libraries(imu-core ${LAB_STD_LIBS} ${Boost_LIBRARIES})

add_executable(imu-test ${SRCS_IMU_TEST})
target_link_libraries(imu-test imu-core)

add_executable(imu-test-interface ${SRCS_IMU_TEST_INTERFACE})
target_link_libraries(imu-test-interface imu-core)

catkin_package(
  INCLUDE_DIRS ${header_directory}
  LIBRARIES imu-core
)

