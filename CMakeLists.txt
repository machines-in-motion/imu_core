###################
# standard header # 
###################
cmake_minimum_required(VERSION 2.8.3)

# required to use std::shared_ptr in code
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")

# ensuring path to libraries are set during install
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

project(imu_core)

#####################
# find dependencies #
#####################
find_package(catkin REQUIRED COMPONENTS
  mpi_cmake_modules
  real_time_tools
)
find_package(X11 REQUIRED)
search_for_boost()

####################################
# Define the OS to be used locally #
####################################

# This macro sets the C++ preprocessor flags "XENOMAI", "RT_PREEMPT", or
# "NON_REAL_TIME" according to the current operating system.
define_os()

#############################
# get the include directory #
#############################
include_directories(BEFORE ${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})

#############################
# create the catkin package #
#############################
catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
  LIBRARIES imu-core
  DEPENDS real_time_tools
)

###########################
# create the main library #
###########################
set(src_directory ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(SRCS_IMU
  ${src_directory}/imu_3DM_GX3_25.cpp
  # ${src_directory}/imu_getter.cpp
)
add_library(imu-core ${SRCS_IMU})
target_link_libraries(imu-core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

##################################
# debugging or analytic programs #
##################################
set(programs_directory ${src_directory}/programs)

add_executable(imu-test ${programs_directory}/test.cpp)
target_link_libraries(imu-test imu-core real_time_tools)

# add_executable(imu-test-interface ${programs_directory}/imu-test-interface.cpp)
# target_link_libraries(imu-test-interface imu-core real_time_tools)

##########################
# building documentation #
##########################
build_doxygen_documentation()