###################
# standard header # 
###################
cmake_minimum_required(VERSION 3.1)

# required to use std::shared_ptr in code and to link the python bindings
if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-as-needed")
endif()
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# Use C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED on)

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
search_for_boost()
search_for_eigen()

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
set(SRCS_IMU
  src/imu_3DM_GX3_25.cpp
  src/imu_3DM_GX5_25.cpp
  src/imu_interface.cpp
)
add_library(imu-core ${SRCS_IMU})
target_link_libraries(imu-core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

##################################
# debugging or analytic programs #
##################################

macro(add_imu_core_program source_file)
  add_executable(${source_file} src/programs/${source_file}.cpp)
  target_link_libraries(${source_file} imu-core)
endmacro(add_imu_core_program source_file)

add_imu_core_program(reset_3DM_GX3_25)



#########
# Demos #
#########
macro(add_imu_core_demo source_file)
  add_executable(${source_file} demos/${source_file}.cpp)
  target_link_libraries(${source_file} imu-core)
endmacro(add_imu_core_demo source_file)

add_imu_core_demo(demo_imu_3DM_GX3_25)
add_imu_core_demo(demo_imu_3DM_GX5_25)

##########################
# building documentation #
##########################
build_doxygen_documentation()