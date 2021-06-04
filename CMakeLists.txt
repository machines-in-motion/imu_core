#
# Standard header
#
cmake_minimum_required(VERSION 3.10.2)

project(imu_core VERSION 1.0.0)

# Using C++17
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#
# Find dependencies.
#

find_package(mpi_cmake_modules REQUIRED)
find_package(real_time_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system thread)

set(all_targets)

#
# Create the main library.
#
set(SRCS_IMU src/imu_3DM_GX3_25.cpp src/imu_3DM_GX5_25.cpp
             src/imu_interface.cpp)
add_library(${PROJECT_NAME} SHARED ${SRCS_IMU})
# Add the include dependencies
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} Boost::boost)
target_link_libraries(${PROJECT_NAME} Boost::filesystem)
target_link_libraries(${PROJECT_NAME} Boost::system)
target_link_libraries(${PROJECT_NAME} Boost::thread)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} real_time_tools::real_time_tools)

# For the installation
list(APPEND all_targets ${PROJECT_NAME})

#
# debugging or analytic programs
#

macro(add_imu_core_program source_file)
  add_executable(${source_file} src/programs/${source_file}.cpp)
  target_link_libraries(${source_file} ${PROJECT_NAME})

  # save the name for installation
  set(all_targets ${all_targets} ${source_file})
endmacro(add_imu_core_program source_file)

add_imu_core_program(reset_3DM_GX3_25)

#
# Demos
#
macro(add_imu_core_demo source_file)
  add_executable(${source_file} demos/${source_file}.cpp)

  # dependencies
  target_include_directories(
    ${source_file} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:include>)

  target_link_libraries(${source_file} ${PROJECT_NAME})

  # Save the name for installation
  set(all_targets ${all_targets} ${source_file})
endmacro(add_imu_core_demo source_file)

add_imu_core_demo(demo_imu_3DM_GX3_25)
add_imu_core_demo(demo_imu_3DM_GX5_25)

#
# building documentation
#
add_documentation()

#
# Install the package
#

# install the include directory
install(DIRECTORY include/ DESTINATION include)

# command to install the library and binaries
install(
  TARGETS ${all_targets}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

# Export this package as a cmake package.
generate_cmake_package()
