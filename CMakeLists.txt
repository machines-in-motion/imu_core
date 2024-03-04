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
find_package(pybind11 REQUIRED)

set(all_targets)

#
# Create the main library.
#
set(SRCS_IMU src/imu_3DM_GX3_45.cpp
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

add_imu_core_demo(demo_imu_3DM_GX3_45)

#
# building documentation
#
add_documentation()

# python bindings
set(py_imu_core_SRC_FILES
  srcpy/imu_core_cpp.cpp
)

pybind11_add_module(imu_core_cpp MODULE ${py_imu_core_SRC_FILES})
target_link_libraries(imu_core_cpp PRIVATE pybind11::module)
target_link_libraries(imu_core_cpp PRIVATE imu_core)

# install the bindings
get_python_install_dir(python_install_dir)
install(TARGETS imu_core_cpp DESTINATION ${python_install_dir}/${PROJECT_NAME})

# install python package
install(
  DIRECTORY python/${PROJECT_NAME}
  DESTINATION "${python_install_dir}"
  PATTERN "*.pyc" EXCLUDE 
  PATTERN "__pycache__" EXCLUDE
)
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
