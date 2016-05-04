cmake_minimum_required(VERSION 2.8.3)
project(ahl_gazebo_interface)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0x)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(
  catkin REQUIRED COMPONENTS
    ahl_utils
    cmake_modules
    gazebo_msgs
    roscpp
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ahl_gazebo_interface
  CATKIN_DEPENDS
    ahl_utils
    gazebo_msgs
    roscpp
)

add_library(ahl_gazebo_interface
  src/gazebo_interface.cpp
  src/torque_sensor.cpp
  src/force_sensor.cpp
)
add_dependencies(ahl_gazebo_interface gazebo_msgs_gencpp)
target_link_libraries(ahl_gazebo_interface ${catkin_LIBRARIES})
