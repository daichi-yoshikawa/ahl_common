cmake_minimum_required(VERSION 2.8.3)
project(gl_wrapper)

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

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    cv_bridge
    cv_wrapper
    dynamic_reconfigure
    std_msgs
    std_srvs
    ahl_utils
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
  LIBRARIES gl_wrapper
  CATKIN_DEPENDS
    cmake_modules
    cv_bridge
    cv_wrapper
    dynamic_reconfigure
    std_msgs
    std_srvs
    ahl_utils
    roscpp
)

add_library(gl_wrapper
  src/render/render.cpp
  src/render/param.cpp
  src/render/display.cpp
  src/render/light.cpp
  src/render/camera.cpp
  src/render/material.cpp
  src/render/quaternion.cpp
  src/interface/mouse.cpp
  src/object/object.cpp
  src/object/x_object.cpp
  src/object/x_deformable_object.cpp
  src/object/x_hand.cpp
  src/object/x_right_hand.cpp
  src/object/x_left_hand.cpp
  src/object/simple_object.cpp
  src/object/grid.cpp
  src/object/coordinate.cpp
  src/object/pipe.cpp
)
target_link_libraries(gl_wrapper
  GL
  glut
  GLU
  ${catkin_LIBRARIES}
)

add_executable(gl_wrapper_test test/test.cpp)
target_link_libraries(gl_wrapper_test
  ${catkin_LIBRARIES}
  gl_wrapper
)