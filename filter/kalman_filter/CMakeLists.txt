cmake_minimum_required(VERSION 2.8.3)
project(kalman_filter)

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

#################################################################################
# Find dependencies
#################################################################################

find_package(
  catkin REQUIRED COMPONENTS
    roscpp
    tf2
)

find_package(
  cmake_modules REQUIRED
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${EIGEN3_LIBRARY_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

#################################################################################
# Define package
#################################################################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    kalman_filter
  CATKIN_DEPENDS
    roscpp
    tf2
  DEPENDS
    Eigen3
)

#################################################################################
# Build
#################################################################################

add_library(
  kalman_filter
    src/kalman_filter.cpp
)

target_link_libraries(
  kalman_filter
    ${catkin_LIBRARIES}
)

add_executable(
  kf_test
     test/kf_test.cpp
)

target_link_libraries(
  kf_test
    kalman_filter
)

#################################################################################
# Install
#################################################################################

install(
  TARGETS
    kalman_filter
    kf_test
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)