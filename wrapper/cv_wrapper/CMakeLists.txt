cmake_minimum_required(VERSION 2.8.3)
project(cv_wrapper)

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

##############################################################################
# Find dependencies
##############################################################################

find_package(
  catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    message_filters
    roscpp
    std_srvs
)

find_package(
  OpenCV REQUIRED
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

##############################################################################
# Define package
##############################################################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    cv_wrapper
  CATKIN_DEPENDS
    cv_bridge
    image_transport
    message_filters
    roscpp
    std_srvs
)

##############################################################################
# Build
##############################################################################

add_library(
  cv_wrapper
    src/image_subscriber.cpp
    src/utils.cpp
    src/rect.cpp
    src/hsv_mask.cpp
)

target_link_libraries(
  cv_wrapper
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

#add_executable(
#  cv_wrapper_test
#    test/test.cpp
#)

#target_link_libraries(
#  cv_wrapper_test
#    ${catkin_LIBRARIES}
#    cv_wrapper
#)

##############################################################################
# Install
##############################################################################

install(
  TARGETS
    cv_wrapper
#    cv_wrapper_test
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)