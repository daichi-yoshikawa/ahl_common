cmake_minimum_required(VERSION 2.8.3)
project(ahl_exception)

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
    roscpp
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
