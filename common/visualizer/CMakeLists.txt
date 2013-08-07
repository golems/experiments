# @author Saul Reynolds-Haertle
# @date 06 Aug 2013

# set up cmake
cmake_minimum_required(VERSION 2.6)
project(krang-vis-tab)

# we use c++0x for this project
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x -g")

# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds are not allowed. You may need to remove CMakeCache.txt.")
endif()

# include kore
include_directories(..)

# Include Dart and Grip stuff and eigen
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(cmake/simulation.cmake)
find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

# Link to libraries that we need
link_libraries(protobuf-c amino ach somatic filter ncurses kore stdc++)

# and build our executable
add_executable(krang-vis-tab krang-vis-tab.cpp krang-vis-tab.hpp)
target_link_libraries(krang-vis-tab ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${wxWidgets_LIBRARIES})