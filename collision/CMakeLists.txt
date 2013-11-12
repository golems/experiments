cmake_minimum_required(VERSION 2.8)
project(collision-experiments)

# set the right standards for c and c++
set(CMAKE_CXX_FLAGS "-g -std=gnu++0x")

# in-source builds are bad
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds are not allowed. You may need to remove CMakeCache.txt.")
endif()

# Include Dart and Grip stuff
include(simulation.cmake)

# grab some libraries
link_libraries(amino kore stdc++)

# grab eigen
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

# grab some project files
include_directories(../common)

# and build our executable
add_executable(01-penetration-depth 01-penetration-depth.cpp)
target_link_libraries(01-penetration-depth ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${wxWidgets_LIBRARIES}) 
