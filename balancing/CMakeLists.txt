# @author Can Erdogan
# @date Jan 15, 2012

cmake_minimum_required(VERSION 2.6)
project(balancing)

# Set the top level directory for use in the code
add_definitions(-DTOP_LEVEL_PATH="${CMAKE_SOURCE_DIR}")

# Set the C99 standard for the C files
set(CMAKE_INSTALL_PREFIX /usr)
set(CMAKE_C_FLAGS --std=gnu99 -g)

# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds are not allowed. You may need to remove CMakeCache.txt.")
endif()

# Set the mass table path
set(massTablePathVal "/home/cerdogan/Documents/Krang/Documentation/Design/Mass\ Table/MassProp.table" 
	CACHE STRING "Path to the mass table in your system - should be in doc/krang.git/design/mass table")
add_definitions(-DMASS_TABLE_PATH="${massTablePathVal}")


# ================================================================================================
# Include and link to libraries

# Include Dart and Grip stuff
include(simulation.cmake)

# Link to somatic, amino and ach
# NOTE: Ideally we would like to 'find' these packages but for now, we assume they are either 
# in /usr/lib or /usr/local/lib
link_libraries(amino ntcan protobuf-c ach somatic stdc++ filter pcio pthread kore)

# Include Eigen
include_directories(/usr/include/eigen3)
include_directories(/usr/local/include/eigen3)

# ================================================================================================
# Build the codebase library and the executables

# Include the project files
include_directories(src)
include_directories(../common)

# Collect the source, script and fortran files
file(GLOB main_source "src/*.cpp" "src/*.c")
file(GLOB scripts_source "exe/*.cpp")
list(SORT scripts_source)

# Build the library and set the executables that use it (have to hardcode for now)
add_library(Main SHARED ${main_source})
set(MainDependents balancing)

# Build scripts
message(STATUS "\n-- SCRIPTS: ")
foreach(script_src_file ${scripts_source})
	get_filename_component(script_base ${script_src_file} NAME_WE)
	message(STATUS "Adding script ${script_src_file} with base name ${script_base}" )
	add_executable(${script_base} ${script_src_file})
	target_link_libraries(${script_base} Main) 
	target_link_libraries(${script_base} ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${wxWidgets_LIBRARIES}) 
	add_custom_target(${script_base}.run ${script_base} ${ARGN})
endforeach(script_src_file)
message(STATUS " ")

