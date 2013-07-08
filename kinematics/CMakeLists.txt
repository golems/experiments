# @author Can Erdogan
# @date Jan 15, 2012

cmake_minimum_required(VERSION 2.6)
project(forwardKinematics)

# ================================================================================================
# Macro to check if a set has a particular value

MACRO(LIST_CONTAINS var value)
  SET(${var})
  FOREACH (value2 ${ARGN})
    IF (${value} STREQUAL ${value2})
      SET(${var} TRUE)
    ENDIF (${value} STREQUAL ${value2})
  ENDFOREACH (value2)
ENDMACRO(LIST_CONTAINS)
# ================================================================================================

# Set the top level directory for use in the code
add_definitions(-DTOP_LEVEL_PATH="${CMAKE_SOURCE_DIR}")

# Set the C99 standard for the C files
set(CMAKE_INSTALL_PREFIX /usr)
set(CMAKE_CXX_FLAGS -g)

# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds are not allowed. You may need to remove CMakeCache.txt.")
endif()

# Include Dart and Grip stuff
include(simulation.cmake)

# Link to somatic, amino and ach
# NOTE: Ideally we would like to 'find' these packages but for now, we assume they are either 
# in /usr/lib or /usr/local/lib
link_libraries(amino protobuf-c ach ntcan ntcanopen filter pcio somatic stdc++)

# Include Eigen
include_directories(/usr/include/eigen3)
include_directories(/usr/local/include/eigen3)

# Include the project files and the common helpers for all the experiments
include_directories(src)
include_directories(../common)

# Collect the source, script and fortran files
file(GLOB main_source "src/*.cpp" "src/*.c")
file(GLOB scripts_source "exe/*.cpp")

# Build the library and set the executables that use it (have to hardcode for now)
add_library(Main SHARED ${main_source})
set(MainDependents forward)

# Build scripts
message(STATUS "\n-- SCRIPTS: ")
foreach(script_src_file ${scripts_source})
	get_filename_component(script_base ${script_src_file} NAME_WE)
	message(STATUS "Adding script ${script_src_file} with base name ${script_base}" )
	add_executable(${script_base} ${script_src_file})
	
	# Check if the target needs to be linked against the main library
	LIST_CONTAINS(contains ${script_base} ${MainDependents})
	if(contains)
		target_link_libraries(${script_base} Main) 
	endif(contains)

	# Link everything to dart/grip
	target_link_libraries(${script_base} ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${wxWidgets_LIBRARIES}) 

	# Add the custom target so you can call "make name.run" and both compile and run the code
	add_custom_target(${script_base}.run ${script_base} ${ARGN})
endforeach(script_src_file)
message(STATUS " ")

# Include the tests
enable_testing()
add_subdirectory(tests)
