# @file CMakeLists.txt
# @author Can Erdogan
# @date Feb 02, 2013
# @brief The CMakefile for the tests. NOTE Assumes every .cpp file is a test file with a "main" function.

project(UnitTests)

include_directories(.)
include_directories(../include/)

# Compile each test file
file(GLOB tests "*.cpp")
foreach(test ${tests})

	# Get the name (i.e. bla.cpp => bla)
	get_filename_component(base ${test} NAME_WE)
	add_executable(${base} ${test})
	target_link_libraries(${base} pthread gtest)
  add_custom_target(${base}.run ${base} ${ARGN})

	# Add the executable if not to be ignored
	add_test(${base} ${CMAKE_SOURCE_DIR}/build/tests/${base})
	message("-- Adding test: " ${base})
endforeach(test)

