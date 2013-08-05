cmake_minimum_required(VERSION 2.6)
project(ftExample)

# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds are not allowed. You may need to remove CMakeCache.txt.")
endif()

# Link to somatic, amino and ach
# NOTE: Ideally we would like to 'find' these packages but for now, we assume they are either 
# in /usr/lib or /usr/local/lib
link_libraries(somatic amino ach glut GLU)

# Collect the source, script and test files
file(GLOB scripts_source "exe/*.cpp")
LIST(SORT scripts_source)

# Build scripts
message(STATUS "\n-- SCRIPTS: ")
foreach(script_src_file ${scripts_source})
    get_filename_component(script_base ${script_src_file} NAME_WE)
    message(STATUS "Adding script ${script_src_file} with base name ${script_base}" )
    add_executable(${script_base} ${script_src_file})
    add_custom_target(${script_base}.run ${script_base} ${ARGN})
endforeach(script_src_file)
message(STATUS " ")
