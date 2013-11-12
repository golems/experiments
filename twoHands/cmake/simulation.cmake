# *************************************************************************************************
# @file simulation.cmake
# @author Can Erdogan
# @date May 03, 2013
# @brief This is the cmake file to include if you want to use dart and grip.
# *************************************************************************************************

# Macro to check if a set has a particular value. Useful to list the files that need to be linked
# with dart/grip libraries
MACRO(LIST_CONTAINS var value)
  SET(${var})
  FOREACH (value2 ${ARGN})
    IF (${value} STREQUAL ${value2})
      SET(${var} TRUE)
    ENDIF (${value} STREQUAL ${value2})
  ENDFOREACH (value2)
ENDMACRO(LIST_CONTAINS)

# Include DART and GRIP config files
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/dart")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/grip")

find_package(DARTExt REQUIRED)
find_package(DART REQUIRED)
find_package(GRIP REQUIRED)

include_directories(${DARTExt_INCLUDEDIR})
include_directories(${DART_INCLUDEDIR})
include_directories(${GRIP_INCLUDEDIR})
link_directories(${DART_LIBRARY_DIRS})
link_directories(${GRIP_LIBRARY_DIRS})
link_directories(/usr/local/lib)
link_directories(/usr/local/lib/assimp3)

# Include OpenGL and GLUT 
Find_Package(OpenGL REQUIRED)
Find_Package(GLUT REQUIRED)
include_directories( ${GLUT_INCLUDE_DIRS} )

# Set wxWidgets-related parameters
set (wxWidgets_USE_LIBS base core gl)
find_package (wxWidgets)
include (${wxWidgets_USE_FILE})

# Set the include and link directories
include_directories(${DART_INCLUDE_DIRS} ${GRIP_HEADERS} ${WXWIDGETS_SHARED_INCLUDE_DIRS})
link_directories(${CMAKE_SOURCE_DIR}/lib ${GRIP_LIBDIR} ${WXWIDGETS_SHARED_LIBRARY_DIRS} )

# Example of how to link to an executable
#target_link_libraries (<executable name> ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${wxWidgets_LIBRARIES})
