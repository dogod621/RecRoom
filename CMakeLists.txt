cmake_minimum_required( VERSION 3.2 )

# Create Project
project( RecRoom )

#
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	set ( CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT OFF )
	#set(CMAKE_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME})
	SET(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../install/" CACHE PATH "${CMAKE_CURRENT_SOURCE_DIR}/../install/" FORCE)
endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Cmake/Modules/")

#
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build Shared Library or not" FORCE)


# Use folders (for IDE project grouping)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# ThirdParty
add_subdirectory("ThirdParty")

# RecRoom
add_subdirectory("Core")
add_subdirectory("Test")

