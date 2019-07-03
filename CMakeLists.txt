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
option(POINT_RAW_WITH_RGB "Raw data per-point data can contain scanned RGB or not" ON)
option(POINT_RAW_WITH_INTENSITY "Raw data per-point data can contain scanned intensity or not" ON)
option(POINT_RAW_WITH_NORMAL "Raw data per-point data can contain scanned normal or not" OFF)
option(POINT_RAW_WITH_LABEL "Raw data per-point data can contain scan serial number or not" ON)
option(POINT_REC_WITH_RGB "Reconstruct data per-point data can contain RGB or not" ON)
option(POINT_REC_WITH_INTENSITY "Reconstruct data per-point data can contain intensity or not" ON)
option(POINT_REC_WITH_NORMAL "Reconstruct data per-point data can contain normal or not" ON)
option(POINT_REC_WITH_LABEL "Reconstruct data per-point data can contain segment label or not" ON)

#
if ( ${POINT_RAW_WITH_RGB} )
	add_definitions(-DPOINT_RAW_WITH_RGB)
endif()

if ( ${POINT_RAW_WITH_INTENSITY} )
	add_definitions(-DPOINT_RAW_WITH_INTENSITY)
endif()

if ( ${POINT_RAW_WITH_NORMAL} )
	add_definitions(-DPOINT_RAW_WITH_NORMAL)
endif()

if ( ${POINT_RAW_WITH_LABEL} )
	add_definitions(-DPOINT_RAW_WITH_LABEL)
endif()

if ( ${POINT_REC_WITH_RGB} )
	add_definitions(-DPOINT_REC_WITH_RGB)
endif()

if ( ${POINT_REC_WITH_INTENSITY} )
	add_definitions(-DPOINT_REC_WITH_INTENSITY)
endif()

if ( ${POINT_REC_WITH_NORMAL} )
	add_definitions(-DPOINT_REC_WITH_NORMAL)
endif()

if ( ${POINT_REC_WITH_LABEL} )
	add_definitions(-DPOINT_REC_WITH_LABEL)
endif()

# Use folders (for IDE project grouping)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# ThirdParty
add_subdirectory("ThirdParty")

# RecRoom
add_subdirectory("Core")
add_subdirectory("Test")

