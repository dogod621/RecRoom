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

option(INPUT_PERPOINT_RGB "Raw data per-point data can contain scanned RGB or not" ON)
option(INPUT_PERPOINT_INTENSITY "Raw data per-point data can contain scanned intensity or not" ON)
option(INPUT_PERPOINT_NORMAL "Raw data per-point data can contain scanned normal or not" OFF)
option(INPUT_PERPOINT_SERIAL_NUMBER "Raw data per-point data can contain scan serial number or not" ON)

option(OUTPUT_PERPOINT_RGB "Reconstructed data per-point data can contain RGB or not" ON)
option(OUTPUT_PERPOINT_INTENSITY "Reconstructed data per-point data can contain intensity or not" ON)
option(OUTPUT_PERPOINT_NORMAL "Reconstructed data per-point data can contain normal or not" ON)
option(OUTPUT_PERPOINT_SHARPNESS "Reconstructed data per-point data can contain sharpness or not" ON)
option(OUTPUT_PERPOINT_LABEL "Reconstructed data per-point data can contain segment label or not" ON)

# RGB
if ( ${INPUT_PERPOINT_RGB} )
	add_definitions(-DINPUT_PERPOINT_RGB)
	add_definitions(-DWITH_INPUT_PERPOINT_RGB=true)
else()
	add_definitions(-DWITH_INPUT_PERPOINT_RGB=false)
endif()

if ( ${OUTPUT_PERPOINT_RGB} )
	add_definitions(-DOUTPUT_PERPOINT_RGB)
	add_definitions(-DWITH_OUTPUT_PERPOINT_RGB=true)
else()
	add_definitions(-DWITH_OUTPUT_PERPOINT_RGB=false)
endif()

if ( ${INPUT_PERPOINT_RGB} OR ${OUTPUT_PERPOINT_RGB})
	add_definitions(-DPERPOINT_RGB)
	add_definitions(-DWITH_PERPOINT_RGB=true)
else()
	add_definitions(-DWITH_PERPOINT_RGB=false)
endif()

# INTENSITY
if ( ${INPUT_PERPOINT_INTENSITY} )
	add_definitions(-DINPUT_PERPOINT_INTENSITY)
	add_definitions(-DWITH_INPUT_PERPOINT_INTENSITY=true)
else()
	add_definitions(-DWITH_INPUT_PERPOINT_INTENSITY=false)
endif()

if ( ${OUTPUT_PERPOINT_INTENSITY} )
	add_definitions(-DOUTPUT_PERPOINT_INTENSITY)
	add_definitions(-DWITH_OUTPUT_PERPOINT_INTENSITY=true)
else()
	add_definitions(-DWITH_OUTPUT_PERPOINT_INTENSITY=false)
endif()

if ( ${INPUT_PERPOINT_INTENSITY} OR ${OUTPUT_PERPOINT_INTENSITY})
	add_definitions(-DPERPOINT_INTENSITY)
	add_definitions(-DWITH_PERPOINT_INTENSITY=true)
else()
	add_definitions(-DWITH_PERPOINT_INTENSITY=false)
endif()

# NORMAL
if ( ${INPUT_PERPOINT_NORMAL} )
	add_definitions(-DINPUT_PERPOINT_NORMAL)
	add_definitions(-DWITH_INPUT_PERPOINT_NORMAL=true)
else()
	add_definitions(-DWITH_INPUT_PERPOINT_NORMAL=false)
endif()

if ( ${OUTPUT_PERPOINT_NORMAL} )
	add_definitions(-DOUTPUT_PERPOINT_NORMAL)
	add_definitions(-DWITH_OUTPUT_PERPOINT_NORMAL=true)
else()
	add_definitions(-DWITH_OUTPUT_PERPOINT_NORMAL=false)
endif()

if ( ${INPUT_PERPOINT_NORMAL} OR ${OUTPUT_PERPOINT_NORMAL})
	add_definitions(-DPERPOINT_NORMAL)
	add_definitions(-DWITH_PERPOINT_NORMAL=true)
else()
	add_definitions(-DWITH_PERPOINT_NORMAL=false)
endif()

# SHARPNESS
if ( ${OUTPUT_PERPOINT_SHARPNESS} )
	add_definitions(-DOUTPUT_PERPOINT_SHARPNESS)
	add_definitions(-DWITH_OUTPUT_PERPOINT_SHARPNESS=true)
	
	add_definitions(-DPERPOINT_SHARPNESS)
	add_definitions(-DWITH_PERPOINT_SHARPNESS=true)
else()
	add_definitions(-DWITH_OUTPUT_PERPOINT_NDF=false)
	
	add_definitions(-DWITH_PERPOINT_SHARPNESS=false)
endif()

# SERIAL_NUMBER
if ( ${INPUT_PERPOINT_SERIAL_NUMBER} )
	add_definitions(-DINPUT_PERPOINT_SERIAL_NUMBER)
	add_definitions(-DWITH_INPUT_PERPOINT_SERIAL_NUMBER=true)
	
	add_definitions(-DPERPOINT_SERIAL_NUMBER)
	add_definitions(-DWITH_PERPOINT_SERIAL_NUMBER=true)
else()
	add_definitions(-DWITH_INPUT_PERPOINT_SERIAL_NUMBER=false)
	
	add_definitions(-DWITH_PERPOINT_SERIAL_NUMBER=false)
endif()

# LABEL
if ( ${OUTPUT_PERPOINT_LABEL} )
	add_definitions(-DOUTPUT_PERPOINT_LABEL)
	add_definitions(-DWITH_OUTPUT_PERPOINT_LABEL=true)
	
	add_definitions(-DPERPOINT_LABEL)
	add_definitions(-DWITH_PERPOINT_LABEL=true)
else()
	add_definitions(-DWITH_OUTPUT_PERPOINT_LABEL=false)
	
	add_definitions(-DWITH_PERPOINT_LABEL=false)
endif()


# Use folders (for IDE project grouping)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# ThirdParty
add_subdirectory("ThirdParty")

# RecRoom
add_subdirectory("Core")
add_subdirectory("Test")

