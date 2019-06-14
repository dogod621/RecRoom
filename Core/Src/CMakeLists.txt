# Create Project
file(GLOB srcs ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB hpps ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp)
file(GLOB hdrs ${CMAKE_CURRENT_SOURCE_DIR}/*.h)

add_library( Core ${srcs} ${hpps} ${hdrs})

#
if ( ${POINT_E57_WITH_RGB} )
	add_definitions(-DPOINT_E57_WITH_RGB)
endif()

if ( ${POINT_E57_WITH_INTENSITY} )
	add_definitions(-DPOINT_E57_WITH_INTENSITY)
endif()

if ( ${POINT_E57_WITH_NORMAL} )
	add_definitions(-DPOINT_E57_WITH_NORMAL)
endif()

if ( ${POINT_E57_WITH_LABEL} )
	add_definitions(-DPOINT_E57_WITH_LABEL)
endif()

if ( ${POINT_PCD_WITH_RGB} )
	add_definitions(-DPOINT_PCD_WITH_RGB)
endif()

if ( ${POINT_PCD_WITH_INTENSITY} )
	add_definitions(-DPOINT_PCD_WITH_INTENSITY)
endif()

if ( ${POINT_PCD_WITH_NORMAL} )
	add_definitions(-DPOINT_PCD_WITH_NORMAL)
endif()

if ( ${POINT_PCD_WITH_LABEL} )
	add_definitions(-DPOINT_PCD_WITH_LABEL)
endif()

# ThirdParty
include_directories(${CMAKE_SOURCE_DIR}/ThirdParty/Xerces-C/src/)
target_link_libraries(Core xerces-c)

include_directories(${CMAKE_SOURCE_DIR}/ThirdParty/E57Format/include/)
target_link_libraries(Core E57Format)

include_directories(${CMAKE_SOURCE_DIR}/ThirdParty/Json/include/)
target_link_libraries(Core nlohmann_json::nlohmann_json)

include_directories(${CMAKE_SOURCE_DIR}/ThirdParty/Half/half/)
target_link_libraries(Core half)

# OpenMP
find_package( OpenMP REQUIRED)
if(OPENMP_FOUND)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# PCL
find_package( PCL 1.9 REQUIRED )
if( PCL_FOUND )
	include_directories( ${PCL_INCLUDE_DIRS} )
	add_definitions( ${PCL_DEFINITIONS} )
	link_directories( ${PCL_LIBRARY_DIRS} )
	target_link_libraries( Core ${PCL_LIBRARIES} )
	add_definitions(-DPCL_NO_PRECOMPILE)
endif()

# OpenCV
find_package( OpenCV 4.0 REQUIRED )
if( OpenCV_FOUND )
	include_directories( ${OpenCV_INCLUDE_DIRS} )
	target_link_libraries( Core ${OpenCV_LIBS} )
endif()

# Install
install(FILES ${hpps} ${hdrs} DESTINATION include/Core/)
install(TARGETS Core
		RUNTIME DESTINATION bin
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib)