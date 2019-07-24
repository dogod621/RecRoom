# Create Project
set(modules "Common" "Container" "Estimator" "Filter" "Interpolator" "Mesher" "Preprocessor" "Reconstructor"  "Sampler" "Scanner" "Segmenter")
set(modules_source)
foreach(module ${modules})
	set(module_dir "${CMAKE_CURRENT_SOURCE_DIR}/${module}/")
	file(GLOB module_srcs ${module_dir}*.c ${module_dir}*.cpp)
	file(GLOB module_hdrs ${module_dir}*.h ${module_dir}*.hpp)
	set(module_source ${module_srcs} ${module_hdrs})
	list(APPEND modules_source ${module_source})
	install(FILES ${module_hdrs} DESTINATION include/Core/${module})
	source_group(${module} FILES ${module_source})
endforeach() 
add_library( Core STATIC ${modules_source} )
set_target_properties(Core PROPERTIES FOLDER "API")

#
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

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
install(TARGETS Core
		RUNTIME DESTINATION bin
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib)