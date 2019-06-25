
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../Core/")

# ThirdParty
include_directories(${CMAKE_SOURCE_DIR}/ThirdParty/Xerces-C/src/)
include_directories(${CMAKE_SOURCE_DIR}/ThirdParty/E57Format/include/)
include_directories(${CMAKE_SOURCE_DIR}/ThirdParty/Json/include/)
include_directories(${CMAKE_SOURCE_DIR}/ThirdParty/Half/half/)

#
file(GLOB test_srcs ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
foreach(test_src ${test_srcs})
	get_filename_component(Name ${test_src} NAME_WE)
	add_executable(${Name} ${test_src})
	set_target_properties(${Name} PROPERTIES FOLDER "APP/Test")
	
	target_link_libraries(${Name} Core)
	
	# Install
	install(TARGETS ${Name}
			RUNTIME DESTINATION bin
			LIBRARY DESTINATION lib
			ARCHIVE DESTINATION lib)
endforeach() 

