#
if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/XercesC")
  message(FATAL_ERROR "The Xerces-C submodule directory is missing! "
    "You probably did not clone the project with --recursive. It is possible to recover "
    "by running \"git submodule update --init --recursive\"")
endif()
if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/E57Format")
  message(FATAL_ERROR "The libE57Format submodule directory is missing! "
    "You probably did not clone the project with --recursive. It is possible to recover "
    "by running \"git submodule update --init --recursive\"")
endif()
if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/Json")
  message(FATAL_ERROR "The Json submodule directory is missing! "
    "You probably did not clone the project with --recursive. It is possible to recover "
    "by running \"git submodule update --init --recursive\"")
endif()
if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/Half")
  message(FATAL_ERROR "The Half submodule directory is missing! "
    "You probably did not clone the project with --recursive. It is possible to recover "
    "by running \"git submodule update --init --recursive\"")
endif()

# XercesC
add_subdirectory (XercesC EXCLUDE_FROM_ALL)

# XercesC - Documentation
set(XercesC_Documentation_TARGETS)
list(APPEND XercesC_Documentation_TARGETS doc-xml doc-style)
if(TARGET createdocs)
	list(APPEND XercesC_Documentation_TARGETS createdocs)
endif()
if(TARGET createapidocs)
	list(APPEND XercesC_Documentation_TARGETS createapidocs)
endif()
set_target_properties(${XercesC_Documentation_TARGETS} PROPERTIES FOLDER "ThirdParty/XercesC/Documentation")

# XercesC - Library
set(XercesC_Library_TARGETS)
list(APPEND XercesC_Library_TARGETS xerces-c-headers xerces-c-missing xerces-c)
if(TARGET genicu)
	list(APPEND XercesC_Library_TARGETS genicu)
endif()
if(TARGET gencat)
	list(APPEND XercesC_Library_TARGETS gencat)
endif()
set_target_properties(${XercesC_Library_TARGETS} PROPERTIES FOLDER "ThirdParty/XercesC/Library")

# XercesC - Tests
set(XercesC_Tests_TARGETS)
list(APPEND XercesC_Tests_TARGETS 
	DOMTest
	DOMMemTest
	Normalizer
	RangeTest
	DOMTraversalTest
	DOMTypeInfoTest
	EncodingTest
	InitTermTest
	MemHandlerTest
	NetAccessorTest
	XSerializerTest
	XSTSHarness
	XSValueTest)
if(TARGET ThreadTest)
	list(APPEND XercesC_Tests_TARGETS ThreadTest)
endif()
if(TARGET Char16Test)
	list(APPEND XercesC_Tests_TARGETS Char16Test)
endif()
set_target_properties(${XercesC_Tests_TARGETS} PROPERTIES FOLDER "ThirdParty/XercesC/Tests")

# XercesC - Samples
set(XercesC_Samples_TARGETS)
list(APPEND XercesC_Samples_TARGETS 
	CreateDOMDocument
	DOMCount
	DOMPrint
	EnumVal
	MemParse
	PParse
	PSVIWriter
	Redirect
	SAX2Count
	SAX2Print
	SAXCount
	SAXPrint
	SCMPrint
	SEnumVal
	StdInParse
	XInclude)
set_target_properties(${XercesC_Samples_TARGETS} PROPERTIES FOLDER "ThirdParty/XercesC/Samples")

# E57
if ( WIN32 )
	set(USING_STATIC_XERCES $<NOT:${BUILD_SHARED_LIBS}> CACHE BOOL " " FORCE)
endif()
add_subdirectory (E57Format EXCLUDE_FROM_ALL)
set(E57Format_TARGETS)
list(APPEND E57Format_TARGETS E57Format)
set_target_properties(${E57Format_TARGETS} PROPERTIES FOLDER "ThirdParty/E57Format")

# Json
set(JSON_BuildTests OFF CACHE BOOL " " FORCE)
add_subdirectory (Json EXCLUDE_FROM_ALL)
set(Json_TARGETS)
set_target_properties(${Json_TARGETS} PROPERTIES FOLDER "ThirdParty/Json")

# Half
macro(define_module name)
endmacro()
macro(buildsys_library target)
endmacro()
add_subdirectory (Half/half EXCLUDE_FROM_ALL)
set(Half_TARGETS)
list(APPEND Half_TARGETS half)
set_target_properties(${Half_TARGETS} PROPERTIES FOLDER "ThirdParty/Half")
