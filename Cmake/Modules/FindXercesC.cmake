# Just for correctlly build up libE57Format project, replace original FindXercesC

set ( XercesC_FOUND True )
set ( XercesC_INCLUDE_DIR 
	"${CMAKE_SOURCE_DIR}/ThirdParty/XercesC/src/" 
	"${CMAKE_BINARY_DIR}/ThirdParty/XercesC/src/")
