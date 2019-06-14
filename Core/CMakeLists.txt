option(POINT_E57_WITH_RGB "E57 per-point data can contain scanned RGB or not" ON)
option(POINT_E57_WITH_INTENSITY "E57 per-point data can contain scanned intensity or not" ON)
option(POINT_E57_WITH_NORMAL "E57 per-point data can contain scanned normal or not" ON)
option(POINT_E57_WITH_LABEL "E57 per-point data can contain scan index or not" ON)
option(POINT_PCD_WITH_RGB "PCD per-point data can contain RGB or not" ON)
option(POINT_PCD_WITH_INTENSITY "PCD per-point data can contain intensity or not" ON)
option(POINT_PCD_WITH_NORMAL "PCD per-point data can contain normal or not" ON)
option(POINT_PCD_WITH_LABEL "PCD per-point data can contain label or not" ON)

# 
add_subdirectory("Src")