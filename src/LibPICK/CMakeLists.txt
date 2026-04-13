# === PICK 静态库 ===
file(GLOB_RECURSE PICK_SRC ${SRC_DIR}/LibPICK/source/*.cpp)
set(PICK_MAIN ${SRC_DIR}/LibPICK/source/hwa_pick_main.cpp)
list(REMOVE_ITEM PICK_SRC ${PICK_MAIN}) 
file(GLOB_RECURSE PICK_HDR ${SRC_DIR}/LibPICK/header/*.h)
source_group("Header Files" FILES ${PICK_HDR})
source_group("Source Files" FILES ${PICK_SRC})

add_library(LibPICK STATIC ${PICK_SRC})
target_sources(LibPICK
PRIVATE
${PICK_HDR}
)
target_link_libraries(LibPICK
PUBLIC
LibVIS
Qt6::Widgets 
Qt6::Core 
Qt6::Gui
)
target_include_directories(LibPICK PUBLIC
${SRC_DIR}/LibPICK/header
)
set_target_properties(LibPICK PROPERTIES
AUTOMOC ON         
AUTORCC ON
AUTOUIC ON
)
target_compile_definitions(LibPICK PRIVATE
WIN32_LEAN_AND_MEAN   
NOMINMAX
)

add_executable(AppPICK ${PICK_MAIN})
target_link_libraries(AppPICK
LibPICK
)