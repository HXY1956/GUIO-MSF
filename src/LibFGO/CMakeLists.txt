# === FGO 静态库 ===
set(file_fold
base
sensor
proc
set
factor
)

foreach(fold ${file_fold})
    file(GLOB_RECURSE tmp_header
    ${SRC_DIR}/LibFGO/header/${fold}/*.h)
    source_group("Header Files\\${fold}" FILES ${tmp_header})
    list(APPEND FGO_ALL_HEADER ${tmp_header}) 
    list(APPEND FGO_HEADER_DIR  ${SRC_DIR}/LibFGO/header/${fold})
endforeach()

foreach(fold ${file_fold})
    file(GLOB_RECURSE tmp_source
     ${SRC_DIR}/LibFGO/source/${fold}/*.cpp)
    source_group("Source Files\\${fold}" FILES ${tmp_source})
    list(APPEND FGO_ALL_SRC ${tmp_source})
endforeach()
set(FGO_MAIN ${SRC_DIR}/LibFGO/source/hwa_fgo_main.cpp)
list(REMOVE_ITEM FGO_ALL_SRC ${FGO_MAIN}) 

add_library(LibFGO STATIC ${FGO_ALL_SRC})
target_sources(LibFGO
PRIVATE
${FGO_ALL_HEADER}
)

target_link_libraries(LibFGO
PUBLIC
LibIMU
LibVIS
LibGNSS
LibUWB
LibLIDAR
LibPG
)
target_include_directories(LibFGO
PUBLIC
${FGO_HEADER_DIR}
)

target_compile_definitions(LibFGO PRIVATE DEBUG_NEW)

target_precompile_headers(LibFGO PRIVATE
    ${SRC_DIR}/LibFGO/pch/pch_libfgo.h
)

add_executable(AppFGO ${FGO_MAIN})
target_link_libraries(AppFGO
LibFGO
)


