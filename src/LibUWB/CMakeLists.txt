# === UWB 静态库 ===
set(file_fold
coder
data
process
set
)

foreach(fold ${file_fold})
    file(GLOB_RECURSE tmp_header
    ${SRC_DIR}/LibUWB/header/${fold}/*.h)
    source_group("Header Files\\${fold}" FILES ${tmp_header})
    list(APPEND UWB_ALL_HEADER ${tmp_header}) 
    list(APPEND UWB_HEADER_DIR  ${SRC_DIR}/LibUWB/header/${fold})
endforeach()

foreach(fold ${file_fold})
    file(GLOB_RECURSE tmp_source
     ${SRC_DIR}/LibUWB/source/${fold}/*.cpp)
    source_group("Source Files\\${fold}" FILES ${tmp_source})
    list(APPEND UWB_ALL_SRC ${tmp_source})
endforeach()

add_library(LibUWB STATIC ${UWB_ALL_SRC})

target_sources(LibUWB
PRIVATE
${UWB_ALL_HEADER}
)

target_link_libraries(LibUWB
PUBLIC  
LibBASE
)

target_include_directories(LibUWB 
PUBLIC
${UWB_HEADER_DIR}
)

