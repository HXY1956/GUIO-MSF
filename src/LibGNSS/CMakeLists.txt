# === GNSS 静态库 ===
set(file_fold
all
ambfix
coders
data
models
proc
utils
prod
set
param
)

foreach(fold ${file_fold})
    file(GLOB_RECURSE tmp_header
    ${SRC_DIR}/LibGNSS/header/${fold}/*.h)
    source_group("Header Files\\${fold}" FILES ${tmp_header})
    list(APPEND GNSS_ALL_HEADER ${tmp_header}) 
    list(APPEND GNSS_HEADER_DIR ${SRC_DIR}/LibGNSS/header/${fold})
endforeach()

foreach(fold ${file_fold})
    file(GLOB_RECURSE tmp_source
    ${SRC_DIR}/LibGNSS/source/${fold}/*.cpp)
    source_group("Source Files\\${fold}" FILES ${tmp_source})
    list(APPEND GNSS_ALL_SRC ${tmp_source})
endforeach()

add_library(LibGNSS STATIC ${GNSS_ALL_SRC})
target_sources(LibGNSS
PRIVATE
${GNSS_ALL_HEADER}
)
target_link_libraries(LibGNSS
PUBLIC  
LibBASE
)
target_include_directories(LibGNSS 
PUBLIC
${GNSS_HEADER_DIR}
)