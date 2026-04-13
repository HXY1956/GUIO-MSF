# === MSF 静态库 ===
set(file_fold
base
sensor
proc
set
)

foreach(fold ${file_fold})
    file(GLOB_RECURSE tmp_header
    ${SRC_DIR}/LibMSF/header/${fold}/*.h)
    source_group("Header Files\\${fold}" FILES ${tmp_header})
    list(APPEND MSF_ALL_HEADER ${tmp_header}) 
    list(APPEND MSF_HEADER_DIR  ${SRC_DIR}/LibMSF/header/${fold})
endforeach()

foreach(fold ${file_fold})
    file(GLOB_RECURSE tmp_source
     ${SRC_DIR}/LibMSF/source/${fold}/*.cpp)
    source_group("Source Files\\${fold}" FILES ${tmp_source})
    list(APPEND MSF_ALL_SRC ${tmp_source})
endforeach()
set(MSF_MAIN ${SRC_DIR}/LibMSF/source/hwa_msf_main.cpp)
list(REMOVE_ITEM MSF_ALL_SRC ${MSF_MAIN}) 

add_library(LibMSF STATIC ${MSF_ALL_SRC})
target_sources(LibMSF
PRIVATE
${MSF_ALL_HEADER}
)

target_link_libraries(LibMSF
PUBLIC
LibIMU
LibVIS
LibGNSS
LibUWB
LibLIDAR
)
target_include_directories(LibMSF 
PUBLIC
${MSF_HEADER_DIR}
)

add_executable(AppMSF ${MSF_MAIN})
target_link_libraries(AppMSF
LibMSF
)
