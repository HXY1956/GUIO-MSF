# === IMU 静态库 ===
set(file_fold
coder
data
process
util
set
)

foreach(fold ${file_fold})
    file(GLOB_RECURSE tmp_header
    ${SRC_DIR}/LibIMU/header/${fold}/*.h)
    source_group("Header Files\\${fold}" FILES ${tmp_header})
    list(APPEND IMU_ALL_HEADER ${tmp_header}) 
    list(APPEND IMU_HEADER_DIR  ${SRC_DIR}/LibIMU/header/${fold})
endforeach()

foreach(fold ${file_fold})
    file(GLOB_RECURSE tmp_source
     ${SRC_DIR}/LibIMU/source/${fold}/*.cpp)
    source_group("Source Files\\${fold}" FILES ${tmp_source})
    list(APPEND IMU_ALL_SRC ${tmp_source})
endforeach()

add_library(LibIMU STATIC ${IMU_ALL_SRC})
target_sources(LibIMU
PRIVATE
${IMU_ALL_HEADER}
)
target_link_libraries(LibIMU
PUBLIC 
LibBASE
glfw
opengl32
)
target_include_directories(LibIMU 
PUBLIC
${IMU_HEADER_DIR}
)
target_compile_definitions(LibIMU PUBLIC
WIN32_LEAN_AND_MEAN   
NOMINMAX
)