# === Lidar 静态库 ===
set(file_fold
coder
data
proc
set
util
)

foreach(fold ${file_fold})
    file(GLOB_RECURSE tmp_header
    ${SRC_DIR}/LibLIDAR/header/${fold}/*.h)
    source_group("Header Files\\${fold}" FILES ${tmp_header})
    list(APPEND LIDAR_ALL_HEADER ${tmp_header}) 
    list(APPEND LIDAR_HEADER_DIR  ${SRC_DIR}/LibLIDAR/header/${fold})
endforeach()

foreach(fold ${file_fold})
    file(GLOB_RECURSE tmp_source
     ${SRC_DIR}/LibLIDAR/source/${fold}/*.cpp)
    source_group("Source Files\\${fold}" FILES ${tmp_source})
    list(APPEND LIDAR_ALL_SRC ${tmp_source})
endforeach()

add_library(LibLIDAR STATIC ${LIDAR_ALL_SRC})

target_sources(LibLIDAR
    PRIVATE
    ${LIDAR_ALL_HEADER} 
)

target_include_directories(LibLIDAR
  PUBLIC
  ${LIDAR_HEADER_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(LibLIDAR
    PUBLIC
    LibBASE
    glfw 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)