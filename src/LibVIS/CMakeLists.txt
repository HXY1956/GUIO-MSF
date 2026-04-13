# === VIS 静态库 ===
set(file_fold
base
coder
data
process
yolo
tracker
set
)

foreach(fold ${file_fold})
    file(GLOB_RECURSE tmp_header
    ${SRC_DIR}/LibVIS/header/${fold}/*.h)
    source_group("Header Files\\${fold}" FILES ${tmp_header})
    list(APPEND VIS_ALL_HEADER ${tmp_header}) 
    list(APPEND VIS_HEADER_DIR  ${SRC_DIR}/LibVIS/header/${fold})
endforeach()

foreach(fold ${file_fold})
    file(GLOB_RECURSE tmp_source
     ${SRC_DIR}/LibVIS/source/${fold}/*.cpp
     ${SRC_DIR}/LibVIS/source/${fold}/*.cu)
    source_group("Source Files\\${fold}" FILES ${tmp_source})
    list(APPEND VIS_ALL_SRC ${tmp_source})
endforeach()

add_library(LibVIS STATIC ${VIS_ALL_SRC})
target_compile_options(LibVIS PRIVATE
      $<$<COMPILE_LANGUAGE:CXX>:/bigobj /EHsc /Zi /Ob0 /utf-8>
)

target_sources(LibVIS
    PRIVATE
    ${VIS_ALL_HEADER} 
)

target_compile_definitions(LibVIS PRIVATE 
EIGEN_HAS_CUDA_FP16=0
NOMINMAX)

target_include_directories(LibVIS
  PUBLIC
    ${SRC_DIR}/LibVIS/HikVision_Includes
    ${SRC_DIR}/LibVIS/Daheng_Includes
    ${VIS_HEADER_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${GSTREAMER_INCLUDE_DIRS}
    ${GSTREAMER_APP_INCLUDE_DIRS}
    ${GSTREAMER_VIDEO_INCLUDE_DIRS}
)


target_link_directories(
    LibVIS
    PUBLIC
    ${GSTREAMER_LIBRARY_DIRS}
    ${GSTREAMER_APP_LIBRARY_DIRS}
    ${GSTREAMER_VIDEO_LIBRARY_DIRS}
)


target_link_libraries(LibVIS
    PUBLIC
    LibBASE
    CUDA::cudart
    ${SRC_DIR}/LibVIS/HikVision_Includes/MvCameraControl.lib
    ${SRC_DIR}/LibVIS/Daheng_Includes/GxIAPICPPEx.lib
    ${OpenCV_LIBS}
    ${GSTREAMER_LIBRARIES}
    ${GSTREAMER_APP_LIBRARIES}
    ${GSTREAMER_VIDEO_LIBRARIES}
    Qt6::Widgets 
    Qt6::Core 
    Qt6::Gui
    openvino::runtime
)
set_target_properties(LibVIS PROPERTIES
   CUDA_SEPARABLE_COMPILATION ON
   CUDA_RESOLVE_DEVICE_SYMBOLS ON
   CUDA_ARCHITECTURES "89"
LINKER_LANGUAGE CUDA
   AUTOMOC ON         
   AUTORCC ON
   AUTOUIC ON
)