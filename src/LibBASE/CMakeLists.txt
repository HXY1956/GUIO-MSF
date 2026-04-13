# === Base 静态库 ===
set(file_fold
filter
data
global
io
mutex
xml
coder
time
spdlog
set
)

foreach(fold ${file_fold})
    file(GLOB_RECURSE tmp_header
    ${SRC_DIR}/LibBASE/header/${fold}/*.h)
    source_group("Header Files\\${fold}" FILES ${tmp_header})
    list(APPEND BASE_ALL_HEADER ${tmp_header}) 
    list(APPEND BASE_HEADER_DIR  ${SRC_DIR}/LibBASE/header/${fold})
endforeach()

foreach(fold ${file_fold})
    file(GLOB_RECURSE tmp_source
     ${SRC_DIR}/LibBASE/source/${fold}/*.cpp)
    source_group("Source Files\\${fold}" FILES ${tmp_source})
    list(APPEND BASE_ALL_SRC ${tmp_source})
endforeach()

add_library(LibBASE STATIC ${BASE_ALL_SRC})
target_sources(LibBASE
PRIVATE
${BASE_ALL_HEADER}
)

target_include_directories(LibBASE PUBLIC
${BASE_HEADER_DIR}
# ${Third_Eigen_ROOT}
${Third_Zlib_ROOT}
${Third_Ceres_ROOT}/include/
)

target_link_libraries(LibBASE
    PUBLIC
    ${Third_Zlib_ROOT}/zlibd.lib
    ${Third_Zlib_ROOT}/zlibstatic.lib
    ${Third_Zlib_ROOT}/zlibstaticd.lib
    ${Third_Ceres_ROOT}/lib/ceres.lib
    ${Third_Ceres_ROOT}/lib/gflags.lib
    ${Third_Ceres_ROOT}/lib/glog.lib
    ${Third_Ceres_ROOT}/lib/lapack.lib
    ${Third_Ceres_ROOT}/lib/libamd.lib
    ${Third_Ceres_ROOT}/lib/libbtf.lib
    ${Third_Ceres_ROOT}/lib/libcamd.lib
    ${Third_Ceres_ROOT}/lib/libccolamd.lib
    ${Third_Ceres_ROOT}/lib/libcholmod.lib
    ${Third_Ceres_ROOT}/lib/libcolamd.lib
    ${Third_Ceres_ROOT}/lib/libcxsparse.lib
    ${Third_Ceres_ROOT}/lib/libklu.lib
    ${Third_Ceres_ROOT}/lib/libldl.lib
    ${Third_Ceres_ROOT}/lib/libspqr.lib
    ${Third_Ceres_ROOT}/lib/libumfpack.lib
    ${Third_Ceres_ROOT}/lib/openblas.lib
    ${Third_Ceres_ROOT}/lib/suitesparseconfig.lib
    Boost::thread
    Boost::filesystem
)
