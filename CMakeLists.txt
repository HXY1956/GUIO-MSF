cmake_minimum_required(VERSION 3.10)
project(GVIO LANGUAGES CXX CUDA)

option(USE_AVX2 "Use AVX2 instruction set" ON)

if(USE_AVX2)
    if(MSVC)
        add_compile_options(
            $<$<COMPILE_LANGUAGE:CXX>:/arch:AVX2>
        )
        add_compile_options(
            $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/arch:AVX2>
        )
        add_compile_definitions(__AVX2__)
        
    elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(
            $<$<COMPILE_LANGUAGE:CXX>:-mavx2 -mfma>
        )
        add_compile_options(
            $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-mavx2,-mfma>
        )
        add_compile_definitions(__AVX2__)
    endif()
endif()

if(MSVC)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:/MP>)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:/bigobj>)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(Third_Eigen_ROOT "D:/TeamCode/third-party/eigen-3.4.0")
set(Third_Ceres_ROOT "D:/TeamCode/third-party/ceres")
set(Third_Zlib_ROOT "D:/TeamCode/third-party/zlib/zlib-1.2.11/vs19")
set(PCL_DIR "D:/TeamCode/third-party/PCL2022/cmake")

find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread filesystem)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets)
find_package(CUDAToolkit REQUIRED)
find_package(OpenVINO REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GSTREAMER_VIDEO REQUIRED gstreamer-video-1.0)

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
add_subdirectory(${SRC_DIR}/LibBASE)
add_subdirectory(${SRC_DIR}/LibIMU)
add_subdirectory(${SRC_DIR}/LibVIS)
add_subdirectory(${SRC_DIR}/LibGNSS)
add_subdirectory(${SRC_DIR}/LibUWB)
add_subdirectory(${SRC_DIR}/LibLIDAR)
add_subdirectory(${SRC_DIR}/LibQT)
add_subdirectory(${SRC_DIR}/LibMSF)
add_subdirectory(${SRC_DIR}/LibPICK)
