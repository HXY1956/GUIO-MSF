# === POSEGRAPH static library (loop closure / pose graph / relocalization) ===
set(file_fold
header
source
)

foreach(fold ${file_fold})
    file(GLOB_RECURSE tmp_header
    ${SRC_DIR}/LibPG/header/*.h)
    source_group("Header Files" FILES ${tmp_header})
    list(APPEND PG_ALL_HEADER ${tmp_header})
    list(APPEND PG_HEADER_DIR ${SRC_DIR}/LibPG/header)
endforeach()

file(GLOB_RECURSE PG_ALL_SRC ${SRC_DIR}/LibPG/source/*.cpp)

add_library(LibPG STATIC ${PG_ALL_SRC})
target_sources(LibPG
PRIVATE
${PG_ALL_HEADER}
)

# === DBoW2 静态库 ===
file(GLOB_RECURSE DBOW2_ALL_SRC
    ${SRC_DIR}/LibPG/DBoW2/*.cpp)
file(GLOB_RECURSE DBOW2_ALL_HEADER
    ${SRC_DIR}/LibPG/DBoW2/*.h
    ${SRC_DIR}/LibPG/DBoW2/*.hpp)
source_group("Header Files\\DBoW2" FILES ${DBOW2_ALL_HEADER})
source_group("Source Files\\DBoW2" FILES ${DBOW2_ALL_SRC})

add_library(DBoW2 STATIC ${DBOW2_ALL_SRC})
target_sources(DBoW2
    PRIVATE
    ${DBOW2_ALL_HEADER}
)

target_compile_options(DBoW2 PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:/bigobj /EHsc /Zi /Ob0 /utf-8>
)

target_include_directories(DBoW2
    PUBLIC
    ${SRC_DIR}/LibPG/DBoW2
    ${SRC_DIR}/LibPG/DBoW2/DBoW
    ${SRC_DIR}/LibPG/DBoW2/DUtils
    ${SRC_DIR}/LibPG/DBoW2/DVision
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(DBoW2
    PUBLIC
    ${OpenCV_LIBS}
)

target_link_libraries(LibPG
PUBLIC
LibBASE
DBoW2
${OpenCV_LIBS}
)

target_include_directories(LibPG
PUBLIC
${PG_HEADER_DIR}
)

if(MSVC)
    target_compile_options(LibPG PRIVATE /bigobj)
endif()

target_precompile_headers(LibPG PRIVATE
    ${SRC_DIR}/LibPG/pch/pch_libpg.h
)
