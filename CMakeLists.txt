cmake_minimum_required(VERSION 3.10)
project(MY_PROJECT LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

find_library(CNPY_LIB NAMES cnpy PATHS /usr/local/lib)

include_directories(/usr/include/eigen3)
include_directories(/usr/local/include)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/gitman_sources/CTopPRM/include
    ${CMAKE_CURRENT_SOURCE_DIR}/gitman_sources/PMM/include
#    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ================= CTopPRM ====================
add_library(CTopPRM STATIC
    gitman_sources/CTopPRM/src/common.cpp
    gitman_sources/CTopPRM/src/prm.cpp
    gitman_sources/CTopPRM/src/topological_prm.cpp
    gitman_sources/CTopPRM/src/topological_prm_clustering.cpp
    gitman_sources/CTopPRM/src/base_map.cpp
    gitman_sources/CTopPRM/src/esdf_map.cpp
)

# ================= PMM ====================
add_library(PMM STATIC
    gitman_sources/PMM/src/common.cpp
    gitman_sources/PMM/src/pmm_mg_trajectory3d.cpp
    gitman_sources/PMM/src/pmm_trajectory3d.cpp
    gitman_sources/PMM/src/pmm_trajectory.cpp
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

add_executable(main src/main.cpp)

target_link_libraries(main
    CTopPRM
    PMM
    yaml-cpp
    pthread
    lz4
    log4cxx
    ${CNPY_LIB}
)
