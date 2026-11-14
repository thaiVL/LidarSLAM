cmake_minimum_required(VERSION 3.20)

project(LidarSLAM LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(lidar_slam
    src/main.cpp
    src/data_parser.cpp
)

target_include_directories(lidar_slam PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)

target_compile_definitions(lidar_slam PRIVATE
    PROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}"
)