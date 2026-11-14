cmake_minimum_required(VERSION 3.20)
project(LidarSLAM LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)

add_executable(lidar_slam
    src/main.cpp
    src/data_parser.cpp
    src/visualizer.cpp
)

target_include_directories(lidar_slam PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(lidar_slam PRIVATE ${OpenCV_LIBS})

target_compile_definitions(lidar_slam PRIVATE
    PROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}"
)