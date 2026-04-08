cmake_minimum_required(VERSION 3.10)
project(apriltag_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -pthread")

find_package(Eigen3 REQUIRED)
find_package(apriltag REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core calib3d)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(vision src/vision.cpp)

target_link_libraries(vision PRIVATE apriltag::apriltag Eigen3::Eigen ${OpenCV_LIBS})

# Build with: cmake -DDEBUG_LOGGING=1 ...
option(DEBUG_LOGGING "Log to stdout instead of file" OFF)
if(DEBUG_LOGGING)
    target_compile_definitions(vision PRIVATE DEBUG_LOGGING)
    message(STATUS "DEBUG_LOGGING: ON (output to stdout)")
else()
    message(STATUS "DEBUG_LOGGING: OFF (output to timestamped log file)")
endif()