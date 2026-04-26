cmake_minimum_required(VERSION 3.10)
project(apriltag_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -pthread")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(apriltag REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core calib3d imgcodecs)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(vision
    src/vision.cpp
    src/capture_v4l2.cpp
)

target_link_libraries(vision PRIVATE
    apriltag::apriltag ${OpenCV_LIBS})

