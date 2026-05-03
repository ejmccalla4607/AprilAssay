cmake_minimum_required(VERSION 3.10)
project(apriltag_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -pthread")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Target platform — default Pi 4 (unicam). Pass -DRPI5=ON to build for Pi 5 (RP1/rp1-cfe).
option(RPI5 "Build for Raspberry Pi 5 (RP1 camera frontend)" OFF)

find_package(apriltag REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core calib3d imgcodecs)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(vision
    src/vision.cpp
    src/capture_v4l2.cpp
)

target_link_libraries(vision PRIVATE
    apriltag::apriltag ${OpenCV_LIBS})

if(RPI5)
    target_compile_definitions(vision PRIVATE RPI5=1)
    message(STATUS "Target platform: Raspberry Pi 5 (rp1-cfe)")
else()
    message(STATUS "Target platform: Raspberry Pi 4 (unicam)")
endif()

