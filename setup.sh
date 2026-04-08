#!/bin/bash
set -e
 
echo "=== Installing dependencies ==="
sudo apt update
sudo apt install -y cmake build-essential git libeigen3-dev libjpeg62-turbo-dev v4l-utils
 
echo "=== Cloning and building AprilTag ==="
if [ ! -d ~/apriltag ]; then
    git clone https://github.com/AprilRobotics/apriltag.git ~/apriltag
fi
cmake -DCMAKE_BUILD_TYPE=Release -S ~/apriltag -B ~/apriltag/build
cmake --build ~/apriltag/build -j$(nproc)
sudo cmake --install ~/apriltag/build
sudo ldconfig
 
echo "=== Building your vision project ==="
cmake -S ~/projects/tbd -B ~/projects/tbd/build
cmake --build ~/projects/tbd/build -j$(nproc)
 
echo "=== Setup complete ==="
echo "Executable is at ~/projects/tbd/build/vision"