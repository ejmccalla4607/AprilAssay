#!/bin/bash
set -e
 
echo "=== Installing dependencies ==="
sudo apt update
sudo apt install -y cmake build-essential git libeigen3-dev libjpeg62-turbo-dev v4l-utils libopencv-dev
 
echo "=== Cloning and building AprilTag ==="
if [ ! -d ~/apriltag ]; then
    git clone https://github.com/AprilRobotics/apriltag.git ~/apriltag
fi
cmake -DCMAKE_BUILD_TYPE=Release -S ~/apriltag -B ~/apriltag/build
cmake --build ~/apriltag/build -j$(nproc)
sudo cmake --install ~/apriltag/build
sudo ldconfig
 
echo "=== Building your vision project ==="
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PI_FLAGS=""
if grep -q "Raspberry Pi 5" /proc/cpuinfo 2>/dev/null || grep -q "bcm2712" /proc/cpuinfo 2>/dev/null; then
    PI_FLAGS="-DRPI5=ON"
    echo "Detected Pi 5 — building with RPI5=ON"
fi
cmake $PI_FLAGS -S "$SCRIPT_DIR" -B "$SCRIPT_DIR/build"
cmake --build "$SCRIPT_DIR/build" -j$(nproc)
 
echo "=== Installing PREEMPT_RT kernel ==="
# Use the RPi Foundation's RT kernel (linux-image-rpi-v8-rt), not the Debian
# upstream (linux-image-rt-arm64) which lacks the rp1-cfe/unicam camera drivers.
if uname -r | grep -q "\-rt"; then
    echo "RT kernel already running ($(uname -r)) — skipping install"
else
    if ! dpkg -l linux-image-rpi-v8-rt 2>/dev/null | grep -q "^ii"; then
        sudo apt install -y linux-image-rpi-v8-rt
        echo "RT kernel installed"
    else
        echo "RT kernel package already installed"
    fi
    # On Pi 5 the firmware boots kernel_2712.img by default; override to the RT image.
    if grep -q "Raspberry Pi 5" /proc/cpuinfo 2>/dev/null || grep -q "bcm2712" /proc/cpuinfo 2>/dev/null; then
        CONFIG=/boot/firmware/config.txt
        if grep -q "kernel=kernel8_rt.img" "$CONFIG"; then
            echo "Pi 5 RT kernel boot entry already set — skipping"
        else
            sudo sed -i 's/\[cm4\]/[pi5]\nkernel=kernel8_rt.img\n\n[cm4]/' "$CONFIG"
            echo "Pi 5 config.txt updated to boot kernel8_rt.img"
        fi
    fi
    echo "*** Reboot required to activate RT kernel ***"
fi

echo "=== Disabling PipeWire (conflicts with direct V4L2 camera access) ==="
for svc in pipewire wireplumber pipewire-pulse; do
    if systemctl is-enabled "$svc" 2>/dev/null | grep -qv "not-found"; then
        sudo systemctl disable --now "$svc" 2>/dev/null && echo "Disabled $svc" || true
    fi
done

echo "=== Configuring kernel CPU isolation for low-latency detection ==="
CMDLINE=/boot/firmware/cmdline.txt
if grep -q "isolcpus=1,2,3" "$CMDLINE"; then
    echo "isolcpus already set — skipping"
elif grep -q "isolcpus=2,3" "$CMDLINE"; then
    sudo sed -i 's/isolcpus=2,3 nohz_full=2,3/isolcpus=1,2,3 nohz_full=1,2,3/' "$CMDLINE"
    echo "Updated isolcpus=2,3 → isolcpus=1,2,3 in $CMDLINE"
    echo "*** Reboot required for CPU isolation to take effect ***"
else
    sudo sed -i 's/$/ isolcpus=1,2,3 nohz_full=1,2,3/' "$CMDLINE"
    echo "Added isolcpus=1,2,3 nohz_full=1,2,3 to $CMDLINE"
    echo "*** Reboot required for CPU isolation to take effect ***"
fi

echo "=== Configuring CPU performance governor ==="
if [ -f /etc/rc.local ] && grep -q "scaling_governor" /etc/rc.local; then
    echo "performance governor already configured in rc.local — skipping"
else
    sudo tee /etc/rc.local > /dev/null << 'RCEOF'
#!/bin/bash
for f in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    echo performance > "$f"
done
exit 0
RCEOF
    sudo chmod +x /etc/rc.local
    sudo systemctl start rc-local
    echo "CPU governor set to performance (persistent via rc.local)"
fi

echo "=== Setup complete ==="
echo "Executable is at $SCRIPT_DIR/build/vision"
if ! uname -r | grep -q "\-rt"; then
    echo "*** Reboot required for RT kernel and/or CPU isolation to take effect ***"
fi