#!/usr/bin/env bash
# Camera Module 3 (Wide) on Ubuntu 22.04: build Raspberry Pi's libcamera with its GStreamer element
# (libcamerasrc), which the robot's vision node reads the camera through.
#   sudo bash ~/slam-robot/scripts/setup_pi_camera.sh       (30-60 min on a Pi 4), then: sudo reboot
# Another libcamera version:  sudo LIBCAMERA_TAG=v0.3.2+rpt20240927 bash setup_pi_camera.sh
set -euo pipefail
[ "$(id -u)" -eq 0 ] || { echo "run with sudo"; exit 1; }
TAG="${LIBCAMERA_TAG:-v0.7.2+rpt20261009}"
SRC=/opt/src/libcamera
boot=/boot/firmware; [ -d "$boot" ] || boot=/boot
cfg="$boot/config.txt"

echo "== 1/5 kernel driver for the Camera Module 3 (imx708)"
if modinfo imx708 >/dev/null 2>&1; then echo "  imx708 driver: present"
else
  echo "  !! kernel $(uname -r) has no imx708 driver. Update the kernel first:"
  echo "     sudo apt update && sudo apt full-upgrade -y && sudo reboot   (then run this script again)"
  exit 1
fi

echo "== 2/5 config.txt: camera auto-detect"
cp -n "$cfg" "$cfg.bak-pathik-camera"
if grep -qE '^camera_auto_detect=1' "$cfg"; then echo "  camera_auto_detect=1 already set"
else
  sed -i -E 's/^camera_auto_detect=0/#camera_auto_detect=0/' "$cfg"
  printf '\n[all]\n# pathik: Camera Module 3\ncamera_auto_detect=1\n' >> "$cfg"
  echo "  camera_auto_detect=1 added (reboot needed)"
fi

echo "== 3/5 build tools and libraries"
systemctl stop robot 2>/dev/null || true          # free CPU + RAM for the compiler
apt-get update
apt-get install -y --no-install-recommends git build-essential ninja-build pkg-config cmake \
  python3-pip python3-yaml python3-ply python3-jinja2 libyaml-dev libgnutls28-dev openssl libssl-dev \
  libdw-dev libunwind-dev libudev-dev libevent-dev libtiff-dev libjpeg-dev libdrm-dev libboost-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good python3-opencv python3-numpy
pip3 install --upgrade 'meson>=1.0.1'             # Ubuntu 22.04's meson is too old for libcamera
MESON=$(command -v meson)
[ -x /usr/local/bin/meson ] && MESON=/usr/local/bin/meson

echo "== 4/5 libcamera $TAG (Raspberry Pi fork)"
rm -rf "$SRC"; mkdir -p "$(dirname "$SRC")"
git clone --depth 1 --branch "$TAG" https://github.com/raspberrypi/libcamera.git "$SRC"
cd "$SRC"
"$MESON" setup build --buildtype=release -Dpipelines=rpi/vc4 -Dipas=rpi/vc4 -Dgstreamer=enabled \
  -Dtest=false -Dlc-compliance=disabled -Dcam=enabled -Dqcam=disabled -Ddocumentation=disabled \
  -Dpycamera=disabled
ninja -C build -j2                                 # -j2: a Pi 4 runs out of RAM with more
ninja -C build install
ldconfig

echo "== 5/5 permissions + check"
[ -n "${SUDO_USER:-}" ] && usermod -aG video "$SUDO_USER"
export GST_PLUGIN_PATH=/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0
if gst-inspect-1.0 libcamerasrc >/dev/null 2>&1; then echo "  libcamerasrc: OK"
else echo "  !! libcamerasrc not found: send the output above to Claude"; exit 1; fi
cat <<MSG

Done. Now:  sudo reboot
After the reboot, check the camera (robot stopped):
  sudo systemctl stop robot
  LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu cam -l                 # lists "imx708_wide"
  GST_PLUGIN_PATH=$GST_PLUGIN_PATH gst-launch-1.0 libcamerasrc num-buffers=30 ! \\
     video/x-raw,width=640,height=360 ! videoconvert ! jpegenc ! multifilesink location=/tmp/cam%02d.jpg
  ls -l /tmp/cam*.jpg                                                      # 30 pictures = camera works
Then: bash ~/slam-robot/scripts/setup_vision.sh   (people detection)
MSG
