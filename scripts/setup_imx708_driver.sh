#!/usr/bin/env bash
# Camera Module 3 kernel driver for Ubuntu 22.04 on a Pi 4.
#
# No Ubuntu 22.04 raspi kernel has the Camera Module 3 sensor driver (imx708) or its autofocus driver
# (dw9817, in dw9807-vcm): checked in linux-modules-5.15.0-1108/1110-raspi, which only have imx219 /
# imx477. This builds both from Raspberry Pi's own 5.15 kernel branch with DKMS (so they are rebuilt
# automatically after every Ubuntu kernel update) and installs the imx708 device-tree overlay.
# Compile-tested against linux-headers-5.15.0-1108-raspi.
#
#   sudo bash ~/slam-robot/scripts/setup_imx708_driver.sh      (5-10 min), then: sudo reboot
# Called by setup_pi_camera.sh when the driver is missing; can also be run on its own.
set -euo pipefail
[ "$(id -u)" -eq 0 ] || { echo "run with sudo"; exit 1; }
RPI_COMMIT=14b35093ca68bf2c81bbc90aace5007142b40b40     # raspberrypi/linux rpi-5.15.y
RAW="https://raw.githubusercontent.com/raspberrypi/linux/$RPI_COMMIT"
NAME=pathik-camera
VER=1.0
SRC="/usr/src/$NAME-$VER"
boot=/boot/firmware; [ -d "$boot" ] || boot=/boot
cfg="$boot/config.txt"
KVER="$(uname -r)"

echo "== 1/4 build tools + headers for kernel $KVER"
apt-get update
apt-get install -y --no-install-recommends dkms "linux-headers-$KVER" device-tree-compiler cpp curl \
  || { echo "!! apt failed: is the Pi online?  ping -c2 8.8.8.8 ; ping -c2 ports.ubuntu.com"; exit 1; }

echo "== 2/4 driver sources (Raspberry Pi kernel rpi-5.15.y @ ${RPI_COMMIT:0:12})"
dkms remove "$NAME/$VER" --all >/dev/null 2>&1 || true
rm -rf "$SRC"; mkdir -p "$SRC"
curl -fsSL -o "$SRC/imx708.c" "$RAW/drivers/media/i2c/imx708.c"
curl -fsSL -o "$SRC/dw9807-vcm.c" "$RAW/drivers/media/i2c/dw9807-vcm.c"
echo 'obj-m += imx708.o dw9807-vcm.o' > "$SRC/Makefile"
cat > "$SRC/dkms.conf" <<EOF
PACKAGE_NAME="$NAME"
PACKAGE_VERSION="$VER"
BUILT_MODULE_NAME[0]="imx708"
DEST_MODULE_LOCATION[0]="/updates/dkms"
BUILT_MODULE_NAME[1]="dw9807-vcm"
DEST_MODULE_LOCATION[1]="/updates/dkms"
MAKE[0]="make -C \${kernel_source_dir} M=\${dkms_tree}/\${PACKAGE_NAME}/\${PACKAGE_VERSION}/build modules"
CLEAN="make -C \${kernel_source_dir} M=\${dkms_tree}/\${PACKAGE_NAME}/\${PACKAGE_VERSION}/build clean"
AUTOINSTALL="yes"
EOF

echo "== 3/4 build + install the modules (DKMS)"
dkms add "$NAME/$VER"
dkms build "$NAME/$VER" -k "$KVER" || { echo "!! build failed: send /var/lib/dkms/$NAME/$VER/build/make.log to Claude"; exit 1; }
dkms install "$NAME/$VER" -k "$KVER" --force
depmod -a "$KVER"
modinfo imx708 >/dev/null && modinfo dw9807-vcm >/dev/null && echo "  imx708 + dw9807-vcm installed"

echo "== 4/4 device-tree overlay imx708.dtbo"
OV=$(mktemp -d)
curl -fsSL -o "$OV/imx708-overlay.dts" "$RAW/arch/arm/boot/dts/overlays/imx708-overlay.dts"
curl -fsSL -o "$OV/imx708.dtsi" "$RAW/arch/arm/boot/dts/overlays/imx708.dtsi"
cpp -nostdinc -undef -x assembler-with-cpp -I"$OV" -I"/usr/src/linux-headers-$KVER/include" \
  "$OV/imx708-overlay.dts" > "$OV/imx708.pp.dts"
mkdir -p "$boot/overlays"
dtc -@ -q -I dts -O dtb -o "$boot/overlays/imx708.dtbo" "$OV/imx708.pp.dts"
rm -rf "$OV"
cp -n "$cfg" "$cfg.bak-pathik-imx708"
# Ubuntu's Pi firmware may not know the Camera Module 3: load the overlay ourselves.
sed -i -E 's/^camera_auto_detect=1/camera_auto_detect=0/; /^# pathik: Camera Module 3$/d' "$cfg"
if ! grep -qE '^dtoverlay=imx708' "$cfg"; then
  printf '\n[all]\n# pathik: Camera Module 3 (imx708 driver from setup_imx708_driver.sh)\ncamera_auto_detect=0\ndtoverlay=imx708\n' >> "$cfg"
fi
echo "  $boot/overlays/imx708.dtbo + dtoverlay=imx708 in config.txt"
echo
echo "Done. Reboot:  sudo reboot"
echo "Check after the reboot:  lsmod | grep -E 'imx708|unicam'   and   dmesg | grep -i imx708"
echo "(dmesg should say: camera module ID 0x..., imx708_wide)"
