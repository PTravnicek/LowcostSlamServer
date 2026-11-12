#!/bin/sh
# I2C + GPIO userspace setup for the ROCK Pi X 40-pin header (Debian 13).
#
# The kernel drivers are already part of Debian's kernel and load on their own:
#   i2c_designware (LPSS I2C), cherryview-pinctrl (GPIO), i2c-dev (/dev/i2c-*).
# This script adds the tools, Python libraries, non-root permissions and stable
# device names for the two header I2C buses.
#
# Run on the board:  sudo sh setup_i2c_gpio.sh
#
# Header I2C buses (wiki names = SoC 0-based; ACPI names are 1-based):
#   pins 3/5   I2C2_SDA/SCL  = ACPI \_SB.PCI0.I2C3 = 808622C1:02 -> /dev/i2c-header-3-5
#   pins 27/28 I2C5_SDA/SCL  = ACPI \_SB.PCI0.I2C6 = 808622C1:04 -> /dev/i2c-header-27-28
# Plain /dev/i2c-N numbers are assigned in probe order and can change between
# boots, so use the symlinks.
#
# Scanning: the DesignWare controller can't do SMBus Quick, so use read-byte
# mode, e.g.  /usr/sbin/i2cdetect -y -r $(readlink -f /dev/i2c-header-3-5 | grep -o '[0-9]*$')
set -eu

[ "$(id -u)" -eq 0 ] || { echo "run with sudo" >&2; exit 1; }
USER_NAME=${SUDO_USER:-radxa}

apt-get update
apt-get install -y i2c-tools gpiod python3-libgpiod python3-smbus2

# /dev/i2c-* at boot (Debian already ships this file; make sure it stays).
grep -qx i2c-dev /etc/modules-load.d/i2c-dev.conf 2>/dev/null \
    || echo i2c-dev >> /etc/modules-load.d/i2c-dev.conf
modprobe i2c-dev

# GPIO access without sudo. /dev/i2c-* is already group i2c (60-i2c-tools.rules).
getent group gpio >/dev/null || groupadd --system gpio
usermod -aG gpio,i2c "$USER_NAME"

cat > /etc/udev/rules.d/60-rockpix-header.rules <<'EOF'
# ROCK Pi X 40-pin header (installed by setup_i2c_gpio.sh)
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"
SUBSYSTEM=="i2c-dev", KERNELS=="808622C1:02", SYMLINK+="i2c-header-3-5"
SUBSYSTEM=="i2c-dev", KERNELS=="808622C1:04", SYMLINK+="i2c-header-27-28"
# PM QoS: lets the timing bench keep the CPUs out of C6/C7 (80 us to 10 ms wake-up),
# which otherwise delays GPIO interrupt timestamps by up to ~1 ms.
KERNEL=="cpu_dma_latency", GROUP="gpio", MODE="0660"
EOF
udevadm control --reload
udevadm trigger --action=add --subsystem-match=gpio --subsystem-match=i2c-dev --subsystem-match=misc
udevadm settle

echo
echo "== GPIO chips"; gpiodetect
ls -l /dev/gpiochip*
echo "== Header I2C buses"; ls -l /dev/i2c-header-*
echo "== Header I2C pad mux (function 1 = I2C)"
grep -hE '\((I2C[25]_(SDA|SCL))\)' /sys/kernel/debug/pinctrl/INT33FF:0*/pins 2>/dev/null || echo "(debugfs not available)"
echo "== Python"; python3 -c 'import gpiod, smbus2; print("gpiod", getattr(gpiod, "__version__", "?"), "/ smbus2", getattr(smbus2, "__version__", "?"))'
echo
echo "Done. Log out and back in so '$USER_NAME' picks up the gpio group."
