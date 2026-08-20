#!/bin/bash

# ============================================================
# BHY2CLI Linux Environment Setup Script
# Version: 1.0.0
# ============================================================

ARM_TOOLCHAIN_VERSION="15.2.rel1"
ARM_TOOLCHAIN_NAME="arm-gnu-toolchain-${ARM_TOOLCHAIN_VERSION}-x86_64-arm-none-eabi"
ARM_TOOLCHAIN_URL="https://developer.arm.com/-/media/Files/downloads/gnu/${ARM_TOOLCHAIN_VERSION}/binrel/${ARM_TOOLCHAIN_NAME}.tar.xz"

# ============================================================
# Output helpers
# ============================================================
TOOLS_DIR="$HOME/tools"
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

print_step() { echo -e "\n${GREEN}>>> STEP: $1${NC}"; }
print_ok()   { echo -e "${GREEN}[OK]${NC} $1"; }
print_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
print_err()  { echo -e "${RED}[ERROR]${NC} $1"; }

# ============================================================
# Distro detection (pattern from coines_inst.sh)
# ============================================================

if [ "$(which dpkg 2>/dev/null)" != "" ]; then
    DISTRO="DEBIAN"
    PKG_UPDATE="sudo apt-get update"
    PKG_INSTALL="sudo apt-get -y install"
elif [ "$(which rpm 2>/dev/null)" != "" ]; then
    DISTRO="REDHAT"
    PKG_UPDATE="sudo yum check-update; true"   # yum check-update exits 100 when updates available; 'true' prevents script abort
    PKG_INSTALL="sudo yum -y install"
else
    DISTRO="UNKNOWN"
fi

echo "Detected distro type: $DISTRO"

check_if_installed() {
    if [ "$DISTRO" = "DEBIAN" ]; then
        dpkg -s "$1" 2>/dev/null >/dev/null
    elif [ "$DISTRO" = "REDHAT" ]; then
        rpm -q "$1" >/dev/null 2>&1
    else
        which "$1" >/dev/null 2>&1
    fi
    return $?
}

install_package() {
    local pkg=$1
    if check_if_installed "$pkg"; then
        print_ok "$pkg already installed"
    else
        echo "  Installing $pkg..."
        if [ "$DISTRO" = "UNKNOWN" ]; then
            print_warn "Unknown distro - please install '$pkg' manually"
        else
            $PKG_INSTALL "$pkg"
        fi
    fi
}

# ============================================================
# Step 1: Install dependencies
# ============================================================

print_step "1. Installing build dependencies"

if [ "$DISTRO" = "DEBIAN" ]; then
    echo "  Updating package list..."
    $PKG_UPDATE
    PACKAGES="build-essential python3 libusb-1.0-0-dev dfu-util libdbus-1-dev usbutils libudev-dev"
elif [ "$DISTRO" = "REDHAT" ]; then
    $PKG_UPDATE
    PACKAGES="gcc make python3 libusbx-devel dfu-util dbus-devel usbutils libudev-dev"
else
    PACKAGES="gcc make python3 dfu-util libudev-dev"
fi

for pkg in $PACKAGES; do
    install_package "$pkg"
done

# ============================================================
# Step 2: USB permission / udev rules
# ============================================================

print_step "2. Configuring udev rules for Bosch Application Board"

UDEV_SCRIPT1="$(pwd)/submodules/coines/driver/linux/install_driver.sh"
UDEV_SCRIPT2="$(pwd)/submodules/coines/_installer_/Linux_specific/driver/install_driver.sh"

if [ -f "$UDEV_SCRIPT1" ]; then
    PREV_DIR=$PWD
    cd "$(dirname "$UDEV_SCRIPT1")"
    bash install_driver.sh
    cd "$PREV_DIR"
    print_ok "udev rules installed"
elif  [ -f "$UDEV_SCRIPT2" ]; then
    PREV_DIR=$PWD
    cd "$(dirname "$UDEV_SCRIPT2")"
    bash install_driver.sh
    cd "$PREV_DIR"
    print_ok "udev rules installed"
else
    print_warn "udev script not found at: $UDEV_SCRIPT1 or $UDEV_SCRIPT2"
    print_warn "Please install udev rules manually after setup"
fi

# Add user to dialout group for serial port access
if groups "$USER" | grep -qw "dialout"; then
    print_ok "User '$USER' is already in dialout group"
else
    echo "  Adding '$USER' to dialout group..."
    sudo usermod -aG dialout "$USER"
    print_warn "dialout group added - you must logout and login again for this to take effect"
fi

# ============================================================
# Step 3: Detect board and flash coines_bridge firmware
# ============================================================

print_step "3. Detecting board and flashing coines_bridge firmware"

MANUAL_FW_MSG() {
    print_warn "To flash firmware manually, connect the board then run:"
    print_warn "  APP3.0: cd $(pwd)/submodules/coines/firmware/app3.0/coines_bridge && ./update_coines_bridge_flash_fw.sh"
    print_warn "  APP3.1: cd $(pwd)/submodules/coines/firmware/app3.1/coines_bridge && ./update_coines_bridge_flash_fw.sh"
}

if ! command -v lsusb >/dev/null 2>&1; then
    print_warn "lsusb not available - skipping automatic board detection"
    MANUAL_FW_MSG
else
    LSUSB_OUT=$(lsusb 2>/dev/null)

    # Match by USB VID:PID first (reliable), fall back to name matching
    if echo "$LSUSB_OUT" | grep -qi "108c:ab38"; then
        BOARD="app3.1"
    elif echo "$LSUSB_OUT" | grep -qi "108c:ab3d\|APP3\.0\|APP30"; then
        BOARD="app3.0"
    else
        BOARD=""
    fi

    if [ -n "$BOARD" ]; then
        print_ok "Detected board: $BOARD"
        FW_SCRIPT="$(pwd)/submodules/coines/firmware/$BOARD/coines_bridge/update_coines_bridge_flash_fw.sh"

        if [ -f "$FW_SCRIPT" ]; then
            PREV_DIR=$PWD
            cd "$(dirname "$FW_SCRIPT")"
            bash update_coines_bridge_flash_fw.sh
            cd "$PREV_DIR"
            print_ok "coines_bridge firmware flashed successfully"
        else
            print_warn "Firmware script not found: $FW_SCRIPT"
            MANUAL_FW_MSG
        fi
    else
        print_warn "No Bosch Application Board detected via lsusb"
        print_warn "NOTE: Board must be in COINES Bridge mode (not DFU/bootloader mode) to be detected"
        MANUAL_FW_MSG
    fi
fi

# ============================================================
# Step 4: ARM GNU Toolchain (for MCU builds)
# ============================================================

print_step "4. ARM GNU Toolchain setup (for MCU builds - x86_64)"

TOOLCHAIN_BIN="$TOOLS_DIR/$ARM_TOOLCHAIN_NAME/bin"

if command -v arm-none-eabi-gcc >/dev/null 2>&1; then
    print_ok "arm-none-eabi-gcc already on PATH: $(arm-none-eabi-gcc --version | head -1)"
elif [ -f "$TOOLCHAIN_BIN/arm-none-eabi-gcc" ]; then
    print_ok "Toolchain already extracted at $TOOLS_DIR/$ARM_TOOLCHAIN_NAME"
    # Ensure PATH entry exists in .bashrc
    if ! grep -qF "$TOOLCHAIN_BIN" "$HOME/.bashrc"; then
        echo "" >> "$HOME/.bashrc"
        echo "# ARM GNU Toolchain (added by BHY2CLI setup)" >> "$HOME/.bashrc"
        echo "export PATH=\"$TOOLCHAIN_BIN:\$PATH\"" >> "$HOME/.bashrc"
    fi
    export PATH="$TOOLCHAIN_BIN:$PATH"
    print_ok "Toolchain PATH configured"
else
    echo ""
    echo "ARM GNU Toolchain is only required for MCU firmware development."
    read -r -p "Do you want to install it? [Y/n] " INSTALL_TOOLCHAIN
    INSTALL_TOOLCHAIN="${INSTALL_TOOLCHAIN:-Y}"

    if [[ "$INSTALL_TOOLCHAIN" =~ ^[Yy]$ ]]; then
        mkdir -p "$TOOLS_DIR"
        ARCHIVE_PATH="$TOOLS_DIR/${ARM_TOOLCHAIN_NAME}.tar.xz"

        echo "  Downloading ARM GNU toolchain..."
        echo "  URL: $ARM_TOOLCHAIN_URL"

        if wget --timeout=120 -q --show-progress -O "$ARCHIVE_PATH" "$ARM_TOOLCHAIN_URL"; then
            echo "  Extracting to $TOOLS_DIR..."
            tar -xf "$ARCHIVE_PATH" -C "$TOOLS_DIR"
            rm -f "$ARCHIVE_PATH"

            # Add to ~/.bashrc if not already present
            if ! grep -qF "$TOOLCHAIN_BIN" "$HOME/.bashrc"; then
                echo "" >> "$HOME/.bashrc"
                echo "# ARM GNU Toolchain (added by BHY2CLI setup)" >> "$HOME/.bashrc"
                echo "export PATH=\"$TOOLCHAIN_BIN:\$PATH\"" >> "$HOME/.bashrc"
                print_ok "Toolchain path added to ~/.bashrc"
            fi

            # Reload environment 
            source "$HOME/.bashrc"

            # Apply to current session
            export PATH="$TOOLCHAIN_BIN:$PATH"
            print_ok "ARM GNU toolchain ready: $(arm-none-eabi-gcc --version | head -1)"
            print_warn "Run 'source ~/.bashrc' or open a new terminal to make this permanent"
        else
            rm -f "$ARCHIVE_PATH"
            print_warn "Download failed - skipping ARM GNU toolchain installation"
            print_warn "To install manually, download from:"
            print_warn "  $ARM_TOOLCHAIN_URL"
            print_warn "Then extract to $TOOLS_DIR and add $TOOLCHAIN_BIN to PATH in ~/.bashrc"
        fi
    else
        print_warn "Skipping ARM GNU toolchain installation"
        print_warn "To install later, re-run this script or download from:"
        print_warn "  $ARM_TOOLCHAIN_URL"
    fi
fi

# ============================================================
# Summary
# ============================================================

echo ""
echo "============================================================"
echo " BHY2CLI Setup Complete"
echo "============================================================"
echo ""
echo "Checklist:"
echo "  [1] If dialout group was just added: logout and login again"
echo "  [2] Apply toolchain to current shell: source ~/.bashrc"
echo ""
echo "Next steps:"
echo "  Load sensor firmware:"
echo "    ./bhy2cli -b <path_to_firmware.fw>"
echo ""
echo "  Test BHY2CLI:"
echo "    ./bhy2cli -i"
echo "    ./bhy2cli -h"
echo "============================================================"
