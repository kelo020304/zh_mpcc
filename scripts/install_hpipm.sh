#!/bin/bash
# ============================================================================
# HPIPM and BLASFEO Installation Script
#
# This script automatically installs HPIPM and BLASFEO to /usr/local
# Supports x86_64 and ARM (aarch64) architectures
#
# Usage:
#   sudo ./install_hpipm.sh
#   or
#   bash install_hpipm.sh
#
# Author: Local Planner Project
# Date: 2026-02
# ============================================================================

set -e  # Exit on error

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running with sudo
if [ "$EUID" -ne 0 ]; then
    print_warning "This script requires sudo privileges for 'make install'"
    print_warning "You may be prompted for your password during installation"
fi

# Detect architecture
ARCH=$(uname -m)
print_info "Detected architecture: $ARCH"

# Set BLASFEO target based on architecture
if [ "$ARCH" = "x86_64" ]; then
    TARGET=X64_AUTOMATIC
    print_info "Using BLASFEO target: $TARGET (Intel/AMD x86_64)"
elif [ "$ARCH" = "aarch64" ]; then
    # Default to Cortex-A57 (compatible with most ARM boards)
    TARGET=ARMV8A_ARM_CORTEX_A57
    print_info "Using BLASFEO target: $TARGET (ARM Cortex-A57/A72)"
    print_info "  Compatible with: Raspberry Pi 4, Jetson Nano/TX2, etc."
    print_warning "  If compilation fails, the script will retry with GENERIC target"
else
    print_warning "Unsupported architecture: $ARCH"
    print_warning "Falling back to GENERIC target (slower but compatible)"
    TARGET=GENERIC
fi

# Installation prefix
INSTALL_PREFIX=/usr/local
print_info "Installation prefix: $INSTALL_PREFIX"

# Create temporary build directory
BUILD_DIR=~/temp_build_hpipm
print_info "Creating temporary build directory: $BUILD_DIR"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# ============================================================================
# Install BLASFEO
# ============================================================================

print_info "===== Installing BLASFEO ====="

# Skip if already installed
if [ -f "$INSTALL_PREFIX/lib/libblasfeo.a" ] && \
   { [ -f "$INSTALL_PREFIX/include/blasfeo.h" ] || [ -d "$INSTALL_PREFIX/include/blasfeo" ]; }; then
    print_info "BLASFEO already installed at $INSTALL_PREFIX, skipping build."
else

if [ -d "blasfeo" ]; then
    print_warning "BLASFEO directory already exists, removing..."
    rm -rf blasfeo
fi

print_info "Cloning BLASFEO repository..."
git clone https://github.com/giaf/blasfeo.git
cd blasfeo

print_info "Configuring BLASFEO..."
mkdir -p build && cd build

# Try to compile with architecture-specific target
if cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DTARGET=$TARGET \
    -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
    -DBUILD_SHARED_LIBS=OFF; then

    print_info "Compiling BLASFEO (this may take a few minutes)..."
    if make -j$(nproc); then
        print_info "Installing BLASFEO to $INSTALL_PREFIX..."
        sudo make install
        print_info "BLASFEO installed successfully!"
    else
        print_error "BLASFEO compilation failed with target $TARGET"
        exit 1
    fi
else
    # If CMake configuration fails, try GENERIC target
    print_warning "CMake configuration failed with target $TARGET"
    print_info "Retrying with GENERIC target..."
    rm -rf *

    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DTARGET=GENERIC \
        -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
        -DBUILD_SHARED_LIBS=OFF

    print_info "Compiling BLASFEO (GENERIC target)..."
    make -j$(nproc)

    print_info "Installing BLASFEO to $INSTALL_PREFIX..."
    sudo make install
    print_info "BLASFEO installed successfully (GENERIC target)!"
fi
fi

# Verify BLASFEO installation
# Some installs place headers in $INSTALL_PREFIX/include, others in $INSTALL_PREFIX/include/blasfeo
if [ -f "$INSTALL_PREFIX/lib/libblasfeo.a" ] && \
   { [ -f "$INSTALL_PREFIX/include/blasfeo.h" ] || [ -d "$INSTALL_PREFIX/include/blasfeo" ]; }; then
    print_info "✓ BLASFEO installation verified"
    ls -lh "$INSTALL_PREFIX/lib/libblasfeo.a"
else
    print_error "BLASFEO installation verification failed!"
    exit 1
fi

# ============================================================================
# Install HPIPM
# ============================================================================

cd "$BUILD_DIR"
print_info "===== Installing HPIPM ====="

# Skip if already installed
if [ -f "$INSTALL_PREFIX/lib/libhpipm.a" ] && \
   { [ -f "$INSTALL_PREFIX/include/hpipm_d_ocp_qp.h" ] || [ -d "$INSTALL_PREFIX/include/hpipm" ]; }; then
    print_info "HPIPM already installed at $INSTALL_PREFIX, skipping build."
else

if [ -d "hpipm" ]; then
    print_warning "HPIPM directory already exists, removing..."
    rm -rf hpipm
fi

print_info "Cloning HPIPM repository..."
git clone https://github.com/giaf/hpipm.git
cd hpipm

print_info "Configuring HPIPM..."
mkdir -p build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
    -DBUILD_SHARED_LIBS=OFF \
    -DTARGET=$TARGET \
    -DBLASFEO_PATH=$INSTALL_PREFIX

print_info "Compiling HPIPM (this may take a few minutes)..."
make -j$(nproc)

print_info "Installing HPIPM to $INSTALL_PREFIX..."
sudo make install
fi

# Verify HPIPM installation
# Some installs place headers in $INSTALL_PREFIX/include, others in $INSTALL_PREFIX/include/hpipm
if [ -f "$INSTALL_PREFIX/lib/libhpipm.a" ] && \
   { [ -f "$INSTALL_PREFIX/include/hpipm_d_ocp_qp.h" ] || [ -d "$INSTALL_PREFIX/include/hpipm" ]; }; then
    print_info "✓ HPIPM installation verified"
    ls -lh "$INSTALL_PREFIX/lib/libhpipm.a"
else
    print_error "HPIPM installation verification failed!"
    exit 1
fi

# ============================================================================
# Post-installation
# ============================================================================

print_info "Updating library cache..."
sudo ldconfig

print_info "Cleaning up temporary build directory..."
cd ~
rm -rf "$BUILD_DIR"

# ============================================================================
# Verification
# ============================================================================

print_info "===== Installation Summary ====="
echo ""
echo "BLASFEO:"
echo "  Library: $INSTALL_PREFIX/lib/libblasfeo.a"
echo "  Headers: $INSTALL_PREFIX/include/blasfeo/ (or $INSTALL_PREFIX/include/blasfeo.h)"
echo ""
echo "HPIPM:"
echo "  Library: $INSTALL_PREFIX/lib/libhpipm.a"
echo "  Headers: $INSTALL_PREFIX/include/hpipm/ (or $INSTALL_PREFIX/include/hpipm_*.h)"
echo ""

# Create verification test
print_info "Creating verification test..."
TEST_FILE=$(mktemp /tmp/test_hpipm_XXXXXX.cpp)
cat > "$TEST_FILE" << 'EOF'
#include <hpipm_d_ocp_qp.h>
#include <blasfeo_d_aux_ext_dep.h>
#include <iostream>

int main() {
    std::cout << "✓ HPIPM and BLASFEO headers found!" << std::endl;
    std::cout << "✓ Installation successful!" << std::endl;
    return 0;
}
EOF

print_info "Compiling verification test..."
if g++ "$TEST_FILE" -o /tmp/test_hpipm \
    -I$INSTALL_PREFIX/include \
    -L$INSTALL_PREFIX/lib \
    -lhpipm -lblasfeo -lm 2>/dev/null; then

    print_info "Running verification test..."
    /tmp/test_hpipm
    rm -f /tmp/test_hpipm "$TEST_FILE"
    echo ""
    print_info "===== All installations completed successfully! ====="
    print_info "You can now compile local_planner with MPCC support."
    print_info "Run: cd /path/to/catkin_ws && catkin_make"
else
    print_error "Verification test failed!"
    print_error "Headers/libraries may not be correctly installed."
    rm -f "$TEST_FILE"
    exit 1
fi

exit 0
