#!/bin/bash
# Build and install unitree_sdk2 from the submodule
# Usage: ./build_unitree_sdk2.sh [install_prefix]
#   Default install prefix: /opt/unitree_robotics

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_DIR="${SCRIPT_DIR}/unitree_sdk2"
INSTALL_PREFIX="${1:-/opt/unitree_robotics}"

if [ ! -d "${SDK_DIR}" ]; then
    echo "Error: unitree_sdk2 submodule not found at ${SDK_DIR}"
    echo "Run: git submodule update --init --recursive"
    exit 1
fi

echo "=== Building unitree_sdk2 ==="
echo "  Source:  ${SDK_DIR}"
echo "  Install: ${INSTALL_PREFIX}"

mkdir -p "${SDK_DIR}/build"
cd "${SDK_DIR}/build"

cmake .. -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}"
make -j$(nproc)

echo ""
echo "=== Installing unitree_sdk2 to ${INSTALL_PREFIX} ==="
sudo make install

echo ""
echo "=== unitree_sdk2 installed successfully ==="
