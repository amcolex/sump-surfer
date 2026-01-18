#!/bin/bash
#-----------------------------------------------------------------------------
# build.sh - Build and run the SUMP3 AXI wrapper testbench
#-----------------------------------------------------------------------------

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}SUMP3 AXI Wrapper Build Script${NC}"
echo -e "${GREEN}============================================${NC}"

# Check for Verilator
if ! command -v verilator &> /dev/null; then
    echo -e "${RED}ERROR: Verilator not found in PATH${NC}"
    echo "Please install Verilator or add it to your PATH"
    exit 1
fi

echo -e "${YELLOW}Verilator version:${NC}"
verilator --version

# Create build directory
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

# Configure with CMake
echo -e "\n${YELLOW}Configuring with CMake...${NC}"
cmake ..

# Build
echo -e "\n${YELLOW}Building...${NC}"
cmake --build . -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

# Run simulation
echo -e "\n${YELLOW}Running simulation...${NC}"
cmake --build . --target run

echo -e "\n${GREEN}============================================${NC}"
echo -e "${GREEN}Build complete!${NC}"
echo -e "${GREEN}============================================${NC}"

# Check if VCD was generated
if [ -f "${BUILD_DIR}/tb_top.vcd" ]; then
    echo -e "${GREEN}Waveform file generated: ${BUILD_DIR}/tb_top.vcd${NC}"
    echo "To view: gtkwave ${BUILD_DIR}/tb_top.vcd"
fi
