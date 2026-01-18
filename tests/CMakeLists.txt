#=============================================================================
#
# CMakeLists.txt - SUMP3 AXI Wrapper Simulation Build
#
#=============================================================================
#
# OVERVIEW
# ========
# This CMake configuration builds a Verilator-based simulation for testing
# the SUMP3 AXI4-Lite wrapper. The simulation includes:
#
#   - sump3_axi_wrapper: AXI4-Lite slave with hardware command processing
#   - sump3_core:        Main ILA controller
#   - sump3_rle_hub:     Serial bus hub
#   - sump3_rle_pod:     RLE capture pod
#   - simple_dut:        Example device-under-test
#
# BUILDING
# ========
#   cd tests
#   mkdir build && cd build
#   cmake ..
#   make
#
# RUNNING
# =======
#   make run           # Build and run testbench
#   make wave          # Run and open waveform viewer
#   ./bin/tb_sump3     # Run directly
#
# PREREQUISITES
# =============
#   - Verilator (sudo apt install verilator, or build from source)
#   - GTKWave (optional, for viewing waveforms)
#   - CMake 3.14+
#
#=============================================================================

cmake_minimum_required(VERSION 3.14)
project(sump3_axi_tb)

# Find Verilator
find_package(verilator REQUIRED HINTS $ENV{VERILATOR_ROOT})

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable verbose output for debugging (can be disabled for cleaner output)
set(CMAKE_VERBOSE_MAKEFILE OFF)

# Optimization flags for faster simulation
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")

# Option to enable/disable VCD tracing (tracing is slower)
option(ENABLE_TRACE "Enable VCD waveform tracing" OFF)

#=============================================================================
# Source Directories
#=============================================================================
# Root of the repository (parent of tests/)
set(REPO_ROOT ${CMAKE_SOURCE_DIR}/..)

# Library RTL (the deliverable)
set(LIB_RTL_DIR ${REPO_ROOT}/rtl)

# Test RTL (testbench, DUT)
set(TEST_RTL_DIR ${CMAKE_SOURCE_DIR}/rtl)

# Original SUMP3 files from submodule
set(SUMP3_ORIG_DIR ${REPO_ROOT}/extern/sump3/verilog)

# Patched SUMP3 files go in build directory (not tracked in git)
set(SUMP3_PATCHED_DIR ${CMAKE_BINARY_DIR}/sump3_patched)

# MesaBus protocol (required by SUMP3)
set(MESA_DIR ${REPO_ROOT}/extern/MesaBusProtocol/verilog)

#=============================================================================
# Patch SUMP3 files for Verilator compatibility
#=============================================================================
# SUMP3 uses unnamed module instantiation (halt_synthesis_*()) to cause synthesis
# failures on parameter errors. Verilator requires named instances.
# This patches the files at build time - no modified files tracked in git.

file(MAKE_DIRECTORY ${SUMP3_PATCHED_DIR})

# Patch function: adds instance names to halt_synthesis module calls
function(patch_sump3_file input_file output_file)
    file(READ ${input_file} content)
    # Add instance names to all halt_synthesis_* calls
    string(REGEX REPLACE 
        "halt_synthesis_bad_ana_ram_params\\(\\)"
        "halt_synthesis_bad_ana_ram_params u_halt_ana()"
        content "${content}")
    string(REGEX REPLACE 
        "halt_synthesis_bad_dig_ram_params\\(\\)"
        "halt_synthesis_bad_dig_ram_params u_halt_dig()"
        content "${content}")
    string(REGEX REPLACE 
        "halt_synthesis_rle_pod_bad_ram_width\\(\\)"
        "halt_synthesis_rle_pod_bad_ram_width u_halt_width()"
        content "${content}")
    string(REGEX REPLACE 
        "halt_synthesis_rle_pod_bad_rle_timestamp_bits\\(\\)"
        "halt_synthesis_rle_pod_bad_rle_timestamp_bits u_halt_ts()"
        content "${content}")
    string(REGEX REPLACE 
        "halt_synthesis_rle_pod_bad_ram_length\\(\\)"
        "halt_synthesis_rle_pod_bad_ram_length u_halt_len()"
        content "${content}")
    file(WRITE ${output_file} "${content}")
endfunction()

# Apply patches at configure time
patch_sump3_file(${SUMP3_ORIG_DIR}/sump3_core.v ${SUMP3_PATCHED_DIR}/sump3_core.v)
patch_sump3_file(${SUMP3_ORIG_DIR}/sump3_rle_hub.v ${SUMP3_PATCHED_DIR}/sump3_rle_hub.v)
patch_sump3_file(${SUMP3_ORIG_DIR}/sump3_rle_pod.v ${SUMP3_PATCHED_DIR}/sump3_rle_pod.v)

message(STATUS "Patched SUMP3 files for Verilator in: ${SUMP3_PATCHED_DIR}")

#=============================================================================
# Verilog/SystemVerilog Sources
#=============================================================================
set(VERILOG_SOURCES
    # Testbench top module
    ${TEST_RTL_DIR}/tb_top.sv
    
    # Example DUT (counter/FSM for probing)
    ${TEST_RTL_DIR}/simple_dut.sv
    
    # Library: AXI wrapper with hardware state machine
    ${LIB_RTL_DIR}/sump3_axi_wrapper.sv
    
    # Simulation stubs for SUMP3's halt_synthesis_* parameter-check modules
    ${TEST_RTL_DIR}/sump3_synth_stubs.v
    
    # SUMP3 core components (auto-patched from extern/sump3 at build time)
    ${SUMP3_PATCHED_DIR}/sump3_core.v
    ${SUMP3_PATCHED_DIR}/sump3_rle_hub.v
    ${SUMP3_PATCHED_DIR}/sump3_rle_pod.v
)

#=============================================================================
# C++ Testbench Sources
#=============================================================================
set(TB_SOURCES
    ${CMAKE_SOURCE_DIR}/tb_main.cpp
)

#=============================================================================
# Testbench Executable
#=============================================================================
add_executable(tb_sump3 ${TB_SOURCES})

# Build Verilator arguments
set(VERILATOR_COMMON_ARGS
    --cc
    --exe
    --timing                 # Enable timing simulation
    -O3                      # Verilator optimization level 3
    --x-assign unique        # Handle X assignments uniquely
    --x-initial unique       # Handle X initialization uniquely
    -Wall                    # Enable all warnings
    -Wno-fatal               # Don't treat warnings as errors
    -Wno-WIDTHEXPAND         # Suppress width expansion warnings
    -Wno-WIDTHTRUNC          # Suppress width truncation warnings
    -Wno-UNUSEDSIGNAL        # Suppress unused signal warnings
    -Wno-UNUSEDPARAM         # Suppress unused parameter warnings
    -Wno-PINCONNECTEMPTY     # Suppress empty pin connection warnings
    -Wno-CASEINCOMPLETE      # Suppress incomplete case warnings
    -Wno-UNOPTFLAT           # Suppress unoptimized flatten warnings
    -I${LIB_RTL_DIR}         # Include path for library RTL
    -I${TEST_RTL_DIR}        # Include path for test RTL
    -I${SUMP3_PATCHED_DIR}   # Include path for patched SUMP3 files
    -I${MESA_DIR}            # Include path for MesaBus protocol
    -CFLAGS "-O3 -march=native -DVL_TIME_CONTEXT"
)

# Add trace flag if enabled
if(ENABLE_TRACE)
    list(APPEND VERILATOR_COMMON_ARGS --trace)
    message(STATUS "VCD tracing: ENABLED (slower simulation)")
else()
    message(STATUS "VCD tracing: DISABLED (faster simulation)")
endif()

verilate(tb_sump3
    SOURCES ${VERILOG_SOURCES}
    TOP_MODULE tb_top
    PREFIX Vtb_top
    VERILATOR_ARGS ${VERILATOR_COMMON_ARGS}
)

# Output to bin directory
set_target_properties(tb_sump3 PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

#=============================================================================
# Custom Targets for Convenience
#=============================================================================

# Run testbench
add_custom_target(run
    COMMAND ${CMAKE_BINARY_DIR}/bin/tb_sump3
    DEPENDS tb_sump3
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Running SUMP3 AXI wrapper testbench"
)

# View waveforms with GTKWave
add_custom_target(wave
    COMMAND gtkwave ${CMAKE_BINARY_DIR}/tb_top.vcd
    DEPENDS run
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Opening waveform viewer"
)

# Clean build artifacts (more thorough than default clean)
add_custom_target(clean-all
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/CMakeFiles
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/CMakeCache.txt
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/bin
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/*.vcd
    COMMENT "Cleaning all build artifacts"
)

#=============================================================================
# Print Configuration Summary
#=============================================================================
message(STATUS "")
message(STATUS "=== SUMP3 AXI Wrapper Simulation ===")
message(STATUS "")
message(STATUS "Directories:")
message(STATUS "  Library:   ${LIB_RTL_DIR}")
message(STATUS "  Test RTL:  ${TEST_RTL_DIR}")
message(STATUS "  SUMP3 src: ${SUMP3_ORIG_DIR}")
message(STATUS "  SUMP3 out: ${SUMP3_PATCHED_DIR} (auto-patched)")
message(STATUS "  MesaBus:   ${MESA_DIR}")
message(STATUS "")
message(STATUS "Targets:")
message(STATUS "  tb_sump3   - Build testbench executable")
message(STATUS "  run        - Build and run testbench")
message(STATUS "  wave       - Run and open waveform viewer (requires GTKWave)")
message(STATUS "  clean-all  - Remove all build artifacts")
message(STATUS "")
