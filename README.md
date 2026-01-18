# SUMP-Surfer

An AXI4-Lite wrapper for the [SUMP3](https://blackmesalabs.wordpress.com/sump3-logic-analyzer/) FPGA Integrated Logic Analyzer (ILA).

## Overview

SUMP-Surfer provides a hardware state machine that handles all SUMP3 protocol complexity, enabling simple software drivers for embedded Linux systems. Instead of bit-banging the SUMP3 local bus protocol with precise timing requirements, software can:

1. Write a command to an AXI register
2. Wait for an IRQ (or poll status)
3. Read the result

This design enables efficient Linux kernel drivers using `wait_for_completion()` instead of busy-wait loops with `udelay()`.

## Features

- **AXI4-Lite Slave Interface** - Standard 32-bit data, 8-bit address
- **Hardware Command Processing** - State machine handles SUMP3 serial bus timing
- **IRQ Support** - Completion interrupt for efficient driver integration
- **Timeout Handling** - Configurable timeout with error reporting
- **Full SUMP3 Support** - All commands including:
  - State commands (ARM, RESET, INIT, IDLE, SLEEP)
  - Local reads/writes (trigger config, RAM access)
  - Serial bus operations (RLE hub/pod access)
  - Analog and digital capture

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│  Software (Linux Driver)                                            │
│    - Write CMD, ADDR, WDATA registers                               │
│    - Set CTRL[START]                                                │
│    - Wait for IRQ                                                   │
│    - Read RDATA                                                     │
└──────────────────────────────┬──────────────────────────────────────┘
                               │ AXI4-Lite
┌──────────────────────────────▼──────────────────────────────────────┐
│  sump3_axi_wrapper                                                  │
│    - AXI slave interface                                            │
│    - Command state machine                                          │
│    - Timeout handling                                               │
│    - IRQ generation                                                 │
└──────────────────────────────┬──────────────────────────────────────┘
                               │ SUMP3 Local Bus
┌──────────────────────────────▼──────────────────────────────────────┐
│  sump3_core                                                         │
│    - Central ILA controller                                         │
│    - Analog RAM (optional)                                          │
│    - Serial bus to RLE hubs                                         │
└────────────────┬────────────────────────────────────┬───────────────┘
                 │ Serial Bus                         │
     ┌───────────▼───────────┐            ┌───────────▼───────────┐
     │  sump3_rle_hub        │            │  sump3_rle_hub        │
     │  (Clock Domain 0)     │            │  (Clock Domain 1)     │
     └───────────┬───────────┘            └───────────┬───────────┘
                 │                                    │
     ┌───────────▼───────────┐            ┌───────────▼───────────┐
     │  sump3_rle_pod        │            │  sump3_rle_pod        │
     │  (RLE Capture)        │            │  (RLE Capture)        │
     └───────────────────────┘            └───────────────────────┘
```

## Register Map

| Offset | Name       | Access | Description                                      |
|--------|------------|--------|--------------------------------------------------|
| 0x00   | CMD        | R/W    | Command code to execute                          |
| 0x04   | ADDR       | R/W    | Target address: {hub[23:16], pod[15:8], reg[7:0]}|
| 0x08   | WDATA      | R/W    | Write data for write operations                  |
| 0x0C   | CTRL       | R/W    | Control: [0]=START, [1]=IRQ_EN, [2]=ABORT        |
| 0x10   | STATUS     | R      | Status: [0]=BUSY, [1]=DONE, [2]=ERROR, [3]=IRQ   |
| 0x14   | RDATA      | R      | Read result data from completed command          |
| 0x18   | IRQ_STATUS | R/W1C  | IRQ status (write 1 to clear)                    |
| 0x1C   | HW_INFO    | R      | {ID[31:16], hub_count[15:8], revision[7:0]}      |
| 0x20   | CAP_STATUS | R      | Capture status from SUMP3 core                   |
| 0x24   | TIMEOUT    | R/W    | Timeout value in clock cycles                    |

### Command Codes

**State Commands (0x00-0x0F)**
- `0x01` ARM - Arm the ILA for capture
- `0x02` RESET - Reset the ILA
- `0x03` INIT - Initialize RAM (required before ARM)
- `0x04` IDLE - Return to idle state
- `0x05` SLEEP - Enter sleep mode

**Local Reads (0x10-0x1F)** - Read SUMP3 core registers
- `0x10` RD_HW_ID, `0x11` RD_HUB_COUNT, `0x12` RD_STATUS, etc.

**Local Writes (0x20-0x2F)** - Configure SUMP3 core
- `0x23` WR_TRIG_TYPE, `0x24` WR_TRIG_DIG_FIELD, `0x2A` WR_DIG_POST_TRIG, etc.

**Serial Bus Reads (0x30-0x3F)** - Read from RLE hubs/pods
- `0x30` RD_HUB_FREQ, `0x31` RD_POD_COUNT, `0x32` RD_POD_REG, etc.

**Serial Bus Writes (0x40-0x4F)** - Write to RLE hubs/pods
- `0x40` WR_POD_REG, `0x41` WR_TRIG_WIDTH

See [sump3_axi_wrapper.sv](rtl/sump3_axi_wrapper.sv) for complete documentation.

## Usage Example

```c
// Basic capture workflow (pseudo-code)

// 1. Reset
axi_write(REG_CMD, CMD_RESET);
axi_write(REG_CTRL, CTRL_START | CTRL_IRQ_EN);
wait_for_irq();

// 2. Configure trigger
axi_write(REG_WDATA, TRIG_OR_RISING);
axi_write(REG_CMD, CMD_WR_TRIG_TYPE);
axi_write(REG_CTRL, CTRL_START | CTRL_IRQ_EN);
wait_for_irq();

// 3. Initialize RAM
axi_write(REG_CMD, CMD_INIT);
axi_write(REG_CTRL, CTRL_START | CTRL_IRQ_EN);
wait_for_irq();

// 4. ARM
axi_write(REG_CMD, CMD_ARM);
axi_write(REG_CTRL, CTRL_START | CTRL_IRQ_EN);
wait_for_irq();

// 5. Wait for capture (poll or use external trigger notification)
do {
    axi_write(REG_CMD, CMD_RD_STATUS);
    axi_write(REG_CTRL, CTRL_START | CTRL_IRQ_EN);
    wait_for_irq();
    status = axi_read(REG_RDATA);
} while (!(status & STATUS_ACQUIRED));

// 6. Download data...
```

## Building the Simulation

### Prerequisites

- [Verilator](https://verilator.org/) 5.0+ (`sudo apt install verilator` or build from source)
- CMake 3.14+
- C++17 compiler
- GTKWave (optional, for viewing waveforms)

### Build Steps

```bash
cd tests
mkdir build && cd build
cmake ..
make
```

### Running Tests

```bash
# Run the testbench
make run

# Run with VCD waveform tracing (slower)
cmake -DENABLE_TRACE=ON ..
make run
make wave  # Opens GTKWave
```

### Test Coverage

The testbench includes 30 comprehensive tests:

1. **Basic Connectivity** - HW_INFO, HW_ID, hub enumeration
2. **State Commands** - INIT, ARM, RESET
3. **Local Operations** - Trigger configuration, RAM access
4. **Serial Bus** - Pod register read/write, hub info
5. **Capture Workflow** - Full configure → arm → trigger → download
6. **Signal Verification** - FSM state capture, RLE timestamps
7. **Analog Capture** - ADC configuration, threshold triggers
8. **Multi-Clock Domain** - 50 MHz and 200 MHz capture domains

## Project Structure

```
sump-surfer/
├── rtl/
│   └── sump3_axi_wrapper.sv    # Main deliverable: AXI4-Lite wrapper
├── tests/
│   ├── CMakeLists.txt          # Build configuration
│   ├── tb_main.cpp             # Verilator testbench (C++)
│   └── rtl/
│       ├── tb_top.sv           # Testbench top module
│       ├── simple_dut.sv       # Example DUT with FSM
│       └── sump3_synth_stubs.v # Simulation stubs
├── extern/
│   ├── sump3/                  # SUMP3 core (git submodule)
│   └── MesaBusProtocol/        # MesaBus protocol (git submodule)
└── docs/
    └── SUMP3_COMPLETE_TECHNICAL_REFERENCE.md
```

## Integration

### Vivado / AMD-Xilinx

1. Add `rtl/sump3_axi_wrapper.sv` to your project
2. Add the SUMP3 Verilog files from `extern/sump3/verilog/`
3. Connect the AXI4-Lite interface to your AXI interconnect
4. Connect the SUMP3 local bus signals to `sump3_core`
5. Connect `irq` to your interrupt controller

### Parameters

```systemverilog
module sump3_axi_wrapper #(
    parameter int C_S_AXI_DATA_WIDTH = 32,   // AXI data width (must be 32)
    parameter int C_S_AXI_ADDR_WIDTH = 8,    // AXI address width
    parameter int DEFAULT_TIMEOUT    = 8192  // Clock cycles before timeout
)
```

## SUMP3 Background

[SUMP3](https://blackmesalabs.wordpress.com/sump3-logic-analyzer/) is an open-source FPGA logic analyzer from BlackMesaLabs featuring:

- **RLE Compression** - Efficient storage using run-length encoding
- **Scalable Architecture** - Up to 256 hubs × 256 pods
- **Multi-Clock Domain** - Capture across different clock domains
- **Mixed-Signal** - Digital RLE + analog ADC capture
- **Flexible Triggers** - OR/AND/pattern/analog threshold

The internal serial bus architecture minimizes FPGA routing but introduces latency (100-200+ cycles for pod access). This wrapper handles all timing complexity in hardware.

## License

MIT License - See individual source files for details.

SUMP3 core files are from [BlackMesaLabs](https://github.com/blackmesalabs) under their respective licenses.
