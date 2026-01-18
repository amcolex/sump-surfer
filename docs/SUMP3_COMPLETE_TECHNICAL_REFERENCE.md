# SUMP3 Complete Technical Reference

## Table of Contents
1. [Overview](#1-overview)
2. [Architecture](#2-architecture)
3. [Hardware Components](#3-hardware-components)
4. [Internal Serial Bus Protocols](#4-internal-serial-bus-protocols)
5. [Register Map](#5-register-map)
6. [Timing Analysis](#6-timing-analysis)
7. [Software Control Flow](#7-software-control-flow)
8. [MesaBus Protocol](#8-mesabus-protocol)

---

## 1. Overview

SUMP3 is an open-source FPGA logic analyzer developed by BlackMesaLabs. It provides hardware-efficient signal capture using Run-Length Encoding (RLE) compression, allowing thousands of signals to be captured with minimal Block RAM usage.

### Key Features
- **RLE Compression**: Each RLE Pod can capture dozens of signals with a single Block RAM (512x72 typical)
- **Scalable Architecture**: Supports up to 256 RLE Hubs, each with up to 256 RLE Pods
- **Multiple Clock Domains**: RLE Hubs enable capture across different clock domains
- **Mixed-Signal Support**: Supports digital RLE, low-speed digital sampling, and analog ADC inputs
- **Trigger System**: OR/AND/Pattern triggers with Nth trigger and trigger delay support

### Design Philosophy
SUMP3 uses a hierarchical serial bus architecture internally to minimize routing resources. While this reduces FPGA routing congestion, it introduces latency that must be carefully managed.

---

## 2. Architecture

### Block Diagram

```
                    ┌─────────────────────────────────────────────────────────────┐
                    │                        sump3_core                           │
                    │                                                             │
   Local Bus        │  ┌──────────┐   ┌──────────┐   ┌─────────────────────────┐ │
   ─────────────────┼─►│  Control │──►│ Register │──►│ Serial Bus Interface    │ │
   lb_cs_ctrl      │  │   FSM    │   │   File   │   │ (core_mosi, core_miso)  │ │
   lb_cs_data      │  └──────────┘   └──────────┘   └───────────┬─────────────┘ │
   lb_wr/lb_rd     │                                             │               │
   lb_wr_d[31:0]   │  ┌──────────────┐   ┌─────────────────────┐ │               │
   lb_rd_d[31:0]◄──┼──│ Optional    │◄──│ Trigger Logic       │ │               │
   lb_rd_rdy     ◄─┼──│ Analog RAM  │   │ (dig_triggers[31:0])│ │               │
                    │  └──────────────┘   └─────────────────────┘ │               │
                    └─────────────────────────────────────────────┼───────────────┘
                                                                  │
                    ┌─────────────────────────────────────────────┼───────────────┐
                    │                    sump3_rle_hub (per clock domain)         │
                    │                                             │               │
                    │  ┌───────────────┐   ┌──────────────────────▼─────────────┐ │
        clk_lb ─────┼─►│ Clock Domain │   │ Serial Bus Decode & Arbitration   │ │
                    │  │ Crossing     │◄─►│ (pod_mosi, pod_miso)               │ │
        clk_cap ────┼─►│ Logic        │   └────────────────┬───────────────────┘ │
                    │  └───────────────┘                    │                     │
                    │                                       │                     │
                    │  ┌──────────────┐   trigger_mosi ◄────┼───► trigger_miso   │
                    │  │ Trigger      │◄──────────────────┘                     │
                    │  │ Aggregation  │                                           │
                    │  └──────────────┘                                           │
                    └─────────────────────────────────────────────────────────────┘
                                                            │
              ┌─────────────────────────────────────────────┼─────────────────────┐
              │                   sump3_rle_pod (per signal group)               │
              │                                             │                     │
              │  ┌──────────────────┐   ┌──────────────────▼─────────────────┐   │
  events[] ───┼─►│ RLE Compression  │──►│ Serial Bus Decode                  │   │
              │  │ Engine           │   │                                     │   │
              │  └────────┬─────────┘   └─────────────────────────────────────┘   │
              │           │                                                       │
              │  ┌────────▼─────────┐   ┌─────────────────────────────────────┐   │
              │  │ Block RAM        │   │ Trigger Detection                   │───┼─► pod_miso
              │  │ (RLE Storage)    │   │ (Pattern/OR/AND)                    │   │   (trigger)
              │  └──────────────────┘   └─────────────────────────────────────┘   │
              └───────────────────────────────────────────────────────────────────┘
```

### Hierarchy
```
sump3_top
├── sump3_core (1 instance)
│   ├── Control registers
│   ├── State machine (Idle/Arm/Reset/Init/Sleep)
│   ├── Optional analog RAM
│   └── Serial bus to RLE Hubs
│
├── sump3_rle_hub (1-256 instances, one per clock domain)
│   ├── Clock domain crossing (clk_lb ↔ clk_cap)
│   ├── Serial bus switch
│   └── Trigger aggregation
│
└── sump3_rle_pod (1-256 per hub)
    ├── RLE compression engine
    ├── Block RAM storage
    └── Local trigger detection
```

---

## 3. Hardware Components

### 3.1 sump3_core.v

The central control block providing the local bus interface and global control.

#### Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `ana_ls_enable` | 0 | Enable low-speed analog capture |
| `ana_ram_depth_len` | 1024 | Analog RAM depth |
| `ana_ram_depth_bits` | 10 | Log2 of RAM depth |
| `ana_ram_width` | 32 | RAM data width (must be 32) |
| `rle_hub_num` | 1 | Number of RLE Hubs |
| `view_rom_en` | 0 | Enable signal naming ROM |
| `view_rom_size` | 16384 | ROM size in bits |
| `ck_freq_mhz` | 80 | Capture clock MHz (integer part) |
| `ck_freq_fracts` | 0 | Capture clock MHz (fractional) |
| `thread_lock_en` | 1 | Enable multi-thread bus locking |
| `bus_busy_timer_en` | 1 | Enable bus busy timer |
| `bus_busy_timer_len` | 1023 | Bus busy timeout in clocks |
| `sump_id` | 0x53 | ID = "S3" for SUMP3 |
| `sump_rev` | 0x01 | Hardware revision |

#### Ports
| Port | Direction | Width | Description |
|------|-----------|-------|-------------|
| `clk_lb` | Input | 1 | Local bus clock |
| `clk_cap` | Input | 1 | Capture clock |
| `ck_tick` | Input | 1 | Slow tick clock for LS sampling |
| `lb_cs_ctrl` | Input | 1 | Control register chip select |
| `lb_cs_data` | Input | 1 | Data register chip select |
| `lb_wr` | Input | 1 | Write strobe |
| `lb_rd` | Input | 1 | Read strobe |
| `lb_wr_d` | Input | 32 | Write data |
| `lb_rd_d` | Output | 32 | Read data |
| `lb_rd_rdy` | Output | 1 | Read data valid |
| `core_mosi` | Output | N | Serial bus to Hubs (1 per hub) |
| `core_miso` | Input | N | Serial bus from Hubs |
| `trigger_mosi` | Output | N | Trigger broadcast to Hubs |
| `trigger_miso` | Input | N | Trigger from Hubs |
| `trigger_in` | Input | 1 | External trigger input |
| `trigger_out` | Output | 1 | Trigger output |
| `sump_is_armed` | Output | 1 | Armed status |
| `sump_is_awake` | Output | 1 | Awake status (for clock gating) |
| `core_user_ctrl` | Output | 32 | User control bits |

### 3.2 sump3_rle_hub.v

Clock domain bridge and bus switch between Core and Pods.

#### Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `hub_name` | "name_unknown" | 12-char ASCII name |
| `hub_instance` | 0 | Instance number (0-255) |
| `user_bus_en` | 0 | Enable 32-bit user bus |
| `ck_freq_mhz` | 80 | Hub clock frequency |
| `rle_pod_num` | 1 | Number of pods for this hub |

#### Key Functions
1. **Clock Domain Crossing**: Synchronizes data between `clk_lb` and `clk_cap`
2. **Serial Bus Translation**: Converts core serial protocol to pod serial protocol
3. **Trigger Aggregation**: Combines pod triggers for OR/AND operations
4. **User Bus Interface**: Optional 32-bit user bus for custom logic access

### 3.3 sump3_rle_pod.v

The RLE compression and storage engine.

#### Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `pod_name` | "name_unknown" | 12-char ASCII name |
| `pod_instance` | 0 | Instance number |
| `pod_disable` | 0 | Disable pod (keep on trigger team) |
| `rle_disable` | 0 | Disable RLE for faster Fmax |
| `rle_ram_depth_len` | 1024 | RAM depth |
| `rle_ram_depth_bits` | 10 | Log2 of RAM depth |
| `rle_ram_width` | 72 | Total RAM width |
| `rle_code_bits` | 2 | Code bits (always 2) |
| `rle_timestamp_bits` | 38 | Timestamp bits |
| `rle_data_bits` | 32 | Event data bits (1-8192) |
| `trigger_en` | 1 | Enable trigger capability |
| `trig_bits` | 0xFFFFFFFF | Which bits can trigger |

#### RAM Format
```
[rle_ram_width-1 : rle_ram_width-2] : Code (2 bits)
  - 0x0 = Invalid
  - 0x1 = Pre-Trigger
  - 0x2 = Trigger
  - 0x3 = Post-Trigger

[rle_ram_width-3 : rle_data_bits]   : Timestamp

[rle_data_bits-1 : 0]               : Event Data
```

Example RAM configurations:
| Config | Width | Format | Max Capture Time @ 100MHz |
|--------|-------|--------|---------------------------|
| 1Kx36 | 36 | 2+14+20 | 2^14 × 10ns = 164μs |
| 2Kx36 | 36 | 2+24+10 | 2^24 × 10ns = 168ms |
| 4Kx72 | 72 | 2+30+40 | 2^30 × 10ns = 10.7s |

---

## 4. Internal Serial Bus Protocols

### 4.1 Core to Hub Serial Protocol

The communication between `sump3_core` and `sump3_rle_hub` uses a 1-bit serial bus with variable-length packets.

```
SB HEADER DATA
 1 00                : Read Control Register      (3 bits)
 1 01                : Read Data Register         (3 bits)
 1 10     0x00       : Write Control Register     (11 bits = 3+8)
 1 11     0x00000000 : Write Data Register        (35 bits = 3+32)
```

#### Timing
- The `rle_hub_sr` shift register has a 4-bit preamble delay
- Total packet length: 3, 11, or 35 bits depending on operation
- Readback uses start-bit detection followed by 32-bit shift

### 4.2 Hub to Pod Serial Protocol

Communication between `sump3_rle_hub` and `sump3_rle_pod`:

```
SB HEADER ADDR DATA
 1 000                    : Idle           (4 bits)
 1 001                    : Init           (4 bits)
 1 010                    : Arm            (4 bits)
 1 011                    : Trigger        (4 bits)
 1 100    0x00            : Register Read  (12 bits = 4+8)
 1 101    0x00 0x00000000 : Register Write (44 bits = 4+8+32)
```

### 4.3 Serial Bus Timing Analysis

**Critical Path: Write to RLE Pod Register**

```
                                                          Clock Cycles
┌──────────┐     ┌──────────────┐     ┌──────────────┐
│ lb_wr    │────►│ core_mosi    │────►│ hub decode   │
│ (clk_lb) │     │ serialize    │     │ (clk_lb)     │
└──────────┘     └──────────────┘     └──────────────┘
                      35 cycles            ~3 cycles
                                               │
                                               ▼
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│ pod decode   │◄────│ pod_mosi     │◄────│ CDC crossing │
│ (clk_cap)    │     │ serialize    │     │ (async)      │
└──────────────┘     └──────────────┘     └──────────────┘
   ~44 cycles            44 cycles           ~4 cycles

Total: 35 + 3 + 4 + 44 + 44 = ~130 clock cycles for write propagation
```

**Critical Path: Read from RLE Pod Register**

```
Write address + Read request:  ~130 cycles (as above)
Wait for pod to respond:       ~5 cycles
Pod serializes response:       ~33 cycles (1 start + 32 data)
Hub receives and forwards:     ~5 cycles
Core receives response:        ~33 cycles

Total: ~200+ clock cycles for read operation
```

---

## 5. Register Map

### 5.1 Control Register Commands (lb_cs_ctrl)

Written to control register to select subsequent data operations.

| Cmd | Name | Description |
|-----|------|-------------|
| 0x00 | Idle | Idle state + Read Status |
| 0x01 | Arm | Arm for acquisition + Read Status |
| 0x02 | Reset | Reset all state |
| 0x03 | Init | Initialize RAM |
| 0x04 | Sleep | Enter sleep mode (clock gating) |
| 0x0B | Read HW ID | Read hardware ID and revision |
| 0x0C | Read Ana RAM Config | Read analog RAM dimensions |
| 0x0D | Read Tick Freq | Read tick clock frequency |
| 0x0E | Read Ana First Ptr | Read first analog sample pointer |
| 0x0F | Read RAM Data | Read RAM data (auto-increment) |
| 0x10 | Read Dig First Ptr | Read first digital sample pointer |
| 0x11 | Read Dig Freq | Read digital clock frequency |
| 0x12 | Read Dig RAM Config | Read digital RAM dimensions |
| 0x13 | Read Record Profile | Read acquisition profile |
| 0x14 | Read Trigger Src | Read trigger source |
| 0x15 | Read View ROM Size | Read View ROM size in KB |
| 0x1C | Thread Lock Set | Set thread lock bit |
| 0x1D | Thread Lock Clear | Clear thread lock bit |
| 0x1E | Thread Pool Set | Set thread pool bit |
| 0x1F | Thread Pool Clear | Clear thread pool bit |
| 0x20 | User Ctrl | Load user control register |
| 0x21 | Record Config | Load record configuration |
| 0x22 | Tick Divisor | Load tick clock divisor |
| 0x23 | Trigger Type | Load trigger type |
| 0x24 | Trigger Digital | Load digital trigger field |
| 0x25 | Trigger Analog | Load analog trigger field |
| 0x26 | Ana Post Trig Len | Load analog post-trigger length |
| 0x27 | Trigger Delay | Load trigger delay |
| 0x28 | Trigger Nth | Load Nth trigger count |
| 0x29 | RAM Read Ptr | Load RAM read pointer |
| 0x2A | Dig Post Trig Len | Load digital post-trigger length |
| 0x2B | RAM Page Select | Load RAM page select |
| 0x30 | RLE Hub Num | Read number of RLE Hubs |
| 0x31 | RLE Pod Num | Read number of Pods for selected Hub |
| 0x32 | RLE Instance Addr | Write Hub/Pod/Register address |
| 0x33 | RLE Pod Data | Read/Write Pod register data |
| 0x34 | RLE Trigger Src | Read trigger source pod instance |
| 0x35 | RLE Trigger Width | Write trigger pulse width |
| 0x36 | RLE Hub Freq | Read hub clock frequency |
| 0x37 | User Bus Addr | Write user bus address |
| 0x38 | User Bus Write | Write user bus data |
| 0x39 | User Bus Read | Read user bus data |
| 0x3A | Hub HW Config | Read hub hardware config |
| 0x3C | Hub Instance | Read hub instance number |
| 0x3D-3F | Hub Name | Read hub ASCII name |

### 5.2 RLE Pod Register Map

Accessed via cmd 0x32 (address select) and 0x33 (data access).

| Addr | Name | R/W | Description |
|------|------|-----|-------------|
| 0x00 | Pod HW Config | R | HW revision, enables |
| 0x02 | Trigger Latency | R | Trigger clock latencies |
| 0x03 | Trigger Config | R/W | Type and position |
| 0x04 | Trigger Enable | R/W | Trigger enable bits [31:0] |
| 0x05 | RLE Bit Mask | R/W | RLE mask bits [31:0] |
| 0x07 | Compare Value | R/W | Pattern match value |
| 0x08 | RAM Page Ptr | R/W | Page[27:20], Ptr[19:0] |
| 0x09 | RAM Data | R | RAM readout (auto-increment) |
| 0x0A | RAM Config | R | Depth/width/timestamp bits |
| 0x0B | User Ctrl | R/W | User control bits |
| 0x0E | Triggerable | R | Which bits can trigger |
| 0x0F | Trigger Source | R | Trigger source bit |
| 0x10 | View ROM Size | R | View ROM size in KB |
| 0x1C | Pod Instance | R | Instance number |
| 0x1D-1F | Pod Name | R | ASCII name |

### 5.3 Status Register Format

When reading control register (lb_rd on lb_cs_ctrl):

```
[31]    : Bus Busy Bit
[30]    : Thread Lock Bit  
[28:24] : Capture Status
[5:0]   : Current Command
```

Capture Status bits:
```
[0] : Armed
[1] : Pre-Trigger (RAM pre-filled)
[2] : Triggered
[3] : Acquired (capture complete)
[4] : Init RAM in progress
```

### 5.4 Trigger Types

| Value | Type | Description |
|-------|------|-------------|
| 0x0 | AND Rising | All trigger bits must be high |
| 0x1 | AND Falling | All trigger bits go from high to low |
| 0x2 | OR Rising | Any trigger bit goes high |
| 0x3 | OR Falling | Any trigger bit goes low |
| 0x4 | Analog Rising | Analog crosses above threshold |
| 0x5 | Analog Falling | Analog crosses below threshold |
| 0x6 | External Rising | External trigger rising edge |
| 0x7 | External Falling | External trigger falling edge |
| 0x8 | OR Either Edge | Any trigger bit changes |

---

## 6. Timing Analysis

### 6.1 Critical Timing Paths

#### Path 1: Local Bus Write to Core Register
- **Latency**: 1-2 clock cycles
- **Description**: Direct register write within sump3_core
- **Consideration**: Immediate effect

#### Path 2: Local Bus Write to Hub Register
- **Latency**: ~40 clock cycles
- **Description**: Serialization through core_mosi to hub
- **Calculation**: 
  - 4-bit preamble + 35-bit write packet = 39 bits
  - Plus synchronization: ~2-4 cycles

#### Path 3: Local Bus Write to Pod Register  
- **Latency**: ~130 clock cycles
- **Description**: Through core → hub → pod
- **Calculation**:
  - Core to hub: ~40 cycles
  - CDC crossing: ~4 cycles  
  - Hub to pod: ~48 cycles (4-bit preamble + 44-bit packet)

#### Path 4: Local Bus Read from Pod Register
- **Latency**: ~200+ clock cycles
- **Description**: Full round-trip through hierarchy
- **Calculation**:
  - Address write: ~130 cycles
  - Pod response generation: ~5 cycles
  - Response serialization: ~33 cycles
  - Reverse path: ~35 cycles

#### Path 5: Trigger Propagation
- **Latency**: ~35 clock cycles (from pod detection to core trigger)
- **Per the source comments**:
  ```
  trig_offset_core_cks = 8   // clks from trig_miso to trig_mosi
  trig_offset_miso_cks = 12  // clks from trig-in to trig_miso   
  trig_offset_mosi_cks = 17  // clks from trig_mosi to pod trig
  ```
- **Total**: 8 + 12 + 17 = 37 cycles typical

### 6.2 Bus Busy Timer

The `bus_busy_timer` feature (when enabled) provides hardware-level backpressure:

```verilog
parameter bus_busy_timer_en  = 1,  // Enable bus busy timer
parameter bus_busy_timer_len = 1023, // Number of bus clocks to wait
```

When a bus access starts, the busy bit sets for `bus_busy_timer_len` clocks. Software should poll the control register's busy bit before issuing new commands.

### 6.3 Thread Locking

For multi-threaded software access:

1. **Thread Lock Set** (0x1C): Request exclusive access
   - Write a bit to claim a lock
   - Read back to verify lock was granted
   - Only one lock can be active at a time

2. **Thread Lock Clear** (0x1D): Release lock

3. **Thread Pool** (0x1E/0x1F): ID allocation mechanism

---

## 7. Software Control Flow

### 7.1 Initialization Sequence

```python
# 1. Read hardware configuration
wr(ctrl, 0x0B)           # Select HW ID register
hw_id = rd(data)         # Read HW ID

# 2. Enumerate RLE hierarchy
wr(ctrl, 0x30)           # Select RLE Hub count
num_hubs = rd(data)

for hub in range(num_hubs):
    # Select hub
    wr(ctrl, 0x32)
    wr(data, (hub << 16))
    
    # Get pod count
    wr(ctrl, 0x31)
    num_pods = rd(data)
    
    for pod in range(num_pods):
        # Read pod configuration...
```

### 7.2 Arm and Capture Sequence

```python
# 1. Reset
wr(ctrl, 0x02)           # Reset command
wr(data, 0x00000000)

# 2. Configure triggers
wr(ctrl, 0x23)           # Trigger type
wr(data, 0x02)           # OR Rising

wr(ctrl, 0x24)           # Trigger field
wr(data, trigger_bits)

# 3. Configure post-trigger length
wr(ctrl, 0x26)
wr(data, post_trigger_samples)

# 4. Initialize RAM
wr(ctrl, 0x03)           # Init command
wr(data, 0x00000000)

# 5. Wait for init complete
while True:
    wr(ctrl, 0x00)       # Idle + read status
    status = rd(data)
    if (status & 0x10) == 0:  # Init bit cleared
        break

# 6. Arm
wr(ctrl, 0x01)           # Arm command
wr(data, 0x00000000)

# 7. Poll for trigger/capture complete
while True:
    wr(ctrl, 0x00)       # Read status
    status = rd(data)
    if (status & 0x08):  # Acquired bit
        break
```

### 7.3 Download Sequence

```python
# 1. Return to idle
wr(ctrl, 0x00)

# 2. Read first sample pointer
wr(ctrl, 0x0E)
first_ptr = rd(data)

# 3. Set read pointer
wr(ctrl, 0x29)
wr(data, first_ptr)

# 4. Read samples
wr(ctrl, 0x0F)           # RAM data command (auto-increment)
for i in range(ram_depth):
    sample = rd(data)    # Each read advances pointer
```

### 7.4 RLE Pod Download

```python
# 1. Select hub and pod
wr(ctrl, 0x32)
wr(data, (hub << 16) | (pod << 8) | reg_addr)

# 2. Wait for serial bus (important!)
sleep_ms(10)  # Or poll busy bit

# 3. Set RAM read pointer
# Pod reg 0x08 = page + pointer
wr(ctrl, 0x32)
wr(data, (hub << 16) | (pod << 8) | 0x08)
wr(ctrl, 0x33)
wr(data, (page << 20) | start_ptr)

# 4. Wait for serial bus
sleep_ms(10)

# 5. Read RAM data
# Pod reg 0x09 = RAM data (auto-increment)
wr(ctrl, 0x32)
wr(data, (hub << 16) | (pod << 8) | 0x09)

for i in range(ram_depth):
    wr(ctrl, 0x33)       # Read data register
    sample = rd(data)    # Note: Must wait for serial readback!
    sleep_ms(1)          # Critical: wait for serial round-trip
```

---

## 8. MesaBus Protocol

### 8.1 Overview

MesaBus is the external protocol used to communicate with SUMP3 from a host system (PC via FTDI UART, or potentially AXI).

### 8.2 Packet Format

```
Preamble : 0xFFF0 (establishes nibble-byte alignment)
Slot     : 0x00-0xFD (device address, decremented at each hop)
           0xFE (null destination)
           0xFF (broadcast)
SubSlot  : Upper nibble - destination block (0-F)
           Lower nibble - command
Length   : Number of payload bytes (0-255)
Payload  : Variable length data
```

### 8.3 SubSlot 0x0 - Local Bus

| Command | Description | Payload |
|---------|-------------|---------|
| 0x0 | Write | ADDR(4B) + DATA(4B)... |
| 0x1 | Read | ADDR(4B) + LEN(4B) |
| 0x2 | Write Repeat | ADDR(4B) + DATA(4B)... (same addr) |
| 0x3 | Read Repeat | ADDR(4B) + LEN(4B) (same addr) |
| 0x4 | Write Packet | ADDR(4B) + DATA(4B) + ADDR + DATA... |

### 8.4 Readback Format

```
Header   : 0xF0FE0004 (F0=preamble, FE=return slot, 00=subslot, 04=length)
Payload  : Read data (4 bytes per DWORD)
```

### 8.5 Example: Write 0x12345678 to Address 0x00000098

```
TX: FFF0 00 00 08 00000098 12345678
     │   │  │  │  │        └─ Data
     │   │  │  │  └────────── Address
     │   │  │  └───────────── Length (8 bytes)
     │   │  └──────────────── SubSlot=0, Cmd=0 (Write)
     │   └─────────────────── Slot 0
     └─────────────────────── Preamble
```

### 8.6 Example: Read 1 DWORD from Address 0x00000098

```
TX: FFF0 00 01 08 00000098 00000001
                           └─ Length (1 DWORD)
RX: F0FE 00 04 12345678
              └─ Read data
```

---

## Summary of Key Design Considerations for AXI Wrapper

1. **Timing**: The internal serial buses introduce significant latency (100-200+ cycles for RLE pod access). The AXI wrapper must handle this gracefully.

2. **Two-Register Interface**: SUMP3 uses a control/data register pair model, not a traditional address-mapped register file.

3. **Read Latency**: Reads require multiple steps (write command → wait → read data). The `lb_rd_rdy` signal indicates valid data.

4. **Clock Domains**: The local bus clock (`clk_lb`) and capture clock (`clk_cap`) may be different.

5. **State Machine**: The hardware has distinct states (Idle, Armed, Triggered, etc.) that must be managed properly.

6. **Serial Bus Protocol**: Access to RLE Pods requires serialization through the hub, introducing variable latency.

7. **Busy Indication**: The bus_busy_bit and thread_lock mechanisms exist to handle slow operations.
