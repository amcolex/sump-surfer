`timescale 1ns / 100ps
//=============================================================================
//
//  ███████╗██╗   ██╗███╗   ███╗██████╗ ██████╗      █████╗ ██╗  ██╗██╗
//  ██╔════╝██║   ██║████╗ ████║██╔══██╗╚════██╗    ██╔══██╗╚██╗██╔╝██║
//  ███████╗██║   ██║██╔████╔██║██████╔╝ █████╔╝    ███████║ ╚███╔╝ ██║
//  ╚════██║██║   ██║██║╚██╔╝██║██╔═══╝  ╚═══██╗    ██╔══██║ ██╔██╗ ██║
//  ███████║╚██████╔╝██║ ╚═╝ ██║██║     ██████╔╝    ██║  ██║██╔╝ ╚██╗██║
//  ╚══════╝ ╚═════╝ ╚═╝     ╚═╝╚═╝     ╚═════╝     ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝
//
//  sump3_axi_wrapper.sv - AXI4-Lite Wrapper for SUMP3 Integrated Logic Analyzer
//
//=============================================================================
//
// Copyright (c) 2024 - MIT License
//
//=============================================================================
//
// OVERVIEW
// ========
// This module provides an AXI4-Lite slave interface to the SUMP3 ILA core,
// implementing a hardware state machine that handles all SUMP3 protocol
// complexity. This design philosophy enables:
//
//   - Simple software drivers (write command, wait for IRQ, read result)
//   - Efficient Linux kernel drivers using wait_for_completion() instead of
//     busy-wait loops with udelay()
//   - Robust timing that works across different clock frequencies
//
// REGISTER MAP
// ============
// Offset | Name       | Access | Description
// -------|------------|--------|--------------------------------------------------
// 0x00   | CMD        | R/W    | Command code to execute (see COMMAND CODES below)
// 0x04   | ADDR       | R/W    | Target address: {hub[23:16], pod[15:8], reg[7:0]}
// 0x08   | WDATA      | R/W    | Write data for write operations
// 0x0C   | CTRL       | R/W    | Control: [0]=START, [1]=IRQ_EN, [2]=ABORT
// 0x10   | STATUS     | R      | Status: [0]=BUSY, [1]=DONE, [2]=ERROR, [3]=IRQ_PEND
// 0x14   | RDATA      | R      | Read result data from completed command
// 0x18   | IRQ_STATUS | R/W1C  | IRQ status (write 1 to bit 0 to clear)
// 0x1C   | HW_INFO    | R      | {ID[31:16], hub_count[15:8], revision[7:0]}
// 0x20   | CAP_STATUS | R      | Capture status from SUMP3 core
// 0x24   | TIMEOUT    | R/W    | Timeout value in clock cycles (16-bit)
//
// COMMAND CODES (Write to CMD Register)
// =====================================
//
// STATE COMMANDS (0x00-0x0F): Control ILA state machine
// -----------------------------------------------------
// 0x00 | NOP          | No operation (useful for testing)
// 0x01 | ARM          | Arm the ILA for capture
// 0x02 | RESET        | Reset the ILA
// 0x03 | INIT         | Initialize RAM (required before ARM)
// 0x04 | IDLE         | Return to idle state
// 0x05 | SLEEP        | Enter sleep mode (clock gating)
//
// LOCAL READS (0x10-0x1F): Read from SUMP3 core registers
// -------------------------------------------------------
// 0x10 | RD_HW_ID        | Read hardware ID and revision
// 0x11 | RD_HUB_COUNT    | Read number of RLE hubs
// 0x12 | RD_STATUS       | Read capture status (armed/triggered/acquired)
// 0x13 | RD_ANA_RAM_CFG  | Read analog RAM configuration
// 0x14 | RD_TICK_FREQ    | Read tick frequency
// 0x15 | RD_ANA_FIRST_PTR| Read analog first sample pointer
// 0x16 | RD_RAM_DATA     | Read RAM data (auto-increment)
// 0x17 | RD_DIG_FIRST_PTR| Read digital first sample pointer
// 0x18 | RD_DIG_CK_FREQ  | Read digital clock frequency
// 0x19 | RD_DIG_RAM_CFG  | Read digital RAM configuration
// 0x1A | RD_REC_PROFILE  | Read record profile
// 0x1B | RD_TRIG_SRC     | Read trigger source
// 0x1C | RD_VIEW_ROM_KB  | Read view ROM size in KB
//
// LOCAL WRITES (0x20-0x2F): Write to SUMP3 core configuration
// -----------------------------------------------------------
// 0x20 | WR_USER_CTRL     | Write user control bits
// 0x21 | WR_REC_CONFIG    | Write record configuration
// 0x22 | WR_TICK_DIVISOR  | Write tick clock divisor
// 0x23 | WR_TRIG_TYPE     | Write trigger type (OR/AND/rising/falling/etc)
// 0x24 | WR_TRIG_DIG_FIELD| Write digital trigger field (which bits)
// 0x25 | WR_TRIG_ANA_FIELD| Write analog trigger field (channel + threshold)
// 0x26 | WR_ANA_POST_TRIG | Write analog post-trigger sample count
// 0x27 | WR_TRIG_DELAY    | Write trigger delay
// 0x28 | WR_TRIG_NTH      | Write Nth trigger count
// 0x29 | WR_RAM_RD_PTR    | Write RAM read pointer
// 0x2A | WR_DIG_POST_TRIG | Write digital post-trigger sample count
// 0x2B | WR_RAM_PAGE      | Write RAM page select
//
// SERIAL BUS READS (0x30-0x3F): Read from hub/pod via serial bus
// --------------------------------------------------------------
// 0x30 | RD_HUB_FREQ      | Read hub clock frequency (set ADDR[23:16]=hub)
// 0x31 | RD_POD_COUNT     | Read pod count for hub (set ADDR[23:16]=hub)
// 0x32 | RD_POD_REG       | Read pod register (full ADDR required)
// 0x33 | RD_TRIG_SRC_POD  | Read trigger source pod
// 0x34 | RD_HUB_HW_CFG    | Read hub hardware configuration
// 0x35 | RD_HUB_INSTANCE  | Read hub instance number
// 0x36 | RD_HUB_NAME_0_3  | Read hub name bytes 0-3
// 0x37 | RD_HUB_NAME_4_7  | Read hub name bytes 4-7
// 0x38 | RD_HUB_NAME_8_11 | Read hub name bytes 8-11
//
// SERIAL BUS WRITES (0x40-0x4F): Write to hub/pod via serial bus
// --------------------------------------------------------------
// 0x40 | WR_POD_REG       | Write pod register (full ADDR + WDATA required)
// 0x41 | WR_TRIG_WIDTH    | Write trigger pulse width
//
//=============================================================================

module sump3_axi_wrapper #(
    parameter int C_S_AXI_DATA_WIDTH = 32,
    parameter int C_S_AXI_ADDR_WIDTH = 8,
    parameter int DEFAULT_TIMEOUT    = 8192  // Clock cycles before timeout
) (
    //=========================================================================
    // AXI4-Lite Slave Interface
    //=========================================================================
    input  logic                              s_axi_aclk,     // AXI clock
    input  logic                              s_axi_aresetn,  // AXI reset (active low)
    
    // Write Address Channel
    input  logic [C_S_AXI_ADDR_WIDTH-1:0]     s_axi_awaddr,
    input  logic [2:0]                        s_axi_awprot,
    input  logic                              s_axi_awvalid,
    output logic                              s_axi_awready,
    
    // Write Data Channel
    input  logic [C_S_AXI_DATA_WIDTH-1:0]     s_axi_wdata,
    input  logic [(C_S_AXI_DATA_WIDTH/8)-1:0] s_axi_wstrb,
    input  logic                              s_axi_wvalid,
    output logic                              s_axi_wready,
    
    // Write Response Channel
    output logic [1:0]                        s_axi_bresp,
    output logic                              s_axi_bvalid,
    input  logic                              s_axi_bready,
    
    // Read Address Channel
    input  logic [C_S_AXI_ADDR_WIDTH-1:0]     s_axi_araddr,
    input  logic [2:0]                        s_axi_arprot,
    input  logic                              s_axi_arvalid,
    output logic                              s_axi_arready,
    
    // Read Data Channel
    output logic [C_S_AXI_DATA_WIDTH-1:0]     s_axi_rdata,
    output logic [1:0]                        s_axi_rresp,
    output logic                              s_axi_rvalid,
    input  logic                              s_axi_rready,
    
    //=========================================================================
    // Interrupt Output (directly to CPU/GIC)
    //=========================================================================
    output logic                              irq,
    
    //=========================================================================
    // SUMP3 Local Bus Interface (directly to sump3_core)
    //=========================================================================
    output logic                              lb_cs_ctrl,   // Chip select for CTRL registers
    output logic                              lb_cs_data,   // Chip select for DATA registers
    output logic                              lb_wr,        // Write strobe
    output logic                              lb_rd,        // Read strobe
    output logic [31:0]                       lb_wr_d,      // Write data
    input  logic [31:0]                       lb_rd_d,      // Read data from core
    input  logic                              lb_rd_rdy,    // Read data ready pulse
    
    //=========================================================================
    // SUMP3 Status Inputs (directly from sump3_core)
    //=========================================================================
    input  logic                              sump_is_armed,  // ILA is armed for capture
    input  logic                              sump_is_awake,  // ILA is awake (not in sleep)
    input  logic [7:0]                        sump_hub_count  // Number of hubs (from parameter)
);

    //=========================================================================
    // AXI Register Addresses
    //=========================================================================
    localparam logic [7:0] REG_CMD        = 8'h00;  // Command register
    localparam logic [7:0] REG_ADDR       = 8'h04;  // Address register
    localparam logic [7:0] REG_WDATA      = 8'h08;  // Write data register
    localparam logic [7:0] REG_CTRL       = 8'h0C;  // Control register
    localparam logic [7:0] REG_STATUS     = 8'h10;  // Status register (read-only)
    localparam logic [7:0] REG_RDATA      = 8'h14;  // Read data result (read-only)
    localparam logic [7:0] REG_IRQ_STATUS = 8'h18;  // IRQ status (write-1-to-clear)
    localparam logic [7:0] REG_HW_INFO    = 8'h1C;  // Hardware info (read-only)
    localparam logic [7:0] REG_CAP_STATUS = 8'h20;  // Capture status (read-only)
    localparam logic [7:0] REG_TIMEOUT    = 8'h24;  // Timeout value

    //=========================================================================
    // Wrapper Command Codes - STATE COMMANDS (0x00-0x0F)
    //=========================================================================
    localparam logic [7:0] CMD_NOP          = 8'h00;  // No operation
    localparam logic [7:0] CMD_ARM          = 8'h01;  // Arm the ILA
    localparam logic [7:0] CMD_RESET        = 8'h02;  // Reset the ILA
    localparam logic [7:0] CMD_INIT         = 8'h03;  // Initialize RAM
    localparam logic [7:0] CMD_IDLE         = 8'h04;  // Return to idle
    localparam logic [7:0] CMD_SLEEP        = 8'h05;  // Enter sleep mode

    //=========================================================================
    // Wrapper Command Codes - LOCAL READS (0x10-0x1F)
    //=========================================================================
    localparam logic [7:0] CMD_RD_HW_ID        = 8'h10;  // Read hardware ID
    localparam logic [7:0] CMD_RD_HUB_COUNT    = 8'h11;  // Read hub count
    localparam logic [7:0] CMD_RD_STATUS       = 8'h12;  // Read capture status
    localparam logic [7:0] CMD_RD_ANA_RAM_CFG  = 8'h13;  // Read analog RAM config
    localparam logic [7:0] CMD_RD_TICK_FREQ    = 8'h14;  // Read tick frequency
    localparam logic [7:0] CMD_RD_ANA_FIRST_PTR= 8'h15;  // Read analog first pointer
    localparam logic [7:0] CMD_RD_RAM_DATA     = 8'h16;  // Read RAM data
    localparam logic [7:0] CMD_RD_DIG_FIRST_PTR= 8'h17;  // Read digital first pointer
    localparam logic [7:0] CMD_RD_DIG_CK_FREQ  = 8'h18;  // Read digital clock frequency
    localparam logic [7:0] CMD_RD_DIG_RAM_CFG  = 8'h19;  // Read digital RAM config
    localparam logic [7:0] CMD_RD_REC_PROFILE  = 8'h1A;  // Read record profile
    localparam logic [7:0] CMD_RD_TRIG_SRC     = 8'h1B;  // Read trigger source
    localparam logic [7:0] CMD_RD_VIEW_ROM_KB  = 8'h1C;  // Read view ROM size

    //=========================================================================
    // Wrapper Command Codes - LOCAL WRITES (0x20-0x2F)
    //=========================================================================
    localparam logic [7:0] CMD_WR_USER_CTRL     = 8'h20;  // Write user control
    localparam logic [7:0] CMD_WR_REC_CONFIG    = 8'h21;  // Write record config
    localparam logic [7:0] CMD_WR_TICK_DIVISOR  = 8'h22;  // Write tick divisor
    localparam logic [7:0] CMD_WR_TRIG_TYPE     = 8'h23;  // Write trigger type
    localparam logic [7:0] CMD_WR_TRIG_DIG_FIELD= 8'h24;  // Write digital trigger field
    localparam logic [7:0] CMD_WR_TRIG_ANA_FIELD= 8'h25;  // Write analog trigger field
    localparam logic [7:0] CMD_WR_ANA_POST_TRIG = 8'h26;  // Write analog post-trigger
    localparam logic [7:0] CMD_WR_TRIG_DELAY    = 8'h27;  // Write trigger delay
    localparam logic [7:0] CMD_WR_TRIG_NTH      = 8'h28;  // Write Nth trigger
    localparam logic [7:0] CMD_WR_RAM_RD_PTR    = 8'h29;  // Write RAM read pointer
    localparam logic [7:0] CMD_WR_DIG_POST_TRIG = 8'h2A;  // Write digital post-trigger
    localparam logic [7:0] CMD_WR_RAM_PAGE      = 8'h2B;  // Write RAM page select

    //=========================================================================
    // Wrapper Command Codes - SERIAL BUS READS (0x30-0x3F)
    //=========================================================================
    localparam logic [7:0] CMD_RD_HUB_FREQ     = 8'h30;  // Read hub frequency
    localparam logic [7:0] CMD_RD_POD_COUNT    = 8'h31;  // Read pod count
    localparam logic [7:0] CMD_RD_POD_REG      = 8'h32;  // Read pod register
    localparam logic [7:0] CMD_RD_TRIG_SRC_POD = 8'h33;  // Read trigger source pod
    localparam logic [7:0] CMD_RD_HUB_HW_CFG   = 8'h34;  // Read hub HW config
    localparam logic [7:0] CMD_RD_HUB_INSTANCE = 8'h35;  // Read hub instance
    localparam logic [7:0] CMD_RD_HUB_NAME_0_3 = 8'h36;  // Read hub name 0-3
    localparam logic [7:0] CMD_RD_HUB_NAME_4_7 = 8'h37;  // Read hub name 4-7
    localparam logic [7:0] CMD_RD_HUB_NAME_8_11= 8'h38;  // Read hub name 8-11

    //=========================================================================
    // Wrapper Command Codes - SERIAL BUS WRITES (0x40-0x4F)
    //=========================================================================
    localparam logic [7:0] CMD_WR_POD_REG      = 8'h40;  // Write pod register
    localparam logic [7:0] CMD_WR_TRIG_WIDTH   = 8'h41;  // Write trigger width

    //=========================================================================
    // SUMP3 Internal Command Codes (sent via local bus to core)
    // Reference: sump3_core.v local bus command decoder
    //=========================================================================
    localparam logic [5:0] SUMP_CMD_IDLE          = 6'h00;
    localparam logic [5:0] SUMP_CMD_ARM           = 6'h01;
    localparam logic [5:0] SUMP_CMD_RESET         = 6'h02;
    localparam logic [5:0] SUMP_CMD_INIT          = 6'h03;
    localparam logic [5:0] SUMP_CMD_SLEEP         = 6'h04;
    localparam logic [5:0] SUMP_CMD_RD_HW_ID      = 6'h0B;
    localparam logic [5:0] SUMP_CMD_RD_ANA_RAM_CFG= 6'h0C;
    localparam logic [5:0] SUMP_CMD_RD_TICK_FREQ  = 6'h0D;
    localparam logic [5:0] SUMP_CMD_RD_ANA_FIRST  = 6'h0E;
    localparam logic [5:0] SUMP_CMD_RD_RAM_DATA   = 6'h0F;
    localparam logic [5:0] SUMP_CMD_RD_DIG_FIRST  = 6'h10;
    localparam logic [5:0] SUMP_CMD_RD_DIG_CK_FREQ= 6'h11;
    localparam logic [5:0] SUMP_CMD_RD_DIG_RAM_CFG= 6'h12;
    localparam logic [5:0] SUMP_CMD_RD_REC_PROFILE= 6'h13;
    localparam logic [5:0] SUMP_CMD_RD_TRIG_SRC   = 6'h14;
    localparam logic [5:0] SUMP_CMD_RD_VIEW_ROM   = 6'h15;
    localparam logic [5:0] SUMP_CMD_WR_USER_CTRL  = 6'h20;
    localparam logic [5:0] SUMP_CMD_WR_REC_CONFIG = 6'h21;
    localparam logic [5:0] SUMP_CMD_WR_TICK_DIV   = 6'h22;
    localparam logic [5:0] SUMP_CMD_WR_TRIG_TYPE  = 6'h23;
    localparam logic [5:0] SUMP_CMD_WR_TRIG_DIG   = 6'h24;
    localparam logic [5:0] SUMP_CMD_WR_TRIG_ANA   = 6'h25;
    localparam logic [5:0] SUMP_CMD_WR_ANA_POST   = 6'h26;
    localparam logic [5:0] SUMP_CMD_WR_TRIG_DELAY = 6'h27;
    localparam logic [5:0] SUMP_CMD_WR_TRIG_NTH   = 6'h28;
    localparam logic [5:0] SUMP_CMD_WR_RAM_RD_PTR = 6'h29;
    localparam logic [5:0] SUMP_CMD_WR_DIG_POST   = 6'h2A;
    localparam logic [5:0] SUMP_CMD_WR_RAM_PAGE   = 6'h2B;
    localparam logic [5:0] SUMP_CMD_RD_HUB_NUM    = 6'h30;
    localparam logic [5:0] SUMP_CMD_RD_POD_NUM    = 6'h31;
    localparam logic [5:0] SUMP_CMD_WR_INST_ADDR  = 6'h32;
    localparam logic [5:0] SUMP_CMD_RW_POD_DATA   = 6'h33;
    localparam logic [5:0] SUMP_CMD_RD_TRIG_POD   = 6'h34;
    localparam logic [5:0] SUMP_CMD_WR_TRIG_WIDTH = 6'h35;
    localparam logic [5:0] SUMP_CMD_RD_HUB_FREQ   = 6'h36;
    localparam logic [5:0] SUMP_CMD_RD_HUB_HW_CFG = 6'h3A;
    localparam logic [5:0] SUMP_CMD_RD_HUB_INST   = 6'h3C;
    localparam logic [5:0] SUMP_CMD_RD_HUB_NAME0  = 6'h3D;
    localparam logic [5:0] SUMP_CMD_RD_HUB_NAME1  = 6'h3E;
    localparam logic [5:0] SUMP_CMD_RD_HUB_NAME2  = 6'h3F;

    //=========================================================================
    // Control Register Bit Positions
    //=========================================================================
    localparam int CTRL_START   = 0;  // Start command execution
    localparam int CTRL_IRQ_EN  = 1;  // Enable IRQ on completion
    localparam int CTRL_ABORT   = 2;  // Abort current operation

    //=========================================================================
    // Software-Accessible Registers
    //=========================================================================
    logic [7:0]  reg_cmd;       // Command to execute
    logic [31:0] reg_addr;      // Target address (hub/pod/reg)
    logic [31:0] reg_wdata;     // Write data for write commands
    logic [2:0]  reg_ctrl;      // Control bits
    logic [31:0] reg_rdata;     // Result data from completed read
    logic [15:0] reg_timeout;   // Timeout value in clock cycles
    
    // Status bits (directly visible via STATUS register)
    logic        status_busy;   // Command in progress
    logic        status_done;   // Command completed successfully
    logic        status_error;  // Command failed (timeout)
    logic        irq_pending;   // IRQ is pending

    //=========================================================================
    // Command Execution State Machine
    //=========================================================================
    typedef enum logic [4:0] {
        ST_IDLE,                // Waiting for command
        
        // Simple commands (CTRL write only, no response)
        ST_SIMPLE_CMD,          // Send command to local bus
        ST_SIMPLE_HOLD,         // Hold chip select for timing
        
        // Local reads (CTRL write, DATA read)
        ST_LOCAL_CMD,           // Send read command
        ST_LOCAL_HOLD,          // Hold chip select
        ST_LOCAL_READ,          // Issue read strobe
        ST_LOCAL_WAIT,          // Wait for lb_rd_rdy
        
        // Local writes (CTRL write, DATA write)
        ST_LOCAL_WR_CMD,        // Send write command to CTRL
        ST_LOCAL_WR_HOLD,       // Hold CTRL chip select
        ST_LOCAL_WR_DATA,       // Write data to DATA register
        ST_LOCAL_WR_DATA_HOLD,  // Hold DATA chip select
        
        // Serial bus reads (multi-step via serial bus)
        ST_INST_ADDR_CMD,       // Send INST_ADDR command (0x32)
        ST_INST_ADDR_HOLD,      // Wait for serial TX complete
        ST_INST_ADDR_DATA,      // Send address data
        ST_INST_ADDR_DATA_HOLD, // Wait for address to propagate
        ST_TARGET_CMD,          // Send target command (0x33/0x36/etc)
        ST_TARGET_CMD_HOLD,     // Wait for serial TX complete
        ST_TARGET_TRIGGER,      // Trigger read (returns stale data)
        ST_TARGET_TRIGGER_HOLD, // Brief hold
        ST_TARGET_SERIAL_WAIT,  // CRITICAL: Wait for pod response
        ST_TARGET_READ,         // Second read (returns fresh data)
        ST_TARGET_READ_WAIT,    // Wait for lb_rd_rdy
        
        // Serial bus writes (multi-step via serial bus)
        ST_TARGET_WRITE,        // Write data to pod register
        ST_TARGET_WRITE_HOLD,   // Hold after write
        
        // Completion states
        ST_DONE,                // Command successful
        ST_ERROR                // Command failed (timeout)
    } state_t;
    
    state_t state, state_next;
    
    //=========================================================================
    // Timing Control
    //=========================================================================
    logic [15:0] wait_counter;     // Minimum hold time counter
    logic        wait_done;        // wait_counter reached zero
    logic [15:0] timeout_counter;  // Timeout countdown
    logic        timeout_occurred; // Timeout reached
    logic        capture_enable;   // Gate for capturing read data
    
    //=========================================================================
    // Command Classification Helpers
    //=========================================================================
    logic is_state_cmd;      // ARM, RESET, INIT, IDLE, SLEEP
    logic is_local_read;     // Local bus read (no serial)
    logic is_local_write;    // Local bus write (CTRL+DATA)
    logic is_serial_read;    // Serial bus read (hub/pod)
    logic is_serial_write;   // Serial bus write (hub/pod)
    logic is_hub_level_read; // Hub-level read (uses fixed delay)
    
    always_comb begin
        is_state_cmd = (reg_cmd == CMD_ARM) || (reg_cmd == CMD_RESET) || 
                       (reg_cmd == CMD_INIT) || (reg_cmd == CMD_IDLE) || 
                       (reg_cmd == CMD_SLEEP);
        
        is_local_read = (reg_cmd >= CMD_RD_HW_ID) && (reg_cmd <= CMD_RD_VIEW_ROM_KB);
        
        is_local_write = (reg_cmd >= CMD_WR_USER_CTRL) && (reg_cmd <= CMD_WR_RAM_PAGE);
        
        is_serial_read = (reg_cmd >= CMD_RD_HUB_FREQ) && (reg_cmd <= CMD_RD_HUB_NAME_8_11);
        
        is_serial_write = (reg_cmd == CMD_WR_POD_REG) || (reg_cmd == CMD_WR_TRIG_WIDTH);
        
        // Hub-level reads have CDC timing issues when clk_cap==clk_lb
        is_hub_level_read = (reg_cmd == CMD_RD_HUB_FREQ) || (reg_cmd == CMD_RD_POD_COUNT) ||
                           (reg_cmd == CMD_RD_TRIG_SRC_POD) || (reg_cmd == CMD_RD_HUB_HW_CFG) ||
                           (reg_cmd == CMD_RD_HUB_INSTANCE) || (reg_cmd == CMD_RD_HUB_NAME_0_3) ||
                           (reg_cmd == CMD_RD_HUB_NAME_4_7) || (reg_cmd == CMD_RD_HUB_NAME_8_11);
    end
    
    //=========================================================================
    // Local Bus Control Registers
    //=========================================================================
    logic        lb_cs_ctrl_reg;
    logic        lb_cs_data_reg;
    logic        lb_wr_reg;
    logic        lb_rd_reg;
    logic [31:0] lb_wr_d_reg;

    //=========================================================================
    // AXI Write Channel State Machine
    //=========================================================================
    typedef enum logic [1:0] {
        AXI_WR_IDLE,  // Ready for new write
        AXI_WR_DATA,  // Have address, waiting for data
        AXI_WR_RESP   // Sending write response
    } axi_wr_state_t;
    
    axi_wr_state_t axi_wr_state;
    logic [7:0]  axi_wr_addr_reg;
    logic [31:0] axi_wr_data_reg;
    logic        axi_wr_pulse;  // One-cycle pulse when write completes
    
    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            axi_wr_state  <= AXI_WR_IDLE;
            s_axi_awready <= 1'b1;
            s_axi_wready  <= 1'b1;
            s_axi_bvalid  <= 1'b0;
            axi_wr_pulse  <= 1'b0;
        end else begin
            axi_wr_pulse <= 1'b0;
            
            case (axi_wr_state)
                AXI_WR_IDLE: begin
                    if (s_axi_awvalid && s_axi_wvalid) begin
                        axi_wr_addr_reg <= s_axi_awaddr;
                        axi_wr_data_reg <= s_axi_wdata;
                        axi_wr_pulse    <= 1'b1;
                        s_axi_awready   <= 1'b0;
                        s_axi_wready    <= 1'b0;
                        axi_wr_state    <= AXI_WR_RESP;
                    end else if (s_axi_awvalid) begin
                        axi_wr_addr_reg <= s_axi_awaddr;
                        s_axi_awready   <= 1'b0;
                        axi_wr_state    <= AXI_WR_DATA;
                    end
                end
                
                AXI_WR_DATA: begin
                    if (s_axi_wvalid) begin
                        axi_wr_data_reg <= s_axi_wdata;
                        axi_wr_pulse    <= 1'b1;
                        s_axi_wready    <= 1'b0;
                        axi_wr_state    <= AXI_WR_RESP;
                    end
                end
                
                AXI_WR_RESP: begin
                    s_axi_bvalid <= 1'b1;
                    if (s_axi_bvalid && s_axi_bready) begin
                        s_axi_bvalid  <= 1'b0;
                        s_axi_awready <= 1'b1;
                        s_axi_wready  <= 1'b1;
                        axi_wr_state  <= AXI_WR_IDLE;
                    end
                end
            endcase
        end
    end
    
    assign s_axi_bresp = 2'b00;  // OKAY response

    //=========================================================================
    // Register Write Logic
    //=========================================================================
    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            reg_cmd     <= CMD_NOP;
            reg_addr    <= 32'h0;
            reg_wdata   <= 32'h0;
            reg_ctrl    <= 3'h0;
            reg_timeout <= DEFAULT_TIMEOUT[15:0];
            irq_pending <= 1'b0;
        end else begin
            // Auto-clear START bit once command begins
            if (state != ST_IDLE)
                reg_ctrl[CTRL_START] <= 1'b0;
            
            // Clear IRQ when written with 1 (write-1-to-clear)
            if (axi_wr_pulse && axi_wr_addr_reg == REG_IRQ_STATUS)
                irq_pending <= irq_pending & ~axi_wr_data_reg[0];
            
            // Set IRQ on command completion if enabled
            if ((state == ST_DONE || state == ST_ERROR) && reg_ctrl[CTRL_IRQ_EN])
                irq_pending <= 1'b1;
            
            // Process register writes
            if (axi_wr_pulse) begin
                case (axi_wr_addr_reg)
                    REG_CMD:     reg_cmd     <= axi_wr_data_reg[7:0];
                    REG_ADDR:    reg_addr    <= axi_wr_data_reg;
                    REG_WDATA:   reg_wdata   <= axi_wr_data_reg;
                    REG_CTRL:    reg_ctrl    <= axi_wr_data_reg[2:0];
                    REG_TIMEOUT: reg_timeout <= axi_wr_data_reg[15:0];
                endcase
            end
        end
    end

    //=========================================================================
    // AXI Read Logic
    //=========================================================================
    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_arready <= 1'b1;
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= 32'h0;
        end else begin
            if (s_axi_arvalid && s_axi_arready) begin
                s_axi_arready <= 1'b0;
                s_axi_rvalid  <= 1'b1;
                
                case (s_axi_araddr)
                    REG_CMD:        s_axi_rdata <= {24'h0, reg_cmd};
                    REG_ADDR:       s_axi_rdata <= reg_addr;
                    REG_WDATA:      s_axi_rdata <= reg_wdata;
                    REG_CTRL:       s_axi_rdata <= {29'h0, reg_ctrl};
                    REG_STATUS:     s_axi_rdata <= {28'h0, irq_pending, status_error, 
                                                    status_done, status_busy};
                    REG_RDATA:      s_axi_rdata <= reg_rdata;
                    REG_IRQ_STATUS: s_axi_rdata <= {31'h0, irq_pending};
                    REG_HW_INFO:    s_axi_rdata <= {16'h5303, sump_hub_count, 8'h01};
                    REG_CAP_STATUS: s_axi_rdata <= {30'h0, sump_is_awake, sump_is_armed};
                    REG_TIMEOUT:    s_axi_rdata <= {16'h0, reg_timeout};
                    default:        s_axi_rdata <= 32'hDEADCAFE;  // Unmapped address
                endcase
            end
            
            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid  <= 1'b0;
                s_axi_arready <= 1'b1;
            end
        end
    end
    
    assign s_axi_rresp = 2'b00;  // OKAY response

    // Note: All serial operations use fixed delays rather than monitoring internal signals.
    // This avoids CDC issues and matches how MesaBus/Python interfaces with SUMP3.
    
    //=========================================================================
    // Wait Counter
    //=========================================================================
    assign wait_done = (wait_counter == 0);
    
    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            wait_counter <= 16'h0;
        end else begin
            if (wait_counter != 0)
                wait_counter <= wait_counter - 1;
            
            // Load wait time on state transition
            // Wait times are sized for slow capture domains (e.g., 50 MHz with 100 MHz bus)
            // Serial ops need: ~40 clk_lb + ~90 clk_cap each way
            // For 2x slower clk_cap: 40 + 180 + 40 + 180 = 440 cycles minimum
            if (state != state_next) begin
                // Wait times sized for slow capture domains (e.g., 50 MHz with 100 MHz bus)
                // Serial shift register is 39 bits; at 2:1 clock ratio = ~80 bus cycles
                // Adding margin for CDC and processing time
                case (state_next)
                    ST_SIMPLE_HOLD:         wait_counter <= 16'd8;    // CS hold time
                    ST_LOCAL_HOLD:          wait_counter <= 16'd8;    // CS hold time
                    ST_LOCAL_WAIT:          wait_counter <= 16'd20;   // Min before lb_rd_rdy check
                    ST_LOCAL_WR_HOLD:       wait_counter <= 16'd8;    // CS hold time
                    ST_LOCAL_WR_DATA_HOLD:  wait_counter <= 16'd8;    // CS hold time
                    ST_INST_ADDR_HOLD:      wait_counter <= 16'd100;  // Wait for cmd to shift out
                    ST_INST_ADDR_DATA_HOLD: wait_counter <= 16'd200;  // Wait for addr to shift to hub
                    ST_TARGET_CMD_HOLD:     wait_counter <= 16'd100;  // Wait for cmd to shift out
                    ST_TARGET_TRIGGER_HOLD: wait_counter <= 16'd20;   // Brief hold
                    ST_TARGET_SERIAL_WAIT:  wait_counter <= 16'd600;  // Wait for serial round-trip
                    ST_TARGET_READ_WAIT:    wait_counter <= 16'd20;   // Min before lb_rd_rdy check
                    ST_TARGET_WRITE_HOLD:   wait_counter <= 16'd400;  // Wait for write to complete
                    default:                wait_counter <= 16'd0;
                endcase
            end
        end
    end

    //=========================================================================
    // Timeout Counter
    //=========================================================================
    assign timeout_occurred = (timeout_counter == 0);
    
    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            timeout_counter <= 16'h0;
        end else begin
            if (state == ST_IDLE)
                timeout_counter <= reg_timeout;
            else if (timeout_counter != 0)
                timeout_counter <= timeout_counter - 1;
        end
    end

    //=========================================================================
    // Status Flags
    //=========================================================================
    assign status_busy = (state != ST_IDLE);
    
    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            status_done  <= 1'b0;
            status_error <= 1'b0;
        end else begin
            // Clear status on new command
            if (state == ST_IDLE && reg_ctrl[CTRL_START]) begin
                status_done  <= 1'b0;
                status_error <= 1'b0;
            end
            // Set status on completion
            if (state == ST_DONE)
                status_done <= 1'b1;
            if (state == ST_ERROR)
                status_error <= 1'b1;
        end
    end
    
    assign irq = irq_pending;

    //=========================================================================
    // Capture Enable - only during legitimate final read states
    //=========================================================================
    always_comb begin
        case (state)
            ST_LOCAL_READ, ST_LOCAL_WAIT:        capture_enable = 1'b1;
            ST_TARGET_READ, ST_TARGET_READ_WAIT: capture_enable = 1'b1;
            default:                              capture_enable = 1'b0;
        endcase
    end
    
    //=========================================================================
    // Capture Read Data
    //=========================================================================
    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            reg_rdata <= 32'h0;
        end else if (lb_rd_rdy && capture_enable) begin
            reg_rdata <= lb_rd_d;
        end
    end

    //=========================================================================
    // Command State Machine - Sequential
    //=========================================================================
    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn)
            state <= ST_IDLE;
        else
            state <= state_next;
    end

    //=========================================================================
    // Command State Machine - Combinatorial Next State
    //=========================================================================
    always_comb begin
        state_next = state;
        
        case (state)
            //=================================================================
            // IDLE: Waiting for software to set CTRL[START]
            //=================================================================
            ST_IDLE: begin
                if (reg_ctrl[CTRL_START]) begin
                    case (1'b1)
                        (reg_cmd == CMD_NOP):    state_next = ST_DONE;
                        is_state_cmd:            state_next = ST_SIMPLE_CMD;
                        is_local_read:           state_next = ST_LOCAL_CMD;
                        is_local_write:          state_next = ST_LOCAL_WR_CMD;
                        is_serial_read:          state_next = ST_INST_ADDR_CMD;
                        is_serial_write:         state_next = ST_INST_ADDR_CMD;
                        default:                 state_next = ST_ERROR;
                    endcase
                end
            end
            
            //=================================================================
            // Simple Commands (ARM, RESET, INIT, IDLE, SLEEP)
            //=================================================================
            ST_SIMPLE_CMD:  state_next = ST_SIMPLE_HOLD;
            ST_SIMPLE_HOLD: state_next = wait_done ? ST_DONE : ST_SIMPLE_HOLD;
            
            //=================================================================
            // Local Reads (HW_ID, STATUS, RAM configs, etc)
            //=================================================================
            ST_LOCAL_CMD:  state_next = ST_LOCAL_HOLD;
            ST_LOCAL_HOLD: state_next = wait_done ? ST_LOCAL_READ : ST_LOCAL_HOLD;
            ST_LOCAL_READ: state_next = ST_LOCAL_WAIT;
            ST_LOCAL_WAIT: begin
                if (lb_rd_rdy)
                    state_next = ST_DONE;
                else if (wait_done && timeout_occurred)
                    state_next = ST_ERROR;
            end
            
            //=================================================================
            // Local Writes (trigger config, post-trig, etc)
            //=================================================================
            ST_LOCAL_WR_CMD:       state_next = ST_LOCAL_WR_HOLD;
            ST_LOCAL_WR_HOLD:      state_next = wait_done ? ST_LOCAL_WR_DATA : ST_LOCAL_WR_HOLD;
            ST_LOCAL_WR_DATA:      state_next = ST_LOCAL_WR_DATA_HOLD;
            ST_LOCAL_WR_DATA_HOLD: state_next = wait_done ? ST_DONE : ST_LOCAL_WR_DATA_HOLD;
            
            //=================================================================
            // Serial Bus Operations - Address phase
            // Uses fixed delays instead of monitoring internal SR signals
            // This avoids CDC issues and matches MesaBus/Python approach
            //=================================================================
            ST_INST_ADDR_CMD:  state_next = ST_INST_ADDR_HOLD;
            ST_INST_ADDR_HOLD: begin
                // Wait for command to shift out (fixed delay)
                if (wait_done)
                    state_next = ST_INST_ADDR_DATA;
                else if (timeout_occurred)
                    state_next = ST_ERROR;
            end
            
            ST_INST_ADDR_DATA:      state_next = ST_INST_ADDR_DATA_HOLD;
            ST_INST_ADDR_DATA_HOLD: begin
                // Wait for address to shift out to hub (fixed delay)
                if (wait_done)
                    state_next = ST_TARGET_CMD;
                else if (timeout_occurred)
                    state_next = ST_ERROR;
            end
            
            //=================================================================
            // Serial Bus Operations - Target command
            //=================================================================
            ST_TARGET_CMD:      state_next = ST_TARGET_CMD_HOLD;
            ST_TARGET_CMD_HOLD: begin
                // Wait for command to shift out (fixed delay)
                if (wait_done) begin
                    // Branch based on read vs write
                    if (is_serial_write)
                        state_next = ST_TARGET_WRITE;
                    else
                        state_next = ST_TARGET_TRIGGER;
                end else if (timeout_occurred)
                    state_next = ST_ERROR;
            end
            
            //=================================================================
            // Serial Bus Read - Trigger and wait for response
            //=================================================================
            ST_TARGET_TRIGGER:      state_next = ST_TARGET_TRIGGER_HOLD;
            ST_TARGET_TRIGGER_HOLD: begin
                if (wait_done)
                    state_next = ST_TARGET_SERIAL_WAIT;
            end
            
            ST_TARGET_SERIAL_WAIT: begin
                // All serial reads use fixed delay - long enough for slow clock domains
                // This matches how MesaBus/Python interfaces with SUMP3
                if (wait_done)
                    state_next = ST_TARGET_READ;
                else if (timeout_occurred)
                    state_next = ST_ERROR;
            end
            
            ST_TARGET_READ:      state_next = ST_TARGET_READ_WAIT;
            ST_TARGET_READ_WAIT: begin
                if (lb_rd_rdy)
                    state_next = ST_DONE;
                else if (timeout_occurred)
                    state_next = ST_ERROR;
            end
            
            //=================================================================
            // Serial Bus Write - Write data and wait
            //=================================================================
            ST_TARGET_WRITE:      state_next = ST_TARGET_WRITE_HOLD;
            ST_TARGET_WRITE_HOLD: begin
                // Wait for write data to shift out (fixed delay)
                if (wait_done)
                    state_next = ST_DONE;
                else if (timeout_occurred)
                    state_next = ST_ERROR;
            end
            
            //=================================================================
            // Completion States
            //=================================================================
            ST_DONE:  state_next = ST_IDLE;
            ST_ERROR: state_next = ST_IDLE;
            
            default: state_next = ST_IDLE;
        endcase
        
        // ABORT: Immediately return to IDLE from any state
        if (reg_ctrl[CTRL_ABORT] && state != ST_IDLE)
            state_next = ST_IDLE;
    end

    //=========================================================================
    // Local Bus Signal Generation
    //=========================================================================
    
    // Helper function to get SUMP3 command code
    function automatic logic [5:0] get_sump_cmd(input logic [7:0] cmd);
        case (cmd)
            // State commands
            CMD_ARM:             return SUMP_CMD_ARM;
            CMD_RESET:           return SUMP_CMD_RESET;
            CMD_INIT:            return SUMP_CMD_INIT;
            CMD_IDLE:            return SUMP_CMD_IDLE;
            CMD_SLEEP:           return SUMP_CMD_SLEEP;
            // Local reads
            CMD_RD_HW_ID:        return SUMP_CMD_RD_HW_ID;
            CMD_RD_HUB_COUNT:    return SUMP_CMD_RD_HUB_NUM;
            CMD_RD_STATUS:       return SUMP_CMD_IDLE;  // Status from IDLE command
            CMD_RD_ANA_RAM_CFG:  return SUMP_CMD_RD_ANA_RAM_CFG;
            CMD_RD_TICK_FREQ:    return SUMP_CMD_RD_TICK_FREQ;
            CMD_RD_ANA_FIRST_PTR:return SUMP_CMD_RD_ANA_FIRST;
            CMD_RD_RAM_DATA:     return SUMP_CMD_RD_RAM_DATA;
            CMD_RD_DIG_FIRST_PTR:return SUMP_CMD_RD_DIG_FIRST;
            CMD_RD_DIG_CK_FREQ:  return SUMP_CMD_RD_DIG_CK_FREQ;
            CMD_RD_DIG_RAM_CFG:  return SUMP_CMD_RD_DIG_RAM_CFG;
            CMD_RD_REC_PROFILE:  return SUMP_CMD_RD_REC_PROFILE;
            CMD_RD_TRIG_SRC:     return SUMP_CMD_RD_TRIG_SRC;
            CMD_RD_VIEW_ROM_KB:  return SUMP_CMD_RD_VIEW_ROM;
            // Local writes
            CMD_WR_USER_CTRL:    return SUMP_CMD_WR_USER_CTRL;
            CMD_WR_REC_CONFIG:   return SUMP_CMD_WR_REC_CONFIG;
            CMD_WR_TICK_DIVISOR: return SUMP_CMD_WR_TICK_DIV;
            CMD_WR_TRIG_TYPE:    return SUMP_CMD_WR_TRIG_TYPE;
            CMD_WR_TRIG_DIG_FIELD:return SUMP_CMD_WR_TRIG_DIG;
            CMD_WR_TRIG_ANA_FIELD:return SUMP_CMD_WR_TRIG_ANA;
            CMD_WR_ANA_POST_TRIG:return SUMP_CMD_WR_ANA_POST;
            CMD_WR_TRIG_DELAY:   return SUMP_CMD_WR_TRIG_DELAY;
            CMD_WR_TRIG_NTH:     return SUMP_CMD_WR_TRIG_NTH;
            CMD_WR_RAM_RD_PTR:   return SUMP_CMD_WR_RAM_RD_PTR;
            CMD_WR_DIG_POST_TRIG:return SUMP_CMD_WR_DIG_POST;
            CMD_WR_RAM_PAGE:     return SUMP_CMD_WR_RAM_PAGE;
            // Serial reads - target command
            CMD_RD_HUB_FREQ:     return SUMP_CMD_RD_HUB_FREQ;
            CMD_RD_POD_COUNT:    return SUMP_CMD_RD_POD_NUM;
            CMD_RD_POD_REG:      return SUMP_CMD_RW_POD_DATA;
            CMD_RD_TRIG_SRC_POD: return SUMP_CMD_RD_TRIG_POD;
            CMD_RD_HUB_HW_CFG:   return SUMP_CMD_RD_HUB_HW_CFG;
            CMD_RD_HUB_INSTANCE: return SUMP_CMD_RD_HUB_INST;
            CMD_RD_HUB_NAME_0_3: return SUMP_CMD_RD_HUB_NAME0;
            CMD_RD_HUB_NAME_4_7: return SUMP_CMD_RD_HUB_NAME1;
            CMD_RD_HUB_NAME_8_11:return SUMP_CMD_RD_HUB_NAME2;
            // Serial writes
            CMD_WR_POD_REG:      return SUMP_CMD_RW_POD_DATA;
            CMD_WR_TRIG_WIDTH:   return SUMP_CMD_WR_TRIG_WIDTH;
            default:             return SUMP_CMD_IDLE;
        endcase
    endfunction
    
    always_ff @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            lb_cs_ctrl_reg <= 1'b0;
            lb_cs_data_reg <= 1'b0;
            lb_wr_reg      <= 1'b0;
            lb_rd_reg      <= 1'b0;
            lb_wr_d_reg    <= 32'h0;
        end else begin
            // Default: deassert strobes each cycle
            lb_wr_reg <= 1'b0;
            lb_rd_reg <= 1'b0;
            
            case (state)
                //=============================================================
                // Simple Commands (ARM, RESET, INIT, etc) - CTRL write only
                //=============================================================
                ST_SIMPLE_CMD: begin
                    lb_cs_ctrl_reg <= 1'b1;
                    lb_cs_data_reg <= 1'b0;
                    lb_wr_reg      <= 1'b1;
                    lb_wr_d_reg    <= {26'h0, get_sump_cmd(reg_cmd)};
                end
                ST_SIMPLE_HOLD: begin
                    lb_cs_ctrl_reg <= 1'b1;
                    lb_cs_data_reg <= 1'b0;
                end
                
                //=============================================================
                // Local Reads - CTRL write + DATA read
                //=============================================================
                ST_LOCAL_CMD: begin
                    lb_cs_ctrl_reg <= 1'b1;
                    lb_cs_data_reg <= 1'b0;
                    lb_wr_reg      <= 1'b1;
                    lb_wr_d_reg    <= {26'h0, get_sump_cmd(reg_cmd)};
                end
                ST_LOCAL_HOLD: begin
                    lb_cs_ctrl_reg <= 1'b1;
                    lb_cs_data_reg <= 1'b0;
                end
                ST_LOCAL_READ, ST_LOCAL_WAIT: begin
                    lb_cs_ctrl_reg <= 1'b0;
                    lb_cs_data_reg <= 1'b1;
                    lb_rd_reg      <= (state == ST_LOCAL_READ);
                end
                
                //=============================================================
                // Local Writes - CTRL write + DATA write
                //=============================================================
                ST_LOCAL_WR_CMD: begin
                    lb_cs_ctrl_reg <= 1'b1;
                    lb_cs_data_reg <= 1'b0;
                    lb_wr_reg      <= 1'b1;
                    lb_wr_d_reg    <= {26'h0, get_sump_cmd(reg_cmd)};
                end
                ST_LOCAL_WR_HOLD: begin
                    lb_cs_ctrl_reg <= 1'b1;
                    lb_cs_data_reg <= 1'b0;
                end
                ST_LOCAL_WR_DATA: begin
                    lb_cs_ctrl_reg <= 1'b0;
                    lb_cs_data_reg <= 1'b1;
                    lb_wr_reg      <= 1'b1;
                    lb_wr_d_reg    <= reg_wdata;
                end
                ST_LOCAL_WR_DATA_HOLD: begin
                    lb_cs_ctrl_reg <= 1'b0;
                    lb_cs_data_reg <= 1'b1;
                end
                
                //=============================================================
                // Serial Bus - INST_ADDR command (0x32)
                //=============================================================
                ST_INST_ADDR_CMD: begin
                    lb_cs_ctrl_reg <= 1'b1;
                    lb_cs_data_reg <= 1'b0;
                    lb_wr_reg      <= 1'b1;
                    lb_wr_d_reg    <= {26'h0, SUMP_CMD_WR_INST_ADDR};
                end
                ST_INST_ADDR_HOLD: begin
                    lb_cs_ctrl_reg <= 1'b1;
                    lb_cs_data_reg <= 1'b0;
                end
                
                //=============================================================
                // Serial Bus - Address data
                //=============================================================
                ST_INST_ADDR_DATA: begin
                    lb_cs_ctrl_reg <= 1'b0;
                    lb_cs_data_reg <= 1'b1;
                    lb_wr_reg      <= 1'b1;
                    lb_wr_d_reg    <= reg_addr;  // {hub[23:16], pod[15:8], reg[7:0]}
                end
                ST_INST_ADDR_DATA_HOLD: begin
                    lb_cs_ctrl_reg <= 1'b0;
                    lb_cs_data_reg <= 1'b1;
                end
                
                //=============================================================
                // Serial Bus - Target command (0x33/0x36/etc)
                //=============================================================
                ST_TARGET_CMD: begin
                    lb_cs_ctrl_reg <= 1'b1;
                    lb_cs_data_reg <= 1'b0;
                    lb_wr_reg      <= 1'b1;
                    lb_wr_d_reg    <= {26'h0, get_sump_cmd(reg_cmd)};
                end
                ST_TARGET_CMD_HOLD: begin
                    lb_cs_ctrl_reg <= 1'b1;
                    lb_cs_data_reg <= 1'b0;
                end
                
                //=============================================================
                // Serial Bus Read - Trigger and wait
                //=============================================================
                ST_TARGET_TRIGGER: begin
                    lb_cs_ctrl_reg <= 1'b0;
                    lb_cs_data_reg <= 1'b1;
                    lb_rd_reg      <= 1'b1;  // Trigger serial request
                end
                ST_TARGET_TRIGGER_HOLD, ST_TARGET_SERIAL_WAIT: begin
                    lb_cs_ctrl_reg <= 1'b0;
                    lb_cs_data_reg <= 1'b1;
                end
                
                //=============================================================
                // Serial Bus Read - Final read
                //=============================================================
                ST_TARGET_READ: begin
                    lb_cs_ctrl_reg <= 1'b0;
                    lb_cs_data_reg <= 1'b1;
                    lb_rd_reg      <= 1'b1;
                end
                ST_TARGET_READ_WAIT: begin
                    lb_cs_ctrl_reg <= 1'b0;
                    lb_cs_data_reg <= 1'b1;
                end
                
                //=============================================================
                // Serial Bus Write - Write data
                //=============================================================
                ST_TARGET_WRITE: begin
                    lb_cs_ctrl_reg <= 1'b0;
                    lb_cs_data_reg <= 1'b1;
                    lb_wr_reg      <= 1'b1;
                    lb_wr_d_reg    <= reg_wdata;
                end
                ST_TARGET_WRITE_HOLD: begin
                    lb_cs_ctrl_reg <= 1'b0;
                    lb_cs_data_reg <= 1'b1;
                end
                
                //=============================================================
                // Default: Release bus
                //=============================================================
                default: begin
                    lb_cs_ctrl_reg <= 1'b0;
                    lb_cs_data_reg <= 1'b0;
                end
            endcase
        end
    end
    
    // Drive outputs from registers
    assign lb_cs_ctrl = lb_cs_ctrl_reg;
    assign lb_cs_data = lb_cs_data_reg;
    assign lb_wr      = lb_wr_reg;
    assign lb_rd      = lb_rd_reg;
    assign lb_wr_d    = lb_wr_d_reg;

endmodule
