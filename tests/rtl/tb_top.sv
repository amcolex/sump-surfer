`timescale 1ns / 100ps
//=============================================================================
//
// tb_top.sv - Multi-Clock-Domain Simulation Testbench
//
//=============================================================================
//
// OVERVIEW
// ========
// This testbench validates SUMP3 ILA operation across multiple clock domains
// using clean 2x/4x clock ratios for deterministic CDC timing:
//
//   - AXI Bus Clock: 100 MHz (clk_lb)
//   - Domain 0:       50 MHz (slow capture domain - 2x slower than bus)
//   - Domain 1:      200 MHz (fast capture domain - 2x faster than bus)
//
// CLOCK RELATIONSHIPS
// ===================
//   200 MHz ──┬── base clock (fastest)
//             │
//   100 MHz ──┼── divide by 2 (bus clock)
//             │
//    50 MHz ──┴── divide by 4 (slowest)
//
// This allows clean CDC synchronization with predictable timing.
//
// ARCHITECTURE
// ============
//
//   tb_main.cpp (Verilator)
//        │
//        ▼
//   ┌─────────────────────────────────────────────────────────────────────────┐
//   │  tb_top                                                                 │
//   │                                                                         │
//   │   AXI4-Lite    ┌───────────────┐  Local   ┌────────────────────┐       │
//   │   (100 MHz)    │ sump3_axi_    │  Bus     │   sump3_core       │       │
//   │   ──────────▶  │    wrapper    │ ───────▶ │   (100 MHz lb)     │       │
//   │   ◀──────────  └───────────────┘          │                    │       │
//   │                                           │   Hub 0 ─┬─ Hub 1  │       │
//   │                                           └──────────┼─────────┘       │
//   │                                                      │                 │
//   │        ┌─────────────────────────────────────────────┼─────────────┐   │
//   │        │                                             │             │   │
//   │        ▼ (50 MHz Domain)                             ▼ (200 MHz)   │   │
//   │   ┌────────────────┐                          ┌────────────────┐   │   │
//   │   │  sump3_rle_hub │                          │  sump3_rle_hub │   │   │
//   │   │    (Hub 0)     │                          │    (Hub 1)     │   │   │
//   │   └───────┬────────┘                          └───────┬────────┘   │   │
//   │           │                                           │            │   │
//   │           ▼                                           ▼            │   │
//   │   ┌────────────────┐                          ┌────────────────┐   │   │
//   │   │  sump3_rle_pod │                          │  sump3_rle_pod │   │   │
//   │   └───────┬────────┘                          └───────┬────────┘   │   │
//   │           │                                           │            │   │
//   │           ▼                                           ▼            │   │
//   │   ┌────────────────┐                          ┌────────────────┐   │   │
//   │   │  simple_dut    │                          │  simple_dut    │   │   │
//   │   │  + Analog ADC  │                          │  + Analog ADC  │   │   │
//   │   └────────────────┘                          └────────────────┘   │   │
//   │                                                                    │   │
//   └────────────────────────────────────────────────────────────────────┘   │
//   └─────────────────────────────────────────────────────────────────────────┘
//
//=============================================================================

module tb_top (
    //=========================================================================
    // Clocks and Reset
    //=========================================================================
    input  logic        clk,           // AXI bus clock (100 MHz)
    input  logic        clk_50mhz,     // Slow capture domain (50 MHz) - 2x slower
    input  logic        clk_200mhz,    // Fast capture domain (200 MHz) - 2x faster
    input  logic        rst_n,         // Active-low reset
    
    //=========================================================================
    // AXI4-Lite Interface
    //=========================================================================
    input  logic [7:0]  s_axi_awaddr,
    input  logic        s_axi_awvalid,
    output logic        s_axi_awready,
    
    input  logic [31:0] s_axi_wdata,
    input  logic [3:0]  s_axi_wstrb,
    input  logic        s_axi_wvalid,
    output logic        s_axi_wready,
    
    output logic [1:0]  s_axi_bresp,
    output logic        s_axi_bvalid,
    input  logic        s_axi_bready,
    
    input  logic [7:0]  s_axi_araddr,
    input  logic        s_axi_arvalid,
    output logic        s_axi_arready,
    
    output logic [31:0] s_axi_rdata,
    output logic [1:0]  s_axi_rresp,
    output logic        s_axi_rvalid,
    input  logic        s_axi_rready,
    
    //=========================================================================
    // DUT Control
    //=========================================================================
    input  logic        dut_enable,     // Enable DUT operation
    input  logic        dut_pause,      // Pause DUT
    input  logic        dut_trigger_in, // External trigger input
    
    //=========================================================================
    // Outputs to Verilator
    //=========================================================================
    output logic        irq,
    output logic [31:0] dut_counter,    // DUT0 counter (40 MHz domain)
    output logic [3:0]  dut_fsm_state,  // DUT0 FSM state
    output logic        sump_armed,
    output logic        sump_awake
);

    //=========================================================================
    // SUMP3 Local Bus Signals
    //=========================================================================
    logic        lb_cs_ctrl;
    logic        lb_cs_data;
    logic        lb_wr;
    logic        lb_rd;
    logic [31:0] lb_wr_d;
    logic [31:0] lb_rd_d;
    logic        lb_rd_rdy;
    
    //=========================================================================
    // SUMP3 Serial Bus (Core <-> Hubs) - 2 hubs
    //=========================================================================
    logic [1:0]  core_mosi;
    logic [1:0]  core_miso;
    logic [1:0]  trigger_mosi;
    logic [1:0]  trigger_miso;
    
    //=========================================================================
    // Hub 0 (40 MHz) Serial Bus to Pod
    //=========================================================================
    logic [0:0]  pod0_mosi;
    logic [0:0]  pod0_miso;
    
    //=========================================================================
    // Hub 1 (180 MHz) Serial Bus to Pod
    //=========================================================================
    logic [0:0]  pod1_mosi;
    logic [0:0]  pod1_miso;
    
    //=========================================================================
    // SUMP3 Status
    //=========================================================================
    logic        sump_is_armed;
    logic        sump_is_awake;
    
    //=========================================================================
    // Output Assignments
    //=========================================================================
    assign sump_armed    = sump_is_armed;
    assign sump_awake    = sump_is_awake;

    //=========================================================================
    //=========================================================================
    //
    //  DOMAIN 0: 50 MHz Capture Domain (2x slower than bus)
    //
    //=========================================================================
    //=========================================================================
    
    //-------------------------------------------------------------------------
    // Domain 0 Signals
    //-------------------------------------------------------------------------
    logic [31:0] events0;
    logic [31:0] counter0;
    logic [3:0]  fsm_state0;
    logic        busy0;
    logic        done0;
    logic [7:0]  data_out0;
    
    // Analog signals for domain 0
    logic [11:0] adc0_ch0, adc0_ch1;
    logic [31:0] dig_triggers0;
    logic        rec0_wr_en;
    logic [7:0]  rec0_wr_addr;
    logic [31:0] rec0_wr_data;
    logic        rec0_sample_start;
    logic [31:0] rec0_timestamp;
    logic        trigger0_adc_more, trigger0_adc_less;
    logic [23:0] trigger0_adc_level;  // 24-bit from sump3_core
    logic [3:0]  trigger0_adc_ch;
    
    // Tick clock for domain 0
    logic [7:0]  tick0_divider;
    logic        ck_tick0;
    
    //-------------------------------------------------------------------------
    // Domain 0 Tick Clock (for analog sampling)
    //-------------------------------------------------------------------------
    always_ff @(posedge clk_50mhz or negedge rst_n) begin
        if (!rst_n) begin
            tick0_divider <= 8'h0;
            ck_tick0 <= 1'b0;
        end else begin
            tick0_divider <= tick0_divider + 1'b1;
            ck_tick0 <= (tick0_divider[4:0] == 5'h1F);
        end
    end
    
    //-------------------------------------------------------------------------
    // Domain 0 ADC Signal Generator (Sawtooth)
    //-------------------------------------------------------------------------
    logic [15:0] adc0_counter;
    logic [7:0]  adc0_sample_div;
    logic        adc0_sample_tick;
    
    always_ff @(posedge clk_50mhz or negedge rst_n) begin
        if (!rst_n) begin
            adc0_counter <= 16'h0;
            adc0_sample_div <= 8'h0;
            adc0_sample_tick <= 1'b0;
        end else if (dut_enable) begin
            adc0_sample_div <= adc0_sample_div + 1'b1;
            adc0_sample_tick <= (adc0_sample_div == 8'hFF);
            if (adc0_sample_tick)
                adc0_counter <= adc0_counter + 16'd64;
        end else begin
            adc0_counter <= 16'h0;
            adc0_sample_div <= 8'h0;
            adc0_sample_tick <= 1'b0;
        end
    end
    
    assign adc0_ch0 = adc0_counter[15:4];
    assign adc0_ch1 = ~adc0_counter[15:4];
    assign dig_triggers0 = {20'h0, trigger0_adc_ch[0] ? adc0_ch1 : adc0_ch0};
    assign trigger0_adc_more = dig_triggers0[11:0] > trigger0_adc_level[11:0];
    assign trigger0_adc_less = dig_triggers0[11:0] < trigger0_adc_level[11:0];
    
    //-------------------------------------------------------------------------
    // Domain 0 Analog Sample Writer
    //-------------------------------------------------------------------------
    logic [3:0] ana0_write_state;
    logic [7:0] ana0_slot_addr;
    logic       rec0_sample_start_d1, rec0_sample_start_d2;
    
    always_ff @(posedge clk_50mhz or negedge rst_n) begin
        if (!rst_n) begin
            rec0_wr_en <= 1'b0;
            rec0_wr_addr <= 8'h0;
            rec0_wr_data <= 32'h0;
            ana0_write_state <= 4'h0;
            ana0_slot_addr <= 8'h0;
            rec0_sample_start_d1 <= 1'b0;
            rec0_sample_start_d2 <= 1'b0;
        end else begin
            rec0_wr_en <= 1'b0;
            rec0_sample_start_d1 <= rec0_sample_start;
            rec0_sample_start_d2 <= rec0_sample_start_d1;
            
            case (ana0_write_state)
                4'h0: if (rec0_sample_start_d2 && !rec0_sample_start_d1) begin
                    ana0_write_state <= 4'h1;
                    ana0_slot_addr <= 8'h02;
                end
                4'h1: ana0_write_state <= 4'h2;
                4'h2: begin
                    rec0_wr_en <= 1'b1;
                    rec0_wr_addr <= ana0_slot_addr;
                    rec0_wr_data <= {8'h82, adc0_ch1, adc0_ch0};
                    ana0_write_state <= 4'h3;
                end
                4'h3: begin
                    ana0_slot_addr <= ana0_slot_addr + 1'b1;
                    ana0_write_state <= 4'h4;
                end
                4'h4: ana0_write_state <= 4'h0;
                default: ana0_write_state <= 4'h0;
            endcase
        end
    end
    
    //-------------------------------------------------------------------------
    // Domain 0 Events
    //-------------------------------------------------------------------------
    assign events0 = {
        4'h0,           // [31:28]
        data_out0,      // [27:20]
        2'b0,           // [19:18]
        done0,          // [17]
        busy0,          // [16]
        12'h0,          // [15:4]
        fsm_state0      // [3:0]
    };
    
    // Expose Domain 0 DUT signals to testbench
    assign dut_counter   = counter0;
    assign dut_fsm_state = fsm_state0;
    
    //-------------------------------------------------------------------------
    // Domain 0 DUT Instance
    //-------------------------------------------------------------------------
    simple_dut u_dut0 (
        .clk        (clk_50mhz),
        .rst_n      (rst_n),
        .enable     (dut_enable),
        .pause      (dut_pause),
        .trigger_in (dut_trigger_in),
        .counter    (counter0),
        .fsm_state  (fsm_state0),
        .busy       (busy0),
        .done       (done0),
        .data_out   (data_out0)
    );
    
    //-------------------------------------------------------------------------
    // Domain 0 Hub (50 MHz - 2x slower than bus)
    //-------------------------------------------------------------------------
    sump3_rle_hub #(
        .hub_name       ("slow_50mhz "),
        .hub_name_en    (1),
        .hub_instance   (0),
        .user_bus_en    (0),
        .ck_freq_mhz    (12'd50),
        .ck_freq_fracts (20'h00000),
        .rle_pod_num    (1)
    ) u_hub0 (
        .clk_lb         (clk),
        .clk_cap        (clk_50mhz),
        .core_mosi      (core_mosi[0]),
        .core_miso      (core_miso[0]),
        .trigger_mosi   (trigger_mosi[0]),
        .trigger_miso   (trigger_miso[0]),
        .sump_is_armed  (),
        .user_bus_wr    (),
        .user_bus_rd    (),
        .user_bus_addr  (),
        .user_bus_wr_d  (),
        .user_bus_rd_d  (32'h0),
        .pod_mosi       (pod0_mosi),
        .pod_miso       (pod0_miso)
    );
    
    //-------------------------------------------------------------------------
    // Domain 0 Pod (40 MHz)
    //-------------------------------------------------------------------------
    sump3_rle_pod #(
        .pod_name             ("slow_signals"),
        .pod_instance         (0),
        .rle_hw_revision      (8'h01),
        .pod_disable          (0),
        .rle_disable          (0),
        .pod_user_ctrl_en     (1),
        .pod_user_mask_en     (1),
        .pod_name_en          (1),
        .norom_view_dwords    (0),
        .norom_view_words     (0),
        .norom_view_bytes     (0),
        .norom_view_bits      (1),
        .trigger_comp_en      (1),
        .trigger_en           (1),
        .view_rom_en          (0),
        .view_rom_size        (16384),
        .view_rom_txt         (""),
        .trig_bits            (32'hFFFFFFFF),
        .trig_offset_core_cks (8'd8),
        .trig_offset_miso_cks (8'd12),
        .trig_offset_mosi_cks (8'd17),
        .rle_ram_depth_len    (512),
        .rle_ram_depth_bits   (9),
        .rle_ram_width        (48),
        .rle_code_bits        (2),
        .rle_timestamp_bits   (14),
        .rle_data_bits        (32)
    ) u_pod0 (
        .clk_cap          (clk_50mhz),
        .events           (events0),
        .pod_mosi         (pod0_mosi[0]),
        .pod_miso         (pod0_miso[0]),
        .pod_user_ctrl    (),
        .pod_user_trig    (),
        .pod_user_mask    (),
        .pod_user_pretrig (),
        .pod_is_armed     ()
    );
    
    //=========================================================================
    //=========================================================================
    //
    //  DOMAIN 1: 200 MHz Capture Domain (2x faster than bus)
    //
    //=========================================================================
    //=========================================================================
    
    //-------------------------------------------------------------------------
    // Domain 1 Signals
    //-------------------------------------------------------------------------
    logic [31:0] events1;
    logic [31:0] counter1;
    logic [3:0]  fsm_state1;
    logic        busy1;
    logic        done1;
    logic [7:0]  data_out1;
    
    // Analog signals for domain 1
    logic [11:0] adc1_ch0, adc1_ch1;
    logic [31:0] dig_triggers1;
    logic        rec1_wr_en;
    logic [7:0]  rec1_wr_addr;
    logic [31:0] rec1_wr_data;
    logic        rec1_sample_start;
    logic [31:0] rec1_timestamp;
    logic        trigger1_adc_more, trigger1_adc_less;
    logic [11:0] trigger1_adc_level;
    logic [3:0]  trigger1_adc_ch;
    
    // Tick clock for domain 1
    logic [7:0]  tick1_divider;
    logic        ck_tick1;
    
    //-------------------------------------------------------------------------
    // Domain 1 Tick Clock (for analog sampling)
    //-------------------------------------------------------------------------
    always_ff @(posedge clk_200mhz or negedge rst_n) begin
        if (!rst_n) begin
            tick1_divider <= 8'h0;
            ck_tick1 <= 1'b0;
        end else begin
            tick1_divider <= tick1_divider + 1'b1;
            ck_tick1 <= (tick1_divider[4:0] == 5'h1F);
        end
    end
    
    //-------------------------------------------------------------------------
    // Domain 1 ADC Signal Generator (Triangle wave - different from domain 0)
    //-------------------------------------------------------------------------
    logic [15:0] adc1_counter;
    logic [7:0]  adc1_sample_div;
    logic        adc1_sample_tick;
    logic        adc1_direction;  // 0=up, 1=down (triangle wave)
    
    always_ff @(posedge clk_200mhz or negedge rst_n) begin
        if (!rst_n) begin
            adc1_counter <= 16'h0;
            adc1_sample_div <= 8'h0;
            adc1_sample_tick <= 1'b0;
            adc1_direction <= 1'b0;
        end else if (dut_enable) begin
            adc1_sample_div <= adc1_sample_div + 1'b1;
            adc1_sample_tick <= (adc1_sample_div == 8'h7F);  // Faster sample rate
            if (adc1_sample_tick) begin
                if (adc1_direction == 1'b0) begin
                    if (adc1_counter >= 16'hFF00)
                        adc1_direction <= 1'b1;
                    else
                        adc1_counter <= adc1_counter + 16'd128;
                end else begin
                    if (adc1_counter <= 16'h0100)
                        adc1_direction <= 1'b0;
                    else
                        adc1_counter <= adc1_counter - 16'd128;
                end
            end
        end else begin
            adc1_counter <= 16'h0;
            adc1_sample_div <= 8'h0;
            adc1_sample_tick <= 1'b0;
            adc1_direction <= 1'b0;
        end
    end
    
    assign adc1_ch0 = adc1_counter[15:4];
    assign adc1_ch1 = ~adc1_counter[15:4];
    assign dig_triggers1 = {20'h0, trigger1_adc_ch[0] ? adc1_ch1 : adc1_ch0};
    assign trigger1_adc_more = dig_triggers1[11:0] > trigger1_adc_level;
    assign trigger1_adc_less = dig_triggers1[11:0] < trigger1_adc_level;
    
    //-------------------------------------------------------------------------
    // Domain 1 Analog Sample Writer
    //-------------------------------------------------------------------------
    logic [3:0] ana1_write_state;
    logic [7:0] ana1_slot_addr;
    logic       rec1_sample_start_d1, rec1_sample_start_d2;
    
    always_ff @(posedge clk_200mhz or negedge rst_n) begin
        if (!rst_n) begin
            rec1_wr_en <= 1'b0;
            rec1_wr_addr <= 8'h0;
            rec1_wr_data <= 32'h0;
            ana1_write_state <= 4'h0;
            ana1_slot_addr <= 8'h0;
            rec1_sample_start_d1 <= 1'b0;
            rec1_sample_start_d2 <= 1'b0;
        end else begin
            rec1_wr_en <= 1'b0;
            rec1_sample_start_d1 <= rec1_sample_start;
            rec1_sample_start_d2 <= rec1_sample_start_d1;
            
            case (ana1_write_state)
                4'h0: if (rec1_sample_start_d2 && !rec1_sample_start_d1) begin
                    ana1_write_state <= 4'h1;
                    ana1_slot_addr <= 8'h02;
                end
                4'h1: ana1_write_state <= 4'h2;
                4'h2: begin
                    rec1_wr_en <= 1'b1;
                    rec1_wr_addr <= ana1_slot_addr;
                    rec1_wr_data <= {8'h82, adc1_ch1, adc1_ch0};
                    ana1_write_state <= 4'h3;
                end
                4'h3: begin
                    ana1_slot_addr <= ana1_slot_addr + 1'b1;
                    ana1_write_state <= 4'h4;
                end
                4'h4: ana1_write_state <= 4'h0;
                default: ana1_write_state <= 4'h0;
            endcase
        end
    end
    
    //-------------------------------------------------------------------------
    // Domain 1 Events
    //-------------------------------------------------------------------------
    assign events1 = {
        4'h1,           // [31:28] Domain ID marker
        data_out1,      // [27:20]
        2'b0,           // [19:18]
        done1,          // [17]
        busy1,          // [16]
        12'h0,          // [15:4]
        fsm_state1      // [3:0]
    };
    
    //-------------------------------------------------------------------------
    // Domain 1 DUT Instance
    //-------------------------------------------------------------------------
    simple_dut u_dut1 (
        .clk        (clk_200mhz),
        .rst_n      (rst_n),
        .enable     (dut_enable),
        .pause      (dut_pause),
        .trigger_in (dut_trigger_in),
        .counter    (counter1),
        .fsm_state  (fsm_state1),
        .busy       (busy1),
        .done       (done1),
        .data_out   (data_out1)
    );
    
    //-------------------------------------------------------------------------
    // Domain 1 Hub (200 MHz - 2x faster than bus)
    //-------------------------------------------------------------------------
    sump3_rle_hub #(
        .hub_name       ("fast_200mhz"),
        .hub_name_en    (1),
        .hub_instance   (1),
        .user_bus_en    (0),
        .ck_freq_mhz    (12'd200),
        .ck_freq_fracts (20'h00000),
        .rle_pod_num    (1)
    ) u_hub1 (
        .clk_lb         (clk),
        .clk_cap        (clk_200mhz),
        .core_mosi      (core_mosi[1]),
        .core_miso      (core_miso[1]),
        .trigger_mosi   (trigger_mosi[1]),
        .trigger_miso   (trigger_miso[1]),
        .sump_is_armed  (),
        .user_bus_wr    (),
        .user_bus_rd    (),
        .user_bus_addr  (),
        .user_bus_wr_d  (),
        .user_bus_rd_d  (32'h0),
        .pod_mosi       (pod1_mosi),
        .pod_miso       (pod1_miso)
    );
    
    //-------------------------------------------------------------------------
    // Domain 1 Pod (180 MHz)
    //-------------------------------------------------------------------------
    sump3_rle_pod #(
        .pod_name             ("fast_signals"),
        .pod_instance         (0),
        .rle_hw_revision      (8'h01),
        .pod_disable          (0),
        .rle_disable          (0),
        .pod_user_ctrl_en     (1),
        .pod_user_mask_en     (1),
        .pod_name_en          (1),
        .norom_view_dwords    (0),
        .norom_view_words     (0),
        .norom_view_bytes     (0),
        .norom_view_bits      (1),
        .trigger_comp_en      (1),
        .trigger_en           (1),
        .view_rom_en          (0),
        .view_rom_size        (16384),
        .view_rom_txt         (""),
        .trig_bits            (32'hFFFFFFFF),
        .trig_offset_core_cks (8'd8),
        .trig_offset_miso_cks (8'd12),
        .trig_offset_mosi_cks (8'd17),
        .rle_ram_depth_len    (1024),          // Larger RAM for faster domain
        .rle_ram_depth_bits   (10),
        .rle_ram_width        (48),
        .rle_code_bits        (2),
        .rle_timestamp_bits   (14),
        .rle_data_bits        (32)
    ) u_pod1 (
        .clk_cap          (clk_200mhz),
        .events           (events1),
        .pod_mosi         (pod1_mosi[0]),
        .pod_miso         (pod1_miso[0]),
        .pod_user_ctrl    (),
        .pod_user_trig    (),
        .pod_user_mask    (),
        .pod_user_pretrig (),
        .pod_is_armed     ()
    );

    //=========================================================================
    //=========================================================================
    //
    //  SHARED: AXI Wrapper and SUMP3 Core
    //
    //=========================================================================
    //=========================================================================
    
    //-------------------------------------------------------------------------
    // AXI4-Lite Wrapper
    //-------------------------------------------------------------------------
    sump3_axi_wrapper #(
        .C_S_AXI_DATA_WIDTH (32),
        .C_S_AXI_ADDR_WIDTH (8),
        .DEFAULT_TIMEOUT    (16384)  // Larger timeout for CDC
    ) u_axi_wrapper (
        .s_axi_aclk     (clk),
        .s_axi_aresetn  (rst_n),
        
        .s_axi_awaddr   (s_axi_awaddr),
        .s_axi_awprot   (3'b000),
        .s_axi_awvalid  (s_axi_awvalid),
        .s_axi_awready  (s_axi_awready),
        
        .s_axi_wdata    (s_axi_wdata),
        .s_axi_wstrb    (s_axi_wstrb),
        .s_axi_wvalid   (s_axi_wvalid),
        .s_axi_wready   (s_axi_wready),
        
        .s_axi_bresp    (s_axi_bresp),
        .s_axi_bvalid   (s_axi_bvalid),
        .s_axi_bready   (s_axi_bready),
        
        .s_axi_araddr   (s_axi_araddr),
        .s_axi_arprot   (3'b000),
        .s_axi_arvalid  (s_axi_arvalid),
        .s_axi_arready  (s_axi_arready),
        
        .s_axi_rdata    (s_axi_rdata),
        .s_axi_rresp    (s_axi_rresp),
        .s_axi_rvalid   (s_axi_rvalid),
        .s_axi_rready   (s_axi_rready),
        
        .irq            (irq),
        
        .lb_cs_ctrl     (lb_cs_ctrl),
        .lb_cs_data     (lb_cs_data),
        .lb_wr          (lb_wr),
        .lb_rd          (lb_rd),
        .lb_wr_d        (lb_wr_d),
        .lb_rd_d        (lb_rd_d),
        .lb_rd_rdy      (lb_rd_rdy),
        
        .sump_is_armed  (sump_is_armed),
        .sump_is_awake  (sump_is_awake),
        .sump_hub_count (8'd2)   // TWO hubs
    );

    //-------------------------------------------------------------------------
    // SUMP3 Core (shared between both domains)
    // Core runs on AXI bus clock, connects to hubs in different domains
    //-------------------------------------------------------------------------
    sump3_core #(
        .ana_ls_enable      (1),
        .ana_ram_depth_len  (256),
        .ana_ram_depth_bits (8),
        .ana_ram_width      (32),
        .rle_hub_num        (2),              // TWO hubs
        .view_rom_en        (0),
        .ck_freq_mhz        (12'd100),        // AXI bus clock
        .ck_freq_fracts     (20'h00000),
        .tick_freq_mhz      (12'd1),
        .tick_freq_fracts   (20'h00000),
        .thread_lock_en     (0),
        .bus_busy_timer_en  (0),
        .sump_id            (8'h53),
        .sump_rev           (8'h03),
        .sump_en            (1)
    ) u_sump3_core (
        .clk_lb             (clk),
        .clk_cap            (clk),            // Core's internal capture uses bus clock
        .ck_tick            (ck_tick0),       // Use domain 0 tick for analog RAM
        
        .lb_cs_ctrl         (lb_cs_ctrl),
        .lb_cs_data         (lb_cs_data),
        .lb_wr              (lb_wr),
        .lb_rd              (lb_rd),
        .lb_wr_d            (lb_wr_d),
        .lb_rd_d            (lb_rd_d),
        .lb_rd_rdy          (lb_rd_rdy),
        
        // ADC Trigger (from domain 0 - primary analog)
        .trigger_adc_ch     (trigger0_adc_ch),
        .trigger_adc_level  (trigger0_adc_level),
        .trigger_adc_more   (trigger0_adc_more),
        .trigger_adc_less   (trigger0_adc_less),
        
        // Analog Recording
        .rec_cfg_select     (),
        .rec_cfg_profile    (32'h01000000),
        .rec_wr_en          (rec0_wr_en),
        .rec_wr_addr        (rec0_wr_addr),
        .rec_wr_data        (rec0_wr_data),
        .rec_timestamp      (rec0_timestamp),
        .rec_sample_start   (rec0_sample_start),
        
        // Serial Bus to BOTH Hubs
        .core_mosi          (core_mosi),
        .core_miso          (core_miso),
        .trigger_mosi       (trigger_mosi),
        .trigger_miso       (trigger_miso),
        
        .trigger_in         (dut_trigger_in),
        .trigger_out        (),
        
        .sump_is_armed      (sump_is_armed),
        .sump_is_awake      (sump_is_awake),
        
        .core_user_ctrl     (),
        .dig_triggers       (dig_triggers0)
    );

endmodule
