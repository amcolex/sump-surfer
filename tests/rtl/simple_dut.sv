`timescale 1ns / 100ps
//=============================================================================
// simple_dut.sv
// Simple Device Under Test for SUMP3 ILA Testing
//=============================================================================
//
// OVERVIEW
// --------
// This module provides a simple, deterministic design to verify the SUMP3
// ILA capture functionality. It includes:
//   - 32-bit free-running counter
//   - 8-state FSM with various transitions
//   - Control inputs for triggering state changes
//
// SIGNAL MAPPING FOR SUMP3 CAPTURE
// --------------------------------
// The following signals are captured by the SUMP3 RLE pod:
//
//   events_to_capture[31:28] = Reserved (0)
//   events_to_capture[27:20] = data_out (8-bit derived data)
//   events_to_capture[19:18] = Reserved (0)
//   events_to_capture[17]    = done flag
//   events_to_capture[16]    = busy flag  
//   events_to_capture[15:4]  = Reserved (0)
//   events_to_capture[3:0]   = fsm_state
//
// FSM STATE DIAGRAM
// -----------------
//
//   +------+    enable    +------+              +----------+
//   | IDLE |------------->| INIT |------------->| RUNNING  |
//   +------+              +------+              +----+-----+
//      ^                                             |
//      |                                   pause     |
//      |                                      +------v------+
//      +--------------------------------------+   PAUSED    |
//           !enable                           +------+------+
//                                                    |
//                                             !pause |
//   +------+              +------+   trigger  +------v------+
//   | DONE |<-------------| WAIT |<-----------| COUNTING   |
//   +------+              +------+            +------+------+
//      |                     ^                       |
//      | trigger             |    cnt[7:0]==0xFF     |
//      v                     |                       |
//   +------+             +---+----+                  |
//   | RUN  |             |PROCESS |<-----------------+
//   +------+             +--------+
//
// USAGE
// -----
// 1. Assert 'enable' to start the DUT
// 2. The FSM moves through IDLE -> INIT -> RUNNING
// 3. Assert 'trigger_in' pulse to initiate capture sequence
// 4. FSM progresses: COUNTING -> PROCESS -> WAIT -> DONE
// 5. Another trigger pulse restarts from RUNNING
//
// The 'pause' input suspends operation in PAUSED state.
//
//=============================================================================

module simple_dut (
    input  logic        clk,
    input  logic        rst_n,
    
    // Control inputs
    input  logic        enable,
    input  logic        pause,
    input  logic        trigger_in,
    
    // Outputs
    output logic [31:0] counter,
    output logic [3:0]  fsm_state,
    output logic        busy,
    output logic        done,
    output logic [7:0]  data_out
);

    //-------------------------------------------------------------------------
    // FSM States
    //-------------------------------------------------------------------------
    typedef enum logic [3:0] {
        ST_IDLE     = 4'h0,
        ST_INIT     = 4'h1,
        ST_RUNNING  = 4'h2,
        ST_PAUSED   = 4'h3,
        ST_COUNTING = 4'h4,
        ST_PROCESS  = 4'h5,
        ST_WAIT     = 4'h6,
        ST_DONE     = 4'h7,
        ST_ERROR    = 4'hF
    } state_t;
    
    state_t state, state_next;
    
    //-------------------------------------------------------------------------
    // Internal Signals
    //-------------------------------------------------------------------------
    logic [31:0] counter_reg;
    logic [7:0]  wait_counter;
    logic [7:0]  process_counter;
    logic        trigger_in_d1;
    logic        trigger_edge;
    
    //-------------------------------------------------------------------------
    // Edge detection on trigger
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            trigger_in_d1 <= 1'b0;
        end else begin
            trigger_in_d1 <= trigger_in;
        end
    end
    
    assign trigger_edge = trigger_in && !trigger_in_d1;
    
    //-------------------------------------------------------------------------
    // State Machine
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
        end else begin
            state <= state_next;
        end
    end
    
    always_comb begin
        state_next = state;
        
        case (state)
            ST_IDLE: begin
                if (enable) begin
                    state_next = ST_INIT;
                end
            end
            
            ST_INIT: begin
                state_next = ST_RUNNING;
            end
            
            ST_RUNNING: begin
                if (!enable) begin
                    state_next = ST_IDLE;
                end else if (pause) begin
                    state_next = ST_PAUSED;
                end else if (trigger_edge) begin
                    state_next = ST_COUNTING;
                end
            end
            
            ST_PAUSED: begin
                if (!pause) begin
                    state_next = ST_RUNNING;
                end else if (!enable) begin
                    state_next = ST_IDLE;
                end
            end
            
            ST_COUNTING: begin
                if (counter_reg[7:0] == 8'hFF) begin
                    state_next = ST_PROCESS;
                end
            end
            
            ST_PROCESS: begin
                if (process_counter == 8'h10) begin
                    state_next = ST_WAIT;
                end
            end
            
            ST_WAIT: begin
                if (wait_counter == 8'h20) begin
                    state_next = ST_DONE;
                end
            end
            
            ST_DONE: begin
                if (trigger_edge) begin
                    state_next = ST_RUNNING;
                end
            end
            
            ST_ERROR: begin
                if (!enable) begin
                    state_next = ST_IDLE;
                end
            end
            
            default: begin
                state_next = ST_ERROR;
            end
        endcase
    end
    
    //-------------------------------------------------------------------------
    // Counter Logic
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter_reg <= 32'h0;
        end else begin
            case (state)
                ST_IDLE: begin
                    counter_reg <= 32'h0;
                end
                
                ST_INIT: begin
                    counter_reg <= 32'h0;
                end
                
                ST_RUNNING: begin
                    counter_reg <= counter_reg + 1'b1;
                end
                
                ST_PAUSED: begin
                    // Hold counter
                end
                
                ST_COUNTING: begin
                    counter_reg <= counter_reg + 1'b1;
                end
                
                ST_PROCESS: begin
                    counter_reg <= counter_reg + 2;
                end
                
                ST_WAIT: begin
                    // Hold counter
                end
                
                ST_DONE: begin
                    // Hold counter
                end
                
                default: begin
                    counter_reg <= 32'hDEAD_BEEF;
                end
            endcase
        end
    end
    
    //-------------------------------------------------------------------------
    // Wait and Process Counters
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wait_counter <= 8'h0;
            process_counter <= 8'h0;
        end else begin
            // Process counter
            if (state == ST_PROCESS) begin
                process_counter <= process_counter + 1'b1;
            end else begin
                process_counter <= 8'h0;
            end
            
            // Wait counter
            if (state == ST_WAIT) begin
                wait_counter <= wait_counter + 1'b1;
            end else begin
                wait_counter <= 8'h0;
            end
        end
    end
    
    //-------------------------------------------------------------------------
    // Output Assignments
    //-------------------------------------------------------------------------
    assign counter   = counter_reg;
    assign fsm_state = state;
    assign busy      = (state != ST_IDLE) && (state != ST_DONE);
    assign done      = (state == ST_DONE);
    assign data_out  = counter_reg[7:0] ^ {4'h0, state};

endmodule
