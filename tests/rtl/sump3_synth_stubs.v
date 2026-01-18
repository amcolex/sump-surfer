//-----------------------------------------------------------------------------
// sump3_synth_stubs.v
// Stub modules for SUMP3 synthesis assertion mechanisms
//
// SUMP3 uses a clever trick where it instantiates non-existent modules to
// cause synthesis to fail with descriptive error messages when parameters
// are misconfigured. For simulation, we provide empty stub implementations.
//
// These are NOT functional modules - they exist only to satisfy the parser.
// In a real design, if these get instantiated, you have a parameter error.
//-----------------------------------------------------------------------------

// verilator lint_off DECLFILENAME

module halt_synthesis_bad_ana_ram_params;
  initial begin
    $display("ERROR: Invalid analog RAM parameters in sump3_core!");
    $fatal(1, "ana_ram_depth_bits does not match ana_ram_depth_len");
  end
endmodule

module halt_synthesis_bad_dig_ram_params;
  initial begin
    $display("ERROR: Invalid digital RAM parameters in sump3_core!");
    $fatal(1, "dig_ram_depth_bits does not match dig_ram_depth_len");
  end
endmodule

module halt_synthesis_rle_pod_bad_ram_width;
  initial begin
    $display("ERROR: Invalid RLE pod RAM width!");
    $fatal(1, "rle_code_bits + rle_data_bits + rle_timestamp_bits != rle_ram_width");
  end
endmodule

module halt_synthesis_rle_pod_bad_rle_timestamp_bits;
  initial begin
    $display("ERROR: Invalid RLE timestamp bits when rle_disable=1!");
    $fatal(1, "When rle_disable=1, rle_timestamp_bits must equal rle_ram_depth_bits");
  end
endmodule

module halt_synthesis_rle_pod_bad_ram_length;
  initial begin
    $display("ERROR: Invalid RLE pod RAM length!");
    $fatal(1, "2**rle_ram_depth_bits != rle_ram_depth_len");
  end
endmodule

// verilator lint_on DECLFILENAME
