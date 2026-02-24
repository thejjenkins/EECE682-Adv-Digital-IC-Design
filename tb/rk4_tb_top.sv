`timescale 1ns / 1ps
// =============================================================================
//  rk4_tb_top.sv -- Top-level testbench: clock gen, DUT, interface, UVM entry
// =============================================================================
module rk4_tb_top;

  import uvm_pkg::*;
  import rk4_pkg::*;

  // -------------------------------------------------------------------------
  //  Clock generation -- 100 MHz (10 ns period)
  // -------------------------------------------------------------------------
  localparam real CLK_PERIOD = 10.0;

  logic clk;
  initial clk = 1'b0;
  always #(CLK_PERIOD / 2.0) clk = ~clk;

  // -------------------------------------------------------------------------
  //  Interface
  // -------------------------------------------------------------------------
  rk4_if rk4_vif (.clk(clk));

  // -------------------------------------------------------------------------
  //  DUT -- rk4_top
  //  rk4_top has no parameters.  Internally, rk4_clk_gen divides the
  //  100 MHz input by 100, producing a 1 MHz clock for rk4_projectile_top.
  //  rk4_projectile_top defaults to CLK_FREQ=50 MHz / BAUD_RATE=115200
  //  => BAUD_DIV = 434 on that 1 MHz clock.
  //  From the TB's 100 MHz clock, each UART bit = 434 * 100 = 43400 cycles.
  //  The driver/monitor baud_div is set in the test accordingly.
  // -------------------------------------------------------------------------
  rk4_top dut (
    .clk_100MHz (clk),
    .en         (rk4_vif.en),
    .rst        (rk4_vif.rst),
    .sel        (rk4_vif.sel),
    .uart_rx    (rk4_vif.uart_rx),
    .uart_tx    (rk4_vif.uart_tx),
    .clk_1Hz    (rk4_vif.clk_1Hz)
  );

  // -------------------------------------------------------------------------
  //  UVM hookup
  // -------------------------------------------------------------------------
  initial begin
    uvm_config_db#(virtual rk4_if)::set(null, "*", "vif", rk4_vif);
    run_test();
  end

  // -------------------------------------------------------------------------
  //  Waveform dump (Cadence SimVision / xrun)
  // -------------------------------------------------------------------------
  initial begin
    $shm_open("waves.shm");
    $shm_probe(rk4_tb_top, "ASTM");
  end

endmodule
