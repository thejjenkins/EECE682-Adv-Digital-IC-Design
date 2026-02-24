// =============================================================================
//  rk4_if.sv -- SystemVerilog interface wrapping all DUT-level signals
// =============================================================================
interface rk4_if (input logic clk);

  logic       rst;
  logic       en;
  logic [1:0] sel;
  logic       uart_rx;
  logic       uart_tx;
  logic       clk_1Hz;

  // Driver clocking block -- drives uart_rx, samples uart_tx
  clocking drv_cb @(posedge clk);
    default input #1step output #1;
    output uart_rx;
    output rst;
    output en;
    output sel;
    input  uart_tx;
    input  clk_1Hz;
  endclocking

  // Monitor clocking block -- purely observational
  clocking mon_cb @(posedge clk);
    default input #1step;
    input uart_tx;
    input uart_rx;
    input rst;
    input en;
    input sel;
    input clk_1Hz;
  endclocking

  modport driver  (clocking drv_cb, input clk);
  modport monitor (clocking mon_cb, input clk);

endinterface
