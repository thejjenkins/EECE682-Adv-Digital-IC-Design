// =============================================================================
//  rk4_pkg.sv -- UVM package: imports + ordered includes of all TB components
// =============================================================================
package rk4_pkg;

  import uvm_pkg::*;
  `include "uvm_macros.svh"

  `include "rk4_types.sv"
  `include "rk4_uart_tx_item.sv"
  `include "rk4_uart_rx_item.sv"
  `include "rk4_uart_sequencer.sv"
  `include "rk4_uart_driver.sv"
  `include "rk4_uart_monitor.sv"
  `include "rk4_uart_agent.sv"
  `include "rk4_scoreboard.sv"
  `include "rk4_coverage.sv"
  `include "rk4_env.sv"
  `include "rk4_base_sequence.sv"
  `include "rk4_base_test.sv"

endpackage
