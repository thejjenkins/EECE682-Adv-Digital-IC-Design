// =============================================================================
//  rk4_uart_tx_item.sv -- Sequence item: one UART byte sent TO the DUT
// =============================================================================
class rk4_uart_tx_item extends uvm_sequence_item;

  rand logic [7:0] data;

  `uvm_object_utils_begin(rk4_uart_tx_item)
    `uvm_field_int(data, UVM_ALL_ON)
  `uvm_object_utils_end

  function new(string name = "rk4_uart_tx_item");
    super.new(name);
  endfunction

endclass
