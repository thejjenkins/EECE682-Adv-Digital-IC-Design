// =============================================================================
//  rk4_uart_rx_item.sv -- Sequence item: one UART byte received FROM the DUT
// =============================================================================
class rk4_uart_rx_item extends uvm_sequence_item;

  logic [7:0] data;

  `uvm_object_utils_begin(rk4_uart_rx_item)
    `uvm_field_int(data, UVM_ALL_ON)
  `uvm_object_utils_end

  function new(string name = "rk4_uart_rx_item");
    super.new(name);
  endfunction

endclass
