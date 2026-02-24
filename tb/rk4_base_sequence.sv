// =============================================================================
//  rk4_base_sequence.sv -- Base sequence with UART byte/word send helpers
// =============================================================================
class rk4_base_sequence extends uvm_sequence #(rk4_uart_tx_item);

  `uvm_object_utils(rk4_base_sequence)

  rand real v0y;

  constraint c_v0y_default { v0y >= 1.0; v0y <= 100.0; }

  function new(string name = "rk4_base_sequence");
    super.new(name);
  endfunction

  // Send a single UART byte to the DUT
  task send_byte(input logic [7:0] data);
    rk4_uart_tx_item item;
    item = rk4_uart_tx_item::type_id::create("tx_item");
    start_item(item);
    item.data = data;
    finish_item(item);
  endtask

  // Send v0y as 4-byte little-endian Q16.16
  task send_v0y(input real v0y_val);
    logic signed [31:0] v0y_fixed;
    v0y_fixed = real_to_q16(v0y_val);
    `uvm_info("SEQ", $sformatf("Sending v0y = %.2f (Q16.16 = 0x%08X)", v0y_val, v0y_fixed), UVM_MEDIUM)
    send_byte(v0y_fixed[ 7: 0]);
    send_byte(v0y_fixed[15: 8]);
    send_byte(v0y_fixed[23:16]);
    send_byte(v0y_fixed[31:24]);
  endtask

  virtual task body();
    send_v0y(v0y);
  endtask

endclass
