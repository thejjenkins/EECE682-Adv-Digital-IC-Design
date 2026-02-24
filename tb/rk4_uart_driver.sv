// =============================================================================
//  rk4_uart_driver.sv -- UVM driver: bit-bangs 8N1 UART onto vif.uart_rx
// =============================================================================
class rk4_uart_driver extends uvm_driver #(rk4_uart_tx_item);

  `uvm_component_utils(rk4_uart_driver)

  virtual rk4_if vif;
  int baud_div = 434;

  function new(string name = "rk4_uart_driver", uvm_component parent = null);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    if (!uvm_config_db#(virtual rk4_if)::get(this, "", "vif", vif))
      `uvm_fatal("NOVIF", "Virtual interface not found in config_db")
    void'(uvm_config_db#(int)::get(this, "", "baud_div", baud_div));
  endfunction

  task run_phase(uvm_phase phase);
    rk4_uart_tx_item item;
    vif.uart_rx = 1'b1;
    forever begin
      seq_item_port.get_next_item(item);
      drive_byte(item.data);
      seq_item_port.item_done();
    end
  endtask

  task drive_byte(input logic [7:0] data);
    // Start bit
    vif.uart_rx = 1'b0;
    repeat (baud_div) @(posedge vif.clk);

    // 8 data bits, LSB first
    for (int i = 0; i < 8; i++) begin
      vif.uart_rx = data[i];
      repeat (baud_div) @(posedge vif.clk);
    end

    // Stop bit
    vif.uart_rx = 1'b1;
    repeat (baud_div) @(posedge vif.clk);
  endtask

endclass
