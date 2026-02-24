// =============================================================================
//  rk4_uart_monitor.sv -- UVM monitor: samples DUT's uart_tx, emits rx_items
// =============================================================================
class rk4_uart_monitor extends uvm_monitor;

  `uvm_component_utils(rk4_uart_monitor)

  virtual rk4_if vif;
  int baud_div = 434;

  uvm_analysis_port #(rk4_uart_rx_item) ap;

  function new(string name = "rk4_uart_monitor", uvm_component parent = null);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    ap = new("ap", this);
    if (!uvm_config_db#(virtual rk4_if)::get(this, "", "vif", vif))
      `uvm_fatal("NOVIF", "Virtual interface not found in config_db")
    void'(uvm_config_db#(int)::get(this, "", "baud_div", baud_div));
  endfunction

  task run_phase(uvm_phase phase);
    forever begin
      rk4_uart_rx_item item;
      logic [7:0] data;
      sample_byte(data);
      item = rk4_uart_rx_item::type_id::create("rx_item");
      item.data = data;
      ap.write(item);
    end
  endtask

  task sample_byte(output logic [7:0] data);
    // Wait for start bit (falling edge on uart_tx)
    @(negedge vif.uart_tx);

    // Move to middle of start bit
    repeat (baud_div / 2) @(posedge vif.clk);

    // Verify start bit is still low
    if (vif.uart_tx !== 1'b0)
      `uvm_warning("GLITCH", "Start bit glitch detected on uart_tx")

    // Sample 8 data bits at mid-bit
    for (int i = 0; i < 8; i++) begin
      repeat (baud_div) @(posedge vif.clk);
      data[i] = vif.uart_tx;
    end

    // Consume stop bit
    repeat (baud_div) @(posedge vif.clk);
  endtask

endclass
