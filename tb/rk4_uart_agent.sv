// =============================================================================
//  rk4_uart_agent.sv -- UVM agent: bundles driver + monitor + sequencer
// =============================================================================
class rk4_uart_agent extends uvm_agent;

  `uvm_component_utils(rk4_uart_agent)

  rk4_uart_driver    drv;
  rk4_uart_monitor   mon;
  rk4_uart_sequencer sqr;

  uvm_analysis_port #(rk4_uart_rx_item) ap;

  function new(string name = "rk4_uart_agent", uvm_component parent = null);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    mon = rk4_uart_monitor::type_id::create("mon", this);
    if (get_is_active() == UVM_ACTIVE) begin
      drv = rk4_uart_driver::type_id::create("drv", this);
      sqr = rk4_uart_sequencer::type_id::create("sqr", this);
    end
  endfunction

  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    ap = mon.ap;
    if (get_is_active() == UVM_ACTIVE)
      drv.seq_item_port.connect(sqr.seq_item_export);
  endfunction

endclass
