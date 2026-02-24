// =============================================================================
//  rk4_base_test.sv -- Base UVM test: builds env, resets DUT, runs sequence
// =============================================================================
class rk4_base_test extends uvm_test;

  `uvm_component_utils(rk4_base_test)

  rk4_env       env;
  virtual rk4_if vif;

  real test_v0y = 49.0;

  function new(string name = "rk4_base_test", uvm_component parent = null);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    env = rk4_env::type_id::create("env", this);

    if (!uvm_config_db#(virtual rk4_if)::get(this, "", "vif", vif))
      `uvm_fatal("NOVIF", "Virtual interface not found in config_db")

    // Propagate v0y to scoreboard
    uvm_config_db#(real)::set(this, "env.sb", "expected_v0y", test_v0y);

    // rk4_top has no parameters; its internal rk4_projectile_top defaults to
    // CLK_FREQ=50 MHz, BAUD_RATE=115200 => BAUD_DIV=434 on the *internal*
    // clock.  rk4_clk_gen divides the 100 MHz input by 100, so the internal
    // clock is 1 MHz.  From the TB's 100 MHz perspective each UART bit
    // therefore lasts 434 * 100 = 43400 top-level cycles.
    uvm_config_db#(int)::set(this, "env.agent.*", "baud_div", 43400);
  endfunction

  task run_phase(uvm_phase phase);
    rk4_base_sequence seq;
    phase.raise_objection(this, "rk4_base_test main stimulus");

    // Reset sequence
    apply_reset();

    // Configure clock mux to use external clock
    vif.en  = 1'b0;
    vif.sel = 2'b11;
    repeat (4) @(posedge vif.clk);

    // Run the stimulus sequence
    seq = rk4_base_sequence::type_id::create("seq");
    seq.v0y = test_v0y;
    seq.start(env.agent.sqr);

    // Wait for DUT to finish processing and transmitting
    wait_for_completion();

    phase.drop_objection(this, "rk4_base_test stimulus complete");
  endtask

  task apply_reset();
    vif.rst     = 1'b0;
    vif.uart_rx = 1'b1;
    vif.en      = 1'b0;
    vif.sel     = 2'b11;
    repeat (16) @(posedge vif.clk);
    vif.rst = 1'b1;
    repeat (8) @(posedge vif.clk);
  endtask

  // Wait until the scoreboard sees the EOS marker or a timeout fires
  task wait_for_completion();
    fork
      begin
        wait (env.sb.eos_received);
        `uvm_info("TEST", "EOS marker detected -- test complete", UVM_LOW)
      end
      begin
        #5_000_000_000;  // 5 s watchdog (UART at 434*100 cycles/bit is slow)
        `uvm_warning("TEST", "Watchdog timeout waiting for EOS")
      end
    join_any
    disable fork;
  endtask

endclass
